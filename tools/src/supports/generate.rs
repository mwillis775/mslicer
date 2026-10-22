use common::progress::Progress;
use nalgebra::Vector3;
use rayon::iter::{IntoParallelIterator, ParallelIterator};
use slicer::{builder::MeshBuilder, mesh::Mesh};

use crate::supports::{
    Contact, Support,
    collider::Collider,
    fill,
    islands::find_islands_in,
    mesh::SupportMesh,
    placement::{PoissonGrid, candidates},
    routing::{RouteError, Router},
    settings::{SizePreset, SupportSettings, SupportSize, SupportStyle},
    tree,
};

/// Max number of placement and routing rounds.
const MAX_WAVES: usize = 4;
/// Max number of times the result is checked for spots left hanging.
const VERIFY_ROUNDS: usize = 6;
/// Pixel and layer alignments the checks cycle through, see
/// [`find_islands_in`].
const VERIFY_PHASES: [f32; 2] = [0.0, 0.5];
/// Spots left hanging this close to one that already couldn't be supported
/// aren't tried again.
const RETRY_DISTANCE: f32 = 0.3;

pub struct Generated {
    pub supports: Vec<Support>,
    /// Spots that needed support but no route could be found for. Nearby
    /// failures are merged.
    pub failed: Vec<(Vector3<f32>, RouteError)>,
}

/// Automatically places and routes supports for the collider's mesh.
///
/// Placement and routing are interleaved in waves. Each wave Poisson samples
/// the remaining candidates around the supports placed so far, and routes them
/// in parallel. If a support can't be routed (it's in a crevice too narrow for
/// the tip, for example) its spot is opened back up so the next wave can try
/// a nearby candidate instead.
///
/// Afterwards, the model is checked with its supports in place for anything
/// still left hanging, which is supported directly. If gap filling is enabled,
/// spots that still can't be supported are filled in, see
/// [`crate::supports::fill`].
pub fn generate(
    collider: &Collider,
    settings: &SupportSettings,
    plate_z: f32,
    progress: Progress,
) -> Generated {
    let mut pending = candidates(collider, settings, plate_z);
    let router = Router::new(collider, settings, plate_z);

    let mut grid = PoissonGrid::new(settings);
    let mut supports = Vec::new();
    let mut failed = Vec::new();
    let mut total = 0;

    for wave in 0..MAX_WAVES {
        let mut wave_grid = grid.clone();
        let (selected, rest) = (pending.into_iter()).partition::<Vec<_>, _>(|x| {
            let blocked = wave_grid.blocked(x);
            (!blocked).then(|| wave_grid.insert(x.contact.point));
            !blocked
        });

        pending = rest;
        if selected.is_empty() {
            break;
        }

        total += selected.len();
        progress.set_total(total as u64);

        let results = (selected.into_par_iter())
            .map(|candidate| {
                let support = new_support(collider, &router, settings, candidate.contact);
                progress.add_complete(1);
                (candidate, support)
            })
            .collect::<Vec<_>>();

        // Supports that are still blocked in every direction after a retry are
        // likely in an enclosed area, so there is no point retrying the
        // candidates around them again. Other failures are more local, a
        // different point nearby could well work.
        let mut boxed_in = PoissonGrid::new(settings);
        for (candidate, result) in results {
            match result {
                Ok(support) => {
                    grid.insert(support.contact.point);
                    supports.push(support);
                }
                Err(err) => {
                    (wave > 0 && err == RouteError::Blocked)
                        .then(|| boxed_in.insert(candidate.contact.point));
                    failed.push((candidate, err));
                }
            }
        }

        // Candidates already covered by a working support are done with.
        pending.retain(|x| !grid.blocked(x) && !boxed_in.blocked(x));
    }

    if settings.style == SupportStyle::Tree {
        tree::grow(collider, &router, settings, plate_z, &mut supports);
    }

    // Only report failures that didn't get covered by a nearby support, and
    // only one per spot.
    let mut reported = PoissonGrid::new(settings);
    let mut failed = (failed.into_iter())
        .filter(|(candidate, _)| {
            let covered = grid.blocked(candidate) || reported.blocked(candidate);
            (!covered).then(|| reported.insert(candidate.contact.point));
            !covered
        })
        .map(|(candidate, err)| (candidate.contact, err))
        .collect::<Vec<_>>();

    if settings.gap_fill.enabled {
        failed = fill_gaps(collider, settings, plate_z, &mut supports, failed);
    }

    // Placement can miss spots, as a support nearby on a different part of
    // the model looks like it covers them. Check what is actually still left
    // hanging with the supports in place, and support (or fill) that
    // directly. Holding up one spot can uncover smaller ones next to it, so
    // this is repeated until nothing new is found. Each spot is only tried
    // once, if it's still hanging after that it couldn't be supported.
    let mut tried = Vec::<Vector3<f32>>::new();
    let mut clean = 0;
    for round in 0..(settings.placement.islands as usize * VERIFY_ROUNDS) {
        let phase = VERIFY_PHASES[round % VERIFY_PHASES.len()];
        let (retried, leftover) = (leftover_islands(collider, settings, plate_z, &supports, phase))
            .into_iter()
            .partition::<Vec<_>, _>(|x| {
                (tried.iter()).any(|t| (t - x.point).magnitude() < RETRY_DISTANCE)
            });
        for contact in retried {
            let reported = (failed.iter())
                .any(|(f, _)| (f.point - contact.point).magnitude() < RETRY_DISTANCE);
            (!reported).then(|| failed.push((contact, RouteError::Blocked)));
        }

        tried.extend(leftover.iter().map(|x| x.point));
        if leftover.is_empty() {
            // Done once a check at every alignment comes up clean. Models that
            // came out clean the first time are trusted, as checks are slow on
            // large models.
            clean += 1;
            if round == 0 || clean >= VERIFY_PHASES.len() {
                break;
            }
            continue;
        }
        clean = 0;

        let results = (leftover.into_par_iter())
            .map(|contact| (contact, new_support(collider, &router, settings, contact)))
            .collect::<Vec<_>>();
        let mut unsupported = Vec::new();
        for (contact, result) in results {
            match result {
                Ok(support) => supports.push(support),
                Err(err) => unsupported.push((contact, err)),
            }
        }

        if settings.gap_fill.enabled {
            unsupported = fill_gaps(collider, settings, plate_z, &mut supports, unsupported);
        }
        failed.extend(unsupported);
    }

    progress.set_finished();
    let failed = (failed.into_iter())
        .map(|(contact, err)| (contact.point, err))
        .collect();
    Generated { supports, failed }
}

/// Points on the model that would still start printing in mid-air with the
/// supports in place.
fn leftover_islands(
    collider: &Collider,
    settings: &SupportSettings,
    plate_z: f32,
    supports: &[Support],
    phase: f32,
) -> Vec<Contact> {
    let model = collider.mesh();
    let mut builder = MeshBuilder::new();
    let (vertices, faces) = builder.raw_mut();
    vertices.extend(model.vertices().iter().map(|x| model.transform(x)));
    faces.extend_from_slice(model.faces());

    // The raft can't hold anything up, and is a lot to slice.
    let mut settings = settings.clone();
    settings.base.raft = false;
    if let Some((supports, _)) = SupportMesh::build(&settings, plate_z, supports, &[]) {
        let offset = vertices.len() as u32;
        vertices.extend_from_slice(supports.vertices());
        faces.extend(supports.faces().iter().map(|f| f.map(|x| x + offset)));
    }

    let combined = builder.build();
    let islands = find_islands_in(
        collider,
        &combined,
        plate_z,
        settings.placement.spacing,
        phase,
    );
    (islands.into_iter())
        .flat_map(|x| x.contacts)
        .filter(|x| x.normal.z < 0.0)
        .collect()
}

/// Fills in the gaps under spots that couldn't be supported, returning the
/// ones that couldn't be filled either. A fill grows over the whole gap, so
/// spots already inside one are skipped, and supports it makes unnecessary
/// are removed.
fn fill_gaps(
    collider: &Collider,
    settings: &SupportSettings,
    plate_z: f32,
    supports: &mut Vec<Support>,
    mut failed: Vec<(Contact, RouteError)>,
) -> Vec<(Contact, RouteError)> {
    let mut fills = Vec::<Support>::new();
    failed.retain(|(contact, _)| {
        let shapes = (supports.iter().chain(&fills)).filter_map(|x| x.route.fill.as_deref());
        if shapes.into_iter().any(|x| x.contains(contact.point)) {
            return false;
        }

        let Ok(route) = fill::route(collider, settings, plate_z, contact) else {
            return true;
        };
        let local = local_contact(collider.mesh(), contact);
        let fill = Support::new(SupportStyle::Fill, fill::size(), local, *contact, route);
        fills.push(fill);
        false
    });

    let shapes = (fills.iter())
        .filter_map(|x| x.route.fill.as_deref())
        .collect::<Vec<_>>();
    supports.retain(|x| !fill::covered(&shapes, x));
    supports.extend(fills);
    failed
}

/// Routes a new support from a contact point in support space. If there isn't
/// room for a support of the configured size, a thinner one is tried.
pub fn new_support(
    collider: &Collider,
    router: &Router,
    settings: &SupportSettings,
    contact: Contact,
) -> Result<Support, RouteError> {
    let style = settings.style;
    let (size, route) = match router.route(&contact, style, &settings.size) {
        Ok(route) => (settings.size, route),
        Err(err) => {
            let thinner = thinner_size(settings).ok_or(err)?;
            let route = router.route(&contact, style, &thinner).map_err(|_| err)?;
            (thinner, route)
        }
    };

    let local = local_contact(collider.mesh(), &contact);
    Ok(Support::new(style, size, local, contact, route))
}

/// A thinner support size to fall back to in tight spots, if there is one.
fn thinner_size(settings: &SupportSettings) -> Option<SupportSize> {
    let size = &settings.size;
    let light = SizePreset::Light.size(settings.style);
    (light.shaft_diameter < size.shaft_diameter * 0.9).then(|| {
        let tip_diameter = size.tip_diameter.min(light.shaft_diameter);
        SupportSize {
            tip_diameter,
            tip_length: light.tip_length.min(size.tip_length),
            tip_penetration: size.tip_penetration.min(tip_diameter),
            shaft_diameter: light.shaft_diameter,
        }
    })
}

/// Re-routes an existing support after the model has been transformed, using
/// its contact point on the mesh.
pub fn reroute(
    collider: &Collider,
    router: &Router,
    support: &Support,
) -> Result<Support, RouteError> {
    let mesh = collider.mesh();
    let contact = Contact::new(
        mesh.transform(&support.local.point),
        mesh.transform_normal(&support.local.normal).normalize(),
    );

    let route = router.route(&contact, support.style, &support.size)?;
    Ok(Support {
        contact,
        route,
        ..support.clone()
    })
}

/// Re-routes all supports after the model has been transformed, re-growing
/// any trees. Supports that can no longer be routed are dropped.
pub fn reroute_all(
    collider: &Collider,
    settings: &SupportSettings,
    plate_z: f32,
    supports: &[Support],
) -> Vec<Support> {
    let router = Router::new(collider, settings, plate_z);
    let mut out = (supports.into_par_iter())
        .filter_map(|x| reroute(collider, &router, x).ok())
        .collect::<Vec<_>>();

    if out.iter().any(|x| x.style == SupportStyle::Tree) {
        tree::grow(collider, &router, settings, plate_z, &mut out);
    }

    out
}

/// A contact in support space, in the mesh's own untransformed coordinates.
fn local_contact(mesh: &Mesh, contact: &Contact) -> Contact {
    Contact::new(
        mesh.inv_transform(&contact.point),
        inv_transform_normal(mesh, contact.normal),
    )
}

/// Inverse of [`Mesh::transform_normal`].
fn inv_transform_normal(mesh: &Mesh, normal: Vector3<f32>) -> Vector3<f32> {
    let transpose = mesh.transformation_matrix().transpose();
    (transpose * normal.to_homogeneous()).xyz().normalize()
}
