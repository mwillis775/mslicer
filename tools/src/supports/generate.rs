use common::progress::Progress;
use nalgebra::Vector3;
use rayon::iter::{IntoParallelIterator, ParallelIterator};
use slicer::mesh::Mesh;

use crate::supports::{
    Contact, Support,
    collider::Collider,
    placement::{PoissonGrid, candidates},
    routing::{RouteError, Router},
    settings::{SizePreset, SupportSettings, SupportSize, SupportStyle},
    tree,
};

/// Max number of placement and routing rounds.
const MAX_WAVES: usize = 4;

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
    progress.set_finished();
    let mut reported = PoissonGrid::new(settings);
    let failed = (failed.into_iter())
        .filter(|(candidate, _)| {
            let covered = grid.blocked(candidate) || reported.blocked(candidate);
            (!covered).then(|| reported.insert(candidate.contact.point));
            !covered
        })
        .map(|(candidate, err)| (candidate.contact.point, err))
        .collect();

    Generated { supports, failed }
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

    let mesh = collider.mesh();
    let local = Contact::new(
        mesh.inv_transform(&contact.point),
        inv_transform_normal(mesh, contact.normal),
    );
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

/// Inverse of [`Mesh::transform_normal`].
fn inv_transform_normal(mesh: &Mesh, normal: Vector3<f32>) -> Vector3<f32> {
    let transpose = mesh.transformation_matrix().transpose();
    (transpose * normal.to_homogeneous()).xyz().normalize()
}
