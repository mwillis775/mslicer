//! Picks an orientation for a model that is easy to support.
//!
//! Every orientation is given a cost, roughly the number of supports it will
//! need plus a penalty for height (taller prints take longer and have longer,
//! wobblier supports). It's found in two passes:
//!
//! 1. A quick estimate for many directions spread evenly over the sphere,
//!    refined around the best few. Supports needed are estimated from the area
//!    of overhanging faces projected onto the build plate (the same way face
//!    candidates are spaced) and the number of local minima, which become
//!    islands.
//! 2. A closer look at the best few orientations, and the current one. Support
//!    points are actually placed, and each one is checked for a clear drop to
//!    the build plate. Points with model below them need to land on the model
//!    (leaving marks) or can't be supported at all, so they cost extra.
//!
//! Orientations that don't fit in the build volume are avoided. Only the roll
//! and pitch of the model are changed, its yaw is kept.

use std::collections::HashSet;

use common::progress::Progress;
use nalgebra::{Matrix3, Rotation3, Vector3};
use rayon::iter::{IntoParallelIterator, IntoParallelRefIterator, ParallelIterator};
use slicer::{geometry::bvh::Bvh, mesh::Mesh};

use crate::supports::{collider::Collider, placement::place, settings::SupportSettings};

/// Number of directions evaluated in the first pass.
const SAMPLES: usize = 500;
/// Number of the best directions from the first pass that are refined.
const REFINE: usize = 8;
/// Angle steps used to refine directions, in degrees.
const REFINE_STEPS: [f32; 3] = [4.0, 2.0, 1.0];
/// Number of orientations given a closer look in the second pass.
const FINALISTS: usize = 6;
/// Directions closer together than this are considered the same, in degrees.
const DISTINCT_ANGLE: f32 = 10.0;
/// Cost of each millimeter of model height, in supports.
const HEIGHT_COST: f32 = 0.25;
/// Extra cost of a support point with part of the model below it.
const SHADOWED_COST: f32 = 1.5;
/// Cost of an orientation that doesn't fit in the build volume, high enough
/// that it's only picked if nothing fits.
const OUT_OF_BOUNDS_COST: f32 = 1e6;
/// The current orientation is kept unless another is at least this much
/// cheaper, so the model isn't flipped around for nothing.
const KEEP_CURRENT: f32 = 0.95;

#[derive(Debug, Clone, Copy)]
pub struct Orientation {
    /// Euler angles for the model, see [`Mesh::set_rotation`].
    pub rotation: Vector3<f32>,
    pub cost: f32,
}

/// Finds an orientation for the model that needs few supports and fits in the
/// build volume (`platform`, in mm). The mesh's scale and yaw are kept, its
/// position doesn't matter.
pub fn orient(
    mesh: &Mesh,
    bvh: &Bvh,
    settings: &SupportSettings,
    platform: Vector3<f32>,
    progress: Progress,
) -> Orientation {
    let shape = Shape::new(mesh);
    let yaw = mesh.rotation().z;
    progress.set_total((SAMPLES + REFINE * REFINE_STEPS.len() * 8 + FINALISTS + 1) as u64);

    // First pass, quick estimates spread over the whole sphere.
    let mut directions = (fibonacci_sphere(SAMPLES).into_par_iter())
        .map(|up| {
            progress.add_complete(1);
            (up, shape.quick_cost(up, yaw, settings, platform))
        })
        .collect::<Vec<_>>();
    directions.sort_by(|a, b| a.1.total_cmp(&b.1));
    let mut best = distinct(&directions, REFINE);

    // Refine the best directions by walking downhill in shrinking steps.
    for step in REFINE_STEPS.map(f32::to_radians) {
        for (up, cost) in best.iter_mut() {
            let neighbors = (0..8)
                .map(|i| tilt(*up, step, i as f32 / 8.0 * std::f32::consts::TAU))
                .collect::<Vec<_>>();
            let costs = (neighbors.par_iter())
                .map(|&x| (x, shape.quick_cost(x, yaw, settings, platform)))
                .collect::<Vec<_>>();
            progress.add_complete(8);

            for (x, c) in costs {
                (c < *cost).then(|| (*up, *cost) = (x, c));
            }
        }
    }
    best.sort_by(|a, b| a.1.total_cmp(&b.1));
    let finalists = distinct(&best, FINALISTS);

    // Second pass, a closer look at the best few and the current orientation.
    let current = mesh.rotation();
    let current_cost = detailed_cost(mesh, bvh, settings, platform, &shape, current) * KEEP_CURRENT;
    progress.add_complete(1);

    let best = (finalists.into_par_iter())
        .map(|(up, _)| {
            let rotation = rotation_for_up(up, yaw);
            let cost = detailed_cost(mesh, bvh, settings, platform, &shape, rotation);
            progress.add_complete(1);
            Orientation { rotation, cost }
        })
        .min_by(|a, b| a.cost.total_cmp(&b.cost));

    progress.set_finished();
    match best {
        Some(best) if best.cost < current_cost => best,
        _ => Orientation {
            rotation: current,
            cost: current_cost,
        },
    }
}

/// Euler angles that rotate `up`, a direction in the mesh's own coordinates,
/// to point straight up.
pub fn rotation_for_up(up: Vector3<f32>, yaw: f32) -> Vector3<f32> {
    // With R = Rz(yaw)·Ry(pitch)·Rx(roll), R·up = +Z doesn't depend on yaw.
    // Roll brings `up` into the XZ plane, then pitch brings it onto +Z.
    let roll = up.y.atan2(up.z);
    let pitch = (-up.x).atan2(up.y.hypot(up.z));
    Vector3::new(roll, pitch, yaw)
}

/// The full cost of an orientation, see the module docs.
pub fn detailed_cost(
    mesh: &Mesh,
    bvh: &Bvh,
    settings: &SupportSettings,
    platform: Vector3<f32>,
    shape: &Shape,
    rotation: Vector3<f32>,
) -> f32 {
    let mut mesh = mesh.clone();
    mesh.set_position_unchecked(Vector3::zeros());
    mesh.set_rotation(rotation);

    let (min, max) = mesh.bounds();
    let plate_z = min.z - settings.placement.lift;
    let collider = Collider::new(&mesh, bvh);

    // Islands are found from the mesh's local minima instead, the layer based
    // search is too slow to run for every orientation.
    let mut placement = settings.clone();
    placement.placement.islands = false;
    let points = place(&collider, &placement, plate_z);
    let shadowed = (points.par_iter())
        .filter(|x| {
            let origin = x.contact.point - Vector3::z() * 1e-3;
            collider.ray(origin, -Vector3::z()).is_some()
        })
        .count();

    let matrix = shape.matrix(rotation);
    let islands = settings
        .placement
        .islands
        .then(|| shape.islands(&matrix, settings.placement.spacing))
        .unwrap_or_default();

    points.len() as f32
        + islands as f32
        + shadowed as f32 * SHADOWED_COST
        + size_cost(max - min, settings, platform)
}

/// Cost of a model's bounding box size: its height, and whether it fits in
/// the build volume once lifted for supports.
fn size_cost(size: Vector3<f32>, settings: &SupportSettings, platform: Vector3<f32>) -> f32 {
    let size = size + Vector3::z() * settings.placement.lift;
    let fits = (0..3).all(|i| size[i] <= platform[i]);
    size.z * HEIGHT_COST + (!fits as u8 as f32) * OUT_OF_BOUNDS_COST
}

/// A mesh's geometry, along with which vertices are connected.
pub struct Shape<'a> {
    vertices: &'a [Vector3<f32>],
    faces: &'a [[u32; 3]],
    scale: Vector3<f32>,
    /// Neighbors of vertex `i` are `neighbors[offsets[i]..offsets[i + 1]]`.
    offsets: Vec<u32>,
    neighbors: Vec<u32>,
}

impl<'a> Shape<'a> {
    pub fn new(mesh: &'a Mesh) -> Self {
        let mut edges = (mesh.faces().iter())
            .flat_map(|f| {
                (0..3).flat_map(move |i| [(f[i], f[(i + 1) % 3]), (f[(i + 1) % 3], f[i])])
            })
            .collect::<Vec<_>>();
        edges.sort_unstable();
        edges.dedup();

        let mut offsets = vec![0; mesh.vertex_count() + 1];
        edges
            .iter()
            .for_each(|&(a, _)| offsets[a as usize + 1] += 1);
        (1..offsets.len()).for_each(|i| offsets[i] += offsets[i - 1]);

        Self {
            vertices: mesh.vertices(),
            faces: mesh.faces(),
            scale: mesh.scale(),
            offsets,
            neighbors: edges.into_iter().map(|(_, b)| b).collect(),
        }
    }

    /// Scale and rotation, without the translation.
    fn matrix(&self, rotation: Vector3<f32>) -> Matrix3<f32> {
        let rotation = Rotation3::from_euler_angles(rotation.x, rotation.y, rotation.z);
        Matrix3::from_diagonal(&self.scale) * rotation.matrix()
    }

    /// A rough cost for the orientation with `up` pointing up, see the module
    /// docs.
    pub fn quick_cost(
        &self,
        up: Vector3<f32>,
        yaw: f32,
        settings: &SupportSettings,
        platform: Vector3<f32>,
    ) -> f32 {
        let placement = &settings.placement;
        let matrix = self.matrix(rotation_for_up(up, yaw));
        let vertices = (self.vertices.par_iter())
            .map(|x| matrix * x)
            .collect::<Vec<_>>();

        let overhang = placement.overhang_angle.to_radians();
        let supports = (self.faces.par_iter())
            .map(|f| {
                let [a, b, c] = f.map(|x| vertices[x as usize]);
                let normal = (b - a).cross(&(c - a));
                let area = normal.magnitude();
                if normal.z >= 0.0 || area <= f32::EPSILON {
                    return 0.0;
                }

                // Same spacing as face candidates get, see `slope_spacing`.
                let slope = (-normal.z / area).clamp(-1.0, 1.0).acos();
                if slope > overhang {
                    return 0.0;
                }
                let spacing = placement.spacing * (1.0 + 0.5 * slope / overhang.max(f32::EPSILON));
                -normal.z / 2.0 / (spacing * spacing)
            })
            .sum::<f32>();

        let (min, max) = (vertices.iter()).fold(
            (Vector3::repeat(f32::MAX), Vector3::repeat(f32::MIN)),
            |(min, max), x| (min.inf(x), max.sup(x)),
        );
        let islands = placement
            .islands
            .then(|| self.islands_from(&vertices, placement.spacing))
            .unwrap_or_default();

        supports + islands as f32 + size_cost(max - min, settings, platform)
    }

    fn islands(&self, matrix: &Matrix3<f32>, spacing: f32) -> usize {
        let vertices = (self.vertices.par_iter())
            .map(|x| matrix * x)
            .collect::<Vec<_>>();
        self.islands_from(&vertices, spacing)
    }

    /// Counts the local minima of the transformed vertices, which would start
    /// printing in mid-air. Minima closer than `spacing` are counted once.
    fn islands_from(&self, vertices: &[Vector3<f32>], spacing: f32) -> usize {
        let minima = (0..vertices.len())
            .into_par_iter()
            .filter(|&i| {
                let range = self.offsets[i] as usize..self.offsets[i + 1] as usize;
                let z = vertices[i].z;
                !range.is_empty()
                    && (self.neighbors[range])
                        .iter()
                        .all(|&n| vertices[n as usize].z > z)
            })
            .map(|i| (vertices[i] / spacing.max(f32::EPSILON)).map(|x| x.floor() as i32))
            .collect::<HashSet<_>>();
        minima.len()
    }
}

/// The `count` best entries that point in different directions. Entries must
/// already be sorted best first.
fn distinct(sorted: &[(Vector3<f32>, f32)], count: usize) -> Vec<(Vector3<f32>, f32)> {
    let min_dot = DISTINCT_ANGLE.to_radians().cos();
    let mut out = Vec::<(Vector3<f32>, f32)>::new();
    for &(up, cost) in sorted {
        if out.len() >= count {
            break;
        }
        (out.iter().all(|(x, _)| x.dot(&up) < min_dot)).then(|| out.push((up, cost)));
    }
    out
}

/// Tilts a direction by `angle` radians, towards `azimuth` around it.
fn tilt(dir: Vector3<f32>, angle: f32, azimuth: f32) -> Vector3<f32> {
    let [a, b] = slicer::builder::orthogonal_basis(dir);
    let side = a * azimuth.cos() + b * azimuth.sin();
    (dir * angle.cos() + side * angle.sin()).normalize()
}

/// Evenly spread unit vectors covering the sphere.
fn fibonacci_sphere(count: usize) -> Vec<Vector3<f32>> {
    let golden = std::f32::consts::PI * (3.0 - 5f32.sqrt());
    (0..count)
        .map(|i| {
            let z = 1.0 - 2.0 * (i as f32 + 0.5) / count as f32;
            let r = (1.0 - z * z).sqrt();
            let theta = golden * i as f32;
            Vector3::new(r * theta.cos(), r * theta.sin(), z)
        })
        .collect()
}
