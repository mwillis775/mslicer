//! Picks the points on a model that need supporting.
//!
//! Candidates come from three sources, in order of priority:
//! 1. Islands: regions that would start printing in mid-air.
//! 2. Edges: sharp, downward pointing creases. Each layer of a shallow crease
//!    grows sideways along it with nothing below, even though neither face on
//!    either side is an overhang.
//! 3. Faces: surfaces within the overhang angle of horizontal, sampled
//!    uniformly over their area projected onto the build plate. Steeper
//!    surfaces carry less load per layer so they get up to 50% wider spacing.
//!
//! Candidates are then thinned with Poisson disk sampling, accepting the
//! highest priority and lowest candidates first. Because the face candidates
//! are dense, every point on an overhang ends up within roughly `spacing` of
//! a support.

use std::collections::HashMap;

use nalgebra::Vector3;

use crate::supports::{
    Contact,
    collider::Collider,
    islands::find_islands,
    settings::{PlacementSettings, SupportSettings},
};

/// Points closer than this to the build plate are resting on it.
const ON_PLATE: f32 = 0.1;
/// Angle between face normals for an edge to be considered sharp.
const SHARP_EDGE_ANGLE: f32 = 25.0;
/// Face candidates generated per Poisson disk.
const FACE_OVERSAMPLE: f32 = 3.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Kind {
    Island,
    Edge,
    Face,
}

#[derive(Debug, Clone, Copy)]
pub struct Candidate {
    pub contact: Contact,
    pub kind: Kind,
    /// No other support may be accepted within this distance.
    pub radius: f32,
}

/// Finds every candidate point, sorted in the order they should be accepted:
/// by priority, then lowest first.
pub fn candidates(collider: &Collider, settings: &SupportSettings, plate_z: f32) -> Vec<Candidate> {
    let placement = &settings.placement;
    let mut candidates = Vec::new();

    if placement.islands {
        let islands = find_islands(collider, plate_z, placement.spacing);
        candidates.extend(
            islands
                .iter()
                .flat_map(|x| &x.contacts)
                .map(|&contact| Candidate {
                    contact,
                    kind: Kind::Island,
                    radius: placement.min_spacing,
                }),
        );
    }

    placement
        .edges
        .then(|| edge_candidates(collider, placement, &mut candidates));
    placement
        .faces
        .then(|| face_candidates(collider, placement, &mut candidates));

    let min_z = plate_z + ON_PLATE;
    candidates.retain(|x| x.contact.point.z > min_z);
    candidates.sort_by(|a, b| {
        (a.kind.cmp(&b.kind)).then(a.contact.point.z.total_cmp(&b.contact.point.z))
    });
    candidates
}

/// Thins out candidates so no two are closer than their radius.
pub fn place(collider: &Collider, settings: &SupportSettings, plate_z: f32) -> Vec<Candidate> {
    let candidates = candidates(collider, settings, plate_z);
    let mut grid = PoissonGrid::new(settings);
    (candidates.into_iter())
        .filter(|x| {
            let blocked = grid.blocked(x);
            (!blocked).then(|| grid.insert(x.contact.point));
            !blocked
        })
        .collect()
}

fn slope_spacing(placement: &PlacementSettings, normal: Vector3<f32>) -> Option<f32> {
    let overhang = placement.overhang_angle.to_radians();
    let slope = (-normal.z).clamp(-1.0, 1.0).acos();
    (slope <= overhang)
        .then(|| placement.spacing * (1.0 + 0.5 * slope / overhang.max(f32::EPSILON)))
}

fn face_candidates(collider: &Collider, placement: &PlacementSettings, out: &mut Vec<Candidate>) {
    let mesh = collider.mesh();
    let mut rng = SplitMix64(0x6d73_6c69_6365_72);

    for face in 0..mesh.face_count() {
        let normal = collider.face_normal(face);
        let Some(spacing) = slope_spacing(placement, normal) else {
            continue;
        };

        let [a, b, c] = mesh.face_verts(face);
        let projected_area = (b - a).cross(&(c - a)).z.abs() / 2.0;
        let cell = spacing / FACE_OVERSAMPLE;

        // Stochastic rounding so that meshes made of many tiny triangles still
        // get the right density overall.
        let expected = projected_area / (cell * cell);
        let count = expected.floor() as u32 + (rng.next_f32() < expected.fract()) as u32;

        for _ in 0..count {
            let (u, v) = (rng.next_f32().sqrt(), rng.next_f32());
            let point = a * (1.0 - u) + b * (u * (1.0 - v)) + c * (u * v);
            out.push(Candidate {
                contact: Contact::new(point, normal),
                kind: Kind::Face,
                radius: spacing,
            });
        }
    }
}

fn edge_candidates(collider: &Collider, placement: &PlacementSettings, out: &mut Vec<Candidate>) {
    let mesh = collider.mesh();
    let overhang = placement.overhang_angle.to_radians();
    let sharp = SHARP_EDGE_ANGLE.to_radians().cos();

    // Map each undirected edge to the faces using it. Non-manifold edges are
    // ignored.
    let mut edges = HashMap::<(u32, u32), (usize, Option<usize>, u8)>::new();
    for (idx, face) in mesh.faces().iter().enumerate() {
        for i in 0..3 {
            let (a, b) = (face[i], face[(i + 1) % 3]);
            let entry = edges.entry((a.min(b), a.max(b))).or_insert((idx, None, 0));
            (entry.2 == 1).then(|| entry.1 = Some(idx));
            entry.2 += 1;
        }
    }

    let vertices = mesh.vertices();
    for ((a, b), (f1, f2, count)) in edges {
        let Some(f2) = f2.filter(|_| count == 2) else {
            continue;
        };

        let (n1, n2) = (collider.face_normal(f1), collider.face_normal(f2));
        if n1.dot(&n2) > sharp {
            continue;
        }

        let bisector = (n1 + n2).normalize();
        let Some(spacing) = slope_spacing(placement, bisector) else {
            continue;
        };

        let [a, b] = [a, b].map(|x| mesh.transform(&vertices[x as usize]));
        let edge = b - a;
        let length = edge.magnitude();
        if length < f32::EPSILON || (edge.z / length).abs() > overhang.sin() {
            continue;
        }

        // Only convex creases (pointing away from the model) need supporting.
        let opposite = (mesh.face_verts(f2).into_iter())
            .max_by(|x, y| {
                (x - a)
                    .cross(&edge)
                    .norm()
                    .total_cmp(&(y - a).cross(&edge).norm())
            })
            .unwrap();
        if (opposite - a).dot(&n1) >= -1e-6 * length {
            continue;
        }

        let steps = (length / (spacing / FACE_OVERSAMPLE)).ceil().max(1.0) as u32;
        for i in 0..=steps {
            out.push(Candidate {
                contact: Contact::new(a + edge * (i as f32 / steps as f32), bisector),
                kind: Kind::Edge,
                radius: spacing,
            });
        }
    }
}

/// Spatial hash of accepted support points, used for Poisson disk sampling.
#[derive(Clone)]
pub struct PoissonGrid {
    cell: f32,
    points: HashMap<Vector3<i32>, Vec<Vector3<f32>>>,
}

impl PoissonGrid {
    pub fn new(settings: &SupportSettings) -> Self {
        Self {
            // Candidate radii are at most 1.5× the spacing, so checking the
            // neighboring cells is always enough.
            cell: (settings.placement.spacing * 1.5).max(f32::EPSILON),
            points: HashMap::new(),
        }
    }

    fn key(&self, point: Vector3<f32>) -> Vector3<i32> {
        (point / self.cell).map(|x| x.floor() as i32)
    }

    pub fn insert(&mut self, point: Vector3<f32>) {
        self.points.entry(self.key(point)).or_default().push(point);
    }

    /// Checks if any point is within the candidate's radius.
    pub fn blocked(&self, candidate: &Candidate) -> bool {
        let (point, radius) = (candidate.contact.point, candidate.radius);
        let center = self.key(point);

        (-1..=1).any(|x| {
            (-1..=1).any(|y| {
                (-1..=1).any(|z| {
                    let cell = self.points.get(&(center + Vector3::new(x, y, z)));
                    (cell.into_iter().flatten()).any(|p| (p - point).magnitude() < radius)
                })
            })
        })
    }
}

/// Small deterministic RNG so generation is repeatable.
struct SplitMix64(u64);

impl SplitMix64 {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }

    fn next_f32(&mut self) -> f32 {
        (self.next() >> 40) as f32 / (1u64 << 24) as f32
    }
}
