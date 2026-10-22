//! Builds the printable support mesh.
//!
//! Every part (spheres, cylinders, cones, the raft) is its own closed,
//! outward facing solid. They overlap freely, the slicer uses the nonzero
//! winding rule so overlapping solids are unioned when sliced. The branches of
//! a tree share their trunk, which is only added once.

use std::{
    collections::{HashMap, HashSet},
    range::Range,
};

use common::geometry::convex_hull;
use nalgebra::{Vector2, Vector3};
use slicer::{builder::MeshBuilder, mesh::Mesh};

use crate::supports::{
    Brace, Foot, Support, SupportId,
    routing::foot_top,
    settings::{SupportSettings, SupportSize, SupportStyle},
};

/// Which faces of the support mesh belong to each support.
pub type FaceMap = HashMap<SupportId, Range<u32>>;

/// How far the base is sunk into the raft so they are properly fused.
const RAFT_OVERLAP: f32 = 0.05;
/// Number of points around each base used to build the raft outline.
const RAFT_POINTS: u32 = 16;

pub struct SupportMesh<'a> {
    settings: &'a SupportSettings,
    plate_z: f32,
    builder: MeshBuilder,
    /// Parts that have already been added, as the bits of their parameters.
    added: HashSet<[u32; 8]>,
}

impl<'a> SupportMesh<'a> {
    pub fn new(settings: &'a SupportSettings, plate_z: f32) -> Self {
        Self {
            settings,
            plate_z,
            builder: MeshBuilder::new(),
            added: HashSet::new(),
        }
    }

    /// Builds the full mesh for a set of supports, along with which faces
    /// belong to each support. The raft and braces aren't part of any support.
    pub fn build(
        settings: &SupportSettings,
        plate_z: f32,
        supports: &[Support],
        braces: &[Brace],
    ) -> Option<(Mesh, FaceMap)> {
        let mut this = SupportMesh::new(settings, plate_z);
        let mut map = FaceMap::new();

        for support in supports {
            let start = this.builder.next_face_idx();
            this.add_support(support);
            map.insert(support.id, Range::from(start..this.builder.next_face_idx()));
        }

        braces.iter().for_each(|x| this.add_brace(x));
        settings.base.raft.then(|| this.add_raft(supports));
        this.finish().map(|mesh| (mesh, map))
    }

    pub fn finish(self) -> Option<Mesh> {
        (!self.builder.is_empty()).then(|| self.builder.build())
    }

    fn segments(&self) -> u32 {
        (self.settings.resolution / 2).max(3)
    }

    /// Returns false if an identical part was already added.
    fn first(&mut self, parts: [f32; 8]) -> bool {
        self.added.insert(parts.map(f32::to_bits))
    }

    fn sphere(&mut self, center: Vector3<f32>, radius: f32) {
        if self.first([center.x, center.y, center.z, radius, 0.0, 0.0, 0.0, -1.0]) {
            let segments = self.segments().max(4);
            self.builder.add_sphere(center, radius, segments);
        }
    }

    fn cylinder(&mut self, (a, b): (Vector3<f32>, Vector3<f32>), (ra, rb): (f32, f32)) {
        if (a - b).magnitude_squared() > 1e-10 && self.first([a.x, a.y, a.z, b.x, b.y, b.z, ra, rb])
        {
            let segments = self.segments();
            self.builder.add_cylinder((a, b), (ra, rb), segments);
        }
    }

    /// A spherical head sunk into the surface, then a cone from the tip
    /// diameter out to `end_radius`.
    fn add_tip(
        &mut self,
        contact: Vector3<f32>,
        end: Vector3<f32>,
        end_radius: f32,
        size: &SupportSize,
    ) {
        let tip_r = size.tip_radius();
        let dir = (end - contact).normalize();
        let head = contact + dir * (tip_r - size.tip_penetration);
        self.sphere(head, tip_r);
        self.cylinder((head, end), (tip_r, end_radius));
    }

    pub fn add_support(&mut self, support: &Support) {
        if support.style == SupportStyle::Fill {
            return self.add_fill(support);
        }

        let size = &support.size;
        let top = support.column_top();
        let column_r = support.column_radius();

        // The final column either ends at the base, or above the model.
        let (column_end, landing) = match support.route.foot {
            Foot::Plate => (
                top.xy().push(self.settings.column_bottom(self.plate_z)),
                None,
            ),
            Foot::Model(landing) => {
                let end = foot_top(top, &landing, size)
                    .unwrap_or(landing.point + Vector3::z() * size.tip_length);
                (end, Some(landing))
            }
        };

        // Every point along the shaft, along with its radius.
        let mut points = (support.route.path.iter().enumerate())
            .map(|(i, &point)| (point, support.radius_at(i)))
            .chain([(column_end, column_r)])
            .collect::<Vec<_>>();

        // Tapered supports widen evenly all the way from the contact down to
        // the end of the column.
        if support.style.tapered() {
            let mut lengths = Vec::with_capacity(points.len());
            let mut previous = support.contact.point;
            let mut length = 0.0;
            for (point, _) in points.iter() {
                length += (point - previous).magnitude();
                lengths.push(length);
                previous = *point;
            }

            let (tip_r, shaft_r) = (size.tip_radius(), size.shaft_radius());
            let total = length.max(1e-6);
            for ((_, radius), length) in points.iter_mut().zip(lengths) {
                *radius = tip_r + (shaft_r - tip_r) * length / total;
            }
        }

        self.add_tip(support.contact.point, points[0].0, points[0].1, size);
        self.sphere(points[0].0, points[0].1);
        for (i, pair) in points.windows(2).enumerate() {
            let ((a, ra), (b, rb)) = (pair[0], pair[1]);
            self.cylinder((a, b), (ra, rb));
            // Joints at bends, the column end gets a base or a foot instead.
            (i + 2 < points.len()).then(|| self.sphere(b, rb));
        }

        let (end, end_r) = *points.last().unwrap();
        match landing {
            None => {
                let base = &self.settings.base;
                let base_bottom =
                    self.settings.base_top(self.plate_z) - base.raft as u8 as f32 * RAFT_OVERLAP;
                let base_r = self.base_radius(support);
                self.cylinder((end, end.xy().push(base_bottom)), (end_r, base_r));
            }
            Some(landing) => {
                self.sphere(end, end_r);
                self.add_tip(landing.point, end, end_r, size);
            }
        }
    }

    /// A solid filling the gap, see [`crate::supports::fill`].
    fn add_fill(&mut self, support: &Support) {
        (support.route.fill.iter()).for_each(|x| x.add_to(&mut self.builder));
    }

    /// Radius of the flared base under a support's column. Thick tree trunks
    /// get wider bases.
    fn base_radius(&self, support: &Support) -> f32 {
        let extra = (support.column_radius() - support.size.shaft_radius()).max(0.0);
        self.settings.base.base_diameter / 2.0 + extra * 2.0
    }

    fn add_brace(&mut self, brace: &Brace) {
        let radius = self.settings.bracing.diameter / 2.0;
        let min_z = self.settings.column_bottom(self.plate_z);
        if brace.start.z >= min_z && brace.end.z >= min_z {
            self.cylinder((brace.start, brace.end), (radius, radius));
        }
    }

    /// A flat plate under all the support bases, with an optional 45° bevel
    /// around the top edge.
    fn add_raft(&mut self, supports: &[Support]) {
        let base = &self.settings.base;
        let mut seen = HashSet::new();
        let bases = (supports.iter())
            .filter(|x| x.route.foot == Foot::Plate && x.style != SupportStyle::Fill)
            .map(|x| (x.column_top().xy(), self.base_radius(x)))
            .filter(|(center, _)| seen.insert(center.map(f32::to_bits)))
            .collect::<Vec<_>>();
        if bases.is_empty() {
            return;
        }

        // The chamfer is the same everywhere, so it can't cut into any base.
        let chamfer = if base.raft_chamfer {
            base.raft_thickness.min(base.raft_margin)
        } else {
            0.0
        };

        // The hull of the bottom outline is made of points from the circles
        // around each base. Shrinking every circle by the same amount gives the
        // hull of the top outline, so both have matching vertices.
        let circle = |inset: f32| {
            (bases.iter()).flat_map(move |&(center, radius)| {
                let r = radius + base.raft_margin - inset;
                (0..RAFT_POINTS).map(move |i| {
                    let angle = i as f32 / RAFT_POINTS as f32 * std::f32::consts::TAU;
                    center + Vector2::new(angle.cos(), angle.sin()) * r
                })
            })
        };

        let bottom = circle(0.0).collect::<Vec<_>>();
        let top = circle(chamfer).collect::<Vec<_>>();
        let hull = convex_hull_indices(&bottom);
        if hull.len() < 3 {
            return;
        }

        let (z0, z1) = (self.plate_z, self.settings.base_top(self.plate_z));
        let builder = &mut self.builder;
        let first = builder.next_idx();
        for &i in hull.iter() {
            builder.add_vertex(bottom[i].push(z0));
            builder.add_vertex(top[i].push(z1));
        }

        let n = hull.len() as u32;
        let (bottom_idx, top_idx) = (
            |i: u32| first + (i % n) * 2,
            |i: u32| first + (i % n) * 2 + 1,
        );
        for i in 0..n {
            // Hull is counter-clockwise from above.
            builder.add_face([bottom_idx(i), bottom_idx(i + 1), top_idx(i + 1)]);
            builder.add_face([bottom_idx(i), top_idx(i + 1), top_idx(i)]);
        }

        for i in 1..n - 1 {
            builder.add_face([top_idx(0), top_idx(i), top_idx(i + 1)]);
            builder.add_face([bottom_idx(0), bottom_idx(i + 1), bottom_idx(i)]);
        }
    }
}

/// Indices of the points making up the convex hull, counter-clockwise.
fn convex_hull_indices(points: &[Vector2<f32>]) -> Vec<usize> {
    let hull = convex_hull(points);
    let lookup = (points.iter().enumerate())
        .map(|(i, p)| ((p.x.to_bits(), p.y.to_bits()), i))
        .collect::<HashMap<_, _>>();
    (hull.iter())
        .filter_map(|p| lookup.get(&(p.x.to_bits(), p.y.to_bits())).copied())
        .collect()
}
