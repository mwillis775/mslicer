use std::sync::Arc;

use nalgebra::Vector3;

use crate::{
    geometry::{Primitive, bvh::Ray},
    mesh::MeshInner,
};

pub struct BoundingBox {
    min: Vector3<f32>,
    max: Vector3<f32>,
}

impl BoundingBox {
    pub fn new() -> Self {
        Self {
            min: Vector3::repeat(f32::INFINITY),
            max: Vector3::repeat(f32::NEG_INFINITY),
        }
    }

    pub fn center(&self) -> Vector3<f32> {
        (self.min + self.max) / 2.0
    }

    pub fn longest_axis(&self) -> usize {
        let lengths = (self.max - self.min).abs();

        if lengths.x > lengths.y && lengths.x > lengths.z {
            return 0; // X
        }

        if lengths.y > lengths.x {
            return 1; // Y
        }

        2 // Z
    }

    pub fn expand(&mut self, point: Vector3<f32>) {
        self.min = Vector3::new(
            self.min.x.min(point.x),
            self.min.y.min(point.y),
            self.min.z.min(point.z),
        );
        self.max = Vector3::new(
            self.max.x.max(point.x),
            self.max.y.max(point.y),
            self.max.z.max(point.z),
        );
    }

    pub fn expand_face(&mut self, mesh: &Arc<MeshInner>, face_idx: usize) {
        let face = mesh.faces[face_idx];
        self.expand(mesh.vertices[face[0] as usize]);
        self.expand(mesh.vertices[face[1] as usize]);
        self.expand(mesh.vertices[face[2] as usize]);
    }

    // Returns first intersection point. (Closer to ray origin)
    pub fn intersect<Type: Primitive>(&self, ray: Ray) -> Option<f32> {
        let (mut t_min, mut t_max) = (Type::min_t(), Type::max_t());

        for i in 0..3 {
            if ray.direction[i].abs() > 1e-8 {
                let t0 = (self.min[i] - ray.origin[i]) / ray.direction[i];
                let t1 = (self.max[i] - ray.origin[i]) / ray.direction[i];
                let (t0, t1) = if t0 <= t1 { (t0, t1) } else { (t1, t0) };
                t_min = t_min.max(t0);
                t_max = t_max.min(t1);
                if t_min > t_max {
                    return None;
                }
            } else if ray.origin[i] < self.min[i] || ray.origin[i] > self.max[i] {
                return None;
            }
        }

        (t_max >= Type::min_t()).then_some(t_min)
    }

    // Returns the squared distance from some point to the closest point on the AABB.
    // "Distance of Point to AABB" from Real-Time Collision Detection by Christer Ericson
    pub fn distance(&self, point: Vector3<f32>) -> f32 {
        let mut dist_sq = 0.0;
        for i in 0..3 {
            let v = point[i];
            (v < self.min[i]).then(|| dist_sq += (self.min[i] - v) * (self.min[i] - v));
            (v > self.max[i]).then(|| dist_sq += (v - self.max[i]) * (v - self.max[i]));
        }

        dist_sq
    }

    /// Checks if the segment `a→b` passes within `radius` of this box. This is
    /// conservative (the box is expanded on each axis rather than rounded), so
    /// it may return true for segments slightly further than `radius` away.
    pub fn near_segment(&self, a: Vector3<f32>, b: Vector3<f32>, radius: f32) -> bool {
        let (min, max) = (self.min.add_scalar(-radius), self.max.add_scalar(radius));
        let direction = b - a;
        let (mut t_min, mut t_max) = (0.0_f32, 1.0_f32);

        for i in 0..3 {
            if direction[i].abs() > 1e-12 {
                let inv = direction[i].recip();
                let t0 = (min[i] - a[i]) * inv;
                let t1 = (max[i] - a[i]) * inv;
                let (t0, t1) = if t0 <= t1 { (t0, t1) } else { (t1, t0) };
                t_min = t_min.max(t0);
                t_max = t_max.min(t1);
                if t_min > t_max {
                    return false;
                }
            } else if a[i] < min[i] || a[i] > max[i] {
                return false;
            }
        }

        true
    }
}
