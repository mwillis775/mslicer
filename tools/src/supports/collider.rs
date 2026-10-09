use nalgebra::Vector3;
use slicer::{geometry::bvh::Bvh, mesh::Mesh};

use crate::supports::Contact;

/// Rays used to measure thickness start this far inside the surface, so they
/// don't hit the surface they start on.
const THICKNESS_EPSILON: f32 = 1e-3;

/// Collision queries against a transformed mesh, in that mesh's world space.
///
/// Queries are done in the mesh's local space through its BVH. A world-space
/// radius `r` becomes `r / s` in local space where `s` is the smallest scale
/// factor. This is exact for uniformly scaled meshes and conservative (never
/// reports something as clear when it isn't) for non-uniformly scaled ones.
pub struct Collider<'a> {
    mesh: &'a Mesh,
    bvh: &'a Bvh,
    min_scale: f32,
}

impl<'a> Collider<'a> {
    pub fn new(mesh: &'a Mesh, bvh: &'a Bvh) -> Self {
        let scale = mesh.scale().abs();
        Self {
            mesh,
            bvh,
            min_scale: scale.min().max(f32::EPSILON),
        }
    }

    pub fn mesh(&self) -> &Mesh {
        self.mesh
    }

    /// Returns true if no part of the mesh is within `radius` of the segment
    /// `a→b`. In other words, a capsule can be placed there without touching
    /// the model.
    pub fn clear(&self, a: Vector3<f32>, b: Vector3<f32>, radius: f32) -> bool {
        let (a, b) = (self.mesh.inv_transform(&a), self.mesh.inv_transform(&b));
        !(self.bvh).near_segment_local(self.mesh, (a, b), radius / self.min_scale)
    }

    /// Lower bound for the distance between the segment `a→b` and the mesh,
    /// up to `max`.
    pub fn distance(&self, a: Vector3<f32>, b: Vector3<f32>, max: f32) -> f32 {
        let (a, b) = (self.mesh.inv_transform(&a), self.mesh.inv_transform(&b));
        let local = (self.bvh).segment_distance_local(self.mesh, (a, b), max / self.min_scale);
        (local * self.min_scale).min(max)
    }

    /// Casts a ray, returning the closest surface hit. The normal of the
    /// returned contact always faces away from the model.
    pub fn ray(&self, origin: Vector3<f32>, direction: Vector3<f32>) -> Option<Contact> {
        let hit = self.bvh.intersect_ray(self.mesh, origin, direction)?;
        Some(Contact {
            point: self.mesh.transform(&hit.position),
            normal: self.face_normal(hit.face),
        })
    }

    /// How far the model extends inward from a point on its surface, up to
    /// `max`.
    pub fn thickness(&self, contact: &Contact, max: f32) -> f32 {
        let inward = -contact.normal;
        let origin = contact.point + inward * THICKNESS_EPSILON;
        (self.ray(origin, inward))
            .map(|hit| (hit.point - contact.point).magnitude())
            .unwrap_or(max)
            .min(max)
    }

    /// World space unit normal of a face.
    pub fn face_normal(&self, face: usize) -> Vector3<f32> {
        (self.mesh.transform_normal(&self.mesh.normal(face))).normalize()
    }
}
