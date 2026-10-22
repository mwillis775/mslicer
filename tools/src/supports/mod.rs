//! Support generation for resin printing.
//!
//! All geometry here is in *support space*: millimeters, Z up, with the build
//! plate at some caller-provided height (`plate_z`). Callers are free to pick
//! any rigid frame for this. mslicer uses the model's position as the origin,
//! with the model's yaw undone, so supports only need to be re-routed when the
//! model is scaled or tilted.
//!
//! A support is made of (from the model down), see [`settings::SupportStyle`]
//! for how each style changes this:
//! - a spherical head, partially sunk into the model surface,
//! - a tapered tip, from the tip diameter to the shaft diameter,
//! - a shaft, which is a polyline of segments that may lean to avoid the model,
//! - a vertical column down to either the build plate (ending with a flared
//!   base, optionally on a raft) or the model's own surface (ending with a
//!   second, upside-down tip).

use std::sync::Arc;

use common::{
    id_type,
    serde::{Deserializer, SerdeExt, Serializer},
};
use nalgebra::Vector3;

pub mod bracing;
pub mod collider;
pub mod fill;
pub mod generate;
pub mod islands;
pub mod mesh;
pub mod orient;
pub mod placement;
pub mod routing;
pub mod settings;
pub mod tree;

#[cfg(test)]
mod tests;

use fill::FillShape;
use settings::{SupportSize, SupportStyle};

id_type!(SupportId, u32);

#[derive(Debug, Clone, PartialEq)]
pub struct Support {
    pub id: SupportId,
    pub style: SupportStyle,
    pub size: SupportSize,

    /// Where the support touches the model, in the mesh's own untransformed
    /// coordinates. Used to re-route the support after the model is rotated
    /// or scaled.
    pub local: Contact,
    /// Where the support touches the model, in support space.
    pub contact: Contact,
    pub route: Route,
}

/// A point on a surface and its outward facing unit normal.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Contact {
    pub point: Vector3<f32>,
    pub normal: Vector3<f32>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Route {
    /// The first point is the end of the tip, any following points are bends
    /// in the shaft. The final vertical column starts at the last point.
    pub path: Vec<Vector3<f32>>,
    /// Radius of the shaft at each point of `path`, for tree supports whose
    /// trunks thicken as branches merge. Empty if the shaft has a constant
    /// radius.
    pub radii: Vec<f32>,
    pub foot: Foot,
    /// The shape of a gap fill, for [`SupportStyle::Fill`] supports only. A
    /// fill's path is just its seed point.
    pub fill: Option<Arc<FillShape>>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Foot {
    /// Column goes straight down to the build plate (or raft).
    Plate,
    /// Column goes straight down and lands on the model at this point.
    Model(Contact),
}

/// A diagonal strut connecting the columns of two supports.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Brace {
    pub start: Vector3<f32>,
    pub end: Vector3<f32>,
}

impl Support {
    pub fn new(
        style: SupportStyle,
        size: SupportSize,
        local: Contact,
        contact: Contact,
        route: Route,
    ) -> Self {
        Self {
            id: SupportId::new(),
            style,
            size,
            local,
            contact,
            route,
        }
    }

    /// The point where the final vertical column starts.
    pub fn column_top(&self) -> Vector3<f32> {
        *self.route.path.last().unwrap()
    }

    /// Radius of the shaft at a point of the route's path.
    pub fn radius_at(&self, index: usize) -> f32 {
        (self.route.radii.get(index).copied()).unwrap_or(self.size.shaft_radius())
    }

    /// Radius of the final vertical column.
    pub fn column_radius(&self) -> f32 {
        self.radius_at(self.route.path.len() - 1)
    }

    /// The lowest point of the routed path, not including the final column.
    pub fn lowest_point(&self) -> f32 {
        (self.route.path.iter()).fold(f32::MAX, |min, p| min.min(p.z))
    }

    /// Whether the support needs to be routed again before it can be used, as
    /// it was loaded from an older format.
    pub fn is_stale(&self) -> bool {
        self.style == SupportStyle::Fill && self.route.fill.is_none()
    }

    /// Copy of this support with a new id.
    pub fn duplicate(&self) -> Self {
        Self {
            id: SupportId::new(),
            ..self.clone()
        }
    }

    pub fn serialize<T: Serializer>(&self, ser: &mut T) {
        self.style.serialize(ser);
        self.size.serialize(ser);
        self.local.serialize(ser);
        self.contact.serialize(ser);

        ser.write_u32_be(self.route.path.len() as u32);
        self.route.path.iter().for_each(|x| x.serialize(ser));
        ser.write_u32_be(self.route.radii.len() as u32);
        self.route.radii.iter().for_each(|&x| ser.write_f32_be(x));
        match self.route.foot {
            Foot::Plate => ser.write_u8(0),
            Foot::Model(contact) => {
                ser.write_u8(1);
                contact.serialize(ser);
            }
        }

        ser.write_bool(self.route.fill.is_some());
        (self.route.fill.iter()).for_each(|x| x.serialize(ser));
    }

    /// Reads a support written in the given [`settings::FORMAT_VERSION`].
    pub fn deserialize<T: Deserializer>(des: &mut T, format: u16) -> Self {
        let style = if format < 2 {
            SupportStyle::Standard
        } else {
            SupportStyle::deserialize(des)
        };
        let size = SupportSize::deserialize(des);
        let local = Contact::deserialize(des);
        let contact = Contact::deserialize(des);

        let path = (0..des.read_u32_be())
            .map(|_| Vector3::<f32>::deserialize(des))
            .collect();
        let radii = if format < 2 {
            Vec::new()
        } else {
            (0..des.read_u32_be()).map(|_| des.read_f32_be()).collect()
        };
        let foot = match des.read_u8() {
            1 => Foot::Model(Contact::deserialize(des)),
            _ => Foot::Plate,
        };

        // Fills from format 3 were made of columns, they are left without a
        // shape so they get grown again.
        let fill = (format >= 4 && des.read_bool()).then(|| Arc::new(FillShape::deserialize(des)));

        let route = Route {
            path,
            radii,
            foot,
            fill,
        };
        Self::new(style, size, local, contact, route)
    }
}

impl Route {
    /// A route with a constant radius shaft.
    pub fn new(path: Vec<Vector3<f32>>, foot: Foot) -> Self {
        Self {
            path,
            radii: Vec::new(),
            foot,
            fill: None,
        }
    }
}

impl Contact {
    pub fn new(point: Vector3<f32>, normal: Vector3<f32>) -> Self {
        Self { point, normal }
    }

    pub fn serialize<T: Serializer>(&self, ser: &mut T) {
        self.point.serialize(ser);
        self.normal.serialize(ser);
    }

    pub fn deserialize<T: Deserializer>(des: &mut T) -> Self {
        Self {
            point: Vector3::deserialize(des),
            normal: Vector3::deserialize(des),
        }
    }
}
