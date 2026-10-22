//! Finds a collision free path from a contact point on the model down to the
//! build plate (or, failing that, onto the model itself).
//!
//! In order of preference, a support will:
//! 1. Leave the surface along its normal (clamped to the max tip angle) and
//!    drop straight down.
//! 2. Lean once (up to the max shaft angle) to get around the model, then drop.
//! 3. Lean twice, for supports tucked deep under the model.
//! 4. Repeat the above with alternate tip directions.
//! 5. Drop straight down onto the model, if model landing is enabled.

use std::f32::consts::TAU;

use nalgebra::Vector3;

use crate::supports::{
    Contact, Foot, Route,
    collider::Collider,
    fill,
    settings::{SupportSettings, SupportSize, SupportStyle},
};

/// Number of directions checked around the vertical when searching for a lean.
const AZIMUTH_STEPS: usize = 24;
/// Number of tip directions that are tried before giving up.
const MAX_TIP_ATTEMPTS: usize = 4;
/// Number of tip directions the lean search is run from.
const MAX_LEAN_ATTEMPTS: usize = 4;
/// Number of candidate first bends that are explored for two-bend routes.
const TWO_BEND_BEAM: usize = 6;
/// The tip must leave the surface within this angle of its normal.
const MAX_TIP_SURFACE_ANGLE: f32 = 70.0;
/// A landing surface must face at least this far upward (cos 60°).
const MIN_LANDING_NORMAL_Z: f32 = 0.5;
/// Model under a landing must be at least this much thicker than how far the
/// head is sunk in, so the head doesn't poke out the bottom.
const MIN_LANDING_THICKNESS: f32 = 0.3;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RouteError {
    /// The contact is too close to the build plate to fit a support.
    TooLow,
    /// There is no room for the tip to leave the surface.
    TipBlocked,
    /// No path to the plate (or model, if enabled) could be found.
    Blocked,
}

pub struct Router<'a> {
    collider: &'a Collider<'a>,
    settings: &'a SupportSettings,
    plate_z: f32,
}

#[derive(Clone, Copy)]
enum Target {
    Plate,
    Model,
}

struct Shaft {
    path: Vec<Vector3<f32>>,
    cost: f32,
}

impl<'a> Router<'a> {
    pub fn new(collider: &'a Collider<'a>, settings: &'a SupportSettings, plate_z: f32) -> Self {
        Self {
            collider,
            settings,
            plate_z,
        }
    }

    pub fn route(
        &self,
        contact: &Contact,
        style: SupportStyle,
        size: &SupportSize,
    ) -> Result<Route, RouteError> {
        if style == SupportStyle::Fill {
            return fill::route(self.collider, self.settings, self.plate_z, contact);
        }

        let size = &self.routing_size(style, size);
        let shaft_r = size.shaft_radius();
        let min_z = self.settings.column_bottom(self.plate_z) + shaft_r;

        let tips = (self.tip_directions(contact.normal).into_iter())
            .map(|dir| contact.point + dir * size.tip_length)
            .filter(|&end| end.z >= min_z)
            .collect::<Vec<_>>();
        if tips.is_empty() {
            return Err(RouteError::TooLow);
        }

        let tips = (tips.into_iter())
            .filter(|&end| self.tip_clear(contact.point, end, size))
            .take(MAX_TIP_ATTEMPTS)
            .collect::<Vec<_>>();
        if tips.is_empty() {
            return Err(RouteError::TipBlocked);
        }

        // Prefer reaching the plate from any tip direction over landing on the
        // model, as model landings leave marks on the print. For each target,
        // first try the cheap straight drop from every tip, then the much more
        // expensive lean search from the best few.
        (self.targets().into_iter())
            .find_map(|target| {
                let straight = tips.iter().find_map(|&tip| {
                    let foot = self.terminate(tip, size, target)?;
                    Some(Route::new(vec![tip], foot))
                });

                straight.or_else(|| {
                    tips.iter().take(MAX_LEAN_ATTEMPTS).find_map(|&tip| {
                        let (shaft, foot) = self.route_shaft(tip, size, target)?;
                        Some(Route::new(shaft.path, foot))
                    })
                })
            })
            .ok_or(RouteError::Blocked)
    }

    /// Routes a shaft of the given size from `start` down to the build plate,
    /// or onto the model if that is enabled. Used for the trunks of trees.
    pub fn route_from(&self, start: Vector3<f32>, size: &SupportSize) -> Option<Route> {
        (self.targets().into_iter()).find_map(|target| {
            let (shaft, foot) = self.route_shaft(start, size, target)?;
            Some(Route::new(shaft.path, foot))
        })
    }

    /// The size a support is routed with. Tapered supports are only at their
    /// full width near the base, so they're routed as slightly thinner. The
    /// difference is kept within half the clearance, so they still never
    /// touch the model.
    fn routing_size(&self, style: SupportStyle, size: &SupportSize) -> SupportSize {
        if !style.tapered() {
            return *size;
        }

        let clearance = self.settings.routing.clearance;
        let shaft_diameter = (size.shaft_diameter - clearance)
            .max((size.tip_diameter + size.shaft_diameter) / 2.0)
            .min(size.shaft_diameter);
        SupportSize {
            shaft_diameter,
            ..*size
        }
    }

    fn targets(&self) -> Vec<Target> {
        let mut targets = vec![Target::Plate];
        self.settings
            .routing
            .model_landing
            .then(|| targets.push(Target::Model));
        targets
    }

    /// Candidate tip directions, most preferred first.
    fn tip_directions(&self, normal: Vector3<f32>) -> Vec<Vector3<f32>> {
        let down = -Vector3::z();
        let max_angle = self.settings.routing.max_tip_angle.to_radians();
        let preferred = clamp_to_cone(normal, down, max_angle);

        let mut out = vec![preferred];
        let tilts = [0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0].map(|x| x * max_angle);
        for (i, tilt) in tilts.into_iter().enumerate() {
            let steps = if i == 0 { 1 } else { 8 * i };
            for j in 0..steps {
                let azimuth = j as f32 / steps as f32 * TAU;
                out.push(from_spherical(tilt, azimuth));
            }
        }

        let max_surface = MAX_TIP_SURFACE_ANGLE.to_radians().cos();
        out.retain(|dir| dir.dot(&normal) >= max_surface);
        out.sort_by(|a, b| b.dot(&preferred).total_cmp(&a.dot(&preferred)));
        out.dedup_by(|a, b| a.dot(b) > 0.9999);
        out
    }

    /// Checks that the tapered tip and the joint at its end are clear of the
    /// model. The half of the tip nearest the contact necessarily touches the
    /// surface, so it is ignored.
    fn tip_clear(&self, contact: Vector3<f32>, end: Vector3<f32>, size: &SupportSize) -> bool {
        let clearance = self.settings.routing.clearance;
        let mid = contact.lerp(&end, 0.5);
        let mid_radius = size.tip_radius() + (size.shaft_radius() - size.tip_radius()) * 0.75;

        self.collider.clear(mid, end, mid_radius)
            && self
                .collider
                .clear(end, end, size.shaft_radius() + clearance / 2.0)
    }

    /// Checks if a vertical column can go down from `top` and end at the
    /// target, returning the foot to use if it can.
    fn terminate(&self, top: Vector3<f32>, size: &SupportSize, target: Target) -> Option<Foot> {
        let radius = size.shaft_radius();
        let clearance = self.settings.routing.clearance;

        match target {
            Target::Plate => {
                let column_bottom = self.settings.column_bottom(self.plate_z);
                let base_top = self.settings.base_top(self.plate_z);
                let base_radius = self.settings.base.base_diameter / 2.0;

                let bottom = top.xy().push(column_bottom);
                (self.collider.clear(top, bottom, radius + clearance)
                    && (self.collider).clear(bottom, top.xy().push(base_top), base_radius))
                .then_some(Foot::Plate)
            }
            Target::Model => {
                // Drops straight down onto the model, ending with an upside
                // down tip.
                let landing = self.collider.ray(top, -Vector3::z())?;
                let foot_top = foot_top(top, &landing, size)?;

                let min_thickness = size.tip_penetration + MIN_LANDING_THICKNESS;
                (landing.normal.z >= MIN_LANDING_NORMAL_Z
                    && self.collider.thickness(&landing, min_thickness) >= min_thickness
                    && self.tip_clear(landing.point, foot_top, size)
                    && self.collider.clear(top, foot_top, radius + clearance))
                .then_some(Foot::Model(landing))
            }
        }
    }

    fn route_shaft(
        &self,
        tip: Vector3<f32>,
        size: &SupportSize,
        target: Target,
    ) -> Option<(Shaft, Foot)> {
        if let Some(foot) = self.terminate(tip, size, target) {
            let shaft = Shaft {
                path: vec![tip],
                cost: 0.0,
            };
            return Some((shaft, foot));
        }

        let mut waypoints = Vec::new();
        if let Some(shaft) = self.lean_search(tip, size, target, Some(&mut waypoints)) {
            return Some(shaft);
        }

        // Explore the leans that got furthest from the model, trying to find a
        // second lean from there that reaches the target.
        waypoints.sort_by(|a: &(f32, _), b| b.0.total_cmp(&a.0));
        waypoints.truncate(TWO_BEND_BEAM);
        (waypoints.into_iter())
            .filter_map(|(_, (point, cost))| {
                let (mut shaft, foot) = self.lean_search(point, size, target, None)?;
                shaft.path.insert(0, tip);
                shaft.cost += cost;
                Some((shaft, foot))
            })
            .min_by(|a, b| a.0.cost.total_cmp(&b.0.cost))
    }

    /// Tries leaning the shaft in every direction, marching along each until
    /// it can reach the target or is blocked. Returns the cheapest route
    /// found. If `waypoints` is provided, the furthest clear point along each
    /// direction is pushed to it along with its distance from the model and
    /// the cost to get there.
    fn lean_search(
        &self,
        start: Vector3<f32>,
        size: &SupportSize,
        target: Target,
        mut waypoints: Option<&mut Vec<(f32, (Vector3<f32>, f32))>>,
    ) -> Option<(Shaft, Foot)> {
        let routing = &self.settings.routing;
        let radius = size.shaft_radius();
        let clearance = routing.clearance;
        let max_angle = routing.max_shaft_angle.to_radians();
        let min_z = self.settings.column_bottom(self.plate_z) + radius;
        let step = (radius * 2.0).max(0.75);

        let mut best: Option<(Shaft, Foot)> = None;
        for lean in [1.0 / 3.0, 2.0 / 3.0, 1.0].map(|x| x * max_angle) {
            let max_length = routing.max_reach / lean.sin();
            let penalty = 1.0 + lean / max_angle.max(f32::EPSILON);

            for i in 0..AZIMUTH_STEPS {
                let azimuth = i as f32 / AZIMUTH_STEPS as f32 * TAU;
                let dir = from_spherical(lean, azimuth);

                let (mut prev, mut length) = (start, 0.0);
                loop {
                    length += step;
                    let point = start + dir * length;
                    if length > max_length || point.z < min_z {
                        break;
                    }

                    if best.as_ref().is_some_and(|x| x.0.cost <= length * penalty) {
                        break;
                    }

                    if !self.collider.clear(prev, point, radius + clearance) {
                        break;
                    }

                    if let Some(foot) = self.terminate(point, size, target) {
                        let shaft = Shaft {
                            path: vec![start, point],
                            cost: length * penalty,
                        };
                        best = Some((shaft, foot));
                        break;
                    }

                    prev = point;
                }

                if prev != start
                    && let Some(waypoints) = waypoints.as_deref_mut()
                {
                    let distance = self.collider.distance(prev, prev, routing.max_reach);
                    waypoints.push((distance, (prev, (prev - start).magnitude() * penalty)));
                }
            }
        }

        best
    }
}

/// The top of the upside down tip a support standing on the model ends with.
/// The tip is shortened, down to half its normal length, to fit between
/// closely stacked parts of a model.
pub fn foot_top(
    column_top: Vector3<f32>,
    landing: &Contact,
    size: &SupportSize,
) -> Option<Vector3<f32>> {
    let gap = column_top.z - landing.point.z;
    (gap >= size.tip_length * 0.5).then(|| landing.point + Vector3::z() * gap.min(size.tip_length))
}

/// Unit vector `tilt` radians from straight down, rotated `azimuth` radians
/// around the Z axis.
fn from_spherical(tilt: f32, azimuth: f32) -> Vector3<f32> {
    let (sin, cos) = tilt.sin_cos();
    Vector3::new(sin * azimuth.cos(), sin * azimuth.sin(), -cos)
}

/// Rotates `vector` towards `axis` such that the angle between them is at most
/// `max_angle` radians.
pub fn clamp_to_cone(vector: Vector3<f32>, axis: Vector3<f32>, max_angle: f32) -> Vector3<f32> {
    let vector = vector.normalize();
    if vector.dot(&axis) >= max_angle.cos() {
        return vector;
    }

    let perpendicular = vector - axis * vector.dot(&axis);
    let perpendicular = if perpendicular.magnitude_squared() < 1e-8 {
        // Vector is pointing directly away from the axis, any direction works.
        slicer::builder::orthogonal_basis(axis)[0]
    } else {
        perpendicular.normalize()
    };

    axis * max_angle.cos() + perpendicular * max_angle.sin()
}

impl RouteError {
    pub fn description(&self) -> &'static str {
        match self {
            Self::TooLow => "Too close to the build plate, try lifting the model.",
            Self::TipBlocked => "No room for the support tip.",
            Self::Blocked => "No clear path to the build plate.",
        }
    }
}
