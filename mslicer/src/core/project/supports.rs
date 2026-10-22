//! Per-model supports.
//!
//! Supports are stored in *support space*: world space with the model's
//! position as the origin and its yaw undone. Moving a model or spinning it
//! around Z just moves the support mesh along with it. Supports only need to
//! be re-routed when the model is tilted or scaled (as its overhangs change),
//! or lowered far enough that a support no longer fits under it.

use std::{
    mem,
    sync::Arc,
    thread::{self, JoinHandle},
    time::{Duration, Instant},
};

use nalgebra::{Rotation3, Vector2, Vector3};
use slicer::{geometry::bvh::Bvh, mesh::Mesh};
use tools::supports::{
    Brace, Foot, Support, SupportId,
    bracing::brace,
    collider::Collider,
    generate::{reroute, reroute_all},
    mesh::{FaceMap, SupportMesh},
    routing::Router,
    settings::{SupportSettings, SupportStyle},
};
use tracing::{info, warn};
use wgpu::Device;

use crate::render::util::MeshBuffers;

/// How long a model's transform must stay put before re-routing its supports,
/// so dragging a rotation slider doesn't start a re-route every frame.
const REROUTE_DEBOUNCE: Duration = Duration::from_millis(300);

#[derive(Default)]
pub struct Supports {
    supports: Vec<Support>,
    /// The model transform `supports` were routed for.
    routed_for: Option<RouteKey>,
    /// Spots the last auto generation couldn't support, in support space.
    failed: Vec<Vector3<f32>>,

    reroute: Option<Reroute>,
    /// When the model's transform last changed in a way that needs re-routing.
    unsettled: Option<(RouteKey, Instant)>,

    // Derived data, rebuilt as needed.
    built_for: Option<(SupportSettings, f32)>,
    braces: Vec<Brace>,
    mesh: Option<(Mesh, FaceMap)>,
    buffers: Option<MeshBuffers>,
}

/// A model's supports, along with the transform they were routed for.
#[derive(Clone, PartialEq)]
pub struct Snapshot {
    pub supports: Vec<Support>,
    pub routed_for: Option<RouteKey>,
}

impl Snapshot {
    pub fn new(supports: Vec<Support>, anchor: &Anchor) -> Self {
        Self {
            supports,
            routed_for: Some(anchor.key),
        }
    }
}

struct Reroute {
    key: RouteKey,
    handle: JoinHandle<Vec<Support>>,
}

/// The parts of a model's transform that support routes depend on.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RouteKey {
    rotation: Vector2<f32>,
    scale: Vector3<f32>,
    /// Only included if the model is scaled differently along X and Y, as
    /// then yaw changes the model's shape.
    yaw: Option<f32>,
}

/// Maps between support space and world space for a model.
#[derive(Debug, Clone, Copy)]
pub struct Anchor {
    pub position: Vector3<f32>,
    pub yaw: f32,
    pub key: RouteKey,
}

impl Anchor {
    pub fn for_mesh(mesh: &Mesh) -> Self {
        let (position, rotation, scale) = (mesh.position(), mesh.rotation(), mesh.scale());

        // Yaw can only be factored out if X/Y scaling commutes with it.
        let uniform = (scale.x - scale.y).abs() <= 1e-6 * scale.x.abs().max(1.0);
        let yaw = if uniform { rotation.z } else { 0.0 };

        Self {
            position,
            yaw,
            key: RouteKey {
                rotation: rotation.xy(),
                scale,
                yaw: (!uniform).then_some(rotation.z),
            },
        }
    }

    /// Height of the build plate in support space.
    pub fn plate_z(&self) -> f32 {
        -self.position.z
    }

    /// A copy of the model's mesh, transformed into support space.
    pub fn support_space_mesh(&self, mesh: &Mesh) -> Mesh {
        let mut out = mesh.clone();
        let rotation = mesh.rotation();
        out.set_position_unchecked(Vector3::zeros());
        out.set_rotation_unchecked(rotation.xy().push(rotation.z - self.yaw));
        out.update_transformation_matrix();
        out
    }

    /// Sets a support space mesh's transform so it renders in world space.
    pub fn apply(&self, mesh: &mut Mesh) {
        mesh.set_position_unchecked(self.position);
        mesh.set_rotation_unchecked(Vector3::z() * self.yaw);
        mesh.update_transformation_matrix();
    }

    pub fn to_world(&self, point: Vector3<f32>) -> Vector3<f32> {
        self.position + Rotation3::from_axis_angle(&Vector3::z_axis(), self.yaw) * point
    }

    pub fn to_support_space(&self, point: Vector3<f32>) -> Vector3<f32> {
        Rotation3::from_axis_angle(&Vector3::z_axis(), -self.yaw) * (point - self.position)
    }

    pub fn direction_to_support_space(&self, direction: Vector3<f32>) -> Vector3<f32> {
        Rotation3::from_axis_angle(&Vector3::z_axis(), -self.yaw) * direction
    }
}

impl Supports {
    /// Supports loaded from a project file, which were routed for the model's
    /// current transform. Supports from older formats that need it are routed
    /// again.
    pub fn loaded(supports: Vec<Support>, mesh: &Mesh) -> Self {
        let stale = supports.iter().any(Support::is_stale);
        Self {
            supports,
            routed_for: (!stale).then(|| Anchor::for_mesh(mesh).key),
            ..Default::default()
        }
    }

    pub fn list(&self) -> &[Support] {
        &self.supports
    }

    pub fn len(&self) -> usize {
        self.supports.len()
    }

    pub fn is_empty(&self) -> bool {
        self.supports.is_empty()
    }

    pub fn failed(&self) -> &[Vector3<f32>] {
        &self.failed
    }

    /// False while supports are waiting to be re-routed.
    pub fn is_ready(&self) -> bool {
        self.supports.is_empty() || self.mesh.is_some()
    }

    fn invalidate(&mut self) {
        self.built_for = None;
        self.mesh = None;
        self.buffers = None;
    }

    /// The current supports, for restoring later with [`Supports::restore`].
    pub fn snapshot(&self) -> Snapshot {
        Snapshot {
            supports: self.supports.clone(),
            routed_for: self.routed_for,
        }
    }

    /// Replaces all supports, returning the old ones.
    pub fn restore(&mut self, snapshot: Snapshot) -> Snapshot {
        self.reroute = None;
        self.unsettled = None;
        self.failed.clear();
        self.invalidate();

        Snapshot {
            supports: mem::replace(&mut self.supports, snapshot.supports),
            routed_for: mem::replace(&mut self.routed_for, snapshot.routed_for),
        }
    }

    pub fn set_failed(&mut self, failed: Vec<Vector3<f32>>) {
        self.failed = failed;
    }

    /// Adds a support routed for the model's current transform.
    pub fn add(&mut self, support: Support, mesh: &Mesh) {
        if self.supports.is_empty() {
            self.routed_for = Some(Anchor::for_mesh(mesh).key);
        }

        // Any in progress re-route would be missing this support.
        self.reroute = None;
        self.supports.push(support);
        self.invalidate();
    }

    pub fn remove(&mut self, ids: &[SupportId]) {
        self.reroute = None;
        self.supports.retain(|x| !ids.contains(&x.id));
        self.invalidate();
    }

    /// Copies of all supports with new ids, for duplicated models.
    pub fn duplicate(&self) -> Self {
        Self {
            supports: self.supports.iter().map(Support::duplicate).collect(),
            routed_for: self.routed_for,
            ..Default::default()
        }
    }

    pub fn support_for_face(&self, face: u32) -> Option<SupportId> {
        let (_, map) = self.mesh.as_ref()?;
        (map.iter().find(|(_, r)| r.contains(&face))).map(|(id, _)| *id)
    }

    /// The support mesh, in support space with its transform set to place it
    /// in world space.
    pub fn mesh(&self) -> &Option<(Mesh, FaceMap)> {
        &self.mesh
    }

    pub fn get_buffers(&mut self, device: &Device) -> &Option<MeshBuffers> {
        if self.buffers.is_none()
            && let Some((mesh, _)) = &self.mesh
        {
            self.buffers = Some(MeshBuffers::get_for(device, mesh));
        }

        &self.buffers
    }

    pub fn try_get_buffers(&self) -> Option<(&MeshBuffers, u32)> {
        let face_count = self.mesh.as_ref()?.0.face_count() as u32 * 3;
        self.buffers.as_ref().map(|x| (x, face_count))
    }

    /// Keeps the supports in sync with the model. Called every frame.
    pub fn update(&mut self, mesh: &Mesh, bvh: Option<&Arc<Bvh>>, settings: &SupportSettings) {
        if self.supports.is_empty() {
            self.reroute = None;
            self.unsettled = None;
            self.braces.clear();
            self.mesh.is_some().then(|| self.invalidate());
            return;
        }

        let anchor = Anchor::for_mesh(mesh);
        let Some(bvh) = bvh else { return };

        if self.routed_for != Some(anchor.key) {
            // Supports are hidden until they are re-routed.
            self.mesh.is_some().then(|| self.invalidate());
            self.reroute(mesh, bvh, settings, &anchor);
            return;
        }

        let plate_z = anchor.plate_z();
        self.fix_low_supports(mesh, bvh, settings, &anchor);

        let built_for = Some((settings.clone(), plate_z));
        if self.built_for != built_for {
            let collision_mesh = anchor.support_space_mesh(mesh);
            let collider = Collider::new(&collision_mesh, bvh);
            self.braces = brace(&collider, &self.supports, settings, plate_z);

            let built = SupportMesh::build(settings, plate_z, &self.supports, &self.braces);
            self.mesh = built.map(|(mut mesh, map)| {
                anchor.apply(&mut mesh);
                (mesh, map)
            });
            self.buffers = None;
            self.built_for = built_for;
        } else if let Some((mesh, _)) = &mut self.mesh {
            // Moving the model or rotating it around Z only moves the mesh.
            anchor.apply(mesh);
        }
    }

    /// Re-routes every support in the background after the model has been
    /// tilted or scaled. Waits until the transform has settled first.
    fn reroute(
        &mut self,
        mesh: &Mesh,
        bvh: &Arc<Bvh>,
        settings: &SupportSettings,
        anchor: &Anchor,
    ) {
        if let Some(reroute) = &self.reroute {
            if !reroute.handle.is_finished() {
                return;
            }

            let reroute = self.reroute.take().unwrap();
            match reroute.handle.join() {
                // Results for an old transform are thrown away, a new re-route
                // will be started from the original supports.
                Ok(supports) if reroute.key == anchor.key => {
                    let dropped = self.supports.len() - supports.len();
                    info!("Re-routed {} supports", supports.len());
                    (dropped > 0).then(|| {
                        warn!("Dropped {dropped} supports that could no longer be routed")
                    });

                    self.supports = supports;
                    self.routed_for = Some(anchor.key);
                    self.failed.clear();
                    self.unsettled = None;
                    return;
                }
                Ok(_) => {}
                Err(_) => warn!("Support re-routing failed"),
            }
        }

        match self.unsettled {
            Some((key, since)) if key == anchor.key => {
                if since.elapsed() < REROUTE_DEBOUNCE {
                    return;
                }
            }
            _ => {
                self.unsettled = Some((anchor.key, Instant::now()));
                return;
            }
        }

        let supports = self.supports.clone();
        let (key, plate_z) = (anchor.key, anchor.plate_z());
        let (mesh, bvh, settings) = (
            anchor.support_space_mesh(mesh),
            bvh.clone(),
            settings.clone(),
        );
        let handle = thread::spawn(move || {
            let collider = Collider::new(&mesh, &bvh);
            reroute_all(&collider, &settings, plate_z, &supports)
        });

        self.reroute = Some(Reroute { key, handle });
    }

    /// If the model was lowered, supports may no longer fit under it. These
    /// are re-routed individually, which is quick enough to do inline.
    fn fix_low_supports(
        &mut self,
        mesh: &Mesh,
        bvh: &Bvh,
        settings: &SupportSettings,
        anchor: &Anchor,
    ) {
        let plate_z = anchor.plate_z();
        let column_bottom = settings.column_bottom(plate_z);
        // Gap fill columns go all the way down to the raft, so they always fit.
        let too_low = |x: &Support| {
            x.route.foot == Foot::Plate
                && x.style != SupportStyle::Fill
                && x.lowest_point() < column_bottom + x.column_radius()
        };

        if !self.supports.iter().any(too_low) {
            return;
        }

        let collision_mesh = anchor.support_space_mesh(mesh);
        let collider = Collider::new(&collision_mesh, bvh);
        let router = Router::new(&collider, settings, plate_z);

        let before = self.supports.len();
        let supports = mem::take(&mut self.supports);
        self.supports = (supports.into_iter())
            .filter_map(|x| {
                if too_low(&x) {
                    reroute(&collider, &router, &x).ok()
                } else {
                    Some(x)
                }
            })
            .collect();

        let dropped = before - self.supports.len();
        (dropped > 0)
            .then(|| warn!("Dropped {dropped} supports that no longer fit under the model"));
        self.invalidate();
    }
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use common::{
        progress::Progress,
        serde::{DynamicSerializer, SliceDeserializer},
    };
    use nalgebra::Vector3;
    use slicer::{builder::MeshBuilder, geometry::bvh::Bvh};
    use tools::supports::settings::SupportSettings;

    use crate::core::project::{Project, model::Model, supports::Anchor};

    /// A floating box, tall enough for braced supports.
    fn model() -> Model {
        let mut builder = MeshBuilder::new();
        builder.add_cylinder(
            (Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 6.0)),
            (6.0, 6.0),
            12,
        );
        let mut model = Model::from_mesh(builder.build());
        model.mesh.set_position(Vector3::new(3.0, -2.0, 15.0));
        model.mesh.set_rotation(Vector3::new(0.0, 0.0, 0.7));
        model.bvh = Some(Arc::new(Bvh::build(model.mesh.inner(), Progress::new())));
        model
    }

    fn generate(model: &mut Model, settings: &SupportSettings) {
        let anchor = Anchor::for_mesh(&model.mesh);
        let mesh = anchor.support_space_mesh(&model.mesh);
        let collider = tools::supports::collider::Collider::new(&mesh, model.bvh.as_ref().unwrap());
        let generated = tools::supports::generate::generate(
            &collider,
            settings,
            anchor.plate_z(),
            Progress::new(),
        );
        assert!(!generated.supports.is_empty());
        model
            .supports
            .restore(super::Snapshot::new(generated.supports, &anchor));
        model.update_supports(settings);
    }

    #[test]
    fn support_space_matches_world() {
        let model = model();
        let anchor = Anchor::for_mesh(&model.mesh);
        let local = anchor.support_space_mesh(&model.mesh);
        for vertex in model.mesh.vertices() {
            let world = model.mesh.transform(vertex);
            let roundtrip = anchor.to_world(local.transform(vertex));
            assert!((world - roundtrip).magnitude() < 1e-4);
            assert!((anchor.to_support_space(world) - local.transform(vertex)).magnitude() < 1e-4);
        }
    }

    #[test]
    fn supports_follow_model() {
        let settings = SupportSettings::default();
        let mut model = model();
        generate(&mut model, &settings);

        let contacts = |model: &Model| {
            let (mesh, _) = model.supports.mesh().as_ref().unwrap();
            let support = &model.supports.list()[0];
            (
                mesh.transform(&support.contact.point),
                model.mesh.transform(&support.local.point),
            )
        };

        let (on_support, on_model) = contacts(&model);
        assert!((on_support - on_model).magnitude() < 1e-3);

        // Moving and spinning the model keeps the supports attached, and the
        // support mesh still reaches the build plate.
        model.mesh.set_position(Vector3::new(-10.0, 4.0, 20.0));
        model.mesh.set_rotation(Vector3::new(0.0, 0.0, -1.3));
        model.update_supports(&settings);
        let (on_support, on_model) = contacts(&model);
        assert!((on_support - on_model).magnitude() < 1e-3);

        let (mesh, _) = model.supports.mesh().as_ref().unwrap();
        let (min, _) = mesh.bounds();
        assert!(
            min.z.abs() < 1e-3,
            "supports should touch the plate, min z {}",
            min.z
        );
    }

    #[test]
    fn lowering_model_refits_supports() {
        let settings = SupportSettings::default();
        let mut model = model();
        generate(&mut model, &settings);
        let count = model.supports.len();

        // Lower the model to just above the minimum lift.
        model
            .mesh
            .set_position(model.mesh.position().xy().push(5.0));
        model.update_supports(&settings);
        assert_eq!(model.supports.len(), count);

        let (mesh, _) = model.supports.mesh().as_ref().unwrap();
        let (min, _) = mesh.bounds();
        assert!(min.z > -1e-3, "supports go through the plate: {}", min.z);
    }

    #[test]
    fn project_roundtrip() {
        let mut settings = SupportSettings::default();
        settings.size.shaft_diameter = 1.3;
        settings.base.raft = false;

        let mut model = model();
        generate(&mut model, &settings);

        let project = Project {
            support_settings: settings.clone(),
            models: vec![model],
            ..Default::default()
        };

        let mut ser = DynamicSerializer::new();
        project.serialize(&mut ser, Progress::new());
        let data = ser.into_inner();
        let loaded =
            Project::deserialize(&mut SliceDeserializer::new(&data), Progress::new()).unwrap();

        assert_eq!(loaded.support_settings, settings);
        let (a, b) = (
            project.models[0].supports.list(),
            loaded.models[0].supports.list(),
        );
        assert_eq!(a.len(), b.len());
        for (a, b) in a.iter().zip(b) {
            assert_eq!((a.size, a.local, a.contact), (b.size, b.local, b.contact));
            assert_eq!(a.route, b.route);
        }
        // Loaded supports don't need re-routing.
        assert!(loaded.models[0].supports.routed_for.is_some());
    }

    #[test]
    fn tilting_reroutes_in_background() {
        let settings = SupportSettings::default();
        let mut model = model();
        generate(&mut model, &settings);
        let count = model.supports.len();

        model.mesh.set_rotation(Vector3::new(0.2, 0.0, 0.7));
        model.update_supports(&settings);
        assert!(
            !model.supports.is_ready(),
            "supports are hidden until re-routed"
        );

        let start = std::time::Instant::now();
        while !model.supports.is_ready() {
            assert!(start.elapsed().as_secs() < 10, "re-route never finished");
            std::thread::sleep(std::time::Duration::from_millis(20));
            model.update_supports(&settings);
        }

        // Most supports should survive a small tilt, and still be attached.
        assert!(model.supports.len() * 10 >= count * 8);
        let support = &model.supports.list()[0];
        let (mesh, _) = model.supports.mesh().as_ref().unwrap();
        let attached =
            mesh.transform(&support.contact.point) - model.mesh.transform(&support.local.point);
        assert!(attached.magnitude() < 1e-3);
    }
}
