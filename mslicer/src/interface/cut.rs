//! Viewport tool for cutting a model in two with a plane that can be dragged
//! and tilted with the mouse.

use std::f32::consts::PI;

use egui::{Color32, DragValue, Key, Painter, PointerButton, Pos2, Response, Shape, Stroke, Ui};
use nalgebra::{Matrix4, Rotation3, Unit, Vector3};
use slicer::mesh::MeshId;

use crate::{
    core::{App, project::model::ModelId, state::Tool},
    interface::{
        components::primary_button,
        gizmo::{
            self, AXIS_COLORS, HANDLE_PX, PICK_PX, RingAngle, View, draw_arrow, draw_label,
            handle_color, perpendicular_basis, polyline_distance, ring_sign, screen_angle,
            segment_distance, snap,
        },
    },
    task::CutModel,
};

const ROTATE_SNAP: f32 = 15.0 * PI / 180.0;
/// Most cross-section segments drawn, larger sections are thinned out.
const MAX_SEGMENTS: usize = 40_000;
const PLANE_COLOR: Color32 = Color32::from_rgb(255, 170, 40);

pub struct CutState {
    model: Option<ModelId>,
    pub origin: Vector3<f32>,
    pub normal: Vector3<f32>,

    pub keep_above: bool,
    pub keep_below: bool,
    pub place_on_cut: bool,

    hovered: Option<Handle>,
    drag: Option<Drag>,
    section: Option<Section>,
}

#[derive(Clone, Copy, PartialEq)]
enum Handle {
    Offset,
    Ring(usize),
}

struct Drag {
    handle: Handle,
    origin: Vector3<f32>,
    normal: Vector3<f32>,
    start: f32,
    angle: RingAngle,
    label: String,
}

/// Cached outline of where the plane intersects the model.
struct Section {
    mesh: MeshId,
    transform: Matrix4<f32>,
    origin: Vector3<f32>,
    normal: Vector3<f32>,
    segments: Vec<[Vector3<f32>; 2]>,
}

impl Default for CutState {
    fn default() -> Self {
        Self {
            model: None,
            origin: Vector3::zeros(),
            normal: Vector3::z(),
            keep_above: true,
            keep_below: true,
            place_on_cut: true,
            hovered: None,
            drag: None,
            section: None,
        }
    }
}

/// The model being cut, resetting the plane if the selection changed.
fn target(app: &mut App) -> Option<ModelId> {
    let selected = gizmo::selected_models(app);
    let [id] = selected[..] else {
        app.state.cut.model = None;
        return None;
    };

    if app.state.cut.model != Some(id) {
        let model = app.project.model(id)?;
        let cut = &mut app.state.cut;
        cut.model = Some(id);
        cut.origin = model.get_bounds_center();
        cut.normal = Vector3::z();
        cut.drag = None;
        cut.section = None;
    }

    Some(id)
}

/// Half the size of the drawn plane, so it covers the whole model.
fn plane_extent(app: &mut App, id: ModelId) -> f32 {
    let (min, max) = app.project.model(id).unwrap().mesh.bounds();
    (max - min).norm() * 0.6
}

fn rings(cut: &CutState) -> [usize; 2] {
    // Tilt around the two world axes most perpendicular to the plane normal,
    // keeping the same rings for the whole drag.
    let normal = cut.drag.as_ref().map(|x| x.normal).unwrap_or(cut.normal);
    let mut axes = [0, 1, 2];
    axes.sort_by(|&a, &b| normal[a].abs().total_cmp(&normal[b].abs()));
    [axes[0], axes[1]]
}

pub fn interact(app: &mut App, ui: &Ui, response: &Response, view: &View) -> bool {
    if app.state.tool != Tool::Cut {
        app.state.cut.drag = None;
        return false;
    }
    let Some(_) = target(app) else {
        app.state.cut.hovered = None;
        return false;
    };

    let (ctrl, enter) = ui.input(|x| (x.modifiers.command, x.key_pressed(Key::Enter)));
    if enter && !ui.ctx().wants_keyboard_input() {
        execute(app);
        return false;
    }

    let cut = &mut app.state.cut;
    let size = view.world_per_px(cut.origin) * HANDLE_PX;
    let pointer = response.hover_pos();

    if cut.drag.is_none() {
        cut.hovered = pointer.and_then(|p| pick(cut, view, size, p));
    }

    if response.drag_started_by(PointerButton::Primary)
        && let Some(origin) = ui.input(|x| x.pointer.press_origin())
        && let Some(handle) = pick(cut, view, size, origin)
    {
        cut.drag = Some(Drag {
            handle,
            origin: cut.origin,
            normal: cut.normal,
            start: view
                .axis_param(origin, cut.origin, cut.normal)
                .unwrap_or(0.0),
            angle: (view.project(cut.origin))
                .map(|c| RingAngle::start(screen_angle(c, origin)))
                .unwrap_or_default(),
            label: String::new(),
        });
    }

    let Some(drag) = &mut cut.drag else {
        return false;
    };

    if ui.input(|x| x.key_pressed(Key::Escape)) {
        (cut.origin, cut.normal) = (drag.origin, drag.normal);
        cut.drag = None;
        return true;
    }

    if response.drag_stopped() || !response.dragged_by(PointerButton::Primary) {
        cut.drag = None;
        return true;
    }

    let Some(pointer) = pointer.or_else(|| ui.input(|x| x.pointer.latest_pos())) else {
        return true;
    };

    match drag.handle {
        Handle::Offset => {
            if let Some(t) = view.axis_param(pointer, drag.origin, drag.normal) {
                let delta = snap(t - drag.start, 0.5, ctrl);
                cut.origin = drag.origin + drag.normal * delta;
                drag.label = format!("{delta:+.2} mm");
            }
        }
        Handle::Ring(axis) => {
            if let Some(center) = view.project(drag.origin) {
                let dir = Vector3::ith(axis, 1.0);
                let total = drag.angle.update(screen_angle(center, pointer));
                let angle = snap(total * ring_sign(view, dir), ROTATE_SNAP, ctrl);
                let rotation = Rotation3::from_axis_angle(&Unit::new_unchecked(dir), angle);
                cut.normal = (rotation * drag.normal).normalize();
                let tilt = cut.normal.angle(&Vector3::z()).to_degrees();
                drag.label = format!("{:+.1}°  (tilt {tilt:.1}°)", angle.to_degrees());
            }
        }
    }

    true
}

fn pick(cut: &CutState, view: &View, size: f32, pos: Pos2) -> Option<Handle> {
    view.project(cut.origin)?;
    let mut best = (PICK_PX, None);

    let ends = [1.0, -1.0].map(|s| view.project(cut.origin + cut.normal * size * s));
    if let [Some(a), Some(b)] = ends {
        let dist = segment_distance(pos, a, b);
        (dist < best.0).then(|| best = (dist, Some(Handle::Offset)));
    }

    for axis in rings(cut) {
        let ring = view.ring(cut.origin, Vector3::ith(axis, 1.0), size * 1.3);
        let dist = polyline_distance(pos, &ring);
        (dist < best.0).then(|| best = (dist, Some(Handle::Ring(axis))));
    }

    best.1
}

fn update_section(app: &mut App, id: ModelId) {
    let cut = &app.state.cut;
    let model = app.project.models.iter().find(|x| x.id == id).unwrap();
    let mesh = &model.mesh;
    let transform = *mesh.transformation_matrix();

    if let Some(section) = &cut.section
        && section.mesh == mesh.mesh_id()
        && section.transform == transform
        && section.origin == cut.origin
        && section.normal == cut.normal
    {
        return;
    }

    // Distance to the plane in world space is linear in model space positions,
    // so only the crossing edges need to be transformed.
    let linear = transform.fixed_view::<3, 3>(0, 0);
    let translation = transform.fixed_view::<3, 1>(0, 3);
    let local_normal = linear.transpose() * cut.normal;
    let offset = cut.normal.dot(&(translation - cut.origin));
    let dist = (mesh.vertices().iter())
        .map(|v| local_normal.dot(v) + offset)
        .collect::<Vec<_>>();

    let crossing = |a: u32, b: u32| {
        let (da, db) = (dist[a as usize], dist[b as usize]);
        ((da >= 0.0) != (db >= 0.0)).then(|| {
            let (va, vb) = (mesh.vertices()[a as usize], mesh.vertices()[b as usize]);
            mesh.transform(&(va + (vb - va) * (da / (da - db))))
        })
    };

    let mut segments = Vec::new();
    for &[a, b, c] in mesh.faces() {
        let points = [crossing(a, b), crossing(b, c), crossing(c, a)];
        let mut found = points.into_iter().flatten();
        if let (Some(p), Some(q)) = (found.next(), found.next()) {
            segments.push([p, q]);
        }
    }

    app.state.cut.section = Some(Section {
        mesh: mesh.mesh_id(),
        transform,
        origin: cut.origin,
        normal: cut.normal,
        segments,
    });
}

pub fn paint(app: &mut App, painter: &Painter, view: &View) {
    if app.state.tool != Tool::Cut {
        return;
    }
    let Some(id) = target(app) else {
        return;
    };

    update_section(app, id);
    let extent = plane_extent(app, id);
    let cut = &app.state.cut;

    // The plane itself.
    let (u, v) = perpendicular_basis(cut.normal);
    let corners = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]
        .map(|(a, b)| view.project(cut.origin + (u * a + v * b) * extent));
    if corners.iter().all(Option::is_some) {
        let corners = corners.map(Option::unwrap).to_vec();
        painter.add(Shape::convex_polygon(
            corners.clone(),
            PLANE_COLOR.gamma_multiply(0.12),
            Stroke::NONE,
        ));
        let mut outline = corners;
        outline.push(outline[0]);
        painter.add(Shape::line(
            outline,
            Stroke::new(1.0_f32, PLANE_COLOR.gamma_multiply(0.6)),
        ));
    }

    // Where the model will be cut.
    if let Some(section) = &cut.section {
        let step = section.segments.len().div_ceil(MAX_SEGMENTS).max(1);
        let stroke = Stroke::new(2.0_f32, PLANE_COLOR);
        for [a, b] in section.segments.iter().step_by(step) {
            if let (Some(a), Some(b)) = (view.project(*a), view.project(*b)) {
                painter.line_segment([a, b], stroke);
            }
        }
    }

    // Handles.
    let size = view.world_per_px(cut.origin) * HANDLE_PX;
    let active = cut.drag.as_ref().map(|x| x.handle);
    let state = |handle| (cut.hovered == Some(handle), active == Some(handle));

    for axis in rings(cut) {
        let (hovered, active) = state(Handle::Ring(axis));
        let color = handle_color(AXIS_COLORS[axis], hovered, active);
        let width: f32 = if hovered || active { 3.5 } else { 2.0 };
        let ring = view.ring(cut.origin, Vector3::ith(axis, 1.0), size * 1.3);
        painter.add(Shape::line(ring, Stroke::new(width, color)));
    }

    if let Some(center) = view.project(cut.origin) {
        let (hovered, active) = state(Handle::Offset);
        let color = handle_color(PLANE_COLOR, hovered, active);
        let width: f32 = if hovered || active { 4.0 } else { 2.5 };
        for sign in [1.0, -1.0] {
            if let Some(end) = view.project(cut.origin + cut.normal * size * sign) {
                draw_arrow(painter, center, end, color, width);
            }
        }
        painter.circle_filled(center, 4.0, color);
    }

    if let Some(drag) = &cut.drag
        && let Some(pointer) = painter.ctx().pointer_latest_pos()
    {
        draw_label(painter, pointer, &drag.label);
    }
}

/// Contents of the tool options bar while cutting.
pub fn options(app: &mut App, ui: &mut Ui) {
    let Some(id) = target(app) else {
        ui.label("Select a single model to cut.");
        return;
    };

    let center = app.project.model(id).unwrap().get_bounds_center();
    let cut = &mut app.state.cut;

    ui.horizontal(|ui| {
        ui.label("Plane");
        for (name, normal) in [
            ("Horizontal", Vector3::z()),
            ("X", Vector3::x()),
            ("Y", Vector3::y()),
        ] {
            let selected = (cut.normal - normal).norm() < 1e-4;
            if ui.selectable_label(selected, name).clicked() {
                cut.normal = normal;
            }
        }
        if ui
            .button("Center")
            .on_hover_text("Move the plane back to the model's center")
            .clicked()
        {
            cut.origin = center;
        }
    });

    ui.horizontal(|ui| {
        if cut.normal.z.abs() > 0.9999 {
            ui.label("Height");
            ui.add(DragValue::new(&mut cut.origin.z).speed(0.1).suffix(" mm"));
        } else {
            ui.label("Offset");
            let mut offset = (cut.origin - center).dot(&cut.normal);
            if ui
                .add(DragValue::new(&mut offset).speed(0.1).suffix(" mm"))
                .changed()
            {
                cut.origin = center + cut.normal * offset;
            }
            let tilt = cut.normal.angle(&Vector3::z()).to_degrees();
            ui.label(format!("Tilt {tilt:.1}°"));
        }
    });

    ui.horizontal(|ui| {
        ui.checkbox(&mut cut.keep_above, "Keep upper");
        ui.checkbox(&mut cut.keep_below, "Keep lower");
    });
    ui.checkbox(&mut cut.place_on_cut, "Lay parts on cut face")
        .on_hover_text("Rotate each part so its flat cut face sits on the build plate");

    ui.add_enabled_ui(cut.keep_above || cut.keep_below, |ui| {
        if primary_button(ui, "Cut (Enter)").clicked() {
            execute(app);
        }
    });
}

fn execute(app: &mut App) {
    let Some(id) = target(app) else {
        return;
    };
    let cut = &app.state.cut;
    if !(cut.keep_above || cut.keep_below) {
        return;
    }

    let model = app.project.model(id).unwrap();
    let task = CutModel::new(
        model,
        cut.origin,
        cut.normal,
        [cut.keep_above, cut.keep_below],
        cut.place_on_cut,
    );
    app.tasks.add(task);
    app.state.cut.model = None;
}

/// Whether a cut handle is under the pointer or being dragged.
pub fn is_hovered(app: &App) -> bool {
    app.state.tool == Tool::Cut && (app.state.cut.hovered.is_some() || app.state.cut.drag.is_some())
}

pub fn hint() -> &'static str {
    "Drag the arrows to move the plane, the rings to tilt it. Ctrl snaps."
}
