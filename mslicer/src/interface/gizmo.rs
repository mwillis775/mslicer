//! Mouse driven move, rotate, and scale gizmos drawn over the viewport.

use std::f32::consts::{PI, TAU};

use egui::{
    Align2, Color32, FontId, Key, Painter, PointerButton, Pos2, Rect, Response, Shape, Stroke, Ui,
    pos2, vec2,
};
use nalgebra::{Matrix4, Rotation3, Unit, Vector2, Vector3};

use crate::core::{
    App,
    history::{Action, ModelAction, Transform},
    project::model::{Model, ModelId},
    selected::SelectedModel,
    state::Tool,
};

pub const AXIS_COLORS: [Color32; 3] = [
    Color32::from_rgb(232, 72, 72),
    Color32::from_rgb(98, 196, 82),
    Color32::from_rgb(70, 132, 240),
];
const AXIS_NAMES: [&str; 3] = ["X", "Y", "Z"];

/// Length of the gizmo handles in screen pixels.
pub const HANDLE_PX: f32 = 90.0;
/// How close the pointer has to be to a handle to grab it.
pub const PICK_PX: f32 = 9.0;
const ROTATE_SNAP: f32 = 15.0 * PI / 180.0;
const MOVE_SNAP: f32 = 1.0;
const SCALE_SNAP: f32 = 0.05;

#[derive(Default)]
pub struct GizmoState {
    hovered: Option<Handle>,
    drag: Option<Drag>,
}

#[derive(Clone, Copy, PartialEq)]
enum Handle {
    Axis(usize),
    Ring(usize),
    /// Dragging the model itself, moves it along the bed.
    Body,
}

struct Drag {
    handle: Handle,
    tool: Tool,
    center: Vector3<f32>,
    /// Axis parameter, or point on the bed plane the drag started at.
    start: Vector3<f32>,
    angle: RingAngle,
    models: Vec<StartState>,
    label: String,
}

struct StartState {
    id: ModelId,
    transform: Transform,
    center: Vector3<f32>,
    bottom: f32,
}

/// Tracks the total angle swept around a ring, so it can go past a full turn.
#[derive(Default)]
pub struct RingAngle {
    last: f32,
    total: f32,
}

/// Converts between world space and screen space for the viewport.
pub struct View {
    pub rect: Rect,
    view_projection: Matrix4<f32>,
    /// Unit vector pointing from the camera target towards the camera.
    pub towards_camera: Vector3<f32>,
    ray: Box<dyn Fn(Pos2) -> (Vector3<f32>, Vector3<f32>)>,
}

impl View {
    pub fn new(app: &App, rect: Rect) -> Self {
        let camera = app.camera.clone();
        let projection = app.config.render.projection;
        let aspect = rect.width() / rect.height();

        Self {
            rect,
            view_projection: camera.view_projection_matrix(projection, aspect),
            towards_camera: camera.position(1.0).normalize(),
            ray: Box::new(move |pos| {
                let uv = (pos - rect.min) / rect.size();
                camera.hovered_ray(projection, aspect, Vector2::new(uv.x, uv.y))
            }),
        }
    }

    pub fn project(&self, point: Vector3<f32>) -> Option<Pos2> {
        let clip = self.view_projection * point.push(1.0);
        if clip.w <= 1e-4 {
            return None;
        }

        let ndc = clip.xyz() / clip.w;
        Some(pos2(
            self.rect.min.x + (ndc.x + 1.0) / 2.0 * self.rect.width(),
            self.rect.min.y + (1.0 - ndc.y) / 2.0 * self.rect.height(),
        ))
    }

    pub fn ray(&self, pos: Pos2) -> (Vector3<f32>, Vector3<f32>) {
        (self.ray)(pos)
    }

    /// Size in world units of a screen pixel at the given point.
    pub fn world_per_px(&self, at: Vector3<f32>) -> f32 {
        let side = self.towards_camera.cross(&Vector3::z());
        let side = if side.norm() < 1e-3 {
            Vector3::x()
        } else {
            side.normalize()
        };

        match (self.project(at), self.project(at + side)) {
            (Some(a), Some(b)) if a.distance(b) > 1e-6 => 1.0 / a.distance(b),
            _ => 1.0,
        }
    }

    /// Parameter along the line through `origin` in `dir` closest to the ray
    /// under the given screen position.
    pub fn axis_param(&self, pos: Pos2, origin: Vector3<f32>, dir: Vector3<f32>) -> Option<f32> {
        let (ray_origin, ray_dir) = self.ray(pos);
        let w = origin - ray_origin;
        let (b, d, e) = (dir.dot(&ray_dir), dir.dot(&w), ray_dir.dot(&w));
        let denom = 1.0 - b * b;
        (denom.abs() > 1e-5).then(|| (b * e - d) / denom)
    }

    /// Point where the ray under the given screen position hits a plane.
    pub fn plane_hit(
        &self,
        pos: Pos2,
        origin: Vector3<f32>,
        normal: Vector3<f32>,
    ) -> Option<Vector3<f32>> {
        let (ray_origin, ray_dir) = self.ray(pos);
        let denom = ray_dir.dot(&normal);
        (denom.abs() > 1e-5).then(|| {
            let t = (origin - ray_origin).dot(&normal) / denom;
            ray_origin + ray_dir * t
        })
    }

    pub fn ring(&self, center: Vector3<f32>, axis: Vector3<f32>, radius: f32) -> Vec<Pos2> {
        let (u, v) = perpendicular_basis(axis);
        (0..=64)
            .filter_map(|i| {
                let a = i as f32 / 64.0 * TAU;
                self.project(center + (u * a.cos() + v * a.sin()) * radius)
            })
            .collect()
    }
}

impl RingAngle {
    pub fn start(angle: f32) -> Self {
        Self {
            last: angle,
            total: 0.0,
        }
    }

    pub fn update(&mut self, angle: f32) -> f32 {
        let delta = (angle - self.last + PI).rem_euclid(TAU) - PI;
        self.last = angle;
        self.total += delta;
        self.total
    }
}

pub fn perpendicular_basis(normal: Vector3<f32>) -> (Vector3<f32>, Vector3<f32>) {
    let helper = if normal.x.abs() < 0.9 {
        Vector3::x()
    } else {
        Vector3::y()
    };
    let u = normal.cross(&helper).normalize();
    (u, normal.cross(&u).normalize())
}

/// Angle of a point around a center in screen space, counter clockwise.
pub fn screen_angle(center: Pos2, pos: Pos2) -> f32 {
    (center.y - pos.y).atan2(pos.x - center.x)
}

/// The direction rotation around an axis appears to go on screen.
pub fn ring_sign(view: &View, axis: Vector3<f32>) -> f32 {
    if axis.dot(&view.towards_camera) >= 0.0 {
        1.0
    } else {
        -1.0
    }
}

pub fn segment_distance(p: Pos2, a: Pos2, b: Pos2) -> f32 {
    let ab = b - a;
    let t = ((p - a).dot(ab) / ab.length_sq().max(1e-6)).clamp(0.0, 1.0);
    p.distance(a + ab * t)
}

pub fn polyline_distance(p: Pos2, points: &[Pos2]) -> f32 {
    (points.windows(2))
        .map(|x| segment_distance(p, x[0], x[1]))
        .fold(f32::MAX, f32::min)
}

pub fn snap(value: f32, step: f32, enabled: bool) -> f32 {
    if enabled {
        (value / step).round() * step
    } else {
        value
    }
}

pub fn draw_arrow(painter: &Painter, from: Pos2, to: Pos2, color: Color32, width: f32) {
    let dir = (to - from).normalized();
    if !dir.is_finite() {
        return;
    }

    let normal = vec2(-dir.y, dir.x);
    let head = 6.0 + width * 2.0;
    painter.line_segment([from, to - dir * head], Stroke::new(width, color));
    painter.add(Shape::convex_polygon(
        vec![
            to,
            to - dir * head * 2.0 + normal * head,
            to - dir * head * 2.0 - normal * head,
        ],
        color,
        Stroke::NONE,
    ));
}

pub fn handle_color(color: Color32, hovered: bool, active: bool) -> Color32 {
    if active || hovered {
        color.lerp_to_gamma(Color32::WHITE, 0.45)
    } else {
        color
    }
}

pub fn draw_label(painter: &Painter, pos: Pos2, text: &str) {
    let font = FontId::proportional(14.0);
    let galley = painter.layout_no_wrap(text.to_owned(), font, Color32::WHITE);
    let rect = Align2::LEFT_BOTTOM
        .anchor_size(pos + vec2(16.0, -12.0), galley.size())
        .expand(4.0);
    painter.rect_filled(rect, 4.0, Color32::from_black_alpha(200));
    painter.galley(rect.min + vec2(4.0, 4.0), galley, Color32::WHITE);
}

/// Models the transform tools act on.
pub fn selected_models(app: &App) -> Vec<ModelId> {
    let models = app.project.models.iter().filter(|x| !x.hidden);
    match &app.state.selected {
        SelectedModel::None => Vec::new(),
        SelectedModel::Models(set) => models
            .filter(|x| set.contains(&x.id))
            .map(|x| x.id)
            .collect(),
        SelectedModel::Collection(id) => (models.filter(|x| x.collection == Some(*id)))
            .map(|x| x.id)
            .collect(),
    }
}

fn selection_bounds(app: &App, ids: &[ModelId]) -> Option<(Vector3<f32>, Vector3<f32>)> {
    let mut out: Option<(Vector3<f32>, Vector3<f32>)> = None;
    for model in app.project.models.iter().filter(|x| ids.contains(&x.id)) {
        let (min, max) = model.mesh.bounds();
        out = Some(match out {
            Some((a, b)) => (a.inf(&min), b.sup(&max)),
            None => (min, max),
        });
    }
    out
}

/// Moves the model so its bounding box is centered on `center` in XY and its
/// lowest point sits at `bottom`.
fn place(model: &mut Model, center: Vector3<f32>, bottom: f32) {
    let (min, max) = model.mesh.bounds();
    let current = (min + max) / 2.0;
    let delta = Vector3::new(center.x - current.x, center.y - current.y, bottom - min.z);
    model.mesh.set_position(model.mesh.position() + delta);
}

fn tool_handles(tool: Tool) -> bool {
    matches!(tool, Tool::Move | Tool::Rotate | Tool::Scale)
}

/// Handles pointer input for the gizmo. Returns true if the gizmo is using the
/// pointer, in which case the camera should not move.
pub fn interact(app: &mut App, ui: &Ui, response: &Response, view: &View) -> bool {
    let tool = app.state.tool;
    let mut ids = selected_models(app);
    // Models can be dragged with the move tool without selecting them first.
    if !tool_handles(tool) || (ids.is_empty() && tool != Tool::Move) {
        cancel(app);
        app.state.gizmo.hovered = None;
        return false;
    }

    // Keep the pivot fixed while rotating, otherwise follow the selection.
    let center = match &app.state.gizmo.drag {
        Some(drag) if drag.handle != Handle::Body => Some(drag.center),
        _ => selection_bounds(app, &ids).map(|(min, max)| (min + max) / 2.0),
    };
    let gizmo = center.map(|x| (x, view.world_per_px(x) * HANDLE_PX));
    let (ctrl, escape) = ui.input(|x| (x.modifiers.command, x.key_pressed(Key::Escape)));

    if app.state.gizmo.drag.is_some() && escape {
        cancel(app);
        return true;
    }

    let pointer = response.hover_pos();
    if app.state.gizmo.drag.is_none() {
        app.state.gizmo.hovered = pointer.and_then(|p| pick(app, view, tool, gizmo, p));
    }

    if response.drag_started_by(PointerButton::Primary)
        && let Some(origin) = ui.input(|x| x.pointer.press_origin())
        && let Some(handle) = pick(app, view, tool, gizmo, origin)
    {
        let mut center = center.unwrap_or_default();
        if handle == Handle::Body
            && let Some(hover) = app.state.hovered_geometry
            && !ids.contains(&hover.model)
        {
            app.state.selected.model_clicked(hover.model, false);
            ids = vec![hover.model];
            let (min, max) = selection_bounds(app, &ids).unwrap();
            center = (min + max) / 2.0;
        }

        let start = match handle {
            Handle::Axis(axis) => {
                let dir = Vector3::ith(axis, 1.0);
                Vector3::repeat(view.axis_param(origin, center, dir).unwrap_or(0.0))
            }
            Handle::Body => view
                .plane_hit(origin, center, Vector3::z())
                .unwrap_or(center),
            Handle::Ring(_) => Vector3::zeros(),
        };
        let angle = (view.project(center))
            .map(|c| RingAngle::start(screen_angle(c, origin)))
            .unwrap_or_default();

        let models = (app.project.models.iter())
            .filter(|x| ids.contains(&x.id))
            .map(|model| {
                let (min, max) = model.mesh.bounds();
                StartState {
                    id: model.id,
                    transform: Transform::of(model),
                    center: (min + max) / 2.0,
                    bottom: min.z,
                }
            })
            .collect();

        app.state.gizmo.drag = Some(Drag {
            handle,
            tool,
            center,
            start,
            angle,
            models,
            label: String::new(),
        });
    }

    let Some(drag) = &mut app.state.gizmo.drag else {
        return false;
    };

    if response.drag_stopped() || !response.dragged_by(PointerButton::Primary) {
        commit(app);
        return true;
    }

    if let Some(pointer) = pointer.or_else(|| ui.input(|x| x.pointer.latest_pos())) {
        let platform = app.project.slice_config.platform_size;
        drag_update(
            &mut app.project.models,
            &platform,
            drag,
            view,
            pointer,
            ctrl,
        );
    }

    true
}

fn pick(
    app: &App,
    view: &View,
    tool: Tool,
    gizmo: Option<(Vector3<f32>, f32)>,
    pos: Pos2,
) -> Option<Handle> {
    let mut best = (PICK_PX, None);
    if let Some((center, size)) = gizmo
        && let Some(center_px) = view.project(center)
    {
        pick_handle(view, tool, center, center_px, size, pos, &mut best);
    }

    best.1.or_else(|| {
        // Grab a model to slide it around the bed.
        let hover = app.state.hovered_geometry?;
        let visible = app
            .project
            .models
            .iter()
            .any(|x| x.id == hover.model && !x.hidden);
        (tool == Tool::Move && visible && !hover.support).then_some(Handle::Body)
    })
}

fn pick_handle(
    view: &View,
    tool: Tool,
    center: Vector3<f32>,
    center_px: Pos2,
    size: f32,
    pos: Pos2,
    best: &mut (f32, Option<Handle>),
) {
    match tool {
        Tool::Move | Tool::Scale => {
            for axis in 0..3 {
                let Some(end) = view.project(center + Vector3::ith(axis, size)) else {
                    continue;
                };
                let start = center_px + (end - center_px) * 0.2;
                let dist = segment_distance(pos, start, end);
                (dist < best.0).then(|| *best = (dist, Some(Handle::Axis(axis))));
            }
        }
        Tool::Rotate => {
            for axis in 0..3 {
                let ring = view.ring(center, Vector3::ith(axis, 1.0), size);
                let dist = polyline_distance(pos, &ring);
                (dist < best.0).then(|| *best = (dist, Some(Handle::Ring(axis))));
            }
        }
        _ => {}
    }
}

fn drag_update(
    models: &mut [Model],
    platform: &Vector3<common::units::Milimeters>,
    drag: &mut Drag,
    view: &View,
    pointer: Pos2,
    ctrl: bool,
) {
    let mut each = |f: &mut dyn FnMut(&mut Model, &StartState)| {
        for start in drag.models.iter() {
            if let Some(model) = models.iter_mut().find(|x| x.id == start.id) {
                start.transform.apply(model, platform);
                f(model, start);
                model.update_oob(platform);
            }
        }
    };

    match (drag.tool, drag.handle) {
        (Tool::Move, Handle::Axis(axis)) => {
            let dir = Vector3::ith(axis, 1.0);
            let Some(t) = view.axis_param(pointer, drag.center, dir) else {
                return;
            };
            let delta = snap(t - drag.start.x, MOVE_SNAP, ctrl);
            each(&mut |model, start| {
                model
                    .mesh
                    .set_position(start.transform.position + dir * delta);
            });
            drag.label = format!("{} {delta:+.2} mm", AXIS_NAMES[axis]);
        }
        (_, Handle::Body) => {
            let Some(hit) = view.plane_hit(pointer, drag.center, Vector3::z()) else {
                return;
            };
            let delta = (hit - drag.start).map(|x| snap(x, MOVE_SNAP, ctrl));
            let delta = Vector3::new(delta.x, delta.y, 0.0);
            each(&mut |model, start| {
                model.mesh.set_position(start.transform.position + delta);
            });
            drag.label = format!("X {:+.2}  Y {:+.2} mm", delta.x, delta.y);
        }
        (Tool::Scale, Handle::Axis(axis)) => {
            let dir = Vector3::ith(axis, 1.0);
            let Some(t) = view.axis_param(pointer, drag.center, dir) else {
                return;
            };
            if drag.start.x.abs() < 1e-4 {
                return;
            }

            let factor = snap(t / drag.start.x, SCALE_SNAP, ctrl).max(0.01);
            let mut uniform = true;
            each(&mut |model, start| {
                let mut scale = start.transform.scale;
                if model.ui.locked_scale {
                    scale *= factor;
                } else {
                    scale[axis] *= factor;
                    uniform = false;
                }
                model.mesh.set_scale(scale);
                place(model, start.center, start.bottom);
            });

            let axis = if uniform { "" } else { AXIS_NAMES[axis] };
            drag.label = format!("{axis} {:.1}%", factor * 100.0);
        }
        (Tool::Rotate, Handle::Ring(axis)) => {
            let Some(center_px) = view.project(drag.center) else {
                return;
            };
            let dir = Vector3::ith(axis, 1.0);
            let total = drag.angle.update(screen_angle(center_px, pointer));
            let angle = snap(total * ring_sign(view, dir), ROTATE_SNAP, ctrl);
            let rotation = Rotation3::from_axis_angle(&Unit::new_unchecked(dir), angle);

            each(&mut |model, start| {
                let [x, y, z] = start.transform.rotation.into();
                let (roll, pitch, yaw) =
                    (rotation * Rotation3::from_euler_angles(x, y, z)).euler_angles();
                model.mesh.set_rotation(Vector3::new(roll, pitch, yaw));
                let center = drag.center + rotation * (start.center - drag.center);
                place(model, center, start.bottom);
            });
            drag.label = format!("{} {:+.1}°", AXIS_NAMES[axis], angle.to_degrees());
        }
        _ => {}
    }
}

/// Ends the current drag, adding the changes to the undo history.
fn commit(app: &mut App) {
    let Some(drag) = app.state.gizmo.drag.take() else {
        return;
    };

    let actions = (drag.models.iter())
        .filter_map(|start| {
            let model = app.project.models.iter().find(|x| x.id == start.id)?;
            (Transform::of(model) != start.transform).then(|| Action::Model {
                id: start.id,
                action: ModelAction::Transform(start.transform),
            })
        })
        .collect::<Vec<_>>();

    if !actions.is_empty() {
        let name = match drag.tool {
            Tool::Rotate => "Rotated Models",
            Tool::Scale => "Scaled Models",
            _ => "Moved Models",
        };
        app.history.track(Action::Group { name, actions });
    }
}

/// Aborts the current drag, restoring the models to where they started.
fn cancel(app: &mut App) {
    let Some(drag) = app.state.gizmo.drag.take() else {
        return;
    };

    let platform = app.project.slice_config.platform_size;
    for start in drag.models {
        if let Some(model) = app.project.model(start.id) {
            start.transform.apply(model, &platform);
        }
    }
}

pub fn paint(app: &App, painter: &Painter, view: &View) {
    let tool = app.state.tool;
    let ids = selected_models(app);
    if !tool_handles(tool) || ids.is_empty() {
        return;
    }

    let gizmo = &app.state.gizmo;
    let center = match &gizmo.drag {
        Some(drag) if drag.handle != Handle::Body => drag.center,
        _ => {
            let (min, max) = selection_bounds(app, &ids).unwrap();
            (min + max) / 2.0
        }
    };
    let Some(center_px) = view.project(center) else {
        return;
    };
    let size = view.world_per_px(center) * HANDLE_PX;
    let active = gizmo.drag.as_ref().map(|x| x.handle);
    let state = |handle| (gizmo.hovered == Some(handle), active == Some(handle));

    match tool {
        Tool::Move | Tool::Scale => {
            // Draw the axes pointing away from the camera first.
            let mut axes = [0, 1, 2];
            axes.sort_by(|&a, &b| {
                let depth = |x| Vector3::ith(x, 1.0).dot(&view.towards_camera);
                depth(a).total_cmp(&depth(b))
            });

            for axis in axes {
                let Some(end) = view.project(center + Vector3::ith(axis, size)) else {
                    continue;
                };
                let (hovered, active) = state(Handle::Axis(axis));
                let color = handle_color(AXIS_COLORS[axis], hovered, active);
                let width: f32 = if hovered || active { 4.0 } else { 2.5 };

                if tool == Tool::Move {
                    draw_arrow(painter, center_px, end, color, width);
                } else {
                    painter.line_segment([center_px, end], Stroke::new(width, color));
                    let rect = Rect::from_center_size(end, vec2(1.0, 1.0) * (8.0 + width * 1.5));
                    painter.rect_filled(rect, 1.0, color);
                }
            }
            painter.circle_filled(center_px, 3.5, Color32::WHITE);
        }
        Tool::Rotate => {
            for axis in 0..3 {
                let (hovered, active) = state(Handle::Ring(axis));
                let color = handle_color(AXIS_COLORS[axis], hovered, active);
                let width: f32 = if hovered || active { 4.0 } else { 2.5 };
                let ring = view.ring(center, Vector3::ith(axis, 1.0), size);
                painter.add(Shape::line(ring, Stroke::new(width, color)));
            }

            if let Some(drag) = &gizmo.drag
                && let Handle::Ring(axis) = drag.handle
            {
                // Show the swept angle as a pie wedge from where the drag began.
                let total = drag.angle.total;
                let start = drag.angle.last - total;
                let steps = ((total.abs() / TAU * 64.0).ceil() as usize).clamp(1, 256);
                let radius = HANDLE_PX * 0.5;
                let mut points = vec![center_px];
                for i in 0..=steps {
                    let a = start + total * i as f32 / steps as f32;
                    points.push(center_px + vec2(a.cos(), -a.sin()) * radius);
                }
                let fill = AXIS_COLORS[axis].gamma_multiply(0.25);
                for tri in points[1..].windows(2) {
                    painter.add(Shape::convex_polygon(
                        vec![center_px, tri[0], tri[1]],
                        fill,
                        Stroke::NONE,
                    ));
                }
            }
        }
        _ => {}
    }

    if let Some(drag) = &gizmo.drag
        && !drag.label.is_empty()
        && let Some(pointer) = painter.ctx().pointer_latest_pos()
    {
        draw_label(painter, pointer, &drag.label);
    }
}

/// Short help text shown in the tool options bar.
pub fn hint(tool: Tool) -> Option<&'static str> {
    Some(match tool {
        Tool::Move => "Drag an arrow or the model itself. Ctrl snaps to 1 mm, Esc cancels.",
        Tool::Rotate => "Drag a ring to rotate. Ctrl snaps to 15°, Esc cancels.",
        Tool::Scale => "Drag a handle to scale. Ctrl snaps to 5%, Esc cancels.",
        _ => return None,
    })
}

/// Whether the gizmo should be hit before the pointer reaches the model under
/// it, used to keep model picking from stealing clicks on handles.
pub fn is_hovered(app: &App) -> bool {
    matches!(
        app.state.gizmo.hovered,
        Some(Handle::Axis(_) | Handle::Ring(_))
    ) || app.state.gizmo.drag.is_some()
}
