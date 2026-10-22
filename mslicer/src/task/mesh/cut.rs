use std::f32::consts::PI;

use clone_macro::clone;
use common::{color::LinearRgb, progress::Progress};
use nalgebra::{Rotation3, Unit, Vector3};
use slicer::{
    cut::{CutPart, cut_mesh},
    mesh::Mesh,
};
use tracing::warn;

use crate::{
    core::{
        history::Action,
        project::{
            CollectionId,
            model::{Model, ModelId},
        },
        selected::SelectedModel,
        state::Tool,
    },
    task::{
        BuildAccelerationStructures, MeshDefective, MeshVolume, PollResult, Task, TaskApp,
        TaskStatus, thread::TaskThread,
    },
};

/// Gap left between the parts when they are laid out on the bed.
const PART_GAP: f32 = 5.0;

pub struct CutModel {
    parent: ModelId,
    name: String,
    color: LinearRgb<f32>,
    exposure: u8,
    collection: Option<CollectionId>,
    place_on_cut: bool,

    progress: Progress,
    handle: TaskThread<CutOutput>,
}

struct CutOutput {
    /// Parts paired with whether they are the upper part.
    parts: Vec<(Mesh, bool)>,
    open_loops: usize,
}

impl CutModel {
    pub fn new(
        model: &Model,
        origin: Vector3<f32>,
        normal: Vector3<f32>,
        [keep_above, keep_below]: [bool; 2],
        place_on_cut: bool,
    ) -> Self {
        let mesh = model.mesh.clone();
        let progress = Progress::new();

        let handle = TaskThread::spawn(clone!([progress], move || {
            progress.set_total(3);
            let vertices = (mesh.vertices().iter())
                .map(|v| mesh.transform(v))
                .collect::<Vec<_>>();
            progress.add_complete(1);

            let result = cut_mesh(&vertices, mesh.faces(), origin, normal);
            progress.add_complete(1);

            let mut parts = Vec::new();
            for (part, keep, upper) in [
                (result.below, keep_below, false),
                (result.above, keep_above, true),
            ] {
                if let Some(part) = part.filter(|_| keep) {
                    // The upper part's cap faces against the normal.
                    let cap = if upper { -normal } else { normal };
                    parts.push((build_mesh(part, cap, place_on_cut), upper));
                }
            }

            progress.set_finished();
            CutOutput {
                parts,
                open_loops: result.open_loops,
            }
        }));

        Self {
            parent: model.id,
            name: model.name.clone(),
            color: model.color,
            exposure: model.exposure,
            collection: model.collection,
            place_on_cut,

            progress,
            handle,
        }
    }
}

/// Turns one side of the cut into a mesh, keeping its place in the world
/// unless it is being laid down on its cut face.
fn build_mesh(part: CutPart, cap_normal: Vector3<f32>, place_on_cut: bool) -> Mesh {
    let CutPart {
        mut vertices,
        faces,
    } = part;

    let (min, max) = bounds(&vertices);
    let position = Vector3::new((min.x + max.x) / 2.0, (min.y + max.y) / 2.0, min.z);

    if place_on_cut {
        let down = -Vector3::z();
        let rotation = Rotation3::rotation_between(&cap_normal, &down)
            .unwrap_or_else(|| Rotation3::from_axis_angle(&Unit::new_unchecked(Vector3::x()), PI));
        vertices.iter_mut().for_each(|v| *v = rotation * *v);
    }

    let mut mesh = Mesh::new(vertices, faces);
    mesh.set_position(position);
    mesh
}

fn bounds(vertices: &[Vector3<f32>]) -> (Vector3<f32>, Vector3<f32>) {
    let start = (Vector3::repeat(f32::MAX), Vector3::repeat(f32::MIN));
    (vertices.iter()).fold(start, |(min, max), v| (min.inf(v), max.sup(v)))
}

impl Task for CutModel {
    fn poll(&mut self, app: &mut TaskApp) -> PollResult {
        self.handle
            .poll(app, "Failed to Cut Model")
            .into_poll_result(|output| {
                let mut result = PollResult::complete();
                let platform = app.project.slice_config.platform_size;

                if output.open_loops > 0 {
                    warn!(
                        "Cutting `{}` left {} open cross-section loops, the mesh is likely not watertight",
                        self.name, output.open_loops
                    );
                }

                let Some(index) = (app.project.models.iter()).position(|x| x.id == self.parent)
                else {
                    return result;
                };
                if output.parts.is_empty() {
                    return result;
                }

                let mut actions = Vec::new();
                let mut ids = Vec::new();
                let mut next_x = None::<f32>;

                for (i, (mesh, upper)) in output.parts.into_iter().enumerate() {
                    let suffix = if upper { "upper" } else { "lower" };
                    let mut model = Model::from_mesh(mesh)
                        .with_name(format!("{} ({suffix})", self.name))
                        .with_collection(self.collection)
                        .with_exposure(self.exposure);
                    if upper && i > 0 {
                        model.randomize_color();
                    } else {
                        model.color = self.color;
                    }

                    if self.place_on_cut {
                        model.align_to_bed();
                        // Set the parts down side by side so they don't overlap.
                        let (min, max) = model.mesh.bounds();
                        if let Some(x) = next_x {
                            let center = model.get_bounds_center();
                            let target = x + (max.x - min.x) / 2.0;
                            model.set_bounds_center(
                                &platform,
                                Vector3::new(target, center.y, center.z),
                            );
                        }
                        let (_, max) = model.mesh.bounds();
                        next_x = Some(max.x + PART_GAP);
                    }

                    model.update_oob(&platform);
                    result = result
                        .with_task(MeshDefective::new(&model))
                        .with_task(MeshVolume::new(&model))
                        .with_task(BuildAccelerationStructures::new(&model));

                    ids.push(model.id);
                    actions.push(Action::ModelAdded { id: model.id });
                    app.project.models.insert(index + 1 + i, model);
                }

                let parent = Box::new(app.project.models.remove(index));
                actions.insert(0, Action::ModelRemoved {
                    index,
                    model: parent,
                });
                app.history.track(Action::Group {
                    name: "Cut Model",
                    actions,
                });

                app.state.selected = SelectedModel::Models(ids.into_iter().collect());
                app.state.tool = Tool::Move;
                result
            })
    }

    fn status(&self) -> Option<TaskStatus<'_>> {
        Some(TaskStatus {
            name: "Cutting Model".into(),
            details: Some(format!("Cutting `{}`", self.name)),
            progress: self.progress.progress(),
        })
    }
}
