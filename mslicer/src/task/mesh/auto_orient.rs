use clone_macro::clone;
use common::{
    progress::Progress,
    units::{Milimeter, Milimeters},
};
use nalgebra::Vector3;
use tools::supports::{
    orient::{Orientation, orient},
    settings::SupportSettings,
};
use tracing::info;

use crate::{
    core::{
        history::{Action, ModelAction, Transform},
        project::{
            model::{Model, ModelId},
            supports::{Anchor, Snapshot},
        },
    },
    task::{GenerateSupports, PollResult, Task, TaskApp, TaskStatus, thread::TaskThread},
};

pub struct AutoOrient {
    model: ModelId,
    name: String,
    /// Generate supports once the model has been oriented.
    generate: bool,

    progress: Progress,
    handle: TaskThread<Orientation>,
}

impl AutoOrient {
    /// Starts finding an orientation for a model that is easy to support.
    /// Returns None if the model's acceleration structures haven't been built
    /// yet.
    pub fn new(
        model: &Model,
        settings: &SupportSettings,
        platform: &Vector3<Milimeters>,
        generate: bool,
    ) -> Option<Self> {
        let bvh = model.bvh.clone()?;
        let platform = platform.map(|x| x.get::<Milimeter>());
        let mesh = model.mesh.clone();
        let settings = settings.clone();

        let progress = Progress::new();
        let handle = TaskThread::spawn(clone!([progress], move || {
            orient(&mesh, &bvh, &settings, platform, progress)
        }));

        Some(Self {
            model: model.id,
            name: model.name.clone(),
            generate,

            progress,
            handle,
        })
    }
}

impl Task for AutoOrient {
    fn poll(&mut self, app: &mut TaskApp) -> PollResult {
        (self.handle)
            .poll(app, "Failed to Orient Model")
            .into_poll_result(|orientation| {
                let Some(model) = app.project.models.iter_mut().find(|x| x.id == self.model) else {
                    return PollResult::complete();
                };

                let settings = &app.project.support_settings;
                let platform = app.project.slice_config.platform_size;
                info!(
                    "Oriented `{}` to {:.1?}°, cost {:.1}",
                    self.name,
                    orientation.rotation.map(f32::to_degrees),
                    orientation.cost
                );

                // Rotate in place, then sit the model at the support lift
                // height so there is room for supports under it.
                let old = Transform::of(model);
                let center = model.get_bounds_center();
                model.set_rotation(&platform, orientation.rotation);
                model.set_bounds_center(&platform, center);
                let (min, _) = model.mesh.bounds();
                let lift = settings.placement.lift - min.z;
                model.set_position(&platform, model.mesh.position() + Vector3::z() * lift);

                let mut actions = vec![Action::Model {
                    id: model.id,
                    action: ModelAction::Transform(old),
                }];

                // Supports placed for the old orientation won't make sense in
                // the new one.
                if !model.supports.is_empty() || !model.supports.failed().is_empty() {
                    let empty = Snapshot::new(Vec::new(), &Anchor::for_mesh(&model.mesh));
                    actions.push(Action::Supports {
                        model: model.id,
                        name: "Cleared Supports",
                        supports: model.supports.restore(empty),
                    });
                    app.state.selected_supports.clear();
                }

                app.history.track(Action::Group {
                    name: "Auto Oriented Model",
                    actions,
                });

                let result = PollResult::complete();
                match (self.generate)
                    .then(|| GenerateSupports::new(model, settings))
                    .flatten()
                {
                    Some(task) => result.with_task(task),
                    None => result,
                }
            })
    }

    fn status(&self) -> Option<TaskStatus<'_>> {
        Some(TaskStatus {
            name: "Orienting Model".into(),
            details: Some(format!("For `{}`", self.name)),
            progress: self.progress.progress(),
        })
    }
}
