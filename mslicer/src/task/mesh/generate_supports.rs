use clone_macro::clone;
use common::progress::Progress;
use tools::supports::{
    collider::Collider,
    generate::{Generated, generate},
    settings::SupportSettings,
};
use tracing::info;

use crate::{
    core::project::{
        model::{Model, ModelId},
        supports::{Anchor, Snapshot},
    },
    task::{PollResult, Task, TaskApp, TaskStatus, thread::TaskThread},
};

pub struct GenerateSupports {
    model: ModelId,
    name: String,
    anchor: Anchor,

    progress: Progress,
    handle: TaskThread<Generated>,
}

impl GenerateSupports {
    /// Starts generating supports for a model. Returns None if the model's
    /// acceleration structures haven't been built yet.
    pub fn new(model: &Model, settings: &SupportSettings) -> Option<Self> {
        let bvh = model.bvh.clone()?;
        let anchor = Anchor::for_mesh(&model.mesh);
        let mesh = anchor.support_space_mesh(&model.mesh);
        let settings = settings.clone();

        let progress = Progress::new();
        let handle = TaskThread::spawn(clone!([progress], move || {
            let collider = Collider::new(&mesh, &bvh);
            generate(&collider, &settings, anchor.plate_z(), progress)
        }));

        Some(Self {
            model: model.id,
            name: model.name.clone(),
            anchor,

            progress,
            handle,
        })
    }
}

impl Task for GenerateSupports {
    fn poll(&mut self, app: &mut TaskApp) -> PollResult {
        (self.handle)
            .poll(app, "Failed to Generate Supports")
            .into_poll_result(|generated| {
                let Some(model) = app.project.models.iter_mut().find(|x| x.id == self.model) else {
                    return PollResult::complete();
                };

                info!(
                    "Generated {} supports for `{}`, {} spots could not be supported",
                    generated.supports.len(),
                    self.name,
                    generated.failed.len()
                );

                let snapshot = Snapshot::new(generated.supports, &self.anchor);
                let old = model.supports.restore(snapshot);
                model
                    .supports
                    .set_failed(generated.failed.into_iter().map(|x| x.0).collect());

                app.history
                    .track_supports(model.id, "Generated Supports", old);
                app.state.selected_supports.clear();
                PollResult::complete()
            })
    }

    fn status(&self) -> Option<TaskStatus<'_>> {
        Some(TaskStatus {
            name: "Generating Supports".into(),
            details: Some(format!("For `{}`", self.name)),
            progress: self.progress.progress(),
        })
    }
}
