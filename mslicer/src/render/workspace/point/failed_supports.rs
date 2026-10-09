use nalgebra::Vector4;

use crate::{
    core::{App, project::supports::Anchor},
    render::workspace::point::{Point, PointGenerator},
};

/// Marks spots that auto generation couldn't support.
pub struct FailedSupportsDispatch {
    points: Vec<Point>,
}

impl FailedSupportsDispatch {
    pub fn new() -> Self {
        Self { points: Vec::new() }
    }
}

impl PointGenerator for FailedSupportsDispatch {
    fn generate_points(&mut self, app: &mut App) {
        self.points.clear();
        for model in app.project.models.iter().filter(|x| !x.hidden) {
            let anchor = Anchor::for_mesh(&model.mesh);
            self.points
                .extend(model.supports.failed().iter().map(|&point| Point {
                    position: anchor.to_world(point),
                    radius: 0.6,
                    color: Vector4::new(1.0, 0.23, 0.23, 0.9),
                }));
        }
    }

    fn points(&self) -> &[Point] {
        &self.points
    }
}
