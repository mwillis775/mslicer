use std::{collections::HashMap, sync::Arc};

use egui::Vec2;
use egui_phosphor::regular::{
    ARROW_CLOCKWISE, ARROWS_OUT, ARROWS_OUT_CARDINAL, CURSOR_CLICK, LINE_SEGMENT, SCISSORS,
    TRIANGLE,
};
use egui_tracing::EventCollector;
use nalgebra::{Vector2, Vector3};
use slicer::mesh::Mesh;

use crate::{
    core::{
        config::peripherals::Webhook,
        printer_camera::CameraState,
        project::model::ModelId,
        selected::{SelectedModel, SelectedPrinter, SelectedSupports},
    },
    interface::{cut::CutState, gizmo::GizmoState, tools::Tools},
};

#[derive(Default)]
pub struct UiState {
    pub event_collector: EventCollector,
    pub line_support_debug: Vec<[Vector3<f32>; 2]>,
    pub queue_reset_ui: bool,
    pub tool: Tool,

    // support stuff
    pub workspace: WorkspaceHover,
    pub hovered_geometry: Option<GeometryHit>,

    pub selected: SelectedModel,
    pub selected_printer: SelectedPrinter,
    pub selected_supports: SelectedSupports,
    pub support_preview: Option<Mesh>,
    /// Why a support can't be placed where the cursor is hovering.
    pub support_preview_error: Option<&'static str>,

    pub selected_remap_point: Option<u8>,

    // remote send ui
    pub working_address: String,
    pub working_filename: String,
    pub remote_print_connecting: RemotePrintConnectStatus,
    pub shared_webhook: Arc<SharedPrintCompletion>,
    /// Open camera views, keyed by mainboard ID.
    pub printer_cameras: HashMap<String, CameraState>,

    // slice preview
    pub preview_layer: usize,
    pub last_preview_layer: usize,
    pub preview_offset: Vector2<f32>,
    pub preview_scale: f32,
    pub layer_count: (usize, u8),

    pub anisotropic_aa: bool,

    pub tools: Tools,
    pub move_timeout: u32,

    // viewport manipulation tools
    pub gizmo: GizmoState,
    pub cut: CutState,
    /// Model under the pointer when the viewport context menu was opened.
    pub context_model: Option<ModelId>,
}

#[derive(Default, Clone, Copy, PartialEq, Eq)]
pub enum Tool {
    #[default]
    Select,
    Support,
    Orient,
    Move,
    Rotate,
    Scale,
    Cut,
}

#[derive(Default)]
pub struct SharedPrintCompletion {
    pub webhook: Webhook,
    pub alert: bool,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct GeometryHit {
    pub model: ModelId,
    pub support: bool,

    pub face: u32,
}

#[derive(Default)]
pub struct WorkspaceHover {
    pub is_moving: bool,
    pub aspect: f32,
    pub uv: Vector2<f32>,
}

#[derive(Default, PartialEq, Eq)]
pub enum RemotePrintConnectStatus {
    #[default]
    None,
    Connecting,
    Scanning,
}

impl WorkspaceHover {
    pub fn new(is_moving: bool, aspect: f32, uv: Vec2) -> Self {
        Self {
            is_moving,
            aspect,
            uv: Vector2::new(uv.x, uv.y),
        }
    }

    pub fn hovered(&self) -> bool {
        self.uv.x >= 0.0 && self.uv.y >= 0.0
    }
}

impl Tool {
    pub const ALL: [Self; 7] = [
        Self::Select,
        Self::Support,
        Self::Orient,
        Self::Move,
        Self::Rotate,
        Self::Scale,
        Self::Cut,
    ];

    pub fn icon(&self) -> &str {
        match self {
            Tool::Select => CURSOR_CLICK,
            Tool::Support => LINE_SEGMENT,
            Tool::Orient => TRIANGLE,
            Tool::Move => ARROWS_OUT_CARDINAL,
            Tool::Rotate => ARROW_CLOCKWISE,
            Tool::Scale => ARROWS_OUT,
            Tool::Cut => SCISSORS,
        }
    }

    pub fn name(&self) -> &str {
        match self {
            Tool::Select => "Select (Default)",
            Tool::Support => "Support Placement",
            Tool::Orient => "Orient to Face",
            Tool::Move => "Move",
            Tool::Rotate => "Rotate",
            Tool::Scale => "Scale",
            Tool::Cut => "Cut",
        }
    }

    /// Single letter shortcut, in addition to the tool's number key.
    pub fn letter(&self) -> Option<egui::Key> {
        Some(match self {
            Tool::Move => egui::Key::M,
            Tool::Rotate => egui::Key::R,
            Tool::Scale => egui::Key::S,
            Tool::Cut => egui::Key::C,
            _ => return None,
        })
    }

    /// Tools that select models when they are clicked.
    pub fn selects_models(&self) -> bool {
        matches!(
            self,
            Tool::Select | Tool::Move | Tool::Rotate | Tool::Scale | Tool::Cut
        )
    }
}
