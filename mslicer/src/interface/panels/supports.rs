use std::collections::HashMap;

use const_format::concatcp;
use egui::{Align, Checkbox, Context, DragValue, Layout, Response, RichText, Slider, Ui, Widget};
use egui_phosphor::regular::{
    ARROW_COUNTER_CLOCKWISE, ARROWS_CLOCKWISE, COLUMNS, COMPASS, GRID_FOUR, MAGIC_WAND, NEEDLE,
    PAINT_BUCKET, PUSH_PIN, SELECTION_INVERSE, SPARKLE, TRASH, TREE, WARNING,
};
use nalgebra::Vector3;
use slicer::builder::MeshBuilder;
use tools::supports::{
    Contact, Support, SupportId,
    collider::Collider,
    generate::new_support,
    mesh::SupportMesh,
    routing::Router,
    settings::{MIN_TIP_DIAMETER, SizePreset, SupportSettings, SupportStyle},
};

use crate::{
    core::{
        App,
        history::ModelAction,
        project::{
            model::ModelId,
            supports::{Anchor, Snapshot},
        },
        selected::SelectedModel,
        state::Tool,
    },
    interface::{
        components::{grid, primary_button, section},
        theme::Palette,
    },
    task::{AutoOrient, GenerateSupports},
};

const OVERHANG_TIP: &str = "Surfaces that are within this angle of horizontal are supported.";
const SPACING_TIP: &str = "Target distance between supports on flat overhangs. Steeper overhangs get up to 50% more space.";
const MIN_SPACING_TIP: &str = "Minimum distance between any two supports, used for islands.";
const LIFT_TIP: &str =
    "Before generating, models are raised so they are at least this far above the build plate.";
const ISLANDS_TIP: &str =
    "Support parts of the model that would otherwise start printing in mid-air.";
const EDGES_TIP: &str = "Support sharp, downward pointing creases.";
const FACES_TIP: &str = "Support downward facing surfaces.";
const TIP_DIAMETER_TIP: &str =
    "Diameter of the ball that touches the model. Smaller leaves smaller marks, but holds less.";
const TIP_LENGTH_TIP: &str = "Length of the tapered section between the tip and the shaft.";
const PENETRATION_TIP: &str = "How far the tip is sunk into the model, for a stronger connection.";
const CLEARANCE_TIP: &str = "Minimum gap between a support's shaft and the model.";
const TIP_ANGLE_TIP: &str = "Maximum angle of the tip from vertical.";
const SHAFT_ANGLE_TIP: &str =
    "Maximum angle a shaft can lean from vertical to get around the model.";
const REACH_TIP: &str = "Maximum horizontal distance a shaft can lean to get around the model.";
const LANDING_TIP: &str = "If a support can't reach the build plate, let it stand on the model instead. This leaves marks on upward facing surfaces.";
const RAFT_TIP: &str = "A plate under all the supports, so they stick to the build plate better.";
const CHAMFER_TIP: &str = "Bevel the edge of the raft so it's easier to pry off the build plate.";
const BRACING_TIP: &str = "Connect tall support columns with diagonal struts so they don't wobble.";
const APPLY_TIP: &str =
    "Apply the current style and size to the existing supports, re-routing them.";
const AGGRESSIVENESS_TIP: &str = "How eagerly supports are added. Higher places supports closer together and on gentler overhangs, lower uses fewer supports that are easier to clean up.";
const CONTACT_TIP: &str = "How fine the point touching the model is. Finer tips leave smaller marks and snap off cleaner, thicker tips hold heavier parts.";
const BRANCH_ANGLE_TIP: &str = "Maximum angle of a branch from vertical.";
const MERGE_DISTANCE_TIP: &str = "Branches further apart than this aren't merged.";
const TRUNK_TIP: &str = "Trunks thicken as branches merge into them, up to this diameter.";
const GAP_FILL_TIP: &str = "Spots that can't be supported normally, like gaps too narrow for a support tip or pockets with no way down to the build plate, are filled in with solid resin instead. The fill becomes part of the print.";
const GAP_FILL_HEIGHT_TIP: &str =
    "Gaps taller than this aren't filled, so whole cavities aren't made solid. Zero for no limit.";
const ORIENT_TIP: &str = "Rotate the model so it needs as few supports as possible, with as much of it as possible able to reach the build plate. The model keeps its spin around Z, and existing supports are cleared.";
const ORIENT_GENERATE_TIP: &str = "Auto orient the model, then generate supports for it.";

pub fn ui(app: &mut App, ui: &mut Ui, _ctx: &Context) {
    let palette = Palette::of(ui);
    let targets = targets(app);
    let target_text = match &app.state.selected {
        SelectedModel::None => "all models".to_owned(),
        _ => format!("{} selected model{}", targets.len(), plural(targets.len())),
    };

    let (mut supports, mut fills, mut fill_area, mut failed) = (0, 0, 0.0, 0);
    let models = (targets.iter()).filter_map(|&id| app.project.models.iter().find(|x| x.id == id));
    for model in models {
        failed += model.supports.failed().len();
        for support in model.supports.list() {
            match &support.route.fill {
                Some(fill) => (fills, fill_area) = (fills + 1, fill_area + fill.area()),
                None => supports += 1,
            }
        }
    }

    let fill_text = (fills > 0)
        .then(|| {
            format!(
                " and {fills} gap fill{} ({fill_area:.1} mm²)",
                plural(fills)
            )
        })
        .unwrap_or_default();
    ui.label(format!(
        "{supports} support{}{fill_text} on {target_text}.",
        plural(supports)
    ));
    if failed > 0 {
        ui.label(RichText::new(format!(
            "{WARNING} {failed} spot{} couldn't be supported, shown in red.",
            plural(failed)
        )).color(palette.amber))
        .on_hover_text("These are usually inside enclosed areas, or in gaps too narrow for a support. Try auto orienting the model, using smaller supports, allowing model landing, or turning on gap fill.");
    }

    let settings = &mut app.project.support_settings;
    section(ui, "Style");
    style_ui(settings, ui);

    section(ui, "Tuning");
    tuning_ui(settings, ui);

    ui.add_space(4.0);
    ui.checkbox(
        &mut settings.gap_fill.enabled,
        concatcp!(PAINT_BUCKET, " Fill gaps that can't be supported"),
    )
    .on_hover_text(GAP_FILL_TIP);

    ui.add_space(8.0);
    ui.horizontal_wrapped(|ui| {
        ui.add_enabled_ui(!targets.is_empty(), |ui| {
            if primary_button(ui, concatcp!(SPARKLE, " Generate")).clicked() {
                generate(app, &targets);
            }

            if ui
                .button(concatcp!(ARROWS_CLOCKWISE, " Apply"))
                .on_hover_text(APPLY_TIP)
                .clicked()
            {
                apply(app, &targets);
            }

            if ui
                .button(RichText::new(concatcp!(TRASH, " Clear")).color(palette.red))
                .clicked()
            {
                clear(app, &targets);
            }
        });
    });

    ui.horizontal_wrapped(|ui| {
        ui.add_enabled_ui(!targets.is_empty(), |ui| {
            if ui
                .button(concatcp!(COMPASS, " Auto Orient"))
                .on_hover_text(ORIENT_TIP)
                .clicked()
            {
                auto_orient(app, &targets, false);
            }

            if ui
                .button(concatcp!(MAGIC_WAND, " Orient & Generate"))
                .on_hover_text(ORIENT_GENERATE_TIP)
                .clicked()
            {
                auto_orient(app, &targets, true);
            }
        });
    });

    section(ui, "Selection");
    selection_ui(app, ui);

    ui.add_space(4.0);
    section(ui, "Advanced");
    settings_ui(&mut app.project.support_settings, ui);
}

fn style_icon(style: SupportStyle) -> &'static str {
    match style {
        SupportStyle::Standard => COLUMNS,
        SupportStyle::Tree => TREE,
        SupportStyle::Lattice => GRID_FOUR,
        SupportStyle::Mini => NEEDLE,
        SupportStyle::Needle => PUSH_PIN,
        SupportStyle::Fill => PAINT_BUCKET,
    }
}

/// A row of buttons for picking the support style.
fn style_ui(settings: &mut SupportSettings, ui: &mut Ui) {
    ui.horizontal_wrapped(|ui| {
        ui.spacing_mut().item_spacing.x = 4.0;
        for style in SupportStyle::ALL {
            let selected = settings.style == style;
            let text = format!("{} {}", style_icon(style), style.name());
            let response = ui
                .selectable_label(selected, text)
                .on_hover_text(style.description());
            (response.clicked() && !selected).then(|| settings.apply_style(style));
        }
    });

    ui.label(RichText::new(settings.style.description()).weak());
}

/// The settings most worth adjusting: size, how many supports to add and how
/// fine their tips are.
fn tuning_ui(settings: &mut SupportSettings, ui: &mut Ui) {
    let palette = Palette::of(ui);
    ui.horizontal_wrapped(|ui| {
        ui.label("Size");
        ui.add_space(8.0);
        ui.spacing_mut().item_spacing.x = 4.0;

        let current = SizePreset::matching(&settings.size, settings.style);
        for preset in SizePreset::ALL {
            (ui.selectable_label(current == Some(preset), preset.name())
                .clicked())
            .then(|| settings.size = preset.size(settings.style));
        }
        (current.is_none()).then(|| ui.label(RichText::new("Custom").color(palette.amber)));
    });
    ui.add_space(4.0);

    let placement = &settings.placement;
    let detail = format!(
        "every {:.1} mm, under {:.0}°",
        placement.spacing, placement.overhang_angle
    );
    let mut value = settings.aggressiveness() * 100.0;
    let slider = Slider::new(&mut value, 0.0..=100.0)
        .suffix("%")
        .fixed_decimals(0);
    if slider_row(ui, ("Aggressiveness", AGGRESSIVENESS_TIP), &detail, slider).changed() {
        settings.set_aggressiveness(value / 100.0);
    }

    let max = settings.size.shaft_diameter.max(MIN_TIP_DIAMETER + 0.01);
    let fineness = (settings.size.tip_diameter - MIN_TIP_DIAMETER) / (max - MIN_TIP_DIAMETER);
    let detail = match fineness {
        x if x < 0.25 => "fine, smallest marks",
        x if x < 0.6 => "balanced",
        _ => "strong, for heavy parts",
    };
    let mut diameter = settings.size.tip_diameter;
    let slider = Slider::new(&mut diameter, MIN_TIP_DIAMETER..=max)
        .suffix(" mm")
        .fixed_decimals(2)
        .step_by(0.01);
    if slider_row(ui, ("Contact Tip", CONTACT_TIP), detail, slider).changed() {
        settings.set_tip_diameter(diameter);
    }
}

/// A label with a short description of the current value, above a slider that
/// fills the available width.
fn slider_row(ui: &mut Ui, (label, tip): (&str, &str), detail: &str, slider: Slider) -> Response {
    ui.horizontal(|ui| {
        ui.label(label).on_hover_text(tip);
        ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
            ui.label(RichText::new(detail).small().weak())
        });
    });

    ui.scope(|ui| {
        // Leaves room for the value box next to the slider.
        ui.spacing_mut().slider_width = (ui.available_width() - 80.0).max(60.0);
        slider.ui(ui).on_hover_text(tip)
    })
    .inner
}

fn plural(count: usize) -> &'static str {
    ["s", ""][(count == 1) as usize]
}

/// The models support actions apply to: the selection, or every visible model
/// if nothing is selected.
fn targets(app: &App) -> Vec<ModelId> {
    let models = app.project.models.iter().filter(|x| !x.hidden);
    match &app.state.selected {
        SelectedModel::None => models.map(|x| x.id).collect(),
        SelectedModel::Models(set) => models
            .filter(|x| set.contains(&x.id))
            .map(|x| x.id)
            .collect(),
        SelectedModel::Collection(id) => models
            .filter(|x| x.collection == Some(*id))
            .map(|x| x.id)
            .collect(),
    }
}

fn generate(app: &mut App, targets: &[ModelId]) {
    let settings = &app.project.support_settings;
    let platform = app.project.slice_config.platform_size;
    let lift = settings.placement.lift;

    for &id in targets {
        let Some(model) = app.project.models.iter_mut().find(|x| x.id == id) else {
            continue;
        };

        // Raise the model so there is room for supports under it.
        let (min, _) = model.mesh.bounds();
        if min.z < lift {
            let old = model.mesh.position();
            model.set_position(&platform, old + Vector3::z() * (lift - min.z));
            app.history.track_model(id, ModelAction::Position(old));
        }

        match GenerateSupports::new(model, settings) {
            Some(task) => app.tasks.add(task),
            None => tracing::warn!(
                "Acceleration structures for `{}` aren't ready yet",
                model.name
            ),
        }
    }
}

fn auto_orient(app: &mut App, targets: &[ModelId], generate: bool) {
    let settings = &app.project.support_settings;
    let platform = &app.project.slice_config.platform_size;
    for &id in targets {
        let Some(model) = app.project.models.iter().find(|x| x.id == id) else {
            continue;
        };

        match AutoOrient::new(model, settings, platform, generate) {
            Some(task) => app.tasks.add(task),
            None => tracing::warn!(
                "Acceleration structures for `{}` aren't ready yet",
                model.name
            ),
        }
    }
}

fn clear(app: &mut App, targets: &[ModelId]) {
    for &id in targets {
        let Some(model) = app.project.models.iter_mut().find(|x| x.id == id) else {
            continue;
        };

        if !model.supports.is_empty() || !model.supports.failed().is_empty() {
            let old = model
                .supports
                .restore(Snapshot::new(Vec::new(), &Anchor::for_mesh(&model.mesh)));
            app.history.track_supports(id, "Cleared Supports", old);
        }
    }

    app.state.selected_supports.clear();
}

/// Applies the current style and size to the existing supports.
fn apply(app: &mut App, targets: &[ModelId]) {
    let settings = &app.project.support_settings;
    let (style, size) = (settings.style, settings.size);
    for &id in targets {
        let Some(model) = app.project.models.iter_mut().find(|x| x.id == id) else {
            continue;
        };

        // Gap fill columns keep their own shape.
        let fill = |x: &&Support| x.style == SupportStyle::Fill;
        if (model.supports.list().iter())
            .filter(|x| !fill(x))
            .all(|x| x.size == size && x.style == style)
        {
            continue;
        }

        // Clearing the routed transform makes every support get re-routed
        // (and trees re-grown) with its new style and size.
        let mut snapshot = model.supports.snapshot();
        for support in snapshot.supports.iter_mut().filter(|x| !fill(&&**x)) {
            support.style = style;
            support.size = size;
        }
        snapshot.routed_for = None;

        let old = model.supports.restore(snapshot);
        app.history.track_supports(id, "Changed Supports", old);
    }
}

fn selection_ui(app: &mut App, ui: &mut Ui) {
    let selected = &app.state.selected_supports;
    let count = selected.count();

    if count == 0 {
        let hint = if app.state.tool == Tool::Support {
            "Click on a model to add a support."
        } else {
            "Use the support tool to add supports by hand, or click a support with the select tool to select it."
        };
        ui.label(RichText::new(hint).weak());
        return;
    }

    let models = selected.model_count();
    ui.label(format!(
        "{count} support{} selected across {models} model{}.",
        plural(count),
        plural(models),
    ));

    ui.horizontal_wrapped(|ui| {
        if ui
            .button(concatcp!(TRASH, " Delete"))
            .on_hover_text("Shortcut: Delete")
            .clicked()
        {
            delete_selected(app);
        }

        (ui.button(concatcp!(SELECTION_INVERSE, " Deselect"))
            .clicked())
        .then(|| app.state.selected_supports.clear());
    });
}

pub fn delete_selected(app: &mut App) {
    let mut by_model = HashMap::<ModelId, Vec<SupportId>>::new();
    for support in app.state.selected_supports.iter() {
        by_model
            .entry(support.model)
            .or_default()
            .push(support.support);
    }

    for (id, supports) in by_model {
        let Some(model) = app.project.model(id) else {
            continue;
        };

        let old = model.supports.snapshot();
        model.supports.remove(&supports);
        app.history.track_supports(id, "Deleted Supports", old);
    }

    app.state.selected_supports.clear();
}

fn settings_ui(settings: &mut SupportSettings, ui: &mut Ui) {
    let mm = |value: &mut f32, ui: &mut Ui| {
        DragValue::new(value)
            .suffix(" mm")
            .speed(0.01)
            .range(0.0..=f32::MAX)
            .ui(ui)
    };
    let degrees = |value: &mut f32, ui: &mut Ui| {
        DragValue::new(value)
            .suffix("°")
            .speed(0.1)
            .range(0.0..=89.0)
            .ui(ui)
    };

    ui.collapsing("Tip & Shaft", |ui| {
        grid("support_tip").show(ui, |ui| {
            let size = &mut settings.size;
            ui.label("Tip Diameter").on_hover_text(TIP_DIAMETER_TIP);
            mm(&mut size.tip_diameter, ui);
            ui.end_row();

            ui.label("Tip Length").on_hover_text(TIP_LENGTH_TIP);
            mm(&mut size.tip_length, ui);
            ui.end_row();

            ui.label("Tip Penetration").on_hover_text(PENETRATION_TIP);
            mm(&mut size.tip_penetration, ui);
            ui.end_row();

            ui.label("Shaft Diameter");
            mm(&mut size.shaft_diameter, ui);
            ui.end_row();
        });
    });

    ui.collapsing("Placement", |ui| {
        grid("support_placement").show(ui, |ui| {
            let placement = &mut settings.placement;
            ui.label("Overhang Angle").on_hover_text(OVERHANG_TIP);
            degrees(&mut placement.overhang_angle, ui);
            ui.end_row();

            ui.label("Spacing").on_hover_text(SPACING_TIP);
            mm(&mut placement.spacing, ui);
            ui.end_row();

            ui.label("Minimum Spacing").on_hover_text(MIN_SPACING_TIP);
            mm(&mut placement.min_spacing, ui);
            ui.end_row();

            ui.label("Lift").on_hover_text(LIFT_TIP);
            mm(&mut placement.lift, ui);
            ui.end_row();

            ui.label("Support");
            ui.horizontal(|ui| {
                Checkbox::new(&mut placement.islands, "Islands")
                    .ui(ui)
                    .on_hover_text(ISLANDS_TIP);
                Checkbox::new(&mut placement.edges, "Edges")
                    .ui(ui)
                    .on_hover_text(EDGES_TIP);
                Checkbox::new(&mut placement.faces, "Faces")
                    .ui(ui)
                    .on_hover_text(FACES_TIP);
            });
            ui.end_row();
        });
    });

    ui.collapsing("Routing", |ui| {
        grid("support_routing").show(ui, |ui| {
            let routing = &mut settings.routing;
            ui.label("Clearance").on_hover_text(CLEARANCE_TIP);
            mm(&mut routing.clearance, ui);
            ui.end_row();

            ui.label("Max Tip Angle").on_hover_text(TIP_ANGLE_TIP);
            degrees(&mut routing.max_tip_angle, ui);
            ui.end_row();

            ui.label("Max Shaft Angle").on_hover_text(SHAFT_ANGLE_TIP);
            degrees(&mut routing.max_shaft_angle, ui);
            ui.end_row();

            ui.label("Max Reach").on_hover_text(REACH_TIP);
            mm(&mut routing.max_reach, ui);
            ui.end_row();

            ui.label("Model Landing").on_hover_text(LANDING_TIP);
            ui.checkbox(&mut routing.model_landing, "");
            ui.end_row();
        });
    });

    ui.collapsing("Base & Raft", |ui| {
        grid("support_base").show(ui, |ui| {
            let base = &mut settings.base;
            ui.label("Base Diameter");
            mm(&mut base.base_diameter, ui);
            ui.end_row();

            ui.label("Base Height");
            mm(&mut base.base_height, ui);
            ui.end_row();

            ui.label("Raft").on_hover_text(RAFT_TIP);
            ui.checkbox(&mut base.raft, "");
            ui.end_row();

            ui.add_enabled_ui(base.raft, |ui| ui.label("Raft Thickness"));
            ui.add_enabled_ui(base.raft, |ui| mm(&mut base.raft_thickness, ui));
            ui.end_row();

            ui.add_enabled_ui(base.raft, |ui| ui.label("Raft Margin"));
            ui.add_enabled_ui(base.raft, |ui| mm(&mut base.raft_margin, ui));
            ui.end_row();

            ui.add_enabled_ui(base.raft, |ui| {
                ui.label("Raft Chamfer").on_hover_text(CHAMFER_TIP)
            });
            ui.add_enabled_ui(base.raft, |ui| ui.checkbox(&mut base.raft_chamfer, ""));
            ui.end_row();
        });
    });

    ui.collapsing("Bracing", |ui| {
        grid("support_bracing").show(ui, |ui| {
            let bracing = &mut settings.bracing;
            ui.label("Bracing").on_hover_text(BRACING_TIP);
            ui.checkbox(&mut bracing.enabled, "");
            ui.end_row();

            ui.add_enabled_ui(bracing.enabled, |ui| ui.label("Diameter"));
            ui.add_enabled_ui(bracing.enabled, |ui| mm(&mut bracing.diameter, ui));
            ui.end_row();

            ui.add_enabled_ui(bracing.enabled, |ui| ui.label("Min Column Height"));
            ui.add_enabled_ui(bracing.enabled, |ui| mm(&mut bracing.min_height, ui));
            ui.end_row();

            ui.add_enabled_ui(bracing.enabled, |ui| ui.label("Max Distance"));
            ui.add_enabled_ui(bracing.enabled, |ui| mm(&mut bracing.max_distance, ui));
            ui.end_row();
        });
    });

    (settings.style == SupportStyle::Tree).then(|| {
        ui.collapsing("Tree", |ui| {
            grid("support_tree").show(ui, |ui| {
                let tree = &mut settings.tree;
                ui.label("Branch Angle").on_hover_text(BRANCH_ANGLE_TIP);
                degrees(&mut tree.branch_angle, ui);
                ui.end_row();

                ui.label("Merge Distance").on_hover_text(MERGE_DISTANCE_TIP);
                mm(&mut tree.merge_distance, ui);
                ui.end_row();

                ui.label("Max Trunk Diameter").on_hover_text(TRUNK_TIP);
                mm(&mut tree.max_trunk_diameter, ui);
                ui.end_row();
            });
        })
    });

    ui.collapsing("Gap Fill", |ui| {
        grid("support_gap_fill").show(ui, |ui| {
            let gap_fill = &mut settings.gap_fill;
            ui.label("Gap Fill").on_hover_text(GAP_FILL_TIP);
            ui.checkbox(&mut gap_fill.enabled, "");
            ui.end_row();

            ui.add_enabled_ui(gap_fill.enabled, |ui| {
                ui.label("Max Height").on_hover_text(GAP_FILL_HEIGHT_TIP)
            });
            ui.add_enabled_ui(gap_fill.enabled, |ui| mm(&mut gap_fill.max_height, ui));
            ui.end_row();
        });
    });

    ui.collapsing("Mesh", |ui| {
        grid("support_mesh").show(ui, |ui| {
            ui.label("Resolution")
                .on_hover_text("Number of segments around each support.");
            DragValue::new(&mut settings.resolution)
                .range(6..=64)
                .ui(ui);
            ui.end_row();
        });
    });

    ui.add_space(4.0);
    let defaults = SupportSettings::for_style(settings.style);
    ui.add_enabled_ui(*settings != defaults, |ui| {
        (ui.button(concatcp!(ARROW_COUNTER_CLOCKWISE, " Reset to Defaults"))
            .on_hover_text("Reset every setting to the defaults for this style.")
            .clicked())
        .then(|| *settings = defaults.clone());
    });

    settings.sanitize();
}

/// Shows a preview of the support that would be placed where the cursor is
/// hovering, adding it if `clicked`.
pub fn manual_support_placement(app: &mut App, clicked: bool) {
    app.state.support_preview = None;
    app.state.support_preview_error = None;
    if app.state.workspace.is_moving {
        return;
    }

    let Some((origin, direction)) = app.hovered_ray() else {
        return;
    };

    // Find the closest model under the cursor.
    let hit = (app.project.models.iter().enumerate())
        .filter(|(_, x)| !x.hidden)
        .filter_map(|(i, model)| {
            let hit = model
                .bvh
                .as_ref()?
                .intersect_ray(&model.mesh, origin, direction)?;
            Some((i, hit))
        })
        .min_by(|a, b| a.1.t.total_cmp(&b.1.t));
    let Some((index, hit)) = hit else { return };

    let settings = &app.project.support_settings;
    let model = &mut app.project.models[index];
    let anchor = Anchor::for_mesh(&model.mesh);
    let mesh = anchor.support_space_mesh(&model.mesh);
    let bvh = model.bvh.as_ref().unwrap();

    let point = anchor.to_support_space(model.mesh.transform(&hit.position));
    let normal = model
        .mesh
        .transform_normal(&model.mesh.normal(hit.face))
        .normalize();
    let contact = Contact::new(point, anchor.direction_to_support_space(normal));

    let collider = Collider::new(&mesh, bvh);
    let router = Router::new(&collider, settings, anchor.plate_z());
    let support = match new_support(&collider, &router, settings, contact) {
        Ok(support) => support,
        Err(err) => {
            app.state.support_preview_error = Some(err.description());
            return;
        }
    };

    // Preview is drawn in world space.
    let mut preview = SupportMesh::new(settings, anchor.plate_z());
    preview.add_support(&support);
    if let Some(preview) = preview.finish() {
        let mut builder = MeshBuilder::new();
        let (vertices, faces) = builder.raw_mut();
        vertices.extend(preview.vertices().iter().map(|&x| anchor.to_world(x)));
        faces.extend_from_slice(preview.faces());
        app.state.support_preview = Some(builder.build());
    }

    if clicked {
        let old = model.supports.snapshot();
        model.supports.add(support, &model.mesh);
        app.history.track_supports(model.id, "Added Support", old);
    }
}
