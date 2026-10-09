//! mslicer's look: Rooted Revival Mono on black, with phosphor green text and
//! red, sky blue and amber accents. The light theme keeps the same layout and
//! accents, darkened enough to read on white.

use std::sync::Arc;

use egui::{
    Color32, Context, CornerRadius, FontData, FontDefinitions, FontFamily, FontId, Margin, Shadow,
    Stroke, Style, TextStyle, Theme, Ui, Visuals, vec2,
};

use crate::include_asset;

const FONT_NAME: &str = "Rooted Revival Mono";

/// The colors used throughout the interface.
#[derive(Clone, Copy)]
pub struct Palette {
    /// Body text.
    pub text: Color32,
    /// Secondary text, hints and disabled labels.
    pub weak: Color32,
    /// Errors, failures and destructive actions.
    pub red: Color32,
    /// Selections, links and information.
    pub sky: Color32,
    /// Warnings, active widgets and things that need attention.
    pub amber: Color32,

    /// Behind everything.
    pub background: Color32,
    /// Panels and tab bodies.
    pub panel: Color32,
    /// Windows, menus and popups.
    pub window: Color32,
    /// Buttons and other widgets at rest.
    pub widget: Color32,
    pub widget_hovered: Color32,
    pub widget_active: Color32,
    /// Separators and widget outlines.
    pub line: Color32,
}

pub const DARK: Palette = Palette {
    text: Color32::from_rgb(0x00, 0xff, 0x00),
    weak: Color32::from_rgb(0x00, 0x9c, 0x1a),
    red: Color32::from_rgb(0xff, 0x3b, 0x3b),
    sky: Color32::from_rgb(0x5c, 0xd0, 0xff),
    amber: Color32::from_rgb(0xff, 0xb0, 0x00),

    background: Color32::from_rgb(0x00, 0x00, 0x00),
    panel: Color32::from_rgb(0x06, 0x08, 0x06),
    window: Color32::from_rgb(0x08, 0x0c, 0x08),
    widget: Color32::from_rgb(0x0b, 0x16, 0x0b),
    widget_hovered: Color32::from_rgb(0x10, 0x24, 0x10),
    widget_active: Color32::from_rgb(0x16, 0x30, 0x16),
    line: Color32::from_rgb(0x12, 0x3a, 0x16),
};

pub const LIGHT: Palette = Palette {
    text: Color32::from_rgb(0x0a, 0x2a, 0x0a),
    weak: Color32::from_rgb(0x4a, 0x6a, 0x4a),
    red: Color32::from_rgb(0xc4, 0x1e, 0x1e),
    sky: Color32::from_rgb(0x00, 0x6f, 0xa8),
    amber: Color32::from_rgb(0xa8, 0x62, 0x00),

    background: Color32::from_rgb(0xff, 0xff, 0xff),
    panel: Color32::from_rgb(0xf6, 0xf8, 0xf6),
    window: Color32::from_rgb(0xfb, 0xfc, 0xfb),
    widget: Color32::from_rgb(0xe8, 0xee, 0xe8),
    widget_hovered: Color32::from_rgb(0xdc, 0xe8, 0xdc),
    widget_active: Color32::from_rgb(0xcc, 0xde, 0xcc),
    line: Color32::from_rgb(0xc4, 0xd4, 0xc4),
};

impl Palette {
    pub fn for_theme(theme: Theme) -> Self {
        match theme {
            Theme::Dark => DARK,
            Theme::Light => LIGHT,
        }
    }

    /// The palette for the theme a ui is drawn with.
    pub fn of(ui: &Ui) -> Self {
        Self::for_ctx(ui.ctx())
    }

    pub fn for_ctx(ctx: &Context) -> Self {
        if ctx.style().visuals.dark_mode {
            DARK
        } else {
            LIGHT
        }
    }

    /// Text drawn on top of an accent colored fill.
    pub fn on_accent(&self) -> Color32 {
        self.background
    }
}

/// Registers the interface font, with the icon font as a fallback.
pub fn install_fonts(ctx: &Context) {
    let mut fonts = FontDefinitions::default();
    egui_phosphor::add_to_fonts(&mut fonts, egui_phosphor::Variant::Regular);

    let font = FontData::from_static(include_asset!("fonts/RootedRevivalMonoCore-Regular.ttf"));
    fonts.font_data.insert(FONT_NAME.to_owned(), Arc::new(font));

    // The custom font comes first, egui's own fonts and the icons are kept as
    // fallbacks for anything it doesn't cover.
    for family in [FontFamily::Proportional, FontFamily::Monospace] {
        let list = fonts.families.entry(family).or_default();
        list.retain(|x| x != FONT_NAME);
        list.insert(0, FONT_NAME.to_owned());
    }

    ctx.set_fonts(fonts);
}

/// The full interface style for a theme.
pub fn style(theme: Theme) -> Style {
    let palette = Palette::for_theme(theme);
    let mut style = Style {
        visuals: visuals(theme, &palette),
        ..Style::default()
    };

    style.text_styles = [
        (TextStyle::Small, FontId::proportional(11.0)),
        (TextStyle::Body, FontId::proportional(13.5)),
        (TextStyle::Button, FontId::proportional(13.5)),
        (TextStyle::Heading, FontId::proportional(17.0)),
        (TextStyle::Monospace, FontId::monospace(13.0)),
    ]
    .into();

    let spacing = &mut style.spacing;
    spacing.item_spacing = vec2(8.0, 6.0);
    spacing.button_padding = vec2(10.0, 4.0);
    spacing.interact_size = vec2(40.0, 22.0);
    spacing.window_margin = Margin::same(12);
    spacing.menu_margin = Margin::same(8);
    spacing.indent = 16.0;
    spacing.slider_width = 160.0;
    spacing.slider_rail_height = 4.0;
    spacing.combo_width = 120.0;
    spacing.icon_width = 15.0;
    spacing.icon_width_inner = 9.0;
    spacing.icon_spacing = 6.0;
    spacing.tooltip_width = 320.0;

    style
}

fn visuals(theme: Theme, palette: &Palette) -> Visuals {
    let mut visuals = match theme {
        Theme::Dark => Visuals::dark(),
        Theme::Light => Visuals::light(),
    };

    let radius = CornerRadius::same(5);
    let outline = Stroke::new(1.0_f32, palette.line);

    visuals.weak_text_color = Some(palette.weak);
    visuals.hyperlink_color = palette.sky;
    visuals.warn_fg_color = palette.amber;
    visuals.error_fg_color = palette.red;

    visuals.panel_fill = palette.panel;
    visuals.window_fill = palette.window;
    visuals.window_stroke = outline;
    visuals.window_corner_radius = CornerRadius::same(8);
    visuals.menu_corner_radius = CornerRadius::same(6);
    visuals.window_shadow = Shadow {
        offset: [0, 6],
        blur: 18,
        spread: 0,
        color: Color32::from_black_alpha(160),
    };
    visuals.popup_shadow = Shadow {
        offset: [0, 4],
        blur: 12,
        spread: 0,
        color: Color32::from_black_alpha(140),
    };

    visuals.extreme_bg_color = palette.background;
    visuals.text_edit_bg_color = Some(palette.background);
    visuals.faint_bg_color = palette.widget.gamma_multiply(0.6);
    visuals.code_bg_color = palette.widget;

    visuals.selection.bg_fill = palette.sky.gamma_multiply(0.28);
    visuals.selection.stroke = Stroke::new(1.0_f32, palette.sky);
    visuals.text_cursor.stroke = Stroke::new(2.0_f32, palette.text);

    visuals.collapsing_header_frame = true;
    visuals.slider_trailing_fill = true;
    visuals.striped = true;
    visuals.indent_has_left_vline = true;

    let widgets = &mut visuals.widgets;
    widgets.noninteractive.bg_fill = palette.panel;
    widgets.noninteractive.weak_bg_fill = palette.panel;
    widgets.noninteractive.bg_stroke = outline;
    widgets.noninteractive.fg_stroke = Stroke::new(1.0_f32, palette.text);
    widgets.noninteractive.corner_radius = radius;

    widgets.inactive.bg_fill = palette.widget;
    widgets.inactive.weak_bg_fill = palette.widget;
    widgets.inactive.bg_stroke = outline;
    widgets.inactive.fg_stroke = Stroke::new(1.0_f32, palette.text);
    widgets.inactive.corner_radius = radius;

    widgets.hovered.bg_fill = palette.widget_hovered;
    widgets.hovered.weak_bg_fill = palette.widget_hovered;
    widgets.hovered.bg_stroke = Stroke::new(1.0_f32, palette.text.gamma_multiply(0.7));
    widgets.hovered.fg_stroke = Stroke::new(1.5_f32, palette.text);
    widgets.hovered.corner_radius = radius;
    widgets.hovered.expansion = 0.0;

    widgets.active.bg_fill = palette.widget_active;
    widgets.active.weak_bg_fill = palette.widget_active;
    widgets.active.bg_stroke = Stroke::new(1.0_f32, palette.amber);
    widgets.active.fg_stroke = Stroke::new(1.5_f32, palette.amber);
    widgets.active.corner_radius = radius;
    widgets.active.expansion = 0.0;

    widgets.open.bg_fill = palette.widget_hovered;
    widgets.open.weak_bg_fill = palette.widget_hovered;
    widgets.open.bg_stroke = Stroke::new(1.0_f32, palette.sky);
    widgets.open.fg_stroke = Stroke::new(1.0_f32, palette.sky);
    widgets.open.corner_radius = radius;

    visuals
}

/// Docking area style matching the rest of the interface. Tabs are flat, with
/// the active one marked in the accent color.
pub fn dock_style(style: &Style) -> egui_dock::Style {
    let palette = if style.visuals.dark_mode { DARK } else { LIGHT };
    let mut dock = egui_dock::Style::from_egui(style);

    dock.main_surface_border_stroke = Stroke::NONE;
    dock.separator.width = 2.0;
    dock.separator.color_idle = palette.background;
    dock.separator.color_hovered = palette.weak;
    dock.separator.color_dragged = palette.amber;

    dock.tab_bar.bg_fill = palette.background;
    dock.tab_bar.hline_color = palette.line;
    dock.tab_bar.height = 28.0;
    dock.tab_bar.corner_radius = CornerRadius::ZERO;

    let tab = |bg_fill, text_color, outline_color| egui_dock::TabInteractionStyle {
        outline_color,
        corner_radius: CornerRadius {
            nw: 5,
            ne: 5,
            sw: 0,
            se: 0,
        },
        bg_fill,
        text_color,
    };
    dock.tab.active = tab(palette.panel, palette.text, palette.line);
    dock.tab.focused = tab(
        palette.panel,
        palette.text,
        palette.text.gamma_multiply(0.6),
    );
    dock.tab.inactive = tab(palette.background, palette.weak, Color32::TRANSPARENT);
    dock.tab.hovered = tab(palette.widget, palette.text, palette.line);
    dock.tab.active_with_kb_focus = dock.tab.focused.clone();
    dock.tab.inactive_with_kb_focus = dock.tab.hovered.clone();
    dock.tab.focused_with_kb_focus = dock.tab.focused.clone();
    dock.tab.hline_below_active_tab_name = true;
    dock.tab.tab_body.bg_fill = palette.panel;
    dock.tab.tab_body.stroke = Stroke::new(1.0_f32, palette.line);
    dock.tab.tab_body.inner_margin = Margin::same(10);

    dock.overlay.selection_color = palette.sky.gamma_multiply(0.4);
    dock
}

/// Applies the style for a theme, if it isn't applied already.
pub fn apply(ctx: &Context, theme: Theme) {
    let dark = matches!(theme, Theme::Dark);
    let applied = ctx.style().visuals.dark_mode == dark
        && ctx.style().visuals.weak_text_color == Some(Palette::for_theme(theme).weak);
    if !applied {
        ctx.set_style(style(theme));
    }
}
