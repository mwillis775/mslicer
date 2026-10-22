//! Support generation settings. All lengths are in millimeters and all angles
//! are in degrees.

use common::serde::{Deserializer, Serializer};

/// Version of the serialized support format, see
/// [`SupportSettings::deserialize`]. Bumped whenever the format changes.
///
/// 1. Initial version.
/// 2. Support styles, tree settings and per-point shaft radii.
/// 3. Gap fill settings and fill columns.
/// 4. Fills are stored as the shape of the whole gap, see
///    [`crate::supports::fill`].
pub const FORMAT_VERSION: u16 = 4;

#[derive(Debug, Clone, PartialEq)]
pub struct SupportSettings {
    /// Style of newly created supports. Like the size, each support stores
    /// its own style.
    pub style: SupportStyle,
    /// Size of newly created supports. Each support stores its own copy so
    /// differently sized supports can be mixed on one model.
    pub size: SupportSize,
    pub placement: PlacementSettings,
    pub routing: RoutingSettings,
    pub base: BaseSettings,
    pub bracing: BracingSettings,
    pub tree: TreeSettings,
    pub gap_fill: GapFillSettings,
    /// Number of segments used around the circumference of each support.
    pub resolution: u32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SupportSize {
    /// Diameter of the spherical head that touches the model.
    pub tip_diameter: f32,
    /// Length of the tapered section between the head and the shaft.
    pub tip_length: f32,
    /// How far the head is sunk into the model, more makes for a stronger
    /// connection but leaves larger marks.
    pub tip_penetration: f32,
    pub shaft_diameter: f32,
}

/// Support spacing at the lowest and highest aggressiveness.
const AGGRESSIVENESS_SPACING: (f32, f32) = (6.0, 1.5);
/// Overhang angle at the lowest and highest aggressiveness.
const AGGRESSIVENESS_ANGLE: (f32, f32) = (30.0, 60.0);
/// Smallest allowed tip diameter.
pub const MIN_TIP_DIAMETER: f32 = 0.1;
/// How far the head is sunk into the model, relative to the tip diameter.
const TIP_PENETRATION: f32 = 0.45;

#[derive(Debug, Clone, PartialEq)]
pub struct PlacementSettings {
    /// Surfaces that are within this angle of horizontal need to be supported.
    pub overhang_angle: f32,
    /// Target distance between supports on flat overhangs. Steeper overhangs
    /// are given up to 50% more space.
    pub spacing: f32,
    /// Minimum distance between any two supports.
    pub min_spacing: f32,
    /// Support parts of the model that would otherwise start printing in
    /// mid-air (local minima).
    pub islands: bool,
    /// Support sharp downward facing edges.
    pub edges: bool,
    /// Support downward facing surfaces.
    pub faces: bool,
    /// Before generating, models are raised so their lowest point is at least
    /// this far above the build plate.
    pub lift: f32,
}

#[derive(Debug, Clone, PartialEq)]
pub struct RoutingSettings {
    /// Minimum distance between the support shaft and the model.
    pub clearance: f32,
    /// Max angle of the tip from vertical.
    pub max_tip_angle: f32,
    /// Max angle of a leaning shaft segment from vertical.
    pub max_shaft_angle: f32,
    /// Max horizontal distance a shaft may lean to get around the model.
    pub max_reach: f32,
    /// If a support can't reach the build plate, allow it to stand on the
    /// model instead. This will leave marks on the model's upward facing
    /// surfaces.
    pub model_landing: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub struct BaseSettings {
    pub base_diameter: f32,
    pub base_height: f32,

    pub raft: bool,
    pub raft_thickness: f32,
    /// How far the raft extends past the outermost support bases.
    pub raft_margin: f32,
    /// Bevel the raft edge at 45°, making it easier to pry off the plate.
    pub raft_chamfer: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub struct BracingSettings {
    pub enabled: bool,
    pub diameter: f32,
    /// Only columns taller than this are braced.
    pub min_height: f32,
    /// Max horizontal distance between two braced columns.
    pub max_distance: f32,
}

#[derive(Debug, Clone, PartialEq)]
pub struct TreeSettings {
    /// Max angle of a branch from vertical.
    pub branch_angle: f32,
    /// Max horizontal distance between two branches that are merged.
    pub merge_distance: f32,
    /// Trunks get thicker as more branches merge into them, up to this.
    pub max_trunk_diameter: f32,
}

/// Spots that can't be supported normally (a gap too narrow for a support
/// tip, or a pocket with no way out to the build plate) can instead be filled
/// in with solid resin, from the overhang down to whatever is below it.
#[derive(Debug, Clone, PartialEq)]
pub struct GapFillSettings {
    pub enabled: bool,
    /// Gaps taller than this are left alone, so whole cavities aren't filled.
    /// Zero for no limit.
    pub max_height: f32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SupportStyle {
    /// A vertical column with a tapered tip and a flared base.
    Standard,
    /// Branches from nearby tips merge into shared, thicker trunks. Uses less
    /// resin and leaves more room around the model.
    Tree,
    /// Standard columns tied together with X cross bracing, for large and
    /// heavy parts.
    Lattice,
    /// Very thin tapered needles with small bases, for small detailed models.
    Mini,
    /// A single long taper from a thick base to a fine tip. Stiff, but easy to
    /// snap off.
    Needle,
    /// A solid column filling a gap between the model and the surface below
    /// it, see [`GapFillSettings`]. Only made by gap filling, never picked as
    /// the style for new supports.
    Fill,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SizePreset {
    Light,
    Medium,
    Heavy,
}

impl SupportSettings {
    pub fn base_top(&self, plate_z: f32) -> f32 {
        plate_z + self.base.raft as u8 as f32 * self.base.raft_thickness
    }

    /// The height at which the vertical column of a support meets its base.
    pub fn column_bottom(&self, plate_z: f32) -> f32 {
        self.base_top(plate_z) + self.base.base_height
    }

    /// Clamps all values into usable ranges.
    pub fn sanitize(&mut self) {
        let size = &mut self.size;
        size.shaft_diameter = size.shaft_diameter.max(0.1);
        size.tip_diameter = size
            .tip_diameter
            .clamp(MIN_TIP_DIAMETER, size.shaft_diameter);
        size.tip_length = size.tip_length.max(0.1);
        size.tip_penetration = size.tip_penetration.clamp(0.0, size.tip_diameter);

        let placement = &mut self.placement;
        placement.overhang_angle = placement.overhang_angle.clamp(0.0, 89.0);
        placement.spacing = placement.spacing.max(0.25);
        placement.min_spacing = placement.min_spacing.clamp(0.1, placement.spacing);
        placement.lift = placement.lift.max(0.0);

        let routing = &mut self.routing;
        routing.clearance = routing.clearance.max(0.0);
        routing.max_tip_angle = routing.max_tip_angle.clamp(0.0, 89.0);
        routing.max_shaft_angle = routing.max_shaft_angle.clamp(0.0, 70.0);
        routing.max_reach = routing.max_reach.max(0.0);

        let base = &mut self.base;
        base.base_diameter = base.base_diameter.max(0.1);
        base.base_height = base.base_height.max(0.0);
        base.raft_thickness = base.raft_thickness.max(0.05);
        base.raft_margin = base.raft_margin.max(0.0);

        let bracing = &mut self.bracing;
        bracing.diameter = bracing.diameter.max(0.1);
        bracing.min_height = bracing.min_height.max(0.0);
        bracing.max_distance = bracing.max_distance.max(0.5);

        let tree = &mut self.tree;
        tree.branch_angle = tree.branch_angle.clamp(5.0, 70.0);
        tree.merge_distance = tree.merge_distance.max(0.0);
        tree.max_trunk_diameter = tree.max_trunk_diameter.max(self.size.shaft_diameter);

        self.gap_fill.max_height = self.gap_fill.max_height.max(0.0);

        self.resolution = self.resolution.clamp(6, 64);
    }

    /// Switches to a style, along with the sizes and bracing that suit it.
    pub fn apply_style(&mut self, style: SupportStyle) {
        let defaults = Self::for_style(style);

        self.style = style;
        self.size = defaults.size;
        self.placement.lift = defaults.placement.lift;
        self.base = BaseSettings {
            raft: self.base.raft,
            raft_thickness: self.base.raft_thickness,
            raft_margin: self.base.raft_margin,
            raft_chamfer: self.base.raft_chamfer,
            ..defaults.base
        };
        self.bracing = BracingSettings {
            enabled: self.bracing.enabled,
            ..defaults.bracing
        };
    }

    /// The default settings for a style.
    pub fn for_style(style: SupportStyle) -> Self {
        let mut out = Self {
            style,
            size: SizePreset::Medium.size(style),
            ..Self::default()
        };

        match style {
            SupportStyle::Standard | SupportStyle::Fill => {}
            // Branches need some height to merge in.
            SupportStyle::Tree => out.placement.lift = 8.0,
            SupportStyle::Lattice => {
                out.bracing.min_height = 4.0;
                out.bracing.max_distance = 10.0;
                out.bracing.diameter = 0.8;
            }
            SupportStyle::Mini => {
                out.base.base_diameter = 1.2;
                out.base.base_height = 0.4;
                out.bracing.diameter = 0.4;
                out.bracing.max_distance = 8.0;
            }
            SupportStyle::Needle => {
                out.base.base_diameter = 4.0;
                out.base.base_height = 1.0;
                out.bracing.diameter = 0.9;
                out.bracing.min_height = 12.0;
            }
        }

        out
    }

    /// How aggressively supports are placed, from 0 (sparse, only the most
    /// needed) to 1 (dense, supporting even gentle overhangs). This is derived
    /// from the support spacing.
    pub fn aggressiveness(&self) -> f32 {
        let (sparse, dense) = AGGRESSIVENESS_SPACING;
        ((self.placement.spacing / sparse).ln() / (dense / sparse).ln()).clamp(0.0, 1.0)
    }

    /// Sets the support spacing and overhang angle from an aggressiveness, see
    /// [`SupportSettings::aggressiveness`].
    pub fn set_aggressiveness(&mut self, value: f32) {
        let value = value.clamp(0.0, 1.0);
        let (sparse, dense) = AGGRESSIVENESS_SPACING;
        let (low, high) = AGGRESSIVENESS_ANGLE;

        let placement = &mut self.placement;
        placement.spacing = sparse * (dense / sparse).powf(value);
        placement.min_spacing = (placement.spacing / 3.0).max(0.5).min(placement.spacing);
        placement.overhang_angle = low + (high - low) * value;
    }

    /// Sets how fine the point touching the model is. The head is sunk into
    /// the model in proportion, so finer tips leave smaller marks.
    pub fn set_tip_diameter(&mut self, diameter: f32) {
        let size = &mut self.size;
        size.tip_diameter = diameter.clamp(MIN_TIP_DIAMETER, size.shaft_diameter);
        size.tip_penetration = size.tip_diameter * TIP_PENETRATION;
    }

    pub fn serialize<T: Serializer>(&self, ser: &mut T) {
        self.style.serialize(ser);
        self.size.serialize(ser);

        let placement = &self.placement;
        ser.write_f32_be(placement.overhang_angle);
        ser.write_f32_be(placement.spacing);
        ser.write_f32_be(placement.min_spacing);
        ser.write_bool(placement.islands);
        ser.write_bool(placement.edges);
        ser.write_bool(placement.faces);
        ser.write_f32_be(placement.lift);

        let routing = &self.routing;
        ser.write_f32_be(routing.clearance);
        ser.write_f32_be(routing.max_tip_angle);
        ser.write_f32_be(routing.max_shaft_angle);
        ser.write_f32_be(routing.max_reach);
        ser.write_bool(routing.model_landing);

        let base = &self.base;
        ser.write_f32_be(base.base_diameter);
        ser.write_f32_be(base.base_height);
        ser.write_bool(base.raft);
        ser.write_f32_be(base.raft_thickness);
        ser.write_f32_be(base.raft_margin);
        ser.write_bool(base.raft_chamfer);

        let bracing = &self.bracing;
        ser.write_bool(bracing.enabled);
        ser.write_f32_be(bracing.diameter);
        ser.write_f32_be(bracing.min_height);
        ser.write_f32_be(bracing.max_distance);

        let tree = &self.tree;
        ser.write_f32_be(tree.branch_angle);
        ser.write_f32_be(tree.merge_distance);
        ser.write_f32_be(tree.max_trunk_diameter);

        ser.write_u32_be(self.resolution);

        ser.write_bool(self.gap_fill.enabled);
        ser.write_f32_be(self.gap_fill.max_height);
    }

    /// Reads settings written in the given [`FORMAT_VERSION`].
    pub fn deserialize<T: Deserializer>(des: &mut T, format: u16) -> Self {
        let style = if format < 2 {
            SupportStyle::Standard
        } else {
            SupportStyle::deserialize(des)
        };

        let mut out = Self {
            style,
            size: SupportSize::deserialize(des),
            placement: PlacementSettings {
                overhang_angle: des.read_f32_be(),
                spacing: des.read_f32_be(),
                min_spacing: des.read_f32_be(),
                islands: des.read_bool(),
                edges: des.read_bool(),
                faces: des.read_bool(),
                lift: des.read_f32_be(),
            },
            routing: RoutingSettings {
                clearance: des.read_f32_be(),
                max_tip_angle: des.read_f32_be(),
                max_shaft_angle: des.read_f32_be(),
                max_reach: des.read_f32_be(),
                model_landing: des.read_bool(),
            },
            base: BaseSettings {
                base_diameter: des.read_f32_be(),
                base_height: des.read_f32_be(),
                raft: des.read_bool(),
                raft_thickness: des.read_f32_be(),
                raft_margin: des.read_f32_be(),
                raft_chamfer: des.read_bool(),
            },
            bracing: BracingSettings {
                enabled: des.read_bool(),
                diameter: des.read_f32_be(),
                min_height: des.read_f32_be(),
                max_distance: des.read_f32_be(),
            },
            tree: if format < 2 {
                TreeSettings::default()
            } else {
                TreeSettings {
                    branch_angle: des.read_f32_be(),
                    merge_distance: des.read_f32_be(),
                    max_trunk_diameter: des.read_f32_be(),
                }
            },
            resolution: des.read_u32_be(),
            gap_fill: GapFillSettings::default(),
        };
        if format >= 3 {
            out.gap_fill = GapFillSettings {
                enabled: des.read_bool(),
                max_height: des.read_f32_be(),
            };
        }
        out.sanitize();
        out
    }
}

impl SupportSize {
    pub fn tip_radius(&self) -> f32 {
        self.tip_diameter / 2.0
    }

    pub fn shaft_radius(&self) -> f32 {
        self.shaft_diameter / 2.0
    }

    pub fn serialize<T: Serializer>(&self, ser: &mut T) {
        ser.write_f32_be(self.tip_diameter);
        ser.write_f32_be(self.tip_length);
        ser.write_f32_be(self.tip_penetration);
        ser.write_f32_be(self.shaft_diameter);
    }

    pub fn deserialize<T: Deserializer>(des: &mut T) -> Self {
        Self {
            tip_diameter: des.read_f32_be(),
            tip_length: des.read_f32_be(),
            tip_penetration: des.read_f32_be(),
            shaft_diameter: des.read_f32_be(),
        }
    }
}

impl SupportStyle {
    pub const ALL: [Self; 5] = [
        Self::Standard,
        Self::Tree,
        Self::Lattice,
        Self::Mini,
        Self::Needle,
    ];

    pub fn name(&self) -> &'static str {
        match self {
            Self::Standard => "Standard",
            Self::Tree => "Tree",
            Self::Lattice => "Lattice",
            Self::Mini => "Mini Needle",
            Self::Needle => "Thick Needle",
            Self::Fill => "Gap Fill",
        }
    }

    pub fn description(&self) -> &'static str {
        match self {
            Self::Standard => {
                "Vertical columns with a tapered tip and a flared base. A good default for most prints."
            }
            Self::Tree => {
                "Nearby tips branch into shared trunks. Uses less resin, leaves more room around the model and fewer bases to clean up."
            }
            Self::Lattice => {
                "Columns tied together with X cross bracing. Very stiff, for large, heavy or tall parts."
            }
            Self::Mini => {
                "Very thin tapered needles with small bases, for miniatures and fine detail. Snap off cleanly."
            }
            Self::Needle => {
                "A single long taper from a thick base to a fine tip. Stiff like a heavy support, but with a small contact mark."
            }
            Self::Fill => {
                "A solid column filling a gap that couldn't be supported normally. Becomes part of the print."
            }
        }
    }

    /// Whether the whole support tapers from its base to the tip, rather than
    /// having a separate tip and constant width shaft.
    pub fn tapered(&self) -> bool {
        matches!(self, Self::Mini | Self::Needle)
    }

    pub fn serialize<T: Serializer>(&self, ser: &mut T) {
        ser.write_u8(*self as u8);
    }

    pub fn deserialize<T: Deserializer>(des: &mut T) -> Self {
        match des.read_u8() {
            1 => Self::Tree,
            2 => Self::Lattice,
            3 => Self::Mini,
            4 => Self::Needle,
            5 => Self::Fill,
            _ => Self::Standard,
        }
    }
}

impl SizePreset {
    pub const ALL: [Self; 3] = [Self::Light, Self::Medium, Self::Heavy];

    pub fn name(&self) -> &'static str {
        match self {
            Self::Light => "Light",
            Self::Medium => "Medium",
            Self::Heavy => "Heavy",
        }
    }

    /// Sizes for a preset. For tapered styles the shaft diameter is the width
    /// at the base, for trees it is the width of a single branch.
    pub fn size(&self, style: SupportStyle) -> SupportSize {
        // (tip diameter, tip length, shaft diameter) for light, medium and
        // heavy supports.
        let table = match style {
            SupportStyle::Standard | SupportStyle::Lattice | SupportStyle::Fill => {
                [(0.3, 1.5, 0.7), (0.45, 2.0, 1.0), (0.65, 2.5, 1.5)]
            }
            SupportStyle::Tree => [(0.3, 1.5, 0.6), (0.4, 2.0, 0.8), (0.6, 2.5, 1.2)],
            SupportStyle::Mini => [(0.15, 1.0, 0.45), (0.2, 1.2, 0.6), (0.3, 1.5, 0.8)],
            SupportStyle::Needle => [(0.3, 2.0, 1.6), (0.4, 2.5, 2.2), (0.55, 3.0, 3.0)],
        };

        let (tip_diameter, tip_length, shaft_diameter) = table[*self as usize];
        SupportSize {
            tip_diameter,
            tip_length,
            tip_penetration: tip_diameter * TIP_PENETRATION,
            shaft_diameter,
        }
    }

    pub fn matching(size: &SupportSize, style: SupportStyle) -> Option<Self> {
        Self::ALL.into_iter().find(|x| x.size(style) == *size)
    }
}

impl Default for GapFillSettings {
    fn default() -> Self {
        Self {
            enabled: false,
            max_height: 5.0,
        }
    }
}

impl Default for TreeSettings {
    fn default() -> Self {
        Self {
            branch_angle: 40.0,
            merge_distance: 10.0,
            max_trunk_diameter: 3.0,
        }
    }
}

impl Default for SupportSettings {
    fn default() -> Self {
        Self {
            style: SupportStyle::Standard,
            size: SizePreset::Medium.size(SupportStyle::Standard),
            placement: PlacementSettings {
                overhang_angle: 45.0,
                spacing: 3.0,
                min_spacing: 1.0,
                islands: true,
                edges: true,
                faces: true,
                lift: 5.0,
            },
            routing: RoutingSettings {
                clearance: 0.4,
                max_tip_angle: 45.0,
                max_shaft_angle: 35.0,
                max_reach: 25.0,
                model_landing: true,
            },
            base: BaseSettings {
                base_diameter: 3.0,
                base_height: 0.8,
                raft: true,
                raft_thickness: 1.0,
                raft_margin: 1.5,
                raft_chamfer: true,
            },
            bracing: BracingSettings {
                enabled: true,
                diameter: 0.7,
                min_height: 8.0,
                max_distance: 12.0,
            },
            tree: TreeSettings::default(),
            gap_fill: GapFillSettings::default(),
            resolution: 12,
        }
    }
}
