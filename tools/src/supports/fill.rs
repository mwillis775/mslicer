//! Gap filling, for spots that can't be supported normally.
//!
//! Some overhangs have no room for a support: a gap between two parts of the
//! model that is too narrow for the tip, or a pocket with no way out to the
//! build plate. With gap filling enabled, these gaps are filled in with solid
//! resin instead.
//!
//! A fill starts from a seed point on the unsupported overhang and spreads
//! over a grid of small cells in plan view. Each cell probes the overhang
//! above it (the ceiling) and the surface below it (the floor, or the build
//! plate). A cell is added if its ceiling continues smoothly from its
//! neighbor's, faces down steeply enough to need support, and isn't too far
//! above its floor. This way the fill takes on the shape of the gap itself.
//!
//! The fill is meshed as one solid: a top surface following the ceiling and a
//! bottom surface following the floor, both sunk slightly into the model so
//! they fuse with it, joined by vertical walls around the outline. Only the
//! walls end up visible, and their outline is smoothed so it doesn't show the
//! grid.
//!
//! A fill is stored as a [`Support`] with the
//! [`SupportStyle::Fill`](crate::supports::settings::SupportStyle::Fill)
//! style, whose contact is the seed point. It's saved, selected, deleted and
//! re-routed like any other support. Re-routing grows the fill again from its
//! seed.

use std::{
    collections::{HashMap, HashSet, VecDeque},
    sync::Arc,
};

use common::serde::{Deserializer, Serializer};
use nalgebra::{Vector2, Vector3};
use slicer::builder::MeshBuilder;

use crate::supports::{
    Contact, Foot, Route, Support,
    collider::Collider,
    routing::RouteError,
    settings::{SupportSettings, SupportSize},
};

/// Width of a fill cell.
pub const CELL: f32 = 0.15;
/// Cells this close to the seed are always filled, so even the very tip of an
/// island gets a solid footing.
const MIN_RADIUS: f32 = 0.4;
/// Fills stop growing at this many cells (about 900 mm²).
const MAX_CELLS: usize = 40_000;
/// Max difference in ceiling height between neighboring cells. Bigger jumps
/// are a different surface, not a continuation of the same overhang.
const MAX_STEP: f32 = 0.5;
/// How far below the expected ceiling probe rays start.
const PROBES: [f32; 3] = [0.05, 0.25, 0.6];
/// The ceiling must face at least this far downward.
const MIN_CEILING_NORMAL_Z: f32 = 0.1;
/// The floor must face at least this far upward.
const MIN_FLOOR_NORMAL_Z: f32 = 0.1;
/// How far the fill is sunk into the model at the top and bottom.
const SINK: f32 = 0.15;
/// How far into the floor it's checked for being solid.
const SOLID_DEPTH: f32 = 0.02;
/// Smoothing passes applied to the outline.
const SMOOTHING: usize = 4;

/// The cells making up a fill, in support space.
#[derive(Debug, Clone, PartialEq)]
pub struct FillShape {
    /// Cell (0, 0) is centered on this point.
    pub origin: Vector2<f32>,
    pub cells: Vec<FillCell>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FillCell {
    pub key: Vector2<i32>,
    /// Height of the top and bottom of the fill, already sunk into the model.
    pub top: f32,
    pub bottom: f32,
}

/// The ceiling and floor found above and below a point.
struct Probe {
    ceiling: Contact,
    /// Height of the bottom of the fill: the floor sunk in, or the plate.
    bottom: f32,
}

/// The size given to fills. It's only there to keep [`Support`]s uniform, a
/// fill's shape comes from its cells.
pub fn size() -> SupportSize {
    SupportSize {
        tip_diameter: CELL * 2.0,
        tip_length: CELL * 2.0,
        tip_penetration: SINK,
        shaft_diameter: CELL * 2.0,
    }
}

/// Grows a fill from a seed point on an overhang. Fails if there is no gap to
/// fill there.
pub fn route(
    collider: &Collider,
    settings: &SupportSettings,
    plate_z: f32,
    seed: &Contact,
) -> Result<Route, RouteError> {
    let shape = grow(collider, settings, plate_z, seed).ok_or(RouteError::Blocked)?;
    Ok(Route {
        fill: Some(Arc::new(shape)),
        ..Route::new(vec![seed.point], Foot::Plate)
    })
}

fn grow(
    collider: &Collider,
    settings: &SupportSettings,
    plate_z: f32,
    seed: &Contact,
) -> Option<FillShape> {
    let min_normal_z = settings.placement.overhang_angle.to_radians().cos();
    let origin = seed.point.xy();
    let position = |key: Vector2<i32>| origin + key.cast::<f32>() * CELL;

    let first = probe(collider, settings, plate_z, origin, seed.point.z)?;
    let mut queue = VecDeque::from([(Vector2::zeros(), first.ceiling.point.z)]);
    let mut cells = vec![cell(Vector2::zeros(), &first)];
    let mut visited = HashSet::from([Vector2::<i32>::zeros()]);

    while let Some((key, ceiling)) = queue.pop_front() {
        for (x, y) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
            let next = key + Vector2::new(x, y);
            if cells.len() >= MAX_CELLS || !visited.insert(next) {
                continue;
            }

            let xy = position(next);
            let Some(probe) = probe(collider, settings, plate_z, xy, ceiling) else {
                continue;
            };

            // Past the seed's footing, only overhangs that need support are
            // filled, not the steep sides around them.
            let near_seed = (xy - origin).magnitude() <= MIN_RADIUS;
            if near_seed || -probe.ceiling.normal.z >= min_normal_z {
                cells.push(cell(next, &probe));
                queue.push_back((next, probe.ceiling.point.z));
            }
        }
    }

    Some(FillShape { origin, cells })
}

fn cell(key: Vector2<i32>, probe: &Probe) -> FillCell {
    FillCell {
        key,
        top: probe.ceiling.point.z + SINK,
        bottom: probe.bottom,
    }
}

/// Looks for a ceiling near `expected` above `xy`, with a floor close enough
/// below it. Probing starts from a point a little below where the ceiling is
/// expected that is outside the model, and looks for the nearest surfaces
/// above and below it.
///
/// Whether a point is outside is decided like the slicer does, see
/// [`Collider::is_empty`], so meshes that overlap themselves (common with
/// sculpted and generated models) work. Surfaces in such meshes can face
/// either way, so they're taken as facing into the gap.
fn probe(
    collider: &Collider,
    settings: &SupportSettings,
    plate_z: f32,
    xy: Vector2<f32>,
    expected: f32,
) -> Option<Probe> {
    let start = (PROBES.into_iter())
        .map(|below| xy.push(expected - below))
        .find(|&x| collider.is_empty(x))?;

    let hit = collider.ray(start, Vector3::z())?;
    let ceiling = Contact::new(hit.point, hit.normal * -hit.normal.z.signum());
    if ceiling.normal.z > -MIN_CEILING_NORMAL_Z || (hit.point.z - expected).abs() > MAX_STEP {
        return None;
    }

    let (floor, bottom) = match collider.ray(start, -Vector3::z()) {
        // It has to really be solid below the floor, the fill can't hang
        // off the model's underside.
        Some(floor)
            if floor.normal.z.abs() >= MIN_FLOOR_NORMAL_Z
                && !collider.is_empty(floor.point - Vector3::z() * SOLID_DEPTH) =>
        {
            (floor.point.z, floor.point.z - SINK)
        }
        // A wall straight below isn't something to stand on.
        Some(_) => return None,
        // Goes right down through the raft, which may not reach this far.
        None => (settings.base_top(plate_z), plate_z),
    };

    let height = ceiling.point.z - floor;
    let max_height = settings.gap_fill.max_height;
    (height > 0.0 && (max_height <= 0.0 || height <= max_height))
        .then_some(Probe { ceiling, bottom })
}

/// Whether a support's contact is held up by any of the fills.
pub fn covered(fills: &[&FillShape], support: &Support) -> bool {
    fills.iter().any(|x| x.contains(support.contact.point))
}

impl FillShape {
    /// Whether a point is inside the fill, or on the overhang just above it.
    pub fn contains(&self, point: Vector3<f32>) -> bool {
        let key = ((point.xy() - self.origin) / CELL).map(|x| x.round() as i32);
        (self.cells.iter()).any(|x| x.key == key && (x.bottom..=x.top + SINK).contains(&point.z))
    }

    /// Area of the fill in plan view, in mm².
    pub fn area(&self) -> f32 {
        self.cells.len() as f32 * CELL * CELL
    }

    /// Adds the fill as one closed solid, see the module docs.
    pub fn add_to(&self, builder: &mut MeshBuilder) {
        let cells = (self.cells.iter())
            .map(|x| (x.key, *x))
            .collect::<HashMap<_, _>>();

        // Corner (i, j) is the bottom left corner of cell (i, j). Its top and
        // bottom are at least as far into the model as the deepest cell
        // around it needs, so slopes don't leave slivers of gap.
        let mut corners = HashMap::<Vector2<i32>, (Vector2<f32>, f32, f32)>::new();
        for &key in cells.keys() {
            for corner in cell_corners(key) {
                corners.entry(corner).or_insert_with(|| {
                    let around = [(0, 0), (-1, 0), (0, -1), (-1, -1)]
                        .into_iter()
                        .filter_map(|(x, y)| cells.get(&(corner + Vector2::new(x, y))))
                        .collect::<Vec<_>>();
                    let n = around.len() as f32;
                    let mean_top = around.iter().map(|x| x.top).sum::<f32>() / n;
                    let mean_bottom = around.iter().map(|x| x.bottom).sum::<f32>() / n;
                    let max_top = around.iter().map(|x| x.top).fold(f32::MIN, f32::max);
                    let min_bottom = around.iter().map(|x| x.bottom).fold(f32::MAX, f32::min);

                    let xy = self.origin + (corner.cast::<f32>() - Vector2::repeat(0.5)) * CELL;
                    let top = mean_top.max(max_top - SINK);
                    (xy, top, mean_bottom.min(min_bottom + SINK))
                });
            }
        }

        // The outline, as edges going counter-clockwise around the fill seen
        // from above.
        let mut outline = Vec::new();
        for &key in cells.keys() {
            let c = cell_corners(key);
            let sides = [(0, -1), (1, 0), (0, 1), (-1, 0)];
            for (i, (x, y)) in sides.into_iter().enumerate() {
                (!cells.contains_key(&(key + Vector2::new(x, y))))
                    .then(|| outline.push((c[i], c[(i + 1) % 4])));
            }
        }

        smooth_outline(&mut corners, &outline);

        let mut index = HashMap::new();
        for (&key, &(xy, top, bottom)) in corners.iter() {
            index.insert(key, builder.add_vertex(xy.push(top)));
            builder.add_vertex(xy.push(bottom));
        }
        let top = |key| index[&key];
        let bottom = |key| index[&key] + 1;

        for &key in cells.keys() {
            let c = cell_corners(key);
            builder.add_face([top(c[0]), top(c[1]), top(c[2])]);
            builder.add_face([top(c[0]), top(c[2]), top(c[3])]);
            builder.add_face([bottom(c[0]), bottom(c[2]), bottom(c[1])]);
            builder.add_face([bottom(c[0]), bottom(c[3]), bottom(c[2])]);
        }

        for (a, b) in outline {
            builder.add_face([bottom(a), bottom(b), top(b)]);
            builder.add_face([bottom(a), top(b), top(a)]);
        }
    }

    pub fn serialize<T: Serializer>(&self, ser: &mut T) {
        ser.write_f32_be(self.origin.x);
        ser.write_f32_be(self.origin.y);
        ser.write_u32_be(self.cells.len() as u32);
        for cell in self.cells.iter() {
            ser.write_u32_be(cell.key.x as u32);
            ser.write_u32_be(cell.key.y as u32);
            ser.write_f32_be(cell.top);
            ser.write_f32_be(cell.bottom);
        }
    }

    pub fn deserialize<T: Deserializer>(des: &mut T) -> Self {
        let origin = Vector2::new(des.read_f32_be(), des.read_f32_be());
        let cells = (0..des.read_u32_be())
            .map(|_| FillCell {
                key: Vector2::new(des.read_i32_be(), des.read_i32_be()),
                top: des.read_f32_be(),
                bottom: des.read_f32_be(),
            })
            .collect();
        Self { origin, cells }
    }
}

/// Corners of a cell, counter-clockwise from its bottom left seen from above.
fn cell_corners(key: Vector2<i32>) -> [Vector2<i32>; 4] {
    [(0, 0), (1, 0), (1, 1), (0, 1)].map(|(x, y)| key + Vector2::new(x, y))
}

/// Rounds off the stair steps of the grid by moving each outline corner
/// towards the middle of its two outline neighbors. Corners where the outline
/// touches itself are left alone.
fn smooth_outline(
    corners: &mut HashMap<Vector2<i32>, (Vector2<f32>, f32, f32)>,
    outline: &[(Vector2<i32>, Vector2<i32>)],
) {
    let mut neighbors = HashMap::<Vector2<i32>, Vec<Vector2<i32>>>::new();
    for &(a, b) in outline {
        neighbors.entry(a).or_default().push(b);
        neighbors.entry(b).or_default().push(a);
    }
    neighbors.retain(|_, x| x.len() == 2);

    for _ in 0..SMOOTHING {
        let moved = (neighbors.iter())
            .map(|(key, n)| {
                let xy = |k: &Vector2<i32>| corners[k].0;
                (*key, xy(key) * 0.5 + (xy(&n[0]) + xy(&n[1])) * 0.25)
            })
            .collect::<Vec<_>>();
        for (key, xy) in moved {
            corners.get_mut(&key).unwrap().0 = xy;
        }
    }
}
