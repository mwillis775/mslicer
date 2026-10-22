//! Cross bracing between tall support columns.
//!
//! Tall, thin columns flex during peeling, which shows up as layer lines and
//! wobble in the print. Neighboring columns are connected with a zig-zag of 45°
//! struts, turning the forest of columns into a set of trusses. Neighbors are
//! chosen with a relative neighborhood graph, which is planar (no braces
//! crossing each other) and only connects columns that don't have another
//! column between them.
//!
//! Lattice style columns are tied together with X bracing instead, struts in
//! both directions in every bay.

use std::collections::{HashMap, HashSet};

use nalgebra::Vector2;

use crate::supports::{
    Brace, Foot, Support,
    collider::Collider,
    settings::{SupportSettings, SupportStyle},
};

/// Space left between the bottom brace and the base, and the top brace and
/// any bends in the shaft.
const END_MARGIN: f32 = 1.0;
/// Minimum height gained by each strut.
const MIN_RISE: f32 = 5.0;
/// Columns closer than this are basically touching and aren't braced.
const MIN_DISTANCE: f32 = 1.0;

struct Column {
    position: Vector2<f32>,
    top: f32,
    lattice: bool,
}

pub fn brace(
    collider: &Collider,
    supports: &[Support],
    settings: &SupportSettings,
    plate_z: f32,
) -> Vec<Brace> {
    let bracing = &settings.bracing;
    if !bracing.enabled {
        return Vec::new();
    }

    // Branches of a tree share their trunk, which is only braced once.
    let bottom = settings.column_bottom(plate_z) + END_MARGIN;
    let mut seen = HashSet::new();
    let columns = (supports.iter())
        .filter(|x| x.route.foot == Foot::Plate && x.style != SupportStyle::Fill)
        .filter(|x| seen.insert(x.column_top().map(f32::to_bits)))
        .map(|x| Column {
            position: x.column_top().xy(),
            top: x.column_top().z - x.column_radius() * 2.0 - END_MARGIN,
            lattice: x.style == SupportStyle::Lattice,
        })
        .filter(|x| x.top - bottom >= bracing.min_height)
        .collect::<Vec<_>>();

    let cell = bracing.max_distance;
    let key = |p: Vector2<f32>| (p / cell).map(|x| x.floor() as i32);
    let mut grid = HashMap::<Vector2<i32>, Vec<usize>>::new();
    for (i, column) in columns.iter().enumerate() {
        grid.entry(key(column.position)).or_default().push(i);
    }

    let nearby = |p: Vector2<f32>| {
        let center = key(p);
        (-1..=1)
            .flat_map(move |x| (-1..=1).map(move |y| center + Vector2::new(x, y)))
            .filter_map(|cell| grid.get(&cell))
            .flatten()
            .copied()
    };

    let radius = bracing.diameter / 2.0 + settings.routing.clearance;
    let mut out = Vec::new();

    for (i, a) in columns.iter().enumerate() {
        for j in nearby(a.position).filter(|&j| j > i) {
            let b = &columns[j];
            let distance = (b.position - a.position).magnitude();
            if !(MIN_DISTANCE..=bracing.max_distance).contains(&distance) {
                continue;
            }

            // Relative neighborhood graph: skip the pair if some other column
            // is closer to both of them than they are to each other.
            let blocked = nearby(a.position).any(|k| {
                let c = columns[k].position;
                k != i
                    && k != j
                    && (c - a.position).magnitude() < distance
                    && (c - b.position).magnitude() < distance
            });
            if blocked {
                continue;
            }

            // Zig-zag up between the columns. Struts rise at 45°, or steeper
            // for close columns so they aren't packed too tightly.
            let rise = distance.max(MIN_RISE);
            let top = a.top.min(b.top);
            let lattice = a.lattice && b.lattice;
            let (mut z, mut flip) = (bottom, false);
            while z + rise <= top {
                let mut strut = |low: &Column, high: &Column| {
                    let start = low.position.push(z);
                    let end = high.position.push(z + rise);
                    (collider.clear(start, end, radius)).then(|| out.push(Brace { start, end }));
                };

                if lattice {
                    strut(a, b);
                    strut(b, a);
                } else if flip {
                    strut(b, a);
                } else {
                    strut(a, b);
                }

                z += rise;
                flip ^= true;
            }
        }
    }

    out
}
