//! Layer based island detection.
//!
//! The model is sliced into thin layers which are rasterized with the same
//! scanline filler used by the real slicer. Any connected region of a layer
//! that doesn't touch the layer below it would start printing in mid-air, so
//! it needs supports. Unlike looking for local minima in the mesh directly,
//! this is robust to noisy, finely tessellated, or non-manifold meshes and
//! catches minima along edges and flat faces.

use std::collections::HashMap;

use common::container::{ArrayCluster, Run};
use nalgebra::{Vector2, Vector3};
use rayon::iter::{IntoParallelIterator, ParallelIterator};
use slicer::{
    geometry::Segments1D,
    mesh::Mesh,
    slicer::raster::{Segment, layer},
};

use crate::supports::{Contact, collider::Collider};

/// Width of one raster pixel.
const PIXEL_SIZE: f32 = 0.05;
/// Distance between sampled layers.
const LAYER_HEIGHT: f32 = 0.05;
/// Number of layers rasterized at once.
const CHUNK_SIZE: usize = 64;
/// Spatial bins for the slicing acceleration structure.
const SEGMENT_LAYERS: usize = 200;

/// Horizontal spans of filled pixels for each row of a layer.
type Rows = Vec<Vec<(u32, u32)>>;

pub struct Island {
    /// Points on the bottom of the island to support. Large islands get
    /// multiple points spread `spacing` apart.
    pub contacts: Vec<Contact>,
    /// Area in mm².
    pub area: f32,
}

pub fn find_islands(collider: &Collider, plate_z: f32, spacing: f32) -> Vec<Island> {
    islands(collider, collider.mesh(), plate_z, spacing, (0.0, false))
}

/// Finds islands in `mesh`, which can be the collider's mesh combined with
/// other parts (like its supports) to check what is still left hanging.
/// Contacts are only found on the collider's mesh.
///
/// This uses the same, stricter, test as the slicer's island detection: a
/// region is only held up by pixels directly below it, not ones diagonally
/// below. Very small islands also only show up at some pixel and layer
/// alignments, which are shifted by `phase` (from 0 to 1) of a pixel and a
/// layer.
pub fn find_islands_in(
    collider: &Collider,
    mesh: &Mesh,
    plate_z: f32,
    spacing: f32,
    phase: f32,
) -> Vec<Island> {
    islands(collider, mesh, plate_z, spacing, (phase, true))
}

fn islands(
    collider: &Collider,
    mesh: &Mesh,
    plate_z: f32,
    spacing: f32,
    (phase, strict): (f32, bool),
) -> Vec<Island> {
    let (min, max) = mesh.bounds();
    if mesh.face_count() == 0 || max.z <= min.z {
        return Vec::new();
    }

    // Raster space: origin at the bounding box corner with a two pixel
    // margin, x/y scaled to pixels, z unchanged.
    let origin = min.xy() - Vector2::repeat(PIXEL_SIZE * (2.0 + phase));
    let size = ((max.xy() - origin) / PIXEL_SIZE).map(|x| x.ceil() as u32 + 2);
    let raster_mesh = to_raster_space(mesh, origin);
    let segments = Segments1D::from_mesh(&raster_mesh, SEGMENT_LAYERS);

    let layers = ((max.z - min.z) / LAYER_HEIGHT).ceil() as usize;
    let height = |i: usize| min.z + (i as f32 + 0.5 + phase) * LAYER_HEIGHT;

    let mut islands = Vec::new();
    let mut prev: Rows = vec![Vec::new(); size.y as usize];

    for chunk in (0..layers).collect::<Vec<_>>().chunks(CHUNK_SIZE) {
        let rows = (chunk.to_vec().into_par_iter())
            .map(|i| rasterize(&raster_mesh, &segments, size, height(i)))
            .collect::<Vec<_>>();

        for (i, rows) in chunk.iter().zip(rows) {
            let z = height(*i);
            if z - plate_z > LAYER_HEIGHT * 1.5 {
                for pixels in unsupported_regions(&rows, &prev, strict) {
                    islands.push(to_island(collider, &pixels, (origin, z), spacing));
                }
            }
            prev = rows;
        }
    }

    islands.retain(|x| !x.contacts.is_empty());
    islands
}

fn to_raster_space(mesh: &Mesh, origin: Vector2<f32>) -> Mesh {
    let scale = Vector3::new(PIXEL_SIZE.recip(), PIXEL_SIZE.recip(), 1.0);
    let mut out = mesh.clone();
    out.set_scale_unchecked(mesh.scale().component_mul(&scale));
    out.set_position_unchecked((mesh.position() - origin.push(0.0)).component_mul(&scale));
    out.update_transformation_matrix();
    out
}

fn rasterize(mesh: &Mesh, segments: &Segments1D, size: Vector2<u32>, z: f32) -> Rows {
    let segments = (segments.intersect_plane(mesh, z).into_iter()).map(|(pos, dir)| Segment {
        endpoints: pos,
        entering: dir,
        priority: 0,
        exposure: 255,
    });

    let (runs, _) = layer(1, size, segments);
    runs_to_rows(&runs, size.x)
}

fn runs_to_rows(runs: &[Run], width: u32) -> Rows {
    let width = width as u64;
    let mut rows = Vec::new();
    let mut pos = 0;

    for run in runs {
        let (mut start, end) = (pos, pos + run.length);
        pos = end;
        if run.value == 0 {
            continue;
        }

        // Runs can wrap across multiple rows.
        while start < end {
            let row = (start / width) as usize;
            let row_end = end.min((row as u64 + 1) * width);

            rows.resize_with(rows.len().max(row + 1), Vec::new);
            rows[row].push((
                (start % width) as u32,
                (row_end - row as u64 * width) as u32,
            ));
            start = row_end;
        }
    }

    rows
}

/// Spans are half-open `[start, end)`. Touching diagonally counts as
/// overlapping.
fn spans_touch(a: (u32, u32), b: (u32, u32)) -> bool {
    a.0 <= b.1 && b.0 <= a.1
}

/// Groups the filled pixels of `rows` into connected regions, and returns the
/// spans of each region that doesn't touch anything in the `prev` layer. Spans
/// are returned as `(row, (start, end))`.
fn unsupported_regions(rows: &Rows, prev: &Rows, strict: bool) -> Vec<Vec<(usize, (u32, u32))>> {
    let mut offsets = Vec::with_capacity(rows.len());
    let mut spans = Vec::new();
    for (row, row_spans) in rows.iter().enumerate() {
        offsets.push(spans.len());
        spans.extend(row_spans.iter().map(|&span| (row, span)));
    }

    // Join spans that touch spans in the next row. Spans in a row are sorted
    // and disjoint, so a two pointer sweep finds every touching pair.
    let mut clusters = ArrayCluster::new(spans.len());
    for row in 1..rows.len() {
        let (above, below) = (&rows[row - 1], &rows[row]);
        let (mut i, mut j) = (0, 0);
        while i < above.len() && j < below.len() {
            if spans_touch(above[i], below[j]) {
                let (a, b) = (offsets[row - 1] + i, offsets[row] + j);
                clusters.union(a as u32, b as u32);
            }

            if above[i].1 < below[j].1 {
                i += 1;
            } else {
                j += 1;
            }
        }
    }

    // A region is supported if any of its spans touch the previous layer.
    let mut supported = vec![false; spans.len()];
    for (idx, &(row, span)) in spans.iter().enumerate() {
        let root = clusters.find(idx as u32) as usize;
        if !supported[root] && touches(prev, row, span, strict) {
            supported[root] = true;
        }
    }

    // Not using ArrayCluster::clusters as it drops single span regions, which
    // are exactly what the very bottom of a pointy island looks like.
    let mut regions = HashMap::<u32, Vec<_>>::new();
    for (idx, &span) in spans.iter().enumerate() {
        let root = clusters.find(idx as u32);
        (!supported[root as usize]).then(|| regions.entry(root).or_default().push(span));
    }

    regions.into_values().collect()
}

/// Checks if a span touches any span in the same or adjacent rows of a layer.
/// If `strict`, the span has to overlap one in the same row.
fn touches(layer: &Rows, row: usize, span: (u32, u32), strict: bool) -> bool {
    let rows = if strict {
        row..(row + 1).min(layer.len())
    } else {
        row.saturating_sub(1)..(row + 2).min(layer.len())
    };
    (layer.get(rows).unwrap_or_default().iter()).any(|spans| {
        // First span ending at or after this one starts.
        let i = spans.partition_point(|other| other.1 < span.0);
        spans.get(i).is_some_and(|&other| {
            if strict {
                span.0 < other.1 && other.0 < span.1
            } else {
                spans_touch(span, other)
            }
        })
    })
}

/// Picks points to support on an island and finds the matching points on the
/// model's surface.
fn to_island(
    collider: &Collider,
    spans: &[(usize, (u32, u32))],
    (origin, z): (Vector2<f32>, f32),
    spacing: f32,
) -> Island {
    let pixels = spans.iter().flat_map(|&(row, (start, end))| {
        (start..end).map(move |x| Vector2::new(x as f32 + 0.5, row as f32 + 0.5))
    });

    let count = spans.iter().map(|(_, (s, e))| (e - s) as f32).sum::<f32>();
    let center = pixels.clone().sum::<Vector2<f32>>() / count.max(1.0);

    // Always support the pixel closest to the center, then spread more
    // supports across large islands on a grid.
    let first = closest(pixels.clone(), center);
    let cell = spacing / PIXEL_SIZE;
    let mut cells = HashMap::new();
    for pixel in pixels.clone() {
        let key = (pixel / cell).map(|x| x.floor() as i32);
        let cell_center = (key.cast::<f32>() + Vector2::repeat(0.5)) * cell;
        let dist = (pixel - cell_center).magnitude_squared();
        let entry = cells.entry((key.x, key.y)).or_insert((pixel, dist));
        (dist < entry.1).then(|| *entry = (pixel, dist));
    }

    let rest =
        (cells.into_values().map(|x| x.0)).filter(|pixel| (first - pixel).magnitude() > cell * 0.5);
    let picks = std::iter::once(first).chain(rest);

    // Search a few layers around where the island was found for the surface.
    let contacts = (picks.into_iter())
        .filter_map(|pixel| {
            let xy = origin + pixel * PIXEL_SIZE;
            let below = xy.push(z - LAYER_HEIGHT * 3.0);
            let hit = collider.ray(below, Vector3::z())?;
            // Meshes that overlap themselves can have surfaces facing either
            // way, but seen from below the island, this is its underside.
            let hit = Contact::new(hit.point, hit.normal * -hit.normal.z.signum());
            (hit.point.z <= z + LAYER_HEIGHT * 2.0).then_some(hit)
        })
        .collect();

    Island {
        contacts,
        area: count * PIXEL_SIZE * PIXEL_SIZE,
    }
}

fn closest(points: impl Iterator<Item = Vector2<f32>>, target: Vector2<f32>) -> Vector2<f32> {
    points
        .min_by(|a, b| {
            let (a, b) = (
                (a - target).magnitude_squared(),
                (b - target).magnitude_squared(),
            );
            a.total_cmp(&b)
        })
        .unwrap_or(target)
}
