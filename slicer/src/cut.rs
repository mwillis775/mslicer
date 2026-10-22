//! Planar mesh cutting. Splits a mesh into the parts above and below a plane,
//! closing the open cross-section of each part with a triangulated cap so both
//! halves stay watertight.

use std::collections::{HashMap, HashSet};

use nalgebra::{Vector2, Vector3};

/// Raw vertices and faces of one side of a cut.
pub struct CutPart {
    pub vertices: Vec<Vector3<f32>>,
    pub faces: Vec<[u32; 3]>,
}

pub struct CutResult {
    /// The part on the side the plane normal points towards.
    pub above: Option<CutPart>,
    pub below: Option<CutPart>,
    /// Number of cross-section loops that could not be closed. Non-zero values
    /// mean the input mesh was not watertight along the cut.
    pub open_loops: usize,
}

/// Cuts the mesh with the plane through `origin` with the given `normal`. The
/// vertices are expected to already be in the space the plane is defined in.
pub fn cut_mesh(
    vertices: &[Vector3<f32>],
    faces: &[[u32; 3]],
    origin: Vector3<f32>,
    normal: Vector3<f32>,
) -> CutResult {
    let normal = normal.normalize();
    let dist = (vertices.iter())
        .map(|v| (v - origin).dot(&normal))
        .collect::<Vec<_>>();
    // Vertices exactly on the plane count as above, so every edge is strictly
    // classified and no zero length cap segments are produced.
    let above = |i: u32| dist[i as usize] >= 0.0;

    let mut points = vertices.to_vec();
    let mut intersections = HashMap::<(u32, u32), u32>::new();
    let mut intersect = |a: u32, b: u32| {
        *intersections
            .entry((a.min(b), a.max(b)))
            .or_insert_with(|| {
                let (da, db) = (dist[a as usize], dist[b as usize]);
                let (va, vb) = (vertices[a as usize], vertices[b as usize]);
                points.push(va + (vb - va) * (da / (da - db)));
                points.len() as u32 - 1
            })
    };

    let (mut faces_above, mut faces_below) = (Vec::new(), Vec::new());
    // Directed edges along the open boundary of the above part.
    let mut boundary = Vec::new();

    for &[a, b, c] in faces {
        let sides = [above(a), above(b), above(c)];
        if sides[0] == sides[1] && sides[1] == sides[2] {
            [&mut faces_below, &mut faces_above][sides[0] as usize].push([a, b, c]);
            continue;
        }

        // Rotate the face (keeping winding) so the lone vertex comes first.
        let [l, x, y] = if sides[1] == sides[2] {
            [a, b, c]
        } else if sides[0] == sides[2] {
            [b, c, a]
        } else {
            [c, a, b]
        };

        let (i1, i2) = (intersect(l, x), intersect(l, y));
        let (lone, rest) = if above(l) {
            boundary.push((i1, i2));
            (&mut faces_above, &mut faces_below)
        } else {
            boundary.push((i2, i1));
            (&mut faces_below, &mut faces_above)
        };

        lone.push([l, i1, i2]);
        rest.push([i1, x, y]);
        rest.push([i1, y, i2]);
    }

    let (loops, open_loops) = chain_loops(&boundary);
    // Faces against the normal, as the above part's cap. The below part's cap
    // is the same with the opposite winding.
    let cap_above = triangulate_cap(&mut points, &loops, origin, normal);
    let cap_below = cap_above
        .iter()
        .map(|&[a, b, c]| [a, c, b])
        .collect::<Vec<_>>();

    faces_above.extend(cap_above);
    faces_below.extend(cap_below);

    CutResult {
        above: compact(&points, faces_above),
        below: compact(&points, faces_below),
        open_loops,
    }
}

/// Joins directed boundary edges into closed loops of point indices. Returns
/// the loops along with how many chains failed to close.
fn chain_loops(edges: &[(u32, u32)]) -> (Vec<Vec<u32>>, usize) {
    let mut next = HashMap::with_capacity(edges.len());
    for &(a, b) in edges {
        next.insert(a, b);
    }

    let starts = edges.iter().map(|x| x.0).collect::<Vec<_>>();
    let (mut loops, mut open) = (Vec::new(), 0);

    for start in starts {
        let Some(mut current) = next.remove(&start) else {
            continue;
        };

        let mut points = vec![start];
        while current != start {
            points.push(current);
            match next.remove(&current) {
                Some(n) => current = n,
                None => break,
            }
        }

        if current != start {
            open += 1;
        }

        // Even unclosed chains are capped so small gaps in defective meshes
        // don't leave the whole cross-section open.
        if points.len() >= 3 {
            loops.push(points);
        }
    }

    (loops, open)
}

/// Triangulates the planar polygons formed by the loops, handling holes, and
/// returns triangles facing against the normal that index into `points`. New
/// points may be added.
fn triangulate_cap(
    points: &mut Vec<Vector3<f32>>,
    loops: &[Vec<u32>],
    origin: Vector3<f32>,
    normal: Vector3<f32>,
) -> Vec<[u32; 3]> {
    let helper = if normal.x.abs() < 0.9 {
        Vector3::x()
    } else {
        Vector3::y()
    };
    let u = normal.cross(&helper).normalize();
    let v = normal.cross(&u);
    let project = |p: Vector3<f32>| {
        let p = p - origin;
        Vector2::new(p.dot(&u) as f64, p.dot(&v) as f64)
    };

    let polygons = (loops.iter())
        .map(|l| {
            (l.iter())
                .map(|&i| project(points[i as usize]))
                .collect::<Vec<_>>()
        })
        .collect::<Vec<_>>();

    // A loop nested inside an even number of other loops is an outer boundary,
    // otherwise it is a hole in the closest enclosing loop.
    let containers = (0..loops.len())
        .map(|i| {
            (0..loops.len())
                .filter(|&j| j != i && point_in_polygon(polygons[i][0], &polygons[j]))
                .collect::<Vec<_>>()
        })
        .collect::<Vec<_>>();

    let mut holes = vec![Vec::new(); loops.len()];
    for (i, parents) in containers.iter().enumerate() {
        if parents.len() % 2 == 1
            && let Some(&parent) =
                (parents.iter()).find(|&&p| containers[p].len() + 1 == parents.len())
        {
            holes[parent].push(i);
        }
    }

    let mut out = Vec::new();
    for (outer, parents) in containers.iter().enumerate() {
        if parents.len() % 2 == 1 {
            continue;
        }

        let rings = std::iter::once(outer).chain(holes[outer].iter().copied());
        let mut coords = Vec::new();
        let mut indices = Vec::new();
        let mut hole_starts = Vec::new();
        for (n, ring) in rings.clone().enumerate() {
            if n > 0 {
                hole_starts.push(indices.len());
            }

            for (&p, &i) in polygons[ring].iter().zip(&loops[ring]) {
                coords.extend([p.x, p.y]);
                indices.push(i);
            }
        }

        let Ok(triangles) = earcutr::earcut(&coords, &hole_starts, 2) else {
            continue;
        };
        let mut triangles = (triangles.chunks_exact(3))
            .map(|t| [indices[t[0]], indices[t[1]], indices[t[2]]])
            .collect::<Vec<_>>();

        // Earcut emits every triangle with the same winding, so orient them
        // all by their total area. Slivers alone can't be trusted for this.
        let area = (triangles.iter())
            .map(|&[a, b, c]| {
                let [a, b, c] = [a, b, c].map(|x| project(points[x as usize]));
                (b - a).perp(&(c - a))
            })
            .sum::<f64>();
        if area > 0.0 {
            triangles.iter_mut().for_each(|t| t.swap(1, 2));
        }

        restore_skipped(points, rings.map(|x| loops[x].as_slice()), &mut triangles);
        out.extend(triangles);
    }

    out
}

/// Earcut skips collinear and duplicate points, leaving cap edges that span
/// several edges of the cut surface. This splits the cap triangles at those
/// points so the cap shares every edge with the cut surface.
fn restore_skipped<'a>(
    points: &mut Vec<Vector3<f32>>,
    rings: impl Iterator<Item = &'a [u32]>,
    triangles: &mut Vec<[u32; 3]>,
) {
    let used = triangles.iter().flatten().copied().collect::<HashSet<_>>();

    // Skipped points between two used ones, keyed by the (min, max) pair of
    // used points and ordered from min to max.
    let mut runs = HashMap::<(u32, u32), Vec<u32>>::new();
    for ring in rings {
        let Some(first) = ring.iter().position(|x| used.contains(x)) else {
            continue;
        };

        let mut start = ring[first];
        let mut run = Vec::new();
        for n in 1..=ring.len() {
            let point = ring[(first + n) % ring.len()];
            if !used.contains(&point) {
                run.push(point);
                continue;
            }

            if !run.is_empty() {
                if start > point {
                    run.reverse();
                }
                runs.insert(
                    (start.min(point), start.max(point)),
                    std::mem::take(&mut run),
                );
            }
            start = point;
        }
    }

    if runs.is_empty() {
        return;
    }

    let mut out = Vec::with_capacity(triangles.len() + runs.values().map(Vec::len).sum::<usize>());
    for &t in triangles.iter() {
        // The triangle's outline with the skipped points along each edge.
        let mut outline = Vec::new();
        let mut split_edges = Vec::new();
        for i in 0..3 {
            let (a, b) = (t[i], t[(i + 1) % 3]);
            outline.push(a);
            if let Some(run) = runs.get(&(a.min(b), a.max(b))) {
                split_edges.push(i);
                if a < b {
                    outline.extend(run);
                } else {
                    outline.extend(run.iter().rev());
                }
            }
        }

        match split_edges[..] {
            [] => out.push(t),
            // Fan from the corner opposite the split edge.
            [edge] => {
                let corner = t[(edge + 2) % 3];
                let start = outline.iter().position(|&x| x == t[edge]).unwrap();
                let end = outline
                    .iter()
                    .position(|&x| x == t[(edge + 1) % 3])
                    .unwrap();
                let end = if end == 0 { outline.len() } else { end };
                for i in start..end {
                    let next = outline[(i + 1) % outline.len()];
                    out.push([outline[i], next, corner]);
                }
            }
            // Fan from a new point in the middle of the triangle.
            _ => {
                let center = t.iter().map(|&x| points[x as usize]).sum::<Vector3<f32>>() / 3.0;
                points.push(center);
                let center = points.len() as u32 - 1;
                for i in 0..outline.len() {
                    out.push([outline[i], outline[(i + 1) % outline.len()], center]);
                }
            }
        }
    }

    *triangles = out;
}

fn point_in_polygon(point: Vector2<f64>, polygon: &[Vector2<f64>]) -> bool {
    let mut inside = false;
    let mut j = polygon.len() - 1;
    for i in 0..polygon.len() {
        let (a, b) = (polygon[i], polygon[j]);
        if (a.y > point.y) != (b.y > point.y)
            && point.x < (b.x - a.x) * (point.y - a.y) / (b.y - a.y) + a.x
        {
            inside = !inside;
        }
        j = i;
    }
    inside
}

/// Builds a part containing only the vertices referenced by its faces.
fn compact(points: &[Vector3<f32>], faces: Vec<[u32; 3]>) -> Option<CutPart> {
    if faces.is_empty() {
        return None;
    }

    let mut remap = HashMap::new();
    let mut vertices = Vec::new();
    let faces = (faces.into_iter())
        .map(|face| {
            face.map(|i| {
                *remap.entry(i).or_insert_with(|| {
                    vertices.push(points[i as usize]);
                    vertices.len() as u32 - 1
                })
            })
        })
        .collect();

    Some(CutPart { vertices, faces })
}

#[cfg(test)]
mod test {
    use std::collections::HashMap;

    use nalgebra::Vector3;

    use super::{CutPart, cut_mesh};

    /// An axis aligned cube from -1 to 1 with outward facing triangles.
    fn cube() -> (Vec<Vector3<f32>>, Vec<[u32; 3]>) {
        let vertices = (0..8)
            .map(|i| {
                let c = |bit| if i & bit != 0 { 1.0 } else { -1.0 };
                Vector3::new(c(1), c(2), c(4))
            })
            .collect();
        let faces = vec![
            [0, 2, 1],
            [1, 2, 3], // -z
            [4, 5, 6],
            [5, 7, 6], // +z
            [0, 1, 4],
            [1, 5, 4], // -y
            [2, 6, 3],
            [3, 6, 7], // +y
            [0, 4, 2],
            [2, 4, 6], // -x
            [1, 3, 5],
            [3, 7, 5], // +x
        ];
        (vertices, faces)
    }

    /// A ring (square with a square hole) extruded from z=-1 to z=1.
    fn square_tube() -> (Vec<Vector3<f32>>, Vec<[u32; 3]>) {
        let mut vertices = Vec::new();
        for z in [-1.0, 1.0] {
            for r in [2.0, 1.0] {
                for (x, y) in [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)] {
                    vertices.push(Vector3::new(x * r, y * r, z));
                }
            }
        }

        // Index of corner `c` on ring `r` (0 outer, 1 inner) at level `z`.
        let idx = |z: u32, r: u32, c: u32| z * 8 + r * 4 + c % 4;
        let mut faces = Vec::new();
        for c in 0..4 {
            let (o0, o1) = (idx(0, 0, c), idx(0, 0, c + 1));
            let (o2, o3) = (idx(1, 0, c), idx(1, 0, c + 1));
            faces.extend([[o0, o1, o3], [o0, o3, o2]]);

            let (i0, i1) = (idx(0, 1, c), idx(0, 1, c + 1));
            let (i2, i3) = (idx(1, 1, c), idx(1, 1, c + 1));
            faces.extend([[i0, i3, i1], [i0, i2, i3]]);

            let (b0, b1, b2, b3) = (o0, o1, idx(0, 1, c), idx(0, 1, c + 1));
            faces.extend([[b0, b3, b1], [b0, b2, b3]]);

            let (t0, t1, t2, t3) = (o2, o3, idx(1, 1, c), idx(1, 1, c + 1));
            faces.extend([[t0, t1, t3], [t0, t3, t2]]);
        }
        (vertices, faces)
    }

    fn assert_watertight(part: &CutPart) {
        let mut edges = HashMap::<(u32, u32), i32>::new();
        for &[a, b, c] in &part.faces {
            for (a, b) in [(a, b), (b, c), (c, a)] {
                *edges.entry((a, b)).or_default() += 1;
                *edges.entry((b, a)).or_default() -= 1;
            }
        }
        assert!(edges.values().all(|&x| x == 0), "mesh is not closed");
    }

    fn volume(part: &CutPart) -> f32 {
        (part.faces.iter())
            .map(|f| {
                let [a, b, c] = f.map(|i| part.vertices[i as usize]);
                a.dot(&b.cross(&c)) / 6.0
            })
            .sum()
    }

    #[test]
    fn cut_cube_horizontal() {
        let (vertices, faces) = cube();
        let result = cut_mesh(&vertices, &faces, Vector3::new(0.0, 0.0, 0.5), Vector3::z());
        let (above, below) = (result.above.unwrap(), result.below.unwrap());

        assert_eq!(result.open_loops, 0);
        assert_watertight(&above);
        assert_watertight(&below);
        assert!((volume(&above) - 2.0).abs() < 1e-4);
        assert!((volume(&below) - 6.0).abs() < 1e-4);
    }

    #[test]
    fn cut_cube_tilted() {
        let (vertices, faces) = cube();
        let normal = Vector3::new(1.0, 1.0, 1.0);
        let result = cut_mesh(&vertices, &faces, Vector3::zeros(), normal);
        let (above, below) = (result.above.unwrap(), result.below.unwrap());

        assert_watertight(&above);
        assert_watertight(&below);
        assert!((volume(&above) - 4.0).abs() < 1e-4);
        assert!((volume(&below) - 4.0).abs() < 1e-4);
    }

    #[test]
    fn cut_tube_with_hole() {
        let (vertices, faces) = square_tube();
        let result = cut_mesh(&vertices, &faces, Vector3::zeros(), Vector3::z());
        let (above, below) = (result.above.unwrap(), result.below.unwrap());

        assert_watertight(&above);
        assert_watertight(&below);
        // (4² - 2²) * 1 for each half.
        assert!((volume(&above) - 12.0).abs() < 1e-4);
        assert!((volume(&below) - 12.0).abs() < 1e-4);
    }

    #[test]
    fn cut_misses_mesh() {
        let (vertices, faces) = cube();
        let result = cut_mesh(&vertices, &faces, Vector3::new(0.0, 0.0, 5.0), Vector3::z());
        assert!(result.above.is_none());
        assert_eq!(result.below.unwrap().faces.len(), 12);
    }
}
