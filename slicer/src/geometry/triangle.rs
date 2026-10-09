use nalgebra::{Vector2, Vector3};

use crate::{
    geometry::{Primitive, Ray},
    mesh::Mesh,
};

/// Intersects a plane with a triangle.
pub fn plane_triangle_intersection(
    mesh: &Mesh,
    points: &[Vector3<f32>],
    face: usize,
    height: f32,
) -> Option<[Vector2<f32>; 2]> {
    // Get all the vertices of the face
    let face = mesh.face(face);
    let v0 = points[face[0] as usize];
    let v1 = points[face[1] as usize];
    let v2 = points[face[2] as usize];

    // By subtracting the height from each vertex z coord, we can now check if
    // each line segment is crossing the plane if one end is above and one is
    // below. We can use xor to do this quickly.
    let (a, b, c) = (v0.z - height, v1.z - height, v2.z - height);
    let (a_pos, b_pos, c_pos) = (a > 0.0, b > 0.0, c > 0.0);

    let mut out = [Vector2::zeros(); 2];
    let mut n = 0;

    // Closure called when the line segment from v0 to v1 is intersecting the
    // plane. t is how far along the line the intersection is and intersections,
    // it well the point that is intersecting with the plane.
    let mut push_intersection = |a: f32, b: f32, v0: Vector2<f32>, v1: Vector2<f32>| {
        let t = a / (a - b);
        let intersection = v0 + t * (v1 - v0);
        out[n] = intersection;
        n += 1;
    };

    // And as you can see my aversion to else blocks now includes if blocks...
    // Anyway here we check each line segment of the face is intersecting, if it
    // is we push the intersection to the out vec. Note that the cases of A→B
    // and B→A need to be handled separately due to floating point rounding so
    // that two segments that should be attached have bit-exact float positions.
    (a_pos && !b_pos).then(|| push_intersection(a, b, v0.xy(), v1.xy()));
    (!a_pos && b_pos).then(|| push_intersection(b, a, v1.xy(), v0.xy()));

    (b_pos && !c_pos).then(|| push_intersection(b, c, v1.xy(), v2.xy()));
    (!b_pos && c_pos).then(|| push_intersection(c, b, v2.xy(), v1.xy()));

    (c_pos && !a_pos).then(|| push_intersection(c, a, v2.xy(), v0.xy()));
    (!c_pos && a_pos).then(|| push_intersection(a, c, v0.xy(), v2.xy()));

    (n == 2).then_some(out)
}

/// References:
///  - [Ray-Tracing: Rendering a Triangle](https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/ray-triangle-intersection-geometric-solution.html)
///  - [Check whether a point is within a 3D Triangle](https://math.stackexchange.com/questions/4322/check-whether-a-point-is-within-a-3d-triangle)
pub fn triangle_intersection<Type: Primitive>(
    ([v0, v1, v2], normal): ([Vector3<f32>; 3], Vector3<f32>),
    ray: Ray,
) -> Option<(f32, Vector3<f32>)> {
    // Check if triangle and direction are parallel
    let denominator = normal.dot(&ray.direction);
    if denominator == 0.0 {
        return None;
    }

    // Find distance along line where intersection would occur on the triangle's
    // plane. If the value is not in range for the primitive, no intersection.
    let t = -(normal.dot(&ray.origin) - normal.dot(&v0)) / denominator;
    if !Type::in_range(t) {
        return None;
    }

    // Check if the point of intersection is actually inside triangle.
    let intersection = ray.origin + t * ray.direction;
    let c0 = (v1 - v0).cross(&(intersection - v0));
    let c1 = (v2 - v1).cross(&(intersection - v1));
    let c2 = (v0 - v2).cross(&(intersection - v2));

    let inside_triangle =
        normal.dot(&c0) >= 0.0 && normal.dot(&c1) >= 0.0 && normal.dot(&c2) >= 0.0;

    inside_triangle.then_some((t, intersection))
}

/// Reference: "Closest Point on Triangle to Point" from Real-Time Collision Detection by Christer Ericson
pub fn closest_point([v0, v1, v2]: [Vector3<f32>; 3], point: Vector3<f32>) -> Vector3<f32> {
    let ab = v1 - v0;
    let ac = v2 - v0;
    let bc = v2 - v1;

    // Compute parametric position s for projection P’ of P on AB,
    let snom = (point - v0).dot(&ab);
    let sdenom = (point - v1).dot(&(v0 - v1));

    // Compute parametric position t for projection P’ of P on AC,
    let tnom = (point - v0).dot(&ac);
    let tdenom = (point - v2).dot(&(v0 - v2));
    if snom <= 0.0 && tnom <= 0.0 {
        return v0;
    }

    // Compute parametric position u for projection P’ of P on BC,
    let unom = (point - v1).dot(&bc);
    if sdenom <= 0.0 && unom <= 0.0 {
        return v1;
    }

    let udenom = (point - v2).dot(&(v1 - v2));
    if tdenom <= 0.0 && udenom <= 0.0 {
        return v2;
    }

    // P is outside (or on) AB if the triple scalar product [N PA PB] <= 0
    let n = (v1 - v0).cross(&(v2 - v0));
    let vc = n.dot(&(v0 - point).cross(&(v1 - point)));

    // If P outside AB and within feature region of AB, return projection of P onto AB
    if vc <= 0.0 && snom >= 0.0 && sdenom >= 0.0 {
        return v0 + snom / (snom + sdenom) * ab;
    }

    // P is outside (or on) BC if the triple scalar product [N PB PC] <= 0
    // If P outside BC and within feature region of BC, return projection of P onto BC
    let va = n.dot(&(v1 - point).cross(&(v2 - point)));
    if va <= 0.0 && unom >= 0.0 && udenom >= 0.0 {
        return v1 + unom / (unom + udenom) * bc;
    }

    // P is outside (or on) CA if the triple scalar product [N PC PA] <= 0
    // If P outside CA and within feature region of CA, return projection of P onto CA
    let vb = n.dot(&(v2 - point).cross(&(v0 - point)));
    if vb <= 0.0 && tnom >= 0.0 && tdenom >= 0.0 {
        return v0 + tnom / (tnom + tdenom) * ac;
    }

    // P must project inside face region. Compute Q using barycentric coordinates
    let u = va / (va + vb + vc);
    let v = vb / (va + vb + vc);
    let w = 1.0 - u - v;

    u * v0 + v * v1 + w * v2
}

/// Returns the closest points between segments `p1→q1` and `p2→q2` as
/// `(point_on_first, point_on_second)`.
///
/// Reference: "Closest Points of Two Line Segments" from Real-Time Collision Detection by Christer Ericson
pub fn closest_points_segments(
    (p1, q1): (Vector3<f32>, Vector3<f32>),
    (p2, q2): (Vector3<f32>, Vector3<f32>),
) -> (Vector3<f32>, Vector3<f32>) {
    const EPSILON: f32 = 1e-12;

    let d1 = q1 - p1;
    let d2 = q2 - p2;
    let r = p1 - p2;
    let a = d1.magnitude_squared();
    let e = d2.magnitude_squared();
    let f = d2.dot(&r);

    let (s, t) = if a <= EPSILON && e <= EPSILON {
        (0.0, 0.0)
    } else if a <= EPSILON {
        (0.0, (f / e).clamp(0.0, 1.0))
    } else {
        let c = d1.dot(&r);
        if e <= EPSILON {
            ((-c / a).clamp(0.0, 1.0), 0.0)
        } else {
            let b = d1.dot(&d2);
            let denom = a * e - b * b;

            let mut s = if denom > EPSILON {
                ((b * f - c * e) / denom).clamp(0.0, 1.0)
            } else {
                0.0
            };

            let mut t = (b * s + f) / e;
            if t < 0.0 {
                t = 0.0;
                s = (-c / a).clamp(0.0, 1.0);
            } else if t > 1.0 {
                t = 1.0;
                s = ((b - c) / a).clamp(0.0, 1.0);
            }

            (s, t)
        }
    };

    (p1 + d1 * s, p2 + d2 * t)
}

/// Returns the squared distance between the segment `a→b` and a triangle.
/// This is exact: the minimum is either zero (the segment pierces the
/// triangle), between an endpoint and the triangle's face, or between the
/// segment and one of the triangle's edges.
pub fn segment_triangle_distance_sq(
    tri @ [v0, v1, v2]: [Vector3<f32>; 3],
    (a, b): (Vector3<f32>, Vector3<f32>),
) -> f32 {
    let normal = (v1 - v0).cross(&(v2 - v0));
    let (da, db) = (normal.dot(&(a - v0)), normal.dot(&(b - v0)));

    // Segment endpoints are on opposite sides of the plane, check if the
    // crossing point is inside the triangle.
    if (da <= 0.0) != (db <= 0.0) && da != db {
        let point = a + (b - a) * (da / (da - db));
        let c0 = (v1 - v0).cross(&(point - v0));
        let c1 = (v2 - v1).cross(&(point - v1));
        let c2 = (v0 - v2).cross(&(point - v2));
        if normal.dot(&c0) >= 0.0 && normal.dot(&c1) >= 0.0 && normal.dot(&c2) >= 0.0 {
            return 0.0;
        }
    }

    let mut best = (closest_point(tri, a) - a)
        .magnitude_squared()
        .min((closest_point(tri, b) - b).magnitude_squared());

    for edge in [(v0, v1), (v1, v2), (v2, v0)] {
        let (p, q) = closest_points_segments((a, b), edge);
        best = best.min((p - q).magnitude_squared());
    }

    best
}
