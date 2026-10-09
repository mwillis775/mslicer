//! Tree supports.
//!
//! Starts from individually routed supports, then repeatedly merges the
//! branches of nearby supports. Two branches descend at up to the branch angle
//! until they meet, and carry on down as a single, thicker branch. Once nothing
//! more can be merged, each trunk is routed down to the plate like a normal
//! support. Supports whose trunk can't be routed keep their own route.

use nalgebra::Vector3;
use rayon::iter::{IntoParallelIterator, ParallelIterator};

use crate::supports::{
    Support, collider::Collider, routing::Router, settings::SupportSettings, settings::SupportStyle,
};

/// Branches must meet at least this far above the top of the column bases, so
/// trunks aren't just stubs.
const MIN_TRUNK_HEIGHT: f32 = 2.0;
/// Number of merge partners checked for collisions before giving up on a node.
const MAX_MERGE_CHECKS: usize = 4;

#[derive(Clone)]
struct Node {
    position: Vector3<f32>,
    radius: f32,
    /// Indices of the supports that branch into this node.
    members: Vec<usize>,
}

/// The path of a support from its tip down to the trunk it's part of.
struct Branch {
    path: Vec<Vector3<f32>>,
    radii: Vec<f32>,
}

/// Merges the routes of all tree style supports into trees.
pub fn grow(
    collider: &Collider,
    router: &Router,
    settings: &SupportSettings,
    plate_z: f32,
    supports: &mut [Support],
) {
    let tree = &settings.tree;
    let clearance = settings.routing.clearance;
    let tan = tree.branch_angle.to_radians().tan();
    let min_z = settings.column_bottom(plate_z) + MIN_TRUNK_HEIGHT;
    let max_radius = tree.max_trunk_diameter / 2.0;

    let mut branches = (supports.iter())
        .map(|x| Branch {
            path: vec![x.route.path[0]],
            radii: vec![x.size.shaft_radius()],
        })
        .collect::<Vec<_>>();
    let mut nodes = (supports.iter().enumerate())
        .filter(|(_, x)| x.style == SupportStyle::Tree)
        .map(|(i, x)| Node {
            position: x.route.path[0],
            radius: x.size.shaft_radius(),
            members: vec![i],
        })
        .collect::<Vec<_>>();

    let branch_clear = |node: &Node, meet: Vector3<f32>, radius: f32| {
        collider.clear(node.position, meet, node.radius + clearance / 2.0)
            && collider.clear(meet, meet, radius + clearance)
    };

    // Each round merges every node with its best partner, highest nodes first,
    // until no more merges can be made.
    loop {
        nodes.sort_by(|a, b| b.position.z.total_cmp(&a.position.z));
        let mut used = vec![false; nodes.len()];
        let mut next = Vec::with_capacity(nodes.len());
        let mut merged = false;

        for i in 0..nodes.len() {
            if used[i] {
                continue;
            }
            used[i] = true;

            let a = &nodes[i];
            let mut partners = (0..nodes.len())
                .filter(|&j| !used[j])
                .filter_map(|j| {
                    let b = &nodes[j];
                    let distance = (b.position.xy() - a.position.xy()).magnitude();
                    if distance > tree.merge_distance {
                        return None;
                    }

                    let meet = meeting_point(a.position, b.position, tan);
                    (meet.z >= min_z).then_some((j, meet))
                })
                .collect::<Vec<_>>();

            // Prefer partners that can be met without descending much.
            partners.sort_by(|a, b| b.1.z.total_cmp(&a.1.z));
            let partner = (partners.into_iter().take(MAX_MERGE_CHECKS)).find(|&(j, meet)| {
                let b = &nodes[j];
                let radius = merged_radius(a.radius, b.radius, max_radius);
                branch_clear(a, meet, radius) && branch_clear(b, meet, radius)
            });

            let Some((j, meet)) = partner else {
                next.push(a.clone());
                continue;
            };

            used[j] = true;
            merged = true;

            let b = &nodes[j];
            let radius = merged_radius(a.radius, b.radius, max_radius);
            let members = [a.members.as_slice(), b.members.as_slice()].concat();
            for &member in members.iter() {
                let branch = &mut branches[member];
                if (branch.path.last().unwrap() - meet).magnitude_squared() > 1e-8 {
                    branch.path.push(meet);
                    branch.radii.push(radius);
                } else {
                    *branch.radii.last_mut().unwrap() = radius;
                }
            }

            next.push(Node {
                position: meet,
                radius,
                members,
            });
        }

        nodes = next;
        if !merged {
            break;
        }
    }

    // Route each trunk down from where its last branches met.
    let trunks = (nodes.into_par_iter())
        .filter(|x| x.members.len() > 1)
        .filter_map(|node| {
            let mut size = supports[node.members[0]].size;
            size.shaft_diameter = node.radius * 2.0;
            let route = router.route_from(node.position, &size)?;
            Some((node, route))
        })
        .collect::<Vec<_>>();

    for (node, route) in trunks {
        for member in node.members {
            let branch = &branches[member];
            let support = &mut supports[member];

            let trunk = &route.path[1..];
            support.route.path = [branch.path.as_slice(), trunk].concat();
            support.route.radii = (branch.radii.iter().copied())
                .chain(trunk.iter().map(|_| node.radius))
                .collect();
            support.route.foot = route.foot;
        }
    }
}

/// Radius of the branch formed by merging two branches. The cross-sectional
/// area is kept, so trunks are as strong as the branches they carry.
fn merged_radius(a: f32, b: f32, max: f32) -> f32 {
    a.hypot(b).min(max).max(a.max(b))
}

/// The highest point both `a` and `b` can reach by descending at most `tan`
/// horizontal units per vertical unit.
fn meeting_point(a: Vector3<f32>, b: Vector3<f32>, tan: f32) -> Vector3<f32> {
    let (high, low) = if a.z >= b.z { (a, b) } else { (b, a) };
    let offset = low.xy() - high.xy();
    let distance = offset.magnitude();
    if distance < 1e-6 {
        return low;
    }

    // Horizontal distance traveled by the higher branch. It makes up for the
    // height difference first, then both descend equally.
    let x = ((tan * (high.z - low.z) + distance) / 2.0).clamp(0.0, distance);
    let z = (high.z - x / tan).min(low.z - (distance - x) / tan);
    (high.xy() + offset / distance * x).push(z)
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use super::meeting_point;

    #[test]
    fn meeting_points() {
        let tan = 1.0;

        // Same height, meet halfway down a 45° V.
        let meet = meeting_point(
            Vector3::new(0.0, 0.0, 10.0),
            Vector3::new(4.0, 0.0, 10.0),
            tan,
        );
        assert!((meet - Vector3::new(2.0, 0.0, 8.0)).magnitude() < 1e-5);

        // One far above the other, joins the lower one where it is.
        let meet = meeting_point(
            Vector3::new(0.0, 0.0, 20.0),
            Vector3::new(4.0, 0.0, 10.0),
            tan,
        );
        assert!((meet - Vector3::new(4.0, 0.0, 10.0)).magnitude() < 1e-5);

        // Stacked directly on top of each other.
        let meet = meeting_point(
            Vector3::new(1.0, 1.0, 5.0),
            Vector3::new(1.0, 1.0, 9.0),
            tan,
        );
        assert!((meet - Vector3::new(1.0, 1.0, 5.0)).magnitude() < 1e-5);

        // Small height difference, both descend.
        let (a, b) = (Vector3::new(0.0, 0.0, 10.0), Vector3::new(6.0, 0.0, 8.0));
        let meet = meeting_point(a, b, tan);
        let slope = |p: Vector3<f32>| (p.xy() - meet.xy()).magnitude() / (p.z - meet.z);
        assert!((slope(a) - 1.0).abs() < 1e-4 && (slope(b) - 1.0).abs() < 1e-4);
    }
}
