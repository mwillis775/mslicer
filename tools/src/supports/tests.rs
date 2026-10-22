use std::collections::{HashMap, HashSet};

use common::progress::Progress;
use nalgebra::Vector3;
use slicer::{
    builder::MeshBuilder,
    geometry::{
        bvh::Bvh,
        triangle::{closest_points_segments, segment_triangle_distance_sq},
    },
    mesh::Mesh,
};

use crate::supports::{
    Contact, Foot, Support,
    bracing::brace,
    collider::Collider,
    generate::{generate, new_support, reroute, reroute_all},
    islands::{find_islands, find_islands_in},
    mesh::SupportMesh,
    orient::{Shape, detailed_cost, orient, rotation_for_up},
    routing::Router,
    settings::{FORMAT_VERSION, SupportSettings, SupportStyle},
};

fn with_bvh(mesh: Mesh) -> (Mesh, Bvh) {
    let bvh = Bvh::build(mesh.inner(), Progress::new());
    (mesh, bvh)
}

/// Axis aligned box as a closed mesh.
fn add_box(builder: &mut MeshBuilder, min: Vector3<f32>, max: Vector3<f32>) {
    let corner = |i: u32| {
        Vector3::new(
            if i & 1 != 0 { max.x } else { min.x },
            if i & 2 != 0 { max.y } else { min.y },
            if i & 4 != 0 { max.z } else { min.z },
        )
    };
    let base = builder.next_idx();
    (0..8).for_each(|i| _ = builder.add_vertex(corner(i)));
    #[rustfmt::skip]
    let faces = [
        [0, 2, 1], [1, 2, 3], [4, 5, 6], [5, 7, 6], // -z, +z
        [0, 1, 4], [1, 5, 4], [2, 6, 3], [3, 6, 7], // -y, +y
        [0, 4, 2], [2, 4, 6], [1, 3, 5], [3, 7, 5], // -x, +x
    ];
    faces
        .into_iter()
        .for_each(|f| builder.add_face(f.map(|x| x + base)));
}

/// Brute force minimum distance from a segment to a mesh.
fn brute_distance(mesh: &Mesh, a: Vector3<f32>, b: Vector3<f32>) -> f32 {
    (0..mesh.face_count())
        .map(|f| segment_triangle_distance_sq(mesh.face_verts(f), (a, b)).sqrt())
        .fold(f32::MAX, f32::min)
}

/// Checks that every edge has a matching reverse edge and returns the volume.
fn closed_volume(mesh: &Mesh) -> (usize, f64) {
    let mut edges = HashMap::<(u32, u32), i32>::new();
    let mut volume = 0.0;
    let v = mesh.vertices();
    for f in mesh.faces() {
        let [a, b, c] = f.map(|x| v[x as usize]);
        volume += a.dot(&b.cross(&c)) as f64 / 6.0;
        for (x, y) in [(f[0], f[1]), (f[1], f[2]), (f[2], f[0])] {
            *edges.entry((x, y)).or_default() += 1;
        }
    }

    let open = (edges.keys()).filter(|(a, b)| !edges.contains_key(&(*b, *a)));
    (open.count(), volume)
}

#[test]
fn segment_triangle_distance_matches_sampling() {
    let tri = [
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(4.0, 0.0, 0.0),
        Vector3::new(0.0, 3.0, 1.0),
    ];

    let mut seed = 1u32;
    let mut rand = || {
        seed ^= seed << 13;
        seed ^= seed >> 17;
        seed ^= seed << 5;
        (seed as f32 / u32::MAX as f32) * 10.0 - 5.0
    };

    for _ in 0..500 {
        let a = Vector3::new(rand(), rand(), rand());
        let b = Vector3::new(rand(), rand(), rand());
        let exact = segment_triangle_distance_sq(tri, (a, b)).sqrt();

        // Dense sampling over both shapes can only overestimate.
        let mut sampled = f32::MAX;
        for i in 0..=60 {
            let p = a.lerp(&b, i as f32 / 60.0);
            for u in 0..=30 {
                for v in 0..=(30 - u) {
                    let (u, v) = (u as f32 / 30.0, v as f32 / 30.0);
                    let q = tri[0] + (tri[1] - tri[0]) * u + (tri[2] - tri[0]) * v;
                    sampled = sampled.min((p - q).magnitude());
                }
            }
        }

        assert!(exact <= sampled + 1e-4, "{exact} > {sampled}");
        assert!(sampled - exact < 0.2, "{exact} ≪ {sampled}");
    }
}

#[test]
fn segment_segment_parallel() {
    let (p, q) = closest_points_segments(
        (Vector3::new(0.0, 0.0, 0.0), Vector3::new(2.0, 0.0, 0.0)),
        (Vector3::new(1.0, 1.0, 0.0), Vector3::new(3.0, 1.0, 0.0)),
    );
    assert!(((p - q).magnitude() - 1.0).abs() < 1e-6);
}

#[test]
fn collider_matches_brute_force() {
    let mut builder = MeshBuilder::new();
    builder.add_sphere(Vector3::new(0.0, 0.0, 10.0), 5.0, 16);
    add_box(
        &mut builder,
        Vector3::new(8.0, -2.0, 0.0),
        Vector3::new(12.0, 2.0, 4.0),
    );
    let (mut mesh, bvh) = with_bvh(builder.build());
    mesh.set_position(Vector3::new(1.0, -2.0, 3.0));
    mesh.set_rotation(Vector3::new(0.3, 0.0, 1.0));
    let collider = Collider::new(&mesh, &bvh);

    for i in 0..200 {
        let t = i as f32 * 0.37;
        let a = Vector3::new(t.sin() * 12.0, t.cos() * 12.0, 5.0 + (t * 1.3).sin() * 10.0);
        let b = a + Vector3::new((t * 0.7).cos(), (t * 2.1).sin(), -1.0) * 6.0;

        let expected = brute_distance(&mesh, a, b);
        let distance = collider.distance(a, b, 100.0);
        assert!(
            (expected - distance).abs() < 1e-3,
            "{expected} vs {distance}"
        );

        for radius in [0.5, 2.0] {
            assert_eq!(collider.clear(a, b, radius), expected >= radius);
        }
    }
}

#[test]
fn support_mesh_is_closed() {
    for style in SupportStyle::ALL {
        let settings = SupportSettings::for_style(style);
        let size = settings.size;
        let contact = Contact::new(Vector3::new(0.0, 0.0, 10.0), -Vector3::z());
        let landing = Contact::new(Vector3::new(5.0, 0.0, 2.0), Vector3::z());

        let supports = [
            Support::new(
                style,
                size,
                contact,
                contact,
                crate::supports::Route {
                    path: vec![Vector3::new(0.0, 0.0, 8.0), Vector3::new(2.0, 0.0, 6.0)],
                    radii: vec![size.shaft_radius(), size.shaft_radius() * 1.5],
                    foot: Foot::Plate,
                    fill: None,
                },
            ),
            Support::new(
                style,
                size,
                contact,
                Contact::new(Vector3::new(5.0, 0.0, 10.0), -Vector3::z()),
                crate::supports::Route::new(
                    vec![Vector3::new(5.0, 0.0, 8.0)],
                    Foot::Model(landing),
                ),
            ),
        ];
        let braces = [crate::supports::Brace {
            start: Vector3::new(0.0, 0.0, 3.0),
            end: Vector3::new(5.0, 0.0, 8.0),
        }];

        for raft in [true, false] {
            let mut settings = settings.clone();
            settings.base.raft = raft;
            let (mesh, map) = SupportMesh::build(&settings, 0.0, &supports, &braces).unwrap();
            let (open, volume) = closed_volume(&mesh);
            assert_eq!(open, 0, "{style:?}");
            assert!(volume > 0.0, "{style:?}");
            assert_eq!(map.len(), 2);

            let (min, _) = mesh.bounds();
            assert!(min.z >= -1e-5, "support goes below the plate: {}", min.z);
        }
    }
}

/// A table: a slab held up by one leg, with a block sitting under one side of
/// the slab. Supports under the block side have to lean around it.
fn table() -> (Mesh, Bvh) {
    let mut builder = MeshBuilder::new();
    add_box(
        &mut builder,
        Vector3::new(-10.0, -10.0, 20.0),
        Vector3::new(10.0, 10.0, 22.0),
    );
    add_box(
        &mut builder,
        Vector3::new(-1.5, -1.5, 5.0),
        Vector3::new(1.5, 1.5, 20.0),
    );
    add_box(
        &mut builder,
        Vector3::new(3.0, -4.0, 5.0),
        Vector3::new(9.0, 4.0, 10.0),
    );
    with_bvh(builder.build())
}

/// Checks that a support keeps its distance from the model. Standard supports
/// keep the full clearance, tapered supports and tree branches may use up to
/// half of it, see [`Router::routing_size`] and [`crate::supports::tree`].
fn assert_route_clear(mesh: &Mesh, settings: &SupportSettings, support: &Support) {
    let clearance = settings.routing.clearance;
    let slack = if support.style == SupportStyle::Standard {
        0.0
    } else {
        clearance / 2.0
    };
    let shaft = |i: usize| {
        if support.style.tapered() {
            support.size.shaft_radius()
        } else {
            support.radius_at(i)
        }
    };

    let path = &support.route.path;
    for (i, pair) in path.windows(2).enumerate() {
        let distance = brute_distance(mesh, pair[0], pair[1]);
        let radius = shaft(i) + clearance - slack;
        assert!(distance >= radius - 1e-3, "shaft touches model: {distance}");
    }

    let radius = shaft(path.len() - 1) + clearance - slack;

    let top = support.column_top();
    let bottom = match support.route.foot {
        Foot::Plate => top.xy().push(settings.column_bottom(0.0)),
        Foot::Model(landing) => {
            crate::supports::routing::foot_top(top, &landing, &support.size).unwrap()
        }
    };
    let distance = brute_distance(mesh, top, bottom);
    assert!(
        distance >= radius - 1e-3,
        "column touches model: {distance}"
    );
}

#[test]
fn routes_straight_down_when_clear() {
    let (mesh, bvh) = table();
    let collider = Collider::new(&mesh, &bvh);
    let settings = SupportSettings::default();
    let router = Router::new(&collider, &settings, 0.0);

    let contact = Contact::new(Vector3::new(-6.0, -6.0, 20.0), -Vector3::z());
    let support = new_support(&collider, &router, &settings, contact).unwrap();
    assert_eq!(support.route.path.len(), 1);
    assert_eq!(support.route.foot, Foot::Plate);
    assert_route_clear(&mesh, &settings, &support);
}

#[test]
fn routes_around_obstacles() {
    let (mesh, bvh) = table();
    let collider = Collider::new(&mesh, &bvh);
    let mut settings = SupportSettings::default();
    settings.routing.model_landing = false;
    let router = Router::new(&collider, &settings, 0.0);

    // Directly above the block, 2mm from its edge.
    for (x, y) in [(7.0, 2.0), (8.0, 0.0), (4.5, -3.0)] {
        let contact = Contact::new(Vector3::new(x, y, 20.0), -Vector3::z());
        let support = new_support(&collider, &router, &settings, contact).unwrap();
        assert!(support.route.path.len() >= 2, "should have leaned");
        assert_eq!(support.route.foot, Foot::Plate);
        assert_route_clear(&mesh, &settings, &support);
    }
}

#[test]
fn lands_on_model_when_boxed_in() {
    // A cup: a slab with walls hanging down all around it, sitting on a big
    // floor. Supports under the slab can't get out.
    let mut builder = MeshBuilder::new();
    add_box(
        &mut builder,
        Vector3::new(-10.0, -10.0, 20.0),
        Vector3::new(10.0, 10.0, 22.0),
    );
    add_box(
        &mut builder,
        Vector3::new(-30.0, -30.0, 5.0),
        Vector3::new(30.0, 30.0, 8.0),
    );
    for (min, max) in [
        ((-10.0, -10.0), (-8.0, 10.0)),
        ((8.0, -10.0), (10.0, 10.0)),
        ((-10.0, -10.0), (10.0, -8.0)),
        ((-10.0, 8.0), (10.0, 10.0)),
    ] {
        let (min, max) = (
            Vector3::new(min.0, min.1, 8.0),
            Vector3::new(max.0, max.1, 20.0),
        );
        add_box(&mut builder, min, max);
    }
    let (mesh, bvh) = with_bvh(builder.build());
    let collider = Collider::new(&mesh, &bvh);

    let mut settings = SupportSettings::default();
    let contact = Contact::new(Vector3::new(0.0, 0.0, 20.0), -Vector3::z());

    settings.routing.model_landing = false;
    let router = Router::new(&collider, &settings, 0.0);
    assert!(new_support(&collider, &router, &settings, contact).is_err());

    settings.routing.model_landing = true;
    let router = Router::new(&collider, &settings, 0.0);
    let support = new_support(&collider, &router, &settings, contact).unwrap();
    let Foot::Model(landing) = support.route.foot else {
        panic!("expected model landing");
    };
    assert!((landing.point.z - 8.0).abs() < 1e-3);
    assert_route_clear(&mesh, &settings, &support);
}

#[test]
fn finds_islands() {
    // A floating cone pointing down, a floating box and a box on the plate.
    let mut builder = MeshBuilder::new();
    builder.add_cylinder(
        (Vector3::new(0.0, 0.0, 10.0), Vector3::new(0.0, 0.0, 15.0)),
        (0.0, 4.0),
        16,
    );
    add_box(
        &mut builder,
        Vector3::new(10.0, 0.0, 6.0),
        Vector3::new(14.0, 2.0, 9.0),
    );
    add_box(
        &mut builder,
        Vector3::new(-14.0, 0.0, 0.0),
        Vector3::new(-10.0, 2.0, 9.0),
    );
    let (mesh, bvh) = with_bvh(builder.build());
    let collider = Collider::new(&mesh, &bvh);

    let islands = find_islands(&collider, 0.0, 3.0);
    assert_eq!(islands.len(), 2);

    let cone = (islands.iter())
        .find(|x| x.contacts[0].point.x.abs() < 1.0)
        .expect("cone tip island");
    assert!((cone.contacts[0].point - Vector3::new(0.0, 0.0, 10.0)).magnitude() < 0.3);

    // 4×2mm bottom, 3mm spacing: the center plus at least one more point.
    let block = islands
        .iter()
        .find(|x| x.contacts[0].point.x > 5.0)
        .unwrap();
    assert!((block.area - 8.0).abs() < 1.0, "area {}", block.area);
    assert!(block.contacts.len() >= 2);
    assert!((block.contacts.iter()).all(|x| (x.point.z - 6.0).abs() < 1e-3));
}

#[test]
fn supports_floating_sphere() {
    let mut builder = MeshBuilder::new();
    builder.add_sphere(Vector3::new(0.0, 0.0, 30.0), 8.0, 48);
    let (mesh, bvh) = with_bvh(builder.build());
    let collider = Collider::new(&mesh, &bvh);
    let settings = SupportSettings::default();

    let generated = generate(&collider, &settings, 0.0, Progress::new());
    assert!(generated.failed.is_empty(), "{:?}", generated.failed);

    let supports = &generated.supports;
    assert!(supports.len() >= 6, "{} supports", supports.len());

    // The bottom of the sphere must be supported.
    let lowest = (supports.iter())
        .map(|x| x.contact.point.z)
        .fold(f32::MAX, f32::min);
    assert!(lowest < 22.0 + 0.05, "lowest support at {lowest}");

    // Every overhanging point should be close to a support.
    let overhang = settings.placement.overhang_angle.to_radians();
    let max_gap = settings.placement.spacing * 1.5 + 0.5;
    for i in 0..400 {
        let (theta, phi) = (i as f32 * 2.399_963, (i as f32 / 400.0) * overhang);
        let dir = Vector3::new(phi.sin() * theta.cos(), phi.sin() * theta.sin(), -phi.cos());
        let point = Vector3::new(0.0, 0.0, 30.0) + dir * 8.0;
        let gap = (supports.iter())
            .map(|x| (x.contact.point - point).magnitude())
            .fold(f32::MAX, f32::min);
        assert!(gap < max_gap, "{point:?} is {gap}mm from a support");
    }

    for support in supports {
        assert!(support.contact.normal.z < 0.0);
        assert_route_clear(&mesh, &settings, support);
    }

    // Tall columns get braced, and braces never touch the model.
    let braces = brace(&collider, supports, &settings, 0.0);
    assert!(!braces.is_empty());
    let radius = settings.bracing.diameter / 2.0 + settings.routing.clearance;
    for x in braces {
        assert!(brute_distance(&mesh, x.start, x.end) >= radius - 1e-3);
    }
}

#[test]
fn reroutes_after_rotation() {
    let (mesh, bvh) = table();
    let mut mesh = mesh;
    let settings = SupportSettings::default();

    let support = {
        let collider = Collider::new(&mesh, &bvh);
        let router = Router::new(&collider, &settings, 0.0);
        let contact = Contact::new(Vector3::new(-6.0, -6.0, 20.0), -Vector3::z());
        new_support(&collider, &router, &settings, contact).unwrap()
    };

    mesh.set_rotation(Vector3::new(0.1, 0.0, 0.0));
    mesh.set_scale(Vector3::repeat(1.2));
    let collider = Collider::new(&mesh, &bvh);
    let router = Router::new(&collider, &settings, 0.0);
    let moved = reroute(&collider, &router, &support).unwrap();

    let expected = mesh.transform(&support.local.point);
    assert!((moved.contact.point - expected).magnitude() < 1e-4);
    assert_eq!(moved.id, support.id);
}

/// A floating sphere, with its lowest point at z = 22.
fn floating_sphere() -> (Mesh, Bvh) {
    let mut builder = MeshBuilder::new();
    builder.add_sphere(Vector3::new(0.0, 0.0, 30.0), 8.0, 48);
    with_bvh(builder.build())
}

#[test]
fn every_style_supports_floating_sphere() {
    let (mesh, bvh) = floating_sphere();
    let collider = Collider::new(&mesh, &bvh);

    for style in SupportStyle::ALL {
        let settings = SupportSettings::for_style(style);
        let generated = generate(&collider, &settings, 0.0, Progress::new());
        assert!(
            generated.failed.is_empty(),
            "{style:?}: {:?}",
            generated.failed
        );
        assert!(generated.supports.len() >= 6, "{style:?}");

        for support in generated.supports.iter() {
            assert_eq!(support.style, style);
            (style != SupportStyle::Tree).then(|| assert!(support.route.radii.is_empty()));
            assert_route_clear(&mesh, &settings, support);
        }

        let braces = brace(&collider, &generated.supports, &settings, 0.0);
        let radius = settings.bracing.diameter / 2.0 + settings.routing.clearance;
        for x in braces.iter() {
            assert!(brute_distance(&mesh, x.start, x.end) >= radius - 1e-3);
        }

        let (support_mesh, _) =
            SupportMesh::build(&settings, 0.0, &generated.supports, &braces).unwrap();
        let (open, volume) = closed_volume(&support_mesh);
        assert_eq!(open, 0, "{style:?}");
        assert!(volume > 0.0, "{style:?}");
    }
}

#[test]
fn tree_branches_merge_into_trunks() {
    let (mesh, bvh) = floating_sphere();
    let collider = Collider::new(&mesh, &bvh);
    let settings = SupportSettings::for_style(SupportStyle::Tree);

    let generated = generate(&collider, &settings, 0.0, Progress::new());
    let supports = &generated.supports;
    let trunks = (supports.iter())
        .filter(|x| x.route.foot == Foot::Plate)
        .map(|x| x.column_top().map(f32::to_bits))
        .collect::<HashSet<_>>();
    assert!(
        trunks.len() * 3 <= supports.len(),
        "{} trunks for {} supports",
        trunks.len(),
        supports.len()
    );

    let max_angle = settings
        .tree
        .branch_angle
        .max(settings.routing.max_shaft_angle);
    let max_radius = settings.tree.max_trunk_diameter / 2.0;
    for support in supports {
        let path = &support.route.path;
        assert_eq!(path.len(), support.route.radii.len().max(path.len()));

        // Branches only go down, never too far from vertical, and only get
        // thicker towards the trunk.
        for (i, pair) in path.windows(2).enumerate() {
            let offset = pair[1] - pair[0];
            assert!(offset.z < 0.0);
            let angle = offset.xy().magnitude().atan2(-offset.z).to_degrees();
            assert!(angle <= max_angle + 0.1, "branch at {angle}°");
            assert!(support.radius_at(i + 1) >= support.radius_at(i) - 1e-6);
        }
        assert!(support.column_radius() <= max_radius + 1e-6);
    }

    // Trees are re-grown after the model is moved.
    let rerouted = reroute_all(&collider, &settings, 0.0, supports);
    assert_eq!(rerouted.len(), supports.len());
    let rerouted_trunks = (rerouted.iter())
        .map(|x| x.column_top().map(f32::to_bits))
        .collect::<HashSet<_>>();
    assert!(rerouted_trunks.len() <= trunks.len() + 2);
}

#[test]
fn lattice_braces_in_both_directions() {
    let (mesh, bvh) = floating_sphere();
    let collider = Collider::new(&mesh, &bvh);

    let count = |style| {
        let settings = SupportSettings::for_style(style);
        let mut generated = generate(&collider, &SupportSettings::default(), 0.0, Progress::new());
        generated.supports.iter_mut().for_each(|x| x.style = style);
        brace(&collider, &generated.supports, &settings, 0.0).len()
    };

    let (standard, lattice) = (count(SupportStyle::Standard), count(SupportStyle::Lattice));
    assert!(lattice > standard * 3 / 2, "{lattice} vs {standard}");
}

#[test]
fn sliders_map_to_settings() {
    let mut settings = SupportSettings::default();
    assert!((settings.aggressiveness() - 0.5).abs() < 1e-3);

    settings.set_aggressiveness(0.5);
    let defaults = SupportSettings::default();
    assert!((settings.placement.spacing - defaults.placement.spacing).abs() < 1e-3);
    assert!((settings.placement.min_spacing - defaults.placement.min_spacing).abs() < 1e-3);
    assert!((settings.placement.overhang_angle - defaults.placement.overhang_angle).abs() < 1e-3);

    // More aggressive means denser supports on more of the model.
    settings.set_aggressiveness(0.9);
    assert!(settings.placement.spacing < defaults.placement.spacing);
    assert!(settings.placement.overhang_angle > defaults.placement.overhang_angle);
    assert!((settings.aggressiveness() - 0.9).abs() < 1e-3);

    settings.set_tip_diameter(0.2);
    assert_eq!(settings.size.tip_diameter, 0.2);
    assert!(settings.size.tip_penetration < 0.2 * 0.5);
    settings.set_tip_diameter(5.0);
    assert_eq!(settings.size.tip_diameter, settings.size.shaft_diameter);
}

#[test]
fn settings_and_supports_roundtrip() {
    use common::serde::{DynamicSerializer, SliceDeserializer};

    let (mesh, bvh) = floating_sphere();
    let collider = Collider::new(&mesh, &bvh);
    let mut settings = SupportSettings::for_style(SupportStyle::Tree);
    settings.tree.branch_angle = 33.0;
    settings.gap_fill.enabled = true;
    settings.gap_fill.max_height = 7.0;
    let generated = generate(&collider, &settings, 0.0, Progress::new());

    let mut ser = DynamicSerializer::new();
    settings.serialize(&mut ser);
    generated
        .supports
        .iter()
        .for_each(|x| x.serialize(&mut ser));
    let bytes = ser.into_inner();

    let mut des = SliceDeserializer::new(&bytes);
    assert_eq!(
        SupportSettings::deserialize(&mut des, FORMAT_VERSION),
        settings
    );
    for support in generated.supports.iter() {
        let loaded = Support::deserialize(&mut des, FORMAT_VERSION);
        assert_eq!(loaded.style, support.style);
        assert_eq!(loaded.route, support.route);
    }
}

/// Two floating slabs with a gap between them too narrow for any support.
fn sandwich() -> (Mesh, Bvh) {
    let mut builder = MeshBuilder::new();
    add_box(
        &mut builder,
        Vector3::new(-6.0, -6.0, 8.0),
        Vector3::new(6.0, 6.0, 11.0),
    );
    add_box(
        &mut builder,
        Vector3::new(-6.0, -6.0, 11.6),
        Vector3::new(6.0, 6.0, 15.0),
    );
    with_bvh(builder.build())
}

/// Islands left in the model with its supports in place.
fn leftover_area(
    mesh: &Mesh,
    collider: &Collider,
    settings: &SupportSettings,
    supports: &[Support],
) -> f32 {
    let mut builder = MeshBuilder::new();
    let (vertices, faces) = builder.raw_mut();
    vertices.extend_from_slice(mesh.vertices());
    faces.extend_from_slice(mesh.faces());
    let (support_mesh, _) = SupportMesh::build(settings, 0.0, supports, &[]).unwrap();
    let offset = vertices.len() as u32;
    vertices.extend_from_slice(support_mesh.vertices());
    faces.extend(support_mesh.faces().iter().map(|f| f.map(|x| x + offset)));
    let combined = builder.build();
    [0.0, 0.5]
        .into_iter()
        .flat_map(|phase| {
            find_islands_in(collider, &combined, 0.0, settings.placement.spacing, phase)
        })
        .map(|x| x.area)
        .sum()
}

#[test]
fn gap_fill_fills_narrow_gaps() {
    let (mesh, bvh) = sandwich();
    let collider = Collider::new(&mesh, &bvh);

    let mut settings = SupportSettings::default();
    let without = generate(&collider, &settings, 0.0, Progress::new());
    assert!(!without.failed.is_empty(), "the gap can't be supported");
    assert!(
        without
            .supports
            .iter()
            .all(|x| x.style != SupportStyle::Fill)
    );

    settings.gap_fill.enabled = true;
    let with = generate(&collider, &settings, 0.0, Progress::new());
    let fills = (with.supports.iter())
        .filter(|x| x.style == SupportStyle::Fill)
        .collect::<Vec<_>>();
    assert!(with.failed.is_empty(), "{:?}", with.failed);

    // The whole gap is one fill, covering all of it, sunk into both slabs.
    assert_eq!(fills.len(), 1);
    let shape = fills[0].route.fill.as_ref().unwrap();
    assert!(shape.area() > 12.0 * 12.0 * 0.95, "area {}", shape.area());
    for cell in shape.cells.iter() {
        assert!(cell.top > 11.6 && cell.top < 11.9, "top {}", cell.top);
        assert!(
            cell.bottom < 11.0 && cell.bottom > 10.7,
            "bottom {}",
            cell.bottom
        );
    }

    // No supports are left inside the gap, and nothing is left hanging.
    assert!(
        (with.supports.iter())
            .filter(|x| x.style != SupportStyle::Fill)
            .all(|x| x.contact.point.z < 9.0)
    );
    assert_eq!(
        leftover_area(&mesh, &collider, &settings, &with.supports),
        0.0
    );

    // It re-routes to the same shape, and builds a closed mesh that doesn't
    // stick out past the slabs.
    let router = Router::new(&collider, &settings, 0.0);
    let rerouted = reroute(&collider, &router, fills[0]).unwrap();
    assert_eq!(rerouted.route, fills[0].route);

    let fills = fills.into_iter().cloned().collect::<Vec<_>>();
    let (fill_mesh, _) = SupportMesh::build(&settings, 0.0, &fills, &[]).unwrap();
    let (open, volume) = closed_volume(&fill_mesh);
    assert_eq!(open, 0);
    assert!(volume > 12.0 * 12.0 * 0.6 * 0.9, "volume {volume}");
    let (min, max) = fill_mesh.bounds();
    assert!(
        min.xy().min() > -6.2 && max.xy().max() < 6.2,
        "{min:?} {max:?}"
    );

    // Gaps taller than the limit are left alone.
    settings.gap_fill.max_height = 0.3;
    let limited = generate(&collider, &settings, 0.0, Progress::new());
    assert!(
        limited
            .supports
            .iter()
            .all(|x| x.style != SupportStyle::Fill)
    );
    assert!(!limited.failed.is_empty());
}

#[test]
fn fill_roundtrip() {
    use common::serde::{DynamicSerializer, SliceDeserializer};

    let (mesh, bvh) = sandwich();
    let collider = Collider::new(&mesh, &bvh);
    let mut settings = SupportSettings::default();
    settings.gap_fill.enabled = true;
    let generated = generate(&collider, &settings, 0.0, Progress::new());
    let fill = (generated.supports.iter())
        .find(|x| x.style == SupportStyle::Fill)
        .unwrap();

    let mut ser = DynamicSerializer::new();
    fill.serialize(&mut ser);
    let bytes = ser.into_inner();
    let loaded = Support::deserialize(&mut SliceDeserializer::new(&bytes), FORMAT_VERSION);
    assert_eq!(loaded.route, fill.route);
    assert!(!loaded.is_stale());
}

#[test]
fn rotation_for_up_points_up() {
    for i in 0..50 {
        let t = i as f32 * 0.37;
        let up = Vector3::new(t.sin(), (t * 1.7).cos(), (t * 0.3).sin() - 0.5).normalize();
        for yaw in [0.0, 1.1, -2.5] {
            let r = rotation_for_up(up, yaw);
            assert!((r.z - yaw).abs() < 1e-6);
            let rotated = nalgebra::Rotation3::from_euler_angles(r.x, r.y, r.z) * up;
            assert!(
                (rotated - Vector3::z()).magnitude() < 1e-4,
                "{up:?} -> {rotated:?}"
            );
        }
    }
}

#[test]
fn orients_flat_plate_on_its_side() {
    let mut builder = MeshBuilder::new();
    add_box(
        &mut builder,
        Vector3::new(-15.0, -15.0, -1.0),
        Vector3::new(15.0, 15.0, 1.0),
    );
    let (mut mesh, bvh) = with_bvh(builder.build());
    mesh.set_rotation(Vector3::new(0.0, 0.0, 0.4));

    let settings = SupportSettings::default();
    let shape = Shape::new(&mesh);
    let platform = Vector3::new(200.0, 200.0, 200.0);
    let flat = detailed_cost(&mesh, &bvh, &settings, platform, &shape, mesh.rotation());
    let best = orient(&mesh, &bvh, &settings, platform, Progress::new());
    assert!(best.cost < flat * 0.5, "{} vs flat {}", best.cost, flat);
    assert!((best.rotation.z - 0.4).abs() < 1e-6, "yaw is kept");

    // The plate's face normal shouldn't end up pointing up or down.
    let rotation =
        nalgebra::Rotation3::from_euler_angles(best.rotation.x, best.rotation.y, best.rotation.z);
    let normal = rotation * Vector3::z();
    assert!(normal.z.abs() < 0.9, "still lying flat: {normal:?}");

    // Already good orientations are kept.
    mesh.set_rotation(best.rotation);
    let again = orient(&mesh, &bvh, &settings, platform, Progress::new());
    assert!(again.cost <= best.cost * 1.01);

    // On a build volume too short to stand it up, it stays flat.
    mesh.set_rotation(Vector3::new(0.0, 0.0, 0.4));
    let short = Vector3::new(200.0, 200.0, 12.0);
    let best = orient(&mesh, &bvh, &settings, short, Progress::new());
    let (min, max) = {
        let mut mesh = mesh.clone();
        mesh.set_rotation(best.rotation);
        mesh.bounds()
    };
    assert!(max.z - min.z + settings.placement.lift <= 12.0);
}
