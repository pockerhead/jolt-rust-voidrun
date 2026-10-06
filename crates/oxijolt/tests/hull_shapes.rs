//! Convex hull bodies in contact and under queries.

mod common;

use common::meshes::*;
use common::*;
use oxijolt::*;

#[test]
fn irregular_rotated_hull_falls_and_rests_on_a_floor() {
    let hull = Shape::new_convex_hull(&irregular_points(), 0.05).unwrap();
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    add_floor(&mut world);
    let settings = BodySettings::new_dynamic()
        .position(RVec3::new(0.0, 2.0, 0.0))
        .rotation(quat_about(Vec3::new(0.0, 0.6, 0.8), 0.9));
    let id = world.create_body(&hull, &settings).unwrap();
    step(&mut world, 240);
    assert!(is_finite_body(&world, id));
    let body = world.body(id).unwrap();
    assert!(is_calm(&body), "{:?}", body.linear_velocity());
    // Resting: above the floor (top face y = 0) within the penetration slop, below the start.
    let y = body.position().y;
    assert!(y > 0.2 && y < 1.0, "{y}");
}

#[test]
fn thin_box_hull_is_a_dynamic_body() {
    let hull = Shape::new_convex_hull(&box_corners(Vec3::new(1.0, 0.005, 1.0)), 0.0).unwrap();
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    add_floor(&mut world);
    let id = world
        .create_body(
            &hull,
            &BodySettings::new_dynamic().position(RVec3::new(0.0, 0.5, 0.0)),
        )
        .unwrap();
    step(&mut world, 180);
    let body = world.body(id).unwrap();
    assert!(is_calm(&body));
    let y = body.position().y;
    assert!((-0.02..0.05).contains(&y), "{y}");
}

/// A needle hull along a direction rotated 30 degrees about two axes, `length` long and
/// `thickness` thick.
fn needle(length: f32, thickness: f32) -> Shape {
    let rotation = quat_about(Vec3::new(0.0, 0.0, 1.0), 30f32.to_radians());
    let tilt = quat_about(Vec3::new(1.0, 0.0, 0.0), 30f32.to_radians());
    let rotate = |v: Vec3| {
        // Rotation by `rotation` then `tilt`, written out with the quaternion formula.
        let apply = |q: Quat, v: Vec3| {
            let (qx, qy, qz, w) = (q.x, q.y, q.z, q.w);
            let t = Vec3::new(
                2.0 * (qy * v.z - qz * v.y),
                2.0 * (qz * v.x - qx * v.z),
                2.0 * (qx * v.y - qy * v.x),
            );
            Vec3::new(
                v.x + w * t.x + (qy * t.z - qz * t.y),
                v.y + w * t.y + (qz * t.x - qx * t.z),
                v.z + w * t.z + (qx * t.y - qy * t.x),
            )
        };
        apply(tilt, apply(rotation, v))
    };
    let points: Vec<Vec3> = box_corners(Vec3::new(length / 2.0, thickness / 2.0, thickness / 2.0))
        .into_iter()
        .map(rotate)
        .collect();
    Shape::new_convex_hull(&points, 0.0).unwrap()
}

#[test]
fn needles_too_thin_for_the_hull_builder_are_degenerate() {
    // 20 m by 2 mm near the origin: rounding tilts faces across the width by more than Jolt's
    // hull tolerance over the length.
    let rotate = |p: Vec3| Vec3::new(0.8 * p.x - 0.6 * p.y, 0.6 * p.x + 0.8 * p.y, p.z);
    let points: Vec<Vec3> = box_corners(Vec3::new(10.0, 0.001, 0.001))
        .into_iter()
        .map(rotate)
        .collect();
    assert!(matches!(
        Shape::new_convex_hull(&points, 0.0),
        Err(ShapeError::ConvexHull(ConvexHullError::Degenerate))
    ));
}

#[test]
fn rotated_needle_hull_is_refused_as_dynamic_and_accepted_as_static() {
    // 2 m by 1 cm: thick enough for Jolt's hull builder, too slender for a dynamic inertia.
    let shape = needle(2.0, 0.01);
    let mut world = world(Vec3::ZERO, 1);
    match world.create_body(&shape, &BodySettings::new_dynamic()) {
        Err(BodyError::InvalidValue(rule)) => assert!(rule.contains("inertia"), "{rule}"),
        other => panic!("the needle was accepted as dynamic: {other:?}"),
    }
    assert!(world
        .create_body(&shape, &BodySettings::new_static())
        .is_ok());
}

/// A static irregular hull at the origin, raised so its top is the highest point at y = 1.6.
fn static_hull_world() -> (PhysicsWorld, BodyId) {
    let hull = Shape::new_convex_hull(&box_corners(Vec3::new(1.0, 0.5, 1.0)), 0.05).unwrap();
    let mut world = world(Vec3::ZERO, 1);
    let id = world
        .create_body(
            &hull,
            &BodySettings::new_static().position(RVec3::new(0.0, 1.0, 0.0)),
        )
        .unwrap();
    (world, id)
}

#[test]
fn queries_hit_a_static_hull_from_above() {
    let (world, id) = static_hull_world();
    let ray = RayCast::new(RVec3::new(0.2, 5.0, -0.3), Vec3::new(0.0, -10.0, 0.0));
    let hit = world.cast_ray(ray, &QueryFilter::new()).unwrap().unwrap();
    assert_eq!(hit.body, id);
    assert!(
        (ray.point_at(hit.fraction).y - 1.5).abs() < 1.0e-4,
        "{hit:?}"
    );
    assert!(hit.normal.y > 0.99, "{hit:?}");

    let sphere = Shape::new_sphere(0.25).unwrap();
    let cast = ShapeCast::new(
        &sphere,
        RVec3::new(0.0, 4.0, 0.0),
        Quat::IDENTITY,
        Vec3::new(0.0, -5.0, 0.0),
    );
    let hit = world
        .cast_shape(&cast, &QueryFilter::new())
        .unwrap()
        .unwrap();
    assert_eq!(hit.body, id);
    // The sphere stops when its bottom touches the top face at y = 1.5.
    assert!((hit.distance - 2.25).abs() < 1.0e-3, "{hit:?}");
    assert!(hit.normal.y > 0.99, "{hit:?}");

    let capsule = Shape::new_capsule(0.5, 0.2).unwrap();
    let query = CollideShape::new(&capsule, RVec3::new(0.0, 2.1, 0.0), Quat::IDENTITY);
    let hits = world.collide_shape(&query, &QueryFilter::new()).unwrap();
    assert_eq!(hits.len(), 1, "{hits:?}");
    assert_eq!(hits[0].body, id);
    // Capsule bottom at 2.1 - 0.7 = 1.4: 0.1 m into the hull.
    assert!((hits[0].penetration_depth - 0.1).abs() < 1.0e-3, "{hits:?}");
    assert!(hits[0].normal.y > 0.99, "{hits:?}");
}

/// Clouds on which Jolt's hull builder asserts in a build with the `asserts` feature
/// (docs/limits.md#convex-hulls): flat cones with a dense rim a few coplanar distances off its
/// plane, at the origin or recentred from far out, and a densely sampled noisy box 1.8 km out.
/// They pass the hull rules. Without asserts Jolt refuses the first six and builds the last two.
const BUILDER_ASSERT_CLOUDS: [(&str, &str); 8] = [
    (
        "flat_cone_1",
        include_str!("fixtures/hulls/flat_cone_1.txt"),
    ),
    (
        "flat_cone_2",
        include_str!("fixtures/hulls/flat_cone_2.txt"),
    ),
    (
        "flat_cone_3",
        include_str!("fixtures/hulls/flat_cone_3.txt"),
    ),
    (
        "recentred_cone_1",
        include_str!("fixtures/hulls/recentred_cone_1.txt"),
    ),
    (
        "recentred_cone_2",
        include_str!("fixtures/hulls/recentred_cone_2.txt"),
    ),
    (
        "far_noisy_box",
        include_str!("fixtures/hulls/far_noisy_box.txt"),
    ),
    (
        "built_flat_cone",
        include_str!("fixtures/hulls/built_flat_cone.txt"),
    ),
    (
        "built_recentred_cone",
        include_str!("fixtures/hulls/built_recentred_cone.txt"),
    ),
];

/// Points of a fixture: one point per line, three numbers each.
fn fixture_points(text: &str) -> Vec<Vec3> {
    text.lines()
        .filter(|line| !line.trim().is_empty())
        .map(|line| {
            let c: Vec<f32> = line
                .split_whitespace()
                .map(|value| value.parse().unwrap())
                .collect();
            Vec3::new(c[0], c[1], c[2])
        })
        .collect()
}

#[test]
fn clouds_the_hull_builder_asserts_on_are_refused_or_built_without_asserts() {
    // With asserts Jolt aborts the process on these clouds: a documented limit of that feature.
    if oxijolt_sys::ASSERTS_ENABLED {
        return;
    }
    for (name, text) in BUILDER_ASSERT_CLOUDS {
        let points = fixture_points(text);
        let hull = match Shape::new_convex_hull(&points, 0.0) {
            Err(ShapeError::Rejected(_)) => continue,
            Ok(hull) => hull,
            Err(error) => panic!("{name}: {error}"),
        };
        // Whatever Jolt builds is a usable shape: a ray down through the mean of the points,
        // which lies inside the hull, hits it.
        let mut world = world(Vec3::ZERO, 1);
        let settings = BodySettings::new_static().position(RVec3::new(0.0, 100.0, 0.0));
        let id = world.create_body(&hull, &settings).unwrap();
        let count = points.len() as f32;
        let mean_x = points.iter().map(|p| p.x).sum::<f32>() / count;
        let mean_z = points.iter().map(|p| p.z).sum::<f32>() / count;
        let ray = RayCast::new(
            RVec3::new(mean_x as Real, 300.0, mean_z as Real),
            Vec3::new(0.0, -400.0, 0.0),
        );
        let hit = world.cast_ray(ray, &QueryFilter::new()).unwrap();
        assert_eq!(hit.map(|hit| hit.body), Some(id), "{name}");
    }
}
