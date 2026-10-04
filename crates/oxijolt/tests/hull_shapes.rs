//! Convex hull bodies in contact and under queries.

mod common;

use common::*;
use oxijolt::*;

/// An irregular hull of eleven points, roughly 1 m across, with its lowest point at y = -0.45.
fn irregular_points() -> Vec<Vec3> {
    vec![
        Vec3::new(-0.5, -0.45, -0.4),
        Vec3::new(0.55, -0.4, -0.35),
        Vec3::new(0.45, -0.45, 0.5),
        Vec3::new(-0.4, -0.35, 0.45),
        Vec3::new(0.0, 0.6, 0.0),
        Vec3::new(-0.45, 0.3, -0.3),
        Vec3::new(0.4, 0.35, -0.25),
        Vec3::new(0.3, 0.25, 0.45),
        Vec3::new(-0.3, 0.2, 0.4),
        Vec3::new(0.1, -0.1, 0.0),
        Vec3::new(0.0, 0.0, -0.55),
    ]
}

/// The eight corners of a box with the given half extents.
fn box_corners(half: Vec3) -> Vec<Vec3> {
    let mut points = Vec::new();
    for x in [-half.x, half.x] {
        for y in [-half.y, half.y] {
            for z in [-half.z, half.z] {
                points.push(Vec3::new(x, y, z));
            }
        }
    }
    points
}

fn is_finite_body(world: &PhysicsWorld, id: BodyId) -> bool {
    let body = world.body(id).unwrap();
    let position: [Real; 3] = body.position().into();
    let rotation: [f32; 4] = body.rotation().into();
    let linear: [f32; 3] = body.linear_velocity().into();
    let angular: [f32; 3] = body.angular_velocity().into();
    position.iter().all(|v| v.is_finite())
        && rotation
            .iter()
            .chain(&linear)
            .chain(&angular)
            .all(|v| v.is_finite())
}

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
/// 0.002 m thick.
fn needle(length: f32) -> Shape {
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
    let points: Vec<Vec3> = box_corners(Vec3::new(length / 2.0, 0.001, 0.001))
        .into_iter()
        .map(rotate)
        .collect();
    Shape::new_convex_hull(&points, 0.0).unwrap()
}

#[test]
fn rotated_needle_hull_is_refused_as_dynamic_and_accepted_as_static() {
    let shape = needle(20.0);
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
