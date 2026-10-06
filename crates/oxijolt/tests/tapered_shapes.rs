//! Tapered capsules and cones as bodies and under queries.

mod common;

use common::meshes::*;
use common::*;
use oxijolt::*;

// `Real` is `f32` without the `double-precision` feature.
#[allow(clippy::unnecessary_cast)]
fn f32_of(value: Real) -> f32 {
    value as f32
}

#[test]
fn tapered_capsule_and_cone_rest_on_a_mesh() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    world
        .create_body(&flat_grid(16, 1.0), &BodySettings::new_static())
        .unwrap();
    let capsule = Shape::new_tapered_capsule(0.4, 0.15, 0.3).unwrap();
    let cone = Shape::new_tapered_cylinder(0.5, 0.0, 0.4, 0.05).unwrap();
    let ids = [(&capsule, -2.0), (&cone, 2.0)].map(|(shape, x)| {
        world
            .create_body(
                shape,
                &BodySettings::new_dynamic().position(RVec3::new(x, 1.5, 0.0)),
            )
            .unwrap()
    });
    step(&mut world, 300);
    for id in ids {
        assert!(is_finite_body(&world, id), "{id:?}");
        let body = world.body(id).unwrap();
        assert!(is_calm(&body), "{id:?}");
        let y = f32_of(body.position().y);
        assert!(y > 0.0 && y < 1.0, "{id:?}: {y}");
    }
    // The cone stands on its base: its origin, the middle of the axis, is 0.5 m up.
    let y = f32_of(world.body(ids[1]).unwrap().position().y);
    assert!((y - 0.5).abs() < 0.03, "{y}");
}

#[test]
fn queries_hit_tapered_shapes() {
    let mut world = world(Vec3::ZERO, 1);
    let capsule = Shape::new_tapered_capsule(0.4, 0.15, 0.3).unwrap();
    let cone = Shape::new_tapered_cylinder(0.5, 0.0, 0.4, 0.0).unwrap();
    let ids = [(&capsule, -2.0), (&cone, 2.0)].map(|(shape, x)| {
        world
            .create_body(
                shape,
                &BodySettings::new_static().position(RVec3::new(x, 0.0, 0.0)),
            )
            .unwrap()
    });
    // The top of the capsule's upper sphere is at 0.4 + 0.15; the cone's tip at 0.5.
    for (id, x, top) in [(ids[0], -2.0, 0.55), (ids[1], 2.0, 0.5)] {
        let ray = RayCast::new(RVec3::new(x, 3.0, 0.0), Vec3::new(0.0, -5.0, 0.0));
        let hit = world.cast_ray(&ray, &QueryFilter::new()).unwrap().unwrap();
        assert_eq!(hit.body, id);
        let y = f32_of(ray.point_at(hit.fraction).y);
        assert!((y - top).abs() < 1.0e-3, "{y} vs {top}");

        let sphere = Shape::new_sphere(0.2).unwrap();
        let cast = ShapeCast::new(
            &sphere,
            RVec3::new(x, 3.0, 0.0),
            Quat::IDENTITY,
            Vec3::new(0.0, -5.0, 0.0),
        );
        let hit = world
            .cast_shape(&cast, &QueryFilter::new())
            .unwrap()
            .unwrap();
        assert_eq!(hit.body, id);
        assert!(hit.distance > 2.0 && hit.distance < 2.4, "{hit:?}");
    }
}
