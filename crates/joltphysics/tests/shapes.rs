//! Shapes and the closest-hit ray.

mod common;

use common::*;
use joltphysics::*;

fn down_from(x: Real, y: Real, z: Real, length: f32) -> RayCast {
    RayCast::new(RVec3::new(x, y, z), Vec3::new(0.0, -length, 0.0))
}

#[test]
fn ray_hits_a_body_right_after_creation() {
    let mut world = world(Vec3::ZERO, 1);
    let floor = add_floor(&mut world);
    let hit = world
        .cast_ray(down_from(0.0, 5.0, 0.0, 10.0))
        .unwrap()
        .expect("the floor is hit");
    assert_eq!(hit.body, floor);
    assert!((hit.fraction - 0.5).abs() <= 1.0e-6, "{hit:?}");
}

#[test]
fn ray_misses_return_none() {
    let mut world = world(Vec3::ZERO, 1);
    add_floor(&mut world);
    let up = RayCast::new(RVec3::new(0.0, 5.0, 0.0), Vec3::new(0.0, 10.0, 0.0));
    assert_eq!(world.cast_ray(up), Ok(None));
}

#[test]
fn ray_starting_inside_a_box_hits_at_zero() {
    let mut world = world(Vec3::ZERO, 1);
    let floor = add_floor(&mut world);
    let hit = world
        .cast_ray(down_from(0.0, -0.5, 0.0, 10.0))
        .unwrap()
        .expect("the floor is hit");
    assert_eq!(hit.body, floor);
    assert_eq!(hit.fraction, 0.0);
}

#[test]
fn invalid_rays_are_rejected() {
    let world = world(Vec3::ZERO, 1);
    let invalid = [
        RayCast::new(RVec3::new(Real::NAN, 0.0, 0.0), Vec3::new(0.0, -1.0, 0.0)),
        RayCast::new(RVec3::new(0.0, 0.0, 0.0), Vec3::ZERO),
        RayCast::new(
            RVec3::new(0.0, 0.0, 0.0),
            Vec3::new(f32::INFINITY, 0.0, 0.0),
        ),
    ];
    for ray in invalid {
        assert!(
            matches!(world.cast_ray(ray), Err(QueryError::InvalidValue(_))),
            "{ray:?} was accepted"
        );
    }
}
