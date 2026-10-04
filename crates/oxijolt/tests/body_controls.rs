//! Momentary inputs to bodies: impulses and kinematic moves.

mod common;

use common::controls::*;
use common::events::add_cloth;
use common::*;
use oxijolt::*;

const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);
const Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);

#[test]
fn impulses_change_velocity_at_once_and_wake() {
    let mut world = world(Vec3::ZERO, 1);
    // 2 kg: Jolt's inverse mass 0.5 is exact, so the velocity change is exact too.
    let cube = add_sleeping(&mut world, &cube_shape(), 2.0, RVec3::ZERO);
    assert!(world.body(cube).unwrap().is_sleeping());
    let mut body = world.body_mut(cube).unwrap();
    body.add_impulse(Vec3::new(3.0, -1.0, 0.5)).unwrap();
    assert!(body.is_active());
    assert_eq!(body.linear_velocity(), Vec3::new(1.5, -0.5, 0.25));
    body.add_impulse(Vec3::new(1.0, 1.0, -0.5)).unwrap();
    assert_eq!(body.linear_velocity(), Vec3::new(2.0, 0.0, 0.0));

    // A zero impulse is accepted and still wakes a sleeping body, as in Jolt.
    let other = add_sleeping(&mut world, &cube_shape(), 2.0, RVec3::new(5.0, 0.0, 0.0));
    let mut body = world.body_mut(other).unwrap();
    body.add_impulse(Vec3::ZERO).unwrap();
    body.add_angular_impulse(Vec3::ZERO).unwrap();
    assert!(body.is_active());
    assert_eq!(body.linear_velocity(), Vec3::ZERO);
    step(&mut world, 1);
    assert!(world.body(cube).unwrap().position().x > 0.0);
}

#[test]
fn impulse_at_point_spins_the_body() {
    let mut world = world(Vec3::ZERO, 1);
    // A sphere of 2.5 kg and radius 1 m has an inertia of 1 kg·m² about every axis.
    let ball = add_sleeping(
        &mut world,
        &Shape::new_sphere(1.0).unwrap(),
        2.5,
        RVec3::ZERO,
    );
    let mut body = world.body_mut(ball).unwrap();
    body.add_impulse_at_point(Vec3::new(0.0, 1.0, 0.0), RVec3::new(1.0, 0.0, 0.0))
        .unwrap();
    assert!(body.is_active());
    assert_eq!(body.linear_velocity(), Vec3::new(0.0, 0.4, 0.0));
    let w = body.angular_velocity();
    assert!(w.x.abs() < 1.0e-6 && w.y.abs() < 1.0e-6, "{w:?}");
    assert!((w.z - 1.0).abs() < 1.0e-5, "{w:?}");
}

#[test]
fn impulses_are_bounded_by_the_velocity_change_they_give() {
    let mut world = world(Vec3::ZERO, 1);
    for (i, mass) in [limits::MIN_MASS, 2.0, limits::MAX_MASS]
        .into_iter()
        .enumerate()
    {
        let id = add_sleeping(
            &mut world,
            &cube_shape(),
            mass,
            RVec3::new(0.0, 0.0, 10.0 * i as Real),
        );
        // Jolt's inverse mass of a body whose mass is overridden.
        let inverse_mass = f64::from(1.0 / mass);
        let bound = f64::from(limits::MAX_VELOCITY_CHANGE);
        let largest = largest_accepted(f32::MAX, |j| f64::from(j) * inverse_mass <= bound);
        let mut body = world.body_mut(id).unwrap();
        let before = (body.linear_velocity(), body.is_active());
        assert!(invalid(body.add_impulse(Vec3::new(
            0.0,
            largest.next_up(),
            0.0
        ))));
        assert!(invalid(body.add_impulse_at_point(
            Vec3::new(0.0, -largest.next_up(), 0.0),
            body.position()
        )));
        for value in [f32::NAN, f32::INFINITY] {
            assert!(invalid(body.add_impulse(Vec3::new(value, 0.0, 0.0))));
        }
        assert_eq!((body.linear_velocity(), body.is_active()), before);
        body.add_impulse(Vec3::new(0.0, largest, 0.0)).unwrap();
        // Jolt clamps the new velocity to the speed bound.
        let speed = length(body.linear_velocity());
        assert!(
            (speed - limits::MAX_LINEAR_VELOCITY).abs() < 1.0e-3,
            "{speed}"
        );
        body.add_impulse(Vec3::new(0.0, -largest, 0.0)).unwrap();
        assert!(body.linear_velocity().y < -499.9);
    }
    step(&mut world, 60);
}

#[test]
fn angular_impulses_are_bounded_by_the_angular_velocity_change() {
    let mut world = world(Vec3::ZERO, 1);
    // A 1 g cube of 6 cm: its inertia is just above Jolt's near-zero threshold, which gives the
    // largest inverse inertia a body can have.
    let tiny = Shape::new_box(Vec3::new(0.03, 0.03, 0.03)).unwrap();
    let moment = limits::MIN_MASS * (2.0 * 0.06 * 0.06) / 12.0;
    let rotations = [
        Quat::IDENTITY,
        quat_about(Vec3::new(0.6, 0.0, 0.8), 0.7),
        quat_about(Vec3::new(0.0, 0.28, 0.96), 2.1),
    ];
    let directions = [
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 0.6, 0.8),
        Vec3::new(0.48, 0.6, -0.64),
    ];
    let mut ids = Vec::new();
    for (i, (&rotation, &direction)) in rotations.iter().zip(&directions).enumerate() {
        let id = world
            .create_body(
                &tiny,
                &BodySettings::new_dynamic()
                    .position(RVec3::new(i as Real, 0.0, 0.0))
                    .rotation(rotation)
                    .mass(limits::MIN_MASS)
                    .angular_damping(0.0),
            )
            .unwrap();
        ids.push(id);
        let mut body = world.body_mut(id).unwrap();
        let kick = |body: &mut BodyMut<'_>, size: f32| {
            body.add_angular_impulse(Vec3::new(
                direction.x * size,
                direction.y * size,
                direction.z * size,
            ))
        };
        let largest = largest_accepted(1.0, |size| kick(&mut body, size).is_ok());
        // The bound is the angular velocity change over the largest inverse inertia.
        let expected = limits::MAX_ANGULAR_VELOCITY_CHANGE * moment;
        assert!(
            (largest / expected - 1.0).abs() < 1.0e-3,
            "{largest} {expected}"
        );
        let before = body.angular_velocity();
        assert!(invalid(kick(&mut body, largest.next_up())));
        assert_eq!(body.angular_velocity(), before);
        let point = body.position();
        let lever = RVec3::new(point.x, point.y + 0.03, point.z);
        assert!(invalid(body.add_impulse_at_point(
            Vec3::new(largest.next_up() * 2.0 / 0.03, 0.0, 0.0),
            lever
        )));
        kick(&mut body, largest).unwrap();
        // Jolt clamps the spin to the bound; an overflow would show as an exact zero.
        let spin = length(body.angular_velocity());
        assert!(
            (spin - limits::MAX_ANGULAR_VELOCITY).abs() < 1.0e-3,
            "{spin}"
        );
    }
    step(&mut world, 60);
    for id in ids {
        assert_finite(&world, id);
        let spin = length(world.body(id).unwrap().angular_velocity());
        assert!(spin > 40.0, "{spin}");
    }
}

#[test]
fn point_impulses_are_bounded_by_the_angular_impulse_of_their_lever() {
    let mut world = world(Vec3::ZERO, 1);
    let ball = add_sleeping(
        &mut world,
        &Shape::new_sphere(1.0).unwrap(),
        2.5,
        RVec3::ZERO,
    );
    let mut body = world.body_mut(ball).unwrap();
    // 10 N·s at the centre is accepted; at 50 m from it, it is a 500 N·m·s angular impulse on an
    // inertia of 1 kg·m², beyond the bound.
    let impulse = Vec3::new(0.0, 10.0, 0.0);
    assert!(invalid(
        body.add_impulse_at_point(impulse, RVec3::new(50.0, 0.0, 0.0))
    ));
    let outside = RVec3::new(limits::MAX_POSITION.next_up(), 0.0, 0.0);
    assert!(invalid(body.add_impulse_at_point(impulse, outside)));
    assert!(body.is_sleeping());
    body.add_impulse_at_point(impulse, RVec3::ZERO).unwrap();
    assert_eq!(body.angular_velocity(), Vec3::ZERO);
    body.add_impulse_at_point(impulse, RVec3::new(2.0, 0.0, 0.0))
        .unwrap();
    assert!((body.angular_velocity().z - 20.0).abs() < 1.0e-3);
    step(&mut world, 30);
    assert_finite(&world, ball);
}

#[test]
fn impulses_respect_locked_axes() {
    let mut world = world(Vec3::ZERO, 1);
    let plane = add_locked_cube(&mut world, RVec3::ZERO, AllowedDofs::PLANE_2D);
    let mut body = world.body_mut(plane).unwrap();
    body.add_impulse(Vec3::new(0.0, 0.0, 50.0)).unwrap();
    body.add_angular_impulse(Vec3::new(5.0, 5.0, 0.0)).unwrap();
    assert_eq!(body.linear_velocity(), Vec3::ZERO);
    assert_eq!(body.angular_velocity(), Vec3::ZERO);
    body.add_impulse_at_point(Vec3::new(0.0, 0.0, 10.0), RVec3::new(0.5, 0.5, 0.0))
        .unwrap();
    let (v, w) = (body.linear_velocity(), body.angular_velocity());
    assert_eq!((v.z, w.x, w.y), (0.0, 0.0, 0.0));
    step(&mut world, 10);
    assert_eq!(world.body(plane).unwrap().position(), RVec3::ZERO);
}

#[test]
fn impulses_ignore_static_and_kinematic_bodies_and_refuse_soft_bodies() {
    let mut world = world(Vec3::ZERO, 1);
    let fixed = world
        .create_body(&cube_shape(), &BodySettings::new_static())
        .unwrap();
    let platform = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_kinematic().position(RVec3::new(3.0, 0.0, 0.0)),
        )
        .unwrap();
    for id in [fixed, platform] {
        let mut body = world.body_mut(id).unwrap();
        let huge = Vec3::new(1.0e30, 0.0, 0.0);
        body.add_impulse(huge).unwrap();
        body.add_angular_impulse(huge).unwrap();
        body.add_impulse_at_point(huge, RVec3::new(0.0, 4.0, 0.0))
            .unwrap();
        assert!(invalid(body.add_impulse(Vec3::new(f32::NAN, 0.0, 0.0))));
        assert_eq!(body.linear_velocity(), Vec3::ZERO);
        assert_eq!(body.angular_velocity(), Vec3::ZERO);
    }
    let cloth = add_cloth(&mut world, RVec3::new(0.0, 5.0, 0.0), Quat::IDENTITY);
    let mut body = world.body_mut(cloth).unwrap();
    assert_eq!(
        body.add_impulse(Vec3::ZERO),
        Err(BodyError::SoftBody(cloth))
    );
    assert_eq!(
        body.add_angular_impulse(Vec3::ZERO),
        Err(BodyError::SoftBody(cloth))
    );
    assert_eq!(
        body.add_impulse_at_point(Vec3::ZERO, RVec3::ZERO),
        Err(BodyError::SoftBody(cloth))
    );
    step(&mut world, 5);
}

#[test]
fn a_kinematic_platform_moves_toward_its_target() {
    let mut world = world(GRAVITY, 1);
    let block = Shape::new_box(Vec3::new(1.0, 0.2, 1.0)).unwrap();
    let off_centre = Shape::new_offset_center_of_mass(&block, Vec3::new(0.3, 0.0, -0.2)).unwrap();
    for (i, shape) in [&block, &off_centre].into_iter().enumerate() {
        let start = RVec3::new(10.0 * i as Real, 0.0, 0.0);
        let platform = add_kinematic(&mut world, shape, start, Activation::Activate);
        let target = RVec3::new(start.x + 0.5, 0.2, -0.1);
        let turn = quat_about(Vec3::new(0.0, 1.0, 0.0), 0.03);
        world
            .body_mut(platform)
            .unwrap()
            .move_kinematic(target, turn, DT)
            .unwrap();
        step(&mut world, 1);
        let body = world.body(platform).unwrap();
        assert!(distance(body.position(), target) < 1.0e-5, "{i}");
        // Jolt's small-angle angular velocity turns by 2 sin(θ/2) instead of θ.
        let error = angle_between(body.rotation(), turn);
        assert!(error < 2.0e-6, "{i}: {error}");
    }
}

#[test]
fn a_kinematic_move_keeps_its_velocity_after_the_step() {
    let mut world = world(Vec3::ZERO, 1);
    let platform = add_kinematic(&mut world, &cube_shape(), RVec3::ZERO, Activation::Activate);
    world
        .body_mut(platform)
        .unwrap()
        .move_kinematic(RVec3::new(0.1, 0.0, 0.0), Quat::IDENTITY, DT)
        .unwrap();
    let velocity = world.body(platform).unwrap().linear_velocity();
    step(&mut world, 2);
    let body = world.body(platform).unwrap();
    assert_eq!(body.linear_velocity(), velocity);
    assert!(distance(body.position(), RVec3::new(0.2, 0.0, 0.0)) < 1.0e-5);
}

#[test]
fn a_tiny_move_does_not_wake_a_sleeping_kinematic_body() {
    let mut world = world(Vec3::ZERO, 1);
    let platform = add_kinematic(
        &mut world,
        &cube_shape(),
        RVec3::ZERO,
        Activation::DontActivate,
    );
    let mut body = world.body_mut(platform).unwrap();
    // 1e-9 m in 1/60 s: a squared speed of 3.6e-15 m²/s², below Jolt's 1e-12.
    body.move_kinematic(RVec3::new(1.0e-9, 0.0, 0.0), Quat::IDENTITY, DT)
        .unwrap();
    assert!(body.is_sleeping());
    step(&mut world, 1);
    assert_eq!(world.body(platform).unwrap().position(), RVec3::ZERO);
    let mut body = world.body_mut(platform).unwrap();
    body.move_kinematic(RVec3::new(1.0e-3, 0.0, 0.0), Quat::IDENTITY, DT)
        .unwrap();
    assert!(body.is_active());
}

#[test]
fn kinematic_moves_respect_locked_axes() {
    let mut world = world(Vec3::ZERO, 1);
    let rail = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_kinematic()
                .allowed_dofs(AllowedDofs::TRANSLATION_X | AllowedDofs::ROTATION_Y),
        )
        .unwrap();
    let turn = quat_about(Vec3::new(0.6, 0.8, 0.0), 0.2);
    world
        .body_mut(rail)
        .unwrap()
        .move_kinematic(RVec3::new(1.0, 2.0, 3.0), turn, DT)
        .unwrap();
    let body = world.body(rail).unwrap();
    let (v, w) = (body.linear_velocity(), body.angular_velocity());
    assert_eq!((v.y, v.z, w.x, w.z), (0.0, 0.0, 0.0, 0.0));
    assert!(v.x > 59.0 && w.y > 0.0, "{v:?} {w:?}");
    step(&mut world, 1);
    let p = world.body(rail).unwrap().position();
    assert_eq!((p.y, p.z), (0.0, 0.0));
}

#[test]
fn a_kinematic_platform_carries_a_resting_cube() {
    let mut world = world(GRAVITY, 1);
    let deck = Shape::new_box(Vec3::new(2.0, 0.2, 2.0)).unwrap();
    let platform = add_kinematic(&mut world, &deck, RVec3::ZERO, Activation::Activate);
    let cube = add_cube(&mut world, RVec3::new(0.0, 0.7, 0.0));
    step(&mut world, 30);
    for tick in 1..=60 {
        let target = RVec3::new(tick as Real / 60.0, 0.0, 0.0);
        world
            .body_mut(platform)
            .unwrap()
            .move_kinematic(target, Quat::IDENTITY, DT)
            .unwrap();
        step(&mut world, 1);
    }
    let platform_x = world.body(platform).unwrap().position().x;
    let cube_x = world.body(cube).unwrap().position().x;
    assert!((platform_x - 1.0).abs() < 1.0e-4, "{platform_x}");
    // Friction accelerates the cube to the platform's 1 m/s within about half a second.
    assert!(cube_x > 0.7, "{cube_x}");
    let cube_speed = world.body(cube).unwrap().linear_velocity().x;
    assert!((cube_speed - 1.0).abs() < 0.05, "{cube_speed}");
}

#[test]
fn kinematic_moves_are_bounded_by_the_velocities_they_imply() {
    let mut world = world(Vec3::ZERO, 1);
    // dt = 1/64: a move of 7.8125 m is exactly 500 m/s.
    let dt = 1.0 / 64.0;
    let platform = add_kinematic(&mut world, &cube_shape(), RVec3::ZERO, Activation::Activate);
    let along = |distance: f32| RVec3::new(Real::from(distance), 0.0, 0.0);
    let mut body = world.body_mut(platform).unwrap();
    assert!(invalid(body.move_kinematic(
        along(7.8125_f32.next_up()),
        Quat::IDENTITY,
        dt
    )));
    body.move_kinematic(along(7.8125), Quat::IDENTITY, dt)
        .unwrap();
    assert_eq!(body.linear_velocity(), Vec3::new(500.0, 0.0, 0.0));
    assert!(world.step(dt).unwrap().is_complete());
    assert_eq!(world.body(platform).unwrap().position(), along(7.8125));

    // The largest turn about z the bound accepts, found on the angle's bits.
    let spinner = add_kinematic(
        &mut world,
        &cube_shape(),
        RVec3::new(0.0, 5.0, 0.0),
        Activation::Activate,
    );
    let mut body = world.body_mut(spinner).unwrap();
    let here = body.position();
    let about_z = |angle: f32| quat_about(Z, angle);
    let largest = largest_accepted(1.5, |angle| {
        body.move_kinematic(here, about_z(angle), dt).is_ok()
    });
    assert!(
        (largest - limits::MAX_ANGULAR_VELOCITY * dt).abs() < 1.0e-3,
        "{largest}"
    );
    assert!(invalid(body.move_kinematic(
        here,
        about_z(largest.next_up()),
        dt
    )));
    body.move_kinematic(here, about_z(largest), dt).unwrap();
    let spin = length(body.angular_velocity());
    assert!(
        spin <= limits::MAX_ANGULAR_VELOCITY * (1.0 + 1.0e-6),
        "{spin}"
    );
    assert!(world.step(dt).unwrap().is_complete());
    assert_finite(&world, spinner);

    // Invalid inputs change nothing.
    let mut body = world.body_mut(platform).unwrap();
    let before = (body.linear_velocity(), body.angular_velocity());
    let far = RVec3::new(limits::MAX_POSITION.next_up(), 0.0, 0.0);
    assert!(invalid(body.move_kinematic(far, Quat::IDENTITY, dt)));
    let skewed = Quat::from_xyzw(0.0, 0.0, 0.0, 2.0);
    assert!(invalid(body.move_kinematic(along(8.0), skewed, dt)));
    for bad_dt in [0.0, -dt, f32::NAN, PhysicsWorld::MAX_DELTA_TIME * 2.0] {
        assert!(invalid(body.move_kinematic(
            along(8.0),
            Quat::IDENTITY,
            bad_dt
        )));
    }
    assert_eq!((body.linear_velocity(), body.angular_velocity()), before);

    let ball = add_cube(&mut world, RVec3::new(0.0, -5.0, 0.0));
    let fixed = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_static().position(RVec3::new(0.0, -10.0, 0.0)),
        )
        .unwrap();
    for id in [ball, fixed] {
        assert_eq!(
            world
                .body_mut(id)
                .unwrap()
                .move_kinematic(RVec3::ZERO, Quat::IDENTITY, dt),
            Err(BodyError::NotKinematic(id))
        );
    }
    let cloth = add_cloth(&mut world, RVec3::new(0.0, 20.0, 0.0), Quat::IDENTITY);
    assert_eq!(
        world
            .body_mut(cloth)
            .unwrap()
            .move_kinematic(RVec3::ZERO, Quat::IDENTITY, dt),
        Err(BodyError::SoftBody(cloth))
    );
    step(&mut world, 1);
}
