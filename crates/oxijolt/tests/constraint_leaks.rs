//! A leak gate for world constraints: every kind is created, driven, read and removed, and a
//! pulley is moved by a rebase, over many rounds against a budget of 200 bytes per round. A
//! control run then creates Hermite paths of 16 points through `oxijolt-sys` and never
//! releases them, and must exceed the budget, which shows the gate can see such a leak.
//!
//! It measures the private bytes of the process, because Jolt's constraints are allocated by
//! C++, which a Rust global allocator does not see. The file holds exactly one test, so its
//! binary runs alone and no parallel test disturbs the counter.
#![cfg(windows)]

mod common;

use common::memory::private_bytes;
use common::*;
use oxijolt::*;
use oxijolt_sys::*;

const WARM_UP_ROUNDS: usize = 1_000;
const MEASURED_ROUNDS: usize = 10_000;
/// 200 bytes per measured round.
const MAX_GROWTH: usize = 200 * MEASURED_ROUNDS;

const X: Vec3 = Vec3::new(1.0, 0.0, 0.0);
const Y: Vec3 = Vec3::new(0.0, 1.0, 0.0);
const Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);

/// A static anchor and one dynamic box per constraint, 3 m apart along x.
struct Scene {
    world: PhysicsWorld,
    anchor: BodyId,
    boxes: Vec<BodyId>,
    path: HermitePath,
}

fn scene() -> Scene {
    let mut world = world(Vec3::ZERO, 1);
    let shape = Shape::new_box(Vec3::new(0.2, 0.2, 0.2)).unwrap();
    let anchor = world
        .create_body(
            &shape,
            &BodySettings::new_static().position(RVec3::new(0.0, -10.0, 0.0)),
        )
        .unwrap();
    let boxes = (0..14)
        .map(|i| {
            world
                .create_body(
                    &shape,
                    &BodySettings::new_dynamic().position(RVec3::new(3.0 * i as Real, 0.0, 0.0)),
                )
                .unwrap()
        })
        .collect();
    let point = |x: f32| HermitePathPoint {
        position: Vec3::new(x, 0.0, 0.0),
        tangent: X,
    };
    let path = HermitePath::new(Z, vec![point(0.0), point(1.0), point(2.0)], false).unwrap();
    Scene {
        world,
        anchor,
        boxes,
        path,
    }
}

fn constraint_round(scene: &mut Scene, round: usize) {
    let (anchor, b) = (scene.anchor, scene.boxes.clone());
    let mut ids: Vec<AnyConstraintId> = Vec::new();
    let world = &mut scene.world;
    let p = |world: &PhysicsWorld, i: usize| world.body(b[i]).unwrap().position();

    let fixed = FixedConstraintSettings::default().auto_detect_point();
    ids.push(
        world
            .create_constraint(anchor, b[0], &fixed)
            .unwrap()
            .into(),
    );
    let point = PointConstraintSettings::new(p(world, 1));
    ids.push(
        world
            .create_constraint(anchor, b[1], &point)
            .unwrap()
            .into(),
    );
    let rod = DistanceConstraintSettings::new(p(world, 2), p(world, 3));
    let rod = world.create_constraint(b[2], b[3], &rod).unwrap();
    world
        .constraint_mut(rod)
        .unwrap()
        .set_distance(0.0, 5.0)
        .unwrap();
    ids.push(rod.into());

    let hinge = |world: &mut PhysicsWorld, i: usize| {
        world
            .create_constraint(
                anchor,
                b[i],
                &HingeConstraintSettings::new(p(world, i), Z, X),
            )
            .unwrap()
    };
    let (hinge1, hinge2) = (hinge(world, 4), hinge(world, 5));
    let mut motor = world.constraint_mut(hinge1).unwrap();
    motor.set_target_angular_velocity(1.0).unwrap();
    motor.set_motor_state(MotorState::Velocity);
    let gear = GearConstraintSettings::new(Z, Z, 2.0).hinges(hinge1, hinge2);
    let gear = world.create_constraint(b[4], b[5], &gear).unwrap();

    let slider = SliderConstraintSettings::new(p(world, 6), X, Y);
    let slider = world.create_constraint(anchor, b[6], &slider).unwrap();
    let mut motor = world.constraint_mut(slider).unwrap();
    motor.set_target_velocity(0.1).unwrap();
    motor.set_motor_state(MotorState::Velocity);
    let pinion = hinge(world, 7);
    let rack = RackAndPinionConstraintSettings::new(Z, X, 4.0).constraints(pinion, slider);
    let rack = world.create_constraint(b[7], b[6], &rack).unwrap();

    let cone = ConeConstraintSettings::new(p(world, 8), X, 0.5);
    ids.push(world.create_constraint(anchor, b[8], &cone).unwrap().into());
    let swing = SwingTwistConstraintSettings::new(p(world, 9), X, Y).half_cone_angles(0.5, 0.5);
    let swing = world.create_constraint(anchor, b[9], &swing).unwrap();
    let mut motor = world.constraint_mut(swing).unwrap();
    motor.set_target_orientation_cs(quat_about(Z, 0.2)).unwrap();
    motor.set_swing_motor_state(MotorState::Position);
    ids.push(swing.into());
    let six = SixDofConstraintSettings::new(p(world, 10), X, Y);
    let six = world.create_constraint(anchor, b[10], &six).unwrap();
    let mut motor = world.constraint_mut(six).unwrap();
    motor
        .set_target_velocity_cs(Vec3::new(0.1, 0.0, 0.0))
        .unwrap();
    motor.set_motor_state(SixDofConstraintAxis::TranslationX, MotorState::Velocity);
    ids.push(six.into());

    let (a, c) = (p(world, 11), p(world, 12));
    let pulley = PulleyConstraintSettings::new(
        a,
        RVec3::new(a.x, a.y + 2.0, a.z),
        c,
        RVec3::new(c.x, c.y + 2.0, c.z),
    );
    let pulley = world.create_constraint(b[11], b[12], &pulley).unwrap();
    // The path starts at the box, so the constraint holds it near its centre of mass.
    let (start, origin) = (p(world, 13), world.body(anchor).unwrap().position());
    // `Real` is `f32` without the `double-precision` feature, so the casts are no-ops there.
    #[allow(clippy::unnecessary_cast)]
    let offset = Vec3::new(
        (start.x - origin.x) as f32,
        (start.y - origin.y) as f32,
        (start.z - origin.z) as f32,
    );
    let path = PathConstraintSettings::new(scene.path.clone()).path_position(offset);
    let path = world.create_constraint(anchor, b[13], &path).unwrap();
    let mut motor = world.constraint_mut(path).unwrap();
    motor.set_target_path_fraction(1.0).unwrap();
    motor.set_motor_state(MotorState::Position);

    assert!(world.step(DT).unwrap().is_complete());

    // Readings, then a rebase that moves the pulley.
    assert!(world
        .constraint(hinge1)
        .unwrap()
        .current_angle()
        .is_finite());
    assert!(world
        .constraint(slider)
        .unwrap()
        .current_position()
        .is_finite());
    assert!(world.constraint(pulley).unwrap().current_length() > 0.0);
    assert!(world.constraint(path).unwrap().path_fraction() >= 0.0);
    let reading = world.constraint(path).unwrap();
    assert!(reading.closest_fraction(Vec3::ZERO, 0.0).is_ok());
    let mut all = vec![anchor];
    all.extend(&b);
    let shift = if round.is_multiple_of(2) { 1.0 } else { -1.0 };
    world
        .rebase(&all, Quat::IDENTITY, RVec3::new(shift, 0.0, 0.0))
        .unwrap();
    assert!(world.step(DT).unwrap().is_complete());

    // Couplings first: they hold their references.
    world.remove_constraint(gear).unwrap();
    world.remove_constraint(rack).unwrap();
    for id in [hinge1, hinge2, pinion] {
        world.remove_constraint(id).unwrap();
    }
    world.remove_constraint(slider).unwrap();
    world.remove_constraint(pulley).unwrap();
    world.remove_constraint(path).unwrap();
    for id in ids {
        world.remove_constraint(id).unwrap();
    }
    // Put the boxes back where they started, so every round runs the same scene.
    for (i, &id) in b.iter().enumerate() {
        let mut body = world.body_mut(id).unwrap();
        body.set_position_and_rotation(
            RVec3::new(3.0 * i as Real, 0.0, 0.0),
            Quat::IDENTITY,
            Activation::DontActivate,
        )
        .unwrap();
        body.set_linear_velocity(Vec3::ZERO).unwrap();
        body.set_angular_velocity(Vec3::ZERO).unwrap();
    }
}

/// Creates `count` Hermite paths of 16 points and never releases them.
///
/// # Safety
/// Jolt is initialised.
unsafe fn leak_paths(count: usize) {
    for _ in 0..count {
        // SAFETY: Jolt is initialised (function contract); the vectors are live locals. The
        // paths are deliberately never released.
        unsafe {
            let path = JPH_PathConstraintPathHermite_Create();
            assert!(!path.is_null());
            for i in 0..16 {
                let position = JPH_Vec3 {
                    x: i as f32,
                    y: 0.0,
                    z: 0.0,
                };
                let tangent = JPH_Vec3 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                };
                let normal = JPH_Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 1.0,
                };
                JPH_PathConstraintPathHermite_AddPoint(path, &position, &tangent, &normal);
            }
        }
    }
}

#[test]
fn constraints_do_not_leak() {
    let mut scene = scene();
    for round in 0..WARM_UP_ROUNDS {
        constraint_round(&mut scene, round);
    }
    let before = private_bytes();
    for round in 0..MEASURED_ROUNDS {
        constraint_round(&mut scene, round);
    }
    let after = private_bytes();
    let growth = after.saturating_sub(before);
    eprintln!("constraints: private bytes before {before}, after {after}, growth {growth}");
    assert_eq!(scene.world.constraint_count(), 0);
    assert!(
        growth < MAX_GROWTH,
        "private bytes grew by {growth} (before {before}, after {after}): a constraint leaks"
    );

    // The control: as many paths, never released, must show.
    let before = private_bytes();
    // SAFETY: Jolt is initialised (the world above exists).
    unsafe { leak_paths(MEASURED_ROUNDS) };
    let after = private_bytes();
    let growth = after.saturating_sub(before);
    eprintln!("control: private bytes before {before}, after {after}, growth {growth}");
    assert!(
        growth >= MAX_GROWTH,
        "the control leaked {MEASURED_ROUNDS} paths but private bytes grew only by {growth}"
    );
}
