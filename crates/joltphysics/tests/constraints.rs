//! World constraints: each kind's defining behaviour (limits hold, motors reach their targets,
//! springs oscillate and damp), typed ids, and the guards that keep constraint bodies alive.

mod common;

use std::f32::consts::{FRAC_PI_2, PI};

use common::ragdoll::{humanoid_settings, ragdoll_world};
use common::*;
use joltphysics::*;

const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);
const X: Vec3 = Vec3::new(1.0, 0.0, 0.0);
const Y: Vec3 = Vec3::new(0.0, 1.0, 0.0);
const Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);

/// A small static box at `position`, the world end of a constraint.
fn add_anchor(world: &mut PhysicsWorld, position: RVec3) -> BodyId {
    let shape = Shape::new_box(Vec3::new(0.1, 0.1, 0.1)).unwrap();
    world
        .create_body(&shape, &BodySettings::new_static().position(position))
        .unwrap()
}

/// A dynamic box of half extents `half` at `position`.
fn add_box(world: &mut PhysicsWorld, half: Vec3, position: RVec3) -> BodyId {
    let shape = Shape::new_box(half).unwrap();
    world
        .create_body(&shape, &BodySettings::new_dynamic().position(position))
        .unwrap()
}

/// `value` as `f64`, which it already is with the `double-precision` feature.
#[allow(clippy::useless_conversion)]
fn wide(value: Real) -> f64 {
    f64::from(value)
}

fn distance(a: RVec3, b: RVec3) -> f64 {
    let d = [a.x - b.x, a.y - b.y, a.z - b.z].map(f64::from);
    (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
}

/// `p + rotation * offset`.
fn attached(position: RVec3, rotation: Quat, offset: Vec3) -> RVec3 {
    let r = common::ragdoll::rotate(rotation, offset);
    RVec3::new(
        position.x + r.x as Real,
        position.y + r.y as Real,
        position.z + r.z as Real,
    )
}

/// Number of sign changes in `values`, ignoring exact zeros.
fn sign_changes(values: &[f64]) -> usize {
    let signs: Vec<bool> = values
        .iter()
        .filter(|v| **v != 0.0)
        .map(|v| *v > 0.0)
        .collect();
    signs.windows(2).filter(|w| w[0] != w[1]).count()
}

/// The largest magnitude in each half of `values`.
fn half_peaks(values: &[f64]) -> (f64, f64) {
    let (first, last) = values.split_at(values.len() / 2);
    let peak = |v: &[f64]| v.iter().fold(0.0_f64, |m, x| m.max(x.abs()));
    (peak(first), peak(last))
}

#[test]
fn fixed_constraint_holds_a_box_under_a_static_bar() {
    let mut world = world(GRAVITY, 1);
    let bar = add_anchor(&mut world, RVec3::new(0.0, 5.0, 0.0));
    let start = RVec3::new(0.0, 4.0, 0.0);
    let cube = add_cube(&mut world, start);
    world
        .create_constraint(
            bar,
            cube,
            &FixedConstraintSettings::default().auto_detect_point(),
        )
        .unwrap();
    let mut worst: f64 = 0.0;
    let mut worst_turn: f32 = 0.0;
    for _ in 0..120 {
        step(&mut world, 1);
        let body = world.body(cube).unwrap();
        worst = worst.max(distance(body.position(), start));
        worst_turn = worst_turn.max(common::ragdoll::angle_between(
            body.rotation(),
            Quat::IDENTITY,
        ));
    }
    // Measured: 0 m and 0 rad.
    assert!(worst < 1e-2, "moved {worst} m");
    assert!(worst_turn < 1e-2, "turned {worst_turn} rad");
}

#[test]
fn point_constraint_keeps_the_pendulum_pivot() {
    let mut world = world(GRAVITY, 1);
    let pivot = RVec3::new(0.0, 5.0, 0.0);
    let anchor = add_anchor(&mut world, pivot);
    let bob = add_cube(&mut world, RVec3::new(1.5, 5.0, 0.0));
    world
        .create_constraint(anchor, bob, &PointConstraintSettings::new(pivot))
        .unwrap();
    let mut worst: f64 = 0.0;
    for _ in 0..120 {
        step(&mut world, 1);
        let body = world.body(bob).unwrap();
        let end = attached(body.position(), body.rotation(), Vec3::new(-1.5, 0.0, 0.0));
        worst = worst.max(distance(end, pivot));
    }
    // The bob swings down; the pivot stays put. Measured: 0.0041 m.
    assert!(world.body(bob).unwrap().position().y < 4.0);
    assert!(worst < 1e-2, "pivot drifted {worst} m");
}

#[test]
fn distance_constraint_keeps_a_rod_length() {
    let mut world = world(GRAVITY, 1);
    let top = RVec3::new(0.0, 5.0, 0.0);
    let anchor = add_anchor(&mut world, top);
    let start = RVec3::new(2.0, 5.0, 0.0);
    let ball = add_box(&mut world, Vec3::new(0.2, 0.2, 0.2), start);
    let rod = world
        .create_constraint(anchor, ball, &DistanceConstraintSettings::new(top, start))
        .unwrap();
    let reading = world.constraint(rod).unwrap();
    assert_eq!((reading.min_distance(), reading.max_distance()), (2.0, 2.0));
    let mut worst: f64 = 0.0;
    let mut lowest = start.y;
    for _ in 0..120 {
        step(&mut world, 1);
        let position = world.body(ball).unwrap().position();
        worst = worst.max((distance(position, top) - 2.0).abs());
        lowest = lowest.min(position.y);
    }
    // Measured: 0.0050 m, lowest point 2.996 m.
    assert!(lowest < 3.1, "the ball did not swing down: {lowest}");
    assert!(worst < 1e-2, "length off by {worst} m");
    assert!(world
        .constraint(rod)
        .unwrap()
        .total_lambda_position()
        .is_finite());
}

#[test]
fn soft_distance_spring_oscillates_and_damps() {
    let mut world = world(Vec3::ZERO, 1);
    let anchor = add_anchor(&mut world, RVec3::ZERO);
    let ball = add_box(
        &mut world,
        Vec3::new(0.2, 0.2, 0.2),
        RVec3::new(2.2, 0.0, 0.0),
    );
    let spring = SpringSettings::FrequencyAndDamping {
        frequency: 2.0,
        damping: 0.1,
    };
    world
        .create_constraint(
            anchor,
            ball,
            &DistanceConstraintSettings::new(RVec3::ZERO, RVec3::new(2.2, 0.0, 0.0))
                .range(DistanceRange::Range { min: 2.0, max: 2.0 })
                .limits_spring(spring),
        )
        .unwrap();
    let mut offsets = Vec::new();
    for _ in 0..240 {
        step(&mut world, 1);
        offsets.push(distance(world.body(ball).unwrap().position(), RVec3::ZERO) - 2.0);
    }
    let (first, last) = half_peaks(&offsets);
    // Measured: 5 sign changes, peaks 0.19 m then 0.0004 m.
    assert!(sign_changes(&offsets) >= 2, "{offsets:?}");
    assert!(last < 0.5 * first, "peaks {first} then {last}");
}

/// A door: a static post and a 1 m panel hinged on its left edge about `axis`.
fn door(
    world: &mut PhysicsWorld,
    hinge: HingeConstraintSettings,
) -> (BodyId, ConstraintId<HingeConstraint>) {
    // The post stands apart: Jolt lets the two bodies of a constraint collide.
    let post = add_anchor(world, RVec3::new(-2.0, 0.0, 0.0));
    let panel = add_box(world, Vec3::new(0.5, 0.05, 0.5), RVec3::new(0.6, 0.0, 0.0));
    let id = world.create_constraint(post, panel, &hinge).unwrap();
    (panel, id)
}

#[test]
fn hinge_limit_holds_a_falling_door() {
    let mut world = world(GRAVITY, 1);
    // A horizontal hinge along Z: gravity swings the panel down, -Z rotations, until the limit.
    let (_, hinge) = door(
        &mut world,
        HingeConstraintSettings::new(RVec3::new(0.1, 0.0, 0.0), Z, X).limits(-0.5, 0.5),
    );
    step(&mut world, 180);
    let angle = world.constraint(hinge).unwrap().current_angle();
    // At tick 180 the panel rests on the limit; measured: -0.5006 rad.
    assert!(angle < -0.45, "the door did not fall: {angle}");
    assert!(angle > -0.5 - 1e-2, "past the limit: {angle}");
}

#[test]
fn hinge_velocity_motor_reaches_its_target_speed() {
    let mut world = world(Vec3::ZERO, 1);
    let (panel, hinge) = door(
        &mut world,
        HingeConstraintSettings::new(RVec3::new(0.1, 0.0, 0.0), Y, X),
    );
    let mut motor = world.constraint_mut(hinge).unwrap();
    motor.set_target_angular_velocity(2.0).unwrap();
    motor.set_motor_state(MotorState::Velocity);
    step(&mut world, 60);
    let hinge_reading = world.constraint(hinge).unwrap();
    assert_eq!(hinge_reading.motor_state(), MotorState::Velocity);
    assert_eq!(hinge_reading.target_angular_velocity(), 2.0);
    let spin = world.body(panel).unwrap().angular_velocity().y;
    // Measured: 2.00001 rad/s.
    assert!((spin - 2.0).abs() < 0.04, "{spin} rad/s");
}

#[test]
fn hinge_position_motor_reaches_its_target_angle() {
    let mut world = world(Vec3::ZERO, 1);
    let (_, hinge) = door(
        &mut world,
        HingeConstraintSettings::new(RVec3::new(0.1, 0.0, 0.0), Y, X),
    );
    let mut motor = world.constraint_mut(hinge).unwrap();
    motor.set_target_angle(1.0).unwrap();
    motor.set_motor_state(MotorState::Position);
    step(&mut world, 180);
    let angle = world.constraint(hinge).unwrap().current_angle();
    // Measured: 0.9979 rad.
    assert!((angle - 1.0).abs() < 0.02, "{angle}");
    assert_eq!(world.constraint(hinge).unwrap().target_angle(), 1.0);
}

#[test]
fn bodies_of_constraints_cannot_be_removed() {
    let mut world = world(GRAVITY, 1);
    let anchor = add_anchor(&mut world, RVec3::new(0.0, 5.0, 0.0));
    let cube = add_cube(&mut world, RVec3::new(0.0, 3.0, 0.0));
    let rope = world
        .create_constraint(
            anchor,
            cube,
            &DistanceConstraintSettings::new(RVec3::new(0.0, 5.0, 0.0), RVec3::new(0.0, 3.0, 0.0)),
        )
        .unwrap();
    for body in [anchor, cube] {
        assert_eq!(
            world.remove_body(body),
            Err(BodyError::UsedByConstraint(body))
        );
        assert_eq!(world.constraints_of_body(body), vec![rope.into()]);
    }
    step(&mut world, 10);
    world.remove_constraint(rope).unwrap();
    assert!(world.constraints_of_body(cube).is_empty());
    world.remove_body(cube).unwrap();
    world.remove_body(anchor).unwrap();
    step(&mut world, 10);
}

#[test]
fn constraints_refuse_ragdoll_parts_inner_bodies_and_one_body() {
    let (mut world, layers) = ragdoll_world(1);
    let anchor = add_anchor(&mut world, RVec3::new(0.0, 5.0, 0.0));
    let point = PointConstraintSettings::new(RVec3::new(0.0, 4.0, 0.0));

    let ragdoll = world
        .create_ragdoll(
            &humanoid_settings(layers.ragdoll),
            None,
            Activation::Activate,
        )
        .unwrap();
    let part = world.ragdoll(ragdoll).unwrap().body_ids()[0];
    assert_eq!(
        world.create_constraint(anchor, part, &point),
        Err(ConstraintError::Body(BodyError::OwnedByRagdoll(part)))
    );

    let capsule = Shape::new_capsule(0.7, 0.4).unwrap();
    let character = world
        .create_character(
            &CharacterSettings::new(&capsule).inner_body(Some(InnerBody {
                shape: &capsule,
                object_layer: layers.ragdoll,
            })),
            RVec3::new(5.0, 1.0, 0.0),
            Quat::IDENTITY,
        )
        .unwrap();
    let inner = world.character(character).unwrap().inner_body().unwrap();
    assert_eq!(
        world.create_constraint(inner, anchor, &point),
        Err(ConstraintError::Body(BodyError::OwnedByCharacter(inner)))
    );

    assert!(matches!(
        world.create_constraint(anchor, anchor, &point),
        Err(ConstraintError::InvalidValue(_))
    ));
    let (mut other, _) = ragdoll_world(1);
    let foreign = add_anchor(&mut other, RVec3::ZERO);
    assert_eq!(
        world.create_constraint(anchor, foreign, &point),
        Err(ConstraintError::Body(BodyError::WrongWorld(foreign)))
    );
    assert_eq!(world.constraint_count(), 0);
}

#[test]
fn constraint_ids_are_typed_ordered_and_never_reused() {
    let mut world = world(GRAVITY, 1);
    let anchor = add_anchor(&mut world, RVec3::new(0.0, 5.0, 0.0));
    let cubes: Vec<BodyId> = (0..3)
        .map(|i| add_cube(&mut world, RVec3::new(2.0 * i as Real, 3.0, 0.0)))
        .collect();
    let point = |world: &mut PhysicsWorld, cube| {
        world
            .create_constraint(
                anchor,
                cube,
                &PointConstraintSettings::new(RVec3::new(0.0, 5.0, 0.0)),
            )
            .unwrap()
    };
    let first = point(&mut world, cubes[0]);
    let hinge = world
        .create_constraint(
            anchor,
            cubes[1],
            &HingeConstraintSettings::new(RVec3::new(0.0, 5.0, 0.0), Z, X),
        )
        .unwrap();
    let third = point(&mut world, cubes[2]);
    assert_eq!([first.to_raw(), hinge.to_raw(), third.to_raw()], [1, 2, 3]);
    assert!(first < third);
    world.remove_constraint(hinge).unwrap();
    let fourth = point(&mut world, cubes[1]);
    assert_eq!(fourth.to_raw(), 4);

    let ids: Vec<AnyConstraintId> = world.constraint_ids().collect();
    assert_eq!(ids, vec![first.into(), third.into(), fourth.into()]);
    assert_eq!(ids[0].kind(), ConstraintType::Point);
    assert_eq!(ids[0].downcast::<PointConstraint>(), Some(first));
    assert_eq!(ids[0].downcast::<HingeConstraint>(), None);
    assert_eq!(AnyConstraintId::from(hinge).kind(), ConstraintType::Hinge);
}

#[test]
fn wrong_world_and_removed_ids_are_rejected() {
    let mut world = world(GRAVITY, 1);
    let mut other = common::world(GRAVITY, 1);
    let make = |world: &mut PhysicsWorld| {
        let anchor = add_anchor(world, RVec3::new(0.0, 5.0, 0.0));
        let cube = add_cube(world, RVec3::new(0.0, 3.0, 0.0));
        world
            .create_constraint(
                anchor,
                cube,
                &HingeConstraintSettings::new(RVec3::new(0.0, 5.0, 0.0), Z, X),
            )
            .unwrap()
    };
    let ours = make(&mut world);
    let theirs = make(&mut other);
    assert_eq!(ours.to_raw(), theirs.to_raw());
    assert_ne!(ours, theirs);
    assert_eq!(
        world.constraint(theirs).err(),
        Some(ConstraintError::WrongWorld(theirs.into()))
    );
    assert_eq!(
        world.remove_constraint(theirs),
        Err(ConstraintError::WrongWorld(theirs.into()))
    );
    world.remove_constraint(ours).unwrap();
    assert_eq!(
        world.constraint_mut(ours).err().map(|e| e.to_string()),
        Some(ConstraintError::NotFound(ours.into()).to_string())
    );
    assert_eq!(
        world.remove_constraint(ours),
        Err(ConstraintError::NotFound(ours.into()))
    );
}

#[test]
fn dropping_a_world_with_constraints_is_clean() {
    for steps in [0, 30] {
        let mut world = world(GRAVITY, 1);
        let anchor = add_anchor(&mut world, RVec3::new(0.0, 5.0, 0.0));
        let a = add_cube(&mut world, RVec3::new(0.0, 3.0, 0.0));
        let b = add_cube(&mut world, RVec3::new(2.0, 3.0, 0.0));
        world
            .create_constraint(
                anchor,
                a,
                &PointConstraintSettings::new(RVec3::new(0.0, 5.0, 0.0)),
            )
            .unwrap();
        world
            .create_constraint(
                a,
                b,
                &FixedConstraintSettings::default().auto_detect_point(),
            )
            .unwrap();
        world
            .create_constraint(
                anchor,
                b,
                &HingeConstraintSettings::new(RVec3::new(0.0, 5.0, 0.0), Z, X),
            )
            .unwrap();
        step(&mut world, steps);
        drop(world);
    }
}

/// Asserts that `settings` are refused as invalid and leave the world's counts unchanged.
fn assert_creates_nothing<S: ConstraintSettings>(
    world: &mut PhysicsWorld,
    bodies: [BodyId; 2],
    settings: &S,
) {
    let body_count = world.body_count();
    let constraint_count = world.constraint_count();
    assert!(matches!(
        world.create_constraint(bodies[0], bodies[1], settings),
        Err(ConstraintError::InvalidValue(_))
    ));
    assert_eq!(world.body_count(), body_count);
    assert_eq!(world.constraint_count(), constraint_count);
}

#[test]
fn invalid_constraint_creates_nothing() {
    let mut world = world(GRAVITY, 1);
    let anchor = add_anchor(&mut world, RVec3::new(0.0, 5.0, 0.0));
    let cube = add_cube(&mut world, RVec3::new(0.0, 3.0, 0.0));
    let bodies = [anchor, cube];
    assert_creates_nothing(
        &mut world,
        bodies,
        &PointConstraintSettings::new(RVec3::new(Real::NAN, 0.0, 0.0)),
    );
    assert_creates_nothing(
        &mut world,
        bodies,
        &HingeConstraintSettings::new(RVec3::ZERO, Z, Z),
    );
    assert_creates_nothing(
        &mut world,
        bodies,
        &DistanceConstraintSettings::new(RVec3::ZERO, RVec3::ZERO)
            .range(DistanceRange::Range { min: 2.0, max: 1.0 }),
    );
    assert_creates_nothing(
        &mut world,
        bodies,
        &FixedConstraintSettings::default()
            .space(ConstraintSpace::LocalToBodyCom)
            .auto_detect_point(),
    );
    let valid = world
        .create_constraint(
            anchor,
            cube,
            &PointConstraintSettings::new(RVec3::new(0.0, 5.0, 0.0)),
        )
        .unwrap();
    assert_eq!(valid.to_raw(), 1);
}

#[test]
fn removing_a_constraint_wakes_its_bodies() {
    let mut world = world(GRAVITY, 1);
    let anchor = add_anchor(&mut world, RVec3::new(0.0, 5.0, 0.0));
    let cube = add_cube(&mut world, RVec3::new(0.0, 4.0, 0.0));
    let weld = world
        .create_constraint(
            anchor,
            cube,
            &FixedConstraintSettings::default().auto_detect_point(),
        )
        .unwrap();
    let asleep = (0..600).any(|_| {
        step(&mut world, 1);
        world.body(cube).unwrap().is_sleeping()
    });
    assert!(asleep, "the welded cube never fell asleep");
    world.remove_constraint(weld).unwrap();
    assert!(!world.body(cube).unwrap().is_sleeping());
    step(&mut world, 30);
    assert!(world.body(cube).unwrap().position().y < 3.5);
}

#[test]
fn hinge_setters_check_their_values() {
    let mut world = world(Vec3::ZERO, 1);
    let (_, hinge) = door(
        &mut world,
        HingeConstraintSettings::new(RVec3::new(0.1, 0.0, 0.0), Y, X),
    );
    let mut hinge = world.constraint_mut(hinge).unwrap();
    let invalid = |result: Result<(), ConstraintError>| {
        assert!(
            matches!(result, Err(ConstraintError::InvalidValue(_))),
            "{result:?}"
        );
    };
    invalid(hinge.set_target_angle(PI + 0.01));
    invalid(hinge.set_target_angle(f32::NAN));
    hinge.set_target_angle(-PI).unwrap();
    invalid(hinge.set_limits(0.1, 0.5));
    invalid(hinge.set_limits(0.0, 0.0));
    hinge.set_limits(-FRAC_PI_2, FRAC_PI_2).unwrap();
    invalid(hinge.set_max_friction_torque(-1.0));
    hinge.set_max_friction_torque(2.0).unwrap();
    invalid(hinge.set_motor_settings(MotorSettings::default().torque_limits(1.0, -1.0)));
    hinge
        .set_motor_settings(MotorSettings::default().torque_limits(-10.0, 10.0))
        .unwrap();
    let soft = SpringSettings::FrequencyAndDamping {
        frequency: 5.0,
        damping: 0.5,
    };
    hinge.set_limits_spring(soft).unwrap();
    hinge.set_limits(0.0, 0.0).unwrap();
    // Limits with min == max keep needing a soft spring.
    invalid(hinge.set_limits_spring(SpringSettings::default()));
    hinge.set_enabled(false);
    assert!(!hinge.is_enabled());
    let id = hinge.id();
    let reading = world.constraint(id).unwrap();
    assert_eq!(reading.limits(), (0.0, 0.0));
    assert_eq!(reading.max_friction_torque(), 2.0);
}

/// A static anchor far from `body`'s path and a dynamic 0.4 m box at the origin.
fn anchored_box(world: &mut PhysicsWorld) -> (BodyId, BodyId) {
    let anchor = add_anchor(world, RVec3::new(0.0, 10.0, 10.0));
    let body = add_box(world, Vec3::new(0.2, 0.2, 0.2), RVec3::ZERO);
    (anchor, body)
}

/// Twist about X and swing angles about Y and Z of a constraint-space rotation, with Jolt's
/// swing-twist decomposition (`Quat::GetSwingTwist`): twist first, swing angles as twice the
/// half angles `atan2(swing.y, swing.w)` and `atan2(swing.z, swing.w)`.
fn twist_and_swing(q: Quat) -> (f32, f32, f32) {
    let s = (q.w * q.w + q.x * q.x).sqrt();
    let twist = Quat::from_xyzw(q.x / s, 0.0, 0.0, q.w / s);
    let swing = common::ragdoll::mul(q, common::ragdoll::conj(twist));
    let half = |v: f32, w: f32| 2.0 * v.atan2(w);
    (
        half(twist.x, twist.w),
        half(swing.y, swing.w),
        half(swing.z, swing.w),
    )
}

#[test]
fn slider_limit_holds() {
    let mut world = world(GRAVITY, 1);
    let (anchor, body) = anchored_box(&mut world);
    let slider = world
        .create_constraint(
            anchor,
            body,
            &SliderConstraintSettings::new(RVec3::ZERO, Y, X).limits(-1.0, 0.0),
        )
        .unwrap();
    step(&mut world, 120);
    let position = world.constraint(slider).unwrap().current_position();
    let body = world.body(body).unwrap();
    // Measured: -1.0000 m; the box stays on the axis.
    assert!((position + 1.0).abs() < 1e-2, "{position}");
    assert!(body.position().x.abs() < 1e-3 && body.position().z.abs() < 1e-3);
    assert_eq!(
        world.constraint(slider).unwrap().limits(),
        Some((-1.0, 0.0))
    );
}

#[test]
fn slider_position_motor_reaches_its_target() {
    let mut world = world(Vec3::ZERO, 1);
    let (anchor, body) = anchored_box(&mut world);
    let slider = world
        .create_constraint(
            anchor,
            body,
            &SliderConstraintSettings::new(RVec3::ZERO, X, Y),
        )
        .unwrap();
    let mut motor = world.constraint_mut(slider).unwrap();
    motor.set_target_position(0.5).unwrap();
    motor.set_motor_state(MotorState::Position);
    step(&mut world, 180);
    let position = world.constraint(slider).unwrap().current_position();
    // Measured: 0.4999 m.
    assert!((position - 0.5).abs() < 1e-2, "{position}");
    let reading = world.constraint(slider).unwrap();
    assert_eq!(reading.motor_state(), MotorState::Position);
    assert_eq!(reading.target_position(), 0.5);
}

#[test]
fn slider_soft_limit_spring_oscillates_and_damps() {
    let mut world = world(Vec3::ZERO, 1);
    let (anchor, body) = anchored_box(&mut world);
    let spring = SpringSettings::FrequencyAndDamping {
        frequency: 2.0,
        damping: 0.1,
    };
    let slider = world
        .create_constraint(
            anchor,
            body,
            &SliderConstraintSettings::new(RVec3::ZERO, X, Y)
                .limits(0.0, 0.0)
                .limits_spring(spring),
        )
        .unwrap();
    world
        .body_mut(body)
        .unwrap()
        .set_linear_velocity(X)
        .unwrap();
    let mut offsets = Vec::new();
    for _ in 0..240 {
        step(&mut world, 1);
        offsets.push(f64::from(
            world.constraint(slider).unwrap().current_position(),
        ));
    }
    let (first, last) = half_peaks(&offsets);
    // Measured: 3 sign changes, peaks 0.059 m then 0.0005 m.
    assert!(sign_changes(&offsets) >= 2, "{offsets:?}");
    assert!(last < 0.5 * first, "peaks {first} then {last}");
}

#[test]
fn cone_holds_a_hanging_body_within_its_angle() {
    // Gravity tilted by atan(5 / 9.81) = 0.47 rad would hang the body outside the 0.3 rad cone,
    // so it comes to rest on the cone's edge.
    let mut world = world(Vec3::new(5.0, -9.81, 0.0), 1);
    let anchor = add_anchor(&mut world, RVec3::new(0.0, 10.0, 10.0));
    let body = add_box(
        &mut world,
        Vec3::new(0.1, 0.5, 0.1),
        RVec3::new(0.0, -0.5, 0.0),
    );
    let down = Vec3::new(0.0, -1.0, 0.0);
    let cone = world
        .create_constraint(
            anchor,
            body,
            &ConeConstraintSettings::new(RVec3::ZERO, down, 0.3),
        )
        .unwrap();
    let swing = |world: &PhysicsWorld| {
        let axis = common::ragdoll::rotate(world.body(body).unwrap().rotation(), down);
        (-axis.y).clamp(-1.0, 1.0).acos()
    };
    step(&mut world, 180);
    let settled = swing(&world);
    let reading = world.constraint(cone).unwrap();
    assert!((reading.half_cone_angle() - 0.3).abs() < 1e-5);
    // Measured at tick 180: 0.3037 rad; on the way the swing into the cone overshot to 0.3098.
    assert!((settled - 0.3).abs() < 1e-2, "{settled} rad");
}

#[test]
fn swing_twist_twist_limit_holds() {
    let mut world = world(Vec3::ZERO, 1);
    let (anchor, body) = anchored_box(&mut world);
    let joint = world
        .create_constraint(
            anchor,
            body,
            &SwingTwistConstraintSettings::new(RVec3::ZERO, X, Y)
                .half_cone_angles(0.5, 0.5)
                .twist_limits(-0.3, 0.3),
        )
        .unwrap();
    // A velocity motor keeps twisting the body into the limit.
    let mut motor = world.constraint_mut(joint).unwrap();
    motor
        .set_target_angular_velocity_cs(Vec3::new(2.0, 0.0, 0.0))
        .unwrap();
    motor.set_twist_motor_state(MotorState::Velocity);
    step(&mut world, 120);
    let reading = world.constraint(joint).unwrap();
    let twist = twist_and_swing(reading.rotation_in_constraint_space()).0;
    assert_eq!(reading.twist_limits(), (-0.3, 0.3));
    // Measured at tick 120: 0.3000 rad.
    assert!((twist - 0.3).abs() < 1e-2, "{twist} rad");
}

#[test]
fn swing_twist_motor_reaches_target_orientation_with_a_rotated_parent() {
    let mut world = world(Vec3::ZERO, 1);
    let turn = quat_about(Vec3::new(0.48, 0.6, 0.64), 0.9);
    let shape = Shape::new_box(Vec3::new(0.2, 0.2, 0.2)).unwrap();
    let dynamic = |position| {
        BodySettings::new_dynamic()
            .position(position)
            .rotation(turn)
    };
    let parent = world.create_body(&shape, &dynamic(RVec3::ZERO)).unwrap();
    let child = world
        .create_body(&shape, &dynamic(RVec3::new(1.0, 0.0, 0.0)))
        .unwrap();
    // Twist axis X and plane axis Z make the constraint frame the world axes (the normal axis
    // is plane × twist = Y). It is given in world space, so in both bodies' frames it is turn⁻¹.
    let joint = world
        .create_constraint(
            parent,
            child,
            &SwingTwistConstraintSettings::new(RVec3::new(0.5, 0.0, 0.0), X, Z)
                .half_cone_angles(1.0, 1.0)
                .twist_limits(-1.0, 1.0),
        )
        .unwrap();
    let target = common::ragdoll::mul(quat_about(Z, 0.5), quat_about(X, 0.4));
    let mut motor = world.constraint_mut(joint).unwrap();
    motor.set_target_orientation_cs(target).unwrap();
    motor.set_swing_motor_state(MotorState::Position);
    motor.set_twist_motor_state(MotorState::Position);
    step(&mut world, 240);

    let rotation = |id| world.body(id).unwrap().rotation();
    let relative = common::ragdoll::mul(common::ragdoll::conj(rotation(parent)), rotation(child));
    let wanted = common::ragdoll::mul(
        common::ragdoll::mul(common::ragdoll::conj(turn), target),
        turn,
    );
    let error = common::ragdoll::angle_between(relative, wanted);
    let reading = world.constraint(joint).unwrap();
    let in_constraint_space =
        common::ragdoll::angle_between(reading.rotation_in_constraint_space(), target);
    // Measured: 0.0012 rad for both; the relative rotation is 0.26 rad from the target itself.
    assert!(error < 2e-2, "{error} rad");
    assert!(in_constraint_space < 2e-2, "{in_constraint_space} rad");
    assert_eq!(reading.swing_motor_state(), MotorState::Position);
}

/// A six-DOF joint at the origin with every translation fixed except as `settings` say.
fn six_dof(
    world: &mut PhysicsWorld,
    settings: SixDofConstraintSettings,
) -> (BodyId, ConstraintId<SixDofConstraint>) {
    let (anchor, body) = anchored_box(world);
    let id = world.create_constraint(anchor, body, &settings).unwrap();
    (body, id)
}

fn six_dof_settings() -> SixDofConstraintSettings {
    let mut settings = SixDofConstraintSettings::new(RVec3::ZERO, X, Y);
    for axis in [
        SixDofConstraintAxis::TranslationX,
        SixDofConstraintAxis::TranslationY,
        SixDofConstraintAxis::TranslationZ,
    ] {
        settings = settings.axis(axis, SixDofAxis::Fixed);
    }
    settings
}

#[test]
fn six_dof_translation_limit_holds() {
    let mut world = world(GRAVITY, 1);
    let (body, joint) = six_dof(
        &mut world,
        six_dof_settings().axis(
            SixDofConstraintAxis::TranslationY,
            SixDofAxis::Limited {
                min: -0.5,
                max: 0.0,
            },
        ),
    );
    step(&mut world, 120);
    let y = world.body(body).unwrap().position().y;
    // Measured: -0.5000 m.
    assert!((y + 0.5).abs() < 1e-2, "{y}");
    assert_eq!(
        world
            .constraint(joint)
            .unwrap()
            .limits(SixDofConstraintAxis::TranslationY),
        Some((-0.5, 0.0))
    );
}

#[test]
fn six_dof_translation_spring_oscillates_and_damps() {
    let mut world = world(Vec3::ZERO, 1);
    let y = SixDofConstraintAxis::TranslationY;
    let spring = SpringSettings::FrequencyAndDamping {
        frequency: 2.0,
        damping: 0.1,
    };
    let (body, _) = six_dof(
        &mut world,
        six_dof_settings()
            .axis(
                y,
                SixDofAxis::Limited {
                    min: -0.01,
                    max: 0.01,
                },
            )
            .limits_spring(y, spring),
    );
    world
        .body_mut(body)
        .unwrap()
        .set_linear_velocity(Y)
        .unwrap();
    let mut offsets = Vec::new();
    for _ in 0..240 {
        step(&mut world, 1);
        offsets.push(wide(world.body(body).unwrap().position().y));
    }
    let (first, last) = half_peaks(&offsets);
    // Measured: 5 sign changes, peaks 0.071 m then 0.0037 m.
    assert!(sign_changes(&offsets) >= 2, "{offsets:?}");
    assert!(last < 0.5 * first, "peaks {first} then {last}");
}

#[test]
fn six_dof_rotation_position_motor_reaches_its_target() {
    let mut world = world(Vec3::ZERO, 1);
    let mut settings = six_dof_settings();
    for axis in SIX_DOF_ROTATIONS {
        settings = settings.motor(axis, MotorSettings::default());
    }
    let (_, joint) = six_dof(&mut world, settings);
    let target = common::ragdoll::mul(quat_about(Y, 0.6), quat_about(X, -0.3));
    let mut motor = world.constraint_mut(joint).unwrap();
    motor.set_target_orientation_cs(target).unwrap();
    for axis in SIX_DOF_ROTATIONS {
        motor.set_motor_state(axis, MotorState::Position);
    }
    step(&mut world, 240);
    let reading = world.constraint(joint).unwrap();
    let error = common::ragdoll::angle_between(reading.rotation_in_constraint_space(), target);
    // Measured: 0.0012 rad.
    assert!(error < 2e-2, "{error} rad");
    assert_eq!(
        reading.motor_state(SixDofConstraintAxis::RotationY),
        MotorState::Position
    );
}

const SIX_DOF_ROTATIONS: [SixDofConstraintAxis; 3] = [
    SixDofConstraintAxis::RotationX,
    SixDofConstraintAxis::RotationY,
    SixDofConstraintAxis::RotationZ,
];

#[test]
fn six_dof_pyramid_holds_asymmetric_limits() {
    let limited = |min, max| SixDofAxis::Limited { min, max };
    // A velocity motor keeps turning the body into one swing limit, then the other.
    for (spin, bound) in [(1.0, 0.6), (-1.0, -0.2)] {
        let mut world = world(Vec3::ZERO, 1);
        let y = SixDofConstraintAxis::RotationY;
        let (_, joint) = six_dof(
            &mut world,
            six_dof_settings()
                .swing_type(SwingType::Pyramid)
                .axis(SixDofConstraintAxis::RotationX, SixDofAxis::Fixed)
                .axis(y, limited(-0.2, 0.6))
                .axis(SixDofConstraintAxis::RotationZ, limited(-0.1, 0.1)),
        );
        let mut motor = world.constraint_mut(joint).unwrap();
        motor
            .set_target_angular_velocity_cs(Vec3::new(0.0, spin, 0.0))
            .unwrap();
        motor.set_motor_state(y, MotorState::Velocity);
        step(&mut world, 120);
        let rotation = world
            .constraint(joint)
            .unwrap()
            .rotation_in_constraint_space();
        let (_, swing_y, swing_z) = twist_and_swing(rotation);
        // Measured at tick 120: 0.6000 and -0.2000 rad, swing about z 0.
        assert!((swing_y - bound).abs() < 1e-2, "spin {spin}: {swing_y} rad");
        assert!(swing_z.abs() < 1e-2, "{swing_z} rad");
    }
}
