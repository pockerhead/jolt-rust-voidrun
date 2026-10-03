//! The magnitude policy of `joltphysics::limits`: every bounded input is accepted at its bound
//! and rejected just beyond it without changing the world, and scenes with every input at its
//! bound step with finite state. In a native build with the `asserts` feature these scenes also
//! show that no Jolt assertion fires for them.

mod common;

use std::f32::consts::PI;

use common::vehicle::*;
use common::walker::{f3, v3};
use common::*;
use joltphysics::*;

/// A world without gravity.
fn empty_world() -> PhysicsWorld {
    world(Vec3::ZERO, 1)
}

fn sphere() -> Shape {
    Shape::new_sphere(0.5).unwrap()
}

/// The axis-aligned vectors with `value` on one axis, in both directions.
fn on_axes(value: f32) -> Vec<Vec3> {
    let mut vectors = Vec::new();
    for axis in 0..3 {
        for sign in [1.0, -1.0] {
            let mut v = [0.0; 3];
            v[axis] = sign * value;
            vectors.push(Vec3::from(v));
        }
    }
    vectors
}

/// The axis-aligned positions with `value` on one axis, in both directions.
fn real_on_axes(value: Real) -> Vec<RVec3> {
    let mut vectors = Vec::new();
    for axis in 0..3 {
        for sign in [1.0, -1.0] {
            let mut v = [0.0; 3];
            v[axis] = sign * value;
            vectors.push(RVec3::from(v));
        }
    }
    vectors
}

const NON_FINITE: [f32; 3] = [f32::NAN, f32::INFINITY, f32::NEG_INFINITY];

fn bits(v: Vec3) -> [u32; 3] {
    <[f32; 3]>::from(v).map(f32::to_bits)
}

/// `value` as `f64`, which it already is with the `double-precision` feature.
#[allow(clippy::useless_conversion)]
fn wide(value: Real) -> f64 {
    f64::from(value)
}

fn real_bits(v: RVec3) -> [u64; 3] {
    <[Real; 3]>::from(v).map(|c| wide(c).to_bits())
}

/// A body's pose and velocities as bits.
fn body_bits(world: &PhysicsWorld, id: BodyId) -> ([u64; 3], [u32; 4], [u32; 3], [u32; 3]) {
    let body = world.body(id).unwrap();
    (
        real_bits(body.position()),
        <[f32; 4]>::from(body.rotation()).map(f32::to_bits),
        bits(body.linear_velocity()),
        bits(body.angular_velocity()),
    )
}

fn assert_body_finite(world: &PhysicsWorld, id: BodyId, what: &str) {
    let body = world.body(id).unwrap();
    let state = [
        v3(body.position()),
        f3(body.linear_velocity()),
        f3(body.angular_velocity()),
    ];
    assert!(
        state.iter().flatten().all(|value| value.is_finite()),
        "{what}: {state:?}"
    );
}

fn body_invalid(result: Result<impl Sized, BodyError>) -> bool {
    matches!(result, Err(BodyError::InvalidValue(_)))
}

fn query_invalid(result: Result<impl Sized, QueryError>) -> bool {
    matches!(result, Err(QueryError::InvalidValue(_)))
}

#[test]
fn world_gravity_is_bounded_by_max_acceleration() {
    let bound = limits::MAX_ACCELERATION;
    for gravity in on_axes(bound) {
        assert!(PhysicsWorld::new(WorldSettings::default().gravity(gravity)).is_ok());
    }
    let mut world = empty_world();
    for gravity in on_axes(bound) {
        world.set_gravity(gravity).unwrap();
    }
    let before = bits(world.gravity());
    let beyond = on_axes(bound.next_up()).into_iter().chain(
        NON_FINITE
            .into_iter()
            .map(|value| Vec3::new(0.0, value, 0.0)),
    );
    for gravity in beyond {
        assert!(matches!(
            PhysicsWorld::new(WorldSettings::default().gravity(gravity)),
            Err(WorldError::InvalidSettings(_))
        ));
        assert!(matches!(
            world.set_gravity(gravity),
            Err(WorldError::InvalidSettings(_))
        ));
        assert_eq!(bits(world.gravity()), before);
    }
}

#[test]
fn rebase_translation_is_bounded_by_twice_the_frame() {
    let mut world = empty_world();
    let id = world
        .create_body(&sphere(), &BodySettings::new_dynamic())
        .unwrap();
    let span = 2.0 * limits::MAX_POSITION;
    // The translation is bounded; the resulting position is only checked to be finite.
    world
        .rebase(&[id], Quat::IDENTITY, RVec3::new(span, 0.0, 0.0))
        .unwrap();
    let before = body_bits(&world, id);
    for translation in real_on_axes(span.next_up()) {
        assert!(body_invalid(world.rebase(
            &[id],
            Quat::IDENTITY,
            translation
        )));
        assert_eq!(body_bits(&world, id), before);
    }
}

#[test]
fn body_settings_are_bounded() {
    let mut world = empty_world();
    let shape = sphere();
    let dynamic = BodySettings::new_dynamic;
    let mut accepted = Vec::new();
    let mut rejected = Vec::new();
    for p in real_on_axes(limits::MAX_POSITION) {
        accepted.push(dynamic().position(p));
    }
    for p in real_on_axes(limits::MAX_POSITION.next_up()) {
        rejected.push(dynamic().position(p));
    }
    for v in on_axes(limits::MAX_LINEAR_VELOCITY) {
        accepted.push(dynamic().linear_velocity(v));
    }
    for v in on_axes(limits::MAX_LINEAR_VELOCITY.next_up()) {
        rejected.push(dynamic().linear_velocity(v));
    }
    for v in on_axes(limits::MAX_ANGULAR_VELOCITY) {
        accepted.push(dynamic().angular_velocity(v));
    }
    for v in on_axes(limits::MAX_ANGULAR_VELOCITY.next_up()) {
        rejected.push(dynamic().angular_velocity(v));
    }
    for restitution in [0.0, 1.0] {
        accepted.push(dynamic().restitution(restitution));
    }
    for restitution in [-0.1, 1.0f32.next_up(), f32::NAN] {
        rejected.push(dynamic().restitution(restitution));
    }
    for friction in [0.0, limits::MAX_FRICTION] {
        accepted.push(dynamic().friction(friction));
    }
    for friction in [-0.1, limits::MAX_FRICTION.next_up(), f32::MAX, f32::NAN] {
        rejected.push(dynamic().friction(friction));
    }
    let factor = limits::MAX_GRAVITY_FACTOR;
    for gravity_factor in [-factor, factor] {
        accepted.push(dynamic().gravity_factor(gravity_factor));
    }
    for gravity_factor in [(-factor).next_down(), factor.next_up(), f32::NAN] {
        rejected.push(dynamic().gravity_factor(gravity_factor));
    }
    for mass in [limits::MIN_MASS, limits::MAX_MASS] {
        accepted.push(dynamic().mass(mass));
    }
    for mass in [limits::MIN_MASS.next_down(), limits::MAX_MASS.next_up()] {
        rejected.push(dynamic().mass(mass));
    }
    for settings in &accepted {
        world.create_body(&shape, settings).unwrap();
    }
    let count = world.body_count();
    for settings in &rejected {
        assert!(
            body_invalid(world.create_body(&shape, settings)),
            "{settings:?}"
        );
    }
    assert_eq!(world.body_count(), count);
}

#[test]
fn computed_dynamic_mass_is_bounded_and_kinematic_mass_is_not() {
    let mut world = empty_world();
    // Jolt's density 1000 kg/m³: a 10 m cube weighs 1e6 kg, a 1 cm cube 1e-3 kg.
    let heavy = Shape::new_box(Vec3::new(6.0, 6.0, 6.0)).unwrap();
    let light = Shape::new_box_with_convex_radius(Vec3::new(0.004, 0.004, 0.004), 0.0).unwrap();
    for shape in [&heavy, &light] {
        assert!(body_invalid(
            world.create_body(shape, &BodySettings::new_dynamic())
        ));
        world
            .create_body(shape, &BodySettings::new_kinematic())
            .unwrap();
        world
            .create_body(shape, &BodySettings::new_dynamic().mass(1.0))
            .unwrap();
    }
}

#[test]
fn body_setters_are_bounded_and_rejection_changes_nothing() {
    let mut world = empty_world();
    let id = world
        .create_body(&sphere(), &BodySettings::new_dynamic())
        .unwrap();
    let mut body = world.body_mut(id).unwrap();
    for p in real_on_axes(limits::MAX_POSITION) {
        body.set_position(p, Activation::DontActivate).unwrap();
        body.set_position_and_rotation(p, Quat::IDENTITY, Activation::DontActivate)
            .unwrap();
    }
    for v in on_axes(limits::MAX_LINEAR_VELOCITY) {
        body.set_linear_velocity(v).unwrap();
    }
    for v in on_axes(limits::MAX_ANGULAR_VELOCITY) {
        body.set_angular_velocity(v).unwrap();
    }
    let before = body_bits(&world, id);
    let mut body = world.body_mut(id).unwrap();
    for p in real_on_axes(limits::MAX_POSITION.next_up()) {
        assert!(body_invalid(body.set_position(p, Activation::DontActivate)));
        assert!(body_invalid(body.set_position_and_rotation(
            p,
            Quat::IDENTITY,
            Activation::DontActivate
        )));
    }
    for v in on_axes(limits::MAX_LINEAR_VELOCITY.next_up()) {
        assert!(body_invalid(body.set_linear_velocity(v)));
    }
    for v in on_axes(limits::MAX_ANGULAR_VELOCITY.next_up()) {
        assert!(body_invalid(body.set_angular_velocity(v)));
    }
    assert_eq!(body_bits(&world, id), before);
}

/// The f32 vector along `direction` whose length by Jolt's own `Vec3::Length` is the largest
/// at most `bound`.
fn on_the_jolt_bound(direction: Vec3, bound: f32) -> Vec3 {
    let jolt_length = |v: Vec3| {
        let raw = joltphysics_sys::JPH_Vec3 {
            x: v.x,
            y: v.y,
            z: v.z,
        };
        // SAFETY: a pure function of a live local.
        unsafe { joltphysics_sys::JPH_Vec3_Length(&raw) }
    };
    let length = jolt_length(direction);
    let mut v = Vec3::new(
        direction.x * bound / length,
        direction.y * bound / length,
        direction.z * bound / length,
    );
    let largest = |v: Vec3| {
        if v.x.abs() >= v.y.abs() && v.x.abs() >= v.z.abs() {
            0
        } else if v.y.abs() >= v.z.abs() {
            1
        } else {
            2
        }
    };
    let axis = largest(v);
    let step = |v: Vec3, up: bool| {
        let mut c = <[f32; 3]>::from(v);
        let grow = (c[axis] >= 0.0) == up;
        c[axis] = if grow {
            c[axis].next_up()
        } else {
            c[axis].next_down()
        };
        Vec3::from(c)
    };
    while jolt_length(v) > bound {
        v = step(v, false);
    }
    while jolt_length(step(v, true)) <= bound {
        v = step(v, true);
    }
    v
}

#[test]
fn creation_velocities_agree_with_jolts_length_in_many_directions() {
    let mut world = empty_world();
    let shape = sphere();
    let mut seed: u64 = 0x2545_f491_4f6c_dd1d;
    let mut next = || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        (seed >> 40) as f32 / (1u64 << 24) as f32 * 2.0 - 1.0
    };
    for _ in 0..64 {
        let direction = Vec3::new(next(), next(), next());
        if direction.x == 0.0 && direction.y == 0.0 && direction.z == 0.0 {
            continue;
        }
        for (bound, angular) in [
            (limits::MAX_LINEAR_VELOCITY, false),
            (limits::MAX_ANGULAR_VELOCITY, true),
        ] {
            let at = on_the_jolt_bound(direction, bound);
            let mut beyond = <[f32; 3]>::from(at);
            let axis = (0..3)
                .max_by(|&a, &b| beyond[a].abs().total_cmp(&beyond[b].abs()))
                .unwrap();
            beyond[axis] = if beyond[axis] >= 0.0 {
                beyond[axis].next_up()
            } else {
                beyond[axis].next_down()
            };
            let settings = |v: Vec3| {
                if angular {
                    BodySettings::new_dynamic().angular_velocity(v)
                } else {
                    BodySettings::new_dynamic().linear_velocity(v)
                }
            };
            // With the `asserts` feature Jolt asserts `Length() <= max` on this creation.
            world.create_body(&shape, &settings(at)).unwrap();
            assert!(body_invalid(
                world.create_body(&shape, &settings(Vec3::from(beyond)))
            ));
        }
    }
}

#[test]
fn forces_are_bounded_by_the_acceleration_they_give() {
    let mut world = empty_world();
    // Mass 2 kg: Jolt's inverse mass 0.5 is exact.
    let id = world
        .create_body(&sphere(), &BodySettings::new_dynamic().mass(2.0))
        .unwrap();
    let half = limits::MAX_ACCELERATION;
    let mut body = world.body_mut(id).unwrap();
    // 2 * MAX_ACCELERATION newtons in total give exactly MAX_ACCELERATION.
    body.add_force(Vec3::new(half, 0.0, 0.0)).unwrap();
    body.add_force(Vec3::new(half, 0.0, 0.0)).unwrap();
    assert!(body_invalid(body.add_force(Vec3::new(1.0e3, 0.0, 0.0))));
    body.reset_forces();
    assert!(body_invalid(body.add_force(Vec3::new(
        0.0,
        -(2.0 * half).next_up(),
        0.0
    ))));
    body.add_force(Vec3::new(0.0, -2.0 * half, 0.0)).unwrap();
    for value in NON_FINITE {
        assert!(body_invalid(body.add_force(Vec3::new(value, 0.0, 0.0))));
    }
    assert!(world.step(DT).unwrap().is_complete());
    assert_body_finite(&world, id, "after the bound force");

    // Static and kinematic bodies ignore loads.
    for settings in [BodySettings::new_static(), BodySettings::new_kinematic()] {
        let other = world.create_body(&sphere(), &settings).unwrap();
        let mut body = world.body_mut(other).unwrap();
        body.add_force(Vec3::new(1.0e30, 0.0, 0.0)).unwrap();
        body.add_torque(Vec3::new(1.0e30, 0.0, 0.0)).unwrap();
    }
}

#[test]
fn torques_are_bounded_by_the_angular_acceleration_they_give() {
    let mut world = empty_world();
    // A sphere of mass m and radius r has the inertia 0.4 * m * r² about every axis: 1 kg·m² for
    // 2.5 kg and 1 m, within Jolt's rounding.
    let shape = Shape::new_sphere(1.0).unwrap();
    let id = world
        .create_body(&shape, &BodySettings::new_dynamic().mass(2.5))
        .unwrap();
    let bound = limits::MAX_ANGULAR_ACCELERATION;
    let mut body = world.body_mut(id).unwrap();
    body.add_torque(Vec3::new(0.0, 0.0, bound * 0.999)).unwrap();
    assert!(body_invalid(body.add_torque(Vec3::new(
        0.0,
        0.0,
        bound * 0.002
    ))));
    body.reset_forces();
    assert!(body_invalid(body.add_torque(Vec3::new(
        bound * 1.001,
        0.0,
        0.0
    ))));
    // A force of F newtons at 1 m from the centre adds F newton-metres.
    let point = RVec3::new(1.0, 0.0, 0.0);
    body.add_force_at_point(Vec3::new(0.0, bound * 0.999, 0.0), point)
        .unwrap();
    assert!(body_invalid(
        body.add_force_at_point(Vec3::new(0.0, bound * 0.002, 0.0), point)
    ));
    let corner = RVec3::new(limits::MAX_POSITION.next_up(), 0.0, 0.0);
    assert!(body_invalid(
        body.add_force_at_point(Vec3::new(0.0, 1.0, 0.0), corner)
    ));
    assert!(world.step(DT).unwrap().is_complete());
    assert_body_finite(&world, id, "after the bound torque");
}

#[test]
fn character_settings_and_setters_are_bounded() {
    let mut world = empty_world();
    let capsule = Shape::new_capsule(0.5, 0.3).unwrap();
    let base = || CharacterSettings::new(&capsule);
    let extent = limits::MAX_SHAPE_EXTENT;
    let accepted = [
        base().mass(0.0),
        base().mass(limits::MAX_MASS),
        base().shape_offset(Vec3::new(0.0, extent, 0.0)),
        base().predictive_contact_distance(extent),
        base().character_padding(extent),
        base().collision_tolerance(extent),
    ];
    for settings in &accepted {
        world
            .create_character(settings, RVec3::ZERO, Quat::IDENTITY)
            .unwrap();
    }
    let beyond = extent.next_up();
    let rejected = [
        base().mass(limits::MAX_MASS.next_up()),
        base().shape_offset(Vec3::new(0.0, beyond, 0.0)),
        base().predictive_contact_distance(beyond),
        base().character_padding(beyond),
        base().collision_tolerance(beyond),
    ];
    for settings in &rejected {
        assert!(matches!(
            world.create_character(settings, RVec3::ZERO, Quat::IDENTITY),
            Err(CharacterError::InvalidValue(_))
        ));
    }

    let corner = RVec3::new(limits::MAX_POSITION, limits::MAX_POSITION, 0.0);
    let id = world
        .create_character(&base(), corner, Quat::IDENTITY)
        .unwrap();
    assert!(matches!(
        world.create_character(
            &base(),
            RVec3::new(limits::MAX_POSITION.next_up(), 0.0, 0.0),
            Quat::IDENTITY
        ),
        Err(CharacterError::InvalidValue(_))
    ));
    let mut character = world.character_mut(id).unwrap();
    for p in real_on_axes(limits::MAX_POSITION) {
        character.set_position(p).unwrap();
    }
    for v in on_axes(limits::MAX_LINEAR_VELOCITY) {
        character.set_linear_velocity(v).unwrap();
    }
    for p in real_on_axes(limits::MAX_POSITION.next_up()) {
        assert!(matches!(
            character.set_position(p),
            Err(CharacterError::InvalidValue(_))
        ));
    }
    for v in on_axes(limits::MAX_LINEAR_VELOCITY.next_up()) {
        assert!(matches!(
            character.set_linear_velocity(v),
            Err(CharacterError::InvalidValue(_))
        ));
    }
    let reached = world.character(id).unwrap();
    assert_eq!(
        real_bits(reached.position()),
        real_bits(RVec3::new(0.0, 0.0, -limits::MAX_POSITION))
    );
    assert_eq!(
        bits(reached.linear_velocity()),
        bits(Vec3::new(0.0, 0.0, -limits::MAX_LINEAR_VELOCITY))
    );
}

#[test]
fn character_update_gravity_and_steps_are_bounded() {
    let mut world = empty_world();
    let capsule = Shape::new_capsule(0.5, 0.3).unwrap();
    let id = world
        .create_character(
            &CharacterSettings::new(&capsule),
            RVec3::ZERO,
            Quat::IDENTITY,
        )
        .unwrap();
    let all = QueryFilter::new();
    let defaults = ExtendedUpdateSettings::default();
    let update = |world: &mut PhysicsWorld, gravity: Vec3, settings: &ExtendedUpdateSettings| {
        world.update_character(id, DT, gravity, settings, &all)
    };
    for gravity in on_axes(limits::MAX_ACCELERATION) {
        update(&mut world, gravity, &defaults).unwrap();
    }
    for gravity in on_axes(limits::MAX_ACCELERATION.next_up()) {
        assert!(matches!(
            update(&mut world, gravity, &defaults),
            Err(CharacterError::InvalidValue(_))
        ));
    }
    let extent = limits::MAX_SHAPE_EXTENT;
    let gravity = Vec3::new(0.0, -9.81, 0.0);
    let down = Vec3::new(0.0, -extent, 0.0);
    let accepted = [
        defaults.stick_to_floor_step_down(down),
        defaults.walk_stairs_step_up(Vec3::new(0.0, extent, 0.0)),
        defaults.walk_stairs_step_down_extra(down),
        defaults.walk_stairs_min_step_forward(extent),
        defaults.walk_stairs_step_forward_test(extent),
    ];
    for settings in &accepted {
        update(&mut world, gravity, settings).unwrap();
    }
    let beyond = extent.next_up();
    let rejected = [
        defaults.stick_to_floor_step_down(Vec3::new(0.0, -beyond, 0.0)),
        defaults.walk_stairs_step_up(Vec3::new(0.0, beyond, 0.0)),
        defaults.walk_stairs_step_down_extra(Vec3::new(beyond, 0.0, 0.0)),
        defaults.walk_stairs_min_step_forward(beyond),
        defaults.walk_stairs_step_forward_test(beyond),
    ];
    for settings in &rejected {
        assert!(matches!(
            update(&mut world, gravity, settings),
            Err(CharacterError::InvalidValue(_))
        ));
    }
}

#[test]
fn constraint_frame_points_are_bounded() {
    let (_, layers) = common::ragdoll::ragdoll_layers();
    let skeleton = Skeleton::new(&[
        SkeletonJoint {
            name: "a",
            parent: None,
        },
        SkeletonJoint {
            name: "b",
            parent: Some(0),
        },
    ])
    .unwrap();
    let shape = sphere();
    let build = |anchor: RVec3| {
        let joint = HingeConstraintSettings::new(
            anchor,
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        );
        let body = BodySettings::new_dynamic().object_layer(layers.ragdoll);
        let parts = [
            RagdollPart {
                shape: &shape,
                body: body.clone(),
                joint: None,
            },
            RagdollPart {
                shape: &shape,
                body,
                joint: Some(RagdollJoint::Hinge(joint)),
            },
        ];
        RagdollSettings::new(&skeleton, &parts)
    };
    for anchor in real_on_axes(limits::MAX_POSITION) {
        assert!(build(anchor).is_ok());
    }
    for anchor in real_on_axes(limits::MAX_POSITION.next_up()) {
        assert!(matches!(build(anchor), Err(RagdollError::InvalidValue(_))));
    }
}

#[test]
fn six_dof_translation_limits_at_the_bound_step_finitely() {
    // Two parts at the mass extremes, joined by a six-DOF joint whose translation limit along x
    // excludes where they start, so Jolt corrects them by about the extent bound each step.
    let skeleton = Skeleton::new(&[
        SkeletonJoint {
            name: "a",
            parent: None,
        },
        SkeletonJoint {
            name: "b",
            parent: Some(0),
        },
    ])
    .unwrap();
    let shape = sphere();
    let extent = limits::MAX_SHAPE_EXTENT;
    let tx = SixDofConstraintAxis::TranslationX;
    for (min, max) in [(extent.next_down(), extent), (-extent, (-extent).next_up())] {
        let (mut world, layers) = common::ragdoll::ragdoll_world(1);
        let joint = SixDofConstraintSettings::new(
            RVec3::ZERO,
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .axis(tx, SixDofAxis::Limited { min, max });
        let body = BodySettings::new_dynamic().object_layer(layers.ragdoll);
        let parts = [
            RagdollPart {
                shape: &shape,
                body: body.clone().mass(limits::MAX_MASS),
                joint: None,
            },
            RagdollPart {
                shape: &shape,
                body: body
                    .position(RVec3::new(0.0, 2.0, 0.0))
                    .mass(limits::MIN_MASS),
                joint: Some(RagdollJoint::SixDof(joint)),
            },
        ];
        let settings = RagdollSettings::new(&skeleton, &parts).unwrap();
        let id = world
            .create_ragdoll(&settings, None, Activation::Activate)
            .unwrap();
        let bodies = world.ragdoll(id).unwrap().body_ids().to_vec();
        step_at_both_extremes(&mut world, &bodies, &format!("limits {min}..{max}"));
    }
}

/// A flat heightfield of 4 x 4 samples, three cells of `scale` metres per side from `offset`
/// on x and z.
fn flat_field(offset: f32, scale: f32) -> Result<Shape, ShapeError> {
    Shape::new_height_field(
        4,
        &[0.0; 16],
        &HeightFieldSettings::default()
            .offset(Vec3::new(offset, 0.0, offset))
            .scale(Vec3::new(scale, 1.0, scale)),
    )
}

#[test]
fn height_field_extent_is_bounded() {
    let extent = limits::MAX_SHAPE_EXTENT;
    let field = |offset: f32, scale: f32| flat_field(offset, scale);
    // Three cells of 1333 m from -1999 m end at 2000 m.
    assert!(field(-1999.0, 1333.0).is_ok());
    assert!(matches!(
        field(-1999.0, 1333.0f32.next_up()),
        Err(ShapeError::InvalidDimensions(_))
    ));
    assert!(matches!(
        field(-extent.next_up(), 1.0),
        Err(ShapeError::InvalidDimensions(_))
    ));
}

#[test]
fn query_inputs_are_bounded_by_the_frame() {
    let mut world = empty_world();
    add_floor(&mut world);
    let all = QueryFilter::new();
    let ball = sphere();
    let bound = limits::MAX_POSITION;
    // `Real` is `f32` without the `double-precision` feature, so the cast is a no-op there.
    #[allow(clippy::unnecessary_cast)]
    let span = (2.0 * bound) as f32;
    let corner = RVec3::new(bound, bound, bound);
    let beyond = RVec3::new(bound.next_up(), 0.0, 0.0);
    let across = Vec3::new(-span, -span, -span);

    world.cast_ray(RayCast::new(corner, across), &all).unwrap();
    assert!(query_invalid(world.cast_ray(
        RayCast::new(beyond, Vec3::new(0.0, -1.0, 0.0)),
        &all
    )));

    let cast = |position: RVec3, direction: Vec3| {
        world.cast_shape(
            &ShapeCast::new(&ball, position, Quat::IDENTITY, direction),
            &all,
        )
    };
    cast(corner, across).unwrap();
    assert!(query_invalid(cast(beyond, Vec3::new(0.0, -1.0, 0.0))));
    assert!(query_invalid(cast(
        corner,
        Vec3::new(-span.next_up(), 0.0, 0.0)
    )));

    let extent = limits::MAX_SHAPE_EXTENT;
    let collide = |position: RVec3, separation: f32| {
        world.collide_shape(
            &CollideShape::new(&ball, position, Quat::IDENTITY).max_separation_distance(separation),
            &all,
        )
    };
    collide(corner, extent).unwrap();
    assert!(query_invalid(collide(beyond, 0.0)));
    assert!(query_invalid(collide(corner, extent.next_up())));
}

#[test]
fn queries_with_shapes_at_the_extent_bound_stay_finite() {
    let mut world = empty_world();
    let extent = limits::MAX_SHAPE_EXTENT;
    let slab = Shape::new_box(Vec3::new(extent, 1.0, extent)).unwrap();
    world
        .create_body(&slab, &BodySettings::new_static())
        .unwrap();
    let all = QueryFilter::new();
    let largest = Shape::new_sphere(extent).unwrap();
    let bound = limits::MAX_POSITION;
    // `Real` is `f32` without the `double-precision` feature, so the cast is a no-op there.
    #[allow(clippy::unnecessary_cast)]
    let span = (2.0 * bound) as f32;
    for sign in [1.0_f32, -1.0] {
        // From a frame corner across the frame, through the slab.
        let at = Real::from(sign) * bound;
        let corner = RVec3::new(at, at, at);
        let across = Vec3::new(-sign * span, -sign * span, -sign * span);
        let hit = world
            .cast_shape(
                &ShapeCast::new(&largest, corner, Quat::IDENTITY, across),
                &all,
            )
            .unwrap()
            .expect("the cast passes through the slab");
        let values = [
            v3(hit.point),
            f3(hit.normal),
            [hit.fraction, hit.distance, hit.penetration_depth].map(f64::from),
        ];
        assert!(
            values.iter().flatten().all(|value| value.is_finite()),
            "{hit:?}"
        );
    }
    // Deep overlap at the centre, reporting separations up to the extent bound.
    let hits = world
        .collide_shape(
            &CollideShape::new(&largest, RVec3::ZERO, Quat::IDENTITY)
                .max_separation_distance(extent),
            &all,
        )
        .unwrap();
    assert!(!hits.is_empty());
    for hit in &hits {
        let values = [
            v3(hit.point_on_shape),
            v3(hit.point_on_body),
            f3(hit.normal),
            [f64::from(hit.penetration_depth); 3],
        ];
        assert!(
            values.iter().flatten().all(|value| value.is_finite()),
            "{hit:?}"
        );
    }
}

#[test]
fn a_ray_with_a_huge_finite_direction_is_cast() {
    // Ray directions are only checked to be finite and not zero; this one reaches far beyond the
    // frame and is cast without overflow.
    let mut world = empty_world();
    let floor = add_floor(&mut world);
    let hit = world
        .cast_ray(
            RayCast::new(RVec3::new(0.0, 10.0, 0.0), Vec3::new(0.0, -1.0e30, 0.0)),
            &QueryFilter::new(),
        )
        .unwrap()
        .expect("the ray hits the floor");
    assert_eq!(hit.body, floor);
    assert!(hit.fraction.is_finite() && hit.fraction > 0.0);
}

/// Steps `world` ten times at the largest and ten times at the smallest time step, checking
/// `bodies` finite after each step.
fn step_at_both_extremes(world: &mut PhysicsWorld, bodies: &[BodyId], what: &str) {
    for delta_time in [PhysicsWorld::MAX_DELTA_TIME, PhysicsWorld::MIN_DELTA_TIME] {
        for tick in 0..10 {
            let _ = world.step(delta_time).unwrap();
            for &id in bodies {
                assert_body_finite(world, id, &format!("{what}, dt {delta_time}, tick {tick}"));
            }
        }
    }
}

#[test]
fn bodies_at_every_bound_step_finitely() {
    let gravity = Vec3::new(0.0, -limits::MAX_ACCELERATION, 0.0);
    let mut world = world(gravity, 1);
    let bound = limits::MAX_POSITION;
    let floor_shape = Shape::new_box(Vec3::new(100.0, 1.0, 100.0)).unwrap();
    world
        .create_body(
            &floor_shape,
            &BodySettings::new_static().position(RVec3::new(bound - 100.0, -bound, 0.0)),
        )
        .unwrap();
    let ball = sphere();
    let speed = limits::MAX_LINEAR_VELOCITY;
    let spin = Vec3::new(0.0, limits::MAX_ANGULAR_VELOCITY, 0.0);
    let factor = limits::MAX_GRAVITY_FACTOR;
    let mut bodies = Vec::new();
    // Head-on pairs of the lightest and the heaviest body at a frame corner, both velocity
    // maxima and both gravity factor extremes.
    for (mass, gravity_factor, y) in [
        (limits::MIN_MASS, factor, bound),
        (limits::MAX_MASS, -factor, bound - 5.0),
        (limits::MAX_MASS, factor, bound - 10.0),
        (limits::MIN_MASS, -factor, bound - 15.0),
    ] {
        for (x, direction) in [(bound, -1.0), (bound - 1.5, 1.0)] {
            let settings = BodySettings::new_dynamic()
                .position(RVec3::new(x, y, bound))
                .mass(mass)
                .gravity_factor(gravity_factor)
                .linear_velocity(Vec3::new(direction * speed, 0.0, 0.0))
                .angular_velocity(spin)
                .allow_sleeping(false);
            bodies.push(world.create_body(&ball, &settings).unwrap());
        }
    }
    // A bouncing body with restitution 1, and two bodies with the largest friction sliding on
    // each other, on the floor.
    let on_floor = |x: Real, y: Real| RVec3::new(bound - 100.0 + x, -bound + y, 0.0);
    let cube = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
    bodies.push(
        world
            .create_body(
                &ball,
                &BodySettings::new_dynamic()
                    .position(on_floor(0.0, 3.0))
                    .restitution(1.0),
            )
            .unwrap(),
    );
    for (x, y, v) in [(10.0, 1.5, speed), (10.0, 2.6, -speed)] {
        bodies.push(
            world
                .create_body(
                    &cube,
                    &BodySettings::new_dynamic()
                        .position(on_floor(x, y))
                        .friction(limits::MAX_FRICTION)
                        .linear_velocity(Vec3::new(v, 0.0, 0.0)),
                )
                .unwrap(),
        );
    }
    // Loads at the acceleration bounds, one through a force at a frame corner.
    let loaded = world
        .create_body(&ball, &BodySettings::new_dynamic().mass(2.0))
        .unwrap();
    let pushed = world
        .create_body(
            &ball,
            &BodySettings::new_dynamic()
                .position(RVec3::new(bound - 1.0, bound - 1.0, bound - 1.0))
                .mass(limits::MAX_MASS)
                .gravity_factor(0.0),
        )
        .unwrap();
    let corner = RVec3::new(bound, bound, bound);
    bodies.extend([loaded, pushed]);
    for delta_time in [PhysicsWorld::MAX_DELTA_TIME, PhysicsWorld::MIN_DELTA_TIME] {
        for tick in 0..10 {
            let mut body = world.body_mut(loaded).unwrap();
            body.add_force(Vec3::new(2.0 * limits::MAX_ACCELERATION, 0.0, 0.0))
                .unwrap();
            // The torque bound depends on the inertia; Jolt's sphere inertia is 0.4 m r².
            let torque = 0.999 * limits::MAX_ANGULAR_ACCELERATION * 0.4 * 2.0 * 0.25;
            body.add_torque(Vec3::new(torque, 0.0, 0.0)).unwrap();
            // A force along y at the frame corner, sized so that its torque about the moving
            // centre stays just below the bound (Jolt's sphere inertia, 0.4 m r²).
            let mut body = world.body_mut(pushed).unwrap();
            let center = body.position();
            let lever =
                (wide(corner.x - center.x).powi(2) + wide(corner.z - center.z).powi(2)).sqrt();
            let inertia = 0.4 * f64::from(limits::MAX_MASS) * 0.25;
            let torque = 0.999 * f64::from(limits::MAX_ANGULAR_ACCELERATION) * inertia;
            let force = (torque / lever.max(1.0)) as f32;
            body.add_force_at_point(Vec3::new(0.0, force, 0.0), corner)
                .unwrap();
            let _ = world.step(delta_time).unwrap();
            for &id in &bodies {
                assert_body_finite(&world, id, &format!("dt {delta_time}, tick {tick}"));
            }
        }
    }
}

/// A character of [`limits::MAX_MASS`] whose capsule rests on the +x edge of a cube of
/// [`limits::MIN_MASS`] with half extent 0.03 m, in a world without gravity. That cube is about
/// the smallest one for which Jolt keeps its own inertia, so the weight impulse at the edge
/// turns it fastest.
fn character_on_the_edge_of_a_light_cube() -> (PhysicsWorld, BodyId, CharacterId) {
    let mut world = empty_world();
    let half = 0.03;
    let cube_shape = Shape::new_box_with_convex_radius(Vec3::new(half, half, half), 0.0).unwrap();
    let cube = world
        .create_body(
            &cube_shape,
            &BodySettings::new_dynamic()
                .position(RVec3::new(0.0, -Real::from(half), 0.0))
                .mass(limits::MIN_MASS)
                .allow_sleeping(false),
        )
        .unwrap();
    let capsule = Shape::new_capsule(0.5, 0.3).unwrap();
    let settings = CharacterSettings::new(&capsule)
        .mass(limits::MAX_MASS)
        .shape_offset(Vec3::new(0.0, 0.8, 0.0));
    // The lower sphere (radius 0.3) is centred 0.1 m beyond the edge and touches it.
    let sphere_height = (0.3f32 * 0.3 - 0.1 * 0.1).sqrt();
    let position = RVec3::new(Real::from(half + 0.1), Real::from(sphere_height - 0.3), 0.0);
    let id = world
        .create_character(&settings, position, Quat::IDENTITY)
        .unwrap();
    world
        .refresh_character_contacts(id, &QueryFilter::new())
        .unwrap();
    (world, cube, id)
}

#[test]
fn character_weight_impulse_at_a_lever_arm_is_bounded() {
    let delta_time = PhysicsWorld::MAX_DELTA_TIME;
    let update = |world: &mut PhysicsWorld, id, gravity: f32| {
        world.update_character(
            id,
            delta_time,
            Vec3::new(0.0, -gravity, 0.0),
            &ExtendedUpdateSettings::default(),
            &QueryFilter::new(),
        )
    };
    // Mass times gravity times delta time is exactly the bound.
    let at_bound = limits::MAX_WEIGHT_IMPULSE / (limits::MAX_MASS * delta_time);
    let (mut world, cube, id) = character_on_the_edge_of_a_light_cube();
    update(&mut world, id, at_bound).unwrap();
    assert_eq!(world.character(id).unwrap().ground_body(), Some(cube));
    assert_body_finite(&world, cube, "weight impulse at the bound");
    // The impulse turns the cube up to Jolt's clamp; an overflow would have zeroed it.
    let spin = f3(world.body(cube).unwrap().angular_velocity())
        .iter()
        .map(|c| c * c)
        .sum::<f64>()
        .sqrt();
    assert!(
        spin >= 0.99 * f64::from(limits::MAX_ANGULAR_VELOCITY),
        "angular speed {spin}"
    );
    for gravity in [at_bound.next_up(), limits::MAX_ACCELERATION] {
        let (mut world, cube, id) = character_on_the_edge_of_a_light_cube();
        let position = world.character(id).unwrap().position();
        assert!(matches!(
            update(&mut world, id, gravity),
            Err(CharacterError::InvalidValue(_))
        ));
        assert_eq!(
            real_bits(world.character(id).unwrap().position()),
            real_bits(position)
        );
        let body = world.body(cube).unwrap();
        assert_eq!(bits(body.linear_velocity()), bits(Vec3::ZERO));
        assert_eq!(bits(body.angular_velocity()), bits(Vec3::ZERO));
    }
}

#[test]
fn character_at_its_bounds_pushes_the_lightest_body() {
    let mut world = empty_world();
    add_floor(&mut world);
    let crate_shape = Shape::new_box_with_convex_radius(Vec3::new(0.2, 0.2, 0.2), 0.0).unwrap();
    let lightest = BodySettings::new_dynamic().mass(limits::MIN_MASS);
    let under = world
        .create_body(
            &crate_shape,
            &lightest.clone().position(RVec3::new(0.0, 0.2, 0.0)),
        )
        .unwrap();
    let ahead = world
        .create_body(&crate_shape, &lightest.position(RVec3::new(1.0, 0.6, 0.0)))
        .unwrap();
    let capsule = Shape::new_capsule(0.5, 0.3).unwrap();
    let settings = CharacterSettings::new(&capsule)
        .mass(limits::MAX_MASS)
        .max_strength(f32::MAX)
        .shape_offset(Vec3::new(0.0, 0.8, 0.0));
    let id = world
        .create_character(&settings, RVec3::new(0.0, 0.4, 0.0), Quat::IDENTITY)
        .unwrap();
    let all = QueryFilter::new();
    world.refresh_character_contacts(id, &all).unwrap();
    let velocity = Vec3::new(limits::MAX_LINEAR_VELOCITY, 0.0, 0.0);
    for delta_time in [PhysicsWorld::MAX_DELTA_TIME, PhysicsWorld::MIN_DELTA_TIME] {
        let weight_bound = limits::MAX_WEIGHT_IMPULSE / (limits::MAX_MASS * delta_time);
        let gravity = Vec3::new(0.0, -limits::MAX_ACCELERATION.min(weight_bound), 0.0);
        for tick in 0..30 {
            world
                .character_mut(id)
                .unwrap()
                .set_linear_velocity(velocity)
                .unwrap();
            world
                .update_character(
                    id,
                    delta_time,
                    gravity,
                    &ExtendedUpdateSettings::default(),
                    &all,
                )
                .unwrap();
            let _ = world.step(delta_time).unwrap();
            let what = format!("dt {delta_time}, tick {tick}");
            for body in [under, ahead] {
                assert_body_finite(&world, body, &what);
            }
            let character = world.character(id).unwrap();
            let state = [v3(character.position()), f3(character.linear_velocity())];
            assert!(
                state.iter().flatten().all(|value| value.is_finite()),
                "{what}: {state:?}"
            );
        }
    }
}

#[test]
fn vehicle_gravity_at_the_bound_steps_finitely() {
    let (mut world, layers) = car_world(Vec3::ZERO, 1);
    let ground_shape = Shape::new_box(Vec3::new(50.0, 1.0, 50.0)).unwrap();
    world
        .create_body(
            &ground_shape,
            &BodySettings::new_static()
                .object_layer(layers.ground)
                .position(RVec3::new(0.0, -1.0, 0.0)),
        )
        .unwrap();
    let body = chassis_settings(&layers, RVec3::new(0.0, 0.9, 0.0), Quat::IDENTITY);
    let (chassis, car) = add_car_with(&mut world, &body, VehicleCollisionTester::ray(layers.probe));
    let mass = world.body(chassis).unwrap().mass().unwrap();
    for tick in 0..60 {
        world
            .vehicle_mut(car)
            .unwrap()
            .set_gravity(Vec3::new(0.0, -limits::MAX_ACCELERATION, 0.0))
            .unwrap();
        // Within the load bound: the force alone gives at most MAX_ACCELERATION.
        let force = 0.999 * limits::MAX_ACCELERATION * mass;
        world
            .body_mut(chassis)
            .unwrap()
            .add_force(Vec3::new(force, 0.0, 0.0))
            .unwrap();
        let _ = world.step(DT).unwrap();
        assert_body_finite(&world, chassis, &format!("tick {tick}"));
    }
}

#[test]
fn vehicle_springs_and_anti_roll_bar_at_their_bounds_step_finitely() {
    let (mut world, layers) = car_world(Vec3::ZERO, 1);
    let ground_shape = Shape::new_box(Vec3::new(50.0, 1.0, 50.0)).unwrap();
    world
        .create_body(
            &ground_shape,
            &BodySettings::new_static()
                .object_layer(layers.ground)
                .position(RVec3::new(0.0, -1.0, 0.0)),
        )
        .unwrap();
    let coefficient = limits::MAX_SPRING_COEFFICIENT;
    // The largest frequency whose stiffness stays within the bound for a chassis of MAX_MASS.
    let max_frequency = (f64::from(coefficient) / f64::from(limits::MAX_MASS)).sqrt()
        / (2.0 * std::f64::consts::PI);
    let springs = [
        SuspensionSpring::StiffnessAndDamping {
            stiffness: coefficient,
            damping: coefficient,
        },
        SuspensionSpring::FrequencyAndDamping {
            frequency: (0.999 * max_frequency) as f32,
            damping: 1.0,
        },
        SuspensionSpring::default(),
        SuspensionSpring::default(),
    ];
    let wheels = WHEEL_POSITIONS
        .iter()
        .zip(springs)
        .map(|(&position, spring)| {
            WheelSettings::new(position)
                .radius(WHEEL_RADIUS)
                .width(WHEEL_WIDTH)
                .suspension_min_length(SUSPENSION_MIN)
                .suspension_max_length(SUSPENSION_MAX)
                .suspension_spring(spring)
        })
        .collect();
    let settings = VehicleSettings::new(
        wheels,
        vec![VehicleDifferentialSettings::new(Some(0), Some(1))],
        VehicleCollisionTester::ray(layers.probe),
    )
    .anti_roll_bars(vec![
        VehicleAntiRollBar::new(0, 1).stiffness(VehicleAntiRollBar::MAX_STIFFNESS),
        VehicleAntiRollBar::new(2, 3).stiffness(VehicleAntiRollBar::MAX_STIFFNESS),
    ]);
    // Tilted, so that the suspension lengths differ and the anti-roll bars push.
    let tilt = Quat::from_xyzw(0.0, 0.0, 0.05, (1.0_f32 - 0.0025).sqrt());
    let body = chassis_settings(&layers, RVec3::new(0.0, 0.9, 0.0), tilt);
    let chassis = world.create_body(&chassis_shape(), &body).unwrap();
    let car = world.create_vehicle(chassis, &settings).unwrap();
    world
        .vehicle_mut(car)
        .unwrap()
        .set_gravity(GRAVITY)
        .unwrap();
    for tick in 0..60 {
        let _ = world.step(DT).unwrap();
        assert_body_finite(&world, chassis, &format!("tick {tick}"));
    }
    step_at_both_extremes(&mut world, &[chassis], "vehicle springs");
}

#[test]
fn friction_at_the_bound_keeps_contacts_finite() {
    // A spinning, sliding cube 1 cm above a floor, both at the largest friction: Jolt makes a
    // speculative contact whose normal impulse is zero, the case in which an infinite combined
    // friction (two bodies at `f32::MAX`) turned the cube's velocities into NaN within three
    // 60 Hz steps. Then the same cube resting on the floor under gravity.
    for (gravity, height) in [(Vec3::ZERO, 1.51), (Vec3::new(0.0, -9.81, 0.0), 1.5)] {
        let mut world = world(gravity, 1);
        let floor = Shape::new_box(Vec3::new(10.0, 1.0, 10.0)).unwrap();
        world
            .create_body(
                &floor,
                &BodySettings::new_static().friction(limits::MAX_FRICTION),
            )
            .unwrap();
        let cube = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
        let id = world
            .create_body(
                &cube,
                &BodySettings::new_dynamic()
                    .position(RVec3::new(0.0, height, 0.0))
                    .friction(limits::MAX_FRICTION)
                    .linear_velocity(Vec3::new(1.0, 0.0, 0.0))
                    .angular_velocity(Vec3::new(0.0, 5.0, 0.0))
                    .allow_sleeping(false),
            )
            .unwrap();
        for tick in 0..30 {
            let _ = world.step(DT).unwrap();
            assert_body_finite(&world, id, &format!("gravity {gravity:?}, tick {tick}"));
        }
        step_at_both_extremes(&mut world, &[id], &format!("gravity {gravity:?}"));
    }
}

#[test]
fn largest_shapes_collide_finitely() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    let extent = limits::MAX_SHAPE_EXTENT;
    let terrain = flat_field(-1999.0, 1333.0).unwrap();
    world
        .create_body(&terrain, &BodySettings::new_static())
        .unwrap();
    let heaviest = BodySettings::new_dynamic().mass(limits::MAX_MASS);
    let slab = Shape::new_box(Vec3::new(extent, 1.0, extent)).unwrap();
    let pole = Shape::new_capsule(extent - 1.0, 1.0).unwrap();
    let compound = Shape::new_compound(&[
        CompoundChild {
            shape: &slab,
            position: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            user_data: 1,
        },
        CompoundChild {
            shape: &pole,
            position: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            user_data: 2,
        },
    ])
    .unwrap();
    let mut bodies = Vec::new();
    for (shape, y) in [
        (&slab, 2.0),
        (&pole, Real::from(extent) + 4.0),
        (&compound, 10.0),
    ] {
        bodies.push(
            world
                .create_body(shape, &heaviest.clone().position(RVec3::new(0.0, y, 0.0)))
                .unwrap(),
        );
    }
    step_at_both_extremes(&mut world, &bodies, "largest shapes");
}

/// A static anchor and a dynamic sphere of `mass` at the origin, hinged about z at the origin.
fn hinged_sphere(
    mass: f32,
    settings: HingeConstraintSettings,
) -> (
    PhysicsWorld,
    BodyId,
    Result<ConstraintId<HingeConstraint>, ConstraintError>,
) {
    let mut world = empty_world();
    let anchor = world
        .create_body(
            &sphere(),
            &BodySettings::new_static().position(RVec3::new(0.0, 5.0, 0.0)),
        )
        .unwrap();
    let body = world
        .create_body(&sphere(), &BodySettings::new_dynamic().mass(mass))
        .unwrap();
    let hinge = world.create_constraint(anchor, body, &settings);
    (world, body, hinge)
}

fn z_hinge() -> HingeConstraintSettings {
    HingeConstraintSettings::new(
        RVec3::ZERO,
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
    )
}

fn constraint_invalid(result: Result<impl Sized, ConstraintError>) -> bool {
    matches!(result, Err(ConstraintError::InvalidValue(_)))
}

#[test]
fn world_constraint_springs_are_bounded_by_the_bodies_effective_mass() {
    // A sphere's largest principal moment, 0.1 m for a radius of 0.5 m, is below its mass, so
    // the bound is the mass as Jolt stores it: 1 / (1 / m) with the inverse in f32.
    let mass = limits::MAX_MASS;
    let bound = 1.0 / f64::from(1.0 / mass);
    let fits = |frequency: f32| {
        let omega = 2.0 * std::f64::consts::PI * f64::from(frequency);
        bound * omega * omega <= f64::from(limits::MAX_SPRING_COEFFICIENT)
    };
    let mut frequency = ((f64::from(limits::MAX_SPRING_COEFFICIENT) / bound).sqrt()
        / (2.0 * std::f64::consts::PI)) as f32;
    while !fits(frequency) {
        frequency = frequency.next_down();
    }
    while fits(frequency.next_up()) {
        frequency = frequency.next_up();
    }
    let spring = |frequency| SpringSettings::FrequencyAndDamping {
        frequency,
        damping: 0.0,
    };
    let motor = |frequency| MotorSettings::default().spring(spring(frequency));

    let (_, _, accepted) = hinged_sphere(mass, z_hinge().motor(motor(frequency)));
    assert!(accepted.is_ok());
    let (_, _, rejected) = hinged_sphere(mass, z_hinge().motor(motor(frequency.next_up())));
    assert!(constraint_invalid(rejected));

    let (mut world, _, hinge) = hinged_sphere(mass, z_hinge());
    let mut hinge = world.constraint_mut(hinge.unwrap()).unwrap();
    assert!(hinge.set_motor_settings(motor(frequency)).is_ok());
    assert!(constraint_invalid(
        hinge.set_motor_settings(motor(frequency.next_up()))
    ));
    assert!(hinge.set_limits_spring(spring(frequency)).is_ok());
    assert!(constraint_invalid(
        hinge.set_limits_spring(spring(frequency.next_up()))
    ));

    // A static body has no effective mass, so any valid frequency fits it; between two static
    // bodies nothing bounds a spring.
    let (mut world, body, _) = hinged_sphere(mass, z_hinge());
    let anchor = world
        .create_body(
            &sphere(),
            &BodySettings::new_static().position(RVec3::new(3.0, 0.0, 0.0)),
        )
        .unwrap();
    let other = world
        .create_body(
            &sphere(),
            &BodySettings::new_static().position(RVec3::new(-3.0, 0.0, 0.0)),
        )
        .unwrap();
    let rod =
        DistanceConstraintSettings::new(RVec3::new(3.0, 0.0, 0.0), RVec3::new(-3.0, 0.0, 0.0))
            .limits_spring(spring(f32::MAX));
    assert!(world.create_constraint(anchor, other, &rod).is_ok());
    let rod = DistanceConstraintSettings::new(RVec3::new(3.0, 0.0, 0.0), RVec3::ZERO);
    let rod = world.create_constraint(anchor, body, &rod).unwrap();
    let mut rod = world.constraint_mut(rod).unwrap();
    assert!(rod.set_limits_spring(spring(frequency)).is_ok());
    assert!(constraint_invalid(
        rod.set_limits_spring(spring(frequency.next_up()))
    ));
}

#[test]
fn constraint_targets_are_bounded() {
    let (mut world, body, hinge) = hinged_sphere(1.0, z_hinge());
    let mut hinge = world.constraint_mut(hinge.unwrap()).unwrap();
    for angle in [PI, -PI] {
        assert!(hinge.set_target_angle(angle).is_ok());
    }
    for angle in [PI.next_up(), (-PI).next_down(), f32::NAN] {
        assert!(constraint_invalid(hinge.set_target_angle(angle)));
    }
    let speed = limits::MAX_ANGULAR_VELOCITY;
    for velocity in [speed, -speed] {
        assert!(hinge.set_target_angular_velocity(velocity).is_ok());
    }
    for velocity in [speed.next_up(), (-speed).next_down(), f32::INFINITY] {
        assert!(constraint_invalid(
            hinge.set_target_angular_velocity(velocity)
        ));
    }

    let extent = limits::MAX_SHAPE_EXTENT;
    let anchor = world
        .create_body(
            &sphere(),
            &BodySettings::new_static().position(RVec3::new(3.0, 0.0, 0.0)),
        )
        .unwrap();
    let rod = |range| {
        DistanceConstraintSettings::new(RVec3::new(3.0, 0.0, 0.0), RVec3::ZERO).range(range)
    };
    for (min, max) in [(extent.next_up(), extent.next_up()), (-1.0e-6, 1.0)] {
        assert!(constraint_invalid(world.create_constraint(
            anchor,
            body,
            &rod(DistanceRange::Range { min, max })
        )));
    }
    let rod = world
        .create_constraint(
            anchor,
            body,
            &rod(DistanceRange::Range {
                min: 0.0,
                max: extent,
            }),
        )
        .unwrap();
    let mut rod = world.constraint_mut(rod).unwrap();
    assert!(rod.set_distance(extent, extent).is_ok());
    assert!(rod.set_distance(0.0, 0.0).is_ok());
    for (min, max) in [
        (0.0, extent.next_up()),
        (-1.0e-6, 0.0),
        (1.0, 0.5),
        (f32::NAN, 1.0),
    ] {
        assert!(constraint_invalid(rod.set_distance(min, max)));
    }

    // Constraint points follow the frame rule. A kinematic body has no lever-arm check, so
    // only the frame rule applies.
    let held = world
        .create_body(&sphere(), &BodySettings::new_kinematic())
        .unwrap();
    let point = |at| PointConstraintSettings::new(RVec3::ZERO).point2(at);
    for at in real_on_axes(limits::MAX_POSITION) {
        let id = world.create_constraint(anchor, held, &point(at)).unwrap();
        world.remove_constraint(id).unwrap();
    }
    for at in real_on_axes(limits::MAX_POSITION.next_up()) {
        assert!(constraint_invalid(world.create_constraint(
            anchor,
            held,
            &point(at)
        )));
    }
}

#[test]
fn constraint_friction_at_f32_max_steps_finitely() {
    // Jolt clamps a constraint's friction impulse to `dt` times the friction: a hinge of
    // unlimited friction on bodies of both mass extremes spinning at the velocity bound.
    for mass in [limits::MIN_MASS, limits::MAX_MASS] {
        let (mut world, body, hinge) = hinged_sphere(mass, z_hinge().max_friction_torque(f32::MAX));
        let hinge = hinge.unwrap();
        world
            .constraint_mut(hinge)
            .unwrap()
            .set_max_friction_torque(f32::MAX)
            .unwrap();
        world
            .body_mut(body)
            .unwrap()
            .set_angular_velocity(Vec3::new(0.0, 0.0, limits::MAX_ANGULAR_VELOCITY))
            .unwrap();
        for tick in 0..120 {
            let _ = world.step(DT).unwrap();
            assert_body_finite(&world, body, &format!("mass {mass}, tick {tick}"));
        }
        let lambda = world.constraint(hinge).unwrap().total_lambda_position();
        assert!(
            f3(lambda).iter().all(|value| value.is_finite()),
            "{lambda:?}"
        );
        step_at_both_extremes(&mut world, &[body], &format!("friction, mass {mass}"));
    }
}

/// A static anchor and a dynamic sphere of `mass` at the origin joined by `settings`.
fn joined_sphere<S: ConstraintSettings>(
    mass: f32,
    settings: &S,
) -> (
    PhysicsWorld,
    BodyId,
    Result<ConstraintId<S::Kind>, ConstraintError>,
) {
    let mut world = empty_world();
    let anchor = world
        .create_body(
            &sphere(),
            &BodySettings::new_static().position(RVec3::new(0.0, 5.0, 0.0)),
        )
        .unwrap();
    let body = world
        .create_body(&sphere(), &BodySettings::new_dynamic().mass(mass))
        .unwrap();
    let id = world.create_constraint(anchor, body, settings);
    (world, body, id)
}

/// As [`joined_sphere`], with a kinematic sphere: without a dynamic body no lever-arm check
/// applies, so only the settings' own ranges decide.
fn joined_kinematic_sphere<S: ConstraintSettings>(
    settings: &S,
) -> Result<ConstraintId<S::Kind>, ConstraintError> {
    let mut world = empty_world();
    let anchor = world
        .create_body(
            &sphere(),
            &BodySettings::new_static().position(RVec3::new(0.0, 5.0, 0.0)),
        )
        .unwrap();
    let body = world
        .create_body(&sphere(), &BodySettings::new_kinematic())
        .unwrap();
    world.create_constraint(anchor, body, settings)
}

/// Two dynamic spheres of 1 kg, 3 m apart along x, joined by `settings`: gears, racks and
/// pinions and pulleys need two dynamic bodies.
fn coupled_spheres<S: ConstraintSettings>(
    settings: &S,
) -> Result<ConstraintId<S::Kind>, ConstraintError> {
    let mut world = empty_world();
    let [first, second] = [0.0, 3.0].map(|x| {
        world
            .create_body(
                &sphere(),
                &BodySettings::new_dynamic().position(RVec3::new(x, 0.0, 0.0)),
            )
            .unwrap()
    });
    world.create_constraint(first, second, settings)
}

const AXIS_X: Vec3 = Vec3::new(1.0, 0.0, 0.0);
const AXIS_Y: Vec3 = Vec3::new(0.0, 1.0, 0.0);

#[test]
fn slider_cone_swing_twist_and_six_dof_targets_are_bounded() {
    let extent = limits::MAX_SHAPE_EXTENT;
    let speed = limits::MAX_LINEAR_VELOCITY;
    let spin = limits::MAX_ANGULAR_VELOCITY;
    let slider = || SliderConstraintSettings::new(RVec3::ZERO, AXIS_X, AXIS_Y);
    for (min, max) in [(-extent, extent), (0.0, extent), (-extent, 0.0)] {
        assert!(joined_sphere(1.0, &slider().limits(min, max)).2.is_ok());
    }
    for (min, max) in [
        (-extent.next_up(), 0.0),
        (0.0, extent.next_up()),
        (0.1, 1.0),
    ] {
        assert!(constraint_invalid(
            joined_sphere(1.0, &slider().limits(min, max)).2
        ));
    }
    let (mut world, _, id) = joined_sphere(1.0, &slider());
    let mut slider = world.constraint_mut(id.unwrap()).unwrap();
    for position in [extent, -extent] {
        assert!(slider.set_target_position(position).is_ok());
    }
    for position in [extent.next_up(), (-extent).next_down(), f32::NAN] {
        assert!(constraint_invalid(slider.set_target_position(position)));
    }
    for velocity in [speed, -speed] {
        assert!(slider.set_target_velocity(velocity).is_ok());
    }
    for velocity in [speed.next_up(), (-speed).next_down()] {
        assert!(constraint_invalid(slider.set_target_velocity(velocity)));
    }
    assert!(slider.set_limits(Some((-extent, extent))).is_ok());
    assert!(constraint_invalid(
        slider.set_limits(Some((-extent, extent.next_up())))
    ));
    assert!(slider.set_limits(None).is_ok());

    let cone = |angle| ConeConstraintSettings::new(RVec3::ZERO, AXIS_X, angle);
    for angle in [0.0, PI] {
        assert!(joined_sphere(1.0, &cone(angle)).2.is_ok());
    }
    for angle in [-1.0e-6, PI.next_up(), f32::NAN] {
        assert!(constraint_invalid(joined_sphere(1.0, &cone(angle)).2));
    }
    let (mut world, _, id) = joined_sphere(1.0, &cone(0.5));
    let mut cone = world.constraint_mut(id.unwrap()).unwrap();
    assert!(cone.set_half_cone_angle(PI).is_ok());
    assert!(constraint_invalid(cone.set_half_cone_angle(PI.next_up())));

    let rotation_bound = |set: &mut dyn FnMut(Quat) -> Result<(), ConstraintError>| {
        assert!(set(Quat::from_xyzw(0.0, 0.6, 0.0, 0.8)).is_ok());
        assert!(constraint_invalid(set(Quat::from_xyzw(
            0.0, 0.6, 0.0, 0.81
        ))));
        assert!(constraint_invalid(set(Quat::from_xyzw(
            f32::NAN,
            0.0,
            0.0,
            1.0
        ))));
    };
    let swing_twist = SwingTwistConstraintSettings::new(RVec3::ZERO, AXIS_X, AXIS_Y)
        .half_cone_angles(1.0, 1.0)
        .twist_limits(-1.0, 1.0);
    let (mut world, _, id) = joined_sphere(1.0, &swing_twist);
    let mut joint = world.constraint_mut(id.unwrap()).unwrap();
    for velocity in on_axes(spin) {
        assert!(joint.set_target_angular_velocity_cs(velocity).is_ok());
    }
    for velocity in on_axes(spin.next_up()) {
        assert!(constraint_invalid(
            joint.set_target_angular_velocity_cs(velocity)
        ));
    }
    rotation_bound(&mut |q| joint.set_target_orientation_cs(q));

    let (mut world, _, id) = joined_sphere(1.0, &SixDofConstraintSettings::default());
    let mut joint = world.constraint_mut(id.unwrap()).unwrap();
    for velocity in on_axes(speed) {
        assert!(joint.set_target_velocity_cs(velocity).is_ok());
    }
    for velocity in on_axes(speed.next_up()) {
        assert!(constraint_invalid(joint.set_target_velocity_cs(velocity)));
    }
    for velocity in on_axes(spin) {
        assert!(joint.set_target_angular_velocity_cs(velocity).is_ok());
    }
    for velocity in on_axes(spin.next_up()) {
        assert!(constraint_invalid(
            joint.set_target_angular_velocity_cs(velocity)
        ));
    }
    for position in on_axes(extent) {
        assert!(joint.set_target_position_cs(position).is_ok());
    }
    for position in on_axes(extent.next_up()) {
        assert!(constraint_invalid(joint.set_target_position_cs(position)));
    }
    rotation_bound(&mut |q| joint.set_target_orientation_cs(q));
}

#[test]
fn slider_swing_twist_and_six_dof_motor_springs_are_bounded() {
    // As in `world_constraint_springs_are_bounded_by_the_bodies_effective_mass`, for the motor
    // setters of the other driven kinds.
    let mass = limits::MAX_MASS;
    let bound = 1.0 / f64::from(1.0 / mass);
    let fits = |frequency: f32| {
        let omega = 2.0 * std::f64::consts::PI * f64::from(frequency);
        bound * omega * omega <= f64::from(limits::MAX_SPRING_COEFFICIENT)
    };
    let mut frequency = ((f64::from(limits::MAX_SPRING_COEFFICIENT) / bound).sqrt()
        / (2.0 * std::f64::consts::PI)) as f32;
    while !fits(frequency) {
        frequency = frequency.next_down();
    }
    while fits(frequency.next_up()) {
        frequency = frequency.next_up();
    }
    let motor = |frequency| {
        MotorSettings::default().spring(SpringSettings::FrequencyAndDamping {
            frequency,
            damping: 0.0,
        })
    };
    let check = |set: &mut dyn FnMut(MotorSettings) -> Result<(), ConstraintError>| {
        assert!(set(motor(frequency)).is_ok());
        assert!(constraint_invalid(set(motor(frequency.next_up()))));
    };

    let slider = SliderConstraintSettings::new(RVec3::ZERO, AXIS_X, AXIS_Y);
    assert!(joined_sphere(mass, &slider.clone().motor(motor(frequency)))
        .2
        .is_ok());
    assert!(constraint_invalid(
        joined_sphere(mass, &slider.clone().motor(motor(frequency.next_up()))).2
    ));
    let (mut world, _, id) = joined_sphere(mass, &slider);
    let mut slider = world.constraint_mut(id.unwrap()).unwrap();
    check(&mut |m| slider.set_motor_settings(m));

    let (mut world, _, id) = joined_sphere(mass, &SwingTwistConstraintSettings::default());
    let mut joint = world.constraint_mut(id.unwrap()).unwrap();
    check(&mut |m| joint.set_swing_motor_settings(m));
    check(&mut |m| joint.set_twist_motor_settings(m));

    let (mut world, _, id) = joined_sphere(mass, &SixDofConstraintSettings::default());
    let mut joint = world.constraint_mut(id.unwrap()).unwrap();
    for axis in [
        SixDofConstraintAxis::TranslationX,
        SixDofConstraintAxis::RotationZ,
    ] {
        check(&mut |m| joint.set_motor_settings(axis, m));
    }
}

#[test]
fn slider_and_swing_twist_friction_at_f32_max_steps_finitely() {
    // As for the hinge: a slider sliding and a swing-twist joint spinning at the velocity
    // bounds, with unlimited friction, on bodies of both mass extremes.
    for mass in [limits::MIN_MASS, limits::MAX_MASS] {
        let slider =
            SliderConstraintSettings::new(RVec3::ZERO, AXIS_X, AXIS_Y).max_friction_force(f32::MAX);
        let (mut world, body, id) = joined_sphere(mass, &slider);
        world
            .constraint_mut(id.unwrap())
            .unwrap()
            .set_max_friction_force(f32::MAX)
            .unwrap();
        world
            .body_mut(body)
            .unwrap()
            .set_linear_velocity(Vec3::new(limits::MAX_LINEAR_VELOCITY, 0.0, 0.0))
            .unwrap();
        for tick in 0..120 {
            let _ = world.step(DT).unwrap();
            assert_body_finite(&world, body, &format!("slider, mass {mass}, tick {tick}"));
        }
        step_at_both_extremes(&mut world, &[body], &format!("slider, mass {mass}"));

        let joint = SwingTwistConstraintSettings::new(RVec3::ZERO, AXIS_X, AXIS_Y)
            .twist_limits(-PI, PI)
            .max_friction_torque(f32::MAX);
        let (mut world, body, id) = joined_sphere(mass, &joint);
        world
            .constraint_mut(id.unwrap())
            .unwrap()
            .set_max_friction_torque(f32::MAX)
            .unwrap();
        world
            .body_mut(body)
            .unwrap()
            .set_angular_velocity(Vec3::new(limits::MAX_ANGULAR_VELOCITY, 0.0, 0.0))
            .unwrap();
        for tick in 0..120 {
            let _ = world.step(DT).unwrap();
            assert_body_finite(
                &world,
                body,
                &format!("swing-twist, mass {mass}, tick {tick}"),
            );
        }
        step_at_both_extremes(&mut world, &[body], &format!("swing-twist, mass {mass}"));
    }
}

#[test]
fn coupling_ratios_are_bounded() {
    let z = Vec3::new(0.0, 0.0, 1.0);
    let max = limits::MAX_RATIO;
    let min = 1.0 / limits::MAX_RATIO;
    let gear = |ratio| GearConstraintSettings::new(z, z, ratio);
    let rack = |ratio| RackAndPinionConstraintSettings::new(z, AXIS_X, ratio);
    // A gear's ratio is between 1 and its own bound (see `GearConstraintSettings`).
    let gear_max = limits::MAX_GEAR_RATIO;
    for ratio in [1.0, gear_max] {
        assert!(coupled_spheres(&gear(ratio)).is_ok());
    }
    for ratio in [
        1.0_f32.next_down(),
        gear_max.next_up(),
        max,
        0.5,
        -1.0,
        -2.0,
        f32::NAN,
    ] {
        assert!(constraint_invalid(coupled_spheres(&gear(ratio))));
    }
    for ratio in [max, -max, min, -min] {
        assert!(coupled_spheres(&rack(ratio)).is_ok());
    }
    for ratio in [
        max.next_up(),
        -max.next_up(),
        min.next_down(),
        0.0,
        f32::NAN,
    ] {
        assert!(constraint_invalid(coupled_spheres(&rack(ratio))));
    }
    // Teeth give Jolt's ratios; zero teeth or a rack length out of range give none.
    assert!(coupled_spheres(&gear(1.0).teeth(10, 30)).is_ok());
    for (teeth1, teeth2) in [(0, 30), (30, 10), (1, 11)] {
        assert!(constraint_invalid(coupled_spheres(
            &gear(1.0).teeth(teeth1, teeth2)
        )));
    }
    assert!(coupled_spheres(&rack(1.0).teeth(20, 1.0, 10)).is_ok());
    for (teeth, length) in [
        (0, 1.0),
        (20, 0.0),
        (20, limits::MAX_SHAPE_EXTENT.next_up()),
    ] {
        assert!(constraint_invalid(coupled_spheres(
            &rack(1.0).teeth(teeth, length, 10)
        )));
    }
}

/// A static base far away and three spheres hinged to it about z at x = 0 and 3 and held on a
/// slider along x at x = -3, of masses `masses`.
fn coupling_scene(
    masses: [f32; 3],
) -> (
    PhysicsWorld,
    [BodyId; 3],
    [ConstraintId<HingeConstraint>; 2],
    ConstraintId<SliderConstraint>,
) {
    let z = Vec3::new(0.0, 0.0, 1.0);
    let mut world = empty_world();
    let base = world
        .create_body(
            &sphere(),
            &BodySettings::new_static().position(RVec3::new(0.0, -10.0, 0.0)),
        )
        .unwrap();
    let xs = [0.0, 3.0, -3.0];
    let bodies = [0, 1, 2].map(|i| {
        world
            .create_body(
                &sphere(),
                &BodySettings::new_dynamic()
                    .mass(masses[i])
                    .position(RVec3::new(xs[i], 0.0, 0.0)),
            )
            .unwrap()
    });
    let hinges = [0, 1].map(|i| {
        world
            .create_constraint(
                base,
                bodies[i],
                &HingeConstraintSettings::new(RVec3::new(xs[i], 0.0, 0.0), z, AXIS_X),
            )
            .unwrap()
    });
    let slider = world
        .create_constraint(
            base,
            bodies[2],
            &SliderConstraintSettings::new(RVec3::new(-3.0, 0.0, 0.0), AXIS_X, AXIS_Y),
        )
        .unwrap();
    (world, bodies, hinges, slider)
}

/// Steps 120 ticks and then both time step extremes, checking `bodies` finite.
fn step_coupling(world: &mut PhysicsWorld, bodies: &[BodyId], what: &str) {
    for tick in 0..120 {
        let _ = world.step(DT).unwrap();
        for &id in bodies {
            assert_body_finite(world, id, &format!("{what}, tick {tick}"));
        }
    }
    step_at_both_extremes(world, bodies, what);
}

#[test]
fn ratios_at_the_bound_step_finitely() {
    // Gears and racks and pinions at their ratio bounds between bodies of both mass extremes,
    // one of the coupled bodies spinning at the angular velocity bound.
    let z = Vec3::new(0.0, 0.0, 1.0);
    let spin = Vec3::new(0.0, 0.0, limits::MAX_ANGULAR_VELOCITY);
    let masses = [limits::MIN_MASS, limits::MAX_MASS];
    for mass1 in masses {
        for mass2 in masses {
            for spun in [0, 1] {
                for ratio in [1.0, limits::MAX_GEAR_RATIO] {
                    let (mut world, bodies, hinges, _) = coupling_scene([mass1, mass2, mass2]);
                    world
                        .create_constraint(
                            bodies[0],
                            bodies[1],
                            &GearConstraintSettings::new(z, z, ratio).hinges(hinges[0], hinges[1]),
                        )
                        .unwrap();
                    world
                        .body_mut(bodies[spun])
                        .unwrap()
                        .set_angular_velocity(spin)
                        .unwrap();
                    let what = format!("gear {ratio}, masses {mass1} {mass2}, body {spun} spun");
                    step_coupling(&mut world, &bodies[..2], &what);
                }
                let max = limits::MAX_RATIO;
                for ratio in [max, -max, 1.0 / max, -1.0 / max] {
                    let (mut world, bodies, hinges, slider) = coupling_scene([mass1, mass2, mass2]);
                    world
                        .create_constraint(
                            bodies[0],
                            bodies[2],
                            &RackAndPinionConstraintSettings::new(z, AXIS_X, ratio)
                                .constraints(hinges[0], slider),
                        )
                        .unwrap();
                    let driven = if spun == 0 { bodies[0] } else { bodies[2] };
                    let mut body = world.body_mut(driven).unwrap();
                    if spun == 0 {
                        body.set_angular_velocity(spin).unwrap();
                    } else {
                        body.set_linear_velocity(Vec3::new(limits::MAX_LINEAR_VELOCITY, 0.0, 0.0))
                            .unwrap();
                    }
                    let what = format!("rack {ratio}, masses {mass1} {mass2}, body {spun} driven");
                    step_coupling(&mut world, &[bodies[0], bodies[2]], &what);
                }
            }
        }
    }
}

/// A pulley between two dynamic spheres of `masses` hanging 3 m below their fixed points.
fn pulley_pair(
    gravity: Vec3,
    masses: [f32; 2],
    settings: impl Fn(PulleyConstraintSettings) -> PulleyConstraintSettings,
) -> (
    PhysicsWorld,
    [BodyId; 2],
    Result<ConstraintId<PulleyConstraint>, ConstraintError>,
) {
    let mut world = world(gravity, 1);
    let xs = [-2.0, 2.0];
    let bodies = [0, 1].map(|i| {
        world
            .create_body(
                &sphere(),
                &BodySettings::new_dynamic()
                    .mass(masses[i])
                    .position(RVec3::new(xs[i], 0.0, 0.0)),
            )
            .unwrap()
    });
    let pulley = settings(PulleyConstraintSettings::new(
        RVec3::new(-2.0, 0.0, 0.0),
        RVec3::new(-2.0, 3.0, 0.0),
        RVec3::new(2.0, 0.0, 0.0),
        RVec3::new(2.0, 3.0, 0.0),
    ));
    let id = world.create_constraint(bodies[0], bodies[1], &pulley);
    (world, bodies, id)
}

#[test]
fn pulley_ratio_and_lengths_are_bounded() {
    let max = limits::MAX_RATIO;
    let min = 1.0 / limits::MAX_RATIO;
    let create = |settings: &dyn Fn(PulleyConstraintSettings) -> PulleyConstraintSettings| {
        pulley_pair(Vec3::ZERO, [1.0, 1.0], settings).2
    };
    for ratio in [min, max] {
        assert!(create(&|s| s.ratio(ratio)).is_ok());
    }
    for ratio in [min.next_down(), max.next_up(), -1.0, 0.0, f32::NAN] {
        assert!(constraint_invalid(create(&|s| s.ratio(ratio))));
    }
    let longest = 3.0 * limits::MAX_SHAPE_EXTENT;
    let range = |min, max| PulleyLength::Range { min, max };
    for (lo, hi) in [(0.0, longest), (longest, longest), (0.0, 0.0)] {
        assert!(create(&|s| s.ratio(2.0).length(range(lo, hi))).is_ok());
    }
    for (lo, hi) in [(0.0, longest.next_up()), (-1.0e-6, 1.0), (2.0, 1.0)] {
        assert!(constraint_invalid(create(&|s| s
            .ratio(2.0)
            .length(range(lo, hi)))));
    }
    // Fixed points are world points under the frame rule, like body points.
    for point in real_on_axes(limits::MAX_POSITION.next_up()) {
        assert!(constraint_invalid(create(&|_| {
            PulleyConstraintSettings::new(
                RVec3::new(-2.0, 0.0, 0.0),
                point,
                RVec3::new(2.0, 0.0, 0.0),
                RVec3::new(2.0, 3.0, 0.0),
            )
        })));
    }

    let (mut world, _, id) = pulley_pair(Vec3::ZERO, [1.0, 1.0], |s| s.ratio(2.0));
    let mut pulley = world.constraint_mut(id.unwrap()).unwrap();
    assert!(pulley.set_length(0.0, longest).is_ok());
    assert!(pulley.set_length(longest, longest).is_ok());
    for (lo, hi) in [(0.0, longest.next_up()), (-1.0e-6, 1.0), (2.0, 1.0)] {
        assert!(constraint_invalid(pulley.set_length(lo, hi)));
    }
}

#[test]
fn pulleys_at_the_ratio_bound_step_finitely() {
    let gravity = Vec3::new(0.0, -9.81, 0.0);
    let masses = [limits::MIN_MASS, limits::MAX_MASS];
    for ratio in [limits::MAX_RATIO, 1.0 / limits::MAX_RATIO] {
        for mass1 in masses {
            for mass2 in masses {
                let (mut world, bodies, id) =
                    pulley_pair(gravity, [mass1, mass2], |s| s.ratio(ratio));
                id.unwrap();
                let what = format!("pulley {ratio}, masses {mass1} {mass2}");
                step_coupling(&mut world, &bodies, &what);
            }
        }
    }
}

/// A dynamic cube of half extent `half` and `mass` at `position`.
fn cube(world: &mut PhysicsWorld, half: f32, mass: f32, position: RVec3) -> BodyId {
    world
        .create_body(
            &Shape::new_box(Vec3::new(half, half, half)).unwrap(),
            &BodySettings::new_dynamic().mass(mass).position(position),
        )
        .unwrap()
}

/// The lever at which a cube of half extent `half` has the lever-arm ratio `ratio`: a cube's
/// ratio is `2 · |r|² / k²` with `k² = (2 · half)² / 6`.
fn cube_lever(half: f32, ratio: f32) -> f32 {
    let k_squared = (2.0 * half) * (2.0 * half) / 6.0;
    (ratio * k_squared / 2.0).sqrt()
}

/// `direction` scaled to `length`, as a position.
fn at(direction: Vec3, length: f32) -> RVec3 {
    let unit = length
        / (direction.x * direction.x + direction.y * direction.y + direction.z * direction.z)
            .sqrt();
    RVec3::new(
        Real::from(direction.x * unit),
        Real::from(direction.y * unit),
        Real::from(direction.z * unit),
    )
}

#[test]
fn lever_arms_are_bounded_by_the_bodies_size() {
    let bound = limits::MAX_LEVER_ARM_RATIO;
    let lever = cube_lever(0.5, bound);
    let below = lever * 0.999;
    let above = lever * 1.001;
    let directions = [AXIS_X, Vec3::new(0.0, -1.0, 0.0), Vec3::new(1.0, 1.0, -1.0)];

    // A point in world space on a 1 kg, 1 m cube held by a static anchor, in several
    // directions; the anchor itself is not checked.
    let mut world = empty_world();
    let anchor = world
        .create_body(&sphere(), &BodySettings::new_static())
        .unwrap();
    let body = cube(&mut world, 0.5, 1.0, RVec3::new(0.0, 0.0, 0.0));
    for direction in directions {
        let id = world
            .create_constraint(
                anchor,
                body,
                &PointConstraintSettings::new(at(direction, below)),
            )
            .unwrap();
        world.remove_constraint(id).unwrap();
        assert!(constraint_invalid(world.create_constraint(
            anchor,
            body,
            &PointConstraintSettings::new(at(direction, above))
        )));
    }
    // The same in the bodies' own frames, on a turned body: the ratio does not depend on the
    // frame.
    let turned = world
        .create_body(
            &Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap(),
            &BodySettings::new_dynamic()
                .position(RVec3::new(0.0, 30.0, 0.0))
                .rotation(Quat::from_xyzw(0.0, 0.6, 0.0, 0.8)),
        )
        .unwrap();
    let local = |length: f32| {
        let point = at(Vec3::new(1.0, 2.0, 3.0), length);
        HingeConstraintSettings::new(RVec3::ZERO, AXIS_X, AXIS_Y)
            .space(ConstraintSpace::LocalToBodyCom)
            .frame2(point, AXIS_X, AXIS_Y)
    };
    let id = world
        .create_constraint(anchor, turned, &local(below))
        .unwrap();
    world.remove_constraint(id).unwrap();
    assert!(constraint_invalid(world.create_constraint(
        anchor,
        turned,
        &local(above)
    )));
    // A rod held at its end has a ratio of 6 at any length.
    let rod = world
        .create_body(
            &Shape::new_box(Vec3::new(0.02, 1000.0, 0.02)).unwrap(),
            &BodySettings::new_dynamic().position(RVec3::new(0.0, -1000.0, 50.0)),
        )
        .unwrap();
    world
        .create_constraint(
            anchor,
            rod,
            &HingeConstraintSettings::new(RVec3::new(0.0, 0.0, 50.0), AXIS_X, AXIS_Y),
        )
        .unwrap();

    // An automatic point is where Jolt puts it: between the centres of mass, weighted by inverse
    // mass towards the lighter body, kinematic bodies included. A 1 kg, 1 m cube, dynamic or
    // kinematic, welded to a dynamic 1000 kg, 4 m cube holds the large cube at 1000/1001 of the
    // distance between them, which its ratio bounds.
    let weld = FixedConstraintSettings::default().auto_detect_point();
    let reach = cube_lever(2.0, bound) * 1.001;
    for light_settings in [BodySettings::new_dynamic(), BodySettings::new_kinematic()] {
        for (distance, accepted) in [(reach * 0.999, true), (reach * 1.001, false)] {
            let mut world = empty_world();
            let light = world
                .create_body(
                    &Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap(),
                    &light_settings.clone().mass(1.0),
                )
                .unwrap();
            let heavy = cube(&mut world, 2.0, 1000.0, at(AXIS_X, distance));
            let result = world.create_constraint(light, heavy, &weld);
            assert_eq!(result.is_ok(), accepted, "{distance}: {result:?}");
        }
    }
    // A 10 g, 6 cm part welded 3 cm from a 1000 kg cube is held at its own centre.
    for half in [0.5, 1.0, 2.0] {
        let mut world = empty_world();
        let big = cube(&mut world, half, 1000.0, RVec3::ZERO);
        let part = cube(&mut world, 0.03, 0.01, at(AXIS_X, half + 0.06));
        world.create_constraint(big, part, &weld).unwrap();
    }
    // Next to a static body it is the dynamic body's centre of mass.
    let far = cube(&mut world, 0.5, 1.0, RVec3::new(4000.0, 0.0, 0.0));
    world.create_constraint(anchor, far, &weld).unwrap();

    // A path holds body 1 anywhere along it: a path reaching beyond the bound from a dynamic
    // platform is refused, one within it accepted; body 2 sits at the path's start.
    let path = |length: f32| {
        PathConstraintSettings::new(
            HermitePath::new(
                Vec3::new(0.0, 0.0, 1.0),
                vec![
                    path_point([0.0, 0.0, 0.0], [length, 0.0, 0.0]),
                    path_point([length, 0.0, 0.0], [length, 0.0, 0.0]),
                ],
                false,
            )
            .unwrap(),
        )
    };
    for (length, accepted) in [(below * 0.7, true), (above, false)] {
        let mut world = empty_world();
        let platform = cube(&mut world, 0.5, 1.0, RVec3::ZERO);
        let car = cube(&mut world, 0.1, 1.0, RVec3::ZERO);
        let result = world.create_constraint(platform, car, &path(length));
        assert_eq!(result.is_ok(), accepted, "{length}: {result:?}");
    }
}

#[test]
fn far_and_light_constraint_points_are_refused() {
    // The three scenes in which a far point drove Jolt to a non-finite velocity in a release
    // build or to a failed assertion with the `asserts` feature.
    // Two 1 kg, 1 m cubes joined 4000 m away (ratio 1.9e8).
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    let first = cube(&mut world, 0.5, 1.0, RVec3::ZERO);
    let second = cube(&mut world, 0.5, 1.0, RVec3::new(2.0, 0.0, 0.0));
    assert!(constraint_invalid(world.create_constraint(
        first,
        second,
        &PointConstraintSettings::new(RVec3::new(2400.0, 3200.0, 0.0))
    )));
    // A 1 g, 6 cm cube hinged 116 m away to a 1000 kg plate (4.5e7).
    let plate = world
        .create_body(
            &Shape::new_box(Vec3::new(0.5, 0.05, 0.5)).unwrap(),
            &BodySettings::new_dynamic()
                .mass(1000.0)
                .position(RVec3::new(0.0, 20.0, 0.0)),
        )
        .unwrap();
    let light = cube(
        &mut world,
        0.03,
        limits::MIN_MASS,
        RVec3::new(2.250432, 21.509048, -2.93712),
    );
    let hinge = HingeConstraintSettings::new(
        RVec3::new(-96.0364, 77.2034, 37.558807),
        Vec3::new(0.8384718, 0.37976828, -0.39082107),
        Vec3::new(0.5449333, -0.57961416, 0.6058838),
    );
    assert!(constraint_invalid(
        world.create_constraint(plate, light, &hinge)
    ));
    // A weld 3 m from a 1 g and 1.2 m from a 10 g cube of 6 cm (3.2e4 and 4400).
    let heavier = cube(&mut world, 0.03, 0.01, RVec3::new(0.0, 40.0, 0.0));
    let lighter = cube(
        &mut world,
        0.03,
        limits::MIN_MASS,
        RVec3::new(0.5571059, 42.71482, -2.755086),
    );
    let weld = FixedConstraintSettings::new(
        RVec3::new(0.68242, 40.884994, -0.28849602),
        Vec3::new(0.63742137, 0.31528664, 0.70305645),
        Vec3::new(0.52797884, -0.8432883, -0.10051466),
    );
    assert!(constraint_invalid(
        world.create_constraint(heavier, lighter, &weld)
    ));
    assert_eq!(world.constraint_count(), 0);
}

/// `direction` scaled to `length`.
fn scaled(direction: Vec3, length: f32) -> Vec3 {
    let unit = length
        / (direction.x * direction.x + direction.y * direction.y + direction.z * direction.z)
            .sqrt();
    Vec3::new(direction.x * unit, direction.y * unit, direction.z * unit)
}

/// Holds a cube of `half` and `mass` at the origin by `settings(point)`, the point `lever`
/// away: from a static anchor, or from a cube of the same size and ten times the mass on the
/// far side of the point. The cube moves and spins at the velocity bounds under gravity for
/// 120 ticks, then steps at both time step extremes.
fn step_held_cube<S: ConstraintSettings>(
    what: &str,
    half: f32,
    mass: f32,
    anchored: bool,
    settings: impl Fn(RVec3) -> S,
) {
    let direction = Vec3::new(0.1, 0.65, -0.7);
    let lever = cube_lever(half, limits::MAX_LEVER_ARM_RATIO * 0.999);
    let point = at(direction, lever);
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    let other = if anchored {
        world
            .create_body(
                &sphere(),
                &BodySettings::new_static().position(at(direction, 3.0 * lever)),
            )
            .unwrap()
    } else {
        cube(&mut world, half, mass * 10.0, at(direction, 2.0 * lever))
    };
    let held = cube(&mut world, half, mass, RVec3::ZERO);
    world
        .create_constraint(other, held, &settings(point))
        .unwrap();
    let mut body = world.body_mut(held).unwrap();
    body.set_linear_velocity(scaled(
        Vec3::new(-0.45542818, 0.78363067, 0.42250222),
        limits::MAX_LINEAR_VELOCITY * 0.999,
    ))
    .unwrap();
    body.set_angular_velocity(scaled(
        Vec3::new(-0.59818655, -0.4488059, 0.6638871),
        limits::MAX_ANGULAR_VELOCITY * 0.999,
    ))
    .unwrap();
    let what = format!("{what}, cube of {half} m and {mass} kg");
    for tick in 0..120 {
        let _ = world.step(DT).unwrap();
        for id in [other, held] {
            assert_body_finite(&world, id, &format!("{what}, tick {tick}"));
        }
    }
    step_at_both_extremes(&mut world, &[other, held], &what);
}

#[test]
fn constraints_at_the_lever_arm_bound_step_finitely() {
    let axis = Vec3::new(0.63742137, 0.31528664, 0.70305645);
    let normal = Vec3::new(0.52797884, -0.8432883, -0.10051466);
    let locked = |point| {
        let mut settings = SixDofConstraintSettings::new(point, axis, normal);
        for which in [
            SixDofConstraintAxis::TranslationX,
            SixDofConstraintAxis::TranslationY,
            SixDofConstraintAxis::TranslationZ,
            SixDofConstraintAxis::RotationX,
            SixDofConstraintAxis::RotationY,
            SixDofConstraintAxis::RotationZ,
        ] {
            settings = settings.axis(which, SixDofAxis::Fixed);
        }
        settings
    };
    for (half, mass) in [(0.03, limits::MIN_MASS), (0.5, 1.0)] {
        step_held_cube("ball joint", half, mass, true, PointConstraintSettings::new);
        step_held_cube("hinge", half, mass, true, |p| {
            HingeConstraintSettings::new(p, axis, normal)
        });
        for anchored in [true, false] {
            step_held_cube("weld", half, mass, anchored, |p| {
                FixedConstraintSettings::new(p, axis, normal)
            });
            step_held_cube("locked six-DOF joint", half, mass, anchored, locked);
            step_held_cube(
                "swing-twist joint with zero ranges",
                half,
                mass,
                anchored,
                |p| SwingTwistConstraintSettings::new(p, axis, normal),
            );
        }
    }
}

fn path_point(position: [f32; 3], tangent: [f32; 3]) -> HermitePathPoint {
    HermitePathPoint {
        position: Vec3::from(position),
        tangent: Vec3::from(tangent),
    }
}

/// A straight path of one segment of `length` metres along x, in the plane of normal z.
fn straight_path(start: f32, length: f32) -> Result<HermitePath, ConstraintError> {
    HermitePath::new(
        Vec3::new(0.0, 0.0, 1.0),
        vec![
            path_point([start, 0.0, 0.0], [length, 0.0, 0.0]),
            path_point([start + length, 0.0, 0.0], [length, 0.0, 0.0]),
        ],
        false,
    )
}

#[test]
fn path_inputs_are_bounded() {
    let extent = limits::MAX_SHAPE_EXTENT;
    // Positions and tangents within the extent.
    assert!(straight_path(extent - 1.0, 1.0).is_ok());
    assert!(constraint_invalid(straight_path(extent, 1.0)));
    assert!(straight_path(0.0, extent).is_ok());
    assert!(constraint_invalid(straight_path(0.0, extent.next_up())));

    let path = straight_path(0.0, 1.0).unwrap();
    let settings = |f: &dyn Fn(PathConstraintSettings) -> PathConstraintSettings| {
        f(PathConstraintSettings::new(path.clone()))
    };
    // A kinematic body, so that the lever-arm check does not decide.
    let create = |settings: PathConstraintSettings| joined_kinematic_sphere(&settings);
    for offset in on_axes(extent) {
        assert!(create(settings(&|s| s.path_position(offset))).is_ok());
    }
    for offset in on_axes(extent.next_up()) {
        assert!(constraint_invalid(create(settings(
            &|s| s.path_position(offset)
        ))));
    }
    assert!(constraint_invalid(create(settings(&|s| {
        s.path_rotation(Quat::from_xyzw(0.0, 0.6, 0.0, 0.81))
    }))));
    for fraction in [0.0, 1.0] {
        assert!(create(settings(&|s| s.path_fraction(fraction))).is_ok());
    }
    for fraction in [-1.0e-6, 1.0_f32.next_up(), f32::NAN] {
        assert!(constraint_invalid(create(settings(
            &|s| s.path_fraction(fraction)
        ))));
    }
    assert!(constraint_invalid(create(settings(
        &|s| s.max_friction_force(-1.0)
    ))));

    let (mut world, _, id) = joined_sphere(1.0, &PathConstraintSettings::new(path.clone()));
    let id = id.unwrap();
    let reading = world.constraint(id).unwrap();
    assert!(reading
        .closest_fraction(Vec3::new(extent, 0.0, 0.0), 0.0)
        .is_ok());
    assert!(constraint_invalid(
        reading.closest_fraction(Vec3::new(extent.next_up(), 0.0, 0.0), 0.0)
    ));
    assert!(constraint_invalid(
        reading.closest_fraction(Vec3::ZERO, f32::NAN)
    ));
    let mut motor = world.constraint_mut(id).unwrap();
    let speed = limits::MAX_LINEAR_VELOCITY;
    for velocity in [speed, -speed] {
        assert!(motor.set_target_velocity(velocity).is_ok());
    }
    for velocity in [speed.next_up(), (-speed).next_down()] {
        assert!(constraint_invalid(motor.set_target_velocity(velocity)));
    }
    for fraction in [0.0, 1.0] {
        assert!(motor.set_target_path_fraction(fraction).is_ok());
    }
    for fraction in [-1.0e-6, 1.0_f32.next_up()] {
        assert!(constraint_invalid(motor.set_target_path_fraction(fraction)));
    }
    assert!(constraint_invalid(motor.set_max_friction_force(f32::NAN)));
}

#[test]
fn path_motor_springs_and_friction_are_bounded() {
    // As in `world_constraint_springs_are_bounded_by_the_bodies_effective_mass`.
    let mass = limits::MAX_MASS;
    let bound = 1.0 / f64::from(1.0 / mass);
    let fits = |frequency: f32| {
        let omega = 2.0 * std::f64::consts::PI * f64::from(frequency);
        bound * omega * omega <= f64::from(limits::MAX_SPRING_COEFFICIENT)
    };
    let mut frequency = ((f64::from(limits::MAX_SPRING_COEFFICIENT) / bound).sqrt()
        / (2.0 * std::f64::consts::PI)) as f32;
    while !fits(frequency) {
        frequency = frequency.next_down();
    }
    while fits(frequency.next_up()) {
        frequency = frequency.next_up();
    }
    let motor = |frequency| {
        MotorSettings::default().spring(SpringSettings::FrequencyAndDamping {
            frequency,
            damping: 0.0,
        })
    };
    let path = straight_path(0.0, 1.0).unwrap();
    let settings = PathConstraintSettings::new(path);
    assert!(
        joined_sphere(mass, &settings.clone().position_motor(motor(frequency)))
            .2
            .is_ok()
    );
    assert!(constraint_invalid(
        joined_sphere(
            mass,
            &settings.clone().position_motor(motor(frequency.next_up()))
        )
        .2
    ));
    let (mut world, _, id) = joined_sphere(mass, &settings);
    let mut path = world.constraint_mut(id.unwrap()).unwrap();
    assert!(path.set_position_motor_settings(motor(frequency)).is_ok());
    assert!(constraint_invalid(
        path.set_position_motor_settings(motor(frequency.next_up()))
    ));

    // Unlimited friction on a body sliding along the path at the velocity bound.
    for mass in [limits::MIN_MASS, limits::MAX_MASS] {
        let (mut world, body, id) =
            joined_sphere(mass, &settings.clone().max_friction_force(f32::MAX));
        world
            .constraint_mut(id.unwrap())
            .unwrap()
            .set_max_friction_force(f32::MAX)
            .unwrap();
        world
            .body_mut(body)
            .unwrap()
            .set_linear_velocity(Vec3::new(limits::MAX_LINEAR_VELOCITY, 0.0, 0.0))
            .unwrap();
        step_coupling(&mut world, &[body], &format!("path friction, mass {mass}"));
    }
}
