//! Body controls: locked axes (allowed degrees of freedom) and where they are refused.

mod common;

use common::constraint_kinds::create_every_kind;
use common::ragdoll::{conj, humanoid_parts, mul, part_shapes, ragdoll_world, skeleton};
use common::vehicle::{add_car, car_settings, car_world, chassis_settings, chassis_shape};
use common::*;
use oxijolt::*;

const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);
const Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);

/// A dynamic unit cube at `position` with `dofs`.
fn add_locked_cube(world: &mut PhysicsWorld, position: RVec3, dofs: AllowedDofs) -> BodyId {
    world
        .create_body(
            &cube_shape(),
            &BodySettings::new_dynamic()
                .position(position)
                .allowed_dofs(dofs),
        )
        .unwrap()
}

#[test]
fn a_plane_2d_body_stays_in_its_plane() {
    let mut world = world(GRAVITY, 1);
    // A static slab tilted about a diagonal axis, so its normal leans along z as well as x.
    let slab = Shape::new_box(Vec3::new(5.0, 0.5, 5.0)).unwrap();
    let tilt = quat_about(Vec3::new(0.6, 0.0, 0.8), 0.4);
    world
        .create_body(
            &slab,
            &BodySettings::new_static()
                .position(RVec3::new(0.0, -1.0, 0.0))
                .rotation(tilt),
        )
        .unwrap();
    let cube = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_dynamic()
                .position(RVec3::new(0.5, 2.0, 0.0))
                .linear_velocity(Vec3::new(1.0, -2.0, 3.0))
                .angular_velocity(Vec3::new(2.0, -1.0, 0.5))
                .allowed_dofs(AllowedDofs::PLANE_2D),
        )
        .unwrap();
    let start = world.body(cube).unwrap().position();
    let mut turned = false;
    for _ in 0..120 {
        step(&mut world, 1);
        let body = world.body(cube).unwrap();
        let (p, q) = (body.position(), body.rotation());
        let (v, w) = (body.linear_velocity(), body.angular_velocity());
        assert_eq!(p.z, 0.0);
        assert_eq!((q.x, q.y), (0.0, 0.0));
        assert_eq!(v.z, 0.0);
        assert_eq!((w.x, w.y), (0.0, 0.0));
        turned |= q.z != 0.0;
    }
    let end = world.body(cube).unwrap().position();
    assert!(turned, "the cube never turned about z");
    assert!(end.x != start.x && end.y < start.y, "{end:?}");
}

#[test]
fn axes_stay_world_axes_for_a_rotated_body() {
    let mut world = world(Vec3::ZERO, 1);
    // The body's own z axis points along world -y.
    let start = quat_about(Vec3::new(1.0, 0.0, 0.0), std::f32::consts::FRAC_PI_2);
    let translations =
        AllowedDofs::TRANSLATION_X | AllowedDofs::TRANSLATION_Y | AllowedDofs::TRANSLATION_Z;
    let cube = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_dynamic()
                .rotation(start)
                .allowed_dofs(translations | AllowedDofs::ROTATION_Z),
        )
        .unwrap();
    world
        .body_mut(cube)
        .unwrap()
        .set_angular_velocity(Vec3::new(1.0, 1.0, 1.0))
        .unwrap();
    assert_eq!(
        world.body(cube).unwrap().angular_velocity(),
        Vec3::new(0.0, 0.0, 1.0)
    );
    step(&mut world, 30);
    // The turn since the start is about world z, not about the body's own z.
    let q = world.body(cube).unwrap().rotation();
    let turn = mul(q, conj(start));
    assert!(turn.x.abs() < 1.0e-6 && turn.y.abs() < 1.0e-6, "{turn:?}");
    assert!(turn.z.abs() > 0.2, "{turn:?}");
}

#[test]
fn allowed_dofs_read_back() {
    let mut world = world(Vec3::ZERO, 1);
    let plane = add_locked_cube(&mut world, RVec3::ZERO, AllowedDofs::PLANE_2D);
    let rail = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_kinematic()
                .position(RVec3::new(3.0, 0.0, 0.0))
                .allowed_dofs(AllowedDofs::TRANSLATION_Y),
        )
        .unwrap();
    let free = add_cube(&mut world, RVec3::new(6.0, 0.0, 0.0));
    let fixed = world
        .create_body(&cube_shape(), &BodySettings::new_static())
        .unwrap();
    let dofs = |id| world.body(id).unwrap().allowed_dofs();
    assert_eq!(dofs(plane), AllowedDofs::PLANE_2D);
    assert_eq!(dofs(rail), AllowedDofs::TRANSLATION_Y);
    assert_eq!(dofs(free), AllowedDofs::ALL);
    assert_eq!(dofs(fixed), AllowedDofs::ALL);
}

#[test]
fn a_static_body_that_cannot_move_refuses_restricted_dofs() {
    let mut world = world(Vec3::ZERO, 1);
    let refused = world.create_body(
        &cube_shape(),
        &BodySettings::new_static().allowed_dofs(AllowedDofs::PLANE_2D),
    );
    assert!(
        matches!(refused, Err(BodyError::InvalidValue(_))),
        "{refused:?}"
    );
    let without_translation = world.create_body(
        &cube_shape(),
        &BodySettings::new_dynamic().allowed_dofs(AllowedDofs::ROTATION_Y),
    );
    assert!(
        matches!(without_translation, Err(BodyError::InvalidValue(_))),
        "{without_translation:?}"
    );
    assert_eq!(world.body_count(), 0);
}

#[test]
fn bodies_with_restricted_dofs_are_refused_by_every_constraint_kind_vehicles_and_ragdolls() {
    let mut world = world(Vec3::ZERO, 1);
    let locked = add_locked_cube(&mut world, RVec3::ZERO, AllowedDofs::PLANE_2D);
    let free = add_cube(&mut world, RVec3::new(1.5, 0.0, 0.0));
    for (body1, body2) in [(locked, free), (free, locked)] {
        let results = create_every_kind(&mut world, body1, body2);
        assert_eq!(results.len(), 12);
        for result in results {
            assert_eq!(
                result,
                Err(ConstraintError::Body(BodyError::RestrictedDofs(locked)))
            );
        }
    }
    assert_eq!(world.constraint_count(), 0);
    // The same two bodies with all six degrees of freedom take every kind.
    let other = add_cube(&mut world, RVec3::new(0.0, 0.0, 3.0));
    let accepted = create_every_kind(&mut world, other, free);
    assert!(accepted.iter().all(Result::is_ok), "{accepted:?}");
    step(&mut world, 2);

    let (mut world, layers) = car_world(GRAVITY, 1);
    let chassis = world
        .create_body(
            &chassis_shape(),
            &chassis_settings(&layers, RVec3::new(0.0, 2.0, 0.0), Quat::IDENTITY)
                .allowed_dofs(AllowedDofs::PLANE_2D),
        )
        .unwrap();
    let settings = car_settings(VehicleCollisionTester::ray(layers.probe));
    assert_eq!(
        world.create_vehicle(chassis, &settings),
        Err(VehicleError::Body(BodyError::RestrictedDofs(chassis)))
    );
    assert_eq!(world.vehicle_ids().count(), 0);
    add_car(
        &mut world,
        &layers,
        RVec3::new(5.0, 2.0, 0.0),
        Quat::IDENTITY,
    );

    let (_, layers) = ragdoll_world(1);
    let shapes = part_shapes();
    let mut parts = humanoid_parts(&shapes, layers.ragdoll);
    parts[1].body = parts[1].body.clone().allowed_dofs(AllowedDofs::PLANE_2D);
    assert!(matches!(
        RagdollSettings::new(&skeleton(), &parts),
        Err(RagdollError::InvalidValue(_))
    ));
}

/// The pose and velocities of `id`.
fn motion(world: &PhysicsWorld, id: BodyId) -> (RVec3, Quat, Vec3, Vec3) {
    let body = world.body(id).unwrap();
    (
        body.position(),
        body.rotation(),
        body.linear_velocity(),
        body.angular_velocity(),
    )
}

#[test]
fn rotated_rebase_refuses_restricted_dofs_and_changes_nothing() {
    let mut world = world(Vec3::ZERO, 1);
    let free = add_cube(&mut world, RVec3::new(-3.0, 0.0, 0.0));
    let rail = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_dynamic()
                .linear_velocity(Vec3::new(1.0, 0.0, 0.0))
                .allowed_dofs(AllowedDofs::TRANSLATION_X),
        )
        .unwrap();
    step(&mut world, 1);
    let saved = world.save_state();
    let before = [motion(&world, free), motion(&world, rail)];
    let quarter_turn = quat_about(Z, std::f32::consts::FRAC_PI_2);
    assert_eq!(
        world.rebase(&[free, rail], quarter_turn, RVec3::ZERO),
        Err(BodyError::RestrictedDofs(rail))
    );
    assert_eq!([motion(&world, free), motion(&world, rail)], before);
    assert_eq!(world.gravity(), Vec3::ZERO);
    world.restore_state(&saved).unwrap();
}

#[test]
fn translation_rebase_keeps_restricted_dofs() {
    let mut world = world(Vec3::ZERO, 1);
    let rail = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_dynamic()
                .linear_velocity(Vec3::new(1.0, 0.0, 0.0))
                .allowed_dofs(AllowedDofs::TRANSLATION_X),
        )
        .unwrap();
    world
        .rebase(&[rail], Quat::IDENTITY, RVec3::new(10.0, 2.0, -3.0))
        .unwrap();
    let body = world.body(rail).unwrap();
    assert_eq!(body.position(), RVec3::new(10.0, 2.0, -3.0));
    assert_eq!(body.linear_velocity(), Vec3::new(1.0, 0.0, 0.0));
    assert_eq!(body.allowed_dofs(), AllowedDofs::TRANSLATION_X);
    step(&mut world, 10);
    let p = world.body(rail).unwrap().position();
    assert!(p.x > 10.1 && p.y == 2.0 && p.z == -3.0, "{p:?}");
}

#[test]
fn remove_body_guards_still_hold() {
    let (mut world, layers) = ragdoll_world(1);
    let ragdoll = world
        .create_ragdoll(
            &common::ragdoll::humanoid_settings(layers.ragdoll),
            None,
            Activation::Activate,
        )
        .unwrap();
    let part = world.ragdoll(ragdoll).unwrap().body_ids()[0];
    assert_eq!(
        world.remove_body(part),
        Err(BodyError::OwnedByRagdoll(part))
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
        world.remove_body(inner),
        Err(BodyError::OwnedByCharacter(inner))
    );

    let anchor = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_static().position(RVec3::new(-5.0, 3.0, 0.0)),
        )
        .unwrap();
    let cube = add_cube(&mut world, RVec3::new(-5.0, 1.0, 0.0));
    world
        .create_constraint(
            anchor,
            cube,
            &PointConstraintSettings::new(RVec3::new(-5.0, 2.0, 0.0)),
        )
        .unwrap();
    for body in [anchor, cube] {
        assert_eq!(
            world.remove_body(body),
            Err(BodyError::UsedByConstraint(body))
        );
    }

    let (mut world, layers) = car_world(GRAVITY, 1);
    let (chassis, _) = add_car(
        &mut world,
        &layers,
        RVec3::new(0.0, 2.0, 0.0),
        Quat::IDENTITY,
    );
    assert_eq!(
        world.remove_body(chassis),
        Err(BodyError::UsedByVehicle(chassis))
    );
}
