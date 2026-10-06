//! Structural changes of bodies: motion type changes, the movement capability of static bodies,
//! and the bodies they are refused for.

mod common;

use common::constraint_kinds::create_every_kind;
use common::controls::*;
use common::events::add_cloth;
use common::meshes::grid;
use common::ragdoll::{humanoid_settings, ragdoll_world};
use common::vehicle::{add_car, car_world};
use common::*;
use oxijolt::*;

const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);

/// A world without gravity that records activation events.
fn activation_world(gravity: Vec3) -> PhysicsWorld {
    let mut world = world(gravity, 1);
    world.set_event_settings(EventSettings::default().body_activation(true));
    world
}

/// A static unit cube at `position` in the moving layer that may become kinematic or dynamic.
fn add_movable_static(world: &mut PhysicsWorld, position: RVec3) -> BodyId {
    world
        .create_body(
            &cube_shape(),
            &BodySettings::new_static()
                .position(position)
                .object_layer(ObjectLayer::MOVING)
                .allow_dynamic_or_kinematic(true),
        )
        .unwrap()
}

/// What a refused change must leave as it was: the body's digest, the events since the last
/// call, and whether `saved` still restores.
fn assert_unchanged(world: &mut PhysicsWorld, id: BodyId, before: &[u8], saved: &WorldState) {
    let mut after = Vec::new();
    record_body(world, id, &mut after);
    assert_eq!(after, before);
    assert!(world.take_events().is_empty());
    world.restore_state(saved).unwrap();
}

#[test]
fn a_static_body_that_may_move_falls_when_made_dynamic() {
    let mut world = activation_world(GRAVITY);
    add_floor(&mut world);
    let block = add_movable_static(&mut world, RVec3::new(0.0, 3.0, 0.0));
    let body = world.body(block).unwrap();
    assert_eq!(body.motion_type(), MotionType::Static);
    assert!(body.can_be_kinematic_or_dynamic());
    assert_eq!(body.mass(), None);
    step(&mut world, 10);
    assert_eq!(
        world.body(block).unwrap().position(),
        RVec3::new(0.0, 3.0, 0.0)
    );
    world.take_events();
    world
        .body_mut(block)
        .unwrap()
        .set_motion_type(MotionType::Dynamic, Activation::Activate)
        .unwrap();
    assert_eq!(
        world.take_events().activations,
        [ActivationEvent::Activated(block)]
    );
    let body = world.body(block).unwrap();
    assert_eq!(body.motion_type(), MotionType::Dynamic);
    // The unit cube of water Jolt computed at creation.
    let mass = body.mass().unwrap();
    assert!((mass - 1000.0).abs() < 0.01, "{mass}");
    step(&mut world, 120);
    let y = world.body(block).unwrap().position().y;
    assert!((y - 0.5).abs() < 0.03, "{y}");
}

#[test]
fn a_static_body_without_the_flag_cannot_move() {
    let mut world = world(GRAVITY, 1);
    let wall = world
        .create_body(&cube_shape(), &BodySettings::new_static())
        .unwrap();
    assert!(!world.body(wall).unwrap().can_be_kinematic_or_dynamic());
    let saved = world.save_state();
    for motion_type in [MotionType::Kinematic, MotionType::Dynamic] {
        assert_eq!(
            world
                .body_mut(wall)
                .unwrap()
                .set_motion_type(motion_type, Activation::Activate),
            Err(BodyError::CannotMove(wall))
        );
    }
    world.restore_state(&saved).unwrap();
    // Making it static again is no change at all.
    world
        .body_mut(wall)
        .unwrap()
        .set_motion_type(MotionType::Static, Activation::Activate)
        .unwrap();
    world.restore_state(&saved).unwrap();
}

#[test]
fn a_movable_static_body_needs_zero_initial_velocities() {
    let mut world = world(GRAVITY, 1);
    let movable = BodySettings::new_static().allow_dynamic_or_kinematic(true);
    for settings in [
        movable.clone().linear_velocity(Vec3::new(1.0, 0.0, 0.0)),
        movable.clone().angular_velocity(Vec3::new(0.0, 1.0, 0.0)),
    ] {
        assert!(invalid(world.create_body(&cube_shape(), &settings)));
    }
    // Without the flag Jolt ignores a static body's velocities, as before.
    world
        .create_body(
            &cube_shape(),
            &BodySettings::new_static().linear_velocity(Vec3::new(1.0, 0.0, 0.0)),
        )
        .unwrap();
    world.create_body(&cube_shape(), &movable).unwrap();
}

#[test]
fn making_a_body_static_stops_and_deactivates_it() {
    let mut world = activation_world(Vec3::ZERO);
    let cube = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_dynamic().linear_velocity(Vec3::new(2.0, 0.0, 0.0)),
        )
        .unwrap();
    step(&mut world, 5);
    world.take_events();
    let mut body = world.body_mut(cube).unwrap();
    body.set_motion_type(MotionType::Static, Activation::Activate)
        .unwrap();
    assert_eq!(body.motion_type(), MotionType::Static);
    assert!(!body.is_active());
    assert_eq!(body.linear_velocity(), Vec3::ZERO);
    let at = body.position();
    assert_eq!(
        world.take_events().activations,
        [ActivationEvent::Deactivated(cube)]
    );
    step(&mut world, 10);
    assert_eq!(world.body(cube).unwrap().position(), at);
    // A body created dynamic may move again.
    world
        .body_mut(cube)
        .unwrap()
        .set_motion_type(MotionType::Dynamic, Activation::DontActivate)
        .unwrap();
    assert!(world.body(cube).unwrap().is_sleeping());
    assert!(world.take_events().is_empty());
}

#[test]
fn dynamic_to_kinematic_drops_forces() {
    let mut world = world(Vec3::ZERO, 1);
    let cube = add_cube(&mut world, RVec3::ZERO);
    let mut body = world.body_mut(cube).unwrap();
    body.add_force(Vec3::new(100.0, 0.0, 0.0)).unwrap();
    body.set_motion_type(MotionType::Kinematic, Activation::Activate)
        .unwrap();
    body.set_motion_type(MotionType::Dynamic, Activation::Activate)
        .unwrap();
    step(&mut world, 1);
    assert_eq!(world.body(cube).unwrap().linear_velocity(), Vec3::ZERO);
}

#[test]
fn kinematic_mesh_body_cannot_become_dynamic() {
    let mut world = world(Vec3::ZERO, 1);
    let (vertices, triangles) = grid(2, 1.0, |_, _| 0.0);
    let (mesh, _) = Shape::new_mesh(&vertices, &triangles).unwrap();
    let platform = world
        .create_body(&mesh, &BodySettings::new_kinematic().mass(10.0))
        .unwrap();
    let saved = world.save_state();
    let mut before = Vec::new();
    record_body(&world, platform, &mut before);
    assert!(invalid(
        world
            .body_mut(platform)
            .unwrap()
            .set_motion_type(MotionType::Dynamic, Activation::Activate)
    ));
    assert_unchanged(&mut world, platform, &before, &saved);
    // Static and back to kinematic are allowed.
    let mut body = world.body_mut(platform).unwrap();
    body.set_motion_type(MotionType::Static, Activation::Activate)
        .unwrap();
    body.set_motion_type(MotionType::Kinematic, Activation::Activate)
        .unwrap();
}

#[test]
fn light_kinematic_body_cannot_become_dynamic() {
    let mut world = world(Vec3::ZERO, 1);
    // Jolt's density 1000 kg/m³: a 12 m cube weighs 1.7e6 kg and a 8 mm cube 5e-4 kg, outside
    // the dynamic range, which kinematic bodies are exempt from at creation.
    let heavy = Shape::new_box(Vec3::new(6.0, 6.0, 6.0)).unwrap();
    let light = Shape::new_box_with_convex_radius(Vec3::new(0.004, 0.004, 0.004), 0.0).unwrap();
    for (i, shape) in [&heavy, &light].into_iter().enumerate() {
        let id = world
            .create_body(
                shape,
                &BodySettings::new_kinematic().position(RVec3::new(20.0 * i as Real, 0.0, 0.0)),
            )
            .unwrap();
        let saved = world.save_state();
        let mut before = Vec::new();
        record_body(&world, id, &mut before);
        assert!(invalid(
            world
                .body_mut(id)
                .unwrap()
                .set_motion_type(MotionType::Dynamic, Activation::Activate)
        ));
        assert_unchanged(&mut world, id, &before, &saved);
    }
    // The bounds themselves are accepted.
    for (i, mass) in [limits::MIN_MASS, limits::MAX_MASS].into_iter().enumerate() {
        let id = world
            .create_body(
                &cube_shape(),
                &BodySettings::new_kinematic()
                    .mass(mass)
                    .position(RVec3::new(50.0 + 5.0 * i as Real, 0.0, 0.0)),
            )
            .unwrap();
        world
            .body_mut(id)
            .unwrap()
            .set_motion_type(MotionType::Dynamic, Activation::Activate)
            .unwrap();
    }
    step(&mut world, 5);
}

#[test]
fn same_motion_type_changes_nothing() {
    let mut world = activation_world(GRAVITY);
    add_floor(&mut world);
    let cube = add_cube(&mut world, RVec3::new(0.0, 0.5, 0.0));
    step(&mut world, 2);
    world.take_events();
    let saved = world.save_state();
    let mut before = Vec::new();
    record_body(&world, cube, &mut before);
    world
        .body_mut(cube)
        .unwrap()
        .set_motion_type(MotionType::Dynamic, Activation::Activate)
        .unwrap();
    assert_unchanged(&mut world, cube, &before, &saved);
}

#[test]
fn motion_type_changes_refuse_owned_bodies() {
    let mut world = world(Vec3::ZERO, 1);
    let first = add_cube(&mut world, RVec3::ZERO);
    let second = add_cube(&mut world, RVec3::new(1.5, 0.0, 0.0));
    let created = create_every_kind(&mut world, first, second);
    assert!(created.iter().all(Result::is_ok), "{created:?}");
    step(&mut world, 1);
    world.set_event_settings(EventSettings::default().body_activation(true));
    let saved = world.save_state();
    for id in [first, second] {
        let mut before = Vec::new();
        record_body(&world, id, &mut before);
        for motion_type in [
            MotionType::Static,
            MotionType::Kinematic,
            MotionType::Dynamic,
        ] {
            assert_eq!(
                world
                    .body_mut(id)
                    .unwrap()
                    .set_motion_type(motion_type, Activation::Activate),
                Err(BodyError::UsedByConstraint(id))
            );
        }
        assert_unchanged(&mut world, id, &before, &saved);
    }

    let (mut world, layers) = ragdoll_world(1);
    let ragdoll = world
        .create_ragdoll(
            &humanoid_settings(layers.ragdoll),
            None,
            Activation::Activate,
        )
        .unwrap();
    let part = world.ragdoll(ragdoll).unwrap().body_ids()[0];
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
    let cloth = add_cloth(&mut world, RVec3::new(-5.0, 1.0, 0.0), Quat::IDENTITY);
    let saved = world.save_state();
    for (id, error) in [
        (part, BodyError::OwnedByRagdoll(part)),
        (inner, BodyError::OwnedByCharacter(inner)),
        (cloth, BodyError::NotRigidBody(cloth)),
    ] {
        for motion_type in [
            MotionType::Static,
            MotionType::Kinematic,
            MotionType::Dynamic,
        ] {
            assert_eq!(
                world
                    .body_mut(id)
                    .unwrap()
                    .set_motion_type(motion_type, Activation::Activate),
                Err(error)
            );
        }
    }
    world.restore_state(&saved).unwrap();

    let (mut world, layers) = car_world(GRAVITY, 1);
    let (chassis, _) = add_car(
        &mut world,
        &layers,
        RVec3::new(0.0, 2.0, 0.0),
        Quat::IDENTITY,
    );
    for motion_type in [MotionType::Static, MotionType::Dynamic] {
        assert_eq!(
            world
                .body_mut(chassis)
                .unwrap()
                .set_motion_type(motion_type, Activation::Activate),
            Err(BodyError::UsedByVehicle(chassis))
        );
    }
}

#[test]
fn removing_the_constraint_releases_the_guard() {
    let mut world = world(Vec3::ZERO, 1);
    let first = add_cube(&mut world, RVec3::ZERO);
    let second = add_cube(&mut world, RVec3::new(1.5, 0.0, 0.0));
    let rope = world
        .create_constraint(
            first,
            second,
            &DistanceConstraintSettings::new(RVec3::ZERO, RVec3::new(1.5, 0.0, 0.0)),
        )
        .unwrap();
    assert_eq!(
        world
            .body_mut(second)
            .unwrap()
            .set_motion_type(MotionType::Kinematic, Activation::Activate),
        Err(BodyError::UsedByConstraint(second))
    );
    world.remove_constraint(rope).unwrap();
    world
        .body_mut(second)
        .unwrap()
        .set_motion_type(MotionType::Kinematic, Activation::Activate)
        .unwrap();
    assert_eq!(
        world.body(second).unwrap().motion_type(),
        MotionType::Kinematic
    );
}

#[test]
fn a_static_body_that_may_move_joins_a_fixed_constraint_with_an_automatic_point() {
    // Jolt weighs the automatic point by the inverse masses of every body that has motion
    // properties, a static one that may move included; reading them through Jolt's checked
    // getter asserts on such a body.
    let mut world = world(GRAVITY, 1);
    let post = add_movable_static(&mut world, RVec3::ZERO);
    let cube = add_cube(&mut world, RVec3::new(1.2, 0.0, 0.0));
    world
        .create_constraint(
            post,
            cube,
            &FixedConstraintSettings::default().auto_detect_point(true),
        )
        .unwrap();
    step(&mut world, 30);
    let p = world.body(cube).unwrap().position();
    assert!((p.y - 0.0).abs() < 0.05, "{p:?}");
}

#[test]
fn allow_dynamic_or_kinematic_pays_the_moving_rules_at_creation() {
    let mut world = world(GRAVITY, 1);
    let movable = BodySettings::new_static().allow_dynamic_or_kinematic(true);
    assert!(invalid(world.create_body(&flat_height_field(), &movable)));
    let (vertices, triangles) = grid(2, 1.0, |_, _| 0.0);
    let (mesh, _) = Shape::new_mesh(&vertices, &triangles).unwrap();
    assert!(invalid(world.create_body(&mesh, &movable)));
    // Its computed mass has no finite inverse.
    let tiny = Shape::new_sphere(1.0e-20).unwrap();
    assert!(invalid(world.create_body(&tiny, &movable)));
    assert_eq!(world.body_count(), 0);
    // A mesh with a mass may become kinematic; restricted DOFs are allowed on such a body.
    let platform = world
        .create_body(
            &mesh,
            &movable
                .clone()
                .mass(50.0)
                .allowed_dofs(AllowedDofs::TRANSLATION_Y),
        )
        .unwrap();
    assert_eq!(
        world.body(platform).unwrap().allowed_dofs(),
        AllowedDofs::TRANSLATION_Y
    );
    world
        .body_mut(platform)
        .unwrap()
        .set_motion_type(MotionType::Kinematic, Activation::Activate)
        .unwrap();
    world
        .create_body(&flat_height_field(), &BodySettings::new_static())
        .unwrap();
    step(&mut world, 5);
}
