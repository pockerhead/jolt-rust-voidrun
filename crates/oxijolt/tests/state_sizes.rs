//! What each object adds to a world state: `WorldState::data_size` for an empty world, static
//! and moving bodies, a resting contact and a hinge, and the length of a character's state, in
//! the precision of the build (`Real` is `f32` or `f64`). The byte counts come from Jolt's
//! `SaveState` functions, which write a `Vec3` as three floats, an `RVec3` as three `Real`s and
//! other values at their size; docs/state.md lists them for both precisions.

mod common;

use common::{add_floor, step, DT};
use oxijolt::*;

/// The size of one `Real` in Jolt's stream.
fn real() -> usize {
    size_of::<Real>()
}

/// An empty world: the saved parts flag (1), the previous step's delta time and the gravity
/// (4 + 12), the body count (4), the contact pair and continuous contact counts (4 + 4) and the
/// constraint count (4).
const EMPTY_WORLD: usize = 1 + 4 + 12 + 4 + 4 + 4 + 4;

/// A static body: its id (4), active flag (1), position and rotation.
fn static_body() -> usize {
    4 + 1 + 3 * real() + 16
}

/// A body with motion properties (dynamic, kinematic, asleep or awake): a static body's bytes,
/// linear and angular velocity, force and torque (4 * 12), the sleep test offset in double
/// precision (24), three sleep test spheres (3 * 16), the sleep timer (4) and the allow-sleep
/// flag (1).
fn moving_body() -> usize {
    let sleep_test_offset = if real() == 8 { 24 } else { 0 };
    static_body() + 4 * 12 + sleep_test_offset + 3 * 16 + 4 + 1
}

/// A contact pair with one manifold: the body pair key (8), its relative pose (12 + 12), the
/// manifold count (4), and the manifold: its key (16), point count (2), normal (12) and friction
/// impulses (8 + 4).
const CONTACT_PAIR_WITH_ONE_MANIFOLD: usize = 8 + 12 + 12 + 4 + 16 + 2 + 12 + 8 + 4;
/// A contact point in a manifold: both positions (12 + 12) and the normal impulse (4).
const CONTACT_POINT: usize = 12 + 12 + 4;

/// A hinge: its index (4), enabled flag (1), the impulses of its motor (4), rotation (8), point
/// (12) and limits (4) parts, the motor state (an `int` enum, 4) and the target velocity and
/// angle (4 + 4).
const HINGE: usize = 4 + 1 + 4 + 8 + 12 + 4 + 4 + 4 + 4;

/// A character's own stream without contacts: ground state (an `int` enum, 4), ground body and
/// sub-shape
/// (4 + 4), ground position, normal and velocity, position, rotation (16), velocity (12), last
/// delta time (4), the max-hits flag (1) and the contact count (4).
fn character_stream() -> usize {
    4 + 4 + 4 + 3 * real() + 12 + 12 + 3 * real() + 16 + 12 + 4 + 1 + 4
}

/// A character contact with collision: body, character and sub-shape ids (12), position,
/// velocity, contact and surface normals (3 * 12), distance and fraction (4 + 4) and five
/// one-byte fields.
fn character_contact() -> usize {
    12 + 3 * real() + 3 * 12 + 4 + 4 + 5
}

fn data_size(world: &PhysicsWorld) -> usize {
    world.save_state().data_size()
}

fn empty_world() -> PhysicsWorld {
    PhysicsWorld::new(WorldSettings::default()).unwrap()
}

fn cube() -> Shape {
    Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap()
}

#[test]
fn an_empty_world_holds_the_counts() {
    assert_eq!(data_size(&empty_world()), EMPTY_WORLD);
}

#[test]
fn a_static_body_adds_its_id_flag_and_pose() {
    let mut world = empty_world();
    add_floor(&mut world);
    assert_eq!(data_size(&world), EMPTY_WORLD + static_body());
}

#[test]
fn every_body_that_can_move_adds_the_same_bytes() {
    let at = |x| RVec3::new(x, 10.0, 0.0);
    for settings in [
        BodySettings::new_dynamic().position(at(0.0)),
        BodySettings::new_kinematic().position(at(3.0)),
    ] {
        let mut world = empty_world();
        let id = world.create_body(&cube(), &settings).unwrap();
        assert!(world.body(id).unwrap().is_active());
        assert_eq!(data_size(&world), EMPTY_WORLD + moving_body());
        world.body_mut(id).unwrap().deactivate().unwrap();
        assert!(!world.body(id).unwrap().is_active());
        assert_eq!(data_size(&world), EMPTY_WORLD + moving_body());
    }
}

#[test]
fn a_resting_cube_adds_one_contact_pair_with_four_points() {
    let mut world = empty_world();
    add_floor(&mut world);
    world
        .create_body(
            &cube(),
            &BodySettings::new_dynamic().position(RVec3::new(0.0, 0.5, 0.0)),
        )
        .unwrap();
    let bodies = EMPTY_WORLD + static_body() + moving_body();
    assert_eq!(data_size(&world), bodies);
    step(&mut world, 1);
    assert_eq!(
        data_size(&world),
        bodies + CONTACT_PAIR_WITH_ONE_MANIFOLD + 4 * CONTACT_POINT
    );
}

#[test]
fn a_hinge_adds_its_index_and_solver_state() {
    let mut world = empty_world();
    let settings = BodySettings::new_dynamic().gravity_factor(0.0);
    let a = world
        .create_body(
            &cube(),
            &settings.clone().position(RVec3::new(0.0, 0.0, 0.0)),
        )
        .unwrap();
    let b = world
        .create_body(&cube(), &settings.position(RVec3::new(3.0, 0.0, 0.0)))
        .unwrap();
    let bodies = EMPTY_WORLD + 2 * moving_body();
    assert_eq!(data_size(&world), bodies);
    world
        .create_constraint(
            a,
            b,
            &HingeConstraintSettings::new(
                RVec3::new(1.5, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
                Vec3::new(1.0, 0.0, 0.0),
            ),
        )
        .unwrap();
    assert_eq!(data_size(&world), bodies + HINGE);
}

/// The length of a character's stream: `CharacterState::as_bytes` without up's 12 bytes.
fn character_stream_len(world: &PhysicsWorld, id: CharacterId) -> usize {
    world.character(id).unwrap().save_state().to_bytes().len() - 12
}

#[test]
fn a_character_adds_its_inner_body_and_keeps_its_contacts_in_its_own_state() {
    let mut world = empty_world();
    let capsule = Shape::new_capsule(0.7, 0.4).unwrap();
    let settings = CharacterSettings::new(&capsule)
        .shape_offset(Vec3::new(0.0, 1.1, 0.0))
        .inner_body(Some(InnerBody {
            shape: &capsule,
            object_layer: ObjectLayer::MOVING,
        }));
    let id = world
        .create_character(&settings, RVec3::new(0.0, 5.0, 0.0), Quat::IDENTITY)
        .unwrap();
    assert_eq!(data_size(&world), EMPTY_WORLD + moving_body());
    assert_eq!(character_stream_len(&world, id), character_stream());

    add_floor(&mut world);
    let mut character = world.character_mut(id).unwrap();
    character.set_position(RVec3::ZERO).unwrap();
    for _ in 0..3 {
        world
            .update_character(
                id,
                DT,
                Vec3::new(0.0, -9.81, 0.0),
                &ExtendedUpdateSettings::default(),
                &QueryFilter::new(),
            )
            .unwrap();
    }
    assert_eq!(
        character_stream_len(&world, id),
        character_stream() + character_contact()
    );
}
