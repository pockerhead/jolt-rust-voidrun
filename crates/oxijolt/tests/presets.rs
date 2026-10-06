//! Settings presets: a humanoid character standing and walking on a floor.

mod common;

use common::*;
use oxijolt::*;

const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);

/// One character update with horizontal `velocity` plus 1 m/s down, under gravity.
fn walk_tick(world: &mut PhysicsWorld, id: CharacterId, velocity: Vec3) {
    world
        .character_mut(id)
        .unwrap()
        .set_linear_velocity(Vec3::new(velocity.x, velocity.y - 1.0, velocity.z))
        .unwrap();
    world
        .update_character(
            id,
            DT,
            GRAVITY,
            &ExtendedUpdateSettings::default(),
            &QueryFilter::new(),
        )
        .unwrap();
}

#[test]
fn a_humanoid_stands_on_a_floor_and_walks() {
    let mut world = world(GRAVITY, 1);
    add_floor(&mut world);
    let settings = CharacterSettings::humanoid(1.8, 0.3).unwrap();
    let id = world
        .create_character(&settings, RVec3::ZERO, Quat::IDENTITY)
        .unwrap();
    world
        .refresh_character_contacts(id, &QueryFilter::new())
        .unwrap();
    assert_eq!(
        world.character(id).unwrap().ground_state(),
        GroundState::OnGround
    );

    for _ in 0..60 {
        walk_tick(&mut world, id, Vec3::new(1.0, 0.0, 0.0));
    }
    let character = world.character(id).unwrap();
    assert_eq!(character.ground_state(), GroundState::OnGround);
    let position = character.position();
    assert!(
        (position.x - 1.0).abs() < 0.02,
        "walked to x = {}",
        position.x
    );
    assert!(
        position.y.abs() < 0.05,
        "stays on the floor at y = {}",
        position.y
    );
}

#[test]
fn humanoid_settings_outlive_their_clones() {
    let mut world = world(GRAVITY, 1);
    add_floor(&mut world);
    let original = CharacterSettings::humanoid(1.8, 0.3).unwrap();
    let clone = original.clone();
    drop(original);
    let first = world
        .create_character(&clone, RVec3::ZERO, Quat::IDENTITY)
        .unwrap();
    let second_clone = clone.clone();
    drop(clone);
    let second = world
        .create_character(&second_clone, RVec3::new(3.0, 0.0, 0.0), Quat::IDENTITY)
        .unwrap();
    drop(second_clone);
    for _ in 0..60 {
        walk_tick(&mut world, first, Vec3::new(0.0, 0.0, 1.0));
        walk_tick(&mut world, second, Vec3::new(0.0, 0.0, -1.0));
    }
    for id in [first, second] {
        assert_eq!(
            world.character(id).unwrap().ground_state(),
            GroundState::OnGround
        );
    }
}

#[test]
fn a_humanoid_accepts_a_borrowed_inner_body() {
    let mut world = world(GRAVITY, 1);
    add_floor(&mut world);
    let bodies = world.body_count();
    {
        let inner_shape = Shape::new_capsule(0.6, 0.3).unwrap();
        let settings = CharacterSettings::humanoid(1.8, 0.3)
            .unwrap()
            .inner_body(Some(InnerBody {
                shape: &inner_shape,
                object_layer: ObjectLayer::MOVING,
            }));
        let id = world
            .create_character(&settings, RVec3::ZERO, Quat::IDENTITY)
            .unwrap();
        assert_eq!(world.body_count(), bodies + 1);
        world.remove_character(id).unwrap();
    }
    assert_eq!(world.body_count(), bodies);
}
