//! The penetration recovery speed of a character, set while it runs.

mod common;

use common::math::v3;
use common::DT;
use oxijolt::*;

const RADIUS: f32 = 0.4;
const HALF_HEIGHT: f32 = 0.70845;
/// How far the capsule reaches into the wall at the start, metres.
const OVERLAP: f32 = 0.05;

/// A world without gravity, with a floor whose top is at y = 0 and a sharp wall whose face at
/// `x = RADIUS - OVERLAP` cuts into a character standing at the origin.
fn wall_scene() -> (PhysicsWorld, CharacterId) {
    let mut world = common::world(Vec3::ZERO, 1);
    common::add_floor(&mut world);
    let wall = Shape::new_box_with_convex_radius(Vec3::new(0.1, 1.0, 2.0), 0.0).unwrap();
    let face = RADIUS - OVERLAP;
    world
        .create_body(
            &wall,
            &BodySettings::new_static().position(RVec3::new(face + 0.1, 1.0, 0.0)),
        )
        .unwrap();
    let capsule = Shape::new_capsule(HALF_HEIGHT, RADIUS).unwrap();
    let settings = CharacterSettings::new(&capsule)
        .shape_offset(Vec3::new(0.0, HALF_HEIGHT + RADIUS, 0.0))
        .penetration_recovery_speed(0.0);
    let id = world
        .create_character(&settings, RVec3::ZERO, Quat::IDENTITY)
        .unwrap();
    world
        .refresh_character_contacts(id, &QueryFilter::new())
        .unwrap();
    (world, id)
}

/// One update without velocity, stick to floor or stairs.
fn still_update(world: &mut PhysicsWorld, id: CharacterId) {
    world
        .character_mut(id)
        .unwrap()
        .set_linear_velocity(Vec3::ZERO)
        .unwrap();
    let extended = ExtendedUpdateSettings::default()
        .stick_to_floor_step_down(Vec3::ZERO)
        .walk_stairs_step_up(Vec3::ZERO);
    world
        .update_character(
            id,
            DT,
            Vec3::new(0.0, -9.8, 0.0),
            &extended,
            &QueryFilter::new(),
        )
        .unwrap();
}

fn x_of(world: &PhysicsWorld, id: CharacterId) -> f64 {
    v3(world.character(id).unwrap().position())[0]
}

#[test]
fn penetration_recovery_speed_set_at_runtime_decides_the_push() {
    let (mut world, id) = wall_scene();
    assert_eq!(
        world.character(id).unwrap().penetration_recovery_speed(),
        0.0
    );
    let start = x_of(&world, id);
    still_update(&mut world, id);
    let held = x_of(&world, id);
    assert!(
        (held - start).abs() <= 1e-6,
        "moved {} with speed 0",
        held - start
    );

    world
        .character_mut(id)
        .unwrap()
        .set_penetration_recovery_speed(1.0)
        .unwrap();
    assert_eq!(
        world.character(id).unwrap().penetration_recovery_speed(),
        1.0
    );
    still_update(&mut world, id);
    let pushed = held - x_of(&world, id);
    assert!(pushed >= 0.04, "pushed {pushed} away from the wall");

    for invalid in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY, -0.1, 1.1] {
        let result = world
            .character_mut(id)
            .unwrap()
            .set_penetration_recovery_speed(invalid);
        assert!(
            matches!(result, Err(CharacterError::InvalidValue(_))),
            "{invalid}: {result:?}"
        );
        assert_eq!(
            world.character(id).unwrap().penetration_recovery_speed(),
            1.0
        );
    }

    let state = world.character(id).unwrap().save_state();
    let mut character = world.character_mut(id).unwrap();
    character.set_penetration_recovery_speed(0.3).unwrap();
    character.restore_state(&state).unwrap();
    assert_eq!(
        world.character(id).unwrap().penetration_recovery_speed(),
        0.3
    );
}
