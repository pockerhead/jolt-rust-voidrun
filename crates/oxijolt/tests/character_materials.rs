//! Characters keep the materials of their cached contacts alive: a character whose contacts
//! still name a body that changed shape or left the world keeps updating.
//!
//! Jolt keeps a raw material pointer in each cached contact, and an update shorter than the
//! character's `min_time_remaining` moves nothing and picks its ground from those contacts.

mod common;

use std::panic::{catch_unwind, AssertUnwindSafe};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use common::math::rvec3;
use oxijolt::*;

const RADIUS: f32 = 0.3;
const HALF_HEIGHT: f32 = 0.5;
/// Puts the capsule's bottom at the character position.
const FOOT_OFFSET: Vec3 = Vec3::new(0.0, HALF_HEIGHT + RADIUS, 0.0);
const DT: f32 = 1.0 / 60.0;
/// Below the default `min_time_remaining` (1e-4 s): Jolt's move runs no iteration.
const TINY_DT: f32 = 1e-6;
/// How far the character sinks into the floor and the wall, so both contacts collide.
const OVERLAP: f32 = 0.005;
/// The wall's surface normal toward the character.
const TOWARD_CHARACTER: Vec3 = Vec3::new(-1.0, 0.0, 0.0);
const WALL_HALF_EXTENT: Vec3 = Vec3::new(0.5, 2.0, 5.0);

/// A floor made of material `floor` and a wall made of material `wall` that the character
/// touches, and the character standing in the corner.
struct Corner {
    world: PhysicsWorld,
    floor: BodyId,
    wall: BodyId,
    character: CharacterId,
}

fn corner(floor: &PhysicsMaterial, wall: &PhysicsMaterial) -> Corner {
    let mut world = PhysicsWorld::new(WorldSettings::default()).unwrap();
    let floor_shape = Shape::new_box_with_material(Vec3::new(5.0, 0.5, 5.0), 0.05, floor).unwrap();
    let floor = world
        .create_body(
            &floor_shape,
            &BodySettings::new_static().position(RVec3::new(0.0, -0.5, 0.0)),
        )
        .unwrap();
    let wall_shape = Shape::new_box_with_material(WALL_HALF_EXTENT, 0.05, wall).unwrap();
    let wall_x = RADIUS - OVERLAP + 0.5;
    let wall = world
        .create_body(
            &wall_shape,
            &BodySettings::new_static().position(rvec3([f64::from(wall_x), 2.0, 0.0])),
        )
        .unwrap();
    let capsule = Shape::new_capsule(HALF_HEIGHT, RADIUS).unwrap();
    let settings = CharacterSettings::new(&capsule).shape_offset(FOOT_OFFSET);
    let character = world
        .create_character(
            &settings,
            rvec3([0.0, -f64::from(OVERLAP), 0.0]),
            Quat::IDENTITY,
        )
        .unwrap();
    Corner {
        world,
        floor,
        wall,
        character,
    }
}

/// Extended update settings without stick to floor and walk stairs.
fn plain_update() -> ExtendedUpdateSettings {
    ExtendedUpdateSettings::default()
        .stick_to_floor_step_down(Vec3::ZERO)
        .walk_stairs_step_up(Vec3::ZERO)
}

fn update(scene: &mut Corner, delta_time: f32) {
    scene
        .world
        .update_character(
            scene.character,
            delta_time,
            Vec3::new(0.0, -9.81, 0.0),
            &plain_update(),
            &QueryFilter::new(),
        )
        .unwrap();
}

fn refresh(scene: &mut Corner) {
    scene
        .world
        .refresh_character_contacts(scene.character, &QueryFilter::new())
        .unwrap();
}

/// Checks that the character stands on the floor and collides with the wall.
fn assert_in_the_corner(scene: &Corner) {
    let character = scene.world.character(scene.character).unwrap();
    assert_eq!(character.ground_body(), Some(scene.floor));
    let contacts = character.active_contacts();
    for body in [scene.floor, scene.wall] {
        assert!(
            contacts
                .iter()
                .any(|contact| contact.body == Some(body) && contact.had_collision),
            "the character collides with {body:?}"
        );
    }
}

/// Turns the character's up toward the wall's normal, so the cached wall contact supports it
/// best, and runs updates that move nothing, then normal ones.
fn reuse_the_cached_contacts(scene: &mut Corner) {
    scene
        .world
        .character_mut(scene.character)
        .unwrap()
        .set_up(TOWARD_CHARACTER)
        .unwrap();
    for _ in 0..3 {
        update(scene, TINY_DT);
        let character = scene.world.character(scene.character).unwrap();
        assert_eq!(
            character.ground_body(),
            Some(scene.wall),
            "the cached wall contact is the ground"
        );
    }
    let up = Vec3::new(0.0, 1.0, 0.0);
    scene
        .world
        .character_mut(scene.character)
        .unwrap()
        .set_up(up)
        .unwrap();
    for _ in 0..30 {
        update(scene, DT);
    }
}

/// Checks that the character stands on the floor and no contact names the wall.
fn assert_on_the_floor_only(scene: &Corner) {
    let character = scene.world.character(scene.character).unwrap();
    assert_eq!(character.ground_body(), Some(scene.floor));
    assert!(character
        .active_contacts()
        .iter()
        .all(|contact| contact.body != Some(scene.wall)));
}

#[test]
fn a_character_whose_contact_body_was_removed_keeps_updating() {
    let floor_material = PhysicsMaterial::new(1).unwrap();
    let wall_material = PhysicsMaterial::new(2).unwrap();
    let mut scene = corner(&floor_material, &wall_material);
    refresh(&mut scene);
    assert_in_the_corner(&scene);

    scene.world.remove_body(scene.wall).unwrap();
    drop(wall_material);
    reuse_the_cached_contacts(&mut scene);
    assert_on_the_floor_only(&scene);
}

#[test]
fn a_character_whose_contact_shape_was_replaced_keeps_updating() {
    let floor_material = PhysicsMaterial::new(1).unwrap();
    let wall_material = PhysicsMaterial::new(2).unwrap();
    let mut scene = corner(&floor_material, &wall_material);
    refresh(&mut scene);
    assert_in_the_corner(&scene);

    let plain = Shape::new_box(WALL_HALF_EXTENT).unwrap();
    scene
        .world
        .body_mut(scene.wall)
        .unwrap()
        .set_shape(&plain, None, Activation::DontActivate)
        .unwrap();
    drop((plain, wall_material));
    reuse_the_cached_contacts(&mut scene);

    // The wall is still there, now of the default material.
    let character = scene.world.character(scene.character).unwrap();
    assert_eq!(character.ground_body(), Some(scene.floor));
}

/// Panics in the first `contact_added`.
struct PanicOnce(AtomicBool);

impl CharacterContactListener for PanicOnce {
    fn contact_added(
        &self,
        _: CharacterId,
        _: &CharacterContact,
        _: &mut CharacterContactSettings,
    ) {
        if self.0.swap(false, Ordering::SeqCst) {
            panic!("contact_added panics once");
        }
    }
}

#[test]
fn a_listener_panic_still_retains_the_new_contacts_materials() {
    let floor_material = PhysicsMaterial::new(1).unwrap();
    let wall_material = PhysicsMaterial::new(2).unwrap();
    let mut scene = corner(&floor_material, &wall_material);
    scene
        .world
        .set_character_contact_listener(Some(Arc::new(PanicOnce(AtomicBool::new(true)))));
    let resumed = catch_unwind(AssertUnwindSafe(|| refresh(&mut scene)));
    assert!(resumed.is_err(), "the refresh resumes the listener's panic");
    assert_in_the_corner(&scene);

    scene.world.remove_body(scene.wall).unwrap();
    drop(wall_material);
    reuse_the_cached_contacts(&mut scene);
    assert_on_the_floor_only(&scene);
}
