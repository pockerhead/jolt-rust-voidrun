//! A leak gate for characters: creating, updating, saving, restoring and removing a character
//! with an inner body that collides with other characters, 40 000 measured rounds against a
//! 4 MiB threshold, so a leak of more than about 100 bytes per round fails. A control run then
//! creates characters through `oxijolt-sys` and never releases them, and must exceed the
//! threshold, which shows the gate can see such a leak.
//!
//! It measures the private bytes of the process, because these objects are allocated by C++,
//! which a Rust global allocator does not see. The file holds exactly one test, so its binary
//! runs alone and no parallel test disturbs the counter.
#![cfg(windows)]

mod common;

use std::ptr::null;

use common::memory::private_bytes;
use common::*;
use oxijolt::*;
use oxijolt_sys::*;

const WARM_UP_ROUNDS: usize = 5_000;
const MEASURED_ROUNDS: usize = 40_000;
const MAX_GROWTH: usize = 4 * 1024 * 1024;

/// One round: a character with an inner body that collides with characters, next to the
/// neighbour character of the test.
fn character_round(world: &mut PhysicsWorld, capsule: &Shape, round: usize) {
    let settings = CharacterSettings::new(capsule)
        .shape_offset(Vec3::new(0.0, 1.1, 0.0))
        .collide_with_characters(true)
        .inner_body(Some(InnerBody {
            shape: capsule,
            object_layer: ObjectLayer::MOVING,
        }));
    let x = (round % 5) as Real * 0.1;
    let id = world
        .create_character(&settings, RVec3::new(x, 0.0, 0.0), Quat::IDENTITY)
        .unwrap();
    world
        .character_mut(id)
        .unwrap()
        .set_linear_velocity(Vec3::new(1.0, -1.0, 0.0))
        .unwrap();
    world
        .update_character(
            id,
            DT,
            Vec3::new(0.0, -9.81, 0.0),
            &ExtendedUpdateSettings::default(),
            &QueryFilter::new(),
        )
        .unwrap();
    let state = world.character(id).unwrap().save_state();
    world
        .character_mut(id)
        .unwrap()
        .restore_state(&state)
        .unwrap();
    world.remove_character(id).unwrap();
}

/// Creates `count` characters on `system` and never releases them.
///
/// # Safety
/// `system` and `capsule` are live.
unsafe fn leak_characters(system: *mut JPH_PhysicsSystem, capsule: *const JPH_Shape, count: usize) {
    let mut settings = JPH_CharacterVirtualSettings {
        base: JPH_CharacterBaseSettings {
            up: JPH_Vec3 {
                x: 0.0,
                y: 1.0,
                z: 0.0,
            },
            supportingVolume: JPH_Plane {
                normal: JPH_Vec3 {
                    x: 0.0,
                    y: 1.0,
                    z: 0.0,
                },
                distance: -1.0e10,
            },
            maxSlopeAngle: 0.8,
            enhancedInternalEdgeRemoval: false,
            shape: capsule,
        },
        ID: 1,
        mass: 70.0,
        maxStrength: 100.0,
        shapeOffset: JPH_Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        backFaceMode: JPH_BackFaceMode_CollideWithBackFaces,
        predictiveContactDistance: 0.1,
        maxCollisionIterations: 5,
        maxConstraintIterations: 15,
        minTimeRemaining: 1.0e-4,
        collisionTolerance: 1.0e-3,
        characterPadding: 0.02,
        maxNumHits: 256,
        hitReductionCosMaxAngle: 0.999,
        penetrationRecoverySpeed: 1.0,
        innerBodyShape: null(),
        innerBodyIDOverride: u32::MAX,
        innerBodyLayer: 0,
    };
    let position = JPH_RVec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };
    for i in 0..count {
        settings.ID = i as u32 + 1;
        // SAFETY: `system` and `capsule` are live (function contract); the settings and
        // position are live locals. The returned reference is deliberately never released.
        let character =
            unsafe { JPH_CharacterVirtual_Create(&settings, &position, null(), 0, system) };
        assert!(!character.is_null());
    }
}

#[test]
fn characters_do_not_leak() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    add_floor(&mut world);
    let capsule = Shape::new_capsule(0.7, 0.4).unwrap();
    let neighbour = CharacterSettings::new(&capsule)
        .shape_offset(Vec3::new(0.0, 1.1, 0.0))
        .collide_with_characters(true);
    world
        .create_character(&neighbour, RVec3::new(0.6, 0.0, 0.0), Quat::IDENTITY)
        .unwrap();
    let bodies = world.body_count();

    for round in 0..WARM_UP_ROUNDS {
        character_round(&mut world, &capsule, round);
    }
    let before = private_bytes();
    for round in 0..MEASURED_ROUNDS {
        character_round(&mut world, &capsule, round);
    }
    let after = private_bytes();
    let growth = after.saturating_sub(before);
    eprintln!("characters: private bytes before {before}, after {after}, growth {growth}");
    assert_eq!(world.body_count(), bodies);
    assert!(
        growth < MAX_GROWTH,
        "private bytes grew by {growth} (before {before}, after {after}): a character leaks"
    );

    // The control: the same number of characters, never released, must show.
    // SAFETY: Jolt is initialised (the world above exists), and this test is the only one in
    // its binary, so no other thread creates or destroys a physics system.
    let system = unsafe { raw_system() };
    assert!(!system.is_null());
    // SAFETY: Jolt is initialised; the shape holds one reference, released below.
    let raw_capsule = unsafe { JPH_CapsuleShape_Create(0.7, 0.4) }.cast::<JPH_Shape>();
    let before = private_bytes();
    // SAFETY: both are live.
    unsafe { leak_characters(system, raw_capsule, MEASURED_ROUNDS) };
    let after = private_bytes();
    let growth = after.saturating_sub(before);
    eprintln!("control: private bytes before {before}, after {after}, growth {growth}");
    // SAFETY: the leaked characters never use the shape or the system again; each holds its
    // own shape reference and none has an inner body. Ours is released once, and the system is
    // destroyed once, by this thread only.
    unsafe {
        JPH_Shape_Destroy(raw_capsule);
        JPH_PhysicsSystem_Destroy(system);
    }
    assert!(
        growth >= MAX_GROWTH,
        "the control leaked {MEASURED_ROUNDS} characters but private bytes grew only by {growth}"
    );
}
