//! A leak gate for world states: saving the whole world, saving selected bodies and restoring,
//! 40 000 measured rounds against a 4 MiB threshold, so a leak of more than about 100 bytes per
//! round fails. A control run then saves a physics system into state recorders created through
//! `oxijolt-sys` and never destroys them, and must exceed the threshold, which shows the gate
//! can see such a leak.
//!
//! It measures the private bytes of the process, because the recorders are allocated by C++,
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

/// One round: a full save, a save of one body and a restore of each.
fn state_round(world: &mut PhysicsWorld, cube: BodyId) {
    let full = world.save_state();
    let partial = world.save_state_of(&[cube]).unwrap();
    world.restore_state(&partial).unwrap();
    world.restore_state(&full).unwrap();
}

#[test]
fn world_states_do_not_leak() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    add_floor(&mut world);
    let cube = add_cube(&mut world, RVec3::new(2.0, 0.5, 0.0));
    let capsule = Shape::new_capsule(0.7, 0.4).unwrap();
    let settings = CharacterSettings::new(&capsule)
        .shape_offset(Vec3::new(0.0, 1.1, 0.0))
        .inner_body(Some(InnerBody {
            shape: &capsule,
            object_layer: ObjectLayer::MOVING,
        }));
    world
        .create_character(&settings, RVec3::ZERO, Quat::IDENTITY)
        .unwrap();
    step(&mut world, 10);

    for _ in 0..WARM_UP_ROUNDS {
        state_round(&mut world, cube);
    }
    let before = private_bytes();
    for _ in 0..MEASURED_ROUNDS {
        state_round(&mut world, cube);
    }
    let after = private_bytes();
    let growth = after.saturating_sub(before);
    eprintln!("world states: private bytes before {before}, after {after}, growth {growth}");
    assert!(
        growth < MAX_GROWTH,
        "private bytes grew by {growth} (before {before}, after {after}): a world state leaks"
    );

    // The control: the same number of recorders, never destroyed, must show.
    // SAFETY: Jolt is initialised (the world above exists), and this test is the only one in
    // its binary, so no other thread creates or destroys a physics system.
    let system = unsafe { raw_system() };
    assert!(!system.is_null());
    let before = private_bytes();
    for _ in 0..MEASURED_ROUNDS {
        // SAFETY: the system is live and not stepping; a null body list saves every body. The
        // recorder is deliberately never destroyed.
        unsafe {
            let recorder = JPH_StateRecorder_Create();
            JPH_PhysicsSystem_SaveState(system, recorder, JPH_StateRecorderState_All, null(), 0);
        }
    }
    let after = private_bytes();
    let growth = after.saturating_sub(before);
    eprintln!("control: private bytes before {before}, after {after}, growth {growth}");
    // SAFETY: the leaked recorders do not refer to the system; it is destroyed once, by this
    // thread only.
    unsafe { JPH_PhysicsSystem_Destroy(system) };
    assert!(
        growth >= MAX_GROWTH,
        "the control leaked {MEASURED_ROUNDS} recorders but private bytes grew only by {growth}"
    );
}
