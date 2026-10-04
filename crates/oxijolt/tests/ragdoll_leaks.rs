//! A leak gate for ragdolls: building a skeleton and the humanoid's settings (swing-twist, hinge
//! and six-DOF joints, stabilized every other round), creating a ragdoll at a pose, setting its
//! velocities, driving it with motors and kinematically, reading its pose and joints, stepping
//! and removing it, measured over many rounds against a budget of 200 bytes per round. A control
//! run then creates ragdoll settings with 12 parts through `oxijolt-sys` and never releases
//! them, and must exceed the budget, which shows the gate can see such a leak.
//!
//! It measures the private bytes of the process, because these objects are allocated by C++,
//! which a Rust global allocator does not see. The file holds exactly one test, so its binary
//! runs alone and no parallel test disturbs the counter.
#![cfg(windows)]

mod common;

use common::memory::private_bytes;
use common::ragdoll::*;
use common::*;
use oxijolt::*;
use oxijolt_sys::*;

const WARM_UP_ROUNDS: usize = 1_000;
const MEASURED_ROUNDS: usize = 10_000;
/// 200 bytes per measured round.
const MAX_GROWTH: usize = 200 * MEASURED_ROUNDS;

fn ragdoll_round(world: &mut PhysicsWorld, layers: &RagdollLayers, shapes: &[Shape], round: usize) {
    let skeleton = skeleton();
    let parts = humanoid_parts(shapes, layers.ragdoll);
    let settings = if round.is_multiple_of(2) {
        RagdollSettings::new(&skeleton, &parts)
    } else {
        RagdollSettings::new_stabilized(&skeleton, &parts)
    }
    .unwrap();
    let turn = quat_about(Y, 0.01 * (round % 7) as f32);
    let pose = transformed_pose(&bind_pose(), turn, [0.0, 1.0, 0.0]);
    let ragdoll = world
        .create_ragdoll(&settings, Some(&pose), Activation::Activate)
        .unwrap();
    let mut handle = world.ragdoll_mut(ragdoll).unwrap();
    handle
        .set_linear_and_angular_velocity(Vec3::new(0.1, 0.0, 0.0), Vec3::ZERO)
        .unwrap();
    handle.drive_to_pose_using_motors(&pose).unwrap();
    handle
        .set_motion_type(MotionType::Kinematic, Activation::Activate)
        .unwrap();
    handle.drive_to_pose_using_kinematics(&pose, DT).unwrap();
    handle
        .set_motion_type(MotionType::Dynamic, Activation::Activate)
        .unwrap();
    handle.stop_motors();
    assert!(world.step(DT).unwrap().is_complete());
    let reading = world.ragdoll(ragdoll).unwrap();
    assert_eq!(reading.pose().joints.len(), PART_COUNT);
    assert!((1..PART_COUNT as u32).all(|part| reading.joint(part).is_some()));
    world.remove_ragdoll(ragdoll).unwrap();
}

/// Creates `count` ragdoll settings with 12 parts and never releases them.
///
/// # Safety
/// Jolt is initialised.
unsafe fn leak_settings(count: usize) {
    for _ in 0..count {
        // SAFETY: Jolt is initialised (function contract). The settings are deliberately never
        // released.
        unsafe {
            let settings = JPH_RagdollSettings_Create();
            assert!(!settings.is_null());
            JPH_RagdollSettings_ResizeParts(settings, PART_COUNT as i32);
        }
    }
}

#[test]
fn ragdolls_do_not_leak() {
    let (mut world, layers) = ragdoll_world(1);
    let shapes = part_shapes();

    for round in 0..WARM_UP_ROUNDS {
        ragdoll_round(&mut world, &layers, &shapes, round);
    }
    let before = private_bytes();
    for round in 0..MEASURED_ROUNDS {
        ragdoll_round(&mut world, &layers, &shapes, round);
    }
    let after = private_bytes();
    let growth = after.saturating_sub(before);
    eprintln!("ragdolls: private bytes before {before}, after {after}, growth {growth}");
    assert_eq!(world.body_count(), 0);
    assert_eq!(world.ragdoll_ids().count(), 0);
    assert!(
        growth < MAX_GROWTH,
        "private bytes grew by {growth} (before {before}, after {after}): a ragdoll leaks"
    );

    // The control: as many ragdoll settings, never released, must show.
    let before = private_bytes();
    // SAFETY: Jolt is initialised (the world above exists).
    unsafe { leak_settings(MEASURED_ROUNDS) };
    let after = private_bytes();
    let growth = after.saturating_sub(before);
    eprintln!("control: private bytes before {before}, after {after}, growth {growth}");
    assert!(
        growth >= MAX_GROWTH,
        "the control leaked {MEASURED_ROUNDS} ragdoll settings but private bytes grew only by {growth}"
    );
}
