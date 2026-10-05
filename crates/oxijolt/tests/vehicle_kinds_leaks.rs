//! A leak gate for tracked vehicles and motorcycles: every round creates a tank and a
//! motorcycle on new chassis bodies, sets their input and gravity, steps once, reads tracks,
//! lean (the target lean accessor builds and releases a settings object per call) and wheels,
//! replaces both testers, and removes both vehicles and both chassis.
//!
//! It measures the private bytes of the process, because these objects are allocated by C++,
//! which a Rust global allocator does not see. After the warm-up it measures seven consecutive
//! blocks of rounds and holds the median block to the budget: a leak grows every block, a heap
//! step of the allocator only one. A control run then also creates a raw tracked vehicle
//! constraint per round through `oxijolt-sys` and never releases it, and must exceed the
//! budget, which shows the gate can see such a leak. The file holds exactly one test, so its
//! binary runs alone and no parallel test disturbs the counter.
#![cfg(windows)]

mod common;

use std::f32::consts::PI;
use std::ptr::null;

use common::memory::private_bytes;
use common::vehicle::{car_world, CarLayers, GRAVITY};
use common::vehicle_kinds::*;
use common::*;
use oxijolt::*;
use oxijolt_sys::*;

const WARM_UP_ROUNDS: usize = 2000;
const BLOCK_ROUNDS: usize = 1000;
const BLOCKS: usize = 7;
const MAX_ROUND_GROWTH: usize = 100;
const MAX_TOTAL_GROWTH: usize = 4 * 1024 * 1024;

fn round(world: &mut PhysicsWorld, layers: &CarLayers) {
    let tank_body = behaviour_chassis(layers, 4000.0, RVec3::new(-5.0, 1.2, 0.0), Quat::IDENTITY)
        .gravity_factor(0.0);
    let (tank_chassis, tank) =
        add_tank_with(world, &tank_body, VehicleCollisionTester::ray(layers.probe));
    let bike_body = behaviour_chassis(layers, BIKE_MASS, RVec3::new(5.0, 1.2, 0.0), Quat::IDENTITY)
        .gravity_factor(0.0);
    let settings = MotorcycleSettings::new(bike_vehicle_settings(bike_tester(layers)));
    let (bike_chassis, bike) = add_bike_with(world, &bike_body, &settings);

    let mut vehicle = world.vehicle_mut(tank).unwrap();
    vehicle.set_gravity(GRAVITY).unwrap();
    vehicle.set_driver_input(tracks(1.0, -1.0, 1.0)).unwrap();
    let mut vehicle = world.vehicle_mut(bike).unwrap();
    vehicle.set_gravity(GRAVITY).unwrap();
    vehicle.set_driver_input(ride(0.5, 0.2)).unwrap();
    assert!(world.step(DT).unwrap().is_complete());

    let vehicle = world.vehicle(tank).unwrap();
    assert!(vehicle.tracks().iter().all(|track| track.speed.is_finite()));
    assert_eq!(vehicle.wheels().len(), 18);
    let vehicle = world.vehicle(bike).unwrap();
    assert!(vehicle.lean().angle.is_finite());
    assert_eq!(vehicle.wheels().len(), 2);

    world
        .vehicle_mut(tank)
        .unwrap()
        .set_collision_tester(VehicleCollisionTester::cast_sphere(layers.probe, 0.2))
        .unwrap();
    world
        .vehicle_mut(bike)
        .unwrap()
        .set_collision_tester(VehicleCollisionTester::cast_cylinder(layers.probe))
        .unwrap();
    world.remove_vehicle(tank).unwrap();
    world.remove_vehicle(bike).unwrap();
    world.remove_body(tank_chassis).unwrap();
    world.remove_body(bike_chassis).unwrap();
}

/// Creates a two-wheel tracked vehicle constraint on `body` and never releases it.
///
/// # Safety
/// `system` is live and `body` names one of its bodies.
unsafe fn leak_tracked_constraint(system: *mut JPH_PhysicsSystem, body: JPH_BodyID) {
    // SAFETY: the system is live and holds the body (function contract); the lock is released
    // before returning. Every settings object is created holding one reference and released
    // after the constraint took what it keeps; the track settings and their wheel lists are
    // live locals that joltc copies. The constraint is deliberately never released and never
    // registered with the system.
    unsafe {
        let lock_interface = JPH_PhysicsSystem_GetBodyLockInterface(system);
        let lock = JPH_BodyLockInterface_LockMultiWrite(lock_interface, &body, 1);
        let jolt_body = JPH_BodyLockMultiWrite_GetBody(lock, 0);
        assert!(!jolt_body.is_null());
        let mut wheels: [*mut JPH_WheelSettings; 2] = [
            JPH_WheelSettingsTV_Create().cast(),
            JPH_WheelSettingsTV_Create().cast(),
        ];
        let controller = JPH_TrackedVehicleControllerSettings_Create();
        let indices = [[0_u32], [1_u32]];
        for (side, wheel) in indices.iter().enumerate() {
            let mut track: JPH_VehicleTrackSettings = std::mem::zeroed();
            JPH_VehicleTrackSettings_Init(&mut track);
            track.drivenWheel = wheel[0];
            track.wheels = wheel.as_ptr();
            track.wheelsCount = 1;
            JPH_TrackedVehicleControllerSettings_SetTrack(controller, side as u32, &track);
        }
        let settings = JPH_VehicleConstraintSettings {
            base: JPH_ConstraintSettings {
                enabled: true,
                constraintPriority: 0,
                numVelocityStepsOverride: 0,
                numPositionStepsOverride: 0,
                drawConstraintSize: 1.0,
                userData: 0,
            },
            up: JPH_Vec3 {
                x: 0.0,
                y: 1.0,
                z: 0.0,
            },
            forward: JPH_Vec3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            },
            maxPitchRollAngle: PI,
            wheelsCount: 2,
            wheels: wheels.as_mut_ptr(),
            antiRollBarsCount: 0,
            antiRollBars: null(),
            controller: controller.cast(),
        };
        let constraint = JPH_VehicleConstraint_Create(jolt_body, &settings);
        assert!(!constraint.is_null());
        for wheel in wheels {
            JPH_WheelSettings_Destroy(wheel);
        }
        JPH_VehicleControllerSettings_Destroy(controller.cast());
        JPH_BodyLockMultiWrite_Destroy(lock);
    }
}

/// The private-bytes growth of each of [`BLOCKS`] consecutive blocks of `round`s.
fn block_growth(mut round: impl FnMut()) -> Vec<usize> {
    (0..BLOCKS)
        .map(|_| {
            let before = private_bytes();
            for _ in 0..BLOCK_ROUNDS {
                round();
            }
            private_bytes().saturating_sub(before)
        })
        .collect()
}

fn median(blocks: &[usize]) -> usize {
    let mut sorted = blocks.to_vec();
    sorted.sort_unstable();
    sorted[sorted.len() / 2]
}

#[test]
fn tracked_vehicles_and_motorcycles_do_not_leak() {
    let (mut world, layers) = car_world(Vec3::ZERO, 1);
    world
        .create_body(
            &Shape::new_box(Vec3::new(50.0, 1.0, 50.0)).unwrap(),
            &BodySettings::new_static()
                .position(RVec3::new(0.0, -1.0, 0.0))
                .object_layer(layers.ground),
        )
        .unwrap();
    let bodies = world.body_count();
    for _ in 0..WARM_UP_ROUNDS {
        round(&mut world, &layers);
    }
    let blocks = block_growth(|| round(&mut world, &layers));
    eprintln!("vehicle kinds: growth per block of {BLOCK_ROUNDS} rounds: {blocks:?}");
    assert_eq!(world.body_count(), bodies);
    assert_eq!(world.vehicle_ids().count(), 0);
    let (median_growth, total): (usize, usize) = (median(&blocks), blocks.iter().sum());
    assert!(
        median_growth < MAX_ROUND_GROWTH * BLOCK_ROUNDS,
        "median block growth {median_growth} bytes: a vehicle kind leaks ({blocks:?})"
    );
    assert!(
        total < MAX_TOTAL_GROWTH,
        "total growth {total} bytes: a vehicle kind leaks ({blocks:?})"
    );

    // The control: the same rounds, each also forgetting a raw tracked vehicle constraint.
    // SAFETY: Jolt is initialised (the world above exists), and this test is the only one in
    // its binary, so no other thread creates or destroys a physics system.
    let (system, body) = unsafe { raw_system_with_body() };
    assert!(!system.is_null());
    let blocks = block_growth(|| {
        round(&mut world, &layers);
        // SAFETY: the system is live and holds the body.
        unsafe { leak_tracked_constraint(system, body) };
    });
    eprintln!("control: growth per block of {BLOCK_ROUNDS} rounds: {blocks:?}");
    // SAFETY: the leaked constraints were never registered with the system and are never used
    // again; the system is destroyed once, by this thread only.
    unsafe { JPH_PhysicsSystem_Destroy(system) };
    assert!(
        median(&blocks) >= MAX_ROUND_GROWTH * BLOCK_ROUNDS,
        "the control leaked a tracked vehicle constraint per round but the median block grew only by {} ({blocks:?})",
        median(&blocks)
    );
}
