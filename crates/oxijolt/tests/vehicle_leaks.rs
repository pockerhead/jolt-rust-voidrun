//! A leak gate for vehicles: creating a chassis and a vehicle with custom friction and torque
//! curves, two differentials, anti-roll bars and a sphere tester, driving it a step, reading its
//! wheels, replacing its tester and removing vehicle and chassis, 100 000 measured rounds
//! against a 2 MiB threshold, so a leak of more than about 20 bytes per round fails. A control
//! run then creates vehicle constraints through `oxijolt-sys` and never releases them, and
//! must exceed the threshold, which shows the gate can see such a leak.
//!
//! It measures the private bytes of the process, because these objects are allocated by C++,
//! which a Rust global allocator does not see. The file holds exactly one test, so its binary
//! runs alone and no parallel test disturbs the counter.
#![cfg(windows)]

mod common;

use std::f32::consts::PI;
use std::ptr::null;

use common::memory::private_bytes;
use common::vehicle::*;
use common::*;
use oxijolt::*;
use oxijolt_sys::*;

const WARM_UP_ROUNDS: usize = 10_000;
const MEASURED_ROUNDS: usize = 100_000;
const MAX_GROWTH: usize = 2 * 1024 * 1024;

/// The vehicle of a round: every joltc settings object the safe layer builds.
fn round_settings(layers: &CarLayers) -> VehicleSettings {
    let wheels = WHEEL_POSITIONS
        .iter()
        .map(|&position| {
            WheelSettings::new(position)
                .radius(WHEEL_RADIUS)
                .longitudinal_friction(vec![(0.0, 0.0), (0.05, 1.3), (0.3, 1.1)])
                .lateral_friction(vec![(0.0, 0.0), (4.0, 1.1), (15.0, 0.9)])
        })
        .collect();
    VehicleSettings::new(
        wheels,
        vec![
            VehicleDifferentialSettings::new(Some(0), Some(1)).engine_torque_ratio(0.5),
            VehicleDifferentialSettings::new(Some(2), Some(3)).engine_torque_ratio(0.5),
        ],
        VehicleCollisionTester::cast_sphere(layers.probe, 0.2),
    )
    .anti_roll_bars(vec![
        VehicleAntiRollBar::new(0, 1),
        VehicleAntiRollBar::new(2, 3),
    ])
    .engine(VehicleEngineSettings::default().normalized_torque(vec![
        (0.0, 0.7),
        (0.5, 1.0),
        (1.0, 0.9),
    ]))
}

fn vehicle_round(world: &mut PhysicsWorld, layers: &CarLayers, shape: &Shape, round: usize) {
    let x = (round % 5) as Real * 0.1;
    let chassis = world
        .create_body(
            shape,
            &chassis_settings(layers, RVec3::new(x, 0.9, 0.0), Quat::IDENTITY),
        )
        .unwrap();
    let car = world
        .create_vehicle(chassis, &round_settings(layers))
        .unwrap();
    let mut vehicle = world.vehicle_mut(car).unwrap();
    vehicle
        .set_driver_input(DriverInput {
            forward: 1.0,
            right: 0.5,
            ..DriverInput::default()
        })
        .unwrap();
    vehicle.set_gravity(GRAVITY).unwrap();
    assert!(world.step(DT).unwrap().is_complete());
    assert_eq!(world.vehicle(car).unwrap().wheels().len(), 4);
    world
        .vehicle_mut(car)
        .unwrap()
        .set_collision_tester(VehicleCollisionTester::cast_cylinder(layers.probe))
        .unwrap();
    world.remove_vehicle(car).unwrap();
    world.remove_body(chassis).unwrap();
}

/// Creates `count` one-wheel vehicle constraints on `body` and never releases them.
///
/// # Safety
/// `system` is live and `body` names one of its bodies.
unsafe fn leak_constraints(system: *mut JPH_PhysicsSystem, body: JPH_BodyID, count: usize) {
    // SAFETY: the system is live and holds the body (function contract); the lock is released
    // before returning. Every settings object is created holding one reference and released
    // after the constraint took what it keeps; the constraints are deliberately never released
    // and never registered with the system.
    unsafe {
        let lock_interface = JPH_PhysicsSystem_GetBodyLockInterface(system);
        let lock = JPH_BodyLockInterface_LockMultiWrite(lock_interface, &body, 1);
        let jolt_body = JPH_BodyLockMultiWrite_GetBody(lock, 0);
        assert!(!jolt_body.is_null());
        for _ in 0..count {
            let wheel = JPH_WheelSettingsWV_Create();
            let mut wheels = [wheel.cast::<JPH_WheelSettings>()];
            let controller = JPH_WheeledVehicleControllerSettings_Create();
            JPH_WheeledVehicleControllerSettings_AddDifferential(controller, 0, -1);
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
                wheelsCount: 1,
                wheels: wheels.as_mut_ptr(),
                antiRollBarsCount: 0,
                antiRollBars: null(),
                controller: controller.cast(),
            };
            let constraint = JPH_VehicleConstraint_Create(jolt_body, &settings);
            assert!(!constraint.is_null());
            JPH_WheelSettings_Destroy(wheel.cast());
            JPH_VehicleControllerSettings_Destroy(controller.cast());
        }
        JPH_BodyLockMultiWrite_Destroy(lock);
    }
}

#[test]
fn vehicles_do_not_leak() {
    let (mut world, layers) = car_world(Vec3::ZERO, 1);
    let floor = Shape::new_box(Vec3::new(50.0, 1.0, 50.0)).unwrap();
    world
        .create_body(
            &floor,
            &BodySettings::new_static()
                .position(RVec3::new(0.0, -1.0, 0.0))
                .object_layer(layers.ground),
        )
        .unwrap();
    let shape = chassis_shape();
    let bodies = world.body_count();

    for round in 0..WARM_UP_ROUNDS {
        vehicle_round(&mut world, &layers, &shape, round);
    }
    let before = private_bytes();
    for round in 0..MEASURED_ROUNDS {
        vehicle_round(&mut world, &layers, &shape, round);
    }
    let after = private_bytes();
    let growth = after.saturating_sub(before);
    eprintln!("vehicles: private bytes before {before}, after {after}, growth {growth}");
    assert_eq!(world.body_count(), bodies);
    assert_eq!(world.vehicle_ids().count(), 0);
    assert!(
        growth < MAX_GROWTH,
        "private bytes grew by {growth} (before {before}, after {after}): a vehicle leaks"
    );

    // The control: the same number of vehicle constraints, never released, must show.
    // SAFETY: Jolt is initialised (the world above exists), and this test is the only one in
    // its binary, so no other thread creates or destroys a physics system.
    let (system, body) = unsafe { raw_system_with_body() };
    assert!(!system.is_null());
    let before = private_bytes();
    // SAFETY: the system is live and holds the body.
    unsafe { leak_constraints(system, body, MEASURED_ROUNDS) };
    let after = private_bytes();
    let growth = after.saturating_sub(before);
    eprintln!("control: private bytes before {before}, after {after}, growth {growth}");
    // SAFETY: the leaked constraints were never registered with the system and are never used
    // again; the system is destroyed once, by this thread only.
    unsafe { JPH_PhysicsSystem_Destroy(system) };
    assert!(
        growth >= MAX_GROWTH,
        "the control leaked {MEASURED_ROUNDS} vehicle constraints but private bytes grew only by {growth}"
    );
}
