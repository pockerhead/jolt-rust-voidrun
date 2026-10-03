//! Smoke test for the vehicle constraint through the raw API: a box chassis with one driven
//! wheel, registered as a constraint (through the fork's `JPH_VehicleConstraint_AsConstraint`)
//! and as a step listener, steps on a floor and is torn down cleanly.

mod framework;

use std::f32::consts::PI;
use std::ptr::null;

use framework::*;
use joltphysics_sys::*;

const DT: f32 = 1.0 / 60.0;

/// One wheel at the chassis' bottom centre, Jolt's other wheel defaults.
fn wheel_settings() -> *mut JPH_WheelSettingsWV {
    // SAFETY: Jolt is initialised (`TestWorld::new` ran). The settings are returned holding one
    // reference, which the caller releases; `position` is a live local.
    unsafe {
        let settings = JPH_WheelSettingsWV_Create();
        assert!(!settings.is_null());
        let position = vec3(0.0, -0.3, 0.0);
        JPH_WheelSettings_SetPosition(settings.cast(), &position);
        settings
    }
}

/// A wheeled controller with one differential that drives wheel 0 alone.
fn controller_settings() -> *mut JPH_WheeledVehicleControllerSettings {
    let differential = JPH_VehicleDifferentialSettings {
        leftWheel: 0,
        rightWheel: -1,
        differentialRatio: 3.42,
        leftRightSplit: 0.5,
        limitedSlipRatio: 1.4,
        engineTorqueRatio: 1.0,
    };
    // SAFETY: Jolt is initialised. The settings are returned holding one reference, which the
    // caller releases; `differential` is a live local that joltc copies.
    unsafe {
        let settings = JPH_WheeledVehicleControllerSettings_Create();
        assert!(!settings.is_null());
        JPH_WheeledVehicleControllerSettings_SetDifferentials(settings, &differential, 1);
        settings
    }
}

#[test]
fn vehicle_constraint_steps_and_tears_down() {
    let world = TestWorld::new(1);
    let floor = create_box(
        world.body_interface(),
        vec3(100.0, 1.0, 100.0),
        rvec3(0.0, -1.0, 0.0),
        JPH_MotionType_Static,
        OL_NON_MOVING,
        JPH_Activation_DontActivate,
    );
    let chassis = create_box(
        world.body_interface(),
        vec3(1.0, 0.3, 2.0),
        rvec3(0.0, 1.0, 0.0),
        JPH_MotionType_Dynamic,
        OL_MOVING,
        JPH_Activation_Activate,
    );

    // SAFETY: an all-zero `JPH_PhysicsSettings` is valid: integers, floats and `false`.
    let mut physics_settings: JPH_PhysicsSettings = unsafe { std::mem::zeroed() };
    // SAFETY: the system is live; `physics_settings` is a live local that joltc fills.
    unsafe { JPH_PhysicsSystem_GetPhysicsSettings(world.system(), &mut physics_settings) };
    // Jolt's defaults, which decide how many jobs run the step listeners.
    assert_eq!(physics_settings.stepListenersBatchSize, 8);
    assert_eq!(physics_settings.stepListenerBatchesPerJob, 1);

    let wheel = wheel_settings();
    let mut wheels = [wheel.cast::<JPH_WheelSettings>()];
    let controller = controller_settings();
    let settings = JPH_VehicleConstraintSettings {
        base: JPH_ConstraintSettings {
            enabled: true,
            constraintPriority: 0,
            numVelocityStepsOverride: 0,
            numPositionStepsOverride: 0,
            drawConstraintSize: 1.0,
            userData: 0,
        },
        up: vec3(0.0, 1.0, 0.0),
        forward: vec3(0.0, 0.0, 1.0),
        maxPitchRollAngle: PI,
        wheelsCount: 1,
        wheels: wheels.as_mut_ptr(),
        antiRollBarsCount: 0,
        antiRollBars: null(),
        controller: controller.cast(),
    };

    // SAFETY: the lock interface belongs to the live system; the multi-lock copies the one id.
    // The chassis exists, so the lock yields its body, which stays locked until the lock is
    // destroyed. The constructor stores the body pointer and reads the body; Jolt bodies do not
    // move in memory until destroyed. The settings, wheel and controller settings are live, and
    // the constraint keeps its own references to the wheel settings and copies the controller
    // settings. The new constraint holds one reference, ours.
    let constraint = unsafe {
        let lock_interface = JPH_PhysicsSystem_GetBodyLockInterface(world.system());
        let lock = JPH_BodyLockInterface_LockMultiWrite(lock_interface, &chassis, 1);
        let body = JPH_BodyLockMultiWrite_GetBody(lock, 0);
        assert!(!body.is_null());
        let constraint = JPH_VehicleConstraint_Create(body, &settings);
        JPH_BodyLockMultiWrite_Destroy(lock);
        JPH_WheelSettings_Destroy(wheel.cast());
        JPH_VehicleControllerSettings_Destroy(controller.cast());
        constraint
    };
    assert!(!constraint.is_null());

    // SAFETY: the constraint is live and holds our reference. `GetSubType` is a virtual call
    // through the returned pointer, so a wrong base address would not return the vehicle
    // subtype.
    let as_constraint = unsafe { JPH_VehicleConstraint_AsConstraint(constraint) };
    // SAFETY: as above.
    let sub_type = unsafe { JPH_Constraint_GetSubType(as_constraint) };
    assert_eq!(sub_type, JPH_ConstraintSubType_Vehicle);

    let up = vec3(0.0, 1.0, 0.0);
    // SAFETY: the system and constraint are live. The tester is created holding one reference;
    // the constraint takes its own, so ours is released right after. The constraint is added
    // both as a constraint and as a step listener, with the tester set before the first step,
    // which is how Jolt expects a vehicle to be registered.
    unsafe {
        let tester = JPH_VehicleCollisionTesterRay_Create(OL_MOVING, &up, 80.0_f32.to_radians());
        assert!(!tester.is_null());
        JPH_VehicleConstraint_SetVehicleCollisionTester(constraint, tester.cast());
        JPH_VehicleCollisionTester_Destroy(tester.cast());
        JPH_PhysicsSystem_AddConstraint(world.system(), as_constraint);
        JPH_PhysicsSystem_AddStepListener(
            world.system(),
            JPH_VehicleConstraint_AsPhysicsStepListener(constraint),
        );
    }

    for _ in 0..10 {
        world.step(DT);
    }

    // SAFETY: the constraint is live and has one wheel; the wheel is owned by the constraint
    // and read only. No step runs meanwhile.
    let has_contact = unsafe {
        assert_eq!(JPH_VehicleConstraint_GetWheelsCount(constraint), 1);
        let wheel = JPH_VehicleConstraint_GetWheel(constraint, 0);
        JPH_Wheel_HasContact(wheel)
    };
    // The chassis bottom is 0.7 above the floor and the wheel reaches 0.5 + 0.3 below it.
    assert!(has_contact);

    // SAFETY: the vehicle is removed from the listeners and the constraints before our
    // reference, the last one, is released, and before its body is destroyed. The body ids
    // name live bodies.
    unsafe {
        JPH_PhysicsSystem_RemoveStepListener(
            world.system(),
            JPH_VehicleConstraint_AsPhysicsStepListener(constraint),
        );
        JPH_PhysicsSystem_RemoveConstraint(world.system(), as_constraint);
        JPH_Constraint_Destroy(as_constraint);
        JPH_BodyInterface_RemoveAndDestroyBody(world.body_interface(), chassis);
        JPH_BodyInterface_RemoveAndDestroyBody(world.body_interface(), floor);
    }
}
