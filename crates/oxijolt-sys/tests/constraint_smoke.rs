//! Smoke test for the fork's constraint additions through the raw API: a body on a planar
//! Hermite path, a pulley, a rack and pinion with its hinge and slider, and the swing-twist and
//! six-DOF motor accessors. Each constraint is created, added, stepped, read and torn down; the
//! `_Init` functions return Jolt's defaults.

mod framework;

use framework::*;
use oxijolt_sys::*;

const DT: f32 = 1.0 / 60.0;

/// Jolt's `MotorSettings` defaults: a 2 Hz, critically damped spring and unlimited force and
/// torque.
fn assert_default_motor(motor: &JPH_MotorSettings) {
    assert_eq!(
        motor.springSettings.mode,
        JPH_SpringMode_FrequencyAndDamping
    );
    assert_eq!(motor.springSettings.frequencyOrStiffness, 2.0);
    assert_eq!(motor.springSettings.damping, 1.0);
    assert_eq!(motor.minForceLimit, -f32::MAX);
    assert_eq!(motor.maxForceLimit, f32::MAX);
    assert_eq!(motor.minTorqueLimit, -f32::MAX);
    assert_eq!(motor.maxTorqueLimit, f32::MAX);
}

/// Jolt's `ConstraintSettings` defaults.
fn assert_default_base(base: &JPH_ConstraintSettings) {
    assert!(base.enabled);
    assert_eq!(base.constraintPriority, 0);
    assert_eq!(base.numVelocityStepsOverride, 0);
    assert_eq!(base.numPositionStepsOverride, 0);
    assert_eq!(base.drawConstraintSize, 1.0);
    assert_eq!(base.userData, 0);
}

fn same_motor(a: &JPH_MotorSettings, b: &JPH_MotorSettings) -> bool {
    a.springSettings.mode == b.springSettings.mode
        && a.springSettings.frequencyOrStiffness == b.springSettings.frequencyOrStiffness
        && a.springSettings.damping == b.springSettings.damping
        && a.minForceLimit == b.minForceLimit
        && a.maxForceLimit == b.maxForceLimit
        && a.minTorqueLimit == b.minTorqueLimit
        && a.maxTorqueLimit == b.maxTorqueLimit
}

/// A motor that differs from Jolt's defaults in every field.
fn custom_motor() -> JPH_MotorSettings {
    JPH_MotorSettings {
        springSettings: JPH_SpringSettings {
            mode: JPH_SpringMode_StiffnessAndDamping,
            frequencyOrStiffness: 50.0,
            damping: 3.0,
        },
        minForceLimit: -10.0,
        maxForceLimit: 11.0,
        minTorqueLimit: -12.0,
        maxTorqueLimit: 13.0,
    }
}

fn finite3(v: JPH_Vec3) -> bool {
    v.x.is_finite() && v.y.is_finite() && v.z.is_finite()
}

/// Locks both bodies for writing, calls `create` with their pointers and returns its result.
fn with_two_bodies<T>(
    world: &TestWorld,
    ids: [JPH_BodyID; 2],
    create: impl FnOnce(*mut JPH_Body, *mut JPH_Body) -> T,
) -> T {
    // SAFETY: the lock interface belongs to the live system; joltc copies both ids. Both bodies
    // exist, so the lock yields them, locked until the lock is destroyed after `create`. The
    // constructors store the body pointers; Jolt bodies do not move until destroyed.
    unsafe {
        let lock_interface = JPH_PhysicsSystem_GetBodyLockInterface(world.system());
        let lock = JPH_BodyLockInterface_LockMultiWrite(lock_interface, ids.as_ptr(), 2);
        let body1 = JPH_BodyLockMultiWrite_GetBody(lock, 0);
        let body2 = JPH_BodyLockMultiWrite_GetBody(lock, 1);
        assert!(!body1.is_null() && !body2.is_null());
        let result = create(body1, body2);
        JPH_BodyLockMultiWrite_Destroy(lock);
        result
    }
}

fn dynamic_box(world: &TestWorld, half: JPH_Vec3, at: JPH_RVec3) -> JPH_BodyID {
    create_box(
        world.body_interface(),
        half,
        at,
        JPH_MotionType_Dynamic,
        OL_MOVING,
        JPH_Activation_Activate,
    )
}

fn static_box(world: &TestWorld, at: JPH_RVec3) -> JPH_BodyID {
    create_box(
        world.body_interface(),
        vec3(0.5, 0.5, 0.5),
        at,
        JPH_MotionType_Static,
        OL_NON_MOVING,
        JPH_Activation_DontActivate,
    )
}

#[test]
fn new_settings_init_to_jolts_defaults() {
    init();
    // SAFETY: all-zero settings are valid values (floats, integers, enums with a zero value and
    // a null pointer); each `_Init` fills a live local and allocates nothing that outlives it.
    let (path, pulley, rack) = unsafe {
        let mut path: JPH_PathConstraintSettings = std::mem::zeroed();
        let mut pulley: JPH_PulleyConstraintSettings = std::mem::zeroed();
        let mut rack: JPH_RackAndPinionConstraintSettings = std::mem::zeroed();
        JPH_PathConstraintSettings_Init(&mut path);
        JPH_PulleyConstraintSettings_Init(&mut pulley);
        JPH_RackAndPinionConstraintSettings_Init(&mut rack);
        (path, pulley, rack)
    };

    assert_default_base(&path.base);
    assert!(path.path.is_null());
    assert_eq!(
        [
            path.pathPosition.x,
            path.pathPosition.y,
            path.pathPosition.z
        ],
        [0.0; 3]
    );
    let q = path.pathRotation;
    assert_eq!([q.x, q.y, q.z, q.w], [0.0, 0.0, 0.0, 1.0]);
    assert_eq!(path.pathFraction, 0.0);
    assert_eq!(path.maxFrictionForce, 0.0);
    assert_eq!(
        path.rotationConstraintType,
        JPH_PathRotationConstraintType_Free
    );
    assert_default_motor(&path.positionMotorSettings);

    assert_default_base(&pulley.base);
    assert_eq!(pulley.space, JPH_ConstraintSpace_WorldSpace);
    for p in [
        pulley.bodyPoint1,
        pulley.fixedPoint1,
        pulley.bodyPoint2,
        pulley.fixedPoint2,
    ] {
        assert_eq!([p.x, p.y, p.z], [0.0; 3]);
    }
    assert_eq!(pulley.ratio, 1.0);
    assert_eq!(pulley.minLength, 0.0);
    assert_eq!(pulley.maxLength, -1.0);

    assert_default_base(&rack.base);
    assert_eq!(rack.space, JPH_ConstraintSpace_WorldSpace);
    for axis in [rack.hingeAxis, rack.sliderAxis] {
        assert_eq!([axis.x, axis.y, axis.z], [1.0, 0.0, 0.0]);
    }
    assert_eq!(rack.ratio, 1.0);
}

#[test]
fn path_constraint_steps_reads_and_tears_down() {
    let world = TestWorld::new(1);
    let anchor = static_box(&world, rvec3(0.0, 10.0, 0.0));
    let slider = dynamic_box(&world, vec3(0.1, 0.1, 0.1), rvec3(0.0, 0.0, 0.0));

    // SAFETY: Jolt is initialised. The path is created holding one reference, ours; every
    // argument is a live local.
    let path = unsafe {
        let path = JPH_PathConstraintPathHermite_Create();
        assert!(!path.is_null());
        let normal = vec3(0.0, 1.0, 0.0);
        let tangent = vec3(1.0, 0.0, 0.0);
        for position in [
            vec3(0.0, 0.0, 0.0),
            vec3(1.0, 0.0, 0.2),
            vec3(2.0, 0.0, 0.0),
        ] {
            JPH_PathConstraintPathHermite_AddPoint(path, &position, &tangent, &normal);
        }
        assert!(!JPH_PathConstraintPath_IsLooping(path));
        JPH_PathConstraintPath_SetIsLooping(path, false);
        assert_eq!(JPH_PathConstraintPath_GetPathMaxFraction(path), 2.0);
        path
    };

    // SAFETY: an all-zero settings value is valid; `_Init` fills it.
    let mut settings: JPH_PathConstraintSettings = unsafe { std::mem::zeroed() };
    // SAFETY: `settings` is a live local.
    unsafe { JPH_PathConstraintSettings_Init(&mut settings) };
    settings.path = path;
    // The path starts at the slider: 10 m below the anchor's centre of mass.
    settings.pathPosition = vec3(0.0, -10.0, 0.0);
    settings.rotationConstraintType = JPH_PathRotationConstraintType_ConstrainToPath;

    let constraint = with_two_bodies(&world, [anchor, slider], |body1, body2| {
        // SAFETY: both bodies are locked and live; the settings and the path are live. The
        // constraint takes its own reference to the path and is returned holding one reference,
        // ours.
        unsafe { JPH_PathConstraint_Create(&settings, body1, body2) }
    });
    assert!(!constraint.is_null());
    let as_constraint = constraint.cast::<JPH_Constraint>();
    // SAFETY: the constraint holds its own reference to the path, so ours can go; the path
    // getter returns the constraint's path, which stays live with it.
    unsafe {
        JPH_PathConstraintPath_Destroy(path);
        assert_eq!(
            JPH_Constraint_GetSubType(as_constraint),
            JPH_ConstraintSubType_Path
        );
        let own_path = JPH_PathConstraint_GetPath(constraint);
        assert!(!own_path.is_null());
        assert_eq!(JPH_PathConstraintPath_GetPathMaxFraction(own_path), 2.0);
        let near_end = vec3(1.9, 0.0, 0.0);
        let closest = JPH_PathConstraintPath_GetClosestPoint(own_path, &near_end, 1.0);
        assert!((1.5..=2.0).contains(&closest), "{closest}");
        JPH_PhysicsSystem_AddConstraint(world.system(), as_constraint);
    }

    // SAFETY: the constraint is live and added; the setters write members and no step runs.
    // The default motor settings are valid, as the velocity motor state asserts.
    unsafe {
        JPH_PathConstraint_SetMaxFrictionForce(constraint, 5.0);
        assert_eq!(JPH_PathConstraint_GetMaxFrictionForce(constraint), 5.0);
        let motor = custom_motor();
        JPH_PathConstraint_SetPositionMotorSettings(constraint, &motor);
        let mut read = std::mem::zeroed();
        JPH_PathConstraint_GetPositionMotorSettings(constraint, &mut read);
        assert!(same_motor(&motor, &read));
        JPH_PathConstraint_SetTargetVelocity(constraint, 1.0);
        assert_eq!(JPH_PathConstraint_GetTargetVelocity(constraint), 1.0);
        JPH_PathConstraint_SetTargetPathFraction(constraint, 1.5);
        assert_eq!(JPH_PathConstraint_GetTargetPathFraction(constraint), 1.5);
        JPH_PathConstraint_SetPositionMotorState(constraint, JPH_MotorState_Velocity);
        assert_eq!(
            JPH_PathConstraint_GetPositionMotorState(constraint),
            JPH_MotorState_Velocity
        );
    }

    for _ in 0..30 {
        world.step(DT);
    }

    // SAFETY: the constraint is live; the getters read members and no step runs.
    unsafe {
        let fraction = JPH_PathConstraint_GetPathFraction(constraint);
        assert!(fraction > 0.0 && fraction <= 2.0, "{fraction}");
        let mut pair = [f32::NAN; 2];
        JPH_PathConstraint_GetTotalLambdaPosition(constraint, pair.as_mut_ptr());
        assert!(pair.iter().all(|v| v.is_finite()));
        JPH_PathConstraint_GetTotalLambdaRotationHinge(constraint, pair.as_mut_ptr());
        assert!(pair.iter().all(|v| v.is_finite()));
        assert!(JPH_PathConstraint_GetTotalLambdaPositionLimits(constraint).is_finite());
        assert!(JPH_PathConstraint_GetTotalLambdaMotor(constraint).is_finite());
        let mut rotation = vec3(f32::NAN, 0.0, 0.0);
        JPH_PathConstraint_GetTotalLambdaRotation(constraint, &mut rotation);
        assert!(finite3(rotation));
    }

    // SAFETY: the constraint is removed from the system before our reference, the last one, is
    // released, and before its bodies are destroyed.
    unsafe {
        JPH_PhysicsSystem_RemoveConstraint(world.system(), as_constraint);
        JPH_Constraint_Destroy(as_constraint);
        JPH_BodyInterface_RemoveAndDestroyBody(world.body_interface(), slider);
        JPH_BodyInterface_RemoveAndDestroyBody(world.body_interface(), anchor);
    }
}

#[test]
fn pulley_constraint_steps_reads_and_tears_down() {
    let world = TestWorld::new(1);
    let half = vec3(0.2, 0.2, 0.2);
    let left = dynamic_box(&world, half, rvec3(-1.0, 0.0, 0.0));
    let right = dynamic_box(&world, half, rvec3(1.0, 0.0, 0.0));

    // SAFETY: an all-zero settings value is valid; `_Init` fills it.
    let mut settings: JPH_PulleyConstraintSettings = unsafe { std::mem::zeroed() };
    // SAFETY: `settings` is a live local.
    unsafe { JPH_PulleyConstraintSettings_Init(&mut settings) };
    settings.bodyPoint1 = rvec3(-1.0, 0.0, 0.0);
    settings.fixedPoint1 = rvec3(-1.0, 2.0, 0.0);
    settings.bodyPoint2 = rvec3(1.0, 0.0, 0.0);
    settings.fixedPoint2 = rvec3(1.0, 2.0, 0.0);

    let constraint = with_two_bodies(&world, [left, right], |body1, body2| {
        // SAFETY: both bodies are locked and live; the settings are live. The constraint is
        // returned holding one reference, ours.
        unsafe { JPH_PulleyConstraint_Create(&settings, body1, body2) }
    });
    assert!(!constraint.is_null());
    let as_constraint = constraint.cast::<JPH_Constraint>();

    // SAFETY: the constraint is live; the getters read members or copy them into a new settings
    // object, and no step runs.
    unsafe {
        assert_eq!(
            JPH_Constraint_GetSubType(as_constraint),
            JPH_ConstraintSubType_Pulley
        );
        // A maximum of -1 resolves to the length at creation: 2 m on each side.
        assert_eq!(JPH_PulleyConstraint_GetMinLength(constraint), 0.0);
        assert_eq!(JPH_PulleyConstraint_GetMaxLength(constraint), 4.0);
        assert_eq!(JPH_PulleyConstraint_GetCurrentLength(constraint), 4.0);
        let mut read: JPH_PulleyConstraintSettings = std::mem::zeroed();
        JPH_PulleyConstraint_GetSettings(constraint, &mut read);
        assert_default_base(&read.base);
        assert_eq!(read.space, JPH_ConstraintSpace_LocalToBodyCOM);
        let p = read.bodyPoint1;
        assert_eq!([p.x, p.y, p.z], [0.0; 3]);
        let p = read.fixedPoint2;
        assert_eq!([p.x, p.y, p.z], [1.0, 2.0, 0.0]);
        assert_eq!(read.ratio, 1.0);
        assert_eq!((read.minLength, read.maxLength), (0.0, 4.0));
        JPH_PhysicsSystem_AddConstraint(world.system(), as_constraint);
    }

    for _ in 0..30 {
        world.step(DT);
    }

    // SAFETY: the constraint is live and no step runs; `SetLength` gets 0 <= min <= max, as
    // Jolt asserts.
    unsafe {
        let length = JPH_PulleyConstraint_GetCurrentLength(constraint);
        assert!((length - 4.0).abs() < 0.05, "{length}");
        assert!(JPH_PulleyConstraint_GetTotalLambdaPosition(constraint).is_finite());
        JPH_PulleyConstraint_SetLength(constraint, 1.0, 4.5);
        assert_eq!(JPH_PulleyConstraint_GetMinLength(constraint), 1.0);
        assert_eq!(JPH_PulleyConstraint_GetMaxLength(constraint), 4.5);
    }
    world.step(DT);

    // SAFETY: as in the path test.
    unsafe {
        JPH_PhysicsSystem_RemoveConstraint(world.system(), as_constraint);
        JPH_Constraint_Destroy(as_constraint);
        JPH_BodyInterface_RemoveAndDestroyBody(world.body_interface(), left);
        JPH_BodyInterface_RemoveAndDestroyBody(world.body_interface(), right);
    }
}

#[test]
fn rack_and_pinion_steps_with_its_hinge_and_slider() {
    let world = TestWorld::new(1);
    let base = static_box(&world, rvec3(0.0, -10.0, 0.0));
    let pinion = dynamic_box(&world, vec3(0.5, 0.5, 0.1), rvec3(0.0, 0.0, 0.0));
    let rack = dynamic_box(&world, vec3(1.0, 0.2, 0.2), rvec3(0.0, -1.0, 0.0));
    let z = vec3(0.0, 0.0, 1.0);
    let x = vec3(1.0, 0.0, 0.0);
    let y = vec3(0.0, 1.0, 0.0);

    // SAFETY: all-zero settings values are valid; `_Init` fills them.
    let (mut hinge_settings, mut slider_settings, mut rack_settings) = unsafe {
        let mut hinge: JPH_HingeConstraintSettings = std::mem::zeroed();
        let mut slider: JPH_SliderConstraintSettings = std::mem::zeroed();
        let mut rack: JPH_RackAndPinionConstraintSettings = std::mem::zeroed();
        JPH_HingeConstraintSettings_Init(&mut hinge);
        JPH_SliderConstraintSettings_Init(&mut slider);
        JPH_RackAndPinionConstraintSettings_Init(&mut rack);
        (hinge, slider, rack)
    };
    hinge_settings.point1 = rvec3(0.0, 0.0, 0.0);
    hinge_settings.point2 = rvec3(0.0, 0.0, 0.0);
    hinge_settings.hingeAxis1 = z;
    hinge_settings.hingeAxis2 = z;
    hinge_settings.normalAxis1 = x;
    hinge_settings.normalAxis2 = x;
    slider_settings.autoDetectPoint = false;
    slider_settings.point1 = rvec3(0.0, -1.0, 0.0);
    slider_settings.point2 = rvec3(0.0, -1.0, 0.0);
    slider_settings.sliderAxis1 = x;
    slider_settings.sliderAxis2 = x;
    slider_settings.normalAxis1 = y;
    slider_settings.normalAxis2 = y;
    rack_settings.hingeAxis = z;
    rack_settings.sliderAxis = x;
    rack_settings.ratio = 4.0;

    // SAFETY (each closure): both bodies are locked and live; the settings are live. Each
    // constraint is returned holding one reference, ours.
    let hinge = with_two_bodies(&world, [base, pinion], |b1, b2| unsafe {
        JPH_HingeConstraint_Create(&hinge_settings, b1, b2)
    });
    let slider = with_two_bodies(&world, [base, rack], |b1, b2| unsafe {
        JPH_SliderConstraint_Create(&slider_settings, b1, b2)
    });
    let coupling = with_two_bodies(&world, [pinion, rack], |b1, b2| unsafe {
        JPH_RackAndPinionConstraint_Create(&rack_settings, b1, b2)
    });
    assert!(!hinge.is_null() && !slider.is_null() && !coupling.is_null());
    let constraints = [
        hinge.cast::<JPH_Constraint>(),
        slider.cast::<JPH_Constraint>(),
        coupling.cast::<JPH_Constraint>(),
    ];

    // SAFETY: all three constraints are live. The coupling takes a reference to the hinge and the
    // slider, which are of the kinds Jolt's rack and pinion supports.
    unsafe {
        assert_eq!(
            JPH_Constraint_GetSubType(constraints[2]),
            JPH_ConstraintSubType_RackAndPinion
        );
        JPH_RackAndPinionConstraint_SetConstraints(coupling, constraints[0], constraints[1]);
        for constraint in constraints {
            JPH_PhysicsSystem_AddConstraint(world.system(), constraint);
        }
        let mut spin = vec3(0.0, 0.0, 2.0);
        JPH_BodyInterface_SetAngularVelocity(world.body_interface(), pinion, &mut spin);
    }

    for _ in 0..30 {
        world.step(DT);
    }

    // SAFETY: the constraint is live and no step runs.
    let lambda = unsafe { JPH_RackAndPinionConstraint_GetTotalLambda(coupling) };
    assert!(lambda.is_finite());

    // SAFETY: the coupling goes first, before the hinge and slider it references; each is
    // removed from the system before our reference is released, and before the bodies go.
    unsafe {
        for constraint in constraints.into_iter().rev() {
            JPH_PhysicsSystem_RemoveConstraint(world.system(), constraint);
            JPH_Constraint_Destroy(constraint);
        }
        for body in [rack, pinion, base] {
            JPH_BodyInterface_RemoveAndDestroyBody(world.body_interface(), body);
        }
    }
}

#[test]
fn swing_twist_and_six_dof_motor_accessors_round_trip() {
    let world = TestWorld::new(1);
    let parent = static_box(&world, rvec3(0.0, 2.0, 0.0));
    let child = dynamic_box(&world, vec3(0.2, 0.2, 0.2), rvec3(0.0, 0.0, 0.0));

    // SAFETY: all-zero settings values are valid; `_Init` fills them.
    let (mut swing_settings, six_settings) = unsafe {
        let mut swing: JPH_SwingTwistConstraintSettings = std::mem::zeroed();
        let mut six: JPH_SixDOFConstraintSettings = std::mem::zeroed();
        JPH_SwingTwistConstraintSettings_Init(&mut swing);
        JPH_SixDOFConstraintSettings_Init(&mut six);
        (swing, six)
    };
    swing_settings.normalHalfConeAngle = 0.5;
    swing_settings.planeHalfConeAngle = 0.4;
    swing_settings.twistMinAngle = -0.3;
    swing_settings.twistMaxAngle = 0.2;

    // SAFETY (each closure): as in the rack and pinion test.
    let swing = with_two_bodies(&world, [parent, child], |b1, b2| unsafe {
        JPH_SwingTwistConstraint_Create(&swing_settings, b1, b2)
    });
    let six = with_two_bodies(&world, [parent, child], |b1, b2| unsafe {
        JPH_SixDOFConstraint_Create(&six_settings, b1, b2)
    });
    assert!(!swing.is_null() && !six.is_null());

    // SAFETY: both constraints are live and not added; the accessors write and read members.
    unsafe {
        assert_eq!(JPH_SwingTwistConstraint_GetNormalHalfConeAngle(swing), 0.5);
        assert_eq!(JPH_SwingTwistConstraint_GetPlaneHalfConeAngle(swing), 0.4);
        assert_eq!(JPH_SwingTwistConstraint_GetTwistMinAngle(swing), -0.3);
        assert_eq!(JPH_SwingTwistConstraint_GetTwistMaxAngle(swing), 0.2);

        let velocity = vec3(0.1, -0.2, 0.3);
        JPH_SwingTwistConstraint_SetTargetAngularVelocityCS(swing, &velocity);
        let mut read = vec3(0.0, 0.0, 0.0);
        JPH_SwingTwistConstraint_GetTargetAngularVelocityCS(swing, &mut read);
        assert_eq!([read.x, read.y, read.z], [0.1, -0.2, 0.3]);

        // A small twist within the limits is kept as given.
        let (sin, cos) = 0.05_f32.sin_cos();
        let target = JPH_Quat {
            x: sin,
            y: 0.0,
            z: 0.0,
            w: cos,
        };
        JPH_SwingTwistConstraint_SetTargetOrientationCS(swing, &target);
        let mut read = quat_identity();
        JPH_SwingTwistConstraint_GetTargetOrientationCS(swing, &mut read);
        assert_eq!([read.x, read.w], [sin, cos]);

        let motor = custom_motor();
        let mut read = std::mem::zeroed();
        JPH_SwingTwistConstraint_SetSwingMotorSettings(swing, &motor);
        JPH_SwingTwistConstraint_GetSwingMotorSettings(swing, &mut read);
        assert!(same_motor(&motor, &read));
        JPH_SwingTwistConstraint_GetTwistMotorSettings(swing, &mut read);
        assert_default_motor(&read);
        JPH_SwingTwistConstraint_SetTwistMotorSettings(swing, &motor);
        JPH_SwingTwistConstraint_GetTwistMotorSettings(swing, &mut read);
        assert!(same_motor(&motor, &read));

        JPH_SwingTwistConstraint_SetMaxFrictionTorque(swing, 2.5);
        assert_eq!(JPH_SwingTwistConstraint_GetMaxFrictionTorque(swing), 2.5);

        let axis = JPH_SixDOFConstraintAxis_RotationY;
        JPH_SixDOFConstraint_SetMotorSettings(six, axis, &motor);
        JPH_SixDOFConstraint_GetMotorSettings(six, axis, &mut read);
        assert!(same_motor(&motor, &read));
        JPH_SixDOFConstraint_GetMotorSettings(six, JPH_SixDOFConstraintAxis_RotationZ, &mut read);
        assert_default_motor(&read);

        JPH_Constraint_Destroy(swing.cast());
        JPH_Constraint_Destroy(six.cast());
        JPH_BodyInterface_RemoveAndDestroyBody(world.body_interface(), child);
        JPH_BodyInterface_RemoveAndDestroyBody(world.body_interface(), parent);
    }
}
