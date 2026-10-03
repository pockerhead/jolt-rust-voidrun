//! Smoke test for ragdolls through the raw API and the fork's extension: a three-part chain whose
//! parts are copied whole from body creation settings (`JPH_RagdollSettings_SetPart`) and whose
//! swing-twist and hinge constraints keep every field (`JPH_RagdollSettings_SetPartToParent*`).
//! It checks what Jolt received, the constraint priorities, the motor and readout functions, and
//! tears everything down once.

mod framework;

use std::ffi::CStr;

use framework::*;
use joltphysics_sys::*;

const DT: f32 = 1.0 / 60.0;
const PARTS: i32 = 3;
const LINEAR_DAMPING: f32 = 0.3;
const PRIORITY: u32 = 5;
const MOTOR_TORQUE: f32 = 50.0;
const HINGE_MIN: f32 = -1.0;
const HINGE_MAX: f32 = 0.5;

/// A motor whose spring is set by stiffness and damping, with torque limits ±`MOTOR_TORQUE`.
fn motor(base: JPH_MotorSettings) -> JPH_MotorSettings {
    JPH_MotorSettings {
        springSettings: JPH_SpringSettings {
            mode: JPH_SpringMode_StiffnessAndDamping,
            frequencyOrStiffness: 100.0,
            damping: 10.0,
        },
        minTorqueLimit: -MOTOR_TORQUE,
        maxTorqueLimit: MOTOR_TORQUE,
        ..base
    }
}

/// The swing-twist joint between root and child at y = 5.5, twist about +Y.
fn swing_twist() -> JPH_SwingTwistConstraintSettings {
    // SAFETY: an all-zero `JPH_SwingTwistConstraintSettings` is valid: floats, integers, enums
    // with a zero value and `false`. joltc fills it with Jolt's defaults and allocates nothing.
    let mut settings: JPH_SwingTwistConstraintSettings = unsafe { std::mem::zeroed() };
    // SAFETY: Jolt is initialised and `settings` is a live local.
    unsafe { JPH_SwingTwistConstraintSettings_Init(&mut settings) };
    settings.base.constraintPriority = PRIORITY;
    settings.space = JPH_ConstraintSpace_WorldSpace;
    settings.position1 = rvec3(0.0, 5.5, 0.0);
    settings.position2 = settings.position1;
    settings.twistAxis1 = vec3(0.0, 1.0, 0.0);
    settings.twistAxis2 = settings.twistAxis1;
    settings.planeAxis1 = vec3(1.0, 0.0, 0.0);
    settings.planeAxis2 = settings.planeAxis1;
    settings.normalHalfConeAngle = 0.5;
    settings.planeHalfConeAngle = 0.5;
    settings.twistMinAngle = -0.3;
    settings.twistMaxAngle = 0.3;
    settings.swingMotorSettings = motor(settings.swingMotorSettings);
    settings.twistMotorSettings = motor(settings.twistMotorSettings);
    settings
}

/// The hinge between child and grandchild at y = 6.5, about +X, limited to
/// `[HINGE_MIN, HINGE_MAX]`.
fn hinge() -> JPH_HingeConstraintSettings {
    // SAFETY: as in `swing_twist`, for `JPH_HingeConstraintSettings`.
    let mut settings: JPH_HingeConstraintSettings = unsafe { std::mem::zeroed() };
    // SAFETY: Jolt is initialised and `settings` is a live local.
    unsafe { JPH_HingeConstraintSettings_Init(&mut settings) };
    settings.base.constraintPriority = PRIORITY;
    settings.space = JPH_ConstraintSpace_WorldSpace;
    settings.point1 = rvec3(0.0, 6.5, 0.0);
    settings.point2 = settings.point1;
    settings.hingeAxis1 = vec3(1.0, 0.0, 0.0);
    settings.hingeAxis2 = settings.hingeAxis1;
    settings.normalAxis1 = vec3(0.0, 1.0, 0.0);
    settings.normalAxis2 = settings.normalAxis1;
    settings.limitsMin = HINGE_MIN;
    settings.limitsMax = HINGE_MAX;
    settings.motorSettings = motor(settings.motorSettings);
    settings
}

/// The skeleton root, child, grandchild, holding one reference the caller releases.
fn skeleton() -> *mut JPH_Skeleton {
    let names: [&CStr; 3] = [c"root", c"child", c"grandchild"];
    // SAFETY: Jolt is initialised. The skeleton is returned holding one reference; the names are
    // NUL-terminated literals that Jolt copies, and every parent precedes its child.
    unsafe {
        let skeleton = JPH_Skeleton_Create();
        assert!(!skeleton.is_null());
        for (index, name) in names.iter().enumerate() {
            let parent = index as i32 - 1;
            assert_eq!(
                JPH_Skeleton_AddJoint2(skeleton, name.as_ptr(), parent),
                index as u32
            );
        }
        assert!(JPH_Skeleton_AreJointsCorrectlyOrdered(skeleton));
        skeleton
    }
}

/// Ragdoll settings over `skeleton` with three capsule parts stacked along +Y at y = 5, 6, 7,
/// each in its own sub group of `filter`, a swing-twist joint on part 1 and a hinge on part 2.
///
/// # Safety
/// Jolt is initialised; `skeleton` and `filter` are live and stay so while the settings use them.
unsafe fn ragdoll_settings(
    skeleton: *mut JPH_Skeleton,
    filter: *mut JPH_GroupFilterTable,
) -> *mut JPH_RagdollSettings {
    let rotation = quat_identity();
    let swing_twist = swing_twist();
    let hinge = hinge();
    // SAFETY: Jolt is initialised and both handles are live (function contract). The settings
    // and the shape are returned holding one reference each; the settings take their own
    // reference to the skeleton, each part copies the creation settings with its own shape and
    // group filter references, and the parts' constraint settings are created by the extension
    // with the reference the parts hold. The temporary creation settings and our shape reference
    // are released at once.
    unsafe {
        let settings = JPH_RagdollSettings_Create();
        assert!(!settings.is_null());
        JPH_RagdollSettings_SetSkeleton(settings, skeleton);
        JPH_RagdollSettings_ResizeParts(settings, PARTS);
        let shape = JPH_CapsuleShape_Create(0.3, 0.2);
        for part in 0..PARTS {
            let position = rvec3(0.0, 5.0 + part as Real, 0.0);
            let body = JPH_BodyCreationSettings_Create3(
                shape.cast(),
                &position,
                &rotation,
                JPH_MotionType_Dynamic,
                OL_MOVING,
            );
            JPH_BodyCreationSettings_SetLinearDamping(body, LINEAR_DAMPING);
            let group = JPH_CollisionGroup {
                groupFilter: filter.cast(),
                groupID: 0,
                subGroupID: part as u32,
            };
            JPH_BodyCreationSettings_SetCollisionGroup(body, &group);
            JPH_RagdollSettings_SetPart(settings, part, body);
            JPH_BodyCreationSettings_Destroy(body);
        }
        JPH_Shape_Destroy(shape.cast());
        JPH_RagdollSettings_SetPartToParentSwingTwist(settings, 1, &swing_twist);
        JPH_RagdollSettings_SetPartToParentHinge(settings, 2, &hinge);
        assert_eq!(JPH_RagdollSettings_GetPartCount(settings), PARTS);
        settings
    }
}

/// The swing-twist and hinge constraints of `ragdoll`, in part order.
///
/// # Safety
/// `ragdoll` is live and was created from `ragdoll_settings`.
unsafe fn constraints(
    ragdoll: *mut JPH_Ragdoll,
) -> (*mut JPH_SwingTwistConstraint, *mut JPH_HingeConstraint) {
    // SAFETY: the ragdoll is live (function contract) and has two constraints, which it owns.
    // Each is the most derived object, as joltc casts them; the subtypes are checked first.
    unsafe {
        assert_eq!(JPH_Ragdoll_GetConstraintCount(ragdoll), 2);
        let first = JPH_Ragdoll_GetConstraint(ragdoll, 0);
        let second = JPH_Ragdoll_GetConstraint(ragdoll, 1);
        assert_eq!(
            JPH_Constraint_GetSubType(first.cast()),
            JPH_ConstraintSubType_SwingTwist
        );
        assert_eq!(
            JPH_Constraint_GetSubType(second.cast()),
            JPH_ConstraintSubType_Hinge
        );
        (first.cast(), second.cast())
    }
}

fn assert_motor(motor: &JPH_MotorSettings) {
    assert_eq!(
        motor.springSettings.mode,
        JPH_SpringMode_StiffnessAndDamping
    );
    assert_eq!(motor.springSettings.frequencyOrStiffness, 100.0);
    assert_eq!(motor.minTorqueLimit, -MOTOR_TORQUE);
    assert_eq!(motor.maxTorqueLimit, MOTOR_TORQUE);
}

#[test]
fn ragdoll_parts_and_constraints_reach_jolt() {
    let world = TestWorld::new(1);
    let skeleton = skeleton();
    // SAFETY: Jolt is initialised; the table is returned holding one reference, released at the
    // end after the settings and ragdolls that hold their own are gone.
    let filter = unsafe { JPH_GroupFilterTable_Create(PARTS as u32) };
    assert!(!filter.is_null());
    for a in 0..PARTS as u32 {
        for b in a + 1..PARTS as u32 {
            // SAFETY: the table is live and both sub groups are below its size.
            unsafe { JPH_GroupFilterTable_DisableCollision(filter, a, b) };
        }
    }
    // SAFETY: Jolt is initialised and both handles are live until the end of the test.
    let settings = unsafe { ragdoll_settings(skeleton, filter) };

    // a) Before any priority calculation: what the typed setters stored.
    // SAFETY: the world has room for three bodies, so `CreateRagdoll` returns a ragdoll holding
    // one reference. Its bodies were never added, so destroying it only destroys them; nothing
    // else uses it. Every output is a live local.
    unsafe {
        let ragdoll = JPH_RagdollSettings_CreateRagdoll(settings, world.system(), 7, 0);
        assert!(!ragdoll.is_null());
        let (swing_twist, hinge) = constraints(ragdoll);
        let mut stored: JPH_SwingTwistConstraintSettings = std::mem::zeroed();
        JPH_SwingTwistConstraint_GetSettings(swing_twist, &mut stored);
        assert_eq!(stored.base.constraintPriority, PRIORITY);
        assert_eq!(stored.twistMinAngle, -0.3);
        assert_eq!(stored.twistMaxAngle, 0.3);
        assert_motor(&stored.swingMotorSettings);
        assert_motor(&stored.twistMotorSettings);
        let mut stored: JPH_HingeConstraintSettings = std::mem::zeroed();
        JPH_HingeConstraint_GetSettings(hinge, &mut stored);
        assert_eq!(stored.base.constraintPriority, PRIORITY);
        assert_eq!(stored.limitsMin, HINGE_MIN);
        assert_eq!(stored.limitsMax, HINGE_MAX);
        assert_motor(&stored.motorSettings);
        JPH_Ragdoll_Destroy(ragdoll);
    }

    // b) With priorities and the index tables: a ragdoll in the world.
    // SAFETY: the settings are live and nothing else uses them; the world has room for the
    // three bodies. The ragdoll holds one reference, ours; it is added to the system and removed
    // again before that reference is released, and released before the world is dropped. Every
    // output is a live local, and no step runs while the ragdoll is read.
    unsafe {
        JPH_RagdollSettings_CalculateConstraintPriorities(settings, 0);
        JPH_RagdollSettings_CalculateBodyIndexToConstraintIndex(settings);
        JPH_RagdollSettings_CalculateConstraintIndexToBodyIdxPair(settings);
        let ragdoll = JPH_RagdollSettings_CreateRagdoll(settings, world.system(), 7, 0);
        assert!(!ragdoll.is_null());
        JPH_Ragdoll_AddToPhysicsSystem(ragdoll, JPH_Activation_Activate, true);
        for _ in 0..10 {
            world.step(DT);
        }
        assert_eq!(JPH_Ragdoll_GetBodyCount(ragdoll), PARTS);
        assert_eq!(JPH_PhysicsSystem_GetNumBodies(world.system()), PARTS as u32);
        let (swing_twist, hinge) = constraints(ragdoll);
        // Priorities grow toward the root.
        assert!(
            JPH_Constraint_GetConstraintPriority(swing_twist.cast())
                > JPH_Constraint_GetConstraintPriority(hinge.cast())
        );

        let lock_interface = JPH_PhysicsSystem_GetBodyLockInterface(world.system());
        for part in 0..PARTS {
            let id = JPH_Ragdoll_GetBodyID(ragdoll, part);
            let lock = JPH_BodyLockInterface_LockMultiRead(lock_interface, &id, 1);
            let body = JPH_BodyLockMultiRead_GetBody(lock, 0);
            assert!(!body.is_null());
            let mut group: JPH_CollisionGroup = std::mem::zeroed();
            JPH_Body_GetCollisionGroup(body, &mut group);
            assert_eq!(group.groupID, 7);
            assert_eq!(group.subGroupID, part as u32);
            let motion = JPH_Body_GetMotionProperties(body.cast_mut());
            assert_eq!(
                JPH_MotionProperties_GetLinearDamping(motion),
                LINEAR_DAMPING
            );
            JPH_BodyLockMultiRead_Destroy(lock);
        }

        for state in [
            JPH_MotorState_Off,
            JPH_MotorState_Position,
            JPH_MotorState_Off,
        ] {
            JPH_SwingTwistConstraint_SetSwingMotorState(swing_twist, state);
            JPH_SwingTwistConstraint_SetTwistMotorState(swing_twist, state);
            assert_eq!(
                JPH_SwingTwistConstraint_GetSwingMotorState(swing_twist),
                state
            );
            assert_eq!(
                JPH_SwingTwistConstraint_GetTwistMotorState(swing_twist),
                state
            );
        }
        let identity = quat_identity();
        JPH_SwingTwistConstraint_SetTargetOrientationBS(swing_twist, &identity);
        let mut rotation = JPH_Quat {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 0.0,
        };
        JPH_SwingTwistConstraint_GetRotationInConstraintSpace(swing_twist, &mut rotation);
        let length_sq = rotation.x * rotation.x
            + rotation.y * rotation.y
            + rotation.z * rotation.z
            + rotation.w * rotation.w;
        assert!((length_sq - 1.0).abs() < 1e-5, "{length_sq}");

        JPH_HingeConstraint_SetMotorState(hinge, JPH_MotorState_Position);
        JPH_HingeConstraint_SetTargetOrientationBS(hinge, &identity);
        world.step(DT);
        assert!(JPH_HingeConstraint_GetCurrentAngle(hinge).is_finite());

        JPH_Ragdoll_RemoveFromPhysicsSystem(ragdoll, true);
        JPH_Ragdoll_Destroy(ragdoll);
        assert_eq!(JPH_PhysicsSystem_GetNumBodies(world.system()), 0);
    }

    // SAFETY: every ragdoll is gone; each handle holds the one reference created above, released
    // exactly once here. The table is released through its `GroupFilter` base.
    unsafe {
        JPH_RagdollSettings_Destroy(settings);
        JPH_Skeleton_Destroy(skeleton);
        JPH_GroupFilter_Destroy(filter.cast());
    }
}
