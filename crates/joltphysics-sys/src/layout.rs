//! Compile-time layout checks for the joltc types the raw tests and the safe layer use.
//!
//! These numbers mirror `native/layout_checks.cpp`, which checks the same C types
//! against Jolt's C++ types; change both together.

use core::mem::{align_of, offset_of, size_of};

use crate::*;

const _: () = {
    assert!(size_of::<JPH_Vec3>() == 12);
    assert!(align_of::<JPH_Vec3>() == 4);
    assert!(offset_of!(JPH_Vec3, z) == 8);

    assert!(size_of::<JPH_Vec4>() == 16);
    assert!(align_of::<JPH_Vec4>() == 4);
    assert!(offset_of!(JPH_Vec4, w) == 12);

    assert!(size_of::<JPH_Quat>() == 16);
    assert!(align_of::<JPH_Quat>() == 4);
    assert!(offset_of!(JPH_Quat, w) == 12);

    assert!(size_of::<JPH_Mat4>() == 64);
    assert!(align_of::<JPH_Mat4>() == 4);
    assert!(offset_of!(JPH_Mat4, column) == 0);

    assert!(size_of::<JPH_AABox>() == 24);
    assert!(align_of::<JPH_AABox>() == 4);
    assert!(offset_of!(JPH_AABox, min) == 0);
    assert!(offset_of!(JPH_AABox, max) == 12);

    assert!(size_of::<JPH_MassProperties>() == 68);
    assert!(align_of::<JPH_MassProperties>() == 4);
    assert!(offset_of!(JPH_MassProperties, mass) == 0);
    assert!(offset_of!(JPH_MassProperties, inertia) == 4);

    assert!(size_of::<JobSystemThreadPoolConfig>() == 12);
    assert!(align_of::<JobSystemThreadPoolConfig>() == 4);
    assert!(offset_of!(JobSystemThreadPoolConfig, numThreads) == 8);

    assert!(size_of::<JPH_BodyID>() == 4);
    assert!(size_of::<JPH_ObjectLayer>() == 4);
    assert!(size_of::<JPH_BroadPhaseLayer>() == 1);

    assert!(size_of::<JPH_MotionType>() == 4);
    assert!(size_of::<JPH_Activation>() == 4);
    assert!(size_of::<JPH_PhysicsUpdateError>() == 4);
    assert!(size_of::<JPH_MotionQuality>() == 4);
    assert!(size_of::<JPH_OverrideMassProperties>() == 4);
};

#[cfg(feature = "double-precision")]
const _: () = {
    assert!(size_of::<JPH_RVec3>() == 24);
    assert!(align_of::<JPH_RVec3>() == 8);
    assert!(offset_of!(JPH_RVec3, z) == 16);

    assert!(size_of::<JPH_RMat4>() == 72);
    assert!(align_of::<JPH_RMat4>() == 8);
    assert!(offset_of!(JPH_RMat4, column3) == 48);
};

#[cfg(not(feature = "double-precision"))]
const _: () = {
    assert!(size_of::<JPH_RVec3>() == 12);
    assert!(size_of::<JPH_RMat4>() == 64);
};

#[cfg(target_pointer_width = "64")]
const _: () = {
    assert!(size_of::<JPH_PhysicsSystemSettings>() == 48);
    assert!(align_of::<JPH_PhysicsSystemSettings>() == 8);
    assert!(offset_of!(JPH_PhysicsSystemSettings, maxContactConstraints) == 12);
    assert!(offset_of!(JPH_PhysicsSystemSettings, broadPhaseLayerInterface) == 24);
    assert!(offset_of!(JPH_PhysicsSystemSettings, objectLayerPairFilter) == 32);
    assert!(offset_of!(JPH_PhysicsSystemSettings, objectVsBroadPhaseLayerFilter) == 40);
};
