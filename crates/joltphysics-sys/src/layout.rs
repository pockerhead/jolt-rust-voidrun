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

    assert!(size_of::<JPH_RayCastResult>() == 12);
    assert!(align_of::<JPH_RayCastResult>() == 4);
    assert!(offset_of!(JPH_RayCastResult, bodyID) == 0);
    assert!(offset_of!(JPH_RayCastResult, fraction) == 4);
    assert!(offset_of!(JPH_RayCastResult, subShapeID2) == 8);

    assert!(size_of::<JPH_RayCastSettings>() == 12);
    assert!(align_of::<JPH_RayCastSettings>() == 4);
    assert!(offset_of!(JPH_RayCastSettings, backFaceModeTriangles) == 0);
    assert!(offset_of!(JPH_RayCastSettings, backFaceModeConvex) == 4);
    assert!(offset_of!(JPH_RayCastSettings, treatConvexAsSolid) == 8);

    assert!(size_of::<JPH_CollideSettingsBase>() == 28);
    assert!(align_of::<JPH_CollideSettingsBase>() == 4);
    assert!(offset_of!(JPH_CollideSettingsBase, activeEdgeMode) == 0);
    assert!(offset_of!(JPH_CollideSettingsBase, collectFacesMode) == 4);
    assert!(offset_of!(JPH_CollideSettingsBase, collisionTolerance) == 8);
    assert!(offset_of!(JPH_CollideSettingsBase, penetrationTolerance) == 12);
    assert!(offset_of!(JPH_CollideSettingsBase, activeEdgeMovementDirection) == 16);

    assert!(size_of::<JPH_CollideShapeSettings>() == 36);
    assert!(align_of::<JPH_CollideShapeSettings>() == 4);
    assert!(offset_of!(JPH_CollideShapeSettings, base) == 0);
    assert!(offset_of!(JPH_CollideShapeSettings, maxSeparationDistance) == 28);
    assert!(offset_of!(JPH_CollideShapeSettings, backFaceMode) == 32);

    assert!(size_of::<JPH_ShapeCastSettings>() == 40);
    assert!(align_of::<JPH_ShapeCastSettings>() == 4);
    assert!(offset_of!(JPH_ShapeCastSettings, base) == 0);
    assert!(offset_of!(JPH_ShapeCastSettings, backFaceModeTriangles) == 28);
    assert!(offset_of!(JPH_ShapeCastSettings, backFaceModeConvex) == 32);
    assert!(offset_of!(JPH_ShapeCastSettings, useShrunkenShapeAndConvexRadius) == 36);
    assert!(offset_of!(JPH_ShapeCastSettings, returnDeepestPoint) == 37);

    assert!(size_of::<JPH_ShapeCastResult>() == 60);
    assert!(align_of::<JPH_ShapeCastResult>() == 4);
    assert!(offset_of!(JPH_ShapeCastResult, contactPointOn1) == 0);
    assert!(offset_of!(JPH_ShapeCastResult, contactPointOn2) == 12);
    assert!(offset_of!(JPH_ShapeCastResult, penetrationAxis) == 24);
    assert!(offset_of!(JPH_ShapeCastResult, penetrationDepth) == 36);
    assert!(offset_of!(JPH_ShapeCastResult, subShapeID1) == 40);
    assert!(offset_of!(JPH_ShapeCastResult, subShapeID2) == 44);
    assert!(offset_of!(JPH_ShapeCastResult, bodyID2) == 48);
    assert!(offset_of!(JPH_ShapeCastResult, fraction) == 52);
    assert!(offset_of!(JPH_ShapeCastResult, isBackFaceHit) == 56);

    assert!(size_of::<JobSystemThreadPoolConfig>() == 12);
    assert!(align_of::<JobSystemThreadPoolConfig>() == 4);
    assert!(offset_of!(JobSystemThreadPoolConfig, numThreads) == 8);

    assert!(size_of::<JPH_BodyID>() == 4);
    assert!(size_of::<JPH_SubShapeID>() == 4);
    assert!(size_of::<JPH_ObjectLayer>() == 4);
    assert!(size_of::<JPH_BroadPhaseLayer>() == 1);

    assert!(size_of::<JPH_MotionType>() == 4);
    assert!(size_of::<JPH_Activation>() == 4);
    assert!(size_of::<JPH_PhysicsUpdateError>() == 4);
    assert!(size_of::<JPH_MotionQuality>() == 4);
    assert!(size_of::<JPH_OverrideMassProperties>() == 4);
    assert!(size_of::<JPH_ShapeSubType>() == 4);
    assert!(size_of::<JPH_BackFaceMode>() == 4);
    assert!(size_of::<JPH_ActiveEdgeMode>() == 4);
    assert!(size_of::<JPH_CollectFacesMode>() == 4);
    assert!(size_of::<JPH_CollisionCollectorType>() == 4);
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

    assert!(size_of::<JPH_CollideShapeResult>() == 80);
    assert!(align_of::<JPH_CollideShapeResult>() == 8);
    assert!(offset_of!(JPH_CollideShapeResult, contactPointOn1) == 0);
    assert!(offset_of!(JPH_CollideShapeResult, contactPointOn2) == 12);
    assert!(offset_of!(JPH_CollideShapeResult, penetrationAxis) == 24);
    assert!(offset_of!(JPH_CollideShapeResult, penetrationDepth) == 36);
    assert!(offset_of!(JPH_CollideShapeResult, subShapeID1) == 40);
    assert!(offset_of!(JPH_CollideShapeResult, subShapeID2) == 44);
    assert!(offset_of!(JPH_CollideShapeResult, bodyID2) == 48);
    assert!(offset_of!(JPH_CollideShapeResult, shape1FaceCount) == 52);
    assert!(offset_of!(JPH_CollideShapeResult, shape1Faces) == 56);
    assert!(offset_of!(JPH_CollideShapeResult, shape2FaceCount) == 64);
    assert!(offset_of!(JPH_CollideShapeResult, shape2Faces) == 72);

    assert!(size_of::<JPH_ObjectLayerFilter_Procs>() == 8);
    assert!(align_of::<JPH_ObjectLayerFilter_Procs>() == 8);
    assert!(offset_of!(JPH_ObjectLayerFilter_Procs, ShouldCollide) == 0);

    assert!(size_of::<JPH_BodyFilter_Procs>() == 16);
    assert!(align_of::<JPH_BodyFilter_Procs>() == 8);
    assert!(offset_of!(JPH_BodyFilter_Procs, ShouldCollide) == 0);
    assert!(offset_of!(JPH_BodyFilter_Procs, ShouldCollideLocked) == 8);

    assert!(size_of::<JPH_ShapeFilter_Procs>() == 16);
    assert!(align_of::<JPH_ShapeFilter_Procs>() == 8);
    assert!(offset_of!(JPH_ShapeFilter_Procs, ShouldCollide) == 0);
    assert!(offset_of!(JPH_ShapeFilter_Procs, ShouldCollide2) == 8);
};
