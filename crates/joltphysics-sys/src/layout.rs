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

    assert!(size_of::<JPH_Plane>() == 16);
    assert!(align_of::<JPH_Plane>() == 4);
    assert!(offset_of!(JPH_Plane, normal) == 0);
    assert!(offset_of!(JPH_Plane, distance) == 12);

    assert!(size_of::<JPH_ExtendedUpdateSettings>() == 48);
    assert!(align_of::<JPH_ExtendedUpdateSettings>() == 4);
    assert!(offset_of!(JPH_ExtendedUpdateSettings, stickToFloorStepDown) == 0);
    assert!(offset_of!(JPH_ExtendedUpdateSettings, walkStairsStepUp) == 12);
    assert!(offset_of!(JPH_ExtendedUpdateSettings, walkStairsMinStepForward) == 24);
    assert!(offset_of!(JPH_ExtendedUpdateSettings, walkStairsStepForwardTest) == 28);
    assert!(offset_of!(JPH_ExtendedUpdateSettings, walkStairsCosAngleForwardContact) == 32);
    assert!(offset_of!(JPH_ExtendedUpdateSettings, walkStairsStepDownExtra) == 36);

    assert!(size_of::<JPH_Point>() == 8);
    assert!(align_of::<JPH_Point>() == 4);
    assert!(offset_of!(JPH_Point, x) == 0);
    assert!(offset_of!(JPH_Point, y) == 4);

    assert!(size_of::<JPH_SpringSettings>() == 12);
    assert!(align_of::<JPH_SpringSettings>() == 4);
    assert!(offset_of!(JPH_SpringSettings, mode) == 0);
    assert!(offset_of!(JPH_SpringSettings, frequencyOrStiffness) == 4);
    assert!(offset_of!(JPH_SpringSettings, damping) == 8);

    assert!(size_of::<JPH_VehicleAntiRollBar>() == 12);
    assert!(align_of::<JPH_VehicleAntiRollBar>() == 4);
    assert!(offset_of!(JPH_VehicleAntiRollBar, leftWheel) == 0);
    assert!(offset_of!(JPH_VehicleAntiRollBar, rightWheel) == 4);
    assert!(offset_of!(JPH_VehicleAntiRollBar, stiffness) == 8);

    assert!(size_of::<JPH_VehicleDifferentialSettings>() == 24);
    assert!(align_of::<JPH_VehicleDifferentialSettings>() == 4);
    assert!(offset_of!(JPH_VehicleDifferentialSettings, leftWheel) == 0);
    assert!(offset_of!(JPH_VehicleDifferentialSettings, rightWheel) == 4);
    assert!(offset_of!(JPH_VehicleDifferentialSettings, differentialRatio) == 8);
    assert!(offset_of!(JPH_VehicleDifferentialSettings, leftRightSplit) == 12);
    assert!(offset_of!(JPH_VehicleDifferentialSettings, limitedSlipRatio) == 16);
    assert!(offset_of!(JPH_VehicleDifferentialSettings, engineTorqueRatio) == 20);

    assert!(size_of::<JPH_ConstraintSettings>() == 32);
    assert!(align_of::<JPH_ConstraintSettings>() == 8);
    assert!(offset_of!(JPH_ConstraintSettings, enabled) == 0);
    assert!(offset_of!(JPH_ConstraintSettings, constraintPriority) == 4);
    assert!(offset_of!(JPH_ConstraintSettings, numVelocityStepsOverride) == 8);
    assert!(offset_of!(JPH_ConstraintSettings, numPositionStepsOverride) == 12);
    assert!(offset_of!(JPH_ConstraintSettings, drawConstraintSize) == 16);
    assert!(offset_of!(JPH_ConstraintSettings, userData) == 24);

    assert!(size_of::<JPH_PhysicsSettings>() == 84);
    assert!(align_of::<JPH_PhysicsSettings>() == 4);
    assert!(offset_of!(JPH_PhysicsSettings, maxInFlightBodyPairs) == 0);
    assert!(offset_of!(JPH_PhysicsSettings, stepListenersBatchSize) == 4);
    assert!(offset_of!(JPH_PhysicsSettings, stepListenerBatchesPerJob) == 8);
    assert!(offset_of!(JPH_PhysicsSettings, baumgarte) == 12);
    assert!(offset_of!(JPH_PhysicsSettings, numVelocitySteps) == 56);
    assert!(offset_of!(JPH_PhysicsSettings, numPositionSteps) == 60);
    assert!(offset_of!(JPH_PhysicsSettings, pointVelocitySleepThreshold) == 72);
    assert!(offset_of!(JPH_PhysicsSettings, deterministicSimulation) == 76);
    assert!(offset_of!(JPH_PhysicsSettings, checkActiveEdges) == 82);

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
    assert!(size_of::<JPH_GroundState>() == 4);
    assert!(size_of::<JPH_SpringMode>() == 4);
    assert!(size_of::<JPH_TransmissionMode>() == 4);
    assert!(size_of::<JPH_ConstraintSubType>() == 4);
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

    assert!(size_of::<JPH_CharacterBaseSettings>() == 48);
    assert!(align_of::<JPH_CharacterBaseSettings>() == 8);
    assert!(offset_of!(JPH_CharacterBaseSettings, up) == 0);
    assert!(offset_of!(JPH_CharacterBaseSettings, supportingVolume) == 12);
    assert!(offset_of!(JPH_CharacterBaseSettings, maxSlopeAngle) == 28);
    assert!(offset_of!(JPH_CharacterBaseSettings, enhancedInternalEdgeRemoval) == 32);
    assert!(offset_of!(JPH_CharacterBaseSettings, shape) == 40);

    assert!(size_of::<JPH_CharacterVirtualSettings>() == 128);
    assert!(align_of::<JPH_CharacterVirtualSettings>() == 8);
    assert!(offset_of!(JPH_CharacterVirtualSettings, base) == 0);
    assert!(offset_of!(JPH_CharacterVirtualSettings, ID) == 48);
    assert!(offset_of!(JPH_CharacterVirtualSettings, mass) == 52);
    assert!(offset_of!(JPH_CharacterVirtualSettings, maxStrength) == 56);
    assert!(offset_of!(JPH_CharacterVirtualSettings, shapeOffset) == 60);
    assert!(offset_of!(JPH_CharacterVirtualSettings, backFaceMode) == 72);
    assert!(offset_of!(JPH_CharacterVirtualSettings, predictiveContactDistance) == 76);
    assert!(offset_of!(JPH_CharacterVirtualSettings, maxCollisionIterations) == 80);
    assert!(offset_of!(JPH_CharacterVirtualSettings, maxConstraintIterations) == 84);
    assert!(offset_of!(JPH_CharacterVirtualSettings, minTimeRemaining) == 88);
    assert!(offset_of!(JPH_CharacterVirtualSettings, collisionTolerance) == 92);
    assert!(offset_of!(JPH_CharacterVirtualSettings, characterPadding) == 96);
    assert!(offset_of!(JPH_CharacterVirtualSettings, maxNumHits) == 100);
    assert!(offset_of!(JPH_CharacterVirtualSettings, hitReductionCosMaxAngle) == 104);
    assert!(offset_of!(JPH_CharacterVirtualSettings, penetrationRecoverySpeed) == 108);
    assert!(offset_of!(JPH_CharacterVirtualSettings, innerBodyShape) == 112);
    assert!(offset_of!(JPH_CharacterVirtualSettings, innerBodyIDOverride) == 120);
    assert!(offset_of!(JPH_CharacterVirtualSettings, innerBodyLayer) == 124);

    assert!(size_of::<JPH_VehicleConstraintSettings>() == 96);
    assert!(align_of::<JPH_VehicleConstraintSettings>() == 8);
    assert!(offset_of!(JPH_VehicleConstraintSettings, base) == 0);
    assert!(offset_of!(JPH_VehicleConstraintSettings, up) == 32);
    assert!(offset_of!(JPH_VehicleConstraintSettings, forward) == 44);
    assert!(offset_of!(JPH_VehicleConstraintSettings, maxPitchRollAngle) == 56);
    assert!(offset_of!(JPH_VehicleConstraintSettings, wheelsCount) == 60);
    assert!(offset_of!(JPH_VehicleConstraintSettings, wheels) == 64);
    assert!(offset_of!(JPH_VehicleConstraintSettings, antiRollBarsCount) == 72);
    assert!(offset_of!(JPH_VehicleConstraintSettings, antiRollBars) == 80);
    assert!(offset_of!(JPH_VehicleConstraintSettings, controller) == 88);

    assert!(size_of::<JPH_VehicleEngineSettings>() == 32);
    assert!(align_of::<JPH_VehicleEngineSettings>() == 8);
    assert!(offset_of!(JPH_VehicleEngineSettings, maxTorque) == 0);
    assert!(offset_of!(JPH_VehicleEngineSettings, minRPM) == 4);
    assert!(offset_of!(JPH_VehicleEngineSettings, maxRPM) == 8);
    assert!(offset_of!(JPH_VehicleEngineSettings, normalizedTorque) == 16);
    assert!(offset_of!(JPH_VehicleEngineSettings, inertia) == 24);
    assert!(offset_of!(JPH_VehicleEngineSettings, angularDamping) == 28);

    assert!(align_of::<JPH_CharacterContact>() == 8);
    assert!(offset_of!(JPH_CharacterContact, hash) == 0);
    assert!(offset_of!(JPH_CharacterContact, bodyB) == 8);
    assert!(offset_of!(JPH_CharacterContact, characterIDB) == 12);
    assert!(offset_of!(JPH_CharacterContact, subShapeIDB) == 16);
};

// The `JPH_CharacterContact` fields from its `JPH_RVec3` position on, whose size depends on
// the precision.
#[cfg(all(target_pointer_width = "64", feature = "double-precision"))]
const _: () = {
    assert!(size_of::<JPH_CharacterContact>() == 136);
    assert!(offset_of!(JPH_CharacterContact, position) == 24);
    assert!(offset_of!(JPH_CharacterContact, linearVelocity) == 48);
    assert!(offset_of!(JPH_CharacterContact, contactNormal) == 60);
    assert!(offset_of!(JPH_CharacterContact, surfaceNormal) == 72);
    assert!(offset_of!(JPH_CharacterContact, distance) == 84);
    assert!(offset_of!(JPH_CharacterContact, fraction) == 88);
    assert!(offset_of!(JPH_CharacterContact, motionTypeB) == 92);
    assert!(offset_of!(JPH_CharacterContact, isSensorB) == 96);
    assert!(offset_of!(JPH_CharacterContact, characterB) == 104);
    assert!(offset_of!(JPH_CharacterContact, userData) == 112);
    assert!(offset_of!(JPH_CharacterContact, material) == 120);
    assert!(offset_of!(JPH_CharacterContact, hadCollision) == 128);
    assert!(offset_of!(JPH_CharacterContact, wasDiscarded) == 129);
    assert!(offset_of!(JPH_CharacterContact, canPushCharacter) == 130);
    assert!(offset_of!(JPH_CharacterContact, isBackFacingContact) == 131);
};

#[cfg(all(target_pointer_width = "64", not(feature = "double-precision")))]
const _: () = {
    assert!(size_of::<JPH_CharacterContact>() == 120);
    assert!(offset_of!(JPH_CharacterContact, position) == 20);
    assert!(offset_of!(JPH_CharacterContact, linearVelocity) == 32);
    assert!(offset_of!(JPH_CharacterContact, contactNormal) == 44);
    assert!(offset_of!(JPH_CharacterContact, surfaceNormal) == 56);
    assert!(offset_of!(JPH_CharacterContact, distance) == 68);
    assert!(offset_of!(JPH_CharacterContact, fraction) == 72);
    assert!(offset_of!(JPH_CharacterContact, motionTypeB) == 76);
    assert!(offset_of!(JPH_CharacterContact, isSensorB) == 80);
    assert!(offset_of!(JPH_CharacterContact, characterB) == 88);
    assert!(offset_of!(JPH_CharacterContact, userData) == 96);
    assert!(offset_of!(JPH_CharacterContact, material) == 104);
    assert!(offset_of!(JPH_CharacterContact, hadCollision) == 112);
    assert!(offset_of!(JPH_CharacterContact, wasDiscarded) == 113);
    assert!(offset_of!(JPH_CharacterContact, canPushCharacter) == 114);
    assert!(offset_of!(JPH_CharacterContact, isBackFacingContact) == 115);
};

#[cfg(feature = "debug-renderer")]
const _: () = {
    assert!(size_of::<JPH_DebugRenderer_Procs>() == 24);
    assert!(align_of::<JPH_DebugRenderer_Procs>() == 8);
    assert!(offset_of!(JPH_DebugRenderer_Procs, DrawLine) == 0);
    assert!(offset_of!(JPH_DebugRenderer_Procs, DrawTriangle) == 8);
    assert!(offset_of!(JPH_DebugRenderer_Procs, DrawText3D) == 16);
};
