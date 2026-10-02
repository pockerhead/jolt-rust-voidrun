// Compile-time layout checks for the joltc C types that the Rust bindings use.
//
// joltc declares C mirrors of Jolt values and enums and converts or casts them
// to the C++ types. These checks pin the C ABI view (the same numbers are
// asserted on the Rust side in src/layout.rs; change both together) and the
// agreement between the C mirrors and Jolt's C++ types.
//
// More types are added here when the safe layer starts to use them.

#include <Jolt/Jolt.h>

#include <Jolt/Math/Mat44.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyID.h>
#include <Jolt/Physics/Body/MotionQuality.h>
#include <Jolt/Physics/Body/MotionType.h>
#include <Jolt/Physics/Character/CharacterBase.h>
#include <Jolt/Physics/Collision/ActiveEdgeMode.h>
#include <Jolt/Physics/Collision/BackFaceMode.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/CollectFacesMode.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Physics/Collision/Shape/SubShapeID.h>
#include <Jolt/Physics/EActivation.h>
#include <Jolt/Physics/EPhysicsUpdateError.h>

#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "joltc.h"

#define JOLTPHYSICS_SYS_ASSERT_LAYOUT(T, size, align)                                \
    static_assert(sizeof(T) == (size), #T ": unexpected size");                \
    static_assert(alignof(T) == (align), #T ": unexpected alignment")

#define JOLTPHYSICS_SYS_ASSERT_OFFSET(T, field, offset)                              \
    static_assert(offsetof(T, field) == (offset), #T "." #field ": unexpected offset")

// C ABI value structs passed by the raw tests and joltphysics.
JOLTPHYSICS_SYS_ASSERT_LAYOUT(JPH_Vec3, 12, 4);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_Vec3, z, 8);
JOLTPHYSICS_SYS_ASSERT_LAYOUT(JPH_Vec4, 16, 4);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_Vec4, w, 12);
JOLTPHYSICS_SYS_ASSERT_LAYOUT(JPH_Quat, 16, 4);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_Quat, w, 12);
JOLTPHYSICS_SYS_ASSERT_LAYOUT(JPH_Mat4, 64, 4);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_Mat4, column, 0);

#ifdef JPH_DOUBLE_PRECISION
JOLTPHYSICS_SYS_ASSERT_LAYOUT(JPH_RVec3, 24, 8);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_RVec3, z, 16);
JOLTPHYSICS_SYS_ASSERT_LAYOUT(JPH_RMat4, 72, 8);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_RMat4, column3, 48);
#else
static_assert(std::is_same_v<JPH_RVec3, JPH_Vec3>, "JPH_RVec3 is JPH_Vec3 in single precision");
static_assert(std::is_same_v<JPH_RMat4, JPH_Mat4>, "JPH_RMat4 is JPH_Mat4 in single precision");
#endif

// joltc converts JPH_AABox and JPH_MassProperties field by field, so they have
// no C++ layout to compare against.
JOLTPHYSICS_SYS_ASSERT_LAYOUT(JPH_AABox, 24, 4);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_AABox, min, 0);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_AABox, max, 12);
JOLTPHYSICS_SYS_ASSERT_LAYOUT(JPH_MassProperties, 68, 4);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_MassProperties, mass, 0);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_MassProperties, inertia, 4);
// JPH_NarrowPhaseQuery_CastRay fills JPH_RayCastResult field by field, and
// joltphysics passes it by pointer.
JOLTPHYSICS_SYS_ASSERT_LAYOUT(JPH_RayCastResult, 12, 4);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_RayCastResult, bodyID, 0);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_RayCastResult, fraction, 4);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_RayCastResult, subShapeID2, 8);

// Scene query settings and results. joltc converts these field by field
// (ToJolt/FromJolt), so only the C ABI is pinned here, not agreement with the
// C++ layout.
JOLTPHYSICS_SYS_ASSERT_LAYOUT(JPH_RayCastSettings, 12, 4);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_RayCastSettings, backFaceModeTriangles, 0);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_RayCastSettings, backFaceModeConvex, 4);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_RayCastSettings, treatConvexAsSolid, 8);
JOLTPHYSICS_SYS_ASSERT_LAYOUT(JPH_CollideSettingsBase, 28, 4);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CollideSettingsBase, activeEdgeMode, 0);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CollideSettingsBase, collectFacesMode, 4);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CollideSettingsBase, collisionTolerance, 8);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CollideSettingsBase, penetrationTolerance, 12);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CollideSettingsBase, activeEdgeMovementDirection, 16);
JOLTPHYSICS_SYS_ASSERT_LAYOUT(JPH_CollideShapeSettings, 36, 4);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CollideShapeSettings, base, 0);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CollideShapeSettings, maxSeparationDistance, 28);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CollideShapeSettings, backFaceMode, 32);
JOLTPHYSICS_SYS_ASSERT_LAYOUT(JPH_ShapeCastSettings, 40, 4);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_ShapeCastSettings, base, 0);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_ShapeCastSettings, backFaceModeTriangles, 28);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_ShapeCastSettings, backFaceModeConvex, 32);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_ShapeCastSettings, useShrunkenShapeAndConvexRadius, 36);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_ShapeCastSettings, returnDeepestPoint, 37);
JOLTPHYSICS_SYS_ASSERT_LAYOUT(JPH_ShapeCastResult, 60, 4);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_ShapeCastResult, contactPointOn1, 0);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_ShapeCastResult, contactPointOn2, 12);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_ShapeCastResult, penetrationAxis, 24);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_ShapeCastResult, penetrationDepth, 36);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_ShapeCastResult, subShapeID1, 40);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_ShapeCastResult, subShapeID2, 44);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_ShapeCastResult, bodyID2, 48);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_ShapeCastResult, fraction, 52);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_ShapeCastResult, isBackFaceHit, 56);

// Character values. joltc converts these field by field, so only the C ABI is
// pinned here, not agreement with the C++ layout.
JOLTPHYSICS_SYS_ASSERT_LAYOUT(JPH_Plane, 16, 4);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_Plane, normal, 0);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_Plane, distance, 12);
JOLTPHYSICS_SYS_ASSERT_LAYOUT(JPH_ExtendedUpdateSettings, 48, 4);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_ExtendedUpdateSettings, stickToFloorStepDown, 0);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_ExtendedUpdateSettings, walkStairsStepUp, 12);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_ExtendedUpdateSettings, walkStairsMinStepForward, 24);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_ExtendedUpdateSettings, walkStairsStepForwardTest, 28);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_ExtendedUpdateSettings, walkStairsCosAngleForwardContact, 32);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_ExtendedUpdateSettings, walkStairsStepDownExtra, 36);

JOLTPHYSICS_SYS_ASSERT_LAYOUT(JobSystemThreadPoolConfig, 12, 4);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JobSystemThreadPoolConfig, numThreads, 8);

#if UINTPTR_MAX == UINT64_MAX
JOLTPHYSICS_SYS_ASSERT_LAYOUT(JPH_PhysicsSystemSettings, 48, 8);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_PhysicsSystemSettings, maxContactConstraints, 12);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_PhysicsSystemSettings, broadPhaseLayerInterface, 24);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_PhysicsSystemSettings, objectLayerPairFilter, 32);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_PhysicsSystemSettings, objectVsBroadPhaseLayerFilter, 40);

JOLTPHYSICS_SYS_ASSERT_LAYOUT(JPH_CollideShapeResult, 80, 8);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CollideShapeResult, contactPointOn1, 0);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CollideShapeResult, contactPointOn2, 12);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CollideShapeResult, penetrationAxis, 24);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CollideShapeResult, penetrationDepth, 36);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CollideShapeResult, subShapeID1, 40);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CollideShapeResult, subShapeID2, 44);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CollideShapeResult, bodyID2, 48);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CollideShapeResult, shape1FaceCount, 52);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CollideShapeResult, shape1Faces, 56);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CollideShapeResult, shape2FaceCount, 64);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CollideShapeResult, shape2Faces, 72);

// Filter proc tables that joltphysics fills with its callbacks.
JOLTPHYSICS_SYS_ASSERT_LAYOUT(JPH_ObjectLayerFilter_Procs, 8, 8);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_ObjectLayerFilter_Procs, ShouldCollide, 0);
JOLTPHYSICS_SYS_ASSERT_LAYOUT(JPH_BodyFilter_Procs, 16, 8);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_BodyFilter_Procs, ShouldCollide, 0);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_BodyFilter_Procs, ShouldCollideLocked, 8);
JOLTPHYSICS_SYS_ASSERT_LAYOUT(JPH_ShapeFilter_Procs, 16, 8);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_ShapeFilter_Procs, ShouldCollide, 0);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_ShapeFilter_Procs, ShouldCollide2, 8);

#ifdef JPH_DEBUG_RENDERER
// Debug renderer proc table that joltphysics fills with its line callback.
JOLTPHYSICS_SYS_ASSERT_LAYOUT(JPH_DebugRenderer_Procs, 24, 8);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_DebugRenderer_Procs, DrawLine, 0);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_DebugRenderer_Procs, DrawTriangle, 8);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_DebugRenderer_Procs, DrawText3D, 16);
#endif

// Character settings and contacts, which hold pointers. joltc converts these
// field by field, so only the C ABI is pinned here.
JOLTPHYSICS_SYS_ASSERT_LAYOUT(JPH_CharacterBaseSettings, 48, 8);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterBaseSettings, up, 0);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterBaseSettings, supportingVolume, 12);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterBaseSettings, maxSlopeAngle, 28);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterBaseSettings, enhancedInternalEdgeRemoval, 32);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterBaseSettings, shape, 40);
JOLTPHYSICS_SYS_ASSERT_LAYOUT(JPH_CharacterVirtualSettings, 128, 8);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, base, 0);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, ID, 48);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, mass, 52);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, maxStrength, 56);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, shapeOffset, 60);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, backFaceMode, 72);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, predictiveContactDistance, 76);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, maxCollisionIterations, 80);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, maxConstraintIterations, 84);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, minTimeRemaining, 88);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, collisionTolerance, 92);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, characterPadding, 96);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, maxNumHits, 100);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, hitReductionCosMaxAngle, 104);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, penetrationRecoverySpeed, 108);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, innerBodyShape, 112);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, innerBodyIDOverride, 120);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, innerBodyLayer, 124);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterContact, hash, 0);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterContact, bodyB, 8);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterContact, characterIDB, 12);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterContact, subShapeIDB, 16);
#ifdef JPH_DOUBLE_PRECISION
JOLTPHYSICS_SYS_ASSERT_LAYOUT(JPH_CharacterContact, 136, 8);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterContact, position, 24);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterContact, linearVelocity, 48);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterContact, contactNormal, 60);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterContact, surfaceNormal, 72);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterContact, distance, 84);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterContact, fraction, 88);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterContact, motionTypeB, 92);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterContact, isSensorB, 96);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterContact, characterB, 104);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterContact, userData, 112);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterContact, material, 120);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterContact, hadCollision, 128);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterContact, wasDiscarded, 129);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterContact, canPushCharacter, 130);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterContact, isBackFacingContact, 131);
#else
JOLTPHYSICS_SYS_ASSERT_LAYOUT(JPH_CharacterContact, 120, 8);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterContact, position, 20);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterContact, linearVelocity, 32);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterContact, contactNormal, 44);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterContact, surfaceNormal, 56);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterContact, distance, 68);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterContact, fraction, 72);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterContact, motionTypeB, 76);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterContact, isSensorB, 80);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterContact, characterB, 88);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterContact, userData, 96);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterContact, material, 104);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterContact, hadCollision, 112);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterContact, wasDiscarded, 113);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterContact, canPushCharacter, 114);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_CharacterContact, isBackFacingContact, 115);
#endif
#endif

// joltc copies JPH_Mat4 to and from JPH::Mat44 with memcpy, so the sizes must
// agree. The alignments do not: Mat44 is 16-aligned while JPH_Mat4 is
// 4-aligned, which is why build.rs leaves out the joltc functions that
// reinterpret_cast JPH_Mat4 arrays to Mat44 arrays.
static_assert(sizeof(JPH_Mat4) == sizeof(JPH::Mat44), "JPH_Mat4 must have the size of JPH::Mat44");
static_assert(alignof(JPH::Mat44) == 16, "JPH::Mat44 is expected to be 16-aligned");
// In double precision joltc converts JPH_RVec3 and JPH_RMat4 field by field,
// so they have no C++ mirror to check against.

// Scalar handles cast between C and C++.
static_assert(sizeof(JPH_BodyID) == sizeof(JPH::BodyID), "JPH_BodyID must have the size of JPH::BodyID");
static_assert(sizeof(JPH_BodyID) == 4, "JPH_BodyID is expected to be 4 bytes");
static_assert(sizeof(JPH_SubShapeID) == sizeof(JPH::SubShapeID), "JPH_SubShapeID must have the size of JPH::SubShapeID");
static_assert(sizeof(JPH_SubShapeID) == 4, "JPH_SubShapeID is expected to be 4 bytes");
static_assert(sizeof(JPH_ObjectLayer) == sizeof(JPH::ObjectLayer), "JPH_ObjectLayer must match Jolt's OBJECT_LAYER_BITS");
static_assert(sizeof(JPH_BroadPhaseLayer) == sizeof(JPH::BroadPhaseLayer::Type), "JPH_BroadPhaseLayer must match JPH::BroadPhaseLayer::Type");

// Enums cast across the boundary: 4 bytes each and every enumerator equal to
// Jolt's value.
static_assert(sizeof(JPH_MotionType) == 4, "JPH_MotionType: unexpected size");
static_assert(sizeof(JPH_Activation) == 4, "JPH_Activation: unexpected size");
static_assert(sizeof(JPH_PhysicsUpdateError) == 4, "JPH_PhysicsUpdateError: unexpected size");
static_assert(sizeof(JPH_MotionQuality) == 4, "JPH_MotionQuality: unexpected size");
static_assert(sizeof(JPH_OverrideMassProperties) == 4, "JPH_OverrideMassProperties: unexpected size");
static_assert(sizeof(JPH_ShapeSubType) == 4, "JPH_ShapeSubType: unexpected size");
static_assert(sizeof(JPH_BackFaceMode) == 4, "JPH_BackFaceMode: unexpected size");
static_assert(sizeof(JPH_ActiveEdgeMode) == 4, "JPH_ActiveEdgeMode: unexpected size");
static_assert(sizeof(JPH_CollectFacesMode) == 4, "JPH_CollectFacesMode: unexpected size");
static_assert(sizeof(JPH_CollisionCollectorType) == 4, "JPH_CollisionCollectorType: unexpected size");
static_assert(sizeof(JPH_GroundState) == 4, "JPH_GroundState: unexpected size");

static_assert(int(JPH_MotionType_Static) == int(JPH::EMotionType::Static), "JPH_MotionType_Static");
static_assert(int(JPH_MotionType_Kinematic) == int(JPH::EMotionType::Kinematic), "JPH_MotionType_Kinematic");
static_assert(int(JPH_MotionType_Dynamic) == int(JPH::EMotionType::Dynamic), "JPH_MotionType_Dynamic");

static_assert(int(JPH_Activation_Activate) == int(JPH::EActivation::Activate), "JPH_Activation_Activate");
static_assert(int(JPH_Activation_DontActivate) == int(JPH::EActivation::DontActivate), "JPH_Activation_DontActivate");

static_assert(int(JPH_PhysicsUpdateError_None) == int(JPH::EPhysicsUpdateError::None), "JPH_PhysicsUpdateError_None");
static_assert(int(JPH_PhysicsUpdateError_ManifoldCacheFull) == int(JPH::EPhysicsUpdateError::ManifoldCacheFull),
              "JPH_PhysicsUpdateError_ManifoldCacheFull");
static_assert(int(JPH_PhysicsUpdateError_BodyPairCacheFull) == int(JPH::EPhysicsUpdateError::BodyPairCacheFull),
              "JPH_PhysicsUpdateError_BodyPairCacheFull");
static_assert(int(JPH_PhysicsUpdateError_ContactConstraintsFull) == int(JPH::EPhysicsUpdateError::ContactConstraintsFull),
              "JPH_PhysicsUpdateError_ContactConstraintsFull");

// Jolt declares these as uint8 enums; joltc casts the 4-byte C values.
static_assert(int(JPH_MotionQuality_Discrete) == int(JPH::EMotionQuality::Discrete), "JPH_MotionQuality_Discrete");
static_assert(int(JPH_MotionQuality_LinearCast) == int(JPH::EMotionQuality::LinearCast), "JPH_MotionQuality_LinearCast");

static_assert(int(JPH_OverrideMassProperties_CalculateMassAndInertia) == int(JPH::EOverrideMassProperties::CalculateMassAndInertia),
              "JPH_OverrideMassProperties_CalculateMassAndInertia");
static_assert(int(JPH_OverrideMassProperties_CalculateInertia) == int(JPH::EOverrideMassProperties::CalculateInertia),
              "JPH_OverrideMassProperties_CalculateInertia");
static_assert(int(JPH_OverrideMassProperties_MassAndInertiaProvided) == int(JPH::EOverrideMassProperties::MassAndInertiaProvided),
              "JPH_OverrideMassProperties_MassAndInertiaProvided");

// Jolt declares EShapeSubType as a uint8 enum; joltc casts it to the 4-byte C value.
static_assert(int(JPH_ShapeSubType_Sphere) == int(JPH::EShapeSubType::Sphere), "JPH_ShapeSubType_Sphere");
static_assert(int(JPH_ShapeSubType_Box) == int(JPH::EShapeSubType::Box), "JPH_ShapeSubType_Box");
static_assert(int(JPH_ShapeSubType_Capsule) == int(JPH::EShapeSubType::Capsule), "JPH_ShapeSubType_Capsule");
static_assert(int(JPH_ShapeSubType_Cylinder) == int(JPH::EShapeSubType::Cylinder), "JPH_ShapeSubType_Cylinder");
static_assert(int(JPH_ShapeSubType_StaticCompound) == int(JPH::EShapeSubType::StaticCompound), "JPH_ShapeSubType_StaticCompound");
static_assert(int(JPH_ShapeSubType_MutableCompound) == int(JPH::EShapeSubType::MutableCompound), "JPH_ShapeSubType_MutableCompound");
static_assert(int(JPH_ShapeSubType_HeightField) == int(JPH::EShapeSubType::HeightField), "JPH_ShapeSubType_HeightField");

// Jolt declares these as uint8 enums; joltc casts the 4-byte C values.
// JPH_CollisionCollectorType is joltc's own and has no Jolt counterpart.
static_assert(int(JPH_BackFaceMode_IgnoreBackFaces) == int(JPH::EBackFaceMode::IgnoreBackFaces), "JPH_BackFaceMode_IgnoreBackFaces");
static_assert(int(JPH_BackFaceMode_CollideWithBackFaces) == int(JPH::EBackFaceMode::CollideWithBackFaces), "JPH_BackFaceMode_CollideWithBackFaces");
static_assert(int(JPH_ActiveEdgeMode_CollideOnlyWithActive) == int(JPH::EActiveEdgeMode::CollideOnlyWithActive), "JPH_ActiveEdgeMode_CollideOnlyWithActive");
static_assert(int(JPH_ActiveEdgeMode_CollideWithAll) == int(JPH::EActiveEdgeMode::CollideWithAll), "JPH_ActiveEdgeMode_CollideWithAll");
static_assert(int(JPH_CollectFacesMode_CollectFaces) == int(JPH::ECollectFacesMode::CollectFaces), "JPH_CollectFacesMode_CollectFaces");
static_assert(int(JPH_CollectFacesMode_NoFaces) == int(JPH::ECollectFacesMode::NoFaces), "JPH_CollectFacesMode_NoFaces");

// Jolt declares EGroundState as a uint8 enum; joltc casts it to the 4-byte C value.
static_assert(int(JPH_GroundState_OnGround) == int(JPH::CharacterBase::EGroundState::OnGround), "JPH_GroundState_OnGround");
static_assert(int(JPH_GroundState_OnSteepGround) == int(JPH::CharacterBase::EGroundState::OnSteepGround), "JPH_GroundState_OnSteepGround");
static_assert(int(JPH_GroundState_NotSupported) == int(JPH::CharacterBase::EGroundState::NotSupported), "JPH_GroundState_NotSupported");
static_assert(int(JPH_GroundState_InAir) == int(JPH::CharacterBase::EGroundState::InAir), "JPH_GroundState_InAir");
