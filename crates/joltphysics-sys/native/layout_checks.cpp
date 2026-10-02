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
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
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

JOLTPHYSICS_SYS_ASSERT_LAYOUT(JobSystemThreadPoolConfig, 12, 4);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JobSystemThreadPoolConfig, numThreads, 8);

#if UINTPTR_MAX == UINT64_MAX
JOLTPHYSICS_SYS_ASSERT_LAYOUT(JPH_PhysicsSystemSettings, 48, 8);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_PhysicsSystemSettings, maxContactConstraints, 12);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_PhysicsSystemSettings, broadPhaseLayerInterface, 24);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_PhysicsSystemSettings, objectLayerPairFilter, 32);
JOLTPHYSICS_SYS_ASSERT_OFFSET(JPH_PhysicsSystemSettings, objectVsBroadPhaseLayerFilter, 40);
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
