// Functions this fork adds to joltc, in joltc's naming and style. They are compiled into the
// joltc archive (see ../CMakeLists.txt) and are meant to be offered upstream.
// Licensed under the MIT License (MIT), like joltc.
//
// This translation unit does not include joltc.cpp. It converts joltc handles the way joltc
// does: filters, the temp allocator and the character are reinterpret_casts of the Jolt objects
// they stand for (joltc's DEF_MAP_DECL and its Managed*Filter classes, which derive from the
// Jolt filter classes with single inheritance). The vehicle constraint is a reinterpret_cast of
// the most derived object, as in joltc; its bases are reached with static_cast, because
// VehicleConstraint inherits from both Constraint and PhysicsStepListener. Body creation settings,
// ragdoll settings and the swing-twist, hinge and six-DOF constraints are reinterpret_casts of the
// Jolt objects, as joltc's DEF_MAP_DECL defines them. The path, pulley and rack-and-pinion
// constraints follow the same convention: each handle is a reinterpret_cast of the most derived
// object, and Constraint is the first base of their single-inheritance chain, so the same address
// is the JPH_Constraint joltc's AsConstraint expects. A path handle is a JPH::PathConstraintPath*
// (the base that carries the reference count), never a pointer to a derived path.
//
// Unlike the other handles, a JPH_PhysicsSystem is joltc's own wrapper struct around the Jolt
// system, which is reached through its physicsSystem member. The struct's definition comes from a
// header that the build generates from joltc.cpp at configure time (joltc_physics_system.h), so
// this translation unit and joltc.cpp define the same type.

#include <Jolt/Jolt.h>

#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyFilter.h>
#include <Jolt/Physics/Character/CharacterVirtual.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <Jolt/Physics/Collision/ShapeFilter.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/PathConstraint.h>
#include <Jolt/Physics/Constraints/PathConstraintPathHermite.h>
#include <Jolt/Physics/Constraints/PulleyConstraint.h>
#include <Jolt/Physics/Constraints/RackAndPinionConstraint.h>
#include <Jolt/Physics/Constraints/SixDOFConstraint.h>
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>
#include <Jolt/Physics/Ragdoll/Ragdoll.h>
#include <Jolt/Physics/StateRecorderImpl.h>
#include <Jolt/Physics/Vehicle/VehicleConstraint.h>

#include <algorithm>
#include <cstring>
#include <string>
#include <vector>

#include "joltc_ext.h"
#include "joltc_physics_system.h"

// joltc's own conversions of the constraint settings, defined in joltc.cpp at global scope with C++
// linkage but not declared in joltc.h. Declaring them here keeps one conversion shared with joltc's
// JPH_*Constraint_Create functions.
void JPH_SwingTwistConstraintSettings_ToJolt(JPH::SwingTwistConstraintSettings* joltSettings, const JPH_SwingTwistConstraintSettings* settings);
void JPH_HingeConstraintSettings_ToJolt(JPH::HingeConstraintSettings* joltSettings, const JPH_HingeConstraintSettings* settings);
void JPH_SixDOFConstraintSettings_ToJolt(JPH::SixDOFConstraintSettings* joltSettings, const JPH_SixDOFConstraintSettings* settings);
void JPH_ConstraintSettings_Init(const JPH::ConstraintSettings& joltSettings, JPH_ConstraintSettings* settings);
void JPH_ConstraintSettings_ToJolt(JPH::ConstraintSettings* joltSettings, const JPH_ConstraintSettings* settings);

namespace
{
	// JPH_Vec3 is three floats; JPH::Vec3 is a 16-byte SIMD type, so it is built, never cast.
	JPH::Vec3 ToVec3(const JPH_Vec3& vec)
	{
		return JPH::Vec3(vec.x, vec.y, vec.z);
	}

	// A null filter accepts everything, as in joltc.
	const JPH::BroadPhaseLayerFilter& ToJolt(const JPH_BroadPhaseLayerFilter* filter)
	{
		static const JPH::BroadPhaseLayerFilter accept_all{};
		return filter ? *reinterpret_cast<const JPH::BroadPhaseLayerFilter*>(filter) : accept_all;
	}

	const JPH::ObjectLayerFilter& ToJolt(const JPH_ObjectLayerFilter* filter)
	{
		static const JPH::ObjectLayerFilter accept_all{};
		return filter ? *reinterpret_cast<const JPH::ObjectLayerFilter*>(filter) : accept_all;
	}

	const JPH::BodyFilter& ToJolt(const JPH_BodyFilter* filter)
	{
		static const JPH::BodyFilter accept_all{};
		return filter ? *reinterpret_cast<const JPH::BodyFilter*>(filter) : accept_all;
	}

	const JPH::ShapeFilter& ToJolt(const JPH_ShapeFilter* filter)
	{
		static const JPH::ShapeFilter accept_all{};
		return filter ? *reinterpret_cast<const JPH::ShapeFilter*>(filter) : accept_all;
	}

	// The handle is always the most derived object. StateRecorderImpl inherits StreamIn and
	// StreamOut through StateRecorder, so it is passed on as *impl through the implicit upcast and
	// never cast to a base directly.
	JPH::StateRecorderImpl* AsStateRecorder(JPH_StateRecorder* recorder)
	{
		return reinterpret_cast<JPH::StateRecorderImpl*>(recorder);
	}

	const JPH::StateRecorderImpl* AsStateRecorder(const JPH_StateRecorder* recorder)
	{
		return reinterpret_cast<const JPH::StateRecorderImpl*>(recorder);
	}

	JPH::PhysicsSystem& AsJoltPhysicsSystem(JPH_PhysicsSystem* system)
	{
		return *system->physicsSystem;
	}

	const JPH::PhysicsSystem& AsJoltPhysicsSystem(const JPH_PhysicsSystem* system)
	{
		return *system->physicsSystem;
	}

	// Saves only the bodies whose id is in the list. The list is copied, sorted and deduplicated
	// here, so callers may pass the ids in any order.
	class SortedBodyIdFilter final : public JPH::StateRecorderFilter
	{
	public:
		SortedBodyIdFilter(const JPH_BodyID* bodies, uint32_t bodyCount)
		{
			// The pointer of an empty list need not point to any object (Rust passes a dangling
			// one for an empty slice), so it takes part in no pointer arithmetic.
			if (bodyCount == 0)
				return;
			mBodies.assign(bodies, bodies + bodyCount);
			std::sort(mBodies.begin(), mBodies.end());
			mBodies.erase(std::unique(mBodies.begin(), mBodies.end()), mBodies.end());
		}

		bool ShouldSaveBody(const JPH::Body& inBody) const override
		{
			return std::binary_search(mBodies.begin(), mBodies.end(), inBody.GetID().GetIndexAndSequenceNumber());
		}

	private:
		std::vector<JPH::uint32> mBodies;
	};

	JPH::CharacterVirtual* AsCharacterVirtual(JPH_CharacterVirtual* character)
	{
		return reinterpret_cast<JPH::CharacterVirtual*>(character);
	}

	const JPH::CharacterVirtual* AsCharacterVirtual(const JPH_CharacterVirtual* character)
	{
		return reinterpret_cast<const JPH::CharacterVirtual*>(character);
	}

	JPH::TempAllocator& AsTempAllocator(JPH_TempAllocator* allocator)
	{
		return *reinterpret_cast<JPH::TempAllocator*>(allocator);
	}

	JPH::RagdollSettings* AsRagdollSettings(JPH_RagdollSettings* settings)
	{
		return reinterpret_cast<JPH::RagdollSettings*>(settings);
	}

	JPH::SwingTwistConstraint* AsSwingTwistConstraint(JPH_SwingTwistConstraint* constraint)
	{
		return reinterpret_cast<JPH::SwingTwistConstraint*>(constraint);
	}

	const JPH::SwingTwistConstraint* AsSwingTwistConstraint(const JPH_SwingTwistConstraint* constraint)
	{
		return reinterpret_cast<const JPH::SwingTwistConstraint*>(constraint);
	}

	JPH::HingeConstraint* AsHingeConstraint(JPH_HingeConstraint* constraint)
	{
		return reinterpret_cast<JPH::HingeConstraint*>(constraint);
	}

	// JPH_Quat is four floats; JPH::Quat is a 16-byte SIMD type, so it is built, never cast.
	JPH::Quat ToQuat(const JPH_Quat& quat)
	{
		return JPH::Quat(quat.x, quat.y, quat.z, quat.w);
	}

	void FromQuat(JPH::QuatArg quat, JPH_Quat* result)
	{
		result->x = quat.GetX();
		result->y = quat.GetY();
		result->z = quat.GetZ();
		result->w = quat.GetW();
	}

	void FromVec3(JPH::Vec3Arg vec, JPH_Vec3* result)
	{
		result->x = vec.GetX();
		result->y = vec.GetY();
		result->z = vec.GetZ();
	}

	// JPH_RVec3 is three Reals and JPH::RVec3 a SIMD type, so it is built field by field.
	JPH::RVec3 ToRVec3(const JPH_RVec3& vec)
	{
		return JPH::RVec3(vec.x, vec.y, vec.z);
	}

	void FromRVec3(JPH::RVec3Arg vec, JPH_RVec3* result)
	{
		result->x = vec.GetX();
		result->y = vec.GetY();
		result->z = vec.GetZ();
	}

	// joltc's spring and motor conversions are static in joltc.cpp, so this translation unit has
	// its own, field by field like joltc's.
	JPH::SpringSettings ToSpring(const JPH_SpringSettings& settings)
	{
		JPH::SpringSettings result;
		result.mMode = static_cast<JPH::ESpringMode>(settings.mode);
		result.mFrequency = settings.frequencyOrStiffness;
		result.mDamping = settings.damping;
		return result;
	}

	void FromSpring(const JPH::SpringSettings& settings, JPH_SpringSettings* result)
	{
		result->mode = static_cast<JPH_SpringMode>(settings.mMode);
		result->frequencyOrStiffness = settings.mFrequency;
		result->damping = settings.mDamping;
	}

	JPH::MotorSettings ToMotor(const JPH_MotorSettings& settings)
	{
		JPH::MotorSettings result;
		result.mSpringSettings = ToSpring(settings.springSettings);
		result.mMinForceLimit = settings.minForceLimit;
		result.mMaxForceLimit = settings.maxForceLimit;
		result.mMinTorqueLimit = settings.minTorqueLimit;
		result.mMaxTorqueLimit = settings.maxTorqueLimit;
		return result;
	}

	void FromMotor(const JPH::MotorSettings& settings, JPH_MotorSettings* result)
	{
		FromSpring(settings.mSpringSettings, &result->springSettings);
		result->minForceLimit = settings.mMinForceLimit;
		result->maxForceLimit = settings.mMaxForceLimit;
		result->minTorqueLimit = settings.mMinTorqueLimit;
		result->maxTorqueLimit = settings.mMaxTorqueLimit;
	}

	JPH::SixDOFConstraint* AsSixDOFConstraint(JPH_SixDOFConstraint* constraint)
	{
		return reinterpret_cast<JPH::SixDOFConstraint*>(constraint);
	}

	JPH::PathConstraintPath* AsPath(JPH_PathConstraintPath* path)
	{
		return reinterpret_cast<JPH::PathConstraintPath*>(path);
	}

	const JPH::PathConstraintPath* AsPath(const JPH_PathConstraintPath* path)
	{
		return reinterpret_cast<const JPH::PathConstraintPath*>(path);
	}

	JPH::PathConstraint* AsPathConstraint(JPH_PathConstraint* constraint)
	{
		return reinterpret_cast<JPH::PathConstraint*>(constraint);
	}

	const JPH::PathConstraint* AsPathConstraint(const JPH_PathConstraint* constraint)
	{
		return reinterpret_cast<const JPH::PathConstraint*>(constraint);
	}

	JPH::PulleyConstraint* AsPulleyConstraint(JPH_PulleyConstraint* constraint)
	{
		return reinterpret_cast<JPH::PulleyConstraint*>(constraint);
	}

	const JPH::PulleyConstraint* AsPulleyConstraint(const JPH_PulleyConstraint* constraint)
	{
		return reinterpret_cast<const JPH::PulleyConstraint*>(constraint);
	}

	JPH::RackAndPinionConstraint* AsRackAndPinionConstraint(JPH_RackAndPinionConstraint* constraint)
	{
		return reinterpret_cast<JPH::RackAndPinionConstraint*>(constraint);
	}

	const JPH::RackAndPinionConstraint* AsRackAndPinionConstraint(const JPH_RackAndPinionConstraint* constraint)
	{
		return reinterpret_cast<const JPH::RackAndPinionConstraint*>(constraint);
	}

	const JPH::Constraint* AsConstraint(const JPH_Constraint* constraint)
	{
		return reinterpret_cast<const JPH::Constraint*>(constraint);
	}

	// Creates the constraint joltSettings describe between the two bodies and returns it holding
	// one reference, as joltc's JPH_*Constraint_Create functions do.
	template <class JoltConstraint, class Handle, class JoltSettings>
	Handle* CreateConstraint(const JoltSettings& joltSettings, JPH_Body* body1, JPH_Body* body2)
	{
		JPH_ASSERT(body1 && body2);

		JPH::Body& joltBody1 = *reinterpret_cast<JPH::Body*>(body1);
		JPH::Body& joltBody2 = *reinterpret_cast<JPH::Body*>(body2);
		JoltConstraint* constraint = static_cast<JoltConstraint*>(joltSettings.Create(joltBody1, joltBody2));
		constraint->AddRef();
		return reinterpret_cast<Handle*>(constraint);
	}

	// Replaces the constraint of part partIndex to its parent by a new JoltSettings that convert
	// fills from constraintSettings; null removes it. The part's Ref takes the reference.
	template <class JoltSettings, class CSettings, class Convert>
	void SetPartToParent(JPH_RagdollSettings* settings, int partIndex, const CSettings* constraintSettings, Convert convert)
	{
		JPH::RagdollSettings::Part& part = AsRagdollSettings(settings)->mParts[partIndex];
		if (constraintSettings == nullptr)
		{
			part.mToParent = nullptr;
			return;
		}

		JoltSettings* joltSettings = new JoltSettings();
		convert(joltSettings, constraintSettings);
		part.mToParent = joltSettings;
	}
}

/* StateRecorder */
JPH_StateRecorder* JPH_StateRecorder_Create(void)
{
	return reinterpret_cast<JPH_StateRecorder*>(new JPH::StateRecorderImpl());
}

void JPH_StateRecorder_Destroy(JPH_StateRecorder* recorder)
{
	delete AsStateRecorder(recorder);
}

void JPH_StateRecorder_Rewind(JPH_StateRecorder* recorder)
{
	AsStateRecorder(recorder)->Rewind();
}

void JPH_StateRecorder_WriteBytes(JPH_StateRecorder* recorder, const void* data, size_t size)
{
	AsStateRecorder(recorder)->WriteBytes(data, size);
}

size_t JPH_StateRecorder_GetDataSize(JPH_StateRecorder* recorder)
{
	return AsStateRecorder(recorder)->GetDataSize();
}

void JPH_StateRecorder_CopyData(JPH_StateRecorder* recorder, void* data, size_t size)
{
	std::string recorded = AsStateRecorder(recorder)->GetData();
	size_t count = std::min(size, recorded.size());
	if (count == 0)
		return; // data may be null then, and memcpy needs valid pointers even for no bytes
	memcpy(data, recorded.data(), count);
}

bool JPH_StateRecorder_IsFailed(const JPH_StateRecorder* recorder)
{
	return AsStateRecorder(recorder)->IsFailed();
}

/* PhysicsSystem state */
static_assert(static_cast<int>(JPH::EStateRecorderState::None) == JPH_StateRecorderState_None);
static_assert(static_cast<int>(JPH::EStateRecorderState::Global) == JPH_StateRecorderState_Global);
static_assert(static_cast<int>(JPH::EStateRecorderState::Bodies) == JPH_StateRecorderState_Bodies);
static_assert(static_cast<int>(JPH::EStateRecorderState::Contacts) == JPH_StateRecorderState_Contacts);
static_assert(static_cast<int>(JPH::EStateRecorderState::Constraints) == JPH_StateRecorderState_Constraints);
static_assert(static_cast<int>(JPH::EStateRecorderState::All) == JPH_StateRecorderState_All);

void JPH_PhysicsSystem_SaveState(const JPH_PhysicsSystem* system, JPH_StateRecorder* recorder, JPH_StateRecorderState state, const JPH_BodyID* bodies, uint32_t bodyCount)
{
	JPH_ASSERT(bodies != nullptr || bodyCount == 0);

	const JPH::EStateRecorderState joltState = static_cast<JPH::EStateRecorderState>(state);
	if (bodies == nullptr)
	{
		AsJoltPhysicsSystem(system).SaveState(*AsStateRecorder(recorder), joltState, nullptr);
		return;
	}

	SortedBodyIdFilter filter(bodies, bodyCount);
	AsJoltPhysicsSystem(system).SaveState(*AsStateRecorder(recorder), joltState, &filter);
}

bool JPH_PhysicsSystem_RestoreState(JPH_PhysicsSystem* system, JPH_StateRecorder* recorder)
{
	return AsJoltPhysicsSystem(system).RestoreState(*AsStateRecorder(recorder));
}

/* CharacterVirtual */
void JPH_CharacterVirtual_SaveState(const JPH_CharacterVirtual* character, JPH_StateRecorder* recorder)
{
	AsCharacterVirtual(character)->SaveState(*AsStateRecorder(recorder));
}

void JPH_CharacterVirtual_RestoreState(JPH_CharacterVirtual* character, JPH_StateRecorder* recorder)
{
	AsCharacterVirtual(character)->RestoreState(*AsStateRecorder(recorder));
}

void JPH_CharacterVirtual_ExtendedUpdate2(JPH_CharacterVirtual* character, float deltaTime,
	const JPH_Vec3* gravity, const JPH_ExtendedUpdateSettings* settings,
	const JPH_BroadPhaseLayerFilter* broadPhaseLayerFilter, const JPH_ObjectLayerFilter* objectLayerFilter,
	const JPH_BodyFilter* bodyFilter, const JPH_ShapeFilter* shapeFilter, JPH_TempAllocator* tempAllocator)
{
	JPH_ASSERT(settings && gravity && tempAllocator);

	JPH::CharacterVirtual::ExtendedUpdateSettings joltSettings = {};
	joltSettings.mStickToFloorStepDown = ToVec3(settings->stickToFloorStepDown);
	joltSettings.mWalkStairsStepUp = ToVec3(settings->walkStairsStepUp);
	joltSettings.mWalkStairsMinStepForward = settings->walkStairsMinStepForward;
	joltSettings.mWalkStairsStepForwardTest = settings->walkStairsStepForwardTest;
	joltSettings.mWalkStairsCosAngleForwardContact = settings->walkStairsCosAngleForwardContact;
	joltSettings.mWalkStairsStepDownExtra = ToVec3(settings->walkStairsStepDownExtra);

	AsCharacterVirtual(character)->ExtendedUpdate(deltaTime,
		ToVec3(*gravity),
		joltSettings,
		ToJolt(broadPhaseLayerFilter),
		ToJolt(objectLayerFilter),
		ToJolt(bodyFilter),
		ToJolt(shapeFilter),
		AsTempAllocator(tempAllocator)
	);
}

void JPH_CharacterVirtual_RefreshContacts2(JPH_CharacterVirtual* character,
	const JPH_BroadPhaseLayerFilter* broadPhaseLayerFilter, const JPH_ObjectLayerFilter* objectLayerFilter,
	const JPH_BodyFilter* bodyFilter, const JPH_ShapeFilter* shapeFilter, JPH_TempAllocator* tempAllocator)
{
	JPH_ASSERT(tempAllocator);

	AsCharacterVirtual(character)->RefreshContacts(
		ToJolt(broadPhaseLayerFilter),
		ToJolt(objectLayerFilter),
		ToJolt(bodyFilter),
		ToJolt(shapeFilter),
		AsTempAllocator(tempAllocator)
	);
}

/* VehicleConstraint */
JPH_Constraint* JPH_VehicleConstraint_AsConstraint(JPH_VehicleConstraint* constraint)
{
	JPH_ASSERT(constraint);

	JPH::Constraint* joltConstraint = static_cast<JPH::Constraint*>(reinterpret_cast<JPH::VehicleConstraint*>(constraint));
	return reinterpret_cast<JPH_Constraint*>(joltConstraint);
}

/* RagdollSettings */
void JPH_RagdollSettings_SetPart(JPH_RagdollSettings* settings, int partIndex, const JPH_BodyCreationSettings* bodySettings)
{
	JPH_ASSERT(settings && bodySettings);

	// Assigns the BodyCreationSettings base only, so the part keeps its mToParent.
	static_cast<JPH::BodyCreationSettings&>(AsRagdollSettings(settings)->mParts[partIndex]) =
		*reinterpret_cast<const JPH::BodyCreationSettings*>(bodySettings);
}

void JPH_RagdollSettings_SetPartToParentSwingTwist(JPH_RagdollSettings* settings, int partIndex, const JPH_SwingTwistConstraintSettings* constraintSettings)
{
	SetPartToParent<JPH::SwingTwistConstraintSettings>(settings, partIndex, constraintSettings, JPH_SwingTwistConstraintSettings_ToJolt);
}

void JPH_RagdollSettings_SetPartToParentHinge(JPH_RagdollSettings* settings, int partIndex, const JPH_HingeConstraintSettings* constraintSettings)
{
	SetPartToParent<JPH::HingeConstraintSettings>(settings, partIndex, constraintSettings, JPH_HingeConstraintSettings_ToJolt);
}

void JPH_RagdollSettings_SetPartToParentSixDOF(JPH_RagdollSettings* settings, int partIndex, const JPH_SixDOFConstraintSettings* constraintSettings)
{
	SetPartToParent<JPH::SixDOFConstraintSettings>(settings, partIndex, constraintSettings, JPH_SixDOFConstraintSettings_ToJolt);
}

void JPH_RagdollSettings_CalculateConstraintPriorities(JPH_RagdollSettings* settings, uint32_t basePriority)
{
	AsRagdollSettings(settings)->CalculateConstraintPriorities(basePriority);
}

/* SwingTwistConstraint */
void JPH_SwingTwistConstraint_SetSwingMotorState(JPH_SwingTwistConstraint* constraint, JPH_MotorState state)
{
	AsSwingTwistConstraint(constraint)->SetSwingMotorState(static_cast<JPH::EMotorState>(state));
}

JPH_MotorState JPH_SwingTwistConstraint_GetSwingMotorState(const JPH_SwingTwistConstraint* constraint)
{
	return static_cast<JPH_MotorState>(AsSwingTwistConstraint(constraint)->GetSwingMotorState());
}

void JPH_SwingTwistConstraint_SetTwistMotorState(JPH_SwingTwistConstraint* constraint, JPH_MotorState state)
{
	AsSwingTwistConstraint(constraint)->SetTwistMotorState(static_cast<JPH::EMotorState>(state));
}

JPH_MotorState JPH_SwingTwistConstraint_GetTwistMotorState(const JPH_SwingTwistConstraint* constraint)
{
	return static_cast<JPH_MotorState>(AsSwingTwistConstraint(constraint)->GetTwistMotorState());
}

void JPH_SwingTwistConstraint_SetTargetOrientationBS(JPH_SwingTwistConstraint* constraint, const JPH_Quat* orientation)
{
	JPH_ASSERT(orientation);

	AsSwingTwistConstraint(constraint)->SetTargetOrientationBS(ToQuat(*orientation));
}

void JPH_SwingTwistConstraint_GetRotationInConstraintSpace(const JPH_SwingTwistConstraint* constraint, JPH_Quat* result)
{
	JPH_ASSERT(result);

	FromQuat(AsSwingTwistConstraint(constraint)->GetRotationInConstraintSpace(), result);
}

/* HingeConstraint */
void JPH_HingeConstraint_SetTargetOrientationBS(JPH_HingeConstraint* constraint, const JPH_Quat* orientation)
{
	JPH_ASSERT(orientation);

	AsHingeConstraint(constraint)->SetTargetOrientationBS(ToQuat(*orientation));
}

/* SwingTwistConstraint: targets, motor settings, friction and limits */
void JPH_SwingTwistConstraint_SetTargetAngularVelocityCS(JPH_SwingTwistConstraint* constraint, const JPH_Vec3* angularVelocity)
{
	JPH_ASSERT(angularVelocity);

	AsSwingTwistConstraint(constraint)->SetTargetAngularVelocityCS(ToVec3(*angularVelocity));
}

void JPH_SwingTwistConstraint_GetTargetAngularVelocityCS(const JPH_SwingTwistConstraint* constraint, JPH_Vec3* result)
{
	JPH_ASSERT(result);

	FromVec3(AsSwingTwistConstraint(constraint)->GetTargetAngularVelocityCS(), result);
}

void JPH_SwingTwistConstraint_SetTargetOrientationCS(JPH_SwingTwistConstraint* constraint, const JPH_Quat* orientation)
{
	JPH_ASSERT(orientation);

	AsSwingTwistConstraint(constraint)->SetTargetOrientationCS(ToQuat(*orientation));
}

void JPH_SwingTwistConstraint_GetTargetOrientationCS(const JPH_SwingTwistConstraint* constraint, JPH_Quat* result)
{
	JPH_ASSERT(result);

	FromQuat(AsSwingTwistConstraint(constraint)->GetTargetOrientationCS(), result);
}

void JPH_SwingTwistConstraint_SetSwingMotorSettings(JPH_SwingTwistConstraint* constraint, const JPH_MotorSettings* settings)
{
	JPH_ASSERT(settings);

	AsSwingTwistConstraint(constraint)->GetSwingMotorSettings() = ToMotor(*settings);
}

void JPH_SwingTwistConstraint_GetSwingMotorSettings(const JPH_SwingTwistConstraint* constraint, JPH_MotorSettings* result)
{
	JPH_ASSERT(result);

	FromMotor(AsSwingTwistConstraint(constraint)->GetSwingMotorSettings(), result);
}

void JPH_SwingTwistConstraint_SetTwistMotorSettings(JPH_SwingTwistConstraint* constraint, const JPH_MotorSettings* settings)
{
	JPH_ASSERT(settings);

	AsSwingTwistConstraint(constraint)->GetTwistMotorSettings() = ToMotor(*settings);
}

void JPH_SwingTwistConstraint_GetTwistMotorSettings(const JPH_SwingTwistConstraint* constraint, JPH_MotorSettings* result)
{
	JPH_ASSERT(result);

	FromMotor(AsSwingTwistConstraint(constraint)->GetTwistMotorSettings(), result);
}

void JPH_SwingTwistConstraint_SetMaxFrictionTorque(JPH_SwingTwistConstraint* constraint, float frictionTorque)
{
	AsSwingTwistConstraint(constraint)->SetMaxFrictionTorque(frictionTorque);
}

float JPH_SwingTwistConstraint_GetMaxFrictionTorque(const JPH_SwingTwistConstraint* constraint)
{
	return AsSwingTwistConstraint(constraint)->GetMaxFrictionTorque();
}

float JPH_SwingTwistConstraint_GetPlaneHalfConeAngle(const JPH_SwingTwistConstraint* constraint)
{
	return AsSwingTwistConstraint(constraint)->GetPlaneHalfConeAngle();
}

float JPH_SwingTwistConstraint_GetTwistMinAngle(const JPH_SwingTwistConstraint* constraint)
{
	return AsSwingTwistConstraint(constraint)->GetTwistMinAngle();
}

float JPH_SwingTwistConstraint_GetTwistMaxAngle(const JPH_SwingTwistConstraint* constraint)
{
	return AsSwingTwistConstraint(constraint)->GetTwistMaxAngle();
}

/* SixDOFConstraint */
void JPH_SixDOFConstraint_SetMotorSettings(JPH_SixDOFConstraint* constraint, JPH_SixDOFConstraintAxis axis, const JPH_MotorSettings* settings)
{
	JPH_ASSERT(settings);

	AsSixDOFConstraint(constraint)->GetMotorSettings(static_cast<JPH::SixDOFConstraintSettings::EAxis>(axis)) = ToMotor(*settings);
}

/* PathConstraintPath */
void JPH_PathConstraintPath_Destroy(JPH_PathConstraintPath* path)
{
	if (path)
		AsPath(path)->Release();
}

void JPH_PathConstraintPath_SetIsLooping(JPH_PathConstraintPath* path, bool isLooping)
{
	AsPath(path)->SetIsLooping(isLooping);
}

bool JPH_PathConstraintPath_IsLooping(const JPH_PathConstraintPath* path)
{
	return AsPath(path)->IsLooping();
}

float JPH_PathConstraintPath_GetPathMaxFraction(const JPH_PathConstraintPath* path)
{
	return AsPath(path)->GetPathMaxFraction();
}

float JPH_PathConstraintPath_GetClosestPoint(const JPH_PathConstraintPath* path, const JPH_Vec3* position, float fractionHint)
{
	JPH_ASSERT(position);

	return AsPath(path)->GetClosestPoint(ToVec3(*position), fractionHint);
}

JPH_PathConstraintPath* JPH_PathConstraintPathHermite_Create(void)
{
	JPH::PathConstraintPath* path = new JPH::PathConstraintPathHermite();
	path->AddRef();
	return reinterpret_cast<JPH_PathConstraintPath*>(path);
}

void JPH_PathConstraintPathHermite_AddPoint(JPH_PathConstraintPath* path, const JPH_Vec3* position, const JPH_Vec3* tangent, const JPH_Vec3* normal)
{
	JPH_ASSERT(position && tangent && normal);

	// JPH_PathConstraintPathHermite_Create is the only function that creates Hermite paths, and
	// this function accepts only its handles, so the object is a PathConstraintPathHermite.
	static_cast<JPH::PathConstraintPathHermite*>(AsPath(path))->AddPoint(ToVec3(*position), ToVec3(*tangent), ToVec3(*normal));
}

/* PathConstraint */
static_assert(static_cast<int>(JPH::EPathRotationConstraintType::Free) == JPH_PathRotationConstraintType_Free);
static_assert(static_cast<int>(JPH::EPathRotationConstraintType::ConstrainAroundTangent) == JPH_PathRotationConstraintType_ConstrainAroundTangent);
static_assert(static_cast<int>(JPH::EPathRotationConstraintType::ConstrainAroundNormal) == JPH_PathRotationConstraintType_ConstrainAroundNormal);
static_assert(static_cast<int>(JPH::EPathRotationConstraintType::ConstrainAroundBinormal) == JPH_PathRotationConstraintType_ConstrainAroundBinormal);
static_assert(static_cast<int>(JPH::EPathRotationConstraintType::ConstrainToPath) == JPH_PathRotationConstraintType_ConstrainToPath);
static_assert(static_cast<int>(JPH::EPathRotationConstraintType::FullyConstrained) == JPH_PathRotationConstraintType_FullyConstrained);

void JPH_PathConstraintSettings_Init(JPH_PathConstraintSettings* settings)
{
	JPH_ASSERT(settings);

	JPH::PathConstraintSettings joltSettings;
	JPH_ConstraintSettings_Init(joltSettings, &settings->base);
	settings->path = nullptr;
	FromVec3(joltSettings.mPathPosition, &settings->pathPosition);
	FromQuat(joltSettings.mPathRotation, &settings->pathRotation);
	settings->pathFraction = joltSettings.mPathFraction;
	settings->maxFrictionForce = joltSettings.mMaxFrictionForce;
	settings->rotationConstraintType = static_cast<JPH_PathRotationConstraintType>(joltSettings.mRotationConstraintType);
	FromMotor(joltSettings.mPositionMotorSettings, &settings->positionMotorSettings);
}

JPH_PathConstraint* JPH_PathConstraint_Create(const JPH_PathConstraintSettings* settings, JPH_Body* body1, JPH_Body* body2)
{
	JPH_ASSERT(settings && settings->path);

	// The Jolt settings take a reference to the path, which the constraint shares; the settings
	// release theirs on return.
	JPH::PathConstraintSettings joltSettings;
	JPH_ConstraintSettings_ToJolt(&joltSettings, &settings->base);
	joltSettings.mPath = AsPath(settings->path);
	joltSettings.mPathPosition = ToVec3(settings->pathPosition);
	joltSettings.mPathRotation = ToQuat(settings->pathRotation);
	joltSettings.mPathFraction = settings->pathFraction;
	joltSettings.mMaxFrictionForce = settings->maxFrictionForce;
	joltSettings.mRotationConstraintType = static_cast<JPH::EPathRotationConstraintType>(settings->rotationConstraintType);
	joltSettings.mPositionMotorSettings = ToMotor(settings->positionMotorSettings);
	return CreateConstraint<JPH::PathConstraint, JPH_PathConstraint>(joltSettings, body1, body2);
}

const JPH_PathConstraintPath* JPH_PathConstraint_GetPath(const JPH_PathConstraint* constraint)
{
	return reinterpret_cast<const JPH_PathConstraintPath*>(AsPathConstraint(constraint)->GetPath());
}

float JPH_PathConstraint_GetPathFraction(const JPH_PathConstraint* constraint)
{
	return AsPathConstraint(constraint)->GetPathFraction();
}

void JPH_PathConstraint_SetMaxFrictionForce(JPH_PathConstraint* constraint, float frictionForce)
{
	AsPathConstraint(constraint)->SetMaxFrictionForce(frictionForce);
}

float JPH_PathConstraint_GetMaxFrictionForce(const JPH_PathConstraint* constraint)
{
	return AsPathConstraint(constraint)->GetMaxFrictionForce();
}

void JPH_PathConstraint_SetPositionMotorSettings(JPH_PathConstraint* constraint, const JPH_MotorSettings* settings)
{
	JPH_ASSERT(settings);

	AsPathConstraint(constraint)->GetPositionMotorSettings() = ToMotor(*settings);
}

void JPH_PathConstraint_GetPositionMotorSettings(const JPH_PathConstraint* constraint, JPH_MotorSettings* result)
{
	JPH_ASSERT(result);

	FromMotor(AsPathConstraint(constraint)->GetPositionMotorSettings(), result);
}

void JPH_PathConstraint_SetPositionMotorState(JPH_PathConstraint* constraint, JPH_MotorState state)
{
	AsPathConstraint(constraint)->SetPositionMotorState(static_cast<JPH::EMotorState>(state));
}

JPH_MotorState JPH_PathConstraint_GetPositionMotorState(const JPH_PathConstraint* constraint)
{
	return static_cast<JPH_MotorState>(AsPathConstraint(constraint)->GetPositionMotorState());
}

void JPH_PathConstraint_SetTargetVelocity(JPH_PathConstraint* constraint, float velocity)
{
	AsPathConstraint(constraint)->SetTargetVelocity(velocity);
}

float JPH_PathConstraint_GetTargetVelocity(const JPH_PathConstraint* constraint)
{
	return AsPathConstraint(constraint)->GetTargetVelocity();
}

void JPH_PathConstraint_SetTargetPathFraction(JPH_PathConstraint* constraint, float fraction)
{
	AsPathConstraint(constraint)->SetTargetPathFraction(fraction);
}

float JPH_PathConstraint_GetTargetPathFraction(const JPH_PathConstraint* constraint)
{
	return AsPathConstraint(constraint)->GetTargetPathFraction();
}

void JPH_PathConstraint_GetTotalLambdaPosition(const JPH_PathConstraint* constraint, float result[2])
{
	JPH_ASSERT(result);

	JPH::Vector<2> lambda = AsPathConstraint(constraint)->GetTotalLambdaPosition();
	result[0] = lambda[0];
	result[1] = lambda[1];
}

float JPH_PathConstraint_GetTotalLambdaPositionLimits(const JPH_PathConstraint* constraint)
{
	return AsPathConstraint(constraint)->GetTotalLambdaPositionLimits();
}

float JPH_PathConstraint_GetTotalLambdaMotor(const JPH_PathConstraint* constraint)
{
	return AsPathConstraint(constraint)->GetTotalLambdaMotor();
}

void JPH_PathConstraint_GetTotalLambdaRotationHinge(const JPH_PathConstraint* constraint, float result[2])
{
	JPH_ASSERT(result);

	JPH::Vector<2> lambda = AsPathConstraint(constraint)->GetTotalLambdaRotationHinge();
	result[0] = lambda[0];
	result[1] = lambda[1];
}

void JPH_PathConstraint_GetTotalLambdaRotation(const JPH_PathConstraint* constraint, JPH_Vec3* result)
{
	JPH_ASSERT(result);

	FromVec3(AsPathConstraint(constraint)->GetTotalLambdaRotation(), result);
}

/* PulleyConstraint */
void JPH_PulleyConstraintSettings_Init(JPH_PulleyConstraintSettings* settings)
{
	JPH_ASSERT(settings);

	JPH::PulleyConstraintSettings joltSettings;
	JPH_ConstraintSettings_Init(joltSettings, &settings->base);
	settings->space = static_cast<JPH_ConstraintSpace>(joltSettings.mSpace);
	FromRVec3(joltSettings.mBodyPoint1, &settings->bodyPoint1);
	FromRVec3(joltSettings.mFixedPoint1, &settings->fixedPoint1);
	FromRVec3(joltSettings.mBodyPoint2, &settings->bodyPoint2);
	FromRVec3(joltSettings.mFixedPoint2, &settings->fixedPoint2);
	settings->ratio = joltSettings.mRatio;
	settings->minLength = joltSettings.mMinLength;
	settings->maxLength = joltSettings.mMaxLength;
}

JPH_PulleyConstraint* JPH_PulleyConstraint_Create(const JPH_PulleyConstraintSettings* settings, JPH_Body* body1, JPH_Body* body2)
{
	JPH_ASSERT(settings);

	JPH::PulleyConstraintSettings joltSettings;
	JPH_ConstraintSettings_ToJolt(&joltSettings, &settings->base);
	joltSettings.mSpace = static_cast<JPH::EConstraintSpace>(settings->space);
	joltSettings.mBodyPoint1 = ToRVec3(settings->bodyPoint1);
	joltSettings.mFixedPoint1 = ToRVec3(settings->fixedPoint1);
	joltSettings.mBodyPoint2 = ToRVec3(settings->bodyPoint2);
	joltSettings.mFixedPoint2 = ToRVec3(settings->fixedPoint2);
	joltSettings.mRatio = settings->ratio;
	joltSettings.mMinLength = settings->minLength;
	joltSettings.mMaxLength = settings->maxLength;
	return CreateConstraint<JPH::PulleyConstraint, JPH_PulleyConstraint>(joltSettings, body1, body2);
}

void JPH_PulleyConstraint_GetSettings(const JPH_PulleyConstraint* constraint, JPH_PulleyConstraintSettings* settings)
{
	JPH_ASSERT(settings);

	JPH::Ref<JPH::PulleyConstraintSettings> joltSettings = JPH::StaticCast<JPH::PulleyConstraintSettings>(AsPulleyConstraint(constraint)->GetConstraintSettings());
	JPH_ConstraintSettings_Init(*joltSettings, &settings->base);
	settings->space = static_cast<JPH_ConstraintSpace>(joltSettings->mSpace);
	FromRVec3(joltSettings->mBodyPoint1, &settings->bodyPoint1);
	FromRVec3(joltSettings->mFixedPoint1, &settings->fixedPoint1);
	FromRVec3(joltSettings->mBodyPoint2, &settings->bodyPoint2);
	FromRVec3(joltSettings->mFixedPoint2, &settings->fixedPoint2);
	settings->ratio = joltSettings->mRatio;
	settings->minLength = joltSettings->mMinLength;
	settings->maxLength = joltSettings->mMaxLength;
}

void JPH_PulleyConstraint_SetLength(JPH_PulleyConstraint* constraint, float minLength, float maxLength)
{
	AsPulleyConstraint(constraint)->SetLength(minLength, maxLength);
}

float JPH_PulleyConstraint_GetMinLength(const JPH_PulleyConstraint* constraint)
{
	return AsPulleyConstraint(constraint)->GetMinLength();
}

float JPH_PulleyConstraint_GetMaxLength(const JPH_PulleyConstraint* constraint)
{
	return AsPulleyConstraint(constraint)->GetMaxLength();
}

float JPH_PulleyConstraint_GetCurrentLength(const JPH_PulleyConstraint* constraint)
{
	return AsPulleyConstraint(constraint)->GetCurrentLength();
}

float JPH_PulleyConstraint_GetTotalLambdaPosition(const JPH_PulleyConstraint* constraint)
{
	return AsPulleyConstraint(constraint)->GetTotalLambdaPosition();
}

/* RackAndPinionConstraint */
void JPH_RackAndPinionConstraintSettings_Init(JPH_RackAndPinionConstraintSettings* settings)
{
	JPH_ASSERT(settings);

	JPH::RackAndPinionConstraintSettings joltSettings;
	JPH_ConstraintSettings_Init(joltSettings, &settings->base);
	settings->space = static_cast<JPH_ConstraintSpace>(joltSettings.mSpace);
	FromVec3(joltSettings.mHingeAxis, &settings->hingeAxis);
	FromVec3(joltSettings.mSliderAxis, &settings->sliderAxis);
	settings->ratio = joltSettings.mRatio;
}

JPH_RackAndPinionConstraint* JPH_RackAndPinionConstraint_Create(const JPH_RackAndPinionConstraintSettings* settings, JPH_Body* body1, JPH_Body* body2)
{
	JPH_ASSERT(settings);

	JPH::RackAndPinionConstraintSettings joltSettings;
	JPH_ConstraintSettings_ToJolt(&joltSettings, &settings->base);
	joltSettings.mSpace = static_cast<JPH::EConstraintSpace>(settings->space);
	joltSettings.mHingeAxis = ToVec3(settings->hingeAxis);
	joltSettings.mSliderAxis = ToVec3(settings->sliderAxis);
	joltSettings.mRatio = settings->ratio;
	return CreateConstraint<JPH::RackAndPinionConstraint, JPH_RackAndPinionConstraint>(joltSettings, body1, body2);
}

void JPH_RackAndPinionConstraint_SetConstraints(JPH_RackAndPinionConstraint* constraint, const JPH_Constraint* pinion, const JPH_Constraint* rack)
{
	AsRackAndPinionConstraint(constraint)->SetConstraints(AsConstraint(pinion), AsConstraint(rack));
}

float JPH_RackAndPinionConstraint_GetTotalLambda(const JPH_RackAndPinionConstraint* constraint)
{
	return AsRackAndPinionConstraint(constraint)->GetTotalLambda();
}
