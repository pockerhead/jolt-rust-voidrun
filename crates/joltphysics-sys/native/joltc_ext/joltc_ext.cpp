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
// ragdoll settings and the swing-twist and hinge constraints are reinterpret_casts of the Jolt
// objects, as joltc's DEF_MAP_DECL defines them.

#include <Jolt/Jolt.h>

#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyFilter.h>
#include <Jolt/Physics/Character/CharacterVirtual.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <Jolt/Physics/Collision/ShapeFilter.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/SixDOFConstraint.h>
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>
#include <Jolt/Physics/Ragdoll/Ragdoll.h>
#include <Jolt/Physics/StateRecorderImpl.h>
#include <Jolt/Physics/Vehicle/VehicleConstraint.h>

#include <algorithm>
#include <cstring>
#include <string>

#include "joltc_ext.h"

// joltc's own conversions of the constraint settings, defined in joltc.cpp at global scope with C++
// linkage but not declared in joltc.h. Declaring them here keeps one conversion shared with joltc's
// JPH_*Constraint_Create functions.
void JPH_SwingTwistConstraintSettings_ToJolt(JPH::SwingTwistConstraintSettings* joltSettings, const JPH_SwingTwistConstraintSettings* settings);
void JPH_HingeConstraintSettings_ToJolt(JPH::HingeConstraintSettings* joltSettings, const JPH_HingeConstraintSettings* settings);
void JPH_SixDOFConstraintSettings_ToJolt(JPH::SixDOFConstraintSettings* joltSettings, const JPH_SixDOFConstraintSettings* settings);

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
