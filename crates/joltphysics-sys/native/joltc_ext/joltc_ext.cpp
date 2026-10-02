// Functions this fork adds to joltc, in joltc's naming and style. They are compiled into the
// joltc archive (see ../CMakeLists.txt) and are meant to be offered upstream.
// Licensed under the MIT License (MIT), like joltc.
//
// This translation unit does not include joltc.cpp. It converts joltc handles the way joltc
// does: filters, the temp allocator and the character are reinterpret_casts of the Jolt objects
// they stand for (joltc's DEF_MAP_DECL and its Managed*Filter classes, which derive from the
// Jolt filter classes with single inheritance).

#include <Jolt/Jolt.h>

#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyFilter.h>
#include <Jolt/Physics/Character/CharacterVirtual.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <Jolt/Physics/Collision/ShapeFilter.h>
#include <Jolt/Physics/StateRecorderImpl.h>

#include <algorithm>
#include <cstring>
#include <string>

#include "joltc_ext.h"

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
