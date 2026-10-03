// Physics materials with user data, and shape settings that take materials. Part of this fork's
// joltc additions (see joltc_ext.cpp).
// Licensed under the MIT License (MIT), like joltc.
//
// Handles follow joltc's DEF_MAP_DECL: a JPH_PhysicsMaterial is a reinterpret_cast of the
// JPH::PhysicsMaterial base (the class that carries the reference count), never of a derived
// material; JPH_ConvexShapeSettings and JPH_HeightFieldShapeSettings are reinterpret_casts of
// JPH::ConvexShapeSettings and JPH::HeightFieldShapeSettings, which joltc's typed settings
// handles reach through single inheritance.

#include <Jolt/Jolt.h>

#include <Jolt/Core/Color.h>
#include <Jolt/Physics/Collision/PhysicsMaterialSimple.h>
#include <Jolt/Physics/Collision/Shape/ConvexShape.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>

#include "joltc_ext_internal.h"

namespace joltc_ext
{
	// A PhysicsMaterialSimple that also carries the caller's value. Recognised through Jolt's own
	// RTTI with an exact type match, so no other material is mistaken for one.
	class PhysicsMaterialUserData final : public JPH::PhysicsMaterialSimple
	{
		JPH_DECLARE_RTTI_VIRTUAL(JPH_NO_EXPORT, PhysicsMaterialUserData)

	public:
		// Jolt's RTTI factory needs a default constructor.
		PhysicsMaterialUserData() = default;

		PhysicsMaterialUserData(const JPH::string_view& name, JPH::ColorArg color, JPH::uint64 userData)
			: JPH::PhysicsMaterialSimple(name, color), mUserData(userData)
		{
		}

		const JPH::uint64 mUserData = 0;
	};

	JPH_IMPLEMENT_RTTI_VIRTUAL(PhysicsMaterialUserData)
	{
		JPH_ADD_BASE_CLASS(PhysicsMaterialUserData, JPH::PhysicsMaterialSimple)
	}

	const JPH::PhysicsMaterial* AsJoltMaterial(const JPH_PhysicsMaterial* material)
	{
		return reinterpret_cast<const JPH::PhysicsMaterial*>(material);
	}
}

JPH_PhysicsMaterial* JPH_PhysicsMaterial_Create2(const char* name, uint32_t color, uint64_t userData)
{
	auto material = new joltc_ext::PhysicsMaterialUserData(name, JPH::Color(color), userData);
	material->AddRef();
	return reinterpret_cast<JPH_PhysicsMaterial*>(static_cast<JPH::PhysicsMaterial*>(material));
}

bool JPH_PhysicsMaterial_GetUserData(const JPH_PhysicsMaterial* material, uint64_t* userData)
{
	const JPH::PhysicsMaterial* joltMaterial = joltc_ext::AsJoltMaterial(material);
	if (joltMaterial == nullptr || joltMaterial->GetRTTI() != JPH_RTTI(joltc_ext::PhysicsMaterialUserData))
	{
		return false;
	}
	*userData = static_cast<const joltc_ext::PhysicsMaterialUserData*>(joltMaterial)->mUserData;
	return true;
}

void JPH_ConvexShapeSettings_SetMaterial(JPH_ConvexShapeSettings* settings, const JPH_PhysicsMaterial* material)
{
	reinterpret_cast<JPH::ConvexShapeSettings*>(settings)->mMaterial = joltc_ext::AsJoltMaterial(material);
}

JPH_HeightFieldShapeSettings* JPH_HeightFieldShapeSettings_Create2(const float* samples, const JPH_Vec3* offset, const JPH_Vec3* scale, uint32_t sampleCount, const uint8_t* materialIndices, const JPH_PhysicsMaterial* const* materials, uint32_t materialCount)
{
	if (materialCount == 0 || materialIndices == nullptr || materials == nullptr)
	{
		return nullptr;
	}
	JPH::PhysicsMaterialList materialList;
	materialList.reserve(materialCount);
	for (uint32_t i = 0; i < materialCount; ++i)
	{
		materialList.push_back(joltc_ext::AsJoltMaterial(materials[i]));
	}
	auto settings = new JPH::HeightFieldShapeSettings(samples, joltc_ext::ToVec3(*offset), joltc_ext::ToVec3(*scale), sampleCount, materialIndices, materialList);
	settings->AddRef();
	return reinterpret_cast<JPH_HeightFieldShapeSettings*>(settings);
}
