// Conversions shared by the translation units of this fork's joltc additions. Private to the
// joltc archive build; not installed.
// Licensed under the MIT License (MIT), like joltc.

#ifndef JOLT_C_EXT_INTERNAL_H_
#define JOLT_C_EXT_INTERNAL_H_ 1

#include <Jolt/Jolt.h>

#include <Jolt/Physics/PhysicsSystem.h>

#include "joltc_ext.h"
#include "joltc_physics_system.h"

namespace joltc_ext
{
	// JPH_Vec3 is three floats; JPH::Vec3 is a 16-byte SIMD type, so it is built, never cast.
	inline JPH::Vec3 ToVec3(const JPH_Vec3& vec)
	{
		return JPH::Vec3(vec.x, vec.y, vec.z);
	}

	inline JPH::PhysicsSystem& AsJoltPhysicsSystem(JPH_PhysicsSystem* system)
	{
		return *system->physicsSystem;
	}

	inline const JPH::PhysicsSystem& AsJoltPhysicsSystem(const JPH_PhysicsSystem* system)
	{
		return *system->physicsSystem;
	}
}

#endif /* JOLT_C_EXT_INTERNAL_H_ */
