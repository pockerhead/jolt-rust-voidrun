// Motorcycle target lean access. Part of this fork's joltc additions (see joltc_ext.cpp).
// Licensed under the MIT License (MIT), like joltc.
//
// Every joltc vehicle controller handle is a reinterpret_cast of a Jolt controller class that
// derives from JPH::VehicleController by single inheritance (joltc's DEF_MAP_DECL), so the
// generic handle reaches the virtual GetSettings.

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Vehicle/MotorcycleController.h>

#include "joltc_ext_internal.h"

namespace
{
	// Names Jolt's protected MotorcycleController::mTargetLean. Inside a class derived from
	// MotorcycleController, &Derived::member forms a pointer to the protected member of the base
	// ([class.protected]); the pointer applies to any MotorcycleController. Never instantiated.
	struct MotorcycleTargetLean final : JPH::MotorcycleController
	{
		using Member = JPH::Vec3 JPH::MotorcycleController::*;

		static Member Get()
		{
			return &MotorcycleTargetLean::mTargetLean;
		}
	};

	// The controller as a MotorcycleController, or null when it is of another kind. Jolt is built
	// without C++ RTTI; its own RTTI on the controller's settings tells the kind.
	const JPH::MotorcycleController* AsMotorcycle(const JPH_VehicleController* controller)
	{
		const JPH::VehicleController* jolt = reinterpret_cast<const JPH::VehicleController*>(controller);
		const JPH::Ref<JPH::VehicleControllerSettings> settings = jolt->GetSettings();
		if (!JPH::IsKindOf(settings.GetPtr(), JPH_RTTI(JPH::MotorcycleControllerSettings)))
		{
			return nullptr;
		}
		return static_cast<const JPH::MotorcycleController*>(jolt);
	}
}

bool JPH_MotorcycleController_GetTargetLean(const JPH_VehicleController* controller, JPH_Vec3* result)
{
	JPH_ASSERT(controller);
	JPH_ASSERT(result);

	const JPH::MotorcycleController* motorcycle = AsMotorcycle(controller);
	if (motorcycle == nullptr)
	{
		return false;
	}
	const JPH::Vec3& lean = motorcycle->*MotorcycleTargetLean::Get();
	result->x = lean.GetX();
	result->y = lean.GetY();
	result->z = lean.GetZ();
	return true;
}

bool JPH_MotorcycleController_SetTargetLean(JPH_VehicleController* controller, const JPH_Vec3* value)
{
	JPH_ASSERT(controller);
	JPH_ASSERT(value);

	const JPH::MotorcycleController* motorcycle = AsMotorcycle(controller);
	if (motorcycle == nullptr)
	{
		return false;
	}
	// The caller passed a mutable controller; the const view was only for the kind check.
	const_cast<JPH::MotorcycleController*>(motorcycle)->*MotorcycleTargetLean::Get() = joltc_ext::ToVec3(*value);
	return true;
}
