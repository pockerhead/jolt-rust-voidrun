// Listener functions joltc lacks: the sub-shape pair of a removed contact and a soft body contact
// listener. Part of this fork's joltc additions (see joltc_ext.cpp).
// Licensed under the MIT License (MIT), like joltc.
//
// Handles: joltc passes a removed contact's pair as a reinterpret_cast of the JPH::SubShapeIDPair
// (ManagedContactListener::OnContactRemoved), and bodies are reinterpret_casts of JPH::Body. A
// JPH_SoftBodyContactListener is a reinterpret_cast of the ManagedSoftBodyContactListener below,
// whose only base is JPH::SoftBodyContactListener, and a JPH_SoftBodyManifold one of the
// JPH::SoftBodyManifold Jolt passes to OnSoftBodyContactAdded.

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Collision/Shape/SubShapeIDPair.h>
#include <Jolt/Physics/SoftBody/SoftBodyContactListener.h>
#include <Jolt/Physics/SoftBody/SoftBodyManifold.h>

#include "joltc_ext_internal.h"

static_assert(int(JPH_SoftBodyValidateResult_AcceptContact) == int(JPH::SoftBodyValidateResult::AcceptContact), "JPH_SoftBodyValidateResult_AcceptContact");
static_assert(int(JPH_SoftBodyValidateResult_RejectContact) == int(JPH::SoftBodyValidateResult::RejectContact), "JPH_SoftBodyValidateResult_RejectContact");

namespace
{
	const JPH::SubShapeIDPair& AsSubShapeIDPair(const JPH_SubShapeIDPair* pair)
	{
		return *reinterpret_cast<const JPH::SubShapeIDPair*>(pair);
	}

	const JPH::SoftBodyManifold& AsSoftBodyManifold(const JPH_SoftBodyManifold* manifold)
	{
		return *reinterpret_cast<const JPH::SoftBodyManifold*>(manifold);
	}

	void FromVec3(JPH::Vec3Arg vec, JPH_Vec3* result)
	{
		result->x = vec.GetX();
		result->y = vec.GetY();
		result->z = vec.GetZ();
	}

	class ManagedSoftBodyContactListener final : public JPH::SoftBodyContactListener
	{
	public:
		static const JPH_SoftBodyContactListener_Procs* s_Procs;
		void* userData = nullptr;

		explicit ManagedSoftBodyContactListener(void* userData_)
			: userData(userData_)
		{
		}

		JPH::SoftBodyValidateResult OnSoftBodyContactValidate(const JPH::Body& inSoftBody, const JPH::Body& inOtherBody, JPH::SoftBodyContactSettings& ioSettings) override
		{
			if (s_Procs == nullptr || s_Procs->OnSoftBodyContactValidate == nullptr)
			{
				return JPH::SoftBodyValidateResult::AcceptContact;
			}
			JPH_SoftBodyContactSettings settings;
			settings.invMassScale1 = ioSettings.mInvMassScale1;
			settings.invMassScale2 = ioSettings.mInvMassScale2;
			settings.invInertiaScale2 = ioSettings.mInvInertiaScale2;
			settings.isSensor = ioSettings.mIsSensor;
			JPH_SoftBodyValidateResult result = s_Procs->OnSoftBodyContactValidate(
				userData,
				reinterpret_cast<const JPH_Body*>(&inSoftBody),
				reinterpret_cast<const JPH_Body*>(&inOtherBody),
				&settings);
			ioSettings.mInvMassScale1 = settings.invMassScale1;
			ioSettings.mInvMassScale2 = settings.invMassScale2;
			ioSettings.mInvInertiaScale2 = settings.invInertiaScale2;
			ioSettings.mIsSensor = settings.isSensor;
			return result == JPH_SoftBodyValidateResult_RejectContact ? JPH::SoftBodyValidateResult::RejectContact : JPH::SoftBodyValidateResult::AcceptContact;
		}

		void OnSoftBodyContactAdded(const JPH::Body& inSoftBody, const JPH::SoftBodyManifold& inManifold) override
		{
			if (s_Procs != nullptr && s_Procs->OnSoftBodyContactAdded != nullptr)
			{
				s_Procs->OnSoftBodyContactAdded(
					userData,
					reinterpret_cast<const JPH_Body*>(&inSoftBody),
					reinterpret_cast<const JPH_SoftBodyManifold*>(&inManifold));
			}
		}
	};

	const JPH_SoftBodyContactListener_Procs* ManagedSoftBodyContactListener::s_Procs = nullptr;

	// The vertex at index, or null when index is not below the vertex count.
	const JPH::SoftBodyVertex* VertexAt(const JPH_SoftBodyManifold* manifold, uint32_t index)
	{
		const auto& vertices = AsSoftBodyManifold(manifold).GetVertices();
		return index < vertices.size() ? &vertices[index] : nullptr;
	}
}

JPH_BodyID JPH_SubShapeIDPair_GetBody1ID(const JPH_SubShapeIDPair* pair)
{
	return AsSubShapeIDPair(pair).GetBody1ID().GetIndexAndSequenceNumber();
}

JPH_SubShapeID JPH_SubShapeIDPair_GetSubShapeID1(const JPH_SubShapeIDPair* pair)
{
	return AsSubShapeIDPair(pair).GetSubShapeID1().GetValue();
}

JPH_BodyID JPH_SubShapeIDPair_GetBody2ID(const JPH_SubShapeIDPair* pair)
{
	return AsSubShapeIDPair(pair).GetBody2ID().GetIndexAndSequenceNumber();
}

JPH_SubShapeID JPH_SubShapeIDPair_GetSubShapeID2(const JPH_SubShapeIDPair* pair)
{
	return AsSubShapeIDPair(pair).GetSubShapeID2().GetValue();
}

void JPH_SoftBodyContactListener_SetProcs(const JPH_SoftBodyContactListener_Procs* procs)
{
	ManagedSoftBodyContactListener::s_Procs = procs;
}

JPH_SoftBodyContactListener* JPH_SoftBodyContactListener_Create(void* userData)
{
	auto listener = new ManagedSoftBodyContactListener(userData);
	return reinterpret_cast<JPH_SoftBodyContactListener*>(listener);
}

void JPH_SoftBodyContactListener_Destroy(JPH_SoftBodyContactListener* listener)
{
	delete reinterpret_cast<ManagedSoftBodyContactListener*>(listener);
}

void JPH_PhysicsSystem_SetSoftBodyContactListener(JPH_PhysicsSystem* system, JPH_SoftBodyContactListener* listener)
{
	joltc_ext::AsJoltPhysicsSystem(system).SetSoftBodyContactListener(reinterpret_cast<ManagedSoftBodyContactListener*>(listener));
}

uint32_t JPH_SoftBodyManifold_GetVertexCount(const JPH_SoftBodyManifold* manifold)
{
	return static_cast<uint32_t>(AsSoftBodyManifold(manifold).GetVertices().size());
}

bool JPH_SoftBodyManifold_HasContact(const JPH_SoftBodyManifold* manifold, uint32_t index)
{
	const JPH::SoftBodyVertex* vertex = VertexAt(manifold, index);
	return vertex != nullptr && AsSoftBodyManifold(manifold).HasContact(*vertex);
}

bool JPH_SoftBodyManifold_GetLocalContactPoint(const JPH_SoftBodyManifold* manifold, uint32_t index, JPH_Vec3* result)
{
	if (!JPH_SoftBodyManifold_HasContact(manifold, index))
	{
		return false;
	}
	FromVec3(AsSoftBodyManifold(manifold).GetLocalContactPoint(*VertexAt(manifold, index)), result);
	return true;
}

bool JPH_SoftBodyManifold_GetContactNormal(const JPH_SoftBodyManifold* manifold, uint32_t index, JPH_Vec3* result)
{
	if (!JPH_SoftBodyManifold_HasContact(manifold, index))
	{
		return false;
	}
	FromVec3(AsSoftBodyManifold(manifold).GetContactNormal(*VertexAt(manifold, index)), result);
	return true;
}

JPH_BodyID JPH_SoftBodyManifold_GetContactBodyID(const JPH_SoftBodyManifold* manifold, uint32_t index)
{
	const JPH::SoftBodyVertex* vertex = VertexAt(manifold, index);
	if (vertex == nullptr)
	{
		return JPH::BodyID::cInvalidBodyID;
	}
	return AsSoftBodyManifold(manifold).GetContactBodyID(*vertex).GetIndexAndSequenceNumber();
}

uint32_t JPH_SoftBodyManifold_GetNumSensorContacts(const JPH_SoftBodyManifold* manifold)
{
	return AsSoftBodyManifold(manifold).GetNumSensorContacts();
}

JPH_BodyID JPH_SoftBodyManifold_GetSensorContactBodyID(const JPH_SoftBodyManifold* manifold, uint32_t index)
{
	const JPH::SoftBodyManifold& joltManifold = AsSoftBodyManifold(manifold);
	if (index >= joltManifold.GetNumSensorContacts())
	{
		return JPH::BodyID::cInvalidBodyID;
	}
	return joltManifold.GetSensorContactBodyID(index).GetIndexAndSequenceNumber();
}
