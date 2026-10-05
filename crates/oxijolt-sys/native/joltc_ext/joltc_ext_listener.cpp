// Listener functions joltc lacks: the sub-shape pair of a removed contact, a contact listener whose
// validate callback gets a collide result without faces, and a soft body contact listener. Part of
// this fork's joltc additions (see joltc_ext.cpp).
// Licensed under the MIT License (MIT), like joltc.
//
// Handles: a removed contact's pair is a reinterpret_cast of the JPH::SubShapeIDPair (joltc's
// ManagedContactListener::OnContactRemoved and ManagedContactListener2 below), a contact manifold
// one of the JPH::ContactManifold (as joltc's ToContactManifold), and bodies are reinterpret_casts
// of JPH::Body. A JPH_ContactListener2 is a reinterpret_cast of the ManagedContactListener2 below,
// whose only base is JPH::ContactListener; a JPH_SoftBodyContactListener one of the
// ManagedSoftBodyContactListener, whose only base is JPH::SoftBodyContactListener; and a
// JPH_SoftBodyManifold one of the JPH::SoftBodyManifold Jolt passes to OnSoftBodyContactAdded.

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Collision/Shape/SubShapeIDPair.h>
#include <Jolt/Physics/SoftBody/SoftBodyContactListener.h>
#include <Jolt/Physics/SoftBody/SoftBodyManifold.h>

#include "joltc_ext_internal.h"

static_assert(int(JPH_SoftBodyValidateResult_AcceptContact) == int(JPH::SoftBodyValidateResult::AcceptContact), "JPH_SoftBodyValidateResult_AcceptContact");
static_assert(int(JPH_SoftBodyValidateResult_RejectContact) == int(JPH::SoftBodyValidateResult::RejectContact), "JPH_SoftBodyValidateResult_RejectContact");
static_assert(int(JPH_ValidateResult_AcceptAllContactsForThisBodyPair) == int(JPH::ValidateResult::AcceptAllContactsForThisBodyPair), "JPH_ValidateResult_AcceptAllContactsForThisBodyPair");
static_assert(int(JPH_ValidateResult_AcceptContact) == int(JPH::ValidateResult::AcceptContact), "JPH_ValidateResult_AcceptContact");
static_assert(int(JPH_ValidateResult_RejectContact) == int(JPH::ValidateResult::RejectContact), "JPH_ValidateResult_RejectContact");
static_assert(int(JPH_ValidateResult_RejectAllContactsForThisBodyPair) == int(JPH::ValidateResult::RejectAllContactsForThisBodyPair), "JPH_ValidateResult_RejectAllContactsForThisBodyPair");

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

	// JPH_RVec3 is JPH_Vec3 in single precision and three doubles in double precision, like RVec3.
	void FromRVec3(JPH::RVec3Arg vec, JPH_RVec3* result)
	{
		result->x = vec.GetX();
		result->y = vec.GetY();
		result->z = vec.GetZ();
	}

	// As joltc's FromJolt(const ContactSettings&, JPH_ContactSettings*).
	void FromContactSettings(const JPH::ContactSettings& settings, JPH_ContactSettings* result)
	{
		result->combinedFriction = settings.mCombinedFriction;
		result->combinedRestitution = settings.mCombinedRestitution;
		result->invMassScale1 = settings.mInvMassScale1;
		result->invInertiaScale1 = settings.mInvInertiaScale1;
		result->invMassScale2 = settings.mInvMassScale2;
		result->invInertiaScale2 = settings.mInvInertiaScale2;
		result->isSensor = settings.mIsSensor;
		FromVec3(settings.mRelativeLinearSurfaceVelocity, &result->relativeLinearSurfaceVelocity);
		FromVec3(settings.mRelativeAngularSurfaceVelocity, &result->relativeAngularSurfaceVelocity);
	}

	// As joltc's ToJolt(ContactSettings&, const JPH_ContactSettings*).
	void ToContactSettings(const JPH_ContactSettings& settings, JPH::ContactSettings& result)
	{
		result.mCombinedFriction = settings.combinedFriction;
		result.mCombinedRestitution = settings.combinedRestitution;
		result.mInvMassScale1 = settings.invMassScale1;
		result.mInvInertiaScale1 = settings.invInertiaScale1;
		result.mInvMassScale2 = settings.invMassScale2;
		result.mInvInertiaScale2 = settings.invInertiaScale2;
		result.mIsSensor = settings.isSensor;
		result.mRelativeLinearSurfaceVelocity = joltc_ext::ToVec3(settings.relativeLinearSurfaceVelocity);
		result.mRelativeAngularSurfaceVelocity = joltc_ext::ToVec3(settings.relativeAngularSurfaceVelocity);
	}

	// The collide result without its faces, so nothing is allocated.
	JPH_CollideShapeResult FromCollideShapeResultWithoutFaces(const JPH::CollideShapeResult& result)
	{
		JPH_CollideShapeResult converted{};
		FromVec3(result.mContactPointOn1, &converted.contactPointOn1);
		FromVec3(result.mContactPointOn2, &converted.contactPointOn2);
		FromVec3(result.mPenetrationAxis, &converted.penetrationAxis);
		converted.penetrationDepth = result.mPenetrationDepth;
		converted.subShapeID1 = result.mSubShapeID1.GetValue();
		converted.subShapeID2 = result.mSubShapeID2.GetValue();
		converted.bodyID2 = result.mBodyID2.GetIndexAndSequenceNumber();
		return converted;
	}

	class ManagedContactListener2 final : public JPH::ContactListener
	{
	public:
		static const JPH_ContactListener_Procs* s_Procs;
		void* userData = nullptr;

		explicit ManagedContactListener2(void* userData_)
			: userData(userData_)
		{
		}

		JPH::ValidateResult OnContactValidate(const JPH::Body& inBody1, const JPH::Body& inBody2, JPH::RVec3Arg inBaseOffset, const JPH::CollideShapeResult& inCollisionResult) override
		{
			if (s_Procs == nullptr || s_Procs->OnContactValidate == nullptr)
			{
				return JPH::ValidateResult::AcceptAllContactsForThisBodyPair;
			}
			JPH_RVec3 baseOffset;
			FromRVec3(inBaseOffset, &baseOffset);
			JPH_CollideShapeResult result = FromCollideShapeResultWithoutFaces(inCollisionResult);
			JPH_ValidateResult validate = s_Procs->OnContactValidate(
				userData,
				reinterpret_cast<const JPH_Body*>(&inBody1),
				reinterpret_cast<const JPH_Body*>(&inBody2),
				&baseOffset,
				&result);
			return static_cast<JPH::ValidateResult>(validate);
		}

		void OnContactAdded(const JPH::Body& inBody1, const JPH::Body& inBody2, const JPH::ContactManifold& inManifold, JPH::ContactSettings& ioSettings) override
		{
			if (s_Procs != nullptr && s_Procs->OnContactAdded != nullptr)
			{
				Forward(s_Procs->OnContactAdded, inBody1, inBody2, inManifold, ioSettings);
			}
		}

		void OnContactPersisted(const JPH::Body& inBody1, const JPH::Body& inBody2, const JPH::ContactManifold& inManifold, JPH::ContactSettings& ioSettings) override
		{
			if (s_Procs != nullptr && s_Procs->OnContactPersisted != nullptr)
			{
				Forward(s_Procs->OnContactPersisted, inBody1, inBody2, inManifold, ioSettings);
			}
		}

		void OnContactRemoved(const JPH::SubShapeIDPair& inSubShapePair) override
		{
			if (s_Procs != nullptr && s_Procs->OnContactRemoved != nullptr)
			{
				s_Procs->OnContactRemoved(userData, reinterpret_cast<const JPH_SubShapeIDPair*>(&inSubShapePair));
			}
		}

	private:
		using ManifoldProc = void(JPH_API_CALL*)(void*, const JPH_Body*, const JPH_Body*, const JPH_ContactManifold*, JPH_ContactSettings*);

		// Calls proc with the settings copied out, and copies them back after it returned.
		void Forward(ManifoldProc proc, const JPH::Body& inBody1, const JPH::Body& inBody2, const JPH::ContactManifold& inManifold, JPH::ContactSettings& ioSettings)
		{
			JPH_ContactSettings settings;
			FromContactSettings(ioSettings, &settings);
			proc(
				userData,
				reinterpret_cast<const JPH_Body*>(&inBody1),
				reinterpret_cast<const JPH_Body*>(&inBody2),
				reinterpret_cast<const JPH_ContactManifold*>(&inManifold),
				&settings);
			ToContactSettings(settings, ioSettings);
		}
	};

	const JPH_ContactListener_Procs* ManagedContactListener2::s_Procs = nullptr;

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

void JPH_ContactListener2_SetProcs(const JPH_ContactListener_Procs* procs)
{
	ManagedContactListener2::s_Procs = procs;
}

JPH_ContactListener2* JPH_ContactListener2_Create(void* userData)
{
	auto listener = new ManagedContactListener2(userData);
	return reinterpret_cast<JPH_ContactListener2*>(listener);
}

void JPH_ContactListener2_Destroy(JPH_ContactListener2* listener)
{
	delete reinterpret_cast<ManagedContactListener2*>(listener);
}

void JPH_PhysicsSystem_SetContactListener2(JPH_PhysicsSystem* system, JPH_ContactListener2* listener)
{
	joltc_ext::AsJoltPhysicsSystem(system).SetContactListener(reinterpret_cast<ManagedContactListener2*>(listener));
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
