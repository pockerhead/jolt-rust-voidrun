// Shape creation with Jolt's error message and triangle readback. Part of this fork's joltc
// additions (see joltc_ext.cpp).
// Licensed under the MIT License (MIT), like joltc.
//
// Every joltc shape settings handle is a reinterpret_cast of a Jolt settings class that derives
// from JPH::ShapeSettings by single inheritance (joltc's DEF_MAP_DECL, the convention
// JPH_ShapeSettings_Destroy relies on), so the generic handle reaches the virtual Create.

#include <Jolt/Jolt.h>

#include <Jolt/Geometry/AABox.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>

#include <algorithm>
#include <cstring>

#include "joltc_ext_internal.h"

JPH_Shape* JPH_ShapeSettings_CreateShapeWithError(const JPH_ShapeSettings* settings, char* error, uint32_t errorCapacity)
{
	const JPH::ShapeSettings::ShapeResult& result = reinterpret_cast<const JPH::ShapeSettings*>(settings)->Create();
	const bool hasBuffer = error != nullptr && errorCapacity > 0;
	if (result.IsValid())
	{
		if (hasBuffer)
		{
			error[0] = '\0';
		}
		JPH::Shape* shape = result.Get().GetPtr();
		shape->AddRef();
		return reinterpret_cast<JPH_Shape*>(shape);
	}
	if (hasBuffer)
	{
		// A settings class that returns neither a shape nor an error leaves the message empty.
		const size_t length = result.HasError() ? std::min(result.GetError().size(), static_cast<size_t>(errorCapacity) - 1) : 0;
		if (length > 0)
		{
			std::memcpy(error, result.GetError().data(), length);
		}
		error[length] = '\0';
	}
	return nullptr;
}

uint32_t JPH_Shape_GetTriangles(const JPH_Shape* shape, JPH_Vec3* vertices, uint32_t maxTriangles)
{
	const JPH::Shape* joltShape = reinterpret_cast<const JPH::Shape*>(shape);
	const JPH::EShapeType type = joltShape->GetType();
	if (type == JPH::EShapeType::Compound || type == JPH::EShapeType::Decorated)
	{
		return 0;
	}
	JPH::Shape::GetTrianglesContext context;
	joltShape->GetTrianglesStart(context, JPH::AABox::sBiggest(), JPH::Vec3::sZero(), JPH::Quat::sIdentity(), JPH::Vec3::sOne());
	constexpr int batchSize = JPH::Shape::cGetTrianglesMinTrianglesRequested;
	JPH::Float3 batch[batchSize * 3];
	uint32_t count = 0;
	for (;;)
	{
		const int found = joltShape->GetTrianglesNext(context, batchSize, batch);
		if (found == 0)
		{
			return count;
		}
		for (int i = 0; i < found; ++i, ++count)
		{
			if (count < maxTriangles)
			{
				for (int v = 0; v < 3; ++v)
				{
					const JPH::Float3& vertex = batch[3 * i + v];
					vertices[3 * size_t(count) + v] = JPH_Vec3{ vertex.x, vertex.y, vertex.z };
				}
			}
		}
	}
}
