// Shape binary state: a built shape with its children and materials to bytes and back. Part of
// this fork's joltc additions (see joltc_ext.cpp).
// Licensed under the MIT License (MIT), like joltc.
//
// Jolt's own SaveWithChildren/sRestoreWithChildren trust their input: the sub-shape type indexes a
// function table without a range check, a compound writes as many children as the stream names,
// child ids may point forward (cycles), and materials are created through the RTTI factory by a
// hash from the stream. This file walks the graph itself and stores one record per shape, in
// post-order, so the reader checks every type, count and index before Jolt sees the record.
//
// Payload (native byte order, which the caller records):
//   u32 material count, then per material: u8 kind, u64 user data, u32 colour, u32 name length, name
//   u32 shape count, then per shape (children before their parents, the root last):
//     u32 record length (the bytes that follow, up to the next record)
//     u8 sub-shape type, u32 child count, u32 child index[], u32 material count, u32 material index[]
//     Empty shapes only: 3 x f32 centre of mass (Jolt's EmptyShape does not save it)
//     u32 Jolt length, Jolt's SaveBinaryState bytes (their first byte is the sub-shape type)

#include <Jolt/Jolt.h>

#include <Jolt/Core/StreamIn.h>
#include <Jolt/Core/StreamOut.h>
#include <Jolt/Physics/Collision/PhysicsMaterialSimple.h>
#include <Jolt/Physics/Collision/Shape/CompoundShape.h>
#include <Jolt/Physics/Collision/Shape/EmptyShape.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>

#include <algorithm>
#include <cstring>
#include <string>
#include <unordered_map>
#include <vector>

#include "joltc_ext_internal.h"

struct JPH_ShapeBinaryState
{
	std::vector<uint8_t> data;
};

namespace
{
	// Changes with every change of the payload layout above.
	constexpr uint32_t cShapeBinaryStateVersion = 1;

	// Material kinds of a material record.
	enum class MaterialKind : uint8_t
	{
		Null = 0,			// no material: the shape uses the default one
		Default = 1,		// JPH::PhysicsMaterial::sDefault
		Simple = 2,			// a JPH::PhysicsMaterialSimple (name and colour)
		UserData = 3,		// a JPH_PhysicsMaterial_Create2 material (name, colour and user data)
	};

	// The sub-shape types this file saves and restores: every kind Jolt builds from settings,
	// except triangles (never built by the binding) and soft bodies and user shapes (Jolt cannot
	// construct those from a stream).
	bool IsSupportedSubType(uint8_t subType)
	{
		using JPH::EShapeSubType;
		switch (static_cast<EShapeSubType>(subType))
		{
		case EShapeSubType::Sphere:
		case EShapeSubType::Box:
		case EShapeSubType::Capsule:
		case EShapeSubType::TaperedCapsule:
		case EShapeSubType::Cylinder:
		case EShapeSubType::TaperedCylinder:
		case EShapeSubType::ConvexHull:
		case EShapeSubType::StaticCompound:
		case EShapeSubType::MutableCompound:
		case EShapeSubType::RotatedTranslated:
		case EShapeSubType::Scaled:
		case EShapeSubType::OffsetCenterOfMass:
		case EShapeSubType::Mesh:
		case EShapeSubType::HeightField:
		case EShapeSubType::Plane:
		case EShapeSubType::Empty:
			return true;
		default:
			return false;
		}
	}

	void SetError(char* error, uint32_t errorCapacity, const char* message)
	{
		if (error == nullptr || errorCapacity == 0)
		{
			return;
		}
		const size_t length = std::min(std::strlen(message), static_cast<size_t>(errorCapacity) - 1);
		std::memcpy(error, message, length);
		error[length] = '\0';
	}

	// Appends plain values to a byte vector.
	class Writer
	{
	public:
		explicit Writer(std::vector<uint8_t>& bytes) : mBytes(bytes) {}

		template <class T>
		void Put(const T& value)
		{
			PutBytes(&value, sizeof(T));
		}

		void PutBytes(const void* data, size_t size)
		{
			const uint8_t* begin = static_cast<const uint8_t*>(data);
			mBytes.insert(mBytes.end(), begin, begin + size);
		}

	private:
		std::vector<uint8_t>& mBytes;
	};

	// Jolt's output stream into a byte vector.
	class VectorStreamOut final : public JPH::StreamOut
	{
	public:
		explicit VectorStreamOut(std::vector<uint8_t>& bytes) : mBytes(bytes) {}

		void WriteBytes(const void* data, size_t size) override
		{
			const uint8_t* begin = static_cast<const uint8_t*>(data);
			mBytes.insert(mBytes.end(), begin, begin + size);
		}

		bool IsFailed() const override { return false; }

	private:
		std::vector<uint8_t>& mBytes;
	};

	// Jolt's input stream over a span: a read past the end reads zeros and sets EOF instead of
	// reading outside the span.
	class SpanStreamIn final : public JPH::StreamIn
	{
	public:
		SpanStreamIn(const uint8_t* data, size_t size) : mData(data), mSize(size) {}

		void ReadBytes(void* out, size_t size) override
		{
			if (mEOF || size > mSize - mPosition)
			{
				mEOF = true;
				std::memset(out, 0, size);
				return;
			}
			std::memcpy(out, mData + mPosition, size);
			mPosition += size;
		}

		bool IsEOF() const override { return mEOF; }
		bool IsFailed() const override { return mEOF; }

		size_t Position() const { return mPosition; }

	private:
		const uint8_t* mData;
		size_t mSize;
		size_t mPosition = 0;
		bool mEOF = false;
	};

	// Reads plain values from a span; every read checks the remaining length first.
	class Reader
	{
	public:
		Reader(const uint8_t* data, size_t size) : mData(data), mSize(size) {}

		template <class T>
		bool Get(T& value)
		{
			if (Remaining() < sizeof(T))
			{
				return false;
			}
			std::memcpy(&value, mData + mPosition, sizeof(T));
			mPosition += sizeof(T);
			return true;
		}

		// Points span at the next size bytes and skips them.
		bool Take(size_t size, const uint8_t*& span)
		{
			if (Remaining() < size)
			{
				return false;
			}
			span = mData + mPosition;
			mPosition += size;
			return true;
		}

		size_t Remaining() const { return mSize - mPosition; }

	private:
		const uint8_t* mData;
		size_t mSize;
		size_t mPosition = 0;
	};

	// Reads a u32 count and that many u32 indices, each below limit.
	bool GetIndices(Reader& reader, uint32_t limit, std::vector<uint32_t>& indices)
	{
		uint32_t count = 0;
		if (!reader.Get(count) || count > reader.Remaining() / sizeof(uint32_t))
		{
			return false;
		}
		indices.resize(count);
		for (uint32_t& index : indices)
		{
			reader.Get(index);
			if (index >= limit)
			{
				return false;
			}
		}
		return true;
	}

	// The graph walk of a save: shapes and materials in the order they are first written.
	class Saver
	{
	public:
		// Writes the payload of root; false and a message on a shape or material outside the
		// supported set.
		bool Save(const JPH::Shape* root, std::vector<uint8_t>& payload, const char*& error)
		{
			std::vector<uint8_t> records;
			if (!SaveShapes(root, records, error))
			{
				return false;
			}
			Writer writer(payload);
			writer.Put(static_cast<uint32_t>(mMaterials.size()));
			for (const JPH::PhysicsMaterial* material : mMaterials)
			{
				SaveMaterial(material, writer);
			}
			writer.Put(static_cast<uint32_t>(mShapeIndices.size()));
			writer.PutBytes(records.data(), records.size());
			return true;
		}

	private:
		struct Frame
		{
			const JPH::Shape* shape;
			JPH::ShapeList children;
			size_t next = 0;
		};

		// Post-order walk without recursion: a shape is written after all its children.
		bool SaveShapes(const JPH::Shape* root, std::vector<uint8_t>& records, const char*& error)
		{
			std::vector<Frame> stack;
			if (!Push(root, stack, error))
			{
				return false;
			}
			while (!stack.empty())
			{
				Frame& frame = stack.back();
				if (frame.next < frame.children.size())
				{
					const JPH::Shape* child = frame.children[frame.next++].GetPtr();
					if (child == nullptr)
					{
						error = "a shape has a null child";
						return false;
					}
					if (mShapeIndices.find(child) == mShapeIndices.end() && !Push(child, stack, error))
					{
						return false;
					}
					continue;
				}
				if (!SaveRecord(frame, records, error))
				{
					return false;
				}
				stack.pop_back();
			}
			return true;
		}

		bool Push(const JPH::Shape* shape, std::vector<Frame>& stack, const char*& error)
		{
			if (!IsSupportedSubType(static_cast<uint8_t>(shape->GetSubType())))
			{
				error = "the shape graph holds a shape type that cannot be saved";
				return false;
			}
			for (const Frame& frame : stack)
			{
				if (frame.shape == shape)
				{
					error = "the shape graph has a cycle";
					return false;
				}
			}
			Frame frame{ shape, {}, 0 };
			shape->SaveSubShapeState(frame.children);
			stack.push_back(std::move(frame));
			return true;
		}

		bool SaveRecord(const Frame& frame, std::vector<uint8_t>& records, const char*& error)
		{
			const JPH::Shape* shape = frame.shape;
			JPH::PhysicsMaterialList materials;
			shape->SaveMaterialState(materials);

			std::vector<uint8_t> record;
			Writer writer(record);
			writer.Put(static_cast<uint8_t>(shape->GetSubType()));
			writer.Put(static_cast<uint32_t>(frame.children.size()));
			for (const JPH::ShapeRefC& child : frame.children)
			{
				writer.Put(mShapeIndices.at(child.GetPtr()));
			}
			writer.Put(static_cast<uint32_t>(materials.size()));
			for (const JPH::PhysicsMaterialRefC& material : materials)
			{
				uint32_t index = 0;
				if (!MaterialIndex(material.GetPtr(), index, error))
				{
					return false;
				}
				writer.Put(index);
			}
			if (shape->GetSubType() == JPH::EShapeSubType::Empty)
			{
				const JPH::Vec3 centerOfMass = shape->GetCenterOfMass();
				writer.Put(centerOfMass.GetX());
				writer.Put(centerOfMass.GetY());
				writer.Put(centerOfMass.GetZ());
			}
			std::vector<uint8_t> jolt;
			VectorStreamOut stream(jolt);
			shape->SaveBinaryState(stream);
			writer.Put(static_cast<uint32_t>(jolt.size()));
			writer.PutBytes(jolt.data(), jolt.size());

			Writer(records).Put(static_cast<uint32_t>(record.size()));
			Writer(records).PutBytes(record.data(), record.size());
			mShapeIndices.emplace(shape, static_cast<uint32_t>(mShapeIndices.size()));
			return true;
		}

		bool MaterialIndex(const JPH::PhysicsMaterial* material, uint32_t& index, const char*& error)
		{
			const auto found = std::find(mMaterials.begin(), mMaterials.end(), material);
			if (found != mMaterials.end())
			{
				index = static_cast<uint32_t>(found - mMaterials.begin());
				return true;
			}
			if (material != nullptr && material != JPH::PhysicsMaterial::sDefault.GetPtr()
				&& material->GetRTTI() != JPH_RTTI(JPH::PhysicsMaterialSimple) && !HasUserData(material))
			{
				error = "the shape graph holds a material type that cannot be saved";
				return false;
			}
			index = static_cast<uint32_t>(mMaterials.size());
			mMaterials.push_back(material);
			return true;
		}

		static bool HasUserData(const JPH::PhysicsMaterial* material)
		{
			uint64_t userData = 0;
			return JPH_PhysicsMaterial_GetUserData(reinterpret_cast<const JPH_PhysicsMaterial*>(material), &userData);
		}

		static void SaveMaterial(const JPH::PhysicsMaterial* material, Writer& writer)
		{
			MaterialKind kind = MaterialKind::Null;
			uint64_t userData = 0;
			uint32_t color = 0;
			std::string name;
			if (material == JPH::PhysicsMaterial::sDefault.GetPtr())
			{
				kind = MaterialKind::Default;
			}
			else if (material != nullptr)
			{
				kind = JPH_PhysicsMaterial_GetUserData(reinterpret_cast<const JPH_PhysicsMaterial*>(material), &userData)
					? MaterialKind::UserData : MaterialKind::Simple;
				color = material->GetDebugColor().GetUInt32();
				name = material->GetDebugName();
			}
			writer.Put(static_cast<uint8_t>(kind));
			writer.Put(userData);
			writer.Put(color);
			writer.Put(static_cast<uint32_t>(name.size()));
			writer.PutBytes(name.data(), name.size());
		}

		// Pointer to record index; looked up, never iterated, so its order does not matter.
		std::unordered_map<const JPH::Shape*, uint32_t> mShapeIndices;
		std::vector<const JPH::PhysicsMaterial*> mMaterials;
	};

	// Reads one material record; false on a short record or an unknown kind.
	bool RestoreMaterial(Reader& reader, JPH::PhysicsMaterialRefC& material)
	{
		uint8_t kind = 0;
		uint64_t userData = 0;
		uint32_t color = 0;
		uint32_t nameLength = 0;
		const uint8_t* name = nullptr;
		if (!reader.Get(kind) || !reader.Get(userData) || !reader.Get(color) || !reader.Get(nameLength)
			|| !reader.Take(nameLength, name))
		{
			return false;
		}
		const std::string nameText(reinterpret_cast<const char*>(name), nameLength);
		switch (static_cast<MaterialKind>(kind))
		{
		case MaterialKind::Null:
			material = nullptr;
			return true;
		case MaterialKind::Default:
			material = JPH::PhysicsMaterial::sDefault;
			return true;
		case MaterialKind::Simple:
			material = new JPH::PhysicsMaterialSimple(nameText, JPH::Color(color));
			return true;
		case MaterialKind::UserData:
		{
			// The new material holds one reference; the RefConst adds its own, so release the first.
			JPH::PhysicsMaterial* created = reinterpret_cast<JPH::PhysicsMaterial*>(JPH_PhysicsMaterial_Create2(nameText.c_str(), color, userData));
			material = created;
			created->Release();
			return true;
		}
		default:
			return false;
		}
	}

	// Restores Jolt's bytes of one record. Empty shapes are built here because Jolt's EmptyShape
	// does not save its centre of mass.
	bool RestoreJoltShape(uint8_t subType, const uint8_t* jolt, uint32_t joltLength, JPH::Vec3Arg emptyCenterOfMass, JPH::ShapeRefC& shape, std::string& error)
	{
		if (static_cast<JPH::EShapeSubType>(subType) == JPH::EShapeSubType::Empty)
		{
			JPH::uint64 userData = 0;
			if (joltLength != 1 + sizeof(userData))
			{
				error = "an empty shape record has the wrong length";
				return false;
			}
			std::memcpy(&userData, jolt + 1, sizeof(userData));
			JPH::Ref<JPH::Shape> empty = new JPH::EmptyShape(emptyCenterOfMass);
			empty->SetUserData(userData);
			shape = empty;
			return true;
		}
		SpanStreamIn stream(jolt, joltLength);
		JPH::Shape::ShapeResult result = JPH::Shape::sRestoreFromBinaryState(stream);
		if (result.HasError())
		{
			error = std::string("Jolt refused a record: ") + result.GetError().c_str();
			return false;
		}
		if (stream.Position() != joltLength)
		{
			error = "Jolt read fewer bytes than the record holds";
			return false;
		}
		shape = result.Get();
		return true;
	}

	// The child and material counts Jolt's RestoreSubShapeState and RestoreMaterialState accept for
	// this shape; they index their arrays with only an assert.
	bool HasExpectedCounts(const JPH::Shape& shape, size_t childCount, size_t materialCount, std::string& error)
	{
		size_t expectedChildren = 0;
		switch (shape.GetType())
		{
		case JPH::EShapeType::Decorated:
			expectedChildren = 1;
			break;
		case JPH::EShapeType::Compound:
			expectedChildren = static_cast<const JPH::CompoundShape&>(shape).GetNumSubShapes();
			break;
		default:
			break;
		}
		if (childCount != expectedChildren)
		{
			error = "a record has the wrong number of children";
			return false;
		}
		const JPH::EShapeType type = shape.GetType();
		const bool oneMaterial = type == JPH::EShapeType::Convex || type == JPH::EShapeType::Plane;
		const bool anyMaterials = type == JPH::EShapeType::Mesh || type == JPH::EShapeType::HeightField;
		if (!anyMaterials && materialCount != (oneMaterial ? 1u : 0u))
		{
			error = "a record has the wrong number of materials";
			return false;
		}
		return true;
	}

	// Restores the payload; null and a message on the first failed check.
	JPH::ShapeRefC Restore(const uint8_t* data, size_t size, std::string& error)
	{
		Reader reader(data, size);
		uint32_t materialCount = 0;
		if (!reader.Get(materialCount))
		{
			error = "the payload ends before the material count";
			return nullptr;
		}
		std::vector<JPH::PhysicsMaterialRefC> materials;
		for (uint32_t i = 0; i < materialCount; ++i)
		{
			JPH::PhysicsMaterialRefC material;
			if (!RestoreMaterial(reader, material))
			{
				error = "a material record is malformed";
				return nullptr;
			}
			materials.push_back(material);
		}

		uint32_t shapeCount = 0;
		if (!reader.Get(shapeCount) || shapeCount == 0)
		{
			error = "the payload holds no shape";
			return nullptr;
		}
		std::vector<JPH::ShapeRefC> shapes;
		std::vector<bool> used;
		for (uint32_t index = 0; index < shapeCount; ++index)
		{
			uint32_t recordLength = 0;
			const uint8_t* recordData = nullptr;
			if (!reader.Get(recordLength) || !reader.Take(recordLength, recordData))
			{
				error = "a shape record runs past the end of the payload";
				return nullptr;
			}
			Reader record(recordData, recordLength);

			uint8_t subType = 0;
			if (!record.Get(subType) || !IsSupportedSubType(subType))
			{
				error = "a record has an unsupported shape type";
				return nullptr;
			}
			std::vector<uint32_t> children;
			if (!GetIndices(record, index, children))
			{
				error = "a record's child index does not name an earlier record";
				return nullptr;
			}
			std::vector<uint32_t> materialIndices;
			if (!GetIndices(record, materialCount, materialIndices))
			{
				error = "a record's material index is out of range";
				return nullptr;
			}
			float centerOfMass[3] = { 0.0f, 0.0f, 0.0f };
			if (static_cast<JPH::EShapeSubType>(subType) == JPH::EShapeSubType::Empty
				&& !(record.Get(centerOfMass[0]) && record.Get(centerOfMass[1]) && record.Get(centerOfMass[2])))
			{
				error = "an empty shape record ends early";
				return nullptr;
			}
			uint32_t joltLength = 0;
			const uint8_t* jolt = nullptr;
			if (!record.Get(joltLength) || !record.Take(joltLength, jolt) || record.Remaining() != 0)
			{
				error = "a record's length does not match its contents";
				return nullptr;
			}
			if (joltLength == 0 || jolt[0] != subType)
			{
				error = "a record's shape type does not match Jolt's data";
				return nullptr;
			}

			JPH::ShapeRefC shape;
			if (!RestoreJoltShape(subType, jolt, joltLength, JPH::Vec3(centerOfMass[0], centerOfMass[1], centerOfMass[2]), shape, error)
				|| !HasExpectedCounts(*shape, children.size(), materialIndices.size(), error))
			{
				return nullptr;
			}
			std::vector<JPH::PhysicsMaterialRefC> shapeMaterials;
			for (uint32_t materialIndex : materialIndices)
			{
				shapeMaterials.push_back(materials[materialIndex]);
			}
			std::vector<JPH::ShapeRefC> shapeChildren;
			for (uint32_t child : children)
			{
				shapeChildren.push_back(shapes[child]);
				used[child] = true;
			}
			// The shape is new and owned here alone; Jolt's restore functions are non-const.
			JPH::Shape* mutableShape = const_cast<JPH::Shape*>(shape.GetPtr());
			mutableShape->RestoreMaterialState(shapeMaterials.data(), static_cast<JPH::uint>(shapeMaterials.size()));
			mutableShape->RestoreSubShapeState(shapeChildren.data(), static_cast<JPH::uint>(shapeChildren.size()));
			shapes.push_back(shape);
			used.push_back(false);
		}
		if (reader.Remaining() != 0)
		{
			error = "the payload has bytes after the last record";
			return nullptr;
		}
		if (std::find(used.begin(), used.end() - 1, false) != used.end() - 1)
		{
			error = "a record is not part of the root shape";
			return nullptr;
		}
		return shapes.back();
	}
}

uint32_t JPH_Shape_GetBinaryStateVersion(void)
{
	return cShapeBinaryStateVersion;
}

JPH_ShapeBinaryState* JPH_Shape_SaveBinaryState(const JPH_Shape* shape, char* error, uint32_t errorCapacity)
{
	if (shape == nullptr)
	{
		SetError(error, errorCapacity, "the shape is null");
		return nullptr;
	}
	auto state = new JPH_ShapeBinaryState();
	const char* message = nullptr;
	if (!Saver().Save(reinterpret_cast<const JPH::Shape*>(shape), state->data, message))
	{
		delete state;
		SetError(error, errorCapacity, message);
		return nullptr;
	}
	SetError(error, errorCapacity, "");
	return state;
}

size_t JPH_ShapeBinaryState_GetSize(const JPH_ShapeBinaryState* state)
{
	return state->data.size();
}

void JPH_ShapeBinaryState_CopyData(const JPH_ShapeBinaryState* state, void* data, size_t size)
{
	const size_t count = std::min(size, state->data.size());
	if (count > 0)
	{
		std::memcpy(data, state->data.data(), count);
	}
}

void JPH_ShapeBinaryState_Destroy(JPH_ShapeBinaryState* state)
{
	delete state;
}

JPH_Shape* JPH_Shape_RestoreBinaryState(const void* data, size_t size, char* error, uint32_t errorCapacity)
{
	if (data == nullptr && size != 0)
	{
		SetError(error, errorCapacity, "the data is null");
		return nullptr;
	}
	std::string message;
	const JPH::ShapeRefC shape = Restore(static_cast<const uint8_t*>(data), size, message);
	if (shape == nullptr)
	{
		SetError(error, errorCapacity, message.c_str());
		return nullptr;
	}
	SetError(error, errorCapacity, "");
	// The caller's reference; the RefConst releases its own when it goes out of scope.
	shape->AddRef();
	return reinterpret_cast<JPH_Shape*>(const_cast<JPH::Shape*>(shape.GetPtr()));
}
