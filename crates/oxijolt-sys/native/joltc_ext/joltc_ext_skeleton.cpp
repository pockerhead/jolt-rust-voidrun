// Skeleton mapper entry points that take JPH_Mat4 arrays of any alignment. Part of this fork's
// joltc additions (see joltc_ext.cpp).
// Licensed under the MIT License (MIT), like joltc.
//
// joltc's JPH_SkeletonMapper_Initialize, _LockAllTranslations, _LockTranslations, _Map and
// _MapReverse reinterpret_cast a JPH_Mat4 array (4-byte aligned) to JPH::Mat44 (16-byte aligned).
// These copy every array through JPH::Array<JPH::Mat44>, whose allocator aligns to the element
// type, and refuse every input that would make Jolt assert or read out of bounds. Skeleton and
// mapper handles are reinterpret_casts of the Jolt objects, as joltc's DEF_MAP_DECL defines them.

#include <Jolt/Jolt.h>

#include <Jolt/Skeleton/SkeletonMapper.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

#include "joltc_ext_internal.h"

namespace
{
	// The chain rule of JPH_SkeletonMapper_Map2; the derivation is in the repository's
	// docs/limits.md, section "Skeleton mapper chains".
	// Shortest usable chain direction (actual and desired), metres. Above 2^-10.5, so the
	// quaternion Quat::sFromTo normalises has a normal length.
	constexpr float cMinChainLength = 1.0e-3f;
	// Longest chain direction (actual with its allowance, and desired), metres: keeps each
	// squared length in Quat::sFromTo below FLT_MAX.
	constexpr double cMaxChainLength = 1.0e18;
	// Largest |actual| * |desired|: keeps the product of the squared lengths below FLT_MAX.
	constexpr double cMaxChainProduct = 4.0e18;
	// Largest sum of absolute terms of an entry of the chain's matrix products: keeps every
	// f32 product and partial sum finite.
	constexpr double cMaxChainMagnitude = 1.0e30;
	// Unit roundoff of float, 2^-24, and gamma_4 = 4u / (1 - 4u): a 4-term inner product in
	// float, in any order and with or without fused multiply-adds, is within gamma_4 times the
	// sum of its absolute terms.
	constexpr double cUnitRoundoff = 1.0 / 16777216.0;
	constexpr double cGamma4 = 4.0 * cUnitRoundoff / (1.0 - 4.0 * cUnitRoundoff);
	// 32 * FLT_MIN: per unit of 1 + the norms of a product's factors, covers what flushing
	// results or inputs below FLT_MIN to zero can change in the product.
	constexpr double cUnderflow = 0x1p-121;

	using Mat44Array = JPH::Array<JPH::Mat44>;

	const JPH::Skeleton& AsSkeleton(const JPH_Skeleton* skeleton)
	{
		return *reinterpret_cast<const JPH::Skeleton*>(skeleton);
	}

	JPH::SkeletonMapper& AsMapper(JPH_SkeletonMapper* mapper)
	{
		return *reinterpret_cast<JPH::SkeletonMapper*>(mapper);
	}

	const JPH::SkeletonMapper& AsMapper(const JPH_SkeletonMapper* mapper)
	{
		return *reinterpret_cast<const JPH::SkeletonMapper*>(mapper);
	}

	// Finite entries and a bottom row of exactly (0, 0, 0, 1): what Mat44::GetRotation asserts for
	// a chain start and what keeps the products of a chain finite.
	bool IsRigidLayout(const JPH_Mat4& matrix)
	{
		for (const JPH_Vec4& column : matrix.column)
		{
			if (!std::isfinite(column.x) || !std::isfinite(column.y) || !std::isfinite(column.z) || !std::isfinite(column.w))
			{
				return false;
			}
		}
		return matrix.column[0].w == 0.0f && matrix.column[1].w == 0.0f && matrix.column[2].w == 0.0f && matrix.column[3].w == 1.0f;
	}

	bool IsRigidLayout(JPH::Mat44Arg matrix)
	{
		JPH_Mat4 copy;
		std::memcpy(&copy, &matrix, sizeof(JPH_Mat4));
		return IsRigidLayout(copy);
	}

	bool AreRigidLayouts(const JPH_Mat4* matrices, uint32_t count)
	{
		for (uint32_t i = 0; i < count; ++i)
		{
			if (!IsRigidLayout(matrices[i]))
			{
				return false;
			}
		}
		return true;
	}

	Mat44Array CopyIn(const JPH_Mat4* matrices, uint32_t count)
	{
		Mat44Array result(count, JPH::Mat44::sIdentity());
		for (uint32_t i = 0; i < count; ++i)
		{
			std::memcpy(&result[i], &matrices[i], sizeof(JPH_Mat4));
		}
		return result;
	}

	void CopyOut(const Mat44Array& matrices, JPH_Mat4* result)
	{
		for (size_t i = 0; i < matrices.size(); ++i)
		{
			std::memcpy(&result[i], &matrices[i], sizeof(JPH_Mat4));
		}
	}

	bool IsCount(uint32_t count)
	{
		return count <= uint32_t(INT32_MAX);
	}

	// A count Jolt's int indices can address that equals the skeleton's joint count.
	bool IsJointCount(const JPH_Skeleton* skeleton, uint32_t count)
	{
		return IsCount(count) && AsSkeleton(skeleton).GetJointCount() == int(count);
	}

	bool IsInitialized(const JPH::SkeletonMapper& mapper)
	{
		return !mapper.GetMappings().empty() || !mapper.GetChains().empty() || !mapper.GetUnmapped().empty() || !mapper.GetLockedTranslations().empty();
	}

	bool IsIndex(int index, uint32_t count)
	{
		return index >= 0 && uint32_t(index) < count;
	}

	bool AreIndices(const JPH::Array<int>& indices, uint32_t count)
	{
		for (int index : indices)
		{
			if (!IsIndex(index, count))
			{
				return false;
			}
		}
		return true;
	}

	// Every mapping in range with rigid transforms both ways.
	bool MappingsUseOnly(const JPH::SkeletonMapper& mapper, uint32_t count1, uint32_t count2)
	{
		for (const JPH::SkeletonMapper::Mapping& mapping : mapper.GetMappings())
		{
			if (!IsIndex(mapping.mJointIdx1, count1) || !IsIndex(mapping.mJointIdx2, count2)
				|| !IsRigidLayout(mapping.mJoint1To2) || !IsRigidLayout(mapping.mJoint2To1))
			{
				return false;
			}
		}
		return true;
	}

	// Every index Map reads is within the counts and every stored transform is rigid.
	bool UsesOnly(const JPH::SkeletonMapper& mapper, uint32_t count1, uint32_t count2)
	{
		if (!MappingsUseOnly(mapper, count1, count2))
		{
			return false;
		}
		for (const JPH::SkeletonMapper::Chain& chain : mapper.GetChains())
		{
			if (chain.mJointIndices1.empty() || chain.mJointIndices2.size() < 2
				|| !AreIndices(chain.mJointIndices1, count1) || !AreIndices(chain.mJointIndices2, count2))
			{
				return false;
			}
		}
		for (const JPH::SkeletonMapper::Unmapped& unmapped : mapper.GetUnmapped())
		{
			// Map treats every negative parent as a root and asserts parents first otherwise.
			if (!IsIndex(unmapped.mJointIdx, count2) || unmapped.mParentJointIdx >= unmapped.mJointIdx)
			{
				return false;
			}
		}
		for (const JPH::SkeletonMapper::Locked& locked : mapper.GetLockedTranslations())
		{
			if (!IsIndex(locked.mJointIdx, count2) || !IsIndex(locked.mParentJointIdx, count2))
			{
				return false;
			}
		}
		return true;
	}

	double Length(JPH::Vec3Arg vector)
	{
		const double x = vector.GetX(), y = vector.GetY(), z = vector.GetZ();
		return std::sqrt(x * x + y * y + z * z);
	}

	// Norms of the linear block L and the translation t of a rigid-layout matrix [L t; 0 1],
	// from its float entries, in double.
	struct Norms
	{
		double spectral;    // At least ||L||_2: the root of the largest row sum of |L^T L|.
		double frobenius;   // ||L||_F.
		double translation; // |t|.
	};

	Norms NormsOf(JPH::Mat44Arg matrix)
	{
		double gram[3][3];
		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				gram[i][j] = 0.0;
				for (int k = 0; k < 3; ++k)
				{
					gram[i][j] += double(matrix(k, i)) * double(matrix(k, j));
				}
			}
		}
		double largestRowSum = 0.0;
		for (const auto& row : gram)
		{
			largestRowSum = std::max(largestRowSum, std::fabs(row[0]) + std::fabs(row[1]) + std::fabs(row[2]));
		}
		return { std::sqrt(largestRowSum), std::sqrt(gram[0][0] + gram[1][1] + gram[2][2]), Length(matrix.GetTranslation()) };
	}

	// Bounds on a product of rigid-layout matrices that float arithmetic computes in any
	// evaluation order: on the exact product's linear block (spectral norm) and translation,
	// on how far any computed copy is from them (Frobenius norm, length), and on the sums of
	// absolute terms of every entry computed so far.
	struct ChainBound
	{
		double rotation;
		double translation;
		double rotationError;
		double translationError;
		double magnitude;

		// The bound of a * b.
		static ChainBound Product(JPH::Mat44Arg a, JPH::Mat44Arg b)
		{
			const Norms na = NormsOf(a), nb = NormsOf(b);
			const double rotationTerms = na.frobenius * nb.frobenius;
			const double translationTerms = na.frobenius * nb.translation + na.translation;
			const double underflow = cUnderflow * (1.0 + na.frobenius + nb.frobenius + nb.translation);
			return {
				na.spectral * nb.spectral,
				na.spectral * nb.translation + na.translation,
				cGamma4 * rotationTerms + underflow,
				cGamma4 * translationTerms + underflow,
				std::max(rotationTerms, translationTerms),
			};
		}

		// Multiplies the product by local on the right.
		void Then(JPH::Mat44Arg local)
		{
			const Norms n = NormsOf(local);
			// The computed linear block's Frobenius norm and translation's length, at most.
			const double frobenius = std::sqrt(3.0) * rotation + rotationError;
			const double reach = Reach();
			const double rotationTerms = frobenius * n.frobenius;
			const double translationTerms = frobenius * n.translation + reach;
			const double underflow = cUnderflow * (1.0 + frobenius + n.frobenius + n.translation);
			translationError += rotationError * n.translation + cGamma4 * translationTerms + underflow;
			rotationError = rotationError * n.spectral + cGamma4 * rotationTerms + underflow;
			translation += rotation * n.translation;
			rotation *= n.spectral;
			magnitude = std::max(magnitude, std::max(rotationTerms, translationTerms));
		}

		// The longest translation any computed copy can have.
		double Reach() const
		{
			return translation + translationError;
		}
	};

	// Whether Map can turn this chain without a non-unit quaternion. outPose holds the direct
	// mappings, as Map has them when it reaches the chain; start is the mapping of the chain start.
	// desired is replayed exactly; actual is computed again and bounded, since this product and
	// Map's may round differently (summation order, fused multiply-adds).
	bool IsUsableChain(const JPH::SkeletonMapper::Chain& chain, const JPH::SkeletonMapper::Mapping& start, const Mat44Array& pose1, const Mat44Array& local2, const Mat44Array& outPose)
	{
		const ChainBound startBound = ChainBound::Product(pose1[start.mJointIdx1], start.mJoint1To2);
		const JPH::Mat44& chainStart = outPose[chain.mJointIndices2.front()];
		JPH::Mat44 chainEnd = chainStart;
		// A chain has at least one link, so the loop checks the start's magnitude too.
		ChainBound endBound = startBound;
		for (size_t j = 1; j < chain.mJointIndices2.size(); ++j)
		{
			const JPH::Mat44& local = local2[chain.mJointIndices2[j]];
			endBound.Then(local);
			if (!(endBound.magnitude <= cMaxChainMagnitude))
			{
				return false;
			}
			chainEnd = chainEnd * local;
		}

		const JPH::Vec3 actual = chainEnd.GetTranslation() - chainStart.GetTranslation();
		const JPH::Vec3 desired = pose1[chain.mJointIndices1.back()].GetTranslation() - pose1[chain.mJointIndices1.front()].GetTranslation();

		// How far Map's actual can be from this one: twice the distance of either from the exact
		// difference, with the rounding of the final subtraction.
		const double fromExact = endBound.translationError + startBound.translationError
			+ cUnitRoundoff * (endBound.Reach() + startBound.Reach()) + cUnderflow;
		const double allowance = 1.001 * 2.0 * fromExact;

		const double actualLength = Length(actual);
		const double desiredLength = Length(desired);
		const double minLength = double(cMinChainLength);
		const bool desiredIsZero = desired.GetX() == 0.0f && desired.GetY() == 0.0f && desired.GetZ() == 0.0f;
		return (desiredIsZero || desiredLength >= minLength)
			&& desiredLength <= cMaxChainLength
			&& actualLength - allowance >= minLength
			&& actualLength + allowance <= cMaxChainLength
			&& (actualLength + allowance) * desiredLength <= cMaxChainProduct;
	}
}

bool JPH_SkeletonMapper_Initialize2(JPH_SkeletonMapper* mapper, const JPH_Skeleton* skeleton1, const JPH_Mat4* neutralPose1, uint32_t count1, const JPH_Skeleton* skeleton2, const JPH_Mat4* neutralPose2, uint32_t count2)
{
	if (count1 == 0 || count1 > count2 || !IsJointCount(skeleton1, count1) || !IsJointCount(skeleton2, count2))
	{
		return false;
	}
	if (!AsSkeleton(skeleton1).AreJointsCorrectlyOrdered() || !AsSkeleton(skeleton2).AreJointsCorrectlyOrdered())
	{
		return false;
	}
	if (!AreRigidLayouts(neutralPose1, count1) || !AreRigidLayouts(neutralPose2, count2))
	{
		return false;
	}
	JPH::SkeletonMapper& joltMapper = AsMapper(mapper);
	if (IsInitialized(joltMapper))
	{
		return false;
	}
	const Mat44Array neutral1 = CopyIn(neutralPose1, count1);
	const Mat44Array neutral2 = CopyIn(neutralPose2, count2);
	joltMapper.Initialize(&AsSkeleton(skeleton1), neutral1.data(), &AsSkeleton(skeleton2), neutral2.data());
	return true;
}

bool JPH_SkeletonMapper_LockAllTranslations2(JPH_SkeletonMapper* mapper, const JPH_Skeleton* skeleton2, const JPH_Mat4* neutralPose2, uint32_t count2)
{
	if (!IsJointCount(skeleton2, count2) || !AsSkeleton(skeleton2).AreJointsCorrectlyOrdered() || !AreRigidLayouts(neutralPose2, count2))
	{
		return false;
	}
	JPH::SkeletonMapper& joltMapper = AsMapper(mapper);
	// LockAllTranslations marks the descendants of the first mapping's joint in a stack array of
	// count2 bools, starting at that index.
	if (joltMapper.GetMappings().empty() || !IsIndex(joltMapper.GetMappings()[0].mJointIdx2, count2) || !joltMapper.GetLockedTranslations().empty())
	{
		return false;
	}
	const Mat44Array neutral2 = CopyIn(neutralPose2, count2);
	joltMapper.LockAllTranslations(&AsSkeleton(skeleton2), neutral2.data());
	return true;
}

bool JPH_SkeletonMapper_LockTranslations2(JPH_SkeletonMapper* mapper, const JPH_Skeleton* skeleton2, const bool* lockedTranslations, const JPH_Mat4* neutralPose2, uint32_t count2)
{
	if (!IsJointCount(skeleton2, count2) || !AsSkeleton(skeleton2).AreJointsCorrectlyOrdered())
	{
		return false;
	}
	// Map sets a locked joint's translation from its parent's output: a root has none.
	const JPH::Skeleton& joltSkeleton = AsSkeleton(skeleton2);
	for (uint32_t i = 0; i < count2; ++i)
	{
		if (lockedTranslations[i] && joltSkeleton.GetJoint(int(i)).mParentJointIndex < 0)
		{
			return false;
		}
	}
	if (!AreRigidLayouts(neutralPose2, count2))
	{
		return false;
	}
	JPH::SkeletonMapper& joltMapper = AsMapper(mapper);
	if (!IsInitialized(joltMapper) || !joltMapper.GetLockedTranslations().empty())
	{
		return false;
	}
	const Mat44Array neutral2 = CopyIn(neutralPose2, count2);
	joltMapper.LockTranslations(&joltSkeleton, lockedTranslations, neutral2.data());
	return true;
}

bool JPH_SkeletonMapper_Map2(const JPH_SkeletonMapper* mapper, const JPH_Mat4* pose1ModelSpace, uint32_t count1, const JPH_Mat4* pose2LocalSpace, uint32_t count2, JPH_Mat4* outPose2ModelSpace, int* outDegenerateJoint1)
{
	if (outDegenerateJoint1 != nullptr)
	{
		*outDegenerateJoint1 = -1;
	}
	const JPH::SkeletonMapper& joltMapper = AsMapper(mapper);
	const JPH::SkeletonMapper::MappingVector& mappings = joltMapper.GetMappings();
	if (mappings.empty() || !IsCount(count1) || !IsCount(count2) || !UsesOnly(joltMapper, count1, count2))
	{
		return false;
	}
	if (!AreRigidLayouts(pose1ModelSpace, count1) || !AreRigidLayouts(pose2LocalSpace, count2))
	{
		return false;
	}
	const Mat44Array pose1 = CopyIn(pose1ModelSpace, count1);
	const Mat44Array local2 = CopyIn(pose2LocalSpace, count2);

	// The direct mappings, as Map writes them first. A chain start must be written by exactly one
	// mapping: Map rotates it in place, so a second chain from the same joint would start from a
	// rotated transform.
	Mat44Array outPose(count2, JPH::Mat44::sIdentity());
	JPH::Array<int> mappingOf2(count2, -1);
	for (size_t i = 0; i < mappings.size(); ++i)
	{
		const JPH::SkeletonMapper::Mapping& mapping = mappings[i];
		if (mappingOf2[mapping.mJointIdx2] >= 0)
		{
			return false;
		}
		mappingOf2[mapping.mJointIdx2] = int(i);
		outPose[mapping.mJointIdx2] = pose1[mapping.mJointIdx1] * mapping.mJoint1To2;
	}
	for (const JPH::SkeletonMapper::Chain& chain : joltMapper.GetChains())
	{
		const int start = mappingOf2[chain.mJointIndices2.front()];
		if (start < 0 || !IsUsableChain(chain, mappings[start], pose1, local2, outPose))
		{
			if (outDegenerateJoint1 != nullptr)
			{
				*outDegenerateJoint1 = chain.mJointIndices1.back();
			}
			return false;
		}
	}

	joltMapper.Map(pose1.data(), local2.data(), outPose.data());
	CopyOut(outPose, outPose2ModelSpace);
	return true;
}

bool JPH_SkeletonMapper_MapReverse2(const JPH_SkeletonMapper* mapper, const JPH_Mat4* pose2ModelSpace, uint32_t count2, JPH_Mat4* outPose1ModelSpace, uint32_t count1)
{
	const JPH::SkeletonMapper& joltMapper = AsMapper(mapper);
	if (joltMapper.GetMappings().empty() || !IsCount(count1) || !IsCount(count2) || !MappingsUseOnly(joltMapper, count1, count2))
	{
		return false;
	}
	if (!AreRigidLayouts(pose2ModelSpace, count2))
	{
		return false;
	}
	const Mat44Array pose2 = CopyIn(pose2ModelSpace, count2);
	Mat44Array outPose(count1, JPH::Mat44::sIdentity());
	joltMapper.MapReverse(pose2.data(), outPose.data());
	CopyOut(outPose, outPose1ModelSpace);
	return true;
}
