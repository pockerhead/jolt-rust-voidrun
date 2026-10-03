// Functions this fork adds to joltc, in joltc's naming and style. They are compiled into the
// joltc archive (see ../CMakeLists.txt) and are meant to be offered upstream.
// Licensed under the MIT License (MIT), like joltc.

#ifndef JOLT_C_EXT_H_
#define JOLT_C_EXT_H_ 1

#include "joltc.h"

/* StateRecorder */
typedef struct JPH_StateRecorder JPH_StateRecorder; /* JPH::StateRecorderImpl */

JPH_CAPI JPH_StateRecorder* JPH_StateRecorder_Create(void);
JPH_CAPI void JPH_StateRecorder_Destroy(JPH_StateRecorder* recorder);
JPH_CAPI void JPH_StateRecorder_Rewind(JPH_StateRecorder* recorder);
JPH_CAPI void JPH_StateRecorder_WriteBytes(JPH_StateRecorder* recorder, const void* data, size_t size);
JPH_CAPI size_t JPH_StateRecorder_GetDataSize(JPH_StateRecorder* recorder);
/* Copies min(size, data size) bytes of the recorded data to data, which may be null when that is 0. */
JPH_CAPI void JPH_StateRecorder_CopyData(JPH_StateRecorder* recorder, void* data, size_t size);
JPH_CAPI bool JPH_StateRecorder_IsFailed(const JPH_StateRecorder* recorder);

/* Which parts of a physics system's state JPH_PhysicsSystem_SaveState writes (JPH::EStateRecorderState). */
typedef enum JPH_StateRecorderState {
	JPH_StateRecorderState_None = 0,
	JPH_StateRecorderState_Global = 1 << 0,
	JPH_StateRecorderState_Bodies = 1 << 1,
	JPH_StateRecorderState_Contacts = 1 << 2,
	JPH_StateRecorderState_Constraints = 1 << 3,
	JPH_StateRecorderState_All = 15,

	_JPH_StateRecorderState_Force32 = 0x7fffffff
} JPH_StateRecorderState;

/* PhysicsSystem state */
/* bodies null: every body is saved; otherwise only the bodies whose id is among the bodyCount ids
   (any order, duplicates allowed), none for bodyCount 0, when bodies need not point to any object.
   Contacts and constraints are never filtered. */
JPH_CAPI void JPH_PhysicsSystem_SaveState(const JPH_PhysicsSystem* system, JPH_StateRecorder* recorder, JPH_StateRecorderState state, const JPH_BodyID* bodies, uint32_t bodyCount);
/* Reads from the recorder's current read position (rewind it first). Never validating. Returns
   false when Jolt could not restore; the system may then be partly restored. */
JPH_CAPI bool JPH_PhysicsSystem_RestoreState(JPH_PhysicsSystem* system, JPH_StateRecorder* recorder);

/* CharacterVirtual */
JPH_CAPI void JPH_CharacterVirtual_SaveState(const JPH_CharacterVirtual* character, JPH_StateRecorder* recorder);
JPH_CAPI void JPH_CharacterVirtual_RestoreState(JPH_CharacterVirtual* character, JPH_StateRecorder* recorder);

/* Like JPH_CharacterVirtual_ExtendedUpdate and JPH_CharacterVirtual_RefreshContacts, with explicit
   gravity, filters (null accepts everything) and temp allocator instead of the system's gravity, its
   layer tables and joltc's global temp allocator. */
JPH_CAPI void JPH_CharacterVirtual_ExtendedUpdate2(JPH_CharacterVirtual* character, float deltaTime,
	const JPH_Vec3* gravity, const JPH_ExtendedUpdateSettings* settings,
	const JPH_BroadPhaseLayerFilter* broadPhaseLayerFilter, const JPH_ObjectLayerFilter* objectLayerFilter,
	const JPH_BodyFilter* bodyFilter, const JPH_ShapeFilter* shapeFilter, JPH_TempAllocator* tempAllocator);
JPH_CAPI void JPH_CharacterVirtual_RefreshContacts2(JPH_CharacterVirtual* character,
	const JPH_BroadPhaseLayerFilter* broadPhaseLayerFilter, const JPH_ObjectLayerFilter* objectLayerFilter,
	const JPH_BodyFilter* bodyFilter, const JPH_ShapeFilter* shapeFilter, JPH_TempAllocator* tempAllocator);

/* VehicleConstraint */
/* The Constraint base of the vehicle constraint, for JPH_PhysicsSystem_AddConstraint, _RemoveConstraint
   and JPH_Constraint_Destroy; counterpart of JPH_VehicleConstraint_AsPhysicsStepListener. */
JPH_CAPI JPH_Constraint* JPH_VehicleConstraint_AsConstraint(JPH_VehicleConstraint* constraint);

/* VehicleCollisionTester */
/* Like JPH_VehicleCollisionTesterRay_Create, _CastSphere_Create and _CastCylinder_Create, but the tester
   skips soft bodies as well as the chassis vehicleBody (Jolt's default body filter skips only the
   chassis): Jolt's VehicleConstraint treats the body under a wheel as rigid. Each returns a new tester
   holding one reference. */
JPH_CAPI JPH_VehicleCollisionTesterRay* JPH_VehicleCollisionTesterRay_Create2(JPH_ObjectLayer layer, const JPH_Vec3* up, float maxSlopeAngle, JPH_BodyID vehicleBody);
JPH_CAPI JPH_VehicleCollisionTesterCastSphere* JPH_VehicleCollisionTesterCastSphere_Create2(JPH_ObjectLayer layer, float radius, const JPH_Vec3* up, float maxSlopeAngle, JPH_BodyID vehicleBody);
JPH_CAPI JPH_VehicleCollisionTesterCastCylinder* JPH_VehicleCollisionTesterCastCylinder_Create2(JPH_ObjectLayer layer, float convexRadiusFraction, JPH_BodyID vehicleBody);

/* RagdollSettings */
/* Copies every BodyCreationSettings field (shape reference, collision group, damping, velocities, mass
   override, ...) into part partIndex; the part's constraint to its parent is kept. */
JPH_CAPI void JPH_RagdollSettings_SetPart(JPH_RagdollSettings* settings, int partIndex, const JPH_BodyCreationSettings* bodySettings);
/* Like JPH_RagdollSettings_SetPartToParent, but every field reaches Jolt, base settings, spring modes and
   torque limits included; null removes the constraint. */
JPH_CAPI void JPH_RagdollSettings_SetPartToParentSwingTwist(JPH_RagdollSettings* settings, int partIndex, const JPH_SwingTwistConstraintSettings* constraintSettings);
JPH_CAPI void JPH_RagdollSettings_SetPartToParentHinge(JPH_RagdollSettings* settings, int partIndex, const JPH_HingeConstraintSettings* constraintSettings);
JPH_CAPI void JPH_RagdollSettings_SetPartToParentSixDOF(JPH_RagdollSettings* settings, int partIndex, const JPH_SixDOFConstraintSettings* constraintSettings);
JPH_CAPI void JPH_RagdollSettings_CalculateConstraintPriorities(JPH_RagdollSettings* settings, uint32_t basePriority);

/* SwingTwistConstraint */
JPH_CAPI void JPH_SwingTwistConstraint_SetSwingMotorState(JPH_SwingTwistConstraint* constraint, JPH_MotorState state);
JPH_CAPI JPH_MotorState JPH_SwingTwistConstraint_GetSwingMotorState(const JPH_SwingTwistConstraint* constraint);
JPH_CAPI void JPH_SwingTwistConstraint_SetTwistMotorState(JPH_SwingTwistConstraint* constraint, JPH_MotorState state);
JPH_CAPI JPH_MotorState JPH_SwingTwistConstraint_GetTwistMotorState(const JPH_SwingTwistConstraint* constraint);
JPH_CAPI void JPH_SwingTwistConstraint_SetTargetOrientationBS(JPH_SwingTwistConstraint* constraint, const JPH_Quat* orientation);
JPH_CAPI void JPH_SwingTwistConstraint_GetRotationInConstraintSpace(const JPH_SwingTwistConstraint* constraint, JPH_Quat* result);
JPH_CAPI void JPH_SwingTwistConstraint_SetTargetAngularVelocityCS(JPH_SwingTwistConstraint* constraint, const JPH_Vec3* angularVelocity);
JPH_CAPI void JPH_SwingTwistConstraint_GetTargetAngularVelocityCS(const JPH_SwingTwistConstraint* constraint, JPH_Vec3* result);
JPH_CAPI void JPH_SwingTwistConstraint_SetTargetOrientationCS(JPH_SwingTwistConstraint* constraint, const JPH_Quat* orientation);
JPH_CAPI void JPH_SwingTwistConstraint_GetTargetOrientationCS(const JPH_SwingTwistConstraint* constraint, JPH_Quat* result);
JPH_CAPI void JPH_SwingTwistConstraint_SetSwingMotorSettings(JPH_SwingTwistConstraint* constraint, const JPH_MotorSettings* settings);
JPH_CAPI void JPH_SwingTwistConstraint_GetSwingMotorSettings(const JPH_SwingTwistConstraint* constraint, JPH_MotorSettings* result);
JPH_CAPI void JPH_SwingTwistConstraint_SetTwistMotorSettings(JPH_SwingTwistConstraint* constraint, const JPH_MotorSettings* settings);
JPH_CAPI void JPH_SwingTwistConstraint_GetTwistMotorSettings(const JPH_SwingTwistConstraint* constraint, JPH_MotorSettings* result);
JPH_CAPI void JPH_SwingTwistConstraint_SetMaxFrictionTorque(JPH_SwingTwistConstraint* constraint, float frictionTorque);
JPH_CAPI float JPH_SwingTwistConstraint_GetMaxFrictionTorque(const JPH_SwingTwistConstraint* constraint);
JPH_CAPI float JPH_SwingTwistConstraint_GetPlaneHalfConeAngle(const JPH_SwingTwistConstraint* constraint);
JPH_CAPI float JPH_SwingTwistConstraint_GetTwistMinAngle(const JPH_SwingTwistConstraint* constraint);
JPH_CAPI float JPH_SwingTwistConstraint_GetTwistMaxAngle(const JPH_SwingTwistConstraint* constraint);

/* HingeConstraint */
JPH_CAPI void JPH_HingeConstraint_SetTargetOrientationBS(JPH_HingeConstraint* constraint, const JPH_Quat* orientation);

/* SixDOFConstraint */
JPH_CAPI void JPH_SixDOFConstraint_SetMotorSettings(JPH_SixDOFConstraint* constraint, JPH_SixDOFConstraintAxis axis, const JPH_MotorSettings* settings);

/* PathConstraintPath */
/* A path for a path constraint (JPH::PathConstraintPath). JPH_PathConstraintPathHermite_Create returns
   it holding one reference, which JPH_PathConstraintPath_Destroy releases; the path constraint settings
   and every path constraint created from them take their own reference (RefConst). */
typedef struct JPH_PathConstraintPath JPH_PathConstraintPath; /* JPH::PathConstraintPath */

JPH_CAPI void JPH_PathConstraintPath_Destroy(JPH_PathConstraintPath* path);
JPH_CAPI void JPH_PathConstraintPath_SetIsLooping(JPH_PathConstraintPath* path, bool isLooping);
JPH_CAPI bool JPH_PathConstraintPath_IsLooping(const JPH_PathConstraintPath* path);
JPH_CAPI float JPH_PathConstraintPath_GetPathMaxFraction(const JPH_PathConstraintPath* path);
JPH_CAPI float JPH_PathConstraintPath_GetClosestPoint(const JPH_PathConstraintPath* path, const JPH_Vec3* position, float fractionHint);

/* A JPH::PathConstraintPathHermite, as its JPH::PathConstraintPath base. */
JPH_CAPI JPH_PathConstraintPath* JPH_PathConstraintPathHermite_Create(void);
/* path must come from JPH_PathConstraintPathHermite_Create. */
JPH_CAPI void JPH_PathConstraintPathHermite_AddPoint(JPH_PathConstraintPath* path, const JPH_Vec3* position, const JPH_Vec3* tangent, const JPH_Vec3* normal);

/* PathConstraint */
typedef enum JPH_PathRotationConstraintType {
	JPH_PathRotationConstraintType_Free = 0,
	JPH_PathRotationConstraintType_ConstrainAroundTangent = 1,
	JPH_PathRotationConstraintType_ConstrainAroundNormal = 2,
	JPH_PathRotationConstraintType_ConstrainAroundBinormal = 3,
	JPH_PathRotationConstraintType_ConstrainToPath = 4,
	JPH_PathRotationConstraintType_FullyConstrained = 5,

	_JPH_PathRotationConstraintType_Count,
	_JPH_PathRotationConstraintType_Force32 = 0x7fffffff
} JPH_PathRotationConstraintType;

typedef struct JPH_PathConstraintSettings {
	JPH_ConstraintSettings			base;    /* Inherits JPH_ConstraintSettings */

	const JPH_PathConstraintPath*	path;
	JPH_Vec3						pathPosition;
	JPH_Quat						pathRotation;
	float							pathFraction;
	float							maxFrictionForce;
	JPH_PathRotationConstraintType	rotationConstraintType;
	JPH_MotorSettings				positionMotorSettings;
} JPH_PathConstraintSettings;

typedef struct JPH_PathConstraint JPH_PathConstraint; /* JPH::PathConstraint */

/* Jolt's defaults, with a null path. */
JPH_CAPI void JPH_PathConstraintSettings_Init(JPH_PathConstraintSettings* settings);
JPH_CAPI JPH_PathConstraint* JPH_PathConstraint_Create(const JPH_PathConstraintSettings* settings, JPH_Body* body1, JPH_Body* body2);
/* The constraint's path, borrowed from the constraint. */
JPH_CAPI const JPH_PathConstraintPath* JPH_PathConstraint_GetPath(const JPH_PathConstraint* constraint);
JPH_CAPI float JPH_PathConstraint_GetPathFraction(const JPH_PathConstraint* constraint);
JPH_CAPI void JPH_PathConstraint_SetMaxFrictionForce(JPH_PathConstraint* constraint, float frictionForce);
JPH_CAPI float JPH_PathConstraint_GetMaxFrictionForce(const JPH_PathConstraint* constraint);
JPH_CAPI void JPH_PathConstraint_SetPositionMotorSettings(JPH_PathConstraint* constraint, const JPH_MotorSettings* settings);
JPH_CAPI void JPH_PathConstraint_GetPositionMotorSettings(const JPH_PathConstraint* constraint, JPH_MotorSettings* result);
JPH_CAPI void JPH_PathConstraint_SetPositionMotorState(JPH_PathConstraint* constraint, JPH_MotorState state);
JPH_CAPI JPH_MotorState JPH_PathConstraint_GetPositionMotorState(const JPH_PathConstraint* constraint);
JPH_CAPI void JPH_PathConstraint_SetTargetVelocity(JPH_PathConstraint* constraint, float velocity);
JPH_CAPI float JPH_PathConstraint_GetTargetVelocity(const JPH_PathConstraint* constraint);
JPH_CAPI void JPH_PathConstraint_SetTargetPathFraction(JPH_PathConstraint* constraint, float fraction);
JPH_CAPI float JPH_PathConstraint_GetTargetPathFraction(const JPH_PathConstraint* constraint);
JPH_CAPI void JPH_PathConstraint_GetTotalLambdaPosition(const JPH_PathConstraint* constraint, float result[2]);
JPH_CAPI float JPH_PathConstraint_GetTotalLambdaPositionLimits(const JPH_PathConstraint* constraint);
JPH_CAPI float JPH_PathConstraint_GetTotalLambdaMotor(const JPH_PathConstraint* constraint);
JPH_CAPI void JPH_PathConstraint_GetTotalLambdaRotationHinge(const JPH_PathConstraint* constraint, float result[2]);
JPH_CAPI void JPH_PathConstraint_GetTotalLambdaRotation(const JPH_PathConstraint* constraint, JPH_Vec3* result);

/* PulleyConstraint */
typedef struct JPH_PulleyConstraintSettings {
	JPH_ConstraintSettings		base;    /* Inherits JPH_ConstraintSettings */

	JPH_ConstraintSpace			space;
	JPH_RVec3					bodyPoint1;
	JPH_RVec3					fixedPoint1;
	JPH_RVec3					bodyPoint2;
	JPH_RVec3					fixedPoint2;
	float						ratio;
	float						minLength;
	float						maxLength;
} JPH_PulleyConstraintSettings;

typedef struct JPH_PulleyConstraint JPH_PulleyConstraint; /* JPH::PulleyConstraint */

JPH_CAPI void JPH_PulleyConstraintSettings_Init(JPH_PulleyConstraintSettings* settings);
JPH_CAPI JPH_PulleyConstraint* JPH_PulleyConstraint_Create(const JPH_PulleyConstraintSettings* settings, JPH_Body* body1, JPH_Body* body2);
/* The body points come back relative to the centres of mass (LocalToBodyCOM), the fixed points in
   world space, and the lengths as resolved when the constraint was created. */
JPH_CAPI void JPH_PulleyConstraint_GetSettings(const JPH_PulleyConstraint* constraint, JPH_PulleyConstraintSettings* settings);
JPH_CAPI void JPH_PulleyConstraint_SetLength(JPH_PulleyConstraint* constraint, float minLength, float maxLength);
JPH_CAPI float JPH_PulleyConstraint_GetMinLength(const JPH_PulleyConstraint* constraint);
JPH_CAPI float JPH_PulleyConstraint_GetMaxLength(const JPH_PulleyConstraint* constraint);
JPH_CAPI float JPH_PulleyConstraint_GetCurrentLength(const JPH_PulleyConstraint* constraint);
JPH_CAPI float JPH_PulleyConstraint_GetTotalLambdaPosition(const JPH_PulleyConstraint* constraint);

/* RackAndPinionConstraint */
typedef struct JPH_RackAndPinionConstraintSettings {
	JPH_ConstraintSettings		base;    /* Inherits JPH_ConstraintSettings */

	JPH_ConstraintSpace			space;
	JPH_Vec3					hingeAxis;
	JPH_Vec3					sliderAxis;
	float						ratio;
} JPH_RackAndPinionConstraintSettings;

typedef struct JPH_RackAndPinionConstraint JPH_RackAndPinionConstraint; /* JPH::RackAndPinionConstraint */

JPH_CAPI void JPH_RackAndPinionConstraintSettings_Init(JPH_RackAndPinionConstraintSettings* settings);
JPH_CAPI JPH_RackAndPinionConstraint* JPH_RackAndPinionConstraint_Create(const JPH_RackAndPinionConstraintSettings* settings, JPH_Body* body1, JPH_Body* body2);
/* pinion must be a hinge and rack a slider constraint; the constraint takes a reference to each. */
JPH_CAPI void JPH_RackAndPinionConstraint_SetConstraints(JPH_RackAndPinionConstraint* constraint, const JPH_Constraint* pinion, const JPH_Constraint* rack);
JPH_CAPI float JPH_RackAndPinionConstraint_GetTotalLambda(const JPH_RackAndPinionConstraint* constraint);

/* SoftBodySharedSettings */
typedef enum JPH_SoftBodyLRAType {
	JPH_SoftBodyLRAType_None = 0,
	JPH_SoftBodyLRAType_EuclideanDistance = 1,
	JPH_SoftBodyLRAType_GeodesicDistance = 2,

	_JPH_SoftBodyLRAType_Force32 = 0x7fffffff
} JPH_SoftBodyLRAType;

/* JPH::SoftBodySharedSettings::VertexAttributes, copied field by field (not a layout mirror). */
typedef struct JPH_SoftBodyVertexAttributes {
	float					compliance;
	float					shearCompliance;
	float					bendCompliance;				/* FLT_MAX: no bend constraint */
	JPH_SoftBodyLRAType		lraType;
	float					lraMaxDistanceMultiplier;
} JPH_SoftBodyVertexAttributes;

/* Jolt's defaults: compliance 0, shear compliance 0, bend compliance FLT_MAX, no LRA, multiplier 1. */
JPH_CAPI void JPH_SoftBodyVertexAttributes_Init(JPH_SoftBodyVertexAttributes* attributes);
/* Like JPH_SoftBodySharedSettings_CreateConstraints, with every attribute field and per-vertex values:
   vertex v uses attributes[min(v, attributeCount - 1)], so attributeCount must be at least 1. Replaces
   the edge constraints already in the settings; Jolt's default angle tolerance (8 degrees). */
JPH_CAPI void JPH_SoftBodySharedSettings_CreateConstraints2(JPH_SoftBodySharedSettings* settings,
	const JPH_SoftBodyVertexAttributes* attributes, uint32_t attributeCount, JPH_SoftBodyBendType bendType);
JPH_CAPI void JPH_SoftBodySharedSettings_AddEdgeConstraint(JPH_SoftBodySharedSettings* settings, uint32_t vertex1, uint32_t vertex2, float compliance);
JPH_CAPI void JPH_SoftBodySharedSettings_AddDihedralBendConstraint(JPH_SoftBodySharedSettings* settings, uint32_t vertex1, uint32_t vertex2, uint32_t vertex3, uint32_t vertex4, float compliance);
JPH_CAPI void JPH_SoftBodySharedSettings_AddVolumeConstraint(JPH_SoftBodySharedSettings* settings, uint32_t vertex1, uint32_t vertex2, uint32_t vertex3, uint32_t vertex4, float compliance);
JPH_CAPI void JPH_SoftBodySharedSettings_CalculateEdgeLengths(JPH_SoftBodySharedSettings* settings);
JPH_CAPI void JPH_SoftBodySharedSettings_CalculateBendConstraintConstants(JPH_SoftBodySharedSettings* settings);
JPH_CAPI void JPH_SoftBodySharedSettings_CalculateVolumeConstraintVolumes(JPH_SoftBodySharedSettings* settings);
JPH_CAPI uint32_t JPH_SoftBodySharedSettings_GetEdgeConstraintCount(const JPH_SoftBodySharedSettings* settings);
JPH_CAPI uint32_t JPH_SoftBodySharedSettings_GetDihedralBendConstraintCount(const JPH_SoftBodySharedSettings* settings);
JPH_CAPI uint32_t JPH_SoftBodySharedSettings_GetVolumeConstraintCount(const JPH_SoftBodySharedSettings* settings);
JPH_CAPI uint32_t JPH_SoftBodySharedSettings_GetLRAConstraintCount(const JPH_SoftBodySharedSettings* settings);

/* Soft body Body */
/* Does nothing unless body is a soft body. Copies min(count, vertex count) vertices: positions in world
   space with Real precision, velocities in world space, inverse masses; any output may be null. */
JPH_CAPI void JPH_Body_GetSoftBodyVertices(const JPH_Body* body, JPH_RVec3* outPositions,
	JPH_Vec3* outVelocities, float* outInvMasses, uint32_t count);
/* Does nothing unless body is a soft body. Copies min(count, vertex count) vertex positions as Jolt
   stores them: relative to the centre of mass, in the body frame. */
JPH_CAPI void JPH_Body_GetSoftBodyVertexLocalPositions(const JPH_Body* body, JPH_Vec3* outPositions, uint32_t count);
/* Does nothing unless body is a soft body and index < vertex count; velocity in world space. */
JPH_CAPI void JPH_Body_SetSoftBodyVertexVelocity(JPH_Body* body, uint32_t index, const JPH_Vec3* velocity);
/* Does nothing unless body is a soft body and index < vertex count; then recomputes the body's mass and
   inertia (JPH::SoftBodyMotionProperties::CalculateMassAndInertia). */
JPH_CAPI void JPH_Body_SetSoftBodyVertexInvMass(JPH_Body* body, uint32_t index, float invMass);

#endif /* JOLT_C_EXT_H_ */
