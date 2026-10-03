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

/* HingeConstraint */
JPH_CAPI void JPH_HingeConstraint_SetTargetOrientationBS(JPH_HingeConstraint* constraint, const JPH_Quat* orientation);

#endif /* JOLT_C_EXT_H_ */
