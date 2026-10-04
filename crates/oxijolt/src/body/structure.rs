//! Structural changes of one body: its motion type, and the rules a body's shape and mass meet
//! at creation and after a change.

use oxijolt_sys::*;

use super::load::require;
use super::{
    has_finite_inverse, mass_properties, with_read_locked_body, Activation, BodyMut, MotionType,
    INERTIA_RULE, KINEMATIC_MESH_MASS_RULE, MESH_DYNAMIC_RULE, SENSOR_SHAPE_RULE,
    STATIC_SHAPE_RULE,
};
use crate::limits::{is_mass, is_vertex_inverse_mass, MASS_RULE};
use crate::shape::static_only_leaves_are_meshes_of;
use crate::{BodyError, Shape};

/// How a body uses a shape: what decides which shapes and masses it may take.
#[derive(Clone, Copy, Debug)]
pub(crate) struct ShapeUse {
    pub(crate) motion_type: MotionType,
    /// Whether the body has motion properties: it is not static, or it was created able to
    /// become kinematic or dynamic.
    pub(crate) can_move: bool,
    pub(crate) sensor: bool,
    /// The mass the body is given, or `None` for the shape's own.
    pub(crate) mass: Option<f32>,
}

/// Checks that a body used as `using` describes may take `shape`, and returns the mass
/// properties Jolt gives it when it has motion properties (Jolt itself checks none of this).
///
/// A sensor may not use a shape that only static bodies may use (`Shape::MustBeStatic`). A body
/// that can move may use one only when every such leaf is a mesh, given a mass, and only when it
/// is not dynamic. Jolt cannot collide a mesh with a mesh or a heightfield ("Unsupported shape
/// pair" in `CollisionDispatch`), and such pairs stay out of reach: only bodies that are not
/// dynamic carry meshes, and Jolt pairs a kinematic body with a static or kinematic one only
/// with `mCollideKinematicVsNonDynamic`, which this crate does not expose, or with a sensor
/// (`Body::sFindCollidingPairsCanCollide`), which takes no static-only shape; query, character
/// and ragdoll shapes refuse static-only shapes. Exposing that switch, or letting sensors take
/// such shapes, must revisit this rule.
///
/// A body that can move also needs mass properties with finite inverses
/// ([`has_finite_inverse`]), and a dynamic one a mass within `MIN_MASS..=MAX_MASS`.
pub(crate) fn check_shape_for(
    shape: &Shape,
    using: ShapeUse,
) -> Result<Option<JPH_MassProperties>, BodyError> {
    let static_only = shape.must_be_static();
    require(!(using.sensor && static_only), SENSOR_SHAPE_RULE)?;
    if !using.can_move {
        return Ok(None);
    }
    if static_only {
        require(shape.static_only_leaves_are_meshes(), STATIC_SHAPE_RULE)?;
        require(using.motion_type != MotionType::Dynamic, MESH_DYNAMIC_RULE)?;
        require(using.mass.is_some(), KINEMATIC_MESH_MASS_RULE)?;
    }
    let properties = mass_properties(shape, using.mass);
    require(has_finite_inverse(&properties), INERTIA_RULE)?;
    if using.motion_type == MotionType::Dynamic {
        require(is_mass(properties.mass), MASS_RULE)?;
    }
    Ok(Some(properties))
}

/// Why a body cannot become dynamic or kinematic: its shape or mass.
enum MotionBlock {
    StaticShape,
    Mass,
}

impl BodyMut<'_> {
    /// Makes the body static, kinematic or dynamic (Jolt `BodyInterface::SetMotionType`).
    ///
    /// To static, Jolt puts an awake body to sleep ([`ActivationEvent::Deactivated`]) and zeroes
    /// its velocities; to kinematic or static, it drops the forces added since the last step;
    /// to kinematic or dynamic with [`Activation::Activate`], a sleeping body wakes
    /// ([`ActivationEvent::Activated`]). The body keeps its object layer, shape, velocities
    /// (unless made static) and degrees of freedom. The same motion type changes nothing.
    ///
    /// Jolt does not save motion types, so a successful change makes every earlier
    /// [`WorldState`](crate::WorldState) unrestorable ([`StateError::WorldChanged`]).
    ///
    /// Fails, changing nothing, with:
    /// - [`BodyError::OwnedByCharacter`], [`BodyError::UsedByVehicle`],
    ///   [`BodyError::OwnedByRagdoll`] or [`BodyError::UsedByConstraint`] for a body that a
    ///   character, vehicle, ragdoll or constraint holds, whose checks assume the motion type it
    ///   had, also when the motion type would not change;
    /// - [`BodyError::SoftBody`] for a soft body, which Jolt keeps dynamic;
    /// - [`BodyError::CannotMove`] for kinematic or dynamic on a body created static without
    ///   [`BodySettings::allow_dynamic_or_kinematic`];
    /// - [`BodyError::InvalidValue`] for dynamic when the shape contains a mesh or heightfield,
    ///   or the mass is outside `MIN_MASS..=MAX_MASS` (a kinematic body's mass is not bounded
    ///   at creation), and for kinematic when the shape contains a static-only leaf that is not
    ///   a mesh.
    ///
    /// [`ActivationEvent::Deactivated`]: crate::ActivationEvent::Deactivated
    /// [`ActivationEvent::Activated`]: crate::ActivationEvent::Activated
    /// [`StateError::WorldChanged`]: crate::StateError::WorldChanged
    /// [`BodySettings::allow_dynamic_or_kinematic`]: crate::BodySettings::allow_dynamic_or_kinematic
    pub fn set_motion_type(
        &mut self,
        motion_type: MotionType,
        activation: Activation,
    ) -> Result<(), BodyError> {
        let id = self.inner.id;
        self.world.check_not_owned(id)?;
        self.reject_soft_body()?;
        if self.motion_type() == motion_type {
            return Ok(());
        }
        if motion_type != MotionType::Static {
            if !self.can_be_kinematic_or_dynamic() {
                return Err(BodyError::CannotMove(id));
            }
            match self.motion_block(motion_type) {
                Some(MotionBlock::StaticShape) => {
                    let rule = if motion_type == MotionType::Dynamic {
                        MESH_DYNAMIC_RULE
                    } else {
                        STATIC_SHAPE_RULE
                    };
                    return Err(BodyError::InvalidValue(rule));
                }
                Some(MotionBlock::Mass) => return Err(BodyError::InvalidValue(MASS_RULE)),
                None => {}
            }
        }
        // Jolt does not save motion types.
        self.world.note_structure_change();
        // SAFETY: the world is borrowed mutably through this view and holds the body, a rigid
        // body; a kinematic or dynamic target has motion properties (checked above), as Jolt
        // asserts. Jolt deactivates the body itself before making it static. This thread holds
        // no body lock.
        unsafe {
            JPH_BodyInterface_SetMotionType(
                self.interface(),
                id.raw,
                motion_type.to_jph(),
                activation.to_jph(),
            )
        };
        Ok(())
    }

    /// What keeps the body, which has motion properties, from becoming `motion_type`
    /// (kinematic or dynamic): a static-only leaf that is not a mesh, or for dynamic any
    /// static-only shape or a mass outside the dynamic range.
    fn motion_block(&self, motion_type: MotionType) -> Option<MotionBlock> {
        with_read_locked_body(self.inner.body_lock_interface, self.inner.id, |body| {
            let body = body.as_ptr();
            // SAFETY: `body` is locked for reading for the duration of the closure, which keeps
            // its shape alive. It has motion properties (the caller checked
            // `CanBeKinematicOrDynamic`), so the unchecked getter reads a live member without
            // the assertion of the checked getter on static bodies; the getters only read.
            unsafe {
                let shape = JPH_Body_GetShape(body);
                if JPH_Shape_MustBeStatic(shape) {
                    let meshes_only = static_only_leaves_are_meshes_of(shape);
                    if motion_type == MotionType::Dynamic || !meshes_only {
                        return Some(MotionBlock::StaticShape);
                    }
                }
                let properties = JPH_Body_GetMotionPropertiesUnchecked(body);
                let inverse_mass = JPH_MotionProperties_GetInverseMassUnchecked(properties);
                (motion_type == MotionType::Dynamic && !is_vertex_inverse_mass(inverse_mass))
                    .then_some(MotionBlock::Mass)
            }
        })
        .flatten()
    }
}
