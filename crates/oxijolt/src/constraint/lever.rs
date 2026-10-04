//! The lever-arm and spring bounds a constraint must meet before it is created.

use std::ptr::NonNull;

use oxijolt_sys::*;

use super::world::sealed;
use super::SpringSettings;
use crate::body::with_read_locked_body;
use crate::limits;
use crate::{BodyId, ConstraintError, PhysicsWorld, Quat, RVec3, Real, Vec3};

/// `Err(InvalidValue)` unless `spring` is valid and fits `bound`.
pub(crate) fn check_spring(spring: SpringSettings, bound: f64) -> Result<(), ConstraintError> {
    spring.validate().map_err(ConstraintError::InvalidValue)?;
    if spring.fits_effective_mass(bound) {
        Ok(())
    } else {
        Err(ConstraintError::InvalidValue(SPRING_BOUND_RULE))
    }
}

/// What every point a world constraint holds a dynamic body by must satisfy: a lever-arm
/// ratio of at most [`limits::MAX_LEVER_ARM_RATIO`].
const LEVER_ARM_RULE: &str = "lever-arm ratio must be at most limits::MAX_LEVER_ARM_RATIO";

/// The pose of a body and, for a dynamic one, its mass properties, as the lever-arm check of
/// [`PhysicsWorld::create_constraint`] reads them.
struct LeverState {
    /// Jolt's `CanBeKinematicOrDynamic`: whether the body has motion properties.
    can_move: bool,
    /// The inverse mass of a body that can move, 0 for one that cannot.
    inverse_mass: f32,
    position: RVec3,
    rotation: Quat,
    center_of_mass: RVec3,
    motion: Option<LeverMotion>,
}

/// The mass properties of a dynamic body, with the rotation from its principal frame to the
/// world.
#[derive(Clone, Copy)]
struct LeverMotion {
    inverse_mass: f32,
    inverse_inertia: Vec3,
    principal_to_world: Quat,
}

impl LeverState {
    /// Reads the state of `body`.
    ///
    /// # Safety
    /// `body` is a live body, locked for reading for the call.
    unsafe fn read(body: NonNull<JPH_Body>) -> Self {
        let body = body.as_ptr();
        let mut position = RVec3::ZERO.to_jph();
        let mut rotation = Quat::IDENTITY.to_jph();
        let mut center_of_mass = RVec3::ZERO.to_jph();
        // SAFETY: the caller's contract; the getters only read the body and write the live
        // locals.
        unsafe {
            JPH_Body_GetPosition(body, &mut position);
            JPH_Body_GetRotation(body, &mut rotation);
            JPH_Body_GetCenterOfMassPosition(body, &mut center_of_mass);
        }
        let rotation = Quat::from_jph(rotation);
        // SAFETY: as above; the getter reads whether the body has motion properties.
        let can_move = unsafe { JPH_Body_CanBeKinematicOrDynamic(body) };
        let inverse_mass = if can_move {
            // SAFETY: as above. A body that can move has motion properties, so the unchecked
            // getter reads a live member.
            unsafe {
                JPH_MotionProperties_GetInverseMassUnchecked(JPH_Body_GetMotionProperties(body))
            }
        } else {
            0.0
        };
        // SAFETY: as above. A dynamic body has motion properties, so the unchecked getters read
        // live members, and the getters write only the live locals.
        let motion = unsafe { JPH_Body_IsDynamic(body) }.then(|| unsafe {
            let properties = JPH_Body_GetMotionProperties(body);
            let mut inverse_inertia = Vec3::ZERO.to_jph();
            let mut inertia_rotation = Quat::IDENTITY.to_jph();
            JPH_MotionProperties_GetInverseInertiaDiagonal(properties, &mut inverse_inertia);
            JPH_MotionProperties_GetInertiaRotation(properties, &mut inertia_rotation);
            LeverMotion {
                inverse_mass: JPH_MotionProperties_GetInverseMassUnchecked(properties),
                inverse_inertia: Vec3::from_jph(inverse_inertia),
                principal_to_world: rotation.product(Quat::from_jph(inertia_rotation)),
            }
        });
        Self {
            can_move,
            inverse_mass,
            position: RVec3::from_jph(position),
            rotation,
            center_of_mass: RVec3::from_jph(center_of_mass),
            motion,
        }
    }

    /// The vector from the centre of mass to the world point `point`, in `f32` as Jolt keeps
    /// levers.
    fn lever_from_world(&self, point: RVec3) -> Vec3 {
        // `Real` is `f32` without the `double-precision` feature, so the casts are no-ops there.
        #[allow(clippy::unnecessary_cast)]
        Vec3::new(
            (point.x - self.center_of_mass.x) as f32,
            (point.y - self.center_of_mass.y) as f32,
            (point.z - self.center_of_mass.z) as f32,
        )
    }
}

/// The world point where Jolt puts an automatic point between `states` (`FixedConstraint.cpp`,
/// `SliderConstraint.cpp`): the centre of mass of one body when the other cannot move, and
/// otherwise the centres of mass weighted by their inverse masses, so the point lies near the
/// lighter body.
fn automatic_point([first, second]: &[LeverState; 2]) -> RVec3 {
    if !first.can_move {
        return second.center_of_mass;
    }
    if !second.can_move {
        return first.center_of_mass;
    }
    let (weight1, weight2) = (
        Real::from(first.inverse_mass),
        Real::from(second.inverse_mass),
    );
    let total = weight1 + weight2;
    if total == 0.0 {
        return first.center_of_mass;
    }
    let (c1, c2) = (first.center_of_mass, second.center_of_mass);
    RVec3::new(
        (weight1 * c1.x + weight2 * c2.x) / total,
        (weight1 * c1.y + weight2 * c2.y) / total,
        (weight1 * c1.z + weight2 * c2.z) / total,
    )
}

impl LeverMotion {
    /// The lever-arm ratio at `lever`, a world vector from the centre of mass.
    fn ratio_at(self, lever: Vec3) -> f64 {
        let principal = self.principal_to_world.conjugated().rotate(lever);
        limits::lever_arm_ratio(self.inverse_mass, self.inverse_inertia, principal)
    }

    /// The largest lever-arm ratio at any point within `distance` of the centre of mass.
    fn ratio_within(self, distance: f64) -> f64 {
        limits::lever_arm_ratio_within(self.inverse_mass, self.inverse_inertia, distance)
    }
}

/// What every frequency-mode spring of a world constraint must satisfy: the stiffness and
/// damping derived from the bodies' effective mass are at most
/// [`limits::MAX_SPRING_COEFFICIENT`].
const SPRING_BOUND_RULE: &str =
    "spring stiffness and damping must be at most limits::MAX_SPRING_COEFFICIENT";

impl PhysicsWorld {
    /// `Err(InvalidValue)` unless every point where `anchors` hold the dynamic ones of
    /// `bodies` has a lever-arm ratio of at most [`limits::MAX_LEVER_ARM_RATIO`].
    pub(super) fn check_lever_arms(
        &self,
        bodies: [BodyId; 2],
        anchors: [sealed::Anchor; 2],
    ) -> Result<(), ConstraintError> {
        let states = bodies.map(|id| {
            with_read_locked_body(self.body_lock_interface, id, |body| {
                // SAFETY: `body` is locked for reading for the duration of the closure.
                unsafe { LeverState::read(body) }
            })
            .unwrap_or_else(|| unreachable!("checked by the caller"))
        });
        for (state, anchor) in states.iter().zip(anchors) {
            let Some(motion) = state.motion else {
                continue;
            };
            // The lever from the centre of mass in world space, and how far around its end the
            // held points may lie.
            let (lever, radius) = match anchor {
                sealed::Anchor::World(point) => (state.lever_from_world(point), 0.0),
                sealed::Anchor::CenterOfMass(lever) => (state.rotation.rotate(lever), 0.0),
                sealed::Anchor::OnBody1 { offset, radius } => {
                    let body1 = &states[0];
                    let turned = body1.rotation.rotate(offset);
                    let point = RVec3::new(
                        body1.position.x + Real::from(turned.x),
                        body1.position.y + Real::from(turned.y),
                        body1.position.z + Real::from(turned.z),
                    );
                    (state.lever_from_world(point), radius)
                }
                sealed::Anchor::BetweenCentersOfMass => {
                    (state.lever_from_world(automatic_point(&states)), 0.0)
                }
            };
            let ratio = if radius == 0.0 {
                motion.ratio_at(lever)
            } else {
                motion.ratio_within(limits::f64_length(lever) + radius)
            };
            if !limits::is_lever_arm_ratio(ratio) {
                return Err(ConstraintError::InvalidValue(LEVER_ARM_RULE));
            }
        }
        Ok(())
    }

    /// An upper bound of the effective mass or inertia a constraint between `bodies` sees: over
    /// the dynamic bodies, the largest of each one's mass and largest principal moment of
    /// inertia; 0 without a dynamic body.
    ///
    /// The inverse effective mass of a translation part is at least the inverse mass of a
    /// dynamic body it connects, and that of a rotation part at least that body's smallest
    /// principal inverse inertia.
    pub(super) fn effective_mass_bound(&self, bodies: [BodyId; 2]) -> f64 {
        let bound_of = |id| {
            with_read_locked_body(self.body_lock_interface, id, |body| {
                // SAFETY: `body` is locked for reading for the duration of the closure. A
                // dynamic body has motion properties, so the unchecked getter reads a live
                // member; the getters only read, and `inverse_inertia` is a live local.
                unsafe {
                    if !JPH_Body_IsDynamic(body.as_ptr()) {
                        return 0.0;
                    }
                    let motion = JPH_Body_GetMotionProperties(body.as_ptr());
                    let mut inverse_inertia = crate::Vec3::ZERO.to_jph();
                    JPH_MotionProperties_GetInverseInertiaDiagonal(motion, &mut inverse_inertia);
                    let inverse_mass = JPH_MotionProperties_GetInverseMassUnchecked(motion);
                    let smallest = inverse_inertia
                        .x
                        .min(inverse_inertia.y)
                        .min(inverse_inertia.z);
                    // A zero inverse inertia (a locked rotation) would make the inertia
                    // unbounded; shapes give every body a finite one, so this is defensive.
                    let inertia = if smallest > 0.0 {
                        1.0 / f64::from(smallest)
                    } else {
                        f64::INFINITY
                    };
                    (1.0 / f64::from(inverse_mass)).max(inertia)
                }
            })
            .unwrap_or_else(|| unreachable!("checked by the caller"))
        };
        bodies.into_iter().map(bound_of).fold(0.0, f64::max)
    }
}
