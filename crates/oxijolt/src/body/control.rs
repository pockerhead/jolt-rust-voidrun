//! Momentary inputs to one body: impulses.

use oxijolt_sys::*;

use super::load::{point_torque, read_load, require, Load, LoadState};
use super::with_read_locked_body;
use crate::limits::{
    is_angular_velocity_change, is_in_frame, is_velocity_change, ANGULAR_VELOCITY_CHANGE_RULE,
    VELOCITY_CHANGE_RULE,
};
use crate::{BodyError, BodyMut, RVec3, Vec3};

/// `v` in `f64`.
fn wide(v: Vec3) -> [f64; 3] {
    [v.x, v.y, v.z].map(f64::from)
}

/// The largest component of `v`.
fn largest(v: Vec3) -> f32 {
    v.x.max(v.y).max(v.z)
}

impl BodyMut<'_> {
    /// Adds an impulse in N·s at the centre of mass: the linear velocity changes at once by
    /// `impulse / mass`, and the body wakes.
    ///
    /// The impulse must be finite, and on a dynamic body the velocity change `|impulse| / mass`
    /// at most [`limits::MAX_VELOCITY_CHANGE`]; otherwise [`BodyError::InvalidValue`] is
    /// returned and nothing changes. Jolt then clamps the new velocity to
    /// [`limits::MAX_LINEAR_VELOCITY`] and zeroes its locked axes. Static and kinematic bodies
    /// ignore impulses, as they ignore forces. Fails with [`BodyError::SoftBody`] for a soft
    /// body.
    ///
    /// [`limits::MAX_VELOCITY_CHANGE`]: crate::limits::MAX_VELOCITY_CHANGE
    /// [`limits::MAX_LINEAR_VELOCITY`]: crate::limits::MAX_LINEAR_VELOCITY
    pub fn add_impulse(&mut self, impulse: Vec3) -> Result<(), BodyError> {
        self.reject_soft_body()?;
        require(impulse.is_finite(), "impulse must be finite")?;
        if let Some(state) = self.dynamic_state()? {
            require(
                is_velocity_change(wide(impulse), state.inverse_mass),
                VELOCITY_CHANGE_RULE,
            )?;
        }
        let mut impulse = impulse.to_jph();
        // SAFETY: the world is borrowed mutably through this view and holds the body; this
        // thread holds no body lock, and joltc only reads `impulse`, a live local.
        unsafe { JPH_BodyInterface_AddImpulse(self.interface(), self.id.raw, &mut impulse) };
        Ok(())
    }

    /// Adds an angular impulse in N·m·s: the angular velocity changes at once by the world
    /// inverse inertia times `angular_impulse`, and the body wakes.
    ///
    /// The impulse must be finite, and on a dynamic body `|angular_impulse|` times the largest
    /// principal inverse inertia at most [`limits::MAX_ANGULAR_VELOCITY_CHANGE`]; otherwise
    /// [`BodyError::InvalidValue`] is returned and nothing changes. Jolt clamps the new angular
    /// velocity to [`limits::MAX_ANGULAR_VELOCITY`]. Static and kinematic bodies ignore it;
    /// fails with [`BodyError::SoftBody`] for a soft body.
    ///
    /// [`limits::MAX_ANGULAR_VELOCITY_CHANGE`]: crate::limits::MAX_ANGULAR_VELOCITY_CHANGE
    /// [`limits::MAX_ANGULAR_VELOCITY`]: crate::limits::MAX_ANGULAR_VELOCITY
    pub fn add_angular_impulse(&mut self, angular_impulse: Vec3) -> Result<(), BodyError> {
        self.reject_soft_body()?;
        require(
            angular_impulse.is_finite(),
            "angular impulse must be finite",
        )?;
        if let Some(state) = self.dynamic_state()? {
            require(
                is_angular_velocity_change(wide(angular_impulse), largest(state.inverse_inertia)),
                ANGULAR_VELOCITY_CHANGE_RULE,
            )?;
        }
        let mut angular_impulse = angular_impulse.to_jph();
        // SAFETY: as in `add_impulse`.
        unsafe {
            JPH_BodyInterface_AddAngularImpulse(self.interface(), self.id.raw, &mut angular_impulse)
        };
        Ok(())
    }

    /// Adds an impulse in N·s at a world-space `point`, which also adds the angular impulse
    /// `(point - centre_of_mass) × impulse`, and wakes the body.
    ///
    /// The impulse must be finite and the point within [`limits::MAX_POSITION`]. On a dynamic
    /// body both rules of [`add_impulse`](Self::add_impulse) and
    /// [`add_angular_impulse`](Self::add_angular_impulse) apply, and Jolt's `f32` cross product
    /// must not overflow; otherwise [`BodyError::InvalidValue`] is returned and nothing changes.
    /// Static and kinematic bodies ignore it; fails with [`BodyError::SoftBody`] for a soft body.
    ///
    /// [`limits::MAX_POSITION`]: crate::limits::MAX_POSITION
    pub fn add_impulse_at_point(&mut self, impulse: Vec3, point: RVec3) -> Result<(), BodyError> {
        self.reject_soft_body()?;
        require(impulse.is_finite(), "impulse must be finite")?;
        require(
            is_in_frame(point),
            "point must be finite and within limits::MAX_POSITION",
        )?;
        if let Some(state) = self.dynamic_state()? {
            require(
                is_velocity_change(wide(impulse), state.inverse_mass),
                VELOCITY_CHANGE_RULE,
            )?;
            let angular_impulse = point_torque(impulse, point, state.center_of_mass)?;
            require(
                is_angular_velocity_change(angular_impulse, largest(state.inverse_inertia)),
                ANGULAR_VELOCITY_CHANGE_RULE,
            )?;
        }
        let mut impulse = impulse.to_jph();
        let mut point = point.to_jph();
        // SAFETY: as in `add_impulse`; joltc only reads the two live locals.
        unsafe {
            JPH_BodyInterface_AddImpulse2(self.interface(), self.id.raw, &mut impulse, &mut point)
        };
        Ok(())
    }

    /// What bounds an impulse on this body when it is a dynamic rigid body; `None` for static
    /// and kinematic bodies, which Jolt's impulse calls skip. Reads under a read lock that is
    /// released before the caller calls the locking body interface.
    fn dynamic_state(&self) -> Result<Option<LoadState>, BodyError> {
        let load = with_read_locked_body(self.inner.body_lock_interface, self.id, |body| {
            // SAFETY: `body` is locked for reading for the duration of the closure.
            unsafe { read_load(body) }
        })
        .ok_or(BodyError::NotFound(self.id))?;
        Ok(match load {
            Load::Rigid(state) => Some(state),
            Load::Ignored | Load::Soft(_) => None,
        })
    }
}
