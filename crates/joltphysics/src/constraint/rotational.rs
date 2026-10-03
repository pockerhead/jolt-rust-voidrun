//! World constraints that limit or drive a rotation: hinge.

use std::f32::consts::PI;
use std::ptr::NonNull;

use joltphysics_sys::*;

use super::world::{check_spring, sealed, ConstraintSettings};
use super::{non_negative, validate_hinge_limits, within, MotorSettings, SpringSettings};
use crate::limits;
use crate::{
    ConstraintError, ConstraintMut, ConstraintRef, HingeConstraint, HingeConstraintSettings,
    MotorState, Vec3,
};

/// `Err(InvalidValue)` unless `motor` is valid and its spring fits `bound`.
pub(crate) fn check_motor(motor: MotorSettings, bound: f64) -> Result<(), ConstraintError> {
    motor.validate().map_err(ConstraintError::InvalidValue)?;
    check_spring(motor.spring, bound)
}

/// `Err(InvalidValue)` unless `value` is a finite friction force or torque, at least 0.
pub(crate) fn check_friction(value: f32) -> Result<(), ConstraintError> {
    if non_negative(value) {
        Ok(())
    } else {
        Err(ConstraintError::InvalidValue(
            "friction must be finite and not negative",
        ))
    }
}

/// `Err(InvalidValue)` unless `value` is an angular velocity Jolt accepts, in rad/s.
pub(crate) fn check_angular_speed(value: f32) -> Result<(), ConstraintError> {
    if within(
        value,
        -limits::MAX_ANGULAR_VELOCITY,
        limits::MAX_ANGULAR_VELOCITY,
    ) {
        Ok(())
    } else {
        Err(ConstraintError::InvalidValue(
            "target angular velocity must be finite and at most limits::MAX_ANGULAR_VELOCITY",
        ))
    }
}

impl sealed::Settings for HingeConstraintSettings {
    fn validate(&self) -> Result<(), &'static str> {
        HingeConstraintSettings::validate(self)
    }

    fn springs(&self) -> Vec<SpringSettings> {
        HingeConstraintSettings::springs(self).collect()
    }

    unsafe fn create(
        &self,
        body1: NonNull<JPH_Body>,
        body2: NonNull<JPH_Body>,
    ) -> *mut JPH_Constraint {
        let settings = self.to_jph();
        // SAFETY: the caller locks both live bodies (trait contract); `settings` is a live,
        // validated local that joltc converts.
        unsafe { JPH_HingeConstraint_Create(&settings, body1.as_ptr(), body2.as_ptr()) }.cast()
    }
}

impl ConstraintSettings for HingeConstraintSettings {
    type Kind = HingeConstraint;
}

impl ConstraintRef<'_, HingeConstraint> {
    /// The angle of body 2 about the hinge axis, radians in `[-π, π]`, measured from the pose
    /// where both normal axes coincide.
    pub fn current_angle(&self) -> f32 {
        // SAFETY: the world borrowed here owns the constraint. Jolt's `GetCurrentAngle` is const
        // and reads the constraint's axes and the bodies' rotations, which change only behind
        // `&mut PhysicsWorld`; joltc only declares the handle mutable.
        unsafe { JPH_HingeConstraint_GetCurrentAngle(self.ptr()) }
    }

    /// The motor's state.
    pub fn motor_state(&self) -> MotorState {
        // SAFETY: as in `current_angle`; the getter reads a member.
        MotorState::from_jph(unsafe { JPH_HingeConstraint_GetMotorState(self.ptr()) })
    }

    /// The angle a position motor drives to, radians.
    pub fn target_angle(&self) -> f32 {
        // SAFETY: as in `motor_state`.
        unsafe { JPH_HingeConstraint_GetTargetAngle(self.ptr()) }
    }

    /// The angular velocity a velocity motor drives to, rad/s.
    pub fn target_angular_velocity(&self) -> f32 {
        // SAFETY: as in `motor_state`.
        unsafe { JPH_HingeConstraint_GetTargetAngularVelocity(self.ptr()) }
    }

    /// The angle limits, radians: `(min, max)`; `(-π, π)` means no limits.
    pub fn limits(&self) -> (f32, f32) {
        // SAFETY: as in `motor_state`.
        unsafe {
            (
                JPH_HingeConstraint_GetLimitsMin(self.ptr()),
                JPH_HingeConstraint_GetLimitsMax(self.ptr()),
            )
        }
    }

    /// The friction torque in N·m applied while the motor is off.
    pub fn max_friction_torque(&self) -> f32 {
        // SAFETY: as in `motor_state`.
        unsafe { JPH_HingeConstraint_GetMaxFrictionTorque(self.ptr()) }
    }

    /// The impulse in N·s that kept the hinge points together in the last step.
    pub fn total_lambda_position(&self) -> Vec3 {
        let mut value = Vec3::ZERO.to_jph();
        // SAFETY: as in `motor_state`; `value` is a live local.
        unsafe { JPH_HingeConstraint_GetTotalLambdaPosition(self.ptr(), &mut value) };
        Vec3::from_jph(value)
    }

    /// The angular impulse in N·m·s the motor applied in the last step.
    pub fn total_lambda_motor(&self) -> f32 {
        // SAFETY: as in `motor_state`.
        unsafe { JPH_HingeConstraint_GetTotalLambdaMotor(self.ptr()) }
    }

    /// The angular impulse in N·m·s the limits applied in the last step.
    pub fn total_lambda_rotation_limits(&self) -> f32 {
        // SAFETY: as in `motor_state`.
        unsafe { JPH_HingeConstraint_GetTotalLambdaRotationLimits(self.ptr()) }
    }
}

impl ConstraintMut<'_, HingeConstraint> {
    /// Switches the motor and wakes the constraint's bodies.
    pub fn set_motor_state(&mut self, state: MotorState) {
        // SAFETY: the world is borrowed mutably through this view and owns the constraint; no
        // step runs. Jolt asserts valid motor settings for a running motor: every setting stored
        // was validated.
        unsafe { JPH_HingeConstraint_SetMotorState(self.ptr(), state.to_jph()) };
        self.wake_bodies();
    }

    /// Sets the angle a position motor drives to, radians in `[-π, π]`; Jolt clamps it to the
    /// limits. Wakes the constraint's bodies.
    pub fn set_target_angle(&mut self, angle: f32) -> Result<(), ConstraintError> {
        if !within(angle, -PI, PI) {
            return Err(ConstraintError::InvalidValue(
                "target angle must be between -pi and pi",
            ));
        }
        // SAFETY: as in `set_motor_state`.
        unsafe { JPH_HingeConstraint_SetTargetAngle(self.ptr(), angle) };
        self.wake_bodies();
        Ok(())
    }

    /// Sets the angular velocity a velocity motor drives to, rad/s, at most
    /// [`limits::MAX_ANGULAR_VELOCITY`] in magnitude. Wakes the constraint's bodies.
    pub fn set_target_angular_velocity(&mut self, velocity: f32) -> Result<(), ConstraintError> {
        check_angular_speed(velocity)?;
        // SAFETY: as in `set_motor_state`.
        unsafe { JPH_HingeConstraint_SetTargetAngularVelocity(self.ptr(), velocity) };
        self.wake_bodies();
        Ok(())
    }

    /// Replaces the motor settings, checked as at creation and bounded through the bodies'
    /// effective mass.
    pub fn set_motor_settings(&mut self, motor: MotorSettings) -> Result<(), ConstraintError> {
        check_motor(motor, self.effective_mass_bound())?;
        let mut motor = motor.to_jph();
        // SAFETY: as in `set_motor_state`; `motor` is a live local that joltc copies.
        unsafe { JPH_HingeConstraint_SetMotorSettings(self.ptr(), &mut motor) };
        Ok(())
    }

    /// Sets the angle limits: `min` in `[-π, 0]`, `max` in `[0, π]`; `min == max` needs a soft
    /// limits spring. `(-π, π)` turns the limits off.
    pub fn set_limits(&mut self, min: f32, max: f32) -> Result<(), ConstraintError> {
        validate_hinge_limits(min, max, self.limits_spring())
            .map_err(ConstraintError::InvalidValue)?;
        // SAFETY: as in `set_motor_state`; Jolt asserts the ranges checked above.
        unsafe { JPH_HingeConstraint_SetLimits(self.ptr(), min, max) };
        Ok(())
    }

    /// Replaces the spring that makes the limits soft, bounded through the bodies' effective
    /// mass. Limits with `min == max` keep needing a soft spring.
    pub fn set_limits_spring(&mut self, spring: SpringSettings) -> Result<(), ConstraintError> {
        check_spring(spring, self.effective_mass_bound())?;
        // SAFETY: as in `set_motor_state`.
        let (min, max) = unsafe {
            (
                JPH_HingeConstraint_GetLimitsMin(self.ptr()),
                JPH_HingeConstraint_GetLimitsMax(self.ptr()),
            )
        };
        validate_hinge_limits(min, max, spring).map_err(ConstraintError::InvalidValue)?;
        let mut spring = spring.to_jph();
        // SAFETY: as in `set_motor_state`; `spring` is a live local that joltc copies.
        unsafe { JPH_HingeConstraint_SetLimitsSpringSettings(self.ptr(), &mut spring) };
        Ok(())
    }

    /// Sets the friction torque in N·m applied while the motor is off, finite and at least 0.
    pub fn set_max_friction_torque(&mut self, torque: f32) -> Result<(), ConstraintError> {
        check_friction(torque)?;
        // SAFETY: as in `set_motor_state`.
        unsafe { JPH_HingeConstraint_SetMaxFrictionTorque(self.ptr(), torque) };
        Ok(())
    }

    /// The current limits spring.
    fn limits_spring(&self) -> SpringSettings {
        // SAFETY: an all-zero `JPH_SpringSettings` is valid; the getter fills it from a member.
        let mut spring: JPH_SpringSettings = unsafe { std::mem::zeroed() };
        // SAFETY: as in `set_motor_state`; `spring` is a live local.
        unsafe { JPH_HingeConstraint_GetLimitsSpringSettings(self.ptr(), &mut spring) };
        SpringSettings::from_jph(spring)
    }
}
