//! Settings of a motorcycle: a two-wheeled vehicle with a lean controller.

use std::array::from_fn;

use oxijolt_sys::*;

use super::wheel::WheelBase;
use super::{BuiltSettings, ControllerGuard, VehicleSettings};
use crate::math::is_finite_non_negative;
use crate::owned::{JoltObject, Owned};
use crate::{Vec3, VehicleError};

/// The settings of a motorcycle (Jolt `VehicleConstraintSettings` with a
/// `MotorcycleControllerSettings`): a wheeled vehicle of exactly two wheels, front and rear
/// along the vehicle's forward, with a lean controller that tilts the chassis into turns. The
/// defaults are Jolt's.
///
/// The lean controller pushes the chassis' up toward a target lean, the direction of the ground
/// reaction on both wheels, through a spring and a damper about the chassis' forward axis. It
/// acts only while both wheels touch the ground with a positive suspension impulse; Jolt applies
/// no lean torque in the air, so a motorcycle does not right itself before landing.
#[derive(Clone, Debug, PartialEq)]
pub struct MotorcycleSettings {
    pub(super) vehicle: VehicleSettings,
    pub(super) max_lean_angle: f32,
    pub(crate) lean_spring_constant: f32,
    pub(crate) lean_spring_damping: f32,
    pub(super) lean_spring_integration_coefficient: f32,
    pub(super) lean_smoothing_factor: f32,
    pub(crate) lean_controller: bool,
    pub(crate) lean_steering_limit: bool,
}

impl MotorcycleSettings {
    /// Largest [`max_lean_angle`](Self::max_lean_angle), radians: 80°. Jolt limits the steering
    /// angle with the tangent of the lean angle, which grows without bound toward 90°. See
    /// [docs/limits.md#motorcycle-lean].
    ///
    /// [docs/limits.md#motorcycle-lean]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#motorcycle-lean
    pub const MAX_LEAN_ANGLE: f32 = 1.396_263_4;

    /// A motorcycle on the wheels, drivetrain and tester of `vehicle`, which must have exactly
    /// two wheels at different positions along its forward.
    pub fn new(vehicle: VehicleSettings) -> Self {
        Self {
            vehicle,
            max_lean_angle: 45.0_f32.to_radians(),
            lean_spring_constant: 5000.0,
            lean_spring_damping: 1000.0,
            lean_spring_integration_coefficient: 0.0,
            lean_smoothing_factor: 0.8,
            lean_controller: true,
            lean_steering_limit: true,
        }
    }

    /// How far the motorcycle may lean into a turn, radians in `[0, MAX_LEAN_ANGLE]`. With the
    /// lean steering limit on, it also limits the steering angle at speed. Default 45°.
    #[must_use]
    pub fn max_lean_angle(mut self, radians: f32) -> Self {
        self.max_lean_angle = radians;
        self
    }

    /// Spring constant of the lean controller, N·m per radian of lean error, finite and not
    /// negative. Default 5000. Together with the damping it must keep the chassis' angular
    /// acceleration within [`limits::MAX_ANGULAR_ACCELERATION`](crate::limits::MAX_ANGULAR_ACCELERATION),
    /// which [`PhysicsWorld::create_motorcycle`](crate::PhysicsWorld::create_motorcycle) checks
    /// against the chassis; see [docs/limits.md#motorcycle-lean].
    ///
    /// [docs/limits.md#motorcycle-lean]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#motorcycle-lean
    #[must_use]
    pub fn lean_spring_constant(mut self, value: f32) -> Self {
        self.lean_spring_constant = value;
        self
    }

    /// Damping of the lean controller, N·m·s per radian, finite and not negative. Default 1000.
    /// Checked with the spring constant.
    #[must_use]
    pub fn lean_spring_damping(mut self, value: f32) -> Self {
        self.lean_spring_damping = value;
        self
    }

    /// Integral term of the lean controller (Jolt `mLeanSpringIntegrationCoefficient`), which
    /// would make it a PID controller. Must be 0, Jolt's default: Jolt's `SaveState` does not save
    /// the integrated lean angle, so with another value a restored
    /// [`WorldState`](crate::WorldState) can replay different steps.
    /// [`PhysicsWorld::create_motorcycle`](crate::PhysicsWorld::create_motorcycle) refuses it
    /// with [`VehicleError::LeanSpringIntegrationNotSaved`].
    #[must_use]
    pub fn lean_spring_integration_coefficient(mut self, value: f32) -> Self {
        self.lean_spring_integration_coefficient = value;
        self
    }

    /// How much the target lean keeps of its last value each step, in `[0, 1]`: 0 follows the
    /// ground reaction at once, 1 never changes. It depends on the step length. Default 0.8.
    #[must_use]
    pub fn lean_smoothing_factor(mut self, value: f32) -> Self {
        self.lean_smoothing_factor = value;
        self
    }

    /// Whether the lean controller runs. Without it the motorcycle falls over unless something
    /// else holds it up. Default on. Fixed at creation: Jolt does not save this switch in its
    /// state.
    #[must_use]
    pub fn lean_controller(mut self, enabled: bool) -> Self {
        self.lean_controller = enabled;
        self
    }

    /// Whether the steering angle is limited by speed so the lean the turn needs stays within
    /// [`max_lean_angle`](Self::max_lean_angle). Jolt applies the limit to a wheel while the
    /// forward speed is above 1e-3 m/s and the wheel's steering axis tilts towards the vehicle's
    /// up (`cos` of the caster angle above 1e-6). In zero gravity (world or override) the limit
    /// is then 0: a moving motorcycle cannot steer, one at rest turns its front wheel fully.
    /// Default on. Fixed at creation: Jolt does not save this switch in its state.
    #[must_use]
    pub fn lean_steering_limit(mut self, enabled: bool) -> Self {
        self.lean_steering_limit = enabled;
        self
    }

    /// Checks the vehicle settings and the motorcycle's own rules. The lean spring's bound
    /// depends on the chassis and is checked when the motorcycle is created.
    pub(crate) fn validate(&self, object_layer_count: u32) -> Result<(), VehicleError> {
        let invalid = |what| Err(VehicleError::InvalidValue(what));
        self.vehicle.validate(object_layer_count)?;
        if self.vehicle.wheels.len() != 2 {
            return invalid("a motorcycle needs exactly two wheels");
        }
        if !self.has_wheel_base() {
            return invalid("a motorcycle's wheels must be apart along its forward");
        }
        if !(is_finite_non_negative(self.max_lean_angle)
            && self.max_lean_angle <= Self::MAX_LEAN_ANGLE)
        {
            return invalid(
                "max lean angle must be between 0 and MotorcycleSettings::MAX_LEAN_ANGLE",
            );
        }
        if !(is_finite_non_negative(self.lean_spring_constant)
            && is_finite_non_negative(self.lean_spring_damping))
        {
            return invalid("lean spring constant and damping must be finite and not negative");
        }
        if !(self.lean_smoothing_factor.is_finite()
            && (0.0..=1.0).contains(&self.lean_smoothing_factor))
        {
            return invalid("lean smoothing factor must be between 0 and 1");
        }
        if self.lean_spring_integration_coefficient != 0.0 {
            return Err(VehicleError::LeanSpringIntegrationNotSaved);
        }
        Ok(())
    }

    /// Whether the two validated wheels lie at different positions along the vehicle's forward,
    /// in every order [`along_forward`] evaluates.
    fn has_wheel_base(&self) -> bool {
        let forward = self.vehicle.frame.forward;
        let front = along_forward(&self.vehicle.wheels[0].base, forward);
        let rear = along_forward(&self.vehicle.wheels[1].base, forward);
        front.iter().zip(&rear).all(|(a, b)| a != b)
    }

    /// The joltc objects of these validated settings.
    pub(crate) fn build(&self) -> BuiltSettings {
        self.vehicle
            .build_with(ControllerGuard::Motorcycle(self.create_controller()))
    }

    /// The joltc controller settings: the wheeled vehicle's drivetrain and the lean controller.
    pub(super) fn create_controller(&self) -> Owned<JPH_MotorcycleControllerSettings> {
        // SAFETY: Jolt is initialised (a world exists). The settings are returned holding one
        // reference, which the guard takes over.
        let controller = unsafe { Owned::from_raw(JPH_MotorcycleControllerSettings_Create()) }
            .unwrap_or_else(|| unreachable!("joltc `new`s the settings"));
        let ptr = controller.as_ptr();
        // SAFETY: the guard owns the new settings, which nothing else uses yet; motorcycle
        // controller settings derive from wheeled controller settings with single inheritance,
        // joltc's cast convention. The setters write members. All values were validated.
        unsafe {
            self.vehicle.fill_wheeled_controller(ptr.cast());
            JPH_MotorcycleControllerSettings_SetMaxLeanAngle(ptr, self.max_lean_angle);
            JPH_MotorcycleControllerSettings_SetLeanSpringConstant(ptr, self.lean_spring_constant);
            JPH_MotorcycleControllerSettings_SetLeanSpringDamping(ptr, self.lean_spring_damping);
            JPH_MotorcycleControllerSettings_SetLeanSpringIntegrationCoefficient(
                ptr,
                self.lean_spring_integration_coefficient,
            );
            JPH_MotorcycleControllerSettings_SetLeanSmoothingFactor(
                ptr,
                self.lean_smoothing_factor,
            );
        }
        controller
    }
}

/// A wheel's position along `forward` as `MotorcycleController::GetWheelBase` measures it: the
/// suspension force point, or the fully extended suspension, dotted with the local forward.
/// Evaluated in `f32` with plain and with fused products, the orders Jolt's compiled code may
/// use, and in `f64`.
fn along_forward(wheel: &WheelBase, forward: Vec3) -> [f64; 3] {
    let f = <[f32; 3]>::from(forward);
    let p = <[f32; 3]>::from(wheel.position);
    let d = <[f32; 3]>::from(wheel.suspension_direction);
    let length = wheel.suspension_max_length;
    let (plain, fused, exact): ([f32; 3], [f32; 3], [f64; 3]) = match wheel.suspension_force_point {
        Some(point) => {
            let point = <[f32; 3]>::from(point);
            (point, point, point.map(f64::from))
        }
        None => (
            from_fn(|i| p[i] + d[i] * length),
            from_fn(|i| d[i].mul_add(length, p[i])),
            from_fn(|i| f64::from(p[i]) + f64::from(d[i]) * f64::from(length)),
        ),
    };
    [
        f64::from(plain[0] * f[0] + plain[1] * f[1] + plain[2] * f[2]),
        f64::from(fused[2].mul_add(f[2], fused[1].mul_add(f[1], fused[0] * f[0]))),
        (0..3).map(|i| exact[i] * f64::from(f[i])).sum(),
    ]
}

/// Motorcycle controller settings, of which the owner holds the one reference
/// `JPH_MotorcycleControllerSettings_Create` returns. A vehicle copies them into its own
/// controller when it is created.
impl JoltObject for JPH_MotorcycleControllerSettings {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner holds one reference (trait contract), released here once; the
        // motorcycle settings derive from `VehicleControllerSettings` with single inheritance.
        unsafe { JPH_VehicleControllerSettings_Destroy(ptr.cast()) };
    }
}

#[cfg(test)]
#[path = "motorcycle_tests.rs"]
mod tests;
