//! Controlling a vehicle: the wheeled driver input and the settings every kind shares.

use std::marker::PhantomData;

use oxijolt_sys::*;

use super::settings::MAX_PITCH_ROLL_RULE;
use super::{install_tester, VehicleCollisionTester, VehicleEntry, VehicleKind, WheeledVehicle};
use crate::{limits, Vec3, VehicleError, VehicleRef};

/// What the driver asks of a wheeled vehicle or a motorcycle (Jolt
/// `WheeledVehicleController::SetDriverInput`).
///
/// The default is no input: no throttle, straight ahead, no brakes.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DriverInput {
    /// Throttle in `[-1, 1]`; negative drives backwards. Pressing against the direction of
    /// travel brakes first.
    pub forward: f32,
    /// Steering in `[-1, 1]`; 1 steers fully right, −1 fully left.
    pub right: f32,
    /// Brake in `[0, 1]`.
    pub brake: f32,
    /// Hand brake in `[0, 1]`.
    pub hand_brake: f32,
}

impl DriverInput {
    pub(super) fn validate(&self) -> Result<(), VehicleError> {
        let within = |value: f32, low: f32| value.is_finite() && (low..=1.0).contains(&value);
        let valid = within(self.forward, -1.0)
            && within(self.right, -1.0)
            && within(self.brake, 0.0)
            && within(self.hand_brake, 0.0);
        if valid {
            Ok(())
        } else {
            Err(VehicleError::InvalidValue(
                "forward and right must be in [-1, 1], brake and hand brake in [0, 1]",
            ))
        }
    }

    /// Hands this validated input to a wheeled controller.
    ///
    /// # Safety
    /// `controller` is a live `WheeledVehicleController` (a motorcycle's included) that nothing
    /// else uses during the call.
    pub(super) unsafe fn apply(&self, controller: *mut JPH_WheeledVehicleController) {
        // SAFETY: the controller is live and unshared (contract); the setter writes members.
        unsafe {
            JPH_WheeledVehicleController_SetDriverInput(
                controller,
                self.forward,
                self.right,
                self.brake,
                self.hand_brake,
            );
        }
    }

    /// The input a wheeled controller holds.
    ///
    /// # Safety
    /// `controller` is a live `WheeledVehicleController` (a motorcycle's included).
    pub(super) unsafe fn of(controller: *mut JPH_WheeledVehicleController) -> Self {
        // SAFETY: the controller is live (contract); the getters read members.
        unsafe {
            Self {
                forward: JPH_WheeledVehicleController_GetForwardInput(controller),
                right: JPH_WheeledVehicleController_GetRightInput(controller),
                brake: JPH_WheeledVehicleController_GetBrakeInput(controller),
                hand_brake: JPH_WheeledVehicleController_GetHandBrakeInput(controller),
            }
        }
    }
}

impl VehicleRef<'_, WheeledVehicle> {
    /// The driver input last set.
    pub fn driver_input(&self) -> DriverInput {
        // SAFETY: a wheeled vehicle's controller is a `WheeledVehicleController`, which derives
        // from `VehicleController` with single inheritance; the world borrowed here keeps it
        // alive.
        unsafe { DriverInput::of(self.controller().cast()) }
    }
}

/// Write access to one vehicle of kind `K`, borrowed mutably from its world.
///
/// Read the vehicle through [`PhysicsWorld::vehicle`](crate::PhysicsWorld::vehicle) once this
/// borrow ends.
pub struct VehicleMut<'w, K: VehicleKind = WheeledVehicle> {
    pub(super) entry: &'w mut VehicleEntry,
    pub(super) object_layer_count: u32,
    pub(super) kind: PhantomData<fn() -> K>,
}

impl<K: VehicleKind> VehicleMut<'_, K> {
    pub(super) fn ptr(&self) -> *mut JPH_VehicleConstraint {
        self.entry.constraint.as_ptr()
    }

    /// The vehicle's controller; the world, borrowed mutably through this view, owns the
    /// constraint, which owns the controller.
    pub(super) fn controller(&mut self) -> *mut JPH_VehicleController {
        // SAFETY: the world is borrowed mutably through this view and owns the constraint; the
        // getter returns a member.
        unsafe { JPH_VehicleConstraint_GetController(self.ptr()) }
    }

    /// Replaces the world's gravity for this vehicle by `gravity`, m/s² (Jolt
    /// `VehicleConstraint::OverrideGravity`). It must be finite and at most
    /// [`limits::MAX_ACCELERATION`] long. The chassis is a dynamic body, so its mass is at most
    /// [`limits::MAX_MASS`] and the force Jolt derives from the override stays finite.
    ///
    /// On every step while the chassis is awake, Jolt sets the chassis' gravity factor to 0 and
    /// adds the force `gravity / inverse mass` at its centre of mass; a sleeping chassis gets no
    /// force. The opposite of `gravity` also becomes the world up of the pitch and roll limit;
    /// a zero `gravity` keeps the last world up, which a restore does not undo (see
    /// [`WorldState`](crate::WorldState)).
    /// The override stays until it is set again; there is no reset, because Jolt's reset writes
    /// gravity factor 1 to the chassis. For radial gravity, set it every tick.
    /// Not part of [`WorldState`](crate::WorldState):
    /// [`PhysicsWorld::restore_state`](crate::PhysicsWorld::restore_state) does not undo it.
    pub fn set_gravity(&mut self, gravity: Vec3) -> Result<(), VehicleError> {
        if !limits::is_acceleration(gravity) {
            return Err(VehicleError::InvalidValue(limits::GRAVITY_RULE));
        }
        let gravity = gravity.to_jph();
        // SAFETY: the world is borrowed mutably through this view and owns the constraint;
        // `gravity` is a live local.
        unsafe { JPH_VehicleConstraint_OverrideGravity(self.ptr(), &gravity) };
        Ok(())
    }

    /// Sets the largest pitch and roll angle, radians in `[0, π]`; π turns the limit off. See
    /// [`VehicleSettings::max_pitch_roll_angle`](crate::VehicleSettings::max_pitch_roll_angle).
    /// Not part of [`WorldState`](crate::WorldState):
    /// [`PhysicsWorld::restore_state`](crate::PhysicsWorld::restore_state) does not undo it.
    pub fn set_max_pitch_roll_angle(&mut self, radians: f32) -> Result<(), VehicleError> {
        if !(radians.is_finite() && (0.0..=std::f32::consts::PI).contains(&radians)) {
            return Err(VehicleError::InvalidValue(MAX_PITCH_ROLL_RULE));
        }
        // SAFETY: as in `set_gravity`.
        unsafe { JPH_VehicleConstraint_SetMaxPitchRollAngle(self.ptr(), radians) };
        Ok(())
    }

    /// Replaces the collision tester, checked against this vehicle's wheels as at creation. The
    /// next step tests the wheels with it.
    /// Not part of [`WorldState`](crate::WorldState):
    /// [`PhysicsWorld::restore_state`](crate::PhysicsWorld::restore_state) does not undo it.
    pub fn set_collision_tester(
        &mut self,
        tester: VehicleCollisionTester,
    ) -> Result<(), VehicleError> {
        tester.validate(self.object_layer_count, self.entry.wheels.iter().copied())?;
        install_tester(self.entry, tester);
        Ok(())
    }
}

impl VehicleMut<'_, WheeledVehicle> {
    /// Sets what the driver asks for the next steps; see [`DriverInput`] for the ranges.
    pub fn set_driver_input(&mut self, input: DriverInput) -> Result<(), VehicleError> {
        input.validate()?;
        // SAFETY: a wheeled vehicle's controller is a `WheeledVehicleController`, which derives
        // from `VehicleController` with single inheritance; the world is borrowed mutably
        // through this view.
        unsafe { input.apply(self.controller().cast()) };
        Ok(())
    }
}
