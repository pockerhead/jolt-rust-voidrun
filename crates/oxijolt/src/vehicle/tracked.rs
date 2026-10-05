//! Tracked vehicles: Jolt's `VehicleConstraint` with the tracked controller.

use std::ops::Range;

use oxijolt_sys::*;

use super::{TrackedVehicle, TrackedVehicleSettings, VehicleId};
use crate::{limits, BodyId, PhysicsWorld, VehicleError, VehicleMut, VehicleRef};

/// What the driver asks of a tracked vehicle (Jolt `TrackedVehicleController::SetDriverInput`).
///
/// The tracks turn at `left_ratio` and `right_ratio` times the drivetrain's speed. Equal ratios
/// drive straight; ratios of opposite signs turn the vehicle in place.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TrackedDriverInput {
    /// Throttle in `[-1, 1]`; negative drives backwards.
    pub forward: f32,
    /// Speed ratio of the left track, its magnitude within `1 / limits::MAX_RATIO..=1`.
    pub left_ratio: f32,
    /// Speed ratio of the right track, its magnitude within `1 / limits::MAX_RATIO..=1`.
    pub right_ratio: f32,
    /// Brake in `[0, 1]`.
    pub brake: f32,
}

impl Default for TrackedDriverInput {
    /// No throttle, both tracks at full ratio (straight ahead), no brake.
    fn default() -> Self {
        Self {
            forward: 0.0,
            left_ratio: 1.0,
            right_ratio: 1.0,
            brake: 0.0,
        }
    }
}

impl TrackedDriverInput {
    fn validate(&self) -> Result<(), VehicleError> {
        let within = |value: f32, low: f32| value.is_finite() && (low..=1.0).contains(&value);
        // A floor on the ratios keeps their product and Jolt's synchronisation of the two tracks
        // away from underflow; see docs/limits.md#track-ratios.
        let ratio = |value: f32| limits::is_ratio(value) && value.abs() <= 1.0;
        if within(self.forward, -1.0)
            && within(self.brake, 0.0)
            && ratio(self.left_ratio)
            && ratio(self.right_ratio)
        {
            Ok(())
        } else {
            Err(VehicleError::InvalidValue(
                "forward must be in [-1, 1], brake in [0, 1], track ratios within 1/limits::MAX_RATIO..=1 in magnitude",
            ))
        }
    }
}

/// One of a tracked vehicle's two tracks.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum TrackSide {
    /// The left track, Jolt's track 0.
    Left,
    /// The right track, Jolt's track 1.
    Right,
}

/// The state of a track after the last step.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct TrackState {
    /// The vehicle wheel index of the track's driven wheel.
    pub driven_wheel: u32,
    /// Rotation speed of the driven wheel, rad/s; positive when the track drives the vehicle
    /// forward.
    pub angular_velocity: f32,
    /// Speed of the track over its driven wheel, m/s: the angular velocity times that wheel's
    /// radius.
    pub speed: f32,
}

impl VehicleRef<'_, TrackedVehicle> {
    fn tracked_controller(&self) -> *const JPH_TrackedVehicleController {
        // A tracked vehicle's controller is a `TrackedVehicleController`, which derives from
        // `VehicleController` with single inheritance.
        self.controller().cast()
    }

    /// The state of the track on `side`.
    pub fn track(&self, side: TrackSide) -> TrackState {
        let jolt_side = match side {
            TrackSide::Left => JPH_TrackSide_Left,
            TrackSide::Right => JPH_TrackSide_Right,
        };
        // SAFETY: the controller is live, owned by the constraint that the world borrowed here
        // owns; the track is a member of it and only read.
        let (driven_wheel, angular_velocity) = unsafe {
            let track = JPH_TrackedVehicleController_GetTrack(self.tracked_controller(), jolt_side);
            (
                JPH_VehicleTrack_GetDrivenWheel(track),
                JPH_VehicleTrack_GetAngularVelocity(track),
            )
        };
        let radius = self.entry.wheels[driven_wheel as usize].radius;
        TrackState {
            driven_wheel,
            angular_velocity,
            speed: angular_velocity * radius,
        }
    }

    /// The states of the left and the right track.
    pub fn tracks(&self) -> [TrackState; 2] {
        [self.track(TrackSide::Left), self.track(TrackSide::Right)]
    }

    /// The vehicle wheel indices of the track on `side`: the left track's wheels come first.
    pub fn track_wheels(&self, side: TrackSide) -> Range<u32> {
        let [left, right] = self
            .entry
            .tracks
            .clone()
            .unwrap_or_else(|| unreachable!("a tracked vehicle has tracks"));
        match side {
            TrackSide::Left => left,
            TrackSide::Right => right,
        }
    }

    /// The driver input last set.
    pub fn driver_input(&self) -> TrackedDriverInput {
        let controller = self.tracked_controller();
        // SAFETY: as in `track`; the getters read members.
        unsafe {
            TrackedDriverInput {
                forward: JPH_TrackedVehicleController_GetForwardInput(controller),
                left_ratio: JPH_TrackedVehicleController_GetLeftRatio(controller),
                right_ratio: JPH_TrackedVehicleController_GetRightRatio(controller),
                brake: JPH_TrackedVehicleController_GetBrakeInput(controller),
            }
        }
    }
}

impl VehicleMut<'_, TrackedVehicle> {
    /// Sets what the driver asks for the next steps; see [`TrackedDriverInput`] for the ranges.
    /// A refused input changes nothing.
    pub fn set_driver_input(&mut self, input: TrackedDriverInput) -> Result<(), VehicleError> {
        input.validate()?;
        let controller: *mut JPH_TrackedVehicleController = self.controller().cast();
        // SAFETY: a tracked vehicle's controller is a `TrackedVehicleController`, which derives
        // from `VehicleController` with single inheritance; the world is borrowed mutably
        // through this view. The setter writes members.
        unsafe {
            JPH_TrackedVehicleController_SetDriverInput(
                controller,
                input.forward,
                input.left_ratio,
                input.right_ratio,
                input.brake,
            );
        }
        Ok(())
    }
}

impl PhysicsWorld {
    /// Attaches a tracked vehicle to the dynamic body `body`, its chassis, and returns its id.
    ///
    /// The chassis works as for [`create_vehicle`](Self::create_vehicle): it stays an ordinary
    /// body the vehicle is a constraint and step listener on, and the same chassis rules and
    /// errors apply. The settings are checked as their setters state, plus the step coefficients
    /// of [docs/limits.md#tracked-step-coefficients]. Nothing is created on failure.
    ///
    /// [docs/limits.md#tracked-step-coefficients]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#tracked-step-coefficients
    ///
    /// # Example
    /// A tank with five wheels per track turns in place.
    /// ```
    /// use oxijolt::*;
    ///
    /// # fn main() -> Result<(), Box<dyn std::error::Error>> {
    /// let mut world = PhysicsWorld::new(WorldSettings::default())?;
    /// let floor = Shape::new_box(Vec3::new(50.0, 1.0, 50.0))?;
    /// world.create_body(&floor, &BodySettings::new_static().position(RVec3::new(0.0, -1.0, 0.0)))?;
    ///
    /// let hull = Shape::new_box(Vec3::new(1.7, 0.5, 3.2))?;
    /// let chassis_shape = Shape::new_offset_center_of_mass(&hull, Vec3::new(0.0, -0.5, 0.0))?;
    /// let chassis = world.create_body(
    ///     &chassis_shape,
    ///     &BodySettings::new_dynamic()
    ///         .position(RVec3::new(0.0, 1.0, 0.0))
    ///         .mass(4000.0)
    ///         .allow_sleeping(false),
    /// )?;
    /// let track = |x: f32| {
    ///     let wheels = [2.4, 1.2, 0.0, -1.2, -2.4]
    ///         .map(|z| TrackedWheelSettings::new(Vec3::new(x, -0.3, z)))
    ///         .to_vec();
    ///     VehicleTrackSettings::new(wheels, 4)
    /// };
    /// let settings = TrackedVehicleSettings::new(
    ///     track(1.4),
    ///     track(-1.4),
    ///     VehicleCollisionTester::ray(ObjectLayer::MOVING),
    /// );
    /// let tank = world.create_tracked_vehicle(chassis, &settings)?;
    ///
    /// let pivot = TrackedDriverInput { forward: 1.0, left_ratio: -1.0, right_ratio: 1.0, brake: 0.0 };
    /// world.vehicle_mut(tank)?.set_driver_input(pivot)?;
    /// for _ in 0..60 {
    ///     assert!(world.step(1.0 / 60.0)?.is_complete());
    /// }
    /// let [left, right] = world.vehicle(tank)?.tracks();
    /// assert!(left.angular_velocity < 0.0 && right.angular_velocity > 0.0);
    /// # Ok(())
    /// # }
    /// ```
    pub fn create_tracked_vehicle(
        &mut self,
        body: BodyId,
        settings: &TrackedVehicleSettings,
    ) -> Result<VehicleId<TrackedVehicle>, VehicleError> {
        settings.validate(self.object_layer_count)?;
        self.check_chassis(body)?;
        let id = self.attach_vehicle(body, settings.build())?;
        Ok(VehicleId::new(id.raw, self.tag))
    }
}
