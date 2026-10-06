//! Tracked vehicles: Jolt's `VehicleConstraint` with the tracked controller.

use std::ops::Range;
use std::ptr::NonNull;

use oxijolt_sys::*;

use super::settings::ChassisMass;
use super::{TrackedVehicle, TrackedVehicleSettings, VehicleId};
use crate::body::with_read_locked_body;
use crate::limits::{self, PrincipalMass};
use crate::{BodyId, PhysicsWorld, Quat, Vec3, VehicleError, VehicleMut, VehicleRef};

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
    /// The chassis works as for [`create_wheeled_vehicle`](Self::create_wheeled_vehicle): it stays an ordinary
    /// body the vehicle is a constraint and step listener on, and the same chassis rules and
    /// errors apply. The settings are checked as their setters state, plus the track drive
    /// envelope of [docs/limits.md#track-drive-envelope]; and at every wheel, the track's inertia
    /// must stay small against the chassis' mass and inertia
    /// ([`limits::MAX_TRACK_MASS_RATIO`]). Nothing is created on failure.
    ///
    /// [docs/limits.md#track-drive-envelope]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#track-drive-envelope
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
        let track_mass_ratio = settings.largest_track_mass_ratio(&self.chassis_mass(body));
        if !limits::is_track_mass_ratio(track_mass_ratio) {
            return Err(VehicleError::InvalidValue(limits::TRACK_MASS_RATIO_RULE));
        }
        let id = self.attach_vehicle(body, settings.build())?;
        Ok(VehicleId::new(id.raw, self.tag))
    }

    /// The mass properties of `body`, a chassis [`check_chassis`](Self::check_chassis) accepted.
    fn chassis_mass(&self, body: BodyId) -> ChassisMass {
        with_read_locked_body(self.body_lock_interface, body, |chassis| {
            // SAFETY: the chassis is locked for reading for the call and dynamic.
            unsafe { read_chassis_mass(chassis) }
        })
        .unwrap_or_else(|| unreachable!("`check_chassis` found the body and `&mut self` keeps it"))
    }
}

/// Reads the mass properties of `chassis`.
///
/// # Safety
/// `chassis` is a live dynamic body, locked for reading for the call.
unsafe fn read_chassis_mass(chassis: NonNull<JPH_Body>) -> ChassisMass {
    let body = chassis.as_ptr();
    let mut inverse_inertia = Vec3::ZERO.to_jph();
    let mut inertia_rotation = Quat::IDENTITY.to_jph();
    let mut center_of_mass = Vec3::ZERO.to_jph();
    // SAFETY: the caller's contract. A dynamic body has motion properties, and its shape lives
    // as long as the body; the getters only read them and write the live locals.
    let inverse_mass = unsafe {
        let motion = JPH_Body_GetMotionProperties(body);
        JPH_MotionProperties_GetInverseInertiaDiagonal(motion, &mut inverse_inertia);
        JPH_MotionProperties_GetInertiaRotation(motion, &mut inertia_rotation);
        JPH_Shape_GetCenterOfMass(JPH_Body_GetShape(body), &mut center_of_mass);
        JPH_MotionProperties_GetInverseMassUnchecked(motion)
    };
    let inverse_inertia = Vec3::from_jph(inverse_inertia);
    ChassisMass {
        mass: PrincipalMass {
            inverse_mass: f64::from(inverse_mass),
            inverse_inertia: [inverse_inertia.x, inverse_inertia.y, inverse_inertia.z]
                .map(f64::from),
        },
        // Jolt's local inverse inertia is `R D Rᵀ` with `R` the inertia rotation, so `Rᵀ` takes
        // body space into the principal frame.
        body_to_principal: Quat::from_jph(inertia_rotation).conjugated(),
        center_of_mass: Vec3::from_jph(center_of_mass),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{BodySettings, CompoundChild, Shape, WorldSettings};

    #[test]
    fn chassis_mass_turns_body_space_into_the_principal_frame() {
        // A 6 kg box of half extents (2, 1, 0.5), principal inertia m/3 · (b² + c², a² + c²,
        // a² + b²) = (2.5, 8.5, 10), turned by 30° about z inside a compound.
        let mut world = PhysicsWorld::new(WorldSettings::default()).unwrap();
        let angle = 30.0_f64.to_radians();
        let (half_sin, half_cos) = (0.5 * angle).sin_cos();
        let hull = Shape::new_box(Vec3::new(2.0, 1.0, 0.5)).unwrap();
        let shape = Shape::new_compound(&[CompoundChild {
            shape: &hull,
            position: Vec3::ZERO,
            rotation: Quat::from_xyzw(0.0, 0.0, half_sin as f32, half_cos as f32),
            user_data: 0,
        }])
        .unwrap();
        let body = world
            .create_body(&shape, &BodySettings::new_dynamic().mass(6.0))
            .unwrap();
        let chassis = world.chassis_mass(body);
        // The body-space inverse inertia the chassis mass describes, against the box's turned by
        // the same angle.
        let axes = [
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        ];
        let principal = axes.map(|axis| {
            let v = chassis.body_to_principal.rotate(axis);
            [v.x, v.y, v.z].map(f64::from)
        });
        let (sin, cos) = angle.sin_cos();
        let inverse = [1.0 / 2.5, 1.0 / 8.5, 1.0 / 10.0];
        let turned = [[cos, -sin, 0.0], [sin, cos, 0.0], [0.0, 0.0, 1.0]];
        for i in 0..3 {
            for j in 0..3 {
                let read: f64 = (0..3)
                    .map(|k| chassis.mass.inverse_inertia[k] * principal[i][k] * principal[j][k])
                    .sum();
                let expected: f64 = (0..3)
                    .map(|k| turned[i][k] * inverse[k] * turned[j][k])
                    .sum();
                assert!(
                    (read - expected).abs() < 1e-5,
                    "{i}{j}: {read} vs {expected}"
                );
            }
        }
        assert!((chassis.mass.inverse_mass - 1.0 / 6.0).abs() < 1e-7);
    }
}
