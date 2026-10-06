//! Settings of a tracked vehicle: its wheels, its two tracks and its drivetrain.

use std::ops::Range;

use oxijolt_sys::*;

use super::wheel::WheelBase;
use super::{
    validate_wheel_count, BuiltSettings, ControllerGuard, VehicleCollisionTester,
    VehicleEngineSettings, VehicleFrame, VehicleTransmissionSettings, WheelGeometry, WheelGuard,
    ANGULAR_VELOCITY_TO_RPM,
};
use crate::limits::{self, PrincipalMass};
use crate::math::{is_finite_non_negative, is_finite_positive};
use crate::owned::{JoltObject, Owned};
use crate::{PhysicsWorld, Quat, SpringSettings, Vec3, VehicleError};

/// One wheel of a tracked vehicle (Jolt `WheelSettings` and `WheelSettingsTV`). The wheel
/// turns with its track; its suspension and size work as for [`WheelSettings`], whose methods
/// of the same names state the rules. Positions and directions are in the chassis body's local
/// space, lengths in metres. The defaults are Jolt's.
///
/// [`WheelSettings`]: crate::WheelSettings
#[derive(Clone, Debug, PartialEq)]
pub struct TrackedWheelSettings {
    pub(super) base: WheelBase,
    pub(super) longitudinal_friction: f32,
    pub(super) lateral_friction: f32,
}

impl TrackedWheelSettings {
    /// A wheel whose suspension is attached to the chassis at `position` (body space, each
    /// component within [`limits::MAX_SHAPE_EXTENT`]).
    pub fn new(position: Vec3) -> Self {
        Self {
            base: WheelBase::new(position),
            longitudinal_friction: 4.0,
            lateral_friction: 2.0,
        }
    }

    /// See [`WheelSettings::suspension_force_point`](crate::WheelSettings::suspension_force_point).
    #[must_use]
    pub fn suspension_force_point(mut self, value: Option<Vec3>) -> Self {
        self.base.suspension_force_point = value;
        self
    }

    /// See [`WheelSettings::suspension_direction`](crate::WheelSettings::suspension_direction).
    #[must_use]
    pub fn suspension_direction(mut self, value: Vec3) -> Self {
        self.base.suspension_direction = value;
        self
    }

    /// See [`WheelSettings::wheel_up`](crate::WheelSettings::wheel_up).
    #[must_use]
    pub fn wheel_up(mut self, value: Vec3) -> Self {
        self.base.wheel_up = value;
        self
    }

    /// See [`WheelSettings::wheel_forward`](crate::WheelSettings::wheel_forward).
    #[must_use]
    pub fn wheel_forward(mut self, value: Vec3) -> Self {
        self.base.wheel_forward = value;
        self
    }

    /// See [`WheelSettings::suspension_min_length`](crate::WheelSettings::suspension_min_length).
    #[must_use]
    pub fn suspension_min_length(mut self, value: f32) -> Self {
        self.base.suspension_min_length = value;
        self
    }

    /// See [`WheelSettings::suspension_max_length`](crate::WheelSettings::suspension_max_length).
    #[must_use]
    pub fn suspension_max_length(mut self, value: f32) -> Self {
        self.base.suspension_max_length = value;
        self
    }

    /// See
    /// [`WheelSettings::suspension_preload_length`](crate::WheelSettings::suspension_preload_length).
    #[must_use]
    pub fn suspension_preload_length(mut self, value: f32) -> Self {
        self.base.suspension_preload_length = value;
        self
    }

    /// See [`WheelSettings::suspension_spring`](crate::WheelSettings::suspension_spring).
    #[must_use]
    pub fn suspension_spring(mut self, value: SpringSettings) -> Self {
        self.base.suspension_spring = value;
        self
    }

    /// See [`WheelSettings::radius`](crate::WheelSettings::radius).
    #[must_use]
    pub fn radius(mut self, value: f32) -> Self {
        self.base.radius = value;
        self
    }

    /// See [`WheelSettings::width`](crate::WheelSettings::width).
    #[must_use]
    pub fn width(mut self, value: f32) -> Self {
        self.base.width = value;
        self
    }

    /// Friction coefficient of the track on the ground along the rolling direction, between 0
    /// and [`limits::MAX_FRICTION`]. Default 4.
    #[must_use]
    pub fn longitudinal_friction(mut self, value: f32) -> Self {
        self.longitudinal_friction = value;
        self
    }

    /// Friction coefficient of the track on the ground sideways, between 0 and
    /// [`limits::MAX_FRICTION`]. Default 2.
    #[must_use]
    pub fn lateral_friction(mut self, value: f32) -> Self {
        self.lateral_friction = value;
        self
    }

    fn validate(&self) -> Result<(), VehicleError> {
        self.base.validate()?;
        if !(limits::is_friction(self.longitudinal_friction)
            && limits::is_friction(self.lateral_friction))
        {
            return Err(VehicleError::InvalidValue(
                "track wheel friction must be finite and between 0 and limits::MAX_FRICTION",
            ));
        }
        Ok(())
    }

    /// The joltc settings of this validated wheel.
    pub(super) fn create(&self) -> Owned<JPH_WheelSettingsTV> {
        // SAFETY: Jolt is initialised (a world exists). The settings are returned holding one
        // reference, which the guard takes over.
        let wheel = unsafe { Owned::from_raw(JPH_WheelSettingsTV_Create()) }
            .unwrap_or_else(|| unreachable!("joltc `new`s the settings"));
        let ptr = wheel.as_ptr();
        // SAFETY: the guard owns the new settings, which nothing else uses yet; a
        // `WheelSettingsTV` derives from `WheelSettings` with single inheritance, joltc's own
        // cast convention. All values were validated.
        unsafe {
            self.base.apply(ptr.cast());
            JPH_WheelSettingsTV_SetLongitudinalFriction(ptr, self.longitudinal_friction);
            JPH_WheelSettingsTV_SetLateralFriction(ptr, self.lateral_friction);
        }
        wheel
    }
}

/// One track of a tracked vehicle (Jolt `VehicleTrackSettings`): the wheels it runs over and
/// how the drivetrain turns it. The defaults are Jolt's.
#[derive(Clone, Debug, PartialEq)]
pub struct VehicleTrackSettings {
    pub(super) wheels: Vec<TrackedWheelSettings>,
    pub(super) driven_wheel: u32,
    pub(super) inertia: f32,
    pub(super) angular_damping: f32,
    pub(super) max_brake_torque: f32,
    pub(super) differential_ratio: f32,
}

impl VehicleTrackSettings {
    /// A track over `wheels` (at least one), driven by the engine at `driven_wheel`, an index
    /// into `wheels`. The track's speed is the driven wheel's: every other wheel turns at the
    /// track speed over its own radius, so every radius must be within a factor
    /// [`limits::MAX_RATIO`] of the driven wheel's.
    pub fn new(wheels: Vec<TrackedWheelSettings>, driven_wheel: u32) -> Self {
        Self {
            wheels,
            driven_wheel,
            inertia: 10.0,
            angular_damping: 0.5,
            max_brake_torque: 15000.0,
            differential_ratio: 6.0,
        }
    }

    /// Moment of inertia of the track and its wheels about the driven wheel's axle, kg·m², within
    /// [`limits::MIN_TRACK_INERTIA`]`..=`[`limits::MAX_TRACK_INERTIA`] and within a factor
    /// [`limits::MAX_TRACK_INERTIA_RATIO`] of the other track's; the chassis bounds it too
    /// ([`limits::MAX_TRACK_MASS_RATIO`]). Default 10.
    #[must_use]
    pub fn inertia(mut self, value: f32) -> Self {
        self.inertia = value;
        self
    }

    /// Angular damping of the track, `dω/dt = −c·ω`, not negative. Default 0.5.
    #[must_use]
    pub fn angular_damping(mut self, value: f32) -> Self {
        self.angular_damping = value;
        self
    }

    /// Largest brake torque on the track, N·m, not negative. Default 15000.
    #[must_use]
    pub fn max_brake_torque(mut self, value: f32) -> Self {
        self.max_brake_torque = value;
        self
    }

    /// Ratio between the gearbox and the driven wheel's rotation rates, positive. Default 6.
    #[must_use]
    pub fn differential_ratio(mut self, value: f32) -> Self {
        self.differential_ratio = value;
        self
    }

    fn validate(&self) -> Result<(), VehicleError> {
        let invalid = |what| Err(VehicleError::InvalidValue(what));
        if self.wheels.is_empty() {
            return invalid("a track needs at least one wheel");
        }
        if (self.driven_wheel as usize) >= self.wheels.len() {
            return invalid("a track's driven wheel must be one of its wheels");
        }
        // Jolt asserts only `>= 0` but divides by the inertia
        // (`TrackedVehicleController::PostCollide`).
        if !limits::is_track_inertia(self.inertia) {
            return invalid(
                "track inertia must be within limits::MIN_TRACK_INERTIA..=limits::MAX_TRACK_INERTIA",
            );
        }
        if !(is_finite_non_negative(self.angular_damping)
            && is_finite_non_negative(self.max_brake_torque))
        {
            return invalid(
                "track angular damping and max brake torque must be finite and not negative",
            );
        }
        if !is_finite_positive(self.differential_ratio) {
            return invalid("track differential ratio must be finite and positive");
        }
        for wheel in &self.wheels {
            wheel.validate()?;
        }
        let driven_radius = self.driven_radius();
        if !self
            .wheels
            .iter()
            .all(|wheel| limits::is_ratio(driven_radius / wheel.base.radius))
        {
            return invalid(
                "a track's wheel radii must be within a factor limits::MAX_RATIO of its driven wheel's",
            );
        }
        Ok(())
    }

    /// The smallest wheel radius of this validated track.
    fn smallest_radius(&self) -> f32 {
        self.wheels
            .iter()
            .map(|wheel| wheel.base.radius)
            .fold(f32::INFINITY, f32::min)
    }

    /// The radius of this validated track's driven wheel.
    fn driven_radius(&self) -> f32 {
        self.wheels[self.driven_wheel as usize].base.radius
    }
}

/// The settings of a tracked vehicle such as a tank (Jolt `VehicleConstraintSettings` with a
/// `TrackedVehicleControllerSettings`).
///
/// The vehicle's wheels are those of the left track, then those of the right track: wheel `i`
/// of the right track is vehicle wheel `left wheel count + i`. Directions are in the chassis
/// body's local space; with up +Y and forward +Z, Jolt's samples put the left track at +X.
#[derive(Clone, Debug, PartialEq)]
pub struct TrackedVehicleSettings {
    pub(super) left: VehicleTrackSettings,
    pub(super) right: VehicleTrackSettings,
    pub(super) collision_tester: VehicleCollisionTester,
    frame: VehicleFrame,
    engine: VehicleEngineSettings,
    transmission: VehicleTransmissionSettings,
}

impl TrackedVehicleSettings {
    /// A vehicle on the tracks `left` and `right`, whose wheels find the ground with
    /// `collision_tester`, with Jolt's tracked [`default_engine`](Self::default_engine) and
    /// [`default_transmission`](Self::default_transmission).
    pub fn new(
        left: VehicleTrackSettings,
        right: VehicleTrackSettings,
        collision_tester: VehicleCollisionTester,
    ) -> Self {
        Self {
            left,
            right,
            collision_tester,
            frame: VehicleFrame::default(),
            engine: Self::default_engine(),
            transmission: Self::default_transmission(),
        }
    }

    /// Jolt's engine for tracked vehicles: 500 to 4000 rpm and 500 N·m, otherwise
    /// [`VehicleEngineSettings::default`].
    pub fn default_engine() -> VehicleEngineSettings {
        VehicleEngineSettings::default()
            .min_rpm(500.0)
            .max_rpm(4000.0)
            .max_torque(500.0)
    }

    /// Jolt's transmission for tracked vehicles: gears `[4, 3, 2, 1]`, reverse gears
    /// `[-4, -3]`, shifting down below 1000 rpm and up above 3500 rpm, otherwise
    /// [`VehicleTransmissionSettings::default`].
    pub fn default_transmission() -> VehicleTransmissionSettings {
        VehicleTransmissionSettings::default()
            .gear_ratios(vec![4.0, 3.0, 2.0, 1.0])
            .reverse_gear_ratios(vec![-4.0, -3.0])
            .shift_down_rpm(1000.0)
            .shift_up_rpm(3500.0)
    }

    /// Up of the vehicle in body space, a unit vector. Default +Y.
    #[must_use]
    pub fn up(mut self, value: Vec3) -> Self {
        self.frame.up = value;
        self
    }

    /// Forward of the vehicle in body space, a unit vector perpendicular to
    /// [`up`](Self::up). Default +Z.
    #[must_use]
    pub fn forward(mut self, value: Vec3) -> Self {
        self.frame.forward = value;
        self
    }

    /// See [`WheeledVehicleSettings::max_pitch_roll_angle`](crate::WheeledVehicleSettings::max_pitch_roll_angle).
    #[must_use]
    pub fn max_pitch_roll_angle(mut self, radians: f32) -> Self {
        self.frame.max_pitch_roll_angle = radians;
        self
    }

    /// The engine. Default [`default_engine`](Self::default_engine).
    #[must_use]
    pub fn engine(mut self, value: VehicleEngineSettings) -> Self {
        self.engine = value;
        self
    }

    /// The transmission. Default [`default_transmission`](Self::default_transmission).
    #[must_use]
    pub fn transmission(mut self, value: VehicleTransmissionSettings) -> Self {
        self.transmission = value;
        self
    }

    fn tracks(&self) -> [&VehicleTrackSettings; 2] {
        [&self.left, &self.right]
    }

    /// The vehicle wheel indices of the left and the right track.
    fn track_wheels(&self) -> [Range<u32>; 2] {
        // `validate` bounds the wheel count to `u32`.
        let left = self.left.wheels.len() as u32;
        let right = self.right.wheels.len() as u32;
        [0..left, left..left + right]
    }

    /// Checks every value Jolt asserts on, indexes with or divides by, and the
    /// [drive envelope](Self::validate_drive_envelope).
    pub(crate) fn validate(&self, object_layer_count: u32) -> Result<(), VehicleError> {
        self.frame.validate()?;
        for track in self.tracks() {
            track.validate()?;
        }
        if !limits::is_track_inertia_ratio(self.left.inertia, self.right.inertia) {
            return Err(VehicleError::InvalidValue(
                "track inertia ratio must be at most limits::MAX_TRACK_INERTIA_RATIO",
            ));
        }
        validate_wheel_count(
            self.left
                .wheels
                .len()
                .saturating_add(self.right.wheels.len()),
        )?;
        self.engine.validate()?;
        self.transmission.validate(&self.engine)?;
        self.engine.validate_step_coefficients()?;
        self.validate_drive_envelope()?;
        self.collision_tester
            .validate(object_layer_count, self.wheel_geometry().into_iter())
    }

    /// Checks that each track's speed limit at the engine's rpm keeps its `f32` divisor normal,
    /// and that every term of the [`drive_envelope`](Self::drive_envelope) is at most
    /// [`ENVELOPE_CEILING`].
    fn validate_drive_envelope(&self) -> Result<(), VehicleError> {
        let invalid = |what| Err(VehicleError::InvalidValue(what));
        let (smallest_gear, _) = self.gear_range();
        for track in self.tracks() {
            // The divisor of the speed limit, in Jolt's order (`TrackedVehicleController.cpp:279`).
            let divisor = smallest_gear as f32
                * track.differential_ratio
                * (1.0 / limits::MAX_RATIO)
                * ANGULAR_VELOCITY_TO_RPM;
            if divisor < f32::MIN_POSITIVE {
                return invalid(
                    "gear and track differential ratios give an engine speed limit that underflows",
                );
            }
        }
        if !self
            .drive_envelope()
            .into_iter()
            .all(|term| term <= ENVELOPE_CEILING)
        {
            return invalid("tracked drivetrain exceeds the track drive envelope");
        }
        Ok(())
    }

    /// The smallest and the largest gear ratio in magnitude, forward and reverse.
    fn gear_range(&self) -> (f64, f64) {
        let transmission = &self.transmission;
        let gears = || {
            transmission
                .gear_ratios
                .iter()
                .chain(&transmission.reverse_gear_ratios)
                .map(|ratio| f64::from(ratio.abs()))
        };
        (
            gears().fold(f64::INFINITY, f64::min),
            gears().fold(0.0, f64::max),
        )
    }

    /// The terms the tracked controller's step forms from the settings and from the track
    /// speeds the drivetrain can reach, in real arithmetic: the drive envelope derived in
    /// [docs/limits.md#track-drive-envelope].
    ///
    /// [docs/limits.md#track-drive-envelope]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#track-drive-envelope
    fn drive_envelope(&self) -> Vec<f64> {
        let smallest_ratio = 1.0 / f64::from(limits::MAX_RATIO);
        let dt = f64::from(PhysicsWorld::MAX_DELTA_TIME);
        let rpm_per_rad = f64::from(ANGULAR_VELOCITY_TO_RPM);
        let (smallest_gear, largest_gear) = self.gear_range();
        let transmission_torque = largest_gear * f64::from(self.engine.largest_torque());
        let tracks = self.tracks();
        let inertia = tracks.map(|track| f64::from(track.inertia));
        // How fast the drivetrain alone takes a track: its speed limit at the max rpm and the
        // smallest gear and track ratio, plus one torque step.
        let target = tracks.map(|track| {
            let differential = f64::from(track.differential_ratio);
            let limit = 1.001 * f64::from(self.engine.max_rpm)
                / (smallest_gear * differential * smallest_ratio * rpm_per_rad);
            limit + differential * transmission_torque * dt / f64::from(track.inertia)
        });
        // The largest `|ω_l| / I_l + |ω_r| / I_r`, which the synchronisation, damping and
        // brakes never raise.
        let weighted = (target[0] / inertia[0] + target[1] / inertia[1])
            .max(target[0] * (1.0 / inertia[0] + 1.0 / (smallest_ratio * inertia[1])))
            .max(target[1] * (1.0 / inertia[1] + 1.0 / (smallest_ratio * inertia[0])));
        let speed = [
            weighted * inertia[0].min(inertia[1] / smallest_ratio),
            weighted * inertia[1].min(inertia[0] / smallest_ratio),
        ];
        let sync = (speed[0] + speed[1]) / (smallest_ratio * (inertia[0] + inertia[1]));
        let mut terms = vec![transmission_torque, sync, sync * inertia[0].max(inertia[1])];
        for (side, track) in tracks.into_iter().enumerate() {
            let differential_torque = f64::from(track.differential_ratio) * transmission_torque;
            let brake = f64::from(track.max_brake_torque);
            let brake_per_radius = brake / f64::from(track.smallest_radius());
            terms.extend([
                speed[side],
                speed[side] * inertia[side] / f64::from(PhysicsWorld::MIN_DELTA_TIME),
                differential_torque,
                differential_torque * dt,
                brake * dt / inertia[side],
                brake_per_radius,
                brake_per_radius * dt,
            ]);
            let driven_radius = f64::from(track.driven_radius());
            for wheel in &track.wheels {
                let radius = f64::from(wheel.base.radius);
                let wheel_speed = speed[side] * driven_radius / radius;
                terms.extend([
                    wheel_speed,
                    wheel_speed * dt,
                    inertia[side] / radius,
                    radius / inertia[side],
                ]);
            }
        }
        terms
    }

    /// The largest [track mass ratio](limits::MAX_TRACK_MASS_RATIO) of these validated
    /// settings' wheels on `chassis`, as derived in [docs/limits.md#track-mass-ratio].
    ///
    /// [docs/limits.md#track-mass-ratio]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#track-mass-ratio
    pub(crate) fn largest_track_mass_ratio(&self, chassis: &ChassisMass) -> f64 {
        self.tracks()
            .into_iter()
            .flat_map(|track| track.wheels.iter().map(move |wheel| (track.inertia, wheel)))
            .map(|(inertia, wheel)| {
                let radius = f64::from(wheel.base.radius);
                f64::from(inertia) / (radius * radius) * chassis.inverse_effective_mass(&wheel.base)
            })
            .fold(0.0, f64::max)
    }

    fn wheel_geometry(&self) -> Vec<WheelGeometry> {
        self.tracks()
            .into_iter()
            .flat_map(|track| &track.wheels)
            .map(|wheel| wheel.base.geometry())
            .collect()
    }

    /// The joltc objects of these validated settings.
    pub(crate) fn build(&self) -> BuiltSettings {
        BuiltSettings {
            wheels: self
                .tracks()
                .into_iter()
                .flat_map(|track| &track.wheels)
                .map(|wheel| WheelGuard::Tracked(wheel.create()))
                .collect(),
            controller: ControllerGuard::Tracked(self.create_controller()),
            anti_roll_bars: Vec::new(),
            frame: self.frame,
            collision_tester: self.collision_tester,
            geometry: self.wheel_geometry(),
            tracks: Some(self.track_wheels()),
        }
    }

    /// The joltc controller settings: engine, transmission and both tracks.
    pub(super) fn create_controller(&self) -> Owned<JPH_TrackedVehicleControllerSettings> {
        // SAFETY: Jolt is initialised (a world exists). The settings are returned holding one
        // reference, which the guard takes over.
        let controller = unsafe { Owned::from_raw(JPH_TrackedVehicleControllerSettings_Create()) }
            .unwrap_or_else(|| unreachable!("joltc `new`s the settings"));
        let ptr = controller.as_ptr();
        let transmission = self.transmission.create();
        self.engine.with_jph(|engine| {
            // SAFETY: the guards own the controller and transmission settings, and the engine's
            // curve lives for this closure; joltc copies the engine (with its curve) and the
            // transmission. All values were validated.
            unsafe {
                JPH_TrackedVehicleControllerSettings_SetEngine(ptr, engine);
                JPH_TrackedVehicleControllerSettings_SetTransmission(ptr, transmission.as_ptr());
            }
        });
        for (side, (track, wheels)) in self
            .tracks()
            .into_iter()
            .zip(self.track_wheels())
            .enumerate()
        {
            let indices: Vec<u32> = wheels.clone().collect();
            let jolt_track = JPH_VehicleTrackSettings {
                // Jolt reads the driven wheel as an index into the vehicle's wheels.
                drivenWheel: wheels.start + track.driven_wheel,
                wheels: indices.as_ptr(),
                wheelsCount: indices.len() as u32,
                inertia: track.inertia,
                angularDamping: track.angular_damping,
                maxBrakeTorque: track.max_brake_torque,
                differentialRatio: track.differential_ratio,
            };
            // SAFETY: the guard owns the controller settings; `jolt_track` and `indices` are live
            // for the call, which copies them. `side` is 0 (left) or 1 (right), the two tracks
            // joltc stores.
            unsafe { JPH_TrackedVehicleControllerSettings_SetTrack(ptr, side as u32, &jolt_track) };
        }
        controller
    }
}

/// A tracked vehicle's chassis as its [track mass ratio](limits::MAX_TRACK_MASS_RATIO) needs it.
#[derive(Clone, Copy, Debug)]
pub(crate) struct ChassisMass {
    /// The chassis' inverse mass and principal inverse inertia.
    pub(crate) mass: PrincipalMass,
    /// The rotation from the chassis' body space into its principal frame.
    pub(crate) body_to_principal: Quat,
    /// The centre of mass in body space.
    pub(crate) center_of_mass: Vec3,
}

impl ChassisMass {
    /// The largest inverse effective mass of the chassis where `wheel` can push it along any
    /// direction in the wheel's forward/up plane, which holds Jolt's longitudinal direction for
    /// every ground normal: the lever is taken at the farthest point the wheel's contact can
    /// reach, and at the suspension force point when the wheel has one.
    fn inverse_effective_mass(&self, wheel: &WheelBase) -> f64 {
        let principal = |v: Vec3| {
            let v = self.body_to_principal.rotate(v);
            [v.x, v.y, v.z].map(f64::from)
        };
        let center = self.center_of_mass;
        let from_center =
            |p: Vec3| principal(Vec3::new(p.x - center.x, p.y - center.y, p.z - center.z));
        // Jolt's longitudinal direction is `normal × right` (`VehicleConstraint::OnStep`).
        let up = limits::normalized(principal(wheel.wheel_up));
        let right = limits::normalized(limits::cross(principal(wheel.wheel_forward), up));
        let forward = limits::cross(up, right);
        // Every tester puts the contact within this distance of the attachment point.
        let reach = f64::from(wheel.suspension_max_length)
            + f64::from(wheel.radius).hypot(0.5 * f64::from(wheel.width));
        let lever_at = |point: Vec3| self.mass.lever_in_plane(from_center(point), forward, up);
        let mut lever = lever_at(wheel.position) + reach * self.mass.lever_per_metre();
        if let Some(point) = wheel.suspension_force_point {
            lever = lever.max(lever_at(point));
        }
        self.mass.inverse_mass + lever * lever
    }
}

/// Largest value a term of the [drive envelope](TrackedVehicleSettings::drive_envelope) may
/// reach, 2²⁸ below `f32::MAX`: Jolt forms the terms in `f32`, the envelope in real arithmetic,
/// and the headroom covers the difference. See [docs/limits.md#track-drive-envelope].
///
/// [docs/limits.md#track-drive-envelope]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#track-drive-envelope
const ENVELOPE_CEILING: f64 = 1.0e30;

/// Tracked wheel settings, of which the owner holds the one reference
/// `JPH_WheelSettingsTV_Create` returns; each wheel of a vehicle keeps its own.
impl JoltObject for JPH_WheelSettingsTV {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner holds one reference (trait contract), released here once; a
        // `WheelSettingsTV` is a `WheelSettings` with single inheritance.
        unsafe { JPH_WheelSettings_Destroy(ptr.cast()) };
    }
}

/// Tracked controller settings, of which the owner holds the one reference
/// `JPH_TrackedVehicleControllerSettings_Create` returns. A vehicle copies them into its own
/// controller when it is created.
impl JoltObject for JPH_TrackedVehicleControllerSettings {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner holds one reference (trait contract), released here once; the
        // tracked settings derive from `VehicleControllerSettings` with single inheritance.
        unsafe { JPH_VehicleControllerSettings_Destroy(ptr.cast()) };
    }
}

#[cfg(test)]
#[path = "tracked_tests.rs"]
mod tests;
