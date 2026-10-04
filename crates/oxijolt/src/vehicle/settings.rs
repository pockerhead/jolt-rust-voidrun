//! Settings of a wheeled vehicle, in Jolt's vocabulary, with Jolt's defaults.
//!
//! The types here are plain Rust values. [`PhysicsWorld::create_vehicle`] validates them and
//! builds the joltc settings objects from them, which it releases again once the vehicle exists.
//!
//! [`PhysicsWorld::create_vehicle`]: crate::PhysicsWorld::create_vehicle

use std::f32::consts::PI;

use oxijolt_sys::*;

use crate::math::{is_finite_non_negative, is_unit};
use crate::owned::{JoltObject, Owned};
use crate::{PhysicsWorld, Vec3, VehicleError};

mod collision_tester;
mod drivetrain;
mod wheel;

pub use collision_tester::VehicleCollisionTester;
pub use drivetrain::{
    VehicleDifferentialSettings, VehicleEngineSettings, VehicleTransmissionSettings,
};
pub(crate) use wheel::WheelGeometry;
pub use wheel::{SuspensionSpring, VehicleAntiRollBar, WheelSettings};
/// Jolt's default longitudinal friction curve of a wheel (`WheelSettingsWV`): friction
/// coefficient over longitudinal slip ratio.
pub const DEFAULT_LONGITUDINAL_FRICTION: [(f32, f32); 3] = [(0.0, 0.0), (0.06, 1.2), (0.2, 1.0)];

/// Jolt's default lateral friction curve of a wheel (`WheelSettingsWV`): friction coefficient
/// over slip angle in degrees.
pub const DEFAULT_LATERAL_FRICTION: [(f32, f32); 3] = [(0.0, 0.0), (3.0, 1.2), (20.0, 1.0)];

/// Jolt's default normalized torque curve of an engine (`VehicleEngineSettings`): fraction of
/// the maximum torque over fraction of the maximum rpm.
pub const DEFAULT_NORMALIZED_TORQUE: [(f32, f32); 3] = [(0.0, 0.8), (0.66, 1.0), (1.0, 0.8)];

/// The settings of a wheeled vehicle (Jolt `VehicleConstraintSettings` with a
/// `WheeledVehicleControllerSettings`).
///
/// Directions are in the chassis body's local space. Wheels are indexed in the order given;
/// differentials and anti-roll bars refer to them by index. Jolt's samples, with up +Y and
/// forward +Z, put the left wheels at +X and the right wheels at −X; a differential's
/// `left_wheel` and an anti-roll bar's `left_wheel` name the wheel on the left in that sense.
#[derive(Clone, Debug, PartialEq)]
pub struct VehicleSettings {
    pub(crate) wheels: Vec<WheelSettings>,
    pub(crate) differentials: Vec<VehicleDifferentialSettings>,
    pub(crate) collision_tester: VehicleCollisionTester,
    up: Vec3,
    forward: Vec3,
    max_pitch_roll_angle: f32,
    anti_roll_bars: Vec<VehicleAntiRollBar>,
    engine: VehicleEngineSettings,
    transmission: VehicleTransmissionSettings,
    differential_limited_slip_ratio: f32,
}

impl VehicleSettings {
    /// A vehicle with `wheels`, the engine driving them through `differentials` (at least one
    /// is needed for the vehicle to move), and wheels that find the ground with
    /// `collision_tester`.
    pub fn new(
        wheels: Vec<WheelSettings>,
        differentials: Vec<VehicleDifferentialSettings>,
        collision_tester: VehicleCollisionTester,
    ) -> Self {
        Self {
            wheels,
            differentials,
            collision_tester,
            up: Vec3::new(0.0, 1.0, 0.0),
            forward: Vec3::new(0.0, 0.0, 1.0),
            max_pitch_roll_angle: PI,
            anti_roll_bars: Vec::new(),
            engine: VehicleEngineSettings::default(),
            transmission: VehicleTransmissionSettings::default(),
            differential_limited_slip_ratio: 1.4,
        }
    }

    /// Up of the vehicle in body space, a unit vector. Default +Y.
    #[must_use]
    pub fn up(mut self, value: Vec3) -> Self {
        self.up = value;
        self
    }

    /// Forward of the vehicle in body space, a unit vector perpendicular to
    /// [`up`](Self::up). Default +Z.
    #[must_use]
    pub fn forward(mut self, value: Vec3) -> Self {
        self.forward = value;
        self
    }

    /// Largest angle in radians, in `[0, π]`, between the vehicle's up and the world up (the
    /// opposite of gravity, or the last world up while gravity is zero; see
    /// [`VehicleRef::world_up`](crate::VehicleRef::world_up)) before a constraint keeps it from
    /// tilting further. π, the default, turns the limit off.
    #[must_use]
    pub fn max_pitch_roll_angle(mut self, radians: f32) -> Self {
        self.max_pitch_roll_angle = radians;
        self
    }

    /// Anti-roll bars between pairs of wheels. Default none.
    #[must_use]
    pub fn anti_roll_bars(mut self, value: Vec<VehicleAntiRollBar>) -> Self {
        self.anti_roll_bars = value;
        self
    }

    /// The engine. Default [`VehicleEngineSettings::default`].
    #[must_use]
    pub fn engine(mut self, value: VehicleEngineSettings) -> Self {
        self.engine = value;
        self
    }

    /// The transmission. Default [`VehicleTransmissionSettings::default`].
    #[must_use]
    pub fn transmission(mut self, value: VehicleTransmissionSettings) -> Self {
        self.transmission = value;
        self
    }

    /// Ratio of the fastest to the slowest differential, measured at the clutch, above which
    /// all torque goes to the slowest one: a limited slip between differentials. Above 1;
    /// `f32::MAX` makes it open. Default 1.4.
    #[must_use]
    pub fn differential_limited_slip_ratio(mut self, value: f32) -> Self {
        self.differential_limited_slip_ratio = value;
        self
    }

    /// Checks every value Jolt asserts on or divides by, so none of Jolt's assertions can fire
    /// and no division by a setting can give a non-finite result, and the coefficients Jolt
    /// derives from the settings ([`validate_step_coefficients`](Self::validate_step_coefficients)).
    pub(crate) fn validate(&self, object_layer_count: u32) -> Result<(), VehicleError> {
        let invalid = |what| Err(VehicleError::InvalidValue(what));
        if self.wheels.is_empty() {
            return invalid("a vehicle needs at least one wheel");
        }
        if u32::try_from(self.wheels.len()).is_err() || i32::try_from(self.wheels.len()).is_err() {
            return invalid("too many wheels");
        }
        if !(is_unit(self.up) && is_unit(self.forward)) {
            return invalid("vehicle up and forward must be finite unit vectors");
        }
        if self.up.dot(self.forward).abs() > PERPENDICULAR_TOLERANCE {
            return invalid("vehicle up and forward must be perpendicular");
        }
        if !(is_finite_non_negative(self.max_pitch_roll_angle) && self.max_pitch_roll_angle <= PI) {
            return invalid("max pitch roll angle must be between 0 and pi");
        }
        if !is_limited_slip_ratio(self.differential_limited_slip_ratio) {
            return invalid("differential limited slip ratio must be finite and above 1");
        }
        for wheel in &self.wheels {
            wheel.validate()?;
        }
        self.engine.validate()?;
        self.transmission.validate(&self.engine)?;
        self.validate_differentials()?;
        for bar in &self.anti_roll_bars {
            bar.validate(self.wheels.len())?;
        }
        self.validate_step_coefficients()?;
        self.collision_tester.validate(
            object_layer_count,
            self.wheels.iter().map(WheelGeometry::of),
        )
    }

    fn validate_differentials(&self) -> Result<(), VehicleError> {
        let invalid = |what| Err(VehicleError::InvalidValue(what));
        if self.differentials.is_empty() {
            return invalid("a vehicle needs at least one differential");
        }
        let wheel_count = self.wheels.len();
        let mut torque_ratio_sum = 0.0_f32;
        for differential in &self.differentials {
            differential.validate(wheel_count)?;
            torque_ratio_sum += differential.engine_torque_ratio;
        }
        // Jolt asserts that the ratios, summed in list order, are within 1e-6 of 1 on every
        // step (`WheeledVehicleController::PostCollide`).
        if (torque_ratio_sum - 1.0).abs() >= SUM_TOLERANCE {
            return invalid("differential engine torque ratios must add up to 1");
        }
        Ok(())
    }

    /// Checks that the coefficients the wheeled controller forms from the settings alone
    /// (`WheeledVehicleController::PostCollide`, `VehicleEngine::ApplyTorque`) are finite at
    /// every step [`PhysicsWorld::step`] accepts. Most grow with the step, so the largest step
    /// is their worst case; the brake-lock torque per rad/s of wheel speed, `inertia / step`,
    /// grows as the step shrinks, so the smallest step is its worst case. Values that are valid
    /// one by one can overflow together: a subnormal inertia makes `delta_time / inertia`
    /// infinite, huge gear and differential ratios make their product infinite. What the step
    /// computes from the vehicle's state afterwards is not bounded here.
    fn validate_step_coefficients(&self) -> Result<(), VehicleError> {
        let invalid = |what| Err(VehicleError::InvalidValue(what));
        let dt = PhysicsWorld::MAX_DELTA_TIME;
        let all_finite = |values: &[f32]| values.iter().all(|value| value.is_finite());
        for wheel in &self.wheels {
            let brake_impulse = dt * (wheel.max_brake_torque + wheel.max_hand_brake_torque);
            if !all_finite(&[
                dt / wheel.inertia,
                wheel.inertia / PhysicsWorld::MIN_DELTA_TIME,
                brake_impulse,
                brake_impulse / wheel.inertia,
                brake_impulse / wheel.radius,
                wheel.radius / wheel.inertia,
                wheel.inertia / wheel.radius,
            ]) {
                return invalid(
                    "wheel inertia, radius and brake torques give a non-finite step coefficient",
                );
            }
        }

        let engine = &self.engine;
        let dt_div_ie = dt / engine.inertia;
        let largest_torque_fraction = engine
            .normalized_torque
            .iter()
            .map(|(_, fraction)| fraction.abs())
            .fold(0.0, f32::max);
        let torque = engine.max_torque * largest_torque_fraction;
        if !all_finite(&[
            dt_div_ie,
            torque,
            dt_div_ie * torque,
            ANGULAR_VELOCITY_TO_RPM * torque * dt / engine.inertia,
        ]) {
            return invalid("engine inertia and torque give a non-finite step coefficient");
        }

        // The clutch couples the engine to every driven wheel through the gear ratio times the
        // differential ratio; each wheel gets at most all of the engine torque.
        let transmission = &self.transmission;
        let clutch = transmission.clutch_strength;
        let largest_gear_ratio = transmission
            .gear_ratios
            .iter()
            .chain(&transmission.reverse_gear_ratios)
            .map(|ratio| ratio.abs())
            .fold(0.0, f32::max);
        let clutch_to_wheel_ratio = |differential: &VehicleDifferentialSettings| {
            largest_gear_ratio * differential.differential_ratio
        };
        let largest_clutch_to_wheel_ratio = self
            .differentials
            .iter()
            .map(clutch_to_wheel_ratio)
            .fold(0.0, f32::max);
        if !all_finite(&[1.0 + dt_div_ie * clutch]) {
            return invalid(
                "engine inertia and clutch strength give a non-finite step coefficient",
            );
        }
        for differential in &self.differentials {
            let s_r = clutch * clutch_to_wheel_ratio(differential);
            let driven = [differential.left_wheel, differential.right_wheel];
            for wheel in driven.into_iter().flatten() {
                let dt_s_r_div_iw = dt * s_r / self.wheels[wheel as usize].inertia;
                if !all_finite(&[
                    s_r,
                    dt_div_ie * s_r,
                    dt_s_r_div_iw,
                    dt_s_r_div_iw * largest_clutch_to_wheel_ratio,
                ]) {
                    return invalid(
                        "clutch strength, gear and differential ratios and wheel inertia give a non-finite step coefficient",
                    );
                }
            }
        }
        Ok(())
    }

    /// The joltc wheel settings, one guard per wheel.
    pub(crate) fn create_wheels(&self) -> Vec<Owned<JPH_WheelSettingsWV>> {
        self.wheels.iter().map(WheelSettings::create).collect()
    }

    /// The joltc controller settings: engine, transmission and differentials.
    pub(crate) fn create_controller(&self) -> Owned<JPH_WheeledVehicleControllerSettings> {
        // SAFETY: Jolt is initialised (a world exists before any vehicle settings are built).
        // The settings are returned holding one reference, which the guard takes over.
        let controller = unsafe { Owned::from_raw(JPH_WheeledVehicleControllerSettings_Create()) }
            .unwrap_or_else(|| unreachable!("joltc `new`s the settings"));
        let torque_curve = create_curve(&self.engine.normalized_torque);
        let engine = JPH_VehicleEngineSettings {
            maxTorque: self.engine.max_torque,
            minRPM: self.engine.min_rpm,
            maxRPM: self.engine.max_rpm,
            normalizedTorque: torque_curve.as_ptr(),
            inertia: self.engine.inertia,
            angularDamping: self.engine.angular_damping,
        };
        let transmission = self.transmission.create();
        let differentials: Vec<JPH_VehicleDifferentialSettings> = self
            .differentials
            .iter()
            .map(|differential| differential.to_jph())
            .collect();
        // SAFETY: the controller settings, the curve and the transmission settings are live and
        // owned by their guards; joltc copies the engine (with its curve), the transmission and
        // the differentials into the controller settings. `differentials` holds as many entries
        // as passed and lives for the call. All values were validated.
        unsafe {
            JPH_WheeledVehicleControllerSettings_SetEngine(controller.as_ptr(), &engine);
            JPH_WheeledVehicleControllerSettings_SetTransmission(
                controller.as_ptr(),
                transmission.as_ptr(),
            );
            JPH_WheeledVehicleControllerSettings_SetDifferentials(
                controller.as_ptr(),
                differentials.as_ptr(),
                differentials.len() as u32,
            );
            JPH_WheeledVehicleControllerSettings_SetDifferentialLimitedSlipRatio(
                controller.as_ptr(),
                self.differential_limited_slip_ratio,
            );
        }
        controller
    }

    /// The joltc anti-roll bars.
    pub(crate) fn anti_roll_bars_to_jph(&self) -> Vec<JPH_VehicleAntiRollBar> {
        self.anti_roll_bars
            .iter()
            .map(|bar| JPH_VehicleAntiRollBar {
                leftWheel: bar.left_wheel as i32,
                rightWheel: bar.right_wheel as i32,
                stiffness: bar.stiffness,
            })
            .collect()
    }

    pub(crate) fn up_to_jph(&self) -> JPH_Vec3 {
        self.up.to_jph()
    }

    pub(crate) fn forward_to_jph(&self) -> JPH_Vec3 {
        self.forward.to_jph()
    }

    pub(crate) fn max_pitch_roll_angle_value(&self) -> f32 {
        self.max_pitch_roll_angle
    }
}

/// Jolt's `VehicleEngine::cAngularVelocityToRPM`.
const ANGULAR_VELOCITY_TO_RPM: f32 = 60.0 / (2.0 * PI);

/// Tolerance of the perpendicularity checks on unit vectors, `|a·b|` at most this.
const PERPENDICULAR_TOLERANCE: f32 = 1.0e-3;

/// Tolerance of the engine torque ratio sum, `|sum − 1|` below this, half of Jolt's 1e-6.
const SUM_TOLERANCE: f32 = 5.0e-7;

/// Whether `value` is a valid limited slip ratio: finite and above 1 (`f32::MAX` is open).
fn is_limited_slip_ratio(value: f32) -> bool {
    value.is_finite() && value > 1.0
}

/// Whether `points` make a valid Jolt `LinearCurve`: at least one point, finite, with strictly
/// increasing x.
fn is_curve(points: &[(f32, f32)]) -> bool {
    !points.is_empty()
        && points.iter().all(|(x, y)| x.is_finite() && y.is_finite())
        && points.windows(2).all(|pair| pair[0].0 < pair[1].0)
}

/// A joltc linear curve holding `points`, which [`is_curve`] accepted.
fn create_curve(points: &[(f32, f32)]) -> Owned<JPH_LinearCurve> {
    // SAFETY: Jolt is initialised. joltc `new`s the curve, which the guard owns whole.
    let curve = unsafe { Owned::from_raw(JPH_LinearCurve_Create()) }
        .unwrap_or_else(|| unreachable!("joltc `new`s the curve"));
    for &(x, y) in points {
        // SAFETY: the curve is live and owned by the guard. The points are added in increasing
        // x, the order Jolt's `GetValue` expects.
        unsafe { JPH_LinearCurve_AddPoint(curve.as_ptr(), x, y) };
    }
    curve
}

/// Controller settings, of which the owner holds the one reference
/// `JPH_WheeledVehicleControllerSettings_Create` returns. A vehicle copies them into its own
/// controller when it is created.
impl JoltObject for JPH_WheeledVehicleControllerSettings {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner holds one reference (trait contract), released here once; the
        // wheeled settings derive from `VehicleControllerSettings` with single inheritance.
        unsafe { JPH_VehicleControllerSettings_Destroy(ptr.cast()) };
    }
}

/// A linear curve, owned whole: joltc `new`s it, and wheel and engine settings copy it.
impl JoltObject for JPH_LinearCurve {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner owns the curve (trait contract), which joltc deletes.
        unsafe { JPH_LinearCurve_Destroy(ptr) };
    }
}

#[cfg(test)]
mod tests;
