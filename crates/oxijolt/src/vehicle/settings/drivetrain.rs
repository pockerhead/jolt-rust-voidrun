//! The drivetrain: engine, transmission and differentials.

use oxijolt_sys::*;

use super::{create_curve, is_limited_slip_ratio, ANGULAR_VELOCITY_TO_RPM, LIMITED_SLIP_RULE};
use crate::limits;
use crate::math::{is_finite_non_negative, is_finite_positive};
use crate::owned::{JoltObject, Owned};
use crate::{PhysicsWorld, VehicleError};

/// How far above its largest point Jolt's `LinearCurve::GetValue` may round when it
/// interpolates a torque curve: `1 + 8·2⁻²⁴`.
const CURVE_ROUNDING: f32 = 1.0 + 4.0 * f32::EPSILON;

/// The engine of a vehicle (Jolt `VehicleEngineSettings`). The defaults are Jolt's wheeled
/// vehicle defaults; [`TrackedVehicleSettings::default_engine`](crate::TrackedVehicleSettings::default_engine)
/// gives the tracked ones.
#[derive(Clone, Debug, PartialEq)]
pub struct VehicleEngineSettings {
    pub(super) max_torque: f32,
    pub(super) min_rpm: f32,
    pub(super) max_rpm: f32,
    pub(super) normalized_torque: Vec<(f32, f32)>,
    pub(super) inertia: f32,
    pub(super) angular_damping: f32,
}

impl Default for VehicleEngineSettings {
    fn default() -> Self {
        Self {
            max_torque: 500.0,
            min_rpm: 1000.0,
            max_rpm: 6000.0,
            normalized_torque: Self::DEFAULT_NORMALIZED_TORQUE.to_vec(),
            inertia: 0.5,
            angular_damping: 0.2,
        }
    }
}

impl VehicleEngineSettings {
    /// Jolt's default normalized torque curve of an engine (`VehicleEngineSettings`): fraction
    /// of the maximum torque over fraction of the maximum rpm.
    pub const DEFAULT_NORMALIZED_TORQUE: [(f32, f32); 3] = [(0.0, 0.8), (0.66, 1.0), (1.0, 0.8)];

    /// Largest torque the engine delivers, N·m, not negative. Default 500.
    #[must_use]
    pub fn max_torque(mut self, value: f32) -> Self {
        self.max_torque = value;
        self
    }

    /// Lowest rpm, at which the engine idles, not negative. Default 1000.
    #[must_use]
    pub fn min_rpm(mut self, value: f32) -> Self {
        self.min_rpm = value;
        self
    }

    /// Highest rpm, positive and at least the lowest. Default 6000.
    #[must_use]
    pub fn max_rpm(mut self, value: f32) -> Self {
        self.max_rpm = value;
        self
    }

    /// Fraction of the maximum torque over fraction of the maximum rpm, as `(rpm fraction,
    /// torque fraction)` points: x within `0..=1`, increasing by at least
    /// [`limits::MIN_TORQUE_CURVE_SPACING`]; y within `0..=`[`limits::MAX_NORMALIZED_TORQUE`].
    /// Jolt reads the curve at the current rpm over the max rpm, between `min_rpm / max_rpm` and
    /// 1. Default [`DEFAULT_NORMALIZED_TORQUE`](Self::DEFAULT_NORMALIZED_TORQUE).
    #[must_use]
    pub fn normalized_torque(mut self, points: Vec<(f32, f32)>) -> Self {
        self.normalized_torque = points;
        self
    }

    /// Moment of inertia of the engine, kg·m², positive. Default 0.5.
    #[must_use]
    pub fn inertia(mut self, value: f32) -> Self {
        self.inertia = value;
        self
    }

    /// Angular damping of the engine, `dω/dt = −c·ω`, not negative. Default 0.2.
    #[must_use]
    pub fn angular_damping(mut self, value: f32) -> Self {
        self.angular_damping = value;
        self
    }

    pub(super) fn validate(&self) -> Result<(), VehicleError> {
        let invalid = |what| Err(VehicleError::InvalidValue(what));
        if !is_finite_non_negative(self.max_torque) {
            return invalid("engine max torque must be finite and not negative");
        }
        if !is_finite_non_negative(self.min_rpm) {
            return invalid("engine min rpm must be finite and not negative");
        }
        if !(is_finite_positive(self.max_rpm) && self.max_rpm >= self.min_rpm) {
            return invalid("engine max rpm must be finite, positive and at least the min rpm");
        }
        // Jolt divides by the engine inertia (`VehicleEngine::ApplyTorque`,
        // `WheeledVehicleController::PostCollide`).
        if !is_finite_positive(self.inertia) {
            return invalid("engine inertia must be finite and positive");
        }
        if !is_finite_non_negative(self.angular_damping) {
            return invalid("engine angular damping must be finite and not negative");
        }
        if !limits::is_torque_curve(&self.normalized_torque) {
            return invalid(limits::TORQUE_CURVE_RULE);
        }
        Ok(())
    }

    /// The largest torque these validated settings let the engine deliver, N·m: the max torque
    /// times the largest torque fraction of the curve, with the rounding of Jolt's
    /// interpolation; see [docs/limits.md#torque-curves].
    ///
    /// [docs/limits.md#torque-curves]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#torque-curves
    pub(super) fn largest_torque(&self) -> f32 {
        let largest_torque_fraction = self
            .normalized_torque
            .iter()
            .map(|&(_, fraction)| fraction)
            .fold(0.0, f32::max);
        self.max_torque * largest_torque_fraction * CURVE_ROUNDING
    }

    /// Checks that the coefficients `VehicleEngine::ApplyTorque` forms from these validated
    /// settings are finite at the largest step.
    pub(super) fn validate_step_coefficients(&self) -> Result<(), VehicleError> {
        let dt = PhysicsWorld::MAX_DELTA_TIME;
        let dt_div_ie = dt / self.inertia;
        let torque = self.largest_torque();
        let coefficients = [
            dt_div_ie,
            torque,
            dt_div_ie * torque,
            ANGULAR_VELOCITY_TO_RPM * torque * dt / self.inertia,
        ];
        if coefficients.iter().all(|value| value.is_finite()) {
            Ok(())
        } else {
            Err(VehicleError::InvalidValue(
                "engine inertia and torque give a non-finite step coefficient",
            ))
        }
    }

    /// Calls `f` with the joltc engine settings of these validated settings; their torque curve
    /// lives for the call.
    pub(super) fn with_jph<R>(&self, f: impl FnOnce(&JPH_VehicleEngineSettings) -> R) -> R {
        let torque_curve = create_curve(&self.normalized_torque);
        f(&JPH_VehicleEngineSettings {
            maxTorque: self.max_torque,
            minRPM: self.min_rpm,
            maxRPM: self.max_rpm,
            normalizedTorque: torque_curve.as_ptr(),
            inertia: self.inertia,
            angularDamping: self.angular_damping,
        })
    }
}

/// The automatic transmission of a vehicle (Jolt `VehicleTransmissionSettings` in
/// `ETransmissionMode::Auto`; the manual mode is not offered). The defaults are Jolt's wheeled
/// vehicle defaults;
/// [`TrackedVehicleSettings::default_transmission`](crate::TrackedVehicleSettings::default_transmission)
/// gives the tracked ones.
#[derive(Clone, Debug, PartialEq)]
pub struct VehicleTransmissionSettings {
    pub(super) gear_ratios: Vec<f32>,
    pub(super) reverse_gear_ratios: Vec<f32>,
    pub(super) switch_time: f32,
    pub(super) clutch_release_time: f32,
    pub(super) switch_latency: f32,
    pub(super) shift_up_rpm: f32,
    pub(super) shift_down_rpm: f32,
    pub(super) clutch_strength: f32,
}

impl Default for VehicleTransmissionSettings {
    fn default() -> Self {
        Self {
            gear_ratios: vec![2.66, 1.78, 1.3, 1.0, 0.74],
            reverse_gear_ratios: vec![-2.9],
            switch_time: 0.5,
            clutch_release_time: 0.3,
            switch_latency: 0.5,
            shift_up_rpm: 4000.0,
            shift_down_rpm: 2000.0,
            clutch_strength: 10.0,
        }
    }
}

impl VehicleTransmissionSettings {
    /// Engine to gearbox rotation ratios of the forward gears, first gear first; at least one,
    /// each positive. Default `[2.66, 1.78, 1.3, 1.0, 0.74]`.
    #[must_use]
    pub fn gear_ratios(mut self, value: Vec<f32>) -> Self {
        self.gear_ratios = value;
        self
    }

    /// Ratios of the reverse gears; at least one, each negative. Default `[-2.9]`.
    #[must_use]
    pub fn reverse_gear_ratios(mut self, value: Vec<f32>) -> Self {
        self.reverse_gear_ratios = value;
        self
    }

    /// Seconds a gear switch takes, not negative. Default 0.5.
    #[must_use]
    pub fn switch_time(mut self, value: f32) -> Self {
        self.switch_time = value;
        self
    }

    /// Seconds the clutch takes to engage fully after a switch, not negative. Default 0.3.
    #[must_use]
    pub fn clutch_release_time(mut self, value: f32) -> Self {
        self.clutch_release_time = value;
        self
    }

    /// Seconds to wait after the clutch engaged before another switch, not negative. Default
    /// 0.5.
    #[must_use]
    pub fn switch_latency(mut self, value: f32) -> Self {
        self.switch_latency = value;
        self
    }

    /// Engine rpm above which the transmission shifts up; above the shift down rpm and below
    /// the engine's max rpm. Default 4000.
    #[must_use]
    pub fn shift_up_rpm(mut self, value: f32) -> Self {
        self.shift_up_rpm = value;
        self
    }

    /// Engine rpm below which the transmission shifts down, positive. Default 2000.
    #[must_use]
    pub fn shift_down_rpm(mut self, value: f32) -> Self {
        self.shift_down_rpm = value;
        self
    }

    /// Strength of the fully engaged clutch, kg·m²/s, positive. Default 10.
    #[must_use]
    pub fn clutch_strength(mut self, value: f32) -> Self {
        self.clutch_strength = value;
        self
    }

    pub(super) fn validate(&self, engine: &VehicleEngineSettings) -> Result<(), VehicleError> {
        let invalid = |what| Err(VehicleError::InvalidValue(what));
        // Jolt indexes gear 1 and gear -1 without a check
        // (`VehicleTransmission::GetCurrentRatio`).
        if self.gear_ratios.is_empty() || !self.gear_ratios.iter().all(|&r| is_finite_positive(r)) {
            return invalid("gear ratios need at least one gear, each finite and positive");
        }
        if self.reverse_gear_ratios.is_empty()
            || !self
                .reverse_gear_ratios
                .iter()
                .all(|&r| r.is_finite() && r < 0.0)
        {
            return invalid("reverse gear ratios need at least one gear, each finite and negative");
        }
        if u32::try_from(self.gear_ratios.len()).is_err()
            || u32::try_from(self.reverse_gear_ratios.len()).is_err()
        {
            return invalid("too many gears");
        }
        let times = [
            self.switch_time,
            self.clutch_release_time,
            self.switch_latency,
        ];
        if !times.into_iter().all(is_finite_non_negative) {
            return invalid("transmission times must be finite and not negative");
        }
        if !is_finite_positive(self.shift_down_rpm) {
            return invalid("shift down rpm must be finite and positive");
        }
        if !(self.shift_up_rpm.is_finite() && self.shift_up_rpm > self.shift_down_rpm) {
            return invalid("shift up rpm must be finite and above the shift down rpm");
        }
        if self.shift_up_rpm >= engine.max_rpm {
            return invalid("shift up rpm must be below the engine's max rpm");
        }
        if !is_finite_positive(self.clutch_strength) {
            return invalid("clutch strength must be finite and positive");
        }
        Ok(())
    }

    /// The joltc settings of this validated transmission.
    pub(super) fn create(&self) -> Owned<JPH_VehicleTransmissionSettings> {
        // SAFETY: Jolt is initialised (a world exists). joltc `new`s the settings, which the
        // guard owns whole.
        let settings = unsafe { Owned::from_raw(JPH_VehicleTransmissionSettings_Create()) }
            .unwrap_or_else(|| unreachable!("joltc `new`s the settings"));
        let ptr = settings.as_ptr();
        // SAFETY: the settings are live and owned by the guard; the gear slices live for the
        // calls and hold as many ratios as passed (`validate` bounds the counts), which joltc
        // copies. All values were validated.
        unsafe {
            JPH_VehicleTransmissionSettings_SetMode(ptr, JPH_TransmissionMode_Auto);
            JPH_VehicleTransmissionSettings_SetGearRatios(
                ptr,
                self.gear_ratios.as_ptr(),
                self.gear_ratios.len() as u32,
            );
            JPH_VehicleTransmissionSettings_SetReverseGearRatios(
                ptr,
                self.reverse_gear_ratios.as_ptr(),
                self.reverse_gear_ratios.len() as u32,
            );
            JPH_VehicleTransmissionSettings_SetSwitchTime(ptr, self.switch_time);
            JPH_VehicleTransmissionSettings_SetClutchReleaseTime(ptr, self.clutch_release_time);
            JPH_VehicleTransmissionSettings_SetSwitchLatency(ptr, self.switch_latency);
            JPH_VehicleTransmissionSettings_SetShiftUpRPM(ptr, self.shift_up_rpm);
            JPH_VehicleTransmissionSettings_SetShiftDownRPM(ptr, self.shift_down_rpm);
            JPH_VehicleTransmissionSettings_SetClutchStrength(ptr, self.clutch_strength);
        }
        settings
    }
}

/// A differential: how engine torque reaches a pair of wheels (Jolt
/// `VehicleDifferentialSettings`). The defaults are Jolt's.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VehicleDifferentialSettings {
    pub(super) left_wheel: Option<u32>,
    pub(super) right_wheel: Option<u32>,
    pub(super) differential_ratio: f32,
    left_right_split: f32,
    limited_slip_ratio: f32,
    pub(super) engine_torque_ratio: f32,
}

impl VehicleDifferentialSettings {
    /// A differential driving the wheels with these indices; `None` for a side without a wheel,
    /// but at least one side must have one.
    pub fn new(left_wheel: Option<u32>, right_wheel: Option<u32>) -> Self {
        Self {
            left_wheel,
            right_wheel,
            differential_ratio: 3.42,
            left_right_split: 0.5,
            limited_slip_ratio: 1.4,
            engine_torque_ratio: 1.0,
        }
    }

    /// Ratio between the gearbox and wheel rotation rates, positive. Default 3.42.
    #[must_use]
    pub fn differential_ratio(mut self, value: f32) -> Self {
        self.differential_ratio = value;
        self
    }

    /// How torque is split between the wheels: 0 all left, 1 all right, in `[0, 1]`. Default
    /// 0.5.
    #[must_use]
    pub fn left_right_split(mut self, value: f32) -> Self {
        self.left_right_split = value;
        self
    }

    /// Ratio of the faster to the slower wheel above which all torque goes to the slower one;
    /// above 1, `f32::MAX` for an open differential. Default 1.4.
    #[must_use]
    pub fn limited_slip_ratio(mut self, value: f32) -> Self {
        self.limited_slip_ratio = value;
        self
    }

    /// Fraction of the engine torque this differential gets, not negative. The fractions of
    /// all differentials must add up to 1. Default 1.
    #[must_use]
    pub fn engine_torque_ratio(mut self, value: f32) -> Self {
        self.engine_torque_ratio = value;
        self
    }

    pub(super) fn validate(&self, wheel_count: usize) -> Result<(), VehicleError> {
        let invalid = |what| Err(VehicleError::InvalidValue(what));
        let wheel_exists = |index: Option<u32>| index.is_none_or(|i| (i as usize) < wheel_count);
        if !(wheel_exists(self.left_wheel) && wheel_exists(self.right_wheel)) {
            return invalid("differential wheel index out of range");
        }
        if self.left_wheel.is_none() && self.right_wheel.is_none() {
            return invalid("a differential needs at least one wheel");
        }
        if !is_finite_positive(self.differential_ratio) {
            return invalid("differential ratio must be finite and positive");
        }
        if !(self.left_right_split.is_finite() && (0.0..=1.0).contains(&self.left_right_split)) {
            return invalid("differential left right split must be between 0 and 1");
        }
        if !is_limited_slip_ratio(self.limited_slip_ratio) {
            return invalid(LIMITED_SLIP_RULE);
        }
        if !is_finite_non_negative(self.engine_torque_ratio) {
            return invalid("differential engine torque ratio must be finite and not negative");
        }
        Ok(())
    }

    /// The joltc value of this validated differential; the wheel indices fit in an `i32`
    /// because the wheel count does.
    pub(super) fn to_jph(self) -> JPH_VehicleDifferentialSettings {
        let index = |wheel: Option<u32>| wheel.map_or(-1, |i| i as i32);
        JPH_VehicleDifferentialSettings {
            leftWheel: index(self.left_wheel),
            rightWheel: index(self.right_wheel),
            differentialRatio: self.differential_ratio,
            leftRightSplit: self.left_right_split,
            limitedSlipRatio: self.limited_slip_ratio,
            engineTorqueRatio: self.engine_torque_ratio,
        }
    }
}

/// Transmission settings, owned whole: joltc `new`s them, and controller settings copy them.
impl JoltObject for JPH_VehicleTransmissionSettings {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner owns the settings (trait contract), which joltc deletes.
        unsafe { JPH_VehicleTransmissionSettings_Destroy(ptr) };
    }
}
