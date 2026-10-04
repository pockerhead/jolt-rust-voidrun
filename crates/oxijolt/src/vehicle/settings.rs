//! Settings of a wheeled vehicle, in Jolt's vocabulary, with Jolt's defaults.
//!
//! The types here are plain Rust values. [`PhysicsWorld::create_vehicle`] validates them and
//! builds the joltc settings objects from them, which it releases again once the vehicle exists.
//!
//! [`PhysicsWorld::create_vehicle`]: crate::PhysicsWorld::create_vehicle

use std::f32::consts::PI;

use oxijolt_sys::*;

use crate::limits::{self, is_local_distance, is_local_offset};
use crate::math::{is_finite_non_negative, is_finite_positive, is_unit};
use crate::owned::{JoltObject, Owned};
use crate::{ObjectLayer, PhysicsWorld, SpringSettings, Vec3, VehicleError};

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

/// How a wheel's suspension spring is specified (Jolt `SpringSettings`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SuspensionSpring {
    /// Oscillation frequency in Hz (finite, positive) and damping ratio (finite, not
    /// negative; 1 is critical damping). Independent of the chassis mass.
    FrequencyAndDamping {
        /// Frequency in Hz.
        frequency: f32,
        /// Damping ratio.
        damping: f32,
    },
    /// Spring stiffness in N/m (finite, positive) and damping in N·s/m (finite, not negative).
    StiffnessAndDamping {
        /// Stiffness in N/m.
        stiffness: f32,
        /// Damping in N·s/m.
        damping: f32,
    },
}

impl Default for SuspensionSpring {
    /// Jolt's default: 1.5 Hz with damping ratio 0.5.
    fn default() -> Self {
        Self::FrequencyAndDamping {
            frequency: 1.5,
            damping: 0.5,
        }
    }
}

impl SuspensionSpring {
    fn to_jph(self) -> JPH_SpringSettings {
        match self {
            Self::FrequencyAndDamping { frequency, damping } => JPH_SpringSettings {
                mode: JPH_SpringMode_FrequencyAndDamping,
                frequencyOrStiffness: frequency,
                damping,
            },
            Self::StiffnessAndDamping { stiffness, damping } => JPH_SpringSettings {
                mode: JPH_SpringMode_StiffnessAndDamping,
                frequencyOrStiffness: stiffness,
                damping,
            },
        }
    }

    fn is_valid(self) -> bool {
        let (value, damping) = match self {
            Self::FrequencyAndDamping { frequency, damping } => (frequency, damping),
            Self::StiffnessAndDamping { stiffness, damping } => (stiffness, damping),
        };
        is_finite_positive(value) && is_finite_non_negative(damping)
    }

    /// Whether the stiffness and damping Jolt uses for this valid spring stay at most
    /// [`limits::MAX_SPRING_COEFFICIENT`]. A stiffness-mode spring is used as given. In
    /// frequency mode Jolt multiplies by the suspension's effective mass
    /// (`VehicleConstraint::SetupVelocityConstraint`), which is at most the chassis mass and so
    /// at most [`limits::MAX_MASS`].
    fn fits_the_coefficient_bound(self) -> bool {
        match self {
            Self::FrequencyAndDamping { frequency, damping } => {
                SpringSettings::FrequencyAndDamping { frequency, damping }
                    .fits_effective_mass(f64::from(limits::MAX_MASS))
            }
            Self::StiffnessAndDamping { stiffness, damping } => {
                stiffness <= limits::MAX_SPRING_COEFFICIENT
                    && damping <= limits::MAX_SPRING_COEFFICIENT
            }
        }
    }
}

/// One wheel of a wheeled vehicle (Jolt `WheelSettings` and `WheelSettingsWV`). Positions and
/// directions are in the chassis body's local space, lengths in metres. The defaults are
/// Jolt's.
#[derive(Clone, Debug, PartialEq)]
pub struct WheelSettings {
    position: Vec3,
    suspension_force_point: Option<Vec3>,
    suspension_direction: Vec3,
    steering_axis: Vec3,
    wheel_up: Vec3,
    wheel_forward: Vec3,
    suspension_min_length: f32,
    suspension_max_length: f32,
    suspension_preload_length: f32,
    suspension_spring: SuspensionSpring,
    radius: f32,
    width: f32,
    inertia: f32,
    angular_damping: f32,
    max_steer_angle: f32,
    longitudinal_friction: Vec<(f32, f32)>,
    lateral_friction: Vec<(f32, f32)>,
    max_brake_torque: f32,
    max_hand_brake_torque: f32,
}

impl WheelSettings {
    /// A wheel whose suspension is attached to the chassis at `position` (body space, each
    /// component within [`limits::MAX_SHAPE_EXTENT`]).
    pub fn new(position: Vec3) -> Self {
        Self {
            position,
            suspension_force_point: None,
            suspension_direction: Vec3::new(0.0, -1.0, 0.0),
            steering_axis: Vec3::new(0.0, 1.0, 0.0),
            wheel_up: Vec3::new(0.0, 1.0, 0.0),
            wheel_forward: Vec3::new(0.0, 0.0, 1.0),
            suspension_min_length: 0.3,
            suspension_max_length: 0.5,
            suspension_preload_length: 0.0,
            suspension_spring: SuspensionSpring::default(),
            radius: 0.3,
            width: 0.1,
            inertia: 0.9,
            angular_damping: 0.2,
            max_steer_angle: 70.0_f32.to_radians(),
            longitudinal_friction: DEFAULT_LONGITUDINAL_FRICTION.to_vec(),
            lateral_friction: DEFAULT_LATERAL_FRICTION.to_vec(),
            max_brake_torque: 1500.0,
            max_hand_brake_torque: 4000.0,
        }
    }

    /// Where suspension and tire forces act on the chassis (body space, each component within
    /// [`limits::MAX_SHAPE_EXTENT`]). `None`, the
    /// default, applies them at the contact point, which is more accurate against dynamic
    /// ground but less stable (Jolt `mEnableSuspensionForcePoint`).
    #[must_use]
    pub fn suspension_force_point(mut self, value: Option<Vec3>) -> Self {
        self.suspension_force_point = value;
        self
    }

    /// Direction the suspension extends in, a unit vector pointing down. Default −Y.
    #[must_use]
    pub fn suspension_direction(mut self, value: Vec3) -> Self {
        self.suspension_direction = value;
        self
    }

    /// Axis the wheel steers about, a unit vector pointing up. Default +Y.
    #[must_use]
    pub fn steering_axis(mut self, value: Vec3) -> Self {
        self.steering_axis = value;
        self
    }

    /// Up of the wheel in the neutral steering position, a unit vector; tilt it for camber.
    /// Default +Y.
    #[must_use]
    pub fn wheel_up(mut self, value: Vec3) -> Self {
        self.wheel_up = value;
        self
    }

    /// Forward of the wheel in the neutral steering position, a unit vector perpendicular to
    /// [`wheel_up`](Self::wheel_up); turn it for toe. Default +Z.
    #[must_use]
    pub fn wheel_forward(mut self, value: Vec3) -> Self {
        self.wheel_forward = value;
        self
    }

    /// Suspension length when fully raised, from the attachment point, between 0 and
    /// [`limits::MAX_SHAPE_EXTENT`]. Default 0.3.
    #[must_use]
    pub fn suspension_min_length(mut self, value: f32) -> Self {
        self.suspension_min_length = value;
        self
    }

    /// Suspension length when fully extended, at least the minimum length and at most
    /// [`limits::MAX_SHAPE_EXTENT`]. Default 0.5.
    #[must_use]
    pub fn suspension_max_length(mut self, value: f32) -> Self {
        self.suspension_max_length = value;
        self
    }

    /// How far the spring is already compressed at full extension, between 0 and
    /// [`limits::MAX_SHAPE_EXTENT`]. Default 0.
    #[must_use]
    pub fn suspension_preload_length(mut self, value: f32) -> Self {
        self.suspension_preload_length = value;
        self
    }

    /// The suspension spring. Default [`SuspensionSpring::default`]. The stiffness and damping
    /// Jolt derives from it must stay at most [`limits::MAX_SPRING_COEFFICIENT`] for a chassis of
    /// [`limits::MAX_MASS`]: in frequency mode `MAX_MASS·ω²` and `2·MAX_MASS·ζ·ω` with
    /// `ω = 2π·frequency`, in stiffness mode the values themselves.
    #[must_use]
    pub fn suspension_spring(mut self, value: SuspensionSpring) -> Self {
        self.suspension_spring = value;
        self
    }

    /// Wheel radius, positive and at most [`limits::MAX_SHAPE_EXTENT`]. Default 0.3.
    #[must_use]
    pub fn radius(mut self, value: f32) -> Self {
        self.radius = value;
        self
    }

    /// Wheel width, between 0 and [`limits::MAX_SHAPE_EXTENT`]; positive with
    /// [`VehicleCollisionTester::CastCylinder`]. Default 0.1.
    #[must_use]
    pub fn width(mut self, value: f32) -> Self {
        self.width = value;
        self
    }

    /// Moment of inertia of the wheel about its axle, kg·m², positive. Default 0.9.
    #[must_use]
    pub fn inertia(mut self, value: f32) -> Self {
        self.inertia = value;
        self
    }

    /// Angular damping of the wheel, `dω/dt = −c·ω`, not negative. Default 0.2.
    #[must_use]
    pub fn angular_damping(mut self, value: f32) -> Self {
        self.angular_damping = value;
        self
    }

    /// Largest steering angle in radians, at most π/2 in magnitude; 0 for wheels that do not
    /// steer. Default 70°.
    #[must_use]
    pub fn max_steer_angle(mut self, radians: f32) -> Self {
        self.max_steer_angle = radians;
        self
    }

    /// Friction coefficient over longitudinal slip ratio, as `(slip, friction)` points with
    /// strictly increasing slip. Default [`DEFAULT_LONGITUDINAL_FRICTION`].
    #[must_use]
    pub fn longitudinal_friction(mut self, points: Vec<(f32, f32)>) -> Self {
        self.longitudinal_friction = points;
        self
    }

    /// Friction coefficient over slip angle in degrees, as `(angle, friction)` points with
    /// strictly increasing angle. Default [`DEFAULT_LATERAL_FRICTION`].
    #[must_use]
    pub fn lateral_friction(mut self, points: Vec<(f32, f32)>) -> Self {
        self.lateral_friction = points;
        self
    }

    /// Largest brake torque, N·m, not negative. Default 1500.
    #[must_use]
    pub fn max_brake_torque(mut self, value: f32) -> Self {
        self.max_brake_torque = value;
        self
    }

    /// Largest hand brake torque, N·m, not negative; usually 0 on the front wheels. Default
    /// 4000.
    #[must_use]
    pub fn max_hand_brake_torque(mut self, value: f32) -> Self {
        self.max_hand_brake_torque = value;
        self
    }

    fn validate(&self) -> Result<(), VehicleError> {
        let invalid = |what| Err(VehicleError::InvalidValue(what));
        if !is_local_offset(self.position) {
            return invalid("wheel position must be finite and within limits::MAX_SHAPE_EXTENT");
        }
        if !self.suspension_force_point.is_none_or(is_local_offset) {
            return invalid(
                "wheel suspension force point must be finite and within limits::MAX_SHAPE_EXTENT",
            );
        }
        let directions = [
            self.suspension_direction,
            self.steering_axis,
            self.wheel_up,
            self.wheel_forward,
        ];
        if !directions.into_iter().all(is_unit) {
            return invalid("wheel directions must be finite unit vectors");
        }
        if self.wheel_up.dot(self.wheel_forward).abs() > PERPENDICULAR_TOLERANCE {
            return invalid("wheel up and wheel forward must be perpendicular");
        }
        if !is_local_distance(self.suspension_min_length) {
            return invalid(
                "suspension min length must be finite and between 0 and limits::MAX_SHAPE_EXTENT",
            );
        }
        if !(is_local_distance(self.suspension_max_length)
            && self.suspension_max_length >= self.suspension_min_length)
        {
            return invalid(
                "suspension max length must be at least the min length and at most limits::MAX_SHAPE_EXTENT",
            );
        }
        if !is_local_distance(self.suspension_preload_length) {
            return invalid(
                "suspension preload length must be finite and between 0 and limits::MAX_SHAPE_EXTENT",
            );
        }
        if !self.suspension_spring.is_valid() {
            return invalid(
                "suspension spring frequency or stiffness must be positive, damping not negative",
            );
        }
        if !self.suspension_spring.fits_the_coefficient_bound() {
            return invalid(
                "suspension spring stiffness or damping exceeds limits::MAX_SPRING_COEFFICIENT",
            );
        }
        if !(self.radius > 0.0 && is_local_distance(self.radius)) {
            return invalid("wheel radius must be positive and at most limits::MAX_SHAPE_EXTENT");
        }
        if !is_local_distance(self.width) {
            return invalid(
                "wheel width must be finite and between 0 and limits::MAX_SHAPE_EXTENT",
            );
        }
        // Jolt asserts only `>= 0` but divides by the inertia
        // (`WheeledVehicleController::PostCollide`).
        if !is_finite_positive(self.inertia) {
            return invalid("wheel inertia must be finite and positive");
        }
        if !is_finite_non_negative(self.angular_damping) {
            return invalid("wheel angular damping must be finite and not negative");
        }
        if !(self.max_steer_angle.is_finite() && self.max_steer_angle.abs() <= 0.5 * PI) {
            return invalid("max steer angle must be at most pi/2 in magnitude");
        }
        if !(is_finite_non_negative(self.max_brake_torque)
            && is_finite_non_negative(self.max_hand_brake_torque))
        {
            return invalid("brake torques must be finite and not negative");
        }
        if !(is_curve(&self.longitudinal_friction) && is_curve(&self.lateral_friction)) {
            return invalid(
                "friction curves need at least one finite point and strictly increasing x",
            );
        }
        Ok(())
    }

    /// The joltc settings of this validated wheel.
    fn create(&self) -> Owned<JPH_WheelSettingsWV> {
        // SAFETY: Jolt is initialised (a world exists). The settings are returned holding one
        // reference, which the guard takes over.
        let wheel = unsafe { Owned::from_raw(JPH_WheelSettingsWV_Create()) }
            .unwrap_or_else(|| unreachable!("joltc `new`s the settings"));
        let longitudinal = create_curve(&self.longitudinal_friction);
        let lateral = create_curve(&self.lateral_friction);
        let ptr = wheel.as_ptr();
        let base: *mut JPH_WheelSettings = ptr.cast();
        let position = self.position.to_jph();
        let force_point = self.suspension_force_point.unwrap_or(Vec3::ZERO).to_jph();
        let suspension_direction = self.suspension_direction.to_jph();
        let steering_axis = self.steering_axis.to_jph();
        let wheel_up = self.wheel_up.to_jph();
        let wheel_forward = self.wheel_forward.to_jph();
        let mut spring = self.suspension_spring.to_jph();
        // SAFETY: the settings and both curves are live and owned by their guards; a
        // `WheelSettingsWV` derives from `WheelSettings` with single inheritance, joltc's own
        // cast convention. The setters copy the vectors, the spring and the curves. All values
        // were validated.
        unsafe {
            JPH_WheelSettings_SetPosition(base, &position);
            JPH_WheelSettings_SetSuspensionForcePoint(base, &force_point);
            JPH_WheelSettings_SetEnableSuspensionForcePoint(
                base,
                self.suspension_force_point.is_some(),
            );
            JPH_WheelSettings_SetSuspensionDirection(base, &suspension_direction);
            JPH_WheelSettings_SetSteeringAxis(base, &steering_axis);
            JPH_WheelSettings_SetWheelUp(base, &wheel_up);
            JPH_WheelSettings_SetWheelForward(base, &wheel_forward);
            JPH_WheelSettings_SetSuspensionMinLength(base, self.suspension_min_length);
            JPH_WheelSettings_SetSuspensionMaxLength(base, self.suspension_max_length);
            JPH_WheelSettings_SetSuspensionPreloadLength(base, self.suspension_preload_length);
            JPH_WheelSettings_SetSuspensionSpring(base, &mut spring);
            JPH_WheelSettings_SetRadius(base, self.radius);
            JPH_WheelSettings_SetWidth(base, self.width);
            JPH_WheelSettingsWV_SetInertia(ptr, self.inertia);
            JPH_WheelSettingsWV_SetAngularDamping(ptr, self.angular_damping);
            JPH_WheelSettingsWV_SetMaxSteerAngle(ptr, self.max_steer_angle);
            JPH_WheelSettingsWV_SetLongitudinalFriction(ptr, longitudinal.as_ptr());
            JPH_WheelSettingsWV_SetLateralFriction(ptr, lateral.as_ptr());
            JPH_WheelSettingsWV_SetMaxBrakeTorque(ptr, self.max_brake_torque);
            JPH_WheelSettingsWV_SetMaxHandBrakeTorque(ptr, self.max_hand_brake_torque);
        }
        wheel
    }
}

/// What a collision tester needs to know about a wheel.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct WheelGeometry {
    suspension_max_length: f32,
    radius: f32,
    width: f32,
}

impl WheelGeometry {
    pub(crate) fn of(wheel: &WheelSettings) -> Self {
        Self {
            suspension_max_length: wheel.suspension_max_length,
            radius: wheel.radius,
            width: wheel.width,
        }
    }
}

/// The engine of a wheeled vehicle (Jolt `VehicleEngineSettings`). The defaults are Jolt's.
#[derive(Clone, Debug, PartialEq)]
pub struct VehicleEngineSettings {
    max_torque: f32,
    min_rpm: f32,
    max_rpm: f32,
    normalized_torque: Vec<(f32, f32)>,
    inertia: f32,
    angular_damping: f32,
}

impl Default for VehicleEngineSettings {
    fn default() -> Self {
        Self {
            max_torque: 500.0,
            min_rpm: 1000.0,
            max_rpm: 6000.0,
            normalized_torque: DEFAULT_NORMALIZED_TORQUE.to_vec(),
            inertia: 0.5,
            angular_damping: 0.2,
        }
    }
}

impl VehicleEngineSettings {
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
    /// torque fraction)` points with strictly increasing x. Default
    /// [`DEFAULT_NORMALIZED_TORQUE`].
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

    fn validate(&self) -> Result<(), VehicleError> {
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
        if !is_curve(&self.normalized_torque) {
            return invalid(
                "engine torque curve needs at least one finite point and strictly increasing x",
            );
        }
        Ok(())
    }
}

/// The automatic transmission of a wheeled vehicle (Jolt `VehicleTransmissionSettings` in
/// `ETransmissionMode::Auto`; the manual mode is not offered). The defaults are Jolt's.
#[derive(Clone, Debug, PartialEq)]
pub struct VehicleTransmissionSettings {
    gear_ratios: Vec<f32>,
    reverse_gear_ratios: Vec<f32>,
    switch_time: f32,
    clutch_release_time: f32,
    switch_latency: f32,
    shift_up_rpm: f32,
    shift_down_rpm: f32,
    clutch_strength: f32,
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

    fn validate(&self, engine: &VehicleEngineSettings) -> Result<(), VehicleError> {
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
    fn create(&self) -> Owned<JPH_VehicleTransmissionSettings> {
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
    left_wheel: Option<u32>,
    right_wheel: Option<u32>,
    differential_ratio: f32,
    left_right_split: f32,
    limited_slip_ratio: f32,
    engine_torque_ratio: f32,
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

    fn validate(&self, wheel_count: usize) -> Result<(), VehicleError> {
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
            return invalid("differential limited slip ratio must be finite and above 1");
        }
        if !is_finite_non_negative(self.engine_torque_ratio) {
            return invalid("differential engine torque ratio must be finite and not negative");
        }
        Ok(())
    }

    /// The joltc value of this validated differential; the wheel indices fit in an `i32`
    /// because the wheel count does.
    fn to_jph(self) -> JPH_VehicleDifferentialSettings {
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

/// An anti-roll bar between two wheels (Jolt `VehicleAntiRollBar`): pushes the less compressed
/// suspension down when the other is compressed more.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VehicleAntiRollBar {
    left_wheel: u32,
    right_wheel: u32,
    stiffness: f32,
}

impl VehicleAntiRollBar {
    /// Largest anti-roll bar stiffness, N/m: about 2.5e11.
    ///
    /// Each step Jolt computes `stiffness · suspension length difference · dt`
    /// (`VehicleConstraint::OnStep`) and uses it as the velocity bias of both wheels'
    /// suspension constraints, which the constraint's effective mass turns into an impulse.
    /// With a length difference of at most [`limits::MAX_SHAPE_EXTENT`] this bound keeps the
    /// bias at most about 5e14 for `dt <= 1`, so the velocity change it gives stays finite;
    /// see [`limits`](crate::limits#derived-bounds).
    pub const MAX_STIFFNESS: f32 =
        limits::MAX_ACCELERATION * limits::MAX_MASS / limits::MAX_SHAPE_EXTENT;

    /// A bar between two different wheels, by index.
    pub fn new(left_wheel: u32, right_wheel: u32) -> Self {
        Self {
            left_wheel,
            right_wheel,
            stiffness: 1000.0,
        }
    }

    /// Spring constant, N/m, between 0 and [`MAX_STIFFNESS`](Self::MAX_STIFFNESS); 0 disables
    /// the bar. Default 1000.
    #[must_use]
    pub fn stiffness(mut self, value: f32) -> Self {
        self.stiffness = value;
        self
    }

    fn validate(&self, wheel_count: usize) -> Result<(), VehicleError> {
        let invalid = |what| Err(VehicleError::InvalidValue(what));
        if (self.left_wheel as usize) >= wheel_count || (self.right_wheel as usize) >= wheel_count {
            return invalid("anti-roll bar wheel index out of range");
        }
        if self.left_wheel == self.right_wheel {
            return invalid("an anti-roll bar needs two different wheels");
        }
        if !(0.0..=Self::MAX_STIFFNESS).contains(&self.stiffness) {
            return invalid("anti-roll bar stiffness must be between 0 and MAX_STIFFNESS");
        }
        Ok(())
    }
}

/// How the wheels find the ground (Jolt's `VehicleCollisionTester` kinds). Each wheel casts
/// along its suspension from its attachment point at the start of every step.
///
/// The wheels see the bodies whose object layer collides with the tester's `object_layer` in
/// the world's [`CollisionLayers`](crate::CollisionLayers), never their own chassis, never
/// sensors and never soft bodies: Jolt's `VehicleConstraint` solves the body under a wheel as a
/// rigid body, so a wheel passes through a soft body to the ground below it. Jolt's testers apply no per-sub-shape filter, so compound children cannot be
/// excluded by group; give the wheels a dedicated object layer that collides with exactly the
/// layers they should drive on.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum VehicleCollisionTester {
    /// A ray from the attachment point, `suspension_max_length + radius` long. Cheapest; misses
    /// small steps and edges between the ray and the tire.
    Ray {
        /// The layer the ray queries as.
        object_layer: ObjectLayer,
        /// World-space up for the slope check, a unit vector. It is fixed: replace the tester
        /// to change it.
        up: Vec3,
        /// Steepest ground, radians in `[0, π]`, measured from `up`, that still counts as
        /// ground.
        max_slope_angle: f32,
    },
    /// A sphere of `radius` cast from the attachment point.
    CastSphere {
        /// The layer the sphere queries as.
        object_layer: ObjectLayer,
        /// Radius of the cast sphere, positive and smaller than every wheel's
        /// `suspension_max_length + radius`.
        radius: f32,
        /// World-space up for the slope check, a unit vector, fixed as for `Ray`.
        up: Vec3,
        /// Steepest ground, radians in `[0, π]`, measured from `up`.
        max_slope_angle: f32,
    },
    /// A cylinder of the wheel's radius and width cast along the suspension. Closest to the
    /// tire's shape; no slope check.
    CastCylinder {
        /// The layer the cylinder queries as.
        object_layer: ObjectLayer,
        /// Fraction in `[0, 1]` of `min(width / 2, radius)` used as the cylinder's convex radius.
        convex_radius_fraction: f32,
    },
}

impl VehicleCollisionTester {
    /// A ray tester with up +Y and Jolt's sample slope limit of 80°.
    pub fn ray(object_layer: ObjectLayer) -> Self {
        Self::Ray {
            object_layer,
            up: Vec3::new(0.0, 1.0, 0.0),
            max_slope_angle: 80.0_f32.to_radians(),
        }
    }

    /// A sphere tester of `radius` with up +Y and a slope limit of 80°.
    pub fn cast_sphere(object_layer: ObjectLayer, radius: f32) -> Self {
        Self::CastSphere {
            object_layer,
            radius,
            up: Vec3::new(0.0, 1.0, 0.0),
            max_slope_angle: 80.0_f32.to_radians(),
        }
    }

    /// A cylinder tester with Jolt's default convex radius fraction 0.1.
    pub fn cast_cylinder(object_layer: ObjectLayer) -> Self {
        Self::CastCylinder {
            object_layer,
            convex_radius_fraction: 0.1,
        }
    }

    /// The object layer the tester queries as.
    pub fn object_layer(&self) -> ObjectLayer {
        match *self {
            Self::Ray { object_layer, .. }
            | Self::CastSphere { object_layer, .. }
            | Self::CastCylinder { object_layer, .. } => object_layer,
        }
    }

    /// The world-space up of a ray or sphere tester; `None` for the cylinder.
    pub fn up(&self) -> Option<Vec3> {
        match *self {
            Self::Ray { up, .. } | Self::CastSphere { up, .. } => Some(up),
            Self::CastCylinder { .. } => None,
        }
    }

    /// This tester with its up replaced by `up`; the cylinder has none and stays as it is.
    pub(crate) fn with_up(self, new_up: Vec3) -> Self {
        match self {
            Self::Ray {
                object_layer,
                max_slope_angle,
                ..
            } => Self::Ray {
                object_layer,
                up: new_up,
                max_slope_angle,
            },
            Self::CastSphere {
                object_layer,
                radius,
                max_slope_angle,
                ..
            } => Self::CastSphere {
                object_layer,
                radius,
                up: new_up,
                max_slope_angle,
            },
            cylinder @ Self::CastCylinder { .. } => cylinder,
        }
    }

    /// Checks the tester against the world's layers and the wheels it serves.
    pub(crate) fn validate(
        &self,
        object_layer_count: u32,
        mut wheels: impl Iterator<Item = WheelGeometry>,
    ) -> Result<(), VehicleError> {
        let invalid = |what| Err(VehicleError::InvalidValue(what));
        if self.object_layer().get() >= object_layer_count {
            return invalid("collision tester object layer does not exist in this world");
        }
        let slope_ok = |angle: f32| is_finite_non_negative(angle) && angle <= PI;
        match *self {
            Self::Ray {
                up,
                max_slope_angle,
                ..
            } => {
                // The ray length `max length + radius` (`VehicleCollisionTesterRay::Collide`)
                // is finite: wheel lengths are at most `limits::MAX_SHAPE_EXTENT`.
                if !(is_unit(up) && slope_ok(max_slope_angle)) {
                    return invalid(
                        "ray tester needs a unit up and a max slope angle between 0 and pi",
                    );
                }
            }
            Self::CastSphere {
                radius,
                up,
                max_slope_angle,
                ..
            } => {
                if !(is_unit(up) && slope_ok(max_slope_angle)) {
                    return invalid(
                        "sphere tester needs a unit up and a max slope angle between 0 and pi",
                    );
                }
                if !is_finite_positive(radius) {
                    return invalid("sphere tester radius must be finite and positive");
                }
                // The cast length (`VehicleCollisionTesterCastSphere::Collide`), finite because
                // wheel lengths are at most `limits::MAX_SHAPE_EXTENT`.
                if !wheels.all(|w| w.suspension_max_length + w.radius - radius > 0.0) {
                    return invalid(
                        "sphere tester radius must be below every wheel's max suspension length plus radius",
                    );
                }
            }
            Self::CastCylinder {
                convex_radius_fraction,
                ..
            } => {
                if !(convex_radius_fraction.is_finite()
                    && (0.0..=1.0).contains(&convex_radius_fraction))
                {
                    return invalid("cylinder tester convex radius fraction must be in [0, 1]");
                }
                if !wheels.all(|w| w.width > 0.0 && w.suspension_max_length > 0.0) {
                    return invalid(
                        "cylinder tester needs every wheel's width and max suspension length positive",
                    );
                }
            }
        }
        Ok(())
    }

    /// The joltc tester of this validated tester for the vehicle whose chassis has the Jolt id
    /// `vehicle_body`, holding one reference. It skips the chassis and every soft body: Jolt's
    /// `VehicleConstraint` solves the body under a wheel as a rigid body.
    pub(crate) fn create(&self, vehicle_body: JPH_BodyID) -> Owned<JPH_VehicleCollisionTester> {
        let tester: *mut JPH_VehicleCollisionTester = match *self {
            Self::Ray {
                object_layer,
                up,
                max_slope_angle,
            } => {
                let up = up.to_jph();
                // SAFETY: Jolt is initialised (a world exists); `up` is a live local and every
                // value was validated.
                unsafe {
                    JPH_VehicleCollisionTesterRay_Create2(
                        object_layer.get(),
                        &up,
                        max_slope_angle,
                        vehicle_body,
                    )
                }
                .cast()
            }
            Self::CastSphere {
                object_layer,
                radius,
                up,
                max_slope_angle,
            } => {
                let up = up.to_jph();
                // SAFETY: as for the ray.
                unsafe {
                    JPH_VehicleCollisionTesterCastSphere_Create2(
                        object_layer.get(),
                        radius,
                        &up,
                        max_slope_angle,
                        vehicle_body,
                    )
                }
                .cast()
            }
            Self::CastCylinder {
                object_layer,
                convex_radius_fraction,
            } => {
                // SAFETY: as for the ray.
                unsafe {
                    JPH_VehicleCollisionTesterCastCylinder_Create2(
                        object_layer.get(),
                        convex_radius_fraction,
                        vehicle_body,
                    )
                }
                .cast()
            }
        };
        // SAFETY: the extension returns the new tester holding one reference (it calls `AddRef`),
        // which the guard takes over; the tester owns its body filter. Every tester kind derives
        // from `VehicleCollisionTester` with single inheritance, joltc's cast convention.
        unsafe { Owned::from_raw(tester) }
            .unwrap_or_else(|| unreachable!("joltc `new`s the tester"))
    }
}

/// Wheel settings, of which the owner holds the one reference `JPH_WheelSettingsWV_Create`
/// returns. Each wheel of a vehicle keeps its own `RefConst` to its settings
/// (`Wheel::mSettings`), so the owner may release its reference once the vehicle exists.
impl JoltObject for JPH_WheelSettingsWV {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner holds one reference (trait contract), released here once; a
        // `WheelSettingsWV` is a `WheelSettings` with single inheritance.
        unsafe { JPH_WheelSettings_Destroy(ptr.cast()) };
    }
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

/// Transmission settings, owned whole: joltc `new`s them, and controller settings copy them.
impl JoltObject for JPH_VehicleTransmissionSettings {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner owns the settings (trait contract), which joltc deletes.
        unsafe { JPH_VehicleTransmissionSettings_Destroy(ptr) };
    }
}

/// A linear curve, owned whole: joltc `new`s it, and wheel and engine settings copy it.
impl JoltObject for JPH_LinearCurve {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner owns the curve (trait contract), which joltc deletes.
        unsafe { JPH_LinearCurve_Destroy(ptr) };
    }
}

/// A collision tester, of which the owner holds one reference. A vehicle keeps its own
/// (`VehicleConstraint::mVehicleCollisionTester`), so the owner may release its reference once
/// the tester is set.
impl JoltObject for JPH_VehicleCollisionTester {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner holds one reference (trait contract), released here once.
        unsafe { JPH_VehicleCollisionTester_Destroy(ptr) };
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::world::ensure_initialized;

    fn bits3(v: Vec3) -> [u32; 3] {
        <[f32; 3]>::from(v).map(f32::to_bits)
    }

    fn jolt_vec(get: impl FnOnce(*mut JPH_Vec3)) -> Vec3 {
        let mut value = Vec3::ZERO.to_jph();
        get(&mut value);
        Vec3::from_jph(value)
    }

    /// The points of a curve that joltc lends, read before its owner is released.
    ///
    /// # Safety
    /// `curve` points to a live curve.
    unsafe fn curve_points(curve: *const JPH_LinearCurve) -> Vec<(f32, f32)> {
        // SAFETY: the curve is live (caller contract); the getters only read it, with indices
        // below its point count.
        unsafe {
            (0..JPH_LinearCurve_GetPointCount(curve))
                .map(|index| {
                    let mut point = JPH_Point { x: 0.0, y: 0.0 };
                    JPH_LinearCurve_GetPoint(curve, index, &mut point);
                    (point.x, point.y)
                })
                .collect()
        }
    }

    #[test]
    fn wheel_defaults_are_jolts() {
        assert!(ensure_initialized());
        let ours = WheelSettings::new(Vec3::ZERO);
        // SAFETY: Jolt is initialised; the guard owns the one reference joltc returns.
        let jolt = unsafe { Owned::from_raw(JPH_WheelSettingsWV_Create()) }.unwrap();
        let wv = jolt.as_ptr();
        let base: *mut JPH_WheelSettings = wv.cast();
        // SAFETY: the settings are live and only read; the friction curves are members of the
        // settings, read while the guard keeps them alive. Every output is a live local.
        unsafe {
            let direction = jolt_vec(|v| JPH_WheelSettings_GetSuspensionDirection(base, v));
            assert_eq!(bits3(direction), bits3(ours.suspension_direction));
            let axis = jolt_vec(|v| JPH_WheelSettings_GetSteeringAxis(base, v));
            assert_eq!(bits3(axis), bits3(ours.steering_axis));
            let up = jolt_vec(|v| JPH_WheelSettings_GetWheelUp(base, v));
            assert_eq!(bits3(up), bits3(ours.wheel_up));
            let forward = jolt_vec(|v| JPH_WheelSettings_GetWheelForward(base, v));
            assert_eq!(bits3(forward), bits3(ours.wheel_forward));
            assert!(!JPH_WheelSettings_GetEnableSuspensionForcePoint(base));
            assert_eq!(ours.suspension_force_point, None);
            assert_eq!(
                JPH_WheelSettings_GetSuspensionMinLength(base),
                ours.suspension_min_length
            );
            assert_eq!(
                JPH_WheelSettings_GetSuspensionMaxLength(base),
                ours.suspension_max_length
            );
            assert_eq!(
                JPH_WheelSettings_GetSuspensionPreloadLength(base),
                ours.suspension_preload_length
            );
            let mut spring = JPH_SpringSettings {
                mode: JPH_SpringMode_StiffnessAndDamping,
                frequencyOrStiffness: 0.0,
                damping: 0.0,
            };
            JPH_WheelSettings_GetSuspensionSpring(base, &mut spring);
            let expected = ours.suspension_spring.to_jph();
            assert_eq!(spring.mode, expected.mode);
            assert_eq!(spring.frequencyOrStiffness, expected.frequencyOrStiffness);
            assert_eq!(spring.damping, expected.damping);
            assert_eq!(JPH_WheelSettings_GetRadius(base), ours.radius);
            assert_eq!(JPH_WheelSettings_GetWidth(base), ours.width);
            assert_eq!(JPH_WheelSettingsWV_GetInertia(wv), ours.inertia);
            assert_eq!(
                JPH_WheelSettingsWV_GetAngularDamping(wv),
                ours.angular_damping
            );
            assert_eq!(
                JPH_WheelSettingsWV_GetMaxSteerAngle(wv),
                ours.max_steer_angle
            );
            assert_eq!(
                JPH_WheelSettingsWV_GetMaxBrakeTorque(wv),
                ours.max_brake_torque
            );
            assert_eq!(
                JPH_WheelSettingsWV_GetMaxHandBrakeTorque(wv),
                ours.max_hand_brake_torque
            );
            assert_eq!(
                curve_points(JPH_WheelSettingsWV_GetLongitudinalFriction(wv)),
                ours.longitudinal_friction
            );
            assert_eq!(
                curve_points(JPH_WheelSettingsWV_GetLateralFriction(wv)),
                ours.lateral_friction
            );
        }
    }

    #[test]
    fn controller_defaults_are_jolts() {
        assert!(ensure_initialized());
        let engine = VehicleEngineSettings::default();
        let transmission = VehicleTransmissionSettings::default();
        let vehicle = car();
        // SAFETY: Jolt is initialised; the guard owns the one reference joltc returns.
        let jolt =
            unsafe { Owned::from_raw(JPH_WheeledVehicleControllerSettings_Create()) }.unwrap();
        let ptr = jolt.as_ptr();
        // SAFETY: the settings are live and only read. `GetEngine` lends the settings' own
        // torque curve and `GetTransmission` the settings' own transmission; both are read
        // while the guard keeps the settings alive and never destroyed here.
        unsafe {
            assert_eq!(
                JPH_WheeledVehicleControllerSettings_GetDifferentialLimitedSlipRatio(ptr),
                vehicle.differential_limited_slip_ratio
            );
            let mut jolt_engine: JPH_VehicleEngineSettings = std::mem::zeroed();
            JPH_WheeledVehicleControllerSettings_GetEngine(ptr, &mut jolt_engine);
            assert_eq!(jolt_engine.maxTorque, engine.max_torque);
            assert_eq!(jolt_engine.minRPM, engine.min_rpm);
            assert_eq!(jolt_engine.maxRPM, engine.max_rpm);
            assert_eq!(jolt_engine.inertia, engine.inertia);
            assert_eq!(jolt_engine.angularDamping, engine.angular_damping);
            assert_eq!(
                curve_points(jolt_engine.normalizedTorque),
                engine.normalized_torque
            );

            let jolt_transmission = JPH_WheeledVehicleControllerSettings_GetTransmission(ptr);
            assert_eq!(
                JPH_VehicleTransmissionSettings_GetMode(jolt_transmission),
                JPH_TransmissionMode_Auto
            );
            let forward: Vec<f32> =
                (0..JPH_VehicleTransmissionSettings_GetGearRatioCount(jolt_transmission))
                    .map(|i| JPH_VehicleTransmissionSettings_GetGearRatio(jolt_transmission, i))
                    .collect();
            assert_eq!(forward, transmission.gear_ratios);
            let reverse: Vec<f32> =
                (0..JPH_VehicleTransmissionSettings_GetReverseGearRatioCount(jolt_transmission))
                    .map(|i| {
                        JPH_VehicleTransmissionSettings_GetReverseGearRatio(jolt_transmission, i)
                    })
                    .collect();
            assert_eq!(reverse, transmission.reverse_gear_ratios);
            assert_eq!(
                JPH_VehicleTransmissionSettings_GetSwitchTime(jolt_transmission),
                transmission.switch_time
            );
            assert_eq!(
                JPH_VehicleTransmissionSettings_GetClutchReleaseTime(jolt_transmission),
                transmission.clutch_release_time
            );
            assert_eq!(
                JPH_VehicleTransmissionSettings_GetSwitchLatency(jolt_transmission),
                transmission.switch_latency
            );
            assert_eq!(
                JPH_VehicleTransmissionSettings_GetShiftUpRPM(jolt_transmission),
                transmission.shift_up_rpm
            );
            assert_eq!(
                JPH_VehicleTransmissionSettings_GetShiftDownRPM(jolt_transmission),
                transmission.shift_down_rpm
            );
            assert_eq!(
                JPH_VehicleTransmissionSettings_GetClutchStrength(jolt_transmission),
                transmission.clutch_strength
            );
        }
    }

    #[test]
    fn constraint_differential_and_anti_roll_bar_defaults_are_jolts() {
        assert!(ensure_initialized());
        let vehicle = car();
        // SAFETY: all-zero values of these plain C structs are valid (integers, floats, `false`
        // and null pointers); the `_Init` calls fill them with Jolt's defaults and allocate
        // nothing.
        let (constraint, differential, bar) = unsafe {
            let mut constraint: JPH_VehicleConstraintSettings = std::mem::zeroed();
            let mut differential: JPH_VehicleDifferentialSettings = std::mem::zeroed();
            let mut bar: JPH_VehicleAntiRollBar = std::mem::zeroed();
            JPH_VehicleConstraintSettings_Init(&mut constraint);
            JPH_VehicleDifferentialSettings_Init(&mut differential);
            JPH_VehicleAntiRollBar_Init(&mut bar);
            (constraint, differential, bar)
        };
        assert_eq!(bits3(Vec3::from_jph(constraint.up)), bits3(vehicle.up));
        assert_eq!(
            bits3(Vec3::from_jph(constraint.forward)),
            bits3(vehicle.forward)
        );
        assert_eq!(constraint.maxPitchRollAngle, vehicle.max_pitch_roll_angle);

        let ours = VehicleDifferentialSettings::new(None, None).to_jph();
        assert_eq!(ours.leftWheel, differential.leftWheel);
        assert_eq!(ours.rightWheel, differential.rightWheel);
        assert_eq!(ours.differentialRatio, differential.differentialRatio);
        assert_eq!(ours.leftRightSplit, differential.leftRightSplit);
        assert_eq!(ours.limitedSlipRatio, differential.limitedSlipRatio);
        assert_eq!(ours.engineTorqueRatio, differential.engineTorqueRatio);

        let ours = VehicleAntiRollBar::new(0, 1);
        assert_eq!(bar.leftWheel, ours.left_wheel as i32);
        assert_eq!(bar.rightWheel, ours.right_wheel as i32);
        assert_eq!(bar.stiffness, ours.stiffness);
    }

    #[test]
    fn built_settings_reach_jolt() {
        assert!(ensure_initialized());
        let wheel = WheelSettings::new(Vec3::new(0.9, -0.1, 1.4))
            .suspension_force_point(Some(Vec3::new(0.9, -0.3, 1.4)))
            .radius(0.35)
            .width(0.2)
            .inertia(1.5)
            .max_steer_angle(0.5)
            .longitudinal_friction(vec![(0.0, 0.0), (0.1, 1.5)])
            .suspension_spring(SuspensionSpring::StiffnessAndDamping {
                stiffness: 30000.0,
                damping: 2000.0,
            });
        let jolt = wheel.create();
        let wv = jolt.as_ptr();
        let base: *mut JPH_WheelSettings = wv.cast();
        // SAFETY: the settings are live and only read; outputs are live locals.
        unsafe {
            let position = jolt_vec(|v| JPH_WheelSettings_GetPosition(base, v));
            assert_eq!(bits3(position), bits3(wheel.position));
            assert!(JPH_WheelSettings_GetEnableSuspensionForcePoint(base));
            assert_eq!(JPH_WheelSettings_GetRadius(base), 0.35);
            assert_eq!(JPH_WheelSettings_GetWidth(base), 0.2);
            assert_eq!(JPH_WheelSettingsWV_GetInertia(wv), 1.5);
            assert_eq!(JPH_WheelSettingsWV_GetMaxSteerAngle(wv), 0.5);
            let mut spring = JPH_SpringSettings {
                mode: JPH_SpringMode_FrequencyAndDamping,
                frequencyOrStiffness: 0.0,
                damping: 0.0,
            };
            JPH_WheelSettings_GetSuspensionSpring(base, &mut spring);
            assert_eq!(spring.mode, JPH_SpringMode_StiffnessAndDamping);
            assert_eq!(spring.frequencyOrStiffness, 30000.0);
            assert_eq!(
                curve_points(JPH_WheelSettingsWV_GetLongitudinalFriction(wv)),
                vec![(0.0, 0.0), (0.1, 1.5)]
            );
        }

        let vehicle = car()
            .engine(VehicleEngineSettings::default().max_torque(800.0))
            .transmission(VehicleTransmissionSettings::default().gear_ratios(vec![3.0, 1.5]))
            .differential_limited_slip_ratio(f32::MAX);
        let controller = vehicle.create_controller();
        // SAFETY: as in `controller_defaults_are_jolts`.
        unsafe {
            let ptr = controller.as_ptr();
            let mut engine: JPH_VehicleEngineSettings = std::mem::zeroed();
            JPH_WheeledVehicleControllerSettings_GetEngine(ptr, &mut engine);
            assert_eq!(engine.maxTorque, 800.0);
            let transmission = JPH_WheeledVehicleControllerSettings_GetTransmission(ptr);
            assert_eq!(
                JPH_VehicleTransmissionSettings_GetGearRatioCount(transmission),
                2
            );
            assert_eq!(
                JPH_WheeledVehicleControllerSettings_GetDifferentialsCount(ptr),
                1
            );
            assert_eq!(
                JPH_WheeledVehicleControllerSettings_GetDifferentialLimitedSlipRatio(ptr),
                f32::MAX
            );
        }
        let tester = VehicleCollisionTester::ray(ObjectLayer::new(1)).create(0);
        // SAFETY: the tester is live and only read.
        let layer = unsafe { JPH_VehicleCollisionTester_GetObjectLayer(tester.as_ptr()) };
        assert_eq!(layer, 1);
    }

    /// A valid car: four wheels, front-wheel drive, a ray tester on layer 1.
    fn car() -> VehicleSettings {
        let wheel = |x: f32, z: f32| WheelSettings::new(Vec3::new(x, -0.1, z)).radius(0.35);
        VehicleSettings::new(
            vec![
                wheel(0.9, 1.4),
                wheel(-0.9, 1.4),
                wheel(0.9, -1.4),
                wheel(-0.9, -1.4),
            ],
            vec![VehicleDifferentialSettings::new(Some(0), Some(1))],
            VehicleCollisionTester::ray(ObjectLayer::new(1)),
        )
    }

    const LAYERS: u32 = 2;

    #[track_caller]
    fn assert_rejected(settings: VehicleSettings) {
        assert!(
            matches!(
                settings.validate(LAYERS),
                Err(VehicleError::InvalidValue(_))
            ),
            "{settings:?}"
        );
    }

    fn with_wheel(edit: impl Fn(WheelSettings) -> WheelSettings) -> VehicleSettings {
        let mut settings = car();
        settings.wheels[2] = edit(settings.wheels[2].clone());
        settings
    }

    #[test]
    fn valid_settings_pass() {
        assert_eq!(car().validate(LAYERS), Ok(()));
        let two_differentials = VehicleSettings {
            differentials: vec![
                VehicleDifferentialSettings::new(Some(0), Some(1)).engine_torque_ratio(0.5),
                VehicleDifferentialSettings::new(Some(2), Some(3)).engine_torque_ratio(0.5),
            ],
            ..car()
        }
        .anti_roll_bars(vec![VehicleAntiRollBar::new(0, 1)])
        .differential_limited_slip_ratio(f32::MAX);
        assert_eq!(two_differentials.validate(LAYERS), Ok(()));
        let cylinder = VehicleSettings {
            collision_tester: VehicleCollisionTester::cast_cylinder(ObjectLayer::new(0)),
            ..car()
        };
        assert_eq!(cylinder.validate(LAYERS), Ok(()));
    }

    #[test]
    fn vehicle_values_are_validated() {
        assert_rejected(VehicleSettings {
            wheels: Vec::new(),
            differentials: Vec::new(),
            ..car()
        });
        assert_rejected(car().up(Vec3::new(0.0, 2.0, 0.0)));
        assert_rejected(car().forward(Vec3::new(0.0, f32::NAN, 1.0)));
        assert_rejected(car().forward(Vec3::new(0.0, 0.6, 0.8)));
        assert_rejected(car().max_pitch_roll_angle(-0.1));
        assert_rejected(car().max_pitch_roll_angle(PI + 0.001));
        assert_rejected(car().differential_limited_slip_ratio(1.0));
        assert_rejected(car().differential_limited_slip_ratio(f32::INFINITY));
    }

    #[test]
    fn unit_vectors_are_checked_more_strictly_than_jolt() {
        // |v|² - 1 = 8e-7: inside Jolt's `IsNormalized` tolerance of 1e-6, outside ours.
        let almost = Vec3::new(0.0, -1.000_000_4, 0.0);
        assert!((almost.dot(almost) - 1.0).abs() < 1.0e-6);
        assert!(!is_unit(almost));
        assert_rejected(with_wheel(|w| w.suspension_direction(almost)));
        assert!(is_unit(Vec3::new(0.6, 0.0, 0.8)));
    }

    #[test]
    fn wheel_values_are_validated() {
        let y = Vec3::new(0.0, 1.0, 0.0);
        assert_rejected(with_wheel(|w| {
            w.suspension_direction(Vec3::new(0.0, -1.1, 0.0))
        }));
        assert_rejected(with_wheel(|w| w.steering_axis(Vec3::ZERO)));
        assert_rejected(with_wheel(|w| w.wheel_up(Vec3::new(0.0, 0.0, 1.0))));
        assert_rejected(with_wheel(|w| w.wheel_forward(y)));
        assert_rejected(with_wheel(|w| {
            w.suspension_force_point(Some(Vec3::new(f32::NAN, 0.0, 0.0)))
        }));
        assert_rejected(with_wheel(|w| w.suspension_min_length(-0.1)));
        assert_rejected(with_wheel(|w| w.suspension_max_length(0.2)));
        assert_rejected(with_wheel(|w| w.suspension_preload_length(-0.1)));
        assert_rejected(with_wheel(|w| {
            w.suspension_spring(SuspensionSpring::FrequencyAndDamping {
                frequency: 0.0,
                damping: 0.5,
            })
        }));
        assert_rejected(with_wheel(|w| {
            w.suspension_spring(SuspensionSpring::StiffnessAndDamping {
                stiffness: 1000.0,
                damping: -1.0,
            })
        }));
        assert_rejected(with_wheel(|w| w.radius(0.0)));
        assert_rejected(with_wheel(|w| w.width(-0.1)));
        assert_rejected(with_wheel(|w| w.angular_damping(-0.1)));
        assert_rejected(with_wheel(|w| w.max_steer_angle(0.5 * PI + 0.001)));
        assert_rejected(with_wheel(|w| w.max_brake_torque(-1.0)));
        assert_rejected(with_wheel(|w| w.max_hand_brake_torque(f32::NAN)));
        assert_rejected(with_wheel(|w| w.longitudinal_friction(Vec::new())));
        assert_rejected(with_wheel(|w| {
            w.lateral_friction(vec![(0.0, 0.0), (3.0, 1.2), (3.0, 1.0)])
        }));
        assert_eq!(with_wheel(|w| w.width(0.0)).validate(LAYERS), Ok(()));
        assert_eq!(
            with_wheel(|w| w.max_steer_angle(-0.5 * PI)).validate(LAYERS),
            Ok(())
        );
    }

    #[test]
    fn wheel_magnitudes_are_bounded_by_the_policy() {
        let extent = limits::MAX_SHAPE_EXTENT;
        let beyond = extent.next_up();
        let at = |x: f32| Vec3::new(x, -0.1, 0.0);
        let placed = |p: Vec3| move |_: WheelSettings| WheelSettings::new(p).radius(0.35);
        assert_eq!(with_wheel(placed(at(extent))).validate(LAYERS), Ok(()));
        assert_rejected(with_wheel(placed(at(beyond))));
        assert_eq!(
            with_wheel(|w| w.suspension_force_point(Some(at(-extent)))).validate(LAYERS),
            Ok(())
        );
        assert_rejected(with_wheel(|w| w.suspension_force_point(Some(at(-beyond)))));
        type Edit = fn(WheelSettings, f32) -> WheelSettings;
        let lengths: [Edit; 5] = [
            |w, v| {
                w.suspension_min_length(v)
                    .suspension_max_length(limits::MAX_SHAPE_EXTENT)
            },
            |w, v| w.suspension_max_length(v),
            |w, v| w.suspension_preload_length(v),
            |w, v| w.radius(v),
            |w, v| w.width(v),
        ];
        for edit in lengths {
            assert_eq!(with_wheel(|w| edit(w, extent)).validate(LAYERS), Ok(()));
            assert_rejected(with_wheel(|w| edit(w, beyond)));
        }
    }

    #[test]
    fn suspension_springs_are_bounded_by_the_coefficient() {
        let bound = limits::MAX_SPRING_COEFFICIENT;
        let stiffness =
            |stiffness, damping| SuspensionSpring::StiffnessAndDamping { stiffness, damping };
        let spring = |spring: SuspensionSpring| with_wheel(move |w| w.suspension_spring(spring));
        assert_eq!(spring(stiffness(bound, bound)).validate(LAYERS), Ok(()));
        assert_rejected(spring(stiffness(bound.next_up(), 0.0)));
        assert_rejected(spring(stiffness(1.0, bound.next_up())));
        // Frequency mode with the effective mass at most `MAX_MASS`: `MAX_MASS * ω² <= bound`
        // gives the largest frequency, `2 * MAX_MASS * ζ * ω <= bound` the largest damping ratio
        // at 1 Hz.
        let mass = f64::from(limits::MAX_MASS);
        let two_pi = 2.0 * std::f64::consts::PI;
        let max_frequency = (f64::from(bound) / mass).sqrt() / two_pi;
        let max_damping = f64::from(bound) / (2.0 * mass * two_pi);
        let frequency = |frequency: f64, damping: f64| SuspensionSpring::FrequencyAndDamping {
            frequency: frequency as f32,
            damping: damping as f32,
        };
        for (factor, accepted) in [(0.999, true), (1.001, false)] {
            for candidate in [
                frequency(factor * max_frequency, 0.0),
                frequency(1.0, factor * max_damping),
            ] {
                assert_eq!(
                    spring(candidate).validate(LAYERS).is_ok(),
                    accepted,
                    "{candidate:?}"
                );
            }
        }
    }

    #[test]
    fn wheel_inertia_must_be_positive() {
        // Jolt asserts only `>= 0`, but divides by the wheel inertia.
        assert_rejected(with_wheel(|w| w.inertia(0.0)));
    }

    const SMALLEST_SUBNORMAL: f32 = f32::from_bits(1);

    #[test]
    fn step_coefficients_of_wheels_must_be_finite() {
        // `delta_time / inertia` overflows for a subnormal inertia, driven wheel or not.
        assert_rejected(with_wheel(|w| w.inertia(SMALLEST_SUBNORMAL)));
        assert_rejected(with_wheel(|w| w.radius(SMALLEST_SUBNORMAL)));
        assert_rejected(with_wheel(|w| w.radius(f32::MAX).inertia(1.0e-3)));
        assert_rejected(with_wheel(|w| {
            w.max_brake_torque(f32::MAX).max_hand_brake_torque(f32::MAX)
        }));
        assert_rejected(with_wheel(|w| w.max_brake_torque(f32::MAX).inertia(0.5)));
        // The brake-lock torque per rad/s, `inertia / MIN_DELTA_TIME`, overflows; with radius 1
        // every other coefficient of the wheel stays finite.
        assert_rejected(with_wheel(|w| w.radius(1.0).inertia(f32::MAX / 2.0)));
        assert_eq!(
            with_wheel(|w| w.radius(1.0).inertia(1.0e30)).validate(LAYERS),
            Ok(())
        );
        assert_eq!(with_wheel(|w| w.inertia(1.0e-30)).validate(LAYERS), Ok(()));
        assert_eq!(
            with_wheel(|w| w.max_brake_torque(1.0e30)).validate(LAYERS),
            Ok(())
        );
    }

    #[test]
    fn step_coefficients_of_the_drivetrain_must_be_finite() {
        let engine = VehicleEngineSettings::default();
        assert_rejected(car().engine(engine.clone().inertia(SMALLEST_SUBNORMAL)));
        assert_rejected(car().engine(engine.clone().inertia(1.0e-38)));
        assert_rejected(car().engine(engine.clone().max_torque(f32::MAX)));
        assert_rejected(car().engine(engine.clone().normalized_torque(vec![(0.0, f32::MAX)])));
        let transmission = VehicleTransmissionSettings::default();
        assert_rejected(car().transmission(transmission.clone().clutch_strength(f32::MAX)));
        assert_rejected(car().transmission(transmission.gear_ratios(vec![f32::MAX])));
        assert_rejected(VehicleSettings {
            differentials: vec![
                VehicleDifferentialSettings::new(Some(0), Some(1)).differential_ratio(f32::MAX)
            ],
            ..car()
        });
        // A driven wheel with a tiny inertia overflows `delta_time * S * R / inertia`; the same
        // inertia on an undriven wheel does not.
        let tiny = |w: WheelSettings| {
            w.inertia(1.0e-37)
                .max_brake_torque(0.0)
                .max_hand_brake_torque(0.0)
        };
        let mut tiny_driven = car();
        tiny_driven.wheels[0] = tiny(tiny_driven.wheels[0].clone());
        assert_rejected(tiny_driven);
        assert_eq!(with_wheel(tiny).validate(LAYERS), Ok(()));
        assert_eq!(
            car().engine(engine.max_torque(1.0e30)).validate(LAYERS),
            Ok(())
        );
    }

    #[test]
    fn engine_values_are_validated() {
        let engine = |edit: fn(VehicleEngineSettings) -> VehicleEngineSettings| {
            car().engine(edit(VehicleEngineSettings::default()))
        };
        assert_rejected(engine(|e| e.max_torque(-1.0)));
        assert_rejected(engine(|e| e.min_rpm(-1.0)));
        assert_rejected(engine(|e| e.min_rpm(7000.0)));
        assert_rejected(engine(|e| e.max_rpm(f32::INFINITY)));
        assert_rejected(engine(|e| e.angular_damping(-0.1)));
        assert_rejected(engine(|e| e.normalized_torque(Vec::new())));
        assert_rejected(engine(|e| {
            e.normalized_torque(vec![(0.5, 1.0), (0.2, 0.8)])
        }));
    }

    #[test]
    fn engine_inertia_must_be_positive() {
        assert_rejected(car().engine(VehicleEngineSettings::default().inertia(0.0)));
    }

    #[test]
    fn transmission_values_are_validated() {
        let transmission =
            |edit: fn(VehicleTransmissionSettings) -> VehicleTransmissionSettings| {
                car().transmission(edit(VehicleTransmissionSettings::default()))
            };
        assert_rejected(transmission(|t| t.gear_ratios(vec![2.0, 0.0])));
        assert_rejected(transmission(|t| t.reverse_gear_ratios(vec![0.0])));
        assert_rejected(transmission(|t| t.switch_time(-0.1)));
        assert_rejected(transmission(|t| t.clutch_release_time(f32::NAN)));
        assert_rejected(transmission(|t| t.switch_latency(-0.1)));
        assert_rejected(transmission(|t| t.shift_down_rpm(0.0)));
        assert_rejected(transmission(|t| t.shift_up_rpm(2000.0)));
        assert_rejected(transmission(|t| t.shift_up_rpm(6000.0)));
        assert_rejected(transmission(|t| t.clutch_strength(0.0)));
    }

    #[test]
    fn gear_lists_must_not_be_empty() {
        let transmission = VehicleTransmissionSettings::default();
        assert_rejected(car().transmission(transmission.clone().gear_ratios(Vec::new())));
        assert_rejected(car().transmission(transmission.reverse_gear_ratios(Vec::new())));
    }

    #[test]
    fn differentials_are_required() {
        assert_rejected(VehicleSettings {
            differentials: Vec::new(),
            ..car()
        });
    }

    #[test]
    fn differential_values_are_validated() {
        let with = |differential: VehicleDifferentialSettings| VehicleSettings {
            differentials: vec![differential],
            ..car()
        };
        let front = VehicleDifferentialSettings::new(Some(0), Some(1));
        assert_rejected(with(VehicleDifferentialSettings::new(Some(0), Some(4))));
        assert_rejected(with(VehicleDifferentialSettings::new(None, None)));
        assert_rejected(with(front.differential_ratio(0.0)));
        assert_rejected(with(front.left_right_split(1.5)));
        assert_rejected(with(front.limited_slip_ratio(1.0)));
        assert_rejected(with(front.engine_torque_ratio(-0.5)));
        assert_rejected(with(front.engine_torque_ratio(0.9)));
        assert_eq!(
            with(VehicleDifferentialSettings::new(None, Some(3))).validate(LAYERS),
            Ok(())
        );
        let uneven = VehicleSettings {
            differentials: vec![
                front.engine_torque_ratio(0.5),
                VehicleDifferentialSettings::new(Some(2), Some(3)).engine_torque_ratio(0.4999),
            ],
            ..car()
        };
        assert_rejected(uneven);
    }

    #[test]
    fn anti_roll_bars_are_validated() {
        assert_rejected(car().anti_roll_bars(vec![VehicleAntiRollBar::new(0, 4)]));
        assert_rejected(car().anti_roll_bars(vec![VehicleAntiRollBar::new(1, 1)]));
        assert_rejected(car().anti_roll_bars(vec![VehicleAntiRollBar::new(0, 1).stiffness(-1.0)]));
        let bar = |stiffness| {
            car().anti_roll_bars(vec![VehicleAntiRollBar::new(0, 1).stiffness(stiffness)])
        };
        let bound = VehicleAntiRollBar::MAX_STIFFNESS;
        assert_eq!(bar(bound).validate(LAYERS), Ok(()));
        for stiffness in [bound.next_up(), f32::MAX, f32::NAN] {
            assert_rejected(bar(stiffness));
        }
    }

    #[test]
    fn collision_testers_are_validated() {
        let with = |tester: VehicleCollisionTester| VehicleSettings {
            collision_tester: tester,
            ..car()
        };
        let layer = ObjectLayer::new(1);
        assert_rejected(with(VehicleCollisionTester::ray(ObjectLayer::new(LAYERS))));
        assert_rejected(with(VehicleCollisionTester::Ray {
            object_layer: layer,
            up: Vec3::new(0.0, 0.5, 0.0),
            max_slope_angle: 1.0,
        }));
        assert_rejected(with(VehicleCollisionTester::Ray {
            object_layer: layer,
            up: Vec3::new(0.0, 1.0, 0.0),
            max_slope_angle: PI + 0.01,
        }));
        assert_rejected(with(VehicleCollisionTester::cast_sphere(layer, 0.0)));
        // Max suspension length 0.5 plus wheel radius 0.35 leaves no cast for a 0.85 sphere.
        assert_rejected(with(VehicleCollisionTester::cast_sphere(layer, 0.85)));
        assert_eq!(
            with(VehicleCollisionTester::cast_sphere(layer, 0.84)).validate(LAYERS),
            Ok(())
        );
        // A sphere as large as a wheel at the length bound leaves no cast.
        let largest = |w: WheelSettings| {
            w.suspension_max_length(limits::MAX_SHAPE_EXTENT)
                .radius(limits::MAX_SHAPE_EXTENT)
                .inertia(1.0e30)
        };
        assert_eq!(with_wheel(largest).validate(LAYERS), Ok(()));
        let mut largest_sphere = with_wheel(largest);
        largest_sphere.collision_tester =
            VehicleCollisionTester::cast_sphere(layer, 2.0 * limits::MAX_SHAPE_EXTENT);
        assert_rejected(largest_sphere);
        assert_rejected(with(VehicleCollisionTester::CastCylinder {
            object_layer: layer,
            convex_radius_fraction: 1.5,
        }));
        let cylinder = VehicleCollisionTester::cast_cylinder(layer);
        let mut no_width = with_wheel(|w| w.width(0.0));
        no_width.collision_tester = cylinder;
        assert_rejected(no_width);
        let mut no_travel = with_wheel(|w| w.suspension_min_length(0.0).suspension_max_length(0.0));
        assert_eq!(no_travel.validate(LAYERS), Ok(()));
        no_travel.collision_tester = cylinder;
        assert_rejected(no_travel);
    }
}
