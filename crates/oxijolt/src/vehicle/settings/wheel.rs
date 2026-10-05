//! Wheels: suspension, tyre friction, geometry and anti-roll bars.

use std::f32::consts::PI;

use oxijolt_sys::*;

use super::{
    create_curve, is_curve, DEFAULT_LATERAL_FRICTION, DEFAULT_LONGITUDINAL_FRICTION,
    PERPENDICULAR_TOLERANCE,
};
use crate::limits::{self, is_local_distance, is_local_offset};
use crate::math::{is_finite_non_negative, is_finite_positive, is_unit};
use crate::owned::{JoltObject, Owned};
use crate::{SpringSettings, Vec3, VehicleError};

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
    pub(super) fn to_jph(self) -> JPH_SpringSettings {
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

/// The part of a wheel every vehicle kind shares (Jolt `WheelSettings`): where the suspension is
/// attached, how it moves and springs, and the wheel's size. The builders and rules are those of
/// [`WheelSettings`].
#[derive(Clone, Debug, PartialEq)]
pub(super) struct WheelBase {
    pub(super) position: Vec3,
    pub(super) suspension_force_point: Option<Vec3>,
    pub(super) suspension_direction: Vec3,
    pub(super) steering_axis: Vec3,
    pub(super) wheel_up: Vec3,
    pub(super) wheel_forward: Vec3,
    pub(super) suspension_min_length: f32,
    pub(super) suspension_max_length: f32,
    pub(super) suspension_preload_length: f32,
    pub(super) suspension_spring: SuspensionSpring,
    pub(super) radius: f32,
    pub(super) width: f32,
}

impl WheelBase {
    /// Jolt's defaults, with the suspension attached at `position`.
    pub(super) fn new(position: Vec3) -> Self {
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
        }
    }

    pub(super) fn validate(&self) -> Result<(), VehicleError> {
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
                "suspension max length must be within min length..=limits::MAX_SHAPE_EXTENT",
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
        Ok(())
    }

    /// Writes these validated values into joltc wheel settings of any kind.
    ///
    /// # Safety
    /// `wheel` points to live joltc wheel settings that nothing else uses during the call.
    pub(super) unsafe fn apply(&self, wheel: *mut JPH_WheelSettings) {
        let position = self.position.to_jph();
        let force_point = self.suspension_force_point.unwrap_or(Vec3::ZERO).to_jph();
        let suspension_direction = self.suspension_direction.to_jph();
        let steering_axis = self.steering_axis.to_jph();
        let wheel_up = self.wheel_up.to_jph();
        let wheel_forward = self.wheel_forward.to_jph();
        let mut spring = self.suspension_spring.to_jph();
        // SAFETY: `wheel` is live and unshared (contract). The setters copy the vectors and the
        // spring, which are live locals.
        unsafe {
            JPH_WheelSettings_SetPosition(wheel, &position);
            JPH_WheelSettings_SetSuspensionForcePoint(wheel, &force_point);
            JPH_WheelSettings_SetEnableSuspensionForcePoint(
                wheel,
                self.suspension_force_point.is_some(),
            );
            JPH_WheelSettings_SetSuspensionDirection(wheel, &suspension_direction);
            JPH_WheelSettings_SetSteeringAxis(wheel, &steering_axis);
            JPH_WheelSettings_SetWheelUp(wheel, &wheel_up);
            JPH_WheelSettings_SetWheelForward(wheel, &wheel_forward);
            JPH_WheelSettings_SetSuspensionMinLength(wheel, self.suspension_min_length);
            JPH_WheelSettings_SetSuspensionMaxLength(wheel, self.suspension_max_length);
            JPH_WheelSettings_SetSuspensionPreloadLength(wheel, self.suspension_preload_length);
            JPH_WheelSettings_SetSuspensionSpring(wheel, &mut spring);
            JPH_WheelSettings_SetRadius(wheel, self.radius);
            JPH_WheelSettings_SetWidth(wheel, self.width);
        }
    }

    /// What a collision tester needs to know about this wheel.
    pub(super) fn geometry(&self) -> WheelGeometry {
        WheelGeometry {
            suspension_max_length: self.suspension_max_length,
            radius: self.radius,
            width: self.width,
        }
    }
}

/// One wheel of a wheeled vehicle or a motorcycle (Jolt `WheelSettings` and `WheelSettingsWV`).
/// Positions and directions are in the chassis body's local space, lengths in metres. The
/// defaults are Jolt's.
#[derive(Clone, Debug, PartialEq)]
pub struct WheelSettings {
    pub(super) base: WheelBase,
    pub(super) inertia: f32,
    pub(super) angular_damping: f32,
    pub(super) max_steer_angle: f32,
    pub(super) longitudinal_friction: Vec<(f32, f32)>,
    pub(super) lateral_friction: Vec<(f32, f32)>,
    pub(super) max_brake_torque: f32,
    pub(super) max_hand_brake_torque: f32,
}

impl WheelSettings {
    /// A wheel whose suspension is attached to the chassis at `position` (body space, each
    /// component within [`limits::MAX_SHAPE_EXTENT`]).
    pub fn new(position: Vec3) -> Self {
        Self {
            base: WheelBase::new(position),
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
        self.base.suspension_force_point = value;
        self
    }

    /// Direction the suspension extends in, a unit vector pointing down. Default −Y.
    #[must_use]
    pub fn suspension_direction(mut self, value: Vec3) -> Self {
        self.base.suspension_direction = value;
        self
    }

    /// Axis the wheel steers about, a unit vector pointing up. Default +Y.
    #[must_use]
    pub fn steering_axis(mut self, value: Vec3) -> Self {
        self.base.steering_axis = value;
        self
    }

    /// Up of the wheel in the neutral steering position, a unit vector; tilt it for camber.
    /// Default +Y.
    #[must_use]
    pub fn wheel_up(mut self, value: Vec3) -> Self {
        self.base.wheel_up = value;
        self
    }

    /// Forward of the wheel in the neutral steering position, a unit vector perpendicular to
    /// [`wheel_up`](Self::wheel_up); turn it for toe. Default +Z.
    #[must_use]
    pub fn wheel_forward(mut self, value: Vec3) -> Self {
        self.base.wheel_forward = value;
        self
    }

    /// Suspension length when fully raised, from the attachment point, between 0 and
    /// [`limits::MAX_SHAPE_EXTENT`]. Default 0.3.
    #[must_use]
    pub fn suspension_min_length(mut self, value: f32) -> Self {
        self.base.suspension_min_length = value;
        self
    }

    /// Suspension length when fully extended, at least the minimum length and at most
    /// [`limits::MAX_SHAPE_EXTENT`]. Default 0.5.
    #[must_use]
    pub fn suspension_max_length(mut self, value: f32) -> Self {
        self.base.suspension_max_length = value;
        self
    }

    /// How far the spring is already compressed at full extension, between 0 and
    /// [`limits::MAX_SHAPE_EXTENT`]. Default 0.
    #[must_use]
    pub fn suspension_preload_length(mut self, value: f32) -> Self {
        self.base.suspension_preload_length = value;
        self
    }

    /// The suspension spring. Default [`SuspensionSpring::default`]. The stiffness and damping
    /// Jolt derives from it must stay at most [`limits::MAX_SPRING_COEFFICIENT`] for a chassis of
    /// [`limits::MAX_MASS`]: in frequency mode `MAX_MASS·ω²` and `2·MAX_MASS·ζ·ω` with
    /// `ω = 2π·frequency`, in stiffness mode the values themselves.
    #[must_use]
    pub fn suspension_spring(mut self, value: SuspensionSpring) -> Self {
        self.base.suspension_spring = value;
        self
    }

    /// Wheel radius, positive and at most [`limits::MAX_SHAPE_EXTENT`]. Default 0.3.
    #[must_use]
    pub fn radius(mut self, value: f32) -> Self {
        self.base.radius = value;
        self
    }

    /// Wheel width, between 0 and [`limits::MAX_SHAPE_EXTENT`]; positive with
    /// [`VehicleCollisionTester::CastCylinder`](super::VehicleCollisionTester::CastCylinder).
    /// Default 0.1.
    #[must_use]
    pub fn width(mut self, value: f32) -> Self {
        self.base.width = value;
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

    pub(super) fn validate(&self) -> Result<(), VehicleError> {
        let invalid = |what| Err(VehicleError::InvalidValue(what));
        self.base.validate()?;
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
    pub(super) fn create(&self) -> Owned<JPH_WheelSettingsWV> {
        // SAFETY: Jolt is initialised (a world exists). The settings are returned holding one
        // reference, which the guard takes over.
        let wheel = unsafe { Owned::from_raw(JPH_WheelSettingsWV_Create()) }
            .unwrap_or_else(|| unreachable!("joltc `new`s the settings"));
        let longitudinal = create_curve(&self.longitudinal_friction);
        let lateral = create_curve(&self.lateral_friction);
        let ptr = wheel.as_ptr();
        // SAFETY: the settings and both curves are live and owned by their guards; a
        // `WheelSettingsWV` derives from `WheelSettings` with single inheritance, joltc's own
        // cast convention. The setters copy the curves. All values were validated.
        unsafe {
            self.base.apply(ptr.cast());
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
    pub(super) suspension_max_length: f32,
    pub(super) radius: f32,
    pub(super) width: f32,
}

impl WheelGeometry {
    pub(crate) fn of(wheel: &WheelSettings) -> Self {
        wheel.base.geometry()
    }
}

/// An anti-roll bar between two wheels (Jolt `VehicleAntiRollBar`): pushes the less compressed
/// suspension down when the other is compressed more.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VehicleAntiRollBar {
    pub(super) left_wheel: u32,
    pub(super) right_wheel: u32,
    pub(super) stiffness: f32,
}

impl VehicleAntiRollBar {
    /// Largest anti-roll bar stiffness, N/m: about 2.5e11.
    ///
    /// Each step Jolt computes `stiffness · suspension length difference · dt`
    /// (`VehicleConstraint::OnStep`) and uses it as the velocity bias of both wheels'
    /// suspension constraints, which the constraint's effective mass turns into an impulse.
    /// With a length difference of at most [`limits::MAX_SHAPE_EXTENT`] this bound keeps the
    /// bias at most about 5e14 for `dt <= 1`, so the velocity change it gives stays finite;
    /// see [docs/limits.md#anti-roll-bars].
    ///
    /// [docs/limits.md#anti-roll-bars]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#anti-roll-bars
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

    pub(super) fn validate(&self, wheel_count: usize) -> Result<(), VehicleError> {
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
