//! Settings of the constraints that join ragdoll parts: Jolt's swing-twist, hinge and
//! six-degree-of-freedom constraints, with Jolt's defaults.
//!
//! The types here are plain Rust values, checked against what Jolt asserts or silently rewrites
//! before any of them reaches Jolt. Angles are in radians, torques in N·m, forces in N.
//!
//! # Frames
//! Each constraint attaches a frame to each of its two bodies: a point and two perpendicular unit
//! axes. With [`ConstraintSpace::WorldSpace`] (the default) the frames are given in world space at
//! the bodies' creation pose; with [`ConstraintSpace::LocalToBodyCom`] relative to each body's
//! centre of mass. In the constraint frame, X is the twist (or hinge) axis and Y and Z are the
//! swing axes.

use std::f32::consts::PI;

use joltphysics_sys::*;

use crate::math::is_unit;
use crate::{RVec3, Vec3};

/// Tolerance of the perpendicularity checks on unit axes, `|a·b|` at most this.
const PERPENDICULAR_TOLERANCE: f32 = 1.0e-4;

/// Jolt's `ConstraintSettings` defaults: enabled, priority 0, the world's iteration counts.
pub(crate) fn constraint_base() -> JPH_ConstraintSettings {
    JPH_ConstraintSettings {
        enabled: true,
        constraintPriority: 0,
        numVelocityStepsOverride: 0,
        numPositionStepsOverride: 0,
        drawConstraintSize: 1.0,
        userData: 0,
    }
}

/// The space a constraint's frames are given in (Jolt `EConstraintSpace`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum ConstraintSpace {
    /// Relative to each body's centre of mass.
    LocalToBodyCom,
    /// In world space, at the bodies' creation pose. The default.
    #[default]
    WorldSpace,
}

impl ConstraintSpace {
    fn to_jph(self) -> JPH_ConstraintSpace {
        match self {
            Self::LocalToBodyCom => JPH_ConstraintSpace_LocalToBodyCOM,
            Self::WorldSpace => JPH_ConstraintSpace_WorldSpace,
        }
    }
}

/// How the two swing limits combine (Jolt `ESwingType`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum SwingType {
    /// An elliptic cone around the twist axis; its limits are symmetric. The default.
    #[default]
    Cone,
    /// A pyramid: each swing axis has its own, possibly asymmetric, range.
    Pyramid,
}

impl SwingType {
    fn to_jph(self) -> JPH_SwingType {
        match self {
            Self::Cone => JPH_SwingType_Cone,
            Self::Pyramid => JPH_SwingType_Pyramid,
        }
    }
}

/// A spring (Jolt `SpringSettings`). A spring with frequency or stiffness 0 is rigid. The
/// default is rigid: frequency 0, damping 0.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SpringSettings {
    /// Oscillation frequency in Hz and damping ratio (1 is critical damping), both at least 0.
    FrequencyAndDamping {
        /// Hz, finite and at least 0.
        frequency: f32,
        /// Damping ratio, finite and at least 0.
        damping: f32,
    },
    /// Stiffness in N/m (N·m/rad for rotations) and damping in N·s/m (N·m·s/rad), both at
    /// least 0.
    StiffnessAndDamping {
        /// N/m or N·m/rad, finite and at least 0.
        stiffness: f32,
        /// N·s/m or N·m·s/rad, finite and at least 0.
        damping: f32,
    },
}

impl Default for SpringSettings {
    fn default() -> Self {
        Self::FrequencyAndDamping {
            frequency: 0.0,
            damping: 0.0,
        }
    }
}

impl SpringSettings {
    /// Frequency or stiffness, and damping.
    fn values(&self) -> (f32, f32) {
        match *self {
            Self::FrequencyAndDamping { frequency, damping } => (frequency, damping),
            Self::StiffnessAndDamping { stiffness, damping } => (stiffness, damping),
        }
    }

    /// Whether the spring is soft, which is what Jolt tests (`mFrequency > 0`, which also holds
    /// the stiffness).
    fn is_soft(&self) -> bool {
        self.values().0 > 0.0
    }

    fn validate(&self) -> Result<(), &'static str> {
        let (strength, damping) = self.values();
        if non_negative(strength) && non_negative(damping) {
            Ok(())
        } else {
            Err("spring frequency, stiffness and damping must be finite and not negative")
        }
    }

    fn to_jph(self) -> JPH_SpringSettings {
        let (strength, damping) = self.values();
        let mode = match self {
            Self::FrequencyAndDamping { .. } => JPH_SpringMode_FrequencyAndDamping,
            Self::StiffnessAndDamping { .. } => JPH_SpringMode_StiffnessAndDamping,
        };
        JPH_SpringSettings {
            mode,
            frequencyOrStiffness: strength,
            damping,
        }
    }
}

/// A constraint motor (Jolt `MotorSettings`): the spring that drives it to its target and the
/// force and torque it may use. The defaults are Jolt's: frequency 2 Hz, damping 1, and
/// unlimited (`∓f32::MAX`) force and torque.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorSettings {
    spring: SpringSettings,
    min_force_limit: f32,
    max_force_limit: f32,
    min_torque_limit: f32,
    max_torque_limit: f32,
}

impl Default for MotorSettings {
    fn default() -> Self {
        Self {
            spring: SpringSettings::FrequencyAndDamping {
                frequency: 2.0,
                damping: 1.0,
            },
            min_force_limit: -f32::MAX,
            max_force_limit: f32::MAX,
            min_torque_limit: -f32::MAX,
            max_torque_limit: f32::MAX,
        }
    }
}

impl MotorSettings {
    /// The spring that drives a position motor to its target. Default 2 Hz, damping 1.
    #[must_use]
    pub fn spring(mut self, value: SpringSettings) -> Self {
        self.spring = value;
        self
    }

    /// Force limits in N of a translation motor, finite, `min <= max`.
    #[must_use]
    pub fn force_limits(mut self, min: f32, max: f32) -> Self {
        self.min_force_limit = min;
        self.max_force_limit = max;
        self
    }

    /// Torque limits in N·m of a rotation motor, finite, `min <= max`.
    #[must_use]
    pub fn torque_limits(mut self, min: f32, max: f32) -> Self {
        self.min_torque_limit = min;
        self.max_torque_limit = max;
        self
    }

    /// What Jolt's `MotorSettings::IsValid` asserts for a running motor, and finite limits.
    fn validate(&self) -> Result<(), &'static str> {
        self.spring.validate()?;
        let ordered = |min: f32, max: f32| min.is_finite() && max.is_finite() && min <= max;
        if ordered(self.min_force_limit, self.max_force_limit)
            && ordered(self.min_torque_limit, self.max_torque_limit)
        {
            Ok(())
        } else {
            Err("motor force and torque limits must be finite with min <= max")
        }
    }

    fn to_jph(self) -> JPH_MotorSettings {
        JPH_MotorSettings {
            springSettings: self.spring.to_jph(),
            minForceLimit: self.min_force_limit,
            maxForceLimit: self.max_force_limit,
            minTorqueLimit: self.min_torque_limit,
            maxTorqueLimit: self.max_torque_limit,
        }
    }
}

/// Whether `value` is finite and not negative.
fn non_negative(value: f32) -> bool {
    value.is_finite() && value >= 0.0
}

/// Whether `value` is finite and in `[min, max]`.
fn within(value: f32, min: f32, max: f32) -> bool {
    value.is_finite() && (min..=max).contains(&value)
}

/// Checks one frame: a finite point and two perpendicular unit axes.
fn validate_frame(point: RVec3, axis_a: Vec3, axis_b: Vec3) -> Result<(), &'static str> {
    if !point.is_finite() {
        return Err("constraint frame points must be finite");
    }
    if !(is_unit(axis_a) && is_unit(axis_b)) {
        return Err("constraint frame axes must be unit vectors");
    }
    if axis_a.dot(axis_b).abs() > PERPENDICULAR_TOLERANCE {
        return Err("the two axes of a constraint frame must be perpendicular");
    }
    Ok(())
}

/// A swing-twist constraint (Jolt `SwingTwistConstraintSettings`), the usual shoulder, neck or
/// spine joint: body 2 twists about the twist axis within `[twist_min, twist_max]`, and the twist
/// axis swings away from body 1's within a cone (or pyramid) of half angles `normal` (about the
/// normal axis, `twist × plane`) and `plane` (about the plane axis).
///
/// The default is Jolt's: both frames at the origin with twist axis +X and plane axis +Y, a cone
/// with all limits 0 (locked), no friction and default motors.
#[derive(Clone, Debug, PartialEq)]
pub struct SwingTwistConstraintSettings {
    space: ConstraintSpace,
    position1: RVec3,
    twist_axis1: Vec3,
    plane_axis1: Vec3,
    position2: RVec3,
    twist_axis2: Vec3,
    plane_axis2: Vec3,
    swing_type: SwingType,
    normal_half_cone_angle: f32,
    plane_half_cone_angle: f32,
    twist_min_angle: f32,
    twist_max_angle: f32,
    max_friction_torque: f32,
    swing_motor: MotorSettings,
    twist_motor: MotorSettings,
}

impl Default for SwingTwistConstraintSettings {
    fn default() -> Self {
        Self::new(
            RVec3::ZERO,
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
    }
}

impl SwingTwistConstraintSettings {
    /// A joint at `position` with the same frame on both bodies, in world space: `twist_axis`
    /// and the perpendicular `plane_axis` are unit vectors. The limits are 0 (locked) until set.
    pub fn new(position: RVec3, twist_axis: Vec3, plane_axis: Vec3) -> Self {
        Self {
            space: ConstraintSpace::WorldSpace,
            position1: position,
            twist_axis1: twist_axis,
            plane_axis1: plane_axis,
            position2: position,
            twist_axis2: twist_axis,
            plane_axis2: plane_axis,
            swing_type: SwingType::Cone,
            normal_half_cone_angle: 0.0,
            plane_half_cone_angle: 0.0,
            twist_min_angle: 0.0,
            twist_max_angle: 0.0,
            max_friction_torque: 0.0,
            swing_motor: MotorSettings::default(),
            twist_motor: MotorSettings::default(),
        }
    }

    /// The space the frames are given in. Default [`ConstraintSpace::WorldSpace`].
    #[must_use]
    pub fn space(mut self, value: ConstraintSpace) -> Self {
        self.space = value;
        self
    }

    /// The frame on body 1 (the parent): a point and perpendicular unit twist and plane axes.
    #[must_use]
    pub fn frame1(mut self, position: RVec3, twist_axis: Vec3, plane_axis: Vec3) -> Self {
        self.position1 = position;
        self.twist_axis1 = twist_axis;
        self.plane_axis1 = plane_axis;
        self
    }

    /// The frame on body 2 (the child): a point and perpendicular unit twist and plane axes.
    #[must_use]
    pub fn frame2(mut self, position: RVec3, twist_axis: Vec3, plane_axis: Vec3) -> Self {
        self.position2 = position;
        self.twist_axis2 = twist_axis;
        self.plane_axis2 = plane_axis;
        self
    }

    /// How the swing limits combine. Default [`SwingType::Cone`].
    #[must_use]
    pub fn swing_type(mut self, value: SwingType) -> Self {
        self.swing_type = value;
        self
    }

    /// Half angles of the swing cone, radians in `[0, π]`: about the normal axis
    /// (`twist × plane`) and about the plane axis.
    #[must_use]
    pub fn half_cone_angles(mut self, normal: f32, plane: f32) -> Self {
        self.normal_half_cone_angle = normal;
        self.plane_half_cone_angle = plane;
        self
    }

    /// Twist range, radians in `[-π, π]` with `min <= max`.
    #[must_use]
    pub fn twist_limits(mut self, min: f32, max: f32) -> Self {
        self.twist_min_angle = min;
        self.twist_max_angle = max;
        self
    }

    /// Torque in N·m that friction applies while no motor drives the joint, at least 0.
    /// Default 0.
    #[must_use]
    pub fn max_friction_torque(mut self, value: f32) -> Self {
        self.max_friction_torque = value;
        self
    }

    /// The motor that drives the swing.
    #[must_use]
    pub fn swing_motor(mut self, value: MotorSettings) -> Self {
        self.swing_motor = value;
        self
    }

    /// The motor that drives the twist.
    #[must_use]
    pub fn twist_motor(mut self, value: MotorSettings) -> Self {
        self.twist_motor = value;
        self
    }

    pub(crate) fn validate(&self) -> Result<(), &'static str> {
        validate_frame(self.position1, self.twist_axis1, self.plane_axis1)?;
        validate_frame(self.position2, self.twist_axis2, self.plane_axis2)?;
        if !(within(self.normal_half_cone_angle, 0.0, PI)
            && within(self.plane_half_cone_angle, 0.0, PI))
        {
            return Err("swing-twist half cone angles must be between 0 and pi");
        }
        if !(within(self.twist_min_angle, -PI, PI)
            && within(self.twist_max_angle, -PI, PI)
            && self.twist_min_angle <= self.twist_max_angle)
        {
            return Err("swing-twist twist limits must be between -pi and pi with min <= max");
        }
        if !non_negative(self.max_friction_torque) {
            return Err("friction torque must be finite and not negative");
        }
        self.swing_motor.validate()?;
        self.twist_motor.validate()
    }

    pub(crate) fn to_jph(&self) -> JPH_SwingTwistConstraintSettings {
        JPH_SwingTwistConstraintSettings {
            base: constraint_base(),
            space: self.space.to_jph(),
            position1: self.position1.to_jph(),
            twistAxis1: self.twist_axis1.to_jph(),
            planeAxis1: self.plane_axis1.to_jph(),
            position2: self.position2.to_jph(),
            twistAxis2: self.twist_axis2.to_jph(),
            planeAxis2: self.plane_axis2.to_jph(),
            swingType: self.swing_type.to_jph(),
            normalHalfConeAngle: self.normal_half_cone_angle,
            planeHalfConeAngle: self.plane_half_cone_angle,
            twistMinAngle: self.twist_min_angle,
            twistMaxAngle: self.twist_max_angle,
            maxFrictionTorque: self.max_friction_torque,
            swingMotorSettings: self.swing_motor.to_jph(),
            twistMotorSettings: self.twist_motor.to_jph(),
        }
    }
}

/// A hinge (Jolt `HingeConstraintSettings`), the usual elbow or knee joint: body 2 rotates
/// about the hinge axis by an angle in `[limits_min, limits_max]`, measured from the pose where
/// both normal axes coincide.
///
/// The default is Jolt's: both frames at the origin with hinge axis +Y and normal axis +X, no
/// limits (`[-π, π]`), no friction and a default motor.
#[derive(Clone, Debug, PartialEq)]
pub struct HingeConstraintSettings {
    space: ConstraintSpace,
    point1: RVec3,
    hinge_axis1: Vec3,
    normal_axis1: Vec3,
    point2: RVec3,
    hinge_axis2: Vec3,
    normal_axis2: Vec3,
    limits_min: f32,
    limits_max: f32,
    limits_spring: SpringSettings,
    max_friction_torque: f32,
    motor: MotorSettings,
}

impl Default for HingeConstraintSettings {
    fn default() -> Self {
        Self::new(
            RVec3::ZERO,
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
    }
}

impl HingeConstraintSettings {
    /// A hinge at `point` with the same frame on both bodies, in world space: `hinge_axis` and
    /// the perpendicular `normal_axis` are unit vectors. No limits until set.
    pub fn new(point: RVec3, hinge_axis: Vec3, normal_axis: Vec3) -> Self {
        Self {
            space: ConstraintSpace::WorldSpace,
            point1: point,
            hinge_axis1: hinge_axis,
            normal_axis1: normal_axis,
            point2: point,
            hinge_axis2: hinge_axis,
            normal_axis2: normal_axis,
            limits_min: -PI,
            limits_max: PI,
            limits_spring: SpringSettings::default(),
            max_friction_torque: 0.0,
            motor: MotorSettings::default(),
        }
    }

    /// The space the frames are given in. Default [`ConstraintSpace::WorldSpace`].
    #[must_use]
    pub fn space(mut self, value: ConstraintSpace) -> Self {
        self.space = value;
        self
    }

    /// The frame on body 1 (the parent): a point and perpendicular unit hinge and normal axes.
    #[must_use]
    pub fn frame1(mut self, point: RVec3, hinge_axis: Vec3, normal_axis: Vec3) -> Self {
        self.point1 = point;
        self.hinge_axis1 = hinge_axis;
        self.normal_axis1 = normal_axis;
        self
    }

    /// The frame on body 2 (the child): a point and perpendicular unit hinge and normal axes.
    #[must_use]
    pub fn frame2(mut self, point: RVec3, hinge_axis: Vec3, normal_axis: Vec3) -> Self {
        self.point2 = point;
        self.hinge_axis2 = hinge_axis;
        self.normal_axis2 = normal_axis;
        self
    }

    /// The angle range in radians: `min` in `[-π, 0]`, `max` in `[0, π]`; `[-π, π]` means no
    /// limits. `min == max` needs a soft [`limits_spring`](Self::limits_spring).
    #[must_use]
    pub fn limits(mut self, min: f32, max: f32) -> Self {
        self.limits_min = min;
        self.limits_max = max;
        self
    }

    /// Makes the limits soft. Default rigid.
    #[must_use]
    pub fn limits_spring(mut self, value: SpringSettings) -> Self {
        self.limits_spring = value;
        self
    }

    /// Torque in N·m that friction applies while the motor is off, at least 0. Default 0.
    #[must_use]
    pub fn max_friction_torque(mut self, value: f32) -> Self {
        self.max_friction_torque = value;
        self
    }

    /// The motor.
    #[must_use]
    pub fn motor(mut self, value: MotorSettings) -> Self {
        self.motor = value;
        self
    }

    pub(crate) fn validate(&self) -> Result<(), &'static str> {
        validate_frame(self.point1, self.hinge_axis1, self.normal_axis1)?;
        validate_frame(self.point2, self.hinge_axis2, self.normal_axis2)?;
        // Jolt asserts these ranges in `HingeConstraint::SetLimits`.
        if !(within(self.limits_min, -PI, 0.0) && within(self.limits_max, 0.0, PI)) {
            return Err("hinge limits must be min in [-pi, 0] and max in [0, pi]");
        }
        self.limits_spring.validate()?;
        // Jolt asserts this in the `HingeConstraint` constructor.
        if self.limits_min == self.limits_max && !self.limits_spring.is_soft() {
            return Err("hinge limits with min == max need a soft limits spring");
        }
        if !non_negative(self.max_friction_torque) {
            return Err("friction torque must be finite and not negative");
        }
        self.motor.validate()
    }

    pub(crate) fn to_jph(&self) -> JPH_HingeConstraintSettings {
        JPH_HingeConstraintSettings {
            base: constraint_base(),
            space: self.space.to_jph(),
            point1: self.point1.to_jph(),
            hingeAxis1: self.hinge_axis1.to_jph(),
            normalAxis1: self.normal_axis1.to_jph(),
            point2: self.point2.to_jph(),
            hingeAxis2: self.hinge_axis2.to_jph(),
            normalAxis2: self.normal_axis2.to_jph(),
            limitsMin: self.limits_min,
            limitsMax: self.limits_max,
            limitsSpringSettings: self.limits_spring.to_jph(),
            maxFrictionTorque: self.max_friction_torque,
            motorSettings: self.motor.to_jph(),
        }
    }
}

/// One axis of a six-degree-of-freedom constraint (Jolt `SixDOFConstraintSettings::EAxis`).
/// Translations are along, rotations about, the axes of the constraint frame: X is the twist
/// axis, Y and Z the swing axes.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum SixDofConstraintAxis {
    /// Translation along X.
    TranslationX,
    /// Translation along Y.
    TranslationY,
    /// Translation along Z.
    TranslationZ,
    /// Rotation about X, the twist.
    RotationX,
    /// Rotation about Y, a swing.
    RotationY,
    /// Rotation about Z, a swing.
    RotationZ,
}

impl SixDofConstraintAxis {
    /// Every axis, in Jolt's order.
    pub(crate) const ALL: [Self; 6] = [
        Self::TranslationX,
        Self::TranslationY,
        Self::TranslationZ,
        Self::RotationX,
        Self::RotationY,
        Self::RotationZ,
    ];

    /// The three rotation axes.
    pub(crate) const ROTATIONS: [Self; 3] = [Self::RotationX, Self::RotationY, Self::RotationZ];

    /// Jolt's index of the axis.
    pub(crate) fn index(self) -> usize {
        self as usize
    }

    fn is_translation(self) -> bool {
        self.index() < 3
    }

    pub(crate) fn to_jph(self) -> JPH_SixDOFConstraintAxis {
        match self {
            Self::TranslationX => JPH_SixDOFConstraintAxis_TranslationX,
            Self::TranslationY => JPH_SixDOFConstraintAxis_TranslationY,
            Self::TranslationZ => JPH_SixDOFConstraintAxis_TranslationZ,
            Self::RotationX => JPH_SixDOFConstraintAxis_RotationX,
            Self::RotationY => JPH_SixDOFConstraintAxis_RotationY,
            Self::RotationZ => JPH_SixDOFConstraintAxis_RotationZ,
        }
    }
}

/// How one axis of a six-degree-of-freedom constraint may move.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SixDofAxis {
    /// Moves freely (Jolt `MakeFreeAxis`). The default.
    Free,
    /// Does not move (Jolt `MakeFixedAxis`).
    Fixed,
    /// Moves within `[min, max]`, `min < max`: metres for translations, radians in `[-π, π]`
    /// for rotations.
    Limited {
        /// Lower limit.
        min: f32,
        /// Upper limit.
        max: f32,
    },
}

impl SixDofAxis {
    /// The limits Jolt stores for this axis.
    fn limits(self) -> (f32, f32) {
        match self {
            Self::Free => (-f32::MAX, f32::MAX),
            Self::Fixed => (f32::MAX, -f32::MAX),
            Self::Limited { min, max } => (min, max),
        }
    }
}

/// A six-degree-of-freedom constraint (Jolt `SixDOFConstraintSettings`): each translation and
/// rotation axis of the constraint frame is free, fixed or limited on its own. With
/// [`SwingType::Pyramid`] the two swing ranges may be asymmetric, which suits hips.
///
/// The default is Jolt's: both frames at the origin with axes +X and +Y, every axis free, a cone
/// swing, no friction, rigid translation limits and default motors.
#[derive(Clone, Debug, PartialEq)]
pub struct SixDofConstraintSettings {
    space: ConstraintSpace,
    position1: RVec3,
    axis_x1: Vec3,
    axis_y1: Vec3,
    position2: RVec3,
    axis_x2: Vec3,
    axis_y2: Vec3,
    swing_type: SwingType,
    axes: [SixDofAxis; 6],
    max_friction: [f32; 6],
    limits_springs: [SpringSettings; 6],
    motors: [MotorSettings; 6],
}

impl Default for SixDofConstraintSettings {
    fn default() -> Self {
        Self::new(
            RVec3::ZERO,
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
    }
}

impl SixDofConstraintSettings {
    /// A joint at `position` with the same frame on both bodies, in world space: `axis_x` (the
    /// twist axis) and the perpendicular `axis_y` are unit vectors. Every axis is free until set.
    pub fn new(position: RVec3, axis_x: Vec3, axis_y: Vec3) -> Self {
        Self {
            space: ConstraintSpace::WorldSpace,
            position1: position,
            axis_x1: axis_x,
            axis_y1: axis_y,
            position2: position,
            axis_x2: axis_x,
            axis_y2: axis_y,
            swing_type: SwingType::Cone,
            axes: [SixDofAxis::Free; 6],
            max_friction: [0.0; 6],
            limits_springs: [SpringSettings::default(); 6],
            motors: [MotorSettings::default(); 6],
        }
    }

    /// The space the frames are given in. Default [`ConstraintSpace::WorldSpace`].
    #[must_use]
    pub fn space(mut self, value: ConstraintSpace) -> Self {
        self.space = value;
        self
    }

    /// The frame on body 1 (the parent): a point and perpendicular unit X and Y axes.
    #[must_use]
    pub fn frame1(mut self, position: RVec3, axis_x: Vec3, axis_y: Vec3) -> Self {
        self.position1 = position;
        self.axis_x1 = axis_x;
        self.axis_y1 = axis_y;
        self
    }

    /// The frame on body 2 (the child): a point and perpendicular unit X and Y axes.
    #[must_use]
    pub fn frame2(mut self, position: RVec3, axis_x: Vec3, axis_y: Vec3) -> Self {
        self.position2 = position;
        self.axis_x2 = axis_x;
        self.axis_y2 = axis_y;
        self
    }

    /// How the swing limits combine. Default [`SwingType::Cone`], which needs symmetric
    /// rotation Y and Z limits; [`SwingType::Pyramid`] allows asymmetric ones.
    #[must_use]
    pub fn swing_type(mut self, value: SwingType) -> Self {
        self.swing_type = value;
        self
    }

    /// How `which` may move. Default [`SixDofAxis::Free`].
    #[must_use]
    pub fn axis(mut self, which: SixDofConstraintAxis, value: SixDofAxis) -> Self {
        self.axes[which.index()] = value;
        self
    }

    /// Force (N) or torque (N·m) friction applies along `which` while its motor is off, at
    /// least 0. Default 0.
    #[must_use]
    pub fn max_friction(mut self, which: SixDofConstraintAxis, value: f32) -> Self {
        self.max_friction[which.index()] = value;
        self
    }

    /// Makes the limits of the limited translation axis `which` soft. Jolt has limit springs for
    /// translations only. Default rigid.
    #[must_use]
    pub fn limits_spring(mut self, which: SixDofConstraintAxis, value: SpringSettings) -> Self {
        self.limits_springs[which.index()] = value;
        self
    }

    /// The motor of `which`.
    #[must_use]
    pub fn motor(mut self, which: SixDofConstraintAxis, value: MotorSettings) -> Self {
        self.motors[which.index()] = value;
        self
    }

    pub(crate) fn validate(&self) -> Result<(), &'static str> {
        validate_frame(self.position1, self.axis_x1, self.axis_y1)?;
        validate_frame(self.position2, self.axis_x2, self.axis_y2)?;
        for which in SixDofConstraintAxis::ALL {
            let i = which.index();
            if let SixDofAxis::Limited { min, max } = self.axes[i] {
                if !(min.is_finite() && max.is_finite() && min < max) {
                    return Err(
                        "limited axes need finite limits with min < max; use Fixed for min == max",
                    );
                }
                let in_angle_range = within(min, -PI, PI) && within(max, -PI, PI);
                if !(which.is_translation() || in_angle_range) {
                    return Err("rotation limits must be between -pi and pi");
                }
            }
            if !non_negative(self.max_friction[i]) {
                return Err("friction must be finite and not negative");
            }
            let spring = self.limits_springs[i];
            spring.validate()?;
            // Jolt uses limit springs that are soft (`SixDOFConstraint::CacheHasSpringLimits`).
            if spring.is_soft() {
                if !which.is_translation() {
                    return Err("limit springs exist only for translation axes");
                }
                if self.axes[i] == SixDofAxis::Free {
                    return Err("a limit spring needs a limited or fixed translation axis");
                }
            }
            self.motors[i].validate()?;
        }
        // Jolt silently makes cone swing limits symmetric (`SixDOFConstraint::UpdateRotationLimits`).
        if self.swing_type == SwingType::Cone {
            for which in [
                SixDofConstraintAxis::RotationY,
                SixDofConstraintAxis::RotationZ,
            ] {
                if let SixDofAxis::Limited { min, max } = self.axes[which.index()] {
                    if min != -max {
                        return Err(
                            "cone swing limits must be symmetric; use SwingType::Pyramid for asymmetric ones",
                        );
                    }
                }
            }
        }
        Ok(())
    }

    pub(crate) fn to_jph(&self) -> JPH_SixDOFConstraintSettings {
        let limits = self.axes.map(SixDofAxis::limits);
        JPH_SixDOFConstraintSettings {
            base: constraint_base(),
            space: self.space.to_jph(),
            position1: self.position1.to_jph(),
            axisX1: self.axis_x1.to_jph(),
            axisY1: self.axis_y1.to_jph(),
            position2: self.position2.to_jph(),
            axisX2: self.axis_x2.to_jph(),
            axisY2: self.axis_y2.to_jph(),
            maxFriction: self.max_friction,
            swingType: self.swing_type.to_jph(),
            limitMin: limits.map(|(min, _)| min),
            limitMax: limits.map(|(_, max)| max),
            limitsSpringSettings: [0, 1, 2].map(|i| self.limits_springs[i].to_jph()),
            motorSettings: self.motors.map(MotorSettings::to_jph),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::world::ensure_initialized;
    use crate::Real;

    fn assert_spring_eq(ours: JPH_SpringSettings, jolt: JPH_SpringSettings) {
        assert_eq!(ours.mode, jolt.mode);
        assert_eq!(ours.frequencyOrStiffness, jolt.frequencyOrStiffness);
        assert_eq!(ours.damping, jolt.damping);
    }

    fn assert_motor_eq(ours: JPH_MotorSettings, jolt: JPH_MotorSettings) {
        assert_spring_eq(ours.springSettings, jolt.springSettings);
        assert_eq!(ours.minForceLimit, jolt.minForceLimit);
        assert_eq!(ours.maxForceLimit, jolt.maxForceLimit);
        assert_eq!(ours.minTorqueLimit, jolt.minTorqueLimit);
        assert_eq!(ours.maxTorqueLimit, jolt.maxTorqueLimit);
    }

    fn assert_base_eq(ours: JPH_ConstraintSettings, jolt: JPH_ConstraintSettings) {
        assert_eq!(ours.enabled, jolt.enabled);
        assert_eq!(ours.constraintPriority, jolt.constraintPriority);
        assert_eq!(ours.numVelocityStepsOverride, jolt.numVelocityStepsOverride);
        assert_eq!(ours.numPositionStepsOverride, jolt.numPositionStepsOverride);
        assert_eq!(ours.drawConstraintSize, jolt.drawConstraintSize);
        assert_eq!(ours.userData, jolt.userData);
    }

    fn bits(v: JPH_Vec3) -> [u32; 3] {
        [v.x, v.y, v.z].map(f32::to_bits)
    }

    #[test]
    fn constraint_base_is_jolts_default() {
        assert!(ensure_initialized());
        // SAFETY: an all-zero `JPH_VehicleConstraintSettings` is valid: floats, integers,
        // `false` and null pointers. joltc fills it with Jolt's defaults and allocates nothing.
        let mut settings: JPH_VehicleConstraintSettings = unsafe { std::mem::zeroed() };
        // SAFETY: Jolt is initialised and `settings` is a live local.
        unsafe { JPH_VehicleConstraintSettings_Init(&mut settings) };
        assert_base_eq(constraint_base(), settings.base);
    }

    #[test]
    fn swing_twist_defaults_are_jolts() {
        assert!(ensure_initialized());
        // SAFETY: an all-zero `JPH_SwingTwistConstraintSettings` is valid: floats, integers and
        // enums with a zero value. joltc fills it with Jolt's defaults and allocates nothing.
        let mut jolt: JPH_SwingTwistConstraintSettings = unsafe { std::mem::zeroed() };
        // SAFETY: Jolt is initialised and `jolt` is a live local.
        unsafe { JPH_SwingTwistConstraintSettings_Init(&mut jolt) };
        let ours = SwingTwistConstraintSettings::default();
        assert_eq!(ours.validate(), Ok(()));
        let ours = ours.to_jph();
        assert_base_eq(ours.base, jolt.base);
        assert_eq!(ours.space, jolt.space);
        assert_eq!(
            RVec3::from_jph(ours.position1),
            RVec3::from_jph(jolt.position1)
        );
        assert_eq!(bits(ours.twistAxis1), bits(jolt.twistAxis1));
        assert_eq!(bits(ours.planeAxis1), bits(jolt.planeAxis1));
        assert_eq!(
            RVec3::from_jph(ours.position2),
            RVec3::from_jph(jolt.position2)
        );
        assert_eq!(bits(ours.twistAxis2), bits(jolt.twistAxis2));
        assert_eq!(bits(ours.planeAxis2), bits(jolt.planeAxis2));
        assert_eq!(ours.swingType, jolt.swingType);
        assert_eq!(ours.normalHalfConeAngle, jolt.normalHalfConeAngle);
        assert_eq!(ours.planeHalfConeAngle, jolt.planeHalfConeAngle);
        assert_eq!(ours.twistMinAngle, jolt.twistMinAngle);
        assert_eq!(ours.twistMaxAngle, jolt.twistMaxAngle);
        assert_eq!(ours.maxFrictionTorque, jolt.maxFrictionTorque);
        assert_motor_eq(ours.swingMotorSettings, jolt.swingMotorSettings);
        assert_motor_eq(ours.twistMotorSettings, jolt.twistMotorSettings);
    }

    #[test]
    fn hinge_defaults_are_jolts() {
        assert!(ensure_initialized());
        // SAFETY: as in `swing_twist_defaults_are_jolts`, for `JPH_HingeConstraintSettings`.
        let mut jolt: JPH_HingeConstraintSettings = unsafe { std::mem::zeroed() };
        // SAFETY: Jolt is initialised and `jolt` is a live local.
        unsafe { JPH_HingeConstraintSettings_Init(&mut jolt) };
        let ours = HingeConstraintSettings::default();
        assert_eq!(ours.validate(), Ok(()));
        let ours = ours.to_jph();
        assert_base_eq(ours.base, jolt.base);
        assert_eq!(ours.space, jolt.space);
        assert_eq!(RVec3::from_jph(ours.point1), RVec3::from_jph(jolt.point1));
        assert_eq!(bits(ours.hingeAxis1), bits(jolt.hingeAxis1));
        assert_eq!(bits(ours.normalAxis1), bits(jolt.normalAxis1));
        assert_eq!(RVec3::from_jph(ours.point2), RVec3::from_jph(jolt.point2));
        assert_eq!(bits(ours.hingeAxis2), bits(jolt.hingeAxis2));
        assert_eq!(bits(ours.normalAxis2), bits(jolt.normalAxis2));
        assert_eq!(ours.limitsMin, jolt.limitsMin);
        assert_eq!(ours.limitsMax, jolt.limitsMax);
        assert_spring_eq(ours.limitsSpringSettings, jolt.limitsSpringSettings);
        assert_eq!(ours.maxFrictionTorque, jolt.maxFrictionTorque);
        assert_motor_eq(ours.motorSettings, jolt.motorSettings);
    }

    #[test]
    fn six_dof_defaults_are_jolts() {
        assert!(ensure_initialized());
        // SAFETY: as in `swing_twist_defaults_are_jolts`, for `JPH_SixDOFConstraintSettings`.
        let mut jolt: JPH_SixDOFConstraintSettings = unsafe { std::mem::zeroed() };
        // SAFETY: Jolt is initialised and `jolt` is a live local.
        unsafe { JPH_SixDOFConstraintSettings_Init(&mut jolt) };
        let ours = SixDofConstraintSettings::default();
        assert_eq!(ours.validate(), Ok(()));
        let ours = ours.to_jph();
        assert_base_eq(ours.base, jolt.base);
        assert_eq!(ours.space, jolt.space);
        assert_eq!(
            RVec3::from_jph(ours.position1),
            RVec3::from_jph(jolt.position1)
        );
        assert_eq!(bits(ours.axisX1), bits(jolt.axisX1));
        assert_eq!(bits(ours.axisY1), bits(jolt.axisY1));
        assert_eq!(
            RVec3::from_jph(ours.position2),
            RVec3::from_jph(jolt.position2)
        );
        assert_eq!(bits(ours.axisX2), bits(jolt.axisX2));
        assert_eq!(bits(ours.axisY2), bits(jolt.axisY2));
        assert_eq!(ours.maxFriction, jolt.maxFriction);
        assert_eq!(ours.swingType, jolt.swingType);
        assert_eq!(ours.limitMin, jolt.limitMin);
        assert_eq!(ours.limitMax, jolt.limitMax);
        for (a, b) in ours
            .limitsSpringSettings
            .into_iter()
            .zip(jolt.limitsSpringSettings)
        {
            assert_spring_eq(a, b);
        }
        for (a, b) in ours.motorSettings.into_iter().zip(jolt.motorSettings) {
            assert_motor_eq(a, b);
        }
    }

    #[test]
    fn fixed_and_free_map_to_jolts_sentinels() {
        assert!(ensure_initialized());
        let x = SixDofConstraintAxis::TranslationX;
        let ours = SixDofConstraintSettings::default()
            .axis(x, SixDofAxis::Fixed)
            .to_jph();
        // SAFETY: as in `six_dof_defaults_are_jolts`.
        let mut jolt: JPH_SixDOFConstraintSettings = unsafe { std::mem::zeroed() };
        // SAFETY: Jolt is initialised and `jolt` is a live local; the setters write its arrays.
        unsafe {
            JPH_SixDOFConstraintSettings_Init(&mut jolt);
            JPH_SixDOFConstraintSettings_MakeFixedAxis(&mut jolt, x.to_jph());
            assert!(JPH_SixDOFConstraintSettings_IsFixedAxis(&ours, x.to_jph()));
        }
        assert_eq!(ours.limitMin, jolt.limitMin);
        assert_eq!(ours.limitMax, jolt.limitMax);
        let free = SixDofConstraintSettings::default().to_jph();
        // SAFETY: `free` is a live local; the getter only reads it.
        assert!(unsafe { JPH_SixDOFConstraintSettings_IsFreeAxis(&free, x.to_jph()) });
    }

    const X: Vec3 = Vec3::new(1.0, 0.0, 0.0);
    const Y: Vec3 = Vec3::new(0.0, 1.0, 0.0);

    fn swing_twist() -> SwingTwistConstraintSettings {
        SwingTwistConstraintSettings::new(RVec3::ZERO, X, Y)
    }

    fn hinge() -> HingeConstraintSettings {
        HingeConstraintSettings::new(RVec3::ZERO, X, Y)
    }

    fn six_dof() -> SixDofConstraintSettings {
        SixDofConstraintSettings::new(RVec3::ZERO, X, Y)
    }

    #[test]
    fn frames_are_validated() {
        let skewed = Vec3::new(0.0, 0.00005, 1.0).normalized_or_zero();
        let tilted = Vec3::new(0.001, 1.0, 0.0).normalized_or_zero();
        let not_unit = Vec3::new(0.0, 1.1, 0.0);
        let nan = RVec3::new(Real::NAN, 0.0, 0.0);
        assert!(swing_twist().frame1(nan, X, Y).validate().is_err());
        assert!(swing_twist()
            .frame2(RVec3::ZERO, X, not_unit)
            .validate()
            .is_err());
        assert!(hinge().frame1(RVec3::ZERO, X, tilted).validate().is_err());
        assert!(six_dof().frame2(RVec3::ZERO, Y, X).validate().is_ok());
        assert!(six_dof().frame2(RVec3::ZERO, skewed, Y).validate().is_ok());
        assert!(six_dof().frame2(RVec3::ZERO, X, tilted).validate().is_err());
    }

    #[test]
    fn swing_twist_limits_are_validated() {
        assert!(swing_twist().half_cone_angles(PI, 0.0).validate().is_ok());
        assert!(swing_twist()
            .half_cone_angles(PI + 0.01, 0.0)
            .validate()
            .is_err());
        assert!(swing_twist()
            .half_cone_angles(0.5, -0.01)
            .validate()
            .is_err());
        assert!(swing_twist().twist_limits(-PI, PI).validate().is_ok());
        assert!(swing_twist()
            .twist_limits(-PI - 0.01, 0.0)
            .validate()
            .is_err());
        assert!(swing_twist().twist_limits(0.3, 0.2).validate().is_err());
        assert!(swing_twist().twist_limits(0.2, 0.2).validate().is_ok());
        assert!(swing_twist().max_friction_torque(-1.0).validate().is_err());
        assert!(swing_twist()
            .max_friction_torque(f32::NAN)
            .validate()
            .is_err());
    }

    #[test]
    fn hinge_limits_are_validated() {
        assert!(hinge().limits(-PI, PI).validate().is_ok());
        assert!(hinge().limits(-1.0, 0.0).validate().is_ok());
        assert!(hinge().limits(0.0, 0.5).validate().is_ok());
        assert!(hinge().limits(0.1, 0.5).validate().is_err());
        assert!(hinge().limits(-0.5, -0.1).validate().is_err());
        assert!(hinge().limits(-PI - 0.01, 0.0).validate().is_err());
        assert!(hinge().limits(0.0, 0.0).validate().is_err());
        let soft = SpringSettings::FrequencyAndDamping {
            frequency: 5.0,
            damping: 0.5,
        };
        assert!(hinge()
            .limits(0.0, 0.0)
            .limits_spring(soft)
            .validate()
            .is_ok());
    }

    #[test]
    fn six_dof_limits_are_validated() {
        let rx = SixDofConstraintAxis::RotationX;
        let ry = SixDofConstraintAxis::RotationY;
        let tx = SixDofConstraintAxis::TranslationX;
        let limited = |min, max| SixDofAxis::Limited { min, max };
        assert!(six_dof().axis(rx, limited(-PI, PI)).validate().is_ok());
        assert!(six_dof()
            .axis(rx, limited(-PI - 0.01, 0.0))
            .validate()
            .is_err());
        assert!(six_dof().axis(rx, limited(0.2, 0.2)).validate().is_err());
        assert!(six_dof().axis(tx, limited(-5.0, 5.0)).validate().is_ok());
        assert!(six_dof()
            .axis(tx, limited(f32::NEG_INFINITY, 0.0))
            .validate()
            .is_err());
        // Cone swings are symmetric; asymmetric ones need a pyramid.
        assert!(six_dof().axis(ry, limited(-0.3, 1.6)).validate().is_err());
        assert!(six_dof().axis(ry, limited(-0.5, 0.5)).validate().is_ok());
        assert!(six_dof()
            .swing_type(SwingType::Pyramid)
            .axis(ry, limited(-0.3, 1.6))
            .validate()
            .is_ok());
        let soft = SpringSettings::StiffnessAndDamping {
            stiffness: 100.0,
            damping: 1.0,
        };
        assert!(six_dof().limits_spring(tx, soft).validate().is_err());
        assert!(six_dof()
            .axis(tx, limited(-0.1, 0.1))
            .limits_spring(tx, soft)
            .validate()
            .is_ok());
        assert!(six_dof()
            .axis(rx, limited(-0.1, 0.1))
            .limits_spring(rx, soft)
            .validate()
            .is_err());
        assert!(six_dof().max_friction(rx, -1.0).validate().is_err());
    }

    #[test]
    fn motors_and_springs_are_validated() {
        let negative = SpringSettings::FrequencyAndDamping {
            frequency: -1.0,
            damping: 0.0,
        };
        let motor = MotorSettings::default();
        assert!(hinge().motor(motor.spring(negative)).validate().is_err());
        assert!(hinge()
            .motor(motor.torque_limits(1.0, -1.0))
            .validate()
            .is_err());
        assert!(hinge()
            .motor(motor.force_limits(f32::NEG_INFINITY, 0.0))
            .validate()
            .is_err());
        assert!(hinge()
            .motor(motor.torque_limits(-50.0, 50.0))
            .validate()
            .is_ok());
        assert!(hinge().limits_spring(negative).validate().is_err());
        assert!(swing_twist()
            .twist_motor(motor.spring(negative))
            .validate()
            .is_err());
        assert!(six_dof()
            .motor(SixDofConstraintAxis::RotationZ, motor.spring(negative))
            .validate()
            .is_err());
    }
}
