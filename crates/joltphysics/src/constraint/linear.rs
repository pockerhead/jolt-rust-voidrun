//! World constraints that hold points together, apart or on an axis: fixed, point, distance,
//! slider and pulley.

use std::ptr::NonNull;

use joltphysics_sys::*;

use super::rotational::{check_friction, check_motor};
use super::world::{check_spring, sealed, ConstraintSettings};
use super::{
    constraint_base, non_negative, point_anchors, validate_frame, validate_point, within,
    ConstraintSpace, MotorSettings, SpringSettings,
};
use crate::body::with_locked_bodies;
use crate::limits;
use crate::owned::Owned;
use crate::{
    ConstraintError, ConstraintMut, ConstraintRef, ConstraintType, DistanceConstraint,
    FixedConstraint, MotorState, PhysicsWorld, PointConstraint, PulleyConstraint, RVec3,
    SliderConstraint, Vec3,
};

/// A fixed constraint (Jolt `FixedConstraintSettings`): body 2 keeps its position and rotation
/// relative to body 1, as if the two were welded.
///
/// The default is Jolt's: both frames at the origin with axes +X and +Y, world space, no
/// automatic point.
#[derive(Clone, Debug, PartialEq)]
pub struct FixedConstraintSettings {
    space: ConstraintSpace,
    auto_detect_point: bool,
    point1: RVec3,
    axis_x1: Vec3,
    axis_y1: Vec3,
    point2: RVec3,
    axis_x2: Vec3,
    axis_y2: Vec3,
}

impl Default for FixedConstraintSettings {
    fn default() -> Self {
        Self::new(
            RVec3::ZERO,
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
    }
}

impl FixedConstraintSettings {
    /// A weld at `point` with the same frame on both bodies, in world space: `axis_x` and the
    /// perpendicular `axis_y` are unit vectors.
    pub fn new(point: RVec3, axis_x: Vec3, axis_y: Vec3) -> Self {
        Self {
            space: ConstraintSpace::WorldSpace,
            auto_detect_point: false,
            point1: point,
            axis_x1: axis_x,
            axis_y1: axis_y,
            point2: point,
            axis_x2: axis_x,
            axis_y2: axis_y,
        }
    }

    /// The space the frames are given in. Default [`ConstraintSpace::WorldSpace`].
    #[must_use]
    pub fn space(mut self, value: ConstraintSpace) -> Self {
        self.space = value;
        self
    }

    /// The frame on body 1: a point and perpendicular unit X and Y axes.
    #[must_use]
    pub fn frame1(mut self, point: RVec3, axis_x: Vec3, axis_y: Vec3) -> Self {
        self.point1 = point;
        self.axis_x1 = axis_x;
        self.axis_y1 = axis_y;
        self
    }

    /// The frame on body 2: a point and perpendicular unit X and Y axes.
    #[must_use]
    pub fn frame2(mut self, point: RVec3, axis_x: Vec3, axis_y: Vec3) -> Self {
        self.point2 = point;
        self.axis_x2 = axis_x;
        self.axis_y2 = axis_y;
        self
    }

    /// Lets Jolt place the weld point between the bodies as they are when the constraint is
    /// created, instead of the given points. Only in [`ConstraintSpace::WorldSpace`].
    #[must_use]
    pub fn auto_detect_point(mut self) -> Self {
        self.auto_detect_point = true;
        self
    }

    fn to_jph(&self) -> JPH_FixedConstraintSettings {
        JPH_FixedConstraintSettings {
            base: constraint_base(),
            space: self.space.to_jph(),
            autoDetectPoint: self.auto_detect_point,
            point1: self.point1.to_jph(),
            axisX1: self.axis_x1.to_jph(),
            axisY1: self.axis_y1.to_jph(),
            point2: self.point2.to_jph(),
            axisX2: self.axis_x2.to_jph(),
            axisY2: self.axis_y2.to_jph(),
        }
    }
}

impl sealed::Settings for FixedConstraintSettings {
    fn validate(&self) -> Result<(), &'static str> {
        validate_frame(self.point1, self.axis_x1, self.axis_y1)?;
        validate_frame(self.point2, self.axis_x2, self.axis_y2)?;
        // Jolt reads the bodies' world positions for the point, which only means something in
        // world space.
        if self.auto_detect_point && self.space != ConstraintSpace::WorldSpace {
            return Err("an automatic point needs ConstraintSpace::WorldSpace");
        }
        Ok(())
    }

    fn springs(&self) -> Vec<SpringSettings> {
        Vec::new()
    }

    fn anchors(&self) -> Option<[sealed::Anchor; 2]> {
        Some(if self.auto_detect_point {
            [sealed::Anchor::BetweenCentersOfMass; 2]
        } else {
            point_anchors(self.space, self.point1, self.point2)
        })
    }

    unsafe fn create(
        &self,
        body1: NonNull<JPH_Body>,
        body2: NonNull<JPH_Body>,
    ) -> *mut JPH_Constraint {
        let settings = self.to_jph();
        // SAFETY: the caller locks both live bodies (trait contract); `settings` is a live,
        // validated local that joltc converts.
        unsafe { JPH_FixedConstraint_Create(&settings, body1.as_ptr(), body2.as_ptr()) }.cast()
    }
}

impl ConstraintSettings for FixedConstraintSettings {
    type Kind = FixedConstraint;
}

impl ConstraintRef<'_, FixedConstraint> {
    /// The impulse in N·s the position part applied in the last step.
    pub fn total_lambda_position(&self) -> Vec3 {
        let mut value = Vec3::ZERO.to_jph();
        // SAFETY: the world borrowed here owns the constraint; the getter reads a member, and
        // `value` is a live local.
        unsafe { JPH_FixedConstraint_GetTotalLambdaPosition(self.ptr(), &mut value) };
        Vec3::from_jph(value)
    }

    /// The angular impulse in N·m·s the rotation part applied in the last step.
    pub fn total_lambda_rotation(&self) -> Vec3 {
        let mut value = Vec3::ZERO.to_jph();
        // SAFETY: as in `total_lambda_position`.
        unsafe { JPH_FixedConstraint_GetTotalLambdaRotation(self.ptr(), &mut value) };
        Vec3::from_jph(value)
    }
}

/// A point constraint (Jolt `PointConstraintSettings`): a ball joint that keeps a point of
/// body 1 on a point of body 2 and leaves the rotation free.
///
/// The default is Jolt's: both points at the origin, world space.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct PointConstraintSettings {
    space: ConstraintSpace,
    point1: RVec3,
    point2: RVec3,
}

impl PointConstraintSettings {
    /// A ball joint at `point` on both bodies, in world space.
    pub fn new(point: RVec3) -> Self {
        Self {
            space: ConstraintSpace::WorldSpace,
            point1: point,
            point2: point,
        }
    }

    /// The space the points are given in. Default [`ConstraintSpace::WorldSpace`].
    #[must_use]
    pub fn space(mut self, value: ConstraintSpace) -> Self {
        self.space = value;
        self
    }

    /// The point on body 1.
    #[must_use]
    pub fn point1(mut self, value: RVec3) -> Self {
        self.point1 = value;
        self
    }

    /// The point on body 2.
    #[must_use]
    pub fn point2(mut self, value: RVec3) -> Self {
        self.point2 = value;
        self
    }
}

impl sealed::Settings for PointConstraintSettings {
    fn validate(&self) -> Result<(), &'static str> {
        validate_point(self.point1)?;
        validate_point(self.point2)
    }

    fn springs(&self) -> Vec<SpringSettings> {
        Vec::new()
    }

    fn anchors(&self) -> Option<[sealed::Anchor; 2]> {
        Some(point_anchors(self.space, self.point1, self.point2))
    }

    unsafe fn create(
        &self,
        body1: NonNull<JPH_Body>,
        body2: NonNull<JPH_Body>,
    ) -> *mut JPH_Constraint {
        let settings = JPH_PointConstraintSettings {
            base: constraint_base(),
            space: self.space.to_jph(),
            point1: self.point1.to_jph(),
            point2: self.point2.to_jph(),
        };
        // SAFETY: as in `FixedConstraintSettings::create`.
        unsafe { JPH_PointConstraint_Create(&settings, body1.as_ptr(), body2.as_ptr()) }.cast()
    }
}

impl ConstraintSettings for PointConstraintSettings {
    type Kind = PointConstraint;
}

impl ConstraintRef<'_, PointConstraint> {
    /// The impulse in N·s the constraint applied in the last step.
    pub fn total_lambda_position(&self) -> Vec3 {
        let mut value = Vec3::ZERO.to_jph();
        // SAFETY: the world borrowed here owns the constraint; the getter reads a member, and
        // `value` is a live local.
        unsafe { JPH_PointConstraint_GetTotalLambdaPosition(self.ptr(), &mut value) };
        Vec3::from_jph(value)
    }
}

/// The allowed distance range of a [`DistanceConstraintSettings`].
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum DistanceRange {
    /// Exactly the distance between the two points when the constraint is created, a rigid rod.
    /// The default (Jolt's `-1, -1`).
    #[default]
    Current,
    /// From `min` to `max` metres: `0 <= min <= max <=` [`limits::MAX_SHAPE_EXTENT`]. `min == 0`
    /// with a larger `max` is a rope.
    Range {
        /// Shortest distance, metres.
        min: f32,
        /// Longest distance, metres.
        max: f32,
    },
}

impl DistanceRange {
    fn limits(self) -> (f32, f32) {
        match self {
            Self::Current => (-1.0, -1.0),
            Self::Range { min, max } => (min, max),
        }
    }
}

/// What a distance range must satisfy.
fn validate_distance_range(min: f32, max: f32) -> Result<(), &'static str> {
    if within(min, 0.0, limits::MAX_SHAPE_EXTENT)
        && within(max, 0.0, limits::MAX_SHAPE_EXTENT)
        && min <= max
    {
        Ok(())
    } else {
        Err("distances must be 0 <= min <= max <= limits::MAX_SHAPE_EXTENT")
    }
}

/// A distance constraint (Jolt `DistanceConstraintSettings`): keeps the distance between a point
/// of body 1 and a point of body 2 within a range; the bodies rotate freely.
///
/// The default is Jolt's: both points at the origin, world space, the distance at creation, a
/// rigid limit.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct DistanceConstraintSettings {
    space: ConstraintSpace,
    point1: RVec3,
    point2: RVec3,
    range: DistanceRange,
    limits_spring: SpringSettings,
}

impl DistanceConstraintSettings {
    /// A rod from `point1` on body 1 to `point2` on body 2, in world space, of the length it has
    /// when the constraint is created.
    pub fn new(point1: RVec3, point2: RVec3) -> Self {
        Self {
            point1,
            point2,
            ..Self::default()
        }
    }

    /// The space the points are given in. Default [`ConstraintSpace::WorldSpace`].
    #[must_use]
    pub fn space(mut self, value: ConstraintSpace) -> Self {
        self.space = value;
        self
    }

    /// The allowed distance. Default [`DistanceRange::Current`].
    #[must_use]
    pub fn range(mut self, value: DistanceRange) -> Self {
        self.range = value;
        self
    }

    /// Makes the range limits soft: a spring pulls the points back into the range. Default
    /// rigid.
    #[must_use]
    pub fn limits_spring(mut self, value: SpringSettings) -> Self {
        self.limits_spring = value;
        self
    }
}

impl sealed::Settings for DistanceConstraintSettings {
    fn validate(&self) -> Result<(), &'static str> {
        validate_point(self.point1)?;
        validate_point(self.point2)?;
        if let DistanceRange::Range { min, max } = self.range {
            validate_distance_range(min, max)?;
        }
        self.limits_spring.validate()
    }

    fn springs(&self) -> Vec<SpringSettings> {
        vec![self.limits_spring]
    }

    fn anchors(&self) -> Option<[sealed::Anchor; 2]> {
        Some(point_anchors(self.space, self.point1, self.point2))
    }

    unsafe fn create(
        &self,
        body1: NonNull<JPH_Body>,
        body2: NonNull<JPH_Body>,
    ) -> *mut JPH_Constraint {
        let (min, max) = self.range.limits();
        let settings = JPH_DistanceConstraintSettings {
            base: constraint_base(),
            space: self.space.to_jph(),
            point1: self.point1.to_jph(),
            point2: self.point2.to_jph(),
            minDistance: min,
            maxDistance: max,
            limitsSpringSettings: self.limits_spring.to_jph(),
        };
        // SAFETY: as in `FixedConstraintSettings::create`.
        unsafe { JPH_DistanceConstraint_Create(&settings, body1.as_ptr(), body2.as_ptr()) }.cast()
    }
}

impl ConstraintSettings for DistanceConstraintSettings {
    type Kind = DistanceConstraint;
}

impl ConstraintRef<'_, DistanceConstraint> {
    /// The shortest allowed distance, metres; the distance at creation for
    /// [`DistanceRange::Current`].
    pub fn min_distance(&self) -> f32 {
        // SAFETY: the world borrowed here owns the constraint; Jolt's `GetMinDistance` is a const
        // member read, joltc only declares the handle mutable.
        unsafe { JPH_DistanceConstraint_GetMinDistance(self.ptr()) }
    }

    /// The longest allowed distance, metres.
    pub fn max_distance(&self) -> f32 {
        // SAFETY: as in `min_distance`.
        unsafe { JPH_DistanceConstraint_GetMaxDistance(self.ptr()) }
    }

    /// The impulse in N·s the constraint applied along the line between the points in the last
    /// step.
    pub fn total_lambda_position(&self) -> f32 {
        // SAFETY: as in `min_distance`.
        unsafe { JPH_DistanceConstraint_GetTotalLambdaPosition(self.ptr()) }
    }
}

impl ConstraintMut<'_, DistanceConstraint> {
    /// Sets the allowed distance range in metres: `0 <= min <= max <=`
    /// [`limits::MAX_SHAPE_EXTENT`]. Wakes the constraint's bodies.
    pub fn set_distance(&mut self, min: f32, max: f32) -> Result<(), ConstraintError> {
        validate_distance_range(min, max).map_err(ConstraintError::InvalidValue)?;
        // SAFETY: the world is borrowed mutably through this view and owns the constraint; no
        // step runs. Jolt asserts `min <= max`, checked above.
        unsafe { JPH_DistanceConstraint_SetDistance(self.ptr(), min, max) };
        self.wake_bodies();
        Ok(())
    }

    /// Replaces the spring of the range limits, bounded through the bodies' effective mass as
    /// at creation. Wakes the constraint's bodies.
    pub fn set_limits_spring(&mut self, spring: SpringSettings) -> Result<(), ConstraintError> {
        check_spring(spring, self.effective_mass_bound())?;
        let mut spring = spring.to_jph();
        // SAFETY: as in `set_distance`; `spring` is a live local that joltc copies.
        unsafe { JPH_DistanceConstraint_SetLimitsSpringSettings(self.ptr(), &mut spring) };
        self.wake_bodies();
        Ok(())
    }
}

/// What slider limits must satisfy: `min <= 0 <= max` within [`limits::MAX_SHAPE_EXTENT`], as
/// Jolt's `SliderConstraint::SetLimits` asserts the signs; `min == max` needs a soft spring, as
/// its constructor asserts.
fn validate_slider_limits(
    min: f32,
    max: f32,
    limits_spring: SpringSettings,
) -> Result<(), &'static str> {
    let extent = limits::MAX_SHAPE_EXTENT;
    if !(within(min, -extent, 0.0) && within(max, 0.0, extent)) {
        return Err(
            "slider limits must be min in [-MAX_SHAPE_EXTENT, 0] and max in [0, MAX_SHAPE_EXTENT]",
        );
    }
    if min == max && !limits_spring.is_soft() {
        return Err("slider limits with min == max need a soft limits spring");
    }
    Ok(())
}

/// Jolt's "no limits" for a slider.
const NO_SLIDER_LIMITS: (f32, f32) = (-f32::MAX, f32::MAX);

/// A slider (Jolt `SliderConstraintSettings`): body 2 moves along the slider axis of body 1 and
/// keeps its rotation relative to body 1. The position along the axis is 0 where both frame
/// points coincide.
///
/// The default is Jolt's: both frames at the origin with slider axis +X and normal axis +Y,
/// world space, no limits, no friction and a default motor.
#[derive(Clone, Debug, PartialEq)]
pub struct SliderConstraintSettings {
    space: ConstraintSpace,
    auto_detect_point: bool,
    point1: RVec3,
    slider_axis1: Vec3,
    normal_axis1: Vec3,
    point2: RVec3,
    slider_axis2: Vec3,
    normal_axis2: Vec3,
    limits: Option<(f32, f32)>,
    limits_spring: SpringSettings,
    max_friction_force: f32,
    motor: MotorSettings,
}

impl Default for SliderConstraintSettings {
    fn default() -> Self {
        Self::new(
            RVec3::ZERO,
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
    }
}

impl SliderConstraintSettings {
    /// A slider at `point` with the same frame on both bodies, in world space: `slider_axis` and
    /// the perpendicular `normal_axis` are unit vectors. No limits until set.
    pub fn new(point: RVec3, slider_axis: Vec3, normal_axis: Vec3) -> Self {
        Self {
            space: ConstraintSpace::WorldSpace,
            auto_detect_point: false,
            point1: point,
            slider_axis1: slider_axis,
            normal_axis1: normal_axis,
            point2: point,
            slider_axis2: slider_axis,
            normal_axis2: normal_axis,
            limits: None,
            limits_spring: SpringSettings::default(),
            max_friction_force: 0.0,
            motor: MotorSettings::default(),
        }
    }

    /// The space the frames are given in. Default [`ConstraintSpace::WorldSpace`].
    #[must_use]
    pub fn space(mut self, value: ConstraintSpace) -> Self {
        self.space = value;
        self
    }

    /// The frame on body 1: a point and perpendicular unit slider and normal axes.
    #[must_use]
    pub fn frame1(mut self, point: RVec3, slider_axis: Vec3, normal_axis: Vec3) -> Self {
        self.point1 = point;
        self.slider_axis1 = slider_axis;
        self.normal_axis1 = normal_axis;
        self
    }

    /// The frame on body 2: a point and perpendicular unit slider and normal axes.
    #[must_use]
    pub fn frame2(mut self, point: RVec3, slider_axis: Vec3, normal_axis: Vec3) -> Self {
        self.point2 = point;
        self.slider_axis2 = slider_axis;
        self.normal_axis2 = normal_axis;
        self
    }

    /// Lets Jolt place the frame points between the bodies as they are when the constraint is
    /// created. Only in [`ConstraintSpace::WorldSpace`].
    #[must_use]
    pub fn auto_detect_point(mut self) -> Self {
        self.auto_detect_point = true;
        self
    }

    /// The position limits in metres: `min` in `[-MAX_SHAPE_EXTENT, 0]`, `max` in
    /// `[0, MAX_SHAPE_EXTENT]` ([`limits::MAX_SHAPE_EXTENT`]); `min == max` needs a soft
    /// [`limits_spring`](Self::limits_spring). Default no limits.
    #[must_use]
    pub fn limits(mut self, min: f32, max: f32) -> Self {
        self.limits = Some((min, max));
        self
    }

    /// Makes the limits soft. Default rigid.
    #[must_use]
    pub fn limits_spring(mut self, value: SpringSettings) -> Self {
        self.limits_spring = value;
        self
    }

    /// Force in N that friction applies while the motor is off, at least 0. Default 0.
    #[must_use]
    pub fn max_friction_force(mut self, value: f32) -> Self {
        self.max_friction_force = value;
        self
    }

    /// The motor.
    #[must_use]
    pub fn motor(mut self, value: MotorSettings) -> Self {
        self.motor = value;
        self
    }
}

impl sealed::Settings for SliderConstraintSettings {
    fn validate(&self) -> Result<(), &'static str> {
        validate_frame(self.point1, self.slider_axis1, self.normal_axis1)?;
        validate_frame(self.point2, self.slider_axis2, self.normal_axis2)?;
        if self.auto_detect_point && self.space != ConstraintSpace::WorldSpace {
            return Err("an automatic point needs ConstraintSpace::WorldSpace");
        }
        self.limits_spring.validate()?;
        if let Some((min, max)) = self.limits {
            validate_slider_limits(min, max, self.limits_spring)?;
        }
        if !non_negative(self.max_friction_force) {
            return Err("friction must be finite and not negative");
        }
        self.motor.validate()
    }

    fn springs(&self) -> Vec<SpringSettings> {
        vec![self.limits_spring, self.motor.spring]
    }

    fn anchors(&self) -> Option<[sealed::Anchor; 2]> {
        Some(if self.auto_detect_point {
            [sealed::Anchor::BetweenCentersOfMass; 2]
        } else {
            point_anchors(self.space, self.point1, self.point2)
        })
    }

    unsafe fn create(
        &self,
        body1: NonNull<JPH_Body>,
        body2: NonNull<JPH_Body>,
    ) -> *mut JPH_Constraint {
        let (min, max) = self.limits.unwrap_or(NO_SLIDER_LIMITS);
        let settings = JPH_SliderConstraintSettings {
            base: constraint_base(),
            space: self.space.to_jph(),
            autoDetectPoint: self.auto_detect_point,
            point1: self.point1.to_jph(),
            sliderAxis1: self.slider_axis1.to_jph(),
            normalAxis1: self.normal_axis1.to_jph(),
            point2: self.point2.to_jph(),
            sliderAxis2: self.slider_axis2.to_jph(),
            normalAxis2: self.normal_axis2.to_jph(),
            limitsMin: min,
            limitsMax: max,
            limitsSpringSettings: self.limits_spring.to_jph(),
            maxFrictionForce: self.max_friction_force,
            motorSettings: self.motor.to_jph(),
        };
        // SAFETY: as in `FixedConstraintSettings::create`.
        unsafe { JPH_SliderConstraint_Create(&settings, body1.as_ptr(), body2.as_ptr()) }.cast()
    }
}

impl ConstraintSettings for SliderConstraintSettings {
    type Kind = SliderConstraint;
}

impl ConstraintRef<'_, SliderConstraint> {
    /// The position of body 2 along the slider axis, metres from where both frame points
    /// coincide.
    pub fn current_position(&self) -> f32 {
        // SAFETY: the world borrowed here owns the constraint. Jolt's `GetCurrentPosition` is
        // const and reads the constraint's frames and the bodies' transforms, which change only
        // behind `&mut PhysicsWorld`; joltc only declares the handle mutable.
        unsafe { JPH_SliderConstraint_GetCurrentPosition(self.ptr()) }
    }

    /// The motor's state.
    pub fn motor_state(&self) -> MotorState {
        // SAFETY: as in `current_position`; the getter reads a member.
        MotorState::from_jph(unsafe { JPH_SliderConstraint_GetMotorState(self.ptr()) })
    }

    /// The position a position motor drives to, metres.
    pub fn target_position(&self) -> f32 {
        // SAFETY: as in `motor_state`.
        unsafe { JPH_SliderConstraint_GetTargetPosition(self.ptr()) }
    }

    /// The velocity a velocity motor drives to, m/s.
    pub fn target_velocity(&self) -> f32 {
        // SAFETY: as in `motor_state`.
        unsafe { JPH_SliderConstraint_GetTargetVelocity(self.ptr()) }
    }

    /// The position limits in metres, `None` without limits.
    pub fn limits(&self) -> Option<(f32, f32)> {
        // SAFETY: as in `motor_state`.
        unsafe {
            JPH_SliderConstraint_HasLimits(self.ptr()).then(|| {
                (
                    JPH_SliderConstraint_GetLimitsMin(self.ptr()),
                    JPH_SliderConstraint_GetLimitsMax(self.ptr()),
                )
            })
        }
    }

    /// The friction force in N applied while the motor is off.
    pub fn max_friction_force(&self) -> f32 {
        // SAFETY: as in `motor_state`.
        unsafe { JPH_SliderConstraint_GetMaxFrictionForce(self.ptr()) }
    }

    /// The impulse in N·s the motor applied in the last step.
    pub fn total_lambda_motor(&self) -> f32 {
        // SAFETY: as in `motor_state`.
        unsafe { JPH_SliderConstraint_GetTotalLambdaMotor(self.ptr()) }
    }

    /// The impulse in N·s the limits applied in the last step.
    pub fn total_lambda_position_limits(&self) -> f32 {
        // SAFETY: as in `motor_state`.
        unsafe { JPH_SliderConstraint_GetTotalLambdaPositionLimits(self.ptr()) }
    }

    /// The angular impulse in N·m·s that kept the rotation fixed in the last step.
    pub fn total_lambda_rotation(&self) -> Vec3 {
        let mut value = Vec3::ZERO.to_jph();
        // SAFETY: as in `motor_state`; `value` is a live local.
        unsafe { JPH_SliderConstraint_GetTotalLambdaRotation(self.ptr(), &mut value) };
        Vec3::from_jph(value)
    }
}

impl ConstraintMut<'_, SliderConstraint> {
    /// Switches the motor and wakes the constraint's bodies.
    pub fn set_motor_state(&mut self, state: MotorState) {
        // SAFETY: the world is borrowed mutably through this view and owns the constraint; no
        // step runs. Jolt asserts valid motor settings for a running motor: every setting stored
        // was validated.
        unsafe { JPH_SliderConstraint_SetMotorState(self.ptr(), state.to_jph()) };
        self.wake_bodies();
    }

    /// Sets the position a position motor drives to, metres within
    /// [`limits::MAX_SHAPE_EXTENT`]; Jolt clamps it to the limits. Wakes the constraint's
    /// bodies.
    pub fn set_target_position(&mut self, position: f32) -> Result<(), ConstraintError> {
        let extent = limits::MAX_SHAPE_EXTENT;
        if !within(position, -extent, extent) {
            return Err(ConstraintError::InvalidValue(
                "target position must be finite and within limits::MAX_SHAPE_EXTENT",
            ));
        }
        // SAFETY: as in `set_motor_state`.
        unsafe { JPH_SliderConstraint_SetTargetPosition(self.ptr(), position) };
        self.wake_bodies();
        Ok(())
    }

    /// Sets the velocity a velocity motor drives to, m/s, at most
    /// [`limits::MAX_LINEAR_VELOCITY`] in magnitude. Wakes the constraint's bodies.
    pub fn set_target_velocity(&mut self, velocity: f32) -> Result<(), ConstraintError> {
        let speed = limits::MAX_LINEAR_VELOCITY;
        if !within(velocity, -speed, speed) {
            return Err(ConstraintError::InvalidValue(
                "target velocity must be finite and at most limits::MAX_LINEAR_VELOCITY",
            ));
        }
        // SAFETY: as in `set_motor_state`.
        unsafe { JPH_SliderConstraint_SetTargetVelocity(self.ptr(), velocity) };
        self.wake_bodies();
        Ok(())
    }

    /// Replaces the motor settings, checked as at creation and bounded through the bodies'
    /// effective mass. Wakes the constraint's bodies.
    pub fn set_motor_settings(&mut self, motor: MotorSettings) -> Result<(), ConstraintError> {
        check_motor(motor, self.effective_mass_bound())?;
        let mut motor = motor.to_jph();
        // SAFETY: as in `set_motor_state`; `motor` is a live local that joltc copies.
        unsafe { JPH_SliderConstraint_SetMotorSettings(self.ptr(), &mut motor) };
        self.wake_bodies();
        Ok(())
    }

    /// Sets the position limits, with the rules of [`SliderConstraintSettings::limits`]; `None`
    /// removes them. Wakes the constraint's bodies.
    pub fn set_limits(&mut self, limits: Option<(f32, f32)>) -> Result<(), ConstraintError> {
        if let Some((min, max)) = limits {
            validate_slider_limits(min, max, self.limits_spring())
                .map_err(ConstraintError::InvalidValue)?;
        }
        let (min, max) = limits.unwrap_or(NO_SLIDER_LIMITS);
        // SAFETY: as in `set_motor_state`; Jolt asserts `min <= 0 <= max`, which holds.
        unsafe { JPH_SliderConstraint_SetLimits(self.ptr(), min, max) };
        self.wake_bodies();
        Ok(())
    }

    /// Replaces the spring that makes the limits soft, bounded through the bodies' effective
    /// mass. Limits with `min == max` keep needing a soft spring. Wakes the constraint's bodies.
    pub fn set_limits_spring(&mut self, spring: SpringSettings) -> Result<(), ConstraintError> {
        check_spring(spring, self.effective_mass_bound())?;
        // SAFETY: as in `set_motor_state`; the getters read members.
        let (has_limits, min, max) = unsafe {
            (
                JPH_SliderConstraint_HasLimits(self.ptr()),
                JPH_SliderConstraint_GetLimitsMin(self.ptr()),
                JPH_SliderConstraint_GetLimitsMax(self.ptr()),
            )
        };
        if has_limits {
            validate_slider_limits(min, max, spring).map_err(ConstraintError::InvalidValue)?;
        }
        let mut spring = spring.to_jph();
        // SAFETY: as in `set_motor_state`; `spring` is a live local that joltc copies.
        unsafe { JPH_SliderConstraint_SetLimitsSpringSettings(self.ptr(), &mut spring) };
        self.wake_bodies();
        Ok(())
    }

    /// Sets the friction force in N applied while the motor is off, finite and at least 0.
    /// Wakes the constraint's bodies.
    pub fn set_max_friction_force(&mut self, force: f32) -> Result<(), ConstraintError> {
        check_friction(force)?;
        // SAFETY: as in `set_motor_state`.
        unsafe { JPH_SliderConstraint_SetMaxFrictionForce(self.ptr(), force) };
        self.wake_bodies();
        Ok(())
    }

    /// The current limits spring.
    fn limits_spring(&self) -> SpringSettings {
        // SAFETY: an all-zero `JPH_SpringSettings` is valid; the getter fills it from a member.
        let mut spring: JPH_SpringSettings = unsafe { std::mem::zeroed() };
        // SAFETY: as in `set_motor_state`; `spring` is a live local.
        unsafe { JPH_SliderConstraint_GetLimitsSpringSettings(self.ptr(), &mut spring) };
        SpringSettings::from_jph(spring)
    }
}

/// The allowed rope length of a [`PulleyConstraintSettings`]: the length is
/// `|fixed_point1 - body_point1| + ratio · |fixed_point2 - body_point2|`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum PulleyLength {
    /// From 0 up to the length when the constraint is created: a rope that can go slack. The
    /// default (Jolt's `0, -1`).
    #[default]
    UpToCurrent,
    /// From `min` to `max` metres: `0 <= min <= max <= (1 + ratio) ·`
    /// [`limits::MAX_SHAPE_EXTENT`]. `min == max` is a rigid rod.
    Range {
        /// Shortest length, metres.
        min: f32,
        /// Longest length, metres.
        max: f32,
    },
}

/// What a pulley's length range must satisfy for `ratio`.
fn validate_pulley_length(min: f32, max: f32, ratio: f32) -> Result<(), &'static str> {
    let longest = (1.0 + ratio) * limits::MAX_SHAPE_EXTENT;
    if within(min, 0.0, longest) && within(max, 0.0, longest) && min <= max {
        Ok(())
    } else {
        Err("pulley lengths must be 0 <= min <= max <= (1 + ratio) * limits::MAX_SHAPE_EXTENT")
    }
}

/// A pulley (Jolt `PulleyConstraintSettings`): a rope from a point on body 1 over a fixed world
/// point and a second fixed world point to a point on body 2, so that `length1 + ratio ·
/// length2` stays within the allowed [`PulleyLength`]. A ratio of 2 makes body 2 move half as
/// far as body 1, a block and tackle.
///
/// The fixed points are world points whatever [`space`](Self::space) says;
/// [`PhysicsWorld::rebase`](crate::PhysicsWorld::rebase) moves them with the world. Both bodies
/// must be dynamic ([`ConstraintError::NotDynamic`]); the fixed points anchor the rope.
///
/// The default is Jolt's: every point at the origin, world space, ratio 1, a rope up to the
/// length at creation.
#[derive(Clone, Debug, PartialEq)]
pub struct PulleyConstraintSettings {
    space: ConstraintSpace,
    body_point1: RVec3,
    fixed_point1: RVec3,
    body_point2: RVec3,
    fixed_point2: RVec3,
    ratio: f32,
    length: PulleyLength,
}

impl Default for PulleyConstraintSettings {
    fn default() -> Self {
        Self::new(RVec3::ZERO, RVec3::ZERO, RVec3::ZERO, RVec3::ZERO)
    }
}

impl PulleyConstraintSettings {
    /// A rope from `body_point1` over `fixed_point1` and `fixed_point2` to `body_point2`, all in
    /// world space, with ratio 1 and up to the length it has when the constraint is created.
    pub fn new(
        body_point1: RVec3,
        fixed_point1: RVec3,
        body_point2: RVec3,
        fixed_point2: RVec3,
    ) -> Self {
        Self {
            space: ConstraintSpace::WorldSpace,
            body_point1,
            fixed_point1,
            body_point2,
            fixed_point2,
            ratio: 1.0,
            length: PulleyLength::UpToCurrent,
        }
    }

    /// The space the body points are given in; the fixed points are always world points.
    /// Default [`ConstraintSpace::WorldSpace`].
    #[must_use]
    pub fn space(mut self, value: ConstraintSpace) -> Self {
        self.space = value;
        self
    }

    /// The ratio between the two rope segments, positive and within
    /// `1 /`[`limits::MAX_RATIO`]`..=`[`limits::MAX_RATIO`]. Default 1.
    #[must_use]
    pub fn ratio(mut self, value: f32) -> Self {
        self.ratio = value;
        self
    }

    /// The allowed rope length. Default [`PulleyLength::UpToCurrent`].
    #[must_use]
    pub fn length(mut self, value: PulleyLength) -> Self {
        self.length = value;
        self
    }

    fn to_jph(&self) -> JPH_PulleyConstraintSettings {
        let (min, max) = match self.length {
            PulleyLength::UpToCurrent => (0.0, -1.0),
            PulleyLength::Range { min, max } => (min, max),
        };
        JPH_PulleyConstraintSettings {
            base: constraint_base(),
            space: self.space.to_jph(),
            bodyPoint1: self.body_point1.to_jph(),
            fixedPoint1: self.fixed_point1.to_jph(),
            bodyPoint2: self.body_point2.to_jph(),
            fixedPoint2: self.fixed_point2.to_jph(),
            ratio: self.ratio,
            minLength: min,
            maxLength: max,
        }
    }
}

impl sealed::Settings for PulleyConstraintSettings {
    const NEEDS_DYNAMIC_BODIES: bool = true;

    fn validate(&self) -> Result<(), &'static str> {
        for point in [
            self.body_point1,
            self.fixed_point1,
            self.body_point2,
            self.fixed_point2,
        ] {
            validate_point(point)?;
        }
        if !(self.ratio > 0.0 && limits::is_ratio(self.ratio)) {
            return Err(
                "pulley ratio must be positive and within 1 / limits::MAX_RATIO..=limits::MAX_RATIO",
            );
        }
        if let PulleyLength::Range { min, max } = self.length {
            validate_pulley_length(min, max, self.ratio)?;
        }
        Ok(())
    }

    fn springs(&self) -> Vec<SpringSettings> {
        Vec::new()
    }

    fn anchors(&self) -> Option<[sealed::Anchor; 2]> {
        Some(point_anchors(
            self.space,
            self.body_point1,
            self.body_point2,
        ))
    }

    unsafe fn create(
        &self,
        body1: NonNull<JPH_Body>,
        body2: NonNull<JPH_Body>,
    ) -> *mut JPH_Constraint {
        let settings = self.to_jph();
        // SAFETY: as in `FixedConstraintSettings::create`.
        unsafe { JPH_PulleyConstraint_Create(&settings, body1.as_ptr(), body2.as_ptr()) }.cast()
    }
}

impl ConstraintSettings for PulleyConstraintSettings {
    type Kind = PulleyConstraint;
}

/// The settings of a live pulley as Jolt reports them: body points relative to the centres of
/// mass, world fixed points and the lengths resolved at creation.
///
/// # Safety
/// `pulley` is a live pulley constraint that nothing writes during the call.
unsafe fn pulley_settings(pulley: *const JPH_PulleyConstraint) -> JPH_PulleyConstraintSettings {
    // SAFETY: an all-zero settings value is valid (floats, integers, `false` and enums with a
    // zero value).
    let mut settings: JPH_PulleyConstraintSettings = unsafe { std::mem::zeroed() };
    // SAFETY: the caller's contract; Jolt's `GetConstraintSettings` is const and allocates a new
    // settings object, which joltc copies into `settings` and releases.
    unsafe { JPH_PulleyConstraint_GetSettings(pulley, &mut settings) };
    settings
}

impl ConstraintRef<'_, PulleyConstraint> {
    /// The current rope length, `length1 + ratio · length2`, metres, as of the last step or
    /// creation.
    pub fn current_length(&self) -> f32 {
        // SAFETY: the world borrowed here owns the constraint; the getter is a const member read.
        unsafe { JPH_PulleyConstraint_GetCurrentLength(self.ptr()) }
    }

    /// The shortest allowed length, metres.
    pub fn min_length(&self) -> f32 {
        // SAFETY: as in `current_length`.
        unsafe { JPH_PulleyConstraint_GetMinLength(self.ptr()) }
    }

    /// The longest allowed length, metres; the length at creation for
    /// [`PulleyLength::UpToCurrent`].
    pub fn max_length(&self) -> f32 {
        // SAFETY: as in `current_length`.
        unsafe { JPH_PulleyConstraint_GetMaxLength(self.ptr()) }
    }

    /// The ratio between the two rope segments.
    pub fn ratio(&self) -> f32 {
        // SAFETY: as in `current_length`; `pulley_settings` only reads the constraint.
        unsafe { pulley_settings(self.ptr()) }.ratio
    }

    /// The impulse in N·s the rope applied in the last step.
    pub fn total_lambda_position(&self) -> f32 {
        // SAFETY: as in `current_length`.
        unsafe { JPH_PulleyConstraint_GetTotalLambdaPosition(self.ptr()) }
    }
}

impl ConstraintMut<'_, PulleyConstraint> {
    /// Sets the allowed rope length in metres: `0 <= min <= max <= (1 + ratio) ·`
    /// [`limits::MAX_SHAPE_EXTENT`]. Wakes the constraint's bodies.
    pub fn set_length(&mut self, min: f32, max: f32) -> Result<(), ConstraintError> {
        // SAFETY: the world is borrowed mutably through this view and owns the constraint; no
        // step runs, and `pulley_settings` only reads it.
        let ratio = unsafe { pulley_settings(self.ptr()) }.ratio;
        validate_pulley_length(min, max, ratio).map_err(ConstraintError::InvalidValue)?;
        // SAFETY: as above; Jolt asserts `0 <= min <= max`, checked above.
        unsafe { JPH_PulleyConstraint_SetLength(self.ptr(), min, max) };
        self.wake_bodies();
        Ok(())
    }
}

/// A pulley's settings in the new frame of a [`PhysicsWorld::rebase`], computed before the
/// rebase writes anything.
pub(crate) struct PulleyRebase {
    raw: u32,
    settings: JPH_PulleyConstraintSettings,
}

impl PhysicsWorld {
    /// Every pulley's settings with its fixed points moved by `point`, in id order; an error
    /// names a fixed point that would not be finite.
    pub(crate) fn rebased_pulleys(
        &self,
        point: impl Fn(RVec3) -> RVec3,
    ) -> Result<Vec<PulleyRebase>, &'static str> {
        let mut rebased = Vec::new();
        for (&raw, entry) in &self.constraints {
            if entry.kind != ConstraintType::Pulley {
                continue;
            }
            // SAFETY: the world borrowed here owns the pulley; nothing writes it meanwhile.
            let mut settings = unsafe { pulley_settings(entry.constraint.as_ptr().cast()) };
            let fixed1 = point(RVec3::from_jph(settings.fixedPoint1));
            let fixed2 = point(RVec3::from_jph(settings.fixedPoint2));
            if !(fixed1.is_finite() && fixed2.is_finite()) {
                return Err("rebase would give a pulley a non-finite fixed point");
            }
            settings.fixedPoint1 = fixed1.to_jph();
            settings.fixedPoint2 = fixed2.to_jph();
            rebased.push(PulleyRebase { raw, settings });
        }
        Ok(rebased)
    }

    /// Replaces each pulley by one created from what [`rebased_pulleys`](Self::rebased_pulleys)
    /// computed, in id order, after the bodies have moved. The id, enabled state, ratio and
    /// lengths stay; the warm start goes, and Jolt's constraint order changes.
    pub(crate) fn apply_pulley_rebase(&mut self, rebased: Vec<PulleyRebase>) {
        for pulley in rebased {
            let entry = self
                .constraints
                .get(&pulley.raw)
                .unwrap_or_else(|| unreachable!("computed from this world's pulleys"));
            let replacement = with_locked_bodies(self.body_lock_interface, entry.bodies, |a, b| {
                // SAFETY: both bodies are live bodies of this world (the pulley keeps them),
                // locked for writing for the call; the settings are Jolt's own, with finite
                // fixed points. The handle takes over the one reference joltc returns.
                unsafe {
                    Owned::from_raw(
                        JPH_PulleyConstraint_Create(&pulley.settings, a.as_ptr(), b.as_ptr())
                            .cast::<JPH_Constraint>(),
                    )
                }
            })
            .unwrap_or_else(|| unreachable!("a constraint's bodies stay in the world"))
            .unwrap_or_else(|| unreachable!("joltc `new`s the constraint"));
            // SAFETY: the system and both constraints are live, the system is borrowed mutably
            // and no step runs; the old pulley was added by `create_constraint` or an earlier
            // rebase. The system takes its own reference to the new one.
            unsafe {
                JPH_PhysicsSystem_RemoveConstraint(self.system.as_ptr(), entry.constraint.as_ptr());
                JPH_PhysicsSystem_AddConstraint(self.system.as_ptr(), replacement.as_ptr());
            }
            let entry = self
                .constraints
                .get_mut(&pulley.raw)
                .unwrap_or_else(|| unreachable!("looked up above"));
            // Dropping the old handle releases the world's reference to the old pulley.
            entry.constraint = replacement;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::world::ensure_initialized;

    #[test]
    fn pulley_defaults_are_jolts() {
        assert!(ensure_initialized());
        // SAFETY: an all-zero settings value is valid (floats, integers, `false` and enums with a
        // zero value); joltc fills it with Jolt's defaults and allocates nothing.
        let mut jolt: JPH_PulleyConstraintSettings = unsafe { std::mem::zeroed() };
        // SAFETY: Jolt is initialised and `jolt` is a live local.
        unsafe { JPH_PulleyConstraintSettings_Init(&mut jolt) };
        let ours = PulleyConstraintSettings::default().to_jph();
        assert_eq!(ours.base.enabled, jolt.base.enabled);
        assert_eq!(ours.space, jolt.space);
        for (a, b) in [
            (ours.bodyPoint1, jolt.bodyPoint1),
            (ours.fixedPoint1, jolt.fixedPoint1),
            (ours.bodyPoint2, jolt.bodyPoint2),
            (ours.fixedPoint2, jolt.fixedPoint2),
        ] {
            assert_eq!(RVec3::from_jph(a), RVec3::from_jph(b));
        }
        assert_eq!(ours.ratio.to_bits(), jolt.ratio.to_bits());
        assert_eq!(ours.minLength.to_bits(), jolt.minLength.to_bits());
        assert_eq!(ours.maxLength.to_bits(), jolt.maxLength.to_bits());
    }
}
