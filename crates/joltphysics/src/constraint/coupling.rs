//! World constraints that couple the motion of two bodies: gear and rack and pinion.

use std::f32::consts::PI;
use std::ptr::NonNull;

use joltphysics_sys::*;

use super::world::sealed::{self, ReferencedConstraint};
use super::world::ConstraintSettings;
use super::{constraint_base, ConstraintSpace, SpringSettings};
use crate::limits;
use crate::math::is_unit;
use crate::{
    BodyId, ConstraintId, ConstraintRef, GearConstraint, HingeConstraint, RackAndPinionConstraint,
    SliderConstraint, Vec3,
};

/// What a rack-and-pinion ratio must satisfy.
const RATIO_RULE: &str =
    "ratio must be finite with a magnitude between 1 / limits::MAX_RATIO and limits::MAX_RATIO";

/// What a gear ratio must satisfy.
const GEAR_RATIO_RULE: &str = "gear ratio must be between 1 and limits::MAX_RATIO";

/// What a referenced hinge or slider must satisfy.
const REFERENCE_RULE: &str = "a referenced hinge or slider must join the coupled body as its body 2, with the same axis direction";

/// Smallest cosine between a coupling's axis and its reference's axis in the shared body.
const SAME_DIRECTION: f32 = 1.0 - 1.0e-3;

/// Column 0 of `constraint`'s constraint-to-body matrix for body `which` (1 or 2): the axis
/// Jolt measures the constraint about, in that body's centre-of-mass frame.
///
/// # Safety
/// `constraint` is a live two-body constraint that nothing writes during the call.
unsafe fn constraint_axis(constraint: NonNull<JPH_Constraint>, which: u8) -> Vec3 {
    // SAFETY: an all-zero `JPH_Mat4` is valid (floats).
    let mut matrix: JPH_Mat4 = unsafe { std::mem::zeroed() };
    let two_body: *const JPH_TwoBodyConstraint = constraint.as_ptr().cast();
    // SAFETY: every world constraint is a `TwoBodyConstraint` (single inheritance from
    // `Constraint`, so the same address), live by the caller's contract. joltc copies the matrix
    // with `memcpy` into `matrix`, a live local.
    unsafe {
        if which == 1 {
            JPH_TwoBodyConstraint_GetConstraintToBody1Matrix(two_body, &mut matrix);
        } else {
            JPH_TwoBodyConstraint_GetConstraintToBody2Matrix(two_body, &mut matrix);
        }
    }
    let column = matrix.column[0];
    Vec3::new(column.x, column.y, column.z)
}

/// Checks that `reference` joins `shared` as its body 2 and turns or slides about the same axis
/// direction, in `shared`'s frame, as `axis`.
///
/// Jolt corrects a coupling's drift from the referenced hinge's angle or slider's position,
/// which it measures for body 2 relative to body 1 about body 1's axis
/// (`HingeConstraint::GetCurrentAngle`, `SliderConstraint::GetCurrentPosition`); a reference
/// attached the other way round or about the opposite axis would correct in the wrong
/// direction.
///
/// # Safety
/// `reference.constraint` is a live two-body constraint that nothing writes during the call.
unsafe fn check_reference(
    reference: &ReferencedConstraint,
    shared: BodyId,
    axis: Vec3,
) -> Result<(), &'static str> {
    if reference.bodies[1] != shared {
        return Err(REFERENCE_RULE);
    }
    // SAFETY: the caller's contract.
    let reference_axis = unsafe { constraint_axis(reference.constraint, 2) };
    if reference_axis.dot(axis) >= SAME_DIRECTION {
        Ok(())
    } else {
        Err(REFERENCE_RULE)
    }
}

/// A gear (Jolt `GearConstraintSettings`): body 1 turning about its hinge axis turns body 2
/// about its own: `ω1 + ratio · ω2 = 0` for the rotation rates about the two axes, so a ratio
/// of 2 turns gear 2 half as fast, the other way. The bodies need their own hinges, which hold
/// them in place.
///
/// The ratio is between 1 and [`limits::MAX_RATIO`]: body 2 is the gear that turns slower. Jolt
/// 5.6 applies the gear's impulse to body 2 without the ratio (`GearConstraintPart::
/// ApplyVelocityStep`) while its effective mass includes the ratio squared, so each solver
/// iteration leaves `1 - (A + r·B) / (A + r²·B)` of the error, with `A` and `B` the inverse
/// inertias about the two axes. That stays below 1 for every ratio of at least 1; a negative
/// ratio, or a ratio below 1 with a light body 2, grows the error until the bodies' velocities
/// are not finite. Swap the bodies for a gear that speeds up, and turn one axis around for gears
/// that turn the same way. Because of the same missing factor, the torque the gear passes to
/// body 2 is not `ratio` times the torque on body 1; the rotation rates follow the ratio.
///
/// Without [`hinges`](Self::hinges) Jolt couples the velocities only, and the gears slowly drift
/// apart. With them it also corrects `angle1 + ratio · angle2` (modulo 2π) from the hinges'
/// angles in `[-π, π]`. That correction is consistent across a hinge angle's wrap at ±π only
/// for an integer ratio; with another ratio it corrects towards a wrong angle once gear 2 has
/// wrapped (a limitation of Jolt's `GearConstraint`).
///
/// The default is Jolt's: both axes +X, world space, ratio 1.
#[derive(Clone, Debug, PartialEq)]
pub struct GearConstraintSettings {
    space: ConstraintSpace,
    hinge_axis1: Vec3,
    hinge_axis2: Vec3,
    ratio: f32,
    hinges: Option<[ConstraintId<HingeConstraint>; 2]>,
}

impl Default for GearConstraintSettings {
    fn default() -> Self {
        let x = Vec3::new(1.0, 0.0, 0.0);
        Self::new(x, x, 1.0)
    }
}

impl GearConstraintSettings {
    /// A gear about the unit `hinge_axis1` of body 1 and `hinge_axis2` of body 2, in world
    /// space, with `ratio` within `1..=`[`limits::MAX_RATIO`].
    pub fn new(hinge_axis1: Vec3, hinge_axis2: Vec3, ratio: f32) -> Self {
        Self {
            space: ConstraintSpace::WorldSpace,
            hinge_axis1,
            hinge_axis2,
            ratio,
            hinges: None,
        }
    }

    /// The space the axes are given in. Default [`ConstraintSpace::WorldSpace`].
    #[must_use]
    pub fn space(mut self, value: ConstraintSpace) -> Self {
        self.space = value;
        self
    }

    /// The ratio of two gears with `teeth1` and `teeth2` teeth, `teeth2 / teeth1` (Jolt
    /// `SetRatio`); body 2 is the gear with more teeth. Zero teeth give an invalid ratio.
    #[must_use]
    pub fn teeth(mut self, teeth1: u32, teeth2: u32) -> Self {
        self.ratio = teeth2 as f32 / teeth1 as f32;
        self
    }

    /// The hinges that hold body 1 and body 2, for Jolt's drift correction: each must be a
    /// hinge of the same world whose body 2 is the gear's body 1 (respectively body 2) and whose
    /// axis has the gear's axis direction in that body. They cannot be removed while the gear
    /// exists.
    #[must_use]
    pub fn hinges(
        mut self,
        hinge1: ConstraintId<HingeConstraint>,
        hinge2: ConstraintId<HingeConstraint>,
    ) -> Self {
        self.hinges = Some([hinge1, hinge2]);
        self
    }
}

impl sealed::Settings for GearConstraintSettings {
    fn validate(&self) -> Result<(), &'static str> {
        if !(is_unit(self.hinge_axis1) && is_unit(self.hinge_axis2)) {
            return Err("constraint frame axes must be unit vectors");
        }
        if !(1.0..=limits::MAX_RATIO).contains(&self.ratio) {
            return Err(GEAR_RATIO_RULE);
        }
        Ok(())
    }

    fn springs(&self) -> Vec<SpringSettings> {
        Vec::new()
    }

    fn references(&self) -> [Option<crate::AnyConstraintId>; 2] {
        match self.hinges {
            Some([hinge1, hinge2]) => [Some(hinge1.into()), Some(hinge2.into())],
            None => [None, None],
        }
    }

    unsafe fn create(
        &self,
        body1: NonNull<JPH_Body>,
        body2: NonNull<JPH_Body>,
    ) -> *mut JPH_Constraint {
        let settings = JPH_GearConstraintSettings {
            base: constraint_base(),
            space: self.space.to_jph(),
            hingeAxis1: self.hinge_axis1.to_jph(),
            hingeAxis2: self.hinge_axis2.to_jph(),
            ratio: self.ratio,
        };
        // SAFETY: the caller locks both live bodies (trait contract); `settings` is a live,
        // validated local that joltc converts.
        unsafe { JPH_GearConstraint_Create(&settings, body1.as_ptr(), body2.as_ptr()) }.cast()
    }

    unsafe fn attach_references(
        &self,
        constraint: NonNull<JPH_Constraint>,
        bodies: [BodyId; 2],
        references: [Option<ReferencedConstraint>; 2],
    ) -> Result<(), &'static str> {
        let [Some(hinge1), Some(hinge2)] = references else {
            return Ok(());
        };
        // SAFETY: the gear and both hinges are live and not written meanwhile (trait contract).
        unsafe {
            check_reference(&hinge1, bodies[0], constraint_axis(constraint, 1))?;
            check_reference(&hinge2, bodies[1], constraint_axis(constraint, 2))?;
            // The gear takes a reference to each hinge; both are hinges, as Jolt's position
            // correction requires.
            JPH_GearConstraint_SetConstraints(
                constraint.as_ptr().cast(),
                hinge1.constraint.as_ptr(),
                hinge2.constraint.as_ptr(),
            );
        }
        Ok(())
    }
}

impl ConstraintSettings for GearConstraintSettings {
    type Kind = GearConstraint;
}

impl ConstraintRef<'_, GearConstraint> {
    /// The angular impulse in N·m·s the gear applied in the last step.
    pub fn total_lambda(&self) -> f32 {
        // SAFETY: the world borrowed here owns the constraint; the getter reads a member.
        unsafe { JPH_GearConstraint_GetTotalLambda(self.ptr()) }
    }
}

/// A rack and pinion (Jolt `RackAndPinionConstraintSettings`): body 1, the pinion, turning about
/// its hinge axis moves body 2, the rack, along its slider axis with `rotation = ratio ·
/// translation` (radians per metre). The bodies need their own hinge and slider.
///
/// Without [`constraints`](Self::constraints) Jolt couples the velocities only and the two
/// drift apart; with them it also corrects the rotation from the hinge's angle and the slider's
/// position. The correction wraps only the pinion angle, so it stays consistent over any number
/// of turns.
///
/// The default is Jolt's: both axes +X, world space, ratio 1.
#[derive(Clone, Debug, PartialEq)]
pub struct RackAndPinionConstraintSettings {
    space: ConstraintSpace,
    hinge_axis: Vec3,
    slider_axis: Vec3,
    ratio: f32,
    constraints: Option<(
        ConstraintId<HingeConstraint>,
        ConstraintId<SliderConstraint>,
    )>,
}

impl Default for RackAndPinionConstraintSettings {
    fn default() -> Self {
        let x = Vec3::new(1.0, 0.0, 0.0);
        Self::new(x, x, 1.0)
    }
}

impl RackAndPinionConstraintSettings {
    /// A rack and pinion about the unit `hinge_axis` of body 1 and along the unit `slider_axis`
    /// of body 2, in world space, with `ratio` in radians per metre of a magnitude within
    /// `1 /`[`limits::MAX_RATIO`]`..=`[`limits::MAX_RATIO`].
    pub fn new(hinge_axis: Vec3, slider_axis: Vec3, ratio: f32) -> Self {
        Self {
            space: ConstraintSpace::WorldSpace,
            hinge_axis,
            slider_axis,
            ratio,
            constraints: None,
        }
    }

    /// The space the axes are given in. Default [`ConstraintSpace::WorldSpace`].
    #[must_use]
    pub fn space(mut self, value: ConstraintSpace) -> Self {
        self.space = value;
        self
    }

    /// The ratio of a rack of `rack_length` metres with `rack_teeth` teeth and a pinion with
    /// `pinion_teeth` teeth: `2π · rack_teeth / (rack_length · pinion_teeth)` (Jolt `SetRatio`).
    /// Zero teeth or a length that is not positive and within [`limits::MAX_SHAPE_EXTENT`] give
    /// an invalid ratio.
    #[must_use]
    pub fn teeth(mut self, rack_teeth: u32, rack_length: f32, pinion_teeth: u32) -> Self {
        let length_valid = rack_length > 0.0 && rack_length <= limits::MAX_SHAPE_EXTENT;
        self.ratio = if length_valid && rack_teeth > 0 && pinion_teeth > 0 {
            2.0 * PI * rack_teeth as f32 / (rack_length * pinion_teeth as f32)
        } else {
            f32::NAN
        };
        self
    }

    /// The hinge that holds the pinion (body 1) and the slider that holds the rack (body 2), for
    /// Jolt's drift correction: the hinge's body 2 must be the pinion and the slider's body 2
    /// the rack, each with the axis direction given here. They cannot be removed while the rack
    /// and pinion exists.
    #[must_use]
    pub fn constraints(
        mut self,
        hinge: ConstraintId<HingeConstraint>,
        slider: ConstraintId<SliderConstraint>,
    ) -> Self {
        self.constraints = Some((hinge, slider));
        self
    }
}

impl sealed::Settings for RackAndPinionConstraintSettings {
    fn validate(&self) -> Result<(), &'static str> {
        if !(is_unit(self.hinge_axis) && is_unit(self.slider_axis)) {
            return Err("constraint frame axes must be unit vectors");
        }
        if !limits::is_ratio(self.ratio) {
            return Err(RATIO_RULE);
        }
        Ok(())
    }

    fn springs(&self) -> Vec<SpringSettings> {
        Vec::new()
    }

    fn references(&self) -> [Option<crate::AnyConstraintId>; 2] {
        match self.constraints {
            Some((hinge, slider)) => [Some(hinge.into()), Some(slider.into())],
            None => [None, None],
        }
    }

    unsafe fn create(
        &self,
        body1: NonNull<JPH_Body>,
        body2: NonNull<JPH_Body>,
    ) -> *mut JPH_Constraint {
        let settings = JPH_RackAndPinionConstraintSettings {
            base: constraint_base(),
            space: self.space.to_jph(),
            hingeAxis: self.hinge_axis.to_jph(),
            sliderAxis: self.slider_axis.to_jph(),
            ratio: self.ratio,
        };
        // SAFETY: as in `GearConstraintSettings::create`.
        unsafe { JPH_RackAndPinionConstraint_Create(&settings, body1.as_ptr(), body2.as_ptr()) }
            .cast()
    }

    unsafe fn attach_references(
        &self,
        constraint: NonNull<JPH_Constraint>,
        bodies: [BodyId; 2],
        references: [Option<ReferencedConstraint>; 2],
    ) -> Result<(), &'static str> {
        let [Some(hinge), Some(slider)] = references else {
            return Ok(());
        };
        // SAFETY: as in `GearConstraintSettings::attach_references`; the references are a
        // hinge and a slider, as Jolt's position correction requires.
        unsafe {
            check_reference(&hinge, bodies[0], constraint_axis(constraint, 1))?;
            check_reference(&slider, bodies[1], constraint_axis(constraint, 2))?;
            JPH_RackAndPinionConstraint_SetConstraints(
                constraint.as_ptr().cast(),
                hinge.constraint.as_ptr(),
                slider.constraint.as_ptr(),
            );
        }
        Ok(())
    }
}

impl ConstraintSettings for RackAndPinionConstraintSettings {
    type Kind = RackAndPinionConstraint;
}

impl ConstraintRef<'_, RackAndPinionConstraint> {
    /// The impulse the rack and pinion applied in the last step.
    pub fn total_lambda(&self) -> f32 {
        // SAFETY: the world borrowed here owns the constraint; the getter reads a member.
        unsafe { JPH_RackAndPinionConstraint_GetTotalLambda(self.ptr()) }
    }
}
