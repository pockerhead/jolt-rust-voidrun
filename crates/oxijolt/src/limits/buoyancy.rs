//! The bounds of a buoyancy impulse: every product Jolt's `Body::ApplyBuoyancyImpulse` forms in
//! `f32` stays finite, whatever order the compiler multiplies in, and the buoyant velocity change
//! is bounded like an impulse's. [docs/limits.md#buoyancy] derives the rules.
//!
//! [docs/limits.md#buoyancy]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#buoyancy

use super::{f64_length, F32_PRODUCT_HEADROOM, F32_UNIT_ROUNDOFF, MAX_VELOCITY_CHANGE};
use crate::Vec3;

/// Bound of a length Jolt squares afterwards (a length it compares or a velocity it clamps):
/// its square stays at most 1e36 and three of them sum below `f32::MAX`.
const SQUARED_HEADROOM: f64 = 1.0e18;

/// Everything Jolt's volume overload of the buoyancy impulse computes with
/// (`Body.cpp:196-283`), as Jolt reads it: the inputs of the call and the state of the body.
#[derive(Clone, Copy, Debug)]
pub(crate) struct BuoyancyInputs {
    pub(crate) buoyancy: f32,
    pub(crate) linear_drag: f32,
    pub(crate) angular_drag: f32,
    pub(crate) fluid_velocity: Vec3,
    pub(crate) gravity: Vec3,
    pub(crate) gravity_factor: f32,
    pub(crate) delta_time: f32,
    pub(crate) total_volume: f32,
    pub(crate) submerged_volume: f32,
    /// Centre of buoyancy relative to the centre of mass, in world space.
    pub(crate) center_of_buoyancy: Vec3,
    pub(crate) inverse_mass: f32,
    /// Largest principal inverse inertia: it bounds every entry and the norm of Jolt's world
    /// inverse inertia matrix.
    pub(crate) largest_inverse_inertia: f32,
    pub(crate) linear_velocity: Vec3,
    pub(crate) angular_velocity: Vec3,
    /// Size of the shape's local bounding box.
    pub(crate) bounds_size: Vec3,
}

/// A rule of [`check`], one per product chain of Jolt's buoyancy impulse.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum BuoyancyRule {
    /// The fluid density `buoyancy / (V · 1/m)`.
    Density,
    /// The buoyant impulse `-ρ · Vs · gf · g · dt` and its velocity change.
    BuoyantImpulse,
    /// The velocity of the centre of buoyancy relative to the fluid, `vf - (v + ω × r)`.
    RelativeVelocity,
    /// The area of the bounding box facing the flow.
    Area,
    /// The quadratic drag impulse and its velocity change.
    Drag,
    /// The angular drag impulse and its angular velocity change.
    AngularDrag,
    /// The angular velocity change of the impulses about the centre of mass, `I⁻¹ (r × J)`.
    Lever,
    /// The linear and angular velocity after the impulse, before it is clamped.
    NewVelocity,
    /// The buoyant velocity change is at most `MAX_VELOCITY_CHANGE`, as for an impulse.
    BuoyantVelocityChange,
}

impl BuoyancyRule {
    /// The error text of the rule.
    pub(crate) fn message(self) -> &'static str {
        match self {
            Self::Density => "buoyancy is too large for the body's volume and mass",
            Self::BuoyantImpulse => "buoyant impulse would overflow",
            Self::RelativeVelocity => "relative fluid velocity would overflow",
            Self::Area => "drag area would overflow",
            Self::Drag => "linear drag would overflow",
            Self::AngularDrag => "angular drag would overflow",
            Self::Lever => "buoyancy torque would overflow",
            Self::NewVelocity => "velocity after buoyancy would overflow",
            Self::BuoyantVelocityChange => {
                "buoyancy would change the velocity by more than limits::MAX_VELOCITY_CHANGE"
            }
        }
    }
}

/// Product of `max(1, |f|)` over `factors`: a bound of every sub-product of the factors in any
/// order and association, also when some factor is 0. NaN stays NaN.
fn chain(factors: &[f64]) -> f64 {
    factors
        .iter()
        .map(|f| {
            if f.is_nan() {
                f64::NAN
            } else {
                f.abs().max(1.0)
            }
        })
        .product()
}

/// Whether `bound`, rounded outward, is finite and at most `headroom`; false for NaN.
fn within(bound: f64, headroom: f64) -> bool {
    bound * (1.0 + 64.0 * F32_UNIT_ROUNDOFF) <= headroom
}

fn require(ok: bool, rule: BuoyancyRule) -> Result<(), BuoyancyRule> {
    if ok {
        Ok(())
    } else {
        Err(rule)
    }
}

/// The largest absolute component of `v`; NaN when a component is not finite.
fn largest_component(v: Vec3) -> f64 {
    if v.is_finite() {
        f64::from(v.x.abs().max(v.y.abs()).max(v.z.abs()))
    } else {
        f64::NAN
    }
}

/// Checks that Jolt's buoyancy impulse with `inputs` stays finite in `f32` and that its buoyant
/// velocity change is within [`MAX_VELOCITY_CHANGE`]; the first rule broken otherwise.
///
/// The volumes, the inverse mass and the step are positive and every input finite (the caller
/// refuses anything else first).
pub(crate) fn check(inputs: &BuoyancyInputs) -> Result<(), BuoyancyRule> {
    use BuoyancyRule::*;
    let h = F32_PRODUCT_HEADROOM;
    let h2 = SQUARED_HEADROOM;
    let f = f64::from;
    let i = inputs;
    let inverse_mass = f(i.inverse_mass);
    let lambda = f(i.largest_inverse_inertia);
    let r = f64_length(i.center_of_buoyancy);
    let v = f64_length(i.linear_velocity);
    let w = f64_length(i.angular_velocity);
    // The buoyant velocity change `‖Jb‖ · invM`.
    let change = f(i.buoyancy) * f(i.submerged_volume) / f(i.total_volume)
        * f(i.gravity_factor).abs()
        * f64_length(i.gravity)
        * f(i.delta_time);
    let density = [f(i.buoyancy), 1.0 / f(i.total_volume), 1.0 / inverse_mass];
    require(within(chain(&density), h), Density)?;

    let mut buoyant = density.to_vec();
    buoyant.extend([
        f(i.submerged_volume),
        f(i.gravity_factor),
        largest_component(i.gravity),
        f(i.delta_time),
    ]);
    let buoyant_bound = chain(&buoyant);
    require(within(buoyant_bound, h), BuoyantImpulse)?;
    require(
        within(buoyant_bound * inverse_mass.max(1.0), h),
        BuoyantImpulse,
    )?;

    require(within(chain(&[w, r]), h), RelativeVelocity)?;
    let relative = f64_length(i.fluid_velocity) + v + w * r;
    require(within(relative, h2), RelativeVelocity)?;

    let s = [i.bounds_size.x, i.bounds_size.y, i.bounds_size.z].map(f);
    let facing = f64_length_of([s[1] * s[2], s[2] * s[0], s[0] * s[1]]);
    require(within(chain(&[relative, facing]), h), Area)?;

    let mut drag = density.to_vec();
    drag.extend([
        0.5,
        f(i.linear_drag),
        facing,
        f(i.delta_time),
        relative,
        relative,
    ]);
    require(within(chain(&drag) * inverse_mass.max(1.0), h), Drag)?;
    // The length Jolt squares, `‖Jd · invM‖ <= 0.5 · b / V · Cd · ‖q‖ · dt · ‖vrel‖²`.
    let drag_change = 0.5 * f(i.buoyancy) / f(i.total_volume)
        * f(i.linear_drag)
        * facing
        * f(i.delta_time)
        * relative
        * relative;
    require(within(drag_change, h2), Drag)?;

    let width = (s[0] + s[1] + s[2]) / 3.0;
    let angular_factor = [
        f(i.angular_drag),
        f(i.submerged_volume),
        1.0 / f(i.total_volume),
        f(i.delta_time),
        width,
        width,
        1.0 / inverse_mass,
    ];
    require(
        within(chain(&angular_factor) * w.max(1.0) * lambda.max(1.0), h),
        AngularDrag,
    )?;
    // The length Jolt squares, `‖I⁻¹ K ω‖ <= λ · |K| · ‖ω‖`.
    let angular_drag_change = lambda * angular_factor.iter().product::<f64>() * w;
    require(within(angular_drag_change, h2), AngularDrag)?;

    // Jolt clamps the drag impulse to the body's own momentum, `‖v‖ / invM`.
    let impulse_bound = buoyant_bound + v / inverse_mass;
    require(within(chain(&[lambda, r, impulse_bound]), h), Lever)?;
    let lever = lambda * r * (change + v) / inverse_mass;
    require(within(lever, h2), Lever)?;
    require(within(2.0 * w + lever, h2), NewVelocity)?;
    require(within(2.0 * v + change, h2), NewVelocity)?;
    require(
        within(change, f(MAX_VELOCITY_CHANGE)),
        BuoyantVelocityChange,
    )
}

/// The length of an `f64` vector.
fn f64_length_of(v: [f64; 3]) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

#[cfg(test)]
mod tests;
