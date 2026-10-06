//! Tapered capsules and tapered cylinders.

use std::ptr::null;

use oxijolt_sys::*;

use super::{
    initialize, validate_convex_radius, within_extent, Shape, ShapeSettings, BEYOND_EXTENT,
};
use crate::limits;
use crate::math::{is_finite_non_negative, is_finite_positive};
use crate::ShapeError;

/// Largest `|bottom_radius - top_radius|` of a tapered capsule as a fraction of its cylinder's
/// height, `1 - 2^-21`: keeps the sine of Jolt's cone angle within `[-1, 1]` under `f32`
/// rounding ([docs/limits.md#tapered-shapes]).
///
/// [docs/limits.md#tapered-shapes]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#tapered-shapes
const MAX_TAPER: f64 = 1.0 - 1.0 / (1u32 << 21) as f64;

/// Smallest larger radius of a tapered cylinder, `2^-63` m: keeps the denominator of Jolt's
/// centre of mass, `top^2 + top * bottom + bottom^2`, a normal `f32`
/// ([docs/limits.md#tapered-shapes]).
///
/// [docs/limits.md#tapered-shapes]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#tapered-shapes
const MIN_TAPERED_CYLINDER_RADIUS: f32 = 1.084_202_2e-19;

/// The error of a tapered capsule whose end spheres overlap so much that it is a sphere.
const SPHERE_LIKE: &str = "one end sphere of the tapered capsule contains the other; use a sphere";

impl Shape {
    /// A capsule along the local Y axis whose end spheres differ: a cone section
    /// `2 * half_height` metres high between the centres of a sphere of `top_radius` at
    /// `y = half_height` and one of `bottom_radius` at `y = -half_height`, a Jolt
    /// `TaperedCapsuleShape`. Its centre of mass is halfway between the outer ends of the two
    /// spheres (Jolt's approximation).
    ///
    /// All values must be finite and positive, `half_height + max(top_radius, bottom_radius)`
    /// at most [`limits::MAX_SHAPE_EXTENT`], and the radii may differ by at most
    /// `2 * half_height * (1 - 2^-21)`; otherwise one sphere contains the other and the shape
    /// is a sphere, which [`new_sphere`](Self::new_sphere) builds
    /// ([`ShapeError::InvalidValue`]; [docs/limits.md#tapered-shapes]). Only a uniform
    /// [`scaled`](Self::scaled) applies.
    ///
    /// [docs/limits.md#tapered-shapes]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#tapered-shapes
    pub fn new_tapered_capsule(
        half_height: f32,
        top_radius: f32,
        bottom_radius: f32,
    ) -> Result<Self, ShapeError> {
        validate_tapered_capsule(half_height, top_radius, bottom_radius)?;
        initialize()?;
        // SAFETY: Jolt is initialised. The returned settings hold one reference, which the guard
        // takes over.
        let settings = unsafe {
            ShapeSettings::from_raw(
                JPH_TaperedCapsuleShapeSettings_Create(half_height, top_radius, bottom_radius)
                    .cast(),
            )
        }?;
        settings.create()?.within_extent_bounds()
    }

    /// A cylinder along the local Y axis whose ends differ: `2 * half_height` metres high with
    /// a disc of `top_radius` at the top and one of `bottom_radius` at the bottom, a Jolt
    /// `TaperedCylinderShape`. A radius of 0 makes a cone. The centre of mass lies on the axis
    /// at the centroid of the volume.
    ///
    /// The half height must be finite and positive, the radii finite, not negative and
    /// different (equal radii make a cylinder: use
    /// [`new_cylinder_with_convex_radius`](Self::new_cylinder_with_convex_radius)), the larger
    /// radius at least `2^-63` m, and all of them at most [`limits::MAX_SHAPE_EXTENT`]
    /// ([`ShapeError::InvalidValue`]; [docs/limits.md#tapered-shapes]). The convex radius
    /// must be finite and not negative; Jolt clamps it to the smaller radius, and contacts use
    /// at most 0.05 m of it. Only a [`scaled`](Self::scaled) that is uniform in X and Z
    /// applies.
    ///
    /// [docs/limits.md#tapered-shapes]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#tapered-shapes
    pub fn new_tapered_cylinder(
        half_height: f32,
        top_radius: f32,
        bottom_radius: f32,
        convex_radius: f32,
    ) -> Result<Self, ShapeError> {
        validate_tapered_cylinder(half_height, top_radius, bottom_radius)?;
        validate_convex_radius(convex_radius)?;
        initialize()?;
        // SAFETY: Jolt is initialised; a null material selects the default one. The returned
        // settings hold one reference, which the guard takes over.
        let settings = unsafe {
            ShapeSettings::from_raw(
                JPH_TaperedCylinderShapeSettings_Create(
                    half_height,
                    top_radius,
                    bottom_radius,
                    convex_radius,
                    null(),
                )
                .cast(),
            )
        }?;
        settings.create()?.within_extent_bounds()
    }
}

/// The checks of [`Shape::new_tapered_capsule`].
fn validate_tapered_capsule(
    half_height: f32,
    top_radius: f32,
    bottom_radius: f32,
) -> Result<(), ShapeError> {
    if ![half_height, top_radius, bottom_radius]
        .into_iter()
        .all(is_finite_positive)
    {
        return Err(ShapeError::InvalidValue(
            "tapered capsule half height and radii must be finite and positive",
        ));
    }
    let (larger, smaller) = (top_radius.max(bottom_radius), top_radius.min(bottom_radius));
    if half_height + larger > limits::MAX_SHAPE_EXTENT {
        return Err(ShapeError::InvalidValue(BEYOND_EXTENT));
    }
    // Jolt's `TaperedCapsuleShapeSettings::IsSphere`, evaluated in `f32` as Jolt does.
    if larger >= 2.0 * half_height + smaller {
        return Err(ShapeError::InvalidValue(SPHERE_LIKE));
    }
    let taper = f64::from(bottom_radius - top_radius).abs();
    if taper > f64::from(2.0 * half_height) * MAX_TAPER {
        return Err(ShapeError::InvalidValue(SPHERE_LIKE));
    }
    Ok(())
}

/// The checks of [`Shape::new_tapered_cylinder`] other than the convex radius.
fn validate_tapered_cylinder(
    half_height: f32,
    top_radius: f32,
    bottom_radius: f32,
) -> Result<(), ShapeError> {
    if !(is_finite_positive(half_height)
        && is_finite_non_negative(top_radius)
        && is_finite_non_negative(bottom_radius))
    {
        return Err(ShapeError::InvalidValue(
            "tapered cylinder half height must be finite and positive, its radii finite and not negative",
        ));
    }
    if ![half_height, top_radius, bottom_radius]
        .into_iter()
        .all(within_extent)
    {
        return Err(ShapeError::InvalidValue(BEYOND_EXTENT));
    }
    if top_radius == bottom_radius {
        return Err(ShapeError::InvalidValue(
            "tapered cylinder radii must differ; use a cylinder",
        ));
    }
    if top_radius.max(bottom_radius) < MIN_TAPERED_CYLINDER_RADIUS {
        return Err(ShapeError::InvalidValue(
            "tapered cylinder larger radius must be at least 2^-63 m",
        ));
    }
    Ok(())
}

#[cfg(test)]
mod tests;
