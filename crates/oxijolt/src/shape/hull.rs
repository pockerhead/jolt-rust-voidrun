//! Convex hulls.

use oxijolt_sys::*;

use super::{initialize, validate_convex_radius, Shape, ShapeSettings};
use crate::{limits, HullError, PhysicsMaterial, ShapeError, Vec3};

/// Jolt's `ConvexHullBuilder::cMinTriangleAreaSq`: the squared length of the cross product of two
/// triangle edges below which Jolt finds no initial triangle for a hull.
const MIN_TRIANGLE_AREA_SQ: f64 = 1.0e-12;

impl Shape {
    /// The convex hull of `points` (shape space, metres) with a convex radius in metres.
    ///
    /// Needs at least 4 points ([`HullError::TooFewPoints`]), each finite with every component
    /// at most [`limits::MAX_SHAPE_EXTENT`] in absolute value, and a convex radius that is
    /// finite and not negative ([`ShapeError::InvalidDimensions`]). Points on a line or in one
    /// spot ([`HullError::Degenerate`]: no triangle of them is larger than Jolt's minimum
    /// initial triangle) and points in one plane ([`HullError::Coplanar`]) are refused: a flat
    /// hull has no volume, so Jolt would give a dynamic body made of it zero mass and a
    /// meaningless inertia. Use a mesh or a thin box for a flat surface. Whatever else Jolt's
    /// hull builder refuses comes back as [`ShapeError::Rejected`].
    ///
    /// Jolt keeps at most 256 vertices of the hull and drops the points inside it. It shrinks the
    /// hull by the convex radius and inflates it again, reducing the radius where the hull is too
    /// small for it; contacts and shape casts use at most 0.05 m of it, ray casts see the hull
    /// without it. The hull's centre of mass becomes its shape-space reference for bodies, and
    /// its local bounds, relative to that centre, must lie within
    /// [`limits::MAX_SHAPE_EXTENT`]. See [docs/limits.md#convex-hulls].
    ///
    /// [docs/limits.md#convex-hulls]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#convex-hulls
    pub fn new_convex_hull(points: &[Vec3], convex_radius: f32) -> Result<Self, ShapeError> {
        Self::convex_hull(points, convex_radius, None)
    }

    /// [`new_convex_hull`](Self::new_convex_hull) made of `material`; the same rules apply.
    pub fn new_convex_hull_with_material(
        points: &[Vec3],
        convex_radius: f32,
        material: &PhysicsMaterial,
    ) -> Result<Self, ShapeError> {
        Self::convex_hull(points, convex_radius, Some(material))
    }

    fn convex_hull(
        points: &[Vec3],
        convex_radius: f32,
        material: Option<&PhysicsMaterial>,
    ) -> Result<Self, ShapeError> {
        validate_hull_points(points)?;
        validate_convex_radius(convex_radius)?;
        if !spans_initial_triangle(points) {
            return Err(ShapeError::ConvexHull(HullError::Degenerate));
        }
        initialize()?;
        let jolt_points: Vec<JPH_Vec3> = points.iter().map(|point| point.to_jph()).collect();
        // SAFETY: Jolt is initialised; `jolt_points` is live and holds the count passed, which
        // fits `u32` (checked above) and which Jolt copies. The returned settings hold one
        // reference, which the guard takes over.
        let settings = unsafe {
            ShapeSettings::from_raw(
                JPH_ConvexHullShapeSettings_Create(
                    jolt_points.as_ptr(),
                    jolt_points.len() as u32,
                    convex_radius,
                )
                .cast(),
            )
        }?;
        if let Some(material) = material {
            // SAFETY: the settings are live, owned by the guard, have not created a shape yet
            // and were created as convex hull settings, which derive from convex settings.
            unsafe { Self::attach_material(&settings, material) };
        }
        let shape = settings.create()?;
        if shape.is_flat_hull() {
            return Err(ShapeError::ConvexHull(HullError::Coplanar));
        }
        shape.within_extent_bounds()
    }

    /// Whether this is a convex hull Jolt built from points in one plane: Jolt's coplanar case
    /// yields exactly two back-to-back faces (`ConvexHullBuilder::Initialize`), while a hull
    /// with volume has at least four.
    fn is_flat_hull(&self) -> bool {
        if self.sub_type() != JPH_ShapeSubType_ConvexHull {
            return false;
        }
        // SAFETY: the shape is live and a convex hull (checked above); the getter only reads it.
        unsafe { JPH_ConvexHullShape_GetNumFaces(self.as_ptr().cast()) == 2 }
    }
}

/// The count and magnitude checks of [`Shape::new_convex_hull`].
fn validate_hull_points(points: &[Vec3]) -> Result<(), ShapeError> {
    if points.len() < 4 {
        return Err(ShapeError::ConvexHull(HullError::TooFewPoints));
    }
    // Jolt's hull builder indexes points with `int`.
    if points.len() > i32::MAX as usize {
        return Err(ShapeError::InvalidSettings(
            "a convex hull has at most i32::MAX points",
        ));
    }
    if !points.iter().all(|&point| limits::is_local_offset(point)) {
        return Err(ShapeError::InvalidDimensions(
            "convex hull points must be finite and within limits::MAX_SHAPE_EXTENT",
        ));
    }
    Ok(())
}

/// Whether Jolt's hull builder finds an initial triangle in `points`, replayed in `f64`:
/// the first point farthest from the origin, the first point farthest from it, and the third
/// point that makes the largest triangle with both (`ConvexHullBuilder::Initialize`). Near
/// the threshold Jolt's `f32` result can differ; Jolt then refuses with its own message.
fn spans_initial_triangle(points: &[Vec3]) -> bool {
    let points: Vec<[f64; 3]> = points
        .iter()
        .map(|point| [point.x, point.y, point.z].map(f64::from))
        .collect();
    let first = farthest(&points, None, length_sq);
    let second = farthest(&points, Some(first), |p| length_sq(sub(p, points[first])));
    let best = points
        .iter()
        .enumerate()
        .filter(|&(index, _)| index != first && index != second)
        .map(|(_, &p)| length_sq(cross(sub(points[first], p), sub(points[second], p))))
        .fold(-1.0, f64::max);
    best >= MIN_TRIANGLE_AREA_SQ
}

/// The index of the first point with the largest `measure`, skipping `skip`.
fn farthest(points: &[[f64; 3]], skip: Option<usize>, measure: impl Fn([f64; 3]) -> f64) -> usize {
    let mut best = (0, -1.0);
    for (index, &point) in points.iter().enumerate() {
        let value = measure(point);
        if Some(index) != skip && value > best.1 {
            best = (index, value);
        }
    }
    best.0
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn length_sq(a: [f64; 3]) -> f64 {
    a[0] * a[0] + a[1] * a[1] + a[2] * a[2]
}

#[cfg(test)]
mod tests;
