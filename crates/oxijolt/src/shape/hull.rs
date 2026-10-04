//! Convex hulls.

use oxijolt_sys::*;

use super::geometry::{cross, dot, length_sq, sub, v3};
use super::{initialize, validate_convex_radius, Shape, ShapeSettings};
use crate::{limits, HullError, PhysicsMaterial, ShapeError, Vec3};

/// Jolt's `ConvexHullBuilder::cMinTriangleAreaSq`: the squared length of the cross product of two
/// triangle edges below which Jolt finds no initial triangle for a hull.
const MIN_TRIANGLE_AREA_SQ: f64 = 1.0e-12;
/// Jolt's `ConvexHullShapeSettings::mHullTolerance` default, metres: how far points may lie
/// outside the hull Jolt builds.
const HULL_TOLERANCE: f64 = 1.0e-3;
/// Smallest `width * tolerance / (length * coplanar distance)` of a cloud Jolt's hull builder is
/// given; see [docs/limits.md#convex-hulls]. Measured: 0.052 is the largest value at which the
/// builder still asserted, so the bound keeps a factor of about 5.
///
/// [docs/limits.md#convex-hulls]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#convex-hulls
const MIN_NEEDLE_LEVER: f64 = 0.25;
/// Smallest distance of the farthest point from the initial triangle's plane, in Jolt's coplanar
/// distances, of a cloud Jolt's hull builder is given. Jolt itself treats up to 6 as flat
/// (`cCoplanarSlopFactor`); measured: clouds with noisy faces asserted up to 1948, so the bound
/// keeps a factor of about 3; see [docs/limits.md#convex-hulls].
///
/// [docs/limits.md#convex-hulls]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#convex-hulls
const MIN_SLAB_THICKNESS: f64 = 6000.0;

impl Shape {
    /// The convex hull of `points` (shape space, metres) with a convex radius in metres.
    ///
    /// Needs at least 4 points ([`HullError::TooFewPoints`]), each finite with every component
    /// at most [`limits::MAX_SHAPE_EXTENT`] in absolute value, and a convex radius that is
    /// finite and not negative ([`ShapeError::InvalidDimensions`]). Points on or close to a line
    /// or in one spot are refused as [`HullError::Degenerate`], points on or close to a plane as
    /// [`HullError::Coplanar`]: a flat hull has no volume, so Jolt would give a dynamic body made
    /// of it zero mass and a meaningless inertia, and Jolt's single-precision hull builder cannot
    /// build very thin needles and slabs reliably. "Close" grows with the cloud's length and its
    /// distance from the shape origin; [docs/limits.md#convex-hulls] gives the rules. Use a
    /// mesh, a capsule or a thin box instead. Whatever else Jolt's hull builder refuses comes
    /// back as [`ShapeError::Rejected`].
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
        InitialSimplex::of(points).classify()?;
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
    /// with volume has at least four. The slab rule of [`InitialSimplex::classify`] refuses such
    /// clouds first with a wide margin; this reads Jolt's own verdict on what it built.
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

/// Jolt's initial simplex of a point cloud (`ConvexHullBuilder::Initialize`), replayed in `f64`:
/// the first point farthest from the origin, the first point farthest from it, the point that
/// makes the largest triangle with both, and the point farthest from that triangle's plane.
struct InitialSimplex {
    /// Squared length of the cross product of the triangle's edges.
    area_sq: f64,
    /// Distance between the first two points, metres.
    length: f64,
    /// Distance of the third point from the line through the first two, metres: no point lies
    /// farther from that line.
    width: f64,
    /// Distance of the farthest point from the triangle's plane, metres.
    thickness: f64,
    /// Jolt's `DetermineCoplanarDistance`: `3 * FLT_EPSILON` times the sum of the largest
    /// absolute coordinate per axis, metres.
    coplanar_distance: f64,
}

impl InitialSimplex {
    fn of(points: &[Vec3]) -> Self {
        let points: Vec<[f64; 3]> = points.iter().map(|&point| v3(point)).collect();
        let first = farthest(&points, None, length_sq);
        let second = farthest(&points, Some(first), |p| length_sq(sub(p, points[first])));
        let (a, b) = (points[first], points[second]);
        let mut third = (first, -1.0);
        for (index, &p) in points.iter().enumerate() {
            let area_sq = length_sq(cross(sub(a, p), sub(b, p)));
            if index != first && index != second && area_sq > third.1 {
                third = (index, area_sq);
            }
        }
        let (area_sq, c) = (third.1, points[third.0]);
        let length = length_sq(sub(b, a)).sqrt();
        let normal = cross(sub(b, a), sub(c, a));
        let normal_length = length_sq(normal).sqrt();
        let centroid = [0, 1, 2].map(|i| (a[i] + b[i] + c[i]) / 3.0);
        let thickness = points
            .iter()
            .map(|&p| (dot(sub(p, centroid), normal) / normal_length).abs())
            .fold(0.0, f64::max);
        let largest = [0, 1, 2].map(|i| points.iter().map(|p| p[i].abs()).fold(0.0, f64::max));
        Self {
            area_sq,
            length,
            width: area_sq.sqrt() / length,
            thickness,
            coplanar_distance: 3.0
                * f64::from(f32::EPSILON)
                * (largest[0] + largest[1] + largest[2]),
        }
    }

    /// What [`Shape::new_convex_hull`] refuses before Jolt sees the points. Clouds smaller than
    /// Jolt's minimum initial triangle are degenerate; needles and slabs too thin for Jolt's
    /// single-precision builder are refused as degenerate and coplanar. Near these thresholds
    /// Jolt's `f32` result can differ from this `f64` replay.
    fn classify(&self) -> Result<(), ShapeError> {
        if self.area_sq < MIN_TRIANGLE_AREA_SQ {
            return Err(ShapeError::ConvexHull(HullError::Degenerate));
        }
        // The rounding of a position, about the coplanar distance, tilts a face built over the
        // width by `coplanar / width`, which moves it by `length * coplanar / width` at the far
        // end; the builder needs that well inside its tolerance.
        let tolerance = HULL_TOLERANCE.max(self.coplanar_distance);
        if self.width * tolerance < MIN_NEEDLE_LEVER * self.length * self.coplanar_distance {
            return Err(ShapeError::ConvexHull(HullError::Degenerate));
        }
        if self.thickness < MIN_SLAB_THICKNESS * self.coplanar_distance {
            return Err(ShapeError::ConvexHull(HullError::Coplanar));
        }
        Ok(())
    }
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

#[cfg(test)]
mod tests;
