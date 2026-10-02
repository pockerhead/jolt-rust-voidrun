//! Collision shapes.

use std::ptr::NonNull;

use joltphysics_sys::*;

use crate::world::ensure_initialized;
use crate::{ShapeError, Vec3};

/// A collision shape that bodies are created from.
///
/// Owns one Jolt reference. Every body created from it holds its own reference, so the shape
/// may be dropped while bodies use it. Jolt shapes cannot change after construction, so one
/// shape may serve any number of bodies in any number of worlds.
pub struct Shape {
    ptr: NonNull<JPH_Shape>,
}

// SAFETY: Jolt shapes are immutable after construction and `RefTarget` counts references
// atomically, so a shape may be used and released from any thread
// (https://jrouwe.github.io/JoltPhysicsDocs/5.3.0/index.html#memory-management).
unsafe impl Send for Shape {}
// SAFETY: as for `Send`; `&Shape` only lets bodies take further references and read the
// immutable shape.
unsafe impl Sync for Shape {}

/// Owns a `JPH_ShapeSettings`: the one reference every `JPH_*ShapeSettings_Create` returns
/// (joltc calls `AddRef` on the new settings).
///
/// Dropping it releases that reference, which deletes the settings and with them their
/// cached `ShapeResult` reference. A shape returned by a `*_CreateShape` or `*_Create` call
/// carries its own reference (joltc calls `AddRef` before returning it), so the caller keeps
/// exactly one reference to the shape.
struct ShapeSettings(NonNull<JPH_ShapeSettings>);

impl ShapeSettings {
    fn from_raw(ptr: *mut JPH_ShapeSettings) -> Result<Self, ShapeError> {
        NonNull::new(ptr)
            .map(Self)
            .ok_or(ShapeError::AllocationFailed)
    }

    /// The settings as one of joltc's typed settings pointers. joltc's settings types are
    /// `reinterpret_cast`s of Jolt classes with single inheritance from `ShapeSettings`, the
    /// convention joltc itself uses, so the caller picks the type the settings were created as.
    fn as_ptr<T>(&self) -> *mut T {
        self.0.as_ptr().cast()
    }
}

impl Drop for ShapeSettings {
    fn drop(&mut self) {
        // SAFETY: this value owns exactly one reference to the settings, released here once.
        // Shapes created from them hold their own references.
        unsafe { JPH_ShapeSettings_Destroy(self.0.as_ptr()) };
    }
}

/// Finite and positive.
fn is_positive(value: f32) -> bool {
    value.is_finite() && value > 0.0
}

/// Finite and not negative.
fn is_valid_convex_radius(value: f32) -> bool {
    value.is_finite() && value >= 0.0
}

impl Shape {
    /// A box with the given half extents in metres (each finite and positive) and Jolt's
    /// default convex radius of 0.05 m; see [`new_box_with_convex_radius`].
    ///
    /// [`new_box_with_convex_radius`]: Self::new_box_with_convex_radius
    pub fn new_box(half_extent: Vec3) -> Result<Self, ShapeError> {
        Self::new_box_with_convex_radius(half_extent, JPH_DEFAULT_CONVEX_RADIUS as f32)
    }

    /// A box with the given half extents in metres (each finite and positive) and convex
    /// radius in metres (finite and not negative).
    ///
    /// Jolt shrinks the box by the convex radius and inflates it again, so the faces stay where
    /// they are while edges and corners are rounded by the radius for contacts and shape casts.
    /// Jolt clamps the radius to the smallest half extent (`BoxShape.h`). A radius of 0 gives
    /// sharp edges; collision detection is then somewhat slower, because Jolt falls back to
    /// EPA more often. Ray casts always see the sharp box, whatever the radius
    /// (`BoxShape::CastRay` tests the half extents only).
    pub fn new_box_with_convex_radius(
        half_extent: Vec3,
        convex_radius: f32,
    ) -> Result<Self, ShapeError> {
        let components = [half_extent.x, half_extent.y, half_extent.z];
        if !components.into_iter().all(is_positive) {
            return Err(ShapeError::InvalidDimensions(
                "box half extents must be finite and positive",
            ));
        }
        if !is_valid_convex_radius(convex_radius) {
            return Err(ShapeError::InvalidDimensions(
                "convex radius must be finite and not negative",
            ));
        }
        if !ensure_initialized() {
            return Err(ShapeError::InitFailed);
        }
        let half_extent = half_extent.to_jph();
        // SAFETY: Jolt is initialised, `half_extent` is a live local and both inputs were
        // checked against Jolt's assertions. The returned box holds one reference, which
        // `Self` takes over.
        let ptr = unsafe { JPH_BoxShape_Create(&half_extent, convex_radius) };
        Self::from_raw(ptr.cast())
    }

    /// A sphere with the given radius in metres (finite and positive).
    pub fn new_sphere(radius: f32) -> Result<Self, ShapeError> {
        if !is_positive(radius) {
            return Err(ShapeError::InvalidDimensions(
                "sphere radius must be finite and positive",
            ));
        }
        if !ensure_initialized() {
            return Err(ShapeError::InitFailed);
        }
        // SAFETY: Jolt is initialised. The returned sphere holds one reference, which `Self`
        // takes over.
        let ptr = unsafe { JPH_SphereShape_Create(radius) };
        Self::from_raw(ptr.cast())
    }

    /// A cylinder along the local Y axis, centred on the origin, `2 * half_height` metres high,
    /// with Jolt's default convex radius of 0.05 m; see
    /// [`new_cylinder_with_convex_radius`].
    ///
    /// [`new_cylinder_with_convex_radius`]: Self::new_cylinder_with_convex_radius
    pub fn new_cylinder(half_height: f32, radius: f32) -> Result<Self, ShapeError> {
        Self::new_cylinder_with_convex_radius(half_height, radius, JPH_DEFAULT_CONVEX_RADIUS as f32)
    }

    /// A cylinder along the local Y axis, centred on the origin, `2 * half_height` metres high.
    ///
    /// Half height and radius must be finite and positive, the convex radius finite and not
    /// negative. Like a box's, the convex radius rounds the edges for contacts; Jolt clamps it
    /// to `min(half_height, radius)` (`CylinderShape.cpp`).
    pub fn new_cylinder_with_convex_radius(
        half_height: f32,
        radius: f32,
        convex_radius: f32,
    ) -> Result<Self, ShapeError> {
        if !(is_positive(half_height) && is_positive(radius)) {
            return Err(ShapeError::InvalidDimensions(
                "cylinder half height and radius must be finite and positive",
            ));
        }
        if !is_valid_convex_radius(convex_radius) {
            return Err(ShapeError::InvalidDimensions(
                "convex radius must be finite and not negative",
            ));
        }
        if !ensure_initialized() {
            return Err(ShapeError::InitFailed);
        }
        // `JPH_CylinderShape_Create` would ignore the convex radius (joltc passes 0), so the
        // cylinder is built through its settings.
        // SAFETY: Jolt is initialised; the returned settings hold one reference, which the
        // guard takes over.
        let raw = unsafe { JPH_CylinderShapeSettings_Create(half_height, radius, convex_radius) };
        let settings = ShapeSettings::from_raw(raw.cast())?;
        // SAFETY: the settings are live, owned by the guard and were created as cylinder
        // settings. The returned shape holds one reference, which `Self` takes over.
        let ptr = unsafe { JPH_CylinderShapeSettings_CreateShape(settings.as_ptr()) };
        Self::from_created(ptr.cast())
    }

    /// A capsule along the local Y axis, centred on the origin: a cylinder
    /// `2 * half_height_of_cylinder` metres high with a hemisphere of `radius` at each end, so
    /// `2 * (half_height_of_cylinder + radius)` metres high in total. Both values must be finite
    /// and positive.
    pub fn new_capsule(half_height_of_cylinder: f32, radius: f32) -> Result<Self, ShapeError> {
        if !(is_positive(half_height_of_cylinder) && is_positive(radius)) {
            return Err(ShapeError::InvalidDimensions(
                "capsule half height and radius must be finite and positive",
            ));
        }
        if !ensure_initialized() {
            return Err(ShapeError::InitFailed);
        }
        // SAFETY: Jolt is initialised and both values are positive, as Jolt asserts
        // (`CapsuleShape.h`). The returned capsule holds one reference, which `Self` takes
        // over.
        let ptr = unsafe { JPH_CapsuleShape_Create(half_height_of_cylinder, radius) };
        Self::from_raw(ptr.cast())
    }

    fn from_raw(ptr: *mut JPH_Shape) -> Result<Self, ShapeError> {
        NonNull::new(ptr)
            .map(|ptr| Self { ptr })
            .ok_or(ShapeError::AllocationFailed)
    }

    /// Takes over a shape returned by a settings `Create` call, where null means Jolt refused
    /// the settings.
    fn from_created(ptr: *mut JPH_Shape) -> Result<Self, ShapeError> {
        NonNull::new(ptr)
            .map(|ptr| Self { ptr })
            .ok_or(ShapeError::Rejected)
    }

    pub(crate) fn as_ptr(&self) -> *const JPH_Shape {
        self.ptr.as_ptr()
    }

    /// Jolt's concrete shape type.
    #[cfg(test)]
    fn sub_type(&self) -> JPH_ShapeSubType {
        // SAFETY: the shape is live for the call; the getter only reads it.
        unsafe { JPH_Shape_GetSubType(self.as_ptr()) }
    }
}

impl Drop for Shape {
    fn drop(&mut self) {
        // SAFETY: `self` owns exactly one reference, released here once. Bodies keep their own.
        unsafe { JPH_Shape_Destroy(self.ptr.as_ptr()) };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn many_shapes_are_created_and_released() {
        for i in 0..1000 {
            let size = 0.1 + i as f32 * 0.001;
            drop(Shape::new_box(Vec3::new(size, size, size)).unwrap());
            drop(Shape::new_sphere(size).unwrap());
            drop(Shape::new_cylinder(size, size).unwrap());
            drop(Shape::new_capsule(size, size).unwrap());
        }
    }

    #[test]
    fn invalid_dimensions_are_rejected() {
        let invalid = |result: Result<Shape, ShapeError>| {
            matches!(result, Err(ShapeError::InvalidDimensions(_)))
        };
        for bad in [0.0, -1.0, f32::NAN, f32::INFINITY] {
            assert!(invalid(Shape::new_box(Vec3::new(1.0, bad, 1.0))));
            assert!(invalid(Shape::new_sphere(bad)));
            assert!(invalid(Shape::new_cylinder(bad, 1.0)));
            assert!(invalid(Shape::new_cylinder(1.0, bad)));
            assert!(invalid(Shape::new_capsule(bad, 1.0)));
            assert!(invalid(Shape::new_capsule(1.0, bad)));
        }
        for bad in [-0.1, f32::NAN, f32::INFINITY] {
            assert!(invalid(Shape::new_box_with_convex_radius(
                Vec3::new(1.0, 1.0, 1.0),
                bad
            )));
            assert!(invalid(Shape::new_cylinder_with_convex_radius(
                1.0, 1.0, bad
            )));
        }
        assert!(Shape::new_box_with_convex_radius(Vec3::new(1.0, 1.0, 1.0), 0.0).is_ok());
        assert!(Shape::new_cylinder_with_convex_radius(1.0, 1.0, 0.0).is_ok());
    }

    fn box_convex_radius(shape: &Shape) -> f32 {
        assert_eq!(shape.sub_type(), JPH_ShapeSubType_Box);
        // SAFETY: the shape is live and a box (checked above); the getter only reads it.
        unsafe { JPH_BoxShape_GetConvexRadius(shape.as_ptr().cast()) }
    }

    #[test]
    fn box_convex_radius_reaches_jolt() {
        let unit = Vec3::new(1.0, 1.0, 1.0);
        let sharp = Shape::new_box_with_convex_radius(unit, 0.0).unwrap();
        assert_eq!(box_convex_radius(&sharp), 0.0);
        let default = Shape::new_box(unit).unwrap();
        assert_eq!(box_convex_radius(&default), 0.05);
        let clamped = Shape::new_box_with_convex_radius(Vec3::new(0.1, 1.0, 1.0), 0.5).unwrap();
        assert_eq!(box_convex_radius(&clamped), 0.1);
    }

    #[test]
    fn cylinder_and_capsule_dimensions_reach_jolt() {
        let cylinder = Shape::new_cylinder(0.75, 0.3).unwrap();
        let capsule = Shape::new_capsule(0.70845, 0.4).unwrap();
        // SAFETY: both shapes are live and of the type each getter expects (their subtypes are
        // checked in `new_shapes_have_jolt_subtypes`); the getters only read them.
        unsafe {
            assert_eq!(
                JPH_CylinderShape_GetHalfHeight(cylinder.as_ptr().cast()),
                0.75
            );
            assert_eq!(JPH_CylinderShape_GetRadius(cylinder.as_ptr().cast()), 0.3);
            assert_eq!(
                JPH_CapsuleShape_GetHalfHeightOfCylinder(capsule.as_ptr().cast()),
                0.70845
            );
            assert_eq!(JPH_CapsuleShape_GetRadius(capsule.as_ptr().cast()), 0.4);
        }
    }

    #[test]
    fn new_shapes_have_jolt_subtypes() {
        let unit = Vec3::new(1.0, 1.0, 1.0);
        assert_eq!(
            Shape::new_box(unit).unwrap().sub_type(),
            JPH_ShapeSubType_Box
        );
        assert_eq!(
            Shape::new_cylinder(1.0, 0.5).unwrap().sub_type(),
            JPH_ShapeSubType_Cylinder
        );
        assert_eq!(
            Shape::new_capsule(1.0, 0.5).unwrap().sub_type(),
            JPH_ShapeSubType_Capsule
        );
        assert_eq!(
            Shape::new_sphere(1.0).unwrap().sub_type(),
            JPH_ShapeSubType_Sphere
        );
    }
}
