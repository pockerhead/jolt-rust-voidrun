//! Collision shapes.

use std::ptr::NonNull;

use joltc_sys::*;

use crate::world::ensure_initialized;
use crate::{ShapeError, Vec3};

/// A collision shape that bodies are created from.
///
/// Owns one Jolt reference. Every body created from it holds its own reference, so the shape
/// may be dropped while bodies use it. Jolt shapes cannot change after construction.
pub struct Shape {
    ptr: NonNull<JPH_Shape>,
}

// SAFETY: Jolt shapes are immutable after construction and `RefTarget` counts references
// atomically, so a shape may be used and released from any thread
// (https://jrouwe.github.io/JoltPhysicsDocs/5.3.0/index.html#memory-management).
unsafe impl Send for Shape {}
// SAFETY: as for `Send`; `&Shape` only lets bodies take further references.
unsafe impl Sync for Shape {}

impl Shape {
    /// A box with the given half extents in metres (each finite and positive).
    ///
    /// Jolt gives boxes a convex radius of 0.05 m, clamped to the smallest half extent
    /// (`BoxShape.h`): edges are rounded by that radius, so a thin box behaves like a rounded
    /// slab. Collision detection is faster with a convex radius than without.
    pub fn new_box(half_extent: Vec3) -> Result<Self, ShapeError> {
        let components = [half_extent.x, half_extent.y, half_extent.z];
        if !components.iter().all(|c| c.is_finite() && *c > 0.0) {
            return Err(ShapeError::InvalidDimensions(
                "box half extents must be finite and positive",
            ));
        }
        if !ensure_initialized() {
            return Err(ShapeError::InitFailed);
        }
        let half_extent = half_extent.to_jph();
        // SAFETY: Jolt is initialised and `half_extent` is a live local. The returned box holds
        // one reference, which `Self` takes over.
        let ptr = unsafe { JPH_BoxShape_Create(&half_extent, JPH_DEFAULT_CONVEX_RADIUS as f32) };
        Self::from_raw(ptr.cast())
    }

    /// A sphere with the given radius in metres (finite and positive).
    pub fn new_sphere(radius: f32) -> Result<Self, ShapeError> {
        if !(radius.is_finite() && radius > 0.0) {
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

    fn from_raw(ptr: *mut JPH_Shape) -> Result<Self, ShapeError> {
        NonNull::new(ptr)
            .map(|ptr| Self { ptr })
            .ok_or(ShapeError::AllocationFailed)
    }

    pub(crate) fn as_ptr(&self) -> *const JPH_Shape {
        self.ptr.as_ptr()
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
        }
    }

    #[test]
    fn invalid_dimensions_are_rejected() {
        for bad in [0.0, -1.0, f32::NAN, f32::INFINITY] {
            assert!(matches!(
                Shape::new_box(Vec3::new(1.0, bad, 1.0)),
                Err(ShapeError::InvalidDimensions(_))
            ));
            assert!(matches!(
                Shape::new_sphere(bad),
                Err(ShapeError::InvalidDimensions(_))
            ));
        }
    }
}
