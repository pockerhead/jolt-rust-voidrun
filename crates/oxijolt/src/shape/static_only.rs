//! Which static-only shapes a kinematic body may use.

use std::ptr::null_mut;

use oxijolt_sys::*;

use super::Shape;

impl Shape {
    /// Whether every leaf of this shape that Jolt allows only on static bodies
    /// (`Shape::MustBeStatic`) is a mesh. Then a kinematic body may use the shape: it never
    /// collides with static or kinematic bodies, so Jolt never pairs a mesh with a mesh or a
    /// heightfield. Heightfields and planes keep the shape static-only.
    pub(crate) fn static_only_leaves_are_meshes(&self) -> bool {
        // SAFETY: `&self` keeps the shape alive for the call.
        unsafe { static_only_leaves_are_meshes_of(self.as_ptr()) }
    }
}

/// [`Shape::static_only_leaves_are_meshes`] for a raw shape, such as a body's own.
///
/// # Safety
/// `shape` is live for the call, kept alive by its owner or by a body locked meanwhile.
pub(crate) unsafe fn static_only_leaves_are_meshes_of(shape: *const JPH_Shape) -> bool {
    let mut pending = vec![shape];
    while let Some(shape) = pending.pop() {
        // SAFETY: `shape` is the caller's live shape or a child of a live shape below, which
        // its parent keeps alive; the getters only read it.
        let (static_only, sub_type) =
            unsafe { (JPH_Shape_MustBeStatic(shape), JPH_Shape_GetSubType(shape)) };
        if !static_only {
            continue;
        }
        if sub_type == JPH_ShapeSubType_Mesh {
            continue;
        }
        if sub_type == JPH_ShapeSubType_StaticCompound
            || sub_type == JPH_ShapeSubType_MutableCompound
        {
            // SAFETY: `shape` is live and a compound (checked above).
            unsafe { push_compound_children(shape.cast(), &mut pending) };
        } else if sub_type == JPH_ShapeSubType_Scaled
            || sub_type == JPH_ShapeSubType_OffsetCenterOfMass
            || sub_type == JPH_ShapeSubType_RotatedTranslated
        {
            // SAFETY: `shape` is live and a decorated shape (checked above); the inner shape is
            // kept alive by it.
            pending.push(unsafe { JPH_DecoratedShape_GetInnerShape(shape.cast()) });
        } else {
            return false;
        }
    }
    true
}

/// Pushes every child of `compound` onto `pending`.
///
/// # Safety
/// `compound` is a live compound shape.
unsafe fn push_compound_children(
    compound: *const JPH_CompoundShape,
    pending: &mut Vec<*const JPH_Shape>,
) {
    // SAFETY: `compound` is live (contract); the getter only reads it.
    let count = unsafe { JPH_CompoundShape_GetNumSubShapes(compound) };
    for index in 0..count {
        let mut child = std::ptr::null();
        // SAFETY: as above, `index < count`, and joltc writes only the outputs that are not
        // null; `child` is a live local and receives a shape the compound keeps alive.
        unsafe {
            JPH_CompoundShape_GetSubShape(
                compound,
                index,
                &mut child,
                null_mut(),
                null_mut(),
                null_mut(),
            );
        }
        pending.push(child);
    }
}
