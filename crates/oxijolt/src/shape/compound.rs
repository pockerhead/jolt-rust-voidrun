//! Compound shapes: the child rules, the sub-shape id rule, the builder behind
//! [`Shape::new_compound`] and the decoding of a compound child from a sub-shape id.

use std::collections::BTreeMap;
use std::ptr::{null_mut, NonNull};

use oxijolt_sys::*;

use super::{initialize, CompoundSubShape, Shape, ShapeSettings, SubShapeId};
use crate::{limits, Quat, ShapeError, Vec3};

pub(super) const EMPTY_COMPOUND_RULE: &str = "a compound needs at least one child";
pub(super) const CHILD_POSITION_RULE: &str =
    "compound child position must be finite and within limits::MAX_SHAPE_EXTENT";
pub(super) const CHILD_ROTATION_RULE: &str =
    "compound child rotation must be a finite unit quaternion";
pub(super) const SUB_SHAPE_ID_RULE: &str =
    "compound hierarchy does not fit Jolt's 32-bit sub-shape ids";

/// Bits in a Jolt `SubShapeID`.
const ID_BITS: u32 = 32;

/// The bits a compound of `count` children uses for a child index (Jolt
/// `CompoundShape::GetSubShapeIDBits`): 0 for one child, 1 for two, 2 for three or four.
pub(super) fn index_bits(count: u32) -> u32 {
    ID_BITS - count.saturating_sub(1).leading_zeros()
}

/// Checks a child's pose: position within [`limits::MAX_SHAPE_EXTENT`], rotation a finite unit
/// quaternion.
pub(super) fn check_child_pose(position: Vec3, rotation: Quat) -> Result<(), ShapeError> {
    if !limits::is_local_offset(position) {
        return Err(ShapeError::InvalidSettings(CHILD_POSITION_RULE));
    }
    if !rotation.is_valid_rotation() {
        return Err(ShapeError::InvalidSettings(CHILD_ROTATION_RULE));
    }
    Ok(())
}

/// How a shape uses Jolt's 32-bit sub-shape ids.
///
/// Jolt checks only the total width. It also pushes the index of a one-child compound, which
/// has 0 bits, at the bit where that compound starts (`CompoundShapeVisitors.h`), and a push at
/// bit 32 shifts a 32-bit value by 32 (`SubShapeID::PushID`), which C++ leaves undefined.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct SubShapeIds {
    /// Bits of the longest id path below the shape (Jolt `GetSubShapeIDBitsRecursive`).
    pub(super) width: u32,
    /// Largest bit, relative to the shape's own first bit, at which a 0-bit index is pushed;
    /// `None` when no such push happens.
    pub(super) zero_width_at: Option<u32>,
}

/// The ids of a compound of `count` children whose ids are `children`.
pub(super) fn compound_ids(children: impl Iterator<Item = SubShapeIds>, count: u32) -> SubShapeIds {
    let bits = index_bits(count);
    let own = (count > 0 && bits == 0).then_some(0);
    children.fold(
        SubShapeIds {
            width: bits,
            zero_width_at: own,
        },
        |ids, child| SubShapeIds {
            width: ids.width.max(bits + child.width),
            zero_width_at: ids
                .zero_width_at
                .max(child.zero_width_at.map(|at| at + bits)),
        },
    )
}

/// Whether Jolt can form every id of a root shape with `ids`. The width is left to Jolt's own
/// check; this adds the 0-bit push rule.
pub(super) fn fits_jolt_ids(ids: SubShapeIds) -> bool {
    ids.zero_width_at.is_none_or(|at| at < ID_BITS)
}

/// The ids of `shape`, walking compounds and decorators. `memo` maps shape addresses to their
/// `zero_width_at`, so a graph that shares shapes is walked once per distinct shape.
///
/// # Safety
/// `shape` is live for the call.
pub(super) unsafe fn sub_shape_ids(
    shape: *const JPH_Shape,
    memo: &mut BTreeMap<usize, Option<u32>>,
) -> SubShapeIds {
    // SAFETY: `shape` is live (contract); the getter only reads it.
    let width = unsafe { JPH_Shape_GetSubShapeIDBitsRecursive(shape) };
    SubShapeIds {
        width,
        // SAFETY: as above.
        zero_width_at: unsafe { zero_width_at(shape, memo) },
    }
}

/// [`SubShapeIds::zero_width_at`] of `shape`.
///
/// # Safety
/// `shape` is live for the call.
unsafe fn zero_width_at(
    shape: *const JPH_Shape,
    memo: &mut BTreeMap<usize, Option<u32>>,
) -> Option<u32> {
    if let Some(&known) = memo.get(&(shape as usize)) {
        return known;
    }
    // SAFETY: `shape` is live (contract); the getter only reads it.
    let sub_type = unsafe { JPH_Shape_GetSubType(shape) };
    let at = if sub_type == JPH_ShapeSubType_StaticCompound
        || sub_type == JPH_ShapeSubType_MutableCompound
    {
        let compound: *const JPH_CompoundShape = shape.cast();
        // SAFETY: `shape` is live and a compound (checked above); the getter only reads it.
        let count = unsafe { JPH_CompoundShape_GetNumSubShapes(compound) };
        let bits = index_bits(count);
        let own = (count > 0 && bits == 0).then_some(0);
        (0..count)
            .map(|index| {
                let mut child = std::ptr::null();
                // SAFETY: as above, `index < count`, and joltc writes only the outputs that are
                // not null; `child` receives a shape the compound keeps alive.
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
                // SAFETY: the child is live while its parent is.
                unsafe { zero_width_at(child, memo) }.map(|at| at + bits)
            })
            .fold(own, Option::max)
    } else if sub_type == JPH_ShapeSubType_Scaled
        || sub_type == JPH_ShapeSubType_OffsetCenterOfMass
        || sub_type == JPH_ShapeSubType_RotatedTranslated
    {
        // SAFETY: `shape` is live and a decorated shape (checked above); the inner shape is kept
        // alive by it. Decorators push no index of their own.
        unsafe { zero_width_at(JPH_DecoratedShape_GetInnerShape(shape.cast()), memo) }
    } else {
        // Leaves push no 0-bit index after a full 32 bits: convex shapes and planes push
        // nothing, heightfields at least 3 bits, meshes a triangle index after their block
        // index (`HeightFieldShape.cpp`, `MeshShape.cpp`).
        None
    };
    memo.insert(shape as usize, at);
    at
}

/// A compound child as the builder takes it.
pub(super) struct RawCompoundChild {
    pub(super) shape: *const JPH_Shape,
    pub(super) position: Vec3,
    pub(super) rotation: Quat,
    pub(super) user_data: u32,
    pub(super) ids: SubShapeIds,
}

/// The compound of `children`: a `StaticCompoundShape` for two or more, a
/// `MutableCompoundShape` for one (see [`Shape::new_compound`]).
///
/// Fails with [`ShapeError::InvalidSettings`] for no children or ids Jolt cannot form, with
/// [`ShapeError::Rejected`] when Jolt refuses the settings (a hierarchy wider than 32 bits), and
/// with [`ShapeError::InvalidDimensions`] when the compound's bounds, after Jolt moved its
/// centre of mass, leave [`limits::MAX_SHAPE_EXTENT`].
///
/// # Safety
/// Every `shape` is live for the call; every pose passed [`check_child_pose`]; there are at most
/// `u32::MAX` children.
pub(super) unsafe fn build_compound(children: &[RawCompoundChild]) -> Result<Shape, ShapeError> {
    if children.is_empty() {
        return Err(ShapeError::InvalidSettings(EMPTY_COMPOUND_RULE));
    }
    let ids = compound_ids(
        children.iter().map(|child| child.ids),
        children.len() as u32,
    );
    if !fits_jolt_ids(ids) {
        return Err(ShapeError::InvalidSettings(SUB_SHAPE_ID_RULE));
    }
    initialize()?;
    // SAFETY: Jolt is initialised. The returned settings hold one reference, which the guard
    // takes over.
    let settings = unsafe {
        ShapeSettings::from_raw(if children.len() == 1 {
            JPH_MutableCompoundShapeSettings_Create().cast()
        } else {
            JPH_StaticCompoundShapeSettings_Create().cast()
        })
    }?;
    for child in children {
        let position = child.position.to_jph();
        let rotation = child.rotation.to_jph();
        // SAFETY: the settings are live and owned by the guard; both compound settings types
        // derive from `CompoundShapeSettings` with single inheritance. The child shape is live
        // (contract), and the settings store their own `RefConst` to it. `position` and
        // `rotation` are live locals.
        unsafe {
            JPH_CompoundShapeSettings_AddShape2(
                settings.as_ptr(),
                &position,
                &rotation,
                child.shape,
                child.user_data,
            );
        }
    }
    // Jolt refuses, for example, a hierarchy that needs more than 32 sub-shape id bits.
    settings.create()?.within_extent_bounds()
}

/// The child of the compound `root` that `id` leads to; `None` unless `root` is a compound and
/// `id` names one of its children.
///
/// # Safety
/// `root` points to a live shape for the duration of the call.
pub(crate) unsafe fn compound_sub_shape_of(
    root: NonNull<JPH_Shape>,
    id: SubShapeId,
) -> Option<CompoundSubShape> {
    let root = root.as_ptr();
    // SAFETY: `root` is live (caller contract); the getter only reads it.
    let sub_type = unsafe { JPH_Shape_GetSubType(root) };
    if sub_type != JPH_ShapeSubType_StaticCompound && sub_type != JPH_ShapeSubType_MutableCompound {
        return None;
    }
    let compound: *const JPH_CompoundShape = root.cast();
    // SAFETY: `root` is live and a compound (checked above); the getter only reads it.
    let count = unsafe { JPH_CompoundShape_GetNumSubShapes(compound) };
    if count == 0 {
        return None;
    }
    let bits = index_bits(count);
    let mask = ((1_u64 << bits) - 1) as u32;
    // Rejecting out-of-range indices here keeps Jolt's index assertion unreachable.
    if id.to_raw() & mask >= count {
        return None;
    }
    let mut remainder: JPH_SubShapeID = 0;
    // SAFETY: as above; `remainder` is a live local and the index is in range.
    let index =
        unsafe { JPH_CompoundShape_GetSubShapeIndexFromID(compound, id.to_raw(), &mut remainder) };
    // Jolt indexes its child array without a check in `GetSubShape`.
    if index >= count {
        return None;
    }
    let mut user_data = 0;
    // SAFETY: as above, `index < count`, and joltc writes only the outputs that are not null.
    unsafe {
        JPH_CompoundShape_GetSubShape(
            compound,
            index,
            null_mut(),
            null_mut(),
            null_mut(),
            &mut user_data,
        );
    }
    Some(CompoundSubShape { index, user_data })
}
