//! Scaled shapes.

use std::ptr::{null, null_mut};

use oxijolt_sys::*;

use super::geometry::{add, diagonal, mul, mul_vec, rotation, v3, M3, V3};
use super::mesh::{is_collidable, rounding_displacement};
use super::{initialize, Shape, ShapeSettings};
use crate::{limits, ShapeError, Vec3};

impl Shape {
    /// `shape` scaled by `scale` along its local axes, a Jolt `ScaledShape`. The new shape holds
    /// its own reference to `shape`, which may be dropped afterwards.
    ///
    /// Each component must be finite ([`ShapeError::InvalidDimensions`]) and the scale valid for
    /// the shape under Jolt's rules ([`ShapeError::InvalidSettings`]): every component at least
    /// 1e-6 in absolute value; uniform for spheres, capsules and tapered capsules; uniform in X
    /// and Z for cylinders and tapered cylinders; for a compound, uniform unless every rotated
    /// child is turned so the scale maps onto its own axes. Negative components mirror the
    /// shape; a mirrored mesh's front faces follow the mirrored winding.
    ///
    /// Meshes and heightfields inside the shape must stay collidable: each stored triangle,
    /// scaled, must keep twice its area above the floor [`Shape::new_mesh`] applies, so
    /// shrinking a mesh far below the size it was built at is refused
    /// ([`ShapeError::InvalidSettings`]; [docs/limits.md#scaled-shapes]). The check reads every
    /// stored triangle once.
    ///
    /// The scaled shape's local bounds must lie within [`limits::MAX_SHAPE_EXTENT`] on every
    /// axis, and so must its centre of mass, which moves with the scale
    /// ([`ShapeError::InvalidDimensions`]). Mass and inertia scale with the shape; a body made
    /// of it goes through the usual mass and inertia checks of
    /// [`PhysicsWorld::create_body`](crate::PhysicsWorld::create_body). A shape that only
    /// static bodies may use stays static-only, and a scaled mesh stays usable by kinematic
    /// bodies. Jolt scales the convex radius by the smallest absolute component, and contacts
    /// use at most 0.05 m of it. [`compound_sub_shape`](Self::compound_sub_shape) does not look
    /// through the decorator: it returns `None` for a scaled compound.
    ///
    /// [docs/limits.md#scaled-shapes]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#scaled-shapes
    pub fn scaled(shape: &Shape, scale: Vec3) -> Result<Self, ShapeError> {
        // Jolt's own check lets NaN and infinity through (`ScaleHelpers::IsZeroScale`).
        if !scale.is_finite() {
            return Err(ShapeError::InvalidDimensions("scale must be finite"));
        }
        initialize()?;
        let jolt_scale = scale.to_jph();
        // SAFETY: `shape` is live for the call and `jolt_scale` is a live local; the check only
        // reads them.
        if !unsafe { JPH_Shape_IsValidScale(shape.as_ptr(), &jolt_scale) } {
            return Err(ShapeError::InvalidSettings(
                "scale is not valid for this shape",
            ));
        }
        if !triangles_stay_collidable(shape, scale) {
            return Err(ShapeError::InvalidSettings(
                "scale makes mesh or heightfield triangles too small to collide with",
            ));
        }
        // SAFETY: Jolt is initialised, `shape` is live (the settings take their own reference to
        // it) and `jolt_scale` is a live local. The returned settings hold one reference, which
        // the guard takes over.
        let settings = unsafe {
            ShapeSettings::from_raw(
                JPH_ScaledShapeSettings_Create2(shape.as_ptr(), &jolt_scale).cast(),
            )
        }?;
        let scaled = settings.create()?.within_extent_bounds()?;
        // Bounds are relative to the centre of mass, which the scale moves too.
        if !limits::is_local_offset(scaled.center_of_mass()) {
            return Err(ShapeError::InvalidDimensions(
                "scaled centre of mass must lie within limits::MAX_SHAPE_EXTENT",
            ));
        }
        Ok(scaled)
    }
}

/// Where a shape inside a scaled shape lies: the linear map and offset from its centre of mass
/// space to the scaled shape's.
#[derive(Clone, Copy)]
struct Placement {
    linear: M3,
    offset: V3,
}

impl Placement {
    fn apply(&self, v: V3) -> V3 {
        add(mul_vec(&self.linear, v), self.offset)
    }

    /// The placement of a part at `position` with `rotation` (as a quaternion) in this one.
    fn then(&self, position: V3, rotation_xyzw: [f64; 4]) -> Self {
        Self {
            linear: mul(&self.linear, &rotation(rotation_xyzw)),
            offset: self.apply(position),
        }
    }
}

/// Whether every stored triangle of the meshes and heightfields in `shape` stays collidable once
/// scaled by `scale` ([`is_collidable`], with the rounding of the scaled coordinates).
fn triangles_stay_collidable(shape: &Shape, scale: Vec3) -> bool {
    let root = Placement {
        linear: diagonal(v3(scale)),
        offset: [0.0; 3],
    };
    let mut pending = vec![(shape.as_ptr(), root)];
    while let Some((part, placement)) = pending.pop() {
        // SAFETY: `part` is `shape`, kept alive by the borrow, or a part of a live shape below,
        // which its parent keeps alive; the getters only read it.
        let (static_only, sub_type) =
            unsafe { (JPH_Shape_MustBeStatic(part), JPH_Shape_GetSubType(part)) };
        // Only meshes and heightfields collide through their triangles, and only they (and the
        // shapes containing them) are static-only.
        if !static_only {
            continue;
        }
        if sub_type == JPH_ShapeSubType_Mesh || sub_type == JPH_ShapeSubType_HeightField {
            // SAFETY: `part` is live (see above).
            if !unsafe { stored_triangles_collidable(part, &placement) } {
                return false;
            }
        } else {
            // SAFETY: `part` is live (see above) and of `sub_type`.
            let known = unsafe { push_parts(part, sub_type, &placement, &mut pending) };
            if !known {
                return false;
            }
        }
    }
    true
}

/// Pushes the parts of the compound or decorated shape `shape` with their placements; `false`
/// for any other sub type, whose triangles this check cannot reach.
///
/// # Safety
/// `shape` is a live shape of sub type `sub_type`.
unsafe fn push_parts(
    shape: *const JPH_Shape,
    sub_type: JPH_ShapeSubType,
    placement: &Placement,
    pending: &mut Vec<(*const JPH_Shape, Placement)>,
) -> bool {
    let quat = |q: JPH_Quat| [q.x, q.y, q.z, q.w].map(f64::from);
    let mut position = Vec3::ZERO.to_jph();
    let mut turn = JPH_Quat {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0,
    };
    if sub_type == JPH_ShapeSubType_StaticCompound || sub_type == JPH_ShapeSubType_MutableCompound {
        let compound: *const JPH_CompoundShape = shape.cast();
        // SAFETY: `shape` is a live compound (contract); the getter only reads it.
        let count = unsafe { JPH_CompoundShape_GetNumSubShapes(compound) };
        for index in 0..count {
            let mut child = null();
            // SAFETY: as above, `index < count`, and every output is a live local; the child is
            // kept alive by the compound.
            unsafe {
                JPH_CompoundShape_GetSubShape(
                    compound,
                    index,
                    &mut child,
                    &mut position,
                    &mut turn,
                    null_mut(),
                );
            }
            let child_placement = placement.then(v3(Vec3::from_jph(position)), quat(turn));
            pending.push((child, child_placement));
        }
        return true;
    }
    let decorators = [
        JPH_ShapeSubType_Scaled,
        JPH_ShapeSubType_RotatedTranslated,
        JPH_ShapeSubType_OffsetCenterOfMass,
    ];
    if !decorators.contains(&sub_type) {
        return false;
    }
    let decorated: *const JPH_DecoratedShape = shape.cast();
    // SAFETY: `shape` is a live decorated shape (checked above); the inner shape is kept alive
    // by it.
    let inner = unsafe { JPH_DecoratedShape_GetInnerShape(decorated) };
    let inner_placement = if sub_type == JPH_ShapeSubType_Scaled {
        let mut inner_scale = Vec3::ZERO.to_jph();
        // SAFETY: `shape` is a live scaled shape; `inner_scale` is a live local.
        unsafe { JPH_ScaledShape_GetScale(shape.cast(), &mut inner_scale) };
        Placement {
            linear: mul(
                &placement.linear,
                &diagonal(v3(Vec3::from_jph(inner_scale))),
            ),
            offset: placement.offset,
        }
    } else if sub_type == JPH_ShapeSubType_RotatedTranslated {
        let rotated: *const JPH_RotatedTranslatedShape = shape.cast();
        // SAFETY: `shape` is a live rotated-translated shape; the outputs are live locals.
        unsafe {
            JPH_RotatedTranslatedShape_GetPosition(rotated, &mut position);
            JPH_RotatedTranslatedShape_GetRotation(rotated, &mut turn);
        }
        placement.then(v3(Vec3::from_jph(position)), quat(turn))
    } else {
        // An offset centre of mass moves only the centre of mass; the inner surface stays put.
        *placement
    };
    pending.push((inner, inner_placement));
    true
}

/// Whether every stored triangle of the mesh or heightfield `shape`, placed by `placement`,
/// stays collidable.
///
/// # Safety
/// `shape` is a live mesh or heightfield.
unsafe fn stored_triangles_collidable(shape: *const JPH_Shape, placement: &Placement) -> bool {
    // SAFETY: `shape` is live (contract); a null buffer with capacity 0 only counts.
    let count = unsafe { JPH_Shape_GetTriangles(shape, null_mut(), 0) };
    let mut vertices = vec![Vec3::ZERO.to_jph(); 3 * count as usize];
    // SAFETY: as above; `vertices` holds 3 * count vertices.
    unsafe { JPH_Shape_GetTriangles(shape, vertices.as_mut_ptr(), count) };
    let placed: Vec<V3> = vertices
        .iter()
        .map(|&vertex| placement.apply(v3(Vec3::from_jph(vertex))))
        .collect();
    let largest = placed
        .iter()
        .flatten()
        .fold(0.0, |largest: f64, coordinate| {
            largest.max(coordinate.abs())
        });
    let displacement = rounding_displacement(largest);
    placed
        .as_chunks::<3>()
        .0
        .iter()
        .all(|&corners| is_collidable(corners, displacement))
}

#[cfg(test)]
mod tests;
