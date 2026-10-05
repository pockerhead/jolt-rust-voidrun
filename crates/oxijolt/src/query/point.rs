//! Point queries: which shapes contain a point.

use std::any::Any;
use std::cell::{Cell, RefCell};
use std::ffi::c_void;
use std::panic::{catch_unwind, resume_unwind, AssertUnwindSafe};
use std::ptr::null;

use oxijolt_sys::*;

use crate::filter::{with_query_filters, FilterState};
use crate::limits;
use crate::{
    BodyId, CompoundSubShape, ObjectLayer, PhysicsWorld, QueryError, QueryFilter, RVec3, Shape,
    SubShapeId, Vec3,
};

/// One body whose shape contains the point of a [`PhysicsWorld::collide_point`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct PointHit {
    /// The body whose shape contains the point.
    pub body: BodyId,
    /// Path from the body's shape to the leaf shape that contains the point.
    pub sub_shape_id: SubShapeId,
    /// The object layer of the body.
    pub object_layer: ObjectLayer,
    /// The compound child that contains the point, when the body's shape is a compound.
    pub compound_child: Option<CompoundSubShape>,
}

/// Where the result callback of [`PhysicsWorld::collide_point`] collects every hit as
/// `(body id, sub-shape id)`, next to the query's filter state.
struct WorldPointSlot<'s> {
    state: &'s FilterState<'s>,
    hits: RefCell<Vec<(JPH_BodyID, JPH_SubShapeID)>>,
}

/// Result callback of [`PhysicsWorld::collide_point`]: copies the ids out of joltc's temporary
/// result.
///
/// # Safety
/// Called only by joltc during `collide_point`, with that call's live `WorldPointSlot` as
/// `user_data` and a live result.
unsafe extern "C" fn push_world_point_hit(
    user_data: *mut c_void,
    result: *const JPH_CollidePointResult,
) {
    // SAFETY: guaranteed by the caller (function contract); only shared references to the slot
    // exist.
    let slot = unsafe { &*user_data.cast::<WorldPointSlot<'_>>() };
    // SAFETY: guaranteed by the caller.
    let result = unsafe { *result };
    slot.state.guarded((), || {
        slot.hits
            .borrow_mut()
            .push((result.bodyID, result.subShapeID2));
    });
}

/// Where the result callback of [`Shape::collide_point`] collects every sub-shape id, and the
/// payload of a panic in it.
#[derive(Default)]
struct ShapePointSlot {
    ids: RefCell<Vec<JPH_SubShapeID>>,
    panic: Cell<Option<Box<dyn Any + Send>>>,
}

/// Result callback of [`Shape::collide_point`]: copies the id out of joltc's temporary result.
/// A panic is kept in the slot and resumed after joltc returned.
///
/// # Safety
/// Called only by joltc during `Shape::collide_point`, with that call's live `ShapePointSlot`
/// as `user_data` and a live result.
unsafe extern "C" fn push_shape_point_hit(
    user_data: *mut c_void,
    result: *const JPH_CollidePointResult,
) {
    // SAFETY: guaranteed by the caller (function contract); only shared references to the slot
    // exist.
    let slot = unsafe { &*user_data.cast::<ShapePointSlot>() };
    // SAFETY: guaranteed by the caller.
    let id = unsafe { (*result).subShapeID2 };
    if let Err(payload) = catch_unwind(AssertUnwindSafe(|| slot.ids.borrow_mut().push(id))) {
        slot.panic.set(Some(payload));
    }
}

impl PhysicsWorld {
    /// Every body among those `filter` selects whose shape contains `point` (world space,
    /// metres), with the leaf shape that contains it, sorted by body id, then sub-shape id.
    ///
    /// Jolt first takes the bodies whose broad-phase bounds contain the point (in `f32`, also
    /// with the `double-precision` feature) and then asks each body's shape. A bounds hit alone
    /// is no hit: what "contains" means depends on the shape, as for [`Shape::collide_point`].
    /// A plane body is found only within its half extent; a soft body is tested like a mesh, by
    /// the parity of its faces above the point. Child groups of the filter apply when
    /// the body's shape is a compound; a compound inside a decorator is not filtered by child.
    ///
    /// The point must be finite and within [`limits::MAX_POSITION`], and the filter valid for
    /// this world; otherwise [`QueryError::InvalidValue`] is returned.
    pub fn collide_point(
        &self,
        point: RVec3,
        filter: &QueryFilter<'_>,
    ) -> Result<Vec<PointHit>, QueryError> {
        if !limits::is_in_frame(point) {
            return Err(QueryError::InvalidValue(
                "point must be finite and within limits::MAX_POSITION",
            ));
        }
        filter.validate(self)?;
        let point = point.to_jph();
        let mut hits = with_query_filters(self, filter, |raw, state| {
            let slot = WorldPointSlot {
                state,
                hits: RefCell::new(Vec::new()),
            };
            // SAFETY: the query object lives inside this world's physics system. Jolt's locking
            // narrow-phase query takes body read locks and the broad-phase query lock, and
            // `step` needs `&mut self`, so no update runs meanwhile. `point` and `slot` are live
            // locals, `push_world_point_hit` matches the slot type and copies everything it
            // keeps, and the filters are live or null (Jolt's accept-all defaults).
            unsafe {
                JPH_NarrowPhaseQuery_CollidePoint2(
                    self.narrow_phase_query.as_ptr(),
                    &point,
                    JPH_CollisionCollectorType_AllHit,
                    Some(push_world_point_hit),
                    (&slot as *const WorldPointSlot<'_>).cast_mut().cast(),
                    null(),
                    raw.object_layer,
                    raw.body,
                    raw.shape,
                )
            };
            slot.hits.into_inner()
        })?;
        hits.sort_unstable();
        Ok(hits
            .into_iter()
            .map(|(body, id)| {
                let body = BodyId::new(body, self.tag);
                let sub_shape_id = SubShapeId::new(id);
                PointHit {
                    body,
                    sub_shape_id,
                    object_layer: self.object_layer_of(body),
                    compound_child: self.compound_sub_shape(body, sub_shape_id).ok().flatten(),
                }
            })
            .collect())
    }
}

impl Shape {
    /// The sub-shape ids of every leaf of this shape that contains `point`, given in the shape's
    /// own frame (the frame a body puts at its position), sorted; empty when none does.
    ///
    /// What "contains" means is Jolt's per shape (`Shape::CollidePoint`):
    /// - box, sphere: the solid shape, boundary included (a box's convex radius is ignored);
    /// - capsule, cylinder, tapered cylinder, convex hull: the solid shape (sharp edges for
    ///   cylinders and hulls); points within about 1e-4 m of the surface may go either way;
    /// - tapered capsule: within about 1e-4 m of the rounded shape;
    /// - mesh: the point lies in the shape's bounds and a ray from it along +Y crosses an odd
    ///   number of triangles (the id is the last triangle crossed). Jolt does not check that the
    ///   mesh is closed: an open mesh reports points whose ray happens to cross an odd number of
    ///   triangles. A ray through an edge or a vertex counts every triangle that meets there, so
    ///   such a point can come out on the wrong side even for a closed mesh: the centre of a cube
    ///   mesh whose top face is split along a diagonal is outside. Every other point of a closed
    ///   mesh comes out as the mesh encloses it;
    /// - heightfield: never;
    /// - plane: strictly behind the plane (`normal · p + constant < 0`), anywhere, also beyond
    ///   the half extent, unlike a ray, which counts the plane itself as solid;
    /// - compound: each child whose bounds contain the point is asked (so a plane child is only
    ///   found within its bounds); scaled, rotated-translated and offset shapes ask their inner
    ///   shape.
    ///
    /// Each component of `point` must be finite and at most [`limits::MAX_SHAPE_EXTENT`] in
    /// absolute value; otherwise [`QueryError::InvalidValue`] is returned.
    pub fn collide_point(&self, point: Vec3) -> Result<Vec<SubShapeId>, QueryError> {
        if !limits::is_local_offset(point) {
            return Err(QueryError::InvalidValue(
                "point must be finite and within limits::MAX_SHAPE_EXTENT",
            ));
        }
        // Jolt tests the point relative to the centre of mass.
        let center = self.center_of_mass();
        let local = Vec3::new(point.x - center.x, point.y - center.y, point.z - center.z).to_jph();
        let slot = ShapePointSlot::default();
        // SAFETY: the shape is live for the call (borrowed by `self`); `local` and `slot` are
        // live locals, `push_shape_point_hit` matches the slot type and copies everything it
        // keeps, and a null shape filter is Jolt's accept-all default.
        unsafe {
            JPH_Shape_CollidePoint2(
                self.as_ptr(),
                &local,
                JPH_CollisionCollectorType_AllHit,
                Some(push_shape_point_hit),
                (&slot as *const ShapePointSlot).cast_mut().cast(),
                null(),
            )
        };
        if let Some(payload) = slot.panic.take() {
            resume_unwind(payload);
        }
        let mut ids: Vec<SubShapeId> = slot
            .ids
            .into_inner()
            .into_iter()
            .map(SubShapeId::new)
            .collect();
        ids.sort_unstable();
        Ok(ids)
    }
}
