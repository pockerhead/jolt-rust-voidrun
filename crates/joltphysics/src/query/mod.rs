//! Scene queries: ray casts, shape casts and collide-shape queries.
//!
//! Queries take `&PhysicsWorld` and may run on many threads at once while nobody steps the
//! world. They see bodies created, moved and removed through this API immediately, without a
//! step; [`PhysicsWorld::optimize_broad_phase`] only makes them faster after many bodies were
//! added one by one. Every query takes a [`QueryFilter`].
//!
//! Units are metres. Every reported normal is the outward surface normal of the obstacle (the
//! body that was hit) in world space: a floor below the query gives a normal pointing up, a
//! ceiling above it a normal pointing down.

use std::cell::Cell;
use std::ffi::c_void;
use std::mem::MaybeUninit;
use std::ptr::{null, NonNull};

use joltphysics_sys::*;

use crate::body::with_read_locked_body;
use crate::filter::{with_query_filters, FilterState};
use crate::limits;
use crate::shape::compound_sub_shape_of;
use crate::{
    BodyError, BodyId, CompoundSubShape, ObjectLayer, PhysicsWorld, Quat, QueryError, QueryFilter,
    RVec3, Real, Shape, SubShapeId, Vec3,
};

mod cast;
mod collide;

pub use cast::{ShapeCast, ShapeCastHit};
pub use collide::{CollideShape, CollideShapeHit};

/// A ray from `origin` along `direction`. The direction's length is the ray's length; hits
/// report the fraction along it, in `[0, 1]`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RayCast {
    /// Start of the ray in world space, metres.
    pub origin: RVec3,
    /// Direction and length of the ray, metres.
    pub direction: Vec3,
}

impl RayCast {
    /// A ray from `origin` covering `direction`.
    pub fn new(origin: RVec3, direction: Vec3) -> Self {
        Self { origin, direction }
    }

    /// The point `origin + direction * fraction`, computed as Jolt does
    /// (`RRayCast::GetPointOnRay`).
    pub fn point_at(&self, fraction: f32) -> RVec3 {
        RVec3::new(
            self.origin.x + Real::from(self.direction.x * fraction),
            self.origin.y + Real::from(self.direction.y * fraction),
            self.origin.z + Real::from(self.direction.z * fraction),
        )
    }
}

/// The closest hit of a [`PhysicsWorld::cast_ray`].
///
/// Convex shapes are solid: a ray that starts inside one hits it at fraction `0.0` exactly.
/// Triangle back faces are hit, so a heightfield is hit from below too.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct RayHit {
    /// The body that was hit.
    pub body: BodyId,
    /// Fraction along the ray's direction where it hit, in `[0, 1]`.
    pub fraction: f32,
    /// Path from the body's shape to the leaf shape that was hit.
    pub sub_shape_id: SubShapeId,
    /// Distance from the origin to the hit in metres: `fraction` times the direction's length.
    pub distance: f32,
    /// Outward surface normal of the hit face in world space, unit length.
    ///
    /// For a triangle hit from its back (a heightfield from below) it is still the face's
    /// normal, so it points away from the side the ray came from: `normal . direction > 0`.
    /// For a ray that starts inside a convex shape (`fraction == 0.0`) it is the normal Jolt
    /// computes at the origin, the normal of a nearby face with no unique meaning.
    pub normal: Vec3,
    /// The object layer of the hit body.
    pub object_layer: ObjectLayer,
    /// The compound child that was hit, when the body's shape is a compound.
    pub compound_child: Option<CompoundSubShape>,
}

/// Where a result callback stores what joltc reports, next to the query's filter state.
pub(crate) struct ResultSlot<'s, T> {
    state: &'s FilterState<'s>,
    pub(crate) hit: Cell<Option<T>>,
}

impl<'s, T: Copy> ResultSlot<'s, T> {
    pub(crate) fn new(state: &'s FilterState<'s>) -> Self {
        Self {
            state,
            hit: Cell::new(None),
        }
    }

    pub(crate) fn as_user_data(&self) -> *mut c_void {
        (self as *const Self).cast_mut().cast()
    }

    /// Stores a copy of `*result`.
    ///
    /// # Safety
    /// `user_data` points to a live `ResultSlot<T>` and `result` to a live `T`.
    pub(crate) unsafe fn store(user_data: *mut c_void, result: *const T) {
        // SAFETY: guaranteed by the caller; only shared references to the slot exist.
        let slot = unsafe { &*user_data.cast::<Self>() };
        // SAFETY: guaranteed by the caller.
        let result = unsafe { *result };
        slot.state.guarded((), || slot.hit.set(Some(result)));
    }
}

/// Result callback of [`PhysicsWorld::cast_ray`].
///
/// # Safety
/// Called only by joltc during `cast_ray`, with that call's live
/// `ResultSlot<JPH_RayCastResult>` as `user_data` and a live result.
unsafe extern "C" fn store_ray_hit(user_data: *mut c_void, result: *const JPH_RayCastResult) {
    // SAFETY: guaranteed by the caller (function contract).
    unsafe { ResultSlot::<JPH_RayCastResult>::store(user_data, result) }
}

impl PhysicsWorld {
    /// The closest body that `ray` hits among those `filter` selects, if any.
    ///
    /// Convex shapes are solid: a ray that starts inside one hits it at fraction `0.0` exactly.
    /// Triangle back faces are hit, so a heightfield is hit from below. Rays see a box's sharp
    /// faces whatever its convex radius. [`RayHit::normal`] is the outward normal of the hit
    /// face.
    ///
    /// The origin must be finite with every component at most [`limits::MAX_POSITION`] in
    /// absolute value, the direction finite and not zero, and the filter valid for this world;
    /// otherwise [`QueryError::InvalidValue`] is returned.
    pub fn cast_ray(
        &self,
        ray: RayCast,
        filter: &QueryFilter<'_>,
    ) -> Result<Option<RayHit>, QueryError> {
        if !limits::is_in_frame(ray.origin) {
            return Err(QueryError::InvalidValue(
                "ray origin must be finite and within limits::MAX_POSITION",
            ));
        }
        if !(ray.direction.is_finite() && ray.direction != Vec3::ZERO) {
            return Err(QueryError::InvalidValue(
                "ray direction must be finite and not zero",
            ));
        }
        filter.validate(self)?;
        let origin = ray.origin.to_jph();
        let direction = ray.direction.to_jph();
        // Solid convex shapes and triangle back faces, as Jolt's closest-hit `CastRay` without
        // settings does.
        let settings = JPH_RayCastSettings {
            backFaceModeTriangles: JPH_BackFaceMode_CollideWithBackFaces,
            backFaceModeConvex: JPH_BackFaceMode_IgnoreBackFaces,
            treatConvexAsSolid: true,
        };
        let hit = with_query_filters(self, filter, |raw, state| {
            let slot = ResultSlot::<JPH_RayCastResult>::new(state);
            // SAFETY: the query object lives inside this world's physics system. Jolt's locking
            // narrow-phase query takes body read locks and the broad-phase query lock, and
            // `step` needs `&mut self`, so no update runs meanwhile. `origin`, `direction`,
            // `settings` and `slot` are live locals, `store_ray_hit` matches the slot type, and
            // the filters are live or null (Jolt's accept-all defaults).
            unsafe {
                JPH_NarrowPhaseQuery_CastRay3(
                    self.narrow_phase_query.as_ptr(),
                    &origin,
                    &direction,
                    &settings,
                    JPH_CollisionCollectorType_ClosestHit,
                    Some(store_ray_hit),
                    slot.as_user_data(),
                    null(),
                    raw.object_layer,
                    raw.body,
                    raw.shape,
                )
            };
            slot.hit.get()
        })?;
        let Some(hit) = hit else {
            return Ok(None);
        };
        let body = BodyId::new(hit.bodyID, self.tag);
        let point = ray.point_at(hit.fraction).to_jph();
        let (normal, object_layer) =
            with_read_locked_body(self.body_lock_interface, body, |locked| {
                let mut normal = Vec3::ZERO.to_jph();
                // SAFETY: `locked` is read-locked for the closure; `point` and `normal` are live
                // locals, and the sub-shape id came from a hit on this body.
                unsafe {
                    JPH_Body_GetWorldSpaceSurfaceNormal(
                        locked.as_ptr(),
                        hit.subShapeID2,
                        &point,
                        &mut normal,
                    );
                }
                // SAFETY: as above.
                let layer = unsafe { JPH_Body_GetObjectLayer(locked.as_ptr()) };
                (Vec3::from_jph(normal), ObjectLayer::new(layer))
            })
            .ok_or(QueryError::InvalidValue("the hit body could not be read"))?;
        let sub_shape_id = SubShapeId::new(hit.subShapeID2);
        Ok(Some(RayHit {
            body,
            fraction: hit.fraction,
            sub_shape_id,
            distance: hit.fraction * ray.direction.length(),
            normal,
            object_layer,
            compound_child: self.compound_sub_shape(body, sub_shape_id).ok().flatten(),
        }))
    }

    /// The compound child of `body`'s shape that `id` leads to, as
    /// [`Shape::compound_sub_shape`](crate::Shape::compound_sub_shape) does for the body's
    /// shape: `Ok(None)` when the body's shape is not a compound or `id` names none of its
    /// children.
    pub fn compound_sub_shape(
        &self,
        body: BodyId,
        id: SubShapeId,
    ) -> Result<Option<CompoundSubShape>, BodyError> {
        self.check(body)?;
        // SAFETY: the body interface belongs to this live world. joltc returns the pointer
        // after releasing the temporary `RefConst<Shape>` it got from `BodyInterface::GetShape`,
        // so only the body's own shape reference keeps the shape alive. The body cannot be
        // removed (that needs `&mut self`) and its shape cannot be replaced while `&self` is
        // borrowed. Adding a shape setter that works through `&self` requires revisiting this.
        let shape =
            unsafe { JPH_BodyInterface_GetShape(self.body_interface.as_ptr(), body.to_raw()) };
        let Some(shape) = NonNull::new(shape.cast_mut()) else {
            return Ok(None);
        };
        // SAFETY: `shape` stays alive for the call, as argued above.
        Ok(unsafe { compound_sub_shape_of(shape, id) })
    }

    /// The object layer of a body that a query just reported.
    pub(crate) fn object_layer_of(&self, body: BodyId) -> ObjectLayer {
        // SAFETY: the body interface belongs to this live world; the getter locks the body and
        // accepts any id.
        ObjectLayer::new(unsafe {
            JPH_BodyInterface_GetObjectLayer(self.body_interface.as_ptr(), body.to_raw())
        })
    }
}

/// `base + offset`, for contact points Jolt reports relative to a base offset.
pub(crate) fn offset_from(base: RVec3, offset: JPH_Vec3) -> RVec3 {
    RVec3::new(
        base.x + Real::from(offset.x),
        base.y + Real::from(offset.y),
        base.z + Real::from(offset.z),
    )
}

/// Jolt's rotation-translation matrix for a pose (`RMat44::sRotationTranslation`).
pub(crate) fn rotation_translation(rotation: Quat, position: RVec3) -> JPH_RMat4 {
    let rotation = rotation.to_jph();
    let position = position.to_jph();
    let mut matrix = MaybeUninit::<JPH_RMat4>::uninit();
    // SAFETY: joltc writes the whole matrix; `rotation` and `position` are live locals.
    unsafe {
        JPH_RMat4_RotationTranslation(matrix.as_mut_ptr(), &rotation, &position);
        matrix.assume_init()
    }
}

/// Checks a query shape's pose; Jolt only asserts these.
pub(crate) fn validate_pose(
    shape: &Shape,
    position: RVec3,
    rotation: Quat,
) -> Result<(), QueryError> {
    if !limits::is_in_frame(position) {
        return Err(QueryError::InvalidValue(
            "query shape position must be finite and within limits::MAX_POSITION",
        ));
    }
    if !rotation.is_valid_rotation() {
        return Err(QueryError::InvalidValue(
            "query shape rotation must be a finite unit quaternion",
        ));
    }
    if shape.must_be_static() {
        return Err(QueryError::InvalidValue(
            "heightfields cannot be query shapes",
        ));
    }
    Ok(())
}
