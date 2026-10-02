//! Scene queries: ray casts, shape casts and collide-shape queries.
//!
//! Queries take `&PhysicsWorld` and may run on many threads at once while nobody steps the
//! world. They see bodies created, moved and removed through this API immediately, without a
//! step. Every query takes a [`QueryFilter`].
//!
//! Units are metres. Every reported normal is the outward surface normal of the obstacle (the
//! body that was hit) in world space: a floor below the query gives a normal pointing up, a
//! ceiling above it a normal pointing down.

use std::cell::{Cell, RefCell};
use std::ffi::c_void;
use std::fmt;
use std::mem::MaybeUninit;
use std::ptr::{null, NonNull};

use joltphysics_sys::*;

use crate::body::with_read_locked_body;
use crate::filter::{with_query_filters, FilterState};
use crate::math::is_finite_non_negative;
use crate::shape::compound_sub_shape_of;
use crate::{
    BodyError, BodyId, CompoundSubShape, ObjectLayer, PhysicsWorld, Quat, QueryError, QueryFilter,
    RVec3, Real, Shape, SubShapeId, Vec3,
};

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
struct ResultSlot<'s, T> {
    state: &'s FilterState<'s>,
    hit: Cell<Option<T>>,
}

impl<'s, T: Copy> ResultSlot<'s, T> {
    fn new(state: &'s FilterState<'s>) -> Self {
        Self {
            state,
            hit: Cell::new(None),
        }
    }

    fn as_user_data(&self) -> *mut c_void {
        (self as *const Self).cast_mut().cast()
    }

    /// Stores a copy of `*result`.
    ///
    /// # Safety
    /// `user_data` points to a live `ResultSlot<T>` and `result` to a live `T`.
    unsafe fn store(user_data: *mut c_void, result: *const T) {
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
    /// The origin must be finite, the direction finite and not zero, and the filter valid for
    /// this world; otherwise [`QueryError::InvalidValue`] is returned.
    pub fn cast_ray(
        &self,
        ray: RayCast,
        filter: &QueryFilter<'_>,
    ) -> Result<Option<RayHit>, QueryError> {
        if !ray.origin.is_finite() {
            return Err(QueryError::InvalidValue("ray origin must be finite"));
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
}

/// A shape moved from `position` along `direction`, for [`PhysicsWorld::cast_shape`].
///
/// The shape's origin starts at `position` with `rotation` and moves by `direction`, whose
/// length is the cast's length in metres. Hits report the fraction of `direction` covered.
#[derive(Clone, Copy)]
pub struct ShapeCast<'a> {
    shape: &'a Shape,
    position: RVec3,
    rotation: Quat,
    direction: Vec3,
    target_distance: f32,
    return_deepest_point: bool,
    back_faces: bool,
}

impl fmt::Debug for ShapeCast<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ShapeCast")
            .field("position", &self.position)
            .field("rotation", &self.rotation)
            .field("direction", &self.direction)
            .field("target_distance", &self.target_distance)
            .field("return_deepest_point", &self.return_deepest_point)
            .field("back_faces", &self.back_faces)
            .finish_non_exhaustive()
    }
}

impl<'a> ShapeCast<'a> {
    /// Casts `shape`, whose origin starts at `position` with `rotation`, along `direction`
    /// (metres). No target distance, no deepest point, back faces ignored.
    pub fn new(shape: &'a Shape, position: RVec3, rotation: Quat, direction: Vec3) -> Self {
        Self {
            shape,
            position,
            rotation,
            direction,
            target_distance: 0.0,
            return_deepest_point: false,
            back_faces: false,
        }
    }

    /// Stops the cast `metres` before the shape would touch an obstacle, finite and not
    /// negative. Default 0. Only sphere and capsule query shapes support a target distance
    /// above 0: the cast uses a copy of the shape whose radius is grown by `metres`, which
    /// allocates one temporary shape per cast.
    #[must_use]
    pub fn target_distance(mut self, metres: f32) -> Self {
        self.target_distance = metres;
        self
    }

    /// When the shape already overlaps an obstacle at the start, searches for the deepest
    /// penetration so depth and normal describe the real overlap (Jolt's
    /// `ShapeCastSettings::mReturnDeepestPoint`; costs extra time). Default `false`.
    #[must_use]
    pub fn return_deepest_point(mut self, value: bool) -> Self {
        self.return_deepest_point = value;
        self
    }

    /// Also reports back-facing hits: triangles hit from behind and convex obstacles the shape
    /// starts inside and moves out of. Default `false`.
    #[must_use]
    pub fn collide_with_back_faces(mut self, value: bool) -> Self {
        self.back_faces = value;
        self
    }
}

/// The closest hit of a [`PhysicsWorld::cast_shape`].
///
/// `normal` is the outward surface normal of the obstacle in world space: a floor below the
/// cast gives `normal . up > 0`, a ceiling above it `normal . up < 0`.
///
/// `fraction == 0.0` means the shape at its start position already penetrates the obstacle or
/// lies within the target distance of it. Then `penetration_depth > 0` is a real overlap of the
/// shape (without target distance), and `-target_distance < penetration_depth <= 0` means the
/// shape is `-penetration_depth` metres from the obstacle. At `fraction > 0`,
/// `penetration_depth` is `-target_distance`. When the core of the shape (the segment of a
/// capsule, the centre of a sphere) overlaps the obstacle, depth and normal are reliable only
/// with [`ShapeCast::return_deepest_point`].
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct ShapeCastHit {
    /// The body that was hit.
    pub body: BodyId,
    /// Path from the body's shape to the leaf shape that was hit.
    pub sub_shape_id: SubShapeId,
    /// The object layer of the hit body.
    pub object_layer: ObjectLayer,
    /// The compound child that was hit, when the body's shape is a compound.
    pub compound_child: Option<CompoundSubShape>,
    /// Fraction of the cast's direction the shape moves before it stops, in `[0, 1]`.
    pub fraction: f32,
    /// Distance the shape moves before it stops, metres: `fraction` times the direction's
    /// length.
    pub distance: f32,
    /// Contact point on the obstacle in world space.
    pub point: RVec3,
    /// Outward surface normal of the obstacle in world space, unit length, or zero when Jolt
    /// reports no direction (exactly touching cores).
    pub normal: Vec3,
    /// How far the shape overlaps the obstacle at the hit, metres; see [`ShapeCastHit`].
    pub penetration_depth: f32,
}

/// Result callback of [`PhysicsWorld::cast_shape`].
///
/// # Safety
/// Called only by joltc during `cast_shape`, with that call's live
/// `ResultSlot<JPH_ShapeCastResult>` as `user_data` and a live result.
unsafe extern "C" fn store_shape_cast_hit(
    user_data: *mut c_void,
    result: *const JPH_ShapeCastResult,
) {
    // SAFETY: guaranteed by the caller (function contract).
    unsafe { ResultSlot::<JPH_ShapeCastResult>::store(user_data, result) }
}

/// joltc's shape cast settings with Jolt's defaults.
fn default_cast_settings() -> JPH_ShapeCastSettings {
    let mut settings = MaybeUninit::<JPH_ShapeCastSettings>::uninit();
    // SAFETY: joltc writes every field of the settings it is given (it clears them first).
    // A null pointer would select `CollideWithAll` and back faces instead of Jolt's defaults.
    unsafe {
        JPH_ShapeCastSettings_Init(settings.as_mut_ptr());
        settings.assume_init()
    }
}

/// `base + offset`, for contact points Jolt reports relative to a base offset.
fn offset_from(base: RVec3, offset: JPH_Vec3) -> RVec3 {
    RVec3::new(
        base.x + Real::from(offset.x),
        base.y + Real::from(offset.y),
        base.z + Real::from(offset.z),
    )
}

/// Jolt's rotation-translation matrix for a pose (`RMat44::sRotationTranslation`).
fn rotation_translation(rotation: Quat, position: RVec3) -> JPH_RMat4 {
    let rotation = rotation.to_jph();
    let position = position.to_jph();
    let mut matrix = MaybeUninit::<JPH_RMat4>::uninit();
    // SAFETY: joltc writes the whole matrix; `rotation` and `position` are live locals. In
    // single precision `JPH_RMat4` is `JPH_Mat4` and joltc declares the `JPH_RMat4_*` functions
    // only for double precision.
    unsafe {
        #[cfg(feature = "double-precision")]
        JPH_RMat4_RotationTranslation(matrix.as_mut_ptr(), &rotation, &position);
        #[cfg(not(feature = "double-precision"))]
        JPH_Mat4_RotationTranslation(matrix.as_mut_ptr(), &rotation, &position);
        matrix.assume_init()
    }
}

/// Checks a query shape's pose; Jolt only asserts these.
fn validate_pose(shape: &Shape, position: RVec3, rotation: Quat) -> Result<(), QueryError> {
    if !position.is_finite() {
        return Err(QueryError::InvalidValue(
            "query shape position must be finite",
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

impl PhysicsWorld {
    /// The first obstacle among those `filter` selects that `cast`'s shape hits while moving
    /// along its direction, if any.
    ///
    /// The shape's origin moves `fraction * direction`. Of several obstacles the shape overlaps
    /// at the start, the deepest is returned. By default a cast that starts inside a convex
    /// obstacle and moves out of it does not hit it; see [`ShapeCast::collide_with_back_faces`].
    /// See [`ShapeCastHit`] for the normal and depth conventions.
    ///
    /// The position must be finite, the rotation a finite unit quaternion, the direction finite
    /// and not zero, the target distance finite and not negative (and 0 unless the shape is a
    /// sphere or capsule), the shape not a heightfield, and the filter valid for this world;
    /// otherwise [`QueryError::InvalidValue`] is returned.
    pub fn cast_shape(
        &self,
        cast: &ShapeCast<'_>,
        filter: &QueryFilter<'_>,
    ) -> Result<Option<ShapeCastHit>, QueryError> {
        validate_pose(cast.shape, cast.position, cast.rotation)?;
        if !(cast.direction.is_finite() && cast.direction != Vec3::ZERO) {
            return Err(QueryError::InvalidValue(
                "cast direction must be finite and not zero",
            ));
        }
        let target_distance = cast.target_distance;
        if !is_finite_non_negative(target_distance) {
            return Err(QueryError::InvalidValue(
                "target distance must be finite and not negative",
            ));
        }
        let inflated = if target_distance > 0.0 {
            let inflated = cast
                .shape
                .inflated(target_distance)
                .map_err(|_| QueryError::InvalidValue("target distance is too large"))?;
            Some(inflated.ok_or(QueryError::InvalidValue(
                "target distance needs a sphere or capsule query shape",
            ))?)
        } else {
            None
        };
        filter.validate(self)?;
        let query_shape = inflated.as_ref().unwrap_or(cast.shape);

        let mut settings = default_cast_settings();
        settings.returnDeepestPoint = cast.return_deepest_point;
        let back_face_mode = if cast.back_faces {
            JPH_BackFaceMode_CollideWithBackFaces
        } else {
            JPH_BackFaceMode_IgnoreBackFaces
        };
        settings.backFaceModeTriangles = back_face_mode;
        settings.backFaceModeConvex = back_face_mode;

        // joltc's `CastShape2` moves the transform to the shape's centre of mass itself
        // (`RShapeCast::sFromWorldTransform`).
        let transform = rotation_translation(cast.rotation, cast.position);
        let direction = cast.direction.to_jph();
        let mut base_offset = cast.position.to_jph();

        let hit = with_query_filters(self, filter, |raw, state| {
            let slot = ResultSlot::<JPH_ShapeCastResult>::new(state);
            // SAFETY: as in `cast_ray` for the query object and filters. `query_shape` is a live
            // shape kept alive by `cast` or `inflated` until after the call; `transform`,
            // `direction`, `settings`, `base_offset` and `slot` are live locals, and
            // `store_shape_cast_hit` matches the slot type.
            unsafe {
                JPH_NarrowPhaseQuery_CastShape2(
                    self.narrow_phase_query.as_ptr(),
                    query_shape.as_ptr(),
                    &transform,
                    &direction,
                    &settings,
                    &mut base_offset,
                    JPH_CollisionCollectorType_ClosestHit,
                    Some(store_shape_cast_hit),
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
        let body = BodyId::new(hit.bodyID2, self.tag);
        let sub_shape_id = SubShapeId::new(hit.subShapeID2);
        Ok(Some(ShapeCastHit {
            body,
            sub_shape_id,
            object_layer: self.object_layer_of(body),
            compound_child: self.compound_sub_shape(body, sub_shape_id).ok().flatten(),
            fraction: hit.fraction,
            distance: hit.fraction * cast.direction.length(),
            point: offset_from(cast.position, hit.contactPointOn2),
            normal: Vec3::from_jph(hit.penetrationAxis)
                .normalized_or_zero()
                .scale(-1.0),
            penetration_depth: hit.penetrationDepth - target_distance,
        }))
    }

    /// The object layer of a body that a query just reported.
    fn object_layer_of(&self, body: BodyId) -> ObjectLayer {
        // SAFETY: the body interface belongs to this live world; the getter locks the body and
        // accepts any id.
        ObjectLayer::new(unsafe {
            JPH_BodyInterface_GetObjectLayer(self.body_interface.as_ptr(), body.to_raw())
        })
    }
}

/// A shape placed at a pose, for [`PhysicsWorld::collide_shape`].
#[derive(Clone, Copy)]
pub struct CollideShape<'a> {
    shape: &'a Shape,
    position: RVec3,
    rotation: Quat,
    max_separation_distance: f32,
    back_faces: bool,
}

impl fmt::Debug for CollideShape<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("CollideShape")
            .field("position", &self.position)
            .field("rotation", &self.rotation)
            .field("max_separation_distance", &self.max_separation_distance)
            .field("back_faces", &self.back_faces)
            .finish_non_exhaustive()
    }
}

impl<'a> CollideShape<'a> {
    /// Places `shape` with its origin at `position` and `rotation`. Only overlaps are
    /// reported, back faces are ignored.
    pub fn new(shape: &'a Shape, position: RVec3, rotation: Quat) -> Self {
        Self {
            shape,
            position,
            rotation,
            max_separation_distance: 0.0,
            back_faces: false,
        }
    }

    /// Also reports obstacles up to `metres` away from the shape, with a negative
    /// [`CollideShapeHit::penetration_depth`]; finite and not negative. Default 0.
    #[must_use]
    pub fn max_separation_distance(mut self, metres: f32) -> Self {
        self.max_separation_distance = metres;
        self
    }

    /// Also reports triangles the shape touches from behind. Default `false`.
    #[must_use]
    pub fn collide_with_back_faces(mut self, value: bool) -> Self {
        self.back_faces = value;
        self
    }
}

/// One obstacle a [`PhysicsWorld::collide_shape`] found.
///
/// `normal` is the outward surface normal of the obstacle in world space, so the query shape
/// separates from it by moving `normal * penetration_depth`. (A push that uses the query
/// shape's own normal has the opposite sign.)
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct CollideShapeHit {
    /// The body that was hit.
    pub body: BodyId,
    /// Path from the body's shape to the leaf shape that was hit.
    pub sub_shape_id: SubShapeId,
    /// The object layer of the hit body.
    pub object_layer: ObjectLayer,
    /// The compound child that was hit, when the body's shape is a compound.
    pub compound_child: Option<CompoundSubShape>,
    /// Deepest point of the query shape inside the obstacle, in world space.
    pub point_on_shape: RVec3,
    /// Deepest point of the obstacle inside the query shape, in world space.
    pub point_on_body: RVec3,
    /// Outward surface normal of the obstacle in world space, unit length, or zero when Jolt
    /// reports no direction.
    pub normal: Vec3,
    /// How far the shapes overlap, metres: positive for an overlap, negative for a separation
    /// (only reported with [`CollideShape::max_separation_distance`]).
    pub penetration_depth: f32,
}

/// The scalar part of a `JPH_CollideShapeResult`, copied in the result callback.
#[derive(Clone, Copy)]
struct RawCollideHit {
    point_on_1: JPH_Vec3,
    point_on_2: JPH_Vec3,
    axis: JPH_Vec3,
    depth: f32,
    sub_shape_id2: JPH_SubShapeID,
    body: JPH_BodyID,
}

/// Where the result callback of [`PhysicsWorld::collide_shape`] collects every hit.
struct CollideSlot<'s> {
    state: &'s FilterState<'s>,
    hits: RefCell<Vec<RawCollideHit>>,
}

/// Result callback of [`PhysicsWorld::collide_shape`]. It never reads the face arrays, which
/// joltc leaves empty because faces are not requested.
///
/// # Safety
/// Called only by joltc during `collide_shape`, with that call's live `CollideSlot` as
/// `user_data` and a live result.
unsafe extern "C" fn push_collide_hit(
    user_data: *mut c_void,
    result: *const JPH_CollideShapeResult,
) {
    // SAFETY: guaranteed by the caller (function contract); only shared references to the slot
    // exist.
    let slot = unsafe { &*user_data.cast::<CollideSlot<'_>>() };
    // SAFETY: guaranteed by the caller.
    let result = unsafe { &*result };
    let hit = RawCollideHit {
        point_on_1: result.contactPointOn1,
        point_on_2: result.contactPointOn2,
        axis: result.penetrationAxis,
        depth: result.penetrationDepth,
        sub_shape_id2: result.subShapeID2,
        body: result.bodyID2,
    };
    slot.state.guarded((), || slot.hits.borrow_mut().push(hit));
}

/// joltc's collide settings with Jolt's defaults.
fn default_collide_settings() -> JPH_CollideShapeSettings {
    let mut settings = MaybeUninit::<JPH_CollideShapeSettings>::uninit();
    // SAFETY: joltc writes every field of the settings it is given (it clears them first).
    // A null pointer would select `CollideWithAll` instead of Jolt's defaults.
    unsafe {
        JPH_CollideShapeSettings_Init(settings.as_mut_ptr());
        settings.assume_init()
    }
}

impl PhysicsWorld {
    /// Every obstacle among those `filter` selects that `query`'s shape overlaps (or, with a
    /// maximum separation distance, comes close to).
    ///
    /// See [`CollideShapeHit`] for the normal and depth conventions. Hits come in Jolt's
    /// traversal order, which is the same for the same call history but not sorted.
    ///
    /// The position must be finite, the rotation a finite unit quaternion, the maximum
    /// separation distance finite and not negative, the shape not a heightfield, and the filter
    /// valid for this world; otherwise [`QueryError::InvalidValue`] is returned.
    pub fn collide_shape(
        &self,
        query: &CollideShape<'_>,
        filter: &QueryFilter<'_>,
    ) -> Result<Vec<CollideShapeHit>, QueryError> {
        validate_pose(query.shape, query.position, query.rotation)?;
        if !is_finite_non_negative(query.max_separation_distance) {
            return Err(QueryError::InvalidValue(
                "max separation distance must be finite and not negative",
            ));
        }
        filter.validate(self)?;

        let mut settings = default_collide_settings();
        // joltc's all-hit collector copies collected faces into arrays it never frees, so faces
        // must never be requested.
        debug_assert_eq!(settings.base.collectFacesMode, JPH_CollectFacesMode_NoFaces);
        settings.maxSeparationDistance = query.max_separation_distance;
        settings.backFaceMode = if query.back_faces {
            JPH_BackFaceMode_CollideWithBackFaces
        } else {
            JPH_BackFaceMode_IgnoreBackFaces
        };

        let center_of_mass = query.rotation.rotate(query.shape.center_of_mass());
        let transform = rotation_translation(
            query.rotation,
            offset_from(query.position, center_of_mass.to_jph()),
        );
        let scale = Vec3::new(1.0, 1.0, 1.0).to_jph();
        let mut base_offset = query.position.to_jph();

        let hits = with_query_filters(self, filter, |raw, state| {
            let slot = CollideSlot {
                state,
                hits: RefCell::new(Vec::new()),
            };
            // SAFETY: as in `cast_ray` for the query object and filters. The shape is live
            // (borrowed by `query`); `scale`, `transform`, `settings`, `base_offset` and `slot`
            // are live locals (joltc dereferences scale and base offset without checking), and
            // `push_collide_hit` matches the slot type.
            unsafe {
                JPH_NarrowPhaseQuery_CollideShape2(
                    self.narrow_phase_query.as_ptr(),
                    query.shape.as_ptr(),
                    &scale,
                    &transform,
                    &settings,
                    &mut base_offset,
                    JPH_CollisionCollectorType_AllHit,
                    Some(push_collide_hit),
                    (&slot as *const CollideSlot<'_>).cast_mut().cast(),
                    null(),
                    raw.object_layer,
                    raw.body,
                    raw.shape,
                )
            };
            slot.hits.into_inner()
        })?;
        Ok(hits
            .into_iter()
            .map(|hit| {
                let body = BodyId::new(hit.body, self.tag);
                let sub_shape_id = SubShapeId::new(hit.sub_shape_id2);
                CollideShapeHit {
                    body,
                    sub_shape_id,
                    object_layer: self.object_layer_of(body),
                    compound_child: self.compound_sub_shape(body, sub_shape_id).ok().flatten(),
                    point_on_shape: offset_from(query.position, hit.point_on_1),
                    point_on_body: offset_from(query.position, hit.point_on_2),
                    normal: Vec3::from_jph(hit.axis).normalized_or_zero().scale(-1.0),
                    penetration_depth: hit.depth,
                }
            })
            .collect())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn joltc_settings_init_gives_jolts_defaults() {
        let collide = default_collide_settings();
        let cast = default_cast_settings();
        for base in [collide.base, cast.base] {
            assert_eq!(base.collectFacesMode, JPH_CollectFacesMode_NoFaces);
            assert_eq!(
                base.activeEdgeMode,
                JPH_ActiveEdgeMode_CollideOnlyWithActive
            );
            assert_eq!(base.collisionTolerance, 1.0e-4);
            assert_eq!(base.penetrationTolerance, 1.0e-4);
        }
        assert_eq!(collide.backFaceMode, JPH_BackFaceMode_IgnoreBackFaces);
        assert_eq!(collide.maxSeparationDistance, 0.0);
        assert_eq!(cast.backFaceModeTriangles, JPH_BackFaceMode_IgnoreBackFaces);
        assert_eq!(cast.backFaceModeConvex, JPH_BackFaceMode_IgnoreBackFaces);
        assert!(!cast.useShrunkenShapeAndConvexRadius);
        assert!(!cast.returnDeepestPoint);
    }
}
