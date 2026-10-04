//! Collide-shape queries: every obstacle a shape at a pose overlaps.

use std::cell::RefCell;
use std::ffi::c_void;
use std::fmt;
use std::mem::MaybeUninit;
use std::ptr::null;

use oxijolt_sys::*;

use super::{offset_from, rotation_translation, validate_pose};
use crate::filter::{with_query_filters, FilterState};
use crate::limits;
use crate::{
    BodyId, CompoundSubShape, ObjectLayer, PhysicsWorld, Quat, QueryError, QueryFilter, RVec3,
    Shape, SubShapeId, Vec3,
};

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
    /// See [`CollideShapeHit`] for the normal and depth conventions. Hits come in an unspecified
    /// order: Jolt promises consistent narrow-phase results but not the order they arrive in
    /// (Jolt docs, "Deterministic Simulation"). Sort them (for example by body id and sub-shape
    /// id) when order matters.
    ///
    /// The position must be finite and within [`limits::MAX_POSITION`], the rotation a finite unit
    /// quaternion, the maximum separation distance between 0 and [`limits::MAX_SHAPE_EXTENT`], the
    /// shape not a heightfield, and the filter valid for this world; otherwise
    /// [`QueryError::InvalidValue`] is returned.
    pub fn collide_shape(
        &self,
        query: &CollideShape<'_>,
        filter: &QueryFilter<'_>,
    ) -> Result<Vec<CollideShapeHit>, QueryError> {
        validate_pose(query.shape, query.position, query.rotation)?;
        if !limits::is_local_distance(query.max_separation_distance) {
            return Err(QueryError::InvalidValue(
                "max separation distance must be between 0 and limits::MAX_SHAPE_EXTENT",
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
    fn joltc_collide_settings_init_gives_jolts_defaults() {
        let settings = default_collide_settings();
        assert_eq!(settings.base.collectFacesMode, JPH_CollectFacesMode_NoFaces);
        assert_eq!(
            settings.base.activeEdgeMode,
            JPH_ActiveEdgeMode_CollideOnlyWithActive
        );
        assert_eq!(settings.base.collisionTolerance, 1.0e-4);
        assert_eq!(settings.base.penetrationTolerance, 1.0e-4);
        assert_eq!(settings.backFaceMode, JPH_BackFaceMode_IgnoreBackFaces);
        assert_eq!(settings.maxSeparationDistance, 0.0);
    }
}
