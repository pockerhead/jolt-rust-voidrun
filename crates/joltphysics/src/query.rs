//! Scene queries.
//!
//! Queries take `&PhysicsWorld`, see bodies as soon as they are created (no step is needed),
//! and may run on many threads at once while nobody steps the world.

use std::ptr::{null, NonNull};

use joltphysics_sys::*;

use crate::shape::compound_sub_shape_of;
use crate::{
    BodyError, BodyId, CompoundSubShape, PhysicsWorld, QueryError, RVec3, Real, SubShapeId, Vec3,
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
/// There is no surface normal yet. Convex shapes are solid: a ray that starts inside one hits
/// it at fraction 0. Triangle back faces are hit, so a heightfield is hit from below too, as
/// Jolt documents for this query (`NarrowPhaseQuery.h`). No filters apply: every body in every
/// layer is considered. To read the user data of the compound child that was hit, pass `body`
/// and `sub_shape_id` to [`PhysicsWorld::compound_sub_shape`].
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct RayHit {
    /// The body that was hit.
    pub body: BodyId,
    /// Fraction along the ray's direction where it hit, in `[0, 1]`.
    pub fraction: f32,
    /// Path from the body's shape to the leaf shape that was hit.
    pub sub_shape_id: SubShapeId,
}

impl PhysicsWorld {
    /// The closest body that `ray` hits, if any.
    ///
    /// The origin must be finite and the direction finite and not zero; otherwise
    /// [`QueryError::InvalidValue`] is returned.
    pub fn cast_ray(&self, ray: RayCast) -> Result<Option<RayHit>, QueryError> {
        if !ray.origin.is_finite() {
            return Err(QueryError::InvalidValue("ray origin must be finite"));
        }
        if !(ray.direction.is_finite() && ray.direction != Vec3::ZERO) {
            return Err(QueryError::InvalidValue(
                "ray direction must be finite and not zero",
            ));
        }
        let origin = ray.origin.to_jph();
        let direction = ray.direction.to_jph();
        let mut hit = JPH_RayCastResult {
            bodyID: 0,
            fraction: 0.0,
            subShapeID2: 0,
        };
        // SAFETY: the query object lives inside this world's physics system. Jolt's locking
        // narrow-phase query takes body read locks and the broad-phase query lock, and `step`
        // needs `&mut self`, so no update runs meanwhile. `origin`, `direction` and `hit` are
        // live locals; null filters select Jolt's accept-all defaults.
        let has_hit = unsafe {
            JPH_NarrowPhaseQuery_CastRay(
                self.narrow_phase_query.as_ptr(),
                &origin,
                &direction,
                &mut hit,
                null(),
                null(),
                null(),
            )
        };
        Ok(has_hit.then(|| RayHit {
            body: BodyId::new(hit.bodyID, self.tag),
            fraction: hit.fraction,
            sub_shape_id: SubShapeId::new(hit.subShapeID2),
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
