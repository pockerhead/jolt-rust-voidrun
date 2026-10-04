//! Creating, removing and finding bodies, and broad-phase lookups.

use std::any::Any;
use std::ffi::c_void;
use std::marker::PhantomData;
use std::panic::{catch_unwind, resume_unwind, AssertUnwindSafe};
use std::ptr::{null, NonNull};

use oxijolt_sys::*;

use super::{
    has_finite_inverse, mass_properties, with_locked_body, BodyId, BodyMut, BodyRef, BodySettings,
    CreationSettings, MotionType, INERTIA_RULE, INVALID_BODY_ID,
};
use crate::limits::{is_mass, MASS_RULE};
use crate::owned::Owned;
use crate::{BodyError, PhysicsWorld, Shape, Vec3};

impl PhysicsWorld {
    /// Creates a body from `shape` and adds it to the world. The body keeps its own reference
    /// to the shape, so `shape` may be dropped afterwards.
    ///
    /// Fails with [`BodyError::InvalidValue`] when a setting is out of range, when a dynamic or
    /// kinematic body uses a shape that only static bodies may use (a heightfield, or a
    /// compound that contains one), when a dynamic or kinematic body's mass or inertia
    /// (overridden, or computed from a tiny shape) has no finite inverse, when its inertia
    /// tensor is not diagonal (a rotated or offset compound child, an offset centre of mass) and
    /// too badly conditioned for Jolt to decompose, such as a slender shape in a rotated child
    /// (see the rigid body inertia rule in [`limits`](crate::limits)), and when a dynamic body's mass
    /// (overridden or computed) is outside [`limits::MIN_MASS`](crate::limits::MIN_MASS)`..=`[`limits::MAX_MASS`](crate::limits::MAX_MASS).
    /// Kinematic bodies are exempt from the mass range: Jolt gives them infinite mass in the
    /// solver.
    pub fn create_body(
        &mut self,
        shape: &Shape,
        settings: &BodySettings,
    ) -> Result<BodyId, BodyError> {
        settings.validate(self.object_layer_count)?;
        // Jolt itself never checks this when creating a body.
        if settings.motion_type != MotionType::Static
            // SAFETY: `shape` is live for the call; the getter only reads it.
            && unsafe { JPH_Shape_MustBeStatic(shape.as_ptr()) }
        {
            return Err(BodyError::InvalidValue(
                "this shape can only be used by static bodies",
            ));
        }
        // Jolt computes mass properties for every body that is not static
        // (`BodyCreationSettings::HasMassProperties`).
        if settings.motion_type != MotionType::Static {
            let properties = mass_properties(shape, settings.mass);
            if !has_finite_inverse(&properties) {
                return Err(BodyError::InvalidValue(INERTIA_RULE));
            }
            if settings.motion_type == MotionType::Dynamic && !is_mass(properties.mass) {
                return Err(BodyError::InvalidValue(MASS_RULE));
            }
        }
        let creation = CreationSettings::new(shape, settings)?;
        if !self.has_room_for_bodies(1) {
            return Err(BodyError::TooManyBodies);
        }
        self.note_structure_change();
        // SAFETY: the body interface belongs to this live world, borrowed mutably; `creation`
        // is a fully set up settings object whose layer exists in this world.
        let raw = unsafe {
            JPH_BodyInterface_CreateAndAddBody(
                self.body_interface.as_ptr(),
                creation.as_ptr(),
                settings.activation.to_jph(),
            )
        };
        if raw == INVALID_BODY_ID {
            return Err(BodyError::TooManyBodies);
        }
        Ok(BodyId::new(raw, self.tag))
    }

    /// `Ok` if `id` names a body that is in this world now.
    pub(crate) fn check(&self, id: BodyId) -> Result<(), BodyError> {
        if id.world != self.tag {
            return Err(BodyError::WrongWorld(id));
        }
        // SAFETY: the body interface belongs to this live world. `IsAdded` locks the body and
        // compares index and sequence number, so any id value is safe to pass.
        if unsafe { JPH_BodyInterface_IsAdded(self.body_interface.as_ptr(), id.raw) } {
            Ok(())
        } else {
            Err(BodyError::NotFound(id))
        }
    }

    /// Whether `id` names a body that is in this world now.
    pub fn contains(&self, id: BodyId) -> bool {
        self.check(id).is_ok()
    }

    /// Read access to a body.
    pub fn body(&self, id: BodyId) -> Result<BodyRef<'_>, BodyError> {
        self.check(id)?;
        Ok(BodyRef {
            body_interface: self.body_interface,
            body_lock_interface: self.body_lock_interface,
            id,
            _world: PhantomData,
        })
    }

    /// Read and write access to a body.
    pub fn body_mut(&mut self, id: BodyId) -> Result<BodyMut<'_>, BodyError> {
        self.check(id)?;
        Ok(BodyMut {
            inner: BodyRef {
                body_interface: self.body_interface,
                body_lock_interface: self.body_lock_interface,
                id,
                _world: PhantomData,
            },
            _world: PhantomData,
        })
    }

    /// Removes a body from the world and destroys it; soft bodies are removed the same way.
    ///
    /// Jolt does not wake the bodies around a removed one by itself, so oxijolt wakes every
    /// non-static body whose current bounds overlap (or touch) the removed body's bounds, in
    /// body-id order. The woken set depends only on body poses, not on broad-phase maintenance or
    /// worker threads; a stack whose bottom is removed falls.
    ///
    /// The inner body of a character cannot be removed this way
    /// ([`BodyError::OwnedByCharacter`]); it goes with
    /// [`remove_character`](Self::remove_character). The chassis of a vehicle cannot be removed
    /// while the vehicle exists ([`BodyError::UsedByVehicle`]); remove the vehicle first with
    /// [`remove_vehicle`](Self::remove_vehicle). A part of a ragdoll goes only with its ragdoll
    /// ([`BodyError::OwnedByRagdoll`], [`remove_ragdoll`](Self::remove_ragdoll)). A body that a
    /// constraint uses cannot be removed while the constraint exists
    /// ([`BodyError::UsedByConstraint`]); remove the constraint first with
    /// [`remove_constraint`](Self::remove_constraint).
    pub fn remove_body(&mut self, id: BodyId) -> Result<(), BodyError> {
        self.check(id)?;
        // The character's destructor destroys its inner body, and Jolt does not validate ids in
        // `DestroyBody`: removing it here first would make that a double destroy. Any future
        // API that destroys bodies needs the same check.
        if self.is_inner_body(id) {
            return Err(BodyError::OwnedByCharacter(id));
        }
        // A vehicle keeps a pointer to its chassis and dereferences it on every step. Any future
        // API that destroys bodies or changes their motion type must consult the vehicle bodies
        // the same way.
        if self.is_vehicle_body(id) {
            return Err(BodyError::UsedByVehicle(id));
        }
        // A ragdoll destroys its parts when it is released, and Jolt does not validate ids in
        // `DestroyBody`, so removing a part here would make that a double destroy.
        if self.is_ragdoll_body(id) {
            return Err(BodyError::OwnedByRagdoll(id));
        }
        // A constraint keeps pointers to its bodies and dereferences them on every step. Any
        // future API that destroys bodies must consult the constraint bodies the same way.
        if self.is_constraint_body(id) {
            return Err(BodyError::UsedByConstraint(id));
        }
        let mut bounds = JPH_AABox {
            min: Vec3::ZERO.to_jph(),
            max: Vec3::ZERO.to_jph(),
        };
        with_locked_body(self.body_lock_interface, id, |body| {
            // SAFETY: `body` is locked for writing for the duration of the closure; `bounds` is
            // a live local.
            unsafe { JPH_Body_GetWorldSpaceBounds(body.as_ptr(), &mut bounds) };
        })
        .ok_or(BodyError::NotFound(id))?;
        self.note_structure_change();
        // SAFETY: `check` just confirmed the id names a body in this world, and `&mut self`
        // keeps anyone else from removing it in between, so the body is removed exactly once
        // (Jolt does not validate ids in `DestroyBody`). This thread holds no body lock.
        unsafe { JPH_BodyInterface_RemoveAndDestroyBody(self.body_interface.as_ptr(), id.raw) };
        self.wake_bodies_overlapping(&bounds);
        Ok(())
    }

    /// Whether bodies `a` and `b` touched in the last [`step`](Self::step) (Jolt
    /// `PhysicsSystem::WereBodiesInContact`).
    ///
    /// Jolt answers from the contact cache of the last step, and only for pairs of which at
    /// least one body was awake in it; bodies removed since are allowed. A soft body's
    /// vertices collide inside the soft body solver and never enter that cache, so this is
    /// `false` for every pair with a soft body, even one lying on the other body. Fails with
    /// [`BodyError::WrongWorld`] for an id of another world.
    pub fn were_bodies_in_contact(&self, a: BodyId, b: BodyId) -> Result<bool, BodyError> {
        for id in [a, b] {
            if id.world != self.tag {
                return Err(BodyError::WrongWorld(id));
            }
        }
        // SAFETY: the system is live and no step runs (`step` needs `&mut self`); Jolt looks the
        // pair up in its contact cache and never dereferences a body, so any id is safe.
        Ok(unsafe { JPH_PhysicsSystem_WereBodiesInContact(self.system.as_ptr(), a.raw, b.raw) })
    }

    /// Wakes every non-static body whose world bounds overlap `bounds`, in body-id order.
    ///
    /// Jolt's broad phase keeps widened bounds for moved bodies until its next maintenance, so
    /// which bodies it reports depends on that history (Jolt docs, "Deterministic Simulation").
    /// Here it only proposes candidates; each is kept only when its exact bounds overlap.
    pub(crate) fn wake_bodies_overlapping(&mut self, bounds: &JPH_AABox) {
        let candidates = self.broad_phase_bodies(bounds);
        let woken = self.overlapping_movable_bodies(bounds, &candidates);
        if woken.is_empty() {
            return;
        }
        // SAFETY: the body interface belongs to this live world, borrowed mutably; `woken` holds
        // `woken.len()` ids and lives for the call. This thread holds no body lock:
        // `overlapping_movable_bodies` has released its locks, which `ActivateBodies` takes
        // again (Jolt's body mutexes are not recursive).
        unsafe {
            JPH_BodyInterface_ActivateBodies(
                self.body_interface.as_ptr(),
                woken.as_ptr(),
                woken.len() as u32,
            );
        }
    }

    /// The ids of the bodies whose broad-phase bounds overlap `bounds`, sorted and without
    /// duplicates. The broad phase may keep widened bounds, so callers check exact bounds.
    pub(crate) fn broad_phase_bodies(&self, bounds: &JPH_AABox) -> Vec<JPH_BodyID> {
        let mut hits = BroadPhaseHits {
            ids: Vec::with_capacity(self.body_count() as usize),
            panic: None,
        };
        // SAFETY: the broad-phase query belongs to this live world; a step needs
        // `&mut PhysicsWorld`, so none runs during this `&self` call. `bounds` is live for the
        // call. `hits` is a live local that only the collector touches during the call, as the
        // `BroadPhaseHits` it expects. Null filters select joltc's accept-all defaults.
        unsafe {
            JPH_BroadPhaseQuery_CollideAABox(
                self.broad_phase_query.as_ptr(),
                bounds,
                Some(collect_broad_phase_hit),
                (&mut hits as *mut BroadPhaseHits).cast(),
                null(),
                null(),
            );
        }
        if let Some(payload) = hits.panic {
            resume_unwind(payload);
        }
        let mut candidates = hits.ids;
        // Jolt's `BodyID::operator<` compares the raw value.
        candidates.sort_unstable();
        candidates.dedup();
        candidates
    }

    /// Those of the sorted `candidates` that are non-static bodies whose world bounds overlap
    /// `bounds`, in the same order. Locks all candidates at once and releases them before
    /// returning.
    fn overlapping_movable_bodies(
        &self,
        bounds: &JPH_AABox,
        candidates: &[JPH_BodyID],
    ) -> Vec<JPH_BodyID> {
        let mut woken = Vec::new();
        if candidates.is_empty() {
            return woken;
        }
        // SAFETY: the lock interface belongs to this live world. joltc copies the ids into the
        // lock object, so `candidates` only has to live for the call. The handle takes over the
        // lock and releases it when it goes out of scope.
        let lock = unsafe {
            Owned::from_raw(JPH_BodyLockInterface_LockMultiWrite(
                self.body_lock_interface.as_ptr(),
                candidates.as_ptr(),
                candidates.len() as u32,
            ))
        };
        let Some(lock) = lock else {
            return woken;
        };
        for (index, &candidate) in candidates.iter().enumerate() {
            // SAFETY: `lock` is live and holds `candidates.len()` ids, so `index` is in range.
            // Jolt returns null unless the id still names a live body.
            let body = unsafe { JPH_BodyLockMultiWrite_GetBody(lock.as_ptr(), index as u32) };
            let Some(body) = NonNull::new(body) else {
                continue;
            };
            let mut candidate_bounds = JPH_AABox {
                min: Vec3::ZERO.to_jph(),
                max: Vec3::ZERO.to_jph(),
            };
            // SAFETY: `body` is locked for writing while `lock` lives; the getters only read
            // it, and `candidate_bounds` is a live local.
            let is_static = unsafe {
                JPH_Body_GetWorldSpaceBounds(body.as_ptr(), &mut candidate_bounds);
                JPH_Body_IsStatic(body.as_ptr())
            };
            if !is_static && bounds_overlap(bounds, &candidate_bounds) {
                woken.push(candidate);
            }
        }
        woken
    }
}

/// What the broad-phase collector of `PhysicsWorld::broad_phase_bodies` gathers.
struct BroadPhaseHits {
    /// Candidate ids, at most as many as the capacity reserved before the query.
    ids: Vec<JPH_BodyID>,
    /// The payload of a panic caught in the collector, re-raised after the query.
    panic: Option<Box<dyn Any + Send>>,
}

/// Jolt's `CollisionCollectorTraitsCollideShape::InitialEarlyOutFraction`: keep collecting.
const KEEP_COLLECTING: f32 = f32::MAX;
/// Jolt's `CollisionCollectorTraitsCollideShape::ShouldEarlyOutFraction`: stop the query.
const STOP_COLLECTING: f32 = -f32::MAX;

/// Broad-phase collector of `PhysicsWorld::broad_phase_bodies`: records each candidate id
/// without allocating and never lets a panic unwind into joltc.
///
/// # Safety
/// Called only by joltc during the `JPH_BroadPhaseQuery_CollideAABox` call of
/// `broad_phase_bodies`, with that call's live `*mut BroadPhaseHits` as `user_data`, which
/// nothing else accesses during the call.
unsafe extern "C" fn collect_broad_phase_hit(user_data: *mut c_void, body: JPH_BodyID) -> f32 {
    // SAFETY: guaranteed by the caller (function contract); this is the only reference to the
    // hits during the call.
    let hits = unsafe { &mut *user_data.cast::<BroadPhaseHits>() };
    let ids = &mut hits.ids;
    let collected = catch_unwind(AssertUnwindSafe(|| {
        if ids.len() < ids.capacity() {
            ids.push(body);
        }
    }));
    match collected {
        Ok(()) => KEEP_COLLECTING,
        Err(payload) => {
            hits.panic = Some(payload);
            STOP_COLLECTING
        }
    }
}

/// Whether two boxes overlap, touching included (Jolt `AABox::Overlaps`).
pub(super) fn bounds_overlap(a: &JPH_AABox, b: &JPH_AABox) -> bool {
    let axis = |a_min: f32, a_max: f32, b_min: f32, b_max: f32| a_min <= b_max && b_min <= a_max;
    axis(a.min.x, a.max.x, b.min.x, b.max.x)
        && axis(a.min.y, a.max.y, b.min.y, b.max.y)
        && axis(a.min.z, a.max.z, b.min.z, b.max.z)
}
