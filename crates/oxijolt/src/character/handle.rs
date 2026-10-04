//! Read and write access to one character.

use std::marker::PhantomData;
use std::ptr::NonNull;

use oxijolt_sys::*;

use super::settings::is_unit;
use super::{CharacterContact, CharacterId, CharacterState, GroundState, INVALID_ID};
use crate::limits::{self, LINEAR_VELOCITY_RULE, POSITION_RULE};
use crate::math::ROTATION_RULE;
use crate::owned::Owned;
use crate::{
    BodyId, CharacterError, CompoundSubShape, ObjectLayer, PhysicsWorld, Quat, RVec3, SubShapeId,
    Vec3,
};

/// Read access to one character, borrowed from its world.
///
/// Each method reads the character's state as its last update, refresh or setter left it.
pub struct CharacterRef<'w> {
    pub(super) world: &'w PhysicsWorld,
    pub(super) id: CharacterId,
    pub(super) character: NonNull<JPH_CharacterVirtual>,
}

impl CharacterRef<'_> {
    fn ptr(&self) -> *mut JPH_CharacterVirtual {
        self.character.as_ptr()
    }

    fn base(&self) -> *mut JPH_CharacterBase {
        self.character.as_ptr().cast()
    }

    /// The character's id.
    pub fn id(&self) -> CharacterId {
        self.id
    }

    /// Position in world space, metres. The shape sits at this position plus the rotated shape
    /// offset plus the padding along up.
    pub fn position(&self) -> RVec3 {
        let mut value = RVec3::ZERO.to_jph();
        // SAFETY: the world borrowed here owns the character, and changing it needs
        // `&mut PhysicsWorld`; the getter reads a member. `value` is a live local.
        unsafe { JPH_CharacterVirtual_GetPosition(self.ptr(), &mut value) };
        RVec3::from_jph(value)
    }

    /// Rotation.
    pub fn rotation(&self) -> Quat {
        let mut value = Quat::IDENTITY.to_jph();
        // SAFETY: as in `position`.
        unsafe { JPH_CharacterVirtual_GetRotation(self.ptr(), &mut value) };
        Quat::from_jph(value)
    }

    /// The up direction, a unit vector.
    pub fn up(&self) -> Vec3 {
        let mut value = Vec3::ZERO.to_jph();
        // SAFETY: as in `position`.
        unsafe { JPH_CharacterBase_GetUp(self.base(), &mut value) };
        Vec3::from_jph(value)
    }

    /// Linear velocity, m/s: what the caller set, as the last update changed it.
    pub fn linear_velocity(&self) -> Vec3 {
        let mut value = Vec3::ZERO.to_jph();
        // SAFETY: as in `position`.
        unsafe { JPH_CharacterVirtual_GetLinearVelocity(self.ptr(), &mut value) };
        Vec3::from_jph(value)
    }

    /// How the character stands.
    pub fn ground_state(&self) -> GroundState {
        // SAFETY: as in `position`.
        GroundState::from_jph(unsafe { JPH_CharacterBase_GetGroundState(self.base()) })
    }

    /// Normal of the ground contact, pointing toward the character; zero when there is none.
    pub fn ground_normal(&self) -> Vec3 {
        let mut value = Vec3::ZERO.to_jph();
        // SAFETY: as in `position`.
        unsafe { JPH_CharacterBase_GetGroundNormal(self.base(), &mut value) };
        Vec3::from_jph(value)
    }

    /// The ground contact point in world space, metres.
    pub fn ground_position(&self) -> RVec3 {
        let mut value = RVec3::ZERO.to_jph();
        // SAFETY: as in `position`.
        unsafe { JPH_CharacterBase_GetGroundPosition(self.base(), &mut value) };
        RVec3::from_jph(value)
    }

    /// Velocity of the ground under the character, m/s.
    pub fn ground_velocity(&self) -> Vec3 {
        let mut value = Vec3::ZERO.to_jph();
        // SAFETY: as in `position`.
        unsafe { JPH_CharacterBase_GetGroundVelocity(self.base(), &mut value) };
        Vec3::from_jph(value)
    }

    /// The body the character stands on, or `None` (in the air, or on another character).
    pub fn ground_body(&self) -> Option<BodyId> {
        // SAFETY: as in `position`.
        let raw = unsafe { JPH_CharacterBase_GetGroundBodyId(self.base()) };
        (raw != INVALID_ID).then(|| BodyId::new(raw, self.world.tag))
    }

    /// The leaf of the ground body's shape the character stands on.
    pub fn ground_sub_shape_id(&self) -> SubShapeId {
        // SAFETY: as in `position`.
        SubShapeId::new(unsafe { JPH_CharacterBase_GetGroundSubShapeId(self.base()) })
    }

    /// The compound child the character stands on, with its user data (a collision group);
    /// `None` when the ground is not a compound child or no longer in the world.
    pub fn ground_compound_child(&self) -> Option<CompoundSubShape> {
        let body = self.ground_body()?;
        self.world
            .compound_sub_shape(body, self.ground_sub_shape_id())
            .ok()
            .flatten()
    }

    /// The contacts after the last update or refresh, in Jolt's order (sorted, which is
    /// deterministic while [`max_hits_exceeded`](Self::max_hits_exceeded) is false).
    pub fn active_contacts(&self) -> Vec<CharacterContact> {
        // SAFETY: as in `position`.
        let count = unsafe { JPH_CharacterVirtual_GetNumActiveContacts(self.ptr()) };
        (0..count)
            .map(|index| {
                // SAFETY: an all-zero `JPH_CharacterContact` is valid: integers, floats,
                // `false`, null pointers and `JPH_MotionType_Static` (0).
                let mut contact: JPH_CharacterContact = unsafe { std::mem::zeroed() };
                // SAFETY: as in `position`; `index < count`, so joltc's `at` stays in range, and
                // `contact` is a live local that joltc overwrites.
                unsafe { JPH_CharacterVirtual_GetActiveContact(self.ptr(), index, &mut contact) };
                CharacterContact::from_jph(&contact, self.world.tag)
            })
            .collect()
    }

    /// The compound child `contact` touched, with its user data (a collision group); `None` for
    /// characters, bodies that are not compounds and bodies no longer in the world.
    pub fn contact_compound_child(&self, contact: &CharacterContact) -> Option<CompoundSubShape> {
        self.world
            .compound_sub_shape(contact.body?, contact.sub_shape_id)
            .ok()
            .flatten()
    }

    /// The object layer of the body `contact` touched; `None` for characters and bodies no
    /// longer in the world.
    pub fn contact_object_layer(&self, contact: &CharacterContact) -> Option<ObjectLayer> {
        let body = contact.body?;
        self.world
            .contains(body)
            .then(|| self.world.object_layer_of(body))
    }

    /// The character's inner body, if it has one.
    pub fn inner_body(&self) -> Option<BodyId> {
        self.world.characters[&self.id.raw].inner_body
    }

    /// Whether the last update found more contacts than
    /// [`CharacterSettings::max_num_hits`](crate::CharacterSettings::max_num_hits). Jolt then drops contacts in an order that is not
    /// guaranteed to be deterministic.
    pub fn max_hits_exceeded(&self) -> bool {
        // SAFETY: as in `position`.
        unsafe { JPH_CharacterVirtual_GetMaxHitsExceeded(self.ptr()) }
    }

    /// The character's persistent state, to restore later with
    /// [`CharacterMut::restore_state`].
    pub fn save_state(&self) -> CharacterState {
        // SAFETY: Jolt is initialised (the world exists). The handle takes over the recorder.
        let recorder = unsafe { Owned::from_raw(JPH_StateRecorder_Create()) }
            .unwrap_or_else(|| unreachable!("`new` does not return null"));
        // SAFETY: as in `position`; `SaveState` is const in Jolt, and the recorder is live and
        // used by this thread only.
        let size = unsafe {
            JPH_CharacterVirtual_SaveState(self.ptr(), recorder.as_ptr());
            JPH_StateRecorder_GetDataSize(recorder.as_ptr())
        };
        let mut jolt = vec![0_u8; size];
        // SAFETY: `jolt` holds exactly `size` writable bytes, which joltc copies at most.
        unsafe { JPH_StateRecorder_CopyData(recorder.as_ptr(), jolt.as_mut_ptr().cast(), size) };
        let up: [f32; 3] = self.up().into();
        CharacterState {
            jolt,
            up: up.map(f32::to_bits),
        }
    }
}

/// Write access to one character, borrowed mutably from its world.
///
/// Read the character through [`PhysicsWorld::character`] once this borrow ends.
pub struct CharacterMut<'w> {
    pub(super) character: NonNull<JPH_CharacterVirtual>,
    pub(super) _world: PhantomData<&'w mut PhysicsWorld>,
}

impl CharacterMut<'_> {
    fn ptr(&self) -> *mut JPH_CharacterVirtual {
        self.character.as_ptr()
    }

    /// Moves the character to `position` (metres, every component at most
    /// [`limits::MAX_POSITION`] in absolute value), without collision checks. Also moves the
    /// inner body.
    pub fn set_position(&mut self, position: RVec3) -> Result<(), CharacterError> {
        if !limits::is_in_frame(position) {
            return Err(CharacterError::InvalidValue(POSITION_RULE));
        }
        self.write_position(position);
        Ok(())
    }

    /// Writes a finite position without the frame bound of [`set_position`](Self::set_position),
    /// for state the world re-expresses ([`PhysicsWorld::rebase`]).
    pub(crate) fn write_position(&mut self, position: RVec3) {
        debug_assert!(position.is_finite());
        let position = position.to_jph();
        // SAFETY: the world is borrowed mutably through this view and owns the character. Jolt
        // moves the inner body through the locking body interface; this thread holds no body
        // lock. `position` is a live local.
        unsafe { JPH_CharacterVirtual_SetPosition(self.ptr(), &position) };
    }

    /// Sets the rotation, a finite unit quaternion. Also rotates the inner body.
    pub fn set_rotation(&mut self, rotation: Quat) -> Result<(), CharacterError> {
        if !rotation.is_valid_rotation() {
            return Err(CharacterError::InvalidValue(ROTATION_RULE));
        }
        let rotation = rotation.to_jph();
        // SAFETY: as in `set_position`.
        unsafe { JPH_CharacterVirtual_SetRotation(self.ptr(), &rotation) };
        Ok(())
    }

    /// Sets the up direction, a finite unit vector, for the next updates.
    pub fn set_up(&mut self, up: Vec3) -> Result<(), CharacterError> {
        if !is_unit(up) {
            return Err(CharacterError::InvalidValue(
                "up must be a finite unit vector",
            ));
        }
        let up = up.to_jph();
        // SAFETY: as in `set_position`; the setter writes a member.
        unsafe { JPH_CharacterBase_SetUp(self.ptr().cast(), &up) };
        Ok(())
    }

    /// Sets the linear velocity, m/s, that the next update moves the character with: finite and
    /// at most [`limits::MAX_LINEAR_VELOCITY`] long (Jolt's own length). Jolt does not clamp a
    /// character's velocity; it becomes the displacement the update casts.
    pub fn set_linear_velocity(&mut self, velocity: Vec3) -> Result<(), CharacterError> {
        if !limits::is_linear_velocity(velocity) {
            return Err(CharacterError::InvalidValue(LINEAR_VELOCITY_RULE));
        }
        self.write_linear_velocity(velocity);
        Ok(())
    }

    /// Writes a finite velocity without the bound of
    /// [`set_linear_velocity`](Self::set_linear_velocity), for state the world re-expresses
    /// ([`PhysicsWorld::rebase`]).
    pub(crate) fn write_linear_velocity(&mut self, velocity: Vec3) {
        debug_assert!(velocity.is_finite());
        let velocity = velocity.to_jph();
        // SAFETY: as in `set_up`.
        unsafe { JPH_CharacterVirtual_SetLinearVelocity(self.ptr(), &velocity) };
    }

    /// Restores a state saved with [`CharacterRef::save_state`]: pose, velocity, up, ground
    /// data and the contacts with collision.
    ///
    /// Any state from `save_state` may be restored into any character, also of another world.
    /// The result is meaningful when the worlds match: the same bodies with the same ids and the
    /// same character id and settings, as in a replay that rebuilds the world the same way. Ids
    /// in the state that do not resolve are harmless: Jolt checks every body id it looks up, and
    /// the readers here tolerate any sub-shape id. The inner body is moved to the restored pose,
    /// as [`set_position`](Self::set_position) moves it.
    pub fn restore_state(&mut self, state: &CharacterState) -> Result<(), CharacterError> {
        // SAFETY: Jolt is initialised (the world exists). The handle takes over the recorder.
        let recorder = unsafe { Owned::from_raw(JPH_StateRecorder_Create()) }
            .unwrap_or_else(|| unreachable!("`new` does not return null"));
        let up = Vec3::from(state.up.map(f32::from_bits)).to_jph();
        // SAFETY: the recorder is live and used by this thread only; `state.jolt` is readable
        // for its length. The bytes are a complete stream written by `SaveState` of this build
        // (`CharacterState` has no other constructor), so `RestoreState` reads exactly what was
        // written. The world is borrowed mutably and owns the character; `up` and `position` are
        // live locals. Jolt's `RestoreState` writes the pose members without moving the inner
        // body; setting the restored position again moves it there through the locking body
        // interface (`CharacterVirtual::SetPosition`), and this thread holds no body lock.
        let failed = unsafe {
            JPH_StateRecorder_WriteBytes(
                recorder.as_ptr(),
                state.jolt.as_ptr().cast(),
                state.jolt.len(),
            );
            JPH_StateRecorder_Rewind(recorder.as_ptr());
            JPH_CharacterVirtual_RestoreState(self.ptr(), recorder.as_ptr());
            JPH_CharacterBase_SetUp(self.ptr().cast(), &up);
            let mut position = RVec3::ZERO.to_jph();
            JPH_CharacterVirtual_GetPosition(self.ptr(), &mut position);
            JPH_CharacterVirtual_SetPosition(self.ptr(), &position);
            JPH_StateRecorder_IsFailed(recorder.as_ptr())
        };
        debug_assert!(!failed, "a saved character state failed to restore");
        if failed {
            return Err(CharacterError::InvalidValue(
                "character state stream failed",
            ));
        }
        Ok(())
    }
}
