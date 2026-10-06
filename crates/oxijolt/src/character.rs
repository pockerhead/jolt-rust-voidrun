//! Virtual characters: Jolt's `CharacterVirtual`, owned by a [`PhysicsWorld`].
//!
//! A character is not a body. It is a shape that the world moves with collision queries in
//! [`PhysicsWorld::update_character`], sliding along what it hits, stepping up stairs and
//! sticking to the floor (Jolt `CharacterVirtual::ExtendedUpdate`). It can optionally carry a
//! kinematic inner body so that bodies and queries see it.
//!
//! Reading a character ([`PhysicsWorld::character`]) takes `&PhysicsWorld`; creating, updating,
//! moving, restoring and removing one take `&mut PhysicsWorld`. Every contact normal a
//! character reports points toward the character: a floor gives a normal along its up, a
//! ceiling one against it.
//!
//! [`PhysicsWorld`]: crate::PhysicsWorld
//! [`PhysicsWorld::update_character`]: crate::PhysicsWorld::update_character
//! [`PhysicsWorld::character`]: crate::PhysicsWorld::character

use std::fmt;

use oxijolt_sys::*;

use crate::body::MotionType;
use crate::owned::{JoltObject, Owned};
use crate::world::WorldTag;
use crate::{BodyId, ObjectLayer, RVec3, Shape, SubShapeId, Vec3};

mod handle;
mod listener;
mod settings;
mod update;

pub use handle::{CharacterMut, CharacterRef};
pub use listener::{
    BodyVelocity, CharacterContactKey, CharacterContactListener, CharacterContactSettings,
};
pub use settings::{CharacterSettings, ExtendedUpdateSettings};

/// What the shape of a character's inner body must satisfy: a finite inverse mass and inertia
/// ([`has_finite_inverse`](crate::body::has_finite_inverse)), and, for an inertia that is not
/// diagonal, the rigid body inertia floor of [`limits`](crate::limits).
pub(crate) const INNER_BODY_INERTIA_RULE: &str =
    "inner body shape must give a finite inverse mass and a well-conditioned inertia";

/// Jolt's invalid `BodyID` and `CharacterID` value.
const INVALID_ID: u32 = 0xffff_ffff;

/// Identifies a character in the world that created it.
///
/// The raw value is the Jolt `CharacterID` the world gave the character: 1 for the first
/// character of a world, then 2, 3 and so on. Ids are never reused within a world, so the same
/// creation history gives the same ids. Jolt orders contacts between characters by these ids.
///
/// A `CharacterId` also remembers its world: using it with another world returns
/// [`CharacterError::WrongWorld`](crate::CharacterError::WrongWorld).
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct CharacterId {
    // Declared first, so ids order by Jolt's id before the world.
    raw: u32,
    pub(crate) world: WorldTag,
}

impl CharacterId {
    pub(crate) fn new(raw: u32, world: WorldTag) -> Self {
        Self { raw, world }
    }

    /// The id as Jolt stores it (`CharacterID::GetValue`).
    pub fn to_raw(self) -> u32 {
        self.raw
    }
}

impl fmt::Debug for CharacterId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("CharacterId").field(&self.raw).finish()
    }
}

/// How a character stands, from its last update (Jolt `CharacterBase::EGroundState`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum GroundState {
    /// On ground that is not steeper than the maximum slope angle.
    OnGround,
    /// On ground steeper than the maximum slope angle; the character slides down.
    OnSteepGround,
    /// Touching something below that does not hold it up, for example the edge of a step it is
    /// not standing on; the character falls.
    NotSupported,
    /// Touching nothing below.
    InAir,
}

impl GroundState {
    fn from_jph(value: JPH_GroundState) -> Self {
        [
            (JPH_GroundState_OnGround, Self::OnGround),
            (JPH_GroundState_OnSteepGround, Self::OnSteepGround),
            (JPH_GroundState_NotSupported, Self::NotSupported),
            (JPH_GroundState_InAir, Self::InAir),
        ]
        .into_iter()
        .find_map(|(raw, state)| (raw == value).then_some(state))
        // Jolt has exactly these four, asserted equal to joltc's in
        // `native/layout_checks.cpp`. This runs in Rust after the call returned, so a panic
        // here never unwinds into C++.
        .unwrap_or_else(|| unreachable!("Jolt returned unknown ground state {value}"))
    }

    /// Whether the ground holds the character: [`OnGround`](Self::OnGround) or
    /// [`OnSteepGround`](Self::OnSteepGround) (Jolt `CharacterBase::IsSupported`).
    pub fn is_supported(self) -> bool {
        matches!(self, Self::OnGround | Self::OnSteepGround)
    }
}

/// The optional kinematic body a character carries, so that bodies collide with it and queries
/// find it (Jolt `CharacterVirtualSettings::mInnerBodyShape`).
#[derive(Clone, Copy)]
pub struct InnerBody<'a> {
    /// The body's shape, which must suit a kinematic body: no heightfield, a mass and inertia
    /// Jolt can invert, and an inertia that meets the rigid body inertia floor of [`limits`]
    /// when it is not diagonal.
    ///
    /// [`limits`]: crate::limits
    pub shape: &'a Shape,
    /// The body's object layer, which must exist in the world.
    pub object_layer: ObjectLayer,
}

impl fmt::Debug for InnerBody<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("InnerBody")
            .field("object_layer", &self.object_layer)
            .finish_non_exhaustive()
    }
}

/// A contact of a character after its last update or contact refresh (Jolt
/// `CharacterVirtual::Contact`).
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct CharacterContact {
    /// The body touched, or `None` for another character.
    pub body: Option<BodyId>,
    /// The character touched, or `None` for a body.
    pub character: Option<CharacterId>,
    /// The leaf of the touched body's shape (a compound child, a heightfield triangle); see
    /// [`CharacterRef::contact_compound_child`].
    pub sub_shape_id: SubShapeId,
    /// The contact point in world space, metres.
    pub position: RVec3,
    /// The contact normal, pointing toward the character: along up for a floor, against up for
    /// a ceiling.
    pub contact_normal: Vec3,
    /// The surface normal of what was touched, pointing toward the character. Equal to the
    /// contact normal when that points further up (Jolt replaces it then).
    pub surface_normal: Vec3,
    /// Distance to the contact, metres: at most 0 is touching, positive is a predictive
    /// contact.
    pub distance: f32,
    /// Fraction of the update's movement after which the contact was hit.
    pub fraction: f32,
    /// Velocity of the contact point, m/s.
    pub linear_velocity: Vec3,
    /// How the touched body moves; a character counts as kinematic.
    pub motion_type: MotionType,
    /// Whether the touched body is a sensor.
    pub is_sensor: bool,
    /// Whether the character collided with it in the last update, not just came near.
    pub had_collision: bool,
    /// Whether the character ignored it (Jolt discards contacts it slides past).
    pub was_discarded: bool,
    /// Whether the character touched a back face of a triangle.
    pub is_back_facing: bool,
}

impl CharacterContact {
    fn from_jph(contact: &JPH_CharacterContact, world: WorldTag) -> Self {
        Self {
            body: (contact.bodyB != INVALID_ID).then(|| BodyId::new(contact.bodyB, world)),
            character: (contact.characterIDB != INVALID_ID)
                .then(|| CharacterId::new(contact.characterIDB, world)),
            sub_shape_id: SubShapeId::new(contact.subShapeIDB),
            position: RVec3::from_jph(contact.position),
            contact_normal: Vec3::from_jph(contact.contactNormal),
            surface_normal: Vec3::from_jph(contact.surfaceNormal),
            distance: contact.distance,
            fraction: contact.fraction,
            linear_velocity: Vec3::from_jph(contact.linearVelocity),
            motion_type: MotionType::from_jph(contact.motionTypeB),
            is_sensor: contact.isSensorB,
            had_collision: contact.hadCollision,
            was_discarded: contact.wasDiscarded,
            is_back_facing: contact.isBackFacingContact,
        }
    }
}

/// The persistent state of a character, from [`CharacterRef::save_state`], to continue a run
/// bit for bit with [`CharacterMut::restore_state`].
///
/// It holds Jolt's `CharacterVirtual::SaveState` stream (pose, velocity, ground data and the
/// contacts the character collided with) and the character's up, which Jolt does not save.
/// It does not hold the settings, the shape or the contacts without collision, and Jolt does not
/// restore a contact's character pointer, material or user data, nor the ground's material or
/// user data: restored contacts and ground carry Jolt's default material and user data 0. None
/// of these feed an update that moves the character: before moving, Jolt reads only the normals
/// and velocities of contacts with collision, and the move rebuilds the contacts. An update
/// shorter than [`CharacterSettings::min_time_remaining`] moves nothing and takes its ground from
/// the restored contacts, with the default material and user data 0.
///
/// [`CharacterSettings::min_time_remaining`]: crate::CharacterSettings::min_time_remaining
#[derive(Clone, PartialEq, Eq, Debug)]
pub struct CharacterState {
    jolt: Vec<u8>,
    up: [u32; 3],
}

impl CharacterState {
    /// The state as bytes, for digests: Jolt's stream, then the bits of up's three components
    /// in little-endian order. The bytes are specific to this build; there is no way back.
    pub fn as_bytes(&self) -> Vec<u8> {
        let mut bytes = self.jolt.clone();
        for bits in self.up {
            bytes.extend_from_slice(&bits.to_le_bytes());
        }
        bytes
    }
}

/// A Jolt state recorder (`StateRecorderImpl`), owned whole by its owner.
impl JoltObject for JPH_StateRecorder {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner owns the recorder (trait contract), which the extension deletes.
        unsafe { JPH_StateRecorder_Destroy(ptr) };
    }
}

/// A character, of which the world holds the one reference `JPH_CharacterVirtual_Create`
/// returns. Releasing it deletes the character, whose destructor removes and destroys its inner
/// body through the physics system (`CharacterVirtual.cpp`), so the system must outlive it.
impl JoltObject for JPH_CharacterVirtual {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner holds the one reference (trait contract); the world drops its
        // characters before its physics system (field order in `PhysicsWorld`).
        unsafe { JPH_CharacterBase_Destroy(ptr.cast()) };
    }
}

/// Jolt's `CharacterVsCharacterCollisionSimple`, owned whole by its world. Destroying it only
/// frees its list of character pointers.
impl JoltObject for JPH_CharacterVsCharacterCollision {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner owns the object (trait contract); joltc deletes it through Jolt's
        // virtual destructor.
        unsafe { JPH_CharacterVsCharacterCollision_Destroy(ptr) };
    }
}

/// One character of a world.
pub(crate) struct CharacterEntry {
    character: Owned<JPH_CharacterVirtual>,
    /// One reference to each material the character's cached contacts point to. Jolt keeps a
    /// raw pointer per contact (`CharacterVirtual.h`, `CharacterContact::mMaterial`), and an
    /// update that moves nothing picks the ground from those contacts and takes a reference to
    /// its material (`CharacterVirtual::UpdateSupportingContact`), also after the shape that
    /// held the material is gone. Every native call that may replace the contacts is followed
    /// by [`PhysicsWorld::retain_contact_materials`](crate::PhysicsWorld::retain_contact_materials).
    /// Declared after `character`, so the references are released after the character.
    contact_materials: Vec<Owned<JPH_PhysicsMaterial>>,
    inner_body: Option<BodyId>,
    collides_with_characters: bool,
    /// The mass with which the character presses on what it stands on.
    mass: f32,
}

#[cfg(test)]
mod tests;
