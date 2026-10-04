//! Creating, removing and updating characters.

use std::marker::PhantomData;
use std::ptr::{null, NonNull};

use oxijolt_sys::*;

use super::{
    CharacterEntry, CharacterId, CharacterMut, CharacterRef, CharacterSettings,
    ExtendedUpdateSettings, INVALID_ID,
};
use crate::filter::with_query_filters;
use crate::limits::{self, POSITION_RULE};
use crate::math::ROTATION_RULE;
use crate::owned::Owned;
use crate::world::DELTA_TIME_RULE;
use crate::{BodyId, CharacterError, PhysicsWorld, Quat, QueryFilter, RVec3, Vec3};

impl PhysicsWorld {
    /// Creates a character at `position` (metres, every component at most
    /// [`limits::MAX_POSITION`] in absolute value) with `rotation` and returns its id.
    ///
    /// Fails with [`CharacterError::InvalidValue`] when a setting or the pose is out of range
    /// (see the setters of [`CharacterSettings`]), with [`CharacterError::TooManyBodies`] when an
    /// inner body was asked for and the world is full, and with
    /// [`CharacterError::TooManyCharacters`] when the world has run out of character ids.
    /// Nothing is created on failure.
    ///
    /// The new character knows no contacts and reports [`GroundState::InAir`](crate::GroundState::InAir) until its first
    /// update or [`refresh_character_contacts`](Self::refresh_character_contacts). Refresh a
    /// character that starts on the ground: stick to floor acts only when the character was
    /// supported before the update.
    pub fn create_character(
        &mut self,
        settings: &CharacterSettings<'_>,
        position: RVec3,
        rotation: Quat,
    ) -> Result<CharacterId, CharacterError> {
        settings.validate(self.object_layer_count)?;
        if !limits::is_in_frame(position) {
            return Err(CharacterError::InvalidValue(POSITION_RULE));
        }
        if !rotation.is_valid_rotation() {
            return Err(CharacterError::InvalidValue(ROTATION_RULE));
        }
        let raw = self.next_character_id;
        // Jolt's invalid `CharacterID`.
        if raw == INVALID_ID {
            return Err(CharacterError::TooManyCharacters);
        }
        if settings.inner_body.is_some() && !self.has_room_for_bodies(1) {
            return Err(CharacterError::TooManyBodies);
        }
        let collision = if settings.collide_with_characters {
            Some(self.character_collision()?)
        } else {
            None
        };
        // The world passes an explicit id: Jolt's default comes from a process-wide counter,
        // and Jolt orders contacts between characters by id.
        let jolt_settings = settings.to_jph(raw);
        let position = position.to_jph();
        let rotation = rotation.to_jph();
        self.note_structure_change();
        // SAFETY: the system is live and borrowed mutably; the settings, their shape pointers
        // (borrowed from `settings`) and the pose are live for the call, and validated. The
        // character takes its own references to the shapes. The handle takes over the one
        // reference joltc returns.
        let character = unsafe {
            Owned::from_raw(JPH_CharacterVirtual_Create(
                &jolt_settings,
                &position,
                &rotation,
                settings.user_data,
                self.system.as_ptr(),
            ))
        }
        .unwrap_or_else(|| unreachable!("joltc `new`s the character"));
        let inner_body = if settings.inner_body.is_some() {
            // SAFETY: the character is live; the getter reads a member.
            let raw_body = unsafe { JPH_CharacterVirtual_GetInnerBodyID(character.as_ptr()) };
            if raw_body == INVALID_ID {
                // Jolt created no body because the world is full, which the room check above
                // rules out; dropping the character releases it and creates nothing else.
                return Err(CharacterError::TooManyBodies);
            }
            Some(BodyId::new(raw_body, self.tag))
        } else {
            None
        };
        if let Some(collision) = collision {
            // SAFETY: both objects are live and owned by this world, borrowed mutably. The set
            // keeps a pointer to the character until `remove_character` takes it out; the
            // character keeps a pointer to the set, which the world drops after its characters.
            unsafe {
                JPH_CharacterVsCharacterCollisionSimple_AddCharacter(
                    collision.as_ptr(),
                    character.as_ptr(),
                );
                JPH_CharacterVirtual_SetCharacterVsCharacterCollision(
                    character.as_ptr(),
                    collision.as_ptr(),
                );
            }
        }
        if let Some(body) = inner_body {
            self.inner_bodies.insert(body.to_raw());
        }
        self.characters.insert(
            raw,
            CharacterEntry {
                character,
                inner_body,
                collides_with_characters: settings.collide_with_characters,
                mass: settings.mass,
            },
        );
        self.next_character_id += 1;
        Ok(CharacterId::new(raw, self.tag))
    }

    /// The world's character-versus-character set, created on first use.
    fn character_collision(
        &mut self,
    ) -> Result<NonNull<JPH_CharacterVsCharacterCollision>, CharacterError> {
        if self.character_collision.is_none() {
            // SAFETY: Jolt is initialised. The handle takes over the new object.
            let created =
                unsafe { Owned::from_raw(JPH_CharacterVsCharacterCollision_CreateSimple()) }
                    .ok_or(CharacterError::InvalidValue(
                        "could not create the character collision set",
                    ))?;
            self.character_collision = Some(created);
        }
        Ok(self
            .character_collision
            .as_ref()
            .map(Owned::as_non_null)
            .unwrap_or_else(|| unreachable!("set above")))
    }

    /// The entry of `id`, if it names a character of this world.
    fn character_entry(&self, id: CharacterId) -> Result<&CharacterEntry, CharacterError> {
        if id.world != self.tag {
            return Err(CharacterError::WrongWorld(id));
        }
        self.characters
            .get(&id.raw)
            .ok_or(CharacterError::NotFound(id))
    }

    /// Removes a character, and with it its inner body.
    ///
    /// Bodies around the removed inner body are not woken: a kinematic inner body only touches
    /// dynamic bodies, which stay awake while they touch it.
    pub fn remove_character(&mut self, id: CharacterId) -> Result<(), CharacterError> {
        self.character_entry(id)?;
        self.note_structure_change();
        let entry = self
            .characters
            .remove(&id.raw)
            .unwrap_or_else(|| unreachable!("checked above"));
        if entry.collides_with_characters {
            if let Some(collision) = &self.character_collision {
                // SAFETY: both are live and owned by this world, borrowed mutably; the character
                // is in the set (it was added at creation).
                unsafe {
                    JPH_CharacterVsCharacterCollisionSimple_RemoveCharacter(
                        collision.as_ptr(),
                        entry.character.as_ptr(),
                    )
                };
            }
        }
        if let Some(body) = entry.inner_body {
            self.inner_bodies.remove(&body.to_raw());
        }
        // Other characters may keep a pointer to this one in their cached contacts. That is
        // sound: without a contact listener Jolt never dereferences a cached contact's
        // `mCharacterB` (`ValidateContact` and `ContactAdded` return early), joltc's contact
        // readout copies the pointer without dereferencing it, and oxijolt installs no
        // listener and never reads that pointer.
        drop(entry);
        Ok(())
    }

    /// Read access to a character.
    pub fn character(&self, id: CharacterId) -> Result<CharacterRef<'_>, CharacterError> {
        let entry = self.character_entry(id)?;
        Ok(CharacterRef {
            world: self,
            id,
            character: entry.character.as_non_null(),
        })
    }

    /// Write access to a character.
    pub fn character_mut(&mut self, id: CharacterId) -> Result<CharacterMut<'_>, CharacterError> {
        let entry = self.character_entry(id)?;
        Ok(CharacterMut {
            character: entry.character.as_non_null(),
            _world: PhantomData,
        })
    }

    /// The ids of the world's characters, in id order (creation order).
    pub fn character_ids(&self) -> impl Iterator<Item = CharacterId> + '_ {
        self.characters
            .keys()
            .map(|&raw| CharacterId::new(raw, self.tag))
    }

    /// Whether `id` is the inner body of a character of this world.
    pub fn is_inner_body(&self, id: BodyId) -> bool {
        id.world == self.tag && self.inner_bodies.contains(&id.to_raw())
    }

    /// Moves a character by its linear velocity for `delta_time` seconds, colliding with what
    /// `filter` selects, then sticks it to the floor and walks it up stairs as `settings` says
    /// (Jolt `CharacterVirtual::ExtendedUpdate`).
    ///
    /// Set the velocity first with [`CharacterMut::set_linear_velocity`]. `gravity` (m/s²) is
    /// not added to the velocity; Jolt uses it to press on what the character stands on. The
    /// character's own inner body is never hit. Characters that collide with characters also
    /// hit each other, whatever the filter says.
    ///
    /// `delta_time` must be finite, at least [`MIN_DELTA_TIME`](Self::MIN_DELTA_TIME) and at
    /// most [`MAX_DELTA_TIME`](Self::MAX_DELTA_TIME), `gravity` finite and at most
    /// [`limits::MAX_ACCELERATION`] long, the character's mass times the length of `gravity`
    /// times `delta_time` at most [`limits::MAX_WEIGHT_IMPULSE`], the settings valid and the
    /// filter's layers in this world; otherwise nothing happens and
    /// [`CharacterError::InvalidValue`] is returned.
    ///
    /// # Panics
    /// A panic in a filter callback is caught inside the update (the callback then rejects) and
    /// resumed after joltc has returned.
    ///
    /// # Example
    /// ```
    /// use oxijolt::*;
    ///
    /// # fn main() -> Result<(), Box<dyn std::error::Error>> {
    /// let mut world = PhysicsWorld::new(WorldSettings::default())?;
    /// let floor = Shape::new_box(Vec3::new(10.0, 1.0, 10.0))?;
    /// world.create_body(&floor, &BodySettings::new_static().position(RVec3::new(0.0, -1.0, 0.0)))?;
    ///
    /// let capsule = Shape::new_capsule(0.7, 0.4)?;
    /// let settings = CharacterSettings::new(&capsule).shape_offset(Vec3::new(0.0, 1.1, 0.0));
    /// let id = world.create_character(&settings, RVec3::new(0.0, 0.5, 0.0), Quat::IDENTITY)?;
    /// let gravity = Vec3::new(0.0, -9.81, 0.0);
    /// for _ in 0..60 {
    ///     world.character_mut(id)?.set_linear_velocity(Vec3::new(1.0, -1.0, 0.0))?;
    ///     world.update_character(id, 1.0 / 60.0, gravity, &ExtendedUpdateSettings::default(), &QueryFilter::new())?;
    /// }
    /// assert_eq!(world.character(id)?.ground_state(), GroundState::OnGround);
    /// # Ok(())
    /// # }
    /// ```
    pub fn update_character(
        &mut self,
        id: CharacterId,
        delta_time: f32,
        gravity: Vec3,
        settings: &ExtendedUpdateSettings,
        filter: &QueryFilter<'_>,
    ) -> Result<(), CharacterError> {
        if !Self::is_valid_delta_time(delta_time) {
            return Err(CharacterError::InvalidValue(DELTA_TIME_RULE));
        }
        if !limits::is_acceleration(gravity) {
            return Err(CharacterError::InvalidValue(limits::GRAVITY_RULE));
        }
        settings.validate()?;
        let entry = self.character_entry(id)?;
        if !limits::is_weight_impulse(entry.mass, gravity, delta_time) {
            return Err(CharacterError::InvalidValue(
                "mass times gravity times delta time must be at most limits::MAX_WEIGHT_IMPULSE",
            ));
        }
        let character = entry.character.as_ptr();
        filter.validate(self).map_err(query_error)?;
        let gravity = gravity.to_jph();
        let settings = settings.to_jph();
        let allocator = self.temp_allocator.as_ptr();
        with_query_filters(self, filter, |raw, _| {
            // SAFETY: `&mut self` gives this call exclusive use of the world, the character and
            // the temp allocator; `gravity` and `settings` are live locals and the filters are
            // live or null (accept everything). The filter callbacks get `&PhysicsWorld` while
            // Jolt writes the inner body's pose and pushes the ground body through the locking
            // body interface: C++ memory, not memory behind that reference, and no Rust field
            // of the world changes. CharacterVirtual queries through the system's locking
            // narrow-phase query, which releases the body lock before calling the shape filter
            // (`NarrowPhaseQuery.cpp`), so the callback's `GetShape` does not deadlock. No body
            // is removed and no shape replaced during the call, so the filters' cached root
            // shape stays valid. The character's own inner body never reaches the filters
            // (`IgnoreSingleBodyFilterChained`), and collisions between characters call no
            // filter.
            unsafe {
                JPH_CharacterVirtual_ExtendedUpdate2(
                    character,
                    delta_time,
                    &gravity,
                    &settings,
                    null(),
                    raw.object_layer,
                    raw.body,
                    raw.shape,
                    allocator,
                )
            }
        })
        .map_err(query_error)
    }

    /// Recomputes a character's contacts and ground at its current pose, without moving it,
    /// colliding with what `filter` selects. Call it after moving a character with a setter or
    /// after a rotating [`rebase`](Self::rebase).
    ///
    /// Fails as [`update_character`](Self::update_character) does for the filter, and panics in
    /// filter callbacks resume the same way.
    pub fn refresh_character_contacts(
        &mut self,
        id: CharacterId,
        filter: &QueryFilter<'_>,
    ) -> Result<(), CharacterError> {
        let character = self.character_entry(id)?.character.as_ptr();
        filter.validate(self).map_err(query_error)?;
        let allocator = self.temp_allocator.as_ptr();
        with_query_filters(self, filter, |raw, _| {
            // SAFETY: as in `update_character`, without the move.
            unsafe {
                JPH_CharacterVirtual_RefreshContacts2(
                    character,
                    null(),
                    raw.object_layer,
                    raw.body,
                    raw.shape,
                    allocator,
                )
            }
        })
        .map_err(query_error)
    }
}

/// A query filter error as a character error.
fn query_error(error: crate::QueryError) -> CharacterError {
    match error {
        crate::QueryError::InvalidValue(what) => CharacterError::InvalidValue(what),
    }
}
