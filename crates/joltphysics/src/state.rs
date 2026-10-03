//! Saving and restoring a world's simulation state, for rollback and replay (Jolt
//! `PhysicsSystem::SaveState` and `RestoreState`, plus every character's own state).

use std::fmt;

use joltphysics_sys::*;

use crate::owned::Owned;
use crate::world::WorldTag;
use crate::{BodyError, BodyId, CharacterState, PhysicsWorld, StateError};

/// A world's simulation state at one moment, from [`PhysicsWorld::save_state`] or
/// [`PhysicsWorld::save_state_of`], to go back to with [`PhysicsWorld::restore_state`].
///
/// # What is saved
/// Jolt's saved state of the physics system: the previous step's delta time and the world's
/// gravity; per body its pose, velocities, accumulated force and torque, sleep test data,
/// whether it may sleep and whether it is awake; the contact cache; and every constraint's own
/// state (its enabled flag, the solver parts' warm start, motor states and targets, and for
/// path constraints also the motor settings and maximum friction). For vehicles that covers the
/// driver input, the engine, transmission and wheel state. On top of Jolt's state it holds every
/// character's [`CharacterState`].
///
/// # Which world it restores into
/// A state restores only into the world that saved it, and only while that world's structure is
/// unchanged: creating or removing a body, character, vehicle, ragdoll or constraint, setting a
/// ragdoll's motion type ([`RagdollMut::set_motion_type`](crate::RagdollMut::set_motion_type)) or
/// a [`rebase`](PhysicsWorld::rebase) that moves anything makes every earlier state
/// unrestorable ([`StateError::WorldChanged`]). Calls that fail their checks change nothing and
/// keep states restorable. Jolt saves neither which objects exist nor its body id allocator, so
/// rolling back across a creation or removal is not supported; after a restore, the same calls
/// as in the original run create bodies with the same ids as in that run.
///
/// # What is not saved
/// Configuration Jolt does not save stays as it is when a state is restored. A caller that
/// changes it during a run and rolls back must set it back and replay those calls, as Jolt's
/// rollback documentation asks for body friction. These setters change such configuration:
/// - hinge: `set_motor_settings`, `set_limits`, `set_limits_spring`, `set_max_friction_torque`;
/// - slider: `set_motor_settings`, `set_limits`, `set_limits_spring`, `set_max_friction_force`;
/// - distance: `set_distance`, `set_limits_spring`;
/// - pulley: `set_length`;
/// - cone: `set_half_cone_angle`;
/// - swing-twist: `set_swing_motor_settings`, `set_twist_motor_settings`,
///   `set_max_friction_torque`;
/// - six-DOF: `set_motor_settings`;
/// - vehicle: [`set_gravity`](crate::VehicleMut::set_gravity),
///   [`set_max_pitch_roll_angle`](crate::VehicleMut::set_max_pitch_roll_angle),
///   [`set_collision_tester`](crate::VehicleMut::set_collision_tester).
///
/// Body properties set at creation (shape, mass, friction, layers, ...) are not saved either;
/// the body setters change only poses and velocities. A setting the caller applies again before every
/// [`step`](PhysicsWorld::step), such as a vehicle's gravity on a planet, replays exactly.
///
/// The wheel contacts a vehicle reports ([`WheelState::contact`](crate::WheelState)) are empty
/// right after a restore until the next step, because Jolt clears a wheel's contact body on
/// restore.
///
/// # Bytes and equality
/// [`as_bytes`](Self::as_bytes) gives the state as bytes for diagnostics. No API reads them
/// back: they are specific to this build, carry no version, and Jolt reads its stream without
/// any checks, so they are not a save-game or network format.
///
/// Jolt also writes fields it has never initialised: a wheel's contact position, normal and
/// lateral direction before the wheel's first contact (`VehicleConstraint::SaveState`). Jolt
/// reads them only once the wheel has a contact, so a restore is not affected, but two worlds
/// built with the same calls can give different bytes. Equality (`==`) compares the bytes, so it
/// is meaningful between states of one world, such as a state and the state saved right after
/// restoring it, and not as a digest across worlds or processes.
#[derive(Clone, PartialEq, Eq)]
pub struct WorldState {
    world: WorldTag,
    epoch: u64,
    body_count: u32,
    constraint_count: u32,
    character_ids: Vec<u32>,
    jolt: Vec<u8>,
    characters: Vec<CharacterState>,
}

impl fmt::Debug for WorldState {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("WorldState")
            .field("epoch", &self.epoch)
            .field("body_count", &self.body_count)
            .field("constraint_count", &self.constraint_count)
            .field("characters", &self.character_ids.len())
            .field("jolt_bytes", &self.jolt.len())
            .finish_non_exhaustive()
    }
}

impl WorldState {
    /// The state as bytes, for diagnostics, all lengths and counts little-endian:
    /// the `u64` length of Jolt's stream and the stream, then the `u32` character count and per
    /// character the `u64` length of [`CharacterState::as_bytes`] and those bytes. The bytes are
    /// specific to this build (precision, Jolt revision), carry no version and may hold bytes
    /// Jolt never initialised (see [`WorldState`]); there is no way back.
    pub fn as_bytes(&self) -> Vec<u8> {
        let mut bytes = Vec::with_capacity(self.jolt.len() + 16);
        bytes.extend_from_slice(&(self.jolt.len() as u64).to_le_bytes());
        bytes.extend_from_slice(&self.jolt);
        let count = u32::try_from(self.characters.len()).expect("character ids are u32");
        bytes.extend_from_slice(&count.to_le_bytes());
        for character in &self.characters {
            let character = character.as_bytes();
            bytes.extend_from_slice(&(character.len() as u64).to_le_bytes());
            bytes.extend_from_slice(&character);
        }
        bytes
    }
}

impl PhysicsWorld {
    /// Saves the whole simulation state of the world, every body included. See [`WorldState`]
    /// for what it holds and when it can be restored.
    ///
    /// ```
    /// use joltphysics::*;
    ///
    /// # fn main() -> Result<(), Box<dyn std::error::Error>> {
    /// let mut world = PhysicsWorld::new(WorldSettings::default())?;
    /// let ball_shape = Shape::new_sphere(0.5)?;
    /// let ball = world.create_body(
    ///     &ball_shape,
    ///     &BodySettings::new_dynamic().position(RVec3::new(0.0, 2.0, 0.0)),
    /// )?;
    ///
    /// let saved = world.save_state();
    /// world.step(1.0 / 60.0)?;
    /// let after_one_step = world.body(ball)?.position();
    ///
    /// world.restore_state(&saved)?;
    /// assert_eq!(world.body(ball)?.position(), RVec3::new(0.0, 2.0, 0.0));
    /// world.step(1.0 / 60.0)?;
    /// assert_eq!(world.body(ball)?.position(), after_one_step);
    /// # Ok(())
    /// # }
    /// ```
    pub fn save_state(&self) -> WorldState {
        self.state_with(self.record(None))
    }

    /// Saves the simulation state with only the bodies in `bodies` (any order, duplicates
    /// allowed); global state, contacts, constraints and characters are saved whole.
    ///
    /// Restoring it leaves every other body in its current state, so a replay from it is exact
    /// only when those bodies did not change since the save, for example static bodies the caller
    /// never moved. Fails with [`BodyError::WrongWorld`] for an id of another world and
    /// [`BodyError::NotFound`] for a removed body, before anything is saved.
    pub fn save_state_of(&self, bodies: &[BodyId]) -> Result<WorldState, BodyError> {
        for &id in bodies {
            self.check(id)?;
        }
        let raw: Vec<u32> = bodies.iter().map(|id| id.to_raw()).collect();
        Ok(self.state_with(self.record(Some(&raw))))
    }

    /// Returns the world to `state`. See [`WorldState`] for what this restores and what it
    /// leaves alone.
    ///
    /// Fails with [`StateError::WrongWorld`] for a state of another world and with
    /// [`StateError::WorldChanged`] when the world's structure changed since the save; the world
    /// is unchanged then. [`StateError::RestoreFailed`] is not expected for a state the world
    /// accepts; if it happens, the world may be partly restored.
    ///
    /// After a restore, the same calls as in the original run give bit-identical results; the
    /// tests check this with 1 and 4 worker threads, in one process and across two.
    pub fn restore_state(&mut self, state: &WorldState) -> Result<(), StateError> {
        if state.world != self.tag {
            return Err(StateError::WrongWorld);
        }
        let same_characters = self.characters.keys().eq(state.character_ids.iter());
        if state.epoch != self.structure_epoch
            || state.body_count != self.body_count()
            || state.constraint_count != self.jolt_constraint_count()
            || !same_characters
        {
            return Err(StateError::WorldChanged);
        }

        // The characters go first: restoring one moves its inner body to the character's pose
        // (`CharacterVirtual::SetPosition`), which also resets that body's sleep timer. The
        // system restore then writes the inner body's saved state and activation back.
        let ids: Vec<_> = self.character_ids().collect();
        for (id, character_state) in ids.into_iter().zip(&state.characters) {
            let restored = self
                .character_mut(id)
                .map_err(|_| ())
                .and_then(|mut character| character.restore_state(character_state).map_err(|_| ()));
            if restored.is_err() {
                return Err(StateError::RestoreFailed);
            }
        }

        let restored = self.restore_jolt(&state.jolt);
        debug_assert!(restored, "a saved world state failed to restore");
        if restored {
            Ok(())
        } else {
            Err(StateError::RestoreFailed)
        }
    }

    /// Number of constraints in the Jolt system, vehicles and ragdoll joints included.
    fn jolt_constraint_count(&self) -> u32 {
        // SAFETY: the system is live; Jolt counts under its constraint mutex.
        unsafe { JPH_PhysicsSystem_GetNumConstraints(self.system.as_ptr()) }
    }

    /// A state of this world now, around Jolt's saved stream `jolt`.
    fn state_with(&self, jolt: Vec<u8>) -> WorldState {
        let characters = self
            .character_ids()
            .map(|id| {
                self.character(id)
                    .unwrap_or_else(|_| unreachable!("listed by the world"))
                    .save_state()
            })
            .collect();
        WorldState {
            world: self.tag,
            epoch: self.structure_epoch,
            body_count: self.body_count(),
            constraint_count: self.jolt_constraint_count(),
            character_ids: self.characters.keys().copied().collect(),
            jolt,
            characters,
        }
    }

    /// Jolt's saved stream of every part of the system's state, with only the bodies whose raw
    /// ids are in `bodies`, or with every body for `None`.
    fn record(&self, bodies: Option<&[u32]>) -> Vec<u8> {
        // SAFETY: Jolt is initialised (the world exists). The handle takes over the recorder.
        let recorder = unsafe { Owned::from_raw(JPH_StateRecorder_Create()) }
            .unwrap_or_else(|| unreachable!("`new` does not return null"));
        let (ids, count) = match bodies {
            // A world holds at most 2^23 bodies, and every id was checked to be one of them, but
            // duplicates may make the list longer.
            Some(ids) => (
                ids.as_ptr(),
                u32::try_from(ids.len()).expect("more than u32::MAX body ids"),
            ),
            None => (std::ptr::null(), 0),
        };
        // SAFETY: the system is live and no step runs: `step` needs `&mut self`. `SaveState` is
        // const in Jolt and takes the body and constraint locks itself (see `Sync for
        // PhysicsWorld`). The recorder is live and used by this thread only. `ids` is null or
        // readable for `count` ids, which the extension copies.
        let size = unsafe {
            JPH_PhysicsSystem_SaveState(
                self.system.as_ptr(),
                recorder.as_ptr(),
                JPH_StateRecorderState_All,
                ids,
                count,
            );
            JPH_StateRecorder_GetDataSize(recorder.as_ptr())
        };
        let mut jolt = vec![0_u8; size];
        // SAFETY: `jolt` holds exactly `size` writable bytes, which joltc copies at most.
        unsafe { JPH_StateRecorder_CopyData(recorder.as_ptr(), jolt.as_mut_ptr().cast(), size) };
        jolt
    }

    /// Restores Jolt's saved stream `jolt`; whether Jolt read it without failing.
    fn restore_jolt(&mut self, jolt: &[u8]) -> bool {
        // SAFETY: Jolt is initialised (the world exists). The handle takes over the recorder.
        let recorder = unsafe { Owned::from_raw(JPH_StateRecorder_Create()) }
            .unwrap_or_else(|| unreachable!("`new` does not return null"));
        // SAFETY: the recorder is live and used by this thread only, and `jolt` is readable for
        // its length. The bytes are a complete stream that `SaveState` of this world wrote at
        // the current structure epoch (`restore_state` checked both, and `WorldState` has no
        // other constructor), so the bodies and constraints it names exist, in the same
        // constraint order, and `RestoreState` reads exactly what was written. The world is
        // borrowed mutably, so no step, query or body access runs meanwhile; this thread holds
        // no body lock.
        unsafe {
            JPH_StateRecorder_WriteBytes(recorder.as_ptr(), jolt.as_ptr().cast(), jolt.len());
            JPH_StateRecorder_Rewind(recorder.as_ptr());
            let restored = JPH_PhysicsSystem_RestoreState(self.system.as_ptr(), recorder.as_ptr());
            restored && !JPH_StateRecorder_IsFailed(recorder.as_ptr())
        }
    }
}
