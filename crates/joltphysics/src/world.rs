//! The physics world: a Jolt physics system with its own job system and temp allocator.
//!
//! joltc keeps a global map of physics systems that creating and destroying a system writes
//! without synchronisation, so both run under one process-wide lock here. Stepping uses the
//! world's own temp allocator and job system and needs no global lock, so independent worlds
//! step in parallel. The joltc functions that read that global map (joltc's own step listener,
//! `JPH_PhysicsStepListener_Create`) or use joltc's shared temp allocator
//! (`JPH_PhysicsSystem_Update` and joltc's own character updates) are not used here; wrapping
//! them requires revisiting this lock. Vehicles are Jolt's own step listeners: they get the
//! system from the step context and touch no joltc global, so worlds with vehicles still step
//! in parallel. Characters are updated with
//! the world's own temp allocator through the `joltphysics-sys` extension
//! (`JPH_CharacterVirtual_ExtendedUpdate2`, `JPH_CharacterVirtual_RefreshContacts2`), which
//! needs `&mut self`. Callbacks that run inside a step must not use the locking body interface,
//! which would deadlock.

use std::collections::{BTreeMap, BTreeSet};
use std::num::NonZeroU64;
use std::ptr::NonNull;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Mutex, MutexGuard, OnceLock, PoisonError};

use joltphysics_sys::*;

use crate::body::with_locked_body;
use crate::character::CharacterEntry;
use crate::math::is_finite_positive;
use crate::owned::{JoltObject, Owned};
use crate::vehicle::VehicleEntry;
use crate::{
    BodyError, BodyId, CollisionLayers, MotionType, Quat, RVec3, StepError, Vec3, WorldError,
};

/// Runs `JPH_Init` once per process and returns whether it succeeded. joltphysics never calls
/// `JPH_Shutdown`: Jolt's global state lives as long as the process.
pub(crate) fn ensure_initialized() -> bool {
    static INIT: OnceLock<bool> = OnceLock::new();
    // SAFETY: `OnceLock` runs the closure exactly once per process, which is the only
    // synchronisation `JPH_Init` (an unsynchronised `bool` guard) needs.
    *INIT.get_or_init(|| unsafe { JPH_Init() })
}

/// Serialises `JPH_PhysicsSystem_Create` and `JPH_PhysicsSystem_Destroy`, which write joltc's
/// unsynchronised global map of physics systems.
fn lock_joltc_globals() -> MutexGuard<'static, ()> {
    static GLOBALS: Mutex<()> = Mutex::new(());
    GLOBALS.lock().unwrap_or_else(PoisonError::into_inner)
}

/// Settings for [`PhysicsWorld::new`]. The defaults are Jolt's and joltc's.
#[derive(Clone, Debug)]
pub struct WorldSettings {
    max_bodies: u32,
    max_body_pairs: u32,
    max_contact_constraints: u32,
    worker_threads: u32,
    temp_allocator_size: u32,
    gravity: Vec3,
    layers: CollisionLayers,
}

impl Default for WorldSettings {
    fn default() -> Self {
        Self {
            max_bodies: 10240,
            max_body_pairs: 65536,
            max_contact_constraints: 10240,
            worker_threads: 1,
            temp_allocator_size: 10 * 1024 * 1024,
            gravity: Vec3::new(0.0, -9.81, 0.0),
            layers: CollisionLayers::default(),
        }
    }
}

impl WorldSettings {
    /// Largest body count Jolt supports (`PhysicsSystem::cMaxBodiesLimit`).
    const MAX_BODIES_LIMIT: u32 = 1 << 23;

    /// Largest accepted [`worker_threads`](Self::worker_threads) value. Jolt starts one OS
    /// thread per worker; this bound is chosen by joltphysics to keep thread creation sane and
    /// is not a Jolt limit.
    pub const MAX_WORKER_THREADS: u32 = 64;

    /// Maximum number of bodies in the world, at most 2²³. Default 10240.
    #[must_use]
    pub fn max_bodies(mut self, value: u32) -> Self {
        self.max_bodies = value;
        self
    }

    /// Maximum number of body pairs the broad phase can report per step. Default 65536.
    #[must_use]
    pub fn max_body_pairs(mut self, value: u32) -> Self {
        self.max_body_pairs = value;
        self
    }

    /// Maximum number of contact constraints per step. Default 10240.
    #[must_use]
    pub fn max_contact_constraints(mut self, value: u32) -> Self {
        self.max_contact_constraints = value;
        self
    }

    /// Worker threads Jolt's job system starts in addition to the thread that calls
    /// [`PhysicsWorld::step`], which also runs jobs. At least 1 and at most
    /// [`WorldSettings::MAX_WORKER_THREADS`]. Default 1. Results are bit-identical for any value
    /// on one machine.
    #[must_use]
    pub fn worker_threads(mut self, value: u32) -> Self {
        self.worker_threads = value;
        self
    }

    /// Size in bytes of the scratch memory the world's steps allocate from. Default 10 MiB.
    #[must_use]
    pub fn temp_allocator_size(mut self, value: u32) -> Self {
        self.temp_allocator_size = value;
        self
    }

    /// Gravity in m/s², finite. Default `(0, -9.81, 0)`, Jolt's default. Zero is allowed.
    #[must_use]
    pub fn gravity(mut self, value: Vec3) -> Self {
        self.gravity = value;
        self
    }

    /// The collision layer setup. Default [`CollisionLayers::default`].
    #[must_use]
    pub fn layers(mut self, value: CollisionLayers) -> Self {
        self.layers = value;
        self
    }

    pub(crate) fn validate(&self) -> Result<(), WorldError> {
        let invalid = |what| Err(WorldError::InvalidSettings(what));
        if !(1..=Self::MAX_BODIES_LIMIT).contains(&self.max_bodies) {
            return invalid("max_bodies must be between 1 and 2^23");
        }
        if self.max_body_pairs == 0 {
            return invalid("max_body_pairs must be at least 1");
        }
        if self.max_contact_constraints == 0 {
            return invalid("max_contact_constraints must be at least 1");
        }
        if !(1..=Self::MAX_WORKER_THREADS).contains(&self.worker_threads) {
            return invalid("worker_threads must be between 1 and 64");
        }
        if self.temp_allocator_size == 0 {
            return invalid("temp_allocator_size must be at least 1");
        }
        if !self.gravity.is_finite() {
            return invalid("gravity must be finite");
        }
        self.layers.validate()
    }
}

/// A world's temp allocator, owned by the world.
impl JoltObject for JPH_TempAllocator {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner owns the allocator (trait contract); the system that used it is
        // gone or never existed (field order in `PhysicsWorld`).
        unsafe { JPH_TempAllocator_Destroy(ptr) };
    }
}

/// A world's job system and its worker threads, owned by the world.
impl JoltObject for JPH_JobSystem {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner owns the job system (trait contract), and no step is running
        // (`step` borrows the world mutably). Destroying it joins the worker threads.
        unsafe { JPH_JobSystem_Destroy(ptr) };
    }
}

/// A physics system, owned by its world. Destroying it also deletes its bodies with their
/// shape references and the three layer objects passed to `JPH_PhysicsSystem_Create`
/// (joltc's `JPH_PhysicsSystem_Destroy`).
///
/// Destroying takes the globals lock, so an `Owned<JPH_PhysicsSystem>` must never be dropped
/// while that lock is held: the mutex is not reentrant.
impl JoltObject for JPH_PhysicsSystem {
    unsafe fn destroy(ptr: *mut Self) {
        let _globals = lock_joltc_globals();
        // SAFETY: the owner owns the system (trait contract) and the globals lock is held, so
        // no other thread writes joltc's system map. Nothing borrows the system any more.
        unsafe { JPH_PhysicsSystem_Destroy(ptr) };
    }
}

/// Identifies a world among all worlds of the process, so a [`BodyId`](crate::BodyId) from one
/// world is rejected by another. Never affects simulation results.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Debug)]
pub(crate) struct WorldTag(NonZeroU64);

impl WorldTag {
    fn next() -> Self {
        static NEXT_WORLD: AtomicU64 = AtomicU64::new(1);
        let value = NEXT_WORLD.fetch_add(1, Ordering::Relaxed);
        Self(NonZeroU64::new(value).expect("world counter overflowed"))
    }
}

/// A Jolt physics system with its collision layers, its own job system and temp allocator.
///
/// Changing the world, including [`step`](Self::step), takes `&mut self`; reading takes
/// `&self`, so many threads can read one world while nobody steps it. Several worlds are
/// independent and may be stepped on different threads at the same time.
///
/// Bodies added one at a time leave the broad phase unoptimised, which makes queries slower
/// until Jolt has rebuilt it during steps (Jolt docs, "Bodies"); see
/// [`optimize_broad_phase`](Self::optimize_broad_phase).
pub struct PhysicsWorld {
    // Field order is drop order, after `Drop for PhysicsWorld` has taken every vehicle out of
    // the system's step listeners and constraints and released it, so `vehicles` is empty by
    // then. The characters go first: each destructor removes its inner
    // body through the still-live system. The character collision set follows; it only frees
    // its list of character pointers. The system goes next, before the job system and allocator
    // its steps used, and deletes the layer tables it owns. The interface and query pointers
    // after them are borrowed from the system and have no destructor.
    /// The characters by Jolt character id.
    pub(crate) characters: BTreeMap<u32, CharacterEntry>,
    /// The vehicles by vehicle id.
    pub(crate) vehicles: BTreeMap<u32, VehicleEntry>,
    /// Jolt's `CharacterVsCharacterCollisionSimple` of the characters that collide with each
    /// other, created with the first of them.
    pub(crate) character_collision: Option<Owned<JPH_CharacterVsCharacterCollision>>,
    pub(crate) system: Owned<JPH_PhysicsSystem>,
    job_system: Owned<JPH_JobSystem>,
    pub(crate) temp_allocator: Owned<JPH_TempAllocator>,
    pub(crate) body_interface: NonNull<JPH_BodyInterface>,
    pub(crate) body_lock_interface: NonNull<JPH_BodyLockInterface>,
    pub(crate) narrow_phase_query: NonNull<JPH_NarrowPhaseQuery>,
    pub(crate) broad_phase_query: NonNull<JPH_BroadPhaseQuery>,
    pub(crate) object_layer_count: u32,
    pub(crate) tag: WorldTag,
    /// Raw ids of the characters' inner bodies.
    pub(crate) inner_bodies: BTreeSet<u32>,
    /// The Jolt character id the next character gets; ids start at 1 and are never reused.
    pub(crate) next_character_id: u32,
    /// Raw ids of the vehicles' chassis bodies, with their vehicle ids.
    pub(crate) vehicle_bodies: BTreeMap<u32, u32>,
    /// The id the next vehicle gets; ids start at 1 and are never reused.
    pub(crate) next_vehicle_id: u32,
}

impl Drop for PhysicsWorld {
    fn drop(&mut self) {
        self.remove_all_vehicles();
    }
}

// SAFETY: the physics system, job system, temp allocator, characters and character collision
// set have no thread affinity. `step` and the character updates need `&mut self`, so the
// allocator and job system serve one call at a time
// (https://jrouwe.github.io/JoltPhysicsDocs/5.6.0/index.html#multi-threaded-access).
unsafe impl Send for PhysicsWorld {}
// SAFETY: every `&self` method only calls Jolt's locking body interface, read-only system
// getters, or Jolt's locking narrow-phase queries, which read bodies under body read locks and
// the broad phase under its query lock. Jolt allows all of these from several threads at once.
// Jolt forbids body access only while `PhysicsSystem::Update` runs, and `step` needs `&mut self`
// (https://jrouwe.github.io/JoltPhysicsDocs/5.6.0/index.html#multi-threaded-access). Query filter
// and result callbacks run on the querying thread, read the world only through the locking body
// interface and write only state on that query's stack. Character reads through `&self` are
// joltc getters over const Jolt members (`GetPosition`, `GetGroundState`,
// `GetActiveContacts().at()`, the const `SaveState`); every character change and update takes
// `&mut self`, and so does every use of `CharacterVsCharacterCollisionSimple`, which is not
// thread-safe (`CharacterVirtual.h`). Vehicle reads through `&self` are joltc getters over
// members of the vehicle constraint, its wheels and controller, which Jolt writes only during
// `step` and the vehicle setters, both behind `&mut self`.
unsafe impl Sync for PhysicsWorld {}

/// One body's pose and velocities in a frame.
#[derive(Clone, Copy, Debug, PartialEq)]
struct BodyFrameState {
    position: RVec3,
    rotation: Quat,
    linear_velocity: Vec3,
    angular_velocity: Vec3,
}

/// One character's pose, up and velocity in a frame.
#[derive(Clone, Copy, Debug, PartialEq)]
struct CharacterFrameState {
    position: RVec3,
    rotation: Quat,
    up: Vec3,
    linear_velocity: Vec3,
}

/// The rigid change of frame of a rebase: `p -> rotation * p + translation`.
#[derive(Clone, Copy, Debug)]
struct FrameChange {
    rotation: Quat,
    translation: RVec3,
}

impl FrameChange {
    fn rotates(&self) -> bool {
        self.rotation != Quat::IDENTITY
    }

    fn is_noop(&self) -> bool {
        !self.rotates() && self.translation == RVec3::ZERO
    }

    /// A direction or velocity in the new frame; unchanged, bit for bit, without rotation.
    fn vector(&self, v: Vec3) -> Vec3 {
        if self.rotates() {
            self.rotation.rotate(v)
        } else {
            v
        }
    }

    fn point(&self, p: RVec3) -> RVec3 {
        let p = if self.rotates() {
            self.rotation.rotate_real(p)
        } else {
            p
        };
        RVec3::new(
            p.x + self.translation.x,
            p.y + self.translation.y,
            p.z + self.translation.z,
        )
    }

    /// `state` in the new frame, or `None` when a value would not be finite or the rotation
    /// not a valid unit quaternion: the rule every pose and velocity setter applies.
    fn body(&self, state: BodyFrameState) -> Option<BodyFrameState> {
        let rotation = if self.rotates() {
            self.rotation.product(state.rotation).normalized()
        } else {
            state.rotation
        };
        let new = BodyFrameState {
            position: self.point(state.position),
            rotation,
            linear_velocity: self.vector(state.linear_velocity),
            angular_velocity: self.vector(state.angular_velocity),
        };
        let valid = new.position.is_finite()
            && new.rotation.is_valid_rotation()
            && new.linear_velocity.is_finite()
            && new.angular_velocity.is_finite();
        valid.then_some(new)
    }

    /// `state` in the new frame, or `None` when a value would not be finite or the rotation or
    /// up not of unit length: the rule every character setter applies.
    fn character(&self, state: CharacterFrameState) -> Option<CharacterFrameState> {
        let rotation = if self.rotates() {
            self.rotation.product(state.rotation).normalized()
        } else {
            state.rotation
        };
        let new = CharacterFrameState {
            position: self.point(state.position),
            rotation,
            up: self.vector(state.up),
            linear_velocity: self.vector(state.linear_velocity),
        };
        let unit_up = (new.up.dot(new.up) - 1.0).abs() <= 1.0e-5;
        let valid = new.position.is_finite()
            && new.rotation.is_valid_rotation()
            && new.up.is_finite()
            && unit_up
            && new.linear_velocity.is_finite();
        valid.then_some(new)
    }
}

impl PhysicsWorld {
    /// Largest time step [`step`](Self::step) accepts, in seconds, inclusive.
    ///
    /// A joltphysics guard against overflow-scale steps, which Jolt runs to NaN positions; not a
    /// Jolt limit and not a stability guarantee. Jolt recommends steps of about 1/60 s, and
    /// larger ones may tunnel or sag depending on the scene.
    pub const MAX_DELTA_TIME: f32 = 1.0;

    /// Creates a world. Nothing is allocated when the settings are invalid.
    pub fn new(settings: WorldSettings) -> Result<Self, WorldError> {
        settings.validate()?;
        if !ensure_initialized() {
            return Err(WorldError::InitFailed);
        }

        // SAFETY: Jolt is initialised; the size is positive (`validate`). The handle takes over
        // the returned allocator.
        let temp_allocator =
            unsafe { Owned::from_raw(JPH_TempAllocator_Create(settings.temp_allocator_size)) }
                .ok_or(WorldError::AllocationFailed("temp allocator"))?;

        let config = JobSystemThreadPoolConfig {
            maxJobs: 0,
            maxBarriers: 0,
            // `validate` bounds the count to `1..=MAX_WORKER_THREADS`; 0 would mean all hardware
            // threads.
            numThreads: settings.worker_threads as i32,
        };
        // SAFETY: Jolt is initialised; `config` is a live local. Zero job and barrier limits
        // select Jolt's `cMaxPhysicsJobs` and `cMaxPhysicsBarriers`. The handle takes over the
        // returned job system.
        let job_system = unsafe { Owned::from_raw(JPH_JobSystemThreadPool_Create(&config)) }
            .ok_or(WorldError::AllocationFailed("job system"))?;

        // SAFETY: Jolt is initialised and `validate` checked the layers.
        let tables = unsafe { settings.layers.create_tables() }?;
        let (broad_phase, pair_filter, object_vs_broad_phase) = tables.as_raw();
        let system_settings = JPH_PhysicsSystemSettings {
            maxBodies: settings.max_bodies,
            numBodyMutexes: 0,
            maxBodyPairs: settings.max_body_pairs,
            maxContactConstraints: settings.max_contact_constraints,
            _padding: 0,
            broadPhaseLayerInterface: broad_phase,
            objectLayerPairFilter: pair_filter,
            objectVsBroadPhaseLayerFilter: object_vs_broad_phase,
        };
        let system = {
            let _globals = lock_joltc_globals();
            // SAFETY: Jolt is initialised, the globals lock is held, and the three tables are
            // live and consistent. On success the system owns the tables and the handle owns the
            // system.
            unsafe { Owned::from_raw(JPH_PhysicsSystem_Create(&system_settings)) }
        };
        let Some(system) = system else {
            return Err(WorldError::AllocationFailed("physics system"));
        };
        tables.forget();

        let gravity = settings.gravity.to_jph();
        // SAFETY: `system` is live and `gravity` is a live local.
        unsafe { JPH_PhysicsSystem_SetGravity(system.as_ptr(), &gravity) };

        // SAFETY: `system` is live. The interfaces and the narrow-phase and broad-phase queries
        // live inside the Jolt system and stay valid as long as it does; the world stores them
        // next to the system that owns them.
        let (body_interface, body_lock_interface, narrow_phase_query, broad_phase_query) = unsafe {
            (
                JPH_PhysicsSystem_GetBodyInterface(system.as_ptr()),
                JPH_PhysicsSystem_GetBodyLockInterface(system.as_ptr()),
                JPH_PhysicsSystem_GetNarrowPhaseQuery(system.as_ptr()),
                JPH_PhysicsSystem_GetBroadPhaseQuery(system.as_ptr()),
            )
        };
        let body_interface =
            NonNull::new(body_interface).ok_or(WorldError::AllocationFailed("body interface"))?;
        let body_lock_interface = NonNull::new(body_lock_interface.cast_mut())
            .ok_or(WorldError::AllocationFailed("body lock interface"))?;
        let narrow_phase_query = NonNull::new(narrow_phase_query.cast_mut())
            .ok_or(WorldError::AllocationFailed("narrow phase query"))?;
        let broad_phase_query = NonNull::new(broad_phase_query.cast_mut())
            .ok_or(WorldError::AllocationFailed("broad phase query"))?;

        Ok(Self {
            characters: BTreeMap::new(),
            vehicles: BTreeMap::new(),
            character_collision: None,
            system,
            job_system,
            temp_allocator,
            body_interface,
            body_lock_interface,
            narrow_phase_query,
            broad_phase_query,
            object_layer_count: settings.layers.object_layer_count(),
            tag: WorldTag::next(),
            inner_bodies: BTreeSet::new(),
            next_character_id: 1,
            vehicle_bodies: BTreeMap::new(),
            next_vehicle_id: 1,
        })
    }

    /// Gravity in m/s².
    pub fn gravity(&self) -> Vec3 {
        let mut gravity = Vec3::ZERO.to_jph();
        // SAFETY: the system is live; reading gravity does not change it, and writes need
        // `&mut self`. `gravity` is a live local.
        unsafe { JPH_PhysicsSystem_GetGravity(self.system.as_ptr(), &mut gravity) };
        Vec3::from_jph(gravity)
    }

    /// Sets gravity in m/s². It must be finite; zero is allowed. Sleeping bodies stay asleep.
    pub fn set_gravity(&mut self, gravity: Vec3) -> Result<(), WorldError> {
        if !gravity.is_finite() {
            return Err(WorldError::InvalidSettings("gravity must be finite"));
        }
        let gravity = gravity.to_jph();
        // SAFETY: the system is live and borrowed mutably; `gravity` is a live local.
        unsafe { JPH_PhysicsSystem_SetGravity(self.system.as_ptr(), &gravity) };
        Ok(())
    }

    /// Number of bodies in the world.
    pub fn body_count(&self) -> u32 {
        // SAFETY: the system is live; Jolt counts under its own body mutex.
        unsafe { JPH_PhysicsSystem_GetNumBodies(self.system.as_ptr()) }
    }

    /// Rebuilds the broad phase's trees for fast queries.
    ///
    /// Queries see bodies created, moved and removed through this API immediately, without a
    /// step or a call to this method, and this method never changes which bodies a query finds.
    /// Bodies added one by one (a batch of chunks, say) leave the trees unbalanced, which makes
    /// queries slower until the next steps have rebuilt them; call this after such a batch to
    /// make queries fast at once. It is the explicit refresh point of the world: call it at the
    /// same point of every run, because it is part of the call history that determinism
    /// depends on.
    pub fn optimize_broad_phase(&mut self) {
        // SAFETY: the system is live and borrowed mutably, so no query or body change runs
        // meanwhile, as Jolt requires (`PhysicsSystem::OptimizeBroadPhase`).
        unsafe { JPH_PhysicsSystem_OptimizeBroadPhase(self.system.as_ptr()) };
    }

    /// Moves the whole world into a new frame: one rigid change of coordinates, for a floating
    /// origin.
    ///
    /// Every body origin `p` becomes `rotation * p + translation` (metres), every body rotation
    /// `q` becomes `rotation * q`, linear and angular velocities `v` become `rotation * v`
    /// (m/s, rad/s), and so does the world's gravity.
    ///
    /// `bodies_in_key_order` must name every body of the world exactly once, in the caller's
    /// stable key order, which is the order the poses are written in; otherwise
    /// [`BodyError::InvalidValue`], [`BodyError::WrongWorld`] or [`BodyError::NotFound`] is
    /// returned. `rotation` must be a finite unit quaternion and `translation` finite, and no
    /// new pose, velocity or gravity may overflow; otherwise [`BodyError::InvalidValue`] is
    /// returned. Every check runs before the first write, so an error leaves the world
    /// unchanged.
    ///
    /// No body is woken or put to sleep. An identity `rotation` leaves rotations, velocities
    /// and gravity untouched, bit for bit; an identity rotation with a zero translation changes
    /// nothing. Awake bodies restart Jolt's sleep timer, as for every pose change, so they may
    /// fall asleep later than without the rebase.
    ///
    /// Forces and torques added since the last step are not rotated: rebase between steps,
    /// before adding the tick's forces. Queries see the new poses at once;
    /// [`optimize_broad_phase`](Self::optimize_broad_phase) afterwards is optional and only
    /// makes queries faster until the next step. Jolt caches contacts relative to the bodies,
    /// so bodies at rest keep their contacts on the next step.
    ///
    /// Characters move with the world, in id order after the bodies: position and rotation as
    /// for bodies, up and linear velocity as vectors. The list must still name their inner
    /// bodies, which are bodies of the world. The contacts and ground a character cached in its
    /// last update stay in the old frame. A translation needs nothing more, because the next
    /// update reads only cached normals and velocities; after a rotation call
    /// [`refresh_character_contacts`](Self::refresh_character_contacts) for every character
    /// before its next update.
    ///
    /// Vehicles move with their chassis, which the list names as bodies. A rotation also
    /// rotates each vehicle's gravity override and the world-space up of a ray or sphere
    /// collision tester, in id order after the characters; a translation changes neither. The
    /// wheel contacts a vehicle reports stay in the old frame until the next step tests the
    /// wheels again, and the world up of the pitch and roll limit follows the rotated gravity on
    /// that step.
    pub fn rebase(
        &mut self,
        bodies_in_key_order: &[BodyId],
        rotation: Quat,
        translation: RVec3,
    ) -> Result<(), BodyError> {
        let invalid = |what| Err(BodyError::InvalidValue(what));
        if !rotation.is_valid_rotation() {
            return invalid("rebase rotation must be a finite unit quaternion");
        }
        if !translation.is_finite() {
            return invalid("rebase translation must be finite");
        }
        let frame = FrameChange {
            rotation,
            translation,
        };

        let mut changes = Vec::with_capacity(bodies_in_key_order.len());
        for &id in bodies_in_key_order {
            let body = self.body(id)?;
            let old = BodyFrameState {
                position: body.position(),
                rotation: body.rotation(),
                linear_velocity: body.linear_velocity(),
                angular_velocity: body.angular_velocity(),
            };
            let Some(new) = frame.body(old) else {
                return invalid("rebase would give a body a non-finite pose or velocity");
            };
            changes.push((id, body.motion_type(), old, new));
        }
        let mut raw_ids: Vec<u32> = bodies_in_key_order.iter().map(|id| id.to_raw()).collect();
        raw_ids.sort_unstable();
        if raw_ids.windows(2).any(|pair| pair[0] == pair[1]) {
            return invalid("rebase body list names a body twice");
        }
        if bodies_in_key_order.len() != self.body_count() as usize {
            return invalid("rebase body list must name every body of the world");
        }
        let gravity = frame.vector(self.gravity());
        if !gravity.is_finite() {
            return invalid("rebase would give the world a non-finite gravity");
        }
        if frame.is_noop() {
            return Ok(());
        }
        let mut characters = Vec::with_capacity(self.characters.len());
        for id in self.character_ids().collect::<Vec<_>>() {
            let character = self
                .character(id)
                .unwrap_or_else(|_| unreachable!("listed by the world"));
            let old = CharacterFrameState {
                position: character.position(),
                rotation: character.rotation(),
                up: character.up(),
                linear_velocity: character.linear_velocity(),
            };
            let Some(new) = frame.character(old) else {
                return invalid("rebase would give a character a non-finite pose or velocity");
            };
            characters.push((id, new));
        }
        let vehicles = if frame.rotates() {
            self.rotated_vehicles(|v| frame.vector(v))
                .map_err(BodyError::InvalidValue)?
        } else {
            Vec::new()
        };

        for (id, motion_type, old, new) in changes {
            let position = new.position.to_jph();
            let rotation = new.rotation.to_jph();
            // SAFETY: the body interface belongs to this live world, borrowed mutably; the
            // checks above found `id` in this world and `&mut self` keeps it there. `position`
            // and `rotation` are live locals, finite and a unit quaternion. This thread holds no
            // body lock.
            unsafe {
                JPH_BodyInterface_SetPositionAndRotation(
                    self.body_interface.as_ptr(),
                    id.to_raw(),
                    &position,
                    &rotation,
                    JPH_Activation_DontActivate,
                )
            };
            let moving = old.linear_velocity != Vec3::ZERO || old.angular_velocity != Vec3::ZERO;
            if frame.rotates() && motion_type != MotionType::Static && moving {
                let linear = new.linear_velocity.to_jph();
                let angular = new.angular_velocity.to_jph();
                // `id` resolves (checked above), so the closure runs.
                with_locked_body(self.body_lock_interface, id, |body| {
                    // SAFETY: `body` is locked for writing for the duration of the closure and
                    // is not static, as Jolt's velocity setters assert. The clamped setters
                    // write the motion properties only and never activate the body.
                    // `linear` and `angular` are live locals.
                    unsafe {
                        if !JPH_Body_IsStatic(body.as_ptr()) {
                            JPH_Body_SetLinearVelocityClamped(body.as_ptr(), &linear);
                            JPH_Body_SetAngularVelocityClamped(body.as_ptr(), &angular);
                        }
                    }
                });
            }
        }
        // After the bodies: the character setters place each inner body absolutely at the
        // character's new pose, which is the pose the body loop gave it. Up goes first, because
        // the inner body's position includes the padding along up.
        for (id, new) in characters {
            let mut character = self
                .character_mut(id)
                .unwrap_or_else(|_| unreachable!("listed by the world"));
            let written = character
                .set_up(new.up)
                .and_then(|()| character.set_position(new.position))
                .and_then(|()| character.set_rotation(new.rotation))
                .and_then(|()| character.set_linear_velocity(new.linear_velocity));
            debug_assert_eq!(written, Ok(()), "checked by `FrameChange::character`");
        }
        self.apply_vehicle_rebase(vehicles);
        if frame.rotates() {
            let gravity = gravity.to_jph();
            // SAFETY: the system is live and borrowed mutably; `gravity` is a live local.
            unsafe { JPH_PhysicsSystem_SetGravity(self.system.as_ptr(), &gravity) };
        }
        Ok(())
    }

    /// Advances the world by `delta_time` seconds in one collision step.
    ///
    /// `delta_time` must be finite, positive and at most
    /// [`MAX_DELTA_TIME`](Self::MAX_DELTA_TIME), otherwise nothing happens and
    /// [`StepError::InvalidDeltaTime`] is returned. Every other call advances the world and
    /// returns a [`StepReport`]; check [`StepReport::is_complete`] to learn whether Jolt
    /// dropped work because a fixed-size buffer was full.
    pub fn step(&mut self, delta_time: f32) -> Result<StepReport, StepError> {
        if !(is_finite_positive(delta_time) && delta_time <= Self::MAX_DELTA_TIME) {
            return Err(StepError::InvalidDeltaTime);
        }
        // SAFETY: the system, temp allocator and job system are live and owned by this world;
        // `&mut self` guarantees no other call uses them or touches a body during the update.
        let errors = unsafe {
            JPH_PhysicsSystem_Update2(
                self.system.as_ptr(),
                delta_time,
                1,
                self.temp_allocator.as_ptr(),
                self.job_system.as_ptr(),
            )
        };
        Ok(StepReport {
            manifold_cache_full: errors & JPH_PhysicsUpdateError_ManifoldCacheFull != 0,
            body_pair_cache_full: errors & JPH_PhysicsUpdateError_BodyPairCacheFull != 0,
            contact_constraints_full: errors & JPH_PhysicsUpdateError_ContactConstraintsFull != 0,
        })
    }
}

/// What a [`PhysicsWorld::step`] that ran reports. The world has advanced either way.
///
/// When a fixed-size buffer was full, Jolt finished the step but ignored some contacts; the
/// flags say which buffer, and the matching [`WorldSettings`] limit should be raised.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[must_use = "a step may have dropped contacts; check `is_complete`"]
#[non_exhaustive]
pub struct StepReport {
    /// The contact manifold cache was full: too many contacts between bodies. Raise
    /// [`WorldSettings::max_contact_constraints`].
    pub manifold_cache_full: bool,
    /// The body pair cache was full: too many bodies touched. Raise
    /// [`WorldSettings::max_body_pairs`].
    pub body_pair_cache_full: bool,
    /// The contact constraint buffer was full. Raise
    /// [`WorldSettings::max_contact_constraints`].
    pub contact_constraints_full: bool,
}

impl StepReport {
    /// Whether the step ran without dropping any work.
    pub fn is_complete(&self) -> bool {
        !(self.manifold_cache_full || self.body_pair_cache_full || self.contact_constraints_full)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Real;

    #[test]
    fn any_full_buffer_makes_a_report_incomplete() {
        let complete = StepReport {
            manifold_cache_full: false,
            body_pair_cache_full: false,
            contact_constraints_full: false,
        };
        assert!(complete.is_complete());
        for report in [
            StepReport {
                manifold_cache_full: true,
                ..complete
            },
            StepReport {
                body_pair_cache_full: true,
                ..complete
            },
            StepReport {
                contact_constraints_full: true,
                ..complete
            },
        ] {
            assert!(!report.is_complete(), "{report:?}");
        }
    }

    fn moving_state() -> BodyFrameState {
        BodyFrameState {
            position: RVec3::new(1.0, 0.0, 0.0),
            rotation: Quat::from_xyzw(0.0, 0.0, 0.6, 0.8),
            linear_velocity: Vec3::new(1.0, 0.0, 0.0),
            angular_velocity: Vec3::new(1.0, 0.0, 0.0),
        }
    }

    #[test]
    fn translation_only_frame_change_keeps_rotation_and_velocity_bits() {
        let frame = FrameChange {
            rotation: Quat::IDENTITY,
            translation: RVec3::new(1.0, 2.0, 3.0),
        };
        let state = moving_state();
        let moved = frame.body(state).unwrap();
        assert_eq!(moved.position, RVec3::new(2.0, 2.0, 3.0));
        let bits = |q: Quat| <[f32; 4]>::from(q).map(f32::to_bits);
        assert_eq!(bits(moved.rotation), bits(state.rotation));
        assert_eq!(moved.linear_velocity, state.linear_velocity);
        assert_eq!(moved.angular_velocity, state.angular_velocity);
    }

    #[test]
    fn quarter_turn_about_y_maps_x_to_minus_z() {
        let half = std::f32::consts::FRAC_1_SQRT_2;
        let frame = FrameChange {
            rotation: Quat::from_xyzw(0.0, half, 0.0, half),
            translation: RVec3::ZERO,
        };
        let moved = frame.body(moving_state()).unwrap();
        let p = moved.position;
        let near =
            |a: [f32; 3]| (a[0].abs() < 1e-6) && (a[1].abs() < 1e-6) && ((a[2] + 1.0).abs() < 1e-6);
        assert!(p.x.abs() < 1e-6 && p.y.abs() < 1e-6, "{moved:?}");
        assert!((p.z + 1.0).abs() < 1e-6, "{moved:?}");
        assert!(near(moved.linear_velocity.into()), "{moved:?}");
        assert!(near(moved.angular_velocity.into()), "{moved:?}");
        assert!(moved.rotation.is_valid_rotation(), "{moved:?}");
    }

    #[test]
    fn frame_change_rejects_overflow_and_nan() {
        let frame = FrameChange {
            rotation: Quat::IDENTITY,
            translation: RVec3::new(Real::MAX, 0.0, 0.0),
        };
        let mut state = moving_state();
        state.position.x = Real::MAX / 2.0;
        assert_eq!(frame.body(state), None);

        let frame = FrameChange {
            rotation: Quat::IDENTITY,
            translation: RVec3::ZERO,
        };
        let mut state = moving_state();
        state.linear_velocity.y = f32::NAN;
        assert_eq!(frame.body(state), None);
    }

    #[test]
    fn worker_thread_bounds_are_validated() {
        for valid in [1, WorldSettings::MAX_WORKER_THREADS] {
            assert_eq!(
                WorldSettings::default().worker_threads(valid).validate(),
                Ok(())
            );
        }
        for invalid in [0, WorldSettings::MAX_WORKER_THREADS + 1, u32::MAX] {
            assert!(matches!(
                WorldSettings::default().worker_threads(invalid).validate(),
                Err(WorldError::InvalidSettings(_))
            ));
        }
    }
}
