//! The physics world: a Jolt physics system with its own job system and temp allocator. The job
//! system is Jolt's thread pool or a native object that hands jobs to the caller's
//! [`JobSystem`].
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
//! the world's own temp allocator through the `oxijolt-sys` extension
//! (`JPH_CharacterVirtual_ExtendedUpdate2`, `JPH_CharacterVirtual_RefreshContacts2`), which
//! needs `&mut self`. Callbacks that run inside a step must not use the locking body interface,
//! which would deadlock. The event listeners and their callbacks live in the `listener` module
//! ([`PhysicsWorld::set_event_settings`]).

use std::any::Any;
use std::collections::{BTreeMap, BTreeSet};
use std::fmt;
use std::num::NonZeroU64;
use std::ptr::NonNull;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, OnceLock, PoisonError};

use oxijolt_sys::*;

use crate::character::CharacterEntry;
use crate::constraint::ConstraintEntry;
use crate::job_system::{create_caller_job_system, QueueContext};
use crate::jolt_assert;
use crate::limits;
use crate::listener::{first_payload, Listeners};
use crate::owned::{JoltObject, Owned};
use crate::ragdoll::RagdollEntry;
use crate::vehicle::VehicleEntry;
use crate::{CollisionLayers, JobSystem, StepError, Vec3, WorldError};

mod rebase;

/// Installs the assertion handler and runs `JPH_Init` once per process, and returns whether
/// `JPH_Init` succeeded. oxijolt never calls `JPH_Shutdown`: Jolt's global state lives as
/// long as the process.
pub(crate) fn ensure_initialized() -> bool {
    static INIT: OnceLock<bool> = OnceLock::new();
    *INIT.get_or_init(|| {
        jolt_assert::install();
        // SAFETY: `OnceLock` runs the closure exactly once per process, which is the only
        // synchronisation `JPH_Init` (an unsynchronised `bool` guard) needs. The assertion
        // handler is in place before `JPH_Init` registers Jolt's types.
        unsafe { JPH_Init() }
    })
}

/// Serialises `JPH_PhysicsSystem_Create` and `JPH_PhysicsSystem_Destroy`, which write joltc's
/// unsynchronised global map of physics systems.
fn lock_joltc_globals() -> MutexGuard<'static, ()> {
    static GLOBALS: Mutex<()> = Mutex::new(());
    GLOBALS.lock().unwrap_or_else(PoisonError::into_inner)
}

/// Which job system a world runs its jobs on.
#[derive(Clone)]
enum JobSystemChoice {
    /// Jolt's thread pool with this many worker threads.
    ThreadPool(u32),
    /// The caller's job system.
    Caller(Arc<dyn JobSystem>),
}

impl fmt::Debug for JobSystemChoice {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ThreadPool(workers) => f.debug_tuple("ThreadPool").field(workers).finish(),
            Self::Caller(_) => f.write_str("Caller(JobSystem)"),
        }
    }
}

impl JobSystemChoice {
    /// The most jobs Jolt will run at the same time, counting the stepping thread. Reads the
    /// caller's [`JobSystem::max_concurrency`] once and checks it.
    fn max_concurrency(&self) -> Result<u32, WorldError> {
        match self {
            Self::ThreadPool(workers) => Ok(workers + 1),
            Self::Caller(job_system) => {
                let value = job_system.max_concurrency();
                if (1..=WorldSettings::MAX_CONCURRENCY).contains(&value) {
                    Ok(value)
                } else {
                    Err(WorldError::InvalidSettings(
                        "job system max_concurrency must be between 1 and 65",
                    ))
                }
            }
        }
    }
}

/// Settings for [`PhysicsWorld::new`]. The defaults are Jolt's and joltc's.
#[derive(Clone, Debug)]
pub struct WorldSettings {
    max_bodies: u32,
    max_body_pairs: u32,
    max_contact_constraints: u32,
    jobs: JobSystemChoice,
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
            jobs: JobSystemChoice::ThreadPool(1),
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
    /// thread per worker; this bound is chosen by oxijolt to keep thread creation sane and
    /// is not a Jolt limit.
    pub const MAX_WORKER_THREADS: u32 = 64;

    /// Largest accepted [`JobSystem::max_concurrency`] of a [`job_system`](Self::job_system):
    /// the concurrency of Jolt's thread pool at [`MAX_WORKER_THREADS`](Self::MAX_WORKER_THREADS)
    /// (the workers and the stepping thread). A oxijolt bound, not a Jolt limit: Jolt splits
    /// a stage into at most 32 jobs, and 0 or a value above `i32::MAX` would break its `int`
    /// arithmetic.
    pub const MAX_CONCURRENCY: u32 = Self::MAX_WORKER_THREADS + 1;

    /// Largest accepted [`max_contact_constraints`](Self::max_contact_constraints) value: 2²⁰.
    ///
    /// A oxijolt bound below Jolt's own limit (`ContactConstraintManager::
    /// cMaxContactConstraintsLimit`, above which Jolt asserts), which a native compile-time check
    /// pins.
    pub const MAX_CONTACT_CONSTRAINTS: u32 = 1 << 20;

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

    /// Maximum number of contact constraints per step, at least 1 and at most
    /// [`WorldSettings::MAX_CONTACT_CONSTRAINTS`]. Default 10240.
    #[must_use]
    pub fn max_contact_constraints(mut self, value: u32) -> Self {
        self.max_contact_constraints = value;
        self
    }

    /// Runs the world's jobs on Jolt's thread pool with `value` worker threads, in addition to
    /// the thread that calls [`PhysicsWorld::step`], which also runs jobs. At least 1 and at most
    /// [`WorldSettings::MAX_WORKER_THREADS`]. Default 1. The determinism tests get bit-identical
    /// results with 1 and 4 workers on one machine ([docs/determinism.md]).
    ///
    /// Of this and [`job_system`](Self::job_system), the call made last decides.
    ///
    /// [docs/determinism.md]: https://github.com/pockerhead/oxijolt/blob/main/docs/determinism.md
    #[must_use]
    pub fn worker_threads(mut self, value: u32) -> Self {
        self.jobs = JobSystemChoice::ThreadPool(value);
        self
    }

    /// Runs the world's jobs on the caller's `value` instead of Jolt's thread pool; Jolt then
    /// starts no threads for the world. The world keeps the `Arc` as long as it lives, and
    /// [`PhysicsWorld::new`] reads [`JobSystem::max_concurrency`] once, which must be within
    /// `1..=`[`WorldSettings::MAX_CONCURRENCY`].
    ///
    /// Of this and [`worker_threads`](Self::worker_threads), the call made last decides.
    #[must_use]
    pub fn job_system(mut self, value: Arc<dyn JobSystem>) -> Self {
        self.jobs = JobSystemChoice::Caller(value);
        self
    }

    /// Size in bytes of the scratch memory the world's steps allocate from. Default 10 MiB.
    #[must_use]
    pub fn temp_allocator_size(mut self, value: u32) -> Self {
        self.temp_allocator_size = value;
        self
    }

    /// Gravity in m/s², finite and at most [`limits::MAX_ACCELERATION`] long. Default
    /// `(0, -9.81, 0)`, Jolt's default. Zero is allowed.
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
        if !(1..=Self::MAX_CONTACT_CONSTRAINTS).contains(&self.max_contact_constraints) {
            return invalid("max_contact_constraints must be between 1 and 2^20");
        }
        if let JobSystemChoice::ThreadPool(workers) = self.jobs {
            if !(1..=Self::MAX_WORKER_THREADS).contains(&workers) {
                return invalid("worker_threads must be between 1 and 64");
            }
        }
        if self.temp_allocator_size == 0 {
            return invalid("temp_allocator_size must be at least 1");
        }
        if !limits::is_acceleration(self.gravity) {
            return invalid(limits::GRAVITY_RULE);
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

/// A world's job system: Jolt's thread pool, owned by the world, or joltc's callback job system
/// for the caller's [`JobSystem`], owned by the world and by every job still queued.
impl JoltObject for JPH_JobSystem {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner owns the job system (trait contract), and no step is running: `step`
        // borrows the world mutably, and the last owner of a callback job system holds no job
        // that Jolt could still run. Destroying a thread pool joins its worker threads; a
        // callback job system has no threads and may be destroyed on any thread, also on one of
        // the caller's pool threads.
        unsafe { JPH_JobSystem_Destroy(ptr) };
    }
}

/// The job system a world steps with.
enum WorldJobSystem {
    /// Jolt's thread pool.
    ThreadPool(Owned<JPH_JobSystem>),
    /// joltc's callback job system, which hands jobs to the caller's [`JobSystem`].
    Caller(Arc<QueueContext>),
}

impl WorldJobSystem {
    /// Creates Jolt's thread pool with `workers` worker threads, already validated.
    fn thread_pool(workers: u32) -> Result<Self, WorldError> {
        let config = JobSystemThreadPoolConfig {
            maxJobs: 0,
            maxBarriers: 0,
            // `validate` bounds the count to `1..=MAX_WORKER_THREADS`; 0 would mean all hardware
            // threads.
            numThreads: workers as i32,
        };
        // SAFETY: Jolt is initialised; `config` is a live local. Zero job and barrier limits
        // select Jolt's `cMaxPhysicsJobs` and `cMaxPhysicsBarriers`. The handle takes over the
        // returned job system.
        let pool = unsafe { Owned::from_raw(JPH_JobSystemThreadPool_Create(&config)) }
            .ok_or(WorldError::AllocationFailed("job system"))?;
        Ok(Self::ThreadPool(pool))
    }

    fn as_ptr(&self) -> *mut JPH_JobSystem {
        match self {
            Self::ThreadPool(pool) => pool.as_ptr(),
            Self::Caller(context) => context.as_ptr(),
        }
    }

    /// Finishes a step after the update returned: releases the jobs a caller job system left to
    /// the stepping thread and returns the first panic of its `queue_job` during the step.
    fn finish_update(&self) -> Option<Box<dyn Any + Send>> {
        match self {
            Self::ThreadPool(_) => None,
            Self::Caller(context) => context.finish_update(),
        }
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
    pub(crate) fn next() -> Self {
        static NEXT_WORLD: AtomicU64 = AtomicU64::new(1);
        let value = NEXT_WORLD.fetch_add(1, Ordering::Relaxed);
        Self(NonZeroU64::new(value).expect("world counter overflowed"))
    }
}

/// Advances a world's structure epoch ([`PhysicsWorld::note_structure_change`]), for views that
/// borrow only part of the world. Overflow cannot happen in practice and fails closed.
pub(crate) fn advance_structure_epoch(epoch: &mut u64) {
    *epoch = epoch.checked_add(1).expect("structure epoch overflowed");
}

/// A Jolt physics system with its collision layers, its own job system and temp allocator.
///
/// The job system is Jolt's thread pool, or a native one that hands the jobs to the caller's
/// [`JobSystem`] ([`WorldSettings::job_system`]).
///
/// Changing the world, including [`step`](Self::step), takes `&mut self`; reading takes
/// `&self`, so many threads can read one world while nobody steps it. Several worlds are
/// independent and may be stepped on different threads at the same time.
///
/// Bodies added one at a time leave the broad phase unoptimised, which makes queries slower
/// until Jolt has rebuilt it during steps (Jolt docs, "Bodies"); see
/// [`optimize_broad_phase`](Self::optimize_broad_phase).
pub struct PhysicsWorld {
    // Field order is drop order, after `Drop for PhysicsWorld` has taken every constraint out of
    // the system and released it, then every vehicle out of the system's step listeners and
    // constraints and every ragdoll out of the system, so `constraints`, `vehicles` and
    // `ragdolls` are empty by then. The characters go first: each destructor removes its inner
    // body through the still-live system. The character collision set follows; it only frees
    // its list of character pointers. The system goes next, before the job system and allocator
    // its steps used, and deletes the layer tables it owns. A caller job system's native object
    // may outlive the world while jobs the caller's pool has not run or dropped yet hold it. The
    // interface and query pointers after them are borrowed from the system and have no
    // destructor.
    /// The characters by Jolt character id.
    pub(crate) characters: BTreeMap<u32, CharacterEntry>,
    /// The vehicles by vehicle id.
    pub(crate) vehicles: BTreeMap<u32, VehicleEntry>,
    /// The ragdolls by ragdoll id.
    pub(crate) ragdolls: BTreeMap<u32, RagdollEntry>,
    /// The constraints by constraint id.
    pub(crate) constraints: BTreeMap<u32, ConstraintEntry>,
    /// Jolt's `CharacterVsCharacterCollisionSimple` of the characters that collide with each
    /// other, created with the first of them.
    pub(crate) character_collision: Option<Owned<JPH_CharacterVsCharacterCollision>>,
    pub(crate) system: Owned<JPH_PhysicsSystem>,
    /// The contact, activation and soft body listeners, detached first in `Drop` and destroyed
    /// after the system.
    pub(crate) listeners: Listeners,
    job_system: WorldJobSystem,
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
    /// Raw ids of the ragdolls' part bodies, with their ragdoll ids.
    pub(crate) ragdoll_bodies: BTreeMap<u32, u32>,
    /// The id the next ragdoll gets; ids start at 1 and are never reused.
    pub(crate) next_ragdoll_id: u32,
    /// Raw ids of the bodies constraints use, with the ids of those constraints.
    pub(crate) constraint_bodies: BTreeMap<u32, BTreeSet<u32>>,
    /// The id the next constraint gets; ids start at 1 and are never reused.
    pub(crate) next_constraint_id: u32,
    /// Counts the changes Jolt's saved state cannot express (the body, character, vehicle,
    /// ragdoll and constraint sets, the BodyID allocator, motion types, body shapes, rebases); a
    /// [`WorldState`](crate::WorldState) restores only at the epoch it was saved at.
    pub(crate) structure_epoch: u64,
}

impl Drop for PhysicsWorld {
    fn drop(&mut self) {
        // First: removing constraints, ragdolls and characters' inner bodies deactivates bodies,
        // which must not reach a listener any more.
        // SAFETY: the system is this world's and no step runs during `drop`.
        unsafe { self.listeners.detach(self.system.as_ptr()) };
        self.remove_all_constraints();
        self.remove_all_vehicles();
        self.remove_all_ragdolls();
    }
}

// SAFETY: the physics system, job system, temp allocator, characters and character collision
// set have no thread affinity. `step` and the character updates need `&mut self`, so the
// allocator and job system serve one call at a time
// (https://jrouwe.github.io/JoltPhysicsDocs/5.6.0/index.html#multi-threaded-access). A caller
// job system is `Send + Sync` by the `JobSystem` bound, and its native callback object is
// `Send + Sync` (see `CallbackJobSystem`).
unsafe impl Send for PhysicsWorld {}
// SAFETY: no `&self` method touches the job system. Every `&self` method only calls Jolt's
// locking body interface, read-only system
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
// `step` and the vehicle setters, both behind `&mut self`. Ragdoll reads through `&self` use the
// locking body interface and constraint getters that read constraint members and the bodies'
// rotations, which Jolt writes only in `step` and the ragdoll and body setters, all behind
// `&mut self`. Constraint reads through `&self` are joltc getters over constraint members and the
// bodies' transforms, which Jolt writes only in `step` and the `&mut` constraint and body
// setters. `save_state` and `save_state_of` call `PhysicsSystem::SaveState`, which is const:
// `BodyManager::SaveState` takes every body lock (`LockAllBodies`, in mutex-array order, so
// concurrent saves do not deadlock), `ConstraintManager::SaveState` takes its mutex, and the
// contact cache it reads is written only by `step` and `restore_state`, both behind `&mut self`.
// The characters' `SaveState` is const as well. Soft body reads through `&self` copy the vertices
// under Jolt's body read lock; vertices are written only by `step` and the `&mut` soft body and
// body setters. `active_body_poses` copies the active body lists under Jolt's active-list mutex,
// releases it, and then reads the poses under one multi-body read lock, which takes body mutexes
// shared in mutex-array order. A concurrent `save_state` takes them exclusively in the same order,
// so the two may wait for each other but cannot deadlock. Event callbacks write the listener context only while Jolt steps, activates,
// deactivates or removes bodies, which happens only in `&mut self` methods: no `&self` method
// steps, adds, removes or wakes a body, and `event_settings` only reads the context's immutable
// settings.
unsafe impl Sync for PhysicsWorld {}

/// What a step length must satisfy ([`PhysicsWorld::is_valid_delta_time`]).
pub(crate) const DELTA_TIME_RULE: &str =
    "delta time must be finite and between MIN_DELTA_TIME and MAX_DELTA_TIME";

impl PhysicsWorld {
    /// Largest time step [`step`](Self::step) accepts, in seconds, inclusive.
    ///
    /// A oxijolt guard against overflow-scale steps, which Jolt runs to NaN positions; not a
    /// Jolt limit and not a stability guarantee. Jolt recommends steps of about 1/60 s, and
    /// larger ones may tunnel or sag depending on the scene.
    pub const MAX_DELTA_TIME: f32 = 1.0;

    /// Smallest time step [`step`](Self::step) accepts, in seconds, inclusive.
    ///
    /// A oxijolt guard, not a Jolt limit. Jolt divides by the step: a kinematic body's
    /// velocity is its move over the step (`MoveKinematic`), a character's velocities are
    /// derived the same way, and a wheel's brake-lock torque is `|ω| · inertia / step`
    /// (`WheeledVehicleController::PostCollide`). A subnormal step makes these infinite. The
    /// bound keeps the divisor away from subnormal values; a huge numerator can still overflow a
    /// quotient, and the bound does not limit every force or velocity a step can produce.
    pub const MIN_DELTA_TIME: f32 = 1.0e-6;

    /// Whether `delta_time` is finite and within `MIN_DELTA_TIME..=MAX_DELTA_TIME`.
    pub(crate) fn is_valid_delta_time(delta_time: f32) -> bool {
        delta_time.is_finite()
            && (Self::MIN_DELTA_TIME..=Self::MAX_DELTA_TIME).contains(&delta_time)
    }

    /// Creates a world. Nothing is allocated when the settings are invalid.
    ///
    /// With a caller [`JobSystem`], its [`max_concurrency`](JobSystem::max_concurrency) is read
    /// once, before anything is allocated.
    pub fn new(settings: WorldSettings) -> Result<Self, WorldError> {
        settings.validate()?;
        let max_concurrency = settings.jobs.max_concurrency()?;
        if !ensure_initialized() {
            return Err(WorldError::InitFailed);
        }

        // SAFETY: Jolt is initialised; the size is positive (`validate`). The handle takes over
        // the returned allocator.
        let temp_allocator =
            unsafe { Owned::from_raw(JPH_TempAllocator_Create(settings.temp_allocator_size)) }
                .ok_or(WorldError::AllocationFailed("temp allocator"))?;

        let job_system = match &settings.jobs {
            JobSystemChoice::ThreadPool(workers) => WorldJobSystem::thread_pool(*workers)?,
            JobSystemChoice::Caller(job_system) => WorldJobSystem::Caller(
                create_caller_job_system(Arc::clone(job_system), max_concurrency)?,
            ),
        };

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
            ragdolls: BTreeMap::new(),
            constraints: BTreeMap::new(),
            character_collision: None,
            system,
            listeners: Listeners::default(),
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
            ragdoll_bodies: BTreeMap::new(),
            next_ragdoll_id: 1,
            constraint_bodies: BTreeMap::new(),
            next_constraint_id: 1,
            structure_epoch: 0,
        })
    }

    /// Records a change that a [`WorldState`](crate::WorldState) cannot undo, so states saved
    /// before it are refused. Every API that adds or removes a Jolt object (body, character,
    /// vehicle, ragdoll, constraint) or changes a motion type or a body's shape must call it,
    /// after its checks and before its first Jolt call that makes the change.
    pub(crate) fn note_structure_change(&mut self) {
        advance_structure_epoch(&mut self.structure_epoch);
    }

    /// Gravity in m/s².
    pub fn gravity(&self) -> Vec3 {
        let mut gravity = Vec3::ZERO.to_jph();
        // SAFETY: the system is live; reading gravity does not change it, and writes need
        // `&mut self`. `gravity` is a live local.
        unsafe { JPH_PhysicsSystem_GetGravity(self.system.as_ptr(), &mut gravity) };
        Vec3::from_jph(gravity)
    }

    /// Sets gravity in m/s². It must be finite and at most [`limits::MAX_ACCELERATION`] long;
    /// zero is allowed. Sleeping bodies stay asleep.
    pub fn set_gravity(&mut self, gravity: Vec3) -> Result<(), WorldError> {
        if !limits::is_acceleration(gravity) {
            return Err(WorldError::InvalidSettings(limits::GRAVITY_RULE));
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

    /// Whether the world has room for `count` more bodies. Jolt's `BodyManager::AddBody` fails
    /// only when the world holds `GetMaxBodies()` bodies, so creators check this before they
    /// change anything and a full world rejects them cleanly.
    pub(crate) fn has_room_for_bodies(&self, count: usize) -> bool {
        // SAFETY: the system is live; the getter reads a constant.
        let max_bodies = unsafe { JPH_PhysicsSystem_GetMaxBodies(self.system.as_ptr()) };
        max_bodies
            .checked_sub(self.body_count())
            .is_some_and(|room| count as u64 <= u64::from(room))
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

    /// Advances the world by `delta_time` seconds in one collision step.
    ///
    /// `delta_time` must be finite, at least [`MIN_DELTA_TIME`](Self::MIN_DELTA_TIME) and at
    /// most [`MAX_DELTA_TIME`](Self::MAX_DELTA_TIME), otherwise nothing happens and
    /// [`StepError::InvalidDeltaTime`] is returned. Every other call advances the world and
    /// returns a [`StepReport`]; check [`StepReport::is_complete`] to learn whether Jolt
    /// dropped work because a fixed-size buffer was full.
    ///
    /// # Panics
    /// With a caller [`JobSystem`], a panic in its [`queue_job`](JobSystem::queue_job) does not
    /// stop the step: the world advances, and `step` then resumes the first such panic. Jobs
    /// handed to the caller's job system before the panic may still run on its threads; the jobs
    /// Jolt queues after it are not handed over and run on the stepping thread. The next step
    /// uses the caller's job system again.
    ///
    /// A panic in an event callback (see [`set_event_settings`](Self::set_event_settings))
    /// does not stop the step either: `step` resumes it after the update, unless a `queue_job`
    /// panic came first. It also resumes a callback panic from between steps that
    /// [`take_events`](Self::take_events) has not resumed yet.
    pub fn step(&mut self, delta_time: f32) -> Result<StepReport, StepError> {
        if !Self::is_valid_delta_time(delta_time) {
            return Err(StepError::InvalidDeltaTime);
        }
        self.listeners.begin_step();
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
        let job_panic = self.job_system.finish_update();
        let listeners = self.listeners.finish_step();
        if let Some(payload) = first_payload(job_panic, listeners.panic) {
            std::panic::resume_unwind(payload);
        }
        Ok(StepReport {
            manifold_cache_full: errors & JPH_PhysicsUpdateError_ManifoldCacheFull != 0,
            body_pair_cache_full: errors & JPH_PhysicsUpdateError_BodyPairCacheFull != 0,
            contact_constraints_full: errors & JPH_PhysicsUpdateError_ContactConstraintsFull != 0,
            rejected_contact_settings: listeners.rejected_contact_settings,
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
    /// How many contact settings a [`ContactListener`](crate::ContactListener) returned in this
    /// step that did not fit their contact and were not applied; the world's events name them
    /// ([`WorldEvents::rejected_contact_settings`](crate::WorldEvents::rejected_contact_settings)).
    /// Jolt still resolved those contacts, so they do not make the step incomplete.
    pub rejected_contact_settings: u32,
}

impl StepReport {
    /// Whether the step ran without dropping any work.
    pub fn is_complete(&self) -> bool {
        !(self.manifold_cache_full || self.body_pair_cache_full || self.contact_constraints_full)
    }
}

#[cfg(test)]
mod tests;
