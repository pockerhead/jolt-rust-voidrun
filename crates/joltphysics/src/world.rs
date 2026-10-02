//! The physics world: a Jolt physics system with its own job system and temp allocator.
//!
//! joltc keeps a global map of physics systems that creating and destroying a system writes
//! without synchronisation, so both run under one process-wide lock here. Stepping uses the
//! world's own temp allocator and job system and needs no global lock, so independent worlds
//! step in parallel. The joltc functions that read that global map (step listeners) or use
//! joltc's shared temp allocator (`JPH_PhysicsSystem_Update`, the character updates) are not
//! used here; wrapping them requires revisiting this lock. Callbacks that run inside a step
//! must not use the locking body interface, which would deadlock.

use std::num::NonZeroU64;
use std::ptr::NonNull;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Mutex, MutexGuard, OnceLock, PoisonError};

use joltphysics_sys::*;

use crate::{CollisionLayers, StepError, Vec3, WorldError};

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

/// Owns a `JPH_TempAllocator`.
struct TempAllocator(NonNull<JPH_TempAllocator>);

impl Drop for TempAllocator {
    fn drop(&mut self) {
        // SAFETY: this value owns the allocator; the system that used it is gone or never
        // existed (field order in `PhysicsWorld`).
        unsafe { JPH_TempAllocator_Destroy(self.0.as_ptr()) };
    }
}

/// Owns a `JPH_JobSystem` and its worker threads.
struct JobSystem(NonNull<JPH_JobSystem>);

impl Drop for JobSystem {
    fn drop(&mut self) {
        // SAFETY: this value owns the job system, and no step is running (`step` borrows the
        // world mutably). Destroying it joins the worker threads.
        unsafe { JPH_JobSystem_Destroy(self.0.as_ptr()) };
    }
}

/// Owns a `JPH_PhysicsSystem`, and through it its bodies, their shape references and the
/// three layer tables passed to `JPH_PhysicsSystem_Create`.
struct System(NonNull<JPH_PhysicsSystem>);

impl Drop for System {
    fn drop(&mut self) {
        let _globals = lock_joltc_globals();
        // SAFETY: this value owns the system and the globals lock is held, so no other thread
        // writes joltc's system map. Nothing borrows the system any more.
        unsafe { JPH_PhysicsSystem_Destroy(self.0.as_ptr()) };
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
/// until Jolt has rebuilt it during steps (Jolt docs, "Bodies"); an explicit broad-phase
/// optimisation call comes with the scene-query API.
pub struct PhysicsWorld {
    // Field order is drop order: the system goes before the job system and allocator it used.
    system: System,
    job_system: JobSystem,
    temp_allocator: TempAllocator,
    pub(crate) body_interface: NonNull<JPH_BodyInterface>,
    pub(crate) body_lock_interface: NonNull<JPH_BodyLockInterface>,
    pub(crate) object_layer_count: u32,
    pub(crate) tag: WorldTag,
}

// SAFETY: the physics system, job system and temp allocator have no thread affinity. `step`
// needs `&mut self`, so the allocator and job system serve one `Update` at a time
// (https://jrouwe.github.io/JoltPhysicsDocs/5.3.0/index.html, multithreaded access).
unsafe impl Send for PhysicsWorld {}
// SAFETY: every `&self` method only calls Jolt's locking body interface or read-only system
// getters, which Jolt allows from several threads at once. Jolt forbids body access only while
// `PhysicsSystem::Update` runs, and `step` needs `&mut self`
// (https://jrouwe.github.io/JoltPhysicsDocs/5.3.0/index.html, multithreaded access).
unsafe impl Sync for PhysicsWorld {}

impl PhysicsWorld {
    /// Creates a world. Nothing is allocated when the settings are invalid.
    pub fn new(settings: WorldSettings) -> Result<Self, WorldError> {
        settings.validate()?;
        if !ensure_initialized() {
            return Err(WorldError::InitFailed);
        }

        // SAFETY: Jolt is initialised; the size is positive (`validate`).
        let temp_allocator =
            NonNull::new(unsafe { JPH_TempAllocator_Create(settings.temp_allocator_size) })
                .map(TempAllocator)
                .ok_or(WorldError::AllocationFailed("temp allocator"))?;

        let config = JobSystemThreadPoolConfig {
            maxJobs: 0,
            maxBarriers: 0,
            // `validate` bounds the count to `1..=MAX_WORKER_THREADS`; 0 would mean all hardware
            // threads.
            numThreads: settings.worker_threads as i32,
        };
        // SAFETY: Jolt is initialised; `config` is a live local. Zero job and barrier limits
        // select Jolt's `cMaxPhysicsJobs` and `cMaxPhysicsBarriers`.
        let job_system = NonNull::new(unsafe { JPH_JobSystemThreadPool_Create(&config) })
            .map(JobSystem)
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
            // live and consistent. On success the system owns the tables.
            NonNull::new(unsafe { JPH_PhysicsSystem_Create(&system_settings) })
        };
        let system = match system {
            Some(system) => {
                tables.forget();
                System(system)
            }
            None => return Err(WorldError::AllocationFailed("physics system")),
        };

        let gravity = settings.gravity.to_jph();
        // SAFETY: `system` is live and `gravity` is a live local.
        unsafe { JPH_PhysicsSystem_SetGravity(system.0.as_ptr(), &gravity) };

        // SAFETY: `system` is live. Both interfaces live inside the Jolt system and stay valid
        // as long as it does; the world stores them next to the system that owns them.
        let (body_interface, body_lock_interface) = unsafe {
            (
                JPH_PhysicsSystem_GetBodyInterface(system.0.as_ptr()),
                JPH_PhysicsSystem_GetBodyLockInterface(system.0.as_ptr()),
            )
        };
        let body_interface =
            NonNull::new(body_interface).ok_or(WorldError::AllocationFailed("body interface"))?;
        let body_lock_interface = NonNull::new(body_lock_interface.cast_mut())
            .ok_or(WorldError::AllocationFailed("body lock interface"))?;

        Ok(Self {
            system,
            job_system,
            temp_allocator,
            body_interface,
            body_lock_interface,
            object_layer_count: settings.layers.object_layer_count(),
            tag: WorldTag::next(),
        })
    }

    /// Gravity in m/s².
    pub fn gravity(&self) -> Vec3 {
        let mut gravity = Vec3::ZERO.to_jph();
        // SAFETY: the system is live; reading gravity does not change it, and writes need
        // `&mut self`. `gravity` is a live local.
        unsafe { JPH_PhysicsSystem_GetGravity(self.system.0.as_ptr(), &mut gravity) };
        Vec3::from_jph(gravity)
    }

    /// Sets gravity in m/s². It must be finite; zero is allowed. Sleeping bodies stay asleep.
    pub fn set_gravity(&mut self, gravity: Vec3) -> Result<(), WorldError> {
        if !gravity.is_finite() {
            return Err(WorldError::InvalidSettings("gravity must be finite"));
        }
        let gravity = gravity.to_jph();
        // SAFETY: the system is live and borrowed mutably; `gravity` is a live local.
        unsafe { JPH_PhysicsSystem_SetGravity(self.system.0.as_ptr(), &gravity) };
        Ok(())
    }

    /// Number of bodies in the world.
    pub fn body_count(&self) -> u32 {
        // SAFETY: the system is live; Jolt counts under its own body mutex.
        unsafe { JPH_PhysicsSystem_GetNumBodies(self.system.0.as_ptr()) }
    }

    /// Advances the world by `delta_time` seconds in one collision step.
    ///
    /// `delta_time` must be finite and positive, otherwise nothing happens and
    /// [`StepError::InvalidDeltaTime`] is returned. [`StepError::CacheFull`] means the step
    /// ran but dropped work; the world has advanced.
    pub fn step(&mut self, delta_time: f32) -> Result<(), StepError> {
        if !(delta_time.is_finite() && delta_time > 0.0) {
            return Err(StepError::InvalidDeltaTime);
        }
        // SAFETY: the system, temp allocator and job system are live and owned by this world;
        // `&mut self` guarantees no other call uses them or touches a body during the update.
        let errors = unsafe {
            JPH_PhysicsSystem_Update2(
                self.system.0.as_ptr(),
                delta_time,
                1,
                self.temp_allocator.0.as_ptr(),
                self.job_system.0.as_ptr(),
            )
        };
        if errors == JPH_PhysicsUpdateError_None {
            return Ok(());
        }
        Err(StepError::CacheFull {
            manifold_cache: errors & JPH_PhysicsUpdateError_ManifoldCacheFull != 0,
            body_pair_cache: errors & JPH_PhysicsUpdateError_BodyPairCacheFull != 0,
            contact_constraints: errors & JPH_PhysicsUpdateError_ContactConstraintsFull != 0,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
