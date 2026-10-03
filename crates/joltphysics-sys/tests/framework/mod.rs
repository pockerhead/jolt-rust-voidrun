//! Shared setup for the raw binding tests: one-time `JPH_Init`, a world with
//! table-based layers (no Rust callbacks are called from C++), and box helpers.

// Each test file compiles this module on its own and uses a different subset.
#![allow(dead_code)]

use std::ffi::{c_char, CStr};
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::sync::{Mutex, MutexGuard, OnceLock, PoisonError};

use joltphysics_sys::*;

pub const OL_NON_MOVING: JPH_ObjectLayer = 0;
pub const OL_MOVING: JPH_ObjectLayer = 1;
const OBJECT_LAYER_COUNT: u32 = 2;

pub const BPL_NON_MOVING: JPH_BroadPhaseLayer = 0;
pub const BPL_MOVING: JPH_BroadPhaseLayer = 1;
const BROAD_PHASE_LAYER_COUNT: u32 = 2;

/// A Jolt assertion handler that prints the failure and aborts, so a failed
/// assertion fails the test run (with the `asserts` feature) instead of
/// stopping at joltc's breakpoint.
///
/// # Safety
/// Each pointer is null or a NUL-terminated string that lives for the call,
/// as Jolt passes them.
unsafe extern "C" fn abort_on_assert(
    expression: *const c_char,
    message: *const c_char,
    file: *const c_char,
    line: u32,
) -> bool {
    let _ = catch_unwind(AssertUnwindSafe(|| {
        let text = |text: *const c_char| {
            // SAFETY: not null, and Jolt passes NUL-terminated strings that
            // live for this call (contract).
            (!text.is_null()).then(|| unsafe { CStr::from_ptr(text) }.to_string_lossy())
        };
        let (expression, message, file) = (text(expression), text(message), text(file));
        eprintln!(
            "Jolt assertion failed: {}:{line}: ({}) {}",
            file.unwrap_or_default(),
            expression.unwrap_or_default(),
            message.unwrap_or_default()
        );
    }));
    std::process::abort()
}

/// Calls `JPH_Init` once per process. `JPH_Shutdown` is never called because
/// tests run on parallel threads.
pub fn init() {
    static INIT: OnceLock<()> = OnceLock::new();
    INIT.get_or_init(|| {
        // SAFETY: `OnceLock` runs this exactly once per process, so the
        // unsynchronised initialisation in `JPH_Init` never races, and the
        // handler store before it is a plain write of a joltc static.
        let initialized = unsafe {
            JPH_SetAssertFailureHandler(Some(abort_on_assert));
            JPH_Init()
        };
        assert!(initialized, "JPH_Init failed");
    });
}

/// Serialises physics system creation and destruction, which joltc tracks in
/// an unsynchronised global map.
fn world_lock() -> MutexGuard<'static, ()> {
    static WORLD: Mutex<()> = Mutex::new(());
    WORLD.lock().unwrap_or_else(PoisonError::into_inner)
}

/// A physics system with its own job system and temp allocator.
pub struct TestWorld {
    system: *mut JPH_PhysicsSystem,
    job_system: *mut JPH_JobSystem,
    temp_allocator: *mut JPH_TempAllocator,
    // Last field: released after `Drop::drop` destroyed the system.
    _guard: MutexGuard<'static, ()>,
}

impl TestWorld {
    /// Creates a world stepped by `worker_threads` Jolt worker threads.
    pub fn new(worker_threads: i32) -> Self {
        assert!(worker_threads > 0, "joltc maps 0 or less to automatic");
        init();
        let guard = world_lock();

        // SAFETY: Jolt is initialised and the world lock is held. Every
        // pointer passed in comes from the matching `_Create` call just
        // above it and is not destroyed before `JPH_PhysicsSystem_Create`,
        // which takes ownership of the three layer tables.
        let system = unsafe {
            let pair_filter = JPH_ObjectLayerPairFilterTable_Create(OBJECT_LAYER_COUNT);
            JPH_ObjectLayerPairFilterTable_EnableCollision(pair_filter, OL_NON_MOVING, OL_MOVING);
            JPH_ObjectLayerPairFilterTable_EnableCollision(pair_filter, OL_MOVING, OL_MOVING);

            let broad_phase = JPH_BroadPhaseLayerInterfaceTable_Create(
                OBJECT_LAYER_COUNT,
                BROAD_PHASE_LAYER_COUNT,
            );
            JPH_BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(
                broad_phase,
                OL_NON_MOVING,
                BPL_NON_MOVING,
            );
            JPH_BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(
                broad_phase,
                OL_MOVING,
                BPL_MOVING,
            );

            let object_vs_broad_phase = JPH_ObjectVsBroadPhaseLayerFilterTable_Create(
                broad_phase,
                BROAD_PHASE_LAYER_COUNT,
                pair_filter,
                OBJECT_LAYER_COUNT,
            );

            let settings = JPH_PhysicsSystemSettings {
                maxBodies: 1024,
                maxBodyPairs: 1024,
                maxContactConstraints: 1024,
                broadPhaseLayerInterface: broad_phase,
                objectLayerPairFilter: pair_filter,
                objectVsBroadPhaseLayerFilter: object_vs_broad_phase,
                ..std::mem::zeroed()
            };
            JPH_PhysicsSystem_Create(&settings)
        };
        assert!(!system.is_null(), "JPH_PhysicsSystem_Create failed");

        let pool_config = JobSystemThreadPoolConfig {
            // 0 selects Jolt's default job and barrier counts.
            maxJobs: 0,
            maxBarriers: 0,
            numThreads: worker_threads,
        };
        // SAFETY: `pool_config` is a live local for the duration of the call.
        let job_system = unsafe { JPH_JobSystemThreadPool_Create(&pool_config) };
        // SAFETY: plain allocation with no preconditions besides `JPH_Init`.
        let temp_allocator = unsafe { JPH_TempAllocator_Create(10 * 1024 * 1024) };

        TestWorld {
            system,
            job_system,
            temp_allocator,
            _guard: guard,
        }
    }

    /// Advances the simulation by `dt` seconds with one collision step.
    pub fn step(&self, dt: f32) {
        // SAFETY: all three pointers come from `new` and live until `drop`.
        let result = unsafe {
            JPH_PhysicsSystem_Update2(self.system, dt, 1, self.temp_allocator, self.job_system)
        };
        assert_eq!(result, JPH_PhysicsUpdateError_None);
    }

    /// The physics system, owned by this world.
    pub fn system(&self) -> *mut JPH_PhysicsSystem {
        self.system
    }

    /// The system's locking body interface.
    pub fn body_interface(&self) -> *mut JPH_BodyInterface {
        // SAFETY: `system` comes from `new` and lives until `drop`.
        unsafe { JPH_PhysicsSystem_GetBodyInterface(self.system) }
    }
}

impl Drop for TestWorld {
    fn drop(&mut self) {
        // SAFETY: the pointers come from `new` and are destroyed exactly once
        // here, the system first because it uses the other two. The world
        // lock is still held: `_guard` drops after this function.
        unsafe {
            JPH_PhysicsSystem_Destroy(self.system);
            JPH_JobSystem_Destroy(self.job_system);
            JPH_TempAllocator_Destroy(self.temp_allocator);
        }
    }
}

pub fn vec3(x: f32, y: f32, z: f32) -> JPH_Vec3 {
    JPH_Vec3 { x, y, z }
}

pub fn rvec3(x: Real, y: Real, z: Real) -> JPH_RVec3 {
    JPH_RVec3 { x, y, z }
}

pub fn quat_identity() -> JPH_Quat {
    JPH_Quat {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0,
    }
}

/// Creates a box body, adds it to the world and returns its id.
pub fn create_box(
    body_interface: *mut JPH_BodyInterface,
    half_extent: JPH_Vec3,
    position: JPH_RVec3,
    motion_type: JPH_MotionType,
    layer: JPH_ObjectLayer,
    activation: JPH_Activation,
) -> JPH_BodyID {
    let rotation = quat_identity();
    // SAFETY: `body_interface` belongs to a live `TestWorld`; the vector and
    // quaternion arguments are live locals. The shape is created holding one
    // reference; the body takes its own, so releasing ours with
    // `JPH_Shape_Destroy` keeps the shape alive for the body.
    let body = unsafe {
        let shape = JPH_BoxShape_Create(&half_extent, JPH_DEFAULT_CONVEX_RADIUS as f32);
        let settings = JPH_BodyCreationSettings_Create3(
            shape as *const JPH_Shape,
            &position,
            &rotation,
            motion_type,
            layer,
        );
        let body = JPH_BodyInterface_CreateAndAddBody(body_interface, settings, activation);
        JPH_BodyCreationSettings_Destroy(settings);
        JPH_Shape_Destroy(shape as *mut JPH_Shape);
        body
    };
    assert_ne!(body, u32::MAX, "body creation failed");
    body
}
