//! Smoke tests for the raw bindings: a world can be created, stepped and read.

mod framework;

use std::ffi::c_void;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::sync::atomic::{AtomicUsize, Ordering};

use framework::*;
use oxijolt_sys::*;

#[test]
fn setup_teardown() {
    drop(TestWorld::new(2));
}

#[test]
fn box_falls_onto_static_box() {
    let world = TestWorld::new(2);
    let bodies = world.body_interface();

    let floor = create_box(
        bodies,
        vec3(100.0, 1.0, 100.0),
        rvec3(0.0, -1.0, 0.0),
        JPH_MotionType_Static,
        OL_NON_MOVING,
        JPH_Activation_DontActivate,
    );
    let falling = create_box(
        bodies,
        vec3(0.5, 0.5, 0.5),
        rvec3(0.0, 2.0, 0.0),
        JPH_MotionType_Dynamic,
        OL_MOVING,
        JPH_Activation_Activate,
    );

    for _ in 0..60 {
        world.step(1.0 / 60.0);
    }

    let mut position = rvec3(0.0, 0.0, 0.0);
    // SAFETY: `bodies` belongs to the live `world`, `falling` is a body in it
    // and `position` is a live local.
    unsafe { JPH_BodyInterface_GetPosition(bodies, falling, &mut position) };

    assert!(
        position.x.is_finite() && position.y.is_finite() && position.z.is_finite(),
        "{position:?}"
    );
    assert!(position.x.abs() < 1e-3, "{position:?}");
    assert!(position.z.abs() < 1e-3, "{position:?}");
    // The floor's top face is at y = 0 and the box's half height is 0.5.
    assert!(position.y > 0.45 && position.y < 0.55, "{position:?}");

    // SAFETY: both ids are bodies of the live `world`, each removed once.
    unsafe {
        JPH_BodyInterface_RemoveAndDestroyBody(bodies, falling);
        JPH_BodyInterface_RemoveAndDestroyBody(bodies, floor);
    }
}

/// How often joltc called each queue callback of a callback job system.
#[derive(Default)]
struct QueueCounts {
    single: AtomicUsize,
    batch: AtomicUsize,
}

/// Runs the job at once on the calling thread.
///
/// # Safety
/// joltc calls this with the `context` of the live `QueueCounts` the job system was created with
/// and a job function with its argument, which may be run once.
unsafe extern "C" fn queue_job(context: *mut c_void, job: JPH_JobFunction, arg: *mut c_void) {
    let result = catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: `context` points to the `QueueCounts` that outlives the world (contract).
        let counts = unsafe { &*context.cast::<QueueCounts>() };
        counts.single.fetch_add(1, Ordering::Relaxed);
        match job {
            // SAFETY: joltc hands over a job function and its argument to be run once.
            Some(run) => unsafe { run(arg) },
            None => std::process::abort(),
        }
    }));
    if result.is_err() {
        std::process::abort();
    }
}

/// Runs every job of the batch at once on the calling thread.
///
/// # Safety
/// As [`queue_job`], and `args` points to `count` job arguments that live for this call.
unsafe extern "C" fn queue_jobs(
    context: *mut c_void,
    job: JPH_JobFunction,
    args: *mut *mut c_void,
    count: u32,
) {
    let result = catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: `context` points to the `QueueCounts` that outlives the world (contract).
        let counts = unsafe { &*context.cast::<QueueCounts>() };
        counts.batch.fetch_add(1, Ordering::Relaxed);
        let Some(run) = job else {
            std::process::abort()
        };
        for i in 0..count as usize {
            // SAFETY: `args` holds `count` arguments for this call (contract), each of a job
            // that may be run once.
            unsafe { run(*args.add(i)) };
        }
    }));
    if result.is_err() {
        std::process::abort();
    }
}

#[test]
fn callback_job_system_steps_a_world() {
    // Declared before the world, so it outlives the job system that points to it.
    let counts = QueueCounts::default();
    init();
    let config = JPH_JobSystemConfig {
        context: std::ptr::from_ref(&counts).cast_mut().cast(),
        queueJob: Some(queue_job),
        queueJobs: Some(queue_jobs),
        maxConcurrency: 2,
        maxBarriers: 0,
    };
    // SAFETY: Jolt is initialised and `config` is a live local; joltc copies it. The callbacks
    // only read `counts`, which outlives the job system.
    let job_system = unsafe { JPH_JobSystemCallback_Create(&config) };
    // SAFETY: `job_system` was just created after `init` and nothing else owns it.
    let world = unsafe { TestWorld::with_job_system(job_system) };
    let bodies = world.body_interface();

    let floor = create_box(
        bodies,
        vec3(100.0, 1.0, 100.0),
        rvec3(0.0, -1.0, 0.0),
        JPH_MotionType_Static,
        OL_NON_MOVING,
        JPH_Activation_DontActivate,
    );
    let falling = create_box(
        bodies,
        vec3(0.5, 0.5, 0.5),
        rvec3(0.0, 2.0, 0.0),
        JPH_MotionType_Dynamic,
        OL_MOVING,
        JPH_Activation_Activate,
    );

    for _ in 0..60 {
        world.step(1.0 / 60.0);
    }

    let mut position = rvec3(0.0, 0.0, 0.0);
    // SAFETY: `bodies` belongs to the live `world`, `falling` is a body in it
    // and `position` is a live local.
    unsafe { JPH_BodyInterface_GetPosition(bodies, falling, &mut position) };
    // The floor's top face is at y = 0 and the box's half height is 0.5.
    assert!(position.y > 0.45 && position.y < 0.55, "{position:?}");
    assert!(counts.single.load(Ordering::Relaxed) > 0);
    assert!(counts.batch.load(Ordering::Relaxed) > 0);

    // SAFETY: both ids are bodies of the live `world`, each removed once.
    unsafe {
        JPH_BodyInterface_RemoveAndDestroyBody(bodies, falling);
        JPH_BodyInterface_RemoveAndDestroyBody(bodies, floor);
    }
}
