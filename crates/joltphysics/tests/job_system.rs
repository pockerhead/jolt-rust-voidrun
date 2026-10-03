//! Worlds that run Jolt's jobs on a caller job system: results equal to Jolt's thread pool,
//! inline and re-entrant execution, jobs that outlive their world, panics in the caller's
//! `queue_job`, the concurrency bound and the choice between the two job systems.

mod common;

use std::any::Any;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{mpsc, Arc, Barrier, Mutex};
use std::thread;
use std::time::Duration;

use common::jobs::{InlineJobs, RayonJobs};
use common::ragdoll as humanoid;
use common::*;
use joltphysics::*;

/// Ticks of the stacks scene.
const TICKS: usize = 120;

/// Seconds a test that could deadlock may take before it fails.
const TIMEOUT_SECS: u64 = 300;

/// Runs `f` on its own thread and returns its result; panics if `f` panics or takes longer
/// than `secs` seconds, so that a deadlock fails the test instead of hanging the run.
fn with_timeout<T: Send + 'static>(secs: u64, f: impl FnOnce() -> T + Send + 'static) -> T {
    let (sender, receiver) = mpsc::channel();
    let handle = thread::spawn(move || {
        let _ = sender.send(f());
    });
    match receiver.recv_timeout(Duration::from_secs(secs)) {
        Ok(value) => {
            handle.join().unwrap();
            value
        }
        Err(mpsc::RecvTimeoutError::Timeout) => {
            panic!("no result after {secs} s: the step is probably deadlocked")
        }
        Err(mpsc::RecvTimeoutError::Disconnected) => {
            std::panic::resume_unwind(handle.join().unwrap_err())
        }
    }
}

/// Counts the jobs handed to the job system it wraps.
struct Counting<J> {
    inner: J,
    queued: AtomicUsize,
}

impl<J> Counting<J> {
    fn new(inner: J) -> Self {
        Self {
            inner,
            queued: AtomicUsize::new(0),
        }
    }

    fn queued(&self) -> usize {
        self.queued.load(Ordering::Relaxed)
    }
}

impl<J: JobSystem> JobSystem for Counting<J> {
    fn max_concurrency(&self) -> u32 {
        self.inner.max_concurrency()
    }

    fn queue_job(&self, job: Job) {
        self.queued.fetch_add(1, Ordering::Relaxed);
        self.inner.queue_job(job);
    }
}

/// Keeps every job until the test takes them.
#[derive(Default)]
struct DeferredJobs {
    jobs: Mutex<Vec<Job>>,
}

impl DeferredJobs {
    fn take(&self) -> Vec<Job> {
        std::mem::take(&mut *self.jobs.lock().unwrap())
    }
}

impl JobSystem for DeferredJobs {
    fn max_concurrency(&self) -> u32 {
        4
    }

    fn queue_job(&self, job: Job) {
        self.jobs.lock().unwrap().push(job);
    }
}

/// When a [`PanickingJobs`] panics.
#[derive(Clone, Copy, Debug)]
enum PanicMode {
    /// On the n-th call, while still holding the job, which unwinding then drops.
    BeforeStoring(usize),
    /// On the n-th call, after storing the job for the test to drain.
    AfterStoring(usize),
    /// On every call while armed, also on the calls Jolt makes while the first panic unwinds.
    Always,
}

/// Runs jobs inline, except that it panics as its mode says.
struct PanickingJobs {
    mode: PanicMode,
    armed: AtomicBool,
    calls: AtomicUsize,
    stored: Mutex<Vec<Job>>,
}

impl PanickingJobs {
    fn new(mode: PanicMode) -> Self {
        Self {
            mode,
            armed: AtomicBool::new(true),
            calls: AtomicUsize::new(0),
            stored: Mutex::new(Vec::new()),
        }
    }

    fn calls(&self) -> usize {
        self.calls.load(Ordering::Relaxed)
    }
}

impl JobSystem for PanickingJobs {
    fn max_concurrency(&self) -> u32 {
        3
    }

    fn queue_job(&self, job: Job) {
        let call = self.calls.fetch_add(1, Ordering::Relaxed) + 1;
        match self.mode {
            PanicMode::BeforeStoring(n) if call == n => {
                panic!("injected panic at call {call}")
            }
            PanicMode::AfterStoring(n) if call == n => {
                self.stored.lock().unwrap().push(job);
                panic!("injected panic at call {call}")
            }
            PanicMode::Always if self.armed.load(Ordering::Relaxed) => {
                panic!("injected panic at call {call}")
            }
            _ => job.run(),
        }
    }
}

/// Reads the job system's concurrency from a field and counts the reads; runs jobs inline.
struct FixedConcurrency {
    value: u32,
    reads: AtomicUsize,
}

impl JobSystem for FixedConcurrency {
    fn max_concurrency(&self) -> u32 {
        self.reads.fetch_add(1, Ordering::Relaxed);
        self.value
    }

    fn queue_job(&self, job: Job) {
        job.run();
    }
}

fn panic_message(payload: &(dyn Any + Send)) -> String {
    payload
        .downcast_ref::<String>()
        .cloned()
        .unwrap_or_default()
}

/// The stacks scene in a world with `settings`: per-tick records of every body.
fn stacks_digest(settings: WorldSettings) -> Vec<u8> {
    let mut world = PhysicsWorld::new(settings).unwrap();
    let ids = build_stacks(&mut world);
    run_digest(&mut world, &ids, TICKS)
}

fn native() -> WorldSettings {
    WorldSettings::default().worker_threads(1)
}

#[test]
fn rayon_job_system_matches_the_native_pool() {
    let jobs = Arc::new(Counting::new(RayonJobs::new(3)));
    let caller = stacks_digest(WorldSettings::default().job_system(jobs.clone()));
    assert_eq!(caller, stacks_digest(native()));
    assert!(jobs.queued() > 0);
}

/// Sixteen humanoid ragdolls lying side by side on a floor, touching, for 120 ticks: per-tick
/// records of every part.
fn ragdoll_pile_digest(settings: WorldSettings) -> Vec<u8> {
    let (layers, ids) = humanoid::ragdoll_layers();
    let mut world = PhysicsWorld::new(settings.layers(layers)).unwrap();
    let floor = Shape::new_box(Vec3::new(20.0, 1.0, 20.0)).unwrap();
    world
        .create_body(
            &floor,
            &BodySettings::new_static()
                .position(RVec3::new(0.0, -1.0, 0.0))
                .object_layer(ids.fixed),
        )
        .unwrap();
    let face_up = quat_about(humanoid::X, -std::f32::consts::FRAC_PI_2);
    let ragdoll_settings = humanoid::humanoid_settings(ids.ragdoll);
    let mut parts = Vec::new();
    for key in 0..16 {
        let (i, j) = ((key % 4) as f64 - 1.5, (key / 4) as f64 - 1.5);
        // Row and column spacing let hands and head and feet of neighbours overlap slightly.
        let at = [i * 1.34, 0.63, j * 1.834];
        let pose = humanoid::transformed_pose(&humanoid::bind_pose(), face_up, at);
        let ragdoll = world
            .create_ragdoll(&ragdoll_settings, Some(&pose), Activation::Activate)
            .unwrap();
        parts.extend_from_slice(world.ragdoll(ragdoll).unwrap().body_ids());
    }
    run_digest(&mut world, &parts, TICKS)
}

#[test]
fn inline_job_system_matches_the_native_pool() {
    let jobs = Arc::new(Counting::new(InlineJobs { concurrency: 3 }));
    let caller = stacks_digest(WorldSettings::default().job_system(jobs.clone()));
    assert_eq!(caller, stacks_digest(native()));
    assert!(jobs.queued() > 0);

    // Inline execution nests jobs on one stack; a Jolt job that waited for work held lower on
    // that stack would never return.
    let pile_jobs = Arc::new(InlineJobs { concurrency: 3 });
    let caller = with_timeout(TIMEOUT_SECS, move || {
        ragdoll_pile_digest(WorldSettings::default().job_system(pile_jobs))
    });
    assert_eq!(caller, ragdoll_pile_digest(native()));
}

#[test]
fn step_from_the_only_thread_of_a_rayon_pool_completes() {
    let caller = with_timeout(TIMEOUT_SECS, || {
        let jobs = Arc::new(RayonJobs::new(1));
        let mut world =
            PhysicsWorld::new(WorldSettings::default().job_system(jobs.clone())).unwrap();
        let ids = build_stacks(&mut world);
        jobs.0.install(|| run_digest(&mut world, &ids, TICKS))
    });
    assert_eq!(caller, stacks_digest(native()));
}

/// A world on `jobs` with a floor and four awake cubes.
fn cubes_world(jobs: Arc<dyn JobSystem>) -> PhysicsWorld {
    let mut world = PhysicsWorld::new(WorldSettings::default().job_system(jobs)).unwrap();
    add_floor(&mut world);
    for i in 0..4 {
        add_cube(&mut world, RVec3::new(1.5 * i as Real, 2.0, 0.0));
    }
    world
}

#[test]
fn jobs_may_outlive_their_world() {
    let jobs = Arc::new(DeferredJobs::default());
    let mut world = cubes_world(jobs.clone());
    step(&mut world, 2);
    drop(world);

    let mut deferred = jobs.take();
    assert!(deferred.len() > 1);
    let to_run = deferred.split_off(deferred.len() / 2);
    drop(deferred);
    for job in to_run {
        job.run();
    }
}

#[test]
fn a_deferred_job_may_finish_on_another_thread_while_the_world_drops() {
    let jobs = Arc::new(DeferredJobs::default());
    let mut world = cubes_world(jobs.clone());
    step(&mut world, 2);
    let deferred = jobs.take();
    assert!(!deferred.is_empty());

    let start = Arc::new(Barrier::new(2));
    let runner = {
        let start = Arc::clone(&start);
        thread::spawn(move || {
            start.wait();
            for job in deferred {
                job.run();
            }
        })
    };
    start.wait();
    drop(world);
    runner.join().unwrap();
}

/// The step that panics, the drained jobs and 30 more steps; the result must equal a native
/// world stepped as often.
fn panicking_job_system_case(mode: PanicMode) {
    let jobs = Arc::new(PanickingJobs::new(mode));
    let mut world = PhysicsWorld::new(WorldSettings::default().job_system(jobs.clone())).unwrap();
    let ids = build_stacks(&mut world);

    let payload = catch_unwind(AssertUnwindSafe(|| world.step(DT))).unwrap_err();
    let message = panic_message(&*payload);
    match mode {
        PanicMode::BeforeStoring(n) | PanicMode::AfterStoring(n) => {
            assert_eq!(message, format!("injected panic at call {n}"), "{mode:?}");
        }
        PanicMode::Always => assert!(message.starts_with("injected panic"), "{message}"),
    }
    jobs.armed.store(false, Ordering::Relaxed);
    for job in std::mem::take(&mut *jobs.stored.lock().unwrap()) {
        job.run();
    }

    let calls = jobs.calls();
    step(&mut world, 30);
    assert!(
        jobs.calls() > calls,
        "{mode:?}: the job system was not used again"
    );

    let mut native_world = PhysicsWorld::new(native()).unwrap();
    let native_ids = build_stacks(&mut native_world);
    step(&mut native_world, 31);
    let mut state = Vec::new();
    let mut native_state = Vec::new();
    for (&id, &native_id) in ids.iter().zip(&native_ids) {
        record_body(&world, id, &mut state);
        record_body(&native_world, native_id, &mut native_state);
    }
    assert_eq!(state, native_state, "{mode:?}");
}

#[test]
fn a_panicking_job_system_panics_out_of_step_and_the_world_stays_usable() {
    for mode in [
        PanicMode::BeforeStoring(3),
        PanicMode::AfterStoring(3),
        PanicMode::Always,
    ] {
        panicking_job_system_case(mode);
    }
}

#[test]
fn max_concurrency_is_bounded() {
    for value in [0, WorldSettings::MAX_CONCURRENCY + 1, u32::MAX] {
        let jobs = Arc::new(FixedConcurrency {
            value,
            reads: AtomicUsize::new(0),
        });
        let result = PhysicsWorld::new(WorldSettings::default().job_system(jobs.clone()));
        assert!(
            matches!(result, Err(WorldError::InvalidSettings(_))),
            "{value}"
        );
        assert_eq!(jobs.reads.load(Ordering::Relaxed), 1, "{value}");
    }
    for value in [1, WorldSettings::MAX_CONCURRENCY] {
        let jobs = Arc::new(FixedConcurrency {
            value,
            reads: AtomicUsize::new(0),
        });
        let mut world = cubes_world(jobs.clone());
        step(&mut world, 10);
        assert_eq!(jobs.reads.load(Ordering::Relaxed), 1, "{value}");
    }
}

#[test]
fn worlds_sharing_one_job_system_step_in_parallel() {
    let digests = with_timeout(TIMEOUT_SECS, || {
        let jobs: Arc<dyn JobSystem> = Arc::new(RayonJobs::new(2));
        thread::scope(|scope| {
            let runs: Vec<_> = (0..2)
                .map(|_| {
                    let jobs = Arc::clone(&jobs);
                    scope.spawn(move || stacks_digest(WorldSettings::default().job_system(jobs)))
                })
                .collect();
            runs.into_iter()
                .map(|run| run.join().unwrap())
                .collect::<Vec<_>>()
        })
    });
    let native = stacks_digest(native());
    for digest in digests {
        assert_eq!(digest, native);
    }
}

#[test]
fn the_last_of_job_system_and_worker_threads_wins() {
    let jobs = Arc::new(Counting::new(InlineJobs { concurrency: 3 }));
    let settings = WorldSettings::default()
        .job_system(jobs.clone())
        .worker_threads(2);
    assert!(
        format!("{settings:?}").contains("ThreadPool(2)"),
        "{settings:?}"
    );
    stacks_digest(settings);
    assert_eq!(jobs.queued(), 0);

    let settings = WorldSettings::default()
        .worker_threads(2)
        .job_system(jobs.clone());
    assert!(
        format!("{settings:?}").contains("Caller(JobSystem)"),
        "{settings:?}"
    );
    stacks_digest(settings);
    assert!(jobs.queued() > 0);
}

#[test]
fn a_sleeping_world_queues_no_jobs() {
    let jobs = Arc::new(Counting::new(InlineJobs { concurrency: 3 }));
    let mut world = PhysicsWorld::new(WorldSettings::default().job_system(jobs.clone())).unwrap();
    add_floor(&mut world);
    let cube = add_cube(&mut world, RVec3::new(0.0, 0.5, 0.0));
    let mut ticks = 0;
    while !world.body(cube).unwrap().is_sleeping() {
        assert!(ticks < 600, "the cube never fell asleep");
        step(&mut world, 1);
        ticks += 1;
    }
    assert!(jobs.queued() > 0);

    // Jolt skips the job stages when no body is awake and no step listener is installed
    // (`PhysicsSystem::Update`).
    let queued = jobs.queued();
    step(&mut world, 10);
    assert_eq!(jobs.queued(), queued);
}
