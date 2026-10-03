//! Running Jolt's jobs on the caller's thread pool.
//!
//! A step splits its work into jobs: small closures with dependencies that Jolt hands to a job
//! system to run. By default a world owns Jolt's own thread pool
//! ([`WorldSettings::worker_threads`](crate::WorldSettings::worker_threads)); with
//! [`WorldSettings::job_system`](crate::WorldSettings::job_system) it hands its jobs to a
//! [`JobSystem`] the caller implements instead, for example on Rayon or the game's own pool.
//!
//! The thread that calls [`PhysicsWorld::step`](crate::PhysicsWorld::step) waits for the jobs
//! of each stage at a Jolt barrier, and while it waits it runs the jobs of that stage that no
//! other thread has started. A step therefore finishes even when the caller's pool runs a job
//! late or never; a job the stepping thread already ran does nothing when the pool runs it.
//!
//! Jolt documents its simulation as deterministic for the same binary, the same initial state
//! and the same calls in the same order (Jolt docs, "Deterministic Simulation"); which job
//! system ran the jobs is not part of that state. The determinism gates of this crate compare a
//! Rayon pool and an inline job system with Jolt's thread pool. Side effects of the caller's own
//! code in [`JobSystem::queue_job`] are not covered.

use std::any::Any;
use std::ffi::c_void;
use std::fmt;
use std::io::Write;
use std::mem;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::ptr::NonNull;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, OnceLock, PoisonError};

use joltphysics_sys::*;

use crate::owned::Owned;
use crate::WorldError;

/// A thread pool that runs Jolt's jobs for a world, set with
/// [`WorldSettings::job_system`](crate::WorldSettings::job_system).
///
/// Several worlds may share one job system and step on different threads at the same time.
///
/// # Example
/// A Rayon pool. Rust's orphan rule needs the newtype: neither the trait nor
/// `rayon::ThreadPool` belongs to the caller's crate.
///
/// ```
/// use std::sync::Arc;
/// use joltphysics::*;
///
/// struct RayonJobs(rayon::ThreadPool);
///
/// impl JobSystem for RayonJobs {
///     fn max_concurrency(&self) -> u32 {
///         // The pool's threads and the thread that steps the world.
///         u32::try_from(self.0.current_num_threads()).map_or(u32::MAX, |n| n.saturating_add(1))
///     }
///
///     fn queue_job(&self, job: Job) {
///         self.0.spawn(move || job.run());
///     }
/// }
///
/// # fn main() -> Result<(), Box<dyn std::error::Error>> {
/// let pool = rayon::ThreadPoolBuilder::new().num_threads(2).build()?;
/// let settings = WorldSettings::default().job_system(Arc::new(RayonJobs(pool)));
/// let mut world = PhysicsWorld::new(settings)?;
///
/// let floor = Shape::new_box(Vec3::new(100.0, 1.0, 100.0))?;
/// world.create_body(&floor, &BodySettings::new_static().position(RVec3::new(0.0, -1.0, 0.0)))?;
/// let cube = Shape::new_box(Vec3::new(0.5, 0.5, 0.5))?;
/// let falling = world.create_body(
///     &cube,
///     &BodySettings::new_dynamic().position(RVec3::new(0.0, 2.0, 0.0)),
/// )?;
/// for _ in 0..60 {
///     world.step(1.0 / 60.0)?;
/// }
/// assert!(world.body(falling)?.position().y < 2.0);
/// # Ok(())
/// # }
/// ```
///
/// The spawned closure only calls [`Job::run`], which does not panic, so Rayon's default of
/// aborting the process when a spawned task panics does not come into play; an adapter that
/// does more in the spawned closure owns that risk.
pub trait JobSystem: Send + Sync + 'static {
    /// The most jobs that may run at the same time, counting the thread that calls
    /// [`PhysicsWorld::step`](crate::PhysicsWorld::step).
    ///
    /// [`PhysicsWorld::new`](crate::PhysicsWorld::new) reads it once and rejects the settings
    /// unless it is within `1..=`[`WorldSettings::MAX_CONCURRENCY`](crate::WorldSettings::MAX_CONCURRENCY).
    /// Jolt uses it to decide how many jobs a stage is split into (at most 32 in Jolt 5.6); it
    /// changes how the work is split, not the results.
    fn max_concurrency(&self) -> u32;

    /// Runs `job` soon, on any thread, or right here before returning.
    ///
    /// Running the job inline is supported, as Jolt's own `JobSystemSingleThreaded` does. Jolt
    /// calls this from the stepping thread and from threads that run jobs, also from inside
    /// [`Job::run`] of another job on the same thread. It must not block waiting for any job.
    /// Every job must be run or dropped soon: each one holds a slot of a pool of 2048 jobs per
    /// world, and when that pool is empty Jolt waits for a free slot (and asserts with the
    /// `asserts` feature).
    ///
    /// A panic here does not unwind into Jolt; see [`PhysicsWorld::step`](crate::PhysicsWorld::step).
    fn queue_job(&self, job: Job);
}

/// One Jolt job handed to a [`JobSystem`].
///
/// Run it with [`run`](Self::run). Dropping it runs it as well, so a job that is never run is
/// still finished. A job may be sent to any thread and may outlive its world.
pub struct Job {
    function: unsafe extern "C" fn(*mut c_void),
    arg: NonNull<c_void>,
    /// Keeps the native job system alive until the job has run and been released.
    _native: Arc<CallbackJobSystem>,
}

impl Job {
    /// Runs the Jolt job unless the stepping thread already ran it, then hands the job back to
    /// Jolt.
    pub fn run(self) {
        drop(self);
    }
}

impl Drop for Job {
    fn drop(&mut self) {
        // SAFETY: `function` is joltc's `RunJob` and `arg` a Jolt job carrying the one reference
        // joltc's `QueueJob(s)` added for this hand-off. This value is the only owner of that
        // reference and is dropped once. `RunJob` executes the job at most once (`Job::Execute`
        // in `JobSystem.h` starts only from zero dependencies) and then releases that reference.
        // The native job system it releases into, and whose barrier a running job notifies,
        // stays alive: `self._native` is a field and drops only after this body returns. Job
        // functions capture only references, pointers and integers, so a job freed after its
        // world is gone touches nothing but the native job system. That relies on Jolt adding
        // every job of `PhysicsSystem::Update` to the update barrier, so that `Update` returns
        // only after every job ran; re-check this when updating Jolt or joltc.
        unsafe { (self.function)(self.arg.as_ptr()) }
    }
}

// SAFETY: Jolt lets a queued job run on any thread ("If you want to implement your own job
// system", `JobSystem.h`); its execution and release are atomic. The `Arc` is `Send` because
// `CallbackJobSystem` is `Send + Sync`. `Job` is not `Sync`: it is run by value, once.
unsafe impl Send for Job {}

impl fmt::Debug for Job {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("Job")
    }
}

/// joltc's callback job system of one world, destroyed by its last owner: the world's
/// [`QueueContext`] or a [`Job`] that is still queued.
pub(crate) struct CallbackJobSystem(Owned<JPH_JobSystem>);

// SAFETY: Jolt calls `QueueJob`, `FreeJob` and the barrier methods of one job system from many
// threads (lock-free free list, atomic barrier state), and `GetMaxConcurrency` reads a field
// set at creation. The object is destroyed only by the last `Arc`, so no call overlaps the
// destruction, and destruction has no thread affinity.
unsafe impl Send for CallbackJobSystem {}
// SAFETY: as for `Send`; shared references only hand the pointer to Jolt, as above.
unsafe impl Sync for CallbackJobSystem {}

/// What joltc's queue callbacks of one world get as their `context`: the caller's job system,
/// the native object the jobs belong to, and the first panic of the caller's `queue_job`.
///
/// Jobs do not hold the context, so a job that runs late on one of the caller's threads never
/// drops the caller's job system there.
pub(crate) struct QueueContext {
    job_system: Arc<dyn JobSystem>,
    /// Set once, right after the native object was created and before any step.
    native: OnceLock<Arc<CallbackJobSystem>>,
    /// Whether `panic` holds a payload; from then on jobs run inline.
    panicked: AtomicBool,
    panic: Mutex<Option<Box<dyn Any + Send>>>,
}

impl QueueContext {
    fn new(job_system: Arc<dyn JobSystem>) -> Self {
        Self {
            job_system,
            native: OnceLock::new(),
            panicked: AtomicBool::new(false),
            panic: Mutex::new(None),
        }
    }

    /// The native job system to step with.
    pub(crate) fn as_ptr(&self) -> *mut JPH_JobSystem {
        let native = self
            .native
            .get()
            .expect("caller job system is created with its context");
        native.0.as_ptr()
    }

    /// Wraps a job joltc queued; aborts on input joltc never passes.
    fn job(&self, function: JPH_JobFunction, arg: *mut c_void) -> Job {
        match (function, NonNull::new(arg), self.native.get()) {
            (Some(function), Some(arg), Some(native)) => Job {
                function,
                arg,
                _native: Arc::clone(native),
            },
            _ => abort_in_callback("joltc queued a job without a function or argument"),
        }
    }

    /// Hands `job` to the caller's job system, or runs it here once that panicked.
    fn queue(&self, job: Job) {
        if self.panicked.load(Ordering::Acquire) {
            drop(job);
            return;
        }
        let queued = catch_unwind(AssertUnwindSafe(|| self.job_system.queue_job(job)));
        if let Err(payload) = queued {
            self.record_panic(payload);
        }
    }

    /// Keeps the first panic payload and drops later ones without letting their drop unwind.
    fn record_panic(&self, payload: Box<dyn Any + Send>) {
        self.panicked.store(true, Ordering::Release);
        let discarded = {
            let mut slot = self.panic.lock().unwrap_or_else(PoisonError::into_inner);
            if slot.is_none() {
                *slot = Some(payload);
                None
            } else {
                Some(payload)
            }
        };
        if let Err(panic_in_drop) = catch_unwind(AssertUnwindSafe(|| drop(discarded))) {
            mem::forget(panic_in_drop);
        }
    }

    /// The first panic of the caller's `queue_job` since the last call, after which the caller's
    /// job system is used again. Called by `step` after the update returned, when no queue
    /// callback of this world runs.
    pub(crate) fn take_panic(&self) -> Option<Box<dyn Any + Send>> {
        let payload = self
            .panic
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .take();
        self.panicked.store(false, Ordering::Release);
        payload
    }
}

/// Reports a broken joltc contract and aborts; never unwinds.
fn abort_in_callback(what: &str) -> ! {
    let _ = writeln!(std::io::stderr(), "joltphysics: {what}; aborting");
    std::process::abort()
}

/// joltc's single-job queue callback.
///
/// # Safety
/// joltc calls it with the `context` of a live [`QueueContext`] during that world's
/// `PhysicsSystem::Update`, with `RunJob` and a Jolt job holding one reference for this call.
unsafe extern "C" fn queue_job(context: *mut c_void, function: JPH_JobFunction, arg: *mut c_void) {
    // SAFETY: `context` is the world's live `QueueContext` (contract), which the world keeps in
    // an `Arc` and only reads through shared references.
    let context = unsafe { &*context.cast::<QueueContext>() };
    context.queue(context.job(function, arg));
}

/// joltc's batch queue callback.
///
/// # Safety
/// As [`queue_job`], and `args` points to `count` jobs that each hold one reference for this
/// call. The array itself lives only during the call.
unsafe extern "C" fn queue_jobs(
    context: *mut c_void,
    function: JPH_JobFunction,
    args: *mut *mut c_void,
    count: u32,
) {
    if count == 0 {
        return;
    }
    if args.is_null() {
        abort_in_callback("joltc queued a batch without jobs");
    }
    // SAFETY: as in `queue_job`.
    let context = unsafe { &*context.cast::<QueueContext>() };
    for i in 0..count as usize {
        // SAFETY: `args` points to `count` job pointers that live during this call (contract);
        // each is read once, here, before the call returns.
        let arg = unsafe { *args.add(i) };
        context.queue(context.job(function, arg));
    }
}

/// Creates the native callback job system of a world that runs its jobs on `job_system`, and
/// returns the context that owns both.
pub(crate) fn create_caller_job_system(
    job_system: Arc<dyn JobSystem>,
    max_concurrency: u32,
) -> Result<Arc<QueueContext>, WorldError> {
    let context = Arc::new(QueueContext::new(job_system));
    let config = JPH_JobSystemConfig {
        context: Arc::as_ptr(&context).cast_mut().cast(),
        queueJob: Some(queue_job),
        queueJobs: Some(queue_jobs),
        maxConcurrency: max_concurrency,
        // Jolt's `cMaxPhysicsBarriers`.
        maxBarriers: 0,
    };
    // SAFETY: Jolt is initialised (the caller creates worlds only after `ensure_initialized`)
    // and `config` is a live local that joltc copies. The native object keeps `context` after
    // the world is gone, but joltc reads it only in `QueueJob(s)`, which Jolt calls only from
    // `CreateJob` and dependency removal inside `PhysicsSystem::Update`; a late `RunJob` of a
    // finished job queues nothing. The handle takes over the returned object.
    let native = unsafe { Owned::from_raw(JPH_JobSystemCallback_Create(&config)) }
        .ok_or(WorldError::AllocationFailed("job system"))?;
    // A fresh `OnceLock` is empty, so this always stores.
    let _ = context.native.set(Arc::new(CallbackJobSystem(native)));
    Ok(context)
}

#[cfg(test)]
mod tests {
    use std::sync::atomic::AtomicUsize;

    use super::*;
    use crate::world::ensure_initialized;

    /// A caller job system that panics on every call.
    struct AlwaysPanics {
        calls: AtomicUsize,
    }

    impl JobSystem for AlwaysPanics {
        fn max_concurrency(&self) -> u32 {
            1
        }

        fn queue_job(&self, job: Job) {
            let call = self.calls.fetch_add(1, Ordering::Relaxed) + 1;
            drop(job);
            panic!("queue_job call {call}");
        }
    }

    /// Counts how often a test job ran; its argument is a `RUNS` counter.
    unsafe extern "C" fn count_run(arg: *mut c_void) {
        // SAFETY: the tests pass a pointer to a live `AtomicUsize`.
        unsafe { &*arg.cast::<AtomicUsize>() }.fetch_add(1, Ordering::Relaxed);
    }

    /// A context with a real native object whose queue callbacks are never called: the tests
    /// queue their own jobs through `QueueContext::queue`.
    fn context(job_system: Arc<dyn JobSystem>) -> Arc<QueueContext> {
        assert!(ensure_initialized());
        create_caller_job_system(job_system, 1).unwrap()
    }

    fn counting_job(context: &QueueContext, runs: &AtomicUsize) -> Job {
        Job {
            function: count_run,
            arg: NonNull::from(runs).cast(),
            _native: Arc::clone(context.native.get().unwrap()),
        }
    }

    fn message(payload: &(dyn Any + Send)) -> String {
        payload
            .downcast_ref::<String>()
            .cloned()
            .unwrap_or_default()
    }

    #[test]
    fn run_and_drop_each_run_the_job_once() {
        let context = context(Arc::new(AlwaysPanics {
            calls: AtomicUsize::new(0),
        }));
        let runs = AtomicUsize::new(0);
        counting_job(&context, &runs).run();
        assert_eq!(runs.load(Ordering::Relaxed), 1);
        drop(counting_job(&context, &runs));
        assert_eq!(runs.load(Ordering::Relaxed), 2);
    }

    #[test]
    fn the_first_panic_is_kept_later_jobs_run_inline_and_take_panic_resets() {
        let job_system = Arc::new(AlwaysPanics {
            calls: AtomicUsize::new(0),
        });
        let context = context(job_system.clone());
        let runs = AtomicUsize::new(0);

        context.queue(counting_job(&context, &runs));
        context.queue(counting_job(&context, &runs));
        assert_eq!(runs.load(Ordering::Relaxed), 2);
        assert_eq!(job_system.calls.load(Ordering::Relaxed), 1);

        context.record_panic(Box::new(String::from("second")));
        let payload = context.take_panic().unwrap();
        assert_eq!(message(&*payload), "queue_job call 1");
        assert!(context.take_panic().is_none());

        context.queue(counting_job(&context, &runs));
        assert_eq!(runs.load(Ordering::Relaxed), 3);
        assert_eq!(job_system.calls.load(Ordering::Relaxed), 2);
        assert_eq!(message(&*context.take_panic().unwrap()), "queue_job call 2");
    }

    #[test]
    fn a_discarded_payload_whose_drop_panics_does_not_unwind() {
        struct PanicsOnDrop;

        impl Drop for PanicsOnDrop {
            fn drop(&mut self) {
                panic!("payload drop");
            }
        }

        let context = context(Arc::new(AlwaysPanics {
            calls: AtomicUsize::new(0),
        }));
        context.record_panic(Box::new(String::from("first")));
        context.record_panic(Box::new(PanicsOnDrop));
        assert_eq!(message(&*context.take_panic().unwrap()), "first");
    }
}
