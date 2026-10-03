//! Caller job systems for the tests: a Rayon pool and an inline job system, both counting the
//! jobs they were handed in one process-wide counter, and the process-wide choice of job system
//! that a determinism child reads from [`JOBS_ENV`].

use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;

use joltphysics::*;

/// Jobs handed to any [`RayonJobs`] or [`InlineJobs`] of this process.
static QUEUED: AtomicUsize = AtomicUsize::new(0);

/// How many jobs the caller job systems of this process were handed so far.
pub fn queued() -> usize {
    QUEUED.load(Ordering::Relaxed)
}

/// Jolt's jobs on a Rayon pool of its own.
pub struct RayonJobs(pub rayon::ThreadPool);

impl RayonJobs {
    /// A pool of `threads` threads.
    pub fn new(threads: usize) -> Self {
        Self(
            rayon::ThreadPoolBuilder::new()
                .num_threads(threads)
                .build()
                .unwrap(),
        )
    }
}

impl JobSystem for RayonJobs {
    fn max_concurrency(&self) -> u32 {
        u32::try_from(self.0.current_num_threads()).map_or(u32::MAX, |n| n.saturating_add(1))
    }

    fn queue_job(&self, job: Job) {
        QUEUED.fetch_add(1, Ordering::Relaxed);
        self.0.spawn(move || job.run());
    }
}

/// Calls [`Job::run`] right away inside `queue_job`, which leaves every job to the stepping
/// thread.
pub struct InlineJobs {
    pub concurrency: u32,
}

impl JobSystem for InlineJobs {
    fn max_concurrency(&self) -> u32 {
        self.concurrency
    }

    fn queue_job(&self, job: Job) {
        QUEUED.fetch_add(1, Ordering::Relaxed);
        job.run();
    }
}

/// Tells a determinism child which job system its worlds use: `rayon` or `inline`; unset for
/// Jolt's thread pool.
pub const JOBS_ENV: &str = "JOLTPHYSICS_DIGEST_JOBS";

/// The job system of the worlds of this process.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JobChoice {
    /// Jolt's thread pool with the requested worker threads.
    Native,
    /// A Rayon pool with the requested threads, concurrency one more.
    Rayon,
    /// [`InlineJobs`] with concurrency 3, whatever thread count is requested.
    Inline,
}

impl JobChoice {
    /// The value of [`JOBS_ENV`] that selects this choice; `None` for unset.
    pub fn as_env(self) -> Option<&'static str> {
        match self {
            Self::Native => None,
            Self::Rayon => Some("rayon"),
            Self::Inline => Some("inline"),
        }
    }

    /// The choice [`JOBS_ENV`] selects; panics on a value it does not know.
    pub fn from_env() -> Self {
        match std::env::var(JOBS_ENV) {
            Err(std::env::VarError::NotPresent) => Self::Native,
            Ok(value) if value == "rayon" => Self::Rayon,
            Ok(value) if value == "inline" => Self::Inline,
            value => panic!("{JOBS_ENV}={value:?} is not a job system"),
        }
    }
}

/// `settings` with the job system [`JobChoice::from_env`] selects, given `worker_threads`.
pub fn with_threads(settings: WorldSettings, worker_threads: u32) -> WorldSettings {
    match JobChoice::from_env() {
        JobChoice::Native => settings.worker_threads(worker_threads),
        JobChoice::Rayon => settings.job_system(Arc::new(RayonJobs::new(worker_threads as usize))),
        JobChoice::Inline => settings.job_system(Arc::new(InlineJobs { concurrency: 3 })),
    }
}
