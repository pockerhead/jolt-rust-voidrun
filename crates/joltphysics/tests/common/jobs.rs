//! Caller job systems for the tests: a Rayon pool and an inline job system, both counting the
//! jobs they were handed in one process-wide counter.

use std::sync::atomic::{AtomicUsize, Ordering};

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

/// Runs every job right away on the thread that queues it, which nests jobs on one stack.
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
