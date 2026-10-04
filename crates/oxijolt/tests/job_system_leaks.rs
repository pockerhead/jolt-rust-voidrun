//! A leak gate for caller job systems. Every other round creates a world on a job system that
//! keeps its jobs, steps it, drops the world while the job system still holds jobs, and only then
//! drops half of the jobs and runs the rest; the last job frees joltc's callback job system. The
//! rounds between use a job system that calls `Job::run` inside `queue_job`, whose jobs the step
//! hands back to Jolt itself. Measured over 10 000 rounds against a 4 MiB threshold; a probe that
//! forgot the kept jobs instead grew by about 1.7 GB, about 330 KB per job system.
//!
//! It measures the private bytes of the process (Windows `K32GetProcessMemoryInfo`), because the
//! job system is allocated by C++, which a Rust global allocator does not see. The file holds
//! exactly one test, so its binary runs alone and no parallel test disturbs the counter.
#![cfg(windows)]

mod common;

use std::sync::{Arc, Mutex};

use common::jobs::InlineJobs;
use common::memory::private_bytes;
use common::*;
use oxijolt::*;

const WARM_UP_ROUNDS: usize = 1_000;
const MEASURED_ROUNDS: usize = 10_000;
const MAX_GROWTH: usize = 4 * 1024 * 1024;

/// Keeps every job until the round takes them.
#[derive(Default)]
struct DeferredJobs {
    jobs: Mutex<Vec<Job>>,
}

impl JobSystem for DeferredJobs {
    fn max_concurrency(&self) -> u32 {
        4
    }

    fn queue_job(&self, job: Job) {
        self.jobs.lock().unwrap().push(job);
    }
}

/// One world on its own deferring job system, dropped before its jobs.
fn deferred_round() {
    let jobs = Arc::new(DeferredJobs::default());
    let settings = WorldSettings::default()
        .max_bodies(16)
        .max_body_pairs(64)
        .max_contact_constraints(64)
        .temp_allocator_size(1024 * 1024)
        .job_system(jobs.clone());
    let mut world = PhysicsWorld::new(settings).unwrap();
    add_floor(&mut world);
    for i in 0..3 {
        add_cube(&mut world, RVec3::new(1.5 * i as Real, 2.0, 0.0));
    }
    step(&mut world, 2);
    drop(world);

    let mut deferred = std::mem::take(&mut *jobs.jobs.lock().unwrap());
    assert!(deferred.len() > 1);
    let to_run = deferred.split_off(deferred.len() / 2);
    drop(deferred);
    for job in to_run {
        job.run();
    }
}

/// One world on a job system that calls `Job::run` inside `queue_job`, which leaves every job
/// to the stepping thread.
fn inline_round() {
    let settings = WorldSettings::default()
        .max_bodies(16)
        .max_body_pairs(64)
        .max_contact_constraints(64)
        .temp_allocator_size(1024 * 1024)
        .job_system(Arc::new(InlineJobs { concurrency: 3 }));
    let mut world = PhysicsWorld::new(settings).unwrap();
    add_floor(&mut world);
    add_cube(&mut world, RVec3::new(0.0, 2.0, 0.0));
    step(&mut world, 2);
}

#[test]
fn caller_job_systems_do_not_leak() {
    let round = |i: usize| {
        if i.is_multiple_of(2) {
            deferred_round();
        } else {
            inline_round();
        }
    };
    (0..WARM_UP_ROUNDS).for_each(round);
    let before = private_bytes();
    (0..MEASURED_ROUNDS).for_each(round);
    let growth = private_bytes().saturating_sub(before);
    assert!(
        growth < MAX_GROWTH,
        "private bytes grew by {growth} over {MEASURED_ROUNDS} rounds"
    );
}
