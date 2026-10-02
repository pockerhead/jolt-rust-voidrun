//! Same-machine determinism through the safe API: the per-tick state of a scene must be
//! bit-identical whether the world steps with 1 or 4 worker threads, and the order in which
//! bodies are created must decide their ids.
//!
//! Each run happens in its own child process (this test binary, running the ignored
//! `determinism_child` test), so no state leaks between runs.

mod common;

use std::path::Path;
use std::process::Command;

use common::*;
use rolt::*;

const CHILD_ENV: &str = "ROLT_DIGEST_CHILD";
const TICKS: usize = 120;

/// Bytes recorded per body per tick by [`record_body`]: id, position, rotation, linear and
/// angular velocity, and the sleeping flag.
const BODY_RECORD_SIZE: usize = 4 + 3 * size_of::<Real>() + 4 * 4 + 3 * 4 + 3 * 4 + 1;
/// Bodies in the scene: the floor and the cubes.
fn body_count() -> usize {
    1 + stacks_scene().len()
}

/// Runs the stacks scene and returns its digest. Bodies are created in scene order (floor
/// first), or in reverse when `reversed`, and always recorded in scene order.
fn run_scene(worker_threads: u32, reversed: bool) -> Vec<u8> {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), worker_threads);
    let ids = if reversed {
        let mut ids: Vec<BodyId> = stacks_scene()
            .into_iter()
            .rev()
            .map(|position| add_cube(&mut world, position))
            .collect();
        ids.push(add_floor(&mut world));
        ids.reverse();
        ids
    } else {
        build_stacks(&mut world)
    };
    run_digest(&mut world, &ids, TICKS)
}

/// The raw body ids of the first tick, in scene order.
fn first_tick_ids(digest: &[u8]) -> Vec<u32> {
    digest[..body_count() * BODY_RECORD_SIZE]
        .as_chunks::<BODY_RECORD_SIZE>()
        .0
        .iter()
        .map(|record| u32::from_le_bytes(record[..4].try_into().unwrap()))
        .collect()
}

/// The raw id Jolt gives the `n`-th body added to a fresh world: index `n`, sequence 1.
fn nth_body_id(n: usize) -> u32 {
    (1 << 23) | n as u32
}

#[test]
#[ignore = "child process of digest_is_identical_across_thread_counts"]
fn determinism_child() {
    let Ok(request) = std::env::var(CHILD_ENV) else {
        return;
    };
    let mut parts = request.splitn(3, ',');
    let threads: u32 = parts.next().unwrap().parse().unwrap();
    let reversed = match parts.next().unwrap() {
        "forward" => false,
        "reversed" => true,
        order => panic!("unknown order {order}"),
    };
    let output = parts.next().unwrap();
    std::fs::write(output, run_scene(threads, reversed)).unwrap();
}

/// Runs the scene in a child process and returns its digest.
fn digest_in_child(threads: u32, order: &str, output: &Path) -> Vec<u8> {
    let status = Command::new(std::env::current_exe().unwrap())
        .args([
            "determinism_child",
            "--exact",
            "--ignored",
            "--test-threads=1",
        ])
        .env(CHILD_ENV, format!("{threads},{order},{}", output.display()))
        .status()
        .unwrap();
    assert!(status.success(), "child {threads},{order} failed: {status}");
    std::fs::read(output).unwrap()
}

#[test]
fn digest_is_identical_across_thread_counts() {
    let dir = std::env::temp_dir().join(format!("rolt-digest-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();

    let one_thread = digest_in_child(1, "forward", &dir.join("a"));
    let four_threads = digest_in_child(4, "forward", &dir.join("b"));
    let reversed = digest_in_child(1, "reversed", &dir.join("c"));
    std::fs::remove_dir_all(&dir).unwrap();

    let tick_size = body_count() * BODY_RECORD_SIZE;
    assert_eq!(one_thread.len(), TICKS * tick_size);
    if let Some(offset) = one_thread
        .iter()
        .zip(&four_threads)
        .position(|(a, b)| a != b)
    {
        panic!(
            "1-thread and 4-thread digests differ at byte {offset} (tick {})",
            offset / tick_size
        );
    }
    assert_eq!(one_thread.len(), four_threads.len());

    // Insertion order is part of the state: reversing creation reverses the ids of the same
    // bodies, so the digest changes.
    let created_forward: Vec<u32> = (0..body_count()).map(nth_body_id).collect();
    let created_reversed: Vec<u32> = (0..body_count()).rev().map(nth_body_id).collect();
    assert_eq!(first_tick_ids(&one_thread), created_forward);
    assert_eq!(first_tick_ids(&reversed), created_reversed);
    assert_ne!(one_thread, reversed);
}
