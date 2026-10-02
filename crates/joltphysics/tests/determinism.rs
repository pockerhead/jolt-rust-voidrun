//! Same-machine determinism through the safe API: the per-tick state of a scene must be
//! bit-identical whether the world steps with 1 or 4 worker threads, also across a rebase, and
//! the order in which bodies are created must decide their ids.
//!
//! Each run happens in its own child process (this test binary, running the ignored
//! `determinism_child` test), so no state leaks between runs; see `common::determinism`.

mod common;

use common::determinism::*;
use common::*;
use joltphysics::*;

/// Ticks of the stacks scene.
const TICKS: usize = 120;

/// Bytes recorded per body per tick by [`record_body`]: id, position, rotation, linear and
/// angular velocity, and the sleeping flag.
const BODY_RECORD_SIZE: usize = 4 + 3 * size_of::<Real>() + 4 * 4 + 3 * 4 + 3 * 4 + 1;

/// Bodies in the stacks scene: the floor and the cubes.
fn stacks_body_count() -> usize {
    1 + stacks_scene().len()
}

/// The rotation of the tilted rebases: 30 degrees about a tilted axis.
fn tilted_rotation() -> Quat {
    let axis = Vec3::new(1.0, 2.0, 0.5);
    let axis = Vec3::new(
        axis.x / length(axis),
        axis.y / length(axis),
        axis.z / length(axis),
    );
    quat_about(axis, 30.0_f32.to_radians())
}

/// The translation of the tilted rebases.
fn tilted_translation() -> RVec3 {
    RVec3::new(12.5, -3.0, 7.25)
}

/// Runs the stacks scene and returns one state record per step, of every body in scene order.
/// Bodies are created in scene order (floor first), or in reverse for `"reversed"`.
/// `"rebased"` creates them in scene order and moves the world into a tilted frame halfway
/// through.
fn run_stacks(worker_threads: u32, variant: &str) -> Digest {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), worker_threads);
    let ids = match variant {
        "forward" | "rebased" => build_stacks(&mut world),
        "reversed" => {
            let mut ids: Vec<BodyId> = stacks_scene()
                .into_iter()
                .rev()
                .map(|position| add_cube(&mut world, position))
                .collect();
            ids.push(add_floor(&mut world));
            ids.reverse();
            ids
        }
        variant => panic!("unknown stacks variant {variant}"),
    };
    let mut digest = Digest::new();
    for tick in 0..TICKS {
        if variant == "rebased" && tick == TICKS / 2 {
            world
                .rebase(&ids, tilted_rotation(), tilted_translation())
                .unwrap();
        }
        assert!(world.step(DT).unwrap().is_complete());
        let record = digest.push();
        for &id in &ids {
            record_body(&world, id, &mut record.state);
        }
    }
    digest
}

/// The raw body ids of the first tick, in scene order.
fn first_tick_ids(digest: &Digest) -> Vec<u32> {
    digest.ticks[0]
        .state
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
#[ignore = "child process of the determinism gates"]
fn determinism_child() {
    let Some((scenario, threads, variant)) = child_request() else {
        return;
    };
    let digest = match scenario.as_str() {
        "stacks" => run_stacks(threads, &variant),
        scenario => panic!("unknown scenario {scenario}"),
    };
    finish_child(&digest);
}

fn stacks_in_child(threads: u32, variant: &str) -> Digest {
    digest_in_child("determinism_child", "stacks", threads, variant)
}

#[test]
fn stacks_digest_is_identical_across_thread_counts() {
    let one_thread = stacks_in_child(1, "forward");
    let four_threads = stacks_in_child(4, "forward");
    let reversed = stacks_in_child(1, "reversed");
    let rebased_one_thread = stacks_in_child(1, "rebased");
    let rebased_four_threads = stacks_in_child(4, "rebased");

    assert_eq!(one_thread.ticks.len(), TICKS);
    assert_eq!(
        one_thread.ticks[0].state.len(),
        stacks_body_count() * BODY_RECORD_SIZE
    );
    assert_same("stacks, 1 vs 4 worker threads", &one_thread, &four_threads);
    assert_same(
        "rebased stacks, 1 vs 4 worker threads",
        &rebased_one_thread,
        &rebased_four_threads,
    );
    // The rebase happened: the rebased run leaves the forward run's frame.
    assert!(first_divergence(&rebased_one_thread, &one_thread).is_some());

    // Insertion order is part of the state: reversing creation reverses the ids of the same
    // bodies, so the digest changes.
    let created_forward: Vec<u32> = (0..stacks_body_count()).map(nth_body_id).collect();
    let created_reversed: Vec<u32> = (0..stacks_body_count()).rev().map(nth_body_id).collect();
    assert_eq!(first_tick_ids(&one_thread), created_forward);
    assert_eq!(first_tick_ids(&reversed), created_reversed);
    assert!(first_divergence(&reversed, &one_thread).is_some());
}

/// A digest of `ticks` ticks whose sections hold the given bytes.
fn digest_of(ticks: &[(&[u8], &[u8])]) -> Digest {
    let mut digest = Digest::new();
    for &(shape, state) in ticks {
        let tick = digest.push();
        tick.shape.extend_from_slice(shape);
        tick.state.extend_from_slice(state);
    }
    digest
}

#[test]
fn digest_encoding_round_trips() {
    for digest in [
        Digest::new(),
        digest_of(&[(&[], &[])]),
        digest_of(&[(&[1, 2, 3], &[]), (&[], &[4]), (&[5, 6], &[7, 8, 9])]),
    ] {
        assert_eq!(Digest::decode(&digest.encode()), Ok(digest));
    }
}

#[test]
fn digest_decoding_rejects_malformed_input() {
    let bytes = digest_of(&[(&[1, 2, 3], &[4, 5])]).encode();
    // Tick count 1, shape length 3: offsets 0 and 4; the shape bytes start at offset 8.
    let truncated_length = &bytes[..6];
    let truncated_section = &bytes[..9];
    let mut trailing = bytes.clone();
    trailing.push(0);
    for (input, expected) in [
        (truncated_length, "truncated length at offset 4"),
        (truncated_section, "truncated section at offset 8"),
        (&trailing[..], "1 trailing bytes at offset 17"),
    ] {
        assert_eq!(Digest::decode(input), Err(expected.to_owned()));
    }
}

#[test]
fn first_divergence_names_tick_section_and_byte() {
    let base = digest_of(&[(&[1, 2], &[3, 4]), (&[5, 6], &[7, 8, 9])]);
    assert_eq!(first_divergence(&base, &base.clone()), None);

    let state = digest_of(&[(&[1, 2], &[3, 4]), (&[5, 6], &[7, 0, 9])]);
    let expected = Divergence::Tick {
        tick: 1,
        section: Section::State,
        byte: 1,
    };
    assert_eq!(first_divergence(&base, &state), Some(expected));
    assert_eq!(expected.to_string(), "tick 1, state byte 1");

    // The shape section of a tick is compared before its state section.
    let shape_and_state = digest_of(&[(&[1, 2], &[3, 4]), (&[5, 0], &[0, 8, 9])]);
    assert_eq!(
        first_divergence(&base, &shape_and_state),
        Some(Divergence::Tick {
            tick: 1,
            section: Section::Shape,
            byte: 1,
        })
    );

    let shorter = digest_of(&[(&[1, 2], &[3, 4]), (&[5, 6], &[7, 8])]);
    assert_eq!(
        first_divergence(&base, &shorter),
        Some(Divergence::Tick {
            tick: 1,
            section: Section::State,
            byte: 2,
        })
    );

    let fewer_ticks = digest_of(&[(&[1, 2], &[3, 4])]);
    let expected = Divergence::TickCount { a: 2, b: 1 };
    assert_eq!(first_divergence(&base, &fewer_ticks), Some(expected));
    assert_eq!(expected.to_string(), "tick counts 2 and 1");
}

#[test]
fn assert_same_panics_with_the_divergence() {
    let a = digest_of(&[(&[1], &[2])]);
    let b = digest_of(&[(&[1], &[3])]);
    assert_same("equal", &a, &a.clone());
    let payload = std::panic::catch_unwind(|| assert_same("runs", &a, &b)).unwrap_err();
    assert_eq!(
        payload.downcast_ref::<String>().map(String::as_str),
        Some("runs: digests diverge at tick 0, state byte 0")
    );
}
