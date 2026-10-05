//! A leak gate for contact control, in two phases against a budget of 100 bytes per round each:
//! the extension's contact listener with validation, collision group tables and the groups of
//! rigid and soft bodies, the native character contact listener created for every character
//! update, and contact-cache invalidations.
//!
//! - Replacement: in one world with two characters, each round sets a validating contact
//!   listener ten times, builds a group table, creates four grouped cubes and a grouped cloth,
//!   updates both characters five times with a character contact listener, invalidates a
//!   contact cache, steps and removes the bodies again.
//! - World lifecycle: each round does the same in a new world with one worker thread, which it
//!   drops with its listeners set and its bodies alive.
//!
//! It measures the private bytes of the process as `listener_leaks` does, with the same warm-up
//! and the median of seven blocks of rounds (`docs/events.md` has the reasons). The workload is
//! checked to call both listeners. The first phase was checked against builds that never
//! destroyed the per-update character listener, never released a group table and never
//! destroyed the native contact listener; each exceeded the budget. The file holds exactly one
//! test, so its binary runs alone.
#![cfg(windows)]

mod common;

use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::Arc;

use common::memory::private_bytes;
use common::soft_body::Cloth;
use common::*;
use oxijolt::*;

const WARM_UP_ROUNDS: usize = 500;
const WORLD_WARM_UP_ROUNDS: usize = 4_000;
/// Measured blocks per phase; the median of an odd count is one block's growth.
const BLOCKS: usize = 7;
const BLOCK_ROUNDS: usize = 1_000;
/// Bytes per measured round.
const MAX_GROWTH_PER_ROUND: i64 = 100;
const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);

/// Counts the validations it accepts.
#[derive(Default)]
struct CountingValidator(AtomicU32);

impl ContactListener for CountingValidator {
    fn contact_validate(&self, _: &ContactCandidate) -> ValidateResult {
        self.0.fetch_add(1, Ordering::Relaxed);
        ValidateResult::AcceptContact
    }
}

/// Counts the body velocities it is asked to adjust.
#[derive(Default)]
struct CountingCharacterListener(AtomicU32);

impl CharacterContactListener for CountingCharacterListener {
    fn adjust_body_velocity(&self, _: CharacterId, _: BodyId, _: u64, _: &mut BodyVelocity) {
        self.0.fetch_add(1, Ordering::Relaxed);
    }
}

/// Calls the round's listeners made, to check that the workload creates what it should.
#[derive(Default)]
struct Calls {
    validations: u32,
    adjustments: u32,
}

/// A table of 64 sub groups with the neighbouring pairs disabled. Its bit table (252 bytes) is
/// large enough that a table leaked every round exceeds the budget.
fn group_table() -> GroupFilterTable {
    let mut builder = GroupFilterTableBuilder::new(64).unwrap();
    for sub_group in 0..63 {
        builder.disable_collision(sub_group, sub_group + 1).unwrap();
    }
    builder.build()
}

/// A world with a floor and two characters standing on it.
fn character_world(worker_threads: u32) -> (PhysicsWorld, [CharacterId; 2]) {
    let mut world = world(GRAVITY, worker_threads);
    add_floor(&mut world);
    let capsule = Shape::new_capsule(0.5, 0.3).unwrap();
    let settings = CharacterSettings::new(&capsule)
        .shape_offset(Vec3::new(0.0, 0.8, 0.0))
        .collide_with_characters(true);
    let characters = [-5.0, 5.0].map(|x| {
        world
            .create_character(&settings, RVec3::new(x, 0.0, 5.0), Quat::IDENTITY)
            .unwrap()
    });
    (world, characters)
}

/// One round's work in `world`; returns the bodies it created.
fn round_in(world: &mut PhysicsWorld, characters: &[CharacterId; 2]) -> (Vec<BodyId>, Calls) {
    for _ in 0..10 {
        world.set_contact_listener(Some(Arc::new(CountingValidator::default())));
    }
    let validator = Arc::new(CountingValidator::default());
    world.set_contact_listener(Some(validator.clone()));
    let table = group_table();
    let shape = Shape::new_box(Vec3::new(0.25, 0.25, 0.25)).unwrap();
    let mut ids: Vec<BodyId> = (0..4)
        .map(|i| {
            let group = CollisionGroup::new(&table, 1, i).unwrap();
            let settings = BodySettings::new_dynamic()
                .position(RVec3::new(i as Real, 0.26, 0.0))
                .collision_group(group);
            world.create_body(&shape, &settings).unwrap()
        })
        .collect();
    let cloth = Cloth::new(4, 0.2)
        .builder()
        .create_constraints(SoftBodyBendType::None, SoftBodyVertexAttributes::default())
        .build()
        .unwrap();
    let cloth_settings = SoftBodySettings::default()
        .position(RVec3::new(-3.0, 0.3, 0.0))
        .collision_group(CollisionGroup::new(&table, 1, 7).unwrap());
    ids.push(world.create_soft_body(&cloth, &cloth_settings).unwrap());
    drop(table);

    let character_listener = Arc::new(CountingCharacterListener::default());
    world.set_character_contact_listener(Some(character_listener.clone()));
    for _ in 0..5 {
        for &id in characters {
            world
                .update_character(
                    id,
                    DT,
                    GRAVITY,
                    &ExtendedUpdateSettings::default(),
                    &QueryFilter::new(),
                )
                .unwrap();
        }
    }
    world.body_mut(ids[0]).unwrap().invalidate_contact_cache();
    step(world, 2);
    let calls = Calls {
        validations: validator.0.load(Ordering::Relaxed),
        adjustments: character_listener.0.load(Ordering::Relaxed),
    };
    (ids, calls)
}

fn replacement_round(world: &mut PhysicsWorld, characters: &[CharacterId; 2]) {
    let (ids, _) = round_in(world, characters);
    for id in ids {
        world.remove_body(id).unwrap();
    }
    step(world, 1);
    world.take_events();
}

/// A new world, dropped with its listeners set and its bodies alive.
fn world_round() {
    let (mut world, characters) = character_world(1);
    round_in(&mut world, &characters);
}

/// Runs `warm_up` rounds, then [`BLOCKS`] blocks of [`BLOCK_ROUNDS`] rounds, and checks that the
/// private bytes of the median block grew within the budget.
fn assert_rounds_do_not_leak(phase: &str, warm_up: usize, mut round: impl FnMut()) {
    (0..warm_up).for_each(|_| round());
    let mut growths = Vec::with_capacity(BLOCKS);
    for _ in 0..BLOCKS {
        let before = private_bytes() as i64;
        (0..BLOCK_ROUNDS).for_each(|_| round());
        growths.push(private_bytes() as i64 - before);
    }
    println!("{phase}: private bytes grown per block: {growths:?}");
    let mut sorted = growths.clone();
    sorted.sort_unstable();
    let median = sorted[BLOCKS / 2];
    assert!(
        median < MAX_GROWTH_PER_ROUND * BLOCK_ROUNDS as i64,
        "{phase}: the median block grew by {median} bytes (blocks: {growths:?})"
    );
}

#[test]
fn contact_control_does_not_leak() {
    let (mut world, characters) = character_world(1);
    let (ids, calls) = round_in(&mut world, &characters);
    assert!(calls.validations > 0, "the validating listener is called");
    assert!(calls.adjustments > 0, "the character listener is called");
    for id in ids {
        world.remove_body(id).unwrap();
    }
    assert_rounds_do_not_leak("replacement", WARM_UP_ROUNDS, || {
        replacement_round(&mut world, &characters)
    });
    drop(world);
    assert_rounds_do_not_leak("world lifecycle", WORLD_WARM_UP_ROUNDS, world_round);
}
