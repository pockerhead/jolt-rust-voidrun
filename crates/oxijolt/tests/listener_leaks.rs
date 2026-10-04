//! A leak gate for events, contact listeners and materials, in two phases against a budget of
//! 100 bytes per round each.
//!
//! - Replacement: in one world that records every event, each round replaces the event settings
//!   and the contact listener ten times, makes shapes of new materials, creates bodies and a
//!   cloth from them, steps, takes the events and removes the bodies again.
//! - World lifecycle: each round creates a world with one worker thread, installs every event
//!   and a contact listener, creates material shapes, bodies and a cloth, steps, and drops the
//!   world with its listeners attached, its bodies alive and the last step's events untaken.
//!
//! It measures the private bytes of the process, because the native listeners, materials and
//! worlds are allocated by C++, which a Rust global allocator does not see. A new world per
//! round grows the counter by about 2 MB over its first few thousand rounds, hence the long
//! warm-up of the second phase, and the heap still commits or releases a step of about 2 MB now
//! and then afterwards. Neither is a leak (`docs/events.md` has the measurements), so each phase
//! measures seven consecutive blocks of rounds and holds the median block to the budget: a leak
//! grows every block, a heap step only one. A leak that grows three or fewer of the seven blocks
//! passes the median. The file holds exactly one test, so its binary runs
//! alone and no parallel test disturbs the counter. The first phase was checked against builds
//! that never released a material or never destroyed the native contact listener, the second
//! against one whose dropped world forgot its listeners; each exceeded the budget.
#![cfg(windows)]

mod common;

use std::sync::Arc;

use common::events::*;
use common::memory::private_bytes;
use common::*;
use oxijolt::*;

const WARM_UP_ROUNDS: usize = 500;
const WORLD_WARM_UP_ROUNDS: usize = 4_000;
/// Measured blocks per phase; the median of an odd count is one block's growth.
const BLOCKS: usize = 7;
const BLOCK_ROUNDS: usize = 1_000;
/// Bytes per measured round.
const MAX_GROWTH_PER_ROUND: i64 = 100;

fn every_kind() -> EventSettings {
    EventSettings::default()
        .persisted_contacts(true)
        .body_activation(true)
        .soft_body_contacts(true)
        .soft_body_validations(true)
}

/// Four shapes, each of a new material.
fn material_shapes(round: usize) -> [Shape; 4] {
    let materials: Vec<PhysicsMaterial> = (0..4)
        .map(|i| PhysicsMaterial::new(round as u64 * 4 + i).unwrap())
        .collect();
    [
        Shape::new_box_with_material(Vec3::new(0.25, 0.25, 0.25), 0.05, &materials[0]).unwrap(),
        Shape::new_sphere_with_material(0.25, &materials[1]).unwrap(),
        Shape::new_capsule_with_material(0.2, 0.2, &materials[2]).unwrap(),
        Shape::new_cylinder_with_material(0.25, 0.25, 0.05, &materials[3]).unwrap(),
    ]
}

/// Creates a body of each material shape and a cloth, and returns their ids.
fn add_bodies(world: &mut PhysicsWorld, round: usize) -> Vec<BodyId> {
    let shapes = material_shapes(round);
    let mut ids: Vec<BodyId> = shapes
        .iter()
        .enumerate()
        .map(|(i, shape)| {
            let position = RVec3::new(-1.5 + i as Real, 0.3, 0.0);
            world
                .create_body(shape, &BodySettings::new_dynamic().position(position))
                .unwrap()
        })
        .collect();
    ids.push(add_cloth(world, RVec3::new(10.0, 0.3, 0.0), Quat::IDENTITY));
    ids
}

fn add_ground(world: &mut PhysicsWorld) {
    add_two_box_floor(world);
    add_table(world, 10.0);
}

fn replacement_round(world: &mut PhysicsWorld, round: usize) {
    for i in 0..10 {
        let settings = if i % 2 == 0 {
            EventSettings::default()
        } else {
            every_kind()
        };
        world.set_event_settings(settings);
        world.set_contact_listener(Some(Arc::new(NoOp)));
    }
    world.set_event_settings(every_kind());
    world.set_contact_listener(Some(Arc::new(RoughContacts)));
    let ids = add_bodies(world, round);
    for _ in 0..5 {
        step(world, 1);
        assert!(!world.take_events().is_empty());
    }
    for id in ids {
        world.remove_body(id).unwrap();
    }
    step(world, 1);
    world.take_events();
}

/// A world that records every event, dropped with its listeners, bodies and last events.
fn world_round(round: usize) {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    world.set_event_settings(every_kind());
    world.set_contact_listener(Some(Arc::new(RoughContacts)));
    add_ground(&mut world);
    add_bodies(&mut world, round);
    for _ in 0..4 {
        step(&mut world, 1);
        assert!(!world.take_events().is_empty());
    }
    step(&mut world, 1);
}

/// Runs `warm_up` rounds, then [`BLOCKS`] blocks of [`BLOCK_ROUNDS`] rounds, and checks that the
/// private bytes of the median block grew within the budget.
fn assert_rounds_do_not_leak(phase: &str, warm_up: usize, mut round: impl FnMut(usize)) {
    (0..warm_up).for_each(&mut round);
    let mut growths = Vec::with_capacity(BLOCKS);
    for block in 0..BLOCKS {
        let first = warm_up + block * BLOCK_ROUNDS;
        let before = private_bytes() as i64;
        (first..first + BLOCK_ROUNDS).for_each(&mut round);
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
fn listeners_materials_and_worlds_do_not_leak() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    add_ground(&mut world);
    assert_rounds_do_not_leak("replacement", WARM_UP_ROUNDS, |round| {
        replacement_round(&mut world, round)
    });
    drop(world);
    assert_rounds_do_not_leak("world lifecycle", WORLD_WARM_UP_ROUNDS, world_round);
}
