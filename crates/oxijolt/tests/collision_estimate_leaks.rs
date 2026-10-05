//! A leak gate for collision estimates: each round a row of 32 boxes lands flat on a floor, so
//! that each makes a four-point Added contact whose estimate joltc allocates an impulse array
//! for, and is removed again.
//!
//! It measures the private bytes of the process (Windows `K32GetProcessMemoryInfo`), because
//! joltc allocates the impulse arrays with `malloc`, which a Rust global allocator does not see.
//! As in `body_controls_leaks.rs`, it measures after 3000 warm-up rounds: nine consecutive blocks
//! of 500 rounds. It fails when the median block grows by 100 bytes per round or more, or when
//! all blocks together grow by 4 MiB or more. A control run then forgets 32 arrays of four
//! floats per round, the size of the round's impulse arrays, and must fail the median gate. The
//! file holds exactly one test, so its binary runs alone and no parallel test disturbs the
//! counter.
#![cfg(windows)]

mod common;

use common::memory::private_bytes;
use common::*;
use oxijolt::*;

const WARM_UP_ROUNDS: usize = 3000;
const BLOCK_ROUNDS: usize = 500;
const BLOCKS: usize = 9;
const MAX_ROUND_GROWTH: usize = 100;
const MAX_TOTAL_GROWTH: usize = 4 * 1024 * 1024;
const BOXES: usize = 32;

struct Scene {
    world: PhysicsWorld,
    shape: Shape,
}

impl Scene {
    fn new() -> Self {
        let mut world = world(Vec3::ZERO, 1);
        world.set_event_settings(EventSettings::default().collision_estimates(true));
        add_floor(&mut world);
        Self {
            world,
            shape: Shape::new_box(Vec3::new(0.2, 0.2, 0.2)).unwrap(),
        }
    }

    /// One round: the boxes appear 1 mm above the floor moving down, touch it in one step and
    /// are removed.
    fn round(&mut self) {
        let ids: Vec<BodyId> = (0..BOXES)
            .map(|i| {
                let settings = BodySettings::new_dynamic()
                    .position(RVec3::new(i as Real - 16.0, 0.201, 0.0))
                    .linear_velocity(Vec3::new(0.0, -1.0, 0.0));
                self.world.create_body(&self.shape, &settings).unwrap()
            })
            .collect();
        step(&mut self.world, 1);
        let estimated = self
            .world
            .take_events()
            .contacts
            .iter()
            .filter(|event| {
                matches!(
                    event,
                    ContactEvent::Added { estimate: Some(e), .. } if e.contact_impulses.len() == 4
                )
            })
            .count();
        assert_eq!(estimated, BOXES);
        for id in ids {
            self.world.remove_body(id).unwrap();
        }
    }
}

/// The private-bytes growth of each of [`BLOCKS`] consecutive blocks of `round`s.
fn block_growth(mut round: impl FnMut()) -> Vec<usize> {
    (0..BLOCKS)
        .map(|_| {
            let before = private_bytes();
            for _ in 0..BLOCK_ROUNDS {
                round();
            }
            private_bytes().saturating_sub(before)
        })
        .collect()
}

fn median(blocks: &[usize]) -> usize {
    let mut sorted = blocks.to_vec();
    sorted.sort_unstable();
    sorted[sorted.len() / 2]
}

#[test]
fn collision_estimates_do_not_leak() {
    let mut scene = Scene::new();
    for _ in 0..WARM_UP_ROUNDS {
        scene.round();
    }
    let blocks = block_growth(|| scene.round());
    eprintln!("estimates: growth per block of {BLOCK_ROUNDS} rounds: {blocks:?}");
    let (median_growth, total): (usize, usize) = (median(&blocks), blocks.iter().sum());
    assert!(
        median_growth < MAX_ROUND_GROWTH * BLOCK_ROUNDS,
        "median block growth {median_growth} bytes: estimates leak ({blocks:?})"
    );
    assert!(
        total < MAX_TOTAL_GROWTH,
        "total growth {total} bytes: estimates leak ({blocks:?})"
    );

    // The control: the same rounds, each forgetting arrays the size of its impulse arrays.
    let blocks = block_growth(|| {
        scene.round();
        for _ in 0..BOXES {
            std::mem::forget(vec![0.0_f32; 4]);
        }
    });
    eprintln!("control: growth per block of {BLOCK_ROUNDS} rounds: {blocks:?}");
    assert!(
        median(&blocks) >= MAX_ROUND_GROWTH * BLOCK_ROUNDS,
        "the control forgot {BOXES} arrays per round but the median block grew only by {} ({blocks:?})",
        median(&blocks)
    );
}
