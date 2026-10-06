//! A leak gate for reused state buffers and filtered restores: saves into the same buffers with
//! every selection and restores selected bodies, round after round in one world.
//!
//! It measures the private bytes of the process (Windows `K32GetProcessMemoryInfo`), because
//! Jolt's recorders are allocated by C++, which a Rust global allocator does not see. After 5000
//! warm-up rounds it measures seven consecutive blocks of 1000 rounds and fails when the median
//! block grows by 100 bytes per round or more, or when all blocks together grow by 4 MiB or more.
//! A control run then also saves a physics system into a recorder created through
//! `oxijolt-sys` and never destroyed, once per round, and must fail the median gate. The file
//! holds exactly one test, so its binary runs alone and no parallel test disturbs the counter.
#![cfg(windows)]

mod common;

use std::ptr::null;

use common::memory::private_bytes;
use common::raw_system;
use common::rollback::{Inputs, RollbackScene};
use oxijolt::*;
use oxijolt_sys::*;

const WARM_UP_ROUNDS: usize = 5000;
const BLOCK_ROUNDS: usize = 1000;
const BLOCKS: usize = 7;
const MAX_ROUND_GROWTH: usize = 100;
const MAX_TOTAL_GROWTH: usize = 4 * 1024 * 1024;

/// The world and buffers every round uses.
struct Rounds {
    scene: RollbackScene,
    full: WorldState,
    movable: WorldState,
    some: WorldState,
}

impl Rounds {
    fn new() -> Self {
        let mut scene = RollbackScene::new(1);
        for _ in 0..10 {
            scene.tick(Inputs::PLAYED);
        }
        Self {
            scene,
            full: WorldState::new(),
            movable: WorldState::new(),
            some: WorldState::new(),
        }
    }

    /// One round: a save into each buffer with each selection, and restores of some bodies,
    /// of the movable bodies and of everything.
    fn round(&mut self) {
        let world = &mut self.scene.world;
        let some = [self.scene.bodies[1], self.scene.sleeper];
        world
            .save_state_into(BodySelection::All, &mut self.full)
            .unwrap();
        world
            .save_state_into(BodySelection::Movable, &mut self.movable)
            .unwrap();
        world
            .save_state_into(BodySelection::Only(&some), &mut self.some)
            .unwrap();
        world
            .restore_state_of(&self.full, BodySelection::Only(&some))
            .unwrap();
        world
            .restore_state_of(&self.movable, BodySelection::Movable)
            .unwrap();
        world
            .restore_state_of(&self.some, BodySelection::All)
            .unwrap();
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
fn state_buffers_and_filtered_restores_do_not_leak() {
    let mut rounds = Rounds::new();
    for _ in 0..WARM_UP_ROUNDS {
        rounds.round();
    }
    let blocks = block_growth(|| rounds.round());
    eprintln!("state buffers: growth per block of {BLOCK_ROUNDS} rounds: {blocks:?}");
    let (median_growth, total): (usize, usize) = (median(&blocks), blocks.iter().sum());
    assert!(
        median_growth < MAX_ROUND_GROWTH * BLOCK_ROUNDS,
        "median block growth {median_growth} bytes: a state buffer leaks ({blocks:?})"
    );
    assert!(
        total < MAX_TOTAL_GROWTH,
        "total growth {total} bytes: a state buffer leaks ({blocks:?})"
    );

    // The control: the same rounds, each forgetting one recorder, must show.
    // SAFETY: Jolt is initialised (the scene's world exists), and this test is the only one in
    // its binary, so no other thread creates or destroys a physics system.
    let system = unsafe { raw_system() };
    assert!(!system.is_null());
    let blocks = block_growth(|| {
        rounds.round();
        // SAFETY: the system is live and not stepping; a null body list saves every body. The
        // recorder is deliberately never destroyed.
        unsafe {
            let recorder = JPH_StateRecorder_Create();
            JPH_PhysicsSystem_SaveState(system, recorder, JPH_StateRecorderState_Bodies, null(), 0);
        }
    });
    eprintln!("control: growth per block of {BLOCK_ROUNDS} rounds: {blocks:?}");
    // SAFETY: the leaked recorders do not refer to the system; it is destroyed once, by this
    // thread only.
    unsafe { JPH_PhysicsSystem_Destroy(system) };
    assert!(
        median(&blocks) >= MAX_ROUND_GROWTH * BLOCK_ROUNDS,
        "the control forgot a recorder per round but the median block grew only by {} ({blocks:?})",
        median(&blocks)
    );
}
