//! A leak gate for the body controls: a shape change to a new shape, impulses, a kinematic move
//! and deactivation with activation, round after round in one world.
//!
//! It measures the private bytes of the process (Windows `K32GetProcessMemoryInfo`), because
//! shapes and Jolt's body data are allocated by C++, which a Rust global allocator does not see.
//! As in `mesh_shapes_leaks.rs`, it measures after 3000 warm-up rounds: nine consecutive blocks
//! of 500 rounds. It fails when the median block grows by 100 bytes per round or more, or when
//! all blocks together grow by 4 MiB or more. A control run then forgets one four-point hull per
//! round and must fail the median gate, which shows the gate sees a leak of that size. This is a
//! regression gate for the references `set_shape` hands to Jolt, not a proof of ownership. The
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

/// The bodies every round drives.
struct Scene {
    world: PhysicsWorld,
    cube: BodyId,
    platform: BodyId,
    round: usize,
}

impl Scene {
    fn new() -> Self {
        let mut world = world(Vec3::ZERO, 1);
        let cube = add_cube(&mut world, RVec3::ZERO);
        let platform = world
            .create_body(
                &cube_shape(),
                &BodySettings::new_kinematic().position(RVec3::new(0.0, 5.0, 0.0)),
            )
            .unwrap();
        Self {
            world,
            cube,
            platform,
            round: 0,
        }
    }

    /// One round: the cube takes a new shape of its own (the old one is released by Jolt), gets
    /// impulses that cancel, sleeps and wakes; the platform moves back and forth; one step.
    fn round(&mut self) {
        self.round += 1;
        let shape = if self.round.is_multiple_of(2) {
            Shape::new_box(Vec3::new(0.5, 0.4, 0.3)).unwrap()
        } else {
            Shape::new_sphere(0.5).unwrap()
        };
        let mut cube = self.world.body_mut(self.cube).unwrap();
        cube.set_shape(&shape, Some(1.0), Activation::Activate)
            .unwrap();
        drop(shape);
        let push = Vec3::new(0.5, 0.0, 0.0);
        cube.add_impulse(push).unwrap();
        cube.add_impulse(Vec3::new(-0.5, 0.0, 0.0)).unwrap();
        cube.add_angular_impulse(Vec3::new(0.0, 0.01, 0.0)).unwrap();
        cube.add_impulse_at_point(push, RVec3::new(0.0, 0.1, 0.0))
            .unwrap();
        cube.deactivate().unwrap();
        cube.activate();
        let x = if self.round.is_multiple_of(2) {
            0.0
        } else {
            0.1
        };
        self.world
            .body_mut(self.platform)
            .unwrap()
            .move_kinematic(RVec3::new(x, 5.0, 0.0), Quat::IDENTITY, DT)
            .unwrap();
        step(&mut self.world, 1);
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
fn body_controls_do_not_leak() {
    let mut scene = Scene::new();
    for _ in 0..WARM_UP_ROUNDS {
        scene.round();
    }
    let blocks = block_growth(|| scene.round());
    eprintln!("body controls: growth per block of {BLOCK_ROUNDS} rounds: {blocks:?}");
    let (median_growth, total): (usize, usize) = (median(&blocks), blocks.iter().sum());
    assert!(
        median_growth < MAX_ROUND_GROWTH * BLOCK_ROUNDS,
        "median block growth {median_growth} bytes: a body control leaks ({blocks:?})"
    );
    assert!(
        total < MAX_TOTAL_GROWTH,
        "total growth {total} bytes: a body control leaks ({blocks:?})"
    );

    // The control: the same rounds, each forgetting one four-point hull, must show.
    let tetrahedron = [
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    ];
    let blocks = block_growth(|| {
        scene.round();
        std::mem::forget(Shape::new_convex_hull(&tetrahedron, 0.05).unwrap());
    });
    eprintln!("control: growth per block of {BLOCK_ROUNDS} rounds: {blocks:?}");
    assert!(
        median(&blocks) >= MAX_ROUND_GROWTH * BLOCK_ROUNDS,
        "the control forgot a hull per round but the median block grew only by {} ({blocks:?})",
        median(&blocks)
    );
}
