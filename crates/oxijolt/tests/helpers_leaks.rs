//! A leak gate for the physics helpers: plane shapes with and without a material on a static
//! body, filtered point queries and buoyancy, round after round in one world.
//!
//! It measures the private bytes of the process (Windows `K32GetProcessMemoryInfo`), because
//! shapes and Jolt's body data are allocated by C++, which a Rust global allocator does not see.
//! As in `body_controls_leaks.rs`, it measures after 3000 warm-up rounds: nine consecutive blocks
//! of 500 rounds. It fails when the median block grows by 100 bytes per round or more, or when
//! all blocks together grow by 4 MiB or more. A control run then forgets one plane shape per
//! round and must fail the median gate. The file holds exactly one test, so its binary runs
//! alone and no parallel test disturbs the counter.
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
const UP: Vec3 = Vec3::new(0.0, 1.0, 0.0);

/// The world every round uses.
struct Scene {
    world: PhysicsWorld,
    material: PhysicsMaterial,
    cube: BodyId,
    other: BodyId,
    round: usize,
}

impl Scene {
    fn new() -> Self {
        let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
        let cube = add_cube(&mut world, RVec3::new(0.0, 0.5, 0.0));
        let half = cube_shape();
        let compound = Shape::new_compound(&[
            child(&half, Vec3::ZERO, 1),
            child(&half, Vec3::new(0.2, 0.0, 0.0), 2),
        ])
        .unwrap();
        let other = world
            .create_body(
                &compound,
                &BodySettings::new_static().position(RVec3::new(0.0, 0.5, 3.0)),
            )
            .unwrap();
        Self {
            world,
            material: PhysicsMaterial::new(5).unwrap(),
            cube,
            other,
            round: 0,
        }
    }

    /// One round: a new plane, with a material every other round, on a static body for one
    /// step with the cube floating on it, then removed; point queries with every filter part.
    fn round(&mut self) {
        self.round += 1;
        let plane = if self.round.is_multiple_of(2) {
            Shape::new_plane_with_material(UP, 0.0, 10.0, &self.material).unwrap()
        } else {
            Shape::new_plane(UP, 0.0, 10.0).unwrap()
        };
        let floor = self
            .world
            .create_body(&plane, &BodySettings::new_static())
            .unwrap();
        drop(plane);
        let water = BuoyancySettings::default().surface(RVec3::new(0.0, 0.6, 0.0), UP);
        self.world
            .body_mut(self.cube)
            .unwrap()
            .apply_buoyancy_impulse(&water, Vec3::new(0.0, -9.81, 0.0), DT)
            .unwrap();
        step(&mut self.world, 1);
        self.world.remove_body(floor).unwrap();
        let layers = [ObjectLayer::NON_MOVING, ObjectLayer::MOVING];
        let filter = QueryFilter::new()
            .object_layers(&layers)
            .child_groups(1 << 2)
            .exclude_body(self.cube);
        for point in [RVec3::new(0.1, 0.5, 3.0), RVec3::new(0.0, 0.5, 0.0)] {
            let hits = self.world.collide_point(point, &filter).unwrap();
            assert!(hits.iter().all(|hit| hit.body == self.other));
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
fn helpers_do_not_leak() {
    let mut scene = Scene::new();
    for _ in 0..WARM_UP_ROUNDS {
        scene.round();
    }
    let blocks = block_growth(|| scene.round());
    eprintln!("helpers: growth per block of {BLOCK_ROUNDS} rounds: {blocks:?}");
    let (median_growth, total): (usize, usize) = (median(&blocks), blocks.iter().sum());
    assert!(
        median_growth < MAX_ROUND_GROWTH * BLOCK_ROUNDS,
        "median block growth {median_growth} bytes: a helper leaks ({blocks:?})"
    );
    assert!(
        total < MAX_TOTAL_GROWTH,
        "total growth {total} bytes: a helper leaks ({blocks:?})"
    );

    // The control: the same rounds, each forgetting one plane, must show.
    let blocks = block_growth(|| {
        scene.round();
        std::mem::forget(Shape::new_plane(UP, 0.0, 10.0).unwrap());
    });
    eprintln!("control: growth per block of {BLOCK_ROUNDS} rounds: {blocks:?}");
    assert!(
        median(&blocks) >= MAX_ROUND_GROWTH * BLOCK_ROUNDS,
        "the control forgot a plane per round but the median block grew only by {} ({blocks:?})",
        median(&blocks)
    );
}
