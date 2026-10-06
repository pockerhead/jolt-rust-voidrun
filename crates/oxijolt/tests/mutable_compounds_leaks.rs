//! A leak gate for compound edits: an editor built from children with their own materials,
//! every kind of edit, refused edits, publications installed on a static and a dynamic body, a
//! step, and a body created from a publication and removed, round after round in one world.
//!
//! It measures the private bytes of the process (Windows `K32GetProcessMemoryInfo`), because
//! shapes and materials are allocated by C++, which a Rust global allocator does not see. After
//! 3000 warm-up rounds it measures nine consecutive blocks of 500 rounds and fails when the
//! median block grows by 100 bytes per round or more, or when all blocks together grow by 4 MiB
//! or more (the reasons are in `mesh_shapes_leaks.rs`). A control run then forgets one
//! publication of six children per round, measured at about 1.1 kB, and must fail the median
//! gate. The file holds exactly one test, so its binary runs alone and no parallel test
//! disturbs the counter.
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

/// What every round reuses: the bodies the publications are installed on and a child that
/// uses all 32 sub-shape id bits.
struct Fixture {
    ledge: BodyId,
    debris: BodyId,
    deep: Shape,
}

/// A 33 x 33 heightfield (13 id bits) inside compounds of 64, 64 and 128 children: 32 bits.
fn height_field_at_32_bits() -> Shape {
    let cube = cube_shape();
    let mut shape = flat_height_field();
    for count in [64, 64, 128] {
        let mut children = vec![child(&shape, Vec3::ZERO, 0)];
        children.extend(
            (1..count).map(|i| child(&cube, Vec3::new(20.0 + 1.5 * i as f32, 0.0, 0.0), i)),
        );
        shape = Shape::new_compound(&children).unwrap();
    }
    shape
}

/// One round of edits, refusals, publications and bodies; returns the editor.
fn round(world: &mut PhysicsWorld, fixture: &Fixture) -> MutableCompound {
    let materials = [
        PhysicsMaterial::new(1).unwrap(),
        PhysicsMaterial::new(2).unwrap(),
    ];
    let marked: Vec<_> = materials
        .iter()
        .map(|material| {
            Shape::new_box_with_material(Vec3::new(0.3, 0.3, 0.3), 0.05, material).unwrap()
        })
        .collect();
    let block = Shape::new_box(Vec3::new(0.3, 0.2, 0.3)).unwrap();
    let ball = Shape::new_sphere(0.25).unwrap();
    let mut editor = MutableCompound::from_children(&[
        child(&marked[0], Vec3::ZERO, 0),
        child(&marked[1], Vec3::new(0.7, 0.0, 0.0), 1),
        child(&block, Vec3::new(1.4, 0.0, 0.0), 2),
        child(&ball, Vec3::new(0.0, 0.7, 0.0), 3),
        child(&block, Vec3::new(0.7, 0.7, 0.0), 4),
        child(&ball, Vec3::new(1.4, 0.7, 0.0), 5),
    ])
    .unwrap();
    drop((materials, marked));
    editor
        .add_shape(&child(&ball, Vec3::new(0.0, 1.4, 0.0), 6))
        .unwrap();
    editor.remove_shape(2).unwrap();
    editor
        .modify_shape(1, Vec3::new(0.7, 0.1, 0.0), Quat::IDENTITY, None)
        .unwrap();
    editor
        .modify_shape(3, Vec3::new(0.7, 0.7, 0.1), Quat::IDENTITY, Some(&ball))
        .unwrap();
    assert!(editor.remove_shape(99).is_err());
    let twisted = Quat::from_xyzw(0.0, 0.0, 0.0, 2.0);
    assert!(editor
        .modify_shape(0, Vec3::ZERO, twisted, Some(&block))
        .is_err());
    assert!(editor
        .add_shape(&child(&fixture.deep, Vec3::ZERO, 7))
        .is_err());

    let first = editor.to_shape().unwrap();
    let second = editor.to_shape().unwrap();
    world
        .body_mut(fixture.ledge)
        .unwrap()
        .set_shape(&first, None, Activation::DontActivate)
        .unwrap();
    world
        .body_mut(fixture.debris)
        .unwrap()
        .set_shape(&second, None, Activation::Activate)
        .unwrap();
    step(world, 1);
    let extra = world
        .create_body(
            &first,
            &BodySettings::new_dynamic().position(RVec3::new(10.0, 2.0, 0.0)),
        )
        .unwrap();
    world.remove_body(extra).unwrap();
    editor
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
fn compound_edits_do_not_leak() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    add_floor(&mut world);
    let cube = cube_shape();
    let ledge = world
        .create_body(
            &cube,
            &BodySettings::new_static().position(RVec3::new(-10.0, 3.0, 0.0)),
        )
        .unwrap();
    let debris = world
        .create_body(
            &cube,
            &BodySettings::new_dynamic().position(RVec3::new(0.0, 1.0, 0.0)),
        )
        .unwrap();
    let fixture = Fixture {
        ledge,
        debris,
        deep: height_field_at_32_bits(),
    };
    for _ in 0..WARM_UP_ROUNDS {
        round(&mut world, &fixture);
    }
    let blocks = block_growth(|| {
        round(&mut world, &fixture);
    });
    eprintln!("compound edits: growth per block of {BLOCK_ROUNDS} rounds: {blocks:?}");
    let (median_growth, total): (usize, usize) = (median(&blocks), blocks.iter().sum());
    assert!(
        median_growth < MAX_ROUND_GROWTH * BLOCK_ROUNDS,
        "median block growth {median_growth} bytes: an edit leaks ({blocks:?})"
    );
    assert!(
        total < MAX_TOTAL_GROWTH,
        "total growth {total} bytes: an edit leaks ({blocks:?})"
    );

    // The control: the same rounds, each forgetting one publication, must show.
    let blocks = block_growth(|| {
        let editor = round(&mut world, &fixture);
        std::mem::forget(editor.to_shape().unwrap());
    });
    eprintln!("control: growth per block of {BLOCK_ROUNDS} rounds: {blocks:?}");
    assert!(
        median(&blocks) >= MAX_ROUND_GROWTH * BLOCK_ROUNDS,
        "the control forgot a publication per round but the median block grew only by {} \
         ({blocks:?})",
        median(&blocks)
    );
}
