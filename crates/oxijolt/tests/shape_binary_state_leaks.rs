//! A leak gate for shape binary state: save and restore of a compound of a mesh with materials,
//! a hull and a box, a restore refused by the checksum and one refused by joltc, round after
//! round.
//!
//! It measures the private bytes of the process (Windows `K32GetProcessMemoryInfo`), because
//! shapes, materials and saved states are allocated by C++, which a Rust global allocator does
//! not see. After 3000 warm-up rounds it measures nine consecutive blocks of 500 rounds and fails
//! when the median block grows by 100 bytes per round or more, or all blocks together by 4 MiB or
//! more. A control run then forgets one saved native state per round (several kilobytes) and
//! must fail the median gate. The file holds exactly one test, so its binary runs alone and no
//! parallel test disturbs the counter.
#![cfg(windows)]

mod common;

use std::ptr::null_mut;

use common::memory::private_bytes;
use common::meshes::*;
use oxijolt::*;
use oxijolt_sys::*;

const WARM_UP_ROUNDS: usize = 3000;
const BLOCK_ROUNDS: usize = 500;
const BLOCKS: usize = 9;
const MAX_ROUND_GROWTH: usize = 100;
const MAX_TOTAL_GROWTH: usize = 4 * 1024 * 1024;

fn compound() -> Shape {
    let (a, b) = (
        PhysicsMaterial::new(1).unwrap(),
        PhysicsMaterial::new(2).unwrap(),
    );
    let (vertices, triangles) = grid(16, 0.5, |x, z| 0.1 * (x * z).sin());
    let indices: Vec<u8> = (0..triangles.len()).map(|i| (i % 2) as u8).collect();
    let list = [&a, &b];
    let (mesh, _) = Shape::new_mesh_with_settings(
        &vertices,
        &triangles,
        &MeshSettings::default().materials(&list, &indices),
    )
    .unwrap();
    let hull = Shape::new_convex_hull(&irregular_points(), 0.05).unwrap();
    let block = Shape::new_box(Vec3::new(0.2, 0.2, 0.2)).unwrap();
    let child = |shape, x, user_data| CompoundChild {
        shape,
        position: Vec3::new(x, 0.0, 0.0),
        rotation: Quat::IDENTITY,
        user_data,
    };
    Shape::new_compound(&[
        child(&mesh, 0.0, 0),
        child(&hull, 10.0, 1),
        child(&block, 12.0, 2),
    ])
    .unwrap()
}

/// One round: a save and a restore, a damaged copy refused by the checksum, and a copy whose
/// payload joltc refuses behind a recomputed length (refused by the checksum first, so the
/// joltc path runs through the raw call).
fn round(shape: &Shape) {
    let bytes = shape.save_binary_state().unwrap();
    // SAFETY: the bytes were saved just above by this build, unchanged.
    let restored = unsafe { Shape::restore_binary_state(&bytes) }.unwrap();
    drop(restored);
    let mut damaged = bytes.clone();
    let last = damaged.len() - 1;
    damaged[last] ^= 1;
    // SAFETY: the checksum refuses the damaged copy before joltc reads it.
    assert!(unsafe { Shape::restore_binary_state(&damaged) }.is_err());
    let mut payload = bytes[48..].to_vec();
    payload.push(0);
    // SAFETY: the payload's records were written by this build; joltc refuses the trailing byte
    // after reading every record, so this exercises the refusal after restored shapes exist.
    let refused = unsafe {
        JPH_Shape_RestoreBinaryState(payload.as_ptr().cast(), payload.len(), null_mut(), 0)
    };
    assert!(refused.is_null());
}

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
fn saving_and_restoring_shapes_does_not_leak() {
    let shape = compound();
    for _ in 0..WARM_UP_ROUNDS {
        round(&shape);
    }
    let blocks = block_growth(|| round(&shape));
    eprintln!("binary state: growth per block of {BLOCK_ROUNDS} rounds: {blocks:?}");
    let (median_growth, total): (usize, usize) = (median(&blocks), blocks.iter().sum());
    assert!(
        median_growth < MAX_ROUND_GROWTH * BLOCK_ROUNDS,
        "median block growth {median_growth} bytes: binary state leaks ({blocks:?})"
    );
    assert!(
        total < MAX_TOTAL_GROWTH,
        "total growth {total} bytes: binary state leaks ({blocks:?})"
    );

    // The control: the same rounds, each forgetting one native saved state, must show. The safe
    // API hands out no shape pointer, so the control saves a raw copy restored from the payload.
    let bytes = shape.save_binary_state().unwrap();
    let payload = &bytes[48..];
    // SAFETY: the payload was saved just above by this build; this test owns the shape's one
    // reference and releases it at the end.
    let raw = unsafe {
        JPH_Shape_RestoreBinaryState(payload.as_ptr().cast(), payload.len(), null_mut(), 0)
    };
    assert!(!raw.is_null());
    let blocks = block_growth(|| {
        round(&shape);
        // SAFETY: `raw` is live; the state is deliberately never destroyed.
        let state = unsafe { JPH_Shape_SaveBinaryState(raw, null_mut(), 0) };
        assert!(!state.is_null());
    });
    eprintln!("control: growth per block of {BLOCK_ROUNDS} rounds: {blocks:?}");
    assert!(
        median(&blocks) >= MAX_ROUND_GROWTH * BLOCK_ROUNDS,
        "the control forgot a saved state per round but the median block grew only by {} ({blocks:?})",
        median(&blocks)
    );
    // SAFETY: the test holds the one reference to `raw`.
    unsafe { JPH_Shape_Destroy(raw) };
}
