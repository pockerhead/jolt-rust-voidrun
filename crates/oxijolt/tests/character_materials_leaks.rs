//! A leak gate for the materials characters keep for their cached contacts: a character on a
//! floor of four materials refreshes and updates, the floor's shape is replaced, and the
//! character updates once without moving (reusing its cached contacts) and twice normally.
//!
//! It measures the private bytes of the process (Windows `K32GetProcessMemoryInfo`), because
//! materials are allocated by C++, which a Rust global allocator does not see. After 3000
//! warm-up rounds it measures nine consecutive blocks of 500 rounds and fails when the median
//! block grows by 100 bytes per round or more, or when all blocks together grow by 4 MiB or more
//! (the reasons are in `mesh_shapes_leaks.rs`). A control run then forgets one reference to
//! each of four new materials per round (about 300 bytes together; one alone stays below the
//! gate) and must fail the median gate. The file holds exactly one test, so its binary runs
//! alone and no parallel test disturbs the counter.
#![cfg(windows)]

mod common;

use common::memory::private_bytes;
use common::*;
use oxijolt::*;
use oxijolt_sys::*;

const WARM_UP_ROUNDS: usize = 3000;
const BLOCK_ROUNDS: usize = 500;
const BLOCKS: usize = 9;
const MAX_ROUND_GROWTH: usize = 100;
const MAX_TOTAL_GROWTH: usize = 4 * 1024 * 1024;
const DT: f32 = 1.0 / 60.0;
/// Below the default `min_time_remaining`: the update keeps the cached contacts.
const TINY_DT: f32 = 1e-6;

fn update(world: &mut PhysicsWorld, id: CharacterId, delta_time: f32) {
    let settings = ExtendedUpdateSettings::default();
    world
        .update_character(
            id,
            delta_time,
            Vec3::new(0.0, -9.81, 0.0),
            &settings,
            &QueryFilter::new(),
        )
        .unwrap();
}

/// One round: a floor of four boxes with their own materials, a character standing on it, the
/// floor replaced by a plain box with every handle dropped, then updates.
fn round(world: &mut PhysicsWorld, capsule: &Shape) {
    let materials: Vec<_> = (0..4).map(|i| PhysicsMaterial::new(i).unwrap()).collect();
    let tiles: Vec<_> = materials
        .iter()
        .map(|material| {
            Shape::new_box_with_material(Vec3::new(1.0, 0.5, 1.0), 0.05, material).unwrap()
        })
        .collect();
    let children: Vec<_> = tiles
        .iter()
        .enumerate()
        .map(|(i, tile)| {
            let (x, z) = ((i % 2) as f32 * 2.0 - 1.0, (i / 2) as f32 * 2.0 - 1.0);
            child(tile, Vec3::new(x, 0.0, z), i as u32)
        })
        .collect();
    let floor_shape = Shape::new_compound(&children).unwrap();
    drop(children);
    let floor = world
        .create_body(
            &floor_shape,
            &BodySettings::new_static().position(RVec3::new(0.0, -0.5, 0.0)),
        )
        .unwrap();
    let settings = CharacterSettings::new(capsule).shape_offset(Vec3::new(0.0, 0.8, 0.0));
    let character = world
        .create_character(&settings, RVec3::new(0.0, 0.0, 0.0), Quat::IDENTITY)
        .unwrap();
    world
        .refresh_character_contacts(character, &QueryFilter::new())
        .unwrap();
    update(world, character, DT);

    let plain = Shape::new_box(Vec3::new(2.0, 0.5, 2.0)).unwrap();
    world
        .body_mut(floor)
        .unwrap()
        .set_shape(&plain, None, Activation::DontActivate)
        .unwrap();
    drop((floor_shape, tiles, materials, plain));
    update(world, character, TINY_DT);
    update(world, character, DT);
    update(world, character, DT);
    world.remove_character(character).unwrap();
    world.remove_body(floor).unwrap();
}

/// Creates a material, adds a reference with `JPH_PhysicsMaterial_AddRef` and releases only
/// the first one, so the material is never freed.
fn forget_a_material_reference() {
    // SAFETY: Jolt is initialised (the world exists) and the name is a NUL-terminated literal.
    // The material is live while the test holds its two references; one is released and the
    // other forgotten on purpose.
    unsafe {
        let material = JPH_PhysicsMaterial_Create2(c"leak".as_ptr(), 0, 0);
        assert!(!material.is_null());
        JPH_PhysicsMaterial_AddRef(material);
        JPH_PhysicsMaterial_Destroy(material);
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
fn character_contact_materials_do_not_leak() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    let capsule = Shape::new_capsule(0.5, 0.3).unwrap();
    for _ in 0..WARM_UP_ROUNDS {
        round(&mut world, &capsule);
    }
    let blocks = block_growth(|| round(&mut world, &capsule));
    eprintln!("character materials: growth per block of {BLOCK_ROUNDS} rounds: {blocks:?}");
    let (median_growth, total): (usize, usize) = (median(&blocks), blocks.iter().sum());
    assert!(
        median_growth < MAX_ROUND_GROWTH * BLOCK_ROUNDS,
        "median block growth {median_growth} bytes: a material leaks ({blocks:?})"
    );
    assert!(
        total < MAX_TOTAL_GROWTH,
        "total growth {total} bytes: a material leaks ({blocks:?})"
    );

    // The control: the same rounds, each forgetting a reference to four new materials, must
    // show.
    let blocks = block_growth(|| {
        round(&mut world, &capsule);
        for _ in 0..4 {
            forget_a_material_reference();
        }
    });
    eprintln!("control: growth per block of {BLOCK_ROUNDS} rounds: {blocks:?}");
    assert!(
        median(&blocks) >= MAX_ROUND_GROWTH * BLOCK_ROUNDS,
        "the control forgot four materials per round but the median block grew only by {} \
         ({blocks:?})",
        median(&blocks)
    );
}
