//! A leak gate for the hull, mesh, scaled and tapered shapes: every constructor, the refusals,
//! a failed Jolt creation and bodies of the new shapes, round after round in one world.
//!
//! It measures the private bytes of the process (Windows `K32GetProcessMemoryInfo`), because
//! the shapes are allocated by C++, which a Rust global allocator does not see. The heap grows in
//! steps for about the first 2500 rounds and now and then later (in up to three of seven blocks),
//! so the gate measures after 3000 warm-up rounds: nine consecutive blocks of 500 rounds. It fails
//! when the median block grows by 100 bytes per round or more, or when all blocks together grow
//! by 4 MiB or more: a steady leak of 100 bytes per round shows in the median, a leak that lands
//! in a few blocks only (or a heap step) in the total. Smaller leaks pass. A control run then
//! forgets one four-point hull per round, measured at about 600 bytes, and must fail the median
//! gate, which shows the gate sees a leak of that size. The file holds exactly one test, so its
//! binary runs alone and no parallel test disturbs the counter.
#![cfg(windows)]

mod common;

use std::ptr::null_mut;

use common::memory::private_bytes;
use common::meshes::*;
use common::*;
use oxijolt::*;
use oxijolt_sys::*;

const WARM_UP_ROUNDS: usize = 3000;
const BLOCK_ROUNDS: usize = 500;
const BLOCKS: usize = 9;
const MAX_ROUND_GROWTH: usize = 100;
const MAX_TOTAL_GROWTH: usize = 4 * 1024 * 1024;

/// Inputs built once and reused by every round.
struct Inputs {
    cloud: Vec<Vec3>,
    vertices: Vec<Vec3>,
    triangles: Vec<[u32; 3]>,
    material_indices: Vec<u8>,
}

impl Inputs {
    fn new() -> Self {
        let cloud = (0..64)
            .map(|i| {
                let (y, angle) = (1.0 - 2.0 * (i as f32 + 0.5) / 64.0, i as f32 * 2.4);
                let r = (1.0 - y * y).sqrt();
                Vec3::new(0.5 * r * angle.cos(), 0.5 * y, 0.5 * r * angle.sin())
            })
            .collect();
        let (vertices, triangles) = grid(16, 0.5, |x, z| 0.1 * (x * z).sin());
        let material_indices = (0..triangles.len()).map(|i| (i % 2) as u8).collect();
        Self {
            cloud,
            vertices,
            triangles,
            material_indices,
        }
    }
}

/// One round: every new constructor, the refusals, a raw Jolt failure, and a body of a hull and
/// one of a mesh created, stepped once and removed.
fn round(world: &mut PhysicsWorld, inputs: &Inputs) {
    let (a, b) = (
        PhysicsMaterial::new(1).unwrap(),
        PhysicsMaterial::new(2).unwrap(),
    );
    let hull = Shape::new_convex_hull(&inputs.cloud, 0.05).unwrap();
    let hull_with_material = Shape::new_convex_hull_with_material(&inputs.cloud, 0.05, &a).unwrap();
    let list = [&a, &b];
    let (mesh, _) = Shape::new_mesh_with_settings(
        &inputs.vertices,
        &inputs.triangles,
        &MeshSettings::default().materials(&list, &inputs.material_indices),
    )
    .unwrap();
    let scaled_hull = Shape::scaled(&hull, Vec3::new(1.0, 2.0, 0.5)).unwrap();
    let scaled_mesh = Shape::scaled(&mesh, Vec3::new(2.0, 2.0, 2.0)).unwrap();
    let block = Shape::new_box(Vec3::new(0.2, 0.2, 0.2)).unwrap();
    let compound = Shape::new_compound(&[
        CompoundChild {
            shape: &scaled_hull,
            position: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            user_data: 0,
        },
        CompoundChild {
            shape: &block,
            position: Vec3::new(1.0, 0.0, 0.0),
            rotation: Quat::IDENTITY,
            user_data: 1,
        },
    ])
    .unwrap();
    let tapered_capsule = Shape::new_tapered_capsule(0.4, 0.1, 0.2).unwrap();
    let tapered_cylinder = Shape::new_tapered_cylinder(0.4, 0.0, 0.2, 0.0).unwrap();

    let flat: Vec<Vec3> = inputs
        .cloud
        .iter()
        .map(|p| Vec3::new(p.x, 0.0, p.z))
        .collect();
    assert!(Shape::new_convex_hull(&flat, 0.05).is_err());
    let degenerate = [[0, 0, 1], [2, 3, 3]];
    assert!(Shape::new_mesh(&inputs.vertices, &degenerate).is_err());
    assert!(Shape::scaled(&mesh, Vec3::new(1.0e-3, 1.0, 1.0e-3)).is_err());
    raw_refused_hull();

    let hull_body = world
        .create_body(
            &hull_with_material,
            &BodySettings::new_dynamic().position(RVec3::new(0.0, 2.0, 0.0)),
        )
        .unwrap();
    let mesh_body = world
        .create_body(
            &scaled_mesh,
            &BodySettings::new_kinematic()
                .mass(10.0)
                .position(RVec3::new(30.0, 1.0, 0.0)),
        )
        .unwrap();
    step(world, 1);
    world.remove_body(hull_body).unwrap();
    world.remove_body(mesh_body).unwrap();
    drop((compound, tapered_capsule, tapered_cylinder));
}

/// A hull Jolt refuses, created and released through the raw bindings.
fn raw_refused_hull() {
    let points = [
        JPH_Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        JPH_Vec3 {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        },
        JPH_Vec3 {
            x: 2.0,
            y: 0.0,
            z: 0.0,
        },
    ];
    // SAFETY: Jolt is initialised (the world exists); `points` holds the three points passed and
    // Jolt copies them. Jolt refuses collinear points, so no shape is returned; the settings'
    // one reference is released once.
    unsafe {
        let settings = JPH_ConvexHullShapeSettings_Create(points.as_ptr(), 3, 0.05);
        assert!(!settings.is_null());
        let shape = JPH_ShapeSettings_CreateShapeWithError(settings.cast(), null_mut(), 0);
        assert!(shape.is_null());
        JPH_ShapeSettings_Destroy(settings.cast());
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
fn new_shapes_do_not_leak() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    add_floor(&mut world);
    let inputs = Inputs::new();
    for _ in 0..WARM_UP_ROUNDS {
        round(&mut world, &inputs);
    }
    let blocks = block_growth(|| round(&mut world, &inputs));
    eprintln!("new shapes: growth per block of {BLOCK_ROUNDS} rounds: {blocks:?}");
    let (median_growth, total): (usize, usize) = (median(&blocks), blocks.iter().sum());
    assert!(
        median_growth < MAX_ROUND_GROWTH * BLOCK_ROUNDS,
        "median block growth {median_growth} bytes: a new shape leaks ({blocks:?})"
    );
    assert!(
        total < MAX_TOTAL_GROWTH,
        "total growth {total} bytes: a new shape leaks ({blocks:?})"
    );

    // The control: the same rounds, each forgetting one four-point hull, must show.
    let tetrahedron = [
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    ];
    let blocks = block_growth(|| {
        round(&mut world, &inputs);
        std::mem::forget(Shape::new_convex_hull(&tetrahedron, 0.05).unwrap());
    });
    eprintln!("control: growth per block of {BLOCK_ROUNDS} rounds: {blocks:?}");
    assert!(
        median(&blocks) >= MAX_ROUND_GROWTH * BLOCK_ROUNDS,
        "the control forgot a hull per round but the median block grew only by {} ({blocks:?})",
        median(&blocks)
    );
}
