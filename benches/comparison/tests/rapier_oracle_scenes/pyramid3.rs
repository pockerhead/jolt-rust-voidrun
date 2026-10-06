// Rapier's `examples3d/stress_tests/pyramid3.rs` at v0.36.0
// (https://github.com/dimforge/rapier/blob/v0.36.0/examples3d/stress_tests/pyramid3.rs).
// Copyright Dimforge and contributors, Apache-2.0. Changed: the testbed lines are removed
// and `run` returns the world it built as `build`; the world-building code is verbatim.
#![allow(clippy::all, unused_mut, unused_variables)]

//! A wide brick-laid box pyramid: slightly shrunken 1.95-cubes on
//! a 2.25 pitch with a 1.0 brick offset — which is *not* half of 2.25 — so the
//! boxes rest on four **unequal** corner patches (0.95 and 0.70 wide). The
//! asymmetry is deliberate: a perfectly symmetric brick is a degenerate,
//! marginally-stable configuration.
//!
//! 50 layers with (50−i)² boxes per layer (~43k bodies), each box on
//! four corner supports, dropped onto a large ground box.

use rapier3d::prelude::*;

pub fn build() -> PhysicsWorld {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -9.81, 0.0);

    /*
     * Ground: a 100×1×100 half-extents box at y = -1, so its top face is y = 0.
     */
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -1.0, 0.0)),
        ColliderBuilder::cuboid(100.0, 1.0, 100.0),
    );

    /*
     * The pyramid.
     */
    let pyramid_height = 50i32;
    let box_size = 2.0;
    let box_separation = 0.5;
    let half_box_size = 0.5 * box_size;
    // Shrunken cube: the boxes never *quite* fill their lattice cell.
    let h = half_box_size - 0.025;

    for i in 0..pyramid_height {
        // Odd layers are brick-offset by a half box (1.0) — note this is NOT
        // half of the 2.25 lateral pitch, which is what makes the four corner
        // supports unequal.
        let brick = if i & 1 != 0 { half_box_size } else { 0.0 };
        let y = 1.0 + (box_size + box_separation) * i as f32;

        for j in i / 2..pyramid_height - (i + 1) / 2 {
            for k in i / 2..pyramid_height - (i + 1) / 2 {
                let x = -(pyramid_height as f32) + (box_size + 0.25) * j as f32 + brick;
                let z = -(pyramid_height as f32) + (box_size + 0.25) * k as f32 + brick;

                world.insert(
                    RigidBodyBuilder::dynamic().translation(Vector::new(x, y, z)),
                    // Water density (1000). For a uniform-density pile this
                    // changes nothing dynamically (the soft-contact coefficients
                    // are mass-normalized).
                    ColliderBuilder::cuboid(h, h, h).density(1000.0),
                );
            }
        }
    }

    world
}
