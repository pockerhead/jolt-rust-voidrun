// Rapier's `examples3d/stress_tests/capsules3.rs` at v0.36.0
// (https://github.com/dimforge/rapier/blob/v0.36.0/examples3d/stress_tests/capsules3.rs).
// Copyright Dimforge and contributors, Apache-2.0. Changed: the testbed lines are removed
// and `run` returns the world it built as `build`; the world-building code is verbatim.
#![allow(clippy::all, unused_mut, unused_variables)]

use rapier3d::prelude::*;

pub fn build() -> PhysicsWorld {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    /*
     * Ground
     */
    let ground_size = 200.1;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::fixed().translation(Vec3::new(0.0, -ground_height, 0.0));
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size);
    world.insert(rigid_body, collider);

    /*
     * Create the cubes
     */
    let num = 8;
    let rad = 1.0;

    let shift = rad * 2.0 + rad;
    let shifty = rad * 4.0;
    let centerx = shift * (num / 2) as f32;
    let centery = shift / 2.0;
    let centerz = shift * (num / 2) as f32;

    let mut offset = -(num as f32) * (rad * 2.0 + rad) * 0.5;

    for j in 0usize..47 {
        for i in 0..num {
            for k in 0usize..num {
                let x = i as f32 * shift - centerx + offset;
                let y = j as f32 * shifty + centery + 3.0;
                let z = k as f32 * shift - centerz + offset;

                // Build the rigid body.
                let rigid_body = RigidBodyBuilder::dynamic().translation(Vec3::new(x, y, z));
                let collider = ColliderBuilder::capsule_y(rad, rad);
                world.insert(rigid_body, collider);
            }
        }

        offset -= 0.05 * rad * (num as f32 - 1.0);
    }

    world
}
