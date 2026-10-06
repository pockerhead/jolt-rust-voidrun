// Rapier's `examples3d/stress_tests/balls3.rs` at v0.36.0
// (https://github.com/dimforge/rapier/blob/v0.36.0/examples3d/stress_tests/balls3.rs).
// Copyright Dimforge and contributors, Apache-2.0. Changed: the testbed lines are removed
// and `run` returns the world it built as `build`; `glam::Vec3` comes from Rapier's prelude; the world-building code is verbatim.
#![allow(clippy::all, unused_mut, unused_variables)]

use rapier3d::prelude::*;

pub fn build() -> PhysicsWorld {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    /*
     * Create the balls
     */
    let num = 20;
    let rad = 1.0;

    let shift = rad * 2.0 + 1.0;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift / 2.0;
    let centerz = shift * (num as f32) / 2.0;

    for i in 0..num {
        for j in 0usize..num {
            for k in 0..num {
                let x = i as f32 * shift - centerx;
                let y = j as f32 * shift + centery;
                let z = k as f32 * shift - centerz;

                let status = if j == 0 {
                    RigidBodyType::Fixed
                } else {
                    RigidBodyType::Dynamic
                };
                let density = 0.477;

                // Build the rigid body.
                let rigid_body = RigidBodyBuilder::new(status).translation(Vec3::new(x, y, z));
                let collider = ColliderBuilder::ball(rad).density(density);
                world.insert(rigid_body, collider);
            }
        }
    }

    world
}
