// Rapier's `examples3d/stress_tests/joint_prismatic3.rs` at v0.36.0
// (https://github.com/dimforge/rapier/blob/v0.36.0/examples3d/stress_tests/joint_prismatic3.rs).
// Copyright Dimforge and contributors, Apache-2.0. Changed: the testbed lines are removed
// and `run` returns the world it built as `build`; the world-building code is verbatim.
#![allow(clippy::all, unused_mut, unused_variables)]

use rapier3d::prelude::*;

pub fn build() -> PhysicsWorld {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    let rad = 0.4;
    let num = 5;
    let shift = 1.0;

    for m in 0..8 {
        let z = m as f32 * shift * (num as f32 + 2.0);

        for l in 0..8 {
            let y = l as f32 * shift * (num as f32) * 2.0;

            for j in 0..50 {
                let x = j as f32 * shift * 4.0;

                let ground = RigidBodyBuilder::fixed().translation(Vec3::new(x, y, z));
                let collider = ColliderBuilder::cuboid(rad, rad, rad);
                let (mut curr_parent, _) = world.insert(ground, collider);

                for i in 0..num {
                    let z = z + (i + 1) as f32 * shift;
                    let density = 1.0;
                    let rigid_body = RigidBodyBuilder::dynamic().translation(Vec3::new(x, y, z));
                    let collider = ColliderBuilder::cuboid(rad, rad, rad).density(density);
                    let (curr_child, _) = world.insert(rigid_body, collider);

                    let axis = if i % 2 == 0 {
                        Vec3::new(1.0, 1.0, 0.0).normalize()
                    } else {
                        Vec3::new(-1.0, 1.0, 0.0).normalize()
                    };

                    let prism = PrismaticJointBuilder::new(axis)
                        .local_anchor2(Vec3::new(0.0, 0.0, -shift))
                        .limits([-2.0, 0.0]);
                    world.insert_impulse_joint(curr_parent, curr_child, prism);

                    curr_parent = curr_child;
                }
            }
        }
    }

    world
}
