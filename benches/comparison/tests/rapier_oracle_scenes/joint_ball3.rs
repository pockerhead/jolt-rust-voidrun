// Rapier's `examples3d/stress_tests/joint_ball3.rs` at v0.36.0
// (https://github.com/dimforge/rapier/blob/v0.36.0/examples3d/stress_tests/joint_ball3.rs).
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
    let num = 100;
    let shift = 1.0;

    let mut body_handles = Vec::new();

    for k in 0..num {
        for i in 0..num {
            let fk = k as f32;
            let fi = i as f32;

            let status = if i == 0 && (k % 4 == 0 || k == num - 1) {
                RigidBodyType::Fixed
            } else {
                RigidBodyType::Dynamic
            };

            let rigid_body =
                RigidBodyBuilder::new(status).translation(Vec3::new(fk * shift, 0.0, fi * shift));
            let collider = ColliderBuilder::ball(rad);
            let (child_handle, _) = world.insert(rigid_body, collider);

            // Vertical joint.
            if i > 0 {
                let parent_handle = *body_handles.last().unwrap();
                let joint = SphericalJointBuilder::new().local_anchor2(Vec3::new(0.0, 0.0, -shift));
                world.insert_impulse_joint(parent_handle, child_handle, joint);
            }

            // Horizontal joint.
            if k > 0 {
                let parent_index = body_handles.len() - num;
                let parent_handle = body_handles[parent_index];
                let joint = SphericalJointBuilder::new().local_anchor2(Vec3::new(-shift, 0.0, 0.0));
                world.insert_impulse_joint(parent_handle, child_handle, joint);
            }

            body_handles.push(child_handle);
        }
    }

    world
}
