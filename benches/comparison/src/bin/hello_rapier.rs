//! The smallest Rapier program: a ball dropped on a ground for one second. Built alone, it
//! measures the build time and binary size the engine brings.

use rapier3d::prelude::*;

fn main() {
    let mut world = PhysicsWorld::new();
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5, 0.0)),
        ColliderBuilder::cuboid(50.0, 0.5, 50.0),
    );
    let (ball, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 5.0, 0.0)),
        ColliderBuilder::ball(0.5),
    );
    for _ in 0..60 {
        world.step();
    }
    println!(
        "height after 60 ticks: {}",
        world.bodies[ball].translation().y
    );
}
