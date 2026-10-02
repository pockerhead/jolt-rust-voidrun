//! Helpers shared by the integration tests.

// Each test file compiles this module on its own and uses a different subset.
#![allow(dead_code)]

use joltphysics::*;

pub const DT: f32 = 1.0 / 60.0;

/// A world with the default layers.
pub fn world(gravity: Vec3, worker_threads: u32) -> PhysicsWorld {
    PhysicsWorld::new(
        WorldSettings::default()
            .gravity(gravity)
            .worker_threads(worker_threads),
    )
    .unwrap()
}

/// A static floor whose top face is at y = 0.
pub fn add_floor(world: &mut PhysicsWorld) -> BodyId {
    let shape = Shape::new_box(Vec3::new(100.0, 1.0, 100.0)).unwrap();
    world
        .create_body(
            &shape,
            &BodySettings::new_static().position(RVec3::new(0.0, -1.0, 0.0)),
        )
        .unwrap()
}

/// A dynamic unit cube (half extent 0.5) at `position`.
pub fn add_cube(world: &mut PhysicsWorld, position: RVec3) -> BodyId {
    let shape = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
    world
        .create_body(&shape, &BodySettings::new_dynamic().position(position))
        .unwrap()
}

pub fn step(world: &mut PhysicsWorld, ticks: usize) {
    for _ in 0..ticks {
        world.step(DT).unwrap();
    }
}

/// Positions of eight cubes in two side-by-side stacks of four, dropped from a small gap. Each
/// layer is shifted 0.3 m along +x, which puts the centre of mass of the upper layers past the
/// edge of the bottom cube, so both stacks topple, the left one into the right one.
pub fn stacks_scene() -> Vec<RVec3> {
    let mut cubes = Vec::new();
    for column in 0..2 {
        for layer in 0..4 {
            let x = column as Real + 0.3 * layer as Real;
            let y = 0.5 + 1.05 * layer as Real;
            cubes.push(RVec3::new(x, y, 0.0));
        }
    }
    cubes
}

/// Creates the floor and the cubes of [`stacks_scene`], in that order, and returns their ids.
pub fn build_stacks(world: &mut PhysicsWorld) -> Vec<BodyId> {
    let mut ids = vec![add_floor(world)];
    for position in stacks_scene() {
        ids.push(add_cube(world, position));
    }
    ids
}

/// Appends the state of `id` to `digest`: raw id, position, rotation, linear and angular
/// velocity as little-endian bits, and the sleeping flag.
pub fn record_body(world: &PhysicsWorld, id: BodyId, digest: &mut Vec<u8>) {
    let body = world.body(id).unwrap();
    digest.extend_from_slice(&id.to_raw().to_le_bytes());
    let position: [Real; 3] = body.position().into();
    for value in position {
        digest.extend_from_slice(&value.to_bits().to_le_bytes());
    }
    let rotation: [f32; 4] = body.rotation().into();
    let linear: [f32; 3] = body.linear_velocity().into();
    let angular: [f32; 3] = body.angular_velocity().into();
    for value in rotation.into_iter().chain(linear).chain(angular) {
        digest.extend_from_slice(&value.to_bits().to_le_bytes());
    }
    digest.push(u8::from(body.is_sleeping()));
}

/// Steps the world `ticks` times and records every body in `ids` after each tick.
pub fn run_digest(world: &mut PhysicsWorld, ids: &[BodyId], ticks: usize) -> Vec<u8> {
    let mut digest = Vec::new();
    for _ in 0..ticks {
        world.step(DT).unwrap();
        for &id in ids {
            record_body(world, id, &mut digest);
        }
    }
    digest
}

pub fn length(v: Vec3) -> f32 {
    (v.x * v.x + v.y * v.y + v.z * v.z).sqrt()
}

/// The rotation by `angle` radians about the unit vector `axis`.
pub fn quat_about(axis: Vec3, angle: f32) -> Quat {
    let (sin, cos) = (angle / 2.0).sin_cos();
    Quat::from_xyzw(axis.x * sin, axis.y * sin, axis.z * sin, cos)
}
