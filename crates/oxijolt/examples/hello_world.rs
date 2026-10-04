//! Drops a sphere onto a static floor and prints where it is, without a window.
//!
//! Run with `cargo run -p oxijolt --example hello_world`.

use oxijolt::prelude::math::*;
use oxijolt::prelude::*;

fn main() -> oxijolt::Result<()> {
    let mut world = PhysicsWorld::new(WorldSettings::default())?;

    // A static floor whose top face is at y = 0.
    let floor_shape = Shape::new_box(Vec3::new(100.0, 1.0, 100.0))?;
    let floor = world.create_body(
        &floor_shape,
        &BodySettings::new_static().position(RVec3::new(0.0, -1.0, 0.0)),
    )?;

    // A dynamic sphere 2 m above the floor. The body keeps its own reference to the shape.
    let sphere = world.create_body(
        &Shape::new_sphere(0.5)?,
        &BodySettings::new_dynamic().position(RVec3::new(0.0, 2.0, 0.0)),
    )?;

    for tick in 1..=120 {
        // An `Err` means the step was rejected; a full Jolt buffer is reported in the `Ok` value.
        let report = world.step(1.0 / 60.0)?;
        if !report.is_complete() {
            eprintln!("tick {tick:3}: Jolt dropped contacts: {report:?}");
        }
        if tick % 20 == 0 {
            let body = world.body(sphere)?;
            let position = body.position();
            println!(
                "tick {tick:3}: position ({:.3}, {:.3}, {:.3}), active: {}",
                position.x,
                position.y,
                position.z,
                body.is_active()
            );
        }
    }

    world.remove_body(sphere)?;
    world.remove_body(floor)?;
    Ok(())
}
