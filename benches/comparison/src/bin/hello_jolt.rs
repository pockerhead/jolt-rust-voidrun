//! The smallest oxijolt program: a ball dropped on a ground for one second. Built alone, it
//! measures the build time and binary size the engine brings.

use oxijolt::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new(WorldSettings::default())?;
    let ground = Shape::new_box(Vec3::new(50.0, 0.5, 50.0))?;
    world.create_body(
        &ground,
        &BodySettings::new_static().position(RVec3::new(0.0, -0.5, 0.0)),
    )?;
    let ball = Shape::new_sphere(0.5)?;
    let id = world.create_body(
        &ball,
        &BodySettings::new_dynamic().position(RVec3::new(0.0, 5.0, 0.0)),
    )?;
    for _ in 0..60 {
        assert!(world.step(1.0 / 60.0)?.is_complete());
    }
    println!("height after 60 ticks: {}", world.body(id)?.position().y);
    Ok(())
}
