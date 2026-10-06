//! `oxijolt::prelude` glob-imported next to an engine prelude shaped like Bevy's (a glob of
//! sub-crate prelude globs that exports `Vec3`, `Quat`, `Real` and `Result`): the bare names
//! must stay the engine's, without an ambiguity warning.

#![deny(warnings)]

mod engine {
    pub mod math {
        pub mod prelude {
            #[derive(Clone, Copy)]
            pub struct Vec3(pub [f32; 3]);
            #[derive(Clone, Copy, Default)]
            pub struct Quat(pub [f32; 4]);
        }
    }
    pub mod time {
        pub mod prelude {
            #[derive(Default)]
            pub struct Real;
        }
    }
    pub mod ecs {
        pub mod prelude {
            pub type Result<T = (), E = Box<dyn std::error::Error + Send + Sync>> =
                std::result::Result<T, E>;
        }
    }
    pub mod prelude {
        pub use super::{ecs::prelude::*, math::prelude::*, time::prelude::*};
    }
}

use engine::prelude::*;
use oxijolt::prelude::*;

#[test]
fn the_prelude_leaves_math_and_result_to_the_engine_prelude() -> Result {
    // Bare names in type position, which an oxijolt `Vec3`, `Quat` or `Real` would make ambiguous.
    let up: Vec3 = engine::math::prelude::Vec3([0.0, 1.0, 0.0]);
    let turn: Quat = engine::math::prelude::Quat::default();
    let _clock: Real = engine::time::prelude::Real;
    assert_eq!((up.0[1], turn.0[3]), (1.0, 0.0));
    let mut world = PhysicsWorld::new(WorldSettings::default())?;
    let shape = Shape::new_sphere(0.5)?;
    world.create_body(&shape, &BodySettings::new_dynamic())?;
    assert!(world.step(1.0 / 60.0)?.is_complete());
    let poses: Vec<BodyPose> = world.active_body_poses();
    assert_eq!(poses.len(), 1);
    // Every area error is in the prelude.
    let _: Option<CollisionGroupError> = None;
    Ok(())
}

#[test]
fn the_math_types_come_from_the_prelude_math_module() -> oxijolt::error::Result<()> {
    use oxijolt::prelude::math::{RVec3, Vec3};

    let mut world = PhysicsWorld::new(WorldSettings::default().gravity(Vec3::ZERO))?;
    let shape = Shape::new_box(Vec3::new(0.5, 0.5, 0.5))?;
    let cube = world.create_body(
        &shape,
        &BodySettings::new_dynamic().position(RVec3::new(1.0, 2.0, 3.0)),
    )?;
    assert_eq!(world.body(cube)?.position(), RVec3::new(1.0, 2.0, 3.0));
    Ok(())
}
