//! `use oxijolt::*` next to another glob import that exports `Result` and `Error`: the crate root
//! names neither, so the bare names stay the other crate's. A root `Result` or `Error` would make
//! them ambiguous (E0659) and this file would not compile.

#![deny(warnings)]

mod other {
    #[derive(Debug, PartialEq)]
    pub struct Error(pub &'static str);

    pub type Result<T> = std::result::Result<T, Error>;
}

use other::*;
use oxijolt::*;

fn other_call(fail: bool) -> Result<u32> {
    if fail {
        // `Error` in type position: a value-position `Error(..)` would not see a root enum.
        let error: Error = Error("refused");
        Err(error)
    } else {
        Ok(1)
    }
}

#[test]
fn the_crate_root_leaves_result_and_error_to_other_globs() {
    assert_eq!(other_call(true), Err(Error("refused")));
    assert_eq!(other_call(false), Ok(1));
    let world = PhysicsWorld::new(WorldSettings::default()).unwrap();
    assert!(world.active_body_poses().is_empty());
}

#[test]
fn the_crate_wide_result_is_named_by_the_error_module() -> oxijolt::error::Result<()> {
    let shape = Shape::new_sphere(0.5)?;
    let mut world = PhysicsWorld::new(WorldSettings::default())?;
    world.create_body(&shape, &BodySettings::new_dynamic())?;
    let error: oxijolt::error::Error = world.step(0.0).unwrap_err().into();
    assert_eq!(
        error,
        oxijolt::error::Error::Step(StepError::InvalidDeltaTime)
    );
    Ok(())
}
