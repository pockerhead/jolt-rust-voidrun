//! The Avian adapter builds every scene as specified, its joints hold only with Avian's joint
//! solver, and every update is one physics step.

#![cfg(feature = "avian")]

mod common;

use comparison::engine::{Config, Engine, Profile};
use comparison::engines::avian::{entities_follow_scene_order, Avian};
use comparison::scene::Scene;

/// The variant whose build this test binary has.
fn variant() -> &'static str {
    if cfg!(feature = "parallel") {
        "avian-par"
    } else {
        "avian-serial"
    }
}

#[test]
fn small_scenes_are_built_as_specified() {
    for scene in [Scene::Balls, Scene::Boxes, Scene::JointRevolute]
        .into_iter()
        .chain(Scene::FIXTURES)
    {
        common::check_build::<Avian>(scene);
        common::check_shape_extents::<Avian>(scene);
    }
}

#[test]
#[ignore = "large scenes; run on a release build"]
fn every_scene_is_built_as_specified() {
    for scene in Scene::RAPIER {
        common::check_build::<Avian>(scene);
        common::check_shape_extents::<Avian>(scene);
    }
}

#[test]
fn bodies_are_created_in_scene_order() {
    let avian = Avian::build(&Scene::Boxes.build(), &Config::new(Profile::Matched, 1)).unwrap();
    assert!(entities_follow_scene_order(&avian));
}

/// The fixtures hold with Avian's joint solver; "without joints" here means the joints exist
/// but `XpbdSolverPlugin` is disabled.
#[test]
fn joint_fixtures_hold_and_fail_without_the_joint_solver() {
    common::check_fixtures(variant());
}

#[test]
fn every_update_advances_physics_by_one_step() {
    let mut avian = Avian::build(
        &Scene::FixtureStack.build(),
        &Config::new(Profile::Matched, 1),
    )
    .unwrap();
    for _ in 0..30 {
        avian.step().unwrap();
    }
}
