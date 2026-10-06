//! The Rapier adapter builds every scene as specified and its joints hold.

#![cfg(feature = "rapier")]

mod common;

use comparison::engine::{Config, Engine, Profile};
use comparison::engines::rapier::{handles_follow_scene_order, Rapier};
use comparison::scene::Scene;

/// The variant whose build this test binary has.
fn variant() -> &'static str {
    if cfg!(feature = "parallel") {
        "rapier-par"
    } else {
        "rapier-serial"
    }
}

#[test]
fn small_scenes_are_built_as_specified() {
    for scene in [Scene::Balls, Scene::Boxes, Scene::JointRevolute]
        .into_iter()
        .chain(Scene::FIXTURES)
    {
        common::check_build::<Rapier>(scene);
        common::check_shape_extents::<Rapier>(scene);
    }
}

#[test]
#[ignore = "large scenes; run on a release build"]
fn every_scene_is_built_as_specified() {
    for scene in Scene::RAPIER {
        common::check_build::<Rapier>(scene);
        common::check_shape_extents::<Rapier>(scene);
    }
}

#[test]
fn bodies_are_created_in_scene_order() {
    for scene in [Scene::Boxes, Scene::FixtureRevolute] {
        let rapier = Rapier::build(&scene.build(), &Config::new(Profile::Matched, 1)).unwrap();
        assert!(handles_follow_scene_order(&rapier), "{}", scene.name());
    }
}

#[test]
fn joint_fixtures_hold_and_fail_without_joints() {
    if cfg!(feature = "simd8") {
        return;
    }
    common::check_fixtures(variant());
}
