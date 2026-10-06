//! The Jolt adapter builds every scene as specified, its joints hold, and a full contact buffer
//! fails the run.

mod common;

use comparison::engine::{Config, Engine, Profile};
use comparison::engines::jolt::{ids_follow_scene_order, temp_allocator_size, Buffers, Jolt};
use comparison::scene::{BodySpec, Scene, SceneSpec, Shape};

/// The scenes small enough for a debug test run; the others are checked with `--ignored` on a
/// release build.
const SMALL: [Scene; 2] = [Scene::Boxes, Scene::JointRevolute];

#[test]
fn small_scenes_are_built_as_specified() {
    for scene in SMALL.into_iter().chain(Scene::FIXTURES) {
        common::check_build::<Jolt>(scene);
        common::check_shape_extents::<Jolt>(scene);
    }
}

#[test]
#[ignore = "large scenes; run on a release build"]
fn every_scene_is_built_as_specified() {
    for scene in Scene::RAPIER {
        common::check_build::<Jolt>(scene);
        common::check_shape_extents::<Jolt>(scene);
    }
}

#[test]
fn bodies_are_created_in_scene_order() {
    for scene in [Scene::Boxes, Scene::FixtureRevolute] {
        let jolt = Jolt::build(&scene.build(), &Config::new(Profile::Matched, 1)).unwrap();
        assert!(ids_follow_scene_order(&jolt), "{}", scene.name());
    }
}

#[test]
fn joint_fixtures_hold_and_fail_without_joints() {
    common::check_fixtures("jolt");
}

/// Twenty cubes in a row 2 m above a ground, apart from each other: no contact until they land.
fn falling_row() -> SceneSpec {
    let mut bodies = vec![BodySpec::fixed(
        Shape::Cuboid {
            half_extents: [50.0, 0.1, 50.0],
        },
        [0.0, -0.1, 0.0],
    )];
    for i in 0..20u8 {
        let cube = Shape::Cuboid {
            half_extents: [0.5; 3],
        };
        bodies.push(BodySpec::dynamic(cube, [f32::from(i) * 2.0, 2.5, 0.0]));
    }
    SceneSpec {
        name: "falling_row",
        source: "test",
        bodies,
        joints: Vec::new(),
    }
}

#[test]
fn a_full_contact_buffer_fails_the_tick_of_the_first_contacts() {
    let spec = falling_row();
    let config = Config::new(Profile::Matched, 1);
    let buffers = Buffers {
        contacts: 4,
        temp_allocator: temp_allocator_size(4),
    };
    let mut small = Jolt::build_with(&spec, &config, buffers).unwrap();
    let failed_at = (1..=120).find(|_| small.step().is_err());
    // A 2 m drop takes about 0.64 s: the first contacts form around tick 35.
    let failed_at = failed_at.expect("the small buffer never overflowed");
    assert!(
        failed_at > 20,
        "failed at tick {failed_at}, before any contact"
    );

    let mut sized = Jolt::build(&spec, &config).unwrap();
    for tick in 1..=120 {
        sized.step().unwrap_or_else(|e| panic!("tick {tick}: {e}"));
    }
}
