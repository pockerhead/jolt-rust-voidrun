//! The ported scenes against Rapier's own code: each scene's world-building code from Rapier's
//! `examples3d/stress_tests` at v0.36.0 (in `rapier_oracle_scenes/`, with Rapier's Apache-2.0
//! notice) and the Rapier adapter's world built from the ported data must simulate bit for bit
//! alike. Rapier's code leaves sleeping and CCD at their defaults, so the adapter runs the
//! defaults profile here.

#![cfg(feature = "rapier")]

use comparison::engine::{BodyState, Config, Engine, Profile};
use comparison::engines::rapier::Rapier;
use comparison::quality::digest;
use comparison::scene::Scene;
use rapier3d::prelude::PhysicsWorld;

#[rustfmt::skip]
#[path = "rapier_oracle_scenes/balls3.rs"]
mod balls3;
#[rustfmt::skip]
#[path = "rapier_oracle_scenes/boxes3.rs"]
mod boxes3;
#[rustfmt::skip]
#[path = "rapier_oracle_scenes/capsules3.rs"]
mod capsules3;
#[rustfmt::skip]
#[path = "rapier_oracle_scenes/joint_ball3.rs"]
mod joint_ball3;
#[rustfmt::skip]
#[path = "rapier_oracle_scenes/joint_fixed3.rs"]
mod joint_fixed3;
#[rustfmt::skip]
#[path = "rapier_oracle_scenes/joint_prismatic3.rs"]
mod joint_prismatic3;
#[rustfmt::skip]
#[path = "rapier_oracle_scenes/joint_revolute3.rs"]
mod joint_revolute3;
#[rustfmt::skip]
#[path = "rapier_oracle_scenes/keva3.rs"]
mod keva3;
#[rustfmt::skip]
#[path = "rapier_oracle_scenes/many_pyramids3.rs"]
mod many_pyramids3;
#[rustfmt::skip]
#[path = "rapier_oracle_scenes/pyramid3.rs"]
mod pyramid3;

/// Every body's state in handle order, which is creation order.
fn states(world: &PhysicsWorld) -> Vec<BodyState> {
    world
        .bodies
        .iter()
        .map(|(_, body)| BodyState {
            position: body.translation().to_array(),
            rotation: body.rotation().to_array(),
            linear_velocity: body.linvel().to_array(),
            angular_velocity: body.angvel().to_array(),
            sleeping: body.is_sleeping(),
        })
        .collect()
}

/// Steps Rapier's own world and the adapter's for `ticks`, comparing digests every tick.
fn assert_same_simulation(scene: Scene, mut original: PhysicsWorld, ticks: usize) {
    let spec = scene.build();
    let mut ported = Rapier::build(&spec, &Config::new(Profile::Defaults, 1)).unwrap();
    assert_eq!(original.bodies.len(), spec.bodies.len(), "{}", scene.name());
    assert_eq!(
        original.colliders.len(),
        spec.bodies.len(),
        "{}",
        scene.name()
    );
    assert_eq!(
        original.impulse_joints.len(),
        spec.joints.len(),
        "{}",
        scene.name()
    );
    let mut ported_states = Vec::new();
    for tick in 0..=ticks {
        if tick > 0 {
            original.step();
            ported.step().unwrap();
        }
        ported.read_state(&mut ported_states);
        assert_eq!(
            digest(&states(&original)),
            digest(&ported_states),
            "{} differs at tick {tick}",
            scene.name()
        );
    }
}

#[test]
fn small_ported_scenes_simulate_like_rapiers_own() {
    assert_same_simulation(Scene::Boxes, boxes3::build(), 30);
    assert_same_simulation(Scene::JointRevolute, joint_revolute3::build(), 15);
    assert_same_simulation(Scene::JointPrismatic, joint_prismatic3::build(), 10);
}

#[test]
#[ignore = "large scenes; run on a release build"]
fn every_ported_scene_simulates_like_rapiers_own() {
    let originals: [(Scene, fn() -> PhysicsWorld); 10] = [
        (Scene::Balls, balls3::build),
        (Scene::Boxes, boxes3::build),
        (Scene::Capsules, capsules3::build),
        (Scene::Pyramid, pyramid3::build),
        (Scene::ManyPyramids, many_pyramids3::build),
        (Scene::Keva, keva3::build),
        (Scene::JointBall, joint_ball3::build),
        (Scene::JointFixed, joint_fixed3::build),
        (Scene::JointPrismatic, joint_prismatic3::build),
        (Scene::JointRevolute, joint_revolute3::build),
    ];
    for (scene, build) in originals {
        assert_same_simulation(scene, build(), 60);
    }
}
