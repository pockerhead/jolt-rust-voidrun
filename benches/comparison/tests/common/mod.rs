//! Checks shared by the engine tests: every engine must build the scene it is given, and its
//! joints must hold.

#![allow(dead_code)]

use std::path::PathBuf;

use comparison::engine::{BodyState, Config, Engine, Profile};
use comparison::engines::Variant;
use comparison::run::{validate_with, RunArgs, Validation};
use comparison::scene::{BodySpec, Motion, Scene, SceneSpec};

/// Positions, rotations and velocities right after building are the scene's, in its order;
/// fixed bodies have no mass and dynamic ones `density * volume`; principal inertia, where the
/// engine exposes it, is the shape's for that mass.
pub fn check_build<E: Engine>(scene: Scene) {
    let spec = scene.build();
    let mut engine = E::build(&spec, &Config::new(Profile::Matched, 1)).unwrap();
    let mut states = Vec::new();
    engine.read_state(&mut states);
    assert_eq!(states.len(), spec.bodies.len(), "{}", spec.name);
    for (index, (body, state)) in spec.bodies.iter().zip(&states).enumerate() {
        let at = || format!("{} body {index}", spec.name);
        assert_eq!(state.position, body.position, "{}", at());
        assert_eq!(state.rotation, [0.0, 0.0, 0.0, 1.0], "{}", at());
        assert_eq!(state.linear_velocity, [0.0; 3], "{}", at());
        assert_eq!(state.angular_velocity, [0.0; 3], "{}", at());
        let mass = engine.body_mass(index);
        match body.motion {
            Motion::Fixed => assert_eq!(mass, None, "{} is fixed", at()),
            Motion::Dynamic => {
                let mass = f64::from(mass.unwrap_or_else(|| panic!("{} has no mass", at())));
                assert_close(mass, body.mass(), 1e-4, &format!("{} mass", at()));
                if let Some(inertia) = engine.body_inertia(index) {
                    let expected = body.shape.principal_inertia(body.mass());
                    for axis in 0..3 {
                        let what = format!("{} inertia about axis {axis}", at());
                        assert_close(f64::from(inertia[axis]), expected[axis], 1e-3, &what);
                    }
                }
            }
        }
    }
}

fn assert_close(actual: f64, expected: f64, relative: f64, what: &str) {
    assert!(
        (actual - expected).abs() <= relative * expected.abs(),
        "{what}: {actual} vs {expected}"
    );
}

/// The bodies whose shapes the ray probe checks: the first, the last and every 97th.
pub fn probed_bodies(spec: &SceneSpec) -> Vec<usize> {
    let last = spec.bodies.len() - 1;
    let mut probed: Vec<usize> = (0..=last).step_by(97).collect();
    if probed.last() != Some(&last) {
        probed.push(last);
    }
    probed
}

/// Rebuilds the probed bodies of `scene` apart from each other (large ones alone at the
/// origin, the others on a 10 m grid) and casts six axis rays at each from 1 m outside its
/// surface: every ray must hit that body 1 m away, within 0.1 mm.
pub fn check_shape_extents<E: Engine>(scene: Scene) {
    let spec = scene.build();
    let (large, small): (Vec<usize>, Vec<usize>) = probed_bodies(&spec)
        .into_iter()
        .partition(|&i| spec.bodies[i].shape.half_size().iter().any(|&h| h > 4.0));
    for index in large {
        probe::<E>(&spec, &[index], |_| [0.0; 3]);
    }
    probe::<E>(&spec, &small, |slot| {
        let (row, column) = (slot / 21, slot % 21);
        [column as f32 * 10.0 - 100.0, 0.0, row as f32 * 10.0 - 100.0]
    });
}

/// Builds `indices` of `spec` alone at `place(slot)` and checks them with six rays each.
fn probe<E: Engine>(spec: &SceneSpec, indices: &[usize], place: impl Fn(usize) -> [f32; 3]) {
    let bodies: Vec<BodySpec> = indices
        .iter()
        .enumerate()
        .map(|(slot, &i)| BodySpec {
            position: place(slot),
            ..spec.bodies[i]
        })
        .collect();
    let probe_spec = SceneSpec {
        name: spec.name,
        source: spec.source,
        bodies,
        joints: Vec::new(),
    };
    let mut engine = E::build(&probe_spec, &Config::new(Profile::Matched, 1)).unwrap();
    for (slot, (body, &index)) in probe_spec.bodies.iter().zip(indices).enumerate() {
        let half = body.shape.half_size();
        for axis in 0..3 {
            for sign in [1.0f32, -1.0] {
                let mut origin = body.position;
                origin[axis] += sign * (half[axis] + 1.0);
                let mut direction = [0.0; 3];
                direction[axis] = -sign;
                let hit = engine.cast_ray(origin, direction, 2.0);
                let what = format!("{} body {index}, axis {axis}, side {sign}", spec.name);
                let (hit_slot, distance) = hit.unwrap_or_else(|| panic!("{what}: no hit"));
                assert_eq!(hit_slot, slot, "{what}: hit another body");
                assert!(
                    (distance - 1.0).abs() <= 1e-4,
                    "{what}: distance {distance}"
                );
            }
        }
    }
}

/// A validation run of `scene` with `variant` for `steps` ticks, joints on or off.
pub fn validate_fixture(variant: &str, scene: Scene, steps: usize, joints: bool) -> Validation {
    let args = RunArgs {
        run_id: "test".to_owned(),
        variant: Variant::from_name(variant).unwrap(),
        scene,
        profile: Profile::Matched,
        threads: 1,
        steps,
        iterations: None,
        out: PathBuf::new(),
    };
    let config = Config {
        joints,
        ..args.config()
    };
    validate_with(&args, config, &scene.build()).unwrap()
}

/// Every joint fixture holds within the bounds for 120 ticks, and breaks them with its joints
/// removed; the stack fixture stands.
pub fn check_fixtures(variant: &str) {
    for scene in Scene::FIXTURES {
        let held = validate_fixture(variant, scene, 120, true);
        assert!(
            held.quality.violations().is_empty(),
            "{variant} {}: {:?}",
            scene.name(),
            held.quality
        );
        if scene == Scene::FixtureStack {
            continue;
        }
        let loose = validate_fixture(variant, scene, 120, false);
        assert!(
            !loose.quality.violations().is_empty(),
            "{variant} {} without joints kept every bound: {:?}",
            scene.name(),
            loose.quality
        );
    }
}

/// A body at rest at `position` with identity rotation.
pub fn resting(position: [f32; 3]) -> BodyState {
    BodyState {
        position,
        rotation: [0.0, 0.0, 0.0, 1.0],
        ..BodyState::default()
    }
}
