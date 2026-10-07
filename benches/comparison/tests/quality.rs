//! The quality measures and the digest on hand-made states.

mod common;

use common::resting;
use comparison::engine::BodyState;
use comparison::quality::{digest, first_difference, QualityTracker};
use comparison::scene::{BodySpec, Shape};
use comparison::scene::{JointKind, JointSpec, Scene, SceneClass, SceneSpec};

/// One slider along +x from a fixed cube to a dynamic one, limits [-2, 0].
fn slider() -> SceneSpec {
    let cube = Shape::Cuboid {
        half_extents: [0.4; 3],
    };
    SceneSpec {
        name: "slider",
        source: "test",
        bodies: vec![
            BodySpec::fixed(cube, [0.0, 0.0, 0.0]),
            BodySpec::dynamic(cube, [1.0, 0.0, 0.0]),
        ],
        joints: vec![JointSpec {
            kind: JointKind::Prismatic {
                axis: [1.0, 0.0, 0.0],
                limits: [-2.0, 0.0],
            },
            body1: 0,
            body2: 1,
            local_anchor1: [0.0; 3],
            local_anchor2: [-1.0, 0.0, 0.0],
        }],
    }
}

fn slider_violations(second: [f32; 3]) -> Vec<&'static str> {
    let spec = slider();
    let mut tracker = QualityTracker::new(&spec, SceneClass::Joints);
    tracker.observe(&[resting([0.0; 3]), resting(second)]);
    tracker.finish(2).violations()
}

#[test]
fn a_slider_moved_along_its_axis_within_limits_is_fine() {
    assert!(slider_violations([1.0, 0.0, 0.0]).is_empty());
    assert!(slider_violations([-0.5, 0.0, 0.0]).is_empty());
}

#[test]
fn a_slider_moved_sideways_or_past_its_limit_is_not() {
    assert_eq!(slider_violations([1.0, 0.3, 0.0]), ["anchor_p99"]);
    assert_eq!(slider_violations([-1.5, 0.0, 0.0]), ["limit_p99"]);
    assert_eq!(slider_violations([1.5, 0.0, 0.0]), ["limit_p99"]);
}

#[test]
fn a_box_sunk_into_the_ground_is_reported() {
    let spec = Scene::FixtureStack.build();
    let mut states: Vec<BodyState> = spec.bodies.iter().map(|b| resting(b.position)).collect();
    let mut tracker = QualityTracker::new(&spec, SceneClass::Stack);
    tracker.observe(&states);
    states[1].position[1] -= 0.2;
    tracker.observe(&states);
    let quality = tracker.finish(0);
    assert!((quality.ground_penetration.unwrap() - 0.2).abs() < 1e-6);
    assert_eq!(quality.violations(), ["ground_penetration"]);
}

#[test]
fn the_digest_sees_every_velocity_bit_and_the_sleep_flag() {
    let base = vec![resting([1.0, 2.0, 3.0]); 3];
    let reference = digest(&base);
    for component in 0..3 {
        for field in 0..2 {
            let mut changed = base.clone();
            let v = if field == 0 {
                &mut changed[2].linear_velocity
            } else {
                &mut changed[2].angular_velocity
            };
            v[component] = f32::from_bits(v[component].to_bits() ^ 1);
            assert_ne!(digest(&changed), reference);
        }
    }
    let mut asleep = base.clone();
    asleep[1].sleeping = true;
    assert_ne!(digest(&asleep), reference);
}

#[test]
fn digest_streams_report_the_first_differing_tick() {
    assert_eq!(first_difference(&[1, 2, 3], &[1, 2, 3]), None);
    assert_eq!(first_difference(&[1, 2, 3], &[1, 9, 3]), Some(2));
    assert_eq!(first_difference(&[1, 2], &[1, 2, 3]), Some(3));
}

#[test]
fn an_impact_overlap_that_settles_is_reported_but_not_a_violation() {
    let ball = Shape::Ball { radius: 1.0 };
    let spec = SceneSpec {
        name: "two balls",
        source: "test",
        bodies: vec![
            BodySpec::fixed(ball, [0.0, 0.0, 0.0]),
            BodySpec::dynamic(ball, [3.0, 0.0, 0.0]),
        ],
        joints: Vec::new(),
    };
    let mut tracker = QualityTracker::new(&spec, SceneClass::Balls);
    tracker.observe(&[resting([0.0; 3]), resting([1.7, 0.0, 0.0])]);
    tracker.observe(&[resting([0.0; 3]), resting([2.0, 0.0, 0.0])]);
    let quality = tracker.finish(1);
    assert!((quality.ball_overlap_max.unwrap() - 0.3).abs() < 1e-5);
    assert!(quality.ball_overlap.unwrap().abs() < 1e-6);
    assert!(quality.violations().is_empty());
}

#[test]
fn joint_errors_are_bounded_as_their_warm_average() {
    let spec = slider();
    let mut tracker = QualityTracker::new(&spec, SceneClass::Joints);
    // Pulled 0.5 m sideways for the first second, then back on the axis for the second.
    for tick in 1..=120 {
        let y = if tick <= 60 { 0.5 } else { 0.0 };
        tracker.observe(&[resting([0.0; 3]), resting([1.0, y, 0.0])]);
    }
    let quality = tracker.finish(1);
    assert!((quality.anchor_max.unwrap() - 0.5).abs() < 1e-6);
    assert_eq!(quality.anchor_p99, Some(0.0));
    assert!(quality.violations().is_empty());
}
