//! The ported scenes against the numbers in Rapier's sources at v0.36.0.

use comparison::scene::{JointKind, Motion, Scene, SceneSpec, Shape};
use comparison::scenes::{joints, stacks};

fn fixed_count(spec: &SceneSpec) -> usize {
    spec.bodies
        .iter()
        .filter(|body| body.motion == Motion::Fixed)
        .count()
}

#[test]
fn body_and_joint_counts_match_rapiers_scenes() {
    // (scene, bodies, fixed bodies, joints)
    let expected = [
        (Scene::Balls, 8000, 400, 0),
        (Scene::Boxes, 1001, 1, 0),
        (Scene::Capsules, 3009, 1, 0),
        (Scene::Pyramid, 42926, 1, 0),
        (Scene::ManyPyramids, 8401, 1, 0),
        (Scene::Keva, 38271, 1, 0),
        (Scene::JointBall, 10000, 26, 19800),
        (Scene::JointFixed, 12500, 1000, 20000),
        (Scene::JointPrismatic, 19200, 3200, 16000),
        (Scene::JointRevolute, 8200, 200, 8000),
    ];
    for (scene, bodies, fixed, joint_count) in expected {
        let spec = scene.build();
        assert_eq!(spec.name, scene.name());
        assert_eq!(spec.bodies.len(), bodies, "{} bodies", scene.name());
        assert_eq!(fixed_count(&spec), fixed, "{} fixed bodies", scene.name());
        assert_eq!(spec.joints.len(), joint_count, "{} joints", scene.name());
    }
}

#[test]
fn every_scene_name_round_trips() {
    for scene in Scene::RAPIER.into_iter().chain(Scene::FIXTURES) {
        assert_eq!(Scene::from_name(scene.name()), Some(scene));
    }
    assert_eq!(Scene::from_name("nonsense"), None);
}

#[test]
fn balls_have_rapiers_density_and_a_fixed_bottom_layer() {
    let spec = Scene::Balls.build();
    for body in &spec.bodies {
        assert_eq!(body.density, 0.477);
        assert_eq!(body.shape, Shape::Ball { radius: 1.0 });
        // The bottom layer is at y = 1.5 (shift / 2) and is the only fixed one.
        assert_eq!(body.motion == Motion::Fixed, body.position[1] == 1.5);
    }
    assert_eq!(spec.ground_top(), None);
}

#[test]
fn pyramid_bricks_have_rapiers_size_density_and_pitch() {
    let spec = Scene::Pyramid.build();
    assert_eq!(stacks::PYRAMID_HALF_EXTENT, 0.975);
    assert_eq!(spec.ground_top(), Some(0.0));
    let bricks = &spec.bodies[1..];
    for body in bricks {
        assert_eq!(
            body.shape,
            Shape::Cuboid {
                half_extents: [0.975; 3]
            }
        );
        assert_eq!(body.density, 1000.0);
    }
    // The first two bricks of layer 0 are 2.25 m apart; layer 1 starts one brick offset (1.0)
    // further and 2.5 m higher.
    assert_eq!(bricks[1].position[2] - bricks[0].position[2], 2.25);
    let layer1 = &bricks[50 * 50];
    assert_eq!(layer1.position[1] - bricks[0].position[1], 2.5);
    assert_eq!(layer1.position[0] - bricks[0].position[0], 1.0);
}

#[test]
fn capsules_are_rapiers_unit_capsules() {
    let spec = Scene::Capsules.build();
    for body in &spec.bodies[1..] {
        assert_eq!(
            body.shape,
            Shape::CapsuleY {
                half_height: 1.0,
                radius: 1.0
            }
        );
        assert_eq!(body.density, 1.0);
    }
}

#[test]
fn keva_planks_alternate_their_half_extents() {
    // `0.02 / 2 * 10` in f32 is just below 0.1, in Rapier as here.
    let [hx, hy, hz] = stacks::keva_half_extents();
    assert_eq!([hx, hy, hz], [0.099_999_994, 0.5, 2.0]);
    let spec = Scene::Keva.build();
    let mut seen = Vec::new();
    for body in &spec.bodies[1..] {
        let Shape::Cuboid { half_extents } = body.shape else {
            panic!("keva has only planks");
        };
        if !seen.contains(&half_extents) {
            seen.push(half_extents);
        }
    }
    seen.sort_by(|a, b| a.partial_cmp(b).unwrap());
    assert_eq!(
        seen,
        vec![[hx, hy, hz], [hz, hx, hy], [hz, hy, hx]],
        "the two wall orientations and the top layer"
    );
}

#[test]
fn prismatic_axes_alternate_and_share_rapiers_limits() {
    let spec = Scene::JointPrismatic.build();
    let s = joints::normalize([1.0, 1.0, 0.0])[0];
    for (index, joint) in spec.joints.iter().enumerate() {
        let JointKind::Prismatic { axis, limits } = joint.kind else {
            panic!("joint_prismatic has only sliders");
        };
        assert_eq!(limits, [-2.0, 0.0]);
        let expected = if index % 5 % 2 == 0 {
            [s, s, 0.0]
        } else {
            [-s, s, 0.0]
        };
        assert_eq!(axis, expected, "joint {index}");
    }
}

#[test]
fn joint_anchors_are_at_body_1s_centre_and_meet_body_2s() {
    for scene in Scene::RAPIER.into_iter().chain(Scene::FIXTURES) {
        let spec = scene.build();
        for joint in &spec.joints {
            assert!(joint.body1 < spec.bodies.len() && joint.body2 < spec.bodies.len());
            assert_ne!(joint.body1, joint.body2, "{}", scene.name());
            assert_eq!(joint.local_anchor1, [0.0; 3]);
            let anchor = spec.anchor(joint);
            let p2 = spec.bodies[joint.body2].position;
            let anchor2 = [0, 1, 2].map(|i| p2[i] + joint.local_anchor2[i]);
            assert_eq!(anchor, anchor2, "{}: anchors differ", scene.name());
        }
    }
}

#[test]
fn every_dynamic_body_has_positive_mass() {
    for scene in Scene::RAPIER.into_iter().chain(Scene::FIXTURES) {
        let spec = scene.build();
        assert!(spec.dynamic_count() > 0);
        for body in &spec.bodies {
            assert!(body.mass() > 0.0, "{}", scene.name());
        }
    }
}

#[test]
fn shape_inertia_matches_textbook_values() {
    let cube = Shape::Cuboid {
        half_extents: [0.5; 3],
    };
    assert_eq!(cube.principal_inertia(6.0), [1.0; 3]);
    let ball = Shape::Ball { radius: 1.0 };
    assert_eq!(ball.principal_inertia(5.0), [2.0; 3]);
    // A capsule with no cylinder is a ball.
    let capsule = Shape::CapsuleY {
        half_height: 0.0,
        radius: 1.0,
    };
    for (a, b) in capsule.principal_inertia(5.0).iter().zip([2.0; 3]) {
        assert!((a - b).abs() < 1e-12);
    }
}
