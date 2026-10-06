//! Small scenes for tests and CI: one chain per joint kind and a short stack.

use crate::scene::{BodySpec, JointKind, JointSpec, SceneSpec, Shape};

/// Bodies in each fixture chain, besides its fixed anchor.
pub const CHAIN_LINKS: usize = 5;

/// The slider axis of the prismatic fixture: diagonal, so gravity drives the sliders to their
/// lower limit.
pub const SLIDER_AXIS: [f32; 3] = [
    std::f32::consts::FRAC_1_SQRT_2,
    std::f32::consts::FRAC_1_SQRT_2,
    0.0,
];

fn cube(half_extent: f32) -> Shape {
    Shape::Cuboid {
        half_extents: [half_extent; 3],
    }
}

/// Five 0.8 m cubes in a row along +x, 1 m apart, hanging from a fixed cube at 10 m, each joined
/// to the one before by a `kind` joint anchored at the earlier body's centre.
pub fn chain(name: &'static str, kind: JointKind) -> SceneSpec {
    let mut bodies = vec![BodySpec::fixed(cube(0.4), [0.0, 10.0, 0.0])];
    let mut joints = Vec::new();
    for i in 1..=CHAIN_LINKS {
        bodies.push(BodySpec::dynamic(cube(0.4), [i as f32, 10.0, 0.0]));
        joints.push(JointSpec {
            kind,
            body1: i - 1,
            body2: i,
            local_anchor1: [0.0; 3],
            local_anchor2: [-1.0, 0.0, 0.0],
        });
    }
    SceneSpec {
        name,
        source: "fixture",
        bodies,
        joints,
    }
}

/// Five unit cubes stacked on a ground whose top is at y = 0.
pub fn stack(name: &'static str) -> SceneSpec {
    let mut bodies = vec![BodySpec::fixed(
        Shape::Cuboid {
            half_extents: [50.0, 0.1, 50.0],
        },
        [0.0, -0.1, 0.0],
    )];
    for i in 0..5u8 {
        bodies.push(BodySpec::dynamic(cube(0.5), [0.0, 0.5 + f32::from(i), 0.0]));
    }
    SceneSpec {
        name,
        source: "fixture",
        bodies,
        joints: Vec::new(),
    }
}
