//! Small scenes for tests and CI: one chain per joint kind and a short stack.

use crate::scene::{BodySpec, JointKind, JointSpec, SceneSpec, Shape};

/// Bodies in each fixture chain, besides its fixed anchor.
pub const CHAIN_LINKS: usize = 5;

/// The slider axis of the prismatic fixture: straight down, so gravity holds every slider of the
/// hanging chain against its upper limit (0) from the start.
pub const SLIDER_AXIS: [f32; 3] = [0.0, -1.0, 0.0];

fn cube(half_extent: f32) -> Shape {
    Shape::Cuboid {
        half_extents: [half_extent; 3],
    }
}

/// Five 0.8 m cubes 1 m apart hanging from a fixed cube at 10 m, each joined to the one before by
/// a `kind` joint anchored at the earlier body's centre. Ball joints and hinges start horizontal
/// along +x, so gravity swings them about their free axes; fixed joints and sliders hang straight
/// down, so their locked axes carry the weight without a lever.
pub fn chain(name: &'static str, kind: JointKind) -> SceneSpec {
    let step: [f32; 3] = match kind {
        JointKind::Fixed | JointKind::Prismatic { .. } => [0.0, -1.0, 0.0],
        JointKind::Spherical | JointKind::Revolute { .. } => [1.0, 0.0, 0.0],
    };
    let anchor = [0.0, 10.0, 0.0];
    let mut bodies = vec![BodySpec::fixed(cube(0.4), anchor)];
    let mut joints = Vec::new();
    for i in 1..=CHAIN_LINKS {
        let position = [0, 1, 2].map(|axis| anchor[axis] + i as f32 * step[axis]);
        bodies.push(BodySpec::dynamic(cube(0.4), position));
        joints.push(JointSpec {
            kind,
            body1: i - 1,
            body2: i,
            local_anchor1: [0.0; 3],
            local_anchor2: step.map(|c| -c),
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
