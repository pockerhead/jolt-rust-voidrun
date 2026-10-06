//! Ported from Rapier's `examples3d/stress_tests/` at v0.36.0: `joint_ball3.rs`,
//! `joint_fixed3.rs`, `joint_prismatic3.rs` and `joint_revolute3.rs`
//! (<https://github.com/dimforge/rapier/tree/v0.36.0/examples3d/stress_tests>).
//! Copyright Dimforge and contributors, Apache-2.0. Changed: the testbed setup is removed and
//! the bodies and joints are written to engine-neutral data. Every joint has its anchor at body
//! 1's centre (Rapier's default `local_anchor1`) and `local_anchor2` as in Rapier.

use crate::scene::{BodySpec, JointKind, JointSpec, Motion, SceneSpec, Shape, V3};

/// Limits of every slider in `joint_prismatic3`, in metres along the axis.
pub const PRISMATIC_LIMITS: [f32; 2] = [-2.0, 0.0];

/// Radius of the balls of `joint_ball3` and `joint_fixed3`, and half extent of the cubes of
/// `joint_prismatic3` and `joint_revolute3`.
const RAD: f32 = 0.4;

fn joint(kind: JointKind, body1: usize, body2: usize, local_anchor2: V3) -> JointSpec {
    JointSpec {
        kind,
        body1,
        body2,
        local_anchor1: [0.0; 3],
        local_anchor2,
    }
}

fn cube() -> Shape {
    Shape::Cuboid {
        half_extents: [RAD; 3],
    }
}

fn body(motion: Motion, shape: Shape, position: V3) -> BodySpec {
    match motion {
        Motion::Fixed => BodySpec::fixed(shape, position),
        Motion::Dynamic => BodySpec::dynamic(shape, position),
    }
}

/// A grid of `num` x `num` balls joined to the previous ball of their row and column; the first
/// ball of every fourth row and of the last row is fixed.
fn ball_grid(
    bodies: &mut Vec<BodySpec>,
    joints: &mut Vec<JointSpec>,
    kind: JointKind,
    num: usize,
    is_fixed: impl Fn(usize, usize) -> bool,
    origin: V3,
) {
    let shift = 1.0f32;
    for k in 0..num {
        for i in 0..num {
            let fk = k as f32;
            let fi = i as f32;
            let motion = if is_fixed(i, k) {
                Motion::Fixed
            } else {
                Motion::Dynamic
            };
            let position = [origin[0] + fk * shift, origin[1], origin[2] + fi * shift];
            let child = bodies.len();
            bodies.push(body(motion, Shape::Ball { radius: RAD }, position));
            if i > 0 {
                joints.push(joint(kind, child - 1, child, [0.0, 0.0, -shift]));
            }
            if k > 0 {
                joints.push(joint(kind, child - num, child, [-shift, 0.0, 0.0]));
            }
        }
    }
}

/// `joint_ball3`: a 100 x 100 net of balls joined by ball joints.
pub fn joint_ball() -> SceneSpec {
    let num = 100;
    let mut bodies = Vec::new();
    let mut joints = Vec::new();
    ball_grid(
        &mut bodies,
        &mut joints,
        JointKind::Spherical,
        num,
        |i, k| i == 0 && (k % 4 == 0 || k == num - 1),
        [0.0; 3],
    );
    SceneSpec {
        name: "joint_ball",
        source: "examples3d/stress_tests/joint_ball3.rs",
        bodies,
        joints,
    }
}

/// `joint_fixed3`: 500 nets of 5 x 5 balls joined by fixed joints.
pub fn joint_fixed() -> SceneSpec {
    let num = 5;
    let shift = 1.0f32;
    let mut bodies = Vec::new();
    let mut joints = Vec::new();
    for m in 0..10 {
        let z = m as f32 * shift * (num as f32 + 2.0);
        for l in 0..10 {
            let y = l as f32 * shift * 3.0;
            for j in 0..5 {
                let x = j as f32 * shift * (num as f32) * 2.0;
                ball_grid(
                    &mut bodies,
                    &mut joints,
                    JointKind::Fixed,
                    num,
                    |i, k| i == 0 && (k % 4 == 0 && k != num - 2 || k == num - 1),
                    [x, y, z],
                );
            }
        }
    }
    SceneSpec {
        name: "joint_fixed",
        source: "examples3d/stress_tests/joint_fixed3.rs",
        bodies,
        joints,
    }
}

/// `joint_prismatic3`: 3200 chains of five cubes on a fixed cube, joined by sliders whose axes
/// alternate between `(1, 1, 0)` and `(-1, 1, 0)`, normalised.
pub fn joint_prismatic() -> SceneSpec {
    let num = 5;
    let shift = 1.0f32;
    let mut bodies = Vec::new();
    let mut joints = Vec::new();
    for m in 0..8 {
        let z = m as f32 * shift * (num as f32 + 2.0);
        for l in 0..8 {
            let y = l as f32 * shift * (num as f32) * 2.0;
            for j in 0..50 {
                let x = j as f32 * shift * 4.0;
                let mut parent = bodies.len();
                bodies.push(BodySpec::fixed(cube(), [x, y, z]));
                for i in 0..num {
                    let z = z + (i + 1) as f32 * shift;
                    let child = bodies.len();
                    bodies.push(BodySpec::dynamic(cube(), [x, y, z]));
                    let axis = if i % 2 == 0 {
                        normalize([1.0, 1.0, 0.0])
                    } else {
                        normalize([-1.0, 1.0, 0.0])
                    };
                    let kind = JointKind::Prismatic {
                        axis,
                        limits: PRISMATIC_LIMITS,
                    };
                    joints.push(joint(kind, parent, child, [0.0, 0.0, -shift]));
                    parent = child;
                }
            }
        }
    }
    SceneSpec {
        name: "joint_prismatic",
        source: "examples3d/stress_tests/joint_prismatic3.rs",
        bodies,
        joints,
    }
}

/// `v` scaled to unit length, as glam's `Vec3::normalize` computes it: times the reciprocal of
/// the length.
pub fn normalize(v: V3) -> V3 {
    let length = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    let inverse = 1.0 / length;
    v.map(|c| c * inverse)
}

/// `joint_revolute3`: 200 chains of ten squares of four cubes on a fixed cube, joined by hinges
/// about x and z.
pub fn joint_revolute() -> SceneSpec {
    let num = 10;
    let shift = 2.0f32;
    let x_axis = [1.0, 0.0, 0.0];
    let z_axis = [0.0, 0.0, 1.0];
    let mut bodies = Vec::new();
    let mut joints = Vec::new();
    for l in 0..4 {
        let y = l as f32 * shift * (num as f32) * 3.0;
        for j in 0..50 {
            let x = j as f32 * shift * 4.0;
            let mut parent = bodies.len();
            bodies.push(BodySpec::fixed(cube(), [x, y, 0.0]));
            for i in 0..num {
                let z = i as f32 * shift * 2.0 + shift;
                let positions = [
                    [x, y, z],
                    [x + shift, y, z],
                    [x + shift, y, z + shift],
                    [x, y, z + shift],
                ];
                let handles: [usize; 4] = std::array::from_fn(|k| {
                    bodies.push(BodySpec::dynamic(cube(), positions[k]));
                    bodies.len() - 1
                });
                let hinges = [
                    (z_axis, [0.0, 0.0, -shift]),
                    (x_axis, [-shift, 0.0, 0.0]),
                    (z_axis, [0.0, 0.0, -shift]),
                    (x_axis, [shift, 0.0, 0.0]),
                ];
                let pairs = [
                    (parent, handles[0]),
                    (handles[0], handles[1]),
                    (handles[1], handles[2]),
                    (handles[2], handles[3]),
                ];
                for ((body1, body2), (axis, anchor)) in pairs.into_iter().zip(hinges) {
                    joints.push(joint(JointKind::Revolute { axis }, body1, body2, anchor));
                }
                parent = handles[3];
            }
        }
    }
    SceneSpec {
        name: "joint_revolute",
        source: "examples3d/stress_tests/joint_revolute3.rs",
        bodies,
        joints,
    }
}
