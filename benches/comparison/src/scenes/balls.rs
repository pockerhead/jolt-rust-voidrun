//! Ported from Rapier's `examples3d/stress_tests/balls3.rs` at v0.36.0
//! (<https://github.com/dimforge/rapier/blob/v0.36.0/examples3d/stress_tests/balls3.rs>).
//! Copyright Dimforge and contributors, Apache-2.0. Changed: the testbed setup is removed and
//! the bodies are written to engine-neutral data.

use crate::scene::{BodySpec, SceneSpec, Shape};

/// A 20 x 20 x 20 grid of balls whose bottom layer is fixed; no ground.
pub fn balls() -> SceneSpec {
    let num = 20;
    let rad = 1.0f32;

    let shift = rad * 2.0 + 1.0;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift / 2.0;
    let centerz = shift * (num as f32) / 2.0;

    let mut bodies = Vec::new();
    for i in 0..num {
        for j in 0usize..num {
            for k in 0..num {
                let x = i as f32 * shift - centerx;
                let y = j as f32 * shift + centery;
                let z = k as f32 * shift - centerz;
                let shape = Shape::Ball { radius: rad };
                let body = if j == 0 {
                    BodySpec::fixed(shape, [x, y, z])
                } else {
                    BodySpec::dynamic(shape, [x, y, z])
                };
                bodies.push(body.with_density(0.477));
            }
        }
    }
    SceneSpec {
        name: "balls",
        source: "examples3d/stress_tests/balls3.rs",
        bodies,
        joints: Vec::new(),
    }
}
