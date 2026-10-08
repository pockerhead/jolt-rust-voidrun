//! Ported from Rapier's `examples3d/stress_tests/` at v0.36.0: `boxes3.rs`, `capsules3.rs`,
//! `pyramid3.rs`, `many_pyramids3.rs` and `keva3.rs`
//! (<https://github.com/dimforge/rapier/tree/v0.36.0/examples3d/stress_tests>).
//! Copyright Dimforge and contributors, Apache-2.0. Changed: the testbed setup is removed and
//! the bodies are written to engine-neutral data; the arithmetic is Rapier's, in `f32` and in the
//! same order, so the positions are the same bits.

use crate::scene::{BodySpec, SceneSpec, Shape, V3};

fn cuboid(hx: f32, hy: f32, hz: f32) -> Shape {
    Shape::Cuboid {
        half_extents: [hx, hy, hz],
    }
}

/// The thin ground box of `boxes3`, `capsules3`, `many_pyramids3` and `keva3`.
fn thin_ground(size_x: f32, size_z: f32) -> BodySpec {
    let ground_height = 0.1;
    BodySpec::fixed(
        cuboid(size_x, ground_height, size_z),
        [0.0, -ground_height, 0.0],
    )
}

/// `boxes3`: ten layers of 10 x 10 cubes, each layer shifted sideways, on a ground.
pub fn boxes() -> SceneSpec {
    let ground_size = 200.1;
    let mut bodies = vec![thin_ground(ground_size, ground_size)];

    let num = 10usize;
    let rad = 1.0f32;

    let shift = rad * 2.0;
    let centerx = shift * (num / 2) as f32;
    let centery = shift / 2.0;
    let centerz = shift * (num / 2) as f32;

    let mut offset = -(num as f32) * (rad * 2.0) * 0.5;

    for j in 0usize..num {
        for i in 0..num {
            for k in 0usize..num {
                let x = i as f32 * shift - centerx + offset;
                let y = j as f32 * shift + centery;
                let z = k as f32 * shift - centerz + offset;
                bodies.push(BodySpec::dynamic(cuboid(rad, rad, rad), [x, y, z]));
            }
        }
        offset -= 0.05 * rad * (num as f32 - 1.0);
    }
    SceneSpec {
        name: "boxes",
        source: "examples3d/stress_tests/boxes3.rs",
        bodies,
        joints: Vec::new(),
    }
}

/// `capsules3`: 47 layers of 8 x 8 capsules dropped from 3 m onto a ground.
pub fn capsules() -> SceneSpec {
    let ground_size = 200.1;
    let mut bodies = vec![thin_ground(ground_size, ground_size)];

    let num = 8usize;
    let rad = 1.0f32;

    let shift = rad * 2.0 + rad;
    let shifty = rad * 4.0;
    let centerx = shift * (num / 2) as f32;
    let centery = shift / 2.0;
    let centerz = shift * (num / 2) as f32;

    let mut offset = -(num as f32) * (rad * 2.0 + rad) * 0.5;

    for j in 0usize..47 {
        for i in 0..num {
            for k in 0usize..num {
                let x = i as f32 * shift - centerx + offset;
                let y = j as f32 * shifty + centery + 3.0;
                let z = k as f32 * shift - centerz + offset;
                let shape = Shape::CapsuleY {
                    half_height: rad,
                    radius: rad,
                };
                bodies.push(BodySpec::dynamic(shape, [x, y, z]));
            }
        }
        offset -= 0.05 * rad * (num as f32 - 1.0);
    }
    SceneSpec {
        name: "capsules",
        source: "examples3d/stress_tests/capsules3.rs",
        bodies,
        joints: Vec::new(),
    }
}

/// Half extent of a `pyramid3` box: a 2 m cube shrunk by 2.5 cm on each side.
pub const PYRAMID_HALF_EXTENT: f32 = 1.0 - 0.025;

/// `pyramid3`: 50 brick-laid layers of (50 - i)² boxes of water density, on a ground whose top
/// is at y = 0.
pub fn pyramid() -> SceneSpec {
    let mut bodies = vec![BodySpec::fixed(cuboid(100.0, 1.0, 100.0), [0.0, -1.0, 0.0])];

    let pyramid_height = 50i32;
    let box_size = 2.0f32;
    let box_separation = 0.5;
    let half_box_size = 0.5 * box_size;
    let h = half_box_size - 0.025;

    for i in 0..pyramid_height {
        let brick = if i & 1 != 0 { half_box_size } else { 0.0 };
        let y = 1.0 + (box_size + box_separation) * i as f32;
        for j in i / 2..pyramid_height - (i + 1) / 2 {
            for k in i / 2..pyramid_height - (i + 1) / 2 {
                let x = -(pyramid_height as f32) + (box_size + 0.25) * j as f32 + brick;
                let z = -(pyramid_height as f32) + (box_size + 0.25) * k as f32 + brick;
                bodies.push(BodySpec::dynamic(cuboid(h, h, h), [x, y, z]).with_density(1000.0));
            }
        }
    }
    SceneSpec {
        name: "pyramid",
        source: "examples3d/stress_tests/pyramid3.rs",
        bodies,
        joints: Vec::new(),
    }
}

/// `create_pyramid` of `many_pyramids3`: a 2D pyramid of `stack_height` rows in the x-y plane.
fn push_pyramid(bodies: &mut Vec<BodySpec>, offset: V3, stack_height: usize, rad: f32) {
    let shift = rad * 2.0;
    for i in 0usize..stack_height {
        for j in i..stack_height {
            let fj = j as f32;
            let fi = i as f32;
            let x = (fi * shift / 2.0) + (fj - fi) * shift;
            let y = fi * shift;
            let position = [x + offset[0], y + offset[1], 0.0 + offset[2]];
            bodies.push(BodySpec::dynamic(cuboid(rad, rad, rad), position));
        }
    }
}

/// `many_pyramids3`: 40 pyramids of 20 rows of unit cubes, 4 m apart, on a ground.
pub fn many_pyramids() -> SceneSpec {
    let rad = 0.5f32;
    let pyramid_count = 40;
    let spacing = 4.0f32;

    let ground_size = 50.0;
    let mut bodies = vec![thin_ground(
        ground_size,
        pyramid_count as f32 * spacing / 2.0 + ground_size,
    )];

    for pyramid_index in 0..pyramid_count {
        let bottomy = rad;
        let offset = [
            0.0,
            bottomy,
            (pyramid_index as f32 - pyramid_count as f32 / 2.0) * spacing,
        ];
        push_pyramid(&mut bodies, offset, 20, rad);
    }
    SceneSpec {
        name: "many_pyramids",
        source: "examples3d/stress_tests/many_pyramids3.rs",
        bodies,
        joints: Vec::new(),
    }
}

/// `build_block` of `keva3`: `numy` layers of planks, alternating direction, closed by a top
/// layer.
fn push_keva_block(
    bodies: &mut Vec<BodySpec>,
    half_extents: V3,
    shift: V3,
    (mut numx, numy, mut numz): (usize, usize, usize),
) {
    let [hx, hy, hz] = half_extents;
    let dimensions = [[hx, hy, hz], [hz, hy, hx]];
    let block_width = 2.0 * hz * numx as f32;
    let block_height = 2.0 * hy * numy as f32;
    let spacing = (hz * numx as f32 - hx) / (numz as f32 - 1.0);

    for i in 0..numy {
        std::mem::swap(&mut numx, &mut numz);
        let dim = dimensions[i % 2];
        let y = dim[1] * i as f32 * 2.0;
        for j in 0..numx {
            let x = if i % 2 == 0 {
                spacing * j as f32 * 2.0
            } else {
                dim[0] * j as f32 * 2.0
            };
            for k in 0..numz {
                let z = if i % 2 == 0 {
                    dim[2] * k as f32 * 2.0
                } else {
                    spacing * k as f32 * 2.0
                };
                let position = [
                    x + dim[0] + shift[0],
                    y + dim[1] + shift[1],
                    z + dim[2] + shift[2],
                ];
                bodies.push(BodySpec::dynamic(cuboid(dim[0], dim[1], dim[2]), position));
            }
        }
    }

    // Close the top.
    let dim = [hz, hx, hy];
    for i in 0..(block_width / (dim[0] * 2.0)) as usize {
        for j in 0..(block_width / (dim[2] * 2.0)) as usize {
            let position = [
                i as f32 * dim[0] * 2.0 + dim[0] + shift[0],
                dim[1] + shift[1] + block_height,
                j as f32 * dim[2] * 2.0 + dim[2] + shift[2],
            ];
            bodies.push(BodySpec::dynamic(cuboid(dim[0], dim[1], dim[2]), position));
        }
    }
}

/// Half extents of a Keva plank: `(0.02, 0.1, 0.4) / 2 * 10`, computed as Rapier does.
pub fn keva_half_extents() -> V3 {
    [0.02f32, 0.1, 0.4].map(|c| c / 2.0 * 10.0)
}

/// `keva3`: five stacked Keva towers of planks, 38 270 bodies, on a ground.
pub fn keva() -> SceneSpec {
    let ground_size = 50.0;
    let mut bodies = vec![thin_ground(ground_size, ground_size)];

    let half_extents = keva_half_extents();
    let mut block_height = 0.0;
    let numy = [0, 13, 17, 21, 41, 83];

    for i in (1..=5).rev() {
        let numx = i * 2;
        let numy = numy[i];
        let numz = numx * 3 + 1;
        let block_width = numx as f32 * half_extents[2] * 2.0;
        push_keva_block(
            &mut bodies,
            half_extents,
            [-block_width / 2.0, block_height, -block_width / 2.0],
            (numx, numy, numz),
        );
        block_height += numy as f32 * half_extents[1] * 2.0 + half_extents[0] * 2.0;
    }
    SceneSpec {
        name: "keva",
        source: "examples3d/stress_tests/keva3.rs",
        bodies,
        joints: Vec::new(),
    }
}
