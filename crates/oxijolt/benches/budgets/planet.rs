//! The planet scene of the near-step workload, shared by the benches: 3 x 3 chunks on the
//! planet of radius 99, each a flat terrain heightfield and a compound of structures and
//! features, and 30 walkers on rings around the anchor that walk at 2 m/s along slowly turning
//! headings.

use std::error::Error;

use crate::common::math::{add, normalize, rvec3, scale, vec3};
use crate::common::walker::{
    add_terrain, add_walker, fixture_world, flat_terrain, from_y_to, origin, tangent, up_at,
    Layers, Walker, CENTRE, R, REST_HEIGHT,
};
use crate::common::{Groups, DT};
use oxijolt::*;

/// Side of a chunk, metres.
pub(crate) const CHUNK_SIDE: f32 = 32.0;
/// Walkers (characters) of the workload.
pub(crate) const CHARACTERS: usize = 30;

/// Pose of the planet chunk `(i, k)` of the grid around the anchor: its centre on the planet's
/// surface, its local Y along the radial up there.
pub(crate) fn planet_chunk_pose(i: i32, k: i32) -> (RVec3, Quat) {
    let spacing = 2.0 * (f64::from(CHUNK_SIDE) / 2.0 / R).asin();
    let up = normalize([
        (f64::from(i) * spacing).tan(),
        1.0,
        (f64::from(k) * spacing).tan(),
    ]);
    (rvec3(add(CENTRE, scale(up, R))), from_y_to(up))
}

/// The compound of planet chunk `(i, k)`: 20 sharp boxes (structures) and cylinders (features)
/// standing on its ground, placed from a seed of `(i, k)`.
pub(crate) fn planet_chunk_compound(
    i: i32,
    k: i32,
    block: &Shape,
    pillar: &Shape,
) -> Result<Shape, ShapeError> {
    let mut state = (((i + 1) * 3 + k + 1) as u32).wrapping_mul(2_654_435_761);
    let mut next = move || {
        state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        f64::from(state >> 8) / f64::from(1 << 24) - 0.5
    };
    let children: Vec<CompoundChild<'_>> = (0..20)
        .map(|n| {
            let (x, z) = (next() * 28.0, next() * 28.0);
            let ground = (R * R - x * x - z * z).sqrt() - R;
            let (shape, half, group) = if n % 2 == 0 {
                (block, 1.0, Groups::STRUCTURE)
            } else {
                (pillar, 1.5, Groups::FEATURE)
            };
            CompoundChild {
                shape,
                position: vec3([x, ground + half, z]),
                rotation: Quat::IDENTITY,
                user_data: group,
            }
        })
        .collect();
    Shape::new_compound(&children)
}

/// The shapes every planet chunk compound is made of: a structure block and a feature pillar.
pub(crate) fn chunk_parts() -> Result<(Shape, Shape), ShapeError> {
    Ok((
        Shape::new_box_with_convex_radius(Vec3::new(0.8, 1.0, 0.6), 0.0)?,
        Shape::new_cylinder(1.5, 0.4)?,
    ))
}

/// Static body settings of a planet chunk part at `pose` in `layer`.
pub(crate) fn chunk_body(pose: (RVec3, Quat), layer: ObjectLayer) -> BodySettings {
    BodySettings::new_static()
        .position(pose.0)
        .rotation(pose.1)
        .object_layer(layer)
}

/// The 3 x 3 planet chunks around the anchor, each a flat terrain heightfield and a compound,
/// with an optimised broad phase.
pub(crate) fn planet_scene(worker_threads: u32) -> Result<(PhysicsWorld, Layers), Box<dyn Error>> {
    let (mut world, layers) = fixture_world(worker_threads);
    let (block, pillar) = chunk_parts()?;
    for i in -1..=1 {
        for k in -1..=1 {
            let pose = planet_chunk_pose(i, k);
            add_terrain(&mut world, &layers, &flat_terrain(), pose);
            world.create_body(
                &planet_chunk_compound(i, k, &block, &pillar)?,
                &chunk_body(pose, layers.chunk),
            )?;
        }
    }
    world.optimize_broad_phase();
    Ok((world, layers))
}

/// The point of the planet's surface over chart position `(x, z)` near the anchor.
pub(crate) fn ground_at(x: f64, z: f64) -> [f64; 3] {
    [x, (R * R - x * x - z * z).sqrt() - R, z]
}

/// Where walker `n` of `CHARACTERS` starts: resting on one of five rings around the anchor.
pub(crate) fn walker_start(n: usize) -> [f64; 3] {
    let angle = n as f64 / CHARACTERS as f64 * std::f64::consts::TAU;
    let radius = 6.0 + (n % 5) as f64 * 7.0;
    let ground = ground_at(radius * angle.cos(), radius * angle.sin());
    add(ground, scale(up_at(ground), f64::from(REST_HEIGHT)))
}

/// 30 walkers resting on rings around the anchor.
pub(crate) fn planet_walkers(world: &mut PhysicsWorld, layers: &Layers) -> Vec<Walker> {
    (0..CHARACTERS)
        .map(|n| add_walker(world, layers, walker_start(n)))
        .collect()
}

/// What walker `n` wants at `tick`: 2 m/s along a heading that turns slowly.
pub(crate) fn wanted(world: &PhysicsWorld, walker: &Walker, n: usize, tick: usize) -> [f64; 3] {
    let heading = n as f64 + tick as f64 * 0.01;
    let direction = [heading.cos(), 0.0, heading.sin()];
    tangent(origin(world, walker), direction, 2.0 * f64::from(DT))
}
