//! The pose readout case: the poses of 256 awake bodies read once per tick, one body at a time
//! and with `active_body_poses_into`.

use std::error::Error;
use std::hint::black_box;
use std::time::Instant;

use crate::planet::planet_scene;
use crate::report::{micros, Limit, Row};
use crate::{DT, TICKS, WARMUP_TICKS};
use oxijolt::*;

/// Awake bodies whose poses are read.
const BODIES: usize = 256;
/// Bodies along each side of the square they float in.
const SIDE: usize = 16;

/// Reads every tracked body's pose through `world.body(id)`, as a caller without the batch call
/// does, into `out`.
fn per_body(
    world: &PhysicsWorld,
    ids: &[BodyId],
    out: &mut Vec<(BodyId, RVec3, Quat)>,
) -> Result<(), BodyError> {
    out.clear();
    for &id in ids {
        let body = world.body(id)?;
        if body.is_active() {
            out.push((id, body.position(), body.rotation()));
        }
    }
    Ok(())
}

/// The planet scene with 256 awake cubes floating without gravity well above its structures,
/// warmed up.
fn build() -> Result<(PhysicsWorld, Vec<BodyId>), Box<dyn Error>> {
    let (mut world, layers) = planet_scene(4)?;
    let cube = Shape::new_box(Vec3::new(0.25, 0.25, 0.25))?;
    let mut ids = Vec::with_capacity(BODIES);
    for n in 0..BODIES {
        let (column, row) = ((n % SIDE) as Real, (n / SIDE) as Real);
        let position = RVec3::new(column - 7.5, 30.0, row - 7.5);
        ids.push(
            world.create_body(
                &cube,
                &BodySettings::new_dynamic()
                    .position(position)
                    .object_layer(layers.item)
                    .allow_sleeping(false),
            )?,
        );
    }
    world.optimize_broad_phase();
    for _ in 0..WARMUP_TICKS {
        assert!(world.step(DT)?.is_complete());
    }
    Ok((world, ids))
}

/// Times both readouts on the same state each tick, alternating which runs first.
pub(crate) fn run_poses() -> Result<[Row; 2], Box<dyn Error>> {
    let (mut world, ids) = build()?;
    let mut single = Vec::with_capacity(BODIES);
    let mut batch = Vec::with_capacity(BODIES);
    per_body(&world, &ids, &mut single)?;
    world.active_body_poses_into(&mut batch);
    assert_eq!(batch.len(), BODIES);
    let bits = |id: BodyId, p: RVec3, q: Quat| {
        let position = [p.x, p.y, p.z].map(|c| c.to_bits().to_le_bytes().to_vec());
        (id, position, [q.x, q.y, q.z, q.w].map(f32::to_bits))
    };
    let single_bits: Vec<_> = single.iter().map(|&(id, p, q)| bits(id, p, q)).collect();
    let batch_bits: Vec<_> = batch
        .iter()
        .map(|pose| bits(pose.id, pose.position, pose.rotation))
        .collect();
    // `ids` are in creation order, which is ascending id order in this fresh world.
    assert_eq!(single_bits, batch_bits);

    let mut per_body_us = Vec::with_capacity(TICKS);
    let mut batch_us = Vec::with_capacity(TICKS);
    for tick in 0..TICKS {
        assert!(world.step(DT)?.is_complete());
        let mut time_per_body = || -> Result<(), BodyError> {
            let start = Instant::now();
            per_body(&world, &ids, &mut single)?;
            black_box(&single);
            per_body_us.push(micros(start));
            Ok(())
        };
        if tick % 2 == 0 {
            time_per_body()?;
        }
        let start = Instant::now();
        world.active_body_poses_into(&mut batch);
        black_box(&batch);
        batch_us.push(micros(start));
        if tick % 2 == 1 {
            time_per_body()?;
        }
    }
    Ok([
        Row::new(
            "pose readout, 256 awake bodies, per body",
            "one readout",
            per_body_us,
            Limit::None,
            "256 bodies per sample",
        ),
        Row::new(
            "pose readout, 256 awake bodies, batch",
            "one readout",
            batch_us,
            Limit::None,
            "256 bodies per sample",
        ),
    ])
}
