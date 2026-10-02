//! Measures what one character costs per tick with heightfield terrain in the query, the number
//! the game budgets per actor: 3 x 3 chunks, each a 33 x 33 heightfield and a compound of about 20
//! sharp boxes and cylinders, and 30 characters walking for 600 ticks. Prints the median and 99th
//! percentile of one `update_character` call, with the terrain in the filter and without it, and of
//! one whole near step of the game's reference controller (`tests/common/walker.rs`: underground
//! probe, pose setters, the update, the contact readout, the autostep and the actor capsule sync)
//! on chunks laid on the planet.
//!
//! Run with `cargo run --release -p joltphysics --example character_cost`. The timings are
//! wall-clock and only reported; nothing feeds back into the simulation.

#[path = "../tests/common/mod.rs"]
mod common;

use std::error::Error;
use std::time::Instant;

use joltphysics::*;

const CHUNK_SIDE: f32 = 32.0;
const CHARACTERS: usize = 30;
const TICKS: usize = 600;
const DT: f32 = 1.0 / 60.0;
const GRAVITY: Vec3 = Vec3::new(0.0, -9.8, 0.0);

/// Terrain, chunk compounds and nothing else: the characters' filter layers.
struct Layers {
    terrain: ObjectLayer,
    chunk: ObjectLayer,
}

/// A gently rolling 33 x 33 heightfield, different per chunk.
fn terrain(cx: i32, cz: i32) -> Result<Shape, ShapeError> {
    let mut samples = vec![0.0_f32; 33 * 33];
    for z in 0..33 {
        for x in 0..33 {
            let (wx, wz) = (
                (cx * 32 + x as i32) as f32 * 0.21,
                (cz * 32 + z as i32) as f32 * 0.17,
            );
            samples[z * 33 + x] = 0.4 * wx.sin() + 0.3 * wz.cos();
        }
    }
    let settings = HeightFieldSettings::default()
        .offset(Vec3::new(-CHUNK_SIDE / 2.0, 0.0, -CHUNK_SIDE / 2.0))
        .scale(Vec3::new(1.0, 1.0, 1.0));
    Shape::new_height_field(33, &samples, &settings)
}

/// About 20 boxes and cylinders scattered over a chunk, with group user data.
fn chunk_compound(seed: u32) -> Result<Shape, ShapeError> {
    let block = Shape::new_box_with_convex_radius(Vec3::new(0.8, 1.0, 0.6), 0.0)?;
    let pillar = Shape::new_cylinder(1.5, 0.4)?;
    let mut state = seed.wrapping_mul(747_796_405).wrapping_add(2_891_336_453);
    let mut next = move || {
        state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        (state >> 8) as f32 / (1 << 24) as f32
    };
    let children: Vec<CompoundChild<'_>> = (0..20)
        .map(|i| {
            let position = Vec3::new(
                (next() - 0.5) * (CHUNK_SIDE - 4.0),
                1.0,
                (next() - 0.5) * (CHUNK_SIDE - 4.0),
            );
            let (shape, group) = if i % 2 == 0 {
                (&block, 2)
            } else {
                (&pillar, 3)
            };
            CompoundChild {
                shape,
                position,
                rotation: Quat::IDENTITY,
                user_data: group,
            }
        })
        .collect();
    Shape::new_compound(&children)
}

fn build_world() -> Result<(PhysicsWorld, Layers, Vec<CharacterId>), Box<dyn Error>> {
    let mut collision = CollisionLayers::new(1);
    let fixed = BroadPhaseLayer::new(0);
    let layers = Layers {
        terrain: collision.add_object_layer(fixed),
        chunk: collision.add_object_layer(fixed),
    };
    let mut world = PhysicsWorld::new(
        WorldSettings::default()
            .gravity(Vec3::ZERO)
            .layers(collision),
    )?;
    for cx in -1..=1 {
        for cz in -1..=1 {
            let position = RVec3::new(
                cx as Real * CHUNK_SIDE as Real,
                0.0,
                cz as Real * CHUNK_SIDE as Real,
            );
            for (shape, layer) in [
                (terrain(cx, cz)?, layers.terrain),
                (chunk_compound((cx * 3 + cz + 4) as u32)?, layers.chunk),
            ] {
                world.create_body(
                    &shape,
                    &BodySettings::new_static()
                        .position(position)
                        .object_layer(layer),
                )?;
            }
        }
    }
    world.optimize_broad_phase();

    let capsule = Shape::new_capsule(0.70845, 0.4)?;
    let settings = CharacterSettings::new(&capsule)
        .shape_offset(Vec3::new(0.0, 0.70844734, 0.0))
        .max_slope_angle(45.0_f32.to_radians())
        .enhanced_internal_edge_removal(true);
    let mut characters = Vec::with_capacity(CHARACTERS);
    for i in 0..CHARACTERS {
        let angle = i as f32 / CHARACTERS as f32 * std::f32::consts::TAU;
        let radius = 10.0 + (i % 5) as f32 * 7.0;
        let position = RVec3::new(
            (radius * angle.cos()) as Real,
            2.0,
            (radius * angle.sin()) as Real,
        );
        characters.push(world.create_character(&settings, position, Quat::IDENTITY)?);
    }
    Ok((world, layers, characters))
}

/// The duration of every `update_character` call of the run in microseconds, sorted, and the
/// share of updates after which the character stood on something.
fn run(with_terrain: bool) -> Result<(Vec<f64>, f64), Box<dyn Error>> {
    let (mut world, layers, characters) = build_world()?;
    let all = [layers.terrain, layers.chunk];
    let chunks_only = [layers.chunk];
    let filter = QueryFilter::new().object_layers(if with_terrain { &all } else { &chunks_only });
    let extended = ExtendedUpdateSettings::default()
        .stick_to_floor_step_down(Vec3::new(0.0, -0.3, 0.0))
        .walk_stairs_step_up(Vec3::ZERO);
    let mut durations = Vec::with_capacity(TICKS * CHARACTERS);
    let mut supported_updates = 0;
    for _ in 0..TICKS {
        for (i, &id) in characters.iter().enumerate() {
            let character = world.character(id)?;
            let p = character.position();
            let falling = character.linear_velocity().y.min(0.0);
            let supported = character.ground_state().is_supported();
            // Walk a circle around the world origin at 2 m/s, alternating directions.
            let (x, z) = horizontal(p);
            let length = (x * x + z * z).sqrt().max(1.0);
            let sign = if i % 2 == 0 { 1.0 } else { -1.0 };
            let vertical = if supported {
                -9.8 * DT
            } else {
                falling - 9.8 * DT
            };
            let velocity = Vec3::new(-z / length * 2.0 * sign, vertical, x / length * 2.0 * sign);
            world.character_mut(id)?.set_linear_velocity(velocity)?;
            let start = Instant::now();
            world.update_character(id, DT, GRAVITY, &extended, &filter)?;
            durations.push(start.elapsed().as_secs_f64() * 1e6);
            if world.character(id)?.ground_state().is_supported() {
                supported_updates += 1;
            }
        }
    }
    durations.sort_by(f64::total_cmp);
    let supported = f64::from(supported_updates) / durations.len() as f64;
    Ok((durations, supported))
}

/// The x and z of a position as `f32`.
// `Real` is already `f32` without the `double-precision` feature.
#[allow(clippy::unnecessary_cast)]
fn horizontal(p: RVec3) -> (f32, f32) {
    (p.x as f32, p.z as f32)
}

fn percentile(sorted: &[f64], fraction: f64) -> f64 {
    sorted[((sorted.len() - 1) as f64 * fraction).round() as usize]
}

/// The near-step run: chunks laid on the planet of the walker fixture around the anchor, each a
/// heightfield and a compound of sharp boxes (structures) and cylinders (features) standing on it,
/// and walkers turning slowly as they walk at 2 m/s. Returns the duration of every near step in
/// microseconds, sorted, and the share of steps after which the walker was grounded.
fn run_near_steps() -> Result<(Vec<f64>, f64), Box<dyn Error>> {
    use common::walker::*;
    use common::Groups;

    let (mut world, layers) = fixture_world(1);
    let block = Shape::new_box_with_convex_radius(Vec3::new(0.8, 1.0, 0.6), 0.0)?;
    let pillar = Shape::new_cylinder(1.5, 0.4)?;
    let spacing = 2.0 * (f64::from(CHUNK_SIDE) / 2.0 / R).asin();
    for i in -1..=1 {
        for k in -1..=1 {
            let up = normalize([
                (f64::from(i) * spacing).tan(),
                1.0,
                (f64::from(k) * spacing).tan(),
            ]);
            let pose = (rvec3(add(CENTRE, scale(up, R))), from_y_to(up));
            add_terrain(&mut world, &layers, &flat_terrain(), pose);
            let mut state = ((i + 1) * 3 + k + 1) as u32 * 2_654_435_761;
            let mut next = move || {
                state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                f64::from(state >> 8) / f64::from(1 << 24) - 0.5
            };
            let children: Vec<CompoundChild<'_>> = (0..20)
                .map(|n| {
                    let (x, z) = (next() * 28.0, next() * 28.0);
                    let ground = (R * R - x * x - z * z).sqrt() - R;
                    let (shape, half, group) = if n % 2 == 0 {
                        (&block, 1.0, Groups::STRUCTURE)
                    } else {
                        (&pillar, 1.5, Groups::FEATURE)
                    };
                    CompoundChild {
                        shape,
                        position: vec3([x, ground + half, z]),
                        rotation: Quat::IDENTITY,
                        user_data: group,
                    }
                })
                .collect();
            world.create_body(
                &Shape::new_compound(&children)?,
                &BodySettings::new_static()
                    .position(pose.0)
                    .rotation(pose.1)
                    .object_layer(layers.chunk),
            )?;
        }
    }
    world.optimize_broad_phase();

    let walkers: Vec<Walker> = (0..CHARACTERS)
        .map(|n| {
            let angle = n as f64 / CHARACTERS as f64 * std::f64::consts::TAU;
            let radius = 6.0 + (n % 5) as f64 * 7.0;
            let (x, z) = (radius * angle.cos(), radius * angle.sin());
            let ground = [x, (R * R - x * x - z * z).sqrt() - R, z];
            let origin = add(ground, scale(up_at(ground), f64::from(REST_HEIGHT)));
            add_walker(&mut world, &layers, origin)
        })
        .collect();
    let mut carries = vec![Carry::RESTING; CHARACTERS];
    let mut durations = Vec::with_capacity(TICKS * CHARACTERS);
    let mut grounded_steps = 0;
    for tick in 0..TICKS {
        for (n, (walker, carry)) in walkers.iter().zip(&mut carries).enumerate() {
            let heading = n as f64 + tick as f64 * 0.01;
            let direction = [heading.cos(), 0.0, heading.sin()];
            let wanted = tangent(origin(&world, walker), direction, 2.0 * f64::from(DT));
            let start = Instant::now();
            let out = near_tick(&mut world, walker, carry, wanted);
            durations.push(start.elapsed().as_secs_f64() * 1e6);
            if out.grounded {
                grounded_steps += 1;
            }
        }
    }
    durations.sort_by(f64::total_cmp);
    let grounded = f64::from(grounded_steps) / durations.len() as f64;
    Ok((durations, grounded))
}

fn main() -> Result<(), Box<dyn Error>> {
    let profile = if cfg!(debug_assertions) {
        "debug"
    } else {
        "release"
    };
    let threads = std::thread::available_parallelism().map_or(0, |n| n.get());
    println!(
        "{} {} with {threads} hardware threads, {profile} build of the Rust code (Jolt is always \
         built in Release)",
        std::env::consts::OS,
        std::env::consts::ARCH
    );
    for with_terrain in [true, false] {
        let (durations, supported) = run(with_terrain)?;
        println!(
            "update_character, terrain in the filter: {with_terrain:5}  {} updates  median {:.1} us  \
             p99 {:.1} us  standing after {:.0}% of them",
            durations.len(),
            percentile(&durations, 0.5),
            percentile(&durations, 0.99),
            supported * 100.0
        );
    }
    let (durations, grounded) = run_near_steps()?;
    println!(
        "near step, terrain in the filter: true  {} steps  median {:.1} us  p99 {:.1} us  \
         grounded after {:.0}% of them",
        durations.len(),
        percentile(&durations, 0.5),
        percentile(&durations, 0.99),
        grounded * 100.0
    );
    Ok(())
}
