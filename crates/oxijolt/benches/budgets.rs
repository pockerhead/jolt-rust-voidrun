//! Measures the physics work of one game tick against the game's budgets and prints one markdown
//! table. The cases:
//!
//! - `update_character` on a flat 3 x 3 scene (each chunk a 33 x 33 heightfield and a compound
//!   of about 20 sharp boxes and cylinders), 30 characters, with the terrain in the filter and
//!   without it. One sample is one call.
//! - The near step of the game's reference controller (`tests/common/walker.rs`: underground
//!   probe, pose setters, the update, the contact readout, the autostep and the actor capsule
//!   sync) for 30 walkers on 3 x 3 chunks laid on the planet of radius 99. One sample is one
//!   walker's step.
//! - A chunk landing next to that grid: building its shapes, inserting its terrain and compound
//!   bodies, and the broad-phase optimisation after it. One sample is one landing.
//! - A steady world step with 8 active items, each replaced once it has come to rest, on 1 and on
//!   4 worker threads. One sample is one `step`.
//! - Rays of two kinds: spawn ground (down onto terrain and structures) and line of sight
//!   (sideways at eye height against structures and features). One sample is one ray.
//! - A tick of 30 near steps and 60 structure-top rays for the mid-band actors, on a world with
//!   4 worker threads, once with the mid actors walking rings over mostly open ground and once
//!   with every mid actor on a structure top. One sample is one tick.
//! - A step of an awake pile of 64 cubes on 4 worker threads, recording no events and recording
//!   every event (draining them after each step). One sample is one `step` and `take_events`.
//! - The poses of 256 awake cubes floating above the planet scene, read once per tick one body
//!   at a time (`world.body(id)`) and with `active_body_poses_into`. One sample is one readout of
//!   all 256.
//!
//! Every case runs a warm-up that is not reported. The first call after the scene is built is
//! shown as its own "cold first call" row; the landing reports it for the insertion and the rays
//! for the first (spawn ground) ray. Percentiles use the nearest rank: the p-th percentile of n
//! sorted samples is sample `ceil(p * n)` (counting from 1). Queries see bodies as soon as they
//! are created, so a per-tick broad-phase refresh costs nothing here.
//!
//! Run with `cargo bench -p oxijolt --bench budgets`; words after `--` run only the cases
//! whose names contain one of them (`update_character`, `near step`, `landing`, `steady step`,
//! `ray`, `tick`, `events`, `poses`). The timings are wall-clock and only reported, never checked; nothing timed
//! feeds back into the simulation.

#[path = "../tests/common/mod.rs"]
mod common;
#[path = "budgets/planet.rs"]
mod planet;
#[path = "budgets/poses.rs"]
mod poses;
#[path = "budgets/report.rs"]
mod report;

use std::error::Error;
use std::time::Instant;

use common::math::{add, rvec3, scale, v3, vec3};
use common::walker::{
    flat_terrain, from_y_to, near_tick, tangent, up_at, Carry, Layers as PlanetLayers,
};
use common::{is_calm, Groups};
use oxijolt::*;
use planet::{
    chunk_body, chunk_parts, ground_at, planet_chunk_compound, planet_chunk_pose, planet_scene,
    planet_walkers, wanted, CHARACTERS, CHUNK_SIDE,
};
use report::{micros, print_machine, print_table, Limit, Row};

const DT: f32 = 1.0 / 60.0;
const GRAVITY: Vec3 = Vec3::new(0.0, -9.8, 0.0);

/// Untimed ticks before the controller, near-step, steady-step and tick cases measure.
const WARMUP_TICKS: usize = 300;
/// Measured ticks of the controller, near-step, steady-step and tick cases.
const TICKS: usize = 5_000;
/// Untimed landings before the landing case measures.
const LANDING_WARMUP: usize = 50;
/// Measured landings.
const LANDINGS: usize = 2_000;
/// Untimed rays before the ray case measures.
const RAY_WARMUP: usize = 1_000;
/// Measured rays.
const RAYS: usize = 10_000;
/// Items in flight in the steady step.
const ITEMS: usize = 8;
/// Mid-band actors that query the structure top under them every tick.
const MID_ACTORS: usize = 60;

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

/// A small deterministic generator of numbers in `[0, 1)`.
struct Lcg(u32);

impl Lcg {
    fn next(&mut self) -> f64 {
        self.0 = self.0.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        f64::from(self.0 >> 8) / f64::from(1 << 24)
    }
}

const RAPIER: &str = "Rapier 86-156 us per character with terrain";

/// `update_character` on the flat 3 x 3 scene, with the terrain in the filter or without it.
fn run(with_terrain: bool) -> Result<[Row; 2], Box<dyn Error>> {
    let (mut world, layers, characters) = build_world()?;
    let all = [layers.terrain, layers.chunk];
    let chunks_only = [layers.chunk];
    let filter = QueryFilter::new().object_layers(if with_terrain { &all } else { &chunks_only });
    let extended = ExtendedUpdateSettings::default()
        .stick_to_floor_step_down(Vec3::new(0.0, -0.3, 0.0))
        .walk_stairs_step_up(Vec3::ZERO);
    let mut durations = Vec::with_capacity(TICKS * CHARACTERS);
    let mut cold = 0.0;
    let mut supported_updates = 0;
    for tick in 0..WARMUP_TICKS + TICKS {
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
            let us = micros(start);
            if tick == 0 && i == 0 {
                cold = us;
            }
            if tick >= WARMUP_TICKS {
                durations.push(us);
                if world.character(id)?.ground_state().is_supported() {
                    supported_updates += 1;
                }
            }
        }
    }
    let supported = f64::from(supported_updates) / durations.len() as f64;
    let (case, limit) = if with_terrain {
        (
            "update_character, terrain in filter: yes (flat 3 x 3 scene)",
            Limit::Reference(RAPIER),
        )
    } else {
        // Without terrain the characters stand on nothing: not comparable with the reference.
        (
            "update_character, terrain in filter: no (flat 3 x 3 scene)",
            Limit::None,
        )
    };
    Ok([
        Row::cold(case, "one call", cold),
        Row::new(
            case,
            "one call",
            durations,
            limit,
            format!("supported after {:.0}% of calls", supported * 100.0),
        ),
    ])
}

/// The x and z of a position as `f32`.
// `Real` is already `f32` without the `double-precision` feature.
#[allow(clippy::unnecessary_cast)]
fn horizontal(p: RVec3) -> (f32, f32) {
    (p.x as f32, p.z as f32)
}

/// The near step of 30 walkers on the planet chunks, one sample per walker step.
fn run_near_steps() -> Result<[Row; 2], Box<dyn Error>> {
    let (mut world, layers) = planet_scene(1)?;
    let walkers = planet_walkers(&mut world, &layers);
    let mut carries = vec![Carry::RESTING; CHARACTERS];
    let mut durations = Vec::with_capacity(TICKS * CHARACTERS);
    let mut cold = 0.0;
    let mut grounded_steps = 0;
    for tick in 0..WARMUP_TICKS + TICKS {
        for (n, (walker, carry)) in walkers.iter().zip(&mut carries).enumerate() {
            let desired = wanted(&world, walker, n, tick);
            let start = Instant::now();
            let out = near_tick(&mut world, walker, carry, desired);
            let us = micros(start);
            if tick == 0 && n == 0 {
                cold = us;
            }
            if tick >= WARMUP_TICKS {
                durations.push(us);
                if out.grounded {
                    grounded_steps += 1;
                }
            }
        }
    }
    let grounded = f64::from(grounded_steps) / durations.len() as f64;
    let case = "near step (radial planet)";
    Ok([
        Row::cold(case, "one walker step", cold),
        Row::new(
            case,
            "one walker step",
            durations,
            Limit::Reference(RAPIER),
            format!("grounded after {:.0}% of steps", grounded * 100.0),
        ),
    ])
}

/// A chunk landing next to the planet grid, at chunk `(2, 0)`: shape build, body insertion and
/// the broad-phase optimisation after it, each timed; the chunk is removed again, untimed, so
/// every landing starts from the same world.
fn run_landings() -> Result<Vec<Row>, Box<dyn Error>> {
    let (mut world, layers) = planet_scene(1)?;
    let (block, pillar) = chunk_parts()?;
    let pose = planet_chunk_pose(2, 0);
    let terrain_body = chunk_body(pose, layers.terrain);
    let compound_body = chunk_body(pose, layers.chunk);
    let mut build = Vec::with_capacity(LANDINGS);
    let mut insert = Vec::with_capacity(LANDINGS);
    let mut total = Vec::with_capacity(LANDINGS);
    let mut optimize = Vec::with_capacity(LANDINGS);
    let mut cold = 0.0;
    for cycle in 0..LANDING_WARMUP + LANDINGS {
        let start = Instant::now();
        let terrain_shape = flat_terrain();
        let compound_shape = planet_chunk_compound(2, 0, &block, &pillar)?;
        let build_us = micros(start);

        let start = Instant::now();
        let terrain_id = world.create_body(&terrain_shape, &terrain_body)?;
        let compound_id = world.create_body(&compound_shape, &compound_body)?;
        let insert_us = micros(start);

        let start = Instant::now();
        world.optimize_broad_phase();
        let optimize_us = micros(start);

        // Jolt frees broad-phase nodes only in a step or an optimisation, so every cycle ends
        // with one; it also gives every landing the same starting world.
        world.remove_body(terrain_id)?;
        world.remove_body(compound_id)?;
        world.optimize_broad_phase();

        if cycle == 0 {
            cold = insert_us;
        }
        if cycle >= LANDING_WARMUP {
            build.push(build_us);
            insert.push(insert_us);
            total.push(build_us + insert_us);
            optimize.push(optimize_us);
        }
    }
    Ok(vec![
        Row::new(
            "chunk shape build (off the sim thread in the game)",
            "one chunk",
            build,
            Limit::None,
            "33 x 33 heightfield + 20-child compound",
        ),
        Row::cold(
            "landing: insert terrain + compound bodies",
            "one landing",
            cold,
        ),
        Row::new(
            "landing: insert terrain + compound bodies",
            "one landing",
            insert,
            Limit::AtMost(2000.0),
            "two create_body calls",
        ),
        Row::new(
            "landing incl. shape build",
            "one landing",
            total,
            Limit::AtMost(2000.0),
            "the game builds shapes off the sim thread",
        ),
        Row::new(
            "broad-phase optimize after a landing",
            "one call",
            optimize,
            Limit::None,
            "not needed for queries; the explicit refresh point",
        ),
    ])
}

/// An item in flight in the steady step.
struct Item {
    id: BodyId,
    calm_ticks: usize,
    age: usize,
}

/// Mass of an item, kilograms.
const ITEM_MASS: f32 = 1.2;

/// Drops an item with the game's settings 1-3 m above the ground at a spot inside the centre
/// chunk.
fn drop_item(
    world: &mut PhysicsWorld,
    layers: &PlanetLayers,
    shape: &Shape,
    spots: &mut Lcg,
) -> Result<Item, Box<dyn Error>> {
    let ground = ground_at((spots.next() - 0.5) * 24.0, (spots.next() - 0.5) * 24.0);
    let up = up_at(ground);
    let position = add(ground, scale(up, 1.0 + 2.0 * spots.next()));
    let id = world.create_body(
        shape,
        &BodySettings::new_dynamic()
            .position(rvec3(position))
            .rotation(from_y_to(up))
            .mass(ITEM_MASS)
            .friction(0.8)
            .restitution(0.1)
            .motion_quality(MotionQuality::LinearCast)
            .object_layer(layers.item),
    )?;
    Ok(Item {
        id,
        calm_ticks: 0,
        age: 0,
    })
}

/// A steady world step with 8 items under radial gravity on `worker_threads` threads. An item
/// that has been calm for 30 ticks, or has flown for 600, is replaced by a new one dropped from
/// above the ground.
fn run_steady_steps(worker_threads: u32) -> Result<[Row; 2], Box<dyn Error>> {
    let (mut world, layers) = planet_scene(worker_threads)?;
    let shape = Shape::new_box(Vec3::new(0.35, 0.06, 0.04))?;
    let mut spots = Lcg(7);
    let mut items = Vec::with_capacity(ITEMS);
    for _ in 0..ITEMS {
        items.push(drop_item(&mut world, &layers, &shape, &mut spots)?);
    }
    let mut durations = Vec::with_capacity(TICKS);
    let mut cold = 0.0;
    let mut awake_items = 0;
    let mut replacements = 0;
    for tick in 0..WARMUP_TICKS + TICKS {
        for item in &items {
            let mut body = world.body_mut(item.id)?;
            let up = up_at(v3(body.position()));
            body.reset_forces();
            body.add_force(vec3(scale(up, -f64::from(ITEM_MASS * common::walker::G))))?;
        }
        let start = Instant::now();
        let report = world.step(DT)?;
        let us = micros(start);
        assert!(report.is_complete());
        if tick == 0 {
            cold = us;
        }
        let measured = tick >= WARMUP_TICKS;
        if measured {
            durations.push(us);
        }
        for item in &mut items {
            let body = world.body(item.id)?;
            if measured && !body.is_sleeping() {
                awake_items += 1;
            }
            item.calm_ticks = if is_calm(&body) {
                item.calm_ticks + 1
            } else {
                0
            };
            item.age += 1;
            if item.calm_ticks >= 30 || item.age >= 600 {
                world.remove_body(item.id)?;
                *item = drop_item(&mut world, &layers, &shape, &mut spots)?;
                if measured {
                    replacements += 1;
                }
            }
        }
    }
    let case = if worker_threads == 1 {
        "steady step, 8 items, 1 worker thread"
    } else {
        "steady step, 8 items, 4 worker threads"
    };
    Ok([
        Row::cold(case, "one step", cold),
        Row::new(
            case,
            "one step",
            durations,
            Limit::AtMost(1000.0),
            format!(
                "{:.1} awake items per tick on average; {replacements} items replaced",
                f64::from(awake_items) / TICKS as f64
            ),
        ),
    ])
}

/// What a ray hit, for the notes.
#[derive(Default)]
struct HitCounts {
    terrain: usize,
    structure: usize,
    feature: usize,
    miss: usize,
}

impl HitCounts {
    fn count(&mut self, hit: Option<&RayHit>, terrain: ObjectLayer) {
        match hit {
            None => self.miss += 1,
            Some(hit) if hit.object_layer == terrain => self.terrain += 1,
            Some(hit) => match hit.compound_child.map(|child| child.user_data) {
                Some(Groups::STRUCTURE) => self.structure += 1,
                _ => self.feature += 1,
            },
        }
    }

    fn describe(&self) -> String {
        format!(
            "terrain {}, structure {}, feature {}, miss {}",
            self.terrain, self.structure, self.feature, self.miss
        )
    }
}

/// Spawn-ground and line-of-sight rays over the planet chunks, alternating, from ground points
/// within 40 m of the anchor.
fn run_rays() -> Result<[Row; 4], Box<dyn Error>> {
    let (world, layers) = planet_scene(1)?;
    let ground_layers = [layers.terrain, layers.chunk];
    let spawn = QueryFilter::new()
        .object_layers(&ground_layers)
        .child_groups(1 << Groups::STRUCTURE);
    let sight = QueryFilter::new()
        .object_layers(&ground_layers)
        .child_groups(1 << Groups::STRUCTURE | 1 << Groups::FEATURE);
    let mut picks = Lcg(11);
    let mut all = Vec::with_capacity(RAYS);
    let mut spawn_us = Vec::with_capacity(RAYS / 2);
    let mut sight_us = Vec::with_capacity(RAYS / 2);
    let (mut spawn_hits, mut sight_hits) = (HitCounts::default(), HitCounts::default());
    let mut cold = 0.0;
    for n in 0..RAY_WARMUP + RAYS {
        let angle = picks.next() * std::f64::consts::TAU;
        let radius = 40.0 * picks.next().sqrt();
        let ground = ground_at(radius * angle.cos(), radius * angle.sin());
        let up = up_at(ground);
        let is_spawn = n % 2 == 0;
        let (ray, filter) = if is_spawn {
            let origin = add(ground, scale(up, 5.0));
            (RayCast::new(rvec3(origin), vec3(scale(up, -55.0))), &spawn)
        } else {
            let origin = add(ground, scale(up, 1.6));
            let heading = picks.next() * std::f64::consts::TAU;
            let length = 10.0 + 30.0 * picks.next();
            let direction = tangent(origin, [heading.cos(), 0.0, heading.sin()], length);
            (RayCast::new(rvec3(origin), vec3(direction)), &sight)
        };
        let start = Instant::now();
        let hit = world.cast_ray(ray, filter)?;
        let us = micros(start);
        if n == 0 {
            cold = us;
        }
        if n >= RAY_WARMUP {
            all.push(us);
            if is_spawn {
                spawn_us.push(us);
                spawn_hits.count(hit.as_ref(), layers.terrain);
            } else {
                sight_us.push(us);
                sight_hits.count(hit.as_ref(), layers.terrain);
            }
        }
    }
    Ok([
        Row::cold("ray", "one ray", cold),
        Row::new(
            "ray (all)",
            "one ray",
            all,
            Limit::P99AtMost(50.0),
            "spawn ground and line of sight alternating",
        ),
        Row::new(
            "ray: spawn ground",
            "one ray",
            spawn_us,
            Limit::P99AtMost(50.0),
            format!(
                "55 m down, terrain and structure children; {}",
                spawn_hits.describe()
            ),
        ),
        Row::new(
            "ray: line of sight",
            "one ray",
            sight_us,
            Limit::P99AtMost(50.0),
            format!(
                "10-40 m at eye height, structures and features; {}",
                sight_hits.describe()
            ),
        ),
    ])
}

/// Where the mid-band actors of the tick case stand.
#[derive(Clone, Copy)]
enum MidBand {
    /// Walking rings 20-45 m from the anchor at 2 m/s, over mostly open ground.
    Rings,
    /// Each on a structure top, on another one every tick.
    OnStructures,
}

/// The mid band's filter: chunk compounds, structure children only.
fn structures_only(chunk_layer: &[ObjectLayer; 1]) -> QueryFilter<'_> {
    QueryFilter::new()
        .object_layers(chunk_layer)
        .child_groups(1 << Groups::STRUCTURE)
}

/// The mid band's ray under an actor at the ground point `ground`: 55 m down from 5 m above it.
fn mid_ray(ground: [f64; 3]) -> RayCast {
    let up = up_at(ground);
    RayCast::new(rvec3(add(ground, scale(up, 5.0))), vec3(scale(up, -55.0)))
}

/// The ground points of a 1 m grid within 45 m of the anchor that lie under a structure top.
fn structure_spots(
    world: &PhysicsWorld,
    chunk_layer: &[ObjectLayer; 1],
) -> Result<Vec<[f64; 3]>, Box<dyn Error>> {
    let filter = structures_only(chunk_layer);
    let mut spots = Vec::new();
    for x in -45..=45 {
        for z in -45..=45 {
            let (x, z) = (f64::from(x), f64::from(z));
            let ground = ground_at(x, z);
            if x * x + z * z <= 45.0 * 45.0 && world.cast_ray(mid_ray(ground), &filter)?.is_some() {
                spots.push(ground);
            }
        }
    }
    Ok(spots)
}

/// Where mid actor `n` stands at `tick`.
fn mid_position(mid: MidBand, spots: &[[f64; 3]], n: usize, tick: usize) -> [f64; 3] {
    match mid {
        MidBand::Rings => {
            let radius = 20.0 + (n % 6) as f64 * 5.0;
            let sign = if n.is_multiple_of(2) { 1.0 } else { -1.0 };
            let walked = sign * tick as f64 * 2.0 * f64::from(DT) / radius;
            let angle = n as f64 / MID_ACTORS as f64 * std::f64::consts::TAU + 0.1 + walked;
            ground_at(radius * angle.cos(), radius * angle.sin())
        }
        MidBand::OnStructures => spots[(n * 7 + tick) % spots.len()],
    }
}

/// One tick of the game's near and mid bands on a world with 4 worker threads: 30 near steps,
/// then one structure-top ray under each of 60 mid-band actors (the mid band has no physics
/// solve and queries structures only; its actors are positions here, not bodies). The actors
/// move every tick, so every tick casts different rays.
fn run_ticks(mid: MidBand) -> Result<[Row; 2], Box<dyn Error>> {
    let (mut world, layers) = planet_scene(4)?;
    let walkers = planet_walkers(&mut world, &layers);
    let mut carries = vec![Carry::RESTING; CHARACTERS];
    let chunk_layer = [layers.chunk];
    let structures = structures_only(&chunk_layer);
    let spots = structure_spots(&world, &chunk_layer)?;
    assert!(!spots.is_empty(), "no structure tops in the scene");
    let mut desired = [[0.0; 3]; CHARACTERS];
    let mut rays = Vec::with_capacity(MID_ACTORS);
    let mut grounded = [false; CHARACTERS];
    let mut tops = [false; MID_ACTORS];
    let mut durations = Vec::with_capacity(TICKS);
    let mut cold = 0.0;
    let (mut grounded_steps, mut top_hits) = (0, 0);
    for tick in 0..WARMUP_TICKS + TICKS {
        for (n, walker) in walkers.iter().enumerate() {
            desired[n] = wanted(&world, walker, n, tick);
        }
        rays.clear();
        rays.extend((0..MID_ACTORS).map(|n| mid_ray(mid_position(mid, &spots, n, tick))));
        let start = Instant::now();
        for (n, (walker, carry)) in walkers.iter().zip(&mut carries).enumerate() {
            grounded[n] = near_tick(&mut world, walker, carry, desired[n]).grounded;
        }
        for (top, &ray) in tops.iter_mut().zip(&rays) {
            *top = world.cast_ray(ray, &structures)?.is_some();
        }
        let us = micros(start);
        if tick == 0 {
            cold = us;
        }
        if tick >= WARMUP_TICKS {
            durations.push(us);
            grounded_steps += grounded.iter().filter(|&&g| g).count();
            top_hits += tops.iter().filter(|&&t| t).count();
        }
    }
    let case = match mid {
        MidBand::Rings => "tick: 30 near steps + 60 mid-band rays, mid actors walking rings",
        MidBand::OnStructures => {
            "tick: 30 near steps + 60 mid-band rays, every mid actor on a structure top"
        }
    };
    Ok([
        Row::cold(case, "one tick", cold),
        Row::new(
            case,
            "one tick",
            durations,
            Limit::P99AtMost(2000.0),
            format!(
                "grounded after {:.0}% of steps; {:.0}% of mid rays hit a structure top; the \
                 walker update and queries run on the calling thread, the world's 4 worker \
                 threads take no part; refresh is zero work (queries see bodies immediately)",
                grounded_steps as f64 / (TICKS * CHARACTERS) as f64 * 100.0,
                top_hits as f64 / (TICKS * MID_ACTORS) as f64 * 100.0
            ),
        ),
    ])
}

/// Steps an awake pile of 64 cubes on 4 worker threads, with every event recorded or none, and
/// takes the events after each step.
fn run_event_pile(record: bool) -> Result<Row, Box<dyn Error>> {
    let mut world = PhysicsWorld::new(WorldSettings::default().worker_threads(4).gravity(GRAVITY))?;
    if record {
        let every = EventSettings::default()
            .persisted_contacts(true)
            .body_activation(true)
            .soft_body_contacts(true)
            .soft_body_validations(true);
        world.set_event_settings(every);
    }
    let floor = Shape::new_box(Vec3::new(10.0, 1.0, 10.0))?;
    world.create_body(
        &floor,
        &BodySettings::new_static().position(RVec3::new(0.0, -1.0, 0.0)),
    )?;
    let cube = Shape::new_box(Vec3::new(0.25, 0.25, 0.25))?;
    for i in 0..64 {
        let (x, y, z) = ((i % 4) as Real, (i / 16) as Real, ((i / 4) % 4) as Real);
        let position = RVec3::new(0.6 * x + 0.1 * y, 0.3 + 0.55 * y, 0.6 * z);
        world.create_body(
            &cube,
            &BodySettings::new_dynamic()
                .position(position)
                .allow_sleeping(false),
        )?;
    }
    let mut durations = Vec::with_capacity(TICKS);
    let mut events = 0;
    for tick in 0..WARMUP_TICKS + TICKS {
        let start = Instant::now();
        assert!(world.step(DT)?.is_complete());
        let taken = world.take_events();
        let us = micros(start);
        if tick >= WARMUP_TICKS {
            durations.push(us);
            events += taken.contacts.len() + taken.activations.len();
        }
    }
    let case = if record {
        "events pile, every event, 4 worker threads"
    } else {
        "events pile, no events, 4 worker threads"
    };
    Ok(Row::new(
        case,
        "one step and take",
        durations,
        Limit::None,
        format!("{:.1} events per step", events as f64 / TICKS as f64),
    ))
}

fn main() -> Result<(), Box<dyn Error>> {
    // `cargo test --benches` runs this binary without `--bench`; the full run would take minutes
    // in a debug build.
    if !std::env::args().any(|arg| arg == "--bench") {
        println!("run with `cargo bench -p oxijolt --bench budgets`");
        return Ok(());
    }
    let filters: Vec<String> = std::env::args()
        .skip(1)
        .filter(|arg| !arg.starts_with('-'))
        .collect();
    let selected =
        |case: &str| filters.is_empty() || filters.iter().any(|f| case.contains(f.as_str()));
    let mut rows = Vec::new();
    if selected("update_character") {
        for with_terrain in [true, false] {
            rows.extend(run(with_terrain)?);
        }
    }
    if selected("near step") {
        rows.extend(run_near_steps()?);
    }
    if selected("landing") {
        rows.extend(run_landings()?);
    }
    if selected("steady step") {
        for worker_threads in [1, 4] {
            rows.extend(run_steady_steps(worker_threads)?);
        }
    }
    if selected("ray") {
        rows.extend(run_rays()?);
    }
    if selected("tick") {
        for mid in [MidBand::Rings, MidBand::OnStructures] {
            rows.extend(run_ticks(mid)?);
        }
    }
    if selected("events") {
        for record in [false, true] {
            rows.push(run_event_pile(record)?);
        }
    }
    if selected("poses") {
        rows.extend(poses::run_poses()?);
    }
    print_table(&rows);
    println!();
    print_machine();
    Ok(())
}
