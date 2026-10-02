//! Same-machine determinism through the safe API: the per-tick state of a scene must be
//! bit-identical whether the world steps with 1 or 4 worker threads, also across a rebase, and
//! the order in which bodies are created must decide their ids.
//!
//! Two scenes are gated. The stacks scene topples cubes on a floor for 120 ticks. The chunk
//! scene is the game's case: a static compound chunk on a heightfield terrain, three items the
//! caller pulls towards a planet centre, created, coming to rest and removed under the game's
//! item rules, and a tilted rebase in the middle, recorded for ticks 0 to 1000 with the static
//! shapes and the body states of every tick. Creating the same bodies in another order must
//! fail the gate.
//!
//! Each run happens in its own child process (this test binary, running the ignored
//! `determinism_child` test), so no state leaks between runs; see `common::determinism`.

mod common;

use common::determinism::*;
use common::*;
use joltphysics::*;

/// Ticks of the stacks scene.
const TICKS: usize = 120;

/// Bytes recorded per body per tick by [`record_body`]: id, position, rotation, linear and
/// angular velocity, and the sleeping flag.
const BODY_RECORD_SIZE: usize = 4 + 3 * size_of::<Real>() + 4 * 4 + 3 * 4 + 3 * 4 + 1;

/// Bodies in the stacks scene: the floor and the cubes.
fn stacks_body_count() -> usize {
    1 + stacks_scene().len()
}

/// The rotation of the tilted rebases: 30 degrees about a tilted axis.
fn tilted_rotation() -> Quat {
    let axis = Vec3::new(1.0, 2.0, 0.5);
    let axis = Vec3::new(
        axis.x / length(axis),
        axis.y / length(axis),
        axis.z / length(axis),
    );
    quat_about(axis, 30.0_f32.to_radians())
}

/// The translation of the tilted rebases.
fn tilted_translation() -> RVec3 {
    RVec3::new(12.5, -3.0, 7.25)
}

/// Runs the stacks scene and returns one state record per step, of every body in scene order.
/// Bodies are created in scene order (floor first), or in reverse for `"reversed"`.
/// `"rebased"` creates them in scene order and moves the world into a tilted frame halfway
/// through.
fn run_stacks(worker_threads: u32, variant: &str) -> Digest {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), worker_threads);
    let ids = match variant {
        "forward" | "rebased" => build_stacks(&mut world),
        "reversed" => {
            let mut ids: Vec<BodyId> = stacks_scene()
                .into_iter()
                .rev()
                .map(|position| add_cube(&mut world, position))
                .collect();
            ids.push(add_floor(&mut world));
            ids.reverse();
            ids
        }
        variant => panic!("unknown stacks variant {variant}"),
    };
    let mut digest = Digest::new();
    for tick in 0..TICKS {
        if variant == "rebased" && tick == TICKS / 2 {
            world
                .rebase(&ids, tilted_rotation(), tilted_translation())
                .unwrap();
        }
        assert!(world.step(DT).unwrap().is_complete());
        let record = digest.push();
        for &id in &ids {
            record_body(&world, id, &mut record.state);
        }
    }
    digest
}

/// The raw body ids of the first tick, in scene order.
fn first_tick_ids(digest: &Digest) -> Vec<u32> {
    digest.ticks[0]
        .state
        .as_chunks::<BODY_RECORD_SIZE>()
        .0
        .iter()
        .map(|record| u32::from_le_bytes(record[..4].try_into().unwrap()))
        .collect()
}

/// The raw id Jolt gives the `n`-th body added to a fresh world: index `n`, sequence 1.
fn nth_body_id(n: usize) -> u32 {
    (1 << 23) | n as u32
}

/// Ticks of the chunk scene after the initial record at tick 0.
const CHUNK_TICKS: u32 = 1000;
/// The world moves into a tilted frame between this tick and the next.
const REBASE_TICK: u32 = 500;
/// Item 3 is created before the step after this tick, so it falls through the rebase.
const LATE_ITEM_TICK: u32 = 470;
/// Calm ticks in a row after which an item counts as at rest and is removed (the game's item
/// rest rule).
const REST_TICKS: u32 = 30;
/// Ticks after which an item is removed even if it never came to rest.
const ITEM_TIMEOUT: u32 = 600;
const ITEM_MASS: f32 = 1.2;
/// Strength of the pull towards the planet centre, m/s².
const GRAVITY: f32 = 9.8;
/// Height of the terrain's mean surface above the scene origin, metres.
const SURFACE: f32 = 2.0;
/// Radius of the planet whose surface the terrain is, centred below the scene.
const PLANET_RADIUS: f64 = 99.0;
const ITEM_HALF_EXTENT: Vec3 = Vec3::new(0.35, 0.06, 0.04);
/// Samples per side of the terrain heightfield.
const TERRAIN_SAMPLES: u32 = 33;
/// Convex radius of the chunk's boxes and cylinder, metres (Jolt's default).
const CHILD_CONVEX_RADIUS: f32 = 0.05;

type V = [f64; 3];

// `Real` is already `f64` with the `double-precision` feature.
#[allow(clippy::useless_conversion)]
fn real3(p: RVec3) -> V {
    [f64::from(p.x), f64::from(p.y), f64::from(p.z)]
}

fn cross(a: V, b: V) -> V {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// `p` rotated by `rotation` and moved by `translation`, in `f64`.
fn map_point(rotation: Quat, translation: RVec3, p: V) -> V {
    let [x, y, z, w] = [rotation.x, rotation.y, rotation.z, rotation.w].map(f64::from);
    let q = [x, y, z];
    let t = cross(q, p).map(|c| 2.0 * c);
    let u = cross(q, t);
    let translation = real3(translation);
    [0, 1, 2].map(|i| p[i] + w * t[i] + u[i] + translation[i])
}

/// The world for the chunk scene: terrain and chunk layers in broad-phase layer 0, the item
/// layer in 1; items collide with terrain, chunks and each other. No world gravity: the caller
/// pulls each item towards the planet centre.
fn chunk_world(worker_threads: u32) -> (PhysicsWorld, [ObjectLayer; 3]) {
    let mut layers = CollisionLayers::new(2);
    let terrain = layers.add_object_layer(BroadPhaseLayer::new(0));
    let chunk = layers.add_object_layer(BroadPhaseLayer::new(0));
    let item = layers.add_object_layer(BroadPhaseLayer::new(1));
    layers
        .enable_collision(item, terrain)
        .enable_collision(item, chunk)
        .enable_collision(item, item);
    let settings = WorldSettings::default()
        .gravity(Vec3::ZERO)
        .layers(layers)
        .worker_threads(worker_threads);
    (PhysicsWorld::new(settings).unwrap(), [terrain, chunk, item])
}

/// What one compound child is made of, as the caller authored it.
#[derive(Clone, Copy)]
enum ChildKind {
    Box { half_extent: Vec3 },
    Cylinder { half_height: f32, radius: f32 },
}

/// One authored child of the chunk's static compound.
struct ChildSpec {
    kind: ChildKind,
    convex_radius: f32,
    position: Vec3,
    rotation: Quat,
    group: u32,
}

/// The chunk's children in attach order. The chunk body is turned 90 degrees about +y, so local
/// `(x, y, z)` is world `(z, y, -x)`.
fn chunk_children() -> [ChildSpec; 3] {
    [
        // A block whose top is at y = 3, centred on world (0, 2.5, -3): items 1 and 2 land on it.
        ChildSpec {
            kind: ChildKind::Box {
                half_extent: Vec3::new(1.0, 0.5, 1.0),
            },
            position: Vec3::new(3.0, 2.5, 0.0),
            rotation: Quat::IDENTITY,
            convex_radius: CHILD_CONVEX_RADIUS,
            group: Groups::STRUCTURE,
        },
        ChildSpec {
            kind: ChildKind::Box {
                half_extent: Vec3::new(0.8, 0.3, 0.8),
            },
            position: Vec3::new(-3.0, 2.6, 2.0),
            rotation: quat_about(Vec3::new(0.0, 0.0, 1.0), 20.0_f32.to_radians()),
            convex_radius: CHILD_CONVEX_RADIUS,
            group: Groups::STRUCTURE,
        },
        // A pillar centred on world (-3, 3, 0) whose top is at y = 4: item 3 hits its edge.
        ChildSpec {
            kind: ChildKind::Cylinder {
                half_height: 1.0,
                radius: 0.4,
            },
            position: Vec3::new(0.0, 3.0, -3.0),
            rotation: Quat::IDENTITY,
            convex_radius: CHILD_CONVEX_RADIUS,
            group: Groups::FEATURE,
        },
    ]
}

/// The chunk: a static compound keyed by cube face and grid cell.
struct Chunk {
    key: (u8, i32, i32),
    body: BodyId,
    children: [ChildSpec; 3],
}

fn create_chunk(world: &mut PhysicsWorld, layer: ObjectLayer) -> Chunk {
    let children = chunk_children();
    let shapes: Vec<Shape> = children
        .iter()
        .map(|child| match child.kind {
            ChildKind::Box { half_extent } => {
                Shape::new_box_with_convex_radius(half_extent, child.convex_radius).unwrap()
            }
            ChildKind::Cylinder {
                half_height,
                radius,
            } => Shape::new_cylinder_with_convex_radius(half_height, radius, child.convex_radius)
                .unwrap(),
        })
        .collect();
    let compound_children: Vec<CompoundChild<'_>> = children
        .iter()
        .zip(&shapes)
        .map(|(child, shape)| CompoundChild {
            shape,
            position: child.position,
            rotation: child.rotation,
            user_data: child.group,
        })
        .collect();
    let shape = Shape::new_compound(&compound_children).unwrap();
    let body = world
        .create_body(
            &shape,
            &BodySettings::new_static()
                .rotation(quat_about(Vec3::new(0.0, 1.0, 0.0), 90.0_f32.to_radians()))
                .object_layer(layer),
        )
        .unwrap();
    Chunk {
        key: (0, 0, 0),
        body,
        children,
    }
}

/// The terrain under the chunk: a sloped, ridged heightfield as the caller authored it.
struct Terrain {
    body: BodyId,
    shape: Shape,
    samples: Vec<f32>,
    settings_offset: Vec3,
    settings_scale: Vec3,
}

fn create_terrain(world: &mut PhysicsWorld, layer: ObjectLayer) -> Terrain {
    let n = TERRAIN_SAMPLES as usize;
    let mut samples = Vec::with_capacity(n * n);
    for z in 0..n {
        for x in 0..n {
            let slope = 0.05 * (z as f32 - 16.0);
            let ridges = 0.04 * ((7 * x + 3 * z) % 5) as f32;
            samples.push(SURFACE + slope + ridges);
        }
    }
    let settings_offset = Vec3::new(-16.0, 0.0, -16.0);
    let settings_scale = Vec3::new(1.0, 1.0, 1.0);
    let settings = HeightFieldSettings::default()
        .offset(settings_offset)
        .scale(settings_scale);
    let shape = Shape::new_height_field(TERRAIN_SAMPLES, &samples, &settings).unwrap();
    let body = add_static_in(world, &shape, RVec3::ZERO, layer);
    Terrain {
        body,
        shape,
        samples,
        settings_offset,
        settings_scale,
    }
}

/// Where an item is in its life.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Phase {
    Pending,
    Live(BodyId),
    Removed,
}

/// Why an item was removed.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Reason {
    None = 0,
    Rest = 1,
    Timeout = 2,
}

/// A dropped item, keyed by the caller.
struct Item {
    key: u64,
    position: RVec3,
    rotation: Quat,
    /// The tick before whose step the item is created.
    created_at: u32,
    phase: Phase,
    calm_ticks: u32,
    age: u32,
    reason: Reason,
    /// The item's last [`record_body`] bytes, taken just before its removal.
    last_body: Vec<u8>,
}

/// The three items in key order.
fn items() -> Vec<Item> {
    let item = |key, position, rotation, created_at| Item {
        key,
        position,
        rotation,
        created_at,
        phase: Phase::Pending,
        calm_ticks: 0,
        age: 0,
        reason: Reason::None,
        last_body: Vec::new(),
    };
    vec![
        // One centimetre above the chunk's block.
        item(1, RVec3::new(0.0, 3.07, -3.0), Quat::IDENTITY, 1),
        // Across item 1, dropped onto it.
        item(
            2,
            RVec3::new(0.0, 3.6, -3.0),
            quat_about(Vec3::new(0.0, 1.0, 0.0), 90.0_f32.to_radians()),
            1,
        ),
        // High above the edge of the pillar's top.
        item(
            3,
            RVec3::new(-2.8, 12.0, 0.1),
            Quat::IDENTITY,
            LATE_ITEM_TICK + 1,
        ),
    ]
}

fn create_item(world: &mut PhysicsWorld, layer: ObjectLayer, item: &mut Item) {
    let shape = Shape::new_box(ITEM_HALF_EXTENT).unwrap();
    let body = world
        .create_body(
            &shape,
            &BodySettings::new_dynamic()
                .position(item.position)
                .rotation(item.rotation)
                .mass(ITEM_MASS)
                .friction(0.8)
                .restitution(0.1)
                .motion_quality(MotionQuality::LinearCast)
                .object_layer(layer),
        )
        .unwrap();
    item.phase = Phase::Live(body);
}

/// The world-space bounds `(min, max)` of a live item: its box turned by the body's rotation,
/// as Jolt's `AABox::Transformed` computes them.
fn item_bounds(world: &PhysicsWorld, id: BodyId) -> (V, V) {
    let body = world.body(id).unwrap();
    let centre = real3(body.position());
    let half = <[f32; 3]>::from(ITEM_HALF_EXTENT).map(f64::from);
    let rotation = body.rotation();
    let axes = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        .map(|axis| map_point(rotation, RVec3::ZERO, axis));
    let reach: V = [0, 1, 2].map(|i| (0..3).map(|a| axes[a][i].abs() * half[a]).sum());
    (
        [0, 1, 2].map(|i| centre[i] - reach[i]),
        [0, 1, 2].map(|i| centre[i] + reach[i]),
    )
}

/// Whether two bounds overlap, touching included, as Jolt's `AABox::Overlaps`.
fn overlap((a_min, a_max): (V, V), (b_min, b_max): (V, V)) -> bool {
    (0..3).all(|i| a_min[i] <= b_max[i] && b_min[i] <= a_max[i])
}

fn put_u32(out: &mut Vec<u8>, value: u32) {
    out.extend_from_slice(&value.to_le_bytes());
}

fn put_f32s(out: &mut Vec<u8>, values: impl IntoIterator<Item = f32>) {
    for value in values {
        out.extend_from_slice(&value.to_bits().to_le_bytes());
    }
}

/// The body's raw id, then its position (`Real` bits) and rotation.
fn put_body_pose(world: &PhysicsWorld, id: BodyId, out: &mut Vec<u8>) {
    let body = world.body(id).unwrap();
    put_u32(out, id.to_raw());
    for value in <[Real; 3]>::from(body.position()) {
        out.extend_from_slice(&value.to_bits().to_le_bytes());
    }
    put_f32s(out, <[f32; 4]>::from(body.rotation()));
}

/// The shape section of one tick: the chunk's key, anchor and children, then the terrain's
/// samples as authored and as Jolt stores them.
fn record_shape(world: &PhysicsWorld, chunk: &Chunk, terrain: &Terrain, out: &mut Vec<u8>) {
    let (face, x, z) = chunk.key;
    out.push(face);
    out.extend_from_slice(&x.to_le_bytes());
    out.extend_from_slice(&z.to_le_bytes());
    put_body_pose(world, chunk.body, out);
    put_u32(out, chunk.children.len() as u32);
    for (index, child) in chunk.children.iter().enumerate() {
        // The attach index is the child's identity, as in `CompoundSubShape::index`.
        put_u32(out, index as u32);
        match child.kind {
            ChildKind::Box { half_extent } => {
                out.push(0);
                put_f32s(out, <[f32; 3]>::from(half_extent));
            }
            ChildKind::Cylinder {
                half_height,
                radius,
            } => {
                out.push(1);
                put_f32s(out, [half_height, radius]);
            }
        }
        put_f32s(out, [child.convex_radius]);
        put_f32s(out, <[f32; 3]>::from(child.position));
        put_f32s(out, <[f32; 4]>::from(child.rotation));
        put_u32(out, child.group);
    }

    put_body_pose(world, terrain.body, out);
    put_u32(out, TERRAIN_SAMPLES);
    put_f32s(out, <[f32; 3]>::from(terrain.settings_offset));
    put_f32s(out, <[f32; 3]>::from(terrain.settings_scale));
    put_f32s(out, terrain.samples.iter().copied());
    for y in 0..TERRAIN_SAMPLES {
        for x in 0..TERRAIN_SAMPLES {
            match terrain.shape.height_field_position(x, y) {
                None => out.push(0),
                Some(position) => {
                    out.push(1);
                    put_f32s(out, <[f32; 3]>::from(position));
                }
            }
        }
    }
}

/// The state section of one tick: chunk and terrain bodies, then every item slot in key order.
fn record_state(world: &PhysicsWorld, chunk: &Chunk, terrain: &Terrain, items: &[Item]) -> Vec<u8> {
    let mut out = Vec::new();
    record_body(world, chunk.body, &mut out);
    record_body(world, terrain.body, &mut out);
    for item in items {
        out.extend_from_slice(&item.key.to_le_bytes());
        let tag = match item.phase {
            Phase::Pending => 0,
            Phase::Live(_) => 1,
            Phase::Removed => 2,
        };
        out.push(tag);
        if item.phase == Phase::Pending {
            continue;
        }
        put_u32(&mut out, item.calm_ticks);
        put_u32(&mut out, item.age);
        out.push(item.reason as u8);
        match item.phase {
            Phase::Live(id) => record_body(world, id, &mut out),
            _ => out.extend_from_slice(&item.last_body),
        }
    }
    out
}

/// Runs the chunk scene and returns its records for ticks `0..=CHUNK_TICKS`.
///
/// A static compound chunk, a heightfield terrain and three items that the caller pulls towards
/// the planet centre. Items follow the game's lifecycle: an item that stays calm for
/// [`REST_TICKS`] ticks in a row, or reaches [`ITEM_TIMEOUT`] ticks of age, is removed. Item 3
/// is created late, so it is falling when the world moves into a tilted frame after tick
/// [`REBASE_TICK`]. `"forward"` creates chunk, terrain, item 1 and item 2 in that order;
/// `"permuted"` creates them in reverse. Both record in key order.
///
/// Panics unless the run covers what the gate is for: every step complete, item 3 falling at
/// the rebase in a reused body slot, all three items created, and an item removed at rest while
/// a live item's bounds overlap its own, so the removal's wake has a dynamic neighbour to find.
fn run_chunk(worker_threads: u32, variant: &str) -> Digest {
    let (mut world, [terrain_layer, chunk_layer, item_layer]) = chunk_world(worker_threads);
    let mut items = items();
    let (chunk, terrain) = match variant {
        "forward" => {
            let chunk = create_chunk(&mut world, chunk_layer);
            let terrain = create_terrain(&mut world, terrain_layer);
            for item in items.iter_mut().filter(|item| item.created_at == 1) {
                create_item(&mut world, item_layer, item);
            }
            (chunk, terrain)
        }
        "permuted" => {
            for item in items.iter_mut().rev().filter(|item| item.created_at == 1) {
                create_item(&mut world, item_layer, item);
            }
            let terrain = create_terrain(&mut world, terrain_layer);
            let chunk = create_chunk(&mut world, chunk_layer);
            (chunk, terrain)
        }
        variant => panic!("unknown chunk variant {variant}"),
    };
    let mut centre = [0.0, f64::from(SURFACE) - PLANET_RADIUS, 0.0];

    let mut digest = Digest::new();
    let record = |world: &PhysicsWorld, items: &[Item], digest: &mut Digest| {
        let tick = digest.push();
        record_shape(world, &chunk, &terrain, &mut tick.shape);
        tick.state = record_state(world, &chunk, &terrain, items);
    };
    record(&world, &items, &mut digest);

    let mut rested_touching_a_live_item = false;
    for tick in 1..=CHUNK_TICKS {
        for item in items.iter_mut() {
            if item.created_at == tick && item.phase == Phase::Pending {
                create_item(&mut world, item_layer, item);
            }
        }
        if tick == REBASE_TICK + 1 {
            let Phase::Live(late) = items[2].phase else {
                panic!("item 3 is not live at the rebase");
            };
            // Items 1 and 2 have come to rest and gone, so item 3 reuses a freed body index.
            assert_eq!(late.sequence(), 2, "item 3 got a fresh body index");
            let speed = length(world.body(late).unwrap().linear_velocity());
            assert!(
                speed > 0.5,
                "item 3 moves at only {speed} m/s at the rebase"
            );
            let mut bodies = vec![chunk.body, terrain.body];
            bodies.extend(items.iter().filter_map(|item| match item.phase {
                Phase::Live(id) => Some(id),
                _ => None,
            }));
            let (rotation, translation) = (tilted_rotation(), tilted_translation());
            world.rebase(&bodies, rotation, translation).unwrap();
            centre = map_point(rotation, translation, centre);
        }
        for item in &items {
            let Phase::Live(id) = item.phase else {
                continue;
            };
            let position = real3(world.body(id).unwrap().position());
            let towards: V = [0, 1, 2].map(|i| centre[i] - position[i]);
            let distance =
                (towards[0] * towards[0] + towards[1] * towards[1] + towards[2] * towards[2])
                    .sqrt();
            let pull = towards.map(|c| f64::from(ITEM_MASS * GRAVITY) * c / distance);
            let mut body = world.body_mut(id).unwrap();
            body.reset_forces();
            body.add_force(Vec3::new(pull[0] as f32, pull[1] as f32, pull[2] as f32))
                .unwrap();
        }
        assert!(world.step(DT).unwrap().is_complete(), "tick {tick}");

        let mut leaving = Vec::new();
        for (slot, item) in items.iter_mut().enumerate() {
            let Phase::Live(id) = item.phase else {
                continue;
            };
            let calm = is_calm(&world.body(id).unwrap());
            item.calm_ticks = if calm { item.calm_ticks + 1 } else { 0 };
            item.age += 1;
            if item.calm_ticks == REST_TICKS {
                item.reason = Reason::Rest;
                leaving.push(slot);
            } else if item.age == ITEM_TIMEOUT {
                item.reason = Reason::Timeout;
                leaving.push(slot);
            }
        }
        for slot in leaving {
            let Phase::Live(id) = items[slot].phase else {
                unreachable!("only live items leave");
            };
            let bounds = item_bounds(&world, id);
            let touches_a_live_item = items.iter().any(|item| match item.phase {
                Phase::Live(other) => other != id && overlap(bounds, item_bounds(&world, other)),
                _ => false,
            });
            if items[slot].reason == Reason::Rest && touches_a_live_item {
                rested_touching_a_live_item = true;
            }
            let mut last_body = Vec::new();
            record_body(&world, id, &mut last_body);
            world.remove_body(id).unwrap();
            let item = &mut items[slot];
            item.last_body = last_body;
            item.phase = Phase::Removed;
        }
        record(&world, &items, &mut digest);
    }

    assert!(
        items.iter().all(|item| item.phase != Phase::Pending),
        "an item was never created"
    );
    assert!(
        rested_touching_a_live_item,
        "no item came to rest touching a live item"
    );
    digest
}

#[test]
#[ignore = "child process of the determinism gates"]
fn determinism_child() {
    let Some((scenario, threads, variant)) = child_request() else {
        return;
    };
    let digest = match scenario.as_str() {
        "stacks" => run_stacks(threads, &variant),
        "chunk" => run_chunk(threads, &variant),
        scenario => panic!("unknown scenario {scenario}"),
    };
    finish_child(&digest);
}

fn stacks_in_child(threads: u32, variant: &str) -> Digest {
    digest_in_child("determinism_child", "stacks", threads, variant)
}

#[test]
fn stacks_digest_is_identical_across_thread_counts() {
    let one_thread = stacks_in_child(1, "forward");
    let four_threads = stacks_in_child(4, "forward");
    let reversed = stacks_in_child(1, "reversed");
    let rebased_one_thread = stacks_in_child(1, "rebased");
    let rebased_four_threads = stacks_in_child(4, "rebased");

    assert_eq!(one_thread.ticks.len(), TICKS);
    assert_eq!(
        one_thread.ticks[0].state.len(),
        stacks_body_count() * BODY_RECORD_SIZE
    );
    assert_same("stacks, 1 vs 4 worker threads", &one_thread, &four_threads);
    assert_same(
        "rebased stacks, 1 vs 4 worker threads",
        &rebased_one_thread,
        &rebased_four_threads,
    );
    // The rebase happened: the rebased run leaves the forward run's frame.
    assert!(first_divergence(&rebased_one_thread, &one_thread).is_some());

    // Insertion order is part of the state: reversing creation reverses the ids of the same
    // bodies, so the digest changes.
    let created_forward: Vec<u32> = (0..stacks_body_count()).map(nth_body_id).collect();
    let created_reversed: Vec<u32> = (0..stacks_body_count()).rev().map(nth_body_id).collect();
    assert_eq!(first_tick_ids(&one_thread), created_forward);
    assert_eq!(first_tick_ids(&reversed), created_reversed);
    assert!(first_divergence(&reversed, &one_thread).is_some());
}

fn chunk_in_child(threads: u32, variant: &str) -> Digest {
    digest_in_child("determinism_child", "chunk", threads, variant)
}

#[test]
fn chunk_digest_is_identical_across_thread_counts() {
    let one_thread = chunk_in_child(1, "forward");
    let four_threads = chunk_in_child(4, "forward");
    assert_eq!(one_thread.ticks.len(), CHUNK_TICKS as usize + 1);
    for (tick, record) in one_thread.ticks.iter().enumerate() {
        assert!(!record.shape.is_empty(), "tick {tick}: empty shape record");
        assert!(!record.state.is_empty(), "tick {tick}: empty state record");
    }
    assert_same("chunk, 1 vs 4 worker threads", &one_thread, &four_threads);
}

/// Offset of the chunk's raw body id in a shape record: after the key (`u8`, `i32`, `i32`).
const CHUNK_ID_IN_SHAPE: usize = 1 + 4 + 4;
/// Bytes of an item slot's header before its body record: key, phase, calm ticks, age and
/// reason.
const ITEM_HEADER_SIZE: usize = 8 + 1 + 4 + 4 + 1;

fn u32_at(bytes: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap())
}

/// The raw ids of chunk, terrain, item 1 and item 2 at tick 0, read from the state record, and
/// the chunk's id read from the shape record.
fn tick_zero_ids(digest: &Digest) -> ([u32; 4], u32) {
    let tick = &digest.ticks[0];
    let item_slot = ITEM_HEADER_SIZE + BODY_RECORD_SIZE;
    let ids = [
        0,
        BODY_RECORD_SIZE,
        2 * BODY_RECORD_SIZE + ITEM_HEADER_SIZE,
        2 * BODY_RECORD_SIZE + item_slot + ITEM_HEADER_SIZE,
    ]
    .map(|offset| u32_at(&tick.state, offset));
    (ids, u32_at(&tick.shape, CHUNK_ID_IN_SHAPE))
}

#[test]
fn permuted_insertion_order_fails_the_gate() {
    let forward = chunk_in_child(1, "forward");
    let permuted = chunk_in_child(4, "permuted");
    let divergence =
        first_divergence(&forward, &permuted).expect("a permuted insertion order must diverge");
    assert!(
        matches!(divergence, Divergence::Tick { tick: 0, .. }),
        "{divergence}"
    );

    let (forward_ids, forward_chunk) = tick_zero_ids(&forward);
    let (permuted_ids, permuted_chunk) = tick_zero_ids(&permuted);
    assert_eq!(forward_ids, [0, 1, 2, 3].map(nth_body_id));
    assert_eq!(permuted_ids, [3, 2, 1, 0].map(nth_body_id));
    assert_eq!(forward_chunk, forward_ids[0]);
    assert_eq!(permuted_chunk, permuted_ids[0]);

    let gate = std::panic::catch_unwind(|| assert_same("permuted", &forward, &permuted));
    assert!(
        gate.is_err(),
        "the gate accepted a permuted insertion order"
    );
    println!("permuted insertion order diverges at {divergence}");
}

/// A digest of `ticks` ticks whose sections hold the given bytes.
fn digest_of(ticks: &[(&[u8], &[u8])]) -> Digest {
    let mut digest = Digest::new();
    for &(shape, state) in ticks {
        let tick = digest.push();
        tick.shape.extend_from_slice(shape);
        tick.state.extend_from_slice(state);
    }
    digest
}

#[test]
fn child_requests_are_parsed_or_rejected() {
    assert_eq!(
        parse_child_request("chunk,4,permuted"),
        ("chunk".to_owned(), 4, "permuted".to_owned())
    );
    for (request, expected) in [
        ("chunk", "has no threads field"),
        ("chunk,4", "has no variant field"),
        ("chunk,four,forward", "bad thread count"),
    ] {
        let panic = std::panic::catch_unwind(|| parse_child_request(request)).unwrap_err();
        let message = panic.downcast_ref::<String>().unwrap();
        assert!(message.contains(expected), "{request:?}: {message}");
    }
}

#[test]
fn digest_encoding_round_trips() {
    for digest in [
        Digest::new(),
        digest_of(&[(&[], &[])]),
        digest_of(&[(&[1, 2, 3], &[]), (&[], &[4]), (&[5, 6], &[7, 8, 9])]),
    ] {
        assert_eq!(Digest::decode(&digest.encode()), Ok(digest));
    }
}

#[test]
fn digest_decoding_rejects_malformed_input() {
    let bytes = digest_of(&[(&[1, 2, 3], &[4, 5])]).encode();
    // Tick count 1, shape length 3: offsets 0 and 4; the shape bytes start at offset 8.
    let truncated_length = &bytes[..6];
    let truncated_section = &bytes[..9];
    let mut trailing = bytes.clone();
    trailing.push(0);
    for (input, expected) in [
        (truncated_length, "truncated length at offset 4"),
        (truncated_section, "truncated section at offset 8"),
        (&trailing[..], "1 trailing bytes at offset 17"),
    ] {
        assert_eq!(Digest::decode(input), Err(expected.to_owned()));
    }
}

#[test]
fn first_divergence_names_tick_section_and_byte() {
    let base = digest_of(&[(&[1, 2], &[3, 4]), (&[5, 6], &[7, 8, 9])]);
    assert_eq!(first_divergence(&base, &base.clone()), None);

    let state = digest_of(&[(&[1, 2], &[3, 4]), (&[5, 6], &[7, 0, 9])]);
    let expected = Divergence::Tick {
        tick: 1,
        section: Section::State,
        byte: 1,
    };
    assert_eq!(first_divergence(&base, &state), Some(expected));
    assert_eq!(expected.to_string(), "tick 1, state byte 1");

    // The shape section of a tick is compared before its state section.
    let shape_and_state = digest_of(&[(&[1, 2], &[3, 4]), (&[5, 0], &[0, 8, 9])]);
    assert_eq!(
        first_divergence(&base, &shape_and_state),
        Some(Divergence::Tick {
            tick: 1,
            section: Section::Shape,
            byte: 1,
        })
    );

    let shorter = digest_of(&[(&[1, 2], &[3, 4]), (&[5, 6], &[7, 8])]);
    assert_eq!(
        first_divergence(&base, &shorter),
        Some(Divergence::Tick {
            tick: 1,
            section: Section::State,
            byte: 2,
        })
    );

    let fewer_ticks = digest_of(&[(&[1, 2], &[3, 4])]);
    let expected = Divergence::TickCount { a: 2, b: 1 };
    assert_eq!(first_divergence(&base, &fewer_ticks), Some(expected));
    assert_eq!(expected.to_string(), "tick counts 2 and 1");
}

#[test]
fn assert_same_panics_with_the_divergence() {
    let a = digest_of(&[(&[1], &[2])]);
    let b = digest_of(&[(&[1], &[3])]);
    assert_same("equal", &a, &a.clone());
    let payload = std::panic::catch_unwind(|| assert_same("runs", &a, &b)).unwrap_err();
    assert_eq!(
        payload.downcast_ref::<String>().map(String::as_str),
        Some("runs: digests diverge at tick 0, state byte 0")
    );
}
