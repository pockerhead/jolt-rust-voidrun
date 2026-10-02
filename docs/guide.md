# Guide: a headless game world

This guide builds the scene `joltphysics` was written for: terrain, a building made of several
parts with their own collision groups, a dropped item, the queries a game runs every frame and a
floating origin. Everything runs headless; nothing is drawn. The complete program is at the end
and runs as a test (`cargo test -p joltphysics --doc`).

Units are metres, seconds, kilograms and radians. Jolt is right-handed with Y up.

## Layers and groups

Every body lives in one **object layer**, and each object layer is kept in one **broad-phase
layer** (a tree of the broad phase; static and moving bodies usually get one each). A
`CollisionLayers` table says which pairs of object layers collide. A group whose members are
whole bodies (terrain, chunks, items, actors) gets its own object layer.

Some groups share one body. A chunk is one static body whose shape is a compound of many parts,
and each part has a group: structures you can stand on, features (foliage, canopies) you walk
through. Store the group as the compound child's `user_data`. Queries read it back from hits
(`compound_child`) and filter on it with `QueryFilter::child_groups`, a bit mask over the user
data values 0 to 31.

`QueryFilter` has three parts that combine: `object_layers` selects whole bodies by layer,
`child_groups` selects compound children by group, `exclude_body` skips one body (the querying
actor's own). An unset part accepts everything.

## Shapes

- `Shape::new_box`, `new_sphere`, `new_cylinder` and `new_capsule` (cylinders and capsules along
  local Y). Boxes and cylinders have Jolt's convex radius of 0.05 m, which rounds their edges for
  contacts and shape casts; `new_box_with_convex_radius(.., 0.0)` gives sharp edges. Rays always
  see the sharp shape.
- `Shape::new_height_field` takes `n * n` samples in Jolt's row-major order: sample `(x, z)` is
  `samples[z * n + x]`. A column-major source must be transposed. `f32::MAX` is a hole. With the
  default block size 2, `n = 33` is padded to 34 and covers exactly the 32 x 32 cells given.
  Heightfields are for static bodies only.
- `Shape::new_compound` places children with their own pose and user data. Child order is part of
  the shape.

A `Shape` holds one Jolt reference and every body holds its own, so a shape can be dropped after
creating bodies and shared between bodies and worlds.

## Stepping

`PhysicsWorld::step(dt)` runs one collision step. `Err` means the step was rejected (a bad `dt`)
and nothing happened. `Ok(report)` means the world advanced; `report.is_complete()` is false when
Jolt dropped contacts because a fixed-size buffer was full, and the flag names the `WorldSettings`
limit to raise.

Gravity is per world (`WorldSettings::gravity`, `set_gravity`). A game with radial gravity sets it
to zero and adds a force to each body before the step (`BodyMut::add_force`).

## Queries

Queries take `&PhysicsWorld`, so many threads may run them while nobody steps the world. They see
created, moved and removed bodies at once, without a step. Every normal they report is the
outward surface normal of the obstacle: a floor below gives a normal pointing up.

- `cast_ray`: the closest hit along a ray. Convex shapes are solid (a ray that starts inside hits at
  fraction 0) and triangles are hit from both sides.
- `cast_shape`: the first obstacle a moving shape hits, with the distance it can move, the contact
  point and the obstacle's normal. `ShapeCast::target_distance` (spheres and capsules) stops the
  cast short of the obstacle.
- `collide_shape`: every obstacle a shape at a pose overlaps, with penetration depth and normal.
  The hits come in no particular order; sort them when order matters.

## Floating origin

`PhysicsWorld::rebase(bodies, rotation, translation)` moves the whole world into a new frame with
one rigid change of coordinates: positions, rotations, velocities and gravity. No body wakes up or
falls asleep, and resting bodies keep their contacts. The list must name every body of the world
once, in a stable order of the caller's choice. Rebase between steps, before adding the tick's
forces.

## Determinism

On one machine the same calls in the same order give bit-identical results for any
`WorldSettings::worker_threads`. The order in which bodies are created and removed is part of the
state: it decides the `BodyId`s. Keep hash-map iteration order, time and thread identity out of the
calls that drive the world, and sort `collide_shape` hits before acting on them. Equal results
across platforms and compilers need the `cross-platform-deterministic` feature; the README's
Determinism section has the details.

## Debug lines

With the `debug-renderer` feature, `PhysicsWorld::debug_lines` fills a `DebugLines` buffer with the
wireframe of the colliders around a point, filtered like a query. It produces line data only and
draws nothing. It has no level of detail, so cap the output with `DebugLineSettings::max_lines`.

## The whole example

```rust
use joltphysics::*;

/// The game's collision groups, stored as compound child user data.
const STRUCTURE: u32 = 1;
const FEATURE: u32 = 2;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Layers: static terrain and chunks, moving items. Items collide with everything.
    let mut layers = CollisionLayers::new(2);
    let fixed = BroadPhaseLayer::new(0);
    let moving = BroadPhaseLayer::new(1);
    let terrain = layers.add_object_layer(fixed);
    let chunk = layers.add_object_layer(fixed);
    let item = layers.add_object_layer(moving);
    layers
        .enable_collision(item, terrain)
        .enable_collision(item, chunk)
        .enable_collision(item, item);
    let mut world = PhysicsWorld::new(
        WorldSettings::default()
            .layers(layers)
            .worker_threads(2),
    )?;

    // Terrain: a flat 33 x 33 heightfield at y = 0, one sample per metre, x and z in [-16, 16].
    let ground = Shape::new_height_field(
        33,
        &[0.0; 33 * 33],
        &HeightFieldSettings::default().offset(Vec3::new(-16.0, 0.0, -16.0)),
    )?;
    let terrain_body =
        world.create_body(&ground, &BodySettings::new_static().object_layer(terrain))?;

    // A chunk at (5, 0, 5): a porch you can stand on (top at y = 1) under a canopy you walk
    // through (top at y = 3.5). One static body, two groups.
    let porch = Shape::new_box(Vec3::new(2.0, 0.5, 2.0))?;
    let canopy = Shape::new_cylinder(0.5, 2.0)?;
    let house = Shape::new_compound(&[
        CompoundChild {
            shape: &porch,
            position: Vec3::new(0.0, 0.5, 0.0),
            rotation: Quat::IDENTITY,
            user_data: STRUCTURE,
        },
        CompoundChild {
            shape: &canopy,
            position: Vec3::new(0.0, 3.0, 0.0),
            rotation: Quat::IDENTITY,
            user_data: FEATURE,
        },
    ])?;
    let chunk_body = world.create_body(
        &house,
        &BodySettings::new_static()
            .position(RVec3::new(5.0, 0.0, 5.0))
            .object_layer(chunk),
    )?;

    // An item dropped from 2 m: a 0.5 m crate of 2 kg with continuous collision detection.
    let crate_shape = Shape::new_box(Vec3::new(0.25, 0.25, 0.25))?;
    let item_body = world.create_body(
        &crate_shape,
        &BodySettings::new_dynamic()
            .position(RVec3::new(-5.0, 2.0, 0.0))
            .mass(2.0)
            .friction(0.5)
            .motion_quality(MotionQuality::LinearCast)
            .object_layer(item),
    )?;

    // Two seconds at 60 Hz.
    for _ in 0..120 {
        let report = world.step(1.0 / 60.0)?;
        assert!(report.is_complete(), "raise a WorldSettings limit: {report:?}");
    }
    let resting = world.body(item_body)?.position();
    assert!((resting.y - 0.25).abs() < 0.03, "the crate rests on the terrain");

    // Ground under the house: terrain and chunks, structures only, so the canopy is skipped.
    let ground_layers = [terrain, chunk];
    let ground_filter = QueryFilter::new()
        .object_layers(&ground_layers)
        .child_groups(1 << STRUCTURE);
    let down = RayCast::new(RVec3::new(5.0, 10.0, 5.0), Vec3::new(0.0, -20.0, 0.0));
    let hit = world.cast_ray(down, &ground_filter)?.expect("the porch");
    assert_eq!(hit.body, chunk_body);
    assert_eq!(hit.compound_child.map(|child| child.user_data), Some(STRUCTURE));
    assert!((down.point_at(hit.fraction).y - 1.0).abs() < 1e-3);
    assert!(hit.normal.y > 0.99, "a floor's normal points up");

    // The same ray without a group filter stops on the canopy.
    let hit = world.cast_ray(down, &QueryFilter::new())?.expect("the canopy");
    assert_eq!(hit.compound_child.map(|child| child.user_data), Some(FEATURE));

    // An actor capsule (feet 0.8 m below its origin) cast 5 m down onto the terrain.
    let actor = Shape::new_capsule(0.5, 0.3)?;
    let fall = ShapeCast::new(
        &actor,
        RVec3::new(-10.0, 3.0, -10.0),
        Quat::IDENTITY,
        Vec3::new(0.0, -5.0, 0.0),
    );
    let hit = world.cast_shape(&fall, &QueryFilter::new())?.expect("the terrain");
    assert_eq!(hit.body, terrain_body);
    assert!((hit.distance - 2.2).abs() < 1e-3);
    assert!(hit.normal.y > 0.99);

    // A ball sunk 0.3 m into the porch, skipping the item: one overlap, pushed out upwards.
    let ball = Shape::new_sphere(0.5)?;
    let probe = CollideShape::new(&ball, RVec3::new(5.0, 1.2, 5.0), Quat::IDENTITY);
    let mut hits = world.collide_shape(&probe, &QueryFilter::new().exclude_body(item_body))?;
    hits.sort_by_key(|hit| (hit.body.to_raw(), hit.sub_shape_id.to_raw()));
    assert_eq!(hits.len(), 1);
    assert!((hits[0].penetration_depth - 0.3).abs() < 1e-3);
    assert!(hits[0].normal.y > 0.99);

    // Floating origin: move everything so the house sits at the origin. Every body, in a
    // stable order (here: creation order).
    world.rebase(
        &[terrain_body, chunk_body, item_body],
        Quat::IDENTITY,
        RVec3::new(-5.0, 0.0, -5.0),
    )?;
    let moved = world.body(item_body)?.position();
    assert!((moved.x - (resting.x - 5.0)).abs() < 1e-4);
    assert!((moved.z - (resting.z - 5.0)).abs() < 1e-4);
    let down = RayCast::new(RVec3::new(0.0, 10.0, 0.0), Vec3::new(0.0, -20.0, 0.0));
    let hit = world.cast_ray(down, &ground_filter)?.expect("the porch");
    assert_eq!(hit.body, chunk_body);

    // The world keeps stepping in the new frame.
    for _ in 0..60 {
        assert!(world.step(1.0 / 60.0)?.is_complete());
    }
    Ok(())
}
```
