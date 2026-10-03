# Guide: a headless game world

This guide builds the scene `joltphysics` was written for: terrain, a building made of several
parts with their own collision groups, a dropped item, the queries a game runs every frame and a
floating origin, then a character walking on a small planet. Everything runs headless; nothing is
drawn. The two complete programs are at the end and run as tests (`cargo test -p joltphysics --doc`).

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

## Characters

A character is Jolt's `CharacterVirtual`: a convex shape, usually a capsule, that is not a body.
`PhysicsWorld::create_character` adds one; each tick the game sets its velocity and calls
`update_character`, which moves it by collision queries, slides it along what it hits and then
applies `ExtendedUpdateSettings`: stick to the floor and Jolt's walk stairs. Gravity is an argument
of the update and is not added to the velocity; the caller feeds the vertical speed itself.

- **Shape and pose.** The character position is where the shape's offset starts. With a capsule of
  half height `h` and radius `r`, `shape_offset(0, h + r, 0)` puts the capsule's bottom at the
  position, and Jolt adds `character_padding` (0.02 m) along up, so a character at rest stands that
  far above the ground.
- **Up.** Up and rotation can change before every update (`CharacterMut::set_up`, `set_rotation`),
  so on a planet up is radial. The `ExtendedUpdateSettings` vectors are in world space and default
  to +Y; with another up, pass them along it.
- **Ground.** `ground_state` is `OnGround`, `OnSteepGround` (steeper than `max_slope_angle`),
  `NotSupported` or `InAir`. Below about 0.81° Jolt turns the slope limit off, so with
  `max_slope_angle(0.0)` no ground is too steep. A new character reports `InAir` until its first update or
  `refresh_character_contacts`; refresh one that starts on the ground, or stick to floor (which
  needs support before the update) does nothing on the first tick.
- **Contacts.** `active_contacts` lists what the last update touched. Normals point toward the
  character. `contact_compound_child` gives the touched compound child's `user_data` (the game's
  collision group), `contact_object_layer` the touched body's layer.
- **Filters.** `update_character` and `refresh_character_contacts` take the same `QueryFilter` as
  queries. A game that keeps a kinematic capsule body per actor excludes the character's own with
  `exclude_body`; a character's optional inner body (`CharacterSettings::inner_body`) is never hit
  by its own character.
- **Replays.** `save_state` returns the character's persistent state (pose, velocity, up, ground and
  the contacts with collision); `restore_state` into a world rebuilt the same way continues the run
  bit for bit. Values the caller carries between ticks, such as its vertical speed, belong in the
  replay too. Every world numbers its characters from 1 in creation order, so a rebuilt world gives
  the same ids.
- **Rebase.** `PhysicsWorld::rebase` moves the characters with the bodies; after a rotating rebase,
  call `refresh_character_contacts` for each character before its next update.

### Sharp steps and the game's own autostep

Jolt's walk stairs and its steep-slope test judge an obstacle by the surface normal at the contact.
On a box with sharp edges (`new_box_with_convex_radius(.., 0.0)`) a capsule pressing on a face
touches it at the top edge. There Jolt picks the top face's normal or the side's by float
rounding of the contact point, so walk stairs on sharp steps is unreliable: the same settings
climb a 0.45 m step and refuse a 0.40 m one, and the outcome changes from scene to scene.
The capsule also creeps onto low sharp edges by itself (up to about 0.37 m in the walker tests).
No `CharacterSettings` value makes this dependable. On Jolt's default rounded boxes walk stairs
works (a 0.25 m step with a 0.4 m step-up is climbed), but the height it climbs is about
step-up + padding + `r (1 - cos max_slope_angle)`, not the step-up itself, and the step's rounding
changes it too, so measure it for your capsule.

A game with sharp structures turns walk stairs off (`walk_stairs_step_up(Vec3::ZERO)`) and steps
in its own code after the update, with shape casts, which report the geometry's own normal: when a
grounded walker was held back by a steep contact, cast the capsule up by the step height (head
room), forward by the free room the step needs, and down onto the step; if the top is walkable and
not a dynamic body, set the character's position there and refresh its contacts. The repository's
reference controller for VOIDRUN does exactly this in `crates/joltphysics/tests/common/walker.rs`
(a 0.45 m step with 0.5 m of free room on a planet of radius 99, with underground recovery, the
vertical speed rules, stick to floor as the floor snap and a chained replay). It is test support,
not part of the API: copy and adapt it.

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

## A walking character

A character walks on a planet of radius 50 m, a static sphere at the origin, so up is radial and
changes every tick. Gravity is applied by the caller. The helpers that turn radial up into a
rotation and a walking direction work at every point of the sphere, which the example checks by
starting a short walk at both poles and on the equator. A chunk compound stands in its path: a
feature (a bush) the character walks through and a structure (a wall) that stops it, told apart by
the filter's group mask. Halfway through the walk the character's state is saved, and a rebuilt
world continues from it bit for bit.

```rust
use joltphysics::*;

/// The game's collision groups, stored as compound child user data.
const STRUCTURE: u32 = 1;
const FEATURE: u32 = 2;
const PLANET_RADIUS: f32 = 50.0;
const GRAVITY: f32 = 9.8;
const DT: f32 = 1.0 / 60.0;

fn scale(v: Vec3, s: f32) -> Vec3 {
    Vec3::new(v.x * s, v.y * s, v.z * s)
}

fn add(a: Vec3, b: Vec3) -> Vec3 {
    Vec3::new(a.x + b.x, a.y + b.y, a.z + b.z)
}

fn dot(a: Vec3, b: Vec3) -> f32 {
    a.x * b.x + a.y * b.y + a.z * b.z
}

fn normalize(v: Vec3) -> Vec3 {
    scale(v, 1.0 / dot(v, v).sqrt())
}

/// Radial up at `p`, away from the planet's centre.
fn up_at(p: RVec3) -> Vec3 {
    normalize(Vec3::new(p.x as f32, p.y as f32, p.z as f32))
}

/// A rotation that turns +Y into the unit vector `up`. On the upper half it is the shortest one;
/// on the lower half it first turns +Y half a turn about X to -Y and then takes the shortest way,
/// so no division comes near zero.
fn rotation_to(up: Vec3) -> Quat {
    let q = if up.y >= 0.0 {
        Quat::from_xyzw(up.z, 0.0, -up.x, 1.0 + up.y)
    } else {
        Quat::from_xyzw(1.0 - up.y, up.x, 0.0, up.z)
    };
    let length = (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w).sqrt();
    Quat::from_xyzw(q.x / length, q.y / length, q.z / length, q.w / length)
}

/// The walking direction at unit `up`: east (+X along the ground), or +Z along the ground near
/// the two points where up is ±X and east does not exist.
fn forward_at(up: Vec3) -> Vec3 {
    let reference = if up.x.abs() < 0.9 {
        Vec3::new(1.0, 0.0, 0.0)
    } else {
        Vec3::new(0.0, 0.0, 1.0)
    };
    normalize(add(reference, scale(up, -dot(reference, up))))
}

/// Distance of `p` above the planet's surface.
fn height(p: RVec3) -> f32 {
    let p = Vec3::new(p.x as f32, p.y as f32, p.z as f32);
    dot(p, p).sqrt() - PLANET_RADIUS
}

/// The planet, a chunk 3 m east of the north pole and a character standing on the ground where
/// up is `start`.
fn build(start: Vec3) -> Result<(PhysicsWorld, CharacterId), Box<dyn std::error::Error>> {
    // Radial gravity is the caller's, so the world has none.
    let mut world = PhysicsWorld::new(WorldSettings::default().gravity(Vec3::ZERO))?;
    let planet = Shape::new_sphere(PLANET_RADIUS)?;
    world.create_body(&planet, &BodySettings::new_static())?;

    // The chunk's origin is on the ground with its local Y along up: a bush 1.5 m before a
    // 3 m wall, both sunk 0.5 m into the ground.
    let angle = 3.0 / PLANET_RADIUS;
    let ground = Vec3::new(PLANET_RADIUS * angle.sin(), PLANET_RADIUS * angle.cos(), 0.0);
    let wall = Shape::new_box(Vec3::new(0.25, 1.5, 2.0))?;
    let bush = Shape::new_cylinder(1.0, 0.4)?;
    let chunk = Shape::new_compound(&[
        CompoundChild {
            shape: &wall,
            position: Vec3::new(0.0, 1.0, 0.0),
            rotation: Quat::IDENTITY,
            user_data: STRUCTURE,
        },
        CompoundChild {
            shape: &bush,
            position: Vec3::new(-1.5, 0.5, 0.0),
            rotation: Quat::IDENTITY,
            user_data: FEATURE,
        },
    ])?;
    world.create_body(
        &chunk,
        &BodySettings::new_static()
            .position(RVec3::new(ground.x as Real, ground.y as Real, 0.0))
            .rotation(rotation_to(normalize(ground))),
    )?;

    // A capsule 1.6 m tall whose bottom is at the character position.
    let capsule = Shape::new_capsule(0.5, 0.3)?;
    let settings = CharacterSettings::new(&capsule)
        .shape_offset(Vec3::new(0.0, 0.8, 0.0))
        .max_slope_angle(45.0_f32.to_radians());
    let ground = scale(start, PLANET_RADIUS);
    let position = RVec3::new(ground.x as Real, ground.y as Real, ground.z as Real);
    let id = world.create_character(&settings, position, rotation_to(start))?;
    // A new character knows no ground until its contacts are refreshed.
    world.refresh_character_contacts(id, &walk_filter())?;
    Ok((world, id))
}

/// What the character collides with: every body, but of compounds only the structures.
fn walk_filter() -> QueryFilter<'static> {
    QueryFilter::new().child_groups(1 << STRUCTURE)
}

/// One tick: walk forward at 2 m/s along the ground. `vel_up` is the vertical speed the caller
/// carries from tick to tick.
fn tick(
    world: &mut PhysicsWorld,
    id: CharacterId,
    vel_up: &mut f32,
) -> Result<(), Box<dyn std::error::Error>> {
    let character = world.character(id)?;
    let up = up_at(character.position());
    *vel_up = if character.ground_state() == GroundState::OnGround {
        // One tick of gravity, so that stick to floor has something to follow.
        -GRAVITY * DT
    } else {
        *vel_up - GRAVITY * DT
    };
    let forward = forward_at(up);
    let mut character = world.character_mut(id)?;
    character.set_up(up)?;
    character.set_rotation(rotation_to(up))?;
    character.set_linear_velocity(add(scale(forward, 2.0), scale(up, *vel_up)))?;
    // Stick to floor along -up; walk stairs off, as for a game with sharp steps.
    let extended = ExtendedUpdateSettings::default()
        .stick_to_floor_step_down(scale(up, -0.3))
        .walk_stairs_step_up(Vec3::ZERO);
    world.update_character(id, DT, scale(up, -GRAVITY), &extended, &walk_filter())?;
    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let north = Vec3::new(0.0, 1.0, 0.0);
    let (mut world, id) = build(north)?;
    assert_eq!(world.character(id)?.ground_state(), GroundState::OnGround);

    // One second of walking, then save the state and the caller's vertical speed.
    let mut vel_up = 0.0;
    for _ in 0..60 {
        tick(&mut world, id, &mut vel_up)?;
    }
    let saved = (world.character(id)?.save_state(), vel_up);

    // Another second: the walk ends at the wall, after passing through the bush.
    for _ in 0..60 {
        tick(&mut world, id, &mut vel_up)?;
    }
    let character = world.character(id)?;
    assert_eq!(character.ground_state(), GroundState::OnGround);
    let p = character.position();
    let above = height(p);
    assert!(above.abs() < 0.01, "on the ground: {above}");
    let walked = PLANET_RADIUS * (p.x as f32).atan2(p.y as f32);
    assert!((2.3..2.5).contains(&walked), "stopped at the wall: {walked}");
    let wall = character
        .active_contacts()
        .iter()
        .filter(|contact| contact.had_collision)
        .find_map(|contact| character.contact_compound_child(contact))
        .expect("a contact with the wall");
    assert_eq!(wall.user_data, STRUCTURE);
    let finished = character.save_state();

    // A world built the same way, restored from the saved state, ends the same, bit for bit.
    let (mut replay, replay_id) = build(north)?;
    replay.character_mut(replay_id)?.restore_state(&saved.0)?;
    let mut vel_up = saved.1;
    for _ in 0..60 {
        tick(&mut replay, replay_id, &mut vel_up)?;
    }
    assert_eq!(replay.character(replay_id)?.save_state(), finished);

    // Half a second of walking from the south pole and from the four equator points stays on
    // the ground and covers about a metre.
    for start in [
        Vec3::new(0.0, -1.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(-1.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(0.0, 0.0, -1.0),
    ] {
        let (mut world, id) = build(start)?;
        let mut vel_up = 0.0;
        for _ in 0..30 {
            tick(&mut world, id, &mut vel_up)?;
        }
        let character = world.character(id)?;
        assert_eq!(character.ground_state(), GroundState::OnGround);
        let p = character.position();
        assert!(height(p).abs() < 0.01, "on the ground from {start:?}: {p:?}");
        let ground = scale(start, PLANET_RADIUS);
        let moved = Vec3::new(p.x as f32 - ground.x, p.y as f32 - ground.y, p.z as f32 - ground.z);
        let moved = dot(moved, moved).sqrt();
        assert!((0.9..1.1).contains(&moved), "walked from {start:?}: {moved}");
    }
    Ok(())
}
```
