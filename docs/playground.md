# Playground

`examples/playground` is a program with a window that shows what `oxijolt` does, in ten small
scenes. Each scene owns its own `PhysicsWorld`, built fresh when the scene is chosen or reset. The
same scenes run without a window, for tests and CI, and record the GIFs of the README.

```sh
cargo run -p playground --release
```

The [README](../README.md#playground) says how to get there quickly and lists the keys. This page
says what each scene uses, how the modes work and how the scenes are drawn.

## Scenes

| Key | Scene | What it shows | Calls behind it |
|---|---|---|---|
| 1 | `character` | A humanoid walks a course on heightfield terrain: stairs of rounded boxes, a 30° ramp made of a triangle mesh, a conveyor belt, a kinematic ferry, and jumps. A 50° slope and crates to push stand beside it. | `CharacterSettings::humanoid`, `create_character`, `refresh_character_contacts`, `update_character` with `ExtendedUpdateSettings`, `CharacterContactListener::adjust_body_velocity`, `ground_velocity`, `move_kinematic`, `Shape::new_height_field`, `new_mesh`, `new_convex_hull` |
| 2 | `vehicles` | A car, a tank and a motorcycle on rolling terrain, and a walker; Tab switches which one the keys drive. | `VehicleSettings::car`, `TrackedVehicleSettings`, `MotorcycleSettings::bike`, `create_vehicle`, `create_tracked_vehicle`, `create_motorcycle`, `set_driver_input`, `wheel_world_transform`, `current_gear`, `lean`, `Shape::new_offset_center_of_mass` |
| 3 | `pile` | 400 bodies of nine shape kinds fall into a bin; estimated impacts flash, sleeping bodies turn grey. | box, sphere, capsule, cylinder, tapered shapes, convex hull, `Shape::scaled`, `MotionQuality::LinearCast`, `EventSettings::collision_estimates`, `ActivationEvent`, `active_body_poses_into` |
| 4 | `ragdolls` | Humanoid ragdolls fall onto terrain and stairs until they settle; a puppet follows a wave of a 22-joint animation skeleton, kinematically or with its motors while it falls. | `RagdollSettings`, `create_ragdoll`, `SettleDetector`, `SkeletonMapper::map_reverse` and `map`, `drive_to_pose_using_kinematics`, `drive_to_pose_using_motors`, `set_motion_type`, `set_pose` |
| 5 | `constraints` | A windmill, an elevator, gears, a rack and pinion and a cart on a path, all motorized; a plank bridge, a rope, a pulley and a weld; a lamp on a cone, a pendulum, a swing and a puck with locked axes. | all twelve constraint kinds, `MotorState`, `set_target_angular_velocity`, `set_target_position`, `set_target_velocity`, `HermitePath`, `AllowedDofs` |
| 6 | `soft-bodies` | A cloth pinned on four poles catches boxes, a pressurised balloon squashes, a soft cube keeps its volume. | `SoftBodySharedSettings` (generated and explicit constraints, volume constraints), `SoftBodySettings::pressure`, `create_soft_body`, `vertices_into`, `set_vertex_inverse_mass` |
| 7 | `water` | Crates of buoyancy 0.5, 1, 2 and 4, a raft and balls in a pool; a current carries what floats. | `BodyMut::apply_buoyancy_impulse`, `BuoyancySettings::fluid_velocity`, `Shape::new_compound`, `Shape::new_plane` |
| 8 | `destruction` | A wall of 48 bricks loses the bricks that cannonballs or clicks hit; each brick falls on as a body. | `MutableCompound`, `to_shape`, `BodyMut::set_shape`, `compound_sub_shape`, `CollisionEstimate`, `cast_ray` with `compound_child` |
| 9 | `contacts` | A one-way platform, a conveyor, an ice slab and a trampoline made by a contact listener; a sensor; a chain whose overlapping links do not collide; floor tiles that name their material. | `ContactListener` (`contact_validate`, `contact_added`, `contact_persisted`), `ContactSettings`, `BodySettings::sensor`, `GroupFilterTableBuilder`, `CollisionGroup`, `PhysicsMaterial`, `ContactManifold::materials` |
| 0 | `queries` | Under the cursor: a ray cast, a sphere cast, a sphere overlap and a point test in a small town; pushing and toggling crates, saving and restoring, moving the origin, the collider wireframe. | `cast_ray`, `cast_shape`, `collide_shape`, `collide_point`, `add_impulse_at_point`, `set_motion_type`, `save_state`, `restore_state`, `rebase`, `debug_lines` |

What is not shown, because it has nothing to draw: Jolt's jobs on a caller's thread pool, `f64`
positions, the `glam` and `mint` conversions, the prelude and the error type.

## Keys

These keys work in every scene:

| Keys | Action |
|---|---|
| 1 to 9, 0 | choose a scene |
| R | reset the scene |
| P | pause |
| N | one tick while paused |
| G | collider wireframe (with the `debug-renderer` feature) |
| H | hide or show the help panel |
| right drag, wheel | orbit and zoom the camera |
| Esc | quit |

Each scene adds its own:

| Scene | Keys |
|---|---|
| `character` | W A S D walk relative to the camera, Shift sprint, Space jump |
| `vehicles` | Tab switches walker, car, tank, motorcycle; W S throttle (against the motion it brakes first); A D steer, the tank turns on the spot when standing; Space hand brake, or jump when walking |
| `pile` | Space drops another layer, F fires a heavy ball at the cursor |
| `ragdolls` | Space drops another ragdoll, M switches the puppet between kinematic and motor-driven |
| `constraints` | Up and Down change the windmill motor's speed, E sends the elevator up or down |
| `soft-bodies` | E releases the cloth's pins, F throws a ball at the cursor |
| `water` | C turns the current on or off, Space drops another crate |
| `destruction` | F fires a cannonball at the cursor, a left click knocks out the brick under it |
| `contacts` | Space drops a box over every station, E swings the chain |
| `queries` | the mouse moves the queries, a left click pushes the body under it, T switches the clicked crate between dynamic and kinematic, K saves, L restores, B moves the origin to the cursor |

The window runs the scenes at a fixed step of 60 Hz, at most four ticks per frame (a slower frame
drops the rest), and hands each key press to exactly one tick.

## Without a window

```sh
cargo run -p playground --no-default-features -- --headless --scene all --frames 600 --threads 4
```

runs each scene's script for 600 ticks and prints one line per scene with the bodies at the end and
a digest of everything the scene simulated and drew. The digest folds typed values: poses, vertex
positions, wheel and character state, and the scene's own control state, with body ids as raw
numbers. CI runs every scene with 1 and with 4 worker threads and checks that the lines match, and
`cargo test -p playground --no-default-features` checks the same in one process, along with resets,
pose syncing, the draw-call limits and the GIF encoder. A run as long as a scene's clip fails when
the scene misses one of its milestones, the things its clip must show.

## Recording the media

```sh
cargo run -p playground --release -- --record all
```

writes `docs/media/<scene>.gif` and `<scene>.png` for every scene; `--record <scene>` records one,
`--out <dir>` writes elsewhere. A recording runs the scene's script at the fixed step and renders
every second tick from the scene's fixed record camera into a 720 x 406 render target. Each frame is
read back, halved to 360 x 203, mapped through a fixed palette of 256 colours (a 6 x 6 x 6 colour
cube and greys) and stored as only the rectangle that changed since the last frame, at 30 frames per
second. The PNG still is the middle frame at 240 pixels wide. Recording fails when the clip misses a
milestone, when a GIF passes 1.5 MB, or when all of them together pass 12 MB. The simulation is
deterministic, but the GPU's rendering is not promised to give the same pixels on another machine.

## How the scenes draw

Nothing in `oxijolt` knows about drawing. A scene reports what the binding gives it: the poses of
its bodies (synced after each step from `active_body_poses_into` and the bodies that fell asleep),
wheel poses from `wheel_world_transform`, soft body vertices, the character's position and up, and
debug lines. The binding has no shape introspection, so the playground keeps its own description of
each shape it creates next to the `Shape`, and turns it into a flat-shaded triangle mesh once. Each
frame those meshes are posed and shaded on the CPU and sent to the GPU in draw calls of at most
19 999 triangles, below the 60 000 vertices per draw call the window configures.

## Features

| Feature | Default | Effect |
|---|---|---|
| `window` | on | the window and the recorder ([macroquad](https://crates.io/crates/macroquad)); without it only `--headless` runs |
| `debug-renderer` | on | `oxijolt/debug-renderer`: the collider wireframe |
| `asserts` | off | `oxijolt/asserts`: Jolt's debug assertions; a failed one aborts |
