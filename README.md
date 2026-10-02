# joltphysics
[![CI](https://github.com/pockerhead/jolt-rust-voidrun/actions/workflows/ci.yml/badge.svg)](https://github.com/pockerhead/jolt-rust-voidrun/actions/workflows/ci.yml)
[![License: MIT OR Apache-2.0](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue.svg)](#license)
[![Jolt Physics 5.6.0](https://img.shields.io/badge/Jolt%20Physics-5.6.0-orange.svg)](https://github.com/jrouwe/JoltPhysics/releases/tag/v5.6.0)

Rust bindings for [Jolt Physics](https://github.com/jrouwe/JoltPhysics) 5.6.0 through the [joltc] C
wrapper: raw `unsafe` bindings and a safe, idiomatic API on top. The safe API is written for a
deterministic, headless simulation that a game's ECS owns: rigid bodies, terrain heightfields,
compound shapes with per-child collision groups, scene queries, a floating origin and a virtual
character. Everything runs and is tested without a window.

The project is young. The API changes between versions, and only Windows (MSVC) is built and
tested in CI.

## Origin
The repository started as a fork of jolt-rust. Of its code only the workspace layout, the licences
and a few `build.rs` details (the Android NDK cross toolchain, the switch behind the
`cross-platform-deterministic` feature) are left. The crates were renamed (`joltc-sys` became
`joltphysics-sys`, `rolt` became `joltphysics`), the raw layer sits on a different C wrapper
([joltc] instead of JoltC) and the safe API was written from scratch. What carries over is the
design: a raw crate and a safe crate, Jolt built by CMake from cargo, and JoltC's soundness rules.
[LINEAGE.md](LINEAGE.md) credits the projects this work builds on.

## Crates

### `joltphysics-sys`: raw bindings
`bindgen` output over joltc's `joltc.h` and this repository's extension `native/joltc_ext/joltc_ext.h`,
with joltc's `JPH_*` names. joltc and Jolt are pinned submodules, compiled unmodified; the extension
adds a few functions in joltc's naming (character state save and restore, character updates with
explicit filters and temp allocator). The FFI structs in use have their layout asserted on the C++
side (`native/layout_checks.cpp`) and the Rust side (`src/layout.rs`). The crate docs list joltc's
own rules for using the raw API.

Features:
- `asserts`: compile Jolt with its debug assertions.
- `double-precision`: world positions in `f64` (`Real`, `JPH_RVec3`).
- `cross-platform-deterministic`: build Jolt with `CROSS_PLATFORM_DETERMINISTIC` (see
  [Determinism](#determinism)).
- `debug-renderer`: compile Jolt's debug renderer and bind joltc's drawing functions.

### `joltphysics`: safe API
A `PhysicsWorld` owns a Jolt physics system with its collision layers, job system and temp
allocator. Changing a world takes `&mut`, reading it (body state, queries) takes `&`, and worlds
are `Send` and `Sync`. Inputs Jolt only checks with debug assertions are validated and reported as
typed errors.

Features: `double-precision`, `cross-platform-deterministic` and `debug-renderer` forward to
`joltphysics-sys`; `debug-renderer` also enables `PhysicsWorld::debug_lines`.

Start with the [guide](docs/guide.md) (a terrain, a chunk compound, an item, queries and a rebase,
run as a test), the crate docs (`cargo doc -p joltphysics --open`) and the `hello_world` example
(`cargo run -p joltphysics --example hello_world`).

## What is bound and tested
Tests are in `crates/joltphysics/tests/` unless a path says otherwise; `src/...` names a unit test
in `crates/joltphysics/src/`.

| Feature | Safe API | Tests |
|---|---|---|
| World with settings and validation | `PhysicsWorld::new`, `WorldSettings` | `world.rs`: `default_world_steps`, `invalid_settings_are_rejected`, `gravity_round_trips_including_zero` |
| Step with a report of dropped work | `PhysicsWorld::step`, `StepReport`, `StepError` | `world.rs`: `step_rejects_bad_delta_time`, `full_contact_constraint_buffer_is_reported_and_the_world_advances`, `full_body_pair_cache_is_reported_and_the_world_advances` |
| Collision layers | `CollisionLayers`, `ObjectLayer`, `BroadPhaseLayer` | `dynamics.rs`: `collision_layers_decide_which_bodies_touch`; `src/layers.rs`: `validate_rejects_inconsistent_tables` |
| Several worlds, threads | `PhysicsWorld: Send + Sync`, `WorldSettings::worker_threads` | `dynamics.rs`: `two_worlds_side_by_side_do_not_interfere`, `worlds_step_in_parallel_threads`; `world.rs`: `world_and_shape_are_send_and_sync`, `world_is_readable_from_many_threads` |
| Static, kinematic and dynamic bodies, ids | `BodySettings`, `create_body`, `BodyId` | `bodies.rs`: `ids_follow_insertion_order`, `foreign_and_stale_ids_are_rejected`, `invalid_body_settings_are_rejected`, `full_world_rejects_another_body` |
| Pose and velocity, read and write | `BodyRef`, `BodyMut` | `bodies.rs`: `pose_and_velocity_read_back_as_exact_bits` |
| Forces, torque, reset | `BodyMut::add_force`, `add_force_at_point`, `add_torque`, `reset_forces` | `bodies.rs`: `forces`, `reset_forces_ignores_static_and_kinematic_bodies` |
| Sleeping and active state | `BodyRef::is_sleeping`, `is_active`, `Activation` | `bodies.rs`: `sleeping_flag_is_readable` |
| Removal wakes the bodies around it | `PhysicsWorld::remove_body` | `bodies.rs`: `removal_wakes_bodies_resting_on_it`, `removal_wakes_the_same_bodies_whether_or_not_the_broad_phase_was_optimized` |
| Mass override, continuous collision, gravity factor | `BodySettings::mass`, `motion_quality`, `gravity_factor` | `bodies.rs`: `mass_too_small_to_invert_is_rejected`; `dynamics.rs`: `linear_cast_body_does_not_tunnel_through_thin_static`, `item_settles_under_caller_radial_gravity` |
| Enhanced internal edge removal (setting passed to Jolt; its effect on contacts is not tested) | `BodySettings::enhanced_internal_edge_removal` | `src/body.rs`: `enhanced_internal_edge_removal_reaches_the_body` (reads the flag back from the Jolt body) |
| Box, sphere, Y-cylinder, Y-capsule, convex radius | `Shape::new_box`, `new_box_with_convex_radius`, `new_sphere`, `new_cylinder`, `new_capsule` | `shapes.rs`: `sharp_box_edge_ray_hits_the_exact_corner`, `convex_radius_rounds_box_edges_for_contacts`; `src/shape.rs`: `cylinder_and_capsule_dimensions_reach_jolt`, `invalid_dimensions_are_rejected` |
| Heightfield (n = 33, holes, block size; active-edge threshold passed to Jolt, its effect on contacts not tested) | `Shape::new_height_field`, `HeightFieldSettings` | `shapes.rs`: `height_field_33_builds_and_matches_samples_at_nodes`, `height_field_rising_along_z_matches_analytic_surface`, `height_field_of_holes_has_no_collision`, `height_field_with_custom_active_edge_threshold_builds`, `static_only_shapes_are_rejected_for_moving_bodies` |
| Compound with per-child pose and user data | `Shape::new_compound`, `CompoundChild`, `CompoundSubShape` | `shapes.rs`: `compound_children_report_their_user_data`, `child_pose_is_applied`, `single_child_compound_keeps_its_user_data` |
| Shapes shared between bodies and worlds | `Shape` | `shapes.rs`: `shapes_are_shared_across_bodies_and_worlds`; `bodies.rs`: `shape_can_be_dropped_after_body_creation` |
| Ray cast | `PhysicsWorld::cast_ray`, `RayCast`, `RayHit` | `queries.rs`: `ray_hits_chunk_and_terrain_from_twenty_metres`, `ray_starting_inside_hits_at_exactly_zero`, `ray_normals_point_out_of_the_hit_surface`; `shapes.rs`: `height_field_is_hit_from_below` |
| Shape cast | `PhysicsWorld::cast_shape`, `ShapeCast`, `ShapeCastHit` | `queries.rs`: `capsule_cast_down_reports_floor_normal_and_distance`, `rotated_capsule_cast_uses_its_rotation`, `target_distance_stops_short`, `start_penetrating_is_reported_moving_in_not_out` |
| Collide shape | `PhysicsWorld::collide_shape`, `CollideShape`, `CollideShapeHit` | `queries.rs`: `collide_capsule_with_floor_reports_geometric_depth`, `collide_ceiling_normal_points_down`, `max_separation_reports_negative_depth` |
| Query filters: layers, child groups, excluded body | `QueryFilter` | `queries.rs`: `spawn_ground_ignores_canopy_by_group`, `object_layer_selection_skips_other_layers`, `excluded_body_is_skipped`, `nested_compound_group_governs_shape_casts_and_collide`, `filter_rejects_unknown_layer_and_foreign_body` |
| Queries without a step, broad-phase optimisation | `PhysicsWorld::optimize_broad_phase` | `queries.rs`: `queries_see_created_moved_and_removed_bodies_without_a_step`, `optimize_broad_phase_keeps_query_results` |
| Queries from many threads | queries on `&PhysicsWorld` | `queries.rs`: `filtered_queries_run_in_parallel`; `world.rs`: `rays_are_cast_from_many_threads` |
| Floating origin | `PhysicsWorld::rebase` | `rebase.rs`: `resting_item_stays_across_a_rebase`, `sleeping_body_stays_asleep_across_a_rebase`, `drift_rebase_equals_the_scene_built_in_the_new_frame`, `rays_answer_the_same_across_a_rebase`, `invalid_rebase_changes_nothing` |
| Same results for any thread count | (whole API) | `determinism.rs`: `stacks_digest_is_identical_across_thread_counts`, `chunk_digest_is_identical_across_thread_counts`, `permuted_insertion_order_fails_the_gate`; `crates/joltphysics-sys/tests/determinism.rs`: `digest_is_identical_across_thread_counts` |
| Debug wireframe as line data (`debug-renderer`) | `PhysicsWorld::debug_lines`, `DebugLines`, `DebugLineSettings` | `debug_lines.rs`: `near_colliders_present_far_absent_terrain_present`, `line_cap_gives_exactly_the_cap_and_truncated`, `hidden_groups_vanish`, `compound_child_pose_applies_child_rotation_before_body_rotation`; `debug_lines_leaks.rs`; `crates/joltphysics-sys/tests/debug_renderer_bindings.rs` |
| Leak gate for per-call joltc objects (Windows; bounded memory growth, catches only leaks above its threshold) | queries, rebase | `leaks.rs`: `per_call_joltc_objects_do_not_leak` |
| Virtual character | `create_character`, `update_character`, `CharacterSettings` | `character.rs`: `a_character_lands_on_a_floor_and_reports_it`, `a_chained_replay_of_one_character_is_bit_exact`; `walker.rs`; `character_leaks.rs`; `crates/joltphysics-sys/tests/character_smoke.rs` |
| Raw bindings | `joltphysics-sys` | `crates/joltphysics-sys/tests/smoke_test.rs`: `box_falls_onto_static_box`; layout checks at compile time in `crates/joltphysics-sys/src/layout.rs` |
| Guide example | [docs/guide.md](docs/guide.md) | doctest `Guide` in `crates/joltphysics/src/lib.rs` (`cargo test -p joltphysics --doc`) |

CI runs the test suite in four configurations: default, `cross-platform-deterministic`,
`double-precision` and `debug-renderer`. The tests behind the `debug-renderer` feature run only in
the last one.

Not in the safe API yet (joltc exposes them, so `joltphysics-sys` has them): mesh and convex hull
shapes, constraints and motors, vehicles, ragdolls and skeletons, soft bodies, and contact and
activation listeners. Saving and restoring the state of bodies is in neither layer yet; only a
character's state can be saved.

## Guarantees and limits
- **Validation.** Values Jolt only checks with debug assertions (non-finite poses, non-unit
  quaternions, zero dimensions, ids of another world, unknown layers) are rejected with a typed
  error before they reach Jolt. The rule is that an `Err` means the call changed nothing; a step
  that ran but dropped contacts returns `Ok` with a flag in its `StepReport`.
- **Ids.** A `BodyId` is Jolt's index and 8-bit sequence number. The sequence wraps after 255
  reuses of one index, so a very old id can name a new body.
- **Queries.** Ray casts see a box's sharp faces whatever its convex radius; contacts and shape
  casts use at most 0.05 m of a convex radius. Heightfields cannot be query shapes, and only
  spheres and capsules take a shape-cast target distance. `collide_shape` hits come in no
  particular order.
- **Debug lines.** No level of detail: one capsule draws 6528 lines and one cylinder 768 at any
  distance, and each call builds a new Jolt debug renderer. Calls are serialized process-wide. A
  character is drawn only through its inner body.
- **Global state.** `joltphysics` calls `JPH_Init` once per process and never `JPH_Shutdown`. It
  installs joltc's filter procs (and with `debug-renderer` its debug renderer procs) once and owns
  them; code that also uses `joltphysics-sys` directly must leave them alone.
- **Platforms.** CI builds and tests Windows MSVC only. The build script has code for other hosts
  and for Android cross builds, but nothing checks them.

## Determinism
What is guaranteed: on one machine, the same binary given the same calls in the same order produces
bit-identical body ids, poses, velocities and sleep flags for any `WorldSettings::worker_threads`
(`worker_threads(n)` means n workers plus the thread that calls `step`). The call history includes the
order of body creation and removal, which decides each `BodyId`'s index and sequence number, as well as
rebases, `optimize_broad_phase`, forces and velocity writes. `remove_body` wakes the bodies whose exact
bounds overlap the removed one, in id order, so it adds no hidden state.

What is not guaranteed by default: equal results across compilers, compiler flags, operating systems or
CPU architectures. The `cross-platform-deterministic` feature (off by default, roughly 8 % slower) builds
Jolt with `CROSS_PLATFORM_DETERMINISTIC`. Jolt then
[claims](https://github.com/jrouwe/JoltPhysics/blob/master/Docs/Architecture.md#deterministic-simulation)
equal results across compilers, configurations, operating systems and architectures, as long as the same
source is built with the same defines: never compare a `double-precision` build with a single-precision
one, and FPU rounding and denormal (DAZ/FTZ) modes must match. This repository's gate runs on one
machine and checks thread-count equality in each configuration; it does not check equality across
machines.

Order caveats: narrow-phase query hits (`collide_shape` and friends) come in an unspecified order; sort
them by body id and sub-shape id when order matters. Code that drives the world must not feed hash-map
iteration order, time or thread identity into its calls.

How it is checked: `cargo test -p joltphysics --test determinism` runs each scene in two child
processes, with 1 and with 4 workers, records ticks 0 to 1000 of a chunk, terrain and item scene and
requires them to match byte for byte; the same bodies created in another order must fail the gate. CI
runs it in the default, `cross-platform-deterministic` and `double-precision` configurations.

## Building
Requirements: a C++ toolchain (MSVC on Windows), CMake 3.20 or newer, and LLVM/libclang for `bindgen`
(see the [bindgen guide](https://rust-lang.github.io/rust-bindgen/requirements.html)).

```bash
git submodule update --init         # joltc and Jolt
cargo build                         # builds joltc + Jolt through CMake (always Release), generates bindings
cargo test --workspace              # everything, headless
cargo test -p joltphysics --features debug-renderer   # with the debug wireframe
```

The first build compiles joltc and Jolt with CMake, always in Release. Later builds of the same target and profile reuse it; a change to the native sources, the pinned
commits, a feature or `build.rs` reruns CMake. Run one cargo process at a time on a shared machine: the C++ build is heavy. On Windows a
very long `CARGO_TARGET_DIR` can make the CMake configure step fail (path length limit).

### Prebuilt native library
Set `JOLTC_LIB_DIR` to skip CMake entirely and link a native library built earlier. It points at an
install prefix:

```
<prefix>/lib/          joltc (or joltc_double) and Jolt static libraries
<prefix>/include/joltc.h
<prefix>/include/joltc_ext.h
<prefix>/joltphysics-sys-manifest.txt
```

A normal build leaves such a prefix in `target/<profile>/build/joltphysics-sys-*/out/joltc`. A prefix is tied to
the target, the C runtime, the crate features and the pinned joltc and Jolt commits. The build script
checks all of them against the manifest and refuses a prefix built for another configuration. CI
links every build after the first through such a prefix.

## Submodules
joltc and Jolt Physics are Git submodules under `crates/joltphysics-sys/vendor`, pinned to exact commits:

```bash
git submodule update --init
```

Checkouts made before the move to joltc or the crate rename run `git submodule sync && git submodule update --init` once.

## License
Licensed under either of

* Apache License, Version 2.0, ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
* MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution
Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.

[joltc]: https://github.com/amerkoleci/joltc
