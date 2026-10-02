# Changelog

All notable changes to this fork. The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## Unreleased

- Renamed the crates: `joltc-sys` is now `joltphysics-sys` (`crates/joltphysics-sys`, Rust path
  `joltphysics_sys`) and `rolt` is now `joltphysics` (`crates/joltphysics`); the old names belong to
  jolt-rust on crates.io. The prebuilt manifest is now `joltphysics-sys-manifest.txt`; `JOLTC_LIB_DIR` and
  the features are unchanged. Existing checkouts run `git submodule sync && git submodule update --init` once.
- Project rules (`AGENTS.md`) and lineage (`LINEAGE.md`) for the fork.
- The raw layer moved from JoltC to [joltc](https://github.com/amerkoleci/joltc) over Jolt Physics 5.6.0:
  `joltphysics-sys` now exposes joltc's `JPH_*` API. joltc and Jolt are pinned submodules under
  `crates/joltphysics-sys/vendor`, and the native build no longer fetches anything from the network.
- Layout assertions for the FFI types in use, on the C++ side and the Rust side.
- Eight joltc ragdoll and skeleton-mapper functions that cast 4-aligned matrices to 16-aligned ones are
  left out of the bindings.
- Removed the `object-layer-u32` feature: joltc always uses 32-bit object layers.
- `JOLTC_LIB_DIR` links a prebuilt native library and skips CMake; the prefix is validated against a
  manifest. Rust-only changes no longer rerun CMake.
- `joltphysics` is a new safe API on the joltc raw layer: a physics world with collision layers, box and
  sphere shapes and rigid bodies, with a headless `hello_world` example.
- CI on GitHub Actions (Windows MSVC: build, test, clippy, docs, formatting) with a cached native build.
- `PhysicsWorld::step` returns `Result<StepReport, StepError>`. `Err` now means the step was rejected and
  the world did not advance (`StepError::InvalidDeltaTime`); a step that ran but dropped contacts because
  a fixed-size Jolt buffer was full returns `Ok` with the matching `StepReport` flag set
  (`StepReport::is_complete` is false). `StepError::CacheFull` is gone. The rule for the safe API: `Err`
  means nothing happened; anything that happened but was degraded is reported in the `Ok` value.
- Bodies: kinematic bodies, mass override, continuous collision (`MotionQuality::LinearCast`),
  gravity factor, enhanced internal edge removal, forces at a point and torques, `reset_forces`,
  and sleep and active readout. Invalid values are rejected with typed errors before they reach Jolt.
- `WorldSettings::worker_threads` accepts 1 to `WorldSettings::MAX_WORKER_THREADS` (64).
  `PhysicsWorld` is `Send` and `Sync`: many threads may read one world, and separate worlds step in
  parallel.
- `PhysicsWorld::remove_body` wakes the non-static bodies whose bounds overlap the removed one, in
  body-id order, so a stack whose bottom is removed falls.
- Shapes: Y-cylinders and Y-capsules, boxes and cylinders with a chosen convex radius (0 for sharp
  edges), heightfields (`Shape::new_height_field`, `HeightFieldSettings`: holes, block size, bits per
  sample, active-edge threshold) and compounds whose children carry their own pose and user data
  (`Shape::new_compound`, `CompoundChild`). One shape serves many bodies in many worlds.
- Scene queries on `&PhysicsWorld`: `cast_ray`, `cast_shape` (with target distance, start
  penetration and deepest point) and `collide_shape`, all filtered by `QueryFilter` (object layers,
  compound child groups, one excluded body). Hits report the outward normal of the obstacle and the
  compound child that was hit. `optimize_broad_phase` makes queries fast after many single inserts.
- `PhysicsWorld::rebase` moves the whole world into a new frame (floating origin) without waking or
  putting to sleep any body.
- A determinism gate: scenes run in two processes with 1 and 4 worker threads must match bit for
  bit for 1000 ticks, and another creation order must not. CI runs it, and the whole suite, in the
  default, `cross-platform-deterministic` and `double-precision` configurations.
- `debug-renderer` feature: `PhysicsWorld::debug_lines` returns the wireframe of the colliders around
  a point as line data, with layer and group filters and a line cap. Jolt's debug renderer is left
  out of the native libraries without the feature.
- A guide (`docs/guide.md`) with a headless example of a terrain, a chunk compound, an item, queries
  and a rebase, run as a doctest.
- Benchmarks against the game's budgets (`cargo bench -p joltphysics --bench budgets`, results in
  `docs/benchmarks.md`); the `character_cost` example moved into this bench.
