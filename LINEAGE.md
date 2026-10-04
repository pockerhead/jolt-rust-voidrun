# Lineage

What this repository is built on, who wrote it, and under which licence. Credit stays with the
original authors.

| Layer | Project | Authors | Licence | Where it lives here |
|---|---|---|---|---|
| Physics engine | [Jolt Physics](https://github.com/jrouwe/JoltPhysics) 5.6 | Jorrit Rouwe and contributors | MIT | `crates/oxijolt-sys/vendor/JoltPhysics` (submodule, tag v5.6.0), built from source, unmodified |
| C wrapper | [joltc](https://github.com/amerkoleci/joltc) | Amer Koleci and contributors (the C layer of JoltPhysicsSharp, also used by LÖVR) | MIT | `crates/oxijolt-sys/vendor/joltc` (submodule, pinned commit), unmodified; input of `crates/oxijolt-sys`. Functions this repository adds to joltc, in joltc's naming, live in `crates/oxijolt-sys/native/joltc_ext/` and are compiled into the same archive |

The project began as a fork of
[SecondHalfGames/jolt-rust](https://github.com/SecondHalfGames/jolt-rust); the raw and safe layers
were written anew on joltc.

## What is left of jolt-rust as code
The workspace layout, the licence files (`LICENSE-MIT`, `LICENSE-APACHE`) and two parts of
`crates/oxijolt-sys/build.rs`: the Android NDK cross toolchain and the CMake switch behind the
`cross-platform-deterministic` feature.

## What this repository adds
- `oxijolt-sys`: bindings over joltc's `joltc.h` and the extension's `joltc_ext.h`, committed per
  ABI family so a build needs no LLVM; layout assertions for the FFI types in use on the C++ side
  (`native/layout_checks.cpp`) and the Rust side (`src/layout.rs`); a native build through CMake or
  a validated prebuilt prefix (`JOLTC_LIB_DIR`).
- The joltc extension (`native/joltc_ext/`): state recording, character updates with explicit
  filters and temp allocator, ragdoll parts and joints that keep every setting, constraint motor
  access, path, pulley and rack-and-pinion constraints, soft body functions, materials with user
  data and a soft body contact listener.
- `oxijolt`: a safe, idiomatic Rust API with headless tests. It covers the physics world and rigid
  bodies; box, sphere, cylinder, capsule, heightfield and compound shapes with materials; scene
  queries with layer and group filters; a floating-origin rebase; the virtual character controller;
  the wheeled vehicle; ragdolls; twelve kinds of world constraints; soft bodies; contact, activation
  and soft body contact events with a contact listener; world state save and restore; caller job
  systems; debug lines as data; and a magnitude policy (`oxijolt::limits`).
- Determinism gates over thread counts and job systems, leak gates, and CI on Windows and Linux in
  five configurations, one of them with Jolt's assertions on. `docs/coverage.md` lists each feature
  with its tests.

## Licence
Our additions are dual-licensed MIT OR Apache-2.0 (`LICENSE-MIT`, `LICENSE-APACHE`). Jolt Physics
and joltc remain MIT under their own `LICENSE` files.
