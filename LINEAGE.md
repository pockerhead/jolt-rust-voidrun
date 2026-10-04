# Lineage

What this repository is built on, who wrote it, and under which licence. Credit stays with the
original authors.

| Layer | Project | Authors | Licence | Where it lives here |
|---|---|---|---|---|
| Physics engine | [Jolt Physics](https://github.com/jrouwe/JoltPhysics) 5.6 | Jorrit Rouwe and contributors | MIT | `crates/oxijolt-sys/vendor/JoltPhysics` (submodule, tag v5.6.0), built from source, unmodified |
| C wrapper (current) | [joltc](https://github.com/amerkoleci/joltc) | Amer Koleci and contributors (the C layer of JoltPhysicsSharp, also used by LÖVR) | MIT | `crates/oxijolt-sys/vendor/joltc` (submodule, pinned commit), unmodified; input of `crates/oxijolt-sys`. Functions this fork adds to joltc, in joltc's naming, live in `crates/oxijolt-sys/native/joltc_ext/` and are compiled into the same archive |
| C wrapper (original) | [JoltC](https://github.com/SecondHalfGames/JoltC) | Second Half Games (made for their game *Meanwhile in Sector 80*) | MIT OR Apache-2.0 | the starting point of this fork; replaced because joltc already covers the character controller, heightfields, vehicles and ragdolls |
| Rust bindings | [jolt-rust](https://github.com/SecondHalfGames/jolt-rust) (`joltc-sys`, `rolt`) | Second Half Games and contributors | MIT OR Apache-2.0 | the starting point of this repository (a fork of its `main`); see below for what is left of it |

## What is left of jolt-rust as code
Only the workspace layout, the licences and a few `build.rs` details: the Android NDK cross
toolchain and the CMake switch behind the `cross-platform-deterministic` feature. The crates were
renamed (`joltc-sys` became `oxijolt-sys`, `rolt` became `oxijolt`; before the first release they were
briefly `joltphysics-sys` and `joltphysics`), the raw layer sits on joltc instead of JoltC, and the
safe API was written from scratch. What this repository inherits from jolt-rust and JoltC is design,
listed in the next section.

## Design we inherit
- JoltC's soundness rules: opaque handles for C++ types with unspecified layout, layout-asserted mirrors
  for FFI-compatible types, Jolt's `RefTarget` ownership conventions. We keep these rules on top of
  joltc and add layout assertions where it does not have them, without modifying joltc:
  `crates/oxijolt-sys/native/layout_checks.cpp` (C++ side) and `crates/oxijolt-sys/src/layout.rs` (Rust side).
- joltc's breadth: one flat C header covering most of Jolt.
- jolt-rust's split into raw `joltc-sys` and the ergonomic `rolt` (here `oxijolt-sys` and `oxijolt`), and its build: CMake through the
  `cmake` crate, always in Release, now with `bindgen` over joltc's `joltc.h`.

## What this repository adds
A safe, idiomatic Rust API over joltc for what the game VOIDRUN needs, with headless tests. It has
the physics world and rigid bodies with sleep and state readout, box, sphere, cylinder, capsule,
heightfield and compound shapes (children with their own pose and group data), scene queries with
layer and group filters, a floating-origin rebase, a determinism gate, debug lines as data and the
virtual character controller. The README lists each with its tests. Planned next: the wheeled
vehicle and ragdolls with skeletons and constraints, then the features the Rust community has asked
jolt-rust and JoltC for: the full constraint set with motors, a custom job system, state
save/restore, soft bodies, builds without LLVM.

## Licence
Same as jolt-rust: dual MIT OR Apache-2.0 for our additions (`LICENSE-MIT`, `LICENSE-APACHE`). Jolt
remains MIT under its own `LICENSE`.
