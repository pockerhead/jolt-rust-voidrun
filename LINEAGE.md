# Lineage

What this repository is built on, who wrote it, and under which licence. Credit stays with the
original authors; this fork adds to their work.

| Layer | Project | Authors | Licence | Where it lives here |
|---|---|---|---|---|
| Physics engine | [Jolt Physics](https://github.com/jrouwe/JoltPhysics) 5.6 | Jorrit Rouwe and contributors | MIT | `crates/joltphysics-sys/vendor/JoltPhysics` (submodule, tag v5.6.0), built from source, unmodified |
| C wrapper (current) | [joltc](https://github.com/amerkoleci/joltc) | Amer Koleci and contributors (the C layer of JoltPhysicsSharp, also used by LÖVR) | MIT | `crates/joltphysics-sys/vendor/joltc` (submodule, pinned commit), unmodified; input of `crates/joltphysics-sys`. Functions this fork adds to joltc, in joltc's naming, live in `crates/joltphysics-sys/native/joltc_ext/` and are compiled into the same archive |
| C wrapper (original) | [JoltC](https://github.com/SecondHalfGames/JoltC) | Second Half Games (made for their game *Meanwhile in Sector 80*) | MIT OR Apache-2.0 | the starting point of this fork; replaced because joltc already covers the character controller, heightfields, vehicles and ragdolls |
| Rust bindings | [jolt-rust](https://github.com/SecondHalfGames/jolt-rust) (`joltc-sys`, `rolt`) | Second Half Games and contributors | MIT OR Apache-2.0 | this repository (fork of `main`); the crates are renamed `joltphysics-sys` and `joltphysics` |

## Design we inherit
- JoltC's soundness rules: opaque handles for C++ types with unspecified layout, layout-asserted mirrors
  for FFI-compatible types, Jolt's `RefTarget` ownership conventions. We keep these rules on top of
  joltc and add layout assertions where it does not have them, without modifying joltc:
  `crates/joltphysics-sys/native/layout_checks.cpp` (C++ side) and `crates/joltphysics-sys/src/layout.rs` (Rust side).
- joltc's breadth: one flat C header covering most of Jolt.
- jolt-rust's split into raw `joltc-sys` and the ergonomic `rolt` (here `joltphysics-sys` and `joltphysics`), and its build: CMake through the
  `cmake` crate, always in Release, now with `bindgen` over joltc's `joltc.h`.

## What this fork adds
A safe, idiomatic Rust API over joltc for what the game VOIDRUN needs: the virtual character controller,
heightfield shapes, compound children with their own pose and group data, body sleep and state readout,
the wheeled vehicle, ragdolls with skeletons and constraints, determinism checks, and headless acceptance
tests for all of them. Then the features the Rust community has asked jolt-rust and JoltC for: the full
constraint set with motors, a custom job system, state save/restore, soft bodies, builds without LLVM.

## Licence of this fork
Same as jolt-rust: dual MIT OR Apache-2.0 for our additions (`LICENSE-MIT`, `LICENSE-APACHE`). Jolt
remains MIT under its own `LICENSE`.
