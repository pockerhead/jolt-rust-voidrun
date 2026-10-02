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
