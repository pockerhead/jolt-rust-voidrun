# Changelog

All notable changes to this fork. The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## Unreleased

- Project rules (`AGENTS.md`) and lineage (`LINEAGE.md`) for the fork.
- The raw layer moved from JoltC to [joltc](https://github.com/amerkoleci/joltc) over Jolt Physics 5.6.0:
  `joltc-sys` now exposes joltc's `JPH_*` API. joltc and Jolt are pinned submodules under
  `crates/joltc-sys/vendor`, and the native build no longer fetches anything from the network.
- Layout assertions for the FFI types in use, on the C++ side and the Rust side.
- Eight joltc ragdoll and skeleton-mapper functions that cast 4-aligned matrices to 16-aligned ones are
  left out of the bindings.
- Removed the `object-layer-u32` feature: joltc always uses 32-bit object layers.
- `JOLTC_LIB_DIR` links a prebuilt native library and skips CMake; the prefix is validated against a
  manifest. Rust-only changes no longer rerun CMake.
- `rolt` is empty and the `hello-world` examples are removed until the new safe API lands.
- CI on GitHub Actions (Windows MSVC: build, test, clippy, docs, formatting) with a cached native build.
