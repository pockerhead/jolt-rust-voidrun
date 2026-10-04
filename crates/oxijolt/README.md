# `oxijolt` — safe Rust API for Jolt Physics
The safe API of [oxijolt](https://github.com/pockerhead/oxijolt) over the raw bindings in
[oxijolt-sys](../oxijolt-sys), built on the [joltc] C wrapper of Jolt Physics 5.6. A
`PhysicsWorld` holds rigid bodies of box, sphere, cylinder, capsule, heightfield and compound
shapes, scene queries, a character controller, wheeled vehicles, ragdolls, twelve kinds of
constraints, soft bodies, contact and activation events with a contact listener, state save and
restore, and a floating-origin rebase. Jolt's jobs run on Jolt's thread pool or on the caller's.
Everything is headless. See the repository's README and the guides in its `docs/` directory.

## Features
- `double-precision`: world positions in `f64`; forwards to `oxijolt-sys/double-precision`.
- `cross-platform-deterministic`: forwards to `oxijolt-sys/cross-platform-deterministic`.
- `debug-renderer`: forwards to `oxijolt-sys/debug-renderer` and enables `PhysicsWorld::debug_lines`.
- `asserts`: Jolt's debug assertions (`oxijolt-sys/asserts`); a failed assertion prints its message
  and aborts the process.
- `bindgen`: generates the raw bindings with libclang at build time (`oxijolt-sys/bindgen`).

[joltc]: https://github.com/amerkoleci/joltc
