# `oxijolt` — safe Rust API for Jolt Physics
The safe API of [oxijolt](https://github.com/pockerhead/oxijolt) over the raw bindings in
[oxijolt-sys](../oxijolt-sys), built on the [joltc] C wrapper of Jolt Physics 5.6. Everything is
headless.

A `PhysicsWorld` holds:
- rigid bodies of box, sphere, cylinder, capsule, tapered capsule and cylinder, convex hull,
  triangle mesh, heightfield, compound and scaled shapes;
- scene queries;
- characters, wheeled vehicles and ragdolls;
- twelve kinds of constraints;
- soft bodies;
- contact and activation events, and a contact listener.

A world's state can be saved and restored, and the world moved to a floating origin. Jolt's jobs run
on Jolt's thread pool or on the caller's. The repository's README says what is not bound yet, and
its `docs/` directory has the guides.

`use oxijolt::prelude::*` brings in the commonly used types; the math types are in
`oxijolt::prelude::math`.

## Features
- `double-precision`: world positions in `f64`; forwards to `oxijolt-sys/double-precision`.
- `cross-platform-deterministic`: forwards to `oxijolt-sys/cross-platform-deterministic`.
- `debug-renderer`: forwards to `oxijolt-sys/debug-renderer` and enables `PhysicsWorld::debug_lines`.
- `asserts`: Jolt's debug assertions (`oxijolt-sys/asserts`); a failed assertion prints its message
  and aborts the process. The one exception is the physics-update-error assertion: the step goes
  on and its `StepReport` says what was dropped.
- `bindgen`: generates the raw bindings with libclang at build time (`oxijolt-sys/bindgen`).

[joltc]: https://github.com/amerkoleci/joltc
