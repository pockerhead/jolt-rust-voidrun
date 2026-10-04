# `oxijolt` — safe Rust API for Jolt Physics
The safe API over [oxijolt-sys](../oxijolt-sys), built on the [joltc] raw layer (Jolt Physics
5.6). It offers a physics world with configurable collision layers; rigid bodies with poses,
velocities, forces and sleeping; box, sphere, cylinder, capsule, heightfield and compound shapes;
ray casts, shape casts and collide-shape queries with layer and group filters; a floating-origin
rebase; a virtual character; and, with `debug-renderer`, debug lines as data. See the repository's
[guide](../../docs/guide.md) and README.

## Features
- `double-precision`: forwards to `oxijolt-sys/double-precision`
- `cross-platform-deterministic`: forwards to `oxijolt-sys/cross-platform-deterministic`
- `debug-renderer`: forwards to `oxijolt-sys/debug-renderer` and enables `PhysicsWorld::debug_lines`

[joltc]: https://github.com/amerkoleci/joltc
