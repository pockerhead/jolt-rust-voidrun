# `rolt` — safe Rust API for Jolt Physics
The safe API over [joltc-sys](../joltc-sys), built on the [joltc] raw layer (Jolt Physics
5.6). It offers a physics world with configurable collision layers, box and sphere shapes, and
rigid bodies with poses, velocities, forces and sleeping.

## Features
- `double-precision`: forwards to `joltc-sys/double-precision`
- `cross-platform-deterministic`: forwards to `joltc-sys/cross-platform-deterministic`

[joltc]: https://github.com/amerkoleci/joltc
