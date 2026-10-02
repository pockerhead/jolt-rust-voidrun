# `rolt` — safe Rust API for Jolt Physics
The safe API over [joltc-sys](../joltc-sys) is being rebuilt on the [joltc] raw layer
(Jolt Physics 5.6). This version contains no public items; use `joltc-sys` directly in the
meantime.

## Features
- `double-precision`: forwards to `joltc-sys/double-precision`
- `cross-platform-deterministic`: forwards to `joltc-sys/cross-platform-deterministic`

[joltc]: https://github.com/amerkoleci/joltc
