<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="docs/assets/oxijolt-logo-dark.png">
    <img alt="oxijolt" src="docs/assets/oxijolt-logo-light.png" width="420">
  </picture>
</p>

# oxijolt — Rust bindings for Jolt Physics
[![CI](https://github.com/pockerhead/oxijolt/actions/workflows/ci.yml/badge.svg)](https://github.com/pockerhead/oxijolt/actions/workflows/ci.yml)
[![License: MIT OR Apache-2.0](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue.svg)](#license)
[![Jolt Physics 5.6.0](https://img.shields.io/badge/Jolt%20Physics-5.6.0-orange.svg)](https://github.com/jrouwe/JoltPhysics/releases/tag/v5.6.0)

Rust bindings for [Jolt Physics](https://github.com/jrouwe/JoltPhysics) 5.6: a safe API with
rigid bodies, a character controller, wheeled vehicles, ragdolls, soft bodies, constraints and
contact events, built for deterministic simulation. Underneath are raw bindings to the [joltc] C
wrapper. Everything runs headless and is tested without a window.

## Example

```rust
use oxijolt::prelude::math::*;
use oxijolt::prelude::*;

fn main() -> oxijolt::error::Result<()> {
    let mut world = PhysicsWorld::new(WorldSettings::default())?;

    // A static floor whose top is at y = 0, and a ball dropped onto it.
    let floor = Shape::new_box(Vec3::new(50.0, 1.0, 50.0))?;
    world.create_body(&floor, &BodySettings::new_static().position(RVec3::new(0.0, -1.0, 0.0)))?;
    let ball_shape = Shape::new_sphere(0.5)?;
    let ball = world.create_body(
        &ball_shape,
        &BodySettings::new_dynamic().position(RVec3::new(0.0, 4.0, 0.0)),
    )?;

    for _ in 0..120 {
        // A step that had to drop contacts says so; raise a `WorldSettings` limit then.
        let report = world.step(1.0 / 60.0)?;
        assert!(report.is_complete(), "{report:?}");
    }
    println!("the ball rests at {:?}", world.body(ball)?.position());

    // A ray cast down from above hits the ball first.
    let ray = RayCast::new(RVec3::new(0.0, 10.0, 0.0), Vec3::new(0.0, -20.0, 0.0));
    let hit = world.cast_ray(ray, &QueryFilter::new())?.expect("the ray hits");
    assert_eq!(hit.body, ball);
    Ok(())
}
```

With Bevy, `use oxijolt::prelude::*` next to `bevy::prelude::*`; see `oxijolt::prelude`.

```toml
[dependencies]
oxijolt = { git = "https://github.com/pockerhead/oxijolt" }
```

## Features

- Rigid bodies: static, kinematic and dynamic; box, sphere, cylinder, capsule, heightfield and
  compound shapes; physics materials with user data; forces, sleeping, continuous collision.
- Scene queries: ray casts, shape casts and collide-shape, filtered by layer, compound child
  group and body.
- A character controller (Jolt's `CharacterVirtual`) with stair walking, floor sticking and an up
  direction that can change every update, for walking on a planet.
- Wheeled vehicles with suspension, engine, automatic transmission, differentials and anti-roll
  bars.
- Ragdolls from a skeleton, posed, motor-driven or kinematic, with settle detection.
- Twelve kinds of constraints with motors, springs and limits: fixed, point, distance, hinge,
  slider, cone, swing-twist, six-DOF, gear, rack and pinion, pulley and path
  ([guide](docs/constraints.md)).
- Soft bodies: cloth, pressurised and volume-preserving bodies, with vertex readout and pinning
  ([guide](docs/soft-bodies.md)).
- Contact, activation and soft body contact events in an order that does not depend on the
  thread count, and a contact listener that changes friction, restitution, mass scales or surface
  velocity per contact ([guide](docs/events.md)).
- Saving and restoring a world's state for rollback and replays ([guide](docs/state.md)).
- A floating origin (`PhysicsWorld::rebase`) and optional `f64` world positions.
- Jolt's jobs on Jolt's thread pool or on your own, such as Rayon ([guide](docs/job-system.md)).
- Debug wireframes as line data (feature `debug-renderer`); nothing is drawn.

Not in the safe API yet:

- mesh, convex hull, scaled and tapered shapes;
- impulses on rigid bodies;
- moving a kinematic body to a target;
- activating or deactivating a body on demand;
- sensor bodies;
- changing a body's shape or motion type after creation.

The raw layer, `oxijolt-sys`, has the joltc functions for all of them.
[docs/coverage.md](docs/coverage.md) lists every bound feature with the tests that check it, and
the rest of what is not bound yet.

## Status

- Version 0.4.0, not released and not on crates.io yet. The API changes between versions.
- CI builds and tests Windows (MSVC) and Linux (GCC) on x86_64, each in five configurations:
  default, `cross-platform-deterministic`, `double-precision`, `debug-renderer` and `asserts`.
  The bindings are committed for 64-bit Windows, Linux, macOS and Android targets; only the two
  CI targets are tested.
- Building needs a C++ toolchain and CMake, not LLVM. Jolt and joltc are compiled from pinned
  submodules; `JOLTC_LIB_DIR` links a prebuilt native library instead
  ([building](docs/building.md)).

## Guarantees and limits

- **Errors.** A call that returns `Err` was refused and changed nothing, unless its documentation
  says otherwise. A step that ran but dropped contacts returns `Ok` and says so in its
  `StepReport`.
- **Magnitudes.** Values Jolt checks only with debug assertions (non-finite poses, non-unit
  quaternions, zero dimensions, ids of another world) and magnitudes outside `oxijolt::limits`
  are refused with a typed error before they reach Jolt. Not every accepted value is proven safe:
  [limits](docs/limits.md) shows how the bounds follow from Jolt's arithmetic, and
  [coverage](docs/coverage.md) says which values only tests check and which no bound covers. CI
  also runs every test with Jolt's assertions on.
- **Determinism.** On one machine the tests get bit-identical results with 1 and 4 worker threads
  and with caller job systems, under Jolt's conditions (same binary, same calls in the same
  order). Across platforms and compilers results agree only with `cross-platform-deterministic`
  ([determinism](docs/determinism.md)).
- **Threads.** Changing a world takes `&mut`, reading it takes `&`; worlds are `Send` and `Sync`.
- **Global state.** oxijolt initialises Jolt once per process and owns joltc's process-global
  filter, listener, debug renderer and assertion procs; code that also uses `oxijolt-sys` directly
  must leave them alone.

## Compared with rolt

[rolt](https://crates.io/crates/rolt) and joltc-sys, from Second Half Games, are the Rust
bindings listed in Jolt's README. On crates.io they are at 0.3.1 with Jolt 5.0.0 (May 2024), and
they have no character controller, vehicles, ragdolls or heightfields. oxijolt binds Jolt 5.6
through [joltc] and has a safe API for all of those. It also has constraints, soft bodies, contact
events, saving and restoring a world, and stepping on your own job system. Its tests check that
results match bit for bit with 1 and 4 worker threads and on a caller job system. Out-of-range
values get a typed error before they reach Jolt. It builds without LLVM. Its shape and rigid-body
API is not complete yet; [Features](#features) says what is missing.

## Documentation

- [Guide](docs/guide.md): a game scene with terrain, queries, a floating origin, a character, a
  car and a ragdoll, run as doctests.
- Topic guides: [constraints](docs/constraints.md), [soft bodies](docs/soft-bodies.md),
  [events](docs/events.md), [save and restore](docs/state.md), [job systems](docs/job-system.md),
  [determinism](docs/determinism.md), [building](docs/building.md).
- [Limits](docs/limits.md) and [coverage](docs/coverage.md); [benchmarks](docs/benchmarks.md)
  against a game's budgets.
- [Character study](docs/character-study.md): which character laws CharacterVirtual's settings
  carry, and at what cost.
- API docs: `cargo doc -p oxijolt --open`. Example: `cargo run -p oxijolt --example hello_world`.
- [CHANGELOG](CHANGELOG.md).

## Credits

The project began as a fork of SecondHalfGames/jolt-rust; [LINEAGE.md](LINEAGE.md) credits the
projects it builds on.

## License

Licensed under either of

* Apache License, Version 2.0, ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
* MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option. Jolt Physics and joltc are MIT licensed.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.

[joltc]: https://github.com/amerkoleci/joltc
