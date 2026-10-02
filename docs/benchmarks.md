# Benchmarks against the game's budgets

`crates/joltphysics/benches/budgets.rs` times the physics work of one VOIDRUN tick and compares it
with the game's budgets. The numbers are wall-clock times on one machine, reported and never
checked by a test: they vary between runs and between machines.

## How to run

```bash
cargo bench -p joltphysics --bench budgets
```

Run one cargo process at a time on a quiet machine with a fixed power plan ("High performance"
below). The first run builds Jolt and joltc in Release under `target/release` unless
`JOLTC_LIB_DIR` points at a matching prebuilt prefix. Without the `--bench` argument that
`cargo bench` passes (for example under `cargo test --benches`) the binary returns at once.

## Machine

| | |
|---|---|
| CPU | 11th Gen Intel Core i9-11900K @ 3.50 GHz |
| Cores | 8 physical, 16 logical threads |
| RAM | 32 GB |
| OS | Windows 11 Pro 10.0.26200 |
| Rust | rustc 1.95.0 (59807616e 2026-04-14), `bench` profile (inherits `release`) |
| C++ | MSVC 19.44.35211 (toolset 14.44.35207), Jolt 5.6.0 and joltc in Release, default features |
| Power plan | High performance |
| Date | 2026-10-03 |

The bench prints this line itself:

```
windows x86_64, 16 logical threads, Intel64 Family 6 Model 167 Stepping 1, GenuineIntel; Rust code with debug assertions off; Jolt and joltc built in Release
```

Other programs were not closed for the run, so single samples (the max column) carry scheduler noise.

## Scenes

- **Flat 3 x 3 scene** (`update_character` rows): 3 x 3 chunks of 32 m, each a 33 x 33 rolling
  heightfield and a compound of 20 sharp boxes and Y-cylinders, and 30 characters walking circles
  at 2 m/s. The same scene as the earlier character cost example, so its numbers compare.
- **Planet scene** (all other rows): 3 x 3 chunks laid on a planet of radius 99 around the anchor,
  each a flat 33 x 33 heightfield and a compound of 10 structure boxes and 10 feature cylinders
  standing on it. Up is radial.
- **Near step**: 30 walkers of the reference controller (`tests/common/walker.rs`: underground probe,
  pose setters, `update_character`, the contact readout, the autostep and the actor capsule sync),
  walking at 2 m/s along slowly turning headings, with the terrain in the controller's filter.
- **Landing**: a tenth chunk at grid position (2, 0). Each landing builds its shapes, inserts the
  terrain and compound bodies, optimises the broad phase, and is removed again (untimed, with
  another optimisation) so every landing starts from the same world.
- **Steady step**: 8 items with the game's settings (box 0.35 x 0.06 x 0.04 half extents, 1.2 kg,
  friction 0.8, restitution 0.1, linear cast), pulled radially at 9.8 m/s² by the caller and dropped
  1-3 m above the centre chunk. An item calm for 30 ticks or older than 600 ticks is replaced, so
  items are always in flight.
- **Rays**: ground points within 40 m of the anchor; spawn ground rays go 55 m down from 5 m above
  the ground against terrain and structures; line-of-sight rays go 10-40 m sideways from 1.6 m above
  the ground against structures and features.
- **Tick**: the 30 walkers' near steps, then one structure-top ray (55 m down, structures only)
  under each of 60 mid-band actors on rings 20-45 m from the anchor. The mid band has no physics
  solve, so its actors are positions, not bodies. The world has 4 worker threads, but the near
  steps and the rays run on the calling thread.

## Samples

| case | warm-up (not reported) | samples |
|---|---|---|
| `update_character`, near step | 300 ticks | 5,000 ticks x 30 characters |
| landing | 50 landings | 2,000 landings |
| steady step | 300 ticks | 5,000 steps |
| rays | 1,000 rays | 10,000 rays (5,000 of each kind) |
| tick | 300 ticks | 5,000 ticks |

The first call after a scene is built is shown on its own "cold first call" row and is not among
the samples. Percentiles use the nearest rank: the p-th percentile of n sorted samples is sample
`ceil(p * n)`, counting from 1.

## Results

| case | one sample | samples | p50 us | p99 us | max us | limit / reference | status | note |
|---|---|---:|---:|---:|---:|---|---|---|
| update_character, terrain in filter: yes (flat 3 x 3 scene): cold first call | one call | 1 | 38.3 | 38.3 | 38.3 | - | - |  |
| update_character, terrain in filter: yes (flat 3 x 3 scene) | one call | 150000 | 2.7 | 6.0 | 116.7 | Rapier 86-156 us per character with terrain | - | supported after 100% of calls |
| update_character, terrain in filter: no (flat 3 x 3 scene): cold first call | one call | 1 | 4.8 | 4.8 | 4.8 | - | - |  |
| update_character, terrain in filter: no (flat 3 x 3 scene) | one call | 150000 | 0.2 | 0.3 | 16.6 | Rapier 86-156 us per character with terrain | - | supported after 0% of calls |
| near step (radial planet): cold first call | one walker step | 1 | 34.8 | 34.8 | 34.8 | - | - |  |
| near step (radial planet) | one walker step | 150000 | 5.1 | 9.7 | 87.7 | Rapier 86-156 us per character with terrain | - | grounded after 100% of steps |
| chunk shape build (off the sim thread in the game) | one chunk | 2000 | 36.9 | 43.1 | 72.2 | - | - | 33 x 33 heightfield + 20-child compound |
| landing: insert terrain + compound bodies: cold first call | one landing | 1 | 0.8 | 0.8 | 0.8 | - | - |  |
| landing: insert terrain + compound bodies | one landing | 2000 | 0.6 | 1.0 | 11.9 | <= 2000 us (limit, no percentile given; p99 shown) | within | two create_body calls |
| landing incl. shape build | one landing | 2000 | 37.5 | 43.8 | 77.2 | <= 2000 us (limit, no percentile given; p99 shown) | within | the game builds shapes off the sim thread |
| broad-phase optimize after a landing | one call | 2000 | 1.5 | 1.8 | 6.8 | - | - | not needed for queries; the explicit refresh point |
| steady step, 8 items, 1 worker thread: cold first call | one step | 1 | 111.7 | 111.7 | 111.7 | - | - |  |
| steady step, 8 items, 1 worker thread | one step | 5000 | 22.6 | 34.5 | 129.0 | <= 1000 us (limit, no percentile given; p99 shown) | within | 8.0 awake items per tick on average; 496 items replaced |
| steady step, 8 items, 4 worker threads: cold first call | one step | 1 | 80.1 | 80.1 | 80.1 | - | - |  |
| steady step, 8 items, 4 worker threads | one step | 5000 | 36.7 | 53.9 | 174.6 | <= 1000 us (limit, no percentile given; p99 shown) | within | 8.0 awake items per tick on average; 496 items replaced |
| ray: cold first call | one ray | 1 | 14.0 | 14.0 | 14.0 | - | - |  |
| ray (all) | one ray | 10000 | 0.7 | 1.1 | 114.0 | p99 <= 50 us | within | spawn ground and line of sight alternating |
| ray: spawn ground | one ray | 5000 | 0.8 | 1.2 | 114.0 | p99 <= 50 us | within | 55 m down, structures only; terrain 4876, structure 124, feature 0, miss 0 |
| ray: line of sight | one ray | 5000 | 0.5 | 1.0 | 40.4 | p99 <= 50 us | within | 10-40 m at eye height, structures and features; terrain 0, structure 811, feature 459, miss 3730 |
| tick: 30 near steps + 60 mid-band structure-top rays: cold first call | one tick | 1 | 245.0 | 245.0 | 245.0 | - | - |  |
| tick: 30 near steps + 60 mid-band structure-top rays | one tick | 5000 | 185.6 | 251.1 | 346.1 | p99 <= 2000 us | within | grounded after 100% of steps; 3% of mid rays hit a structure top; the walker update and queries run on the calling thread, the world's 4 worker threads take no part; refresh is zero work (queries see bodies immediately) |

## Reading

- Every limit holds, by a wide margin. Ray p99 is about 1 us against 50 us; the 30 + 60 tick has a
  p99 of 251 us against 2 ms; the steady step's p99 is 35 us on one worker thread against 1 ms.
- The landing limit (2 ms) is compared on the insertion row, the work on the simulation thread. The
  game builds the heightfield and compound off that thread; even with the build included a landing
  costs 44 us at p99. The broad-phase optimisation is not needed for queries (they see new bodies at
  once) and has its own row without a limit.
- The game gives the landing and steady-step limits without a percentile; those rows compare the
  p99 and say so.
- The steady step is slower with 4 worker threads than with 1: with 8 items there is too little work
  to share, and the job system's overhead dominates.
- The character rows compare with Rapier's 86-156 us per character with terrain in the query: one
  `update_character` costs 2.7 us at the median and 6.0 us at p99 with terrain, and a whole near step
  5.1 / 9.7 us. With the terrain left out of the flat scene's filter the characters find nothing to
  stand on and fall, so that row is cheap and only shows what the terrain adds.
- Maximum values are single outliers (the machine's scheduler) and vary most between runs.
