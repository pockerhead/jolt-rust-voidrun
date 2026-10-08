# VOIDRUN: Rapier's character controller replaced by oxijolt's

The owner's measurements from the game VOIDRUN, given on 2026-10-04, copied as given. VOIDRUN
replaced Rapier's kinematic character controller with oxijolt's `CharacterVirtual`. Release
build, on a weak 4-core machine. These are the game's own observations, not runs of this
repository.

| What | Rapier | Jolt | Ratio |
|---|---|---|---|
| Whole walking system, 60 NPCs in the near band, mean per tick | 3.43 ms | 0.83 ms | ~4x |
| One controller move, bare binding bench with terrain | 86-156 us | 2.7 us p50 / 6 us p99 | 15-30x |
| One controller move in the game with the game's wrapper | n/a | 16-19 us p50 / 28-43 us p99 | n/a |
| Physics world step p50/p99 | 110 / 268 us | 45 / 111 us | ~2.5x |
| Query snapshot update p99 | 3.8 ms | 164 us | ~20x |

The owner's caveats: the gains at the level of the game are smaller than the bare controller's,
because the rest of the frame is unchanged; the game's own wrapper around the controller
dominates the controller's cost in the game.
