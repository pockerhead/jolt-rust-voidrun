# Character study: what CharacterVirtual's built-ins carry

VOIDRUN's near-band controller wraps Jolt's `CharacterVirtual` in passes of its own: a floor snap
instead of stick-to-floor, a terrain support cast for the steep-as-wall rule, a depenetration push
before the move, an autostep and an underground recovery. This study measures, on headless and
deterministic scenarios, which of the game's character laws CharacterVirtual's own mechanisms carry,
with which settings, which of the game's passes they replace, and what each configuration costs per
move.

The laws:

1. Walkable up to 45 degrees; steeper slides; never grounded on a face steeper than the limit.
2. Stays on the floor over convex crests and walking down slopes just under the limit (40, 43,
   44.5 degrees).
3. A 0.45 m step up succeeds, 0.5 m does not (3s: sharp boxes, 3r: boxes with a 0.05 m convex
   radius).
4. No hops walking downhill.
5. A push-out from terrain or walls never becomes velocity (5v), and the overlap is resolved,
   also deeper than radius + padding (5r).
6. Heightfield chunk seams (adjacent heightfield bodies) behave like one surface.

The scenarios live in `crates/oxijolt/tests/study/`, the gates in
`crates/oxijolt/tests/character_study.rs` (frame and determinism checks in
`character_study_determinism.rs`, the caller's rule 7 in `character_study_rules.rs`), the cost
harness in `crates/oxijolt/benches/character_study.rs`.

Two of the study's criteria are stricter than the game's acceptance scenarios: law 1 judges the
faces near the capsule (a steep ridge's apex counts as steep, while spec D.2 rule 7 judges the
terrain support normal, which points nearly up there), and law 6 asks for 2 mm and the same ground
state every tick (G.4 #15 asks for the rest height within 5 cm and the path within 8 cm). A broken
study column is therefore not by itself a failed game scenario; the radial family runs G.4 #15
with its own criteria.

## Result in short

- No configuration carries every law. Laws 1 and 6 and the radial family are broken in every
  row; the failures come from Jolt mechanisms listed under [What the built-ins cannot
  carry](#what-the-built-ins-cannot-carry).
- Law 2 is broken by every built-in configuration and by the game's own passes. Two changes to
  the game's Q5 floor snap carry it (`d2-noq4-floor`): a contact refresh before the snap's gate
  (crests) and accepting a steep hit on a structure (ledge edges). Neither alone is enough.
- Law 4 (no hops downhill) holds with stick-to-floor alone (every row but `bare`).
- Law 3 holds with the game's autostep on sharp and rounded steps. Jolt's walk stairs holds it on
  rounded steps with a step-up of 0.33 m (it climbs up to 0.467 m) and does not on sharp steps.
- Law 5v holds when the overlap is resolved before the move (the game's Q6 push, or a still
  update) or Jolt's penetration recovery is 0 inside the move; law 5r needs the underground
  recovery for burials.
- Recommended (`recommended` below, the same as `d2-noq4-floor`): the game's passes without the
  Q4 terrain support cast, with the two snap changes: 9.5 us per move at the median on the
  planet walk, against 10.7 us for all of the game's passes (`spec-d2`) and 5.1 us for the
  repository's reference near step, on one machine. It holds laws 2, 3s, 3r, 4, 5v and 5r and
  breaks 1, 6 and radial; Q4 removes some law 1 failures (a mid-slide grounding on 45.5 degree
  terrain) and no law 6 failure.

## Method

### The caller

Every row runs the same caller (`tests/study/controller.rs`), a generalisation of the repository's
reference near step (`tests/common/walker.rs`, spec D.2). A row switches its mechanisms; the order
is fixed:

1. Up for the tick from the start origin (radial on the planet, the frame's up elsewhere), then
   rotation and position.
2. Q3 underground recovery: a 0.05 m ball cast down from 50 m above the feet; terrain more than
   0.3 m above the feet puts the character on it.
3. Q6 depenetration push: every obstacle the capsule overlaps, the deepest hit per collider (body
   and compound child), each pushing along its outward normal by depth + padding, summed in
   collider order and capped at radius + padding.
4. A contact refresh: never, after a maintenance pass moved the character, or every tick.
5. A still update (zero velocity): never, always, when a current contact penetrates by more than
   1 mm, or when Q6 found an overlap deeper than radius + padding.
6. The vertical feed of rule 3 and the move (`ExtendedUpdate` with the row's stick-to-floor and
   walk-stairs settings; stick-to-floor either always passed, so Jolt's own precondition decides,
   or passed only after a grounded tick and when not rising, as the reference near step does).
   A row may set another penetration recovery speed for the move only.
7. Contact readout, then the game's autostep and the Q5 floor snap (a capsule cast down by 0.3 m
   with target distance = padding, accepted on a walkable, not dynamic hit) when the character
   was grounded, is not rising and Jolt does not report OnGround. Two law 2 remedies, off in the
   game's passes, change the snap: a contact refresh before its gate, and accepting a steep hit
   on a structure.
8. Rule 7: steep terrain is a wall. With Q4 (a capsule cast down by padding + 0.05 m against
   terrain only) the cast's normal decides; without it, Jolt's OnSteepGround on a terrain body.
   Steep terrain makes the character slide and not grounded even after a successful autostep or
   snap; steep structures (a step edge) still hold it (`character_study_rules.rs`).
9. Rule 8 carries vel_up; rule 9 reports the velocity as the displacement from the locomotion
   origin (the origin after every maintenance pass) over dt, zero without input.

The `walker` row reproduces the reference near step bit for bit over a 400-tick scripted run
(`the_walker_row_reproduces_the_reference_near_step_bit_for_bit`). For that it keeps the
reference's rule 7, where a successful autostep grounds the character even on steep terrain
("reference rule 7" in the table); every other row applies the terrain veto after the autostep
and the snap too.

### Geometry and predicates

The predicates read the scene's geometry back from the shapes: every heightfield sample through
`Shape::height_field_position`, with Jolt's cell diagonal, and every box with its convex radius.
The body origin is the lower sphere centre. The gap is the distance from the origin to the closest
triangle or box face minus radius and padding; resting is zero. The support set is every element
within 0.05 m of the closest one. A face is walkable up to 44.9 degrees from the tick's up, steep
from 45.1 degrees, and the 0.2 degree band between is never judged. A tick is "walkable" when its
closest element is walkable terrain or a walkable box top and no element of its support set is in
the band; "all-steep" when every element of its support set is steep terrain, an edge between
steep faces (a steep ridge's apex) included.

| Law | Scenes and inputs | Predicate |
|---|---|---|
| 1 | Heightfield planes at 30, 40, 43, 44.5 (walkable) and 45.5, 50, 60 degrees (steep), nine resting starts each, standing still and walking uphill at 2 m/s for 120 ticks; a 50 degree ridge started 5 cm off its apex | Walkable: grounded every walkable tick; still: drift under 1 mm; uphill: gap within 0.03 m and run at least the projected input minus 0.05 m (G.4 #13). Steep, on all-steep ticks: never grounded; uphill never rising more than 1 mm per tick; still: the downhill step positive from tick 30 and not shrinking over ticks 30 to 40 (G.4 #20). |
| 2 | Ramps (concave toe, convex crest), ridges and planes at 40, 43 and 44.5 degrees at 1.6, 3.5 and 7 m/s (crest descent, crest ascent, over a ridge, long descent); a 0.15 m ledge, sharp and rounded; box ramps at 40 and 44.5 degrees | On every walkable tick: grounded, gap within 0.03 m (G.4 #14) |
| 3 | Box steps 0.40, 0.45, 0.50 m, sharp and rounded, approached at 0, 30, 60 degrees off the face normal, 1.6 and 3.5 m/s, four lateral offsets, two distances | 0.40 and 0.45 land (0.05 m past the face, feet within 0.05 m of the top, grounded for 10 ticks); 0.50 never lands and ends more than 0.3 m before the face (G.4 #12) |
| 4 | Planes at 30, 40, 43, 44.5 and box ramps at 40, 44.5 degrees, three gaits: steady, from rest, stop and restart, 45 degrees to the fall line | On every walkable tick: rise along up at most 1 mm, grounded, gap within 0.03 m |
| 5v | Moving along a wall overlapped by 0.05 and by 0.5 m, over terrain overlapped by 0.1 m, along the contour of 30 and 60 degree planes overlapped by 0.1 m, a teleport into a wall mid-walk, a clear control | Against the same input from the pose where the padded capsule just touches: the output velocity along the surface normal times dt differs by at most 2 mm on every tick |
| 5r | The same overlaps standing and moving; burials of 1, 3 and 10 m; resting on flat terrain | Pushed more than 0.04 m from the thin wall (G.4 #17); clear of the thick wall within 0.01 m; at rest within 0.05 m and grounded after recovery (G.4 #16); no longer overlapping slopes; a resting character moves less than 0.01 m (G.4 #19) |
| 6 | Two 33-sample bodies meeting at x = 0 against one 65-sample body over the same span (decoded heights equal within 0.5 mm): flat, a 30 degree rise, a 44.5 degree descent, crossed perpendicularly, at 45 degrees and along the seam, 3.5 and 7 m/s | Both runs grounded on walkable ticks, the same ground state every tick, positions within 2 mm along up and 5 mm across, path lengths within 0.01 m |
| radial | On the planet of radius 99 with radial up: log-spiral slopes (a constant angle to radial up) at 30, 43, 44.5, 45.5 degrees with 0.5 m sample spacing, a crest from the planet's surface onto a 44.5 degree spiral, the walker fixtures' step (0.45, 0.5 m), wall, a 1 m burial and the chunk seam of G.4 #15 | Laws 1 to 6 on these scenes with the tick's radial up; at most 10 % of a case's ticks in the boundary band |

A planar slope on the planet changes its angle to radial up by about 0.57 degrees per metre (a
44.5 degree plane through the anchor is 45 degrees at x = 0.88 m and 36.7 degrees at x = -12 m),
so the radial family uses log-spiral terrain `r = 99 exp(tan(theta) phi)`. At 0.5 m sample spacing
its 44.5 degree faces stay within 44.19 to 44.81 degrees of the up at any origin 0.3 m away; at
1 m spacing, the game's spacing, the same faces span 43.91 to 45.08 degrees, so on the game's
terrain a nominal 44.5 degree slope has faces past the limit.

Every case needs its witness (30 ticks on the faces it is about, the route past the crest, the
capsule touching the step, the seam crossed); a case without it is a harness error and fails the
gate, unless the controller was blocked, which counts as a failure. `law_predicates_reject_corrupted_traces`
shows that each predicate fails a mutated trace (a grounded steep tick, a 0.05 m gap, a missing or
too-high landing, a 2 mm rise, a 0.05 m push in the velocity, an unrecovered overlap, a seam run
3 mm high, a stalled run).

### Cells

Every case runs from its start and from four starts moved 1 mm along the tangent axes. A case is
stable when the five agree. A cell is **held** when every case passes under every start,
**broken** when a stable case fails (the first by id is named), **variable** otherwise. The law
gates (`law_1_slope_limit` to `law_6_seams_are_one_surface`, `radial_acceptance`) replay every
pinned row from the unperturbed starts (the recommended row from all five) and check its cells.
The survey (`cargo test -p oxijolt --test character_study survey -- --ignored --nocapture`)
produced them in the default debug build on Windows; the gates passed unchanged in the ten CI legs
(Windows and Linux; default, cross-platform-deterministic, double-precision, debug-renderer and
asserts), so no pinned cell turned variable between builds. Within the survey, the cases that
changed with a 1 mm start offset are counted below as "unstable".

### Cost

`cargo bench -p oxijolt --bench character_study` times each row on the planet workload of the
near-step budget row (`benches/budgets.rs`): 3 x 3 chunks on the planet of radius 99, 30 characters
walking 2 m/s along slowly turning headings, 300 untimed ticks, then 5000 timed ticks, three
repetitions with the rows in forward, reverse and forward order (W1, 450,000 samples per row). One
sample is one character's whole move, from the up and pose setters to the velocity output,
including the contact readout and the row's passes; the actor capsule sync, scene building and
character creation are outside the timer. W2 times every tick of the law cases from the starts
the law gates judge (unperturbed, put back at rest for the row's padding), then judges each timed
run untimed and stops unless it gives the outcome the law gates get. W3 reruns W1 once and times
only the physics calls of each move (refreshes, updates, the passes' queries, the contact readout
and the autostep). W2 and W3 run once per row,
one row after another at the end of the run, so drift of the machine shows up there (rows late in
the run have W3 above W1); compare rows by W1. Percentiles use the nearest rank. The timer's own
cost, measured in the run: an empty `Instant` pair p50 0.0 us, p99 0.1 us, resolution 0.1 us;
nothing is subtracted. The planet walk is flat ground, so the passes' rates on it are zero except
the always-on still update (and 1.1 snaps per 1000 moves with the refresh before Q5); the walk
measures what each pass costs when it finds nothing to do.

Machine and build: Intel Core i9-11900K (8 cores, 16 threads), Windows 11 Pro 10.0.26200, rustc
1.95.0 (`bench` profile), MSVC 19.44.35211, Jolt 5.6.0 and joltc in Release, default features, one
worker thread, power plan "High performance", 2026-10-04. In the same run the repository's
reference near step (`near_tick`, including its actor sync) took 5.1 / 9.9 us (p50 / p99) and
`update_character` alone with the walker's settings 3.5 / 6.5 us; the published budget row is
5.1 / 9.7 us ([benchmarks](benchmarks.md)). The game reports 16 to 19 us at the median for its own
controller; that figure is from the game, not from this harness.

The study test binary runs in about 20 s in a local debug build; the survey takes about 7
minutes.

## Results

Cells as the law gates check them; costs from the bench run above (microseconds per move).

<!-- study-table:start -->
| row | mechanisms and parameters | 1 | 2 | 3s | 3r | 4 | 5v | 5r | 6 | radial | W1 p50 us | W1 p99 us | W3 p50 us |
|---|---|---|---|---|---|---|---|---|---|---|---:|---:|---:|
| bare | D.1 settings | broken (`1/ridge50/apex/still`) | broken (`2/boxramp40/descent/v1.6`) | broken (`3s/h0.4/heading0/v1.6/z0.37/d1`) | broken (`3r/h0.4/heading0/v1.6/z0.37/d1`) | broken (`4/boxramp40/diagonal/v1.6`) | broken (`5v/plane30-0.1/contour`) | broken (`5r/burial1/still`) | broken (`6/descent44.5/across/v3.5`) | broken (`radial/1/spiral45.5/x2/still`) | 3.8 | 7.4 | 3.4 |
| jolt-defaults | Jolt default settings, stick 0.5, stairs 0.4 fwd 0.02 test 0.15 | broken (`1/ridge50/apex/still`) | broken (`2/ramp40/crest-descent/v3.5`) | broken (`3s/h0.5/heading0/v1.6/z0.37/d1`) | broken (`3r/h0.5/heading0/v1.6/z0.37/d1`) | held | broken (`5v/plane30-0.1/contour`) | broken (`5r/burial1/still`) | broken (`6/descent44.5/along/v7`) | broken (`radial/2+4/crest44.5/descent/v3.5`) | 4.0 | 9.8 | 3.6 |
| spec-d1 | D.1 settings, stick 0.3, stairs 0.45 fwd 0.15 test 0.5 | broken (`1/ridge50/apex/still`) | broken (`2/ramp40/crest-descent/v3.5`) | broken (`3s/h0.5/heading0/v1.6/z0.37/d1`) | broken (`3r/h0.5/heading0/v1.6/z0.37/d1`) | held | broken (`5v/plane30-0.1/contour`) | broken (`5r/burial1/still`) | broken (`6/flat/across/v3.5`) | broken (`radial/1/spiral45.5/x2/still`) | 3.7 | 9.0 | 3.5 |
| spec-d1+refresh | D.1 settings, stick 0.3, stairs 0.45 fwd 0.15 test 0.5, refresh every tick | broken (`1/ridge50/apex/still`) | broken (`2/ledge0.15-rounded/v1.6`) | broken (`3s/h0.5/heading0/v1.6/z0.37/d1`) | broken (`3r/h0.5/heading0/v1.6/z0.37/d1`) | held | broken (`5v/plane30-0.1/contour`) | broken (`5r/burial1/still`) | broken (`6/flat/across/v3.5`) | broken (`radial/1/spiral45.5/x2/still`) | 5.6 | 12.3 | 5.3 |
| walker | D.1 settings, stick 0.3 (caller-gated), refresh when moved, Q3 underground, autostep, reference rule 7 | broken (`1/ridge50/apex/still`) | broken (`2/ramp40/crest-descent/v3.5`) | held | held | held | broken (`5v/plane30-0.1/contour`) | held | broken (`6/flat/across/v3.5`) | broken (`radial/1/spiral45.5/x2/still`) | 4.7 | 9.3 | 4.5 |
| spec-d2 | D.1 settings, refresh every tick, still update when deep, Q3 underground, Q6 push, autostep, Q5 snap, Q4 support | broken (`1/ridge50/apex/still`) | broken (`2/ledge0.15-rounded/v1.6`) | held | held | held | held | held | broken (`6/descent44.5/diagonal/v3.5`) | broken (`radial/2+4/crest44.5/descent/v7`) | 10.7 | 19.7 | 10.4 |
| d2-stick | D.1 settings, stick 0.3 (caller-gated), refresh every tick, still update when deep, Q3 underground, Q6 push, autostep, Q4 support | broken (`1/ridge50/apex/still`) | broken (`2/ledge0.15-rounded/v1.6`) | held | held | held | held | held | broken (`6/flat/across/v3.5`) | broken (`radial/2+4/crest44.5/descent/v1.6`) | 10.7 | 20.1 | 10.4 |
| d2-stick-norefresh | D.1 settings, stick 0.3 (caller-gated), refresh when moved, still update when deep, Q3 underground, Q6 push, autostep, Q4 support | broken (`1/ridge50/apex/still`) | broken (`2/ramp40/crest-descent/v3.5`) | held | held | held | held | held | broken (`6/flat/across/v3.5`) | broken (`radial/2+4/crest44.5/descent/v7`) | 8.8 | 16.5 | 8.5 |
| d2-noq4 | D.1 settings, refresh every tick, still update when deep, Q3 underground, Q6 push, autostep, Q5 snap | broken (`1/ridge50/apex/still`) | broken (`2/ledge0.15-rounded/v1.6`) | held | held | held | held | held | broken (`6/descent44.5/diagonal/v3.5`) | broken (`radial/1/spiral45.5/x2/still`) | 7.5 | 14.2 | 7.3 |
| d2-still | D.1 settings, refresh every tick, still update when penetrating, recovery 0 in the move, Q3 underground, autostep, Q5 snap, Q4 support | broken (`1/ridge50/apex/still`) | broken (`2/ledge0.15-rounded/v1.6`) | held | held | held | held | held | broken (`6/descent44.5/diagonal/v3.5`) | broken (`radial/2+4/crest44.5/descent/v7`) | 9.8 | 18.1 | 9.5 |
| d2-inmove | D.1 settings, refresh every tick, Q3 underground, autostep, Q5 snap, Q4 support | broken (`1/ridge50/apex/still`) | broken (`2/ledge0.15-rounded/v1.6`) | held | held | held | broken (`5v/plane30-0.1/contour`) | held | broken (`6/descent44.5/diagonal/v3.5`) | broken (`radial/2+4/crest44.5/descent/v7`) | 9.7 | 18.4 | 9.4 |
| d2-stairs | D.1 settings, stairs 0.33 fwd 0.15 test 0.5, refresh every tick, still update when deep, Q3 underground, Q6 push, Q5 snap, Q4 support | broken (`1/ridge50/apex/still`) | broken (`2/ledge0.15-rounded/v1.6`) | broken (`3s/h0.5/heading0/v1.6/z0.37/d1`) | held | held | held | held | broken (`6/descent44.5/diagonal/v3.5`) | broken (`radial/2+4/crest44.5/descent/v7`) | 10.7 | 20.9 | 10.3 |
| d2-noground | D.1 settings, refresh every tick, still update when deep, Q6 push, autostep, Q5 snap, Q4 support | broken (`1/ridge50/apex/still`) | broken (`2/ledge0.15-rounded/v1.6`) | held | held | held | held | broken (`5r/burial1/still`) | broken (`6/descent44.5/diagonal/v3.5`) | broken (`radial/2+4/crest44.5/descent/v7`) | 9.8 | 18.9 | 9.5 |
| d2-noq4-floor | D.1 settings, refresh every tick, still update when deep, Q3 underground, Q6 push, autostep, Q5 snap, refresh before Q5, Q5 on structure edges | broken (`1/ridge50/apex/still`) | held | held | held | held | held | held | broken (`6/descent44.5/diagonal/v3.5`) | broken (`radial/1/spiral45.5/x2/still`) | 9.5 | 17.9 | 9.4 |
| max-slope-50 | D.1 settings, max slope 50, stick 0.3, stairs 0.45 fwd 0.15 test 0.5 | broken (`1/plane45.5/x-4z0.37/still`) | broken (`2/ramp40/crest-descent/v3.5`) | broken (`3s/h0.5/heading0/v1.6/z0.37/d1`) | broken (`3r/h0.5/heading0/v1.6/z0.37/d1`) | held | broken (`5v/plane30-0.1/contour`) | broken (`5r/burial1/still`) | broken (`6/flat/across/v3.5`) | broken (`radial/1/spiral45.5/x2/still`) | 3.8 | 9.3 | 3.5 |
| recommended | D.1 settings, refresh every tick, still update when deep, Q3 underground, Q6 push, autostep, Q5 snap, refresh before Q5, Q5 on structure edges | broken (`1/ridge50/apex/still`) | held | held | held | held | held | held | broken (`6/descent44.5/diagonal/v3.5`) | broken (`radial/1/spiral45.5/x2/still`) | 9.5 | 18.0 | 9.3 |
<!-- study-table:end -->

`walker` is the repository's reference near step; `spec-d2` runs every pass the game added, in
spec D.2's order; the `d2-*` rows each change one thing in `spec-d2`: Jolt's stick-to-floor
instead of the Q5 snap (`d2-stick`, also without the every-tick refresh in `d2-stick-norefresh`),
no Q4 (`d2-noq4`), Jolt's recovery in a still update instead of Q6 (`d2-still`) or inside the move
(`d2-inmove`), Jolt's walk stairs instead of the autostep (`d2-stairs`), no underground recovery
(`d2-noground`); `d2-noq4-floor` is `d2-noq4` with the two law 2 changes to the Q5 snap
(a refresh before its gate, steep structure hits accepted). `max-slope-50` is the law 1 control (it reports OnGround on a 45.5 degree face,
`the_controls_are_seen`). The 3 cm raised seam, the other control, fails law 6 in every case.

### Survey rows

Not pinned; cells from the five-start survey in the default debug build, with passing runs out
of all runs; W1 from the bench run above. The rows `d2-noq4-snap-refresh`, `d2-noq4-snap-edges`
(each law 2 remedy alone on `d2-noq4`) and `d2-floor` (both on `spec-d2`) were surveyed with the
caller as it is now; the others at commit `9fac5f1`, whose caller differs only in rule 7 after an
autostep or snap, which these rows do not run.

<!-- study-survey:start -->
| row | mechanisms and parameters | 1 | 2 | 3s | 3r | 4 | 5v | 5r | 6 | radial | W1 p50 us | W1 p99 us |
|---|---|---|---|---|---|---|---|---|---|---|---:|---:|
| d2-noq4-snap-refresh | D.1 settings, refresh every tick, still update when deep, Q3 underground, Q6 push, autostep, Q5 snap, refresh before Q5 | broken 624/635, 6 unstable | broken 230/240 | held 720/720 | held 720/720 | held 360/360 | held 35/35 | held 70/70 | broken 65/90 | broken 89/120, 1 unstable | 9.5 | 17.9 |
| d2-noq4-snap-edges | D.1 settings, refresh every tick, still update when deep, Q3 underground, Q6 push, autostep, Q5 snap, Q5 on structure edges | broken 624/635, 6 unstable | broken 210/240 | held 720/720 | held 720/720 | held 360/360 | held 35/35 | held 70/70 | broken 55/90 | broken 83/120, 2 unstable | 7.5 | 14.0 |
| d2-floor | D.1 settings, refresh every tick, still update when deep, Q3 underground, Q6 push, autostep, Q5 snap, refresh before Q5, Q5 on structure edges, Q4 support | broken 630/635 | variable 239/240, 1 unstable | held 720/720 | held 720/720 | held 360/360 | held 35/35 | held 70/70 | broken 65/90 | variable 119/120, 1 unstable | 12.8 | 25.5 |
| d1-up0.30-test0.15 | D.1 settings, stick 0.3, stairs 0.3 fwd 0.15 test 0.15 | broken 624/635, 6 unstable | broken 210/240 | broken 478/720, 1 unstable | broken 480/720 | held 360/360 | broken 5/35 | broken 55/70 | broken 65/90 | broken 65/120, 3 unstable | 3.8 | 9.2 |
| d1-up0.30-test0.5 | D.1 settings, stick 0.3, stairs 0.3 fwd 0.15 test 0.5 | broken 624/635, 6 unstable | broken 210/240 | broken 478/720, 1 unstable | broken 480/720 | held 360/360 | broken 5/35 | broken 55/70 | broken 65/90 | broken 65/120, 2 unstable | 3.7 | 9.1 |
| d1-up0.33-test0.15 | D.1 settings, stick 0.3, stairs 0.33 fwd 0.15 test 0.15 | broken 624/635, 6 unstable | broken 210/240 | broken 478/720, 1 unstable | held 720/720 | held 360/360 | broken 5/35 | broken 55/70 | broken 65/90 | broken 67/120, 3 unstable | 3.8 | 9.1 |
| d1-up0.33-test0.5 | D.1 settings, stick 0.3, stairs 0.33 fwd 0.15 test 0.5 | broken 624/635, 6 unstable | broken 210/240 | broken 478/720, 1 unstable | held 720/720 | held 360/360 | broken 5/35 | broken 55/70 | broken 65/90 | broken 66/120, 2 unstable | 3.7 | 9.0 |
| d1-up0.36-test0.15 | D.1 settings, stick 0.3, stairs 0.36 fwd 0.15 test 0.15 | broken 624/635, 6 unstable | broken 210/240 | broken 478/720, 1 unstable | broken 710/720 | held 360/360 | broken 5/35 | broken 55/70 | broken 65/90 | broken 68/120, 3 unstable | 3.7 | 9.0 |
| d1-up0.36-test0.5 | D.1 settings, stick 0.3, stairs 0.36 fwd 0.15 test 0.5 | broken 624/635, 6 unstable | broken 210/240 | broken 478/720, 1 unstable | broken 710/720 | held 360/360 | broken 5/35 | broken 55/70 | broken 65/90 | broken 67/120, 2 unstable | 3.8 | 9.3 |
| d1-up0.40-test0.15 | D.1 settings, stick 0.3, stairs 0.4 fwd 0.15 test 0.15 | broken 624/635, 6 unstable | broken 210/240 | broken 478/720, 1 unstable | broken 480/720 | held 360/360 | broken 5/35 | broken 55/70 | broken 65/90 | broken 67/120, 2 unstable | 3.7 | 8.9 |
| d1-up0.40-test0.5 | D.1 settings, stick 0.3, stairs 0.4 fwd 0.15 test 0.5 | broken 624/635, 6 unstable | broken 210/240 | broken 478/720, 1 unstable | broken 480/720 | held 360/360 | broken 5/35 | broken 55/70 | broken 65/90 | broken 67/120, 2 unstable | 3.8 | 9.1 |
| d1r-tolerance0.01 | D.1 settings, tolerance 0.01, stick 0.3, stairs 0.45 fwd 0.15 test 0.5, refresh every tick | broken 624/635, 6 unstable | broken 138/240, 1 unstable | broken 478/720, 1 unstable | broken 480/720 | held 360/360 | broken 5/35 | broken 55/70 | broken 65/90 | broken 59/120, 1 unstable | 5.6 | 12.7 |
| d1r-tolerance0.02 | D.1 settings, tolerance 0.02, stick 0.3, stairs 0.45 fwd 0.15 test 0.5, refresh every tick | broken 624/635, 6 unstable | broken 172/240, 2 unstable | broken 478/720, 1 unstable | broken 480/720 | held 360/360 | broken 5/35 | broken 55/70 | broken 65/90 | broken 59/120, 1 unstable | 5.6 | 12.8 |
| d1r-stick0.5 | D.1 settings, stick 0.5, stairs 0.45 fwd 0.15 test 0.5, refresh every tick | broken 624/635, 6 unstable | broken 103/240, 3 unstable | broken 478/720, 1 unstable | broken 480/720 | held 360/360 | broken 5/35 | broken 55/70 | broken 65/90 | broken 59/120, 1 unstable | 5.6 | 12.8 |
| d1r-predictive0.025 | D.1 settings, predictive 0.025, stick 0.3, stairs 0.45 fwd 0.15 test 0.5, refresh every tick | broken 624/635, 6 unstable | broken 101/240, 3 unstable | broken 479/720, 1 unstable | broken 480/720 | held 360/360 | broken 5/35 | broken 55/70 | broken 65/90 | broken 57/120, 1 unstable | 5.1 | 11.6 |
| d1r-predictive0.05 | D.1 settings, predictive 0.05, stick 0.3, stairs 0.45 fwd 0.15 test 0.5, refresh every tick | broken 624/635, 6 unstable | broken 105/240, 2 unstable | broken 479/720, 1 unstable | broken 480/720 | held 360/360 | broken 5/35 | broken 55/70 | broken 65/90 | broken 57/120, 1 unstable | 5.3 | 12.0 |
| d1r-predictive0.2 | D.1 settings, predictive 0.2, stick 0.3, stairs 0.45 fwd 0.15 test 0.5, refresh every tick | broken 624/635, 6 unstable | broken 100/240, 2 unstable | broken 480/720 | broken 480/720 | held 360/360 | broken 5/35 | broken 55/70 | broken 65/90 | broken 56/120, 1 unstable | 6.2 | 13.3 |
| d1-padding0.01 | D.1 settings, padding 0.01, stick 0.3, stairs 0.45 fwd 0.15 test 0.5 | broken 540/635 | broken 210/240 | broken 473/720, 6 unstable | broken 480/720 | held 360/360 | broken 5/35 | broken 55/70 | broken 64/90, 1 unstable | broken 66/120, 1 unstable | 3.8 | 9.2 |
| d1-padding0.04 | D.1 settings, padding 0.04, stick 0.3, stairs 0.45 fwd 0.15 test 0.5 | broken 612/635, 18 unstable | broken 225/240 | broken 479/720, 1 unstable | broken 480/720 | held 360/360 | broken 5/35 | broken 49/70, 1 unstable | broken 65/90 | broken 72/120, 2 unstable | 3.7 | 8.6 |
| d1-no-edge-removal | D.1 settings, edge removal off, stick 0.3, stairs 0.45 fwd 0.15 test 0.5 | broken 630/635 | broken 205/240, 2 unstable | broken 478/720, 2 unstable | broken 480/720 | held 360/360 | broken 5/35 | broken 55/70 | broken 35/90 | broken 79/120, 1 unstable | 4.0 | 9.7 |
| d1-ignore-back-faces | D.1 settings, back faces ignored, stick 0.3, stairs 0.45 fwd 0.15 test 0.5 | broken 624/635, 6 unstable | broken 210/240 | broken 478/720, 1 unstable | broken 480/720 | held 360/360 | broken 5/35 | broken 55/70 | broken 65/90 | broken 67/120, 2 unstable | 3.6 | 8.6 |
| d1-still-always | D.1 settings, stick 0.3, stairs 0.45 fwd 0.15 test 0.5, still update always | broken 624/635, 6 unstable | broken 103/240, 3 unstable | broken 478/720, 1 unstable | broken 480/720 | held 360/360 | held 35/35 | broken 55/70 | broken 65/90 | broken 64/120, 1 unstable | 5.7 | 12.4 |
| d1-still-penetrating | D.1 settings, stick 0.3, stairs 0.45 fwd 0.15 test 0.5, still update when penetrating | broken 624/635, 6 unstable | broken 210/240 | broken 478/720, 1 unstable | broken 480/720 | held 360/360 | broken 30/35 | broken 55/70 | broken 65/90 | broken 72/120, 2 unstable | 3.8 | 9.3 |
| d1-moving-recovery0 | D.1 settings, stick 0.3, stairs 0.45 fwd 0.15 test 0.5, recovery 0 in the move | broken 624/635, 6 unstable | broken 210/240 | broken 476/720, 3 unstable | broken 480/720 | held 360/360 | held 35/35 | broken 5/70 | broken 65/90 | broken 70/120, 3 unstable | 3.8 | 9.7 |
| d1-recovery0.5 | D.1 settings, recovery speed 0.5, stick 0.3, stairs 0.45 fwd 0.15 test 0.5 | broken 624/635, 6 unstable | broken 210/240 | broken 472/720, 5 unstable | broken 480/720 | held 360/360 | broken 5/35 | broken 5/70 | broken 65/90 | broken 63/120, 2 unstable | 3.7 | 9.3 |
| d1-inner-body | D.1 settings, inner body, stick 0.3, stairs 0.45 fwd 0.15 test 0.5 | broken 624/635, 6 unstable | broken 210/240 | broken 478/720, 1 unstable | broken 480/720 | held 360/360 | broken 5/35 | broken 55/70 | broken 65/90 | broken 67/120, 2 unstable | 4.1 | 10.2 |
<!-- study-survey:end -->

What the survey adds:

- Walk-stairs step-up, on rounded steps (heading 0, 1 m away, 1.6 m/s, a 1 mm height grid from
  0.40 to 0.55 m): 0.30 climbs up to 0.437 m, 0.33 up to 0.467 m, 0.36 up to 0.497 m, 0.40 up to
  0.537 m, with either forward test (0.15 or 0.5 m); the climbed height is the step-up plus about
  0.137 m (padding plus r (1 - cos 45 degrees)). 0.33 is the smallest step-up that climbs 0.45 m
  and not 0.50 m, and `d2-stairs` uses it. On sharp steps every step-up leaves gaps in the grid
  (0.401, 0.402, 0.404, 0.405, 0.408, 0.411, 0.412, 0.415, 0.417, 0.418 m are missed by all of
  them) and climbs 0.5 m: the float tie-break of the box's surface normal at its top edge decides.
  The autostep climbs 0.400 to 0.469 m on both.
- Collision tolerance, predictive contact distance, padding, back faces, the inner body and the
  recovery speed change no cell. Turning enhanced internal edge removal off removes the 45.5
  degree mid-slide grounding (law 1 then fails only on the ridge apex) and makes law 6 worse
  (35 of 90 runs pass instead of 65).
- A still update before the move (always) or recovery 0 inside the move holds law 5v; with
  recovery 0 in the move and no still update the overlaps are not resolved (5r: 5 of 70).
- 8 bits per sample instead of 16 (survey with `STUDY_BITS=8`, rows `recommended` and `spec-d2`):
  `recommended`'s law 2 turns variable (`2/ridge44.5/over/v3.5` leaves the floor by 39 mm under
  three of the five starts) and its laws 1, 4 and radial keep their cells; `spec-d2`'s law 1 turns
  variable (one case changes with the start). The seam comparison is not valid on the sloped
  profiles, because two bodies quantise their shared edge independently (3.9 mm apart); 16 bits
  per sample keep adjacent chunks within 0.5 mm.
- The law 2 remedies one at a time on `d2-noq4`: the refresh before the snap passes 230 of 240
  runs (the rounded ledge still fails), accepting structure edges 210 of 240 (the crests still
  fail), both together 240 of 240. On `spec-d2` (`d2-floor`) both together leave law 2 and
  radial variable: under one of the five starts `2/ridge44.5/over/v3.5` ends a tick not grounded
  at zero gap, and `radial/2+4/crest44.5/descent/v1.6` fails a tick 17 mm above the floor; neither
  was traced.

## What the built-ins cannot carry

Jolt references are to the pinned revision
[`e77f175`](https://github.com/jrouwe/JoltPhysics/tree/e77f175595e64cb44218cc9d9d56fc365ad0e36a).

**Law 1, grounded on steep faces.** Two mechanisms report OnGround on faces steeper than the
limit:

- A contact whose contact normal points more upward than the face replaces the face's normal
  ([CharacterVirtual.cpp:229-230](https://github.com/jrouwe/JoltPhysics/blob/e77f175595e64cb44218cc9d9d56fc365ad0e36a/Jolt/Physics/Character/CharacterVirtual.cpp#L229-L230)).
  On a 50 degree ridge the apex edge's normal points nearly up: every row is grounded at the apex
  (`1/ridge50/apex/still`). Sliding down a 45.5 degree plane, a contact on a cell diagonal read
  44.94 degrees for one tick after about 115 ticks of slide, with enhanced internal edge removal
  on; the caller grounded and zeroed vel_up, which stopped the slide.
- Several steep contacts that block a solve along -up count as support
  ([CharacterVirtual.cpp:1222-1243](https://github.com/jrouwe/JoltPhysics/blob/e77f175595e64cb44218cc9d9d56fc365ad0e36a/Jolt/Physics/Character/CharacterVirtual.cpp#L1222-L1243)):
  on the radial 45.5 degree spiral, two facets at 45.3 degrees gave OnGround at tick 10.

The game's Q4 cast (D3) removes the mid-slide groundings on planes and spirals (`spec-d2` against
`d2-noq4`: law 1 then fails only on the ridge apex; the radial slope cases pass); it costs
3.2 us per move on the planet walk (10.7 against 7.5 us). It does not fix the ridge
apex, where the cast also returns the edge normal. That apex case is a study criterion, not a
game one: rule 7 judges the support normal, and there it points nearly up. A support check by the
face normal of the touched triangle (not the contact normal) would be the pass for apexes; it is
not measured here.

**Law 2, floating over crests.** A contact that the move's sweep marked as colliding keeps its
flag after the character has moved past it
([CharacterVirtual.cpp:1074-1077](https://github.com/jrouwe/JoltPhysics/blob/e77f175595e64cb44218cc9d9d56fc365ad0e36a/Jolt/Physics/Character/CharacterVirtual.cpp#L1074-L1077)),
so the character reports OnGround 3 to 7 cm above a crest at 3.5 and 7 m/s. `ExtendedUpdate`'s
stick-to-floor runs only when the character was supported before the update, is not after it and
did not move up
([CharacterVirtual.cpp:1808-1824](https://github.com/jrouwe/JoltPhysics/blob/e77f175595e64cb44218cc9d9d56fc365ad0e36a/Jolt/Physics/Character/CharacterVirtual.cpp#L1808-L1824)),
and the game's Q5 snap only when Jolt does not report OnGround (the caller's gate), so neither
acts on that tick and the character drops on the next ones (the owner's "brief floating, hops"
going down a hill). A refresh before the move turns the next tick's state to InAir, and then
stick-to-floor's precondition "supported before the update" fails (`d2-stick` against
`d2-stick-norefresh`: 103 against 210 of 240 law 2 runs pass); this is the game's D2 observation.

The pass that carries crests: refresh the contacts after the move, before the snap's gate. A
refresh judges collisions at the new position (a contact more than the collision tolerance away
does not collide), so the stale OnGround turns into InAir and Q5 snaps. Alone it passes 230 of
240 law 2 runs (`d2-noq4-snap-refresh`); what remains is the rounded ledge.

**Law 2, rounded ledges.** Walking off a 0.15 m ledge with a 0.05 m convex radius at 1.6 m/s, the
capsule rolls over the edge. At tick 86 the character is already InAir, 28 mm above the floor
below, and the snap's cast hits the edge first with a normal past 45 degrees, which Q5 rejects as
not walkable; refreshing the contacts does not change that. The pass that carries it: accept a
steep snap hit on a structure, as rule 7 already excludes structures from steep-as-wall. Alone it
passes 210 of 240 runs (`d2-noq4-snap-edges`, the crests fail); with the refresh, 240 of 240 under
all five starts (`d2-noq4-floor`, the recommended row), at 16 bits per sample. It costs 2.0
us per move on the planet walk (9.5 against 7.5 us), mostly the extra refresh.

`CharacterVirtual::StickToFloor` itself has no ground-state condition: it sweeps down and moves
to the contact
([CharacterVirtual.cpp:1767-1792](https://github.com/jrouwe/JoltPhysics/blob/e77f175595e64cb44218cc9d9d56fc365ad0e36a/Jolt/Physics/Character/CharacterVirtual.cpp#L1767-L1792)).
It is not bound: a caller needs a gate for it either way, and with the refresh before the gate the
existing Q5 cast already carries law 2. Whether an explicit call would do the same is not
measured.

**Law 5v, recovery inside the move.** Jolt's penetration recovery adds the overlap to the move's
velocity
([CharacterVirtual.cpp:709-710](https://github.com/jrouwe/JoltPhysics/blob/e77f175595e64cb44218cc9d9d56fc365ad0e36a/Jolt/Physics/Character/CharacterVirtual.cpp#L709-L710)),
so the push-out is part of the displacement the caller reports as velocity (0.07 m in one tick for
a 0.05 m wall overlap). The game's Q6 push (D4) or a still update before the move with recovery 0
in the move (`d2-still`, the runtime setter `CharacterMut::set_penetration_recovery_speed`) both
hold 5v; `d2-still` costs 9.8 us against 10.7 us for Q6 on the planet walk.

**Law 5r, burials.** Jolt does not lift a character buried 1 m or more under a heightfield
(`5r/burial1/still`, gap -1.47 m after the tick); the underground recovery (Q3) does, and costs
little when it finds nothing (`d2-noground` 9.8 against 10.7 us).

**Law 6, seams.** Enhanced internal edge removal works within one body: its collector flushes at
the end of each body
([InternalEdgeRemovingCollector.h:236-242](https://github.com/jrouwe/JoltPhysics/blob/e77f175595e64cb44218cc9d9d56fc365ad0e36a/Jolt/Physics/Collision/InternalEdgeRemovingCollector.h#L236-L242)),
and a heightfield's border edges are always active
([HeightFieldShape.cpp:368-384](https://github.com/jrouwe/JoltPhysics/blob/e77f175595e64cb44218cc9d9d56fc365ad0e36a/Jolt/Physics/Collision/Shape/HeightFieldShape.cpp#L368-L384)).
Crossing the seam on flat ground, the neighbour's border edge gives a contact tilted 11 degrees and
the character rises 3.1 mm at 3.5 m/s and 4.5 mm at 7 m/s, compared with one continuous field.
No setting or pass in the study removes it.

A geometry candidate: put adjacent heightfields in one body, as the children of one static
compound. The collector then sees both fields before it flushes, and voids the border edge like
an internal one (it matches vertices by position). Report cases `6-compound/*` run the 18 seam
cases on such a body: 16 pass at the unperturbed start, against 11 for two bodies. The two left,
the 44.5 degree diagonal descents, fail the same way with two bodies and with the compound (not
grounded for a tick, 27 mm above walkable slope, after crossing the seam); their cause is not
traced. Its scope is one compound: the border of the compound is again a border between bodies,
so streamed chunks would have to be merged per region, which is not measured.

Law 6 asks more than the game's seam scenario: G.4 #15 (120 ticks at 2 m/s across the radial
chunk seam, path 4.0 +- 0.08 m, rest height within 0.05 m, grounded every tick) is
`radial/6/chunk-seam/v2`, and it passes for every pinned row; of the survey rows only
`d1-padding0.01` fails it.

**Law 3 on sharp boxes with walk stairs.** On a sharp box the surface normal at the top edge is a
float tie-break between the top face and the side
([BoxShape.cpp:161](https://github.com/jrouwe/JoltPhysics/blob/e77f175595e64cb44218cc9d9d56fc365ad0e36a/Jolt/Physics/Collision/Shape/BoxShape.cpp#L161)),
so walk stairs misses some steps and climbs 0.5 m ones. The game's autostep, which judges the
cast's contact normal, holds both 3s and 3r. With a 0.05 m convex radius walk stairs holds 3r.
The safe API's shape casts (Q4, Q5 and the autostep) use Jolt's default `ShapeCastSettings`, whose
`mUseShrunkenShapeAndConvexRadius` is false; casts then take a box's support function in the
default mode, which is the sharp box
([ConvexShape.cpp:268-277](https://github.com/jrouwe/JoltPhysics/blob/e77f175595e64cb44218cc9d9d56fc365ad0e36a/Jolt/Physics/Collision/Shape/ConvexShape.cpp#L268-L277),
[BoxShape.cpp:111-120](https://github.com/jrouwe/JoltPhysics/blob/e77f175595e64cb44218cc9d9d56fc365ad0e36a/Jolt/Physics/Collision/Shape/BoxShape.cpp#L111-L120)).
CharacterVirtual's own sweeps set it to true
([CharacterVirtual.cpp:584](https://github.com/jrouwe/JoltPhysics/blob/e77f175595e64cb44218cc9d9d56fc365ad0e36a/Jolt/Physics/Character/CharacterVirtual.cpp#L584),
[673](https://github.com/jrouwe/JoltPhysics/blob/e77f175595e64cb44218cc9d9d56fc365ad0e36a/Jolt/Physics/Character/CharacterVirtual.cpp#L673)) and, like its contacts, see the
rounding. A cast snap onto a rounded edge leaves the capsule about r (sqrt 2 - 1) = 0.02 m off it.

## Recommendation

No row holds every law column, so the recommendation is the row with the most held columns (of 1,
2, 3s, 4, 5v, 5r, 6 and radial) and, among those, the lowest median cost on the planet walk.
`d2-noq4-floor` holds five (2, 3s, 4, 5v, 5r); `spec-d2`, `d2-stick`, `d2-stick-norefresh`,
`d2-noq4` and `d2-still` hold four each (3s, 4, 5v, 5r). `d2-noq4-floor` is the `recommended`
row:

- Creation: the D.1 settings (capsule radius 0.4, half height 0.70845, shape offset 0.70844734,
  padding 0.02, max slope 45 degrees, enhanced internal edge removal on, supporting plane through
  the lower sphere centre), penetration recovery speed 1.
- Each tick: up, rotation, position; Q3 underground recovery; Q6 push; a contact refresh; a still
  update when Q6 found an overlap deeper than radius + padding; the vertical feed; `ExtendedUpdate`
  with stick-to-floor and walk stairs off; the autostep; when the character was grounded, is not
  rising and did not step, a second contact refresh and then the Q5 snap if Jolt does not report
  OnGround, accepting a walkable hit or a steep hit on a structure; rule 7 by Jolt's OnSteepGround
  on terrain, which also overrides the autostep and the snap.
- Terrain: 16 bits per sample (8 bits make law 2 variable and break the seam comparison);
  structures sharp or rounded (the autostep holds both).
- Cost on the planet walk: 9.5 / 18.0 us (p50 / p99), against 10.7 / 19.7 us for `spec-d2`,
  7.5 / 14.2 us for `d2-noq4` (the same without the two snap changes), 4.7 / 9.3 us for `walker`,
  3.8 / 7.4 us for `bare`, 5.1 / 9.9 us for the reference near step and 3.5 / 6.5 us for
  `update_character` alone, in the same run.

Adding Q4 back removes the mid-slide groundings of law 1 on planes and on the radial spirals (+3.2
us on `d2-noq4`); it is the only pass that changes law 1, but it completes no column, and with
the two snap changes (`d2-floor`, 12.8 us) law 2 and radial turn variable. Without the two snap
changes (`d2-noq4`, 7.5 us) the row loses law 2. Law 6 is carried by no row; the compound-body
candidate above is measured on the seam cases only.

## Mechanism inventory

| Mechanism | Used | Why |
|---|---|---|
| Stick to floor (`stick_to_floor_step_down`) | rows `spec-d1*`, `walker`, `d2-stick*`; 0.3 and 0.5 m | holds law 4; law 2 fails on crests (see above); a refresh before the move disables it |
| Contact refresh (`refresh_character_contacts`) | before the move (`*+refresh`, `spec-d2` and its variants); after the move before Q5 (`d2-noq4-floor`, `d2-noq4-snap-refresh`, `d2-floor`) | after the move it replaces the sweep's stale OnGround over crests, so Q5 acts |
| Walk stairs (step up, min step forward, forward test, cos forward contact, step down extra) | `spec-d1*`, `jolt-defaults`, `d2-stairs`, the step-up survey; step down extra 0 | step-up + 0.137 m is the climbed height on rounded steps; unreliable on sharp steps |
| Max slope angle | 45 degrees; 50 in the control | the law's limit; the control shows the engine grounds on 45.5 degrees with 50 |
| Character padding | 0.02; 0.01 and 0.04 surveyed | no cell changes |
| Penetration recovery speed | 1; 0.5 surveyed; 0 inside the move (runtime setter) | in-move recovery breaks 5v; 0 in the move needs a still update to resolve overlaps |
| Predictive contact distance | 0.1; 0.025, 0.05, 0.2 surveyed | no cell changes |
| Collision tolerance | 1e-3; 0.01, 0.02 surveyed | no cell changes |
| Back-face mode | collide; ignore surveyed | no cell changes |
| Enhanced internal edge removal | on (D.1); off surveyed and in `jolt-defaults` | off removes the 45.5 degree mid-slide grounding and worsens seams |
| Supporting volume | the D.1 plane; Jolt's default in `jolt-defaults` | part of the D.1 settings |
| Inner body | off; on surveyed | no cell changes |
| Max collision / constraint iterations, min time remaining, hit reduction, max hits | Jolt's defaults | no traced failure pointed at them |
| Active edge threshold of the heightfields | Jolt's default (5 degrees) | seams are a per-body effect the threshold does not reach |
| Heightfields as children of one compound body | report cases `6-compound/*` | the per-body edge removal then covers the shared edge: 16 of 18 seam cases pass against 11 |
| Contact listener hooks | not bound | no hook changes the ground classification or separates recovery from displacement; a listener could only re-implement a game pass inside Jolt's solver |
| Explicit `StickToFloor` | not added | it has no ground-state condition of its own; a caller gates it, and with the refresh before the gate Q5 already carries law 2 |
