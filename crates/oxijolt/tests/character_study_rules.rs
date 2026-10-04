//! The study caller's rule 7 (spec D.2): steep terrain is a wall the character slides down, a
//! steep structure (a step edge reads steep) holds it, the terrain veto stands after the
//! autostep and the floor snap, a structure wall beside a high ledge does not hold a character
//! that walks off it, and the structure-edge snap takes a ledge edge and not a leaning wall.
//! `docs/character-study.md` describes the study.

mod common;
mod study;

use common::math::{add, dot, f3, norm, normalize, rvec3, scale, sub, v3, vec3, V3};
use common::walker::{
    capsule, controller_filter, from_y_to, Carry, CENTRE, CENTRE_UP, G, R, RADIUS, REST_HEIGHT,
    SNAP,
};
use common::DT;
use oxijolt::*;
use study::config::{Config, Rule7};
use study::controller::{carried, ends_grounded, feed, tick, Held, TickReport};
use study::passes::{snap, EDGE_DEPTH};
use study::scenes::{Scene, SceneKey, HIGH_LEDGE_DROP};

/// Under spec D.2's rule 7, steep terrain support vetoes every way of grounding, the autostep
/// and the snap included, so a snapped tick on steep terrain keeps its downward vel_up and the
/// next tick falls faster. The reference near step's rule lets a successful autostep or snap
/// ground the character on steep terrain; only the `walker` row uses it.
#[test]
fn the_terrain_veto_stands_after_the_autostep_and_the_snap() {
    let none = Held {
        stepped: false,
        snapped: false,
        jolt: false,
    };
    let stepped = Held {
        stepped: true,
        ..none
    };
    let snapped = Held {
        snapped: true,
        ..none
    };
    let jolt = Held { jolt: true, ..none };
    let falling = feed(0.0, true, true);
    assert_eq!(
        falling,
        -G * DT,
        "grounded and walking feeds one tick of gravity"
    );
    for held in [stepped, snapped, jolt] {
        assert!(!ends_grounded(Rule7::Spec, falling, held, true), "{held:?}");
        assert!(ends_grounded(Rule7::Spec, falling, held, false), "{held:?}");
        assert!(
            !ends_grounded(Rule7::Spec, 0.1, held, false),
            "rising: {held:?}"
        );
    }
    assert!(!ends_grounded(Rule7::Spec, falling, none, false));

    for held in [stepped, snapped] {
        assert!(
            ends_grounded(Rule7::Reference, falling, held, true),
            "{held:?}"
        );
    }
    assert!(!ends_grounded(Rule7::Reference, falling, jolt, true));

    let grounded = ends_grounded(Rule7::Spec, falling, snapped, true);
    let vel_up = carried(falling, grounded, false);
    assert_eq!(vel_up, falling, "the slide keeps its vel_up");
    assert!(feed(vel_up, grounded, true) < vel_up, "and falls faster");
}

/// Standing still on a 50 degree face. On a structure (a sharp box ramp) Jolt reports
/// OnSteepGround and the character stays grounded with vel_up 0 and does not move; on terrain (a
/// heightfield plane) it is never grounded and slides with vel_up falling every tick. Checked
/// for `spec-d2` and `recommended` (the Q4 cast decides), `d2-noq4` (Jolt's ground state decides)
/// and `walker`.
#[test]
fn a_steep_structure_holds_the_character_and_steep_terrain_does_not() {
    let structure = SceneKey::BoxRamp {
        deg10: 500,
        tilted: false,
    };
    for config in [
        Config::spec_d2(),
        Config::recommended(),
        Config::d2_noq4(),
        Config::walker(),
    ] {
        let name = config.name;
        let run = still_run(structure, &config);
        assert!(
            run.iter()
                .all(|r| r.ground == GroundState::OnSteepGround && !r.on_terrain),
            "{name}: the structure must read steep for this case"
        );
        for (t, r) in run.iter().enumerate() {
            assert!(
                r.grounded && !r.sliding && r.vel_up == 0.0,
                "{name} on the structure, tick {t}: grounded {} sliding {} vel_up {}",
                r.grounded,
                r.sliding,
                r.vel_up
            );
        }
        let drift = norm(sub(run.last().unwrap().end, run[0].start));
        assert!(drift < 1e-3, "{name} on the structure drifted {drift} m");

        let run = still_run(SceneKey::plane(50.0), &config);
        assert!(run[0].sliding, "{name} on terrain: tick 0 slides");
        for (t, r) in run.iter().enumerate() {
            assert!(!r.grounded, "{name} on terrain, tick {t}: grounded");
            if t > 0 {
                assert!(
                    r.vel_up < run[t - 1].vel_up,
                    "{name} on terrain, tick {t}: vel_up {} after {}",
                    r.vel_up,
                    run[t - 1].vel_up
                );
            }
        }
        let fallen = common::math::dot(sub(run[0].start, run.last().unwrap().end), run[0].up);
        assert!(fallen > 0.5, "{name} on terrain fell only {fallen} m");
    }
}

/// 60 ticks of `config` standing still from the rest pose at the middle of `key`.
fn still_run(key: SceneKey, config: &Config) -> Vec<TickReport> {
    let mut scene = Scene::build(key);
    let start = study::laws::resting(&scene, 0.0, 0.0);
    let walker = config.create_character(&mut scene, start);
    let mut carry = Carry::RESTING;
    (0..60)
        .map(|_| {
            let report = tick(&mut scene, &walker, config, &mut carry, [0.0; 3], None);
            scene.place_actor(report.end, from_y_to(report.up));
            report
        })
        .collect()
}

/// Walking off a 2 m structure ledge at 2 m/s along a structure wall, pressing into it at
/// 0.5 m/s, on a plane and on the planet: once past the edge the character is never grounded in
/// the air, and it lands on the terrain below. The wall's face is 5 mm inside the padded
/// capsule's reach at the start; Jolt keeps the padding, so the character walks along the wall at
/// padding distance.
///
/// Checked on the plane for every pinned row and the survey rows with the Q5 snap changes, except
/// `jolt-defaults`; on the planet for the rows with the D.1 supporting plane and neither snap
/// change. A snap that accepted a steep structure hit the padded capsule already touches (the
/// wall, at fraction 0) grounded the character beside the wall and held it there. The rows left
/// out hang in some variants for reasons of their own, which `high_ledge_beside_wall_survey`
/// counts: with Jolt's default supporting volume every wall contact supports; on the planet a
/// wall contact can sit on the D.1 supporting plane after the refresh before Q5, and a
/// world-vertical wall faces slightly up along radial up.
#[test]
fn walking_off_a_high_ledge_beside_a_wall_falls_to_the_floor() {
    let mut failures = Vec::new();
    for config in ledge_rows() {
        let snap_changed = config.floor.refresh_before_snap || config.floor.snap_on_structure_edges;
        let checked = [
            (
                planar_ledge(WALL_FACE_MM, 0),
                config.settings.supporting_plane,
            ),
            (
                radial_ledge(WALL_FACE_MM, 0),
                config.settings.supporting_plane && !snap_changed,
            ),
        ];
        for (key, checked) in checked {
            let outcome = walk_off_the_ledge(key, &config, 0.5, 0.0);
            if checked && outcome != LedgeOutcome::Falls {
                failures.push(format!("{} on {key:?}: {outcome:?}", config.name));
            }
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// The Q5 snap's structure-edge change takes a ledge edge under the capsule and not a wall
/// beside it. Past the high ledge, beside a wall 0.43 m from the path that leans back by 2 or
/// 5 degrees, the padded capsule does not touch the wall; the snap's cast reaches it after
/// moving down (fraction above 0) and finds a face looking up, which the snap must still reject.
/// 0.35 m past the edge the cast meets the platform's edge first, steep and 0.23 m below the
/// lower sphere centre, which the snap takes.
#[test]
fn the_structure_edge_snap_takes_a_ledge_edge_and_not_a_wall_leaning_back() {
    let config = Config::d2_noq4_floor();
    let p = config.settings.padding;
    let up = [0.0, 1.0, 0.0];
    for (lean_cdeg, x, takes) in [(200, 1.0, false), (500, 1.0, false), (200, 0.35, true)] {
        let mut scene = Scene::build(planar_ledge(430, lean_cdeg));
        let origin = [x, HIGH_LEDGE_DROP + f64::from(REST_HEIGHT), 0.0];
        let walker = config.create_character(&mut scene, origin);
        let layers = [scene.layers.terrain, scene.layers.chunk, scene.layers.actor];
        let filter = controller_filter(&walker, &layers);
        let shape = capsule();
        let cast = ShapeCast::new(
            &shape,
            rvec3(add(origin, scale(up, f64::from(CENTRE_UP)))),
            from_y_to(up),
            vec3(scale(up, -f64::from(SNAP))),
        )
        .target_distance(p);
        let hit = scene.world.cast_shape(&cast, &filter).unwrap();
        let hit = hit.unwrap_or_else(|| panic!("lean {lean_cdeg}, x {x}: the cast hits nothing"));
        let n_up = dot(f3(hit.normal), up);
        assert!(
            hit.fraction > 0.0 && n_up > 0.0 && n_up < std::f64::consts::FRAC_1_SQRT_2,
            "lean {lean_cdeg}, x {x}: the cast must find a steep face looking up after moving"
        );
        let found = snap(&scene.world, &filter, origin, up, p, true);
        assert_eq!(found.is_some(), takes, "lean {lean_cdeg}, x {x}: {found:?}");
        if let Some(found) = found {
            let below = dot(sub(origin, v3(hit.point)), up) - found.distance;
            assert!(below >= EDGE_DEPTH, "the edge lies {below} m below");
        }
    }
}

/// Counts, for every row of the ledge test, the variants of the high ledge beside a wall in which
/// the character hangs or does not land, and what held it. Upright walls: faces from 0.40 to
/// 0.43 m, presses into the wall of 0 to 1 m/s and three start offsets across the wall, on the
/// plane and on the planet. Walls leaning back (face looking up) by 0.1 to 9 degrees, per lean:
/// faces 0.415, 0.42 and 0.43 m, without and with a 0.5 m/s press, on the plane and the planet.
/// Overhangs of 0.5 to 9 degrees: face 0.42 m. Jumps on the platform along a wall at 0.42 m,
/// upright or leaning back by 0.1 to 2 degrees, per lean, without and with a 0.5 m/s press.
#[test]
#[ignore = "a survey; run it on purpose"]
fn high_ledge_beside_wall_survey() {
    let scenes = |face_mm, lean| [planar_ledge(face_mm, lean), radial_ledge(face_mm, lean)];
    for config in ledge_rows() {
        let mut upright = Tally::default();
        for face_mm in [400, 410, 415, 420, 425, 430] {
            for key in scenes(face_mm, 0) {
                for press in [0.0, 0.25, 0.5, 1.0] {
                    for z0 in [0.0, 0.003, -0.002] {
                        upright.add(key, walk_off_the_ledge(key, &config, press, z0));
                    }
                }
            }
        }
        let mut overhang = Tally::default();
        for lean in [50, 200, 500, 900] {
            for key in scenes(420, -lean) {
                for press in [0.0, 0.5] {
                    overhang.add(key, walk_off_the_ledge(key, &config, press, 0.0));
                }
            }
        }
        let mut back = PerLean::default();
        for lean in [10, 15, 20, 25, 35, 40, 50, 200, 500, 900] {
            for face_mm in [415, 420, 430] {
                for key in scenes(face_mm, lean) {
                    for press in [0.0, 0.5] {
                        back.add(lean, key, walk_off_the_ledge(key, &config, press, 0.0));
                    }
                }
            }
        }
        let mut jump = PerLean::default();
        for lean in [0, 10, 15, 20, 25, 35, 50, 200] {
            for key in scenes(420, lean) {
                for press in [0.0, 0.5] {
                    jump.add(lean, key, jump_beside_the_wall(key, &config, press));
                }
            }
        }
        println!("{}: upright {upright}; overhang {overhang}", config.name);
        println!("    walking off, leaning back {back}");
        println!("    jumping {jump}");
    }
}

/// Tallies per lean of the wall, in hundredths of a degree, and over all leans.
#[derive(Default)]
struct PerLean {
    leans: Vec<(i16, Tally)>,
    all: Tally,
}

impl PerLean {
    fn add(&mut self, lean_cdeg: i16, key: SceneKey, outcome: LedgeOutcome) {
        if self.leans.last().is_none_or(|(lean, _)| *lean != lean_cdeg) {
            self.leans.push((lean_cdeg, Tally::default()));
        }
        self.leans.last_mut().unwrap().1.add(key, outcome);
        self.all.add(key, outcome);
    }
}

impl std::fmt::Display for PerLean {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        for (lean, tally) in &self.leans {
            write!(
                f,
                "{} deg: {}+{}/{}, ",
                f64::from(*lean) / 100.0,
                tally.plane,
                tally.planet,
                tally.played
            )?;
        }
        write!(f, "routes {:?}", self.all.by_route)
    }
}

/// Hangs and missed landings out of the variants played, on the plane and on the planet.
#[derive(Default)]
struct Tally {
    played: usize,
    plane: usize,
    planet: usize,
    by_route: std::collections::BTreeMap<&'static str, usize>,
}

impl Tally {
    fn add(&mut self, key: SceneKey, outcome: LedgeOutcome) {
        self.played += 1;
        if outcome == LedgeOutcome::Falls {
            return;
        }
        if key.is_radial() {
            self.planet += 1;
        } else {
            self.plane += 1;
        }
        *self.by_route.entry(outcome.route()).or_default() += 1;
    }
}

impl std::fmt::Display for Tally {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{} of {} hang or do not land (plane {}, planet {}; {:?})",
            self.plane + self.planet,
            self.played,
            self.plane,
            self.planet,
            self.by_route
        )
    }
}

/// The wall face of the pinned ledge test: 5 mm inside the padded capsule's reach at z = 0.
const WALL_FACE_MM: u16 = 415;

fn planar_ledge(face_mm: u16, lean_cdeg: i16) -> SceneKey {
    SceneKey::HighLedgeBesideWall { face_mm, lean_cdeg }
}

fn radial_ledge(face_mm: u16, lean_cdeg: i16) -> SceneKey {
    SceneKey::RadialHighLedgeBesideWall { face_mm, lean_cdeg }
}

/// Every pinned row and the survey rows with the Q5 snap changes.
fn ledge_rows() -> Vec<Config> {
    Config::pinned()
        .into_iter()
        .chain(
            ["d2-noq4-snap-refresh", "d2-noq4-snap-edges", "d2-floor"]
                .into_iter()
                .map(Config::named),
        )
        .collect()
}

/// How a walk off the high ledge, or a jump beside its wall, ended.
#[derive(Clone, Copy, Debug, PartialEq)]
enum LedgeOutcome {
    /// Never grounded in the air, and landed: on the terrain past the edge, on the platform
    /// after a jump.
    Falls,
    /// Grounded in the air at `tick`, `height` metres above the ground it would land on.
    Hangs {
        tick: usize,
        height: f64,
        snapped: bool,
        ground: GroundState,
    },
    /// Not grounded on the ground it would land on at the end of the run.
    NoLanding { height: f64 },
}

impl LedgeOutcome {
    /// What held the character in the air: the Q5 snap or Jolt's ground state.
    fn route(self) -> &'static str {
        match self {
            Self::Hangs { snapped: true, .. } => "snap",
            Self::Hangs { .. } => "Jolt's ground state",
            Self::Falls | Self::NoLanding { .. } => "no landing",
        }
    }
}

/// Plays 120 ticks of `config` from x = -1 on the platform of `key`, offset `z0` across the wall,
/// walking +x at 2 m/s and pressing into the wall at `press` m/s.
fn walk_off_the_ledge(key: SceneKey, config: &Config, press: f64, z0: f64) -> LedgeOutcome {
    let mut scene = Scene::build(key);
    let start = [-1.0, HIGH_LEDGE_DROP + f64::from(REST_HEIGHT), z0];
    let walker = config.create_character(&mut scene, start);
    let mut carry = Carry::RESTING;
    let mut at = start;
    let mut grounded = false;
    for t in 0..120 {
        let up = scene.up.up_at(at);
        let flat = |d: V3| normalize(sub(d, scale(up, dot(d, up))));
        let velocity = add(
            scale(flat([1.0, 0.0, 0.0]), 2.0),
            scale(flat([0.0, 0.0, 1.0]), press),
        );
        let desired = scale(velocity, f64::from(DT));
        let report = tick(&mut scene, &walker, config, &mut carry, desired, None);
        scene.place_actor(report.end, from_y_to(report.up));
        at = report.end;
        grounded = report.grounded;
        let height = height_above_terrain(key, at);
        let past_edge = at[0] > f64::from(RADIUS) + 0.05;
        if grounded && past_edge && height > f64::from(REST_HEIGHT) + 0.05 {
            return LedgeOutcome::Hangs {
                tick: t,
                height,
                snapped: report.snapped,
                ground: report.ground,
            };
        }
    }
    let height = height_above_terrain(key, at);
    if grounded && (height - f64::from(REST_HEIGHT)).abs() < 0.05 {
        LedgeOutcome::Falls
    } else {
        LedgeOutcome::NoLanding { height }
    }
}

/// Plays 80 ticks of `config` jumping at 4 m/s from rest at x = -3.5 on the platform of `key`,
/// walking +x at 2 m/s and pressing into the wall at `press` m/s; it lands on the platform. A
/// tick grounded more than 0.05 m above the rest height over the platform's top is a hang.
fn jump_beside_the_wall(key: SceneKey, config: &Config, press: f64) -> LedgeOutcome {
    let mut scene = Scene::build(key);
    let rest = HIGH_LEDGE_DROP + f64::from(REST_HEIGHT);
    let start = [-3.5, rest, 0.0];
    let walker = config.create_character(&mut scene, start);
    let mut carry = Carry {
        vel_up: 4.0,
        grounded: false,
    };
    let mut at = start;
    let mut grounded = false;
    for t in 0..80 {
        let up = scene.up.up_at(at);
        let flat = |d: V3| normalize(sub(d, scale(up, dot(d, up))));
        let velocity = add(
            scale(flat([1.0, 0.0, 0.0]), 2.0),
            scale(flat([0.0, 0.0, 1.0]), press),
        );
        let desired = scale(velocity, f64::from(DT));
        let report = tick(&mut scene, &walker, config, &mut carry, desired, None);
        scene.place_actor(report.end, from_y_to(report.up));
        at = report.end;
        grounded = report.grounded;
        // The platform's top is the world plane y = HIGH_LEDGE_DROP on the plane and the planet.
        let height = at[1] - HIGH_LEDGE_DROP;
        if grounded && height > f64::from(REST_HEIGHT) + 0.05 {
            return LedgeOutcome::Hangs {
                tick: t,
                height,
                snapped: report.snapped,
                ground: report.ground,
            };
        }
    }
    let height = at[1] - HIGH_LEDGE_DROP;
    if grounded && (at[1] - rest).abs() < 0.05 {
        LedgeOutcome::Falls
    } else {
        LedgeOutcome::NoLanding { height }
    }
}

/// Height of body origin `origin` above the flat terrain of `key`, along up.
fn height_above_terrain(key: SceneKey, origin: V3) -> f64 {
    if key.is_radial() {
        norm(sub(origin, CENTRE)) - R
    } else {
        origin[1]
    }
}
