//! The six law families as scenario cases, the runner that plays a case under a configuration,
//! and the oracle measurements the predicates share.
//!
//! A case starts from rest (or from a stated overlap or burial), feeds a scripted input for a
//! fixed number of ticks through the study caller and returns every tick's report. A
//! predicate judges the run against the scene's decoded geometry and returns an [`Outcome`].
//! A case whose witness (the situation the law is about) did not occur is `Invalid`: a harness
//! error, not evidence; when the controller was blocked instead, the case fails.

pub mod floor;
pub mod hops;
pub mod pushout;
pub mod radial;
pub mod seam;
pub mod slope;
pub mod step;

use std::collections::BTreeMap;

use oxijolt::*;

use super::config::Config;
use super::controller::{origin_of, tick, TickReport, TraceTick};
use super::geometry::{classify, Element, FaceClass, Kind};
use super::scenes::{Scene, SceneKey};
use crate::common::math::{add, dot, norm, scale, sub, V3};
use crate::common::walker::{from_y_to, Carry, RADIUS};
use crate::common::DT;

/// Largest gap, metres, between the padded capsule and walkable ground on a tick that counts as
/// on the floor (spec G.4 #13, #14).
pub const GAP_TOLERANCE: f64 = 0.03;
/// Largest rise along up, metres, of a tick walking down (law 4).
pub const HOP_RISE: f64 = 1e-3;
/// Largest drift, metres, of a still run on walkable ground (spec G.4 #20: "stands, < 1e-3").
pub const STILL_DRIFT: f64 = 1e-3;
/// Largest rise along up, metres, of a tick on steep terrain (law 1).
pub const STEEP_RISE: f64 = 1e-3;
/// Rest height tolerance, metres, after a recovery (spec G.4 #16).
pub const REST_TOLERANCE: f64 = 0.05;
/// Least push away from an overlapped wall, metres (spec G.4 #17).
pub const WALL_PUSH: f64 = 0.04;
/// Largest part of a push-out, metres per tick, that may show up in the output velocity
/// (law 5v).
pub const PUSH_LEAK: f64 = 2e-3;
/// Largest difference along up, metres, between a seam run and the continuous run (law 6).
pub const SEAM_UP: f64 = 2e-3;
/// Largest tangential difference, metres, between a seam run and the continuous run.
pub const SEAM_ACROSS: f64 = 5e-3;
/// Largest difference in path length, metres, between a seam run and the continuous run.
pub const SEAM_PATH: f64 = 0.01;
/// Largest difference, metres, between the decoded heights of the seam fields and the
/// continuous field at a shared sample, for the comparison to be valid.
pub const SEAM_SURFACE: f64 = 5e-4;

/// The columns of the study table.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Column {
    Slope,
    Floor,
    StepSharp,
    StepRounded,
    Hops,
    PushVelocity,
    PushRecovery,
    Seam,
    Radial,
}

impl Column {
    pub const ALL: [Column; 9] = [
        Column::Slope,
        Column::Floor,
        Column::StepSharp,
        Column::StepRounded,
        Column::Hops,
        Column::PushVelocity,
        Column::PushRecovery,
        Column::Seam,
        Column::Radial,
    ];

    pub fn label(self) -> &'static str {
        match self {
            Column::Slope => "1",
            Column::Floor => "2",
            Column::StepSharp => "3s",
            Column::StepRounded => "3r",
            Column::Hops => "4",
            Column::PushVelocity => "5v",
            Column::PushRecovery => "5r",
            Column::Seam => "6",
            Column::Radial => "radial",
        }
    }

    pub fn index(self) -> usize {
        Column::ALL.iter().position(|&c| c == self).unwrap()
    }
}

/// The verdict of one case.
#[derive(Clone, Debug, PartialEq)]
pub enum Outcome {
    Pass,
    Fail {
        tick: usize,
        metric: &'static str,
        value: f64,
    },
    Invalid {
        reason: String,
    },
}

impl Outcome {
    pub fn fail(tick: usize, metric: &'static str, value: f64) -> Self {
        Self::Fail {
            tick,
            metric,
            value,
        }
    }

    pub fn is_pass(&self) -> bool {
        *self == Outcome::Pass
    }
}

/// The input of a case, per tick.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Input {
    Still,
    /// Walk along the scene-local direction `dir` at `speed` m/s; stand still once the origin
    /// is `stop` metres along `dir` from the scene origin, when given.
    Walk {
        dir: V3,
        speed: f64,
        stop: Option<f64>,
    },
    /// Walk, stand still for `pause` ticks after `walk` ticks, then walk again.
    StopRestart {
        dir: V3,
        speed: f64,
        walk: usize,
        pause: usize,
    },
    /// Walk, and at tick `at` the character is put at world origin `to` before the tick.
    Teleport {
        dir: V3,
        speed: f64,
        at: usize,
        to: V3,
    },
}

impl Input {
    pub fn walk(dir: V3, speed: f64) -> Self {
        Self::Walk {
            dir,
            speed,
            stop: None,
        }
    }

    /// The displacement wanted at `tick` from world origin `origin`.
    pub fn desired(&self, scene: &Scene, tick: usize, origin: V3) -> V3 {
        let (dir, speed) = match *self {
            Input::Still => return [0.0; 3],
            Input::Walk { dir, speed, stop } => {
                let local = scene.frame.to_local_point(origin);
                if stop.is_some_and(|stop| dot(local, dir) > stop) {
                    return [0.0; 3];
                }
                (dir, speed)
            }
            Input::StopRestart {
                dir,
                speed,
                walk,
                pause,
            } => {
                if (walk..walk + pause).contains(&tick) {
                    return [0.0; 3];
                }
                (dir, speed)
            }
            Input::Teleport { dir, speed, .. } => (dir, speed),
        };
        let world_dir = scene.frame.to_world_dir(dir);
        scene.up.tangent(origin, world_dir, speed * f64::from(DT))
    }
}

/// What a case checks, with its parameters.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Check {
    Slope(slope::Check),
    Floor(floor::Check),
    Step(step::Check),
    Hops(hops::Check),
    Push(pushout::Check),
    Seam(seam::Check),
    Radial(radial::Check),
}

/// One scenario.
#[derive(Clone, Debug)]
pub struct Case {
    /// Stable id, used in the pinned table and the survey.
    pub id: String,
    pub column: Column,
    pub scene: SceneKey,
    /// The start's body origin in world space.
    pub start: V3,
    /// When set, a perturbed start is put back at this gap above the surface (resting starts).
    pub rest_gap: Option<f64>,
    pub carry: Carry,
    pub input: Input,
    pub ticks: usize,
    pub check: Check,
    /// The run the predicate compares with, if any.
    pub reference: Option<Reference>,
    /// Reported by the survey, never part of a cell.
    pub report_only: bool,
}

/// A second run of a case: another scene, start or input.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Reference {
    pub scene: SceneKey,
    pub start: V3,
    pub input: Input,
}

/// The start offsets of the perturbation family: none, and 1 mm along each tangent axis.
pub const PERTURBATIONS: [[f64; 2]; 5] = [
    [0.0, 0.0],
    [1e-3, 0.0],
    [-1e-3, 0.0],
    [0.0, 1e-3],
    [0.0, -1e-3],
];

/// A played case.
#[derive(Clone, Debug)]
pub struct Run {
    pub start: V3,
    pub padding: f32,
    pub startup: GroundState,
    pub reports: Vec<TickReport>,
    pub traces: Vec<TraceTick>,
}

/// The scenes of a set of cases, built once each.
#[derive(Default)]
pub struct Scenes(pub BTreeMap<SceneKey, Scene>);

impl Scenes {
    pub fn get(&mut self, key: SceneKey) -> &mut Scene {
        self.0.entry(key).or_insert_with(|| Scene::build(key))
    }
}

/// `point` moved along `up` until the padded capsule's lower sphere is `gap` above the surface.
pub fn rest_at(scene: &Scene, point: V3, gap: f64, padding: f32) -> V3 {
    let target = gap + f64::from(RADIUS) + f64::from(padding);
    let mut p = point;
    for _ in 0..60 {
        let up = scene.up.up_at(p);
        let Some(distance) = scene.surface.distance(p) else {
            return p;
        };
        let error = distance - target;
        if error.abs() < 1e-12 {
            break;
        }
        p = sub(p, scale(up, error));
    }
    p
}

/// The resting body origin over scene-local `(x, z)` with the default padding, found from 60 m
/// above.
pub fn resting(scene: &Scene, x: f64, z: f64) -> V3 {
    rest_at(scene, scene.frame.to_world_point([x, 60.0, z]), 0.0, 0.02)
}

/// The start of `case` under perturbation `offset`, for `config`'s padding.
pub fn perturbed_start(scene: &Scene, case: &Case, offset: [f64; 2], padding: f32) -> V3 {
    let up = scene.up.up_at(case.start);
    let axis = |local: V3| {
        let d = scene.frame.to_world_dir(local);
        let flat = sub(d, scale(up, dot(d, up)));
        scale(flat, 1.0 / norm(flat))
    };
    let shifted = add(
        case.start,
        add(
            scale(axis([1.0, 0.0, 0.0]), offset[0]),
            scale(axis([0.0, 0.0, 1.0]), offset[1]),
        ),
    );
    match case.rest_gap {
        Some(gap) => rest_at(scene, shifted, gap, padding),
        None => shifted,
    }
}

/// Plays `case` from `start` under `config` in `scene`.
pub fn play(scene: &mut Scene, config: &Config, case: &Case, start: V3, trace: bool) -> Run {
    let walker = config.create_character(scene, start);
    let padding = config.settings.padding;
    let startup = scene.world.character(walker.id).unwrap().ground_state();
    let mut carry = case.carry;
    let mut reports = Vec::with_capacity(case.ticks);
    let mut traces = Vec::new();
    for t in 0..case.ticks {
        if let Input::Teleport { at, to, .. } = case.input {
            if t == at {
                let up = scene.up.up_at(to);
                scene
                    .world
                    .character_mut(walker.id)
                    .unwrap()
                    .set_position(super::config::position_for(to, up, padding))
                    .unwrap();
            }
        }
        let origin = origin_of(&scene.world, &walker, padding);
        let desired = case.input.desired(scene, t, origin);
        let mut traced = TraceTick::default();
        let report = tick(
            scene,
            &walker,
            config,
            &mut carry,
            desired,
            trace.then_some(&mut traced),
        );
        scene.place_actor(report.end, from_y_to(report.up));
        reports.push(report);
        if trace {
            traces.push(traced);
        }
    }
    scene.world.remove_character(walker.id).unwrap();
    if config.settings.inner_body {
        scene.world.optimize_broad_phase();
    }
    Run {
        start,
        padding,
        startup,
        reports,
        traces,
    }
}

/// Plays `case` (and its reference, if any) under `config` with start offset `offset` and
/// judges it.
pub fn evaluate(scenes: &mut Scenes, config: &Config, case: &Case, offset: [f64; 2]) -> Outcome {
    let scene = scenes.get(case.scene);
    let start = perturbed_start(scene, case, offset, config.settings.padding);
    let run = play(scene, config, case, start, false);
    let reference = reference_run(scenes, config, case, start);
    judge(scenes, case, &run, reference.as_ref())
}

/// The reference run of `case` (if it has one) for a run from `start`: the reference's start
/// moved by the same offset.
pub fn reference_run(scenes: &mut Scenes, config: &Config, case: &Case, start: V3) -> Option<Run> {
    case.reference.map(|reference| {
        let other = scenes.get(reference.scene);
        let shifted = add(reference.start, sub(start, case.start));
        let mirrored = Case {
            input: reference.input,
            ..case.clone()
        };
        play(other, config, &mirrored, shifted, false)
    })
}

/// The predicate of `case` on `run`.
pub fn judge(scenes: &mut Scenes, case: &Case, run: &Run, reference: Option<&Run>) -> Outcome {
    let scene = scenes.get(case.scene);
    match case.check {
        Check::Slope(check) => slope::judge(scene, check, run),
        Check::Floor(check) => floor::judge(scene, check, run),
        Check::Step(check) => step::judge(scene, check, run),
        Check::Hops(check) => hops::judge(scene, check, run),
        Check::Push(check) => pushout::judge(scene, check, run, reference),
        Check::Seam(check) => {
            let continuous = case.reference.map(|reference| reference.scene);
            seam::judge(scenes, check, case.scene, continuous, run, reference)
        }
        Check::Radial(check) => radial::judge(scene, check, run, reference),
    }
}

/// What the oracle sees at a tick's end.
#[derive(Clone, Debug)]
pub struct Ground {
    /// Gap between the padded lower sphere and the closest element, metres.
    pub gap: f64,
    pub support: Vec<Element>,
    /// The closest element.
    pub closest: Option<Element>,
}

impl Ground {
    /// The oracle at body origin `origin` for padding `padding`.
    pub fn at(scene: &Scene, origin: V3, padding: f32) -> Self {
        let support = scene.surface.support_set(origin);
        let closest = support.first().copied();
        let gap = closest.map_or(f64::INFINITY, |e| {
            e.distance - f64::from(RADIUS) - f64::from(padding)
        });
        Self {
            gap,
            support,
            closest,
        }
    }

    /// Every element of the support set is steep terrain (an edge between steep faces, such as a
    /// steep ridge's apex, included).
    pub fn all_steep_terrain(&self, up: V3) -> bool {
        !self.support.is_empty()
            && self
                .support
                .iter()
                .all(|e| e.kind == Kind::Terrain && classify(e.normal, up) == FaceClass::Steep)
    }

    /// The closest element is walkable terrain or a walkable box top, and no element of the
    /// support set is at the boundary.
    pub fn walkable_applicable(&self, up: V3) -> bool {
        self.closest
            .is_some_and(|e| classify(e.normal, up) == FaceClass::Walkable)
            && self
                .support
                .iter()
                .all(|e| classify(e.normal, up) != FaceClass::Boundary)
    }

    /// Any element of the support set is at the boundary.
    pub fn boundary(&self, up: V3) -> bool {
        self.support
            .iter()
            .any(|e| classify(e.normal, up) == FaceClass::Boundary)
    }
}

/// The oracle at every tick's end.
pub fn grounds(scene: &Scene, run: &Run) -> Vec<Ground> {
    run.reports
        .iter()
        .map(|report| Ground::at(scene, report.end, run.padding))
        .collect()
}

/// Displacement of tick `report` along its up.
pub fn rise(report: &TickReport) -> f64 {
    dot(sub(report.end, report.start), report.up)
}

/// The first tick at which the move achieved less than a tenth of what was wanted, if any: the
/// controller was blocked there.
pub fn blocked_tick(run: &Run) -> Option<usize> {
    run.reports.iter().position(|report| {
        let wanted = norm(report.velocity_wanted());
        wanted > 0.0 && {
            let along = scale(report.velocity_wanted(), 1.0 / wanted);
            let done = sub(report.end, report.start);
            let flat = sub(done, scale(report.up, dot(done, report.up)));
            dot(flat, along) < 0.1 * wanted * f64::from(DT)
        }
    })
}

/// A missing witness: a failure where the controller was blocked, else a harness error.
pub fn missing_witness(run: &Run, what: &str) -> Outcome {
    match blocked_tick(run) {
        Some(tick) => Outcome::fail(tick, "blocked before the witness", 0.0),
        None => Outcome::Invalid {
            reason: format!("witness not reached: {what}"),
        },
    }
}

impl TickReport {
    /// The horizontal velocity the caller wanted this tick, m/s.
    pub fn velocity_wanted(&self) -> V3 {
        let v = crate::common::math::f3(self.supplied);
        sub(v, scale(self.up, dot(v, self.up)))
    }
}
