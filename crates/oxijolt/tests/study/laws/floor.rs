//! Law 2: the character stays on the floor over convex crests and walking down slopes just
//! under the limit.
//!
//! Heightfield ramps, ridges and planes at 40, 43 and 44.5 degrees at the three gaits (crest
//! descent, crest ascent, up and over a ridge, a long descent), a 0.15 m ledge (sharp and
//! rounded) and tilted box ramps. On every tick whose closest geometry is walkable the caller
//! must report grounded and the gap must stay within `GAP_TOLERANCE`.

use super::{
    grounds, missing_witness, resting, Case, Check as LawCheck, Column, Input, Outcome, Run, Scenes,
};
use crate::common::math::{dot, norm, scale, sub, V3};
use crate::common::walker::Carry;
use crate::common::DT;
use crate::study::scenes::{tenths, Scene, SceneKey};

/// The angles, degrees.
pub const ANGLES: [f64; 3] = [40.0, 43.0, 44.5];
/// The game's gaits, m/s (spec D.3).
pub const GAITS: [f64; 3] = [1.6, 3.5, 7.0];

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Check {
    /// Scene-local direction of travel.
    pub dir: V3,
    /// How far, metres along `dir`, the origin must get from the start.
    pub witness: f64,
    /// Report only: no predicate.
    pub report: bool,
}

/// Ticks to cover `flat` metres at `speed` plus `slope` metres climbing at `deg`, with a
/// margin, at most 300.
fn ticks_for(speed: f64, flat: f64, slope: f64, deg: f64) -> usize {
    let climb = speed * deg.to_radians().cos().powi(2) * 0.85;
    let seconds = flat / speed + slope / climb;
    ((seconds / f64::from(DT)) * 1.1).ceil().min(300.0) as usize
}

#[allow(clippy::too_many_arguments)]
fn case(
    scenes: &mut Scenes,
    id: String,
    key: SceneKey,
    x: f64,
    dir: V3,
    speed: f64,
    witness: f64,
    ticks: usize,
    report: bool,
) -> Case {
    let start = resting(scenes.get(key), x, 0.0);
    Case {
        id,
        column: Column::Floor,
        scene: key,
        start,
        rest_gap: Some(0.0),
        carry: Carry::RESTING,
        input: Input::walk(dir, speed),
        ticks,
        check: LawCheck::Floor(Check {
            dir,
            witness,
            report,
        }),
        reference: None,
        report_only: report,
    }
}

pub fn cases(scenes: &mut Scenes) -> Vec<Case> {
    let mut cases = Vec::new();
    let (plus, minus) = ([1.0, 0.0, 0.0], [-1.0, 0.0, 0.0]);
    for deg in ANGLES {
        let deg10 = tenths(deg);
        for speed in GAITS {
            let ramp = SceneKey::Ramp {
                deg10,
                tilted: false,
            };
            let ridge = SceneKey::Ridge {
                deg10,
                tilted: false,
            };
            let plane = SceneKey::plane(deg);
            let down = ticks_for(speed, 4.0, 0.0, deg);
            let up = ticks_for(speed, 2.0, 2.0, deg);
            cases.push(case(
                scenes,
                format!("2/ramp{deg}/crest-descent/v{speed}"),
                ramp,
                8.0,
                minus,
                speed,
                3.0,
                down,
                false,
            ));
            cases.push(case(
                scenes,
                format!("2/ramp{deg}/crest-ascent/v{speed}"),
                ramp,
                4.0,
                plus,
                speed,
                3.0,
                up,
                false,
            ));
            cases.push(case(
                scenes,
                format!("2/ridge{deg}/over/v{speed}"),
                ridge,
                -2.0,
                plus,
                speed,
                3.0,
                up,
                false,
            ));
            cases.push(case(
                scenes,
                format!("2/plane{deg}/descent/v{speed}"),
                plane,
                8.0,
                minus,
                speed,
                3.0,
                down,
                false,
            ));
        }
    }
    for speed in GAITS {
        for rounded in [false, true] {
            let key = SceneKey::Ledge {
                drop_mm: 150,
                rounded,
            };
            let r = if rounded { "rounded" } else { "sharp" };
            let ticks = ticks_for(speed, 4.0, 0.0, 0.0);
            cases.push(case(
                scenes,
                format!("2/ledge0.15-{r}/v{speed}"),
                key,
                -2.0,
                plus,
                speed,
                3.0,
                ticks,
                false,
            ));
        }
        for deg in [40.0, 44.5] {
            let key = SceneKey::BoxRamp {
                deg10: tenths(deg),
                tilted: false,
            };
            let ticks = ticks_for(speed, 4.0, 0.0, deg);
            cases.push(case(
                scenes,
                format!("2/boxramp{deg}/descent/v{speed}"),
                key,
                4.0,
                minus,
                speed,
                3.0,
                ticks,
                false,
            ));
        }
    }
    let key = SceneKey::Ledge {
        drop_mm: 500,
        rounded: false,
    };
    cases.push(case(
        scenes,
        "2-report/ledge0.5-sharp/v3.5".to_owned(),
        key,
        -2.0,
        plus,
        3.5,
        3.0,
        ticks_for(3.5, 4.0, 0.0, 0.0),
        true,
    ));
    cases
}

/// How far the run got along scene-local `dir`, metres, measured horizontally.
pub fn progress(scene: &Scene, run: &Run, dir: V3) -> f64 {
    let last = run.reports.last().unwrap();
    let d = scene.frame.to_world_dir(dir);
    let flat = sub(d, scale(last.up, dot(d, last.up)));
    dot(sub(last.end, run.start), scale(flat, 1.0 / norm(flat)))
}

pub fn judge(scene: &Scene, check: Check, run: &Run) -> Outcome {
    if check.report {
        return Outcome::Pass;
    }
    if progress(scene, run, check.dir) < check.witness {
        return missing_witness(run, "the route past the feature");
    }
    on_the_floor(scene, run, 0)
}

/// The law 2 predicate from tick `from`: grounded with `|gap| <= GAP_TOLERANCE` on every tick
/// whose closest geometry is walkable.
pub fn on_the_floor(scene: &Scene, run: &Run, from: usize) -> Outcome {
    let grounds = grounds(scene, run);
    for (t, report) in run.reports.iter().enumerate().skip(from) {
        if !grounds[t].walkable_applicable(report.up) {
            continue;
        }
        if !report.grounded {
            return Outcome::fail(t, "not grounded on walkable ground", grounds[t].gap);
        }
        if grounds[t].gap.abs() > super::GAP_TOLERANCE {
            return Outcome::fail(t, "gap", grounds[t].gap);
        }
    }
    Outcome::Pass
}
