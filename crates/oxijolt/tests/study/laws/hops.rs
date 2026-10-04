//! Law 4: no hops walking downhill.
//!
//! Heightfield planes at 30, 40, 43 and 44.5 degrees and box ramps at 40 and 44.5 degrees, at
//! the three gaits: a steady descent, a descent from rest, a stop mid-slope and restart, and a
//! descent at 45 degrees to the fall line. On every tick whose closest geometry is walkable the
//! character must not rise along up by more than `HOP_RISE`, must be grounded and must keep the
//! gap within `GAP_TOLERANCE`.

use super::floor::GAITS;
use super::{
    grounds, missing_witness, resting, rise, Case, Check as LawCheck, Column, Input, Outcome, Run,
    Scenes,
};
use crate::common::math::{dot, norm, scale, sub, V3};
use crate::common::walker::Carry;
use crate::common::DT;
use crate::study::scenes::{tenths, Scene, SceneKey};

/// The plane angles, degrees.
pub const PLANES: [f64; 4] = [30.0, 40.0, 43.0, 44.5];
/// The box ramp angles, degrees.
pub const BOX_RAMPS: [f64; 2] = [40.0, 44.5];
const TICKS: usize = 90;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Check {
    /// The first tick judged.
    pub from: usize,
    /// Moving ticks needed after a restart, when the input pauses.
    pub after_restart: Option<usize>,
}

pub fn cases(scenes: &mut Scenes) -> Vec<Case> {
    let mut cases = Vec::new();
    let mut scenes_list: Vec<(String, SceneKey, f64)> = PLANES
        .iter()
        .map(|&deg| (format!("plane{deg}"), SceneKey::plane(deg), 12.0))
        .collect();
    for deg in BOX_RAMPS {
        let key = SceneKey::BoxRamp {
            deg10: tenths(deg),
            tilted: false,
        };
        scenes_list.push((format!("boxramp{deg}"), key, 7.0));
    }
    let down: V3 = [-1.0, 0.0, 0.0];
    let diagonal: V3 = [
        -std::f64::consts::FRAC_1_SQRT_2,
        0.0,
        std::f64::consts::FRAC_1_SQRT_2,
    ];
    for (name, key, x) in scenes_list {
        let start = resting(scenes.get(key), x, 0.0);
        for speed in GAITS {
            let variants = [
                ("steady", Input::walk(down, speed), 10, None),
                ("from-rest", Input::walk(down, speed), 0, None),
                (
                    "stop-restart",
                    Input::StopRestart {
                        dir: down,
                        speed,
                        walk: 20,
                        pause: 20,
                    },
                    0,
                    Some(10),
                ),
                ("diagonal", Input::walk(diagonal, speed), 0, None),
            ];
            for (input_name, input, from, after_restart) in variants {
                cases.push(Case {
                    id: format!("4/{name}/{input_name}/v{speed}"),
                    column: Column::Hops,
                    scene: key,
                    start,
                    rest_gap: Some(0.0),
                    carry: Carry::RESTING,
                    input,
                    ticks: TICKS,
                    check: LawCheck::Hops(Check {
                        from,
                        after_restart,
                    }),
                    reference: None,
                    report_only: false,
                });
            }
        }
    }
    cases
}

pub fn judge(scene: &Scene, check: Check, run: &Run) -> Outcome {
    if let Some(t) = run.reports.iter().position(|report| report.vel_up > 0.0) {
        return Outcome::Invalid {
            reason: format!("the caller carried a rising vel_up at tick {t}"),
        };
    }
    let moving: Vec<usize> = (0..run.reports.len())
        .filter(|&t| norm(run.reports[t].velocity_wanted()) > 0.0)
        .collect();
    let mut progress = 0.0;
    let mut wanted = 0.0;
    for &t in &moving {
        let report = &run.reports[t];
        let v = report.velocity_wanted();
        let speed = norm(v);
        let done = sub(report.end, report.start);
        progress += dot(done, scale(v, 1.0 / speed));
        wanted += speed * f64::from(DT);
    }
    if progress < 0.5 * wanted {
        return missing_witness(run, "half the wanted descent");
    }
    if let Some(needed) = check.after_restart {
        let restarted = moving.iter().filter(|&&t| t >= 40).count();
        if restarted < needed {
            return missing_witness(run, "moving ticks after the restart");
        }
    }
    no_hops(scene, run, check.from)
}

/// The law 4 predicate from tick `from`.
pub fn no_hops(scene: &Scene, run: &Run, from: usize) -> Outcome {
    let grounds = grounds(scene, run);
    for (t, report) in run.reports.iter().enumerate().skip(from) {
        if !grounds[t].walkable_applicable(report.up) {
            continue;
        }
        if rise(report) > super::HOP_RISE {
            return Outcome::fail(t, "rise", rise(report));
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
