//! Law 1: walkable up to 45 degrees, steeper slides, never grounded on a face above the limit.
//!
//! Heightfield planes at 30, 40, 43 and 44.5 degrees (walkable) and 45.5, 50 and 60 degrees
//! (steep), nine resting starts each, standing still and walking uphill at 2 m/s for 120 ticks;
//! a 50 degree ridge started 5 cm off its apex. Exactly 45 degrees is a report-only family.

use super::{
    grounds, missing_witness, resting, rise, Case, Check as LawCheck, Column, Input, Outcome, Run,
};
use crate::common::math::{dot, norm, scale, sub, V3};
use crate::common::walker::{Carry, G};
use crate::common::DT;
use crate::study::scenes::{Scene, SceneKey};

/// The walkable angles, degrees.
pub const WALKABLE: [f64; 4] = [30.0, 40.0, 43.0, 44.5];
/// The steep angles, degrees.
pub const STEEP: [f64; 3] = [45.5, 50.0, 60.0];
const STARTS_X: [f64; 3] = [-4.0, 0.0, 4.0];
const STARTS_Z: [f64; 3] = [0.0, 0.37, 0.61];
const TICKS: usize = 120;
/// Ticks a case needs on the faces it is about.
const WITNESS_TICKS: usize = 30;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Check {
    Walkable {
        deg: f64,
        uphill: bool,
    },
    Steep {
        uphill: bool,
    },
    SteepRidge,
    /// Report only, exactly 45 degrees: fails on the first tick not grounded.
    Boundary,
}

/// The law 1 cases; `boundary` gives the report-only 45 degree family instead.
pub fn cases(scenes: &mut super::Scenes, boundary: bool) -> Vec<Case> {
    let mut cases = Vec::new();
    let angles: Vec<f64> = if boundary {
        vec![45.0]
    } else {
        WALKABLE.iter().chain(&STEEP).copied().collect()
    };
    for deg in angles {
        let key = SceneKey::plane(deg);
        let scene = scenes.get(key);
        for x in STARTS_X {
            for z in STARTS_Z {
                let start = resting(scene, x, z);
                for uphill in [false, true] {
                    let check = if boundary {
                        Check::Boundary
                    } else if deg < 45.0 {
                        Check::Walkable { deg, uphill }
                    } else {
                        Check::Steep { uphill }
                    };
                    let input = if uphill {
                        Input::Walk {
                            dir: [1.0, 0.0, 0.0],
                            speed: 2.0,
                            stop: Some(15.0),
                        }
                    } else {
                        Input::Still
                    };
                    let family = if boundary { "1-boundary" } else { "1" };
                    cases.push(Case {
                        id: format!(
                            "{family}/plane{deg}/x{x}z{z}/{}",
                            if uphill { "uphill" } else { "still" }
                        ),
                        column: Column::Slope,
                        scene: key,
                        start,
                        rest_gap: Some(0.0),
                        carry: Carry::RESTING,
                        input,
                        ticks: TICKS,
                        check: LawCheck::Slope(check),
                        reference: None,
                        report_only: boundary,
                    });
                }
            }
        }
    }
    if !boundary {
        let key = SceneKey::Ridge {
            deg10: 500,
            tilted: false,
        };
        let start = resting(scenes.get(key), 0.05, 0.0);
        cases.push(Case {
            id: "1/ridge50/apex/still".to_owned(),
            column: Column::Slope,
            scene: key,
            start,
            rest_gap: Some(0.0),
            carry: Carry::RESTING,
            input: Input::Still,
            ticks: 60,
            check: LawCheck::Slope(Check::SteepRidge),
            reference: None,
            report_only: false,
        });
    }
    cases
}

/// The scene's horizontal +x at `up`.
fn uphill_axis(scene: &Scene, up: V3) -> V3 {
    let x = scene.frame.to_world_dir([1.0, 0.0, 0.0]);
    let flat = sub(x, scale(up, dot(x, up)));
    scale(flat, 1.0 / norm(flat))
}

pub fn judge(scene: &Scene, check: Check, run: &Run) -> Outcome {
    let grounds = grounds(scene, run);
    match check {
        Check::Walkable { deg, uphill } => {
            let applicable: Vec<usize> = (0..run.reports.len())
                .filter(|&t| grounds[t].walkable_applicable(run.reports[t].up))
                .collect();
            if applicable.len() < WITNESS_TICKS {
                return missing_witness(run, "30 ticks on walkable faces");
            }
            for &t in &applicable {
                let report = &run.reports[t];
                if !report.grounded {
                    return Outcome::fail(t, "not grounded on walkable ground", 0.0);
                }
                if uphill && grounds[t].gap.abs() > super::GAP_TOLERANCE {
                    return Outcome::fail(t, "gap", grounds[t].gap);
                }
                if !uphill {
                    let drift = norm(sub(report.end, run.start));
                    if drift >= super::STILL_DRIFT {
                        return Outcome::fail(t, "still drift", drift);
                    }
                }
            }
            if uphill {
                let theta = deg.to_radians();
                let dt = f64::from(DT);
                let g = f64::from(G);
                let per_tick = |speed: f64| {
                    (speed * theta.cos().powi(2) - g * dt * theta.sin() * theta.cos()) * dt
                };
                let predicted: f64 = run
                    .reports
                    .iter()
                    .map(|report| norm(report.velocity_wanted()))
                    .filter(|&speed| speed > 0.0)
                    .map(per_tick)
                    .sum();
                let last = run.reports.last().unwrap();
                let axis = uphill_axis(scene, last.up);
                let progress = dot(sub(last.end, run.start), axis);
                if progress < predicted - 0.05 {
                    return Outcome::fail(
                        run.reports.len() - 1,
                        "uphill progress short",
                        progress - predicted,
                    );
                }
            }
            Outcome::Pass
        }
        Check::Steep { uphill } => steep(scene, run, &grounds, uphill, true),
        Check::SteepRidge => steep(scene, run, &grounds, false, false),
        Check::Boundary => {
            let ungrounded = run.reports.iter().filter(|r| !r.grounded).count();
            match run.reports.iter().position(|r| !r.grounded) {
                Some(t) => Outcome::fail(t, "not grounded at 45 degrees", ungrounded as f64),
                None => Outcome::Pass,
            }
        }
    }
}

/// The steep predicates on interior-steep ticks: never grounded; walking uphill never rises;
/// standing still the downhill step is positive from tick 30 and does not shrink over spec
/// G.4 #20's window (ticks 30 to 40).
fn steep(
    scene: &Scene,
    run: &Run,
    grounds: &[super::Ground],
    uphill: bool,
    plane: bool,
) -> Outcome {
    let steep: Vec<bool> = (0..run.reports.len())
        .map(|t| grounds[t].interior_steep(run.reports[t].up))
        .collect();
    if steep.iter().filter(|&&s| s).count() < WITNESS_TICKS {
        return missing_witness(run, "30 ticks on interior steep faces");
    }
    let mut last_step: Option<f64> = None;
    for (t, report) in run.reports.iter().enumerate() {
        if !steep[t] {
            last_step = None;
            continue;
        }
        if report.grounded {
            return Outcome::fail(t, "grounded on a steep face", 0.0);
        }
        if uphill && rise(report) > super::STEEP_RISE {
            return Outcome::fail(t, "rise on a steep face", rise(report));
        }
        if plane && !uphill && t >= 30 {
            let downhill = scale(uphill_axis(scene, report.up), -1.0);
            let step = dot(sub(report.end, report.start), downhill);
            if step <= 0.0 {
                return Outcome::fail(t, "no downhill step", step);
            }
            if t <= 40 && last_step.is_some_and(|last| step < last - 1e-5) {
                return Outcome::fail(t, "downhill step shrank", step - last_step.unwrap());
            }
            last_step = Some(step);
        }
    }
    Outcome::Pass
}
