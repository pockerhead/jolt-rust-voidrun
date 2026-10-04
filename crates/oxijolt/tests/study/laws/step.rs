//! Law 3: a 0.45 m step up succeeds, 0.5 m does not.
//!
//! Box steps 0.40, 0.45 and 0.50 m high, sharp (column 3s) and with a 0.05 m convex radius
//! (column 3r), approached at 0, 30 and 60 degrees off the face normal, at 1.6 and 3.5 m/s,
//! from four lateral offsets and two distances, for the time to reach the face plus one second
//! (at least 120 ticks).

use super::{missing_witness, Case, Check as LawCheck, Column, Input, Outcome, Run, Scenes};
use crate::common::math::{norm, sub, V3};
use crate::common::walker::{Carry, CENTRE, R, RADIUS};
use crate::common::Groups;
use crate::study::frame::UpPolicy;
use crate::study::scenes::{Scene, SceneKey};

/// The step heights, metres.
pub const HEIGHTS: [f64; 3] = [0.40, 0.45, 0.50];
const HEADINGS: [f64; 3] = [0.0, 30.0, 60.0];
const SPEEDS: [f64; 2] = [1.6, 3.5];
const OFFSETS: [f64; 4] = [0.0, 0.37, 0.61, 1.13];
const DISTANCES: [f64; 2] = [1.0, 1.37];
/// Consecutive ticks a landing must hold.
const HOLD: usize = 10;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Check {
    /// Step height above the ground at the face, metres.
    pub height: f64,
    /// The face's position along the scene's x, and the ground height there (scene-local
    /// height; for radial scenes the height above the planet's surface).
    pub face: f64,
    pub ground: f64,
    /// Whether the step must be climbed (else it must not be).
    pub climb: bool,
}

/// The law 3 cases on sharp (`rounded == false`) or rounded steps.
pub fn cases(scenes: &mut Scenes, rounded: bool) -> Vec<Case> {
    let mut cases = Vec::new();
    for height in HEIGHTS {
        let key = SceneKey::Step {
            height_mm: (height * 1000.0).round() as u16,
            rounded,
        };
        for heading in HEADINGS {
            for speed in SPEEDS {
                for offset in OFFSETS {
                    for distance in DISTANCES {
                        cases.push(step_case(
                            scenes, key, height, heading, speed, offset, distance,
                        ));
                    }
                }
            }
        }
    }
    cases
}

/// One approach to the step of `key`.
pub fn step_case(
    scenes: &mut Scenes,
    key: SceneKey,
    height: f64,
    heading: f64,
    speed: f64,
    offset: f64,
    distance: f64,
) -> Case {
    let r = match key {
        SceneKey::Step { rounded: true, .. } => "r",
        _ => "s",
    };
    let start = super::resting(scenes.get(key), -(distance + f64::from(RADIUS)), offset);
    let (sin, cos) = heading.to_radians().sin_cos();
    Case {
        id: format!("3{r}/h{height}/heading{heading}/v{speed}/z{offset}/d{distance}"),
        column: if r == "r" {
            Column::StepRounded
        } else {
            Column::StepSharp
        },
        scene: key,
        start,
        rest_gap: Some(0.0),
        carry: Carry::RESTING,
        input: Input::walk([cos, 0.0, -sin], speed),
        ticks: ticks_to_reach(distance / cos, speed),
        check: LawCheck::Step(Check {
            height,
            face: 0.0,
            ground: 0.0,
            climb: height < 0.475,
        }),
        reference: None,
        report_only: false,
    }
}

/// Ticks to walk `metres` at `speed` m/s plus one second to climb and hold, at least 120.
pub fn ticks_to_reach(metres: f64, speed: f64) -> usize {
    (((metres / speed + 1.0) / f64::from(crate::common::DT)).ceil() as usize).max(120)
}

/// The first tick from which the character stands on the step for `HOLD` ticks.
pub fn landing(scene: &Scene, check: Check, run: &Run) -> Option<usize> {
    let landed = |t: usize| {
        let report = &run.reports[t];
        let local = scene.frame.to_local_point(report.end);
        let above = match scene.up {
            UpPolicy::Radial => norm(sub(report.end, CENTRE)) - R,
            UpPolicy::Frame(_) => local[1],
        };
        let feet = above - f64::from(RADIUS) - f64::from(run.padding) - check.ground;
        local[0] >= check.face + 0.05 && (feet - check.height).abs() <= 0.05 && report.grounded
    };
    (0..run.reports.len().saturating_sub(HOLD - 1)).find(|&t| (t..t + HOLD).all(landed))
}

pub fn judge(scene: &Scene, check: Check, run: &Run) -> Outcome {
    let touched = run
        .reports
        .iter()
        .any(|report| report.touched_groups & (1 << Groups::STRUCTURE) != 0);
    if !touched {
        return missing_witness(run, "the capsule touching the step");
    }
    let landed = landing(scene, check, run);
    if check.climb {
        return match landed {
            Some(_) => Outcome::Pass,
            None => Outcome::fail(run.reports.len() - 1, "did not land on the step", 0.0),
        };
    }
    if let Some(t) = landed {
        return Outcome::fail(t, "landed on a step too high", check.height);
    }
    let last = run.reports.last().unwrap();
    let before = check.face - scene.frame.to_local_point(last.end)[0];
    if before <= 0.3 {
        return Outcome::fail(
            run.reports.len() - 1,
            "ended within 0.3 m of the face",
            before,
        );
    }
    Outcome::Pass
}

/// The scene-local x of a run's end, for reports.
pub fn end_x(scene: &Scene, run: &Run) -> V3 {
    scene.frame.to_local_point(run.reports.last().unwrap().end)
}
