//! Law 6: heightfield chunk seams (adjacent heightfield bodies) behave like one surface.
//!
//! Each run on two 33-sample bodies meeting at x = 0 is compared with the same run on one
//! 65-sample body over the same span: flat, a 30 degree rise and a 44.5 degree descent, crossed
//! perpendicularly, at 45 degrees and walked along the seam, at 3.5 and 7 m/s for 180 ticks.

use super::{
    grounds, missing_witness, Case, Check as LawCheck, Column, Input, Outcome, Reference, Run,
    Scenes,
};
use crate::common::math::{dot, norm, scale, sub, V3};
use crate::common::walker::Carry;
use crate::study::scenes::{Profile, SceneKey, Split};

const PROFILES: [Profile; 3] = [Profile::Flat, Profile::Rise30, Profile::Descent44_5];
const SPEEDS: [f64; 2] = [3.5, 7.0];
const TICKS: usize = 180;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Check {
    /// Scene-local direction of travel.
    pub dir: V3,
    /// Whether the run walks along the seam (else across it).
    pub along: bool,
}

/// The law 6 cases on `split` (`Pair`, or `Raised` for the control), each compared with the
/// continuous field.
pub fn cases(scenes: &mut Scenes, split: Split) -> Vec<Case> {
    let mut cases = Vec::new();
    let diagonal = std::f64::consts::FRAC_1_SQRT_2;
    for profile in PROFILES {
        let key = SceneKey::Seam { profile, split };
        let continuous = SceneKey::Seam {
            profile,
            split: Split::Continuous,
        };
        let sign = if profile == Profile::Descent44_5 {
            -1.0
        } else {
            1.0
        };
        let routes = [
            ("across", [sign, 0.0, 0.0], [-3.0 * sign, 0.0], false),
            (
                "diagonal",
                [sign * diagonal, 0.0, diagonal],
                [-3.0 * sign, -3.0],
                false,
            ),
            ("along", [0.0, 0.0, 1.0], [0.0, -7.0], true),
        ];
        for (route, dir, [x, z], along) in routes {
            for speed in SPEEDS {
                let start = super::resting(scenes.get(continuous), x, z);
                let input = Input::walk(dir, speed);
                let prefix = if split == Split::Raised {
                    "6-control"
                } else {
                    "6"
                };
                cases.push(Case {
                    id: format!("{prefix}/{}/{route}/v{speed}", profile.name()),
                    column: Column::Seam,
                    scene: key,
                    start,
                    rest_gap: Some(0.0),
                    carry: Carry::RESTING,
                    input,
                    ticks: TICKS,
                    check: LawCheck::Seam(Check { dir, along }),
                    reference: Some(Reference {
                        scene: continuous,
                        start,
                        input,
                    }),
                    report_only: split == Split::Raised,
                });
            }
        }
    }
    cases
}

/// The largest difference between the decoded heights of a seam scene and of the continuous
/// scene at their shared samples.
pub fn surface_mismatch(scenes: &mut Scenes, seam: SceneKey, continuous: SceneKey) -> f64 {
    let whole = scenes.get(continuous).surface.fields[0].clone();
    let parts = scenes.get(seam).surface.fields.clone();
    let mut worst: f64 = 0.0;
    for part in &parts {
        for position in &part.positions {
            let Some(local) = position else { continue };
            let world = part.frame.to_world_point(*local);
            let in_whole = whole.frame.to_local_point(world);
            let Some(height) = whole.height_at(in_whole[0], in_whole[2]) else {
                continue;
            };
            worst = worst.max((height - in_whole[1]).abs());
        }
    }
    worst
}

pub fn judge(
    scenes: &mut Scenes,
    check: Check,
    seam: SceneKey,
    continuous: Option<SceneKey>,
    run: &Run,
    reference: Option<&Run>,
) -> Outcome {
    let (Some(continuous), Some(reference)) = (continuous, reference) else {
        return Outcome::Invalid {
            reason: "a seam case needs its continuous run".to_owned(),
        };
    };
    let mismatch = surface_mismatch(scenes, seam, continuous);
    let raised = matches!(
        seam,
        SceneKey::Seam {
            split: Split::Raised,
            ..
        }
    );
    if mismatch > super::SEAM_SURFACE && !raised {
        return Outcome::Invalid {
            reason: format!("seam and continuous surfaces differ by {mismatch} m"),
        };
    }
    let scene = scenes.get(seam);
    let travelled = |r: &Run| {
        let last = r.reports.last().unwrap();
        let d = scene.frame.to_world_dir(check.dir);
        dot(sub(last.end, r.start), d)
    };
    let witness = if check.along {
        travelled(run) >= 2.0
    } else {
        let end = scene.frame.to_local_point(run.reports.last().unwrap().end)[0];
        end * check.dir[0].signum() >= 1.0
    };
    if !witness {
        return missing_witness(run, "the route over the seam");
    }
    let seam_grounds = grounds(scene, run);
    let whole_grounds = grounds(scenes.get(continuous), reference);
    let mut path = [0.0, 0.0];
    for (t, (a, b)) in run.reports.iter().zip(&reference.reports).enumerate() {
        for (report, ground) in [(a, &seam_grounds[t]), (b, &whole_grounds[t])] {
            if ground.walkable_applicable(report.up) && !report.grounded {
                return Outcome::fail(t, "not grounded on walkable ground", ground.gap);
            }
        }
        if a.ground != b.ground {
            return Outcome::fail(t, "ground state differs", 0.0);
        }
        let delta = sub(a.end, b.end);
        let along_up = dot(delta, a.up);
        if along_up.abs() > super::SEAM_UP {
            return Outcome::fail(t, "height differs", along_up);
        }
        let across = norm(sub(delta, scale(a.up, along_up)));
        if across > super::SEAM_ACROSS {
            return Outcome::fail(t, "position differs", across);
        }
        path[0] += norm(sub(a.end, a.start));
        path[1] += norm(sub(b.end, b.start));
    }
    if (path[0] - path[1]).abs() > super::SEAM_PATH {
        return Outcome::fail(
            run.reports.len() - 1,
            "path length differs",
            path[0] - path[1],
        );
    }
    Outcome::Pass
}
