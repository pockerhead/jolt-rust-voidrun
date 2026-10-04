//! The radial acceptance family: the laws on the planet of radius 99 with radial up.
//!
//! Slopes near the limit use log-spiral terrain at 0.5 m sample spacing, whose faces keep a
//! constant angle to radial up (a planar slope on the planet changes its angle to up by about
//! 0.57 degrees per metre). Steps, walls, burials and the chunk seam reuse the walker fixtures at
//! the anchor.

use super::floor::{on_the_floor, GAITS};
use super::hops::no_hops;
use super::{
    grounds, missing_witness, Case, Check as LawCheck, Column, Input, Outcome, Reference, Run,
    Scenes,
};
use crate::common::math::{add, dot, norm, scale, sub, V3};
use crate::common::walker::{Carry, CENTRE, PADDING, R, REST_HEIGHT};
use crate::common::DT;
use crate::study::scenes::{resting_on_planet, Scene, SceneKey};

/// Largest share of a case's ticks that may be at the boundary angle.
const MAX_BOUNDARY_SHARE: f64 = 0.1;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Check {
    Slope(super::slope::Check),
    /// Laws 2 and 4 on one descent.
    Descent,
    Step(super::step::Check),
    Push(super::pushout::Check),
    /// Spec G.4 #15 on the seam between two flat chunks.
    Seam,
}

fn spiral(deg10: u16) -> SceneKey {
    SceneKey::RadialSpiral {
        deg10,
        spacing_mm: 500,
        bits: 16,
    }
}

#[allow(clippy::too_many_arguments)]
fn radial_case(
    id: String,
    scene: SceneKey,
    start: V3,
    rest_gap: Option<f64>,
    input: Input,
    ticks: usize,
    check: Check,
    reference: Option<Reference>,
) -> Case {
    Case {
        id,
        column: Column::Radial,
        scene,
        start,
        rest_gap,
        carry: Carry::RESTING,
        input,
        ticks,
        check: LawCheck::Radial(check),
        reference,
        report_only: false,
    }
}

pub fn cases(scenes: &mut Scenes) -> Vec<Case> {
    use super::slope::Check as Slope;
    let mut cases = Vec::new();
    let plus = [1.0, 0.0, 0.0];
    let minus = [-1.0, 0.0, 0.0];
    let uphill = Input::Walk {
        dir: plus,
        speed: 2.0,
        stop: Some(7.0),
    };
    for x in [-3.0, 0.0, 3.0] {
        let key = spiral(300);
        let start = super::resting(scenes.get(key), x, 0.0);
        cases.push(radial_case(
            format!("radial/1/spiral30/x{x}/uphill"),
            key,
            start,
            Some(0.0),
            uphill,
            120,
            Check::Slope(Slope::Walkable {
                deg: 30.0,
                uphill: true,
            }),
            None,
        ));
    }
    for x in [2.0, 4.0, 6.0] {
        let key = spiral(455);
        let start = super::resting(scenes.get(key), x, 0.0);
        for (name, input, up) in [("still", Input::Still, false), ("uphill", uphill, true)] {
            cases.push(radial_case(
                format!("radial/1/spiral45.5/x{x}/{name}"),
                key,
                start,
                Some(0.0),
                input,
                120,
                Check::Slope(Slope::Steep { uphill: up }),
                None,
            ));
        }
    }
    for deg10 in [430, 445] {
        let key = spiral(deg10);
        let start = super::resting(scenes.get(key), 7.0, 0.0);
        for speed in GAITS {
            let ticks = ((4.0 / speed / f64::from(DT)).ceil() as usize).max(90);
            cases.push(radial_case(
                format!(
                    "radial/2+4/spiral{}/descent/v{speed}",
                    f64::from(deg10) / 10.0
                ),
                key,
                start,
                Some(0.0),
                Input::walk(minus, speed),
                ticks,
                Check::Descent,
                None,
            ));
        }
    }
    let crest = SceneKey::RadialCrest { deg10: 445 };
    let start = super::resting(scenes.get(crest), 2.0, 0.0);
    for speed in GAITS {
        let ticks = (4.0 / speed / f64::from(DT) * 1.1).ceil() as usize;
        cases.push(radial_case(
            format!("radial/2+4/crest44.5/descent/v{speed}"),
            crest,
            start,
            Some(0.0),
            Input::walk(minus, speed),
            ticks,
            Check::Descent,
            None,
        ));
    }
    for height_mm in [450, 500] {
        let height = f64::from(height_mm) / 1000.0;
        let check = super::step::Check {
            height,
            face: 1.4,
            ground: 0.0,
            climb: height_mm < 475,
        };
        cases.push(radial_case(
            format!("radial/3s/h{height}"),
            SceneKey::RadialStep { height_mm },
            resting_on_planet(0.0, 0.0),
            None,
            Input::walk(plus, 2.0),
            120,
            Check::Step(check),
            None,
        ));
    }
    let wall = SceneKey::RadialWall;
    let start = resting_on_planet(0.0, 0.0);
    let along_z = Input::walk([0.0, 0.0, 1.0], 2.0);
    let away = [-1.0, 0.0, 0.0];
    let exterior = add(start, scale(away, 0.05 + f64::from(PADDING)));
    cases.push(radial_case(
        "radial/5v/wall0.05/moving".to_owned(),
        wall,
        start,
        None,
        along_z,
        10,
        Check::Push(super::pushout::Check::Velocity { normal: away }),
        Some(Reference {
            scene: wall,
            start: exterior,
            input: along_z,
        }),
    ));
    cases.push(radial_case(
        "radial/5r/wall0.05/moving".to_owned(),
        wall,
        start,
        None,
        along_z,
        10,
        Check::Push(super::pushout::Check::AwayFromWall { normal: away }),
        None,
    ));
    let buried = add(
        resting_on_planet(-5.0, -5.0),
        [0.0, -1.0 - f64::from(REST_HEIGHT), 0.0],
    );
    let mut burial = radial_case(
        "radial/5r/burial1/still".to_owned(),
        wall,
        buried,
        None,
        Input::Still,
        1,
        Check::Push(super::pushout::Check::Rested),
        None,
    );
    burial.carry = Carry {
        vel_up: -3.0,
        grounded: false,
    };
    cases.push(burial);
    cases.push(radial_case(
        "radial/6/chunk-seam/v2".to_owned(),
        SceneKey::RadialSeam,
        resting_on_planet(14.0, 0.3),
        None,
        Input::walk(plus, 2.0),
        120,
        Check::Seam,
        None,
    ));
    cases
}

pub fn judge(scene: &Scene, check: Check, run: &Run, reference: Option<&Run>) -> Outcome {
    if matches!(check, Check::Slope(_) | Check::Descent) {
        let grounds = grounds(scene, run);
        let boundary = run
            .reports
            .iter()
            .zip(&grounds)
            .filter(|(report, ground)| ground.boundary(report.up))
            .count();
        if boundary as f64 > MAX_BOUNDARY_SHARE * run.reports.len() as f64 {
            return Outcome::Invalid {
                reason: format!("{boundary} ticks at the boundary angle"),
            };
        }
    }
    match check {
        Check::Slope(check) => super::slope::judge(scene, check, run),
        Check::Descent => {
            let last = run.reports.last().unwrap();
            let flat = sub(last.end, run.start);
            let horizontal = norm(sub(flat, scale(last.up, dot(flat, last.up))));
            if horizontal < 3.0 {
                return missing_witness(run, "3 m of descent");
            }
            match on_the_floor(scene, run, 0) {
                Outcome::Pass => no_hops(scene, run, 0),
                failed => failed,
            }
        }
        Check::Step(check) => super::step::judge(scene, check, run),
        Check::Push(check) => super::pushout::judge(scene, check, run, reference),
        Check::Seam => {
            let mut path = 0.0;
            let mut previous = run.start;
            for (t, report) in run.reports.iter().enumerate() {
                path += norm(sub(report.end, previous));
                previous = report.end;
                let height = norm(sub(report.end, CENTRE)) - R;
                if (height - f64::from(REST_HEIGHT)).abs() >= 0.05 {
                    return Outcome::fail(t, "feet off rest height", height);
                }
                if !report.grounded {
                    return Outcome::fail(t, "not grounded", height);
                }
            }
            let crossed = run.reports.last().unwrap().end[0] > 17.0;
            if !crossed {
                return missing_witness(run, "crossing the chunk seam");
            }
            if (path - 4.0).abs() >= 0.08 {
                return Outcome::fail(run.reports.len() - 1, "path length", path);
            }
            Outcome::Pass
        }
    }
}
