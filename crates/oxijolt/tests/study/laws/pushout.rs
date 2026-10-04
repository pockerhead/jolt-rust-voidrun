//! Law 5: a push-out from terrain or walls never becomes velocity (5v), and it recovers (5r),
//! including overlaps deeper than radius + padding.
//!
//! 5v judges moving cases only (rule 9 makes a still case's velocity zero by definition): each
//! is compared with the same input started at the analytic exterior pose, where the padded
//! capsule just touches the overlapped surface; the difference of the output velocities along
//! the surface's outward normal times dt must stay within `PUSH_LEAK` on every tick.

use super::{
    grounds, resting, Case, Check as LawCheck, Column, Input, Outcome, Reference, Run, Scenes,
};
use crate::common::math::{add, dot, scale, sub, V3};
use crate::common::walker::{Carry, CENTRE_UP, PADDING, RADIUS, REST_HEIGHT};
use crate::common::DT;
use crate::study::scenes::{Scene, SceneKey};

const TICKS: usize = 10;
const SPEED: f64 = 2.0;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Check {
    /// 5v: the outward normal of the overlapped surface.
    Velocity { normal: V3 },
    /// 5r: pushed away from the wall by more than `WALL_PUSH` on every tick.
    AwayFromWall { normal: V3 },
    /// 5r: the capsule's clearance to the block `block` is at least -0.01 m on every tick.
    WallClear { block: usize },
    /// 5r: at rest on the ground (gap within `REST_TOLERANCE`) and grounded on every tick.
    Rested,
    /// 5r: the gap is at least -0.01 m on every tick.
    GapAbove,
    /// 5r: moves less than 0.01 m.
    Stays,
}

#[allow(clippy::too_many_arguments)]
fn push_case(
    id: &str,
    column: Column,
    scene: SceneKey,
    start: V3,
    input: Input,
    ticks: usize,
    check: Check,
    reference: Option<Reference>,
) -> Case {
    Case {
        id: id.to_owned(),
        column,
        scene,
        start,
        rest_gap: None,
        carry: Carry::RESTING,
        input,
        ticks,
        check: LawCheck::Push(check),
        reference,
        report_only: false,
    }
}

/// The 5v cases.
pub fn velocity_cases(scenes: &mut Scenes) -> Vec<Case> {
    let along_z = Input::walk([0.0, 0.0, 1.0], SPEED);
    let along_x = Input::walk([1.0, 0.0, 0.0], SPEED);
    let away = [-1.0, 0.0, 0.0];
    let exterior =
        |start: V3, normal: V3, depth: f64| add(start, scale(normal, depth + f64::from(PADDING)));
    let mut cases = Vec::new();
    let rest = f64::from(REST_HEIGHT);
    let thin = SceneKey::Wall { thick: false };
    let thick = SceneKey::Wall { thick: true };
    let wall = |key: SceneKey, x: f64, depth: f64, id: &str| {
        let start = [x, rest, 0.0];
        push_case(
            id,
            Column::PushVelocity,
            key,
            start,
            along_z,
            TICKS,
            Check::Velocity { normal: away },
            Some(Reference {
                scene: key,
                start: exterior(start, away, depth),
                input: along_z,
            }),
        )
    };
    cases.push(wall(thin, 0.0, 0.05, "5v/wall0.05/moving"));
    cases.push(wall(thick, 0.45, 0.5, "5v/wall0.5/moving"));
    let up = [0.0, 1.0, 0.0];
    let ground = [0.0, f64::from(RADIUS) - 0.1, 0.0];
    cases.push(push_case(
        "5v/terrain0.1/moving",
        Column::PushVelocity,
        SceneKey::Flat,
        ground,
        along_x,
        TICKS,
        Check::Velocity { normal: up },
        Some(Reference {
            scene: SceneKey::Flat,
            start: exterior(ground, up, 0.1),
            input: along_x,
        }),
    ));
    for deg in [30.0, 60.0] {
        let key = SceneKey::plane(deg);
        let resting_start = resting(scenes.get(key), 0.0, 0.0);
        let normal = slope_normal(deg);
        let start = sub(resting_start, scale(normal, 0.1 + f64::from(PADDING)));
        cases.push(push_case(
            &format!("5v/plane{deg}-0.1/contour"),
            Column::PushVelocity,
            key,
            start,
            along_z,
            TICKS,
            Check::Velocity { normal },
            Some(Reference {
                scene: key,
                start: resting_start,
                input: along_z,
            }),
        ));
    }
    let clear = [-0.07, rest, 0.0];
    let at = 5;
    let z = SPEED * f64::from(DT) * at as f64;
    let teleport = |x: f64| Input::Teleport {
        dir: [0.0, 0.0, 1.0],
        speed: SPEED,
        at,
        to: [x, rest, z],
    };
    cases.push(push_case(
        "5v/teleport-wall0.05/moving",
        Column::PushVelocity,
        thin,
        clear,
        teleport(0.0),
        TICKS,
        Check::Velocity { normal: away },
        Some(Reference {
            scene: thin,
            start: clear,
            input: teleport(-0.07),
        }),
    ));
    cases.push(push_case(
        "5v/clear-control/moving",
        Column::PushVelocity,
        thin,
        clear,
        along_z,
        TICKS,
        Check::Velocity { normal: away },
        Some(Reference {
            scene: thin,
            start: clear,
            input: along_z,
        }),
    ));
    cases
}

/// The 5r cases.
pub fn recovery_cases(scenes: &mut Scenes) -> Vec<Case> {
    let mut cases = Vec::new();
    let rest = f64::from(REST_HEIGHT);
    let away = [-1.0, 0.0, 0.0];
    for (name, input) in [
        ("still", Input::Still),
        ("moving", Input::walk([0.0, 0.0, 1.0], SPEED)),
    ] {
        cases.push(push_case(
            &format!("5r/wall0.05/{name}"),
            Column::PushRecovery,
            SceneKey::Wall { thick: false },
            [0.0, rest, 0.0],
            input,
            TICKS,
            Check::AwayFromWall { normal: away },
            None,
        ));
        cases.push(push_case(
            &format!("5r/wall0.5/{name}"),
            Column::PushRecovery,
            SceneKey::Wall { thick: true },
            [0.45, rest, 0.0],
            input,
            TICKS,
            Check::WallClear { block: 0 },
            None,
        ));
        cases.push(push_case(
            &format!("5r/terrain0.1/{name}"),
            Column::PushRecovery,
            SceneKey::Flat,
            [0.0, f64::from(RADIUS) - 0.1, 0.0],
            input,
            TICKS,
            Check::Rested,
            None,
        ));
        for deg in [30.0, 60.0] {
            let key = SceneKey::plane(deg);
            let resting_start = resting(scenes.get(key), 0.0, 0.0);
            let start = sub(
                resting_start,
                scale(slope_normal(deg), 0.1 + f64::from(PADDING)),
            );
            let slope_input = match input {
                Input::Still => Input::Still,
                _ => Input::walk([0.0, 0.0, 1.0], SPEED),
            };
            cases.push(push_case(
                &format!("5r/plane{deg}-0.1/{name}"),
                Column::PushRecovery,
                key,
                start,
                slope_input,
                TICKS,
                Check::GapAbove,
                None,
            ));
        }
    }
    for depth in [1.0, 3.0, 10.0] {
        let mut case = push_case(
            &format!("5r/burial{depth}/still"),
            Column::PushRecovery,
            SceneKey::Flat,
            [0.5, -depth, 0.5],
            Input::Still,
            1,
            Check::Rested,
            None,
        );
        case.carry = Carry {
            vel_up: -3.0,
            grounded: false,
        };
        cases.push(case);
    }
    cases.push(push_case(
        "5r/resting/still",
        Column::PushRecovery,
        SceneKey::Flat,
        [0.5, rest, 0.5],
        Input::Still,
        1,
        Check::Stays,
        None,
    ));
    cases
}

/// The outward normal of `plane(deg)`.
pub fn slope_normal(deg: f64) -> V3 {
    let (sin, cos) = deg.to_radians().sin_cos();
    [-sin, cos, 0.0]
}

pub fn judge(scene: &Scene, check: Check, run: &Run, reference: Option<&Run>) -> Outcome {
    let n = run.reports.len();
    match check {
        Check::Velocity { normal } => {
            let normal = scene.frame.to_world_dir(normal);
            let reference = reference.expect("5v cases have a reference run");
            for t in 0..n {
                let leak = dot(
                    sub(run.reports[t].velocity, reference.reports[t].velocity),
                    normal,
                ) * f64::from(DT);
                if leak.abs() > super::PUSH_LEAK {
                    return Outcome::fail(t, "push in the output velocity", leak);
                }
            }
            Outcome::Pass
        }
        Check::AwayFromWall { normal } => {
            let normal = scene.frame.to_world_dir(normal);
            for (t, report) in run.reports.iter().enumerate() {
                let away = dot(sub(report.end, run.start), normal);
                if away <= super::WALL_PUSH {
                    return Outcome::fail(t, "pushed from the wall", away);
                }
            }
            Outcome::Pass
        }
        Check::WallClear { block } => {
            let block = scene.surface.blocks[block];
            for (t, report) in run.reports.iter().enumerate() {
                let top = add(report.end, scale(report.up, 2.0 * f64::from(CENTRE_UP)));
                let clearance = block.segment_distance(report.end, top) - f64::from(RADIUS);
                if clearance < -0.01 {
                    return Outcome::fail(t, "inside the wall", clearance);
                }
            }
            Outcome::Pass
        }
        Check::Rested => {
            let grounds = grounds(scene, run);
            for (t, report) in run.reports.iter().enumerate() {
                if grounds[t].gap.abs() > super::REST_TOLERANCE {
                    return Outcome::fail(t, "rest height", grounds[t].gap);
                }
                if !report.grounded {
                    return Outcome::fail(t, "not grounded after recovery", grounds[t].gap);
                }
            }
            Outcome::Pass
        }
        Check::GapAbove => {
            let grounds = grounds(scene, run);
            for (t, ground) in grounds.iter().enumerate() {
                if ground.gap < -0.01 {
                    return Outcome::fail(t, "still overlapping", ground.gap);
                }
            }
            Outcome::Pass
        }
        Check::Stays => {
            let moved = crate::common::math::norm(sub(run.reports[n - 1].end, run.start));
            if moved >= 0.01 {
                return Outcome::fail(n - 1, "moved at rest", moved);
            }
            Outcome::Pass
        }
    }
}

/// The deepest overlap of the capsule at a run's start, metres (for reports).
pub fn start_overlap(scene: &Scene, run: &Run) -> f64 {
    let up = scene.up.up_at(run.start);
    let top = add(run.start, scale(up, 2.0 * f64::from(CENTRE_UP)));
    let terrain = super::Ground::at(scene, run.start, run.padding).gap + f64::from(run.padding);
    let walls = scene
        .surface
        .blocks
        .iter()
        .map(|block| block.segment_distance(run.start, top) - f64::from(RADIUS))
        .fold(f64::INFINITY, f64::min);
    -terrain.min(walls).min(0.0)
}
