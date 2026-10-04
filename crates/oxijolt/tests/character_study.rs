//! The character study: which of the game's character laws CharacterVirtual's built-in
//! mechanisms carry, under which settings, checked against the pinned table of
//! `docs/character-study.md`; plus the instrument's own checks.
//!
//! The survey that produced the table is `survey` (ignored); run it with
//! `cargo test -p oxijolt --test character_study survey -- --ignored --nocapture`.
//! `STUDY_ROWS`, `STUDY_COLUMNS` (comma-separated names or labels) and `STUDY_PERTURBATIONS`
//! (1..=5) narrow it; `STUDY_TRACE=<case id>` prints that case's ticks for each row;
//! `STUDY_BITS=8` builds the study heightfields with 8 bits per sample; `STUDY_GRID=1` prints
//! the 1 mm step-height grid of each row.

mod common;
mod study;

use common::math::{add, dot, norm, scale, sub, V3};
use common::walker::{
    add_walker, near_tick, script_input, script_scene, script_start, tangent, up_at, Carry,
    NearOutput, RADIUS,
};
use common::DT;
use oxijolt::*;
use study::config::Config;
use study::controller::{tick, TickReport};
use study::frame::{Frame, UpPolicy};
use study::geometry::{classify, FaceClass};
use study::laws::{
    evaluate, judge, play, Case, Check, Column, Input, Outcome, Run, Scenes, PERTURBATIONS,
};
use study::matrix;
use study::scenes::{Profile, Scene, SceneKey, Split};

// ---------------------------------------------------------------------------------------------
// Instrument checks.

/// Every decoded heightfield is its profile within the field's quantisation bound.
#[test]
fn decoded_heightfields_match_their_profiles() {
    let keys = [
        SceneKey::plane(30.0),
        SceneKey::Plane {
            deg10: 300,
            bits: 8,
            tilted: false,
        },
        SceneKey::plane(60.0),
        SceneKey::Ramp {
            deg10: 445,
            tilted: false,
        },
        SceneKey::Ridge {
            deg10: 500,
            tilted: false,
        },
        SceneKey::Seam {
            profile: Profile::Rise30,
            split: Split::Pair,
        },
        SceneKey::Seam {
            profile: Profile::Descent44_5,
            split: Split::Continuous,
        },
        SceneKey::RadialSpiral {
            deg10: 445,
            spacing_mm: 500,
            bits: 16,
        },
        SceneKey::RadialSpiral {
            deg10: 445,
            spacing_mm: 500,
            bits: 8,
        },
        SceneKey::RadialCrest { deg10: 445 },
    ];
    for key in keys {
        let scene = Scene::build(key);
        for field in &scene.surface.fields {
            let error = field.worst_decoding_error();
            let bound = field.quantisation_bound();
            assert!(error <= bound, "{key:?}: error {error} over bound {bound}");
        }
    }
}

/// The decoded faces have the scene's angle: planes within 0.01 degrees at 16 bits; the 0.5 m
/// spirals within the ranges of the geometry probe against the up 0.3 m around each face.
#[test]
fn decoded_face_angles_match_the_scene() {
    for deg in [30.0, 44.5, 45.5, 60.0] {
        let scene = Scene::build(SceneKey::plane(deg));
        let up = [0.0, 1.0, 0.0];
        for_each_face(&scene, |_, normal| {
            let angle = dot(normal, up).acos().to_degrees();
            assert!((angle - deg).abs() < 0.01, "plane {deg}: face at {angle}");
        });
    }
    for (deg10, low, high) in [(445, 44.19, 44.81), (455, 45.19, 45.81)] {
        let scene = Scene::build(SceneKey::RadialSpiral {
            deg10,
            spacing_mm: 500,
            bits: 16,
        });
        for_each_face(&scene, |centroid, normal| {
            if centroid[0].abs() > 7.0 || centroid[2].abs() > 0.6 {
                return;
            }
            for dx in [-0.3, 0.0, 0.3] {
                let up = up_at(add(centroid, [dx, 0.42, 0.0]));
                let angle = dot(normal, up).acos().to_degrees();
                assert!(
                    (low..=high).contains(&angle),
                    "spiral {deg10}: face at {centroid:?} is {angle} against the up {dx} away"
                );
            }
        });
    }
}

/// Calls `f(centroid, normal)` for every decoded triangle of the scene's fields, in world space.
fn for_each_face(scene: &Scene, mut f: impl FnMut(V3, V3)) {
    for field in &scene.surface.fields {
        let n = field.samples;
        for y in 0..n - 1 {
            for x in 0..n - 1 {
                let at = |x: usize, y: usize| field.positions[y * n + x].unwrap();
                for [a, b, c] in [
                    [at(x, y), at(x, y + 1), at(x + 1, y + 1)],
                    [at(x, y), at(x + 1, y + 1), at(x + 1, y)],
                ] {
                    let normal = common::math::normalize(common::math::cross(sub(b, a), sub(c, a)));
                    let centroid = scale(add(add(a, b), c), 1.0 / 3.0);
                    f(
                        field.frame.to_world_point(centroid),
                        field.frame.to_world_dir(normal),
                    );
                }
            }
        }
    }
}

/// A character created resting on flat terrain and on a 30 degree plane has a gap of zero.
#[test]
fn a_resting_character_has_zero_gap() {
    for key in [SceneKey::Flat, SceneKey::plane(30.0)] {
        let mut scene = Scene::build(key);
        let start = study::laws::resting(&scene, 0.0, 0.0);
        let config = Config::bare();
        let walker = config.create_character(&mut scene, start);
        let origin = study::controller::origin_of(&scene.world, &walker, 0.02);
        let gap = study::laws::Ground::at(&scene, origin, 0.02).gap;
        assert!(gap.abs() <= 1e-3, "{key:?}: gap {gap}");
        let ground = scene.world.character(walker.id).unwrap().ground_state();
        assert_eq!(ground, GroundState::OnGround, "{key:?}");
    }
}

/// The two seam fields decode to the continuous field at every shared sample; the raised pair
/// is 3 cm off.
#[test]
fn seam_pair_matches_the_continuous_field() {
    let mut scenes = Scenes::default();
    for profile in [Profile::Flat, Profile::Rise30, Profile::Descent44_5] {
        let continuous = SceneKey::Seam {
            profile,
            split: Split::Continuous,
        };
        let pair = SceneKey::Seam {
            profile,
            split: Split::Pair,
        };
        let raised = SceneKey::Seam {
            profile,
            split: Split::Raised,
        };
        let mismatch = study::laws::seam::surface_mismatch(&mut scenes, pair, continuous);
        assert!(
            mismatch <= study::laws::SEAM_SURFACE,
            "{profile:?}: {mismatch}"
        );
        let control = study::laws::seam::surface_mismatch(&mut scenes, raised, continuous);
        assert!(
            (control - 0.03).abs() < 1e-3,
            "{profile:?}: raised by {control}"
        );
    }
}

/// Frames map points and directions there and back, and give unit ups.
#[test]
fn frames_round_trip() {
    let frame = Frame::tilted();
    for p in [[1.0, 2.0, 3.0], [-7.5, 0.25, 16.0], [0.0; 3]] {
        let back = frame.to_local_point(frame.to_world_point(p));
        assert!(norm(sub(back, p)) <= 1e-12, "{p:?} -> {back:?}");
        let dir = frame.to_local_dir(frame.to_world_dir(p));
        assert!(norm(sub(dir, p)) <= 1e-12, "{p:?} -> {dir:?}");
    }
    let up = UpPolicy::Frame(frame).up_at([0.0; 3]);
    assert!((norm(up) - 1.0).abs() <= 1e-12);
    assert!(
        norm(sub(up, [0.0, 1.0, 0.0])) > 0.1,
        "the tilted up is not +Y"
    );
    let radial = UpPolicy::Radial.up_at([3.0, 0.0, 0.0]);
    assert!((norm(radial) - 1.0).abs() <= 1e-12);
    assert!(radial[0] > 0.0);
}

/// The desired displacement of the game's player (spec D.3) after `last`, as
/// `walker::Player::tick` computes it.
fn player_desired(last: &NearOutput, last_up: V3, input: V3, speed: f64) -> V3 {
    let on_ground = last.grounded && !last.sliding;
    if on_ground {
        if input == [0.0; 3] {
            [0.0; 3]
        } else {
            tangent(last.pos, input, speed * f64::from(DT))
        }
    } else {
        let control = if input == [0.0; 3] {
            [0.0; 3]
        } else {
            tangent(last.pos, input, speed)
        };
        let v = last.velocity;
        let horizontal = sub(v, scale(last_up, dot(v, last_up)));
        scale(
            add(scale(horizontal, 0.98), scale(control, 0.05)),
            f64::from(DT),
        )
    }
}

/// The study caller with the `walker` row reproduces the reference near step bit for bit on
/// the scripted run (flat ground, a structure step, 30 and 60 degree rises, a jump).
#[test]
fn the_walker_row_reproduces_the_reference_near_step_bit_for_bit() {
    let (mut reference_world, layers) = script_scene(1);
    let reference = add_walker(&mut reference_world, &layers, script_start());
    let (world, layers) = script_scene(1);
    let mut scene = Scene::from_world(world, layers);
    let config = Config::walker();
    let walker = config.create_character(&mut scene, script_start());
    let (mut reference_carry, mut carry) = (Carry::RESTING, Carry::RESTING);
    let mut last: Option<(NearOutput, V3)> = None;
    for t in 0..=400 {
        let last_up = up_at(common::walker::origin(&reference_world, &reference));
        let desired = match last {
            None => [0.0; 3],
            Some((out, up)) => {
                let (input, speed, jump) = script_input(t - 1);
                if jump && out.grounded && !out.sliding {
                    reference_carry.vel_up = 4.5;
                    carry.vel_up = 4.5;
                }
                player_desired(&out, up, input, speed)
            }
        };
        let expected = near_tick(
            &mut reference_world,
            &reference,
            &mut reference_carry,
            desired,
        );
        let report = tick(&mut scene, &walker, &config, &mut carry, desired, None);
        let got = NearOutput {
            pos: report.end,
            grounded: report.grounded,
            vel_up: report.vel_up,
            velocity: report.velocity,
            sliding: report.sliding,
            ceiling: report.ceiling,
            blocker: report.blocker,
            recovered: report.recovered,
        };
        let bits = |o: &NearOutput| {
            (
                o.pos.map(f64::to_bits),
                o.velocity.map(f64::to_bits),
                o.vel_up.to_bits(),
                (o.grounded, o.sliding, o.ceiling, o.recovered),
                o.blocker.map(|b| {
                    (
                        b.body.map(BodyId::to_raw),
                        b.group,
                        common::math::bits(b.normal),
                    )
                }),
            )
        };
        assert_eq!(bits(&got), bits(&expected), "tick {t}");
        scene.place_actor(report.end, common::walker::from_y_to(report.up));
        last = Some((expected, last_up));
    }
}

/// Maintenance, move and post-move displacements add up to each tick's displacement, and the
/// depenetration push is never longer than radius + padding (spec D.2 rule 2), also against the
/// 0.5 m wall overlap.
#[test]
fn tick_reports_account_for_every_displacement() {
    let mut scenes = Scenes::default();
    let mut cases = matrix::cases(&mut scenes, Column::PushRecovery);
    cases.extend(
        matrix::cases(&mut scenes, Column::StepSharp)
            .into_iter()
            .take(4),
    );
    let limit = f64::from(RADIUS + 0.02) + 1e-9;
    let mut longest: f64 = 0.0;
    for config in [Config::spec_d2(), Config::d2_still(), Config::walker()] {
        for case in &cases {
            let scene = scenes.get(case.scene);
            let run = play(scene, &config, case, case.start, true);
            for (t, (r, traced)) in run.reports.iter().zip(&run.traces).enumerate() {
                let sum = add(add(r.maintenance, r.moving), r.post);
                let error = norm(sub(sum, sub(r.end, r.start)));
                assert!(
                    error <= 1e-12,
                    "{} {}: tick {t} off by {error}",
                    config.name,
                    case.id
                );
                let push = norm(traced.q6_push);
                assert!(
                    push <= limit,
                    "{} {}: tick {t} pushed {push}",
                    config.name,
                    case.id
                );
                longest = longest.max(push);
            }
        }
    }
    assert!(
        longest > 0.4,
        "the deep overlap reaches the push limit: {longest}"
    );
}

// ---------------------------------------------------------------------------------------------
// Corrupted traces: each predicate passes a good run and fails each mutation.

/// A synthetic tick from `start` to `end` with up +Y.
fn synthetic(start: V3, end: V3, grounded: bool, walking: V3) -> TickReport {
    TickReport {
        start,
        up: [0.0, 1.0, 0.0],
        maintenance: [0.0; 3],
        moving: sub(end, start),
        post: [0.0; 3],
        end,
        velocity: if walking == [0.0; 3] {
            [0.0; 3]
        } else {
            scale(sub(end, start), 1.0 / f64::from(DT))
        },
        supplied: common::math::vec3(walking),
        jolt_velocity: Vec3::ZERO,
        vel_up: 0.0,
        grounded,
        sliding: !grounded,
        ceiling: false,
        blocker: None,
        ground_before: GroundState::OnGround,
        ground_after_move: GroundState::OnGround,
        ground: if grounded {
            GroundState::OnGround
        } else {
            GroundState::OnSteepGround
        },
        ground_body: None,
        on_terrain: true,
        recovered: false,
        pushed: false,
        deep: false,
        still_ran: false,
        stepped: false,
        snapped: false,
        max_hits_exceeded: false,
        touched_groups: 1 << common::Groups::STRUCTURE,
    }
}

/// A run through `points` (resting origins), each tick moving to the next point.
fn synthetic_run(points: &[V3], grounded: bool, walking: V3) -> Run {
    Run {
        start: points[0],
        padding: 0.02,
        startup: GroundState::OnGround,
        reports: points
            .windows(2)
            .map(|pair| synthetic(pair[0], pair[1], grounded, walking))
            .collect(),
        traces: Vec::new(),
    }
}

/// `count` resting origins on `scene` along local x from `x0` by `step`.
fn resting_path(scene: &Scene, x0: f64, step: f64, count: usize) -> Vec<V3> {
    (0..count)
        .map(|i| study::laws::resting(scene, x0 + step * i as f64, 0.0))
        .collect()
}

fn expect_fail(outcome: Outcome, metric: &str, what: &str) {
    match outcome {
        Outcome::Fail { metric: m, .. } if m == metric => {}
        other => panic!("{what}: expected a failure on {metric}, got {other:?}"),
    }
}

fn judged(scenes: &mut Scenes, case: &Case, run: &Run, reference: Option<&Run>) -> Outcome {
    judge(scenes, case, run, reference)
}

fn case_of(
    id: &str,
    column: Column,
    scene: SceneKey,
    check: Check,
    reference: Option<study::laws::Reference>,
) -> Case {
    Case {
        id: id.to_owned(),
        column,
        scene,
        start: [0.0; 3],
        rest_gap: None,
        carry: Carry::RESTING,
        input: Input::Still,
        ticks: 0,
        check,
        reference,
        report_only: false,
    }
}

#[test]
fn law_predicates_reject_corrupted_traces() {
    use study::laws::{floor, hops, pushout, seam, slope, step};
    let mut scenes = Scenes::default();

    // Law 1: sliding down 60 degrees, never grounded, faster each tick.
    let key = SceneKey::plane(60.0);
    let scene = scenes.get(key);
    let mut x = 4.0;
    let mut points = Vec::new();
    for i in 0..61 {
        points.push(study::laws::resting(scene, x, 0.0));
        x -= 0.001 * (1 + i) as f64;
    }
    let case = case_of(
        "1/synthetic",
        Column::Slope,
        key,
        Check::Slope(slope::Check::Steep { uphill: false }),
        None,
    );
    let good = synthetic_run(&points, false, [0.0; 3]);
    assert_eq!(judged(&mut scenes, &case, &good, None), Outcome::Pass);
    let mut bad = good.clone();
    bad.reports[40].grounded = true;
    expect_fail(
        judged(&mut scenes, &case, &bad, None),
        "grounded on a steep face",
        "law 1",
    );

    // Law 2: walking down 40 degrees on the floor.
    let key = SceneKey::plane(40.0);
    let points = resting_path(scenes.get(key), 8.0, -0.05, 81);
    let case = case_of(
        "2/synthetic",
        Column::Floor,
        key,
        Check::Floor(floor::Check {
            dir: [-1.0, 0.0, 0.0],
            witness: 3.0,
            report: false,
        }),
        None,
    );
    let good = synthetic_run(&points, true, [-3.0, 0.0, 0.0]);
    assert_eq!(judged(&mut scenes, &case, &good, None), Outcome::Pass);
    let mut bad = good.clone();
    bad.reports[20].grounded = false;
    expect_fail(
        judged(&mut scenes, &case, &bad, None),
        "not grounded on walkable ground",
        "law 2 support",
    );
    let mut bad = good.clone();
    let lifted = add(
        bad.reports[20].end,
        scale(pushout::slope_normal(40.0), 0.05),
    );
    bad.reports[20].end = lifted;
    expect_fail(judged(&mut scenes, &case, &bad, None), "gap", "law 2 gap");

    // Law 3: onto a 0.45 m step, and not onto a 0.50 m one.
    let key = SceneKey::Step {
        height_mm: 450,
        rounded: false,
    };
    let scene = scenes.get(key);
    let mut points = resting_path(scene, -1.4, 0.05, 20);
    let top = |x: f64| [x, 0.45 + f64::from(RADIUS) + 0.02, 0.0];
    points.extend((0..15).map(|i| top(0.1 + 0.05 * f64::from(i))));
    let climb = step::Check {
        height: 0.45,
        face: 0.0,
        ground: 0.0,
        climb: true,
    };
    let case = case_of(
        "3s/synthetic",
        Column::StepSharp,
        key,
        Check::Step(climb),
        None,
    );
    let good = synthetic_run(&points, true, [2.0, 0.0, 0.0]);
    assert_eq!(judged(&mut scenes, &case, &good, None), Outcome::Pass);
    let before = synthetic_run(
        &resting_path(scenes.get(key), -1.4, 0.02, 40),
        true,
        [2.0, 0.0, 0.0],
    );
    expect_fail(
        judged(&mut scenes, &case, &before, None),
        "did not land on the step",
        "law 3 landing removed",
    );
    let high = SceneKey::Step {
        height_mm: 500,
        rounded: false,
    };
    let refuse = step::Check {
        height: 0.5,
        face: 0.0,
        ground: 0.0,
        climb: false,
    };
    let case = case_of(
        "3s/synthetic-high",
        Column::StepSharp,
        high,
        Check::Step(refuse),
        None,
    );
    let on_high: Vec<V3> = points
        .iter()
        .map(|p| {
            if p[0] > 0.0 {
                [p[0], 0.5 + 0.42, 0.0]
            } else {
                *p
            }
        })
        .collect();
    expect_fail(
        judged(
            &mut scenes,
            &case,
            &synthetic_run(&on_high, true, [2.0, 0.0, 0.0]),
            None,
        ),
        "landed on a step too high",
        "law 3 on a 0.50 top",
    );
    let close = synthetic_run(
        &resting_path(scenes.get(high), -1.4, 0.03, 40),
        true,
        [2.0, 0.0, 0.0],
    );
    expect_fail(
        judged(&mut scenes, &case, &close, None),
        "ended within 0.3 m of the face",
        "law 3 close to a 0.50 face",
    );

    // Law 4: a 2 mm rise on one descending tick.
    let key = SceneKey::plane(40.0);
    let points = resting_path(scenes.get(key), 8.0, -0.05, 81);
    let case = case_of(
        "4/synthetic",
        Column::Hops,
        key,
        Check::Hops(hops::Check {
            from: 0,
            after_restart: None,
        }),
        None,
    );
    let good = synthetic_run(&points, true, [-3.0, 0.0, 0.0]);
    assert_eq!(judged(&mut scenes, &case, &good, None), Outcome::Pass);
    let mut bad = good.clone();
    bad.reports[30].end[1] = bad.reports[30].start[1] + 0.002;
    bad.reports[30].end = add(bad.reports[30].end, [0.0; 3]);
    expect_fail(judged(&mut scenes, &case, &bad, None), "rise", "law 4");

    // Law 5v: a 0.05 m push added to the output velocity.
    let key = SceneKey::Wall { thick: false };
    let points: Vec<V3> = (0..11)
        .map(|i| [-0.07, 0.42, 0.033 * f64::from(i)])
        .collect();
    let reference_run = synthetic_run(&points, true, [0.0, 0.0, 2.0]);
    let case = case_of(
        "5v/synthetic",
        Column::PushVelocity,
        key,
        Check::Push(pushout::Check::Velocity {
            normal: [-1.0, 0.0, 0.0],
        }),
        None,
    );
    assert_eq!(
        judged(&mut scenes, &case, &reference_run, Some(&reference_run)),
        Outcome::Pass
    );
    let mut bad = reference_run.clone();
    bad.reports[0].velocity = add(bad.reports[0].velocity, [-0.05 / f64::from(DT), 0.0, 0.0]);
    expect_fail(
        judged(&mut scenes, &case, &bad, Some(&reference_run)),
        "push in the output velocity",
        "law 5v",
    );

    // Law 5r: left 0.1 m inside the terrain.
    let key = SceneKey::Flat;
    let points: Vec<V3> = (0..4).map(|_| [0.5, 0.42, 0.5]).collect();
    let case = case_of(
        "5r/synthetic",
        Column::PushRecovery,
        key,
        Check::Push(pushout::Check::Rested),
        None,
    );
    let good = synthetic_run(&points, true, [0.0; 3]);
    assert_eq!(judged(&mut scenes, &case, &good, None), Outcome::Pass);
    let mut bad = good.clone();
    bad.reports[1].end = [0.5, 0.32, 0.5];
    expect_fail(
        judged(&mut scenes, &case, &bad, None),
        "rest height",
        "law 5r",
    );

    // Law 6: 3 mm up against the continuous run; a stalled run.
    let pair = SceneKey::Seam {
        profile: Profile::Flat,
        split: Split::Pair,
    };
    let whole = SceneKey::Seam {
        profile: Profile::Flat,
        split: Split::Continuous,
    };
    let points: Vec<V3> = (0..121)
        .map(|i| [-3.0 + 0.05 * f64::from(i), 0.42, 0.0])
        .collect();
    let reference = study::laws::Reference {
        scene: whole,
        start: points[0],
        input: Input::Still,
    };
    let case = case_of(
        "6/synthetic",
        Column::Seam,
        pair,
        Check::Seam(seam::Check {
            dir: [1.0, 0.0, 0.0],
            along: false,
        }),
        Some(reference),
    );
    let good = synthetic_run(&points, true, [3.0, 0.0, 0.0]);
    assert_eq!(
        judged(&mut scenes, &case, &good, Some(&good)),
        Outcome::Pass
    );
    let mut bad = good.clone();
    bad.reports[60].end[1] += 0.003;
    expect_fail(
        judged(&mut scenes, &case, &bad, Some(&good)),
        "height differs",
        "law 6 offset",
    );
    let stalled = synthetic_run(&vec![points[0]; 121], true, [3.0, 0.0, 0.0]);
    expect_fail(
        judged(&mut scenes, &case, &stalled, Some(&stalled)),
        "blocked before the witness",
        "law 6 stalled",
    );
}

// ---------------------------------------------------------------------------------------------
// The survey.

fn env_list(name: &str) -> Option<Vec<String>> {
    std::env::var(name)
        .ok()
        .map(|value| value.split(',').map(|s| s.trim().to_owned()).collect())
}

/// Plays every row on every case under every perturbation and prints the cells.
#[test]
#[ignore = "the survey takes minutes; run it on purpose"]
fn survey() {
    let rows: Vec<Config> = Config::pinned()
        .into_iter()
        .chain(Config::survey_rows())
        .filter(|row| {
            env_list("STUDY_ROWS").is_none_or(|names| names.iter().any(|n| n == row.name))
        })
        .collect();
    let columns: Vec<Column> = Column::ALL
        .into_iter()
        .filter(|c| {
            env_list("STUDY_COLUMNS").is_none_or(|names| names.iter().any(|n| n == c.label()))
        })
        .collect();
    let perturbations: usize = std::env::var("STUDY_PERTURBATIONS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(PERTURBATIONS.len());
    let trace = std::env::var("STUDY_TRACE").ok();
    if let Some(bits) = std::env::var("STUDY_BITS")
        .ok()
        .and_then(|v| v.parse().ok())
    {
        study::scenes::FIELD_BITS_OVERRIDE.store(bits, std::sync::atomic::Ordering::Relaxed);
    }
    let mut scenes = Scenes::default();
    let all: Vec<(Column, Vec<Case>)> = columns
        .iter()
        .map(|&column| (column, matrix::cases(&mut scenes, column)))
        .collect();
    let reports = matrix::report_cases(&mut scenes);
    for row in &rows {
        println!("## {}: {}", row.name, row.describe());
        for (column, cases) in &all {
            let results = matrix::outcomes(&mut scenes, row, cases, perturbations);
            println!(
                "{} {:?}: {}",
                column.label(),
                matrix::cell(&results),
                matrix::summary(&results)
            );
            for r in &results {
                if !r.outcomes.iter().all(Outcome::is_pass) {
                    println!("    {} {:?}", r.id, r.outcomes);
                }
            }
        }
        for case in &reports {
            let outcome = evaluate(&mut scenes, row, case, PERTURBATIONS[0]);
            println!("report {}: {:?}", case.id, outcome);
        }
        if std::env::var("STUDY_GRID").is_ok() {
            print_step_grid(row);
        }
        if let Some(id) = &trace {
            let case = all
                .iter()
                .flat_map(|(_, cases)| cases)
                .chain(&reports)
                .find(|case| &case.id == id)
                .unwrap_or_else(|| panic!("no case {id}"));
            print_trace(&mut scenes, row, case);
        }
    }
}

/// Prints the ticks of `case` under `row`.
fn print_trace(scenes: &mut Scenes, row: &Config, case: &Case) {
    let scene = scenes.get(case.scene);
    let run = play(scene, row, case, case.start, true);
    println!(
        "trace {} under {} (startup {:?})",
        case.id, row.name, run.startup
    );
    for (t, (r, traced)) in run.reports.iter().zip(&run.traces).enumerate() {
        let ground = study::laws::Ground::at(scene, r.end, run.padding);
        let classes: Vec<FaceClass> = ground
            .support
            .iter()
            .map(|e| classify(e.normal, r.up))
            .collect();
        println!(
            "t{t}: rise {:+.5} gap {:+.5} grounded {} sliding {} ground {:?}/{:?}/{:?} still {} pushed {} stepped {} snapped {} v {:?} support {:?} q4 {:?} q5 {:?} q6 {:?} depth {:.4}",
            dot(sub(r.end, r.start), r.up),
            ground.gap,
            r.grounded,
            r.sliding,
            r.ground_before,
            r.ground_after_move,
            r.ground,
            r.still_ran,
            r.pushed,
            r.stepped,
            r.snapped,
            r.velocity.map(|c| (c * 1000.0).round() / 1000.0),
            classes,
            traced.q4_normal,
            traced.q5_distance,
            traced.q6_push,
            traced.q6_depth,
        );
        for c in &traced.contacts {
            println!(
                "      contact body {:?} sub {} distance {:+.5} collided {} normal {:?} surface {:?}",
                c.body,
                c.sub_shape,
                c.distance,
                c.had_collision,
                c.contact_normal.map(|v| (v * 1e4).round() / 1e4),
                c.surface_normal.map(|v| (v * 1e4).round() / 1e4)
            );
        }
    }
}

/// Prints, for sharp and rounded steps from 0.40 to 0.55 m in 1 mm steps (heading 0, offset 0,
/// 1.0 m away, 1.6 m/s), which heights `row` lands on, as intervals.
fn print_step_grid(row: &Config) {
    for rounded in [false, true] {
        let mut scenes = Scenes::default();
        let mut landed = Vec::new();
        for mm in 400..=550_u16 {
            let key = SceneKey::Step {
                height_mm: mm,
                rounded,
            };
            let height = f64::from(mm) / 1000.0;
            let case = study::laws::step::step_case(&mut scenes, key, height, 0.0, 1.6, 0.0, 1.0);
            let scene = scenes.get(key);
            let run = play(scene, row, &case, case.start, false);
            let Check::Step(mut check) = case.check else {
                unreachable!()
            };
            check.climb = true;
            landed.push((mm, study::laws::step::landing(scene, check, &run).is_some()));
            drop(scenes.0.remove(&key));
        }
        let mut intervals = Vec::new();
        let mut open: Option<u16> = None;
        for (i, &(mm, ok)) in landed.iter().enumerate() {
            match (ok, open) {
                (true, None) => open = Some(mm),
                (false, Some(from)) => {
                    intervals.push(format!("{from}..={} mm", landed[i - 1].0));
                    open = None;
                }
                _ => {}
            }
        }
        if let Some(from) = open {
            intervals.push(format!("{from}..=550 mm"));
        }
        println!(
            "grid {} {}: lands on {}",
            row.name,
            if rounded { "rounded" } else { "sharp" },
            if intervals.is_empty() {
                "nothing".to_owned()
            } else {
                intervals.join(", ")
            }
        );
    }
}

// ---------------------------------------------------------------------------------------------
// The law gates: every pinned row against the pinned table.

/// Plays `rows` on the cases of `column` and checks each against its pinned cell. The
/// recommended row plays all five start perturbations, the others the unperturbed start.
fn law_gate(column: Column, rows: &[Config]) {
    let mut scenes = Scenes::default();
    let cases = matrix::cases(&mut scenes, column);
    let mut failures = Vec::new();
    for row in rows {
        let cell = study::expected::pinned(row.name)[column.index()];
        let perturbations = if row.name == "recommended" {
            PERTURBATIONS.len()
        } else {
            1
        };
        let results = matrix::outcomes(&mut scenes, row, &cases, perturbations);
        if let Err(error) = matrix::agrees(&results, cell) {
            failures.push(format!("{} column {}: {error}", row.name, column.label()));
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

#[test]
fn law_1_slope_limit() {
    law_gate(Column::Slope, &Config::pinned());
}

#[test]
fn law_2_stays_on_the_floor() {
    law_gate(Column::Floor, &Config::pinned());
}

#[test]
fn law_3_steps_sharp() {
    law_gate(Column::StepSharp, &Config::pinned());
}

#[test]
fn law_3_steps_rounded() {
    law_gate(Column::StepRounded, &Config::pinned());
}

#[test]
fn law_4_no_hops_downhill() {
    law_gate(Column::Hops, &Config::pinned());
}

#[test]
fn law_5v_push_out_is_not_velocity() {
    law_gate(Column::PushVelocity, &Config::pinned());
}

#[test]
fn law_5r_push_out_recovers() {
    law_gate(Column::PushRecovery, &Config::pinned());
}

#[test]
fn law_6_seams_are_one_surface() {
    law_gate(Column::Seam, &Config::pinned());
}

/// The radial family on the planet with radial up, for the rows the game would adopt from.
#[test]
fn radial_acceptance() {
    let rows: Vec<Config> = [
        "walker",
        "spec-d2",
        "d2-stick",
        "d2-stick-norefresh",
        "d2-noq4",
        "d2-still",
        "d2-stairs",
        "recommended",
    ]
    .into_iter()
    .map(Config::named)
    .collect();
    law_gate(Column::Radial, &rows);
}

/// The study's controls: a 50 degree slope limit reports OnGround on a 45.5 degree face, and a
/// seam raised by 3 cm fails law 6.
#[test]
fn the_controls_are_seen() {
    let mut scenes = Scenes::default();
    let key = SceneKey::plane(45.5);
    let start = study::laws::resting(scenes.get(key), 0.0, 0.0);
    let scene = scenes.get(key);
    let walker = Config::max_slope_50().create_character(scene, start);
    let ground = scene.world.character(walker.id).unwrap().ground_state();
    assert_eq!(ground, GroundState::OnGround);
    scene.world.remove_character(walker.id).unwrap();

    let control = study::laws::seam::cases(&mut scenes, Split::Raised);
    let failed = control
        .iter()
        .filter(|case| !evaluate(&mut scenes, &Config::spec_d2(), case, PERTURBATIONS[0]).is_pass())
        .count();
    assert_eq!(failed, control.len(), "every raised seam case fails law 6");
}

// ---------------------------------------------------------------------------------------------
// Frames and determinism.

/// `case` in the tilted frame: the same scene turned and moved, the same start in its frame.
fn tilted(case: &Case) -> Option<Case> {
    let turn = |key: SceneKey| match key {
        SceneKey::Plane { deg10, bits, .. } => Some(SceneKey::Plane {
            deg10,
            bits,
            tilted: true,
        }),
        SceneKey::Ramp { deg10, .. } => Some(SceneKey::Ramp {
            deg10,
            tilted: true,
        }),
        SceneKey::Ridge { deg10, .. } => Some(SceneKey::Ridge {
            deg10,
            tilted: true,
        }),
        SceneKey::BoxRamp { deg10, .. } => Some(SceneKey::BoxRamp {
            deg10,
            tilted: true,
        }),
        _ => None,
    };
    let frame = Frame::tilted();
    Some(Case {
        scene: turn(case.scene)?,
        start: frame.to_world_point(case.start),
        ..case.clone()
    })
}

/// spec-d2 gives the same cells on laws 1, 2 and 4 in a tilted frame (over the cases whose
/// scene can be tilted). Jolt is equivariant under a change of frame only up to `f32` rounding,
/// so paths are compared, once mapped back, within 1 mm on walkable ground and 1 cm sliding down
/// steep faces, and at most one case in twenty may leave that band (measured: 2 of 241, both
/// 1.6 m/s runs over a 44.5 degree crest that part by 0.2-0.3 m; one 7 m/s crest descent fails
/// law 2 here and passes tilted). Divergent cases and flipped verdicts are printed.
#[test]
fn the_spec_d2_row_gives_the_same_cells_in_a_tilted_frame() {
    let mut scenes = Scenes::default();
    let config = Config::spec_d2();
    let frame = Frame::tilted();
    let (mut compared, mut diverged) = (0, 0);
    for column in [Column::Slope, Column::Floor, Column::Hops] {
        let (mut flat_results, mut turned_results) = (Vec::new(), Vec::new());
        for case in matrix::cases(&mut scenes, column) {
            let Some(turned) = tilted(&case) else {
                continue;
            };
            if case.report_only {
                continue;
            }
            let flat = play(scenes.get(case.scene), &config, &case, case.start, false);
            let turned_run = play(
                scenes.get(turned.scene),
                &config,
                &turned,
                turned.start,
                false,
            );
            let verdict = judge(&mut scenes, &case, &flat, None);
            let turned_verdict = judge(&mut scenes, &turned, &turned_run, None);
            if verdict.is_pass() == turned_verdict.is_pass() {
                let steep = matches!(
                    case.check,
                    Check::Slope(study::laws::slope::Check::Steep { .. })
                        | Check::Slope(study::laws::slope::Check::SteepRidge)
                );
                let tolerance = if steep { 1e-2 } else { 1e-3 };
                let worst = flat
                    .reports
                    .iter()
                    .zip(&turned_run.reports)
                    .map(|(a, b)| norm(sub(frame.to_local_point(b.end), a.end)))
                    .fold(0.0, f64::max);
                if worst > tolerance {
                    println!("{}: paths {worst} m apart", case.id);
                    diverged += 1;
                }
            } else {
                println!("{}: {verdict:?} here, {turned_verdict:?} tilted", case.id);
            }
            flat_results.push(matrix::CaseOutcomes {
                id: case.id.clone(),
                outcomes: vec![verdict],
            });
            turned_results.push(matrix::CaseOutcomes {
                id: case.id.clone(),
                outcomes: vec![turned_verdict],
            });
            compared += 1;
        }
        assert_eq!(
            matrix::cell(&flat_results),
            matrix::cell(&turned_results),
            "column {}",
            column.label()
        );
    }
    assert!(compared > 100, "{compared} cases compared");
    println!("{diverged} of {compared} paths diverged beyond the tolerance");
    assert!(
        diverged * 20 <= compared,
        "{diverged} of {compared} paths diverged"
    );
}

/// Appends the bits of a tick report to `out`, for exact comparisons.
fn report_bits(r: &TickReport, out: &mut Vec<u8>) {
    for v in [
        r.start,
        r.up,
        r.maintenance,
        r.moving,
        r.post,
        r.end,
        r.velocity,
    ] {
        for c in v {
            out.extend_from_slice(&c.to_bits().to_le_bytes());
        }
    }
    for v in [r.supplied, r.jolt_velocity] {
        for c in common::math::bits(v) {
            out.extend_from_slice(&c.to_le_bytes());
        }
    }
    out.extend_from_slice(&r.vel_up.to_bits().to_le_bytes());
    out.extend_from_slice(&[
        u8::from(r.grounded),
        u8::from(r.sliding),
        u8::from(r.ceiling),
        r.ground_before as u8,
        r.ground_after_move as u8,
        r.ground as u8,
        u8::from(r.recovered),
        u8::from(r.pushed),
        u8::from(r.deep),
        u8::from(r.still_ran),
        u8::from(r.stepped),
        u8::from(r.snapped),
        u8::from(r.max_hits_exceeded),
    ]);
    out.extend_from_slice(&r.touched_groups.to_le_bytes());
    out.extend_from_slice(&r.ground_body.map_or(u32::MAX, BodyId::to_raw).to_le_bytes());
}

/// The determinism cases: down the radial crest at 7 m/s, and walking along a wall the capsule
/// overlaps by 5 cm.
fn determinism_cases(scenes: &mut Scenes) -> Vec<Case> {
    let mut cases = matrix::cases(scenes, Column::Radial);
    cases.extend(matrix::cases(scenes, Column::PushVelocity));
    cases
        .into_iter()
        .filter(|case| {
            case.id == "radial/2+4/crest44.5/descent/v7" || case.id == "5v/wall0.05/moving"
        })
        .collect()
}

/// The digest of `case_id` under spec-d2 in a scene built with `threads` worker threads: per
/// tick the report and the active contacts (body, sub-shape, normal bits) in Jolt's order.
fn determinism_digest(case_id: &str, threads: u32) -> common::determinism::Digest {
    let mut scenes = Scenes::default();
    let case = determinism_cases(&mut scenes)
        .into_iter()
        .find(|case| case.id == case_id)
        .unwrap();
    let mut scene = Scene::build_with_threads(case.scene, threads);
    let run = play(&mut scene, &Config::spec_d2(), &case, case.start, true);
    let mut digest = common::determinism::Digest::new();
    for (report, traced) in run.reports.iter().zip(&run.traces) {
        let tick = digest.push();
        report_bits(report, &mut tick.state);
        for contact in &traced.contacts {
            tick.state
                .extend_from_slice(&contact.body.unwrap_or(u32::MAX).to_le_bytes());
            tick.state
                .extend_from_slice(&contact.sub_shape.to_le_bytes());
            for c in contact.contact_normal {
                tick.state.extend_from_slice(&c.to_bits().to_le_bytes());
            }
        }
    }
    digest
}

/// The same study run twice in one process gives the same reports, bit for bit.
#[test]
fn a_study_run_is_bit_exact_when_repeated() {
    let mut scenes = Scenes::default();
    let cases = determinism_cases(&mut scenes);
    assert_eq!(cases.len(), 2);
    for case in cases {
        let a = determinism_digest(&case.id, 1);
        let b = determinism_digest(&case.id, 1);
        assert!(!a.ticks.is_empty());
        assert_eq!(a, b, "{}", case.id);
    }
}

/// Child process of `study_runs_match_across_processes_and_worker_counts`.
#[test]
#[ignore = "run as a child process by the determinism gate"]
fn study_child() {
    let Some((scenario, threads, _)) = common::determinism::child_request() else {
        return;
    };
    common::determinism::finish_child(&determinism_digest(&scenario, threads));
}

/// The study's caller gives the same ticks in separate processes with 1 and 4 worker threads.
/// No world step runs: this guards the caller and the character queries, not solver
/// scheduling.
#[test]
fn study_runs_match_across_processes_and_worker_counts() {
    let mut scenes = Scenes::default();
    for case in determinism_cases(&mut scenes) {
        let one = common::determinism::digest_in_child("study_child", &case.id, 1, "-");
        let four = common::determinism::digest_in_child("study_child", &case.id, 4, "-");
        assert!(!one.ticks.is_empty());
        common::determinism::assert_same(&case.id, &one, &four);
    }
}

/// spec-d2 down the radial crest: the character state and the caller's carry saved at tick 40,
/// a detour of 20 ticks (other input, a teleport, recovery speed 0.3), a restore, and ticks 40
/// to 80 replay the original run bit for bit.
#[test]
fn a_restored_study_run_continues_bit_for_bit_after_a_detour() {
    let mut scenes = Scenes::default();
    let case = matrix::cases(&mut scenes, Column::Radial)
        .into_iter()
        .find(|case| case.id == "radial/2+4/crest44.5/descent/v1.6")
        .unwrap();
    let config = Config::spec_d2();
    let scene = scenes.get(case.scene);
    let walker = config.create_character(scene, case.start);
    let mut carry = case.carry;
    let mut original = Vec::new();
    let mut saved = None;
    let step = |scene: &mut Scene, carry: &mut Carry, desired: V3| {
        let report = tick(scene, &walker, &config, carry, desired, None);
        scene.place_actor(report.end, common::walker::from_y_to(report.up));
        let mut bits = Vec::new();
        report_bits(&report, &mut bits);
        bits
    };
    for t in 0..80 {
        if t == 40 {
            let state = scene.world.character(walker.id).unwrap().save_state();
            saved = Some((state, carry));
        }
        let origin = study::controller::origin_of(&scene.world, &walker, 0.02);
        let desired = case.input.desired(scene, t, origin);
        original.push(step(scene, &mut carry, desired));
    }
    let (state, saved_carry) = saved.unwrap();
    let restore = |scene: &mut Scene, speed: f32| {
        let mut character = scene.world.character_mut(walker.id).unwrap();
        character.restore_state(&state).unwrap();
        character.set_penetration_recovery_speed(speed).unwrap();
    };

    restore(scene, 0.3);
    let mut detour = Carry {
        vel_up: 2.0,
        grounded: false,
    };
    for t in 0..20 {
        if t == 10 {
            let to = study::laws::resting(scene, 5.0, 0.0);
            scene
                .world
                .character_mut(walker.id)
                .unwrap()
                .set_position(study::config::position_for(to, up_at(to), 0.02))
                .unwrap();
        }
        let origin = study::controller::origin_of(&scene.world, &walker, 0.02);
        let desired = scene.up.tangent(origin, [0.0, 0.0, 1.0], 0.05);
        step(scene, &mut detour, desired);
    }

    restore(scene, config.settings.recovery_speed);
    let mut carry = saved_carry;
    for (t, expected) in original.iter().enumerate().skip(40) {
        let origin = study::controller::origin_of(&scene.world, &walker, 0.02);
        let desired = case.input.desired(scene, t, origin);
        assert!(
            step(scene, &mut carry, desired) == *expected,
            "tick {t} differs after the restore"
        );
    }
}
