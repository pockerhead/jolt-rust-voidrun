//! Times the character study's configurations (`tests/study/`, `docs/character-study.md`) and
//! prints one markdown table.
//!
//! - W1, the planet walk: the near-step workload of the `budgets` bench (3 x 3 planet chunks, 30
//!   characters on rings walking 2 m/s along slowly turning headings), one character per start
//!   created with the row's settings; 300 untimed ticks, then 5000 timed ticks, three
//!   repetitions with the rows in forward, reverse and forward order. One sample is one
//!   character's whole move: from the up and pose setters through the velocity output,
//!   including the contact readout, the row's passes and the caller's bookkeeping. The actor
//!   capsule sync, scene building and character creation are outside the timer.
//! - W2, the law suite: every case of the study's law columns from the start the law gates
//!   judge (the unperturbed start, put back at rest for the row's padding), one sample per
//!   tick, also per law. Each timed run is judged afterwards and must give the outcome the
//!   untimed evaluation gives.
//! - W3, physics calls: W1 again (one repetition), timing only the physics calls of each move
//!   (refreshes, updates, the passes' queries, the contact readout and the autostep), summed per
//!   move.
//!
//! In the same run it times the repository's reference near step (`tests/common/walker.rs`) on W1
//! and `update_character` alone with the walker's settings ("update only"). Before timing a
//! pinned row, an untimed pass over W2 checks its cells against the pinned table and stops on a
//! mismatch. The timer's own cost is measured and printed, and nothing is subtracted.
//! Percentiles use the nearest rank. Timings are reported, never checked.
//!
//! Run with `cargo bench -p oxijolt --bench character_study`; words after `--` keep only the
//! rows whose names contain one of them.

#[path = "../tests/common/mod.rs"]
mod common;
#[path = "budgets/planet.rs"]
mod planet;
// The budgets bench uses the rest of the report module.
#[allow(dead_code)]
#[path = "budgets/report.rs"]
mod report;
#[path = "../tests/study/mod.rs"]
mod study;

use std::error::Error;
use std::hint::black_box;
use std::time::Instant;

use common::math::{add, scale, vec3, V3};
use common::walker::{
    from_y_to, near_tick, origin, position_for as walker_position, up_at, Carry, Walker, G,
};
use common::DT;
use oxijolt::*;
use report::{micros, print_machine};
use study::config::Config;
use study::controller::{start_physics_timer, take_physics_time, tick, TickReport};
use study::expected::{pinned, Cell, PINNED};
use study::laws::{
    evaluate, judge, perturbed_start, reference_run, Case, Column, Input, Run, Scenes,
    PERTURBATIONS,
};
use study::matrix;
use study::scenes::{add_actor, Scene};

const WARMUP_TICKS: usize = 300;
const TICKS: usize = 5_000;
const REPETITIONS: usize = 3;

/// The nearest-rank `p` percentile of `sorted`, which is not empty.
fn percentile(sorted: &[f64], p: f64) -> f64 {
    let n = sorted.len();
    sorted[((p * n as f64).ceil() as usize).clamp(1, n) - 1]
}

/// Sorted samples with their percentiles.
#[derive(Default)]
struct Samples(Vec<f64>);

impl Samples {
    fn sorted(mut self) -> Vec<f64> {
        self.0.sort_by(f64::total_cmp);
        self.0
    }
}

/// p50, p99 and max of `samples`, or dashes when empty.
fn stats(samples: Samples) -> (String, String, String, usize) {
    let sorted = samples.sorted();
    if sorted.is_empty() {
        return ("-".into(), "-".into(), "-".into(), 0);
    }
    (
        format!("{:.1}", percentile(&sorted, 0.5)),
        format!("{:.1}", percentile(&sorted, 0.99)),
        format!("{:.1}", sorted[sorted.len() - 1]),
        sorted.len(),
    )
}

/// How often a row's passes acted.
#[derive(Default)]
struct Rates {
    moves: usize,
    grounded: usize,
    recovered: usize,
    pushed: usize,
    still: usize,
    stepped: usize,
    snapped: usize,
}

impl Rates {
    fn count(&mut self, report: &TickReport) {
        self.moves += 1;
        self.grounded += usize::from(report.grounded);
        self.recovered += usize::from(report.recovered);
        self.pushed += usize::from(report.pushed);
        self.still += usize::from(report.still_ran);
        self.stepped += usize::from(report.stepped);
        self.snapped += usize::from(report.snapped);
    }

    fn describe(&self) -> String {
        let per_1000 = |n: usize| n as f64 * 1000.0 / self.moves.max(1) as f64;
        format!(
            "grounded {:.1}%; per 1000 moves: underground {:.1}, Q6 push {:.1}, still {:.1}, autostep {:.1}, Q5 snap {:.1}",
            per_1000(self.grounded) / 10.0,
            per_1000(self.recovered),
            per_1000(self.pushed),
            per_1000(self.still),
            per_1000(self.stepped),
            per_1000(self.snapped),
        )
    }
}

/// The planet scene with one character per start for `row`, each with its own actor capsule.
fn planet_characters(row: &Config) -> Result<(Scene, Vec<Walker>), Box<dyn Error>> {
    let (world, layers) = planet::planet_scene(1)?;
    let mut scene = Scene::from_world(world, layers);
    let walkers = (0..planet::CHARACTERS)
        .map(|n| {
            let start = planet::walker_start(n);
            let actor = add_actor(&mut scene.world, &scene.layers);
            let walker =
                row.create_character_with_actor(&mut scene.world, layers, scene.up, actor, start);
            sync_actor(&mut scene.world, &walker, start);
            walker
        })
        .collect();
    scene.world.optimize_broad_phase();
    Ok((scene, walkers))
}

/// Moves `walker`'s actor capsule to body origin `origin`.
fn sync_actor(world: &mut PhysicsWorld, walker: &Walker, origin: V3) {
    world
        .body_mut(walker.actor)
        .unwrap()
        .set_position_and_rotation(
            common::math::rvec3(origin),
            from_y_to(up_at(origin)),
            Activation::DontActivate,
        )
        .unwrap();
}

/// What a W1 run measured.
struct Walk {
    samples: Samples,
    cold: f64,
    rates: Rates,
    checksum: f64,
}

/// W1 for `row`; with `physics_only`, each sample is the move's physics calls (W3).
fn planet_walk(row: &Config, physics_only: bool) -> Result<Walk, Box<dyn Error>> {
    let (mut scene, walkers) = planet_characters(row)?;
    let mut carries = vec![Carry::RESTING; walkers.len()];
    let mut walk = Walk {
        samples: Samples(Vec::with_capacity(TICKS * walkers.len())),
        cold: 0.0,
        rates: Rates::default(),
        checksum: 0.0,
    };
    for t in 0..WARMUP_TICKS + TICKS {
        for (n, (walker, carry)) in walkers.iter().zip(&mut carries).enumerate() {
            let desired = black_box(planet::wanted(&scene.world, walker, n, t));
            if physics_only {
                start_physics_timer();
            }
            let start = Instant::now();
            let report = tick(&mut scene, walker, row, carry, desired, None);
            let us = if physics_only {
                take_physics_time().as_secs_f64() * 1e6
            } else {
                micros(start)
            };
            let end = black_box(report.end);
            black_box(*carry);
            sync_actor(&mut scene.world, walker, end);
            if t == 0 && n == 0 {
                walk.cold = us;
            }
            if t >= WARMUP_TICKS {
                walk.samples.0.push(us);
                walk.rates.count(&report);
                walk.checksum += end.iter().sum::<f64>();
            }
        }
    }
    Ok(walk)
}

/// The reference near step on W1.
fn reference_walk() -> Result<Walk, Box<dyn Error>> {
    let (mut world, layers) = planet::planet_scene(1)?;
    let walkers = planet::planet_walkers(&mut world, &layers);
    let mut carries = vec![Carry::RESTING; walkers.len()];
    let mut walk = Walk {
        samples: Samples(Vec::with_capacity(TICKS * walkers.len())),
        cold: 0.0,
        rates: Rates::default(),
        checksum: 0.0,
    };
    for t in 0..WARMUP_TICKS + TICKS {
        for (n, (walker, carry)) in walkers.iter().zip(&mut carries).enumerate() {
            let desired = black_box(planet::wanted(&world, walker, n, t));
            let start = Instant::now();
            let out = near_tick(&mut world, walker, carry, desired);
            let us = micros(start);
            black_box(out.pos);
            if t == 0 && n == 0 {
                walk.cold = us;
            }
            if t >= WARMUP_TICKS {
                walk.samples.0.push(us);
                walk.rates.moves += 1;
                walk.rates.grounded += usize::from(out.grounded);
                walk.checksum += out.pos.iter().sum::<f64>();
            }
        }
    }
    Ok(walk)
}

/// `update_character` alone on W1 with the walker's settings: pose and velocity set untimed,
/// the update timed, stick to floor 0.3 along -up, no stairs.
fn update_only_walk() -> Result<Walk, Box<dyn Error>> {
    let (mut world, layers) = planet::planet_scene(1)?;
    let walkers = planet::planet_walkers(&mut world, &layers);
    let filter_layers = [layers.terrain, layers.chunk, layers.actor];
    let mut walk = Walk {
        samples: Samples(Vec::with_capacity(TICKS * walkers.len())),
        cold: 0.0,
        rates: Rates::default(),
        checksum: 0.0,
    };
    for t in 0..WARMUP_TICKS + TICKS {
        for (n, walker) in walkers.iter().enumerate() {
            let desired = planet::wanted(&world, walker, n, t);
            let here = origin(&world, walker);
            let up = up_at(here);
            {
                let mut character = world.character_mut(walker.id)?;
                character.set_up(vec3(up))?;
                character.set_rotation(from_y_to(up))?;
                character.set_position(walker_position(here, up))?;
                let velocity = add(
                    scale(desired, 1.0 / f64::from(DT)),
                    scale(up, -f64::from(G * DT)),
                );
                character.set_linear_velocity(vec3(velocity))?;
            }
            let extended = ExtendedUpdateSettings::default()
                .stick_to_floor_step_down(vec3(scale(up, -0.3)))
                .walk_stairs_step_up(Vec3::ZERO);
            let gravity = vec3(scale(up, -f64::from(G)));
            let filter = common::walker::controller_filter(walker, &filter_layers);
            let start = Instant::now();
            world.update_character(walker.id, DT, gravity, &extended, &filter)?;
            let us = micros(start);
            let end = origin(&world, walker);
            common::walker::sync_actor(&mut world, walker);
            if t == 0 && n == 0 {
                walk.cold = us;
            }
            if t >= WARMUP_TICKS {
                walk.samples.0.push(us);
                walk.rates.moves += 1;
                walk.rates.grounded += usize::from(
                    world.character(walker.id)?.ground_state() == GroundState::OnGround,
                );
                walk.checksum += end.iter().sum::<f64>();
            }
        }
    }
    Ok(walk)
}

/// W2: every law case from the start the law gates judge, one sample per tick, per column. The
/// timed run is then judged (untimed) and must give the same outcome as `evaluate`.
fn law_suite(
    scenes: &mut Scenes,
    columns: &[(Column, Vec<Case>)],
    row: &Config,
) -> Vec<(Column, Samples)> {
    columns
        .iter()
        .map(|(column, cases)| {
            let mut samples = Samples::default();
            for case in cases.iter().filter(|case| !case.report_only) {
                let padding = row.settings.padding;
                let scene = scenes.get(case.scene);
                let start = perturbed_start(scene, case, PERTURBATIONS[0], padding);
                let walker = row.create_character(scene, start);
                let startup = scene.world.character(walker.id).unwrap().ground_state();
                let mut carry = case.carry;
                let mut reports = Vec::with_capacity(case.ticks);
                for t in 0..case.ticks {
                    if let Input::Teleport { at, to, .. } = case.input {
                        if t == at {
                            let up = scene.up.up_at(to);
                            scene
                                .world
                                .character_mut(walker.id)
                                .unwrap()
                                .set_position(study::config::position_for(to, up, padding))
                                .unwrap();
                        }
                    }
                    let here = study::controller::origin_of(&scene.world, &walker, padding);
                    let desired = black_box(case.input.desired(scene, t, here));
                    let timer = Instant::now();
                    let report = tick(scene, &walker, row, &mut carry, desired, None);
                    samples.0.push(micros(timer));
                    scene.place_actor(black_box(report.end), from_y_to(report.up));
                    reports.push(report);
                }
                scene.world.remove_character(walker.id).unwrap();
                if row.settings.inner_body {
                    scene.world.optimize_broad_phase();
                }
                let run = Run {
                    start,
                    padding,
                    startup,
                    reports,
                    traces: Vec::new(),
                };
                let reference = reference_run(scenes, row, case, start);
                let timed = judge(scenes, case, &run, reference.as_ref());
                let evaluated = evaluate(scenes, row, case, PERTURBATIONS[0]);
                assert_eq!(timed, evaluated, "{} {}: the timed run", row.name, case.id);
            }
            (*column, samples)
        })
        .collect()
}

/// Checks `row`'s cells on W2 against the pinned table (variable cells skipped).
fn validate(scenes: &mut Scenes, columns: &[(Column, Vec<Case>)], row: &Config) {
    if !PINNED.iter().any(|(name, _)| *name == row.name) {
        return;
    }
    let cells = pinned(row.name);
    for (column, cases) in columns {
        let cell = cells[column.index()];
        if cell == Cell::Variable {
            continue;
        }
        let results = matrix::outcomes(scenes, row, cases, 1);
        if let Err(error) = matrix::agrees(&results, cell) {
            panic!("{} column {}: {error}", row.name, column.label());
        }
    }
}

/// The timer's own cost: 100,000 back-to-back `Instant::now()` pairs.
fn timer_line() -> String {
    let mut samples = Samples(Vec::with_capacity(100_000));
    for _ in 0..100_000 {
        let start = Instant::now();
        let end = Instant::now();
        samples.0.push((end - start).as_secs_f64() * 1e6);
    }
    let sorted = samples.sorted();
    let resolution = sorted.iter().copied().find(|&us| us > 0.0).unwrap_or(0.0);
    format!(
        "timer: empty Instant pair p50 {:.3} us, p99 {:.3} us, smallest non-zero difference {:.3} us (nothing subtracted)",
        percentile(&sorted, 0.5),
        percentile(&sorted, 0.99),
        resolution
    )
}

fn main() -> Result<(), Box<dyn Error>> {
    // `cargo test --benches` runs this binary without `--bench`; the full run takes minutes.
    if !std::env::args().any(|arg| arg == "--bench") {
        println!("run with `cargo bench -p oxijolt --bench character_study`");
        return Ok(());
    }
    let filters: Vec<String> = std::env::args()
        .skip(1)
        .filter(|arg| !arg.starts_with('-'))
        .collect();
    let rows: Vec<Config> = Config::pinned()
        .into_iter()
        .chain(Config::survey_rows())
        .filter(|row| filters.is_empty() || filters.iter().any(|f| row.name.contains(f.as_str())))
        .collect();

    let timer = timer_line();
    let mut scenes = Scenes::default();
    let columns: Vec<(Column, Vec<Case>)> = Column::ALL
        .into_iter()
        .map(|column| (column, matrix::cases(&mut scenes, column)))
        .collect();
    for row in &rows {
        validate(&mut scenes, &columns, row);
    }

    let mut w1: Vec<(Samples, f64, Rates, f64)> = rows
        .iter()
        .map(|_| (Samples::default(), 0.0, Rates::default(), 0.0))
        .collect();
    let mut reference = Samples::default();
    let mut update_only = Samples::default();
    let mut reference_note = String::new();
    let mut checksum = 0.0;
    for repetition in 0..REPETITIONS {
        let order: Vec<usize> = if repetition % 2 == 0 {
            (0..rows.len()).collect()
        } else {
            (0..rows.len()).rev().collect()
        };
        for i in order {
            let walk = planet_walk(&rows[i], false)?;
            let slot = &mut w1[i];
            if repetition == 0 {
                slot.1 = walk.cold;
                slot.2 = walk.rates;
            }
            slot.0 .0.extend(walk.samples.0);
            slot.3 += walk.checksum;
            checksum += walk.checksum;
        }
        let walk = reference_walk()?;
        reference_note = format!(
            "grounded {:.1}%",
            walk.rates.grounded as f64 * 100.0 / walk.rates.moves as f64
        );
        reference.0.extend(walk.samples.0);
        update_only.0.extend(update_only_walk()?.samples.0);
    }

    println!("| row | W1 p50 us | W1 p99 us | W1 max us | W1 samples | W2 p50 us | W2 p99 us | W3 p50 us | W3 p99 us | cold first call us | pass rates (W1) |");
    println!("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|");
    let mut per_law = Vec::new();
    for (row, (samples, cold, rates, _)) in rows.iter().zip(w1) {
        let suite = law_suite(&mut scenes, &columns, row);
        let mut all = Samples::default();
        for (column, samples) in suite {
            all.0.extend(samples.0.iter().copied());
            let (p50, p99, _, n) = stats(samples);
            per_law.push(format!(
                "| {} | {} | {p50} | {p99} | {n} |",
                row.name,
                column.label()
            ));
        }
        let physics = planet_walk(row, true)?;
        let (w1_p50, w1_p99, w1_max, n) = stats(samples);
        let (w2_p50, w2_p99, _, _) = stats(all);
        let (w3_p50, w3_p99, _, _) = stats(physics.samples);
        println!(
            "| {} | {w1_p50} | {w1_p99} | {w1_max} | {n} | {w2_p50} | {w2_p99} | {w3_p50} | {w3_p99} | {cold:.1} | {} |",
            row.name,
            rates.describe()
        );
    }
    let (p50, p99, max, n) = stats(reference);
    println!("| reference near step (`near_tick`) | {p50} | {p99} | {max} | {n} | - | - | - | - | - | {reference_note} |");
    let (p50, p99, max, n) = stats(update_only);
    println!("| update only (`update_character`, walker settings) | {p50} | {p99} | {max} | {n} | - | - | - | - | - | - |");
    println!();
    println!("| row | law | W2 p50 us | W2 p99 us | samples |");
    println!("|---|---|---:|---:|---:|");
    for line in per_law {
        println!("{line}");
    }
    println!();
    println!("{timer}");
    println!("checksum of every W1 final origin: {checksum:.6}");
    print_machine();
    Ok(())
}
