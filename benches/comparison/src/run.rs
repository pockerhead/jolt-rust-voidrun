//! One run in this process: a timed run, a validation run, or Jolt's buffer probe. The matrix
//! runner starts each in a child process of its own.

use std::fmt::Write as _;
use std::path::{Path, PathBuf};
use std::time::Instant;

use crate::engine::{BodyState, Config, Engine, Profile};
use crate::engines::{dispatch, Variant, WithEngine};
use crate::os;
use crate::quality::{digest, Quality, QualityTracker};
use crate::scene::{Scene, SceneSpec};

/// What a child run is asked to do.
#[derive(Clone, Debug)]
pub struct RunArgs {
    pub run_id: String,
    pub variant: Variant,
    pub scene: Scene,
    pub profile: Profile,
    pub threads: u32,
    pub steps: usize,
    /// Solver iterations instead of the variant's.
    pub iterations: Option<u32>,
    /// Where the run writes its per-tick files.
    pub out: PathBuf,
}

impl RunArgs {
    pub fn config(&self) -> Config {
        Config {
            iterations: self.iterations.or(self.variant.iterations),
            ..Config::new(self.profile, self.threads)
        }
    }

    /// The first columns of every result row.
    fn key_columns(&self) -> String {
        format!(
            "{}\t{}\t{}\t{}\t{}\t{}\t{}",
            self.run_id,
            self.variant.name,
            self.scene.name(),
            self.profile.name(),
            self.threads,
            self.iterations
                .or(self.variant.iterations)
                .map_or("default".to_owned(), |n| n.to_string()),
            self.steps,
        )
    }
}

/// Column names shared by the first columns of `runs.tsv` and `quality.tsv`.
pub const KEY_HEADER: &str = "run_id\tvariant\tscene\tprofile\tthreads\titerations\tsteps";

/// Column names of a timing row after the key columns.
pub const TIME_HEADER: &str = "build_ns\tcpu_ns\twall_ns\tbusy_cores\tpeak_resident\tpeak_commit\tbaseline_resident\tbaseline_commit";

/// What a timing run measured besides its per-tick samples.
#[derive(Clone, Debug, PartialEq)]
pub struct TimeResult {
    pub build_ns: u64,
    pub cpu_ns: u64,
    pub wall_ns: u64,
    pub peak_resident: u64,
    pub peak_commit: u64,
    pub baseline_resident: u64,
    pub baseline_commit: u64,
}

impl TimeResult {
    /// CPU time over wall time of the timed loop: how many cores were busy on average.
    pub fn busy_cores(&self) -> f64 {
        self.cpu_ns as f64 / self.wall_ns.max(1) as f64
    }

    fn columns(&self) -> String {
        format!(
            "{}\t{}\t{}\t{:.3}\t{}\t{}\t{}\t{}",
            self.build_ns,
            self.cpu_ns,
            self.wall_ns,
            self.busy_cores(),
            self.peak_resident,
            self.peak_commit,
            self.baseline_resident,
            self.baseline_commit
        )
    }
}

/// The file of per-tick samples of `run_id` under `out`.
pub fn samples_path(out: &Path, run_id: &str) -> PathBuf {
    out.join("samples").join(format!("{run_id}.tsv"))
}

/// The file of per-tick digests of `run_id` under `out`.
pub fn digests_path(out: &Path, run_id: &str) -> PathBuf {
    out.join("digests").join(format!("{run_id}.tsv"))
}

fn write_file(path: &Path, text: &str) -> Result<(), String> {
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
    }
    std::fs::write(path, text).map_err(|e| format!("{}: {e}", path.display()))
}

/// Times one run: `build` once, then `steps` calls of `tick`, which returns the tick's samples
/// in ns (its own wall time first). CPU time and memory are read right after the last tick,
/// before the world is dropped.
fn time_run<W>(
    args: &RunArgs,
    build: impl FnOnce(&SceneSpec) -> Result<W, String>,
    mut tick: impl FnMut(&mut W) -> Result<Vec<u64>, String>,
) -> Result<(TimeResult, Vec<Vec<u64>>), String> {
    let spec = args.scene.build();
    let baseline = os::memory();
    let started = Instant::now();
    let mut world = build(&spec)?;
    let build_ns = started.elapsed().as_nanos() as u64;

    let mut samples = Vec::with_capacity(args.steps);
    let cpu_start = os::cpu_time_ns();
    let loop_start = Instant::now();
    for index in 1..=args.steps {
        samples.push(tick(&mut world).map_err(|e| format!("tick {index}: {e}"))?);
    }
    let wall_ns = loop_start.elapsed().as_nanos() as u64;
    let cpu_ns = os::cpu_time_ns() - cpu_start;
    let memory = os::memory();
    drop(world);
    let result = TimeResult {
        build_ns,
        cpu_ns,
        wall_ns,
        peak_resident: memory.peak_resident,
        peak_commit: memory.peak_commit,
        baseline_resident: baseline.resident,
        baseline_commit: baseline.peak_commit,
    };
    Ok((result, samples))
}

/// Writes the per-tick samples under `header` and returns the run's result row.
fn finish_timing(
    args: &RunArgs,
    header: &str,
    (result, samples): (TimeResult, Vec<Vec<u64>>),
) -> Result<String, String> {
    let mut text = format!("tick\t{header}\n");
    for (i, tick) in samples.iter().enumerate() {
        let columns: Vec<String> = tick.iter().map(u64::to_string).collect();
        writeln!(text, "{}\t{}", i + 1, columns.join("\t")).unwrap();
    }
    write_file(&samples_path(&args.out, &args.run_id), &text)?;
    Ok(format!("{}\t{}", args.key_columns(), result.columns()))
}

struct Timed<'a> {
    args: &'a RunArgs,
}

impl WithEngine for Timed<'_> {
    type Output = Result<(TimeResult, Vec<Vec<u64>>), String>;

    fn run<E: Engine>(self) -> Self::Output {
        let config = self.args.config();
        time_run(
            self.args,
            |spec| E::build(spec, &config),
            |engine| {
                let start = Instant::now();
                engine.step()?;
                Ok(vec![start.elapsed().as_nanos() as u64])
            },
        )
    }
}

/// A timing run: builds the world, times every tick, writes the samples and returns the
/// result row.
pub fn time(args: &RunArgs) -> Result<String, String> {
    args.variant.check_build()?;
    let timing = dispatch(args.variant.engine, Timed { args })??;
    finish_timing(args, "ns", timing)
}

/// A timing run of Avian with its total step timer: per tick the update's time and the time of
/// Avian's physics schedule inside it.
#[cfg(feature = "avian")]
pub fn split(args: &RunArgs) -> Result<String, String> {
    use crate::engines::avian::Avian;
    use crate::engines::EngineKind;

    args.variant.check_build()?;
    if args.variant.engine != EngineKind::Avian {
        return Err(format!("{}: split runs are Avian's", args.variant.name));
    }
    let config = args.config();
    let timing = time_run(
        args,
        |spec| Avian::build_app(spec, &config, true),
        |avian| {
            let start = Instant::now();
            avian.step()?;
            let update = start.elapsed().as_nanos() as u64;
            let physics = avian
                .last_physics_step_time()
                .ok_or("no physics step timer")?;
            Ok(vec![update, physics.as_nanos() as u64])
        },
    )?;
    finish_timing(args, "ns\tphysics_ns", timing)
}

/// What a validation run recorded.
#[derive(Clone, Debug)]
pub struct Validation {
    pub digests: Vec<u64>,
    pub quality: Quality,
}

struct Validated<'a> {
    args: &'a RunArgs,
    config: Config,
    spec: &'a SceneSpec,
}

impl WithEngine for Validated<'_> {
    type Output = Result<Validation, String>;

    fn run<E: Engine>(self) -> Self::Output {
        let mut engine = E::build(self.spec, &self.config)?;
        let mut tracker = QualityTracker::new(self.spec, self.args.scene.class());
        let mut states: Vec<BodyState> = Vec::new();
        let mut digests = Vec::with_capacity(self.args.steps);
        for tick in 1..=self.args.steps {
            engine.step().map_err(|e| format!("tick {tick}: {e}"))?;
            engine.read_state(&mut states);
            tracker.observe(&states);
            digests.push(digest(&states));
        }
        let quality = tracker.finish(engine.awake_bodies());
        Ok(Validation { digests, quality })
    }
}

/// Runs `args` reading every body each tick, with `config` and `spec` (the scene's unless a
/// test passes another).
pub fn validate_with(
    args: &RunArgs,
    config: Config,
    spec: &SceneSpec,
) -> Result<Validation, String> {
    args.variant.check_build()?;
    dispatch(args.variant.engine, Validated { args, config, spec })?
}

/// Column names of a quality row after the key columns.
pub const QUALITY_HEADER: &str = "non_finite_tick\theight_ratio\tground_penetration\tground_penetration_max\tfallen\tfinal_speed_p50\tfinal_speed_p99\tball_overlap\tball_overlap_max\tanchor_max\tanchor_p99\tangle_max\tangle_p99\tlimit_max\tlimit_p99\tpeak_speed\tawake_at_end\tdigest_changes\tfinal_digest\tviolations";

fn opt<T: std::fmt::Display>(value: Option<T>) -> String {
    value.map_or("n/a".to_owned(), |v| v.to_string())
}

fn opt_f(value: Option<f32>) -> String {
    value.map_or("n/a".to_owned(), |v| format!("{v:.6}"))
}

/// The quality columns of a validation.
pub fn quality_columns(validation: &Validation) -> String {
    let q = &validation.quality;
    let violations = q.violations();
    let digest_changes = validation.digests.first() != validation.digests.last();
    format!(
        "{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{:.6}\t{}\t{}\t{:016x}\t{}",
        opt(q.non_finite_tick),
        opt_f(q.height_ratio),
        opt_f(q.ground_penetration),
        opt_f(q.ground_penetration_max),
        opt(q.fallen),
        opt_f(q.final_speed_p50),
        opt_f(q.final_speed_p99),
        opt_f(q.ball_overlap),
        opt_f(q.ball_overlap_max),
        opt_f(q.anchor_max),
        opt_f(q.anchor_p99),
        opt_f(q.angle_max),
        opt_f(q.angle_p99),
        opt_f(q.limit_max),
        opt_f(q.limit_p99),
        q.peak_speed,
        q.awake_at_end,
        digest_changes,
        validation.digests.last().copied().unwrap_or(0),
        if violations.is_empty() {
            "none".to_owned()
        } else {
            violations.join(",")
        },
    )
}

/// A validation run: steps the scene reading every body each tick, writes the digests and
/// returns the quality row.
pub fn validate(args: &RunArgs) -> Result<String, String> {
    let spec = args.scene.build();
    let validation = validate_with(args, args.config(), &spec)?;
    let mut text = String::from("tick\tdigest\n");
    for (i, d) in validation.digests.iter().enumerate() {
        writeln!(text, "{}\t{d:016x}", i + 1).unwrap();
    }
    write_file(&digests_path(&args.out, &args.run_id), &text)?;
    Ok(format!(
        "{}\t{}",
        args.key_columns(),
        quality_columns(&validation)
    ))
}

/// Jolt's buffer probe for one scene: the smallest power of two from 2^14 at which `steps`
/// matched ticks at one thread drop no contact, doubled once and capped at 2^20, then a check
/// that the temp allocator is large enough: 120 ticks with it and with twice it must have mean
/// tick times within 3 %, else it is doubled. Returns the `jolt_sizes.tsv` row.
#[cfg(feature = "jolt")]
pub fn size_jolt(scene: Scene, steps: usize) -> Result<String, String> {
    use crate::engines::jolt::{Buffers, Jolt};

    let spec = scene.build();
    let config = Config::new(Profile::Matched, 1);
    let run = |buffers: Buffers, ticks: usize| -> Result<Option<f64>, String> {
        let mut engine = Jolt::build_with(&spec, &config, buffers)?;
        let start = Instant::now();
        for _ in 0..ticks {
            if engine.step().is_err() {
                return Ok(None);
            }
        }
        Ok(Some(start.elapsed().as_secs_f64() * 1e3 / ticks as f64))
    };
    let mut found = None;
    for exponent in 14..=20 {
        let size = 1u32 << exponent;
        if run(Buffers::sized(size), steps)?.is_some() {
            found = Some(size);
            break;
        }
    }
    let found = found.ok_or("no buffer size up to 2^20 completes the run")?;
    let chosen = (found * 2).min(1 << 20);
    let mut measured = Buffers::sized(chosen);
    let (mut mean, mut mean_doubled);
    loop {
        let doubled = Buffers {
            temp_allocator: measured.temp_allocator.saturating_mul(2),
            ..measured
        };
        let dropped = "the chosen size dropped contacts";
        mean = run(measured, 120)?.ok_or(dropped)?;
        mean_doubled = run(doubled, 120)?.ok_or(dropped)?;
        if mean <= mean_doubled * 1.03 || measured.temp_allocator == u32::MAX {
            break;
        }
        measured = doubled;
    }
    let table = Buffers::for_scene(scene.name());
    Ok(format!(
        "{}\t{found}\t{chosen}\t{}\t{}\t{}\t{mean:.3}\t{mean_doubled:.3}\t{}",
        scene.name(),
        table.contacts,
        measured.temp_allocator,
        table.temp_allocator,
        if measured == table {
            "table matches"
        } else {
            "table differs"
        }
    ))
}

/// Column names of `jolt_sizes.tsv`.
pub const SIZES_HEADER: &str = "scene\tsmallest_complete\tchosen\ttable\ttemp_allocator\ttable_temp_allocator\tmean_ms\tmean_ms_doubled_temp\tcheck";
