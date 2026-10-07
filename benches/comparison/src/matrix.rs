//! `comparison all`: runs every requested case in a child process of its own and writes the raw
//! result files, then the summary.

use std::collections::BTreeMap;
use std::fmt::Write as _;
use std::path::{Path, PathBuf};
use std::process::Command;

use crate::cli::Options;
use crate::engine::Profile;
use crate::engines::Variant;
use crate::measure;
use crate::quality::first_difference;
use crate::run::{digests_path, KEY_HEADER, QUALITY_HEADER, TIME_HEADER};
use crate::scene::Scene;
use crate::summary;

/// What the children of a matrix do.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    /// Timed runs.
    Time,
    /// Validation runs and the determinism gate.
    Validate,
    /// Avian runs with its physics-step timer.
    Split,
    /// Timed runs at each of several solver iteration counts.
    Sweep,
}

impl Mode {
    pub fn name(self) -> &'static str {
        match self {
            Self::Time => "time",
            Self::Validate => "validate",
            Self::Split => "split",
            Self::Sweep => "sweep",
        }
    }

    fn from_name(name: &str) -> Option<Self> {
        [Self::Time, Self::Validate, Self::Split, Self::Sweep]
            .into_iter()
            .find(|m| m.name() == name)
    }

    /// The child command that runs one case.
    fn command(self) -> &'static str {
        match self {
            Self::Time | Self::Sweep => "time",
            Self::Validate => "validate",
            Self::Split => "split",
        }
    }
}

/// One child run.
#[derive(Clone, Debug)]
struct Case {
    run_id: String,
    repeat: u32,
    variant: Variant,
    scene: Scene,
    profile: Profile,
    threads: u32,
    iterations: Option<u32>,
}

/// A comma-separated list of `T`, each parsed by `parse`.
fn list<T>(text: &str, parse: impl Fn(&str) -> Option<T>, what: &str) -> Result<Vec<T>, String> {
    text.split(',')
        .map(|item| parse(item.trim()).ok_or_else(|| format!("unknown {what} {item:?}")))
        .collect()
}

/// Scene names, with `rapier` for Rapier's ten scenes and `fixtures` for the fixtures.
fn scenes(text: &str) -> Result<Vec<Scene>, String> {
    let mut scenes = Vec::new();
    for name in text.split(',') {
        match name.trim() {
            "rapier" => scenes.extend(Scene::RAPIER),
            "fixtures" => scenes.extend(Scene::FIXTURES),
            other => {
                scenes.push(Scene::from_name(other).ok_or(format!("unknown scene {other:?}"))?)
            }
        }
    }
    Ok(scenes)
}

/// `variant=path` pairs naming the binary each variant runs in.
fn executables(text: Option<&str>) -> Result<BTreeMap<String, PathBuf>, String> {
    let mut map = BTreeMap::new();
    for pair in text.into_iter().flat_map(|t| t.split(',')) {
        let (variant, path) = pair
            .split_once('=')
            .ok_or_else(|| format!("--exe expects variant=path, got {pair:?}"))?;
        map.insert(variant.to_owned(), PathBuf::from(path));
    }
    Ok(map)
}

struct Matrix {
    mode: Mode,
    cases: Vec<Case>,
    steps: usize,
    out: PathBuf,
    exes: BTreeMap<String, PathBuf>,
    require_identical: Vec<String>,
}

impl Matrix {
    fn from_options(options: &Options) -> Result<Self, String> {
        let mode = options.get("mode").unwrap_or("time");
        let mode = Mode::from_name(mode).ok_or_else(|| format!("unknown mode {mode:?}"))?;
        let scenes = scenes(options.get("scenes").unwrap_or("rapier"))?;
        let variants = list(
            options.get("variants").unwrap_or("jolt"),
            Variant::from_name,
            "variant",
        )?;
        let threads = list(
            options.get("threads").unwrap_or("1"),
            |t| t.parse::<u32>().ok().filter(|&n| n > 0),
            "thread count",
        )?;
        let profiles = list(
            options.get("profiles").unwrap_or("matched"),
            Profile::from_name,
            "profile",
        )?;
        let iterations: Vec<Option<u32>> = match options.get("iterations") {
            Some(text) => list(text, |t| t.parse().ok().map(Some), "iteration count")?,
            None => vec![None],
        };
        let repeats = options.number("repeat", 1u32)?;
        // A later call can add repeats to an output directory without reusing run ids.
        let first = options.number("first-repeat", 1u32)?.max(1);
        let mut cases = Vec::new();
        for repeat in first..first + repeats {
            // Rotate the variant order per repeat so no variant always runs first.
            let mut order = variants.clone();
            let shift = (repeat as usize - 1) % order.len().max(1);
            order.rotate_left(shift);
            for &scene in &scenes {
                for &profile in &profiles {
                    for &iterations in &iterations {
                        for &variant in &order {
                            for &threads in &threads {
                                cases.push(Case::new(
                                    mode, repeat, variant, scene, profile, threads, iterations,
                                ));
                            }
                        }
                    }
                }
            }
        }
        Ok(Self {
            mode,
            cases,
            steps: options.number("steps", 600)?,
            out: PathBuf::from(options.get("out").unwrap_or("comparison-out")),
            exes: executables(options.get("exe"))?,
            require_identical: options
                .get("require-identical")
                .map(|t| t.split(',').map(str::to_owned).collect())
                .unwrap_or_default(),
        })
    }
}

impl Case {
    fn new(
        mode: Mode,
        repeat: u32,
        variant: Variant,
        scene: Scene,
        profile: Profile,
        threads: u32,
        iterations: Option<u32>,
    ) -> Self {
        let mut run_id = format!(
            "{}-r{repeat}-{}-{}-{}-t{threads}",
            mode.name(),
            variant.name,
            scene.name(),
            profile.name()
        );
        if let Some(n) = iterations {
            write!(run_id, "-i{n}").unwrap();
        }
        Self {
            run_id,
            repeat,
            variant,
            scene,
            profile,
            threads,
            iterations,
        }
    }
}

/// The outcome of one child.
enum Outcome {
    Row(String),
    Failed(String),
}

fn run_case(matrix: &Matrix, case: &Case) -> Result<Outcome, String> {
    let exe = match matrix.exes.get(case.variant.name) {
        Some(path) => path.clone(),
        None => std::env::current_exe().map_err(|e| e.to_string())?,
    };
    let mut command = Command::new(&exe);
    command
        .arg(matrix.mode.command())
        .args(["--run-id", &case.run_id])
        .args(["--variant", case.variant.name])
        .args(["--scene", case.scene.name()])
        .args(["--profile", case.profile.name()])
        .args(["--threads", &case.threads.to_string()])
        .args(["--steps", &matrix.steps.to_string()])
        .arg("--out")
        .arg(&matrix.out);
    if let Some(n) = case.iterations {
        command.args(["--iterations", &n.to_string()]);
    }
    let output = command
        .output()
        .map_err(|e| format!("{}: {e}", exe.display()))?;
    let stdout = String::from_utf8_lossy(&output.stdout);
    if output.status.success() {
        let row = stdout.lines().last().unwrap_or_default().to_owned();
        Ok(Outcome::Row(row))
    } else {
        let stderr = String::from_utf8_lossy(&output.stderr);
        let reason = stderr
            .lines()
            .rev()
            .find(|line| !line.trim().is_empty())
            .unwrap_or("no message")
            .replace('\t', " ");
        Ok(Outcome::Failed(format!("{}: {reason}", output.status)))
    }
}

fn append(path: &Path, header: &str, line: &str) -> Result<(), String> {
    use std::io::Write;
    let new = !path.exists();
    let mut file = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(path)
        .map_err(|e| format!("{}: {e}", path.display()))?;
    if new {
        writeln!(file, "{header}").map_err(|e| e.to_string())?;
    }
    writeln!(file, "{line}").map_err(|e| e.to_string())
}

/// Column names of `cases.tsv`, every case a matrix was asked to run, written before it runs.
pub const CASES_HEADER: &str = "mode\trun_id";

/// Column names of `failures.tsv`.
pub const FAILURES_HEADER: &str = "mode\trun_id\tvariant\tscene\tprofile\tthreads\treason";

/// Column names of `determinism.tsv`.
pub const DETERMINISM_HEADER: &str =
    "variant\tscene\tprofile\titerations\tthreads\tticks\tfirst_differing_tick\tmoved\tdigest_changes\tverdict";

/// The thread counts of one validation group compared tick by tick.
fn determinism_row(out: &Path, group: &[(&Case, &str)]) -> (String, bool) {
    let (first, _) = group[0];
    let digests: Vec<Vec<u64>> = group
        .iter()
        .map(|(case, _)| read_digests(&digests_path(out, &case.run_id)))
        .collect();
    let difference = digests
        .iter()
        .skip(1)
        .filter_map(|d| first_difference(&digests[0], d))
        .min();
    // Columns of the quality row (after the 7 key columns): peak_speed is 13, digest_changes 15.
    let column = |row: &str, i: usize| row.split('\t').nth(7 + i).unwrap_or("").to_owned();
    let moved = group
        .iter()
        .all(|(_, row)| column(row, 13).parse::<f32>().is_ok_and(|v| v > 0.1));
    let changes = group.iter().all(|(_, row)| column(row, 15) == "true");
    let threads: Vec<String> = group.iter().map(|(c, _)| c.threads.to_string()).collect();
    let verdict = if !(moved && changes) {
        "not moving"
    } else if difference.is_some() {
        "differs"
    } else {
        "identical"
    };
    let pass = verdict == "identical";
    let row = format!(
        "{}\t{}\t{}\t{}\t{}\t{}\t{}\t{moved}\t{changes}\t{}",
        first.variant.name,
        first.scene.name(),
        first.profile.name(),
        first
            .iterations
            .map_or("default".to_owned(), |n| n.to_string()),
        threads.join("/"),
        digests[0].len(),
        difference.map_or("none".to_owned(), |t| t.to_string()),
        verdict,
    );
    (row, pass)
}

fn read_digests(path: &Path) -> Vec<u64> {
    std::fs::read_to_string(path)
        .unwrap_or_default()
        .lines()
        .skip(1)
        .filter_map(|line| u64::from_str_radix(line.split('\t').nth(1)?, 16).ok())
        .collect()
}

/// Runs the matrix of `options`; an error when a case failed or a required gate did not hold,
/// after every output is written.
pub fn all(options: &Options) -> Result<String, String> {
    let matrix = Matrix::from_options(options)?;
    if matrix.mode != Mode::Validate {
        crate::cli::require_build_isa()?;
    }
    std::fs::create_dir_all(&matrix.out).map_err(|e| e.to_string())?;
    let mut machine = measure::machine_line();
    for (variant, path) in &matrix.exes {
        write!(machine, "\nexe {variant}={}", path.display()).unwrap();
    }
    std::fs::write(matrix.out.join("machine.txt"), machine + "\n").map_err(|e| e.to_string())?;

    let mut failed = 0;
    let mut rows: Vec<(&Case, String)> = Vec::new();
    for case in &matrix.cases {
        eprintln!("{}", case.run_id);
        let requested = format!("{}\t{}", matrix.mode.name(), case.run_id);
        append(&matrix.out.join("cases.tsv"), CASES_HEADER, &requested)?;
        match run_case(&matrix, case)? {
            Outcome::Row(row) => {
                let (path, header) = match matrix.mode {
                    Mode::Validate => ("quality.tsv", format!("{KEY_HEADER}\t{QUALITY_HEADER}")),
                    _ => ("runs.tsv", format!("mode\t{KEY_HEADER}\t{TIME_HEADER}")),
                };
                let line = match matrix.mode {
                    Mode::Validate => row.clone(),
                    mode => format!("{}\t{row}", mode.name()),
                };
                append(&matrix.out.join(path), &header, &line)?;
                rows.push((case, row));
            }
            Outcome::Failed(reason) => {
                failed += 1;
                let line = format!(
                    "{}\t{}\t{}\t{}\t{}\t{}\t{reason}",
                    matrix.mode.name(),
                    case.run_id,
                    case.variant.name,
                    case.scene.name(),
                    case.profile.name(),
                    case.threads
                );
                append(&matrix.out.join("failures.tsv"), FAILURES_HEADER, &line)?;
            }
        }
    }

    let mut gate_failures = Vec::new();
    if matrix.mode == Mode::Validate {
        let mut groups: BTreeMap<(String, u32), Vec<(&Case, &str)>> = BTreeMap::new();
        for (case, row) in &rows {
            let key = format!(
                "{}\t{}\t{}\t{:?}",
                case.variant.name,
                case.scene.name(),
                case.profile.name(),
                case.iterations
            );
            groups
                .entry((key, case.repeat))
                .or_default()
                .push((case, row.as_str()));
        }
        // A group run at one thread count has nothing to compare.
        for group in groups.values().filter(|group| group.len() > 1) {
            let (row, pass) = determinism_row(&matrix.out, group);
            append(
                &matrix.out.join("determinism.tsv"),
                DETERMINISM_HEADER,
                &row,
            )?;
            let variant = group[0].0.variant.name;
            if !pass && matrix.require_identical.iter().any(|v| v == variant) {
                gate_failures.push(row);
            }
        }
    }

    summary::write(&matrix.out)?;
    match (failed, gate_failures.is_empty()) {
        (0, true) => Ok(format!(
            "{} cases written to {}",
            matrix.cases.len(),
            matrix.out.display()
        )),
        _ => Err(format!(
            "{failed} of {} cases failed; {} required determinism gates failed{}",
            matrix.cases.len(),
            gate_failures.len(),
            gate_failures
                .iter()
                .map(|r| format!("\n  {r}"))
                .collect::<String>()
        )),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn case(threads: u32) -> Case {
        let variant = Variant::from_name("jolt").unwrap();
        Case::new(
            Mode::Validate,
            1,
            variant,
            Scene::FixtureStack,
            Profile::Matched,
            threads,
            None,
        )
    }

    /// A quality row whose peak speed and digest-change columns say the scene moved.
    fn moving_row() -> String {
        let mut columns = vec!["x"; 7 + 18];
        columns[7 + 13] = "1.0";
        columns[7 + 15] = "true";
        columns.join("\t")
    }

    fn write_digests(out: &Path, case: &Case, digests: &[u64]) {
        let mut text = String::from("tick\tdigest\n");
        for (i, d) in digests.iter().enumerate() {
            writeln!(text, "{}\t{d:016x}", i + 1).unwrap();
        }
        let path = digests_path(out, &case.run_id);
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::write(path, text).unwrap();
    }

    #[test]
    fn the_gate_passes_equal_streams_and_names_the_first_differing_tick() {
        let out = std::env::temp_dir().join(format!("comparison-gate-{}", std::process::id()));
        let (one, four) = (case(1), case(4));
        let row = moving_row();
        write_digests(&out, &one, &[1, 2, 3, 4]);
        write_digests(&out, &four, &[1, 2, 3, 4]);
        let group = [(&one, row.as_str()), (&four, row.as_str())];
        let (line, pass) = determinism_row(&out, &group);
        assert!(pass, "{line}");

        write_digests(&out, &four, &[1, 2, 9, 4]);
        let (line, pass) = determinism_row(&out, &group);
        assert!(!pass);
        assert!(line.contains("\t3\t"), "{line}");

        let still = row.replace("1.0", "0.0");
        write_digests(&out, &four, &[1, 2, 3, 4]);
        let (line, pass) =
            determinism_row(&out, &[(&one, still.as_str()), (&four, still.as_str())]);
        assert!(!pass && line.ends_with("not moving"), "{line}");
        let _ = std::fs::remove_dir_all(out);
    }
}
