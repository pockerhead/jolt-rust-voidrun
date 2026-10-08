//! `comparison summarize <dir>`: `summary.md` from the raw files of a matrix run, and nothing
//! else, so every published number can be regenerated.

use std::collections::BTreeMap;
use std::fmt::Write as _;
use std::path::Path;

use crate::measure::{Stats, Window, COLD, TICK_1, WARM};

/// A TSV file as rows of column name to value; empty when the file does not exist.
pub fn read_tsv(path: &Path) -> Vec<BTreeMap<String, String>> {
    let Ok(text) = std::fs::read_to_string(path) else {
        return Vec::new();
    };
    let mut lines = text.lines();
    let Some(header) = lines.next() else {
        return Vec::new();
    };
    let names: Vec<&str> = header.split('\t').collect();
    lines
        .filter(|line| !line.is_empty())
        .map(|line| {
            names
                .iter()
                .zip(line.split('\t'))
                .map(|(n, v)| ((*n).to_owned(), v.to_owned()))
                .collect()
        })
        .collect()
}

type Row = BTreeMap<String, String>;

/// Timing rows by profile and scene, then by variant, threads and iterations; each group holds
/// the repeats.
type RunGroups<'a> = BTreeMap<(String, String), BTreeMap<(String, u32, String), Vec<&'a Row>>>;

fn get<'a>(row: &'a Row, name: &str) -> &'a str {
    row.get(name).map_or("", String::as_str)
}

fn number(row: &Row, name: &str) -> f64 {
    get(row, name).parse().unwrap_or(f64::NAN)
}

/// Column `column` (1-based) of a samples file.
fn samples(dir: &Path, run_id: &str, column: usize) -> Vec<u64> {
    let path = dir.join("samples").join(format!("{run_id}.tsv"));
    std::fs::read_to_string(path)
        .unwrap_or_default()
        .lines()
        .skip(1)
        .filter_map(|line| line.split('\t').nth(column)?.parse().ok())
        .collect()
}

fn ms(ns: f64) -> String {
    format!("{:.3}", ns / 1e6)
}

fn mb(bytes: f64) -> String {
    format!("{:.0}", bytes / (1u64 << 20) as f64)
}

/// The median of `values`, which must not be empty; the mean of the two middle values when
/// their count is even.
fn median(values: &mut [f64]) -> f64 {
    values.sort_by(f64::total_cmp);
    let n = values.len();
    (values[(n - 1) / 2] + values[n / 2]) / 2.0
}

/// The key that joins a timing row to its validation row.
fn quality_key(row: &Row) -> String {
    ["variant", "scene", "profile", "threads", "iterations"]
        .map(|n| get(row, n))
        .join("/")
}

fn quality_cell(quality: &BTreeMap<String, Row>, row: &Row) -> String {
    match quality.get(&quality_key(row)) {
        None => "not validated".to_owned(),
        Some(q) if get(q, "violations") == "none" => "within bounds".to_owned(),
        Some(q) => format!("below bound: {}", get(q, "violations").replace(',', ", ")),
    }
}

/// The timing rows of one mode, grouped.
fn group_runs<'a>(runs: &'a [Row], mode: &str) -> RunGroups<'a> {
    let mut groups: BTreeMap<_, BTreeMap<_, Vec<&Row>>> = BTreeMap::new();
    for row in runs.iter().filter(|r| get(r, "mode") == mode) {
        let outer = (get(row, "profile").to_owned(), get(row, "scene").to_owned());
        let inner = (
            get(row, "variant").to_owned(),
            get(row, "threads").parse().unwrap_or(0),
            get(row, "iterations").to_owned(),
        );
        groups
            .entry(outer)
            .or_default()
            .entry(inner)
            .or_default()
            .push(row);
    }
    groups
}

/// The statistics of `window` over all repeats' samples pooled, and the range of the per-run
/// means.
fn window_stats(dir: &Path, repeats: &[&Row], window: Window) -> Option<(Stats, f64, f64)> {
    let mut pooled = Vec::new();
    let mut means = Vec::new();
    for row in repeats {
        let all = samples(dir, get(row, "run_id"), 1);
        let part = window.of(&all);
        if let Some(stats) = Stats::of(part) {
            means.push(stats.mean);
        }
        pooled.extend_from_slice(part);
    }
    let stats = Stats::of(&pooled)?;
    let low = means.iter().copied().fold(f64::INFINITY, f64::min);
    let high = means.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    Some((stats, low, high))
}

fn timing_section(dir: &Path, runs: &[Row], quality: &BTreeMap<String, Row>, out: &mut String) {
    for ((profile, scene), variants) in group_runs(runs, "time") {
        writeln!(out, "### {scene}, {profile}\n").unwrap();
        out.push_str(
            "| variant | threads | runs | warm mean ms | warm p50 | warm p95 | warm p99 | warm max | run means | cold mean | tick 1 | build + tick 1 | busy cores | peak memory MB | quality |\n",
        );
        out.push_str("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|\n");
        for ((variant, threads, _), repeats) in variants {
            let warm = window_stats(dir, &repeats, WARM);
            let cold = window_stats(dir, &repeats, COLD);
            let mut tick1: Vec<f64> = repeats
                .iter()
                .filter_map(|r| {
                    TICK_1
                        .of(&samples(dir, get(r, "run_id"), 1))
                        .first()
                        .map(|&t| t as f64)
                })
                .collect();
            let mut build_tick1: Vec<f64> = repeats
                .iter()
                .zip(&tick1)
                .map(|(r, t)| number(r, "build_ns") + t)
                .collect();
            let busy =
                repeats.iter().map(|r| number(r, "busy_cores")).sum::<f64>() / repeats.len() as f64;
            let peak = repeats
                .iter()
                .map(|r| number(r, "peak_resident"))
                .fold(0.0, f64::max);
            let cell = |w: Option<(Stats, f64, f64)>, f: fn(&Stats) -> f64| {
                w.map_or("n/a".to_owned(), |(s, _, _)| ms(f(&s)))
            };
            let spread = warm.map_or("n/a".to_owned(), |(_, low, high)| {
                format!("{}–{}", ms(low), ms(high))
            });
            let median_or_na = |v: &mut Vec<f64>| {
                if v.is_empty() {
                    "n/a".to_owned()
                } else {
                    ms(median(v))
                }
            };
            writeln!(
                out,
                "| {variant} | {threads} | {} | {} | {} | {} | {} | {} | {spread} | {} | {} | {} | {busy:.2} | {} | {} |",
                repeats.len(),
                cell(warm, |s| s.mean),
                cell(warm, |s| s.p50 as f64),
                cell(warm, |s| s.p95 as f64),
                cell(warm, |s| s.p99 as f64),
                cell(warm, |s| s.max as f64),
                cell(cold, |s| s.mean),
                median_or_na(&mut tick1),
                median_or_na(&mut build_tick1),
                mb(peak),
                quality_cell(quality, repeats[0]),
            )
            .unwrap();
        }
        out.push('\n');
    }
}

fn sweep_section(dir: &Path, runs: &[Row], quality: &BTreeMap<String, Row>, out: &mut String) {
    let groups = group_runs(runs, "sweep");
    if groups.is_empty() {
        return;
    }
    out.push_str("## Solver sweep\n\n");
    for ((profile, scene), variants) in groups {
        writeln!(out, "### {scene}, {profile}\n").unwrap();
        out.push_str("| variant | threads | iterations | warm mean ms | warm p99 | anchor p99 m | angle p99 rad | height ratio | quality |\n");
        out.push_str("|---|---|---|---|---|---|---|---|---|\n");
        for ((variant, threads, iterations), repeats) in variants {
            let warm = window_stats(dir, &repeats, WARM);
            let q = quality.get(&quality_key(repeats[0]));
            let column = |name| q.map_or("n/a", |q| get(q, name)).to_owned();
            writeln!(
                out,
                "| {variant} | {threads} | {iterations} | {} | {} | {} | {} | {} | {} |",
                warm.map_or("n/a".to_owned(), |(s, _, _)| ms(s.mean)),
                warm.map_or("n/a".to_owned(), |(s, _, _)| ms(s.p99 as f64)),
                column("anchor_p99"),
                column("angle_p99"),
                column("height_ratio"),
                quality_cell(quality, repeats[0]),
            )
            .unwrap();
        }
        out.push('\n');
    }
}

fn split_section(dir: &Path, runs: &[Row], out: &mut String) {
    let groups = group_runs(runs, "split");
    if groups.is_empty() {
        return;
    }
    out.push_str("## Avian: physics schedule and the rest of the update\n\n");
    out.push_str("| scene | profile | threads | update mean ms | physics schedule mean ms | rest of the update mean ms | timing-run update mean ms |\n");
    out.push_str("|---|---|---|---|---|---|---|\n");
    let timed = group_runs(runs, "time");
    for ((profile, scene), variants) in groups {
        for ((variant, threads, iterations), repeats) in variants {
            let pooled = |column| -> Vec<u64> {
                repeats
                    .iter()
                    .flat_map(|r| WARM.of(&samples(dir, get(r, "run_id"), column)).to_vec())
                    .collect()
            };
            let (updates, physics) = (pooled(1), pooled(2));
            let (Some(u), Some(p)) = (Stats::of(&updates), Stats::of(&physics)) else {
                continue;
            };
            let plain = timed
                .get(&(profile.clone(), scene.clone()))
                .and_then(|v| v.get(&(variant.clone(), threads, iterations.clone())))
                .and_then(|r| window_stats(dir, r, WARM))
                .map_or("n/a".to_owned(), |(s, _, _)| ms(s.mean));
            writeln!(
                out,
                "| {scene} | {profile} | {threads} | {} | {} | {} | {plain} |",
                ms(u.mean),
                ms(p.mean),
                ms(u.mean - p.mean)
            )
            .unwrap();
        }
    }
    out.push('\n');
}

/// A table of every row of `rows` with the named columns.
fn table(rows: &[Row], columns: &[&str], out: &mut String) {
    writeln!(out, "| {} |", columns.join(" | ")).unwrap();
    writeln!(out, "|{}", "---|".repeat(columns.len())).unwrap();
    for row in rows {
        let cells: Vec<&str> = columns.iter().map(|c| get(row, c)).collect();
        writeln!(out, "| {} |", cells.join(" | ")).unwrap();
    }
    out.push('\n');
}

/// The summary of the raw files in `dir`.
pub fn summarize(dir: &Path) -> String {
    let runs = read_tsv(&dir.join("runs.tsv"));
    let quality_rows = read_tsv(&dir.join("quality.tsv"));
    let quality: BTreeMap<String, Row> = quality_rows
        .iter()
        .map(|row| (quality_key(row), row.clone()))
        .collect();
    let mut out = String::from("# Comparison results\n\nGenerated by `comparison summarize` from the raw files next to this one. Times in milliseconds per tick; warm is ticks 61-600, cold ticks 1-60; percentiles are nearest-rank over the pooled samples of all runs of a row; run means is the range of the per-run warm means.\n\n");
    if let Ok(machine) = std::fs::read_to_string(dir.join("machine.txt")) {
        writeln!(out, "## Machine\n\n```text\n{}\n```\n", machine.trim_end()).unwrap();
    }
    if runs.iter().any(|r| get(r, "mode") == "time") {
        out.push_str("## Timing\n\n");
        timing_section(dir, &runs, &quality, &mut out);
    }
    sweep_section(dir, &runs, &quality, &mut out);
    split_section(dir, &runs, &mut out);
    if !quality_rows.is_empty() {
        out.push_str("## Quality\n\n");
        table(
            &quality_rows,
            &[
                "variant",
                "scene",
                "profile",
                "threads",
                "iterations",
                "height_ratio",
                "ground_penetration",
                "ground_penetration_max",
                "fallen",
                "final_speed_p99",
                "ball_overlap",
                "ball_overlap_max",
                "anchor_p99",
                "angle_p99",
                "limit_p99",
                "awake_at_end",
                "violations",
            ],
            &mut out,
        );
    }
    let determinism = read_tsv(&dir.join("determinism.tsv"));
    if !determinism.is_empty() {
        out.push_str("## Determinism across thread counts\n\n");
        table(
            &determinism,
            &[
                "variant",
                "scene",
                "profile",
                "iterations",
                "threads",
                "ticks",
                "first_differing_tick",
                "verdict",
            ],
            &mut out,
        );
    }
    let failures = read_tsv(&dir.join("failures.tsv"));
    if !failures.is_empty() {
        out.push_str("## Failed runs\n\n");
        table(
            &failures,
            &["mode", "variant", "scene", "profile", "threads", "reason"],
            &mut out,
        );
    }
    out
}

/// Writes `summary.md` into `dir`.
pub fn write(dir: &Path) -> Result<(), String> {
    std::fs::write(dir.join("summary.md"), summarize(dir)).map_err(|e| e.to_string())
}
