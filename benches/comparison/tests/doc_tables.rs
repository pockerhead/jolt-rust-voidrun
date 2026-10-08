//! Every number `docs/comparison.md` publishes comes from files in this repository: result
//! tables are copied from a committed `summary.md`, each `summary.md` is what `summarize` makes
//! of its raw files, every requested case has a result or a failure, every feature claim has a
//! source, and the real-game numbers are the owner's.

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};

use comparison::summary::{read_tsv, summarize};

fn manifest_dir() -> &'static Path {
    Path::new(env!("CARGO_MANIFEST_DIR"))
}

fn doc() -> String {
    let path = manifest_dir().join("../../docs/comparison.md");
    std::fs::read_to_string(path).unwrap().replace("\r\n", "\n")
}

/// The lines of the `## title` section of `text`, up to the next `## ` heading.
fn section<'a>(text: &'a str, title: &str) -> Vec<&'a str> {
    let heading = format!("## {title}");
    let mut lines = text.lines().skip_while(|line| *line != heading);
    assert!(lines.next().is_some(), "no section {heading:?}");
    lines.take_while(|line| !line.starts_with("## ")).collect()
}

/// The committed result directories (those with a `summary.md`).
fn result_dirs() -> Vec<PathBuf> {
    let mut dirs: Vec<PathBuf> = std::fs::read_dir(manifest_dir().join("results"))
        .unwrap()
        .map(|entry| entry.unwrap().path())
        .filter(|path| path.join("summary.md").exists())
        .collect();
    dirs.sort();
    assert!(!dirs.is_empty(), "no committed results");
    dirs
}

fn read(path: &Path) -> String {
    std::fs::read_to_string(path)
        .unwrap_or_else(|e| panic!("{}: {e}", path.display()))
        .replace("\r\n", "\n")
}

#[test]
fn every_results_table_line_is_in_a_committed_summary() {
    let summaries: Vec<String> = result_dirs()
        .iter()
        .map(|dir| read(&dir.join("summary.md")))
        .collect();
    let doc = doc();
    let rows: Vec<&str> = section(&doc, "Results")
        .into_iter()
        .filter(|line| line.starts_with('|'))
        .collect();
    assert!(!rows.is_empty(), "the Results section has no table");
    for row in rows {
        assert!(
            summaries.iter().any(|s| s.lines().any(|line| line == row)),
            "not in any summary.md: {row}"
        );
    }
}

#[test]
fn every_committed_summary_is_its_raw_files_summarized() {
    for dir in result_dirs() {
        assert_eq!(
            summarize(&dir).replace("\r\n", "\n"),
            read(&dir.join("summary.md")),
            "{}",
            dir.display()
        );
    }
}

#[test]
fn every_requested_case_has_a_result_or_a_failure() {
    for dir in result_dirs() {
        let mut answered = BTreeSet::new();
        for file in ["runs.tsv", "quality.tsv", "failures.tsv"] {
            for row in read_tsv(&dir.join(file)) {
                answered.insert(row["run_id"].clone());
            }
        }
        let cases = read_tsv(&dir.join("cases.tsv"));
        assert!(!cases.is_empty(), "{}: no cases.tsv", dir.display());
        for case in cases {
            assert!(
                answered.contains(&case["run_id"]),
                "{}: {} has no row",
                dir.display(),
                case["run_id"]
            );
        }
    }
}

#[test]
fn every_feature_cell_has_a_source() {
    let doc = doc();
    let rows: Vec<&str> = section(&doc, "Features")
        .into_iter()
        .filter(|line| line.starts_with('|') && !line.starts_with("|---"))
        .skip(1)
        .collect();
    assert!(rows.len() >= 10, "the features table is missing");
    for row in rows {
        let cells: Vec<&str> = row.trim_matches('|').split('|').map(str::trim).collect();
        for cell in &cells[1..] {
            assert!(
                cell.contains("](http") || cell.contains("](") || cell.contains("measured here"),
                "no source in cell {cell:?} of {row}"
            );
        }
    }
}

#[test]
fn the_real_game_numbers_are_the_owners() {
    let owner = read(&manifest_dir().join("results/owner-voidrun-2026-10-04.md"));
    let doc = doc();
    let numbers: Vec<String> = section(&doc, "In a real game")
        .into_iter()
        .filter(|line| line.starts_with('|'))
        .flat_map(|line| {
            line.split(|c: char| !(c.is_ascii_digit() || c == '.'))
                .filter(|token| token.chars().any(|c| c.is_ascii_digit()))
                .map(str::to_owned)
                .collect::<Vec<_>>()
        })
        .collect();
    assert!(!numbers.is_empty());
    for number in numbers {
        assert!(
            owner.contains(&number),
            "{number} is not in the owner's table"
        );
    }
}
