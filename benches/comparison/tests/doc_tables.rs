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

/// The cells of the table rows of `lines`, header and separator left out.
fn table_rows(lines: &[&str]) -> Vec<Vec<String>> {
    lines
        .iter()
        .filter(|line| line.starts_with('|') && !line.starts_with("|---"))
        .skip(1)
        .map(|line| {
            line.trim_matches('|')
                .split('|')
                .map(|cell| cell.trim().to_owned())
                .collect()
        })
        .collect()
}

#[test]
fn the_real_game_numbers_are_the_owners() {
    // Each published row: its workload, then the Rapier and the Jolt cell.
    let expected = [
        (
            "Whole walking system, 60 NPCs in the near band, mean per tick",
            "3.43 ms",
            "0.83 ms",
        ),
        (
            "One controller move in the game",
            "86-156 us",
            "16-19 us p50 / 28-43 us p99",
        ),
        (
            "One controller move, the binding alone",
            "n/a",
            "2.7 us p50 / 6 us p99",
        ),
        ("Physics world step p50/p99", "110 / 268 us", "45 / 111 us"),
        (
            "Refresh before queries when the world changed, p99",
            "3.8 ms",
            "164 us",
        ),
    ];
    let doc = doc();
    let rows = table_rows(&section(&doc, "In a real game"));
    let published: Vec<(&str, &str, &str)> = rows
        .iter()
        .map(|r| (r[0].as_str(), r[1].as_str(), r[2].as_str()))
        .collect();
    assert_eq!(published, expected);

    // Every number is the owner's, in the owner's column for that engine, and each owner value
    // is published once.
    let owner = read(&manifest_dir().join("results/owner-voidrun-2026-10-04.md"));
    let owner_rows = table_rows(&owner.lines().collect::<Vec<_>>());
    for column in [1, 2] {
        let owners: Vec<&str> = owner_rows.iter().map(|r| r[column].as_str()).collect();
        let ours: Vec<&str> = rows.iter().map(|r| r[column].as_str()).collect();
        for value in &owners {
            let count = ours.iter().filter(|v| *v == value).count();
            assert_eq!(count, 1, "owner's {value:?} published {count} times");
        }
        for value in &ours {
            assert!(owners.contains(value), "{value:?} is not the owner's");
        }
    }
}

#[test]
fn the_real_game_rows_name_their_machines() {
    let doc = doc();
    let lines = section(&doc, "In a real game");
    let owner = read(&manifest_dir().join("results/owner-voidrun-2026-10-04.md"));
    assert!(owner.contains("weak 4-core machine"));
    assert!(lines.join("\n").contains("weak 4-core machine"));
    let benchmarks = read(&manifest_dir().join("../../docs/benchmarks.md"));
    assert!(benchmarks.contains("| CPU | 11th Gen Intel Core i9-11900K"));
    let binding = table_rows(&lines)
        .into_iter()
        .find(|r| r[0] == "One controller move, the binding alone")
        .unwrap();
    assert!(binding[3].contains("i9-11900K") && binding[3].contains("Windows 11"));
}

/// The `version` of the package `name` in the workspace's `Cargo.lock`.
fn locked_version(name: &str) -> String {
    let lock = read(&manifest_dir().join("../../Cargo.lock"));
    let entry = format!("[[package]]\nname = \"{name}\"\nversion = \"");
    let start = lock
        .find(&entry)
        .unwrap_or_else(|| panic!("{name} not locked"))
        + entry.len();
    lock[start..].split('"').next().unwrap().to_owned()
}

#[test]
fn the_reproduction_toolchain_is_the_manifests() {
    let manifest = read(&manifest_dir().join("Cargo.toml"));
    let rust = manifest
        .lines()
        .find_map(|line| line.strip_prefix("rust-version = \""))
        .and_then(|rest| rest.strip_suffix('"'))
        .expect("the comparison declares its rust-version");
    let doc = doc();
    let reproduce = section(&doc, "Reproduce").join(" ");
    assert!(
        reproduce.contains(&format!("Rust {rust} or newer")),
        "Reproduce does not ask for Rust {rust}"
    );
    let bevy = locked_version("bevy");
    assert!(reproduce.contains(&format!("Bevy {bevy}")), "Bevy {bevy}");
    let features = section(&doc, "Features").join("\n");
    assert!(features.contains(&format!(
        "its Bevy {bevy} (the version this comparison's lockfile resolves) needs {rust}"
    )));
    assert!(features.contains(&format!("bevyengine/bevy/blob/v{bevy}/Cargo.toml")));
}
