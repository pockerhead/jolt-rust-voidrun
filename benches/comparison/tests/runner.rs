//! The matrix runner and the summary: raw files in, the same summary out, failures kept.

#![cfg(feature = "jolt")]

use std::path::{Path, PathBuf};

use comparison::cli::Options;
use comparison::engines::Variant;
use comparison::matrix;
use comparison::summary::{read_tsv, summarize};

/// A fresh directory under the system's temp directory.
fn temp_dir(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("comparison-{name}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

fn options(args: &[&str]) -> Options {
    let args: Vec<String> = args.iter().map(|a| (*a).to_owned()).collect();
    Options::parse(&args).unwrap()
}

fn fixture_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/summary")
}

#[test]
fn tsv_rows_read_back_by_column_name() {
    let dir = temp_dir("tsv");
    let path = dir.join("rows.tsv");
    std::fs::write(&path, "a\tb\r\n1\tx\r\n2\ty\n").unwrap();
    let rows = read_tsv(&path);
    assert_eq!(rows.len(), 2);
    assert_eq!(rows[0]["a"], "1");
    assert_eq!(rows[1]["b"], "y");
    assert!(read_tsv(&dir.join("missing.tsv")).is_empty());
}

#[test]
fn the_summary_of_the_fixture_files_is_the_checked_in_one() {
    let expected = std::fs::read_to_string(fixture_dir().join("summary.md")).unwrap();
    assert_eq!(
        summarize(&fixture_dir()).replace("\r\n", "\n"),
        expected.replace("\r\n", "\n")
    );
}

#[test]
fn a_validation_matrix_writes_every_file_and_passes_jolts_gate() {
    let out = temp_dir("validate");
    let exe = format!("jolt={}", env!("CARGO_BIN_EXE_comparison"));
    let result = matrix::all(&options(&[
        "--mode",
        "validate",
        "--scenes",
        "fixture_spherical,fixture_revolute",
        "--variants",
        "jolt",
        "--threads",
        "1,4",
        "--steps",
        "20",
        "--require-identical",
        "jolt",
        "--exe",
        &exe,
        "--out",
        out.to_str().unwrap(),
    ]));
    assert!(result.is_ok(), "{result:?}");
    assert_eq!(read_tsv(&out.join("quality.tsv")).len(), 4);
    let gates = read_tsv(&out.join("determinism.tsv"));
    assert_eq!(gates.len(), 2);
    assert!(
        gates.iter().all(|g| g["verdict"] == "identical"),
        "{gates:?}"
    );
    assert!(out.join("digests").read_dir().unwrap().count() == 4);
    assert!(std::fs::read_to_string(out.join("summary.md"))
        .unwrap()
        .contains("## Determinism across thread counts"));
}

#[test]
fn a_failing_case_fails_the_matrix_and_keeps_the_other_rows() {
    let out = temp_dir("failing");
    let failing = Variant::ALL
        .into_iter()
        .find(|v| v.check_build().is_err())
        .expect("a variant this build cannot run");
    let exe = env!("CARGO_BIN_EXE_comparison");
    let exes = format!("jolt={exe},{}={exe}", failing.name);
    let variants = format!("jolt,{}", failing.name);
    let result = matrix::all(&options(&[
        "--mode",
        "time",
        "--scenes",
        "fixture_stack",
        "--variants",
        &variants,
        "--steps",
        "2",
        "--exe",
        &exes,
        "--out",
        out.to_str().unwrap(),
    ]));
    let error = result.unwrap_err();
    assert!(error.contains("1 of 2 cases failed"), "{error}");
    let runs = read_tsv(&out.join("runs.tsv"));
    assert_eq!(runs.len(), 1);
    assert_eq!(runs[0]["variant"], "jolt");
    let failures = read_tsv(&out.join("failures.tsv"));
    assert_eq!(failures.len(), 1);
    assert_eq!(failures[0]["variant"], failing.name);
    assert_eq!(read_tsv(&out.join("cases.tsv")).len(), 2);
    assert!(out.join("summary.md").exists());
}
