//! `cargo xtask bindings` refuses to run while bindgen or libclang would read clang arguments,
//! a target or include paths from the environment, before it loads libclang.

use std::process::Command;

/// Runs `xtask bindings --check` with `name=value` added and returns its stderr after checking
/// that it failed without loading libclang.
fn refused_with(name: &str, value: &str) -> String {
    let out = std::env::temp_dir().join("xtask-environment-test");
    let output = Command::new(env!("CARGO_BIN_EXE_xtask"))
        .args(["bindings", "--check", "--out"])
        .arg(&out)
        .env(name, value)
        .output()
        .expect("xtask runs");
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr).into_owned();
    assert!(
        !output.status.success(),
        "{name} was accepted: {stdout}{stderr}"
    );
    assert!(!stdout.contains("libclang"), "{name}: libclang was loaded");
    stderr
}

fn assert_refused(name: &str, value: &str) {
    let stderr = refused_with(name, value);
    assert!(
        stderr.contains(&format!("unset {name}")) || stderr.contains(&format!(", {name}")),
        "{name} not named in: {stderr}"
    );
}

#[test]
fn target_and_clang_args_are_refused() {
    assert_refused("TARGET", "x86_64-unknown-linux-gnu");
    assert_refused("BINDGEN_EXTRA_CLANG_ARGS", "-DJPH_DOUBLE_PRECISION");
    assert_refused(
        "BINDGEN_EXTRA_CLANG_ARGS_x86_64-unknown-linux-gnu",
        "-DJPH_DOUBLE_PRECISION",
    );
    assert_refused(
        "BINDGEN_EXTRA_CLANG_ARGS_x86_64_unknown_linux_gnu",
        "-DJPH_DOUBLE_PRECISION",
    );
}

#[test]
fn clang_include_paths_are_refused() {
    let dir = std::env::temp_dir().to_string_lossy().into_owned();
    for name in [
        "CPATH",
        "C_INCLUDE_PATH",
        "CPLUS_INCLUDE_PATH",
        "OBJC_INCLUDE_PATH",
        "OBJCPLUS_INCLUDE_PATH",
    ] {
        assert_refused(name, &dir);
    }
}

/// Windows resolves environment names regardless of case, and so does bindgen's lookup there.
#[cfg(windows)]
#[test]
fn other_spellings_are_refused_on_windows() {
    assert_refused("target", "x86_64-unknown-linux-gnu");
    assert_refused("Target", "x86_64-unknown-linux-gnu");
    assert_refused("bindgen_extra_clang_args", "-DJPH_DOUBLE_PRECISION");
    assert_refused("Bindgen_Extra_Clang_Args", "-DJPH_DOUBLE_PRECISION");
    assert_refused(
        "bindgen_extra_clang_args_x86_64-pc-windows-msvc",
        "-DJPH_DOUBLE_PRECISION",
    );
    assert_refused(
        "Bindgen_Extra_Clang_Args_X86_64_Pc_Windows_Msvc",
        "-DJPH_DOUBLE_PRECISION",
    );
    assert_refused("cpath", "include");
    assert_refused("Cplus_Include_Path", "include");
}
