//! The release archive on disk: the fingerprint of the native sources it was built from, its
//! sha256, and the probes of the toolchain that will link it.
//!
//! Shared by `build.rs` (feature `prebuilt`) and the `xtask` list writer through `#[path]`.

use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

use sha2::{Digest, Sha256};

use crate::prebuilt::{Unavailable, Version2, SOURCE_INPUTS};
use crate::targets::normalize_crlf;

/// The sha256 of `bytes`.
pub fn sha256(bytes: &[u8]) -> [u8; 32] {
    Sha256::digest(bytes).into()
}

/// Every regular file under [`SOURCE_INPUTS`] in `crate_dir`, sorted by crate-relative path
/// with `/` separators. A missing input or a symbolic link is
/// [`Unavailable::SourcesMissing`].
pub fn source_files(crate_dir: &Path) -> Result<Vec<(String, PathBuf)>, Unavailable> {
    let mut files = Vec::new();
    for input in SOURCE_INPUTS {
        collect(crate_dir, input, &mut files)?;
    }
    files.sort_by(|a, b| a.0.cmp(&b.0));
    Ok(files)
}

/// Adds the file `relative`, or every file under the directory `relative`, to `files`.
fn collect(
    crate_dir: &Path,
    relative: &str,
    files: &mut Vec<(String, PathBuf)>,
) -> Result<(), Unavailable> {
    let path = crate_dir.join(relative);
    let missing = || Unavailable::SourcesMissing(relative.to_owned());
    let kind = fs::symlink_metadata(&path)
        .map_err(|_| missing())?
        .file_type();
    if kind.is_file() {
        files.push((relative.to_owned(), path));
    } else if kind.is_dir() {
        for entry in fs::read_dir(&path).map_err(|_| missing())? {
            let entry = entry.map_err(|_| missing())?;
            let name = entry.file_name();
            let name = name.to_str().ok_or_else(missing)?;
            collect(crate_dir, &format!("{relative}/{name}"), files)?;
        }
    } else {
        return Err(missing());
    }
    Ok(())
}

/// The sha256 over [`source_files`], each framed as `path \0 length \0 bytes` with CRLF line
/// endings normalized, so a checkout with `core.autocrlf` gives the release's value.
pub fn sources_fingerprint(crate_dir: &Path) -> Result<[u8; 32], Unavailable> {
    let mut hasher = Sha256::new();
    for (relative, path) in source_files(crate_dir)? {
        let bytes = fs::read(&path).map_err(|_| Unavailable::SourcesMissing(relative.clone()))?;
        let bytes = normalize_crlf(&bytes);
        hasher.update(relative.as_bytes());
        hasher.update(b"\0");
        hasher.update(bytes.len().to_string().as_bytes());
        hasher.update(b"\0");
        hasher.update(&bytes);
    }
    Ok(hasher.finalize().into())
}

/// The `link.exe` rustc would use for `target`, found the way rustc finds it.
pub fn find_link_exe(target: &str) -> Option<String> {
    cc::windows_registry::find_tool(target, "link.exe")
        .map(|tool| tool.path().to_string_lossy().into_owned())
}

/// The host's glibc from `getconf GNU_LIBC_VERSION` (`glibc 2.35`).
pub fn host_glibc() -> Option<Version2> {
    let output = Command::new("getconf")
        .arg("GNU_LIBC_VERSION")
        .output()
        .ok()?;
    if !output.status.success() {
        return None;
    }
    let text = String::from_utf8(output.stdout).ok()?;
    Version2::parse(text.trim().strip_prefix("glibc ")?)
}
