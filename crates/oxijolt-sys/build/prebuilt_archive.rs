//! The release archive on disk: the fingerprint of the native sources it was built from, its
//! sha256, its checked unpacking and cache, and the probes of the toolchain that will link it.
//!
//! Shared by `build.rs` (feature `prebuilt`) and the `xtask` list writer through `#[path]`.

use std::fs;
use std::io;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::{Duration, UNIX_EPOCH};

use flate2::read::GzDecoder;
use sha2::{Digest, Sha256};

use crate::prebuilt::{hex, Archive, Unavailable, Version2, SOURCE_INPUTS};
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

/// Fails with [`Unavailable::Checksum`] unless `bytes` hash to `sha256`.
pub fn verify(bytes: &[u8], sha256_expected: &[u8; 32]) -> Result<(), Unavailable> {
    if sha256(bytes) == *sha256_expected {
        Ok(())
    } else {
        Err(Unavailable::Checksum)
    }
}

/// [`verify`], then [`unpack`]: nothing of a download is decompressed before its hash matches
/// the list.
pub fn verify_and_unpack(
    bytes: &[u8],
    archive: &Archive,
    libraries: [&str; 2],
    out_dir: &Path,
) -> Result<PathBuf, Unavailable> {
    verify(bytes, &archive.sha256)?;
    unpack(bytes, archive, libraries, out_dir)
}

/// Most decoded bytes an archive may hold, counted on the entry sizes.
pub const MAX_DECODED_BYTES: u64 = 256 << 20;

/// One entry of an archive that passed [`check_entries`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Entry {
    /// Path in the archive without a trailing `/`, such as `<name>/lib/joltc.lib`.
    pub path: String,
    pub directory: bool,
}

/// The directories and files a release archive named `name` may contain: the prefix layout
/// with the two `libraries` in the target's spelling (`joltc.lib` and `Jolt.lib`, or
/// `libjoltc_double.a` and `libJolt.a`), the manifest, the provenance and the licences.
pub fn allowed_entries(name: &str, libraries: [&str; 2]) -> Vec<Entry> {
    let directories = ["", "lib", "include", "licenses"];
    let files = [
        format!("lib/{}", libraries[0]),
        format!("lib/{}", libraries[1]),
        "include/joltc.h".to_owned(),
        "include/joltc_ext.h".to_owned(),
        "oxijolt-sys-manifest.txt".to_owned(),
        "PROVENANCE.txt".to_owned(),
        "LICENSE-MIT".to_owned(),
        "LICENSE-APACHE".to_owned(),
        "licenses/joltc-LICENSE".to_owned(),
        "licenses/JoltPhysics-LICENSE".to_owned(),
    ];
    let path = |relative: &str| {
        if relative.is_empty() {
            name.to_owned()
        } else {
            format!("{name}/{relative}")
        }
    };
    let directories = directories.iter().map(|d| Entry {
        path: path(d),
        directory: true,
    });
    let files = files.iter().map(|f| Entry {
        path: path(f),
        directory: false,
    });
    directories.chain(files).collect()
}

/// Reads the `.tar.gz` in `bytes` without writing anything and returns its entries in order,
/// or refuses the whole archive: a path outside [`allowed_entries`] (which also rules out `..`,
/// absolute, drive and backslash paths), an entry type other than a regular file or a
/// directory, a path that appears twice (so there are at most 14 entries), more than
/// [`MAX_DECODED_BYTES`], or a damaged stream.
pub fn check_entries(
    bytes: &[u8],
    name: &str,
    libraries: [&str; 2],
) -> Result<Vec<Entry>, Unavailable> {
    let allowed = allowed_entries(name, libraries);
    let mut entries: Vec<Entry> = Vec::new();
    let mut decoded = 0u64;
    let mut archive = tar::Archive::new(GzDecoder::new(bytes));
    for entry in archive.entries().map_err(damaged)? {
        let entry = entry.map_err(damaged)?;
        let entry_path = String::from_utf8(entry.path_bytes().into_owned())
            .map_err(|_| refused("a path is not UTF-8".to_owned()))?;
        let directory = match entry.header().entry_type() {
            tar::EntryType::Regular => false,
            tar::EntryType::Directory => true,
            other => return Err(refused(format!("{entry_path} is a {other:?} entry"))),
        };
        let path = if directory {
            entry_path.strip_suffix('/').unwrap_or(&entry_path)
        } else {
            &entry_path
        };
        let found = Entry {
            path: path.to_owned(),
            directory,
        };
        if !allowed.contains(&found) {
            return Err(refused(format!(
                "{entry_path} is not a file of the archive"
            )));
        }
        if entries.contains(&found) {
            return Err(refused(format!("{entry_path} appears twice")));
        }
        let size = entry.size();
        if directory && size != 0 {
            return Err(refused(format!("{entry_path} is a directory with data")));
        }
        decoded = decoded.saturating_add(size);
        if decoded > MAX_DECODED_BYTES {
            return Err(refused(format!("more than {MAX_DECODED_BYTES} bytes")));
        }
        entries.push(found);
    }
    Ok(entries)
}

fn damaged(error: io::Error) -> Unavailable {
    Unavailable::Unpack(format!("damaged archive: {error}"))
}

fn refused(reason: String) -> Unavailable {
    Unavailable::Unpack(reason)
}

/// The directory that holds the unpacked archives: `out_dir/prebuilt`.
fn cache_dir(out_dir: &Path) -> PathBuf {
    out_dir.join("prebuilt")
}

/// The record of an unpacked archive, written last: the archive's sha256 and the sha256 of
/// each file.
fn marker_path(out_dir: &Path, name: &str) -> PathBuf {
    cache_dir(out_dir).join(format!("{name}.files"))
}

/// Unpacks a verified archive into `out_dir/prebuilt/<name>` and returns that prefix.
///
/// The archive is checked whole by [`check_entries`] first. The files are written into a fresh
/// staging directory with `create_new`, keep the modification times of the archive (so the
/// prefix never looks newer than the build script run), and the prefix replaces any older one
/// only when every file is written. The marker that [`cached`] reads is written last.
pub fn unpack(
    bytes: &[u8],
    archive: &Archive,
    libraries: [&str; 2],
    out_dir: &Path,
) -> Result<PathBuf, Unavailable> {
    let entries = check_entries(bytes, &archive.name, libraries)?;
    let cache = cache_dir(out_dir);
    let staging = cache.join(format!(".tmp-{}", archive.name));
    let prefix = cache.join(&archive.name);
    let marker = marker_path(out_dir, &archive.name);
    let io_error = |what: &str, path: &Path, e: io::Error| {
        Unavailable::Unpack(format!("{what} {}: {e}", path.display()))
    };

    remove_cached(out_dir, &archive.name);
    remove_dir(&staging).map_err(|e| io_error("cannot remove", &staging, e))?;
    fs::create_dir_all(&staging).map_err(|e| io_error("cannot create", &staging, e))?;
    let written = write_entries(bytes, &entries, &staging);
    let written = match written {
        Ok(written) => written,
        Err(error) => {
            let _ = remove_dir(&staging);
            return Err(error);
        }
    };
    let unpacked = staging.join(&archive.name);
    fs::rename(&unpacked, &prefix).map_err(|e| io_error("cannot move", &unpacked, e))?;
    remove_dir(&staging).map_err(|e| io_error("cannot remove", &staging, e))?;

    let mut record = format!("archive {}\n", hex(&archive.sha256));
    for (hash, relative) in written {
        record += &format!("{} {relative}\n", hex(&hash));
    }
    fs::write(&marker, record).map_err(|e| io_error("cannot write", &marker, e))?;
    Ok(prefix)
}

/// Writes the files of `bytes` into `staging` in the order [`check_entries`] returned them, and
/// returns each file's sha256 with its path relative to the prefix.
fn write_entries(
    bytes: &[u8],
    entries: &[Entry],
    staging: &Path,
) -> Result<Vec<([u8; 32], String)>, Unavailable> {
    let mut written = Vec::new();
    let mut archive = tar::Archive::new(GzDecoder::new(bytes));
    let mut expected = entries.iter();
    for entry in archive.entries().map_err(damaged)? {
        let mut entry = entry.map_err(damaged)?;
        let path = String::from_utf8_lossy(&entry.path_bytes()).into_owned();
        let checked = expected
            .next()
            .filter(|checked| path.strip_suffix('/').unwrap_or(&path) == checked.path)
            .ok_or_else(|| refused(format!("{path} changed between two reads")))?;
        let target = staging.join(&checked.path);
        let failed = |e: io::Error| Unavailable::Unpack(format!("cannot write {path}: {e}"));
        if checked.directory {
            fs::create_dir_all(&target).map_err(failed)?;
            continue;
        }
        if let Some(parent) = target.parent() {
            fs::create_dir_all(parent).map_err(failed)?;
        }
        let mut file = fs::File::create_new(&target).map_err(failed)?;
        let mut hashing = HashingWriter {
            file: &mut file,
            hasher: Sha256::new(),
        };
        io::copy(&mut entry, &mut hashing).map_err(failed)?;
        let hash = hashing.hasher.finalize().into();
        let mtime = entry.header().mtime().map_err(damaged)?;
        file.set_modified(UNIX_EPOCH + Duration::from_secs(mtime))
            .map_err(failed)?;
        let relative = checked
            .path
            .split_once('/')
            .map_or("", |(_, rest)| rest)
            .to_owned();
        written.push((hash, relative));
    }
    if expected.next().is_some() {
        return Err(refused("the archive changed between two reads".to_owned()));
    }
    Ok(written)
}

/// A writer into a file that hashes what it writes.
struct HashingWriter<'a> {
    file: &'a mut fs::File,
    hasher: Sha256,
}

impl io::Write for HashingWriter<'_> {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        let n = io::Write::write(self.file, buf)?;
        self.hasher.update(&buf[..n]);
        Ok(n)
    }

    fn flush(&mut self) -> io::Result<()> {
        io::Write::flush(self.file)
    }
}

/// The prefix of `archive` unpacked earlier into `out_dir`, when its marker names this
/// archive's sha256 and every recorded file still has its recorded sha256. Otherwise the
/// cached prefix and its marker are removed and the result is `None`, so a damaged or edited
/// cache is fetched again.
pub fn cached(out_dir: &Path, archive: &Archive) -> Option<PathBuf> {
    let prefix = cache_dir(out_dir).join(&archive.name);
    if cache_is_intact(out_dir, archive, &prefix) {
        Some(prefix)
    } else {
        remove_cached(out_dir, &archive.name);
        None
    }
}

fn cache_is_intact(out_dir: &Path, archive: &Archive, prefix: &Path) -> bool {
    let Ok(record) = fs::read_to_string(marker_path(out_dir, &archive.name)) else {
        return false;
    };
    let mut lines = record.lines();
    if lines.next() != Some(&format!("archive {}", hex(&archive.sha256))) {
        return false;
    }
    let mut files = 0;
    for line in lines {
        let Some((hash, relative)) = line.split_once(' ') else {
            return false;
        };
        if relative
            .split('/')
            .any(|part| part.is_empty() || part == "..")
        {
            return false;
        }
        match fs::read(prefix.join(relative)) {
            Ok(bytes) if hex(&sha256(&bytes)) == hash => files += 1,
            _ => return false,
        }
    }
    files > 0
}

/// Removes the unpacked prefix of the archive `name` and its marker, the marker first.
pub fn remove_cached(out_dir: &Path, name: &str) {
    let _ = fs::remove_file(marker_path(out_dir, name));
    let _ = remove_dir(&cache_dir(out_dir).join(name));
}

/// Removes `path` and everything under it; a missing directory is not an error.
fn remove_dir(path: &Path) -> io::Result<()> {
    match fs::remove_dir_all(path) {
        Err(e) if e.kind() != io::ErrorKind::NotFound => Err(e),
        _ => Ok(()),
    }
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
