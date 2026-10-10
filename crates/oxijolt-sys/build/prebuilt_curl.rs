//! The download of a release archive with the system `curl`.
//!
//! The archive's sha256 comes from the list packaged with the crate, so the transport adds no
//! trust: curl is used for its proxy and certificate handling (the system trust store, the
//! `https_proxy`/`ALL_PROXY`/`NO_PROXY` variables) and keeps a TLS stack out of the build
//! dependencies. Shared by `build.rs` (feature `prebuilt`) and the `xtask` tests through
//! `#[path]`.

use std::ffi::OsStr;
use std::io::{self, Read};
use std::process::{Command, Stdio};
use std::thread;

use crate::prebuilt::Unavailable;

/// Bounds of one download.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Limits {
    /// Seconds to connect (`--connect-timeout`).
    pub connect_s: u32,
    /// Seconds for the whole transfer (`--max-time`).
    pub total_s: u32,
    /// Largest accepted body in bytes.
    pub max_bytes: u64,
}

impl Limits {
    /// 10 s to connect, 120 s in all, 64 MiB; the release archives are about 3 MB.
    pub const DEFAULT: Limits = Limits {
        connect_s: 10,
        total_s: 120,
        max_bytes: 64 << 20,
    };
}

/// The user agent of every request: the crate and version that compiled this module.
const USER_AGENT: &str = concat!(
    env!("CARGO_PKG_NAME"),
    "/",
    env!("CARGO_PKG_VERSION"),
    " build script"
);

/// Most bytes of curl's error output that are kept.
const STDERR_CAP: u64 = 4096;
/// Longest error line in a message.
const MESSAGE_CAP: usize = 200;
/// curl's exit code for `--max-filesize` exceeded.
const CURL_FILESIZE_EXCEEDED: i32 = 63;

/// The body of `url`, fetched by `program` (normally `curl`). Only `https` is allowed, for the
/// request and its redirects, unless `https_only` is false (a `JOLTC_PREBUILT_URL` override,
/// which may be a local `http` mirror).
///
/// A missing program is [`Unavailable::NoCurl`], a body over `limits.max_bytes` is
/// [`Unavailable::TooLarge`], and any other failure is [`Unavailable::Download`] with curl's
/// exit code and first error line.
pub fn download(
    program: &OsStr,
    url: &str,
    https_only: bool,
    limits: &Limits,
) -> Result<Vec<u8>, Unavailable> {
    let protocols = if https_only { "=https" } else { "=http,https" };
    let mut command = Command::new(program);
    // `-q` must come first: it stops curl from reading a `.curlrc`.
    command
        .arg("-q")
        .args([
            "--silent",
            "--show-error",
            "--fail",
            "--globoff",
            "--location",
        ])
        .args(["--max-redirs", "3"])
        .args(["--proto", protocols, "--proto-redir", protocols])
        .args(["--connect-timeout", &limits.connect_s.to_string()])
        .args(["--max-time", &limits.total_s.to_string()])
        .args(["--max-filesize", &limits.max_bytes.to_string()])
        .args(["--user-agent", USER_AGENT])
        .args(["--url", url])
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
    let mut child = command.spawn().map_err(|e| match e.kind() {
        io::ErrorKind::NotFound => Unavailable::NoCurl,
        _ => Unavailable::Download(format!("cannot start curl: {e}")),
    })?;

    let stderr = child.stderr.take().expect("stderr is piped");
    let errors = thread::spawn(move || read_capped_stderr(stderr));
    let stdout = child.stdout.take().expect("stdout is piped");
    let body = read_capped(stdout, limits.max_bytes);
    if !matches!(body, Ok(Some(_))) {
        // Over the cap or a broken pipe: curl is stopped, it may still be writing.
        let _ = child.kill();
    }
    let status = child.wait();
    let errors = errors.join().unwrap_or_default();

    let body = match body {
        Ok(Some(body)) => body,
        Ok(None) => return Err(Unavailable::TooLarge),
        Err(e) => {
            return Err(Unavailable::Download(format!(
                "cannot read curl output: {e}"
            )))
        }
    };
    let status = status.map_err(|e| Unavailable::Download(format!("curl did not finish: {e}")))?;
    match status.code() {
        Some(0) => Ok(body),
        Some(CURL_FILESIZE_EXCEEDED) => Err(Unavailable::TooLarge),
        code => {
            let code = code.map_or_else(|| "signal".to_owned(), |c| c.to_string());
            Err(Unavailable::Download(format!(
                "curl exit {code}: {}",
                first_line(&errors)
            )))
        }
    }
}

/// Everything `reader` gives, or `None` once it gives more than `cap` bytes.
fn read_capped(reader: impl Read, cap: u64) -> io::Result<Option<Vec<u8>>> {
    let mut body = Vec::new();
    reader.take(cap + 1).read_to_end(&mut body)?;
    Ok((body.len() as u64 <= cap).then_some(body))
}

/// The first [`STDERR_CAP`] bytes of curl's error output; the rest is drained so that curl
/// never blocks on a full pipe.
fn read_capped_stderr(mut stderr: impl Read) -> String {
    let mut kept = Vec::new();
    let _ = (&mut stderr).take(STDERR_CAP).read_to_end(&mut kept);
    let _ = io::copy(&mut stderr, &mut io::sink());
    String::from_utf8_lossy(&kept).into_owned()
}

/// The first non-empty line of `text`, at most [`MESSAGE_CAP`] characters, with the values of
/// the proxy variables curl reads replaced: they may hold credentials.
fn first_line(text: &str) -> String {
    let line = text
        .lines()
        .map(str::trim)
        .find(|line| !line.is_empty())
        .unwrap_or("no error output");
    let mut line: String = line.chars().take(MESSAGE_CAP).collect();
    for name in [
        "https_proxy",
        "HTTPS_PROXY",
        "http_proxy",
        "HTTP_PROXY",
        "all_proxy",
        "ALL_PROXY",
    ] {
        if let Some(value) = std::env::var(name).ok().filter(|v| !v.is_empty()) {
            line = line.replace(&value, "<proxy>");
        }
    }
    line
}

/// The base URL of the release assets and whether only `https` is allowed: `env_override`
/// (`JOLTC_PREBUILT_URL`) when set and not empty, which may be `http`, otherwise the list's
/// URL, `https` only. A trailing `/` is dropped.
///
/// An override must start with `http://` or `https://` and hold no `@`, `?`, `#` or
/// whitespace; otherwise it is [`Unavailable::Download`].
pub fn base_url(list_url: &str, env_override: Option<&str>) -> Result<(String, bool), Unavailable> {
    match env_override.filter(|url| !url.is_empty()) {
        Some(url) => {
            let scheme = url.starts_with("http://") || url.starts_with("https://");
            let plain = !url.contains(|c: char| matches!(c, '@' | '?' | '#') || c.is_whitespace());
            if !(scheme && plain) {
                return Err(Unavailable::Download(
                    "invalid JOLTC_PREBUILT_URL".to_owned(),
                ));
            }
            Ok((url.trim_end_matches('/').to_owned(), false))
        }
        None => Ok((list_url.trim_end_matches('/').to_owned(), true)),
    }
}

/// The URL of the archive `name` under `base`.
pub fn archive_url(base: &str, name: &str) -> String {
    format!("{base}/{name}.tar.gz")
}
