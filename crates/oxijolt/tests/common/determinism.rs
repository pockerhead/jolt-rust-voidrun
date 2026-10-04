//! A two-process digest harness for determinism gates.
//!
//! A gate runs one scenario several times, each in its own child process of the same test
//! binary, and compares what the runs recorded tick by tick. Separate processes share no Jolt
//! or allocator state, so equal digests show that the simulation depends only on the calls
//! made, not on the worker thread count or on anything left over from an earlier run. On a
//! mismatch the harness names the first tick, section and byte that differ.
//!
//! A test file using it holds one ignored child test, which calls [`child_request`], runs the
//! requested scenario into a [`Digest`] and hands it to [`finish_child`]. Its gate tests call
//! [`digest_in_child`] for each run and compare the results with [`assert_same`] or
//! [`first_divergence`].
//!
//! The job system is a process-wide choice of the child: [`digest_in_child_with_jobs`] sets
//! [`JOBS_ENV`], and the world constructors of `common` read it through
//! [`with_threads`](super::jobs::with_threads).

use std::fmt;
use std::path::PathBuf;
use std::process::{self, Command};
use std::sync::atomic::{AtomicUsize, Ordering};

use super::jobs::{JobChoice, JOBS_ENV};

/// Tells a child which run to do: exactly three comma-separated fields,
/// `scenario,threads,variant`.
pub const CHILD_ENV: &str = "OXIJOLT_DIGEST_CHILD";
/// Where a child writes its encoded digest. Never parsed, so it may hold commas.
pub const OUTPUT_ENV: &str = "OXIJOLT_DIGEST_OUTPUT";

/// What one tick recorded: the static shape description and the dynamic state.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct TickDigest {
    pub shape: Vec<u8>,
    pub state: Vec<u8>,
}

/// The records of one run, one per tick.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Digest {
    pub ticks: Vec<TickDigest>,
}

impl Digest {
    pub fn new() -> Self {
        Self::default()
    }

    /// Starts the record of the next tick.
    pub fn push(&mut self) -> &mut TickDigest {
        self.ticks.push(TickDigest::default());
        self.ticks.last_mut().unwrap()
    }

    /// Little endian: the tick count, then per tick the shape length and bytes and the state
    /// length and bytes, every count and length a `u32`.
    pub fn encode(&self) -> Vec<u8> {
        let mut bytes = Vec::new();
        let put_len = |bytes: &mut Vec<u8>, len: usize| {
            let len = u32::try_from(len).expect("digest section longer than u32::MAX");
            bytes.extend_from_slice(&len.to_le_bytes());
        };
        put_len(&mut bytes, self.ticks.len());
        for tick in &self.ticks {
            for section in [&tick.shape, &tick.state] {
                put_len(&mut bytes, section.len());
                bytes.extend_from_slice(section);
            }
        }
        bytes
    }

    /// The inverse of [`encode`](Self::encode). Rejects truncated input and trailing bytes.
    pub fn decode(bytes: &[u8]) -> Result<Digest, String> {
        let mut reader = Reader { bytes, offset: 0 };
        let count = reader.len()?;
        let mut digest = Digest::new();
        for _ in 0..count {
            let shape = reader.section()?;
            let state = reader.section()?;
            digest.ticks.push(TickDigest { shape, state });
        }
        if reader.offset != bytes.len() {
            return Err(format!(
                "{} trailing bytes at offset {}",
                bytes.len() - reader.offset,
                reader.offset
            ));
        }
        Ok(digest)
    }
}

/// Reads an encoded digest front to back.
struct Reader<'a> {
    bytes: &'a [u8],
    offset: usize,
}

impl Reader<'_> {
    fn take(&mut self, count: usize, what: &str) -> Result<&[u8], String> {
        let end = self
            .offset
            .checked_add(count)
            .filter(|&end| end <= self.bytes.len());
        let Some(end) = end else {
            return Err(format!("truncated {what} at offset {}", self.offset));
        };
        let taken = &self.bytes[self.offset..end];
        self.offset = end;
        Ok(taken)
    }

    fn len(&mut self) -> Result<usize, String> {
        let bytes = self.take(4, "length")?;
        Ok(u32::from_le_bytes(bytes.try_into().unwrap()) as usize)
    }

    fn section(&mut self) -> Result<Vec<u8>, String> {
        let len = self.len()?;
        Ok(self.take(len, "section")?.to_vec())
    }
}

/// A part of a tick record.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Section {
    Shape,
    State,
}

/// Where two digests first differ.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Divergence {
    /// Every common tick is equal, but one run recorded more ticks.
    TickCount { a: usize, b: usize },
    /// The first differing byte, or the shorter length when one section is a prefix of the
    /// other.
    Tick {
        tick: usize,
        section: Section,
        byte: usize,
    },
}

impl fmt::Display for Divergence {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::TickCount { a, b } => write!(f, "tick counts {a} and {b}"),
            Self::Tick {
                tick,
                section,
                byte,
            } => {
                let section = match section {
                    Section::Shape => "shape",
                    Section::State => "state",
                };
                write!(f, "tick {tick}, {section} byte {byte}")
            }
        }
    }
}

/// The first byte at which `a` and `b` differ, a length difference counting at the shorter
/// length.
fn first_different_byte(a: &[u8], b: &[u8]) -> Option<usize> {
    a.iter()
        .zip(b)
        .position(|(x, y)| x != y)
        .or_else(|| (a.len() != b.len()).then(|| a.len().min(b.len())))
}

/// Where `a` and `b` first differ, scanning ticks in order and the shape section of each tick
/// before its state section; `None` when they are equal.
pub fn first_divergence(a: &Digest, b: &Digest) -> Option<Divergence> {
    for (tick, (x, y)) in a.ticks.iter().zip(&b.ticks).enumerate() {
        for (section, x, y) in [
            (Section::Shape, &x.shape, &y.shape),
            (Section::State, &x.state, &y.state),
        ] {
            if let Some(byte) = first_different_byte(x, y) {
                return Some(Divergence::Tick {
                    tick,
                    section,
                    byte,
                });
            }
        }
    }
    (a.ticks.len() != b.ticks.len()).then_some(Divergence::TickCount {
        a: a.ticks.len(),
        b: b.ticks.len(),
    })
}

/// Panics, naming the first divergence, unless `a` and `b` are equal.
pub fn assert_same(what: &str, a: &Digest, b: &Digest) {
    if let Some(divergence) = first_divergence(a, b) {
        panic!("{what}: digests diverge at {divergence}");
    }
}

/// The run this process is asked to do as a child, `(scenario, threads, variant)`; `None` when
/// it is not a child. Panics on a request that is not Unicode or not well formed.
pub fn child_request() -> Option<(String, u32, String)> {
    match std::env::var(CHILD_ENV) {
        Ok(request) => Some(parse_child_request(&request)),
        Err(std::env::VarError::NotPresent) => None,
        Err(std::env::VarError::NotUnicode(request)) => {
            panic!("{CHILD_ENV}={request:?} is not Unicode")
        }
    }
}

/// Splits a [`CHILD_ENV`] value into `(scenario, threads, variant)`; panics, naming the
/// request, when a field is missing or the thread count is not a number.
pub fn parse_child_request(request: &str) -> (String, u32, String) {
    let mut fields = request.splitn(3, ',');
    let mut field = |name| {
        fields
            .next()
            .unwrap_or_else(|| panic!("{CHILD_ENV}={request:?} has no {name} field"))
            .to_owned()
    };
    let scenario = field("scenario");
    let threads = field("threads");
    let variant = field("variant");
    let threads = threads
        .parse()
        .unwrap_or_else(|_| panic!("{CHILD_ENV}={request:?}: bad thread count"));
    (scenario, threads, variant)
}

/// Writes a child's digest where the parent reads it.
pub fn finish_child(digest: &Digest) {
    let output = std::env::var(OUTPUT_ENV).unwrap_or_else(|_| panic!("{OUTPUT_ENV} is not set"));
    std::fs::write(output, digest.encode()).unwrap();
}

/// A file that is removed when this guard drops, also when a test panics.
struct TempFile(PathBuf);

impl Drop for TempFile {
    fn drop(&mut self) {
        let _ = std::fs::remove_file(&self.0);
    }
}

/// How many bytes of a failed child's stdout and stderr a panic message quotes, from the end.
const OUTPUT_TAIL: usize = 4000;

/// Runs `scenario` with `threads` worker threads of Jolt's thread pool and `variant` in a child
/// process that executes the ignored test `child_test` of this binary, and returns the digest
/// it recorded.
pub fn digest_in_child(child_test: &str, scenario: &str, threads: u32, variant: &str) -> Digest {
    digest_in_child_with_jobs(child_test, scenario, threads, variant, JobChoice::Native)
}

/// As [`digest_in_child`], with the child's worlds on the job system `jobs`.
pub fn digest_in_child_with_jobs(
    child_test: &str,
    scenario: &str,
    threads: u32,
    variant: &str,
    jobs: JobChoice,
) -> Digest {
    static COUNTER: AtomicUsize = AtomicUsize::new(0);
    let file = TempFile(std::env::temp_dir().join(format!(
        "oxijolt-digest-{}-{}.bin",
        process::id(),
        COUNTER.fetch_add(1, Ordering::Relaxed)
    )));
    let request = format!("{scenario},{threads},{variant}");
    let mut command = Command::new(std::env::current_exe().unwrap());
    command
        .args([child_test, "--exact", "--ignored", "--test-threads=1"])
        .env(CHILD_ENV, &request)
        .env(OUTPUT_ENV, &file.0);
    match jobs.as_env() {
        Some(value) => command.env(JOBS_ENV, value),
        None => command.env_remove(JOBS_ENV),
    };
    let request = format!("{request} on {jobs:?} jobs");
    let output = command.output().unwrap();
    if !output.status.success() {
        // libtest reports a failed test's panic on stdout, the process's own errors on stderr.
        let tail = |bytes: &[u8]| {
            String::from_utf8_lossy(&bytes[bytes.len().saturating_sub(OUTPUT_TAIL)..]).into_owned()
        };
        panic!(
            "child {request} failed: {}\nstdout:\n{}\nstderr:\n{}",
            output.status,
            tail(&output.stdout),
            tail(&output.stderr)
        );
    }
    let bytes = std::fs::read(&file.0)
        .unwrap_or_else(|error| panic!("child {request} wrote no digest: {error}"));
    Digest::decode(&bytes).unwrap_or_else(|error| panic!("child {request}: {error}"))
}

/// Runs `scenario` of the ignored child test `child_test` with Jolt's thread pool of 1 worker,
/// a Rayon pool of 4 threads (concurrency 5) and an inline job system (concurrency 3), each in
/// its own child, and asserts that both caller job systems record what the thread pool records.
pub fn assert_caller_job_systems_agree(child_test: &str, scenario: &str, variant: &str) {
    let run =
        |threads, jobs| digest_in_child_with_jobs(child_test, scenario, threads, variant, jobs);
    let native = run(1, JobChoice::Native);
    let rayon = run(4, JobChoice::Rayon);
    let inline = run(1, JobChoice::Inline);
    assert!(!native.ticks.is_empty());
    assert_same(
        &format!("{scenario} {variant}, 1 worker vs Rayon 4 threads"),
        &native,
        &rayon,
    );
    assert_same(
        &format!("{scenario} {variant}, 1 worker vs inline"),
        &native,
        &inline,
    );
}
