//! The bench's report: timed rows, their percentiles and limits, and the markdown table and
//! machine line it prints.

use std::time::Instant;

/// Microseconds since `start`.
pub fn micros(start: Instant) -> f64 {
    start.elapsed().as_secs_f64() * 1e6
}

/// What a row is compared with.
pub enum Limit {
    /// A limit on the 99th percentile, microseconds.
    P99AtMost(f64),
    /// A limit the game's text gives without a percentile, microseconds; compared on p99.
    AtMost(f64),
    /// A number from elsewhere, shown for comparison.
    Reference(&'static str),
    None,
}

/// One line of the table.
pub struct Row {
    case: String,
    sample: &'static str,
    sorted_us: Vec<f64>,
    limit: Limit,
    note: String,
}

impl Row {
    pub fn new(
        case: &str,
        sample: &'static str,
        mut us: Vec<f64>,
        limit: Limit,
        note: impl Into<String>,
    ) -> Self {
        us.sort_by(f64::total_cmp);
        Self {
            case: case.to_owned(),
            sample,
            sorted_us: us,
            limit,
            note: note.into(),
        }
    }

    /// The first call after the scene was built, one sample with no limit.
    pub fn cold(case: &str, sample: &'static str, us: f64) -> Self {
        Self::new(
            &format!("{case}: cold first call"),
            sample,
            vec![us],
            Limit::None,
            "",
        )
    }
}

/// The nearest-rank `p` percentile of `sorted`, which is not empty.
fn percentile(sorted: &[f64], p: f64) -> f64 {
    let n = sorted.len();
    sorted[((p * n as f64).ceil() as usize).clamp(1, n) - 1]
}

pub fn print_table(rows: &[Row]) {
    println!(
        "| case | one sample | samples | p50 us | p99 us | max us | limit / reference | status | note |"
    );
    println!("|---|---|---:|---:|---:|---:|---|---|---|");
    for row in rows {
        let sorted = &row.sorted_us;
        let p99 = percentile(sorted, 0.99);
        let (limit, status) = match row.limit {
            Limit::P99AtMost(us) => (
                format!("p99 <= {us:.0} us"),
                if p99 <= us { "within" } else { "over" },
            ),
            Limit::AtMost(us) => (
                format!("<= {us:.0} us (limit, no percentile given; p99 shown)"),
                if p99 <= us { "within" } else { "over" },
            ),
            Limit::Reference(text) => (text.to_owned(), "-"),
            Limit::None => (String::from("-"), "-"),
        };
        println!(
            "| {} | {} | {} | {:.1} | {:.1} | {:.1} | {limit} | {status} | {} |",
            row.case,
            row.sample,
            sorted.len(),
            percentile(sorted, 0.5),
            p99,
            sorted[sorted.len() - 1],
            row.note,
        );
    }
}

/// The CPU's name as the operating system reports it.
fn cpu_name() -> String {
    if let Ok(name) = std::env::var("PROCESSOR_IDENTIFIER") {
        return name;
    }
    std::fs::read_to_string("/proc/cpuinfo")
        .ok()
        .and_then(|info| {
            info.lines()
                .find(|line| line.starts_with("model name"))
                .and_then(|line| line.split(':').nth(1))
                .map(|name| name.trim().to_owned())
        })
        .unwrap_or_else(|| String::from("unknown CPU"))
}

/// Prints the machine and build the numbers come from.
pub fn print_machine() {
    let threads = std::thread::available_parallelism().map_or(0, |n| n.get());
    let profile = if cfg!(debug_assertions) {
        "debug assertions on"
    } else {
        "debug assertions off"
    };
    println!(
        "{} {}, {threads} logical threads, {}; Rust code with {profile}; Jolt and joltc built in \
         Release",
        std::env::consts::OS,
        std::env::consts::ARCH,
        cpu_name()
    );
}
