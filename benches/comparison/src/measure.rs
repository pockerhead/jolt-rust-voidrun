//! Per-tick timing samples, the windows and statistics reported from them, and a description of
//! the machine.

/// A window of ticks, 1-based and inclusive, reported for every engine alike.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Window {
    pub name: &'static str,
    pub first: usize,
    pub last: usize,
}

/// The first simulated tick.
pub const TICK_1: Window = Window {
    name: "tick1",
    first: 1,
    last: 1,
};

/// The first second: the drop and the first impacts.
pub const COLD: Window = Window {
    name: "cold",
    first: 1,
    last: 60,
};

/// Ticks 61 to 600, after the first second.
pub const WARM: Window = Window {
    name: "warm",
    first: 61,
    last: 600,
};

/// The windows every timing run reports, besides "build + tick 1".
pub const WINDOWS: [Window; 3] = [TICK_1, COLD, WARM];

impl Window {
    /// The samples of this window, `samples[0]` being tick 1; empty when the run was shorter.
    pub fn of<'a>(&self, samples: &'a [u64]) -> &'a [u64] {
        if samples.len() < self.last {
            return &[];
        }
        &samples[self.first - 1..self.last]
    }
}

/// Mean, nearest-rank percentiles and maximum of a set of samples, in the samples' unit.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Stats {
    pub count: usize,
    pub mean: f64,
    pub p50: u64,
    pub p95: u64,
    pub p99: u64,
    pub max: u64,
}

impl Stats {
    /// Statistics of `samples`; `None` when there are none.
    pub fn of(samples: &[u64]) -> Option<Self> {
        if samples.is_empty() {
            return None;
        }
        let mut sorted = samples.to_vec();
        sorted.sort_unstable();
        let sum: u128 = sorted.iter().map(|&s| u128::from(s)).sum();
        Some(Self {
            count: sorted.len(),
            mean: sum as f64 / sorted.len() as f64,
            p50: percentile(&sorted, 50.0),
            p95: percentile(&sorted, 95.0),
            p99: percentile(&sorted, 99.0),
            max: *sorted.last().unwrap(),
        })
    }
}

/// The nearest-rank `p`th percentile of `sorted` (ascending, not empty): the smallest sample
/// with at least `p` percent of the samples at or below it.
pub fn percentile<T: Copy>(sorted: &[T], p: f64) -> T {
    assert!(!sorted.is_empty() && p > 0.0 && p <= 100.0);
    let rank = (p / 100.0 * sorted.len() as f64).ceil() as usize;
    sorted[rank.clamp(1, sorted.len()) - 1]
}

/// The machine a run happens on, as one line of `key=value` fields.
pub fn machine_line() -> String {
    let threads = std::thread::available_parallelism().map_or(0, |n| n.get());
    format!(
        "os={} arch={} cpu={} logical_threads={} avx2={} fma={}",
        std::env::consts::OS,
        std::env::consts::ARCH,
        cpu_name(),
        threads,
        cfg!(target_feature = "avx2"),
        cfg!(target_feature = "fma"),
    )
}

/// The CPU's brand string from `cpuid`, or `unknown`.
pub fn cpu_name() -> String {
    #[cfg(target_arch = "x86_64")]
    {
        use std::arch::x86_64::__cpuid;
        if __cpuid(0x8000_0000).eax >= 0x8000_0004 {
            let bytes: Vec<u8> = (0x8000_0002u32..=0x8000_0004)
                .flat_map(|leaf| {
                    let r = __cpuid(leaf);
                    [r.eax, r.ebx, r.ecx, r.edx]
                })
                .flat_map(u32::to_le_bytes)
                .collect();
            return String::from_utf8_lossy(&bytes)
                .trim_matches(char::from(0))
                .trim()
                .to_owned();
        }
    }
    "unknown".to_owned()
}

/// Whether this CPU has the instructions every build of the comparison is compiled for (AVX2
/// and FMA, as Jolt's defaults); the runs refuse to start without them.
pub fn cpu_has_build_isa() -> bool {
    #[cfg(target_arch = "x86_64")]
    {
        std::arch::is_x86_feature_detected!("avx2") && std::arch::is_x86_feature_detected!("fma")
    }
    #[cfg(not(target_arch = "x86_64"))]
    {
        false
    }
}
