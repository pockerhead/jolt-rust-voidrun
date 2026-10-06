//! Statistics, windows and the operating system readers.

use std::time::{Duration, Instant};

use comparison::measure::{percentile, Stats, COLD, TICK_1, WARM};
use comparison::os;

#[test]
fn nearest_rank_percentiles() {
    let sorted: Vec<u64> = (1..=100).collect();
    assert_eq!(percentile(&sorted, 50.0), 50);
    assert_eq!(percentile(&sorted, 99.0), 99);
    assert_eq!(percentile(&sorted, 100.0), 100);
    assert_eq!(percentile(&[7u64], 1.0), 7);
    assert_eq!(percentile(&[1u64, 2, 3], 50.0), 2);
    let stats = Stats::of(&[4, 1, 3, 2]).unwrap();
    assert_eq!((stats.mean, stats.p50, stats.max), (2.5, 2, 4));
    assert_eq!(Stats::of(&[]), None);
}

#[test]
fn windows_cut_the_documented_ticks() {
    let samples: Vec<u64> = (1..=600).collect();
    assert_eq!(TICK_1.of(&samples), &[1]);
    assert_eq!(COLD.of(&samples), &samples[..60]);
    assert_eq!(WARM.of(&samples).first(), Some(&61));
    assert_eq!(WARM.of(&samples).len(), 540);
    assert!(WARM.of(&samples[..599]).is_empty());
}

#[test]
fn memory_readers_return_values() {
    let memory = os::memory();
    assert!(memory.peak_resident > 0 && memory.resident > 0);
    assert!(memory.peak_resident >= memory.resident);
    let block = vec![1u8; 64 << 20];
    let after = os::memory();
    assert!(
        after.peak_resident >= memory.peak_resident + (32 << 20),
        "{after:?}"
    );
    drop(block);
}

fn spin(duration: Duration) {
    let start = Instant::now();
    let mut x = 0u64;
    while start.elapsed() < duration {
        x = std::hint::black_box(x.wrapping_add(1));
    }
}

/// CPU time over wall time while `threads` threads spin for 200 ms.
fn busy_cores(threads: usize) -> f64 {
    let cpu = os::cpu_time_ns();
    let wall = Instant::now();
    std::thread::scope(|scope| {
        for _ in 0..threads {
            scope.spawn(|| spin(Duration::from_millis(200)));
        }
    });
    (os::cpu_time_ns() - cpu) as f64 / wall.elapsed().as_nanos() as f64
}

#[test]
fn busy_cores_counts_spinning_threads() {
    let one = busy_cores(1);
    assert!((0.5..=1.5).contains(&one), "one thread: {one}");
    if std::thread::available_parallelism().map_or(1, |n| n.get()) >= 2 {
        let two = busy_cores(2);
        assert!(two > 1.2, "two threads: {two}");
    }
}
