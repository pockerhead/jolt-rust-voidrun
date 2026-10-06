//! Process memory and CPU time from the operating system: Windows `K32GetProcessMemoryInfo` and
//! `GetProcessTimes`, Linux `/proc/self/status` and `/proc/self/stat`.

/// Memory counters of this process, in bytes. Values are per operating system and are not
/// compared across them.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Memory {
    /// The largest working set so far (Windows `PeakWorkingSetSize`, Linux `VmHWM`).
    pub peak_resident: u64,
    /// The working set now (Windows `WorkingSetSize`, Linux `VmRSS`).
    pub resident: u64,
    /// The largest commit charge so far (Windows `PeakPagefileUsage`); 0 on Linux.
    pub peak_commit: u64,
}

/// CPU time this process has used on all its threads, user and kernel, in nanoseconds.
pub fn cpu_time_ns() -> u64 {
    imp::cpu_time_ns()
}

/// This process's memory counters.
pub fn memory() -> Memory {
    imp::memory()
}

#[cfg(windows)]
mod imp {
    use std::ffi::c_void;

    use super::Memory;

    /// Win32 `PROCESS_MEMORY_COUNTERS_EX` (psapi.h).
    #[repr(C)]
    #[derive(Default)]
    struct ProcessMemoryCountersEx {
        cb: u32,
        page_fault_count: u32,
        peak_working_set_size: usize,
        working_set_size: usize,
        quota_peak_paged_pool_usage: usize,
        quota_paged_pool_usage: usize,
        quota_peak_non_paged_pool_usage: usize,
        quota_non_paged_pool_usage: usize,
        pagefile_usage: usize,
        peak_pagefile_usage: usize,
        private_usage: usize,
    }

    const _: () = assert!(size_of::<ProcessMemoryCountersEx>() == 8 + 9 * size_of::<usize>());

    /// Win32 `FILETIME`: a count of 100 ns intervals in two halves.
    #[repr(C)]
    #[derive(Clone, Copy, Default)]
    struct FileTime {
        low: u32,
        high: u32,
    }

    const _: () = assert!(size_of::<FileTime>() == 8 && align_of::<FileTime>() == 4);

    impl FileTime {
        fn ns(self) -> u64 {
            ((u64::from(self.high) << 32) | u64::from(self.low)) * 100
        }
    }

    #[link(name = "kernel32")]
    extern "system" {
        fn GetCurrentProcess() -> *mut c_void;
        fn K32GetProcessMemoryInfo(
            process: *mut c_void,
            counters: *mut ProcessMemoryCountersEx,
            cb: u32,
        ) -> i32;
        fn GetProcessTimes(
            process: *mut c_void,
            creation: *mut FileTime,
            exit: *mut FileTime,
            kernel: *mut FileTime,
            user: *mut FileTime,
        ) -> i32;
    }

    pub fn memory() -> Memory {
        let mut counters = ProcessMemoryCountersEx {
            cb: size_of::<ProcessMemoryCountersEx>() as u32,
            ..Default::default()
        };
        // SAFETY: `GetCurrentProcess` returns a pseudo-handle that needs no closing; `counters`
        // is a live local of exactly `cb` bytes with the layout Win32 expects (asserted above).
        let ok =
            unsafe { K32GetProcessMemoryInfo(GetCurrentProcess(), &mut counters, counters.cb) };
        assert_ne!(ok, 0, "K32GetProcessMemoryInfo failed");
        Memory {
            peak_resident: counters.peak_working_set_size as u64,
            resident: counters.working_set_size as u64,
            peak_commit: counters.peak_pagefile_usage as u64,
        }
    }

    pub fn cpu_time_ns() -> u64 {
        let mut times = [FileTime::default(); 4];
        let [creation, exit, kernel, user] = &mut times;
        // SAFETY: the pseudo-handle of this process needs no closing; the four out-pointers are
        // distinct live locals with `FILETIME`'s layout (asserted above).
        let ok = unsafe { GetProcessTimes(GetCurrentProcess(), creation, exit, kernel, user) };
        assert_ne!(ok, 0, "GetProcessTimes failed");
        kernel.ns() + user.ns()
    }
}

#[cfg(target_os = "linux")]
mod imp {
    use super::Memory;

    /// Clock ticks per second of `/proc/self/stat` times (`USER_HZ`), 100 on Linux x86_64.
    const USER_HZ: u64 = 100;

    /// A `kB` field of `/proc/self/status`, in bytes.
    fn status_field(status: &str, name: &str) -> u64 {
        status
            .lines()
            .find_map(|line| line.strip_prefix(name)?.strip_prefix(':'))
            .and_then(|rest| rest.trim().strip_suffix("kB")?.trim().parse::<u64>().ok())
            .map_or(0, |kb| kb * 1024)
    }

    pub fn memory() -> Memory {
        let status = std::fs::read_to_string("/proc/self/status").expect("/proc/self/status");
        Memory {
            peak_resident: status_field(&status, "VmHWM"),
            resident: status_field(&status, "VmRSS"),
            peak_commit: 0,
        }
    }

    pub fn cpu_time_ns() -> u64 {
        let stat = std::fs::read_to_string("/proc/self/stat").expect("/proc/self/stat");
        // The command name in field 2 may hold spaces; the fields after it start past its ')'.
        let after_name = &stat[stat.rfind(')').expect("stat has a command name") + 2..];
        let fields: Vec<&str> = after_name.split_whitespace().collect();
        // utime and stime are fields 14 and 15 of the whole line, 12 and 13 after the name.
        let ticks: u64 = fields[11].parse::<u64>().unwrap() + fields[12].parse::<u64>().unwrap();
        ticks * (1_000_000_000 / USER_HZ)
    }
}
