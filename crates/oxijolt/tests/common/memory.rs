//! The private bytes of the process (Windows `K32GetProcessMemoryInfo`), for leak gates of
//! objects that C++ allocates, which a Rust global allocator does not see.

use std::ffi::c_void;

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

#[link(name = "kernel32")]
extern "system" {
    fn GetCurrentProcess() -> *mut c_void;
    fn K32GetProcessMemoryInfo(
        process: *mut c_void,
        counters: *mut ProcessMemoryCountersEx,
        cb: u32,
    ) -> i32;
}

/// Bytes of memory the process has committed for itself.
pub fn private_bytes() -> usize {
    let mut counters = ProcessMemoryCountersEx {
        cb: size_of::<ProcessMemoryCountersEx>() as u32,
        ..Default::default()
    };
    // SAFETY: `GetCurrentProcess` returns a pseudo-handle that needs no closing; `counters` is
    // a live local of exactly `cb` bytes with the layout Win32 expects (asserted above).
    let ok = unsafe { K32GetProcessMemoryInfo(GetCurrentProcess(), &mut counters, counters.cb) };
    assert_ne!(ok, 0, "K32GetProcessMemoryInfo failed");
    counters.private_usage
}
