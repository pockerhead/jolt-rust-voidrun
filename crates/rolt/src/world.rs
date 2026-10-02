//! Jolt's process-wide initialisation.

use std::sync::OnceLock;

use joltc_sys::JPH_Init;

/// Runs `JPH_Init` once per process and returns whether it succeeded. rolt never calls
/// `JPH_Shutdown`: Jolt's global state lives as long as the process.
pub(crate) fn ensure_initialized() -> bool {
    static INIT: OnceLock<bool> = OnceLock::new();
    // SAFETY: `OnceLock` runs the closure exactly once per process, which is the only
    // synchronisation `JPH_Init` (an unsynchronised `bool` guard) needs.
    *INIT.get_or_init(|| unsafe { JPH_Init() })
}
