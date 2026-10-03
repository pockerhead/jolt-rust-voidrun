//! The handler for Jolt's debug assertions, which exist only in a native build with the
//! `asserts` feature. A failed assertion prints its expression, message, file and line to stderr
//! and aborts the process, so a test or CI run fails loudly instead of stopping at a breakpoint
//! (joltc's default) or continuing past it.

use std::borrow::Cow;
use std::ffi::{c_char, CStr};
use std::io::Write;
use std::panic::{catch_unwind, AssertUnwindSafe};

use joltphysics_sys::JPH_SetAssertFailureHandler;

// The assertion at the end of `PhysicsSystem::Update` (`PhysicsSystem.cpp:679` in the pinned
// Jolt) fires on every update that sets an `EPhysicsUpdateError` bit, and the update then returns
// the same bits (`:678-680`). The pinned enum (`EPhysicsUpdateError.h`) has exactly three bits,
// and [`PhysicsWorld::step`](crate::PhysicsWorld::step) reports each of them in its
// `StepReport`, so this one assertion lets the process continue. A Jolt upgrade must recheck the
// line, the message and the enum before it keeps this exception.
const UPDATE_ERROR_EXPRESSION: &str = "errors == EPhysicsUpdateError::None";
const UPDATE_ERROR_MESSAGE: &str =
    "An error occurred during the physics update, see EPhysicsUpdateError for more information";
const UPDATE_ERROR_FILE: &str = "Jolt/Physics/PhysicsSystem.cpp";
const UPDATE_ERROR_LINE: u32 = 679;

/// Whether the process continues after this failed assertion: only for the physics update
/// error, matched on expression, message, file and line.
fn continues_after(expression: &str, message: Option<&str>, file: &str, line: u32) -> bool {
    expression == UPDATE_ERROR_EXPRESSION
        && message == Some(UPDATE_ERROR_MESSAGE)
        && file.replace('\\', "/").ends_with(UPDATE_ERROR_FILE)
        && line == UPDATE_ERROR_LINE
}

/// A string Jolt passes to the handler, or `None` for null.
///
/// # Safety
/// `text` is null or points to a NUL-terminated string that lives for the call.
unsafe fn jolt_text<'a>(text: *const c_char) -> Option<Cow<'a, str>> {
    if text.is_null() {
        return None;
    }
    // SAFETY: not null, and the caller guarantees a NUL-terminated string that outlives the
    // returned borrow.
    Some(unsafe { CStr::from_ptr(text) }.to_string_lossy())
}

/// Jolt's assertion handler (`JPH_AssertFailureFunc`). Prints the failure and aborts, except for
/// the physics update error, which it reports and lets the update return. It never returns
/// `true`, so Jolt never runs its breakpoint. It may run on Jolt's worker threads and touches only
/// stderr.
///
/// # Safety
/// Each pointer is null or a NUL-terminated string that lives for the call, as Jolt passes them.
unsafe extern "C" fn on_assert_failed(
    expression: *const c_char,
    message: *const c_char,
    file: *const c_char,
    line: u32,
) -> bool {
    let handled = catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: Jolt passes null or NUL-terminated strings that live for this call (contract).
        let (expression, message, file) =
            unsafe { (jolt_text(expression), jolt_text(message), jolt_text(file)) };
        let expression = expression.unwrap_or_default();
        let file = file.unwrap_or_default();
        let mut stderr = std::io::stderr().lock();
        if continues_after(&expression, message.as_deref(), &file, line) {
            let _ = writeln!(
                stderr,
                "Jolt reported a physics update error (see StepReport): {file}:{line}"
            );
            return;
        }
        let _ = writeln!(
            stderr,
            "Jolt assertion failed: {file}:{line}: ({expression}) {}",
            message.unwrap_or_default()
        );
        std::process::abort();
    }));
    if handled.is_err() {
        std::process::abort();
    }
    false
}

/// Installs [`on_assert_failed`] as joltc's assertion handler. Call it once, before `JPH_Init`.
pub(crate) fn install() {
    // SAFETY: `JPH_SetAssertFailureHandler` only stores the pointer in a joltc static (a no-op
    // without asserts) and needs no initialization. The caller runs it once, inside the
    // `OnceLock` of `ensure_initialized`, so the store does not race.
    unsafe { JPH_SetAssertFailureHandler(Some(on_assert_failed)) };
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_the_pinned_update_error_continues() {
        let windows =
            r"D:\a\jolt\crates\joltphysics-sys\vendor\JoltPhysics\Jolt\Physics\PhysicsSystem.cpp";
        let unix = "/home/runner/vendor/JoltPhysics/Jolt/Physics/PhysicsSystem.cpp";
        let message = Some(UPDATE_ERROR_MESSAGE);
        for file in [windows, unix] {
            assert!(continues_after(
                UPDATE_ERROR_EXPRESSION,
                message,
                file,
                UPDATE_ERROR_LINE
            ));
        }
        let cases: [(&str, Option<&str>, &str, u32); 7] = [
            ("isfinite(len_sq)", message, unix, UPDATE_ERROR_LINE),
            (UPDATE_ERROR_EXPRESSION, None, unix, UPDATE_ERROR_LINE),
            (
                UPDATE_ERROR_EXPRESSION,
                Some("other"),
                unix,
                UPDATE_ERROR_LINE,
            ),
            (
                UPDATE_ERROR_EXPRESSION,
                message,
                "/x/Jolt/Physics/Body/MotionProperties.inl",
                UPDATE_ERROR_LINE,
            ),
            (
                UPDATE_ERROR_EXPRESSION,
                message,
                "/x/Jolt/Physics/XPhysicsSystem.cpp",
                UPDATE_ERROR_LINE,
            ),
            (UPDATE_ERROR_EXPRESSION, message, unix, 678),
            (UPDATE_ERROR_EXPRESSION, message, unix, 680),
        ];
        for (expression, message, file, line) in cases {
            assert!(
                !continues_after(expression, message, file, line),
                "{expression} {message:?} {file} {line}"
            );
        }
    }
}
