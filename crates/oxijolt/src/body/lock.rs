//! Body locks: running a closure with one or more bodies locked.

use std::ptr::NonNull;

use oxijolt_sys::*;

use super::BodyId;
use crate::owned::{JoltObject, Owned};

/// A body write lock. Destroying it deletes Jolt's `BodyLockMultiWrite`, which unlocks the
/// bodies, and frees the joltc wrapper; as an `Owned` it is released also while unwinding.
impl JoltObject for JPH_BodyLockMultiWrite {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner owns the lock (trait contract), which is released exactly once here.
        unsafe { JPH_BodyLockMultiWrite_Destroy(ptr) };
    }
}

/// Runs `f` with the body locked for writing; `None` if the id no longer resolves.
///
/// The body pointer never leaves `f`. `f` must not call the body interface for the same body:
/// Jolt's body mutexes are not recursive.
pub(crate) fn with_locked_body<R>(
    lock_interface: NonNull<JPH_BodyLockInterface>,
    id: BodyId,
    f: impl FnOnce(NonNull<JPH_Body>) -> R,
) -> Option<R> {
    let raw = id.raw;
    // SAFETY: the lock interface belongs to a live world. joltc copies the one id into the
    // lock object, so `raw` only has to live for the call. The handle takes over the lock.
    let lock = unsafe {
        Owned::from_raw(JPH_BodyLockInterface_LockMultiWrite(
            lock_interface.as_ptr(),
            &raw,
            1,
        ))
    }?;
    // SAFETY: `lock` is live and holds exactly one id, at index 0. Jolt returns null unless
    // index and sequence number both match a live body (`BodyManager::TryGetBody`).
    let body = NonNull::new(unsafe { JPH_BodyLockMultiWrite_GetBody(lock.as_ptr(), 0) })?;
    Some(f(body))
}

/// Runs `f` with both bodies locked for writing under one multi-body lock; `None` if either id
/// no longer resolves.
///
/// Jolt locks the bodies' mutexes in a fixed order, and two ids that share a mutex are locked
/// once, so one lock cannot deadlock where two separate locks could. The body pointers never
/// leave `f`. `f` must not call the body interface for these bodies: Jolt's body mutexes are not
/// recursive.
pub(crate) fn with_locked_bodies<R>(
    lock_interface: NonNull<JPH_BodyLockInterface>,
    ids: [BodyId; 2],
    f: impl FnOnce(NonNull<JPH_Body>, NonNull<JPH_Body>) -> R,
) -> Option<R> {
    let raw = ids.map(|id| id.raw);
    // SAFETY: the lock interface belongs to a live world. joltc copies the two ids into the lock
    // object, so `raw` only has to live for the call. The handle takes over the lock.
    let lock = unsafe {
        Owned::from_raw(JPH_BodyLockInterface_LockMultiWrite(
            lock_interface.as_ptr(),
            raw.as_ptr(),
            2,
        ))
    }?;
    // SAFETY: `lock` is live and holds exactly two ids, at indices 0 and 1. Jolt returns null
    // unless index and sequence number both match a live body (`BodyManager::TryGetBody`).
    let (first, second) = unsafe {
        (
            JPH_BodyLockMultiWrite_GetBody(lock.as_ptr(), 0),
            JPH_BodyLockMultiWrite_GetBody(lock.as_ptr(), 1),
        )
    };
    Some(f(NonNull::new(first)?, NonNull::new(second)?))
}

/// A body read lock. Destroying it deletes Jolt's `BodyLockMultiRead`, which unlocks the
/// bodies, and frees the joltc wrapper; as an `Owned` it is released also while unwinding.
impl JoltObject for JPH_BodyLockMultiRead {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner owns the lock (trait contract), which is released exactly once here.
        unsafe { JPH_BodyLockMultiRead_Destroy(ptr) };
    }
}

/// Runs `f` with the body locked for reading; `None` if the id no longer resolves.
///
/// The body pointer never leaves `f`, and `f` only reads through it. `f` must not lock the same
/// body for writing.
pub(crate) fn with_read_locked_body<R>(
    lock_interface: NonNull<JPH_BodyLockInterface>,
    id: BodyId,
    f: impl FnOnce(NonNull<JPH_Body>) -> R,
) -> Option<R> {
    let raw = id.raw;
    // SAFETY: the lock interface belongs to a live world. joltc copies the one id into the
    // lock object, so `raw` only has to live for the call. The handle takes over the lock.
    let lock = unsafe {
        Owned::from_raw(JPH_BodyLockInterface_LockMultiRead(
            lock_interface.as_ptr(),
            &raw,
            1,
        ))
    }?;
    // SAFETY: `lock` is live and holds exactly one id, at index 0. Jolt returns null unless
    // index and sequence number both match a live body (`BodyManager::TryGetBody`).
    let body = NonNull::new(unsafe { JPH_BodyLockMultiRead_GetBody(lock.as_ptr(), 0) }.cast_mut())?;
    Some(f(body))
}
