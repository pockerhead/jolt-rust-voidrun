//! Jolt's state recorder calls behind [`WorldState`](super::WorldState): saving the physics
//! system into a stream and restoring it.

use std::mem::MaybeUninit;

use oxijolt_sys::*;

use crate::owned::Owned;
use crate::PhysicsWorld;

impl PhysicsWorld {
    /// Writes Jolt's saved stream of every part of the system's state into `jolt`, reusing its
    /// memory, with only the bodies whose raw ids are in `bodies` (distinct bodies of this world,
    /// from `select_bodies`), or with every body for `None`.
    pub(super) fn record_into(&self, bodies: Option<&[u32]>, jolt: &mut Vec<MaybeUninit<u8>>) {
        // SAFETY: Jolt is initialised (the world exists). The handle takes over the recorder.
        let recorder = unsafe { Owned::from_raw(JPH_StateRecorder_Create()) }
            .unwrap_or_else(|| unreachable!("`new` does not return null"));
        let (ids, count) = match bodies {
            // Distinct bodies of this world, so no more than its body count, a `u32`.
            Some(ids) => (ids.as_ptr(), ids.len() as u32),
            None => (std::ptr::null(), 0),
        };
        // SAFETY: the system is live and no step runs: `step` needs `&mut self`. `SaveState` is
        // const in Jolt and takes the body and constraint locks itself (see `Sync for
        // PhysicsWorld`). The recorder is live and used by this thread only. `ids` is null or
        // readable for `count` ids, which the extension copies; for an empty selection it is
        // dangling with `count` 0, and the extension then does not touch it.
        let size = unsafe {
            JPH_PhysicsSystem_SaveState(
                self.system.as_ptr(),
                recorder.as_ptr(),
                JPH_StateRecorderState_All,
                ids,
                count,
            );
            JPH_StateRecorder_GetDataSize(recorder.as_ptr())
        };
        jolt.clear();
        jolt.resize(size, MaybeUninit::uninit());
        // SAFETY: `jolt` holds exactly `size` writable bytes, which joltc copies at most with
        // `memcpy`. The copied bytes may lack a defined value, which `MaybeUninit` allows; Rust
        // never reads them.
        unsafe { JPH_StateRecorder_CopyData(recorder.as_ptr(), jolt.as_mut_ptr().cast(), size) };
    }

    /// Restores Jolt's saved stream `jolt`; whether Jolt read it without failing.
    pub(super) fn restore_jolt(&mut self, jolt: &[MaybeUninit<u8>]) -> bool {
        // SAFETY: Jolt is initialised (the world exists). The handle takes over the recorder.
        let recorder = unsafe { Owned::from_raw(JPH_StateRecorder_Create()) }
            .unwrap_or_else(|| unreachable!("`new` does not return null"));
        // SAFETY: the recorder is live and used by this thread only, and `jolt` is readable for
        // its length; the recorder copies it byte for byte, bytes without a defined value
        // included, and Jolt uses those only for a wheel with a contact. The bytes are a
        // complete stream that `SaveState` of this world wrote at the current structure epoch (`restore_state` checked both, and `WorldState` has no
        // other constructor), so the bodies and constraints it names exist, in the same
        // constraint order, and `RestoreState` reads exactly what was written. The world is
        // borrowed mutably, so no step, query or body access runs meanwhile; this thread holds
        // no body lock.
        unsafe {
            JPH_StateRecorder_WriteBytes(recorder.as_ptr(), jolt.as_ptr().cast(), jolt.len());
            JPH_StateRecorder_Rewind(recorder.as_ptr());
            let restored = JPH_PhysicsSystem_RestoreState(self.system.as_ptr(), recorder.as_ptr());
            restored && !JPH_StateRecorder_IsFailed(recorder.as_ptr())
        }
    }
}
