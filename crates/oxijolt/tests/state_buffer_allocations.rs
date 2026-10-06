//! `save_state_into` makes no Rust heap allocation once its buffer has held a save of the same
//! size: a counting global allocator sees none over 1000 saves of a scene with stacks, a car
//! and a character. Jolt's own allocations (its recorder) go through the C++ heap, which this
//! allocator does not see. The file holds exactly one test, so no other test allocates while the
//! counter is armed.

mod common;

use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

use common::rollback::{Inputs, RollbackScene};
use oxijolt::*;

/// Counts allocations while armed.
struct CountingAllocator;

static ARMED: AtomicBool = AtomicBool::new(false);
static ALLOCATIONS: AtomicUsize = AtomicUsize::new(0);

// SAFETY: every call forwards to `System` with the caller's arguments; counting touches only
// atomics and never allocates.
unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        count();
        // SAFETY: forwarded unchanged; the caller upholds `GlobalAlloc::alloc`'s contract.
        unsafe { System.alloc(layout) }
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        count();
        // SAFETY: as in `alloc`.
        unsafe { System.alloc_zeroed(layout) }
    }

    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        count();
        // SAFETY: as in `alloc`; `ptr` came from this allocator, which is `System`.
        unsafe { System.realloc(ptr, layout, new_size) }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        // SAFETY: as in `realloc`.
        unsafe { System.dealloc(ptr, layout) }
    }
}

fn count() {
    if ARMED.load(Ordering::SeqCst) {
        ALLOCATIONS.fetch_add(1, Ordering::SeqCst);
    }
}

#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator;

const SAVES: usize = 1000;

#[test]
fn saving_into_a_used_buffer_allocates_nothing() {
    let mut scene = RollbackScene::new(1);
    for _ in 0..30 {
        scene.tick(Inputs::PLAYED);
    }
    let selected = [scene.sleeper, scene.platform, scene.floor];
    let selections = [BodySelection::All, BodySelection::Only(&selected)];
    let mut state = WorldState::new();
    for &selection in &selections {
        scene.world.save_state_into(selection, &mut state).unwrap();
    }

    ALLOCATIONS.store(0, Ordering::SeqCst);
    ARMED.store(true, Ordering::SeqCst);
    for save in 0..SAVES {
        let selection = selections[save % selections.len()];
        let saved = scene.world.save_state_into(selection, &mut state);
        if saved.is_err() {
            ARMED.store(false, Ordering::SeqCst);
            panic!("save {save} failed: {saved:?}");
        }
    }
    ARMED.store(false, Ordering::SeqCst);
    let allocations = ALLOCATIONS.load(Ordering::SeqCst);
    assert_eq!(allocations, 0, "{SAVES} saves into a used buffer allocated");

    // The saves are real: the last one restores.
    scene.world.restore_state(&state).unwrap();
}
