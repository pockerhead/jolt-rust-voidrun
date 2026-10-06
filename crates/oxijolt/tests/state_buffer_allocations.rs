//! `save_state_into` makes no Rust heap allocation once its buffer has held a save that needed as
//! much: a counting global allocator sees none over 1000 saves of a scene with stacks, a car
//! and a character in every selection, in `Movable` and `Only` saves into a buffer that saw only
//! full saves, and in saves of worlds of two, one and two characters into one buffer. Jolt's own
//! allocations (its recorder) go through the C++ heap, which this allocator does not see. The
//! file holds exactly one test, so no other test allocates while the counter is armed.

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

/// Runs `saves` with the counter armed and returns how many allocations it saw.
fn allocations_of(saves: impl FnOnce() -> Result<(), StateError>) -> usize {
    ALLOCATIONS.store(0, Ordering::SeqCst);
    ARMED.store(true, Ordering::SeqCst);
    let saved = saves();
    ARMED.store(false, Ordering::SeqCst);
    saved.unwrap();
    ALLOCATIONS.load(Ordering::SeqCst)
}

/// A world with `count` characters standing on a floor.
fn world_with_characters(count: usize) -> PhysicsWorld {
    let mut world = common::world(Vec3::new(0.0, -9.81, 0.0), 1);
    let floor = Shape::new_box(Vec3::new(10.0, 0.5, 10.0)).unwrap();
    let at = RVec3::new(0.0, -0.5, 0.0);
    world
        .create_body(&floor, &BodySettings::new_static().position(at))
        .unwrap();
    let settings = CharacterSettings::humanoid(1.8, 0.3).unwrap();
    for i in 0..count {
        let feet = RVec3::new(2.0 * i as Real, 0.0, 0.0);
        let character = world
            .create_character(&settings, feet, Quat::IDENTITY)
            .unwrap();
        world
            .refresh_character_contacts(character, &QueryFilter::new())
            .unwrap();
    }
    world
}

#[test]
fn saving_into_a_used_buffer_allocates_nothing() {
    let mut scene = RollbackScene::new(1);
    for _ in 0..30 {
        scene.tick(Inputs::PLAYED);
    }
    let selected = [scene.sleeper, scene.platform, scene.floor];
    let selections = [
        BodySelection::All,
        BodySelection::Movable,
        BodySelection::Only(&selected),
    ];
    let mut state = WorldState::new();
    for &selection in &selections {
        scene.world.save_state_into(selection, &mut state).unwrap();
    }

    let allocations = allocations_of(|| {
        for save in 0..SAVES {
            let selection = selections[save % selections.len()];
            scene.world.save_state_into(selection, &mut state)?;
        }
        Ok(())
    });
    assert_eq!(allocations, 0, "{SAVES} saves into a used buffer allocated");
    // The saves are real: the last one restores.
    scene.world.restore_state(&state).unwrap();

    // A full save prepares a buffer for the selected saves too.
    let mut state = WorldState::new();
    scene
        .world
        .save_state_into(BodySelection::All, &mut state)
        .unwrap();
    let allocations = allocations_of(|| {
        scene
            .world
            .save_state_into(BodySelection::Movable, &mut state)?;
        scene
            .world
            .save_state_into(BodySelection::Only(&selected), &mut state)
    });
    assert_eq!(allocations, 0, "selected saves after a full save allocated");

    // A world with fewer characters leaves the other character buffers for the next save.
    let (mut two, one) = (world_with_characters(2), world_with_characters(1));
    let mut state = WorldState::new();
    two.save_state_into(BodySelection::All, &mut state).unwrap();
    let allocations = allocations_of(|| {
        one.save_state_into(BodySelection::All, &mut state)?;
        two.save_state_into(BodySelection::All, &mut state)
    });
    assert_eq!(
        allocations, 0,
        "saves of two, one and two characters allocated"
    );
    two.restore_state(&state).unwrap();
}
