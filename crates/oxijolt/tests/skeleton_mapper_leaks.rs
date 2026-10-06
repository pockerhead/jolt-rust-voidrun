//! A leak gate for skeleton mappers: building both skeletons and a mapper (without locks, with
//! all locks and with selected locks in turn), mapping both ways, a mapper refused for its
//! hierarchy and a map refused for a degenerate chain, measured over many rounds against a
//! budget of 200 bytes per round. A control run then creates and initialises mappers through
//! `oxijolt-sys` and never releases them, and must exceed the budget, which shows the gate can
//! see such a leak.
//!
//! It measures the private bytes of the process, because these objects are allocated by C++,
//! which a Rust global allocator does not see. The file holds exactly one test, so its binary
//! runs alone and no parallel test disturbs the counter.
#![cfg(windows)]

mod common;

use std::ffi::CString;

use common::animation::Rig;
use common::memory::private_bytes;
use common::ragdoll::*;
use oxijolt::*;
use oxijolt_sys::*;

const WARM_UP_ROUNDS: usize = 1_000;
const MEASURED_ROUNDS: usize = 10_000;
/// 200 bytes per measured round.
const MAX_GROWTH: usize = 200 * MEASURED_ROUNDS;

fn build_mapper(rig: &Rig, locks: TranslationLocks<'_>) -> Result<SkeletonMapper, RagdollError> {
    let (ragdoll, animation) = (skeleton(), rig.skeleton());
    SkeletonMapper::new(
        MappedSkeleton {
            skeleton: &ragdoll,
            neutral_pose: &bind_pose(),
        },
        MappedSkeleton {
            skeleton: &animation,
            neutral_pose: &rig.neutral_pose(),
        },
        locks,
    )
}

fn mapper_round(rig: &Rig, misfit: &Rig, round: usize) {
    let hand = [rig.index("hand_l") as u32];
    let locks = match round % 3 {
        0 => TranslationLocks::None,
        1 => TranslationLocks::All,
        _ => TranslationLocks::Joints(&hand),
    };
    let mapper = build_mapper(rig, locks).unwrap();
    let local = rig.animated_local(0.01 * (round % 11) as f32);
    let mapped = mapper.map(&bind_pose(), &local).unwrap();
    assert_eq!(
        mapper.map_reverse(&mapped).unwrap().joints.len(),
        PART_COUNT
    );

    assert!(matches!(
        build_mapper(misfit, TranslationLocks::None),
        Err(RagdollError::HierarchyMismatch(_))
    ));
    let mut degenerate = bind_pose();
    degenerate.joints[CHEST].translation = Vec3::ZERO;
    degenerate.joints[HEAD].translation = Vec3::new(1.0e-30, 0.0, 0.0);
    assert_eq!(
        mapper.map(&degenerate, &local).map(|_| ()),
        Err(RagdollError::DegenerateChain(HEAD as u32))
    );
}

/// A two-joint raw skeleton `a` <- `b`, released on drop.
struct RawSkeleton(*mut JPH_Skeleton);

impl RawSkeleton {
    fn new() -> Self {
        // SAFETY: Jolt is initialised (the caller built skeletons before). The skeleton is ours
        // until drop; the names are NUL-terminated and copied, the parent was added before.
        unsafe {
            let skeleton = JPH_Skeleton_Create();
            assert!(!skeleton.is_null());
            for (name, parent) in [("a", -1), ("b", 0)] {
                let name = CString::new(name).unwrap();
                JPH_Skeleton_AddJoint2(skeleton, name.as_ptr(), parent);
            }
            Self(skeleton)
        }
    }
}

impl Drop for RawSkeleton {
    fn drop(&mut self) {
        // SAFETY: releases the one reference `JPH_Skeleton_Create` returned.
        unsafe { JPH_Skeleton_Destroy(self.0) };
    }
}

/// Creates and initialises `count` mappers between two two-joint skeletons and never releases
/// them.
fn leak_mappers(count: usize) {
    let skeleton = RawSkeleton::new();
    let column = |x, y, z, w| JPH_Vec4 { x, y, z, w };
    let at = |y| JPH_Mat4 {
        column: [
            column(1.0, 0.0, 0.0, 0.0),
            column(0.0, 1.0, 0.0, 0.0),
            column(0.0, 0.0, 1.0, 0.0),
            column(0.0, y, 0.0, 1.0),
        ],
    };
    let neutral = [at(0.0), at(1.0)];
    for _ in 0..count {
        // SAFETY: Jolt is initialised; the skeleton is live and the neutral pose holds one rigid
        // matrix per joint. The mapper is deliberately never released.
        unsafe {
            let mapper = JPH_SkeletonMapper_Create();
            assert!(!mapper.is_null());
            assert!(JPH_SkeletonMapper_Initialize2(
                mapper,
                skeleton.0,
                neutral.as_ptr(),
                2,
                skeleton.0,
                neutral.as_ptr(),
                2,
            ));
        }
    }
}

#[test]
fn skeleton_mappers_do_not_leak() {
    let rig = Rig::humanoid();
    let mut misfit = rig.clone();
    misfit.joints[rig.index("upper_arm_l")].parent = Some("pelvis".to_owned());

    for round in 0..WARM_UP_ROUNDS {
        mapper_round(&rig, &misfit, round);
    }
    let before = private_bytes();
    for round in 0..MEASURED_ROUNDS {
        mapper_round(&rig, &misfit, round);
    }
    let after = private_bytes();
    let growth = after.saturating_sub(before);
    eprintln!("mappers: private bytes before {before}, after {after}, growth {growth}");
    assert!(
        growth < MAX_GROWTH,
        "private bytes grew by {growth} (before {before}, after {after}): a mapper leaks"
    );

    // The control: as many mappers, never released, must show.
    let before = private_bytes();
    leak_mappers(MEASURED_ROUNDS);
    let after = private_bytes();
    let growth = after.saturating_sub(before);
    eprintln!("control: private bytes before {before}, after {after}, growth {growth}");
    assert!(
        growth >= MAX_GROWTH,
        "the control leaked {MEASURED_ROUNDS} mappers but private bytes grew only by {growth}"
    );
}
