//! Smoke test for the fork's aligned skeleton mapper entry points (`JPH_SkeletonMapper_*2`): a
//! three-joint ragdoll skeleton mapped onto a six-joint animation skeleton through matrix arrays
//! that are 4 bytes past a 16-byte boundary, the stages of Jolt's `Map` and `MapReverse`, and every
//! refusal (nothing changed, nothing written).

mod framework;

use std::ffi::CString;

use framework::*;
use oxijolt_sys::*;

/// The C extension's shortest chain direction, metres.
const MIN_CHAIN_LENGTH: f32 = 1.0e-3;

// Ragdoll joints.
const R_ROOT: usize = 0;
const R_MID: usize = 1;
const R_TIP: usize = 2;
// Animation joints.
const A_BASE: usize = 0;
const A_ROOT: usize = 1;
const A_MID_A: usize = 2;
const A_MID: usize = 3;
const A_TIP: usize = 4;
const A_TIP_END: usize = 5;

/// A row-major 4x4 matrix in f64, for the oracles.
type M = [[f64; 4]; 4];

/// Matrices in a zero-initialised buffer whose first element is 4 bytes past a 16-byte boundary:
/// a `JPH::Mat44*` cast of it would be misaligned.
struct Matrices {
    words: Vec<u32>,
    start: usize,
    count: usize,
}

impl Matrices {
    fn zeroed(count: usize) -> Self {
        let words = vec![0u32; count * 16 + 4];
        let base = words.as_ptr() as usize;
        let start = (0..4)
            .find(|i| (base + 4 * i) % 16 == 4)
            .expect("a 4-aligned address has a word 4 bytes past a 16-byte boundary");
        Self {
            words,
            start,
            count,
        }
    }

    fn from(matrices: &[JPH_Mat4]) -> Self {
        let mut result = Self::zeroed(matrices.len());
        for (i, matrix) in matrices.iter().enumerate() {
            // SAFETY: `i < count`, the buffer holds `count` matrices from `start`, and a
            // `JPH_Mat4` needs 4-byte alignment, which every word has.
            unsafe { result.as_mut_ptr().add(i).write(*matrix) };
        }
        result
    }

    /// Every matrix with all sixteen entries `value`.
    fn filled(count: usize, value: f32) -> Self {
        let column = JPH_Vec4 {
            x: value,
            y: value,
            z: value,
            w: value,
        };
        Self::from(&vec![
            JPH_Mat4 {
                column: [column; 4]
            };
            count
        ])
    }

    fn as_ptr(&self) -> *const JPH_Mat4 {
        assert_eq!(self.words[self.start..].as_ptr() as usize % 16, 4);
        self.words[self.start..].as_ptr().cast()
    }

    fn as_mut_ptr(&mut self) -> *mut JPH_Mat4 {
        self.words[self.start..].as_mut_ptr().cast()
    }

    fn to_vec(&self) -> Vec<JPH_Mat4> {
        // SAFETY: the buffer holds `count` matrices from `start`, 4-byte aligned.
        (0..self.count)
            .map(|i| unsafe { self.as_ptr().add(i).read() })
            .collect()
    }

    fn bits(&self) -> Vec<u32> {
        self.words[self.start..self.start + 16 * self.count].to_vec()
    }
}

fn quat_about(axis: [f64; 3], angle: f64) -> [f64; 4] {
    let (s, c) = (angle / 2.0).sin_cos();
    [axis[0] * s, axis[1] * s, axis[2] * s, c]
}

/// The rigid transform of rotation `q` (x, y, z, w) and translation `t`.
fn rigid(q: [f64; 4], t: [f64; 3]) -> M {
    let [x, y, z, w] = q;
    [
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
            t[0],
        ],
        [
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
            t[1],
        ],
        [
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
            t[2],
        ],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

fn translation(t: [f64; 3]) -> M {
    rigid([0.0, 0.0, 0.0, 1.0], t)
}

fn mul(a: &M, b: &M) -> M {
    let mut result = [[0.0; 4]; 4];
    for (r, row) in result.iter_mut().enumerate() {
        for (c, entry) in row.iter_mut().enumerate() {
            *entry = (0..4).map(|k| a[r][k] * b[k][c]).sum();
        }
    }
    result
}

/// The inverse of a rigid transform.
fn inverse(m: &M) -> M {
    let mut result = [[0.0; 4]; 4];
    for r in 0..3 {
        for c in 0..3 {
            result[r][c] = m[c][r];
        }
        result[r][3] = -(0..3).map(|k| m[k][r] * m[k][3]).sum::<f64>();
    }
    result[3][3] = 1.0;
    result
}

fn to_jph(m: &M) -> JPH_Mat4 {
    let column = |c: usize| JPH_Vec4 {
        x: m[0][c] as f32,
        y: m[1][c] as f32,
        z: m[2][c] as f32,
        w: m[3][c] as f32,
    };
    JPH_Mat4 {
        column: [column(0), column(1), column(2), column(3)],
    }
}

fn from_jph(m: &JPH_Mat4) -> M {
    let mut result = [[0.0; 4]; 4];
    for (c, column) in m.column.iter().enumerate() {
        result[0][c] = f64::from(column.x);
        result[1][c] = f64::from(column.y);
        result[2][c] = f64::from(column.z);
        result[3][c] = f64::from(column.w);
    }
    result
}

fn position(m: &M) -> [f64; 3] {
    [m[0][3], m[1][3], m[2][3]]
}

#[track_caller]
fn assert_close(actual: &M, expected: &M, tolerance: f64) {
    for r in 0..4 {
        for c in 0..4 {
            assert!(
                (actual[r][c] - expected[r][c]).abs() <= tolerance,
                "entry ({r}, {c}): {actual:?} vs {expected:?}"
            );
        }
    }
}

/// A Jolt skeleton built with `JPH_Skeleton_AddJoint2` (name, parent index), or with
/// `JPH_Skeleton_AddJoint3` (name, parent name) and `CalculateParentJointIndices` when
/// `by_name` is set. Destroyed on drop.
struct Skeleton(*mut JPH_Skeleton);

impl Skeleton {
    fn new(joints: &[(&str, i32)]) -> Self {
        init();
        // SAFETY: Jolt is initialised; the skeleton is ours until drop.
        let skeleton = unsafe { JPH_Skeleton_Create() };
        for (name, parent) in joints {
            let name = CString::new(*name).unwrap();
            // SAFETY: live skeleton, NUL-terminated name Jolt copies, parent -1 or added before.
            unsafe { JPH_Skeleton_AddJoint2(skeleton, name.as_ptr(), *parent) };
        }
        Self(skeleton)
    }

    fn by_parent_name(joints: &[(&str, &str)]) -> Self {
        init();
        // SAFETY: Jolt is initialised; the skeleton is ours until drop.
        let skeleton = unsafe { JPH_Skeleton_Create() };
        for (name, parent) in joints {
            let (name, parent) = (CString::new(*name).unwrap(), CString::new(*parent).unwrap());
            // SAFETY: live skeleton and NUL-terminated names Jolt copies.
            unsafe { JPH_Skeleton_AddJoint3(skeleton, name.as_ptr(), parent.as_ptr()) };
        }
        // SAFETY: live skeleton; resolves parent names to indices.
        unsafe { JPH_Skeleton_CalculateParentJointIndices(skeleton) };
        Self(skeleton)
    }

    fn ragdoll() -> Self {
        Self::new(&[("root", -1), ("mid", 0), ("tip", 1)])
    }

    fn animation() -> Self {
        Self::new(&[
            ("base", -1),
            ("root", 0),
            ("mid_a", 1),
            ("mid", 2),
            ("tip", 3),
            ("tip_end", 4),
        ])
    }

    fn count(&self) -> u32 {
        // SAFETY: live skeleton.
        unsafe { JPH_Skeleton_GetJointCount(self.0) as u32 }
    }
}

impl Drop for Skeleton {
    fn drop(&mut self) {
        // SAFETY: releases the one reference `JPH_Skeleton_Create` returned.
        unsafe { JPH_Skeleton_Destroy(self.0) };
    }
}

/// A Jolt skeleton mapper, destroyed on drop.
struct Mapper(*mut JPH_SkeletonMapper);

impl Mapper {
    fn new() -> Self {
        init();
        // SAFETY: Jolt is initialised; the mapper is ours until drop.
        Self(unsafe { JPH_SkeletonMapper_Create() })
    }

    fn initialize(
        &self,
        skeleton1: &Skeleton,
        neutral1: &Matrices,
        count1: u32,
        skeleton2: &Skeleton,
        neutral2: &Matrices,
        count2: u32,
    ) -> bool {
        // SAFETY: live handles; the arrays hold the counts the callers pass (at most their length).
        unsafe {
            JPH_SkeletonMapper_Initialize2(
                self.0,
                skeleton1.0,
                neutral1.as_ptr(),
                count1,
                skeleton2.0,
                neutral2.as_ptr(),
                count2,
            )
        }
    }

    fn mapped(&self, joint1: i32) -> i32 {
        // SAFETY: live mapper; the getter only reads it.
        unsafe { JPH_SkeletonMapper_GetMappedJointIndex(self.0, joint1) }
    }

    fn locked(&self, joint2: i32) -> bool {
        // SAFETY: live mapper; the getter only reads it.
        unsafe { JPH_SkeletonMapper_IsJointTranslationLocked(self.0, joint2) }
    }

    fn lock(&self, skeleton2: &Skeleton, mask: &[bool], neutral2: &Matrices) -> bool {
        assert_eq!(mask.len() as u32, skeleton2.count());
        // SAFETY: live handles; the mask and the neutral pose hold count2 elements.
        unsafe {
            JPH_SkeletonMapper_LockTranslations2(
                self.0,
                skeleton2.0,
                mask.as_ptr(),
                neutral2.as_ptr(),
                skeleton2.count(),
            )
        }
    }

    fn lock_all(&self, skeleton2: &Skeleton, neutral2: &Matrices) -> bool {
        // SAFETY: live handles; the neutral pose holds count2 elements.
        unsafe {
            JPH_SkeletonMapper_LockAllTranslations2(
                self.0,
                skeleton2.0,
                neutral2.as_ptr(),
                skeleton2.count(),
            )
        }
    }

    /// `Map2` into `out`; returns the result and the reported degenerate joint.
    fn map(
        &self,
        pose1: &Matrices,
        count1: u32,
        local2: &Matrices,
        count2: u32,
        out: &mut Matrices,
    ) -> (bool, i32) {
        assert!(pose1.count >= count1 as usize && local2.count >= count2 as usize);
        assert!(out.count >= count2 as usize);
        let mut degenerate = 99;
        // SAFETY: live mapper; every array holds at least its count.
        let ok = unsafe {
            JPH_SkeletonMapper_Map2(
                self.0,
                pose1.as_ptr(),
                count1,
                local2.as_ptr(),
                count2,
                out.as_mut_ptr(),
                &mut degenerate,
            )
        };
        (ok, degenerate)
    }

    fn map_reverse(&self, pose2: &Matrices, count2: u32, out: &mut Matrices, count1: u32) -> bool {
        assert!(pose2.count >= count2 as usize && out.count >= count1 as usize);
        // SAFETY: live mapper; every array holds at least its count.
        unsafe {
            JPH_SkeletonMapper_MapReverse2(self.0, pose2.as_ptr(), count2, out.as_mut_ptr(), count1)
        }
    }
}

impl Drop for Mapper {
    fn drop(&mut self) {
        // SAFETY: releases the one reference `JPH_SkeletonMapper_Create` returned.
        unsafe { JPH_SkeletonMapper_Destroy(self.0) };
    }
}

/// Ragdoll neutral pose (model space): the chain start `root` a quarter turn about Y.
fn neutral1() -> Vec<M> {
    vec![
        rigid(
            quat_about([0.0, 1.0, 0.0], std::f64::consts::FRAC_PI_2),
            [0.1, 0.0, 0.0],
        ),
        translation([0.1, 0.5, 0.0]),
        translation([0.1, 1.0, 0.0]),
    ]
}

/// Animation neutral pose (model space): `root` unrotated and 2 cm higher than the ragdoll's.
fn neutral2() -> Vec<M> {
    vec![
        translation([0.0, 0.0, 0.0]),
        translation([0.1, 0.02, 0.0]),
        translation([0.1, 0.25, 0.0]),
        translation([0.1, 0.5, 0.0]),
        translation([0.1, 1.0, 0.0]),
        translation([0.1, 1.2, 0.0]),
    ]
}

/// An animation local pose: the in-between joint `mid_a` bent about Z.
fn local2() -> Vec<M> {
    vec![
        translation([0.0, 0.0, 0.0]),
        translation([0.1, 0.02, 0.0]),
        rigid(quat_about([0.0, 0.0, 1.0], 0.3), [0.0, 0.23, 0.0]),
        translation([0.0, 0.25, 0.0]),
        translation([0.0, 0.5, 0.0]),
        rigid(quat_about([1.0, 0.0, 0.0], 0.2), [0.0, 0.2, 0.0]),
    ]
}

/// A ragdoll pose (model space) with `mid` off the neutral chain axis.
fn pose1() -> Vec<M> {
    vec![
        rigid(quat_about([1.0, 0.0, 0.0], 0.2), [0.3, 0.1, 0.2]),
        rigid(quat_about([0.0, 0.0, 1.0], -0.4), [0.5, 0.6, 0.1]),
        rigid(quat_about([0.0, 1.0, 0.0], 0.5), [0.6, 1.1, 0.2]),
    ]
}

fn jph(matrices: &[M]) -> Matrices {
    Matrices::from(&matrices.iter().map(to_jph).collect::<Vec<_>>())
}

/// A mapper initialised with the fixture's skeletons and neutral poses.
struct Fixture {
    ragdoll: Skeleton,
    animation: Skeleton,
    neutral1: Matrices,
    neutral2: Matrices,
    mapper: Mapper,
}

impl Fixture {
    fn new() -> Self {
        let fixture = Self::uninitialized();
        assert!(fixture.mapper.initialize(
            &fixture.ragdoll,
            &fixture.neutral1,
            3,
            &fixture.animation,
            &fixture.neutral2,
            6
        ));
        fixture
    }

    fn uninitialized() -> Self {
        Self {
            ragdoll: Skeleton::ragdoll(),
            animation: Skeleton::animation(),
            neutral1: jph(&neutral1()),
            neutral2: jph(&neutral2()),
            mapper: Mapper::new(),
        }
    }

    fn map(&self, pose1: &[M], local2: &[M]) -> Option<Matrices> {
        let mut out = Matrices::zeroed(6);
        let (ok, degenerate) = self.mapper.map(&jph(pose1), 3, &jph(local2), 6, &mut out);
        assert_eq!(degenerate, -1);
        ok.then_some(out)
    }

    /// Asserts that `Map2` refuses with `degenerate` reported and leaves a sentinel output alone.
    #[track_caller]
    fn assert_map_refused(&self, pose1: &[M], local2: &[M], degenerate: i32) {
        let mut out = Matrices::filled(6, 7.0);
        let before = out.bits();
        let result = self.mapper.map(&jph(pose1), 3, &jph(local2), 6, &mut out);
        assert_eq!(result, (false, degenerate));
        assert_eq!(out.bits(), before, "a refused map wrote its output");
    }
}

#[test]
fn initialize2_maps_by_name_through_misaligned_arrays() {
    let fixture = Fixture::new();
    assert_eq!(fixture.mapper.mapped(R_ROOT as i32), A_ROOT as i32);
    assert_eq!(fixture.mapper.mapped(R_MID as i32), A_MID as i32);
    assert_eq!(fixture.mapper.mapped(R_TIP as i32), A_TIP as i32);
    assert_eq!(fixture.mapper.mapped(3), -1);
}

#[test]
fn map2_and_map_reverse2_follow_jolts_stages() {
    let fixture = Fixture::new();
    let (p1, l2, n1, n2) = (pose1(), local2(), neutral1(), neutral2());
    let out: Vec<M> = fixture
        .map(&p1, &l2)
        .unwrap()
        .to_vec()
        .iter()
        .map(from_jph)
        .collect();

    // Direct mappings: in1 * inv(N1) * N2.
    let direct = |j1: usize, j2: usize| mul(&mul(&p1[j1], &inverse(&n1[j1])), &n2[j2]);
    assert_close(&out[A_TIP], &direct(R_TIP, A_TIP), 1e-5);
    assert_close(&out[A_MID], &direct(R_MID, A_MID), 1e-5);

    // The chain start keeps its direct translation, turns, and then points its chain along the
    // ragdoll's root -> mid direction.
    let start = direct(R_ROOT, A_ROOT);
    assert!(
        (0..3).all(|r| (out[A_ROOT][r][3] - start[r][3]).abs() <= 1e-5),
        "the chain start moved"
    );
    let turned = (0..3).any(|r| (0..3).any(|c| (out[A_ROOT][r][c] - start[r][c]).abs() > 1e-3));
    assert!(turned, "the chain correction is the identity");
    let end = mul(&mul(&out[A_ROOT], &l2[A_MID_A]), &l2[A_MID]);
    let actual = sub(position(&end), position(&out[A_ROOT]));
    let desired = sub(position(&p1[R_MID]), position(&p1[R_ROOT]));
    assert!(
        cos(actual, desired) > 1.0 - 1e-6,
        "{actual:?} vs {desired:?}"
    );

    // The in-between joint from its local transform, the extra root as its local transform, the
    // leaf below its parent.
    assert_close(&out[A_MID_A], &mul(&out[A_ROOT], &l2[A_MID_A]), 1e-5);
    assert_close(&out[A_BASE], &l2[A_BASE], 0.0);
    assert_close(&out[A_TIP_END], &mul(&out[A_TIP], &l2[A_TIP_END]), 1e-5);

    // Reverse of the forward output on the direct-only joints.
    let forward = jph(&out);
    let mut reverse = Matrices::zeroed(3);
    assert!(fixture.mapper.map_reverse(&forward, 6, &mut reverse, 3));
    let reverse: Vec<M> = reverse.to_vec().iter().map(from_jph).collect();
    assert_close(&reverse[R_MID], &p1[R_MID], 1e-5);
    assert_close(&reverse[R_TIP], &p1[R_TIP], 1e-5);
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn cos(a: [f64; 3], b: [f64; 3]) -> f64 {
    let dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    let length = |v: [f64; 3]| (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    dot / (length(a) * length(b))
}

#[test]
fn initialize2_refusals() {
    let (ragdoll, animation) = (Skeleton::ragdoll(), Skeleton::animation());
    let (n1, n2) = (jph(&neutral1()), jph(&neutral2()));
    let refused = |s1: &Skeleton, m1: &Matrices, c1: u32, s2: &Skeleton, m2: &Matrices, c2: u32| {
        let mapper = Mapper::new();
        assert!(!mapper.initialize(s1, m1, c1, s2, m2, c2));
        assert_eq!(
            mapper.mapped(0),
            -1,
            "a refused initialisation changed the mapper"
        );
        // The mapper is still uninitialised: a valid call succeeds.
        assert!(mapper.initialize(&ragdoll, &n1, 3, &animation, &n2, 6));
    };
    refused(&ragdoll, &n1, 2, &animation, &n2, 6);
    refused(&ragdoll, &n1, 3, &animation, &n2, 5);
    let empty = Skeleton::new(&[]);
    refused(&empty, &n1, 0, &animation, &n2, 6);
    refused(&animation, &n2, 6, &ragdoll, &n1, 3);

    let disordered = Skeleton::by_parent_name(&[("mid", "root"), ("root", ""), ("tip", "mid")]);
    let mut disordered_neutral = neutral1();
    disordered_neutral.swap(0, 1);
    refused(
        &disordered,
        &jph(&disordered_neutral),
        3,
        &animation,
        &n2,
        6,
    );

    let mut bottom_row = neutral2();
    bottom_row[A_MID][3][0] = 1.0e-7;
    refused(&ragdoll, &n1, 3, &animation, &jph(&bottom_row), 6);
    let mut not_finite = neutral1();
    not_finite[R_TIP][1][2] = f64::NAN;
    refused(&ragdoll, &jph(&not_finite), 3, &animation, &n2, 6);

    // A second initialisation is refused and leaves the mapping as it was.
    let fixture = Fixture::new();
    let before = fixture.map(&pose1(), &local2()).unwrap().bits();
    let mut other = neutral2();
    other[A_TIP] = translation([0.3, 1.0, 0.0]);
    assert!(!fixture
        .mapper
        .initialize(&ragdoll, &n1, 3, &animation, &jph(&other), 6));
    assert_eq!(fixture.map(&pose1(), &local2()).unwrap().bits(), before);
}

#[test]
fn lock_translations2_refuses_a_root_and_changes_nothing() {
    let fixture = Fixture::new();
    let before = fixture.map(&pose1(), &local2()).unwrap().bits();
    let mut mask = [false; 6];
    mask[A_BASE] = true;
    mask[A_TIP_END] = true;
    assert!(!fixture
        .mapper
        .lock(&fixture.animation, &mask, &fixture.neutral2));
    assert!((0..6).all(|joint| !fixture.mapper.locked(joint)));
    assert_eq!(fixture.map(&pose1(), &local2()).unwrap().bits(), before);

    // Before initialisation.
    let fresh = Fixture::uninitialized();
    let mut tip_end = [false; 6];
    tip_end[A_TIP_END] = true;
    assert!(!fresh
        .mapper
        .lock(&fresh.animation, &tip_end, &fresh.neutral2));
    assert!(!fresh.mapper.locked(A_TIP_END as i32));

    // A second lock call.
    assert!(fixture
        .mapper
        .lock(&fixture.animation, &tip_end, &fixture.neutral2));
    assert!(fixture.mapper.locked(A_TIP_END as i32));
    let mut mid_a = [false; 6];
    mid_a[A_MID_A] = true;
    assert!(!fixture
        .mapper
        .lock(&fixture.animation, &mid_a, &fixture.neutral2));
    assert!(!fixture.mapper.locked(A_MID_A as i32));
}

#[test]
fn lock_all_translations2_condition() {
    let fixture = Fixture::new();
    assert!(fixture
        .mapper
        .lock_all(&fixture.animation, &fixture.neutral2));
    let locked: Vec<bool> = (0..6).map(|joint| fixture.mapper.locked(joint)).collect();
    assert_eq!(locked, [false, false, true, true, true, true]);
    assert!(!fixture
        .mapper
        .lock_all(&fixture.animation, &fixture.neutral2));

    // The first mapping's joint (1) is outside a one-joint skeleton.
    let other = Fixture::new();
    let single = Skeleton::new(&[("base", -1)]);
    assert!(!other
        .mapper
        .lock_all(&single, &jph(&[translation([0.0; 3])])));
    assert!((0..6).all(|joint| !other.mapper.locked(joint)));
    // Before initialisation.
    let fresh = Fixture::uninitialized();
    assert!(!fresh.mapper.lock_all(&fresh.animation, &fresh.neutral2));
}

/// A ragdoll pose with unrotated `root` at `root` and `mid` at `mid`.
fn chain_pose(root: [f64; 3], mid: [f64; 3]) -> Vec<M> {
    vec![
        translation(root),
        translation(mid),
        translation([0.1, 1.0, 0.0]),
    ]
}

/// An animation local pose whose root -> mid_a -> mid chain is two unrotated steps along `step`.
fn chain_local(step: [f64; 3]) -> Vec<M> {
    let mut local = neutral_local();
    local[A_MID_A] = translation(step);
    local[A_MID] = translation(step);
    local
}

fn neutral_local() -> Vec<M> {
    let model = neutral2();
    let parents = [None, Some(0), Some(1), Some(2), Some(3), Some(4)];
    (0..6)
        .map(|j| match parents[j] {
            None => model[j],
            Some(parent) => mul(&inverse(&model[parent]), &model[j]),
        })
        .collect()
}

#[test]
fn map2_refuses_degenerate_chains_and_writes_nothing() {
    let fixture = Fixture::new();
    let mid = R_MID as i32;
    let local = neutral_local();

    // A tiny non-zero desired direction along the actual one: Jolt's sFromTo would square it to
    // zero and normalise a quaternion of length zero.
    let tiny = chain_pose([0.0; 3], [0.0, 1.0e-30, 0.0]);
    assert_ne!(
        to_jph(&tiny[R_MID]).column[3].y - to_jph(&tiny[R_ROOT]).column[3].y,
        0.0
    );
    fixture.assert_map_refused(&tiny, &local, mid);

    // A tiny non-zero actual direction: the chain start at exactly the model origin (its direct
    // translation cancelled by the ragdoll root's), two steps of 1e-30 m.
    let start = fixture
        .map(&chain_pose([0.0; 3], [0.0, 0.5, 0.0]), &local)
        .unwrap()
        .to_vec()[A_ROOT]
        .column[3];
    let origin = [-start.x, -start.y, -start.z].map(f64::from);
    let tiny_actual = chain_pose(origin, [0.0, 0.5, 0.0]);
    let tiny_local = chain_local([0.0, 1.0e-30, 0.0]);
    fixture.assert_map_refused(&tiny_actual, &tiny_local, mid);
    // A short actual direction far below the floor.
    let mut short_local = chain_local([0.0; 3]);
    short_local[A_MID] = translation([1.0e-6, 0.0, 0.0]);
    fixture.assert_map_refused(&pose1(), &short_local, mid);

    let below = f64::from(MIN_CHAIN_LENGTH) * 0.999;
    let above = f64::from(MIN_CHAIN_LENGTH) * 1.001;
    // The chain start of an unrotated ragdoll root turns +Y into +Y, so the actual direction is
    // two steps along +Y.
    let up = |length: f64| chain_local([0.0, length / 2.0, 0.0]);
    for sign in [1.0, -1.0] {
        let desired = |length: f64| chain_pose([0.0; 3], [1.0e-6, sign * length, 0.0]);
        fixture.assert_map_refused(&desired(below), &up(0.5), mid);
        fixture.assert_map_refused(&desired(0.5), &up(below), mid);
        assert!(fixture.map(&desired(above), &up(0.5)).is_some());
        assert!(fixture.map(&desired(0.5), &up(above)).is_some());
        assert!(fixture.map(&desired(above), &up(above)).is_some());
    }
    // Exactly zero desired: Jolt's sFromTo returns identity.
    let zero = chain_pose([0.2, 0.3, 0.4], [0.2, 0.3, 0.4]);
    assert!(fixture.map(&zero, &up(0.5)).is_some());
    // An exactly zero actual direction is refused too.
    fixture.assert_map_refused(&pose1(), &chain_local([0.0; 3]), mid);
}

/// A ragdoll `root -> tip` one metre along +Z over an animation chain of `links` links of equal
/// length between the two, all unrotated in the neutral poses.
fn straight_chain(links: usize) -> (Skeleton, Skeleton, Vec<M>, Mapper) {
    let ragdoll = Skeleton::new(&[("root", -1), ("tip", 0)]);
    let names: Vec<String> = (1..links).map(|i| format!("link_{i}")).collect();
    let mut joints = vec![("root", -1)];
    joints.extend(
        names
            .iter()
            .zip(0..)
            .map(|(name, parent)| (name.as_str(), parent)),
    );
    joints.push(("tip", links as i32 - 1));
    let animation = Skeleton::new(&joints);
    let neutral1 = vec![translation([0.0; 3]), translation([0.0, 0.0, 1.0])];
    let neutral2: Vec<M> = (0..=links)
        .map(|i| translation([0.0, 0.0, i as f64 / links as f64]))
        .collect();
    let mapper = Mapper::new();
    let count2 = links as u32 + 1;
    assert!(mapper.initialize(
        &ragdoll,
        &jph(&neutral1),
        2,
        &animation,
        &jph(&neutral2),
        count2
    ));
    (ragdoll, animation, neutral1, mapper)
}

#[test]
fn map2_accepts_a_long_twisting_chain() {
    // Every link turns 45 degrees about the chain's own axis: the chain still ends one metre up,
    // so Jolt turns its start by nothing. Rotations multiplied along the chain stay rotations.
    const LINKS: usize = 128;
    let (_ragdoll, _animation, pose1, mapper) = straight_chain(LINKS);
    let twist = quat_about([0.0, 0.0, 1.0], std::f64::consts::FRAC_PI_4);
    let mut local = vec![translation([0.0; 3])];
    local.extend((0..LINKS).map(|_| rigid(twist, [0.0, 0.0, 1.0 / LINKS as f64])));
    let mut out = Matrices::zeroed(LINKS + 1);
    let result = mapper.map(&jph(&pose1), 2, &jph(&local), LINKS as u32 + 1, &mut out);
    assert_eq!(result, (true, -1));

    let out: Vec<M> = out.to_vec().iter().map(from_jph).collect();
    let mut expected = pose1[0];
    assert_close(&out[0], &expected, 1e-6);
    for joint in 1..LINKS {
        expected = mul(&expected, &local[joint]);
        assert_close(&out[joint], &expected, 1e-4);
    }
    assert_close(&out[LINKS], &pose1[1], 1e-6);
}

#[test]
fn map2_bounds_long_actual_directions_against_a_zero_desired_one() {
    // A two-link chain of `length` metres along +Z, with the ragdoll's root and tip together.
    let (_ragdoll, _animation, _pose1, mapper) = straight_chain(2);
    let together = [translation([0.0; 3]), translation([0.0; 3])];
    let map = |pose1: &[M], length: f64| {
        let local = [
            translation([0.0; 3]),
            translation([0.0, 0.0, length / 2.0]),
            translation([0.0, 0.0, length / 2.0]),
        ];
        let mut out = Matrices::filled(3, 7.0);
        let before = out.bits();
        let result = mapper.map(&jph(pose1), 2, &jph(&local), 3, &mut out);
        if !result.0 {
            assert_eq!(out.bits(), before, "a refused map wrote its output");
        }
        result
    };
    // Jolt squares the actual length alone when the desired one is zero: 1e20 m would square
    // to infinity, which then multiplies zero. The guard keeps the length within 1e18 m.
    assert_eq!(map(&together, 1.0e20), (false, 1));
    assert_eq!(map(&together, 2.0e18), (false, 1));
    assert_eq!(map(&together, 5.0e17), (true, -1));
    // With a desired direction, the product of the lengths is bounded as well, and the desired
    // length on its own (2e18 m beside a 1 cm chain squares to 4e36, 1e20 m to infinity).
    let apart = |length: f64| [translation([0.0; 3]), translation([0.0, 0.0, length])];
    assert_eq!(map(&apart(1.0), 5.0e17), (true, -1));
    assert_eq!(map(&apart(10.0), 5.0e17), (false, 1));
    assert_eq!(map(&apart(1.0e17), 0.01), (true, -1));
    assert_eq!(map(&apart(2.0e18), 0.01), (false, 1));
}

#[test]
fn map2_bounds_the_magnitudes_of_a_chains_products() {
    // Links that scale by 1e10 instead of turning, the first one metre long. Jolt's values stay
    // finite for three of them (1e30), but their sums of absolute terms (3e30) pass the bound,
    // which keeps a factor of 3e8 below f32::MAX.
    let scaling = |length: f64| {
        let mut m = translation([0.0, 0.0, length]);
        (0..3).for_each(|i| m[i][i] = 1.0e10);
        m
    };
    for (links, expected) in [(2, (true, -1)), (3, (false, 1))] {
        let (_ragdoll, _animation, pose1, mapper) = straight_chain(links);
        let mut local = vec![translation([0.0; 3]), scaling(1.0)];
        local.extend((1..links).map(|_| scaling(0.0)));
        let mut out = Matrices::zeroed(links + 1);
        let result = mapper.map(&jph(&pose1), 2, &jph(&local), links as u32 + 1, &mut out);
        assert_eq!(result, expected, "{links} links");
    }
}

#[test]
fn map2_refuses_a_joint_mapped_twice() {
    // Two ragdoll joints named "mid" both map to animation joint 3.
    let ragdoll = Skeleton::new(&[("root", -1), ("mid", 0), ("mid", 1)]);
    let animation = Skeleton::animation();
    let mapper = Mapper::new();
    assert!(mapper.initialize(
        &ragdoll,
        &jph(&neutral1()),
        3,
        &animation,
        &jph(&neutral2()),
        6
    ));
    assert_eq!((mapper.mapped(1), mapper.mapped(2)), (3, 3));
    let mut out = Matrices::filled(6, 7.0);
    let before = out.bits();
    let result = mapper.map(&jph(&pose1()), 3, &jph(&local2()), 6, &mut out);
    assert_eq!(result, (false, -1));
    assert_eq!(out.bits(), before);
    // The reverse direction reads each mapping once and accepts it.
    let mut reverse = Matrices::zeroed(3);
    assert!(mapper.map_reverse(&jph(&neutral2()), 6, &mut reverse, 3));
}

#[test]
fn map2_and_map_reverse2_refuse_truncated_counts() {
    let fixture = Fixture::new();
    let (p1, l2) = (jph(&pose1()), jph(&local2()));
    for (count1, count2) in [
        (3, 5),
        (3, 4),
        (3, 3),
        (3, 1),
        (3, 0),
        (2, 6),
        (1, 6),
        (0, 6),
    ] {
        let mut out = Matrices::filled(6, 7.0);
        let before = out.bits();
        assert_eq!(
            fixture.mapper.map(&p1, count1, &l2, count2, &mut out),
            (false, -1),
            "counts {count1}, {count2}"
        );
        assert_eq!(out.bits(), before);
    }
    for (count2, count1) in [(4, 3), (6, 2), (0, 3)] {
        let mut out = Matrices::filled(3, 7.0);
        let before = out.bits();
        assert!(!fixture.mapper.map_reverse(&l2, count2, &mut out, count1));
        assert_eq!(out.bits(), before);
    }

    // Matrices that are not finite or whose bottom row is not (0, 0, 0, 1).
    let mut bad_pose = pose1();
    bad_pose[R_TIP][3][2] = 0.5;
    fixture.assert_map_refused(&bad_pose, &local2(), -1);
    let mut bad_local = local2();
    bad_local[A_TIP_END][0][0] = f64::INFINITY;
    fixture.assert_map_refused(&pose1(), &bad_local, -1);
    let mut out = Matrices::filled(3, 7.0);
    let before = out.bits();
    assert!(!fixture.mapper.map_reverse(&jph(&bad_local), 6, &mut out, 3));
    assert_eq!(out.bits(), before);

    // An uninitialised mapper has no mapping.
    let fresh = Fixture::uninitialized();
    let mut out = Matrices::filled(6, 7.0);
    assert_eq!(fresh.mapper.map(&p1, 3, &l2, 6, &mut out), (false, -1));
    let mut reverse = Matrices::filled(3, 7.0);
    assert!(!fresh.mapper.map_reverse(&l2, 6, &mut reverse, 3));
}

#[test]
fn outputs_may_alias_inputs() {
    let fixture = Fixture::new();
    let expected = fixture.map(&pose1(), &local2()).unwrap().bits();
    let mut in_place = jph(&local2());
    let local = in_place.as_mut_ptr();
    let mut degenerate = 0;
    // SAFETY: live mapper; the pose holds 3 matrices and `in_place` 6, which Map2 reads in full
    // before it writes the output through the same pointer.
    let ok = unsafe {
        JPH_SkeletonMapper_Map2(
            fixture.mapper.0,
            jph(&pose1()).as_ptr(),
            3,
            local,
            6,
            local,
            &mut degenerate,
        )
    };
    assert!(ok);
    assert_eq!(in_place.bits(), expected);
}

#[test]
fn map_reverse2_writes_identity_for_unmapped_joints_and_may_alias() {
    // `extra` has no animation joint of its name: Jolt's MapReverse leaves it alone, and the
    // extension writes the identity there.
    let ragdoll = Skeleton::new(&[("root", -1), ("mid", 0), ("extra", 1)]);
    let animation = Skeleton::animation();
    let mapper = Mapper::new();
    assert!(mapper.initialize(
        &ragdoll,
        &jph(&neutral1()),
        3,
        &animation,
        &jph(&neutral2()),
        6
    ));
    assert_eq!(mapper.mapped(2), -1);
    let pose2 = jph(&neutral2());
    let mut out = Matrices::filled(3, 7.0);
    assert!(mapper.map_reverse(&pose2, 6, &mut out, 3));
    let out: Vec<M> = out.to_vec().iter().map(from_jph).collect();
    assert_close(&out[R_ROOT], &neutral1()[R_ROOT], 1e-6);
    assert_close(&out[R_MID], &neutral1()[R_MID], 1e-6);
    assert_close(&out[2], &translation([0.0; 3]), 0.0);

    // The output written over the input it was mapped from.
    let mut expected = Matrices::zeroed(3);
    let fixture = Fixture::new();
    let input = jph(&local2());
    assert!(fixture.mapper.map_reverse(&input, 6, &mut expected, 3));
    let mut in_place = jph(&local2());
    let pointer = in_place.as_mut_ptr();
    // SAFETY: live mapper; `in_place` holds 6 matrices, which MapReverse2 reads in full before it
    // writes 3 through the same pointer.
    let ok = unsafe { JPH_SkeletonMapper_MapReverse2(fixture.mapper.0, pointer, 6, pointer, 3) };
    assert!(ok);
    assert_eq!(in_place.bits()[..48], expected.bits()[..]);
    assert_eq!(in_place.bits()[48..], input.bits()[48..]);
}
