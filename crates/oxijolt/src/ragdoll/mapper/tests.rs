use super::*;
use crate::limits;
use crate::ragdoll::SkeletonJoint;

/// A unit quaternion within Rust's tolerance (squares summed left to right: 1.0000099) and
/// outside Jolt's (pairwise sums: 1.00001), so `Mat44::sRotation` asserts on it unless it is
/// normalised first.
const EDGE_ROTATION: Quat =
    Quat::from_xyzw(0.641_428_5, -0.449_664_26, 0.383_378_86, -0.489_287_4);

fn joint(name: &str, parent: Option<u32>) -> SkeletonJoint<'_> {
    SkeletonJoint { name, parent }
}

/// hips <- chest.
fn ragdoll() -> Skeleton {
    Skeleton::new(&[joint("hips", None), joint("chest", Some(0))]).unwrap()
}

/// root <- hips <- spine <- chest: an extra root and one joint between the ragdoll's two.
fn animation() -> Skeleton {
    Skeleton::new(&[
        joint("root", None),
        joint("hips", Some(0)),
        joint("spine", Some(1)),
        joint("chest", Some(2)),
    ])
    .unwrap()
}

fn transform(y: f32) -> JointTransform {
    JointTransform {
        translation: Vec3::new(0.0, y, 0.0),
        rotation: Quat::IDENTITY,
    }
}

fn pose(heights: &[f32]) -> SkeletonPose {
    SkeletonPose {
        root_offset: RVec3::ZERO,
        joints: heights.iter().map(|&y| transform(y)).collect(),
    }
}

fn ragdoll_neutral() -> SkeletonPose {
    pose(&[1.0, 1.5])
}

fn animation_neutral() -> SkeletonPose {
    pose(&[0.0, 1.0, 1.25, 1.5])
}

fn animation_local() -> Vec<JointTransform> {
    [0.0, 1.0, 0.25, 0.25].map(transform).to_vec()
}

fn mapper_with(
    ragdoll: (&Skeleton, &SkeletonPose),
    animation: (&Skeleton, &SkeletonPose),
    locks: TranslationLocks<'_>,
) -> Result<SkeletonMapper, RagdollError> {
    SkeletonMapper::new(
        MappedSkeleton {
            skeleton: ragdoll.0,
            neutral_pose: ragdoll.1,
        },
        MappedSkeleton {
            skeleton: animation.0,
            neutral_pose: animation.1,
        },
        locks,
    )
}

fn mapper(locks: TranslationLocks<'_>) -> Result<SkeletonMapper, RagdollError> {
    mapper_with(
        (&ragdoll(), &ragdoll_neutral()),
        (&animation(), &animation_neutral()),
        locks,
    )
}

#[track_caller]
fn assert_invalid<T: std::fmt::Debug>(result: Result<T, RagdollError>) {
    assert!(
        matches!(result, Err(RagdollError::InvalidValue(_))),
        "{result:?}"
    );
}

impl std::fmt::Debug for SkeletonMapper {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("SkeletonMapper")
    }
}

#[test]
fn a_mapper_matches_joints_by_name() {
    let mapper = mapper(TranslationLocks::None).unwrap();
    assert_eq!(
        (mapper.ragdoll_joint_count(), mapper.animation_joint_count()),
        (2, 4)
    );
    assert_eq!(mapper.mapped_joint(0), Some(1));
    assert_eq!(mapper.mapped_joint(1), Some(3));
    assert_eq!(mapper.mapped_joint(2), None);
    assert!((0..5).all(|joint| !mapper.is_translation_locked(joint)));
}

#[test]
fn unmatched_names_and_hierarchies_are_refused() {
    let neutral = animation_neutral();
    let without_chest = Skeleton::new(&[
        joint("root", None),
        joint("hips", Some(0)),
        joint("spine", Some(1)),
        joint("neck", Some(2)),
    ])
    .unwrap();
    let result = mapper_with(
        (&ragdoll(), &ragdoll_neutral()),
        (&without_chest, &neutral),
        TranslationLocks::None,
    );
    assert_eq!(result.err(), Some(RagdollError::UnmappedJoint(1)));

    // The chest beside the hips instead of below them.
    let beside = Skeleton::new(&[
        joint("root", None),
        joint("hips", Some(0)),
        joint("spine", Some(1)),
        joint("chest", Some(0)),
    ])
    .unwrap();
    let result = mapper_with(
        (&ragdoll(), &ragdoll_neutral()),
        (&beside, &neutral),
        TranslationLocks::None,
    );
    assert_eq!(result.err(), Some(RagdollError::HierarchyMismatch(1)));

    // The ragdoll root below another mapped joint.
    let upside_down = Skeleton::new(&[
        joint("chest", None),
        joint("hips", Some(0)),
        joint("spine", Some(1)),
        joint("root", Some(2)),
    ])
    .unwrap();
    let result = mapper_with(
        (&ragdoll(), &ragdoll_neutral()),
        (&upside_down, &neutral),
        TranslationLocks::None,
    );
    assert_eq!(result.err(), Some(RagdollError::HierarchyMismatch(0)));

    // An animation skeleton with fewer joints than the ragdoll's cannot match every name.
    let small = Skeleton::new(&[joint("hips", None)]).unwrap();
    let result = mapper_with(
        (&ragdoll(), &ragdoll_neutral()),
        (&small, &pose(&[1.0])),
        TranslationLocks::None,
    );
    assert_eq!(result.err(), Some(RagdollError::UnmappedJoint(1)));
}

#[test]
fn neutral_poses_are_validated() {
    let (ragdoll, animation) = (ragdoll(), animation());
    let build = |ragdoll_neutral: &SkeletonPose, animation_neutral: &SkeletonPose| {
        mapper_with(
            (&ragdoll, ragdoll_neutral),
            (&animation, animation_neutral),
            TranslationLocks::None,
        )
    };
    assert!(build(&ragdoll_neutral(), &animation_neutral()).is_ok());
    assert_invalid(build(&pose(&[1.0]), &animation_neutral()));
    assert_invalid(build(&ragdoll_neutral(), &pose(&[0.0, 1.0, 1.5])));
    let mut nan = animation_neutral();
    nan.joints[2].translation.x = f32::NAN;
    assert_invalid(build(&ragdoll_neutral(), &nan));
    let mut not_unit = ragdoll_neutral();
    not_unit.joints[1].rotation = Quat::from_xyzw(0.0, 0.0, 0.0, 1.1);
    assert_invalid(build(&not_unit, &animation_neutral()));
    let mut out_of_frame = animation_neutral();
    out_of_frame.root_offset.x = 2.0 * limits::MAX_POSITION;
    assert_invalid(build(&ragdoll_neutral(), &out_of_frame));

    // Root offsets at opposite edges of the frame: re-expressed translations of 2 *
    // MAX_POSITION.
    let mut far_ragdoll = ragdoll_neutral();
    far_ragdoll.root_offset.x = -limits::MAX_POSITION;
    let mut far_animation = animation_neutral();
    far_animation.root_offset.x = limits::MAX_POSITION;
    let mapper = build(&far_ragdoll, &far_animation).unwrap();
    let mut far_pose = far_ragdoll.clone();
    far_pose.joints[1].translation.y = 1.6;
    assert!(mapper.map(&far_pose, &animation_local()).is_ok());
}

#[test]
fn locks_are_validated() {
    assert!(mapper(TranslationLocks::Joints(&[2, 3, 3])).is_ok());
    assert!(mapper(TranslationLocks::Joints(&[])).is_ok());
    assert_invalid(mapper(TranslationLocks::Joints(&[0])));
    assert_invalid(mapper(TranslationLocks::Joints(&[2, 1])));
    assert_invalid(mapper(TranslationLocks::Joints(&[4])));
    assert_invalid(mapper(TranslationLocks::Joints(&[u32::MAX])));

    let locked = mapper(TranslationLocks::Joints(&[2, 3, 3])).unwrap();
    assert_eq!(
        (0..4)
            .map(|joint| locked.is_translation_locked(joint))
            .collect::<Vec<_>>(),
        [false, false, true, true]
    );
    let all = mapper(TranslationLocks::All).unwrap();
    assert_eq!(
        (0..4)
            .map(|joint| all.is_translation_locked(joint))
            .collect::<Vec<_>>(),
        [false, false, true, true]
    );
}

#[test]
fn lock_masks_flag_each_listed_joint() {
    assert_eq!(
        lock_mask(&[3, 2, 3], 1, 4),
        Ok(vec![false, false, true, true])
    );
    assert_eq!(lock_mask(&[], 1, 3), Ok(vec![false; 3]));
    assert_invalid(lock_mask(&[0], 1, 4));
    assert_invalid(lock_mask(&[1], 1, 4));
    assert_invalid(lock_mask(&[4], 1, 4));
}

#[test]
fn poses_are_validated_per_call() {
    let mapper = mapper(TranslationLocks::None).unwrap();
    assert!(mapper.map(&ragdoll_neutral(), &animation_local()).is_ok());
    assert_invalid(mapper.map(&pose(&[1.0]), &animation_local()));
    assert_invalid(mapper.map(&ragdoll_neutral(), &animation_local()[..3]));
    let mut local = animation_local();
    local[2].rotation = Quat::from_xyzw(0.0, 0.0, 0.0, 0.5);
    assert_invalid(mapper.map(&ragdoll_neutral(), &local));
    let mut local = animation_local();
    local[2].translation.z = f32::INFINITY;
    assert_invalid(mapper.map(&ragdoll_neutral(), &local));

    assert!(mapper.map_reverse(&animation_neutral()).is_ok());
    assert_invalid(mapper.map_reverse(&ragdoll_neutral()));
}

#[test]
fn rotations_at_the_edge_of_the_tolerance_are_normalised() {
    assert!(EDGE_ROTATION.is_valid_rotation());
    let mut ragdoll_neutral = ragdoll_neutral();
    ragdoll_neutral.joints[0].rotation = EDGE_ROTATION;
    let mut animation_neutral = animation_neutral();
    animation_neutral.joints[1].rotation = EDGE_ROTATION;
    let mapper = mapper_with(
        (&ragdoll(), &ragdoll_neutral),
        (&animation(), &animation_neutral),
        TranslationLocks::All,
    )
    .unwrap();

    let mut ragdoll_pose = ragdoll_neutral.clone();
    ragdoll_pose.joints[1].rotation = EDGE_ROTATION;
    let mut local = animation_local();
    local[2].rotation = EDGE_ROTATION;
    let mapped = mapper.map(&ragdoll_pose, &local).unwrap();
    assert_eq!(mapped.joints.len(), 4);

    let mut animation_pose = animation_neutral.clone();
    animation_pose.joints[3].rotation = EDGE_ROTATION;
    let reversed = mapper.map_reverse(&animation_pose).unwrap();
    assert_eq!(reversed.joints.len(), 2);
}

#[test]
fn a_mapper_is_send_and_sync() {
    fn assert_send_sync<T: Send + Sync>() {}
    assert_send_sync::<SkeletonMapper>();
}
