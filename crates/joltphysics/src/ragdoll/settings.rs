//! Skeletons, ragdoll settings and poses: immutable values built and checked before a ragdoll
//! is created from them.

use std::collections::BTreeSet;
use std::ffi::CString;

use joltphysics_sys::*;

use crate::body::{has_finite_inverse, mass_properties, CreationSettings};
use crate::owned::{JoltObject, Owned};
use crate::world::ensure_initialized;
use crate::{
    BodyError, BodySettings, HingeConstraintSettings, MotionType, ObjectLayer, Quat, RVec3,
    RagdollError, Real, Shape, SixDofConstraintSettings, SwingTwistConstraintSettings, Vec3,
};

/// A skeleton, owned with the one reference `JPH_Skeleton_Create` returns. Ragdoll settings hold
/// their own reference.
impl JoltObject for JPH_Skeleton {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner holds one reference (trait contract), which this releases.
        unsafe { JPH_Skeleton_Destroy(ptr) };
    }
}

/// Ragdoll settings, owned with the one reference `JPH_RagdollSettings_Create` returns. Every
/// ragdoll created from them holds its own reference.
impl JoltObject for JPH_RagdollSettings {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner holds one reference (trait contract), which this releases.
        unsafe { JPH_RagdollSettings_Destroy(ptr) };
    }
}

/// A group filter table, owned with the one reference `JPH_GroupFilterTable_Create` returns.
/// Every part's collision group holds its own reference.
impl JoltObject for JPH_GroupFilterTable {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner holds one reference (trait contract). `GroupFilterTable` derives from
        // `GroupFilter` with single inheritance, as joltc's header notes, so the pointer is the
        // `GroupFilter` that `Release` acts on.
        unsafe { JPH_GroupFilter_Destroy(ptr.cast()) };
    }
}

/// One joint of a [`Skeleton`]: its name and the index of its parent joint.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SkeletonJoint<'a> {
    /// The joint's name, unique within the skeleton and without NUL characters.
    pub name: &'a str,
    /// The parent joint's index, which is lower than this joint's; `None` for the root, joint 0.
    pub parent: Option<u32>,
}

/// A skeleton (Jolt `Skeleton`): named joints in a tree, parents before children, joint 0 the
/// root. A ragdoll has one part per joint.
///
/// Never changes after [`new`](Self::new); one skeleton may serve any number of ragdoll settings.
pub struct Skeleton {
    skeleton: Owned<JPH_Skeleton>,
    names: Vec<String>,
    parents: Vec<Option<u32>>,
}

// SAFETY: the Jolt skeleton is never changed after `new`, and `RefTarget` counts references
// atomically, so it may be used and released from any thread
// (https://jrouwe.github.io/JoltPhysicsDocs/5.6.0/index.html#memory-management).
unsafe impl Send for Skeleton {}
// SAFETY: as for `Send`; `&Skeleton` only lets ragdoll settings take further references.
unsafe impl Sync for Skeleton {}

impl Skeleton {
    /// Largest joint count. Jolt's `CreateRagdoll` allocates one pointer per part on the stack,
    /// so the count is bounded here; not a Jolt limit.
    pub const MAX_JOINTS: usize = 1024;

    /// Builds a skeleton from `joints` in order. Fails with [`RagdollError::InvalidValue`]
    /// unless there are 1 to [`MAX_JOINTS`](Self::MAX_JOINTS) joints, joint 0 is the only root,
    /// every other parent precedes its child, and the names are unique and free of NUL.
    pub fn new(joints: &[SkeletonJoint<'_>]) -> Result<Self, RagdollError> {
        let invalid = |what| Err(RagdollError::InvalidValue(what));
        if joints.is_empty() || joints.len() > Self::MAX_JOINTS {
            return invalid("a skeleton has between 1 and 1024 joints");
        }
        let mut names = BTreeSet::new();
        let mut c_names = Vec::with_capacity(joints.len());
        for (index, joint) in joints.iter().enumerate() {
            let valid_parent = match joint.parent {
                None => index == 0,
                Some(parent) => (parent as usize) < index,
            };
            if !valid_parent {
                return invalid("joint 0 is the only root and every parent precedes its child");
            }
            if !names.insert(joint.name) {
                return invalid("skeleton joint names must be unique");
            }
            let Ok(name) = CString::new(joint.name) else {
                return invalid("skeleton joint names must not contain NUL");
            };
            c_names.push(name);
        }
        if !ensure_initialized() {
            return Err(RagdollError::InitFailed);
        }
        // SAFETY: Jolt is initialised. The handle takes over the one reference joltc returns.
        let skeleton = unsafe { Owned::from_raw(JPH_Skeleton_Create()) }
            .unwrap_or_else(|| unreachable!("joltc `new`s the skeleton"));
        for (joint, name) in joints.iter().zip(&c_names) {
            // `MAX_JOINTS` bounds every index to `i32`.
            let parent = joint.parent.map_or(-1, |parent| parent as i32);
            // SAFETY: the skeleton is live and only this function uses it; `name` is a
            // NUL-terminated string that Jolt copies, and `parent` is -1 or a joint added before.
            unsafe { JPH_Skeleton_AddJoint2(skeleton.as_ptr(), name.as_ptr(), parent) };
        }
        Ok(Self {
            skeleton,
            names: joints.iter().map(|joint| joint.name.to_owned()).collect(),
            parents: joints.iter().map(|joint| joint.parent).collect(),
        })
    }

    /// Number of joints.
    pub fn joint_count(&self) -> u32 {
        self.names.len() as u32
    }

    /// Joint `index`, `None` when there is no such joint.
    pub fn joint(&self, index: u32) -> Option<SkeletonJoint<'_>> {
        let index = index as usize;
        Some(SkeletonJoint {
            name: self.names.get(index)?,
            parent: self.parents[index],
        })
    }

    /// The index of the joint named `name`.
    pub fn joint_index(&self, name: &str) -> Option<u32> {
        self.names
            .iter()
            .position(|joint| joint == name)
            .map(|index| index as u32)
    }
}

/// The constraint that joins a ragdoll part to its parent part.
// A joint is built once per ragdoll settings and copied into Jolt, so its size does not matter;
// boxing the large variant would only make the settings harder to write.
#[allow(clippy::large_enum_variant)]
#[derive(Clone, Debug, PartialEq)]
pub enum RagdollJoint {
    /// A swing-twist joint: spine, neck, shoulders.
    SwingTwist(SwingTwistConstraintSettings),
    /// A hinge: elbows, knees.
    Hinge(HingeConstraintSettings),
    /// A six-degree-of-freedom joint with asymmetric swing limits: hips.
    SixDof(SixDofConstraintSettings),
}

impl RagdollJoint {
    fn kind(&self) -> JointKind {
        match self {
            Self::SwingTwist(_) => JointKind::SwingTwist,
            Self::Hinge(_) => JointKind::Hinge,
            Self::SixDof(_) => JointKind::SixDof,
        }
    }

    fn validate(&self) -> Result<(), &'static str> {
        match self {
            Self::SwingTwist(settings) => settings.validate(),
            Self::Hinge(settings) => settings.validate(),
            Self::SixDof(settings) => settings.validate(),
        }
    }
}

/// The kind of constraint between a part and its parent.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum JointKind {
    SwingTwist,
    Hinge,
    SixDof,
}

impl JointKind {
    /// Jolt's subtype of the constraint.
    pub(crate) fn sub_type(self) -> JPH_ConstraintSubType {
        match self {
            Self::SwingTwist => JPH_ConstraintSubType_SwingTwist,
            Self::Hinge => JPH_ConstraintSubType_Hinge,
            Self::SixDof => JPH_ConstraintSubType_SixDOF,
        }
    }
}

/// One part of a ragdoll: a body for one skeleton joint and the joint to its parent part.
///
/// `body` gives the part's motion type (dynamic or kinematic), object layer, mass, friction,
/// damping, gravity factor, initial velocities and sleeping; its position and rotation are the
/// part's bind pose in world space, and the joint frames are given in that same pose. Its
/// activation is ignored: [`PhysicsWorld::create_ragdoll`](crate::PhysicsWorld::create_ragdoll)
/// takes one for the whole ragdoll.
#[derive(Clone)]
pub struct RagdollPart<'a> {
    /// The part's collision shape. The settings keep their own reference.
    pub shape: &'a Shape,
    /// The part's body settings.
    pub body: BodySettings,
    /// The joint to the parent part; `None` exactly for the root part.
    pub joint: Option<RagdollJoint>,
}

/// The settings of a ragdoll (Jolt `RagdollSettings`): a skeleton, one body per joint and the
/// constraints between them.
///
/// Parts of one ragdoll never collide with each other: each part is in its own sub group of a
/// group filter that disables every pair, and each ragdoll gets its own group, so different
/// ragdolls still collide. Joint priorities grow toward the root (Jolt
/// `CalculateConstraintPriorities`).
///
/// # Joint limits on impact
/// Joints do not stay within their limits on every tick. In each solver iteration Jolt solves
/// contacts after constraints, so when a ragdoll hits the ground the contacts win and joints pass
/// their limits for a few dozen ticks; hinges also bend about their fixed axes, and a small error
/// can remain at rest. How far depends on the
/// ragdoll and the fall. In the repository's drop test (a 12-part humanoid dropped with its pelvis
/// 1.5 m above a heightfield) the worst overshoot is 0.29 rad and 0.0037 rad remain at rest; the
/// test's bounds, 0.40 rad during the fall and 0.01 rad at rest, hold for that drop only. Over a
/// sweep of 126 drops of that humanoid the overshoot reached 0.48 rad, and up to 0.15 rad
/// remained when the ragdoll came to rest. Thin parts falling fast can also sink into the ground
/// with
/// [`MotionQuality::Discrete`](crate::MotionQuality::Discrete); use
/// [`MotionQuality::LinearCast`](crate::MotionQuality::LinearCast) on them for high falls.
///
/// The settings hold one reference to the skeleton, the parts' shapes and the group filter;
/// every ragdoll created from them holds one reference to the settings, so they may be dropped
/// while ragdolls live. Never changes after construction; one value may create ragdolls in any
/// number of worlds.
pub struct RagdollSettings {
    settings: Owned<JPH_RagdollSettings>,
    parents: Vec<Option<u32>>,
    joints: Vec<Option<JointKind>>,
    object_layers: Vec<ObjectLayer>,
}

// SAFETY: the Jolt settings are never changed after construction, `CreateRagdoll` only reads them
// (it is `const`), and `RefTarget` counts the references to the settings, shapes, constraint
// settings and group filter atomically
// (https://jrouwe.github.io/JoltPhysicsDocs/5.6.0/index.html#memory-management).
unsafe impl Send for RagdollSettings {}
// SAFETY: as for `Send`; `&RagdollSettings` only creates ragdolls, which reads the settings.
unsafe impl Sync for RagdollSettings {}

impl RagdollSettings {
    /// Settings with one part per joint of `skeleton`, in joint order, keeping each part's mass.
    ///
    /// Fails with [`RagdollError::InvalidValue`] when the part count differs from the joint count,
    /// when the root part has a joint or another part has none, when a part is static, uses a
    /// shape only static bodies may use (a heightfield) or has a mass or inertia too small for
    /// Jolt to invert, or when a body or joint setting is out of range. Object layers are checked
    /// when a ragdoll is created.
    pub fn new(skeleton: &Skeleton, parts: &[RagdollPart<'_>]) -> Result<Self, RagdollError> {
        Self::build(skeleton, parts, false)
    }

    /// Like [`new`](Self::new), then Jolt's `Stabilize`: it keeps the total mass of every chain
    /// but bounds each parent/child mass ratio to `[0.8, 1.2]` and raises parent inertias, which
    /// makes the joints stiffer. The parts' masses change.
    pub fn new_stabilized(
        skeleton: &Skeleton,
        parts: &[RagdollPart<'_>],
    ) -> Result<Self, RagdollError> {
        Self::build(skeleton, parts, true)
    }

    fn build(
        skeleton: &Skeleton,
        parts: &[RagdollPart<'_>],
        stabilize: bool,
    ) -> Result<Self, RagdollError> {
        validate_parts(skeleton, parts)?;
        // SAFETY: Jolt is initialised (the skeleton exists). The handles take over the one
        // reference joltc returns for the settings and the table.
        let (settings, table) = unsafe {
            (
                Owned::from_raw(JPH_RagdollSettings_Create())
                    .unwrap_or_else(|| unreachable!("joltc `new`s the settings")),
                Owned::from_raw(JPH_GroupFilterTable_Create(parts.len() as u32))
                    .unwrap_or_else(|| unreachable!("joltc `new`s the table")),
            )
        };
        let count = parts.len() as u32;
        // SAFETY: the settings, the skeleton and the table are live; the settings take their own
        // reference to the skeleton. `count` is at most `MAX_JOINTS`, so the cast is exact and
        // every sub group is below the table's size.
        unsafe {
            JPH_RagdollSettings_SetSkeleton(settings.as_ptr(), skeleton.skeleton.as_ptr());
            JPH_RagdollSettings_ResizeParts(settings.as_ptr(), count as i32);
            for a in 0..count {
                for b in a + 1..count {
                    JPH_GroupFilterTable_DisableCollision(table.as_ptr(), a, b);
                }
            }
        }
        for (index, part) in parts.iter().enumerate() {
            let creation =
                CreationSettings::new(part.shape, &part.body).map_err(RagdollError::Body)?;
            let group = JPH_CollisionGroup {
                groupFilter: table.as_ptr().cast(),
                groupID: 0,
                subGroupID: index as u32,
            };
            // SAFETY: the settings, the creation settings and the table are live, and `index` is
            // below the part count. The part copies the creation settings with their own shape
            // and group filter references; each constraint setter creates Jolt settings the part
            // holds the reference of. The joints were validated, so Jolt's asserts hold.
            unsafe {
                JPH_BodyCreationSettings_SetCollisionGroup(creation.as_ptr(), &group);
                JPH_RagdollSettings_SetPart(settings.as_ptr(), index as i32, creation.as_ptr());
                match &part.joint {
                    None => {}
                    Some(RagdollJoint::SwingTwist(joint)) => {
                        JPH_RagdollSettings_SetPartToParentSwingTwist(
                            settings.as_ptr(),
                            index as i32,
                            &joint.to_jph(),
                        )
                    }
                    Some(RagdollJoint::Hinge(joint)) => JPH_RagdollSettings_SetPartToParentHinge(
                        settings.as_ptr(),
                        index as i32,
                        &joint.to_jph(),
                    ),
                    Some(RagdollJoint::SixDof(joint)) => JPH_RagdollSettings_SetPartToParentSixDOF(
                        settings.as_ptr(),
                        index as i32,
                        &joint.to_jph(),
                    ),
                }
            }
        }
        // SAFETY: the settings are live and complete: every part has a body and the skeleton
        // lists parents first, as `Stabilize` and `CalculateConstraintPriorities` assert. The
        // base priority 0 plus the part count cannot overflow.
        unsafe {
            if stabilize && !JPH_RagdollSettings_Stabilize(settings.as_ptr()) {
                return Err(RagdollError::InvalidValue(
                    "Jolt could not stabilize the part inertias",
                ));
            }
            JPH_RagdollSettings_CalculateConstraintPriorities(settings.as_ptr(), 0);
            JPH_RagdollSettings_CalculateBodyIndexToConstraintIndex(settings.as_ptr());
            JPH_RagdollSettings_CalculateConstraintIndexToBodyIdxPair(settings.as_ptr());
        }
        // Every part's collision group holds its own reference to the table; ours goes now.
        drop(table);
        Ok(Self {
            settings,
            parents: skeleton.parents.clone(),
            joints: parts
                .iter()
                .map(|part| part.joint.as_ref().map(RagdollJoint::kind))
                .collect(),
            object_layers: parts.iter().map(|part| part.body.object_layer).collect(),
        })
    }

    /// Number of parts, one per skeleton joint.
    pub fn part_count(&self) -> u32 {
        self.parents.len() as u32
    }

    pub(super) fn as_ptr(&self) -> *mut JPH_RagdollSettings {
        self.settings.as_ptr()
    }

    pub(super) fn parents(&self) -> &[Option<u32>] {
        &self.parents
    }

    pub(super) fn joints(&self) -> &[Option<JointKind>] {
        &self.joints
    }

    pub(super) fn object_layers(&self) -> &[ObjectLayer] {
        &self.object_layers
    }
}

/// Every check of [`RagdollSettings::new`], before Jolt is called.
fn validate_parts(skeleton: &Skeleton, parts: &[RagdollPart<'_>]) -> Result<(), RagdollError> {
    let invalid = |what| Err(RagdollError::InvalidValue(what));
    if parts.len() != skeleton.names.len() {
        return invalid("a ragdoll has exactly one part per skeleton joint");
    }
    for (index, part) in parts.iter().enumerate() {
        if part.joint.is_some() != (index > 0) {
            return invalid("the root part has no joint and every other part has one");
        }
        part.body.validate_values().map_err(|error| match error {
            BodyError::InvalidValue(what) => RagdollError::InvalidValue(what),
            error => RagdollError::Body(error),
        })?;
        if part.body.motion_type == MotionType::Static {
            return invalid("ragdoll parts are dynamic or kinematic");
        }
        // SAFETY: the shape is live for the call; the getter only reads it.
        if unsafe { JPH_Shape_MustBeStatic(part.shape.as_ptr()) } {
            return invalid("this shape can only be used by static bodies");
        }
        if !has_finite_inverse(&mass_properties(part.shape, part.body.mass)) {
            return invalid("mass and shape give an infinite inverse mass or inertia");
        }
        if let Some(joint) = &part.joint {
            joint.validate().map_err(RagdollError::InvalidValue)?;
        }
    }
    Ok(())
}

/// One joint of a [`SkeletonPose`]: a translation relative to the pose's root offset and a world
/// rotation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointTransform {
    /// Position of the part's body origin relative to [`SkeletonPose::root_offset`], metres.
    pub translation: Vec3,
    /// The part's rotation in world space, a unit quaternion.
    pub rotation: Quat,
}

/// A pose of every part of a ragdoll, in joint order (Jolt `Ragdoll::SetPose` and `GetPose`).
///
/// Each part's body origin is at `root_offset + translation` with its world `rotation`. The
/// split into an offset and `f32` translations keeps a pose far from the origin precise in
/// double precision. [`RagdollRef::pose`](crate::RagdollRef::pose) puts the root offset at
/// part 0, so a pose read back equals one set with a nonzero root joint translation in the
/// world transforms it describes, not field by field.
#[derive(Clone, Debug, PartialEq)]
pub struct SkeletonPose {
    /// World position the translations are relative to, metres.
    pub root_offset: RVec3,
    /// One transform per joint.
    pub joints: Vec<JointTransform>,
}

impl SkeletonPose {
    /// The world position of joint `index`'s body origin.
    pub(crate) fn position(&self, index: usize) -> RVec3 {
        let t = self.joints[index].translation;
        RVec3::new(
            self.root_offset.x + Real::from(t.x),
            self.root_offset.y + Real::from(t.y),
            self.root_offset.z + Real::from(t.z),
        )
    }

    /// Checks the pose for a ragdoll of `joint_count` parts: one finite transform per joint with
    /// a unit rotation, and finite world positions.
    pub(crate) fn validate(&self, joint_count: usize) -> Result<(), RagdollError> {
        let invalid = |what| Err(RagdollError::InvalidValue(what));
        if self.joints.len() != joint_count {
            return invalid("a pose has exactly one transform per ragdoll part");
        }
        if !self.root_offset.is_finite() {
            return invalid("pose root offset must be finite");
        }
        for (index, joint) in self.joints.iter().enumerate() {
            if !joint.translation.is_finite() {
                return invalid("pose translations must be finite");
            }
            if !joint.rotation.is_valid_rotation() {
                return invalid("pose rotations must be finite unit quaternions");
            }
            if !self.position(index).is_finite() {
                return invalid("pose positions must be finite");
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{MotorSettings, SixDofAxis, SixDofConstraintAxis, SwingType};

    fn joint(name: &str, parent: Option<u32>) -> SkeletonJoint<'_> {
        SkeletonJoint { name, parent }
    }

    fn chain() -> Skeleton {
        Skeleton::new(&[
            joint("root", None),
            joint("child", Some(0)),
            joint("grandchild", Some(1)),
        ])
        .unwrap()
    }

    #[track_caller]
    fn assert_invalid<T>(result: Result<T, RagdollError>) {
        assert!(
            matches!(result, Err(RagdollError::InvalidValue(_))),
            "accepted"
        );
    }

    #[test]
    fn skeletons_are_validated() {
        let skeleton = chain();
        assert_eq!(skeleton.joint_count(), 3);
        assert_eq!(skeleton.joint(1), Some(joint("child", Some(0))));
        assert_eq!(skeleton.joint(3), None);
        assert_eq!(skeleton.joint_index("grandchild"), Some(2));
        assert_eq!(skeleton.joint_index("nobody"), None);
        // SAFETY: the skeleton is live; the getter only reads it.
        assert!(unsafe { JPH_Skeleton_AreJointsCorrectlyOrdered(skeleton.skeleton.as_ptr()) });

        assert_invalid(Skeleton::new(&[]));
        assert_invalid(Skeleton::new(&[joint("root", Some(0))]));
        assert_invalid(Skeleton::new(&[joint("a", None), joint("b", None)]));
        assert_invalid(Skeleton::new(&[joint("a", None), joint("b", Some(1))]));
        assert_invalid(Skeleton::new(&[joint("a", None), joint("a", Some(0))]));
        assert_invalid(Skeleton::new(&[joint("a\0b", None)]));
        let names: Vec<String> = (0..=Skeleton::MAX_JOINTS)
            .map(|i| format!("j{i}"))
            .collect();
        let too_many: Vec<SkeletonJoint<'_>> = names
            .iter()
            .enumerate()
            .map(|(i, name)| joint(name, (i > 0).then_some(0)))
            .collect();
        assert_invalid(Skeleton::new(&too_many));
        assert!(Skeleton::new(&too_many[..Skeleton::MAX_JOINTS]).is_ok());
    }

    fn capsule() -> Shape {
        Shape::new_capsule(0.3, 0.2).unwrap()
    }

    fn twist() -> RagdollJoint {
        RagdollJoint::SwingTwist(
            SwingTwistConstraintSettings::new(
                RVec3::new(0.0, 0.5, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
            )
            .half_cone_angles(0.5, 0.5),
        )
    }

    fn parts(shape: &Shape) -> Vec<RagdollPart<'_>> {
        (0..3)
            .map(|i| RagdollPart {
                shape,
                body: BodySettings::new_dynamic().position(RVec3::new(0.0, i as Real, 0.0)),
                joint: (i > 0).then(twist),
            })
            .collect()
    }

    #[test]
    fn ragdoll_settings_are_validated() {
        let skeleton = chain();
        let shape = capsule();
        let settings = RagdollSettings::new(&skeleton, &parts(&shape)).unwrap();
        assert_eq!(settings.part_count(), 3);
        assert_eq!(settings.parents(), &[None, Some(0), Some(1)]);
        assert_eq!(
            settings.joints(),
            &[
                None,
                Some(JointKind::SwingTwist),
                Some(JointKind::SwingTwist)
            ]
        );

        let edit = |change: &dyn Fn(&mut Vec<RagdollPart<'_>>)| {
            let mut parts = parts(&shape);
            change(&mut parts);
            RagdollSettings::new(&skeleton, &parts).map(|_| ())
        };
        assert_invalid(edit(&|parts| {
            parts.pop();
        }));
        assert_invalid(edit(&|parts| parts[0].joint = Some(twist())));
        assert_invalid(edit(&|parts| parts[2].joint = None));
        assert_invalid(edit(&|parts| parts[1].body = BodySettings::new_static()));
        assert_invalid(edit(&|parts| {
            parts[1].body = BodySettings::new_dynamic().mass(1.0e-39)
        }));
        assert_invalid(edit(&|parts| {
            parts[1].body = BodySettings::new_dynamic().friction(-1.0)
        }));
        assert_invalid(edit(&|parts| {
            parts[1].joint = Some(RagdollJoint::Hinge(
                HingeConstraintSettings::default().limits(0.5, 1.0),
            ))
        }));
        assert_invalid(edit(&|parts| {
            parts[2].joint = Some(RagdollJoint::SixDof(
                SixDofConstraintSettings::default()
                    .axis(
                        SixDofConstraintAxis::RotationZ,
                        SixDofAxis::Limited {
                            min: -0.2,
                            max: 0.5,
                        },
                    )
                    .motor(
                        SixDofConstraintAxis::RotationZ,
                        MotorSettings::default().torque_limits(1.0, 0.0),
                    ),
            ))
        }));
        let pyramid = SixDofConstraintSettings::default()
            .swing_type(SwingType::Pyramid)
            .axis(
                SixDofConstraintAxis::RotationZ,
                SixDofAxis::Limited {
                    min: -0.2,
                    max: 0.5,
                },
            );
        assert_eq!(
            edit(&|parts| parts[2].joint = Some(RagdollJoint::SixDof(pyramid.clone()))),
            Ok(())
        );
        assert_eq!(
            edit(&|parts| parts[1].body = BodySettings::new_kinematic()),
            Ok(())
        );
    }

    #[test]
    fn heightfield_parts_are_rejected() {
        let skeleton = Skeleton::new(&[joint("root", None)]).unwrap();
        let height_field =
            Shape::new_height_field(33, &[0.0; 33 * 33], &crate::HeightFieldSettings::default())
                .unwrap();
        let part = RagdollPart {
            shape: &height_field,
            body: BodySettings::new_dynamic(),
            joint: None,
        };
        assert_invalid(RagdollSettings::new(&skeleton, &[part]));
    }

    #[test]
    fn stabilized_settings_build() {
        let skeleton = chain();
        let shape = capsule();
        let mut parts = parts(&shape);
        parts[2].body = parts[2].body.clone().mass(30.0);
        assert!(RagdollSettings::new_stabilized(&skeleton, &parts).is_ok());
    }

    fn pose() -> SkeletonPose {
        SkeletonPose {
            root_offset: RVec3::new(1.0, 2.0, 3.0),
            joints: vec![
                JointTransform {
                    translation: Vec3::ZERO,
                    rotation: Quat::IDENTITY,
                };
                3
            ],
        }
    }

    #[test]
    fn poses_are_validated() {
        assert_eq!(pose().validate(3), Ok(()));
        assert_invalid(pose().validate(2));
        let mut bad = pose();
        bad.root_offset.x = Real::INFINITY;
        assert_invalid(bad.validate(3));
        let mut bad = pose();
        bad.joints[1].translation.y = f32::NAN;
        assert_invalid(bad.validate(3));
        let mut bad = pose();
        bad.joints[2].rotation = Quat::from_xyzw(0.0, 0.0, 0.0, 2.0);
        assert_invalid(bad.validate(3));
        // The sum overflows in single precision; in double precision it rounds to `Real::MAX`.
        let mut far = pose();
        far.root_offset.x = Real::MAX;
        far.joints[1].translation.x = f32::MAX;
        assert_eq!(far.validate(3).is_ok(), far.position(1).is_finite());
        assert_eq!(pose().position(0), RVec3::new(1.0, 2.0, 3.0));
    }

    #[test]
    fn joint_kinds_map_to_jolts_subtypes() {
        assert_eq!(
            JointKind::SwingTwist.sub_type(),
            JPH_ConstraintSubType_SwingTwist
        );
        assert_eq!(JointKind::Hinge.sub_type(), JPH_ConstraintSubType_Hinge);
        assert_eq!(JointKind::SixDof.sub_type(), JPH_ConstraintSubType_SixDOF);
    }
}
