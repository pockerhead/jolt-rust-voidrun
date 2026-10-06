//! A detailed animation skeleton over the ragdoll: the ragdoll's twelve joints at their bind
//! centres plus a root below the pelvis, a spine, a neck, two clavicles and leaves past the
//! head, hands and feet, 22 joints. A procedural wave bends it over time.

use glam::{Quat, Vec3};
use oxijolt::{JointTransform, RVec3, Skeleton, SkeletonJoint, SkeletonPose};

use super::humanoid::PARTS;
use crate::math::{glam_quat, quat, vec3};
use crate::scene::Result;

/// One joint: name, parent's index, model position with the pelvis at the origin.
struct RigJoint {
    name: &'static str,
    parent: Option<usize>,
    position: Vec3,
}

/// The animation skeleton in its neutral pose.
pub struct Rig {
    joints: Vec<RigJoint>,
}

/// A rig joint's name, its parent's name, and its model position, or `None` for a joint at
/// the bind centre of the ragdoll part of the same name.
type Row = (&'static str, Option<&'static str>, Option<[f32; 3]>);

/// The rig's joints, parents first.
const ROWS: [Row; 22] = [
    ("root", None, Some([0.0, -0.9, 0.0])),
    ("pelvis", Some("root"), None),
    ("spine", Some("pelvis"), Some([0.0, 0.13, 0.0])),
    ("abdomen", Some("spine"), None),
    ("chest", Some("abdomen"), None),
    ("neck", Some("chest"), Some([0.0, 0.72, 0.0])),
    ("head", Some("neck"), None),
    ("head_end", Some("head"), Some([0.0, 1.0, 0.0])),
    ("clavicle_l", Some("chest"), Some([0.12, 0.62, 0.0])),
    ("upper_arm_l", Some("clavicle_l"), None),
    ("forearm_l", Some("upper_arm_l"), None),
    ("hand_l", Some("forearm_l"), Some([0.70, 0.60, 0.0])),
    ("clavicle_r", Some("chest"), Some([-0.12, 0.62, 0.0])),
    ("upper_arm_r", Some("clavicle_r"), None),
    ("forearm_r", Some("upper_arm_r"), None),
    ("hand_r", Some("forearm_r"), Some([-0.70, 0.60, 0.0])),
    ("thigh_l", Some("pelvis"), None),
    ("shin_l", Some("thigh_l"), None),
    ("foot_l", Some("shin_l"), Some([0.09, -0.88, 0.05])),
    ("thigh_r", Some("pelvis"), None),
    ("shin_r", Some("thigh_r"), None),
    ("foot_r", Some("shin_r"), Some([-0.09, -0.88, 0.05])),
];

/// The bind centre of the ragdoll part named `name`.
fn centre(name: &str) -> Vec3 {
    let part = PARTS
        .iter()
        .find(|part| part.name == name)
        .expect("a ragdoll part of that name");
    Vec3::from(part.centre)
}

impl Rig {
    /// The 22-joint humanoid rig.
    pub fn humanoid() -> Self {
        let mut joints: Vec<RigJoint> = Vec::with_capacity(ROWS.len());
        for (name, parent, position) in ROWS {
            let parent = parent.map(|parent| {
                joints
                    .iter()
                    .position(|joint| joint.name == parent)
                    .expect("parents come first")
            });
            joints.push(RigJoint {
                name,
                parent,
                position: position.map_or_else(|| centre(name), Vec3::from),
            });
        }
        Self { joints }
    }

    /// The rig's skeleton.
    pub fn skeleton(&self) -> Result<Skeleton> {
        let joints: Vec<SkeletonJoint<'_>> = self
            .joints
            .iter()
            .map(|joint| SkeletonJoint {
                name: joint.name,
                parent: joint.parent.map(|parent| parent as u32),
            })
            .collect();
        Ok(Skeleton::new(&joints)?)
    }

    /// The neutral pose in model space, the pelvis at the origin.
    pub fn neutral_pose(&self) -> SkeletonPose {
        SkeletonPose {
            root_offset: RVec3::ZERO,
            joints: self
                .joints
                .iter()
                .map(|joint| JointTransform {
                    translation: vec3(joint.position),
                    rotation: oxijolt::Quat::IDENTITY,
                })
                .collect(),
        }
    }

    /// Each joint's transform relative to its parent (the root's relative to the root offset),
    /// with the wave's bends at time `t`: spine, neck, a clavicle and a hand sway, the head
    /// nods and the right arm waves.
    pub fn animated_local(&self, t: f32) -> Vec<JointTransform> {
        let bend = |name: &str| -> Quat {
            let (axis, angle) = match name {
                "spine" => (Vec3::Z, 0.25 * t.sin()),
                "neck" => (Vec3::X, 0.3 * (1.3 * t).sin()),
                "clavicle_l" => (Vec3::Z, 0.15 * t.cos()),
                "hand_l" => (Vec3::Y, 0.4 * t.sin()),
                "head" => (Vec3::X, 0.2 * t.sin()),
                "upper_arm_r" => (Vec3::Z, 0.9 + 0.5 * (2.0 * t).sin()),
                "forearm_r" => (Vec3::Y, -0.6 - 0.4 * (2.0 * t).cos()),
                _ => return Quat::IDENTITY,
            };
            Quat::from_axis_angle(axis, angle)
        };
        self.joints
            .iter()
            .map(|joint| {
                let offset = match joint.parent {
                    Some(parent) => joint.position - self.joints[parent].position,
                    None => joint.position,
                };
                JointTransform {
                    translation: vec3(offset),
                    rotation: quat(bend(joint.name)),
                }
            })
            .collect()
    }

    /// The model pose at `root_offset` of a local pose from [`animated_local`](Self::animated_local).
    pub fn to_model(&self, local: &[JointTransform], root_offset: RVec3) -> SkeletonPose {
        let mut world: Vec<(Vec3, Quat)> = Vec::with_capacity(local.len());
        for (joint, transform) in self.joints.iter().zip(local) {
            let own = (
                crate::math::glam(transform.translation),
                glam_quat(transform.rotation),
            );
            world.push(match joint.parent {
                None => own,
                Some(parent) => {
                    let (position, rotation) = world[parent];
                    (position + rotation * own.0, rotation * own.1)
                }
            });
        }
        SkeletonPose {
            root_offset,
            joints: world
                .into_iter()
                .map(|(position, rotation)| JointTransform {
                    translation: vec3(position),
                    rotation: quat(rotation),
                })
                .collect(),
        }
    }

    /// The parent of each joint, for drawing the skeleton as lines.
    pub fn parents(&self) -> impl Iterator<Item = Option<usize>> + '_ {
        self.joints.iter().map(|joint| joint.parent)
    }
}
