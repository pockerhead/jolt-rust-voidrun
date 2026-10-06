//! A detailed animation skeleton over the humanoid ragdoll of [`super::ragdoll`], for the
//! skeleton mapper tests, and rigid transforms in f64 for their oracles.
//!
//! The rig has the ragdoll's twelve joints at their bind centres plus an extra root below the
//! pelvis, a spine between pelvis and abdomen, a neck between chest and head, a clavicle between
//! the chest and each upper arm, and leaves past the head, hands and feet: 22 joints. Pelvis and
//! chest, the starts of its two chains, are a quarter turn about Y from the ragdoll's.

use std::f32::consts::FRAC_PI_2;

use oxijolt::*;

use super::math::{add, cross, f3, scale, sub, v3, vec3, V3};
use super::quat_about;
use super::ragdoll::PARTS;

/// One rig joint: name, parent name, model position (the pose's root offset is the origin)
/// and rotation.
#[derive(Clone, Debug)]
pub struct RigJoint {
    pub name: String,
    pub parent: Option<String>,
    pub position: V3,
    pub rotation: Quat,
}

/// An animation skeleton in its neutral pose.
#[derive(Clone, Debug)]
pub struct Rig {
    pub joints: Vec<RigJoint>,
}

const Y: Vec3 = Vec3::new(0.0, 1.0, 0.0);

/// The ragdoll bind centre of the part named `name`.
fn bind_centre(name: &str) -> V3 {
    let spec = PARTS.iter().find(|spec| spec.name == name).unwrap();
    spec.centre.map(f64::from)
}

impl Rig {
    /// The 22-joint rig: chains pelvis-spine-abdomen and chest-neck-head; the clavicles tie
    /// with the neck (one joint between), so the head, the lowest ragdoll joint, wins them.
    pub fn humanoid() -> Self {
        let turned = quat_about(Y, FRAC_PI_2);
        let mapped =
            |name: &'static str, parent: &'static str| (name, Some(parent), bind_centre(name));
        let mut rig = Self { joints: Vec::new() };
        let joints: [(&str, Option<&str>, V3); 22] = [
            ("root", None, [0.0, -0.9, 0.0]),
            ("pelvis", Some("root"), bind_centre("pelvis")),
            ("spine", Some("pelvis"), [0.0, 0.13, 0.0]),
            mapped("abdomen", "spine"),
            mapped("chest", "abdomen"),
            ("neck", Some("chest"), [0.0, 0.72, 0.0]),
            mapped("head", "neck"),
            ("head_end", Some("head"), [0.0, 1.0, 0.0]),
            ("clavicle_l", Some("chest"), [0.12, 0.62, 0.0]),
            mapped("upper_arm_l", "clavicle_l"),
            mapped("forearm_l", "upper_arm_l"),
            ("hand_l", Some("forearm_l"), [0.70, 0.60, 0.0]),
            ("clavicle_r", Some("chest"), [-0.12, 0.62, 0.0]),
            mapped("upper_arm_r", "clavicle_r"),
            mapped("forearm_r", "upper_arm_r"),
            ("hand_r", Some("forearm_r"), [-0.70, 0.60, 0.0]),
            mapped("thigh_l", "pelvis"),
            mapped("shin_l", "thigh_l"),
            ("foot_l", Some("shin_l"), [0.09, -0.88, 0.05]),
            mapped("thigh_r", "pelvis"),
            mapped("shin_r", "thigh_r"),
            ("foot_r", Some("shin_r"), [-0.09, -0.88, 0.05]),
        ];
        for (name, parent, position) in joints {
            let rotation = if name == "pelvis" || name == "chest" {
                turned
            } else {
                Quat::IDENTITY
            };
            rig.joints.push(RigJoint {
                name: name.to_owned(),
                parent: parent.map(str::to_owned),
                position,
                rotation,
            });
        }
        rig
    }

    /// The humanoid with a shoulder between the left clavicle and upper arm: the arm path from
    /// the chest is now the longest, so the chest starts the arm chain and the neck is unmapped.
    pub fn with_long_left_arm() -> Self {
        let mut rig = Self::humanoid();
        rig.insert_above("upper_arm_l", "shoulder_l", [0.19, 0.61, 0.0]);
        rig
    }

    /// The humanoid with `count` joints between pelvis and abdomen instead of the spine.
    pub fn with_long_spine(count: usize) -> Self {
        let mut rig = Self::humanoid();
        rig.joints.retain(|joint| joint.name != "spine");
        let abdomen = rig.index("abdomen");
        rig.joints[abdomen].parent = Some("pelvis".to_owned());
        let (from, to) = (bind_centre("pelvis"), bind_centre("abdomen"));
        for i in 0..count {
            let fraction = (i + 1) as f64 / (count + 1) as f64;
            let position = add(from, scale(sub(to, from), fraction));
            rig.insert_above("abdomen", &format!("spine_{i}"), position);
        }
        rig
    }

    /// Inserts a joint named `name` at `position` between `child` and its parent.
    pub fn insert_above(&mut self, child: &str, name: &str, position: V3) {
        let index = self.index(child);
        let parent = self.joints[index].parent.replace(name.to_owned());
        self.joints.insert(
            index,
            RigJoint {
                name: name.to_owned(),
                parent,
                position,
                rotation: Quat::IDENTITY,
            },
        );
    }

    pub fn index(&self, name: &str) -> usize {
        self.joints
            .iter()
            .position(|joint| joint.name == name)
            .unwrap_or_else(|| panic!("no joint {name}"))
    }

    pub fn len(&self) -> usize {
        self.joints.len()
    }

    pub fn parents(&self) -> Vec<Option<u32>> {
        self.joints
            .iter()
            .map(|joint| joint.parent.as_deref().map(|name| self.index(name) as u32))
            .collect()
    }

    pub fn skeleton(&self) -> Skeleton {
        let parents = self.parents();
        let joints: Vec<SkeletonJoint<'_>> = self
            .joints
            .iter()
            .zip(&parents)
            .map(|(joint, &parent)| SkeletonJoint {
                name: &joint.name,
                parent,
            })
            .collect();
        Skeleton::new(&joints).unwrap()
    }

    /// The neutral pose in model space at root offset zero, like the ragdoll's bind pose.
    pub fn neutral_pose(&self) -> SkeletonPose {
        SkeletonPose {
            root_offset: RVec3::ZERO,
            joints: self
                .joints
                .iter()
                .map(|joint| JointTransform {
                    translation: vec3(joint.position),
                    rotation: joint.rotation,
                })
                .collect(),
        }
    }

    /// The neutral pose as local transforms: each relative to its parent, the root's relative
    /// to the root offset.
    pub fn neutral_local(&self) -> Vec<JointTransform> {
        let model = self.neutral_pose();
        let world: Vec<Rigid> = (0..self.len())
            .map(|j| Rigid::of(model.joints[j], RVec3::ZERO))
            .collect();
        self.parents()
            .iter()
            .enumerate()
            .map(|(j, parent)| match parent {
                None => world[j].joint(),
                Some(parent) => world[*parent as usize].inverse().then(world[j]).joint(),
            })
            .collect()
    }

    /// The neutral local pose with joints bent over time `t`: the spine, neck, a clavicle, a
    /// hand and a foot (unmapped) and the head and an upper arm (mapped), so chains and leaves
    /// move off their neutral axes.
    pub fn animated_local(&self, t: f32) -> Vec<JointTransform> {
        let mut local = self.neutral_local();
        let bends: [(&str, Vec3, f32); 7] = [
            ("spine", Vec3::new(0.0, 0.0, 1.0), 0.3 * t.sin()),
            ("neck", Vec3::new(1.0, 0.0, 0.0), 0.4 * (1.3 * t).sin()),
            ("clavicle_l", Vec3::new(0.0, 0.0, 1.0), 0.2 * t.cos()),
            ("hand_l", Y, 0.5 * t.sin()),
            ("foot_r", Vec3::new(1.0, 0.0, 0.0), 0.3 * t.cos()),
            ("head", Vec3::new(1.0, 0.0, 0.0), 0.2 * t.sin()),
            ("upper_arm_r", Vec3::new(0.0, 0.0, 1.0), 0.4 * t.cos()),
        ];
        for (name, axis, angle) in bends {
            if let Some(index) = self.joints.iter().position(|joint| joint.name == name) {
                let joint = &mut local[index];
                joint.rotation = super::ragdoll::mul(joint.rotation, quat_about(axis, angle));
            }
        }
        local
    }

    /// The model pose of a local pose, in f64, at `root_offset`.
    pub fn local_to_model(&self, local: &[JointTransform], root_offset: RVec3) -> SkeletonPose {
        let parents = self.parents();
        let mut world: Vec<Rigid> = Vec::with_capacity(local.len());
        for (j, transform) in local.iter().enumerate() {
            let own = Rigid::of(*transform, RVec3::ZERO);
            world.push(match parents[j] {
                None => Rigid::of(*transform, root_offset),
                Some(parent) => world[parent as usize].then(own),
            });
        }
        SkeletonPose {
            root_offset,
            joints: world.iter().map(|w| w.relative_to(root_offset)).collect(),
        }
    }
}

/// A rigid transform in f64: rotate by `rotation` (x, y, z, w), then translate.
#[derive(Clone, Copy, Debug)]
pub struct Rigid {
    pub rotation: [f64; 4],
    pub translation: V3,
}

fn qmul(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    let ([ax, ay, az, aw], [bx, by, bz, bw]) = (a, b);
    [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]
}

fn quat(q: [f64; 4]) -> Quat {
    Quat::from_xyzw(q[0] as f32, q[1] as f32, q[2] as f32, q[3] as f32)
}

impl Rigid {
    /// The world transform of `joint` of a pose at `root_offset`.
    pub fn of(joint: JointTransform, root_offset: RVec3) -> Self {
        let r = joint.rotation;
        Self {
            rotation: [r.x, r.y, r.z, r.w].map(f64::from),
            translation: add(v3(root_offset), f3(joint.translation)),
        }
    }

    /// The world transform of joint `index` of `pose`.
    pub fn at(pose: &SkeletonPose, index: usize) -> Self {
        Self::of(pose.joints[index], pose.root_offset)
    }

    /// `self` after `other`: `other` given in `self`'s frame.
    pub fn then(self, other: Self) -> Self {
        Self {
            rotation: qmul(self.rotation, other.rotation),
            translation: add(self.translation, self.apply_vector(other.translation)),
        }
    }

    pub fn inverse(self) -> Self {
        let [x, y, z, w] = self.rotation;
        let conjugate = [-x, -y, -z, w];
        let rotated = Self {
            rotation: conjugate,
            translation: [0.0; 3],
        }
        .apply_vector(self.translation);
        Self {
            rotation: conjugate,
            translation: [-rotated[0], -rotated[1], -rotated[2]],
        }
    }

    /// `v` rotated by the transform's rotation: `v + 2w (q x v) + 2 q x (q x v)`.
    pub fn apply_vector(self, v: V3) -> V3 {
        let [x, y, z, w] = self.rotation;
        let u = [x, y, z];
        let t = scale(cross(u, v), 2.0);
        add(add(v, scale(t, w)), cross(u, t))
    }

    /// The transform as a joint of a pose at `root_offset`.
    pub fn relative_to(self, root_offset: RVec3) -> JointTransform {
        JointTransform {
            translation: vec3(sub(self.translation, v3(root_offset))),
            rotation: quat(self.rotation),
        }
    }

    /// The transform as a local joint transform.
    pub fn joint(self) -> JointTransform {
        self.relative_to(RVec3::ZERO)
    }

    /// Distance in metres and rotation angle in radians to `other`.
    pub fn difference(self, other: Self) -> (f64, f64) {
        let d = sub(self.translation, other.translation);
        let distance = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
        let [x, y, z, w] = self.rotation;
        let relative = qmul([-x, -y, -z, w], other.rotation);
        let sin = (relative[0].powi(2) + relative[1].powi(2) + relative[2].powi(2)).sqrt();
        (distance, 2.0 * sin.atan2(relative[3].abs()))
    }
}
