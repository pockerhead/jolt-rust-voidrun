//! A 12-capsule humanoid ragdoll for the ragdoll tests: its skeleton, parts, joints with their
//! limits, the caller's radial gravity, joint-limit checks and part contact pairs.
//!
//! The humanoid stands along +Y facing +Z in a T-pose, its pelvis at the origin, 1.89 m from
//! head to feet. Adjacent capsules overlap by about 2 cm at each joint.

use std::f32::consts::FRAC_PI_2;

use joltphysics::*;

use super::quat_about;
use super::walker::{add, f3, norm, rvec3, scale, sub, v3, vec3, V3};

/// Total mass, kg.
pub const MASS: f32 = 80.0;
/// Number of parts.
pub const PART_COUNT: usize = 12;
/// How far a joint reading may exceed its limit, radians.
pub const LIMIT_TOLERANCE: f32 = 0.1;
/// The hip's rotation motors. The thigh is the heaviest limb, and with Jolt's default 2 Hz
/// spring a driven hip swings around its target for seconds.
pub fn hip_motor() -> MotorSettings {
    MotorSettings::default().spring(SpringSettings::FrequencyAndDamping {
        frequency: 4.0,
        damping: 1.0,
    })
}
/// Friction torque of every joint, N·m. Without it a near-spherical head on its neck joint rocks
/// on the ground for a long time and the humanoid does not come to rest.
pub const JOINT_FRICTION: f32 = 1.0;

pub const PELVIS: usize = 0;
pub const ABDOMEN: usize = 1;
pub const CHEST: usize = 2;
pub const HEAD: usize = 3;
pub const UPPER_ARM_L: usize = 4;
pub const FOREARM_L: usize = 5;
pub const UPPER_ARM_R: usize = 6;
pub const FOREARM_R: usize = 7;
pub const THIGH_L: usize = 8;
pub const SHIN_L: usize = 9;
pub const THIGH_R: usize = 10;
pub const SHIN_R: usize = 11;

/// One part: skeleton name and parent, mass fraction, capsule, bind centre, and whether the
/// capsule lies along X (arms) instead of Y.
pub struct PartSpec {
    pub name: &'static str,
    pub parent: Option<u32>,
    pub mass_fraction: f32,
    pub half_height: f32,
    pub radius: f32,
    pub centre: [f32; 3],
    pub along_x: bool,
}

const fn part(
    name: &'static str,
    parent: Option<u32>,
    mass_fraction: f32,
    half_height: f32,
    radius: f32,
    centre: [f32; 3],
    along_x: bool,
) -> PartSpec {
    PartSpec {
        name,
        parent,
        mass_fraction,
        half_height,
        radius,
        centre,
        along_x,
    }
}

/// The parts in skeleton order; parents precede children.
pub const PARTS: [PartSpec; PART_COUNT] = [
    part("pelvis", None, 0.142, 0.04, 0.12, [0.0, 0.0, 0.0], false),
    part(
        "abdomen",
        Some(0),
        0.139,
        0.04,
        0.10,
        [0.0, 0.26, 0.0],
        false,
    ),
    part("chest", Some(1), 0.216, 0.07, 0.13, [0.0, 0.54, 0.0], false),
    part("head", Some(2), 0.081, 0.04, 0.10, [0.0, 0.86, 0.0], false),
    part(
        "upper_arm_l",
        Some(2),
        0.028,
        0.10,
        0.045,
        [0.255, 0.60, 0.0],
        true,
    ),
    part(
        "forearm_l",
        Some(4),
        0.022,
        0.11,
        0.04,
        [0.53, 0.60, 0.0],
        true,
    ),
    part(
        "upper_arm_r",
        Some(2),
        0.028,
        0.10,
        0.045,
        [-0.255, 0.60, 0.0],
        true,
    ),
    part(
        "forearm_r",
        Some(6),
        0.022,
        0.11,
        0.04,
        [-0.53, 0.60, 0.0],
        true,
    ),
    part(
        "thigh_l",
        Some(0),
        0.100,
        0.14,
        0.065,
        [0.09, -0.305, 0.0],
        false,
    ),
    part(
        "shin_l",
        Some(8),
        0.061,
        0.15,
        0.05,
        [0.09, -0.69, 0.0],
        false,
    ),
    part(
        "thigh_r",
        Some(0),
        0.100,
        0.14,
        0.065,
        [-0.09, -0.305, 0.0],
        false,
    ),
    part(
        "shin_r",
        Some(10),
        0.061,
        0.15,
        0.05,
        [-0.09, -0.69, 0.0],
        false,
    ),
];

/// Half span from hand tip to the body's centre plane, metres.
pub const HALF_SPAN: f32 = 0.68;
/// Head top above the pelvis origin, metres.
pub const HEAD_TOP: f32 = 1.0;
/// Feet below the pelvis origin, metres.
pub const FEET: f32 = 0.89;

/// The limits a joint reading is checked against.
#[derive(Clone, Copy, Debug)]
pub enum Limits {
    /// A circular swing cone and a twist range about X.
    Cone { half_angle: f32, twist: (f32, f32) },
    /// A hinge angle range.
    Hinge { min: f32, max: f32 },
    /// Independent swing ranges about Y and Z and a twist range about X.
    Pyramid {
        twist: (f32, f32),
        swing_y: (f32, f32),
        swing_z: (f32, f32),
    },
}

pub const X: Vec3 = Vec3::new(1.0, 0.0, 0.0);
pub const Y: Vec3 = Vec3::new(0.0, 1.0, 0.0);

pub fn neg(v: Vec3) -> Vec3 {
    Vec3::new(-v.x, -v.y, -v.z)
}

fn point(p: [f32; 3]) -> RVec3 {
    rvec3(p.map(f64::from))
}

/// The joint of `part` to its parent, with its limits; `None` for the pelvis.
pub fn joint(part: usize) -> Option<(RagdollJoint, Limits)> {
    let cone = |at: [f32; 3], twist_axis: Vec3, plane_axis: Vec3, half: f32, twist: f32| {
        (
            RagdollJoint::SwingTwist(
                SwingTwistConstraintSettings::new(point(at), twist_axis, plane_axis)
                    .half_cone_angles(half, half)
                    .twist_limits(-twist, twist)
                    .max_friction_torque(JOINT_FRICTION),
            ),
            Limits::Cone {
                half_angle: half,
                twist: (-twist, twist),
            },
        )
    };
    let hinge = |at: [f32; 3], axis: Vec3, normal: Vec3, min: f32, max: f32| {
        (
            RagdollJoint::Hinge(
                HingeConstraintSettings::new(point(at), axis, normal)
                    .limits(min, max)
                    .max_friction_torque(JOINT_FRICTION),
            ),
            Limits::Hinge { min, max },
        )
    };
    // The thigh is the twist axis; the flexion axis points so that forward flexion is positive
    // on the right leg and negative on the left, which makes abduction (outward) positive on
    // both.
    let hip = |side: f32| {
        let at = [0.09 * side, -0.11, 0.0];
        let axis_y = if side > 0.0 { X } else { neg(X) };
        let flexion = if side > 0.0 { (-1.6, 0.3) } else { (-0.3, 1.6) };
        let twist = (-0.3, 0.3);
        let abduction = (-0.2, 0.5);
        let limited = |(min, max): (f32, f32)| SixDofAxis::Limited { min, max };
        let mut settings = SixDofConstraintSettings::new(point(at), neg(Y), axis_y)
            .swing_type(SwingType::Pyramid)
            .axis(SixDofConstraintAxis::RotationX, limited(twist))
            .axis(SixDofConstraintAxis::RotationY, limited(flexion))
            .axis(SixDofConstraintAxis::RotationZ, limited(abduction));
        for axis in [
            SixDofConstraintAxis::TranslationX,
            SixDofConstraintAxis::TranslationY,
            SixDofConstraintAxis::TranslationZ,
        ] {
            settings = settings.axis(axis, SixDofAxis::Fixed);
        }
        for axis in [
            SixDofConstraintAxis::RotationX,
            SixDofConstraintAxis::RotationY,
            SixDofConstraintAxis::RotationZ,
        ] {
            settings = settings
                .max_friction(axis, JOINT_FRICTION)
                .motor(axis, hip_motor());
        }
        (
            RagdollJoint::SixDof(settings),
            Limits::Pyramid {
                twist,
                swing_y: flexion,
                swing_z: abduction,
            },
        )
    };
    Some(match part {
        ABDOMEN => cone([0.0, 0.14, 0.0], Y, X, 0.4, 0.3),
        CHEST => cone([0.0, 0.37, 0.0], Y, X, 0.3, 0.3),
        HEAD => cone([0.0, 0.73, 0.0], Y, X, 0.6, 0.8),
        UPPER_ARM_L => cone([0.12, 0.60, 0.0], X, Y, 1.3, 0.5),
        UPPER_ARM_R => cone([-0.12, 0.60, 0.0], neg(X), Y, 1.3, 0.5),
        // Elbows flex the forearm forward (+Z); knees flex the shin backward.
        FOREARM_L => hinge([0.39, 0.60, 0.0], neg(Y), X, 0.0, 2.4),
        FOREARM_R => hinge([-0.39, 0.60, 0.0], Y, neg(X), 0.0, 2.4),
        SHIN_L => hinge([0.09, -0.50, 0.0], neg(X), neg(Y), -2.4, 0.0),
        SHIN_R => hinge([-0.09, -0.50, 0.0], neg(X), neg(Y), -2.4, 0.0),
        THIGH_L => hip(1.0),
        THIGH_R => hip(-1.0),
        _ => return None,
    })
}

/// The limits of `part`'s joint.
pub fn limits(part: usize) -> Option<Limits> {
    joint(part).map(|(_, limits)| limits)
}

/// The bind rotation of a part: identity, or a quarter turn about Z for the arms.
pub fn bind_rotation(spec: &PartSpec) -> Quat {
    if spec.along_x {
        quat_about(Vec3::new(0.0, 0.0, 1.0), -FRAC_PI_2)
    } else {
        Quat::IDENTITY
    }
}

/// The capsule of every part, in part order.
pub fn part_shapes() -> Vec<Shape> {
    PARTS
        .iter()
        .map(|spec| Shape::new_capsule(spec.half_height, spec.radius).unwrap())
        .collect()
}

/// The body settings of `part` at its bind pose, in `layer`, under the caller's gravity.
pub fn part_body(part: usize, layer: ObjectLayer) -> BodySettings {
    let spec = &PARTS[part];
    BodySettings::new_dynamic()
        .object_layer(layer)
        .position(point(spec.centre))
        .rotation(bind_rotation(spec))
        .mass(MASS * spec.mass_fraction)
        .friction(0.8)
        .linear_damping(0.1)
        .angular_damping(0.2)
        .gravity_factor(0.0)
}

pub fn skeleton() -> Skeleton {
    let joints: Vec<SkeletonJoint<'_>> = PARTS
        .iter()
        .map(|spec| SkeletonJoint {
            name: spec.name,
            parent: spec.parent,
        })
        .collect();
    Skeleton::new(&joints).unwrap()
}

/// The humanoid's parts over `shapes` (from [`part_shapes`]), in `layer`.
pub fn humanoid_parts(shapes: &[Shape], layer: ObjectLayer) -> Vec<RagdollPart<'_>> {
    (0..PART_COUNT)
        .map(|part| RagdollPart {
            shape: &shapes[part],
            body: part_body(part, layer),
            joint: joint(part).map(|(joint, _)| joint),
        })
        .collect()
}

/// The humanoid's settings with the raw mass fractions, in `layer`.
pub fn humanoid_settings(layer: ObjectLayer) -> RagdollSettings {
    let shapes = part_shapes();
    RagdollSettings::new(&skeleton(), &humanoid_parts(&shapes, layer)).unwrap()
}

/// The humanoid's settings after Jolt's `Stabilize`, in `layer`.
pub fn stabilized_humanoid_settings(layer: ObjectLayer) -> RagdollSettings {
    let shapes = part_shapes();
    RagdollSettings::new_stabilized(&skeleton(), &humanoid_parts(&shapes, layer)).unwrap()
}

/// The bind pose: pelvis at the origin.
pub fn bind_pose() -> SkeletonPose {
    SkeletonPose {
        root_offset: RVec3::ZERO,
        joints: PARTS
            .iter()
            .map(|spec| JointTransform {
                translation: Vec3::from(spec.centre),
                rotation: bind_rotation(spec),
            })
            .collect(),
    }
}

// Quaternion helpers on `[x, y, z, w]`.

pub fn q(v: Quat) -> [f32; 4] {
    v.into()
}

pub fn mul(a: Quat, b: Quat) -> Quat {
    let ([ax, ay, az, aw], [bx, by, bz, bw]) = (q(a), q(b));
    Quat::from_xyzw(
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )
}

pub fn conj(a: Quat) -> Quat {
    Quat::from_xyzw(-a.x, -a.y, -a.z, a.w)
}

pub fn rotate(a: Quat, v: Vec3) -> Vec3 {
    let r = mul(mul(a, Quat::from_xyzw(v.x, v.y, v.z, 0.0)), conj(a));
    Vec3::new(r.x, r.y, r.z)
}

/// The angle of the rotation between `a` and `b`, radians in `[0, π]`; `atan2` keeps it
/// precise near zero, where `acos` of the dot product is not.
pub fn angle_between(a: Quat, b: Quat) -> f32 {
    let d = mul(conj(a), b);
    let sin = (d.x * d.x + d.y * d.y + d.z * d.z).sqrt();
    2.0 * sin.atan2(d.w.abs())
}

/// The world position of joint `index` of `pose`, in f64.
pub fn joint_position(pose: &SkeletonPose, index: usize) -> V3 {
    add(v3(pose.root_offset), f3(pose.joints[index].translation))
}

/// A pose from world positions and rotations, its root offset at the first one.
pub fn pose_from(world: &[(V3, Quat)]) -> SkeletonPose {
    let root = world[0].0;
    SkeletonPose {
        root_offset: rvec3(root),
        joints: world
            .iter()
            .map(|&(p, rotation)| JointTransform {
                translation: vec3(sub(p, root)),
                rotation,
            })
            .collect(),
    }
}

/// `pose` moved rigidly by `rotation` about the world origin, then by `translation`.
pub fn transformed_pose(pose: &SkeletonPose, rotation: Quat, translation: V3) -> SkeletonPose {
    let world: Vec<(V3, Quat)> = (0..pose.joints.len())
        .map(|index| {
            let p = super::walker::rotate(rotation, joint_position(pose, index));
            (
                add(p, translation),
                mul(rotation, pose.joints[index].rotation),
            )
        })
        .collect();
    pose_from(&world)
}

/// Distance in metres and rotation angle in radians between the transforms of joint `index`
/// in two poses.
pub fn transform_difference(a: &SkeletonPose, b: &SkeletonPose, index: usize) -> (f64, f32) {
    let distance = norm(sub(joint_position(a, index), joint_position(b, index)));
    (
        distance,
        angle_between(a.joints[index].rotation, b.joints[index].rotation),
    )
}

// The caller's radial gravity.

/// The planet centre of the tests' radial gravity, metres.
pub const PLANET_CENTRE: [f64; 3] = [0.0, -100.0, 0.0];

/// Gravity at `p` toward `centre`, 9.8 m/s².
pub fn gravity_at(p: RVec3, centre: [f64; 3]) -> Vec3 {
    let d = sub(v3(p), centre);
    vec3(scale(d, -9.8 / norm(d)))
}

/// Adds the force of gravity toward `centre` to `body`, for the next step.
pub fn apply_gravity_to(world: &mut PhysicsWorld, body: BodyId, centre: [f64; 3]) {
    let reading = world.body(body).unwrap();
    let mass = reading.mass().expect("a dynamic body");
    let g = gravity_at(reading.position(), centre);
    world
        .body_mut(body)
        .unwrap()
        .add_force(Vec3::new(g.x * mass, g.y * mass, g.z * mass))
        .unwrap();
}

/// Adds the force of gravity toward `centre` to every part of `ragdoll`.
pub fn apply_gravity(world: &mut PhysicsWorld, ragdoll: RagdollId, centre: [f64; 3]) {
    let parts = world.ragdoll(ragdoll).unwrap().body_ids().to_vec();
    for part in parts {
        apply_gravity_to(world, part, centre);
    }
}

// Joint limits.

/// Twist about X and the swing quaternion `(0, y, z, w)` of a constraint-space rotation, with
/// `w >= 0` (Jolt `SwingTwistConstraintPart::sDecomposeSwingTwist`).
fn swing_twist(rotation: Quat) -> (f32, Quat) {
    let r = if rotation.w < 0.0 {
        Quat::from_xyzw(-rotation.x, -rotation.y, -rotation.z, -rotation.w)
    } else {
        rotation
    };
    let twist_length = (r.x * r.x + r.w * r.w).sqrt();
    let twist = Quat::from_xyzw(r.x / twist_length, 0.0, 0.0, r.w / twist_length);
    let swing = mul(r, conj(twist));
    let swing = if swing.w < 0.0 {
        Quat::from_xyzw(-swing.x, -swing.y, -swing.z, -swing.w)
    } else {
        swing
    };
    (2.0 * twist.x.atan2(twist.w), swing)
}

fn outside(value: f32, (min, max): (f32, f32)) -> f32 {
    (min - value).max(value - max).max(0.0)
}

/// How far, in radians, `reading` is outside `limits`; 0 within them.
pub fn limit_violation(reading: JointReading, limits: Limits) -> f32 {
    match (reading, limits) {
        (JointReading::Hinge { current_angle }, Limits::Hinge { min, max }) => {
            outside(current_angle, (min, max))
        }
        (
            JointReading::SwingTwist {
                rotation_in_constraint_space,
            },
            Limits::Cone { half_angle, twist },
        ) => {
            let (twist_angle, swing) = swing_twist(rotation_in_constraint_space);
            let swing_angle = 2.0 * swing.w.min(1.0).acos();
            outside(twist_angle, twist).max((swing_angle - half_angle).max(0.0))
        }
        (
            JointReading::SixDof {
                rotation_in_constraint_space,
            },
            Limits::Pyramid {
                twist,
                swing_y,
                swing_z,
            },
        ) => {
            // Jolt's pyramid limits the half angles `atan2(swing.y, swing.w)` and
            // `atan2(swing.z, swing.w)` (`SwingTwistConstraintPart::ClampSwingTwist`).
            let (twist_angle, swing) = swing_twist(rotation_in_constraint_space);
            let y = 2.0 * swing.y.atan2(swing.w);
            let z = 2.0 * swing.z.atan2(swing.w);
            outside(twist_angle, twist)
                .max(outside(y, swing_y))
                .max(outside(z, swing_z))
        }
        (reading, limits) => panic!("reading {reading:?} does not match limits {limits:?}"),
    }
}

/// The largest limit violation over every joint of `ragdoll`, radians, with the part.
pub fn worst_limit_violation(world: &PhysicsWorld, ragdoll: RagdollId) -> (f32, usize) {
    let ragdoll = world.ragdoll(ragdoll).unwrap();
    (1..PART_COUNT)
        .map(|part| {
            let reading = ragdoll.joint(part as u32).unwrap();
            (limit_violation(reading, limits(part).unwrap()), part)
        })
        .fold(
            (0.0, 0),
            |worst, next| if next.0 > worst.0 { next } else { worst },
        )
}

/// Every pair of parts of `ragdoll` that touched in the last step.
pub fn part_pairs_in_contact(world: &PhysicsWorld, ragdoll: RagdollId) -> Vec<(usize, usize)> {
    let parts = world.ragdoll(ragdoll).unwrap().body_ids().to_vec();
    let mut pairs = Vec::new();
    for a in 0..parts.len() {
        for b in a + 1..parts.len() {
            if world.were_bodies_in_contact(parts[a], parts[b]).unwrap() {
                pairs.push((a, b));
            }
        }
    }
    pairs
}

/// Samples per side of the relief heightfield, one per metre.
pub const RELIEF_SAMPLES: u32 = 33;
/// World x and z of sample 0 of the relief heightfield.
pub const RELIEF_OFFSET: f32 = -16.0;

/// The relief: 0.15 m bumps on a 0.1 slope along x.
pub fn relief(x: f32, z: f32) -> f32 {
    0.1 * x + 0.15 * (1.3 * x).sin() * (1.1 * z).cos()
}

/// The relief as a 33 x 33 heightfield covering x and z in `[-16, 16]`.
pub fn relief_terrain() -> Shape {
    let n = RELIEF_SAMPLES as usize;
    let samples: Vec<f32> = (0..n * n)
        .map(|i| {
            relief(
                RELIEF_OFFSET + (i % n) as f32,
                RELIEF_OFFSET + (i / n) as f32,
            )
        })
        .collect();
    let settings =
        HeightFieldSettings::default().offset(Vec3::new(RELIEF_OFFSET, 0.0, RELIEF_OFFSET));
    Shape::new_height_field(RELIEF_SAMPLES, &samples, &settings).unwrap()
}

/// The object layers of a ragdoll world: statics and ragdoll parts.
#[derive(Clone, Copy, Debug)]
pub struct RagdollLayers {
    pub fixed: ObjectLayer,
    pub ragdoll: ObjectLayer,
}

/// Layers where ragdoll parts collide with statics and with each other's ragdolls.
pub fn ragdoll_layers() -> (CollisionLayers, RagdollLayers) {
    let mut layers = CollisionLayers::new(2);
    let fixed = layers.add_object_layer(BroadPhaseLayer::new(0));
    let ragdoll = layers.add_object_layer(BroadPhaseLayer::new(1));
    layers.enable_collision(ragdoll, fixed);
    layers.enable_collision(ragdoll, ragdoll);
    (layers, RagdollLayers { fixed, ragdoll })
}

/// A ragdoll world without engine gravity, stepped by `worker_threads` workers.
pub fn ragdoll_world(worker_threads: u32) -> (PhysicsWorld, RagdollLayers) {
    let (layers, ids) = ragdoll_layers();
    let world = PhysicsWorld::new(
        WorldSettings::default()
            .gravity(Vec3::ZERO)
            .worker_threads(worker_threads)
            .layers(layers),
    )
    .unwrap();
    (world, ids)
}
