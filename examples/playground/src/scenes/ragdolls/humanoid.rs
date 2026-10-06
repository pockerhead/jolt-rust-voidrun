//! A 12-capsule, 80 kg humanoid ragdoll standing along +Y and facing +Z in a T-pose, its pelvis
//! at the origin: swing-twist spine, neck and shoulders, hinged elbows and knees, six-DOF hips
//! with asymmetric limits and 4 Hz motors, and 1 N·m of friction on every joint so it comes to
//! rest.

use oxijolt::{
    BodySettings, HingeConstraintSettings, JointTransform, MotorSettings, ObjectLayer,
    RagdollJoint, RagdollPart, RagdollSettings, SixDofAxis, SixDofConstraintAxis,
    SixDofConstraintSettings, Skeleton, SkeletonJoint, SkeletonPose, SpringSettings,
    SwingTwistConstraintSettings, SwingType, Vec3,
};

use crate::math::{about_axis, rvec};
use crate::scene::Result;
use crate::visual::{Shaped, Visual};

/// Total mass, kg.
const MASS: f32 = 80.0;
/// Friction torque of every joint, N·m.
const JOINT_FRICTION: f32 = 1.0;

/// One part: name, parent, mass fraction, capsule half height and radius, bind centre, and
/// whether the capsule lies along X (the arms) instead of Y.
pub struct Part {
    pub name: &'static str,
    pub parent: Option<u32>,
    mass_fraction: f32,
    half_height: f32,
    radius: f32,
    pub centre: [f32; 3],
    along_x: bool,
}

const fn part(
    name: &'static str,
    parent: Option<u32>,
    mass_fraction: f32,
    (half_height, radius): (f32, f32),
    centre: [f32; 3],
    along_x: bool,
) -> Part {
    Part {
        name,
        parent,
        mass_fraction,
        half_height,
        radius,
        centre,
        along_x,
    }
}

/// The parts in skeleton order; parents come before children.
pub const PARTS: [Part; 12] = [
    part("pelvis", None, 0.142, (0.04, 0.12), [0.0, 0.0, 0.0], false),
    part(
        "abdomen",
        Some(0),
        0.139,
        (0.04, 0.10),
        [0.0, 0.26, 0.0],
        false,
    ),
    part(
        "chest",
        Some(1),
        0.216,
        (0.07, 0.13),
        [0.0, 0.54, 0.0],
        false,
    ),
    part(
        "head",
        Some(2),
        0.081,
        (0.04, 0.10),
        [0.0, 0.86, 0.0],
        false,
    ),
    part(
        "upper_arm_l",
        Some(2),
        0.028,
        (0.10, 0.045),
        [0.255, 0.60, 0.0],
        true,
    ),
    part(
        "forearm_l",
        Some(4),
        0.022,
        (0.11, 0.04),
        [0.53, 0.60, 0.0],
        true,
    ),
    part(
        "upper_arm_r",
        Some(2),
        0.028,
        (0.10, 0.045),
        [-0.255, 0.60, 0.0],
        true,
    ),
    part(
        "forearm_r",
        Some(6),
        0.022,
        (0.11, 0.04),
        [-0.53, 0.60, 0.0],
        true,
    ),
    part(
        "thigh_l",
        Some(0),
        0.100,
        (0.14, 0.065),
        [0.09, -0.305, 0.0],
        false,
    ),
    part(
        "shin_l",
        Some(8),
        0.061,
        (0.15, 0.05),
        [0.09, -0.69, 0.0],
        false,
    ),
    part(
        "thigh_r",
        Some(0),
        0.100,
        (0.14, 0.065),
        [-0.09, -0.305, 0.0],
        false,
    ),
    part(
        "shin_r",
        Some(10),
        0.061,
        (0.15, 0.05),
        [-0.09, -0.69, 0.0],
        false,
    ),
];

/// How far below the pelvis the feet are, metres.
pub const FEET: f32 = 0.89;

const X: Vec3 = Vec3::new(1.0, 0.0, 0.0);
const Y: Vec3 = Vec3::new(0.0, 1.0, 0.0);

fn neg(v: Vec3) -> Vec3 {
    Vec3::new(-v.x, -v.y, -v.z)
}

/// The joint of part `index` to its parent; `None` for the pelvis.
fn joint(index: usize) -> Option<RagdollJoint> {
    let cone = |at: [f32; 3], twist_axis: Vec3, plane_axis: Vec3, half: f32, twist: f32| {
        RagdollJoint::SwingTwist(
            SwingTwistConstraintSettings::new(rvec(at), twist_axis, plane_axis)
                .half_cone_angles(half, half)
                .twist_limits(-twist, twist)
                .max_friction_torque(JOINT_FRICTION),
        )
    };
    let hinge = |at: [f32; 3], axis: Vec3, normal: Vec3, min: f32, max: f32| {
        RagdollJoint::Hinge(
            HingeConstraintSettings::new(rvec(at), axis, normal)
                .limits(min, max)
                .max_friction_torque(JOINT_FRICTION),
        )
    };
    // The thigh is the twist axis; forward flexion is positive on the right leg and negative on
    // the left, which makes abduction positive on both.
    let hip = |side: f32| {
        let limited = |(min, max): (f32, f32)| SixDofAxis::Limited { min, max };
        let axis_y = if side > 0.0 { X } else { neg(X) };
        let flexion = if side > 0.0 { (-1.6, 0.3) } else { (-0.3, 1.6) };
        let motor = MotorSettings::default().spring(SpringSettings::FrequencyAndDamping {
            frequency: 4.0,
            damping: 1.0,
        });
        let mut settings =
            SixDofConstraintSettings::new(rvec([0.09 * side, -0.11, 0.0]), neg(Y), axis_y)
                .swing_type(SwingType::Pyramid)
                .axis(SixDofConstraintAxis::RotationX, limited((-0.3, 0.3)))
                .axis(SixDofConstraintAxis::RotationY, limited(flexion))
                .axis(SixDofConstraintAxis::RotationZ, limited((-0.2, 0.5)));
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
                .motor(axis, motor);
        }
        RagdollJoint::SixDof(settings)
    };
    Some(match index {
        1 => cone([0.0, 0.14, 0.0], Y, X, 0.4, 0.3),
        2 => cone([0.0, 0.37, 0.0], Y, X, 0.3, 0.3),
        3 => cone([0.0, 0.73, 0.0], Y, X, 0.6, 0.8),
        4 => cone([0.12, 0.60, 0.0], X, Y, 1.3, 0.5),
        6 => cone([-0.12, 0.60, 0.0], neg(X), Y, 1.3, 0.5),
        // Elbows flex the forearm forward (+Z); knees flex the shin backward.
        5 => hinge([0.39, 0.60, 0.0], neg(Y), X, 0.0, 2.4),
        7 => hinge([-0.39, 0.60, 0.0], Y, neg(X), 0.0, 2.4),
        9 | 11 => hinge(
            [PARTS[index].centre[0], -0.50, 0.0],
            neg(X),
            neg(Y),
            -2.4,
            0.0,
        ),
        8 => hip(1.0),
        10 => hip(-1.0),
        _ => return None,
    })
}

/// The bind rotation of a part: identity, or a quarter turn about Z for the arms.
fn bind_rotation(part: &Part) -> oxijolt::Quat {
    if part.along_x {
        about_axis([0.0, 0.0, 1.0], -std::f32::consts::FRAC_PI_2)
    } else {
        oxijolt::Quat::IDENTITY
    }
}

/// The skeleton of the parts.
pub fn skeleton() -> Result<Skeleton> {
    let joints: Vec<SkeletonJoint<'_>> = PARTS
        .iter()
        .map(|part| SkeletonJoint {
            name: part.name,
            parent: part.parent,
        })
        .collect();
    Ok(Skeleton::new(&joints)?)
}

/// The ragdoll's settings, its parts in `layer`, and the description of each part's capsule.
pub fn settings(layer: ObjectLayer) -> Result<(RagdollSettings, Vec<Visual>)> {
    let shapes = PARTS
        .iter()
        .map(|part| Shaped::capsule(part.half_height, part.radius))
        .collect::<Result<Vec<_>>>()?;
    let parts: Vec<RagdollPart<'_>> = PARTS
        .iter()
        .zip(&shapes)
        .enumerate()
        .map(|(index, (part, shaped))| RagdollPart {
            shape: &shaped.shape,
            body: BodySettings::new_dynamic()
                .object_layer(layer)
                .position(rvec(part.centre))
                .rotation(bind_rotation(part))
                .mass(MASS * part.mass_fraction)
                .friction(0.8)
                .linear_damping(0.1)
                .angular_damping(0.2),
            joint: joint(index),
        })
        .collect();
    let settings = RagdollSettings::new(&skeleton()?, &parts)?;
    Ok((
        settings,
        shapes.into_iter().map(|shaped| shaped.visual).collect(),
    ))
}

/// The bind pose with the pelvis at `pelvis`.
pub fn bind_pose(pelvis: [f32; 3]) -> SkeletonPose {
    SkeletonPose {
        root_offset: rvec(pelvis),
        joints: PARTS
            .iter()
            .map(|part| JointTransform {
                translation: part.centre.into(),
                rotation: bind_rotation(part),
            })
            .collect(),
    }
}
