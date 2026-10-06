//! Conversions between the binding's math types and glam's, which the playground computes with.

use oxijolt::{Quat, RVec3, Vec3};

/// A position from `f32` components.
// `Real` is `f64` with the binding's `double-precision` feature.
#[allow(clippy::useless_conversion)]
pub fn rvec([x, y, z]: [f32; 3]) -> RVec3 {
    RVec3::new(x.into(), y.into(), z.into())
}

/// `p` in `f32`, the precision the renderer draws in.
#[allow(clippy::unnecessary_cast)]
pub fn position_f32(p: RVec3) -> [f32; 3] {
    [p.x as f32, p.y as f32, p.z as f32]
}

/// A position as a glam vector.
pub fn position(p: RVec3) -> glam::Vec3 {
    glam::Vec3::from(position_f32(p))
}

/// A binding vector as a glam vector.
pub fn glam(v: Vec3) -> glam::Vec3 {
    glam::Vec3::from(<[f32; 3]>::from(v))
}

/// A glam vector as a binding vector.
pub fn vec3(v: glam::Vec3) -> Vec3 {
    Vec3::from(v.to_array())
}

/// A binding rotation as a glam rotation.
pub fn glam_quat(q: Quat) -> glam::Quat {
    glam::Quat::from_array(q.into())
}

/// A glam rotation as a binding rotation, normalized.
pub fn quat(q: glam::Quat) -> Quat {
    Quat::from(q.normalize().to_array())
}

/// The rotation by `angle` radians about `axis`, which need not be unit length.
pub fn about_axis(axis: [f32; 3], angle: f32) -> Quat {
    quat(glam::Quat::from_axis_angle(
        glam::Vec3::from(axis).normalize(),
        angle,
    ))
}
