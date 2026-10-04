//! Vector helpers in `f64` and bit views shared by the integration tests.

use oxijolt::*;

pub type V3 = [f64; 3];

pub fn add(a: V3, b: V3) -> V3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

pub fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

pub fn scale(a: V3, s: f64) -> V3 {
    a.map(|c| c * s)
}

pub fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

pub fn norm(a: V3) -> f64 {
    dot(a, a).sqrt()
}

pub fn normalize(a: V3) -> V3 {
    scale(a, 1.0 / norm(a))
}

// `Real` is already `f64` with the `double-precision` feature.
#[allow(clippy::useless_conversion)]
pub fn v3(p: RVec3) -> V3 {
    [f64::from(p.x), f64::from(p.y), f64::from(p.z)]
}

pub fn f3(v: Vec3) -> V3 {
    [f64::from(v.x), f64::from(v.y), f64::from(v.z)]
}

pub fn rvec3(p: V3) -> RVec3 {
    RVec3::new(p[0] as Real, p[1] as Real, p[2] as Real)
}

pub fn vec3(v: V3) -> Vec3 {
    Vec3::new(v[0] as f32, v[1] as f32, v[2] as f32)
}

/// Rotates `v` by the unit quaternion `q`, in f64.
pub fn rotate(q: Quat, v: V3) -> V3 {
    let [x, y, z, w] = [q.x, q.y, q.z, q.w].map(f64::from);
    let u = [x, y, z];
    let t = scale(cross(u, v), 2.0);
    add(add(v, scale(t, w)), cross(u, t))
}

pub fn cross(a: V3, b: V3) -> V3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// `value` as `f64`, which it already is with the `double-precision` feature.
#[allow(clippy::useless_conversion)]
pub fn wide(value: Real) -> f64 {
    f64::from(value)
}

/// The bits of each component of `v`.
pub fn bits(v: Vec3) -> [u32; 3] {
    <[f32; 3]>::from(v).map(f32::to_bits)
}
