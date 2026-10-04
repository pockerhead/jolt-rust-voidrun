//! `f64` vector arithmetic for the checks that replay Jolt's geometry before Jolt sees it.

use crate::Vec3;

pub(super) type V3 = [f64; 3];

pub(super) fn v3(v: Vec3) -> V3 {
    [v.x, v.y, v.z].map(f64::from)
}

pub(super) fn add(a: V3, b: V3) -> V3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

pub(super) fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

pub(super) fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

pub(super) fn length_sq(a: V3) -> f64 {
    dot(a, a)
}

pub(super) fn cross(a: V3, b: V3) -> V3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// A 3x3 matrix, rows first.
pub(super) type M3 = [[f64; 3]; 3];

pub(super) fn diagonal(d: V3) -> M3 {
    [[d[0], 0.0, 0.0], [0.0, d[1], 0.0], [0.0, 0.0, d[2]]]
}

pub(super) fn mul_vec(m: &M3, v: V3) -> V3 {
    [0, 1, 2].map(|row| dot(m[row], v))
}

pub(super) fn mul(a: &M3, b: &M3) -> M3 {
    [0, 1, 2].map(|row| [0, 1, 2].map(|col| (0..3).map(|k| a[row][k] * b[k][col]).sum()))
}

/// The rotation matrix of the unit quaternion `(x, y, z, w)`.
pub(super) fn rotation([x, y, z, w]: [f64; 4]) -> M3 {
    [
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - w * z),
            2.0 * (x * z + w * y),
        ],
        [
            2.0 * (x * y + w * z),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - w * x),
        ],
        [
            2.0 * (x * z - w * y),
            2.0 * (y * z + w * x),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ]
}
