//! Where a study scene sits in the world and which way is up for the character.
//!
//! Planar scenes are built in local coordinates (slope along +x, up +y) and placed in the world
//! by a [`Frame`]; radial scenes sit on the walker fixtures' planet, whose up is radial.

use oxijolt::*;

use crate::common::math::{add, cross, dot, scale, sub, V3};
use crate::common::quat_about;
use crate::common::walker::{tangent, up_at};

/// A rigid placement of local coordinates in the world.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Frame {
    pub origin: V3,
    pub rotation: Quat,
}

impl Frame {
    /// Local coordinates are world coordinates.
    pub const IDENTITY: Self = Self {
        origin: [0.0; 3],
        rotation: Quat::IDENTITY,
    };

    /// A frame turned by 0.4 rad about `(1, 0, 1)` and moved off the origin, so no axis of the
    /// scene is a world axis.
    pub fn tilted() -> Self {
        let axis = std::f32::consts::FRAC_1_SQRT_2;
        Self {
            origin: [3.0, -2.0, 5.0],
            rotation: quat_about(Vec3::new(axis, 0.0, axis), 0.4),
        }
    }

    pub fn to_world_point(self, local: V3) -> V3 {
        add(self.origin, self.to_world_dir(local))
    }

    pub fn to_world_dir(self, local: V3) -> V3 {
        rotate_unit(self.rotation, local, 1.0)
    }

    pub fn to_local_point(self, world: V3) -> V3 {
        self.to_local_dir(sub(world, self.origin))
    }

    pub fn to_local_dir(self, world: V3) -> V3 {
        rotate_unit(self.rotation, world, -1.0)
    }

    /// The world rotation of something turned by `local` in this frame.
    pub fn to_world_rotation(self, local: Quat) -> Quat {
        product(self.rotation, local)
    }

    /// The frame's up, local +Y.
    pub fn up(&self) -> V3 {
        self.to_world_dir([0.0, 1.0, 0.0])
    }
}

/// `v` rotated by `q` normalised in `f64`, or by its inverse when `sign` is -1.
fn rotate_unit(q: Quat, v: V3, sign: f64) -> V3 {
    let [x, y, z, w] = [q.x, q.y, q.z, q.w].map(f64::from);
    let length = (x * x + y * y + z * z + w * w).sqrt();
    let u = scale([x, y, z], sign / length);
    let w = w / length;
    let t = scale(cross(u, v), 2.0);
    add(add(v, scale(t, w)), cross(u, t))
}

/// The Hamilton product `a * b` (apply `b`, then `a`), computed in `f64`.
pub fn product(a: Quat, b: Quat) -> Quat {
    let [ax, ay, az, aw] = [a.x, a.y, a.z, a.w].map(f64::from);
    let [bx, by, bz, bw] = [b.x, b.y, b.z, b.w].map(f64::from);
    let q = [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ];
    let length = q.iter().map(|c| c * c).sum::<f64>().sqrt();
    let [x, y, z, w] = q.map(|c| (c / length) as f32);
    Quat::from_xyzw(x, y, z, w)
}

/// How the character's up is chosen each tick.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum UpPolicy {
    /// The frame's constant up.
    Frame(Frame),
    /// Radial up on the walker fixtures' planet of radius 99.
    Radial,
}

impl UpPolicy {
    /// The up at body origin `origin`.
    pub fn up_at(&self, origin: V3) -> V3 {
        match self {
            Self::Frame(frame) => frame.up(),
            Self::Radial => up_at(origin),
        }
    }

    /// `direction` (world) flattened onto the plane normal to the up at `origin` and scaled to
    /// `metres`.
    pub fn tangent(&self, origin: V3, direction: V3, metres: f64) -> V3 {
        match self {
            Self::Frame(frame) => {
                let up = frame.up();
                let flat = sub(direction, scale(up, dot(direction, up)));
                scale(flat, metres / dot(flat, flat).sqrt())
            }
            Self::Radial => tangent(origin, direction, metres),
        }
    }
}
