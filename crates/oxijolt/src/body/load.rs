//! Checks on the forces and torques a body accumulates before a step.

use std::ptr::{null_mut, NonNull};

use oxijolt_sys::*;

use crate::limits::{is_soft_body_force, F32_PRODUCT_HEADROOM};
use crate::math::jolt_rotate;
use crate::{BodyError, Quat, RVec3, Vec3};

/// Unit roundoff of `f32`, 2^-24: rounding a result to `f32` changes it by at most this
/// fraction of it, or by at most `f32::MIN_POSITIVE` when it is that small.
const F32_UNIT_ROUNDOFF: f64 = 1.0 / 16_777_216.0;

/// What [`BodyMut::check_load`](crate::BodyMut::check_load) reads from a dynamic rigid body.
pub(super) struct LoadState {
    pub(super) force: Vec3,
    pub(super) torque: Vec3,
    pub(super) inverse_mass: f32,
    pub(super) inverse_inertia: Vec3,
    pub(super) center_of_mass: RVec3,
}

/// What [`BodyMut::check_load`](crate::BodyMut::check_load) reads from a soft body.
pub(super) struct SoftLoadState {
    force: Vec3,
    rotation: Quat,
    largest_inverse_mass: f32,
    vertex_count: u32,
}

/// The body state that bounds a load, by kind of body.
pub(super) enum Load {
    /// A static or kinematic body, which ignores loads.
    Ignored,
    Rigid(LoadState),
    Soft(SoftLoadState),
}

/// Reads what bounds a load on `body`.
///
/// # Safety
/// `body` is locked (for reading or writing) for the duration of the call.
pub(super) unsafe fn read_load(body: NonNull<JPH_Body>) -> Load {
    let body = body.as_ptr();
    let mut accumulated_force = Vec3::ZERO.to_jph();
    // SAFETY: `body` is locked (function contract). A dynamic body, soft bodies included, has
    // motion properties, so the unchecked getter reads a live member; the getters only read, and
    // every output is a live local that holds the count joltc writes.
    unsafe {
        if !JPH_Body_IsDynamic(body) {
            return Load::Ignored;
        }
        JPH_Body_GetAccumulatedForce(body, &mut accumulated_force);
        if JPH_Body_IsSoftBody(body) {
            let vertex_count = JPH_Body_GetSoftBodyVertexCount(body);
            let mut inverse_masses = vec![0.0_f32; vertex_count as usize];
            JPH_Body_GetSoftBodyVertices(
                body,
                null_mut(),
                null_mut(),
                inverse_masses.as_mut_ptr(),
                vertex_count,
            );
            let mut rotation = Quat::IDENTITY.to_jph();
            JPH_Body_GetRotation(body, &mut rotation);
            return Load::Soft(SoftLoadState {
                force: Vec3::from_jph(accumulated_force),
                rotation: Quat::from_jph(rotation),
                largest_inverse_mass: inverse_masses.into_iter().fold(0.0, f32::max),
                vertex_count,
            });
        }
        let motion = JPH_Body_GetMotionProperties(body);
        let mut accumulated_torque = Vec3::ZERO.to_jph();
        let mut inverse_inertia = Vec3::ZERO.to_jph();
        let mut center_of_mass = RVec3::ZERO.to_jph();
        JPH_Body_GetAccumulatedTorque(body, &mut accumulated_torque);
        JPH_MotionProperties_GetInverseInertiaDiagonal(motion, &mut inverse_inertia);
        JPH_Body_GetCenterOfMassPosition(body, &mut center_of_mass);
        Load::Rigid(LoadState {
            force: Vec3::from_jph(accumulated_force),
            torque: Vec3::from_jph(accumulated_torque),
            inverse_mass: JPH_MotionProperties_GetInverseMassUnchecked(motion),
            inverse_inertia: Vec3::from_jph(inverse_inertia),
            center_of_mass: RVec3::from_jph(center_of_mass),
        })
    }
}

/// The length of `v`, in `f64`.
pub(super) fn length(v: [f64; 3]) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

/// `a + b + c` per component, in `f64`.
pub(super) fn sum(a: Vec3, b: Vec3, c: [f64; 3]) -> [f64; 3] {
    let [ax, ay, az] = [a.x, a.y, a.z].map(f64::from);
    let [bx, by, bz] = [b.x, b.y, b.z].map(f64::from);
    [ax + bx + c[0], ay + by + c[1], az + bz + c[2]]
}

/// The force to add to a soft body for the world-space `force`: `force` in the body frame Jolt
/// accumulates it in, after checking the accumulated force against the soft body force bound
/// of [`limits`] (Jolt adds `F · w / N` per vertex, `SoftBodyMotionProperties.cpp:334`).
///
/// [`limits`]: crate::limits
pub(super) fn soft_body_force(state: &SoftLoadState, force: Vec3) -> Result<Vec3, BodyError> {
    // Jolt converts only gravity into the body frame (`InitializeUpdateContext`) and adds the
    // accumulated force to the vertex velocities, which are stored in that frame.
    let local = jolt_rotate(state.rotation.conjugated(), force);
    let new_force = sum(state.force, local, [0.0; 3]);
    // At most `MAX_ACCELERATION` for a vertex of this soft body, and at most
    // `MAX_ACCELERATION * MAX_MASS` in all.
    require(
        is_soft_body_force(new_force, state.largest_inverse_mass, state.vertex_count),
        "accumulated soft body force would exceed the limits acceleration bounds",
    )?;
    Ok(local)
}

/// The torque of a force at a point: the exact value, and how far Jolt's `f32` cross product
/// can be from it.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(super) struct PointTorque {
    /// `(point - centre_of_mass) × force`, exact for the `f32` lever and force.
    pub(super) exact: [f64; 3],
    /// Per component, the largest difference between Jolt's `f32` result and `exact`.
    pub(super) rounding: [f64; 3],
}

impl PointTorque {
    /// Per component, the largest magnitude that `base` plus the torque Jolt computes can have.
    pub(super) fn largest_with(&self, base: [f64; 3]) -> [f64; 3] {
        std::array::from_fn(|i| (base[i] + self.exact[i]).abs() + self.rounding[i])
    }
}

/// The torque of `force` at `point` on a body whose centre of mass is `center_of_mass`; an
/// error when Jolt's `f32` arithmetic for it could overflow. See [docs/limits.md#impulses].
///
/// [docs/limits.md#impulses]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#impulses
pub(super) fn point_torque(
    force: Vec3,
    point: RVec3,
    center_of_mass: RVec3,
) -> Result<PointTorque, BodyError> {
    // Jolt converts the lever to `f32` (`Vec3(inPosition - mPosition)`). `Real` is `f32`
    // without the `double-precision` feature, so the casts are no-ops there.
    #[allow(clippy::unnecessary_cast)]
    let lever = [
        (point.x - center_of_mass.x) as f32,
        (point.y - center_of_mass.y) as f32,
        (point.z - center_of_mass.z) as f32,
    ];
    require(
        lever.iter().all(|c| c.is_finite()),
        "point is too far from the body's centre of mass",
    )?;
    let lever = lever.map(f64::from);
    let force = [force.x, force.y, force.z].map(f64::from);
    let products_fit = (0..3)
        .all(|i| (0..3).all(|j| i == j || (lever[i] * force[j]).abs() <= F32_PRODUCT_HEADROOM));
    require(
        products_fit,
        "force at this point would overflow Jolt's torque arithmetic",
    )?;
    // Component `i` is `lever[a] * force[b] - lever[b] * force[a]`; both products are exact in
    // `f64`.
    let products = |i: usize| {
        let (a, b) = ((i + 1) % 3, (i + 2) % 3);
        (lever[a] * force[b], lever[b] * force[a])
    };
    let exact = std::array::from_fn(|i| {
        let (p, q) = products(i);
        p - q
    });
    let rounding = std::array::from_fn(|i| {
        let (p, q) = products(i);
        cross_product_rounding(p, q)
    });
    Ok(PointTorque { exact, rounding })
}

/// How far Jolt's `f32` value of `p - q` can be from the exact one, for products `p` and `q` of
/// `f32` numbers: `u · (|p - q| + 2 · (|p| + |q|)) + 4 · f32::MIN_POSITIVE`, `u` being
/// [`F32_UNIT_ROUNDOFF`]. Jolt rounds both products and the difference, or, where the compiler
/// fuses a product into the subtraction, one product and the fused result
/// ([docs/limits.md#impulses]). For a lever parallel to the force `p - q` is 0 while Jolt's
/// value is not.
///
/// [docs/limits.md#impulses]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#impulses
fn cross_product_rounding(p: f64, q: f64) -> f64 {
    F32_UNIT_ROUNDOFF * ((p - q).abs() + 2.0 * (p.abs() + q.abs()))
        + 4.0 * f64::from(f32::MIN_POSITIVE)
}

pub(super) fn require(valid: bool, what: &'static str) -> Result<(), BodyError> {
    if valid {
        Ok(())
    } else {
        Err(BodyError::InvalidValue(what))
    }
}
