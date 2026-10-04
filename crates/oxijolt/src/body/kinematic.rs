//! The velocities Jolt's `MoveKinematic` gives a body, computed before Jolt writes them.

use std::ptr::NonNull;

use oxijolt_sys::*;

use crate::math::{jolt_angular_velocity, jolt_product, jolt_rotate};
use crate::{BodyId, Quat, RVec3, Real, Vec3};

/// The linear and angular velocity Jolt's `Body::MoveKinematic` gives body `id` to reach
/// `position` and `rotation` in `delta_time` (`Body.cpp:81-95`, `MotionProperties.inl:9-21`),
/// before Jolt masks the locked axes, computed with Jolt's own operations so that they have the
/// bits Jolt writes. `None` when the rotation from the body's rotation to `rotation` is not a
/// unit quaternion within Jolt's tolerance, for which Jolt asserts.
///
/// # Safety
/// `body_interface` belongs to a live world that holds the rigid body `id`, which nothing removes
/// or changes during the call, and this thread holds no body lock.
pub(crate) unsafe fn kinematic_velocities(
    body_interface: NonNull<JPH_BodyInterface>,
    id: BodyId,
    position: RVec3,
    rotation: Quat,
    delta_time: f32,
) -> Option<(Vec3, Vec3)> {
    let interface = body_interface.as_ptr();
    let (mut center_of_mass, mut body_rotation) =
        (RVec3::new(0.0, 0.0, 0.0).to_jph(), Quat::IDENTITY.to_jph());
    let mut shape_center_of_mass = Vec3::ZERO.to_jph();
    // SAFETY: the caller's contract: the world holds the body and this thread holds no body
    // lock, so the getters can lock it. They write live locals. The shape pointer is the body's
    // own shape, which the body keeps alive during the call that reads its centre of mass.
    unsafe {
        JPH_BodyInterface_GetCenterOfMassPosition(interface, id.to_raw(), &mut center_of_mass);
        JPH_BodyInterface_GetRotation(interface, id.to_raw(), &mut body_rotation);
        let shape = JPH_BodyInterface_GetShape(interface, id.to_raw());
        JPH_Shape_GetCenterOfMass(shape, &mut shape_center_of_mass);
    }
    let center_of_mass = RVec3::from_jph(center_of_mass);
    let offset = jolt_rotate(rotation, Vec3::from_jph(shape_center_of_mass));
    let delta = |target: Real, offset: f32, current: Real| {
        // Jolt narrows the position difference to `f32` (`Vec3(new_com - mPosition)`).
        #[allow(clippy::unnecessary_cast)]
        let delta = (target + Real::from(offset) - current) as f32;
        delta / delta_time
    };
    let linear = Vec3::new(
        delta(position.x, offset.x, center_of_mass.x),
        delta(position.y, offset.y, center_of_mass.y),
        delta(position.z, offset.z, center_of_mass.z),
    );
    let turn = jolt_product(rotation, Quat::from_jph(body_rotation).conjugated());
    let angular = jolt_angular_velocity(turn, delta_time)?;
    Some((linear, angular))
}
