//! Jolt's estimate of a new contact's impulses (`EstimateCollisionResponse`), for impact sounds
//! and damage.

use oxijolt_sys::*;

use super::ContactSettings;
use crate::Vec3;

/// What Jolt estimates a new contact does to its two bodies, before the solver runs (Jolt
/// `EstimateCollisionResponse`), from [`ContactEvent::Added`](crate::ContactEvent::Added) when
/// [`EventSettings::collision_estimates`](crate::EventSettings::collision_estimates) is on.
///
/// Jolt solves the contact of the two bodies alone, with the contact's friction and restitution
/// and the world's restitution threshold and velocity iterations, so the result is exact only
/// for an isolated pair: other contacts and constraints on either body in the same step are not
/// part of it. It does not model the contact's mass and inertia scales, surface velocities,
/// locked axes or the speed limits. A speculative contact (negative penetration depth) is
/// estimated as if the bodies touched. A contact found in the continuous collision stage is
/// estimated from the velocities after the step's solve, at the time-of-impact points, which
/// makes its angular part approximate.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct CollisionEstimate {
    /// Linear velocity of body 1 after the impact, m/s.
    pub linear_velocity1: Vec3,
    /// Angular velocity of body 1 after the impact, rad/s.
    pub angular_velocity1: Vec3,
    /// Linear velocity of body 2 after the impact, m/s.
    pub linear_velocity2: Vec3,
    /// Angular velocity of body 2 after the impact, rad/s.
    pub angular_velocity2: Vec3,
    /// Impulse along the contact normal at each point of the manifold, in the order of
    /// [`ContactManifold::points`](crate::ContactManifold::points), N·s. Their sum is the
    /// impact's total normal impulse.
    pub contact_impulses: Vec<f32>,
    /// First friction direction, a unit vector perpendicular to the normal.
    pub tangent1: Vec3,
    /// Second friction direction, `normal × tangent1`.
    pub tangent2: Vec3,
    /// Friction impulse along `tangent1`, N·s.
    pub friction_impulse1: f32,
    /// Friction impulse along `tangent2`, N·s.
    pub friction_impulse2: f32,
    /// Friction impulse about the normal, N·m·s.
    pub angular_friction_impulse: f32,
}

/// The world's solver settings the estimate uses (Jolt `PhysicsSettings`).
#[derive(Clone, Copy, Debug)]
pub(super) struct SolverSettings {
    pub(super) min_velocity_for_restitution: f32,
    pub(super) num_velocity_steps: u32,
}

/// joltc's estimation result, zeroed when created; dropping it frees the impulse array joltc
/// allocated, exactly once, also while unwinding.
struct EstimationResult(JPH_CollisionEstimationResult);

impl EstimationResult {
    fn new() -> Self {
        // SAFETY: an all-zero `JPH_CollisionEstimationResult` is valid: floats, a count of 0 and
        // a null pointer, which `JPH_CollisionEstimationResult_FreeMembers` leaves alone.
        Self(unsafe { std::mem::zeroed() })
    }

    /// The contact impulses joltc wrote.
    fn contact_impulses(&self) -> Vec<f32> {
        let result = &self.0;
        if result.contactImpulseCount == 0 || result.contactImpulses.is_null() {
            return Vec::new();
        }
        // SAFETY: joltc allocated `contactImpulseCount` floats at `contactImpulses` and copied
        // Jolt's impulses into them (`JPH_EstimateCollisionResponse`); they live until `drop`.
        unsafe {
            std::slice::from_raw_parts(result.contactImpulses, result.contactImpulseCount as usize)
        }
        .to_vec()
    }
}

impl Drop for EstimationResult {
    fn drop(&mut self) {
        #[cfg(test)]
        tests::FREED.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        // SAFETY: the result is zeroed or filled by one `JPH_EstimateCollisionResponse`, so its
        // impulse array is null with count 0 or joltc's own `malloc`; it is freed only here.
        unsafe { JPH_CollisionEstimationResult_FreeMembers(&mut self.0) };
    }
}

/// Jolt's estimate for the new contact `manifold` between `body1` and `body2`, resolved with
/// `settings`; `None` for a sensor contact or a manifold without points.
///
/// # Safety
/// The arguments are the live bodies and native manifold of a joltc `OnContactAdded` callback.
/// Jolt calls it from `JobFindCollisions` (body access `Read, Read`, `PhysicsSystem.cpp:897`) or
/// from `JobFindCCDContacts` (`:1786`); no job writes velocities while either runs (gravity is
/// applied before them, the solve and `JobResolveCCDContacts` run after them), so the bodies'
/// velocities, masses and inertias stay as Jolt reads them.
pub(super) unsafe fn estimate(
    body1: *const JPH_Body,
    body2: *const JPH_Body,
    manifold: *const JPH_ContactManifold,
    settings: &ContactSettings,
    solver: SolverSettings,
) -> Option<CollisionEstimate> {
    if settings.is_sensor() {
        return None;
    }
    // SAFETY: the manifold is live (contract); the getter only reads it.
    if unsafe { JPH_ContactManifold_GetPointCount(manifold) } == 0 {
        return None;
    }
    let mut result = EstimationResult::new();
    #[cfg(test)]
    tests::CREATED.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
    // SAFETY: the bodies and the manifold are Jolt's own, live for the callback (contract), and
    // Jolt's manifold has as many points on body 2 as on body 1, which Jolt asserts. `result` is
    // a live, zeroed result, which the guard frees.
    unsafe {
        JPH_EstimateCollisionResponse(
            body1,
            body2,
            manifold,
            settings.combined_friction(),
            settings.combined_restitution(),
            solver.min_velocity_for_restitution,
            solver.num_velocity_steps,
            &mut result.0,
        );
    }
    #[cfg(test)]
    tests::panic_if_asked();
    let r = &result.0;
    Some(CollisionEstimate {
        linear_velocity1: Vec3::from_jph(r.linearVelocity1),
        angular_velocity1: Vec3::from_jph(r.angularVelocity1),
        linear_velocity2: Vec3::from_jph(r.linearVelocity2),
        angular_velocity2: Vec3::from_jph(r.angularVelocity2),
        contact_impulses: result.contact_impulses(),
        tangent1: Vec3::from_jph(r.tangent1),
        tangent2: Vec3::from_jph(r.tangent2),
        friction_impulse1: r.frictionImpulse1,
        friction_impulse2: r.frictionImpulse2,
        angular_friction_impulse: r.angularFrictionImpulse,
    })
}

#[cfg(test)]
pub(super) mod tests {
    use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

    /// Estimation results created and freed, for the unwinding test.
    pub(crate) static CREATED: AtomicUsize = AtomicUsize::new(0);
    pub(crate) static FREED: AtomicUsize = AtomicUsize::new(0);
    /// Makes the copy of an estimate panic, for the unwinding test.
    pub(crate) static PANIC_WHILE_COPYING: AtomicBool = AtomicBool::new(false);

    pub(super) fn panic_if_asked() {
        if PANIC_WHILE_COPYING.load(Ordering::SeqCst) {
            panic!("injected estimate copy panic");
        }
    }
}
