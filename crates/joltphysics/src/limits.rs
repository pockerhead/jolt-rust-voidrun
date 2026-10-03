//! The magnitudes the safe API accepts, and why.
//!
//! Jolt checks most magnitudes only with debug assertions (the `asserts` feature) and otherwise
//! computes with whatever it is given, so a finite but huge input can overflow Jolt's `f32`
//! arithmetic inside a step. joltphysics bounds every caller-given magnitude with the constants
//! of this module, so that the arithmetic below stays finite. This module states which Jolt
//! assertion paths the bounds are derived for, which are only covered by tests, and which are
//! not covered at all. It does not claim that no accepted input can reach a Jolt assertion.
//!
//! # Frame and units
//! Positions are in metres in the world frame, which must keep every component of a caller-given
//! position within [`MAX_POSITION`]; [`PhysicsWorld::rebase`] moves the world into a new frame.
//! Shapes and local offsets are in metres around a shape's centre of mass, within
//! [`MAX_SHAPE_EXTENT`]. Velocities are in m/s and rad/s, accelerations in m/s² and rad/s², masses
//! in kg. [`MAX_LINEAR_VELOCITY`] and [`MAX_ANGULAR_VELOCITY`] are Jolt's own defaults; every
//! other constant is crate policy, chosen from Jolt's documented ranges ("Conventions and
//! Limits", "Big Worlds" in Jolt's `Docs/Architecture.md`) or from the arithmetic below.
//!
//! # Derived bounds
//! These hold with every input at its bound and a time step `dt <= 1` s
//! ([`PhysicsWorld::MAX_DELTA_TIME`]); numbers are rounded.
//! - **Velocities at creation.** Jolt asserts `Length() <= mMaxLinearVelocity` (and the angular
//!   counterpart) when it creates a body (`Body.cpp:424`, `MotionProperties.h:48`). The checks
//!   compare Jolt's own `Vec3::Length`, called through joltc, with Jolt's defaults; a test creates
//!   bodies on the bound in 64 directions, which the asserts leg runs.
//! - **Integration.** Each step Jolt adds gravity times the gravity factor and the accumulated
//!   force times the inverse mass to the velocity, then asserts that the squared speed is finite
//!   (`MotionProperties.inl:26-28`) before clamping it. Before the clamp
//!   `|v| <= 500 + (|gf|·|g| + |F|/m)·dt <= 500 + 1000·5e8 + 2·5e8`, about 5e11 m/s, where the
//!   factor 2 also covers a vehicle's gravity force on its chassis. Its square, about 2.5e23, is
//!   finite. The angular velocity is bounded the same way with [`MAX_ANGULAR_ACCELERATION`] and
//!   the largest principal inverse inertia.
//! - **Force and torque accumulation.** With `mass <= MAX_MASS` an accepted accumulated force is
//!   at most `MAX_ACCELERATION · MAX_MASS`, about 5e14 N, so Jolt's `f32` sums (`Body.h:183,191`)
//!   and `mInvMass * F` (`MotionProperties.inl:134`) cannot overflow. For a force at a point the
//!   lever must be finite in `f32` and every product `lever_i · force_j` at most 1e37, so Jolt's
//!   cross product (`Body.inl:127-131`) stays finite even when its two products cancel.
//! - **Vehicle gravity.** Jolt adds `gravity / inverse_mass` to the chassis
//!   (`VehicleConstraint::OnStep`); the chassis is a dynamic body, so the force is at most
//!   `5e8 · 1e6`, about 5e14 N.
//! - **Character weight and push.** A character presses on what it stands on with the impulse
//!   `mass · |g| · dt` (`CharacterVirtual.cpp:1474-1481`), which gives a body of the smallest
//!   mass at most `1e6 · 5e8 · 1 · 1e3`, about 5e17 m/s, whose square is finite. Its push impulse
//!   is capped at `delta_velocity / inv_effective_mass` (`CharacterVirtual.cpp:795-811`), so the
//!   linear velocity change is at most the relative normal speed whatever the strength.
//! - **Springs.** Jolt derives a stiffness `k` and damping `c` from every spring
//!   (`SpringPart.h:36-55,91-104`). Both stay at most [`MAX_SPRING_COEFFICIENT`]: in stiffness mode
//!   directly, in frequency mode through an upper bound of the effective mass that ragdoll creation
//!   computes from the parts' masses and inertias, including Jolt's `Stabilize`
//!   (`Ragdoll.cpp:135-185`). See [`SpringSettings`](crate::SpringSettings).
//! - **Restitution.** At most 1, so the restitution target speed is at most the approach speed.
//! - **Contact constraint capacity.** [`WorldSettings::MAX_CONTACT_CONSTRAINTS`] stays below the
//!   count above which `ContactConstraintManager::Init` asserts; a native compile-time check pins
//!   it.
//!
//! # Covered by tests only
//! The asserts leg of CI runs every test with Jolt's assertions; these paths are exercised there
//! by scenes with inputs at their bounds, not derived:
//! - contact and constraint impulses the solver generates (heaviest against lightest bodies,
//!   restitution 1, friction `f32::MAX`, motors at the spring bound);
//! - angular velocity from impulses at a lever arm (character pushes and weight impulses,
//!   contacts) on bodies with extreme inertia, such as needle-thin shapes or a far-offset centre
//!   of mass;
//! - query arithmetic on shapes at the extent bound and from frame corners.
//!
//! # Not covered
//! - State the simulation produces itself is not an input and is not checked again: a body Jolt
//!   carries out of the frame, the positions a rebase computes (only checked to be finite), or a
//!   character state restored with [`CharacterMut::restore_state`](crate::CharacterMut::restore_state),
//!   which can only come from [`CharacterRef::save_state`](crate::CharacterRef::save_state).
//! - `RagdollSettings::new_stabilized` reports Jolt's `Stabilize` failing to decompose an
//!   inertia tensor as an error, but Jolt asserts on that path first (`Ragdoll.cpp:158`).
//! - The assertion `errors == EPhysicsUpdateError::None` at the end of every step that drops
//!   contacts (`PhysicsSystem.cpp:679`) is intentional: [`PhysicsWorld::step`] returns the same
//!   errors in its [`StepReport`](crate::StepReport), and joltphysics' assertion handler lets the
//!   process continue for this assertion only.
//!
//! # Audit
//! Every public setter and constructor that takes a magnitude, the rule it applies and the test
//! that covers it at its boundary. "New" rows were added with this policy; "existing" rows name
//! the test that already covered them.
//!
//! | Input | Rule | Test |
//! |---|---|---|
//! | `WorldSettings::gravity`, `PhysicsWorld::set_gravity` | [`MAX_ACCELERATION`] | new: `world_gravity_is_bounded_by_max_acceleration` |
//! | `WorldSettings::max_contact_constraints` | `1..=`[`WorldSettings::MAX_CONTACT_CONSTRAINTS`] | new: `contact_constraint_capacity_is_bounded` |
//! | `WorldSettings::max_bodies`, `worker_threads` | Jolt's and joltphysics' counts | existing: `invalid_settings_are_rejected`, `worker_thread_bounds_are_validated` |
//! | `WorldSettings::max_body_pairs`, `temp_allocator_size` | at least 1; Jolt asserts nothing on their size, and joltc's temp allocator falls back to `malloc` | existing: `invalid_settings_are_rejected` |
//! | `PhysicsWorld::step` delta time | `MIN_DELTA_TIME..=MAX_DELTA_TIME` | existing: `step_rejects_delta_time_above_the_bound`, `step_rejects_delta_time_below_the_bound` |
//! | `PhysicsWorld::rebase` translation | `2 *` [`MAX_POSITION`] per axis; results finite | new: `rebase_translation_is_bounded_by_twice_the_frame` |
//! | `BodySettings::position` | [`MAX_POSITION`] | new: `body_settings_are_bounded` |
//! | `BodySettings::linear_velocity`, `angular_velocity` | [`MAX_LINEAR_VELOCITY`], [`MAX_ANGULAR_VELOCITY`] | new: `body_settings_are_bounded`, `creation_velocities_agree_with_jolts_length_in_many_directions` |
//! | `BodySettings::restitution` | `0..=1` | new: `body_settings_are_bounded` |
//! | `BodySettings::gravity_factor` | [`MAX_GRAVITY_FACTOR`] | new: `body_settings_are_bounded` |
//! | `BodySettings::mass`, `PhysicsWorld::create_body` computed mass | [`MIN_MASS`]`..=`[`MAX_MASS`] for dynamic bodies | new: `body_settings_are_bounded`, `computed_dynamic_mass_is_bounded_and_kinematic_mass_is_not` |
//! | `BodySettings::friction` | finite, at least 0: friction only sets the range Jolt clamps the friction impulse it solves to (`ContactConstraintManager.cpp:1700-1730`) | existing: `invalid_body_settings_are_rejected`; `f32::MAX` in `bodies_at_every_bound_step_finitely` |
//! | `BodySettings::linear_damping`, `angular_damping` | finite, at least 0: Jolt scales by `max(0, 1 - c·dt)` (`MotionProperties.inl:144-145`) | existing: `invalid_damping_is_rejected` |
//! | `BodySettings::rotation`, `BodyMut::set_rotation` | finite unit quaternion | existing: `invalid_body_settings_are_rejected` |
//! | `BodyMut::set_position`, `set_position_and_rotation` | [`MAX_POSITION`] | new: `body_setters_are_bounded_and_rejection_changes_nothing` |
//! | `BodyMut::set_linear_velocity`, `set_angular_velocity` | [`MAX_LINEAR_VELOCITY`], [`MAX_ANGULAR_VELOCITY`] | new: `body_setters_are_bounded_and_rejection_changes_nothing` |
//! | `BodyMut::add_force` | accumulated `|F| / m <=` [`MAX_ACCELERATION`] | new: `forces_are_bounded_by_the_acceleration_they_give` |
//! | `BodyMut::add_torque`, `add_force_at_point` | accumulated torque within [`MAX_ANGULAR_ACCELERATION`]; point within [`MAX_POSITION`]; `f32` torque products | new: `torques_are_bounded_by_the_angular_acceleration_they_give`, `point_torque_rejects_overflowing_products_even_when_they_cancel` |
//! | `BodyMut::reset_forces` | none | existing: `reset_forces_ignores_static_and_kinematic_bodies` |
//! | `CharacterSettings::mass` | `0..=`[`MAX_MASS`] | new: `character_settings_and_setters_are_bounded` |
//! | `CharacterSettings::shape_offset` | [`MAX_SHAPE_EXTENT`] per axis | new: `character_settings_and_setters_are_bounded` |
//! | `CharacterSettings::predictive_contact_distance`, `character_padding`, `collision_tolerance` | `0..=`[`MAX_SHAPE_EXTENT`] (tolerance positive) | new: `character_settings_and_setters_are_bounded` |
//! | `CharacterSettings::max_strength` and the other settings | existing ranges; strength needs no bound (see above) | existing: `invalid_settings_and_poses_are_rejected_without_side_effects` |
//! | `PhysicsWorld::create_character` position, `CharacterMut::set_position` | [`MAX_POSITION`] | new: `character_settings_and_setters_are_bounded` |
//! | `CharacterMut::set_linear_velocity` | [`MAX_LINEAR_VELOCITY`]; Jolt does not clamp a character | new: `character_settings_and_setters_are_bounded` |
//! | `CharacterMut::set_up`, `set_rotation` | unit vector, unit quaternion | existing: `invalid_settings_and_poses_are_rejected_without_side_effects` |
//! | `PhysicsWorld::update_character` gravity | [`MAX_ACCELERATION`] | new: `character_update_gravity_and_steps_are_bounded` |
//! | `ExtendedUpdateSettings` steps and forward distances | [`MAX_SHAPE_EXTENT`] | new: `character_update_gravity_and_steps_are_bounded` |
//! | `CharacterMut::restore_state` | only states from `save_state` exist | existing: `a_restored_state_saves_the_same_bytes` |
//! | `VehicleMut::set_gravity` | [`MAX_ACCELERATION`] | new: `gravity_is_bounded_by_max_acceleration` |
//! | `VehicleSettings`, wheel, engine, transmission, differential, anti-roll bar and tester settings | existing derived checks | existing: `vehicle_values_are_validated`, `wheel_values_are_validated`, `engine_values_are_validated`, `transmission_values_are_validated`, `differential_values_are_validated`, `anti_roll_bars_are_validated`, `collision_testers_are_validated` |
//! | `VehicleMut::set_driver_input`, `set_max_pitch_roll_angle`, `set_collision_tester` | existing ranges | existing: `driver_input_drives_steers_and_brakes`, `invalid_vehicles_create_nothing` |
//! | `SpringSettings::StiffnessAndDamping` | [`MAX_SPRING_COEFFICIENT`] | new: `stiffness_springs_are_bounded_by_the_coefficient` |
//! | `SpringSettings::FrequencyAndDamping` in `RagdollSettings::new`, `new_stabilized` | `B·ω²` and `2·B·ζ·ω` at most [`MAX_SPRING_COEFFICIENT`] | new: `motor_springs_are_bounded_by_the_parts_effective_mass`, `motor_spring_of_1e20_hz_is_rejected` |
//! | `MotorSettings::force_limits`, `torque_limits`; angle limits | finite, `min <= max`; Jolt clamps the motor impulse to `dt · limit` | existing: `motors_and_springs_are_validated`, `swing_twist_limits_are_validated`, `hinge_limits_are_validated`, `six_dof_limits_are_validated` |
//! | constraint frame points | [`MAX_POSITION`] | new: `constraint_frame_points_are_bounded` |
//! | `RagdollSettings::new`, `new_stabilized` part masses | [`MIN_MASS`]`..=`[`MAX_MASS`], also for kinematic parts (`RagdollMut::set_motion_type` can make them dynamic) | new: `part_masses_and_velocities_are_bounded` |
//! | `RagdollMut::set_pose`, `drive_to_pose_using_kinematics`, `drive_to_pose_using_motors` | root offset and positions within [`MAX_POSITION`] | new: `poses_are_validated` |
//! | `RagdollMut::set_linear_and_angular_velocity` | [`MAX_LINEAR_VELOCITY`], [`MAX_ANGULAR_VELOCITY`] | new: `part_masses_and_velocities_are_bounded` |
//! | `Shape::new_box*`, `new_sphere`, `new_cylinder*`, `new_capsule` | dimensions within [`MAX_SHAPE_EXTENT`] | new: `primitive_extents_are_bounded` |
//! | `Shape::new_compound`, `new_offset_center_of_mass` | positions and offset within [`MAX_SHAPE_EXTENT`]; local bounds within it | new: `decorated_and_compound_extents_are_bounded` |
//! | `Shape::new_height_field` | local bounds within [`MAX_SHAPE_EXTENT`] | new: `height_field_extent_is_bounded` |
//! | `HeightFieldSettings`, `CompoundChild::rotation` | existing ranges | existing: `invalid_height_fields_are_rejected`, `empty_or_invalid_compounds_are_rejected` |
//! | `PhysicsWorld::cast_ray` origin | [`MAX_POSITION`] | new: `query_inputs_are_bounded_by_the_frame` |
//! | `RayCast` direction | finite, not zero | existing: `invalid_rays_are_rejected`; new: `a_ray_with_a_huge_finite_direction_is_cast` |
//! | `ShapeCast`, `CollideShape` position | [`MAX_POSITION`] | new: `query_inputs_are_bounded_by_the_frame` |
//! | `ShapeCast` direction | `2 *` [`MAX_POSITION`] per axis | new: `query_inputs_are_bounded_by_the_frame` |
//! | `ShapeCast::target_distance` | `ShapeCast::MAX_TARGET_DISTANCE` | existing: `target_distance_at_the_bound_gives_a_finite_depth` |
//! | `CollideShape::max_separation_distance` | `0..=`[`MAX_SHAPE_EXTENT`] | new: `query_inputs_are_bounded_by_the_frame` |
//! | `DebugLineSettings` (feature `debug-renderer`) | centre within [`MAX_POSITION`], radius at most twice it | new: `center_and_radius_are_bounded_by_the_frame` |
//!
//! [`WorldSettings::MAX_CONTACT_CONSTRAINTS`]: crate::WorldSettings::MAX_CONTACT_CONSTRAINTS

use crate::math::jolt_length;
use crate::{PhysicsWorld, RVec3, Real, Vec3};

/// Largest absolute value of each component of a caller-given world position, in metres:
/// 5 km with `f32` positions, 10 000 km with the `double-precision` feature.
///
/// Crate policy, not a Jolt assertion threshold. Jolt's "Big Worlds" documentation
/// (`Docs/Architecture.md`) says single-precision simulation is accurate within roughly 5 km of
/// the origin, and that double precision handles worlds of thousands of km; at 10 000 km Jolt's
/// `f32` broad phase still has a resolution of about 1 m.
pub const MAX_POSITION: Real = (if core::mem::size_of::<Real>() == 8 {
    1.0e7_f64
} else {
    5.0e3_f64
}) as Real;

/// Largest absolute value of each component of a shape's local bounds (around its centre of
/// mass), of a shape offset and of a local step distance, in metres.
///
/// Crate policy from Jolt's "Conventions and Limits" documentation, which recommends static
/// objects of 0.1 to 2000 m; the bound applies on each side of the centre of mass. It bounds a
/// shape's inertia to at most `6 * mass * MAX_SHAPE_EXTENT²`.
pub const MAX_SHAPE_EXTENT: f32 = 2000.0;

/// Largest linear velocity a caller may give a body or character, in m/s.
///
/// Jolt's default `BodyCreationSettings::mMaxLinearVelocity` (`BodyCreationSettings.h:111`). Jolt
/// asserts `Length() <= mMaxLinearVelocity` when it creates a body (`Body.cpp:424`,
/// `MotionProperties.h:48`) and clamps a body's velocity to it every step.
pub const MAX_LINEAR_VELOCITY: f32 = 500.0;

/// Largest angular velocity a caller may give a body, in rad/s.
///
/// Jolt's default `BodyCreationSettings::mMaxAngularVelocity` (`BodyCreationSettings.h:112`),
/// written as Jolt writes it so the `f32` bits match.
pub const MAX_ANGULAR_VELOCITY: f32 = 0.25 * core::f32::consts::PI * 60.0;

/// Largest length of a caller-given acceleration (world gravity, a character's or vehicle's
/// gravity, the acceleration a body's added forces give it), in m/s²: about 5e8.
///
/// Crate policy, not a Jolt limit. A larger acceleration already reaches Jolt's speed clamp
/// within every step [`PhysicsWorld::step`] accepts, so the bound removes no motion that the
/// clamp keeps.
pub const MAX_ACCELERATION: f32 = MAX_LINEAR_VELOCITY / PhysicsWorld::MIN_DELTA_TIME;

/// Largest angular acceleration a body's added torques may give it, in rad/s²: about 4.71e7.
///
/// Crate policy with the reasoning of [`MAX_ACCELERATION`], for Jolt's angular speed clamp.
pub const MAX_ANGULAR_ACCELERATION: f32 = MAX_ANGULAR_VELOCITY / PhysicsWorld::MIN_DELTA_TIME;

/// Largest absolute gravity factor of a body.
///
/// Crate policy (like [`WorldSettings::MAX_WORKER_THREADS`](crate::WorldSettings::MAX_WORKER_THREADS)).
pub const MAX_GRAVITY_FACTOR: f32 = 1000.0;

/// Smallest mass of a dynamic body or ragdoll part, in kg: an inverse mass of at most 1000 per kg,
/// a 1 cm cube of water.
///
/// Crate policy.
pub const MIN_MASS: f32 = 1.0e-3;

/// Largest mass of a dynamic body, ragdoll part or character, in kg: a 10 m cube of water, the
/// top of the dynamic object sizes Jolt documents.
///
/// Crate policy. It bounds the forces [`MAX_ACCELERATION`] accepts (at most 5e14 N), contact
/// effective masses, a vehicle's gravity force and a character's weight impulse.
pub const MAX_MASS: f32 = 1.0e6;

/// Largest spring stiffness `k` and damping `c` Jolt may derive from a constraint spring
/// (`SpringPart.h:36-55,91-104`).
///
/// Crate policy. `c + dt * k` stays finite (at most 2e30 for `dt <= 1`), so the softness, bias
/// and effective mass Jolt computes from them stay finite.
pub const MAX_SPRING_COEFFICIENT: f32 = 1.0e30;

/// Whether every component of `position` is at most [`MAX_POSITION`] in absolute value.
pub(crate) fn is_in_frame(position: RVec3) -> bool {
    [position.x, position.y, position.z]
        .iter()
        .all(|c| c.abs() <= MAX_POSITION)
}

/// Whether every component of `displacement` is at most `2 * MAX_POSITION` in absolute value:
/// the largest move between two positions in the frame.
pub(crate) fn is_frame_displacement(displacement: RVec3) -> bool {
    [displacement.x, displacement.y, displacement.z]
        .iter()
        .all(|c| c.abs() <= 2.0 * MAX_POSITION)
}

/// [`is_frame_displacement`] for an `f32` vector.
pub(crate) fn is_frame_span(span: Vec3) -> bool {
    [span.x, span.y, span.z]
        .iter()
        .all(|&c| Real::from(c).abs() <= 2.0 * MAX_POSITION)
}

/// Whether every component of `offset` is at most [`MAX_SHAPE_EXTENT`] in absolute value.
pub(crate) fn is_local_offset(offset: Vec3) -> bool {
    [offset.x, offset.y, offset.z]
        .iter()
        .all(|c| c.abs() <= MAX_SHAPE_EXTENT)
}

/// Whether `distance` is finite and within `0..=MAX_SHAPE_EXTENT`.
pub(crate) fn is_local_distance(distance: f32) -> bool {
    (0.0..=MAX_SHAPE_EXTENT).contains(&distance)
}

/// Whether `velocity` is finite and Jolt's own length of it at most [`MAX_LINEAR_VELOCITY`].
pub(crate) fn is_linear_velocity(velocity: Vec3) -> bool {
    velocity.is_finite() && jolt_length(velocity) <= MAX_LINEAR_VELOCITY
}

/// Whether `velocity` is finite and Jolt's own length of it at most [`MAX_ANGULAR_VELOCITY`].
pub(crate) fn is_angular_velocity(velocity: Vec3) -> bool {
    velocity.is_finite() && jolt_length(velocity) <= MAX_ANGULAR_VELOCITY
}

/// Whether `acceleration` is finite and its length, computed in `f64`, at most
/// [`MAX_ACCELERATION`].
pub(crate) fn is_acceleration(acceleration: Vec3) -> bool {
    acceleration.is_finite() && f64_length(acceleration) <= f64::from(MAX_ACCELERATION)
}

/// Whether `factor` is finite and at most [`MAX_GRAVITY_FACTOR`] in absolute value.
pub(crate) fn is_gravity_factor(factor: f32) -> bool {
    factor.abs() <= MAX_GRAVITY_FACTOR
}

/// Whether `mass` is finite and within `MIN_MASS..=MAX_MASS`.
pub(crate) fn is_mass(mass: f32) -> bool {
    (MIN_MASS..=MAX_MASS).contains(&mass)
}

/// The length of `v`, computed in `f64` so that it cannot overflow.
pub(crate) fn f64_length(v: Vec3) -> f64 {
    let [x, y, z] = [v.x, v.y, v.z].map(f64::from);
    (x * x + y * y + z * z).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    const NON_FINITE: [f32; 3] = [f32::NAN, f32::INFINITY, f32::NEG_INFINITY];

    /// The axis-aligned vectors with `value` on one axis, in both directions.
    fn on_axes(value: f32) -> Vec<Vec3> {
        let mut vectors = Vec::new();
        for axis in 0..3 {
            for sign in [1.0, -1.0] {
                let mut v = [0.0; 3];
                v[axis] = sign * value;
                vectors.push(Vec3::from(v));
            }
        }
        vectors
    }

    fn real_on_axes(value: Real) -> Vec<RVec3> {
        let mut vectors = Vec::new();
        for axis in 0..3 {
            for sign in [1.0, -1.0] {
                let mut v = [0.0; 3];
                v[axis] = sign * value;
                vectors.push(RVec3::from(v));
            }
        }
        vectors
    }

    #[test]
    fn max_position_follows_the_precision_of_real() {
        let expected = if core::mem::size_of::<Real>() == 8 {
            1.0e7
        } else {
            5.0e3
        };
        assert_eq!(MAX_POSITION, expected);
    }

    #[test]
    fn position_checks_accept_their_bound_and_reject_beyond() {
        type Check = fn(RVec3) -> bool;
        let checks: [(Check, Real); 2] = [
            (is_in_frame, MAX_POSITION),
            (is_frame_displacement, 2.0 * MAX_POSITION),
        ];
        for (check, bound) in checks {
            for v in real_on_axes(bound) {
                assert!(check(v), "{v:?}");
            }
            for v in real_on_axes(bound.next_up()) {
                assert!(!check(v), "{v:?}");
            }
            for value in NON_FINITE {
                for v in real_on_axes(Real::from(value)) {
                    assert!(!check(v), "{v:?}");
                }
            }
        }
    }

    #[test]
    fn vector_checks_accept_their_bound_and_reject_beyond() {
        type Check = fn(Vec3) -> bool;
        // `Real` is `f32` without the `double-precision` feature, so the cast is a no-op there.
        #[allow(clippy::unnecessary_cast)]
        let span = (2.0 * MAX_POSITION) as f32;
        let checks: [(Check, f32); 5] = [
            (is_frame_span, span),
            (is_local_offset, MAX_SHAPE_EXTENT),
            (is_linear_velocity, MAX_LINEAR_VELOCITY),
            (is_angular_velocity, MAX_ANGULAR_VELOCITY),
            (is_acceleration, MAX_ACCELERATION),
        ];
        for (check, bound) in checks {
            for v in on_axes(bound) {
                assert!(check(v), "{v:?}");
            }
            for v in on_axes(bound.next_up()) {
                assert!(!check(v), "{v:?}");
            }
            for value in NON_FINITE {
                for v in on_axes(value) {
                    assert!(!check(v), "{v:?}");
                }
            }
        }
    }

    #[test]
    fn scalar_checks_accept_their_range_and_reject_beyond() {
        type Check = fn(f32) -> bool;
        let checks: [(Check, &[f32], &[f32]); 3] = [
            (
                is_local_distance,
                &[0.0, MAX_SHAPE_EXTENT],
                &[-f32::MIN_POSITIVE, MAX_SHAPE_EXTENT.next_up()],
            ),
            (
                is_gravity_factor,
                &[-MAX_GRAVITY_FACTOR, 0.0, MAX_GRAVITY_FACTOR],
                &[
                    (-MAX_GRAVITY_FACTOR).next_down(),
                    MAX_GRAVITY_FACTOR.next_up(),
                ],
            ),
            (
                is_mass,
                &[MIN_MASS, MAX_MASS],
                &[0.0, MIN_MASS.next_down(), MAX_MASS.next_up()],
            ),
        ];
        for (check, accepted, rejected) in checks {
            for &value in accepted {
                assert!(check(value), "{value}");
            }
            for &value in rejected.iter().chain(&NON_FINITE) {
                assert!(!check(value), "{value}");
            }
        }
    }

    /// The vector along `direction` (components 1 or 0) whose Jolt length is the largest at
    /// most `bound`, found by stepping one component.
    fn on_the_jolt_bound(direction: Vec3, bound: f32) -> Vec3 {
        let mut v = direction.scale(bound / jolt_length(direction));
        while jolt_length(v) > bound {
            v.x = v.x.next_down();
        }
        while jolt_length(Vec3::new(v.x.next_up(), v.y, v.z)) <= bound {
            v.x = v.x.next_up();
        }
        v
    }

    #[test]
    fn velocity_checks_use_jolts_length_on_diagonals() {
        for (check, bound) in [
            (is_linear_velocity as fn(Vec3) -> bool, MAX_LINEAR_VELOCITY),
            (is_angular_velocity, MAX_ANGULAR_VELOCITY),
        ] {
            let v = on_the_jolt_bound(Vec3::new(1.0, 1.0, 1.0), bound);
            assert!(check(v), "{v:?}");
            assert!(!check(Vec3::new(v.x.next_up(), v.y, v.z)), "{v:?}");
        }
    }

    #[test]
    fn accelerations_are_bounded_by_the_speed_clamp_per_step() {
        let per_step = MAX_ACCELERATION * PhysicsWorld::MIN_DELTA_TIME;
        assert!((per_step - MAX_LINEAR_VELOCITY).abs() <= MAX_LINEAR_VELOCITY * 1.0e-6);
        let per_step = MAX_ANGULAR_ACCELERATION * PhysicsWorld::MIN_DELTA_TIME;
        assert!((per_step - MAX_ANGULAR_VELOCITY).abs() <= MAX_ANGULAR_VELOCITY * 1.0e-6);
    }
}
