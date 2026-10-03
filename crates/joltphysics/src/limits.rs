//! The magnitudes the safe API accepts, and why.
//!
//! Jolt checks most magnitudes only with debug assertions (the `asserts` feature) and otherwise
//! computes with whatever it is given, so a finite but huge input can overflow Jolt's `f32`
//! arithmetic inside a step. joltphysics bounds the caller-given magnitudes the [audit](#audit)
//! table lists with the constants of this module; the same table names the inputs that are only
//! checked to be finite or ordered, and why. This module states which Jolt assertion paths the
//! bounds are derived for, which are only covered by tests, and which are not covered at all. It
//! does not claim that no accepted input can reach a Jolt assertion.
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
//!   `mass · |g| · dt` at the ground contact point (`CharacterVirtual.cpp:1474-1481`), so the
//!   impulse also turns the ground body. [`PhysicsWorld::update_character`] accepts at most
//!   [`MAX_WEIGHT_IMPULSE`], 1e9 N·s, which changes the linear velocity of a body of the
//!   smallest mass by at most 1e12 m/s and the angular velocity of a body whose principal
//!   inverse inertia is at most `√3 · 1e6` by at most about 6e18 rad/s; both squares are
//!   finite (see [`MAX_WEIGHT_IMPULSE`] for the derivation). Its push impulse is capped at
//!   `delta_velocity / inv_effective_mass` (`CharacterVirtual.cpp:795-811`), whose effective
//!   mass includes the body's rotation, so the velocity change at the contact is at most the
//!   relative normal speed whatever the strength.
//! - **Springs.** Jolt derives a stiffness `k` and damping `c` from every spring
//!   (`SpringPart.h:36-55,91-104`). Both stay at most [`MAX_SPRING_COEFFICIENT`]: in stiffness mode
//!   directly, in frequency mode through an upper bound of the effective mass. For a ragdoll joint
//!   that bound is computed at creation from the parts' masses and inertias, including Jolt's
//!   `Stabilize` (`Ragdoll.cpp:135-185`); see [`SpringSettings`](crate::SpringSettings). For a
//!   world constraint it is computed at creation from its two bodies: over the dynamic ones, the
//!   larger of the mass and the largest principal moment of inertia
//!   ([`PhysicsWorld::create_constraint`]); the constraint's spring setters use the same bound.
//!   For a wheel's suspension it is [`MAX_MASS`], since Jolt's suspension effective mass is at most the
//!   chassis mass (`VehicleConstraint.cpp:448-451`).
//! - **Anti-roll bars.** Jolt computes `stiffness · length difference · dt` for each bar
//!   (`VehicleConstraint.cpp:289-293`) and passes it as the bias `b` of the wheel's suspension
//!   constraint (`VehicleConstraint.cpp:508`), whose impulse is `-K⁻¹ (J v + b)`
//!   (`AxisConstraintPart.h:300-301`): a velocity term, scaled by an effective mass that is at
//!   most each body's own along the axis. With
//!   [`VehicleAntiRollBar::MAX_STIFFNESS`](crate::VehicleAntiRollBar::MAX_STIFFNESS) and wheel
//!   lengths at most [`MAX_SHAPE_EXTENT`], `b` is at most 5e14 m/s, so the velocity change along
//!   the suspension axis stays finite and squares finitely.
//! - **Restitution.** At most 1, so the restitution target speed is at most the approach speed.
//! - **Friction.** At most [`MAX_FRICTION`], so Jolt's combined friction `sqrt(f1 · f2)`
//!   (`ContactConstraintManager.h:554`) is finite and the friction impulse bound, combined friction
//!   times the normal impulse (`ContactConstraintManager.cpp:1714-1715`), is never `0 · ∞` = NaN.
//!   A cube sliding on a floor, both at `f32::MAX`, had NaN velocities within three 60 Hz steps.
//! - **Kinematic drive.** Jolt's `MoveKinematic` sets a velocity of `move / dt` without clamping
//!   it (`MotionProperties.inl:9-21`). [`RagdollMut::drive_to_pose_using_kinematics`](crate::RagdollMut::drive_to_pose_using_kinematics)
//!   computes every part's velocity with Jolt's own operations first and accepts the drive only
//!   when each stays within [`MAX_LINEAR_VELOCITY`] and [`MAX_ANGULAR_VELOCITY`], the bounds of
//!   every other velocity input.
//! - **Six-DOF translation limits.** Jolt corrects a violated limit by the distance beyond it
//!   times the effective mass (`SixDOFConstraint.cpp:380-410,780-790`); limits within
//!   [`MAX_SHAPE_EXTENT`] keep that finite. A limit of 1e30 m moved two parts to NaN positions in
//!   a few steps.
//! - **Contact constraint capacity.** [`WorldSettings::MAX_CONTACT_CONSTRAINTS`] stays below the
//!   count above which `ContactConstraintManager::Init` asserts; a native compile-time check pins
//!   it.
//!
//! # Covered by tests only
//! The asserts leg of CI runs every test with Jolt's assertions; these paths are exercised there
//! by scenes with inputs at their bounds, not derived:
//! - contact and constraint impulses the solver generates: spheres of [`MIN_MASS`] and
//!   [`MAX_MASS`] colliding head-on at the velocity bounds, restitution 1, friction at
//!   [`MAX_FRICTION`] including a speculative contact with zero normal impulse, motors at the
//!   spring bound, six-DOF translation limits at the extent bound between parts of both mass
//!   extremes, suspension springs and anti-roll bars at their bounds
//!   (`bodies_at_every_bound_step_finitely`, `friction_at_the_bound_keeps_contacts_finite`,
//!   `six_dof_translation_limits_at_the_bound_step_finitely`,
//!   `vehicle_springs_and_anti_roll_bar_at_their_bounds_step_finitely`,
//!   `motor_springs_at_the_coefficient_bound_drive_finitely`);
//! - impulses at a lever arm on the lightest bodies: the weight impulse of a character of
//!   [`MAX_MASS`] at [`MAX_WEIGHT_IMPULSE`] on the edge of a 6 cm cube of [`MIN_MASS`], the
//!   cube that turns fastest (`character_weight_impulse_at_a_lever_arm_is_bounded`), and a box
//!   of [`MIN_MASS`] pressed on its floor and pushed at the velocity bound by such a character
//!   (`character_at_its_bounds_pushes_the_lightest_body`);
//! - query arithmetic: rays, shape casts and collisions from frame corners
//!   (`query_inputs_are_bounded_by_the_frame`) and a sphere at the extent bound cast across
//!   the frame and collided with separations up to the extent bound
//!   (`queries_with_shapes_at_the_extent_bound_stay_finite`);
//! - world constraints: gears at both ends of `1..=`[`MAX_GEAR_RATIO`], racks and pinions and
//!   pulleys at both [`MAX_RATIO`] bounds
//!   between bodies of both mass extremes, one of them turning or sliding at the velocity bound
//!   (`ratios_at_the_bound_step_finitely`, `pulleys_at_the_ratio_bound_step_finitely`); motor and
//!   limit springs accepted at the effective-mass bound and rejected one representable frequency
//!   above it (`world_constraint_springs_are_bounded_by_the_bodies_effective_mass`,
//!   `slider_swing_twist_and_six_dof_motor_springs_are_bounded`,
//!   `path_motor_springs_and_friction_are_bounded`); targets at their bounds
//!   (`constraint_targets_are_bounded`, `slider_cone_swing_twist_and_six_dof_targets_are_bounded`,
//!   `path_inputs_are_bounded`); hinge, slider, swing-twist and path friction of `f32::MAX` on
//!   bodies moving at the velocity bounds (`constraint_friction_at_f32_max_steps_finitely`,
//!   `slider_and_swing_twist_friction_at_f32_max_steps_finitely`,
//!   `path_motor_springs_and_friction_are_bounded`); and a path segment at the margin of the
//!   segment check stepped end to end with every rotation constraint
//!   (`path_validation_accepts_its_boundary`); points that hold a dynamic body at
//!   [`MAX_LEVER_ARM_RATIO`]: a ball joint and a hinge on a static body, and a weld, a six-DOF
//!   joint with every axis fixed and a swing-twist joint with zero ranges on a static body or
//!   between two bodies, each holding a 1 g, 6 cm cube or a 1 kg, 1 m cube at the velocity
//!   bounds under gravity (`constraints_at_the_lever_arm_bound_step_finitely`).
//!
//! # Not covered
//! - State the simulation produces itself is not an input and is not checked again: a body Jolt
//!   carries out of the frame, the positions a rebase computes (only checked to be finite), or a
//!   character state restored with [`CharacterMut::restore_state`](crate::CharacterMut::restore_state),
//!   which can only come from [`CharacterRef::save_state`](crate::CharacterRef::save_state).
//! - Bodies with a principal inverse inertia above `√3 · 1e6`, such as a needle-thin shape or a
//!   centre of mass far from the shape: [`PhysicsWorld::create_body`] only requires the inverse
//!   inertia to be finite, and no test steps such a body, so impulses at a lever arm on it,
//!   including a character's weight impulse, are neither derived nor tested. The same holds
//!   for the suspension effective mass Jolt forms from a wheel's force point and the chassis's
//!   inverse inertia (`VehicleConstraint.cpp:448-451`).
//! - Constraints whose solver diverges for reasons the lever-arm ratio does not measure. In the
//!   `asserts` build Jolt then asserts that a squared velocity is finite
//!   (`MotionProperties.inl:28` or `:38`); in the release build the joint tears apart by metres
//!   and the state can become NaN: Jolt's velocity clamp scales a velocity whose squared length
//!   overflows to zero, turns an already infinite component into NaN (`inf * 0`) and lets NaN
//!   through. No impact is needed. Measured with one constraint to a static
//!   body under gravity unless stated otherwise:
//!   - a hinge whose pin lies off the least-inertia axis of a slender body, outside its cross
//!     section (an offset `d` of about twice the half thickness `t` or more). A 2 kg rod of 1 m
//!     by 2 cm hinged 5 cm off its axis (a lever-arm ratio of 43.5) asserted while swinging
//!     about the hinge at 10 rad/s; a 10 kg barrier arm of 3 m by 5 cm on a 5 cm bracket
//!     (ratio 12) asserted at 30 rad/s. In the release build both became NaN within a few
//!     hundred steps of such a swing (the step depends on the scene); a 1 m rod 1 cm thick with the pin 5 cm beside it
//!     along the hinge line asserted while falling from horizontal under gravity alone. On 1 m
//!     rods swung at up to 47 rad/s, `t` = 5 mm failed from `d` = 1 cm, `t` = 1 cm from 2 cm,
//!     `t` = 2 cm only at 10 cm, and `t` = 5 cm not up to 10 cm;
//!   - chains of hinges with non-parallel axes, also between cubes: of 100 seeded chains of 3 to
//!     10 cubes, 3 asserted at kicks of 15 m/s and 1.4 rad/s per body at lever-arm ratios of 10
//!     to 100, and 18 at ratios of 100 to 500; in the release build 2 of 100 chains became NaN
//!     at 15 m/s and 31 of 100 at 50 m/s;
//!   - a light body held by two to four constraints to static anchors, also without a hinge
//!     (point and fixed, cone, distance and six-DOF, ...): of 300 seeded scenes at 0.3 to 1 of
//!     the lever-arm bound, 24 asserted at the velocity bounds with every constraint kind and 8
//!     without hinges, 3 at a tenth of the bounds and none at 3 %.
//!
//!   The same probes found no assert and no NaN for a 2 m by 1 m by 5 cm door hinged at its
//!   edge, capsule limbs on hinges, a rod hinged on its axis, or the 12-capsule test ragdoll, at
//!   kicks up to 499 m/s and 47 rad/s, but their joints still opened by more than 0.5 m: of 24
//!   kicks at 150 m/s, 8 for the door, 10 for the rod and 9 for a forearm or shin capsule, and
//!   at 499 m/s 16, 24 and 19, up to 3.3 m for the door, 4.1 m for the rod and 7.3 m for the
//!   capsules; one of 24 exploded ragdolls opened by 1 m at 150 m/s. A six-DOF joint with the
//!   hinge's free axis did not assert on the slender bodies but let the joint drift apart by
//!   0.2 to 3.3 m while swinging and up to 6.9 m under kicks of 50 m/s, so it is no
//!   workaround. No
//!   input bound in this module excludes these cases.
//! - A slider adds the distance travelled along its axis to the lever of body 1
//!   (`SliderConstraint.cpp`), which [`MAX_LEVER_ARM_RATIO`] checks only at creation; with a
//!   dynamic body 1 and a long travel the lever grows beyond the bound.
//! - Inputs that are only checked to be finite or ordered, as the audit table says: ray
//!   directions, damping, motor force and torque limits, ragdoll joint friction (world
//!   constraint friction is probed as above), wheel friction curves
//!   and the wheel and drivetrain values that only have to give finite step coefficients.
//! - A closed soft body with pressure crushed to a tiny positive volume: Jolt divides the
//!   pressure by the enclosed volume (`SoftBodyMotionProperties.cpp:300-307`).
//! - Soft body constraint stability: [`MAX_COMPLIANCE`] keeps Jolt's compliance terms finite,
//!   not the solver convergent.
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
//! | `WorldSettings::job_system` (`JobSystem::max_concurrency`) | `1..=`[`WorldSettings::MAX_CONCURRENCY`](crate::WorldSettings::MAX_CONCURRENCY), read once in `PhysicsWorld::new` | new: `max_concurrency_is_bounded` |
//! | `WorldSettings::max_body_pairs`, `temp_allocator_size` | at least 1; Jolt asserts nothing on their size, and joltc's temp allocator falls back to `malloc` | existing: `invalid_settings_are_rejected` |
//! | `PhysicsWorld::step` delta time | `MIN_DELTA_TIME..=MAX_DELTA_TIME` | existing: `step_rejects_delta_time_above_the_bound`, `step_rejects_delta_time_below_the_bound` |
//! | `PhysicsWorld::rebase` translation | `2 *` [`MAX_POSITION`] per axis; results finite | new: `rebase_translation_is_bounded_by_twice_the_frame` |
//! | `BodySettings::position` | [`MAX_POSITION`] | new: `body_settings_are_bounded` |
//! | `BodySettings::linear_velocity`, `angular_velocity` | [`MAX_LINEAR_VELOCITY`], [`MAX_ANGULAR_VELOCITY`] | new: `body_settings_are_bounded`, `creation_velocities_agree_with_jolts_length_in_many_directions` |
//! | `BodySettings::restitution` | `0..=1` | new: `body_settings_are_bounded` |
//! | `BodySettings::gravity_factor` | [`MAX_GRAVITY_FACTOR`] | new: `body_settings_are_bounded` |
//! | `BodySettings::mass`, `PhysicsWorld::create_body` computed mass | [`MIN_MASS`]`..=`[`MAX_MASS`] for dynamic bodies | new: `body_settings_are_bounded`, `computed_dynamic_mass_is_bounded_and_kinematic_mass_is_not` |
//! | `BodySettings::friction` | `0..=`[`MAX_FRICTION`] | new: `body_settings_are_bounded`, `friction_at_the_bound_keeps_contacts_finite` |
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
//! | `PhysicsWorld::update_character` weight impulse | character mass times gravity times delta time at most [`MAX_WEIGHT_IMPULSE`] | new: `character_weight_impulse_at_a_lever_arm_is_bounded`, `weight_impulse_check_accepts_its_bound_and_rejects_beyond` |
//! | `ExtendedUpdateSettings` steps and forward distances | [`MAX_SHAPE_EXTENT`] | new: `character_update_gravity_and_steps_are_bounded` |
//! | `CharacterMut::restore_state` | only states from `save_state` exist | existing: `a_restored_state_saves_the_same_bytes` |
//! | `PhysicsWorld::restore_state` | only states from `save_state`/`save_state_of` of the same world at the same epoch exist | new: `tests/state.rs` |
//! | `VehicleMut::set_gravity` | [`MAX_ACCELERATION`] | new: `gravity_is_bounded_by_max_acceleration` |
//! | `WheelSettings::new` position, `suspension_force_point` | [`MAX_SHAPE_EXTENT`] per axis | new: `wheel_magnitudes_are_bounded_by_the_policy` |
//! | `WheelSettings` suspension min, max and preload lengths, radius, width | `0..=`[`MAX_SHAPE_EXTENT`] (radius positive, max length at least min length) | new: `wheel_magnitudes_are_bounded_by_the_policy` |
//! | `WheelSettings::suspension_spring` | Jolt's stiffness and damping at most [`MAX_SPRING_COEFFICIENT`] for a chassis of [`MAX_MASS`] | new: `suspension_springs_are_bounded_by_the_coefficient`, `vehicle_springs_and_anti_roll_bar_at_their_bounds_step_finitely` |
//! | `VehicleAntiRollBar::stiffness` | `0..=VehicleAntiRollBar::MAX_STIFFNESS` | new: `anti_roll_bars_are_validated`, `vehicle_springs_and_anti_roll_bar_at_their_bounds_step_finitely` |
//! | `WheelSettings` inertia, angular damping, brake torques; engine, transmission and differential settings | finite, in their ranges, and every step coefficient they form finite at both time-step extremes; not bounded one by one | existing: `wheel_values_are_validated`, `step_coefficients_of_wheels_must_be_finite`, `step_coefficients_of_the_drivetrain_must_be_finite`, `engine_values_are_validated`, `transmission_values_are_validated`, `differential_values_are_validated` |
//! | `WheelSettings` friction curves | finite points with increasing slip; the friction values are not bounded | existing: `wheel_values_are_validated` |
//! | `VehicleSettings` up, forward, max pitch roll angle; collision testers | unit vectors and angle ranges; tester radius below every wheel's reach | existing: `vehicle_values_are_validated`, `collision_testers_are_validated` |
//! | `VehicleMut::set_driver_input`, `set_max_pitch_roll_angle`, `set_collision_tester` | existing ranges | existing: `driver_input_drives_steers_and_brakes`, `invalid_vehicles_create_nothing` |
//! | `SpringSettings::StiffnessAndDamping` | [`MAX_SPRING_COEFFICIENT`] | new: `stiffness_springs_are_bounded_by_the_coefficient` |
//! | `SpringSettings::FrequencyAndDamping` in `RagdollSettings::new`, `new_stabilized` | `B·ω²` and `2·B·ζ·ω` at most [`MAX_SPRING_COEFFICIENT`] | new: `motor_springs_are_bounded_by_the_parts_effective_mass`, `motor_spring_of_1e20_hz_is_rejected` |
//! | `MotorSettings::force_limits`, `torque_limits`; angle limits | finite, `min <= max`; Jolt clamps the motor impulse to `dt · limit` | existing: `motors_and_springs_are_validated`, `swing_twist_limits_are_validated`, `hinge_limits_are_validated`, `six_dof_limits_are_validated` |
//! | constraint frame points | [`MAX_POSITION`] | new: `constraint_frame_points_are_bounded`, `constraint_targets_are_bounded` |
//! | the points where `PhysicsWorld::create_constraint` holds a dynamic body (frame points, automatic points, every point of a path); no setter moves them, and a rebase moves them with their bodies | lever-arm ratio at most [`MAX_LEVER_ARM_RATIO`] | new: `lever_arms_are_bounded_by_the_bodies_size`, `far_and_light_constraint_points_are_refused`, `constraints_at_the_lever_arm_bound_step_finitely` |
//! | `SpringSettings::FrequencyAndDamping` in `PhysicsWorld::create_constraint`, `ConstraintMut::<DistanceConstraint>::set_limits_spring`, `ConstraintMut::<HingeConstraint>::set_motor_settings`, `set_limits_spring` | `B·ω²` and `2·B·ζ·ω` at most [`MAX_SPRING_COEFFICIENT`], `B` from the constraint's two bodies | new: `world_constraint_springs_are_bounded_by_the_bodies_effective_mass` |
//! | `DistanceRange::Range`, `ConstraintMut::<DistanceConstraint>::set_distance` | `0 <= min <= max <=` [`MAX_SHAPE_EXTENT`] | new: `constraint_targets_are_bounded` |
//! | `ConstraintMut::<HingeConstraint>::set_target_angle` | `[-π, π]`; Jolt clamps it to the limits | new: `constraint_targets_are_bounded` |
//! | `ConstraintMut::<HingeConstraint>::set_target_angular_velocity` | [`MAX_ANGULAR_VELOCITY`] | new: `constraint_targets_are_bounded` |
//! | `ConstraintMut::<HingeConstraint>::set_limits` | Jolt's hinge ranges, `min == max` only with a soft spring | new: `hinge_setters_check_their_values` |
//! | `ConstraintMut::<HingeConstraint>::set_max_friction_torque` | finite, at least 0; Jolt clamps the friction impulse to `dt · limit` | new: `constraint_friction_at_f32_max_steps_finitely` |
//! | `SliderConstraintSettings::limits`, `ConstraintMut::<SliderConstraint>::set_limits` | `min` in `[-`[`MAX_SHAPE_EXTENT`]`, 0]`, `max` in `[0, `[`MAX_SHAPE_EXTENT`]`]`, `min == max` only with a soft spring | new: `slider_cone_swing_twist_and_six_dof_targets_are_bounded` |
//! | `ConstraintMut::<SliderConstraint>::set_target_position` | [`MAX_SHAPE_EXTENT`]; Jolt clamps it to the limits | new: `slider_cone_swing_twist_and_six_dof_targets_are_bounded` |
//! | `ConstraintMut::<SliderConstraint>::set_target_velocity` | [`MAX_LINEAR_VELOCITY`] | new: `slider_cone_swing_twist_and_six_dof_targets_are_bounded` |
//! | `SliderConstraintSettings::max_friction_force`, `ConstraintMut::<SliderConstraint>::set_max_friction_force`, `ConstraintMut::<SwingTwistConstraint>::set_max_friction_torque` | finite, at least 0 | new: `slider_and_swing_twist_friction_at_f32_max_steps_finitely` |
//! | `SpringSettings::FrequencyAndDamping` in the slider, swing-twist and six-DOF motor and limit spring setters of `ConstraintMut` | as for `create_constraint` | new: `slider_swing_twist_and_six_dof_motor_springs_are_bounded` |
//! | `ConeConstraintSettings::new` half angle, `ConstraintMut::<ConeConstraint>::set_half_cone_angle` | `[0, π]`, as Jolt asserts | new: `slider_cone_swing_twist_and_six_dof_targets_are_bounded` |
//! | `ConstraintMut::<SwingTwistConstraint>::set_target_angular_velocity_cs`, `ConstraintMut::<SixDofConstraint>::set_target_angular_velocity_cs` | [`MAX_ANGULAR_VELOCITY`] | new: `slider_cone_swing_twist_and_six_dof_targets_are_bounded` |
//! | `ConstraintMut::<SwingTwistConstraint>::set_target_orientation_cs`, `ConstraintMut::<SixDofConstraint>::set_target_orientation_cs` | finite unit quaternion; Jolt clamps it to the limits | new: `slider_cone_swing_twist_and_six_dof_targets_are_bounded` |
//! | `ConstraintMut::<SixDofConstraint>::set_target_velocity_cs` | [`MAX_LINEAR_VELOCITY`] | new: `slider_cone_swing_twist_and_six_dof_targets_are_bounded` |
//! | `ConstraintMut::<SixDofConstraint>::set_target_position_cs` | [`MAX_SHAPE_EXTENT`] per axis | new: `slider_cone_swing_twist_and_six_dof_targets_are_bounded` |
//! | `GearConstraintSettings::new`, `teeth` ratio | `1..=`[`MAX_GEAR_RATIO`] (see `GearConstraintSettings` and [`MAX_GEAR_RATIO`] for both bounds) | new: `coupling_ratios_are_bounded`, `ratios_at_the_bound_step_finitely`; behaviour at the bound in `tests/constraints.rs` |
//! | `RackAndPinionConstraintSettings::new`, `teeth` ratio | magnitude within `1 / `[`MAX_RATIO`]`..=`[`MAX_RATIO`]; `teeth` length positive within [`MAX_SHAPE_EXTENT`] | new: `coupling_ratios_are_bounded`, `ratios_at_the_bound_step_finitely` |
//! | `PulleyConstraintSettings::ratio` | positive, within `1 / `[`MAX_RATIO`]`..=`[`MAX_RATIO`] | new: `pulley_ratio_and_lengths_are_bounded`, `pulleys_at_the_ratio_bound_step_finitely` |
//! | `PulleyLength::Range`, `ConstraintMut::<PulleyConstraint>::set_length` | `0 <= min <= max <= (1 + ratio) ·` [`MAX_SHAPE_EXTENT`] | new: `pulley_ratio_and_lengths_are_bounded` |
//! | `PulleyConstraintSettings::new` body and fixed points | [`MAX_POSITION`]; a rebase moves fixed points as re-expressed state, checked finite only | new: `pulley_ratio_and_lengths_are_bounded` |
//! | `HermitePath::new` points | 2 to `HermitePath::MAX_POINTS`; positions and tangents within [`MAX_SHAPE_EXTENT`] per axis; unit normal | new: `invalid_paths_are_rejected`, `path_inputs_are_bounded` |
//! | `HermitePath::new` segments | chord at least 1 mm; derivative along the chord at least twice its bound along the normal and above an `f32` margin, so Jolt's normal stays unit | new: `invalid_paths_are_rejected`, `path_validation_accepts_its_boundary` |
//! | `PathConstraintSettings::path_position`, `path_rotation`, `path_fraction` | [`MAX_SHAPE_EXTENT`] per axis; unit quaternion; `[0, max_fraction]` | new: `path_inputs_are_bounded` |
//! | `PathConstraintSettings::max_friction_force`, `ConstraintMut::<PathConstraint>::set_max_friction_force` | finite, at least 0 | new: `path_motor_springs_and_friction_are_bounded` |
//! | `PathConstraintSettings::position_motor`, `ConstraintMut::<PathConstraint>::set_position_motor_settings` springs | as for `create_constraint` | new: `path_motor_springs_and_friction_are_bounded` |
//! | `ConstraintMut::<PathConstraint>::set_target_velocity`, `set_target_path_fraction` | [`MAX_LINEAR_VELOCITY`]; `[0, max_fraction]` | new: `path_inputs_are_bounded` |
//! | `ConstraintRef::<PathConstraint>::closest_fraction` | point within [`MAX_SHAPE_EXTENT`] per axis, finite hint | new: `path_inputs_are_bounded` |
//! | `SwingTwistConstraintSettings::max_friction_torque`, `HingeConstraintSettings::max_friction_torque`, `SixDofConstraintSettings::max_friction` | finite, at least 0; Jolt clamps the friction impulse to `dt · limit` and applies no more than stops the relative motion | existing: `swing_twist_limits_are_validated`, `hinge_limits_are_validated`, `six_dof_limits_are_validated` |
//! | `SixDofAxis::Limited` on a translation axis | finite, `min < max`, within [`MAX_SHAPE_EXTENT`] | new: `six_dof_limits_are_validated`, `six_dof_translation_limits_at_the_bound_step_finitely` |
//! | `RagdollSettings::new`, `new_stabilized` part masses | [`MIN_MASS`]`..=`[`MAX_MASS`], also for kinematic parts (`RagdollMut::set_motion_type` can make them dynamic) | new: `part_masses_and_velocities_are_bounded` |
//! | `RagdollMut::set_pose`, `drive_to_pose_using_motors` | root offset and positions within [`MAX_POSITION`] | new: `poses_are_validated` |
//! | `RagdollMut::drive_to_pose_using_kinematics` | pose as above; every part's velocity, as Jolt computes it, within [`MAX_LINEAR_VELOCITY`] and [`MAX_ANGULAR_VELOCITY`], checked for all parts before any changes | new: `poses_are_validated`, `kinematic_drive_is_bounded_by_the_velocities_it_implies` |
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
//! | `SoftBodySharedSettingsBuilder::build` vertices | at least one; position within [`MAX_SHAPE_EXTENT`] per axis; velocity within [`MAX_LINEAR_VELOCITY`]; inverse mass 0 or the inverse of a mass within [`MIN_MASS`]`..=`[`MAX_MASS`] | new: `soft_body_shared_settings_are_bounded`, `invalid_vertices_are_rejected` |
//! | `SoftBodySharedSettingsBuilder::build` total mass | masses of the movable vertices (`1 / w` in `f32`, as Jolt) add up to at most [`MAX_MASS`] | new: `soft_body_total_mass_is_bounded`, `total_movable_mass_is_bounded` |
//! | `SoftBodySharedSettingsBuilder::build` faces | indices name vertices, three different ones; every edge at least [`MIN_SOFT_BODY_EDGE_LENGTH`] in `f32`; area above 0; with distance bends the vertices opposite a shared edge as far apart | new: `soft_body_shared_settings_are_bounded`, `invalid_faces_are_rejected`, `edge_lengths_are_measured_in_f32_like_jolt`, `distance_bends_need_separate_opposite_vertices` |
//! | `SoftBodyVertexAttributes` compliances | `0..=`[`MAX_COMPLIANCE`] | new: `soft_body_shared_settings_are_bounded`, `attributes_are_validated` |
//! | `SoftBodyVertexAttributes::long_range_attachment` multiplier | `1..=`[`MAX_RATIO`] (see there for why Jolt's square of the distance stays finite) | new: `soft_body_shared_settings_are_bounded` |
//! | `SoftBodySharedSettingsBuilder::edge`, `dihedral_bend`, `volume` | indices name different vertices; compliance `0..=`[`MAX_COMPLIANCE`]; edge and shared bend edge at least [`MIN_SOFT_BODY_EDGE_LENGTH`]; tetrahedron six-volume finite and not 0 in `f32` | new: `soft_body_explicit_constraints_are_bounded`, `invalid_explicit_constraints_are_rejected` |
//! | `SoftBodySettings` position, rotation, object layer, friction, restitution, gravity factor | as for `BodySettings` | new: `soft_body_settings_are_bounded` |
//! | `SoftBodySettings::num_iterations` | `1..=SoftBodySettings::MAX_ITERATIONS`; Jolt divides the step by it | new: `soft_body_settings_are_bounded` |
//! | `SoftBodySettings::linear_damping`, `max_linear_velocity`, `vertex_radius` | finite, at least 0; `(0, `[`MAX_LINEAR_VELOCITY`]`]`; `0..=`[`MAX_SHAPE_EXTENT`] | new: `soft_body_settings_are_bounded` |
//! | `SoftBodySettings::pressure` | `0..=`[`MAX_SOFT_BODY_PRESSURE`] | new: `soft_body_settings_are_bounded`, `pressure_at_the_bound_steps_finitely` |
//! | `SoftBodyMut::set_vertex_velocity` | [`MAX_LINEAR_VELOCITY`] | new: `soft_body_vertex_writes_are_bounded_and_rejection_changes_nothing` |
//! | `SoftBodyMut::set_vertex_inverse_mass` | 0 or the inverse of a mass within [`MIN_MASS`]`..=`[`MAX_MASS`]; total movable mass at most [`MAX_MASS`] | new: `soft_body_vertex_writes_are_bounded_and_rejection_changes_nothing` |
//! | `SoftBodyMut::move_kinematic_vertex` | target within [`MAX_POSITION`]; a time step `step` accepts; the implied velocity within [`MAX_LINEAR_VELOCITY`] | new: `soft_body_vertex_writes_are_bounded_and_rejection_changes_nothing` |
//! | `BodyMut::add_force` on a soft body | accumulated `|F| · w_max / N <=` [`MAX_ACCELERATION`], `N` the vertex count (Jolt's divisor) | new: `soft_body_forces_are_bounded_by_the_acceleration_of_a_vertex` |
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

/// Largest friction coefficient of a body.
///
/// Crate policy. Jolt combines the friction of two bodies in contact as
/// `sqrt(friction1 * friction2)` (`ContactConstraintManager.h:554`) and multiplies the result by
/// the contact's normal impulse (`ContactConstraintManager.cpp:1714-1715`). A product that
/// overflows makes the combined friction infinite, and an infinite friction times a zero normal
/// impulse is NaN, which reaches the bodies' velocities; two bodies with friction `f32::MAX` do
/// that within a few steps. Any coefficient whose square is finite (below about 1.8e19) avoids
/// it; 1000 is far above the friction of real materials.
pub const MAX_FRICTION: f32 = 1000.0;

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

/// Largest weight impulse a character may press on what it stands on during one update, its
/// mass times the length of the update's gravity times the update's delta time, in N·s.
///
/// Crate policy. Jolt applies the weight impulse at the ground contact point
/// (`CharacterVirtual.cpp:1474-1481`), so it also turns the ground body. Jolt keeps a body's
/// principal moments of inertia only while their vector is longer than 1e-6 (`Vec3::IsNearZero`
/// in `MotionProperties.cpp:46-56`) and otherwise uses the inertia of a sphere of radius 1, an
/// inverse of `2.5 / mass`, at most 2500 for [`MIN_MASS`]. So a body whose principal moments are
/// equal has an inverse inertia of at most `√3 · 1e6`, and the ground contact lies within
/// `√3 ·` [`MAX_SHAPE_EXTENT`] of its centre of mass. For such a body and every body with a
/// smaller principal inverse inertia, the angular velocity change is at most
/// `√3e6 · 3464 · 1e9`, about 6e18 rad/s, whose square is finite; the linear velocity change is
/// at most `1e9 · 1e3` m/s. Without the bound, a character of [`MAX_MASS`] at
/// [`MAX_ACCELERATION`] with a one-second update (5e14 N·s) on the edge of a 6 cm cube of
/// [`MIN_MASS`] overflows the cube's squared angular speed, which Jolt asserts on
/// (`MotionProperties.inl:38`). The bound allows a character of [`MAX_MASS`] at 1000 m/s² with
/// one-second updates, far above the characters of a game.
pub const MAX_WEIGHT_IMPULSE: f32 = 1.0e9;

/// Largest magnitude of a rack-and-pinion or pulley ratio; the smallest is its inverse. Gears
/// have their own, tighter range (see [`MAX_GEAR_RATIO`]).
///
/// Crate policy. Jolt multiplies the inverse mass or inertia of body 2 by the ratio's square in
/// the effective mass, and body 2's velocity (for racks and pulleys also its impulse) by the
/// ratio (`GearConstraintPart.h:81,122`, `RackAndPinionConstraintPart.h:82,123`,
/// `IndependentAxisConstraintPart.h:71,112` for pulleys). With a principal inverse inertia of at
/// most `√3 · 1e6` (see [`MAX_WEIGHT_IMPULSE`]), `ratio² · I⁻¹` is at most about 1.7e14, far from
/// `f32` overflow. A ratio of 1e4 already turns a pinion ten thousand radians per metre of its
/// rack; tests step both bounds on the lightest and heaviest bodies.
pub const MAX_RATIO: f32 = 1.0e4;

/// Largest gear ratio; the smallest is 1 (see
/// [`GearConstraintSettings`](crate::GearConstraintSettings)).
///
/// Crate policy, measured. Jolt 5.6 applies a gear's impulse to body 2 without the ratio
/// (`GearConstraintPart::ApplyVelocityStep`), so each solver iteration keeps up to `1 − 1/ratio`
/// of the velocity error `ω1 + ratio · ω2`, the worst case being a body 1 much heavier than
/// body 2. With Jolt's 10 velocity iterations per step the gear then needs more steps to restore
/// the relation the larger the ratio. The bound is the largest round ratio that, after a
/// disturbance, brings the error back to within 2 % of its initial value within 10 steps for any
/// mass distribution: measured worst 1.6 % at ratio 10, 6.5 % at 20, 60 % at 100, and at 1e4
/// 91 % still after 60 steps. The first step after a disturbance leaves up to
/// `(1 − 1/ratio)^10`, 35 % at ratio 10. Tested at the bound by
/// `gear_keeps_its_velocity_relation_at_the_largest_ratio`.
pub const MAX_GEAR_RATIO: f32 = 10.0;

/// Largest lever-arm ratio a world constraint may give a dynamic body: how far the point where
/// the constraint holds the body lies from its centre of mass, measured against the body's own
/// size.
///
/// The lever-arm ratio of a body at a point `r` from its centre of mass is
/// `mass · trace([r]× I⁻¹ [r]×ᵀ)`, with `I⁻¹` the body's inverse inertia: summed over the body's
/// principal axes, the squared distance of the point from each axis divided by the squared
/// radius of gyration about it. A sphere or cube with radius of gyration `k` has
/// `2 · (|r| / k)²`, so the bound allows `|r|` up to about `22 · k`; a rod held at its end has a
/// ratio of 6 at any length.
/// [`PhysicsWorld::create_constraint`] checks every point a constraint holds a dynamic body by
/// (for a path, every point of the path; for an automatic point, the point Jolt picks
/// between the centres of mass, weighted by inverse mass towards the lighter body).
///
/// Crate policy, measured, not derived. Jolt solves each constraint part with its effective mass
/// `K = Σ (m⁻¹ · 1 + [r]× I⁻¹ [r]×ᵀ)` (`PointConstraintPart.h`, `AxisConstraintPart.h`) in `f32`.
/// A ratio of at most `B` per body bounds the lever terms by `B · m⁻¹`, so `K`'s condition number
/// stays below `1 + B`. Two failures were measured with a body at the velocity bounds, under
/// gravity, for 120 steps, in the `asserts` build:
/// - a body held far from its centre of mass: a 1 g, 6 cm cube on a hinge 116 m away (a ratio
///   of 4.5e7) went to NaN, two 1 kg, 1 m cubes joined by a point 3000 m away (1.1e8) moved
///   erratically and at 4000 m Jolt asserted that a squared velocity is finite
///   (`MotionProperties.inl:28`), and a cube on a static body took angular velocities rounded to
///   powers of two from a ratio of about 1e8;
/// - two light bodies held rigidly (a fixed constraint, a six-DOF constraint with every axis
///   fixed, a swing-twist constraint with zero ranges): their accumulated impulse grows step
///   after step until Jolt asserts that the squared angular velocity is finite
///   (`MotionProperties.inl:38`), from `|r| / k` of about 37 (a ratio of 2700) for 1 g and 1 kg
///   cubes of 6 cm and 20 cm, for the 6 cm cube also at a tenth of the velocity bounds; none of
///   24 seeded cases failed at `|r| / k` of 34 or below. Point, hinge, cone, slider and six-DOF
///   constraints with limited rotations did not fail at `|r| / k` of 650.
///
/// A derivation in the style of [`MAX_WEIGHT_IMPULSE`] (products of the largest accepted
/// inverse inertia, lever and impulse kept finite in `f32`) allows levers of hundreds of metres
/// and does not exclude the second failure, so the bound is the measured onset divided by 2.7.
/// It allows a door on a hinge at its edge, a weld at the surface of a part, and a pendulum bob
/// of radius `a` on a point or hinge constraint up to about `14 · a` from the pivot; a longer
/// pendulum is a [`DistanceConstraintSettings`](crate::DistanceConstraintSettings) whose points
/// lie on the bodies.
pub const MAX_LEVER_ARM_RATIO: f32 = 1000.0;

/// Shortest distance between two soft body vertices that a face edge or an explicit edge
/// joins, in metres: 1 mm, measured as Jolt measures a rest length (an `f32` difference and an
/// `f32` length).
///
/// Crate policy. Jolt only asserts that a rest length is above zero
/// (`SoftBodySharedSettings.cpp:226,377`) and divides by edge lengths while it solves; the
/// bound keeps a degenerate edge out of the solver with a margin.
pub const MIN_SOFT_BODY_EDGE_LENGTH: f32 = 1.0e-3;

/// Largest compliance (inverse stiffness) of a soft body constraint, in the units of the
/// constraint's own equation; 0 is rigid.
///
/// Crate policy, derived. Jolt divides each compliance by the squared sub-step
/// (`SoftBodyMotionProperties.cpp:371,445,496,577,594`). [`PhysicsWorld::step`] always runs one
/// collision step, so a sub-step is at least [`PhysicsWorld::MIN_DELTA_TIME`] divided by
/// [`SoftBodySettings::MAX_ITERATIONS`](crate::SoftBodySettings::MAX_ITERATIONS), 1e-8 s, and
/// `compliance / dt²` is at most `1e20 · 1e16 = 1e36`, below `f32::MAX`; Jolt's average of two
/// compliances, `0.5 · (c1 + c2)`, stays finite as well. This proves that the product is finite,
/// not that the solver is stable at every compliance.
pub const MAX_COMPLIANCE: f32 = 1.0e20;

/// Largest pressure coefficient of a soft body (`n · R · T` in Jolt's terms, N·m).
///
/// Crate policy, measured. Jolt applies `pressure · dt / (6 · volume)` times each face's area
/// as an impulse (`SoftBodyMotionProperties.cpp:290-312`). A closed ball of 1 m with vertex
/// masses at [`MIN_MASS`] and at the total-mass bound, at this pressure, stepped 600 times on a
/// floor in the `asserts` build, stays finite (`pressure_at_the_bound_steps_finitely`). Not
/// covered: a closed body crushed to a tiny positive volume, which Jolt divides by.
pub const MAX_SOFT_BODY_PRESSURE: f32 = 1.0e6;

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

/// Whether `friction` is finite and within `0..=MAX_FRICTION`.
pub(crate) fn is_friction(friction: f32) -> bool {
    (0.0..=MAX_FRICTION).contains(&friction)
}

/// Whether a character of `mass` updated with `gravity` for `delta_time` seconds presses with
/// a weight impulse of at most [`MAX_WEIGHT_IMPULSE`], computed in `f64` so that it cannot
/// overflow. The inputs are already checked to be finite and within their own bounds.
pub(crate) fn is_weight_impulse(mass: f32, gravity: Vec3, delta_time: f32) -> bool {
    f64::from(mass) * f64_length(gravity) * f64::from(delta_time) <= f64::from(MAX_WEIGHT_IMPULSE)
}

/// Whether `mass` is finite and within `MIN_MASS..=MAX_MASS`.
pub(crate) fn is_mass(mass: f32) -> bool {
    (MIN_MASS..=MAX_MASS).contains(&mass)
}

/// Whether `inverse_mass` is the inverse of a mass within `MIN_MASS..=MAX_MASS`, the range of
/// a movable soft body vertex.
pub(crate) fn is_vertex_inverse_mass(inverse_mass: f32) -> bool {
    (1.0 / MAX_MASS..=1.0 / MIN_MASS).contains(&inverse_mass)
}

/// Whether `compliance` is finite and within `0..=MAX_COMPLIANCE`.
pub(crate) fn is_compliance(compliance: f32) -> bool {
    (0.0..=MAX_COMPLIANCE).contains(&compliance)
}

/// Whether `ratio` is finite and its magnitude within `1 / MAX_RATIO..=MAX_RATIO`.
pub(crate) fn is_ratio(ratio: f32) -> bool {
    (1.0 / MAX_RATIO..=MAX_RATIO).contains(&ratio.abs())
}

/// The lever-arm ratio (see [`MAX_LEVER_ARM_RATIO`]) of a dynamic body with `inverse_mass` and
/// principal inverse inertia `inverse_inertia` at `lever`, given in the body's principal frame;
/// computed in `f64`.
pub(crate) fn lever_arm_ratio(inverse_mass: f32, inverse_inertia: Vec3, lever: Vec3) -> f64 {
    let d = [inverse_inertia.x, inverse_inertia.y, inverse_inertia.z].map(f64::from);
    let r = [lever.x, lever.y, lever.z].map(f64::from);
    let squared = r.iter().map(|c| c * c).sum::<f64>();
    let trace: f64 = (0..3).map(|k| d[k] * (squared - r[k] * r[k])).sum();
    trace / f64::from(inverse_mass)
}

/// The largest lever-arm ratio of the same body at any point within `distance` of its centre of
/// mass: the two largest principal inverse inertias times `distance²`, over the inverse mass.
pub(crate) fn lever_arm_ratio_within(
    inverse_mass: f32,
    inverse_inertia: Vec3,
    distance: f64,
) -> f64 {
    let mut d = [inverse_inertia.x, inverse_inertia.y, inverse_inertia.z].map(f64::from);
    d.sort_by(f64::total_cmp);
    (d[1] + d[2]) * distance * distance / f64::from(inverse_mass)
}

/// Whether `ratio` is at most [`MAX_LEVER_ARM_RATIO`]; false for NaN.
pub(crate) fn is_lever_arm_ratio(ratio: f64) -> bool {
    ratio <= f64::from(MAX_LEVER_ARM_RATIO)
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
        let checks: [(Check, &[f32], &[f32]); 6] = [
            (
                is_friction,
                &[0.0, MAX_FRICTION],
                &[-f32::MIN_POSITIVE, MAX_FRICTION.next_up()],
            ),
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
            (
                is_vertex_inverse_mass,
                &[1.0 / MAX_MASS, 1.0 / MIN_MASS],
                &[
                    0.0,
                    (1.0 / MAX_MASS).next_down(),
                    (1.0 / MIN_MASS).next_up(),
                ],
            ),
            (
                is_compliance,
                &[0.0, MAX_COMPLIANCE],
                &[-f32::MIN_POSITIVE, MAX_COMPLIANCE.next_up()],
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
    fn weight_impulse_keeps_the_angular_speed_of_the_ground_body_finite() {
        let largest_kept_inverse_inertia = 3.0_f64.sqrt() * 1.0e6;
        let largest_lever = 3.0_f64.sqrt() * f64::from(MAX_SHAPE_EXTENT);
        let angular_speed =
            largest_kept_inverse_inertia * largest_lever * f64::from(MAX_WEIGHT_IMPULSE)
                + f64::from(MAX_ANGULAR_VELOCITY);
        assert!(angular_speed * angular_speed < f64::from(f32::MAX) / 4.0);
        let sphere_inverse_inertia = 2.5 / f64::from(MIN_MASS);
        assert!(sphere_inverse_inertia < largest_kept_inverse_inertia);
    }

    #[test]
    fn weight_impulse_check_accepts_its_bound_and_rejects_beyond() {
        let gravity = MAX_WEIGHT_IMPULSE / MAX_MASS;
        for down in on_axes(gravity) {
            assert!(is_weight_impulse(MAX_MASS, down, 1.0), "{down:?}");
            assert!(is_weight_impulse(0.0, down, 1.0), "{down:?}");
        }
        for down in on_axes(gravity.next_up()) {
            assert!(!is_weight_impulse(MAX_MASS, down, 1.0), "{down:?}");
        }
        assert!(!is_weight_impulse(
            MAX_MASS,
            Vec3::new(0.0, -gravity, 0.0),
            1.0f32.next_up()
        ));
    }

    #[test]
    fn lever_arm_ratios_measure_the_lever_against_the_radius_of_gyration() {
        // A 2 kg cube of side 1 m: inertia 2 / 6 about every axis, radius of gyration² 1 / 6.
        let inverse_inertia = Vec3::new(3.0, 3.0, 3.0);
        let ratio = lever_arm_ratio(0.5, inverse_inertia, Vec3::new(0.0, 2.0, 0.0));
        assert!((ratio - 2.0 * 4.0 * 6.0).abs() < 1e-9, "{ratio}");
        // A thin rod of 1 kg and 2 m along y held at its end: 6 whatever its thickness.
        let rod = Vec3::new(3.0, 1.0e4, 3.0);
        let ratio = lever_arm_ratio(1.0, rod, Vec3::new(0.0, 1.0, 0.0));
        assert!((ratio - 6.0).abs() < 1e-9, "{ratio}");
        // Anywhere within a distance: the two largest inverse inertias.
        let within = lever_arm_ratio_within(1.0, rod, 1.0);
        assert!((within - (1.0e4 + 3.0)).abs() < 1e-6, "{within}");
        let bound = f64::from(MAX_LEVER_ARM_RATIO);
        assert!(is_lever_arm_ratio(bound));
        assert!(!is_lever_arm_ratio(bound.next_up()));
        assert!(!is_lever_arm_ratio(f64::NAN));
        assert!(!is_lever_arm_ratio(f64::INFINITY));
    }

    #[test]
    fn accelerations_are_bounded_by_the_speed_clamp_per_step() {
        let per_step = MAX_ACCELERATION * PhysicsWorld::MIN_DELTA_TIME;
        assert!((per_step - MAX_LINEAR_VELOCITY).abs() <= MAX_LINEAR_VELOCITY * 1.0e-6);
        let per_step = MAX_ANGULAR_ACCELERATION * PhysicsWorld::MIN_DELTA_TIME;
        assert!((per_step - MAX_ANGULAR_VELOCITY).abs() <= MAX_ANGULAR_VELOCITY * 1.0e-6);
    }
}
