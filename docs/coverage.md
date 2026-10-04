# Coverage

Which tests check the magnitude rules of `oxijolt::limits`, and what they leave open.
[limits.md](limits.md) derives the bounds themselves. Test names refer to the integration tests
in `crates/oxijolt/tests` and the unit tests in `crates/oxijolt/src`. Line numbers refer to the
vendored Jolt 5.6 sources (`crates/oxijolt-sys/vendor/JoltPhysics/Jolt/Physics/`).

## Inputs and their boundary tests

Every public setter and constructor that takes a magnitude, the rule it applies, and the tests
that check it at its boundary.

| Input | Rule | Boundary test |
|---|---|---|
| `WorldSettings::gravity`, `PhysicsWorld::set_gravity` | `MAX_ACCELERATION` | `world_gravity_is_bounded_by_max_acceleration` |
| `WorldSettings::max_contact_constraints` | `1..=WorldSettings::MAX_CONTACT_CONSTRAINTS` | `contact_constraint_capacity_is_bounded` |
| `WorldSettings::max_bodies`, `worker_threads` | Jolt's and oxijolt's counts | `invalid_settings_are_rejected`, `worker_thread_bounds_are_validated` |
| `WorldSettings::job_system` (`JobSystem::max_concurrency`) | `1..=WorldSettings::MAX_CONCURRENCY`, read once in `PhysicsWorld::new` | `max_concurrency_is_bounded` |
| `WorldSettings::max_body_pairs`, `temp_allocator_size` | at least 1; Jolt asserts nothing on their size, and joltc's temp allocator falls back to `malloc` | `invalid_settings_are_rejected` |
| `PhysicsWorld::step` delta time | `MIN_DELTA_TIME..=MAX_DELTA_TIME` | `step_rejects_delta_time_above_the_bound`, `step_rejects_delta_time_below_the_bound` |
| `PhysicsWorld::rebase` translation | `2 *` `MAX_POSITION` per axis; results finite | `rebase_translation_is_bounded_by_twice_the_frame` |
| `BodySettings::position` | `MAX_POSITION` | `body_settings_are_bounded` |
| `BodySettings::linear_velocity`, `angular_velocity` | `MAX_LINEAR_VELOCITY`, `MAX_ANGULAR_VELOCITY` | `body_settings_are_bounded`, `creation_velocities_agree_with_jolts_length_in_many_directions` |
| `BodySettings::restitution` | `0..=1` | `body_settings_are_bounded` |
| `BodySettings::gravity_factor` | `MAX_GRAVITY_FACTOR` | `body_settings_are_bounded` |
| `BodySettings::mass`, `PhysicsWorld::create_body` computed mass | `MIN_MASS..=MAX_MASS` for dynamic bodies | `body_settings_are_bounded`, `computed_dynamic_mass_is_bounded_and_kinematic_mass_is_not` |
| `PhysicsWorld::create_body` computed inertia of a dynamic or kinematic body (also a character's inner body and a part of `RagdollSettings::new`, `new_stabilized`) | an exactly diagonal tensor with invertible moments or near zero; otherwise smallest principal moment bounded from below at least `MIN_INERTIA_RATIO` (4.8e-4) of its Frobenius norm (see [limits.md](limits.md#rigid-body-inertia)) | `rigid_body_inertia_floor_holds_at_its_boundary`, `rigid_body_inertia_at_its_bound_decomposes`, `inner_body_inertia_at_its_bound_decomposes`, `stabilized_ragdoll_inertia_at_its_bound_decomposes` |
| `BodySettings::friction` | `0..=MAX_FRICTION` | `body_settings_are_bounded`, `friction_at_the_bound_keeps_contacts_finite` |
| `BodySettings::linear_damping`, `angular_damping` | finite, at least 0: Jolt scales by `max(0, 1 - c·dt)` (`MotionProperties.inl:144-145`) | `invalid_damping_is_rejected` |
| `BodySettings::rotation`, `BodyMut::set_rotation` | finite unit quaternion | `invalid_body_settings_are_rejected` |
| `BodyMut::set_position`, `set_position_and_rotation` | `MAX_POSITION` | `body_setters_are_bounded_and_rejection_changes_nothing` |
| `BodyMut::set_linear_velocity`, `set_angular_velocity` | `MAX_LINEAR_VELOCITY`, `MAX_ANGULAR_VELOCITY` | `body_setters_are_bounded_and_rejection_changes_nothing` |
| `BodyMut::add_force` | accumulated `|F| / m <=` `MAX_ACCELERATION` | `forces_are_bounded_by_the_acceleration_they_give` |
| `BodyMut::add_torque`, `add_force_at_point` | accumulated torque within `MAX_ANGULAR_ACCELERATION`; point within `MAX_POSITION`; `f32` torque products | `torques_are_bounded_by_the_angular_acceleration_they_give`, `point_torque_rejects_overflowing_products_even_when_they_cancel` |
| `BodyMut::reset_forces` | none | `reset_forces_ignores_static_and_kinematic_bodies` |
| `CharacterSettings::mass` | `0..=MAX_MASS` | `character_settings_and_setters_are_bounded` |
| `CharacterSettings::shape_offset` | `MAX_SHAPE_EXTENT` per axis | `character_settings_and_setters_are_bounded` |
| `CharacterSettings::predictive_contact_distance`, `character_padding`, `collision_tolerance` | `0..=MAX_SHAPE_EXTENT` (tolerance positive) | `character_settings_and_setters_are_bounded` |
| `CharacterSettings::max_strength` and the other settings | existing ranges; strength needs no bound (see [limits.md](limits.md#character-weight-and-push)) | `invalid_settings_and_poses_are_rejected_without_side_effects` |
| `PhysicsWorld::create_character` position, `CharacterMut::set_position` | `MAX_POSITION` | `character_settings_and_setters_are_bounded` |
| `CharacterMut::set_linear_velocity` | `MAX_LINEAR_VELOCITY`; Jolt does not clamp a character | `character_settings_and_setters_are_bounded` |
| `CharacterMut::set_up`, `set_rotation` | unit vector, unit quaternion | `invalid_settings_and_poses_are_rejected_without_side_effects` |
| `PhysicsWorld::update_character` gravity | `MAX_ACCELERATION` | `character_update_gravity_and_steps_are_bounded` |
| `PhysicsWorld::update_character` weight impulse | character mass times gravity times delta time at most `MAX_WEIGHT_IMPULSE` | `character_weight_impulse_at_a_lever_arm_is_bounded`, `weight_impulse_check_accepts_its_bound_and_rejects_beyond` |
| `ExtendedUpdateSettings` steps and forward distances | `MAX_SHAPE_EXTENT` | `character_update_gravity_and_steps_are_bounded` |
| `CharacterMut::restore_state` | only states from `save_state` exist | `a_restored_state_saves_the_same_bytes` |
| `PhysicsWorld::restore_state` | only states from `save_state`/`save_state_of` of the same world at the same epoch exist | `tests/state.rs` |
| `VehicleMut::set_gravity` | `MAX_ACCELERATION` | `gravity_is_bounded_by_max_acceleration` |
| `WheelSettings::new` position, `suspension_force_point` | `MAX_SHAPE_EXTENT` per axis | `wheel_magnitudes_are_bounded_by_the_policy` |
| `WheelSettings` suspension min, max and preload lengths, radius, width | `0..=MAX_SHAPE_EXTENT` (radius positive, max length at least min length) | `wheel_magnitudes_are_bounded_by_the_policy` |
| `WheelSettings::suspension_spring` | Jolt's stiffness and damping at most `MAX_SPRING_COEFFICIENT` for a chassis of `MAX_MASS` | `suspension_springs_are_bounded_by_the_coefficient`, `vehicle_springs_and_anti_roll_bar_at_their_bounds_step_finitely` |
| `VehicleAntiRollBar::stiffness` | `0..=VehicleAntiRollBar::MAX_STIFFNESS` | `anti_roll_bars_are_validated`, `vehicle_springs_and_anti_roll_bar_at_their_bounds_step_finitely` |
| `WheelSettings` inertia, angular damping, brake torques; engine, transmission and differential settings | finite, in their ranges, and every step coefficient they form finite at both time-step extremes; not bounded one by one | `wheel_values_are_validated`, `step_coefficients_of_wheels_must_be_finite`, `step_coefficients_of_the_drivetrain_must_be_finite`, `engine_values_are_validated`, `transmission_values_are_validated`, `differential_values_are_validated` |
| `WheelSettings` friction curves | finite points with increasing slip; the friction values are not bounded | `wheel_values_are_validated` |
| `VehicleSettings` up, forward, max pitch roll angle; collision testers | unit vectors and angle ranges; tester radius below every wheel's reach | `vehicle_values_are_validated`, `collision_testers_are_validated` |
| `VehicleMut::set_driver_input`, `set_max_pitch_roll_angle`, `set_collision_tester` | existing ranges | `driver_input_drives_steers_and_brakes`, `invalid_vehicles_create_nothing` |
| `SpringSettings::StiffnessAndDamping` | `MAX_SPRING_COEFFICIENT` | `stiffness_springs_are_bounded_by_the_coefficient` |
| `SpringSettings::FrequencyAndDamping` in `RagdollSettings::new`, `new_stabilized` | `B·ω²` and `2·B·ζ·ω` at most `MAX_SPRING_COEFFICIENT` | `motor_springs_are_bounded_by_the_parts_effective_mass`, `motor_spring_of_1e20_hz_is_rejected` |
| `MotorSettings::force_limits`, `torque_limits`; angle limits | finite, `min <= max`; Jolt clamps the motor impulse to `dt · limit` | `motors_and_springs_are_validated`, `swing_twist_limits_are_validated`, `hinge_limits_are_validated`, `six_dof_limits_are_validated` |
| constraint frame points | `MAX_POSITION` | `constraint_frame_points_are_bounded`, `constraint_targets_are_bounded` |
| the points where `PhysicsWorld::create_constraint` holds a dynamic body (frame points, automatic points, every point of a path); no setter moves them, and a rebase moves them with their bodies | lever-arm ratio at most `MAX_LEVER_ARM_RATIO` | `lever_arms_are_bounded_by_the_bodies_size`, `far_and_light_constraint_points_are_refused`, `constraints_at_the_lever_arm_bound_step_finitely` |
| `SpringSettings::FrequencyAndDamping` in `PhysicsWorld::create_constraint`, `ConstraintMut::<DistanceConstraint>::set_limits_spring`, `ConstraintMut::<HingeConstraint>::set_motor_settings`, `set_limits_spring` | `B·ω²` and `2·B·ζ·ω` at most `MAX_SPRING_COEFFICIENT`, `B` from the constraint's two bodies | `world_constraint_springs_are_bounded_by_the_bodies_effective_mass` |
| `DistanceRange::Range`, `ConstraintMut::<DistanceConstraint>::set_distance` | `0 <= min <= max <=` `MAX_SHAPE_EXTENT` | `constraint_targets_are_bounded` |
| `ConstraintMut::<HingeConstraint>::set_target_angle` | `[-π, π]`; Jolt clamps it to the limits | `constraint_targets_are_bounded` |
| `ConstraintMut::<HingeConstraint>::set_target_angular_velocity` | `MAX_ANGULAR_VELOCITY` | `constraint_targets_are_bounded` |
| `ConstraintMut::<HingeConstraint>::set_limits` | Jolt's hinge ranges, `min == max` only with a soft spring | `hinge_setters_check_their_values` |
| `ConstraintMut::<HingeConstraint>::set_max_friction_torque` | finite, at least 0; Jolt clamps the friction impulse to `dt · limit` | `constraint_friction_at_f32_max_steps_finitely` |
| `SliderConstraintSettings::limits`, `ConstraintMut::<SliderConstraint>::set_limits` | `min` in `[-MAX_SHAPE_EXTENT, 0]`, `max` in `[0, MAX_SHAPE_EXTENT]`, `min == max` only with a soft spring | `slider_cone_swing_twist_and_six_dof_targets_are_bounded` |
| `ConstraintMut::<SliderConstraint>::set_target_position` | `MAX_SHAPE_EXTENT`; Jolt clamps it to the limits | `slider_cone_swing_twist_and_six_dof_targets_are_bounded` |
| `ConstraintMut::<SliderConstraint>::set_target_velocity` | `MAX_LINEAR_VELOCITY` | `slider_cone_swing_twist_and_six_dof_targets_are_bounded` |
| `SliderConstraintSettings::max_friction_force`, `ConstraintMut::<SliderConstraint>::set_max_friction_force`, `ConstraintMut::<SwingTwistConstraint>::set_max_friction_torque` | finite, at least 0 | `slider_and_swing_twist_friction_at_f32_max_steps_finitely` |
| `SpringSettings::FrequencyAndDamping` in the slider, swing-twist and six-DOF motor and limit spring setters of `ConstraintMut` | as for `create_constraint` | `slider_swing_twist_and_six_dof_motor_springs_are_bounded` |
| `ConeConstraintSettings::new` half angle, `ConstraintMut::<ConeConstraint>::set_half_cone_angle` | `[0, π]`, as Jolt asserts | `slider_cone_swing_twist_and_six_dof_targets_are_bounded` |
| `ConstraintMut::<SwingTwistConstraint>::set_target_angular_velocity_cs`, `ConstraintMut::<SixDofConstraint>::set_target_angular_velocity_cs` | `MAX_ANGULAR_VELOCITY` | `slider_cone_swing_twist_and_six_dof_targets_are_bounded` |
| `ConstraintMut::<SwingTwistConstraint>::set_target_orientation_cs`, `ConstraintMut::<SixDofConstraint>::set_target_orientation_cs` | finite unit quaternion; Jolt clamps it to the limits | `slider_cone_swing_twist_and_six_dof_targets_are_bounded` |
| `ConstraintMut::<SixDofConstraint>::set_target_velocity_cs` | `MAX_LINEAR_VELOCITY` | `slider_cone_swing_twist_and_six_dof_targets_are_bounded` |
| `ConstraintMut::<SixDofConstraint>::set_target_position_cs` | `MAX_SHAPE_EXTENT` per axis | `slider_cone_swing_twist_and_six_dof_targets_are_bounded` |
| `GearConstraintSettings::new`, `teeth` ratio | `1..=MAX_GEAR_RATIO` (see `GearConstraintSettings` and `MAX_GEAR_RATIO` for both bounds) | `coupling_ratios_are_bounded`, `ratios_at_the_bound_step_finitely`, behaviour at the bound in `tests/constraints.rs` |
| `RackAndPinionConstraintSettings::new`, `teeth` ratio | magnitude within `1 / MAX_RATIO..=MAX_RATIO`; `teeth` length positive within `MAX_SHAPE_EXTENT` | `coupling_ratios_are_bounded`, `ratios_at_the_bound_step_finitely` |
| `PulleyConstraintSettings::ratio` | positive, within `1 / MAX_RATIO..=MAX_RATIO` | `pulley_ratio_and_lengths_are_bounded`, `pulleys_at_the_ratio_bound_step_finitely` |
| `PulleyLength::Range`, `ConstraintMut::<PulleyConstraint>::set_length` | `0 <= min <= max <= (1 + ratio) ·` `MAX_SHAPE_EXTENT` | `pulley_ratio_and_lengths_are_bounded` |
| `PulleyConstraintSettings::new` body and fixed points | `MAX_POSITION`; a rebase moves fixed points as re-expressed state, checked finite only | `pulley_ratio_and_lengths_are_bounded` |
| `HermitePath::new` points | 2 to `HermitePath::MAX_POINTS`; positions and tangents within `MAX_SHAPE_EXTENT` per axis; unit normal | `invalid_paths_are_rejected`, `path_inputs_are_bounded` |
| `HermitePath::new` segments | chord at least 1 mm; derivative along the chord at least twice its bound along the normal and above an `f32` margin, so Jolt's normal stays unit | `invalid_paths_are_rejected`, `path_validation_accepts_its_boundary` |
| `PathConstraintSettings::path_position`, `path_rotation`, `path_fraction` | `MAX_SHAPE_EXTENT` per axis; unit quaternion; `[0, max_fraction]` | `path_inputs_are_bounded` |
| `PathConstraintSettings::max_friction_force`, `ConstraintMut::<PathConstraint>::set_max_friction_force` | finite, at least 0 | `path_motor_springs_and_friction_are_bounded` |
| `PathConstraintSettings::position_motor`, `ConstraintMut::<PathConstraint>::set_position_motor_settings` springs | as for `create_constraint` | `path_motor_springs_and_friction_are_bounded` |
| `ConstraintMut::<PathConstraint>::set_target_velocity`, `set_target_path_fraction` | `MAX_LINEAR_VELOCITY`; `[0, max_fraction]` | `path_inputs_are_bounded` |
| `ConstraintRef::<PathConstraint>::closest_fraction` | point within `MAX_SHAPE_EXTENT` per axis, finite hint | `path_inputs_are_bounded` |
| `SwingTwistConstraintSettings::max_friction_torque`, `HingeConstraintSettings::max_friction_torque`, `SixDofConstraintSettings::max_friction` | finite, at least 0; Jolt clamps the friction impulse to `dt · limit` and applies no more than stops the relative motion | `swing_twist_limits_are_validated`, `hinge_limits_are_validated`, `six_dof_limits_are_validated` |
| `SixDofAxis::Limited` on a translation axis | finite, `min < max`, within `MAX_SHAPE_EXTENT` | `six_dof_limits_are_validated`, `six_dof_translation_limits_at_the_bound_step_finitely` |
| `RagdollSettings::new`, `new_stabilized` part masses | `MIN_MASS..=MAX_MASS`, also for kinematic parts (`RagdollMut::set_motion_type` can make them dynamic) | `part_masses_and_velocities_are_bounded` |
| `RagdollMut::set_pose`, `drive_to_pose_using_motors` | root offset and positions within `MAX_POSITION` | `poses_are_validated` |
| `RagdollMut::drive_to_pose_using_kinematics` | pose as above; every part's velocity, as Jolt computes it, within `MAX_LINEAR_VELOCITY` and `MAX_ANGULAR_VELOCITY`, checked for all parts before any changes | `poses_are_validated`, `kinematic_drive_is_bounded_by_the_velocities_it_implies` |
| `RagdollMut::set_linear_and_angular_velocity` | `MAX_LINEAR_VELOCITY`, `MAX_ANGULAR_VELOCITY` | `part_masses_and_velocities_are_bounded` |
| `Shape::new_box*`, `new_sphere`, `new_cylinder*`, `new_capsule` | dimensions within `MAX_SHAPE_EXTENT` | `primitive_extents_are_bounded` |
| `Shape::new_compound`, `new_offset_center_of_mass` | positions and offset within `MAX_SHAPE_EXTENT`; local bounds within it | `decorated_and_compound_extents_are_bounded` |
| `Shape::new_height_field` | local bounds within `MAX_SHAPE_EXTENT` | `height_field_extent_is_bounded` |
| `HeightFieldSettings`, `CompoundChild::rotation` | existing ranges | `invalid_height_fields_are_rejected`, `empty_or_invalid_compounds_are_rejected` |
| `Shape::new_box_with_material`, `new_sphere_with_material`, `new_capsule_with_material`, `new_cylinder_with_material` | the plain sibling's rules | `convex_constructors_apply_the_plain_rules` |
| `Shape::new_height_field_with_materials` | the rules of `new_height_field`; material count `1..=256`, `(n - 1)^2` indices, each below the count | `height_field_material_lists_are_validated` |
| `PhysicsWorld::cast_ray` origin | `MAX_POSITION` | `query_inputs_are_bounded_by_the_frame` |
| `RayCast` direction | finite, not zero | `invalid_rays_are_rejected`, `a_ray_with_a_huge_finite_direction_is_cast` |
| `ShapeCast`, `CollideShape` position | `MAX_POSITION` | `query_inputs_are_bounded_by_the_frame` |
| `ShapeCast` direction | `2 *` `MAX_POSITION` per axis | `query_inputs_are_bounded_by_the_frame` |
| `ShapeCast::target_distance` | `ShapeCast::MAX_TARGET_DISTANCE` | `target_distance_at_the_bound_gives_a_finite_depth` |
| `CollideShape::max_separation_distance` | `0..=MAX_SHAPE_EXTENT` | `query_inputs_are_bounded_by_the_frame` |
| `SoftBodySharedSettingsBuilder::build` vertices | at least one; position within `MAX_SHAPE_EXTENT` per axis; velocity within `MAX_LINEAR_VELOCITY`; inverse mass 0 or the inverse of a mass within `MIN_MASS..=MAX_MASS` | `soft_body_shared_settings_are_bounded`, `invalid_vertices_are_rejected` |
| `SoftBodySharedSettingsBuilder::build` total mass | masses of the movable vertices (`1 / w` in `f32`, as Jolt) add up to at most `MAX_MASS` | `soft_body_total_mass_is_bounded`, `total_movable_mass_is_bounded` |
| `SoftBodySharedSettingsBuilder::build` faces | indices name vertices, three different ones; every edge at least `MIN_SOFT_BODY_EDGE_LENGTH` in `f32`; area above 0; with distance bends the vertices opposite a shared edge as far apart | `soft_body_shared_settings_are_bounded`, `invalid_faces_are_rejected`, `edge_lengths_are_measured_in_f32_like_jolt`, `distance_bends_need_separate_opposite_vertices` |
| `SoftBodyVertexAttributes` compliances | `0..=MAX_COMPLIANCE` | `soft_body_shared_settings_are_bounded`, `attributes_are_validated` |
| `SoftBodyVertexAttributes::long_range_attachment` multiplier | `1..=MAX_RATIO` (see [limits.md](limits.md#soft-body-long-range-attachments)) | `soft_body_shared_settings_are_bounded` |
| `SoftBodySharedSettingsBuilder::edge`, `dihedral_bend`, `volume` | indices name different vertices; compliance `0..=MAX_COMPLIANCE`; edge and shared bend edge at least `MIN_SOFT_BODY_EDGE_LENGTH`; tetrahedron six-volume finite and not 0 in `f32` | `soft_body_explicit_constraints_are_bounded`, `invalid_explicit_constraints_are_rejected` |
| `SoftBodySettings` position, rotation, object layer, friction, restitution, gravity factor | as for `BodySettings` | `soft_body_settings_are_bounded` |
| `SoftBodySettings::num_iterations` | `1..=SoftBodySettings::MAX_ITERATIONS`; Jolt divides the step by it | `soft_body_settings_are_bounded` |
| `SoftBodySettings::linear_damping`, `max_linear_velocity`, `vertex_radius` | finite, at least 0; `(0, MAX_LINEAR_VELOCITY]`; `0..=MAX_SHAPE_EXTENT` | `soft_body_settings_are_bounded` |
| `SoftBodySettings::pressure` | `0..=MAX_SOFT_BODY_PRESSURE`; above 0 only when the faces enclose a volume large enough for it (see [limits.md](limits.md#soft-body-pressure)) | `soft_body_settings_are_bounded`, `pressure_at_the_bound_steps_finitely`, `pressure_needs_a_volume_for_its_faces`, `a_sliver_at_the_pressure_bound_steps_finitely` |
| `PhysicsWorld::create_soft_body` vertices (positions about the origin, masses) | without a kinematic vertex: smallest principal moment of the inertia at least `MIN_INERTIA_RATIO` (4.8e-4) of its Frobenius norm after Jolt's `f32` error, or an exactly diagonal tensor that is near zero or has every moment above 1e-30 (see [limits.md](limits.md#soft-body-inertia)) | `soft_body_inertia_is_checked_at_creation`, `soft_body_inertia_at_its_bound_decomposes` |
| `SoftBodyMut::set_vertex_velocity` | `MAX_LINEAR_VELOCITY` | `soft_body_vertex_writes_are_bounded_and_rejection_changes_nothing` |
| `SoftBodyMut::set_vertex_inverse_mass` | 0 or the inverse of a mass within `MIN_MASS..=MAX_MASS`; total movable mass at most `MAX_MASS`; the force accumulated this step within the soft body force bound for the new inverse masses; the inertia rule of `create_soft_body` at the current vertex positions | `soft_body_vertex_writes_are_bounded_and_rejection_changes_nothing`, `unpinning_cannot_release_an_accumulated_force`, `unpinning_checks_the_inertia_at_the_current_positions` |
| `SoftBodyMut::move_kinematic_vertex` | target within `MAX_POSITION`; a time step `step` accepts; the implied velocity within `MAX_LINEAR_VELOCITY` | `soft_body_vertex_writes_are_bounded_and_rejection_changes_nothing` |
| `BodyMut::add_force` on a soft body | accumulated `|F| · w_max / N <=` `MAX_ACCELERATION`, `N` the vertex count (Jolt's divisor), and `|F| <= MAX_ACCELERATION · MAX_MASS` | `soft_body_forces_are_bounded_by_the_acceleration_of_a_vertex`, `unpinning_cannot_release_an_accumulated_force` |
| `ContactSettings::set_combined_friction` (in a `ContactListener`) | `0..=MAX_FRICTION` | `contact_settings_setters_accept_their_range_and_refuse_beyond` |
| `ContactSettings::set_combined_restitution` | `0..=1` | `contact_settings_setters_accept_their_range_and_refuse_beyond` |
| `ContactSettings::set_inv_mass_scale1`, `set_inv_mass_scale2`, `set_inv_inertia_scale1`, `set_inv_inertia_scale2`; the `SoftBodyContactSettings` scales | 0 or `MIN_CONTACT_SCALE..=1` (see [limits.md](limits.md#contact-settings)) | `contact_settings_setters_accept_their_range_and_refuse_beyond`, `soft_body_contact_settings_setters_keep_scales_in_range`, `contact_scales_at_their_floor_step_finitely` |
| `ContactSettings::set_is_sensor` | stays `true` for a contact with a sensor body, as Jolt asserts | `a_contact_with_a_sensor_body_stays_a_sensor_contact` |
| `ContactSettings` returned by a `ContactListener` | every rule above, checked again against the contact the listener was called for; a failing value is not applied and is reported in `WorldEvents::rejected_contact_settings` | `settings_are_checked_again_against_the_contact_that_takes_them`, `settings_moved_to_a_contact_they_do_not_fit_are_rejected` |
| `ContactSettings::set_relative_linear_surface_velocity`, `set_relative_angular_surface_velocity` | `MAX_LINEAR_VELOCITY`, `MAX_ANGULAR_VELOCITY`, and `len(v) + len(ω) · R <=` `MAX_LINEAR_VELOCITY` (see [limits.md](limits.md#contact-settings)) | `surface_velocities_are_bounded_alone_and_together`, `a_conveyor_moves_a_resting_cube` |
| `DebugLineSettings` (feature `debug-renderer`) | centre within `MAX_POSITION`, radius at most twice it | `center_and_radius_are_bounded_by_the_frame` |

## Covered by tests only

The asserts leg of CI runs every test with Jolt's assertions. These paths are exercised there by
scenes with inputs at their bounds; no derivation backs them:
- contact and constraint impulses the solver generates: spheres of `MIN_MASS` and
  `MAX_MASS` colliding head-on at the velocity bounds, restitution 1, friction at
  `MAX_FRICTION` including a speculative contact with zero normal impulse, motors at the
  spring bound, six-DOF translation limits at the extent bound between parts of both mass
  extremes, suspension springs and anti-roll bars at their bounds
  (`bodies_at_every_bound_step_finitely`, `friction_at_the_bound_keeps_contacts_finite`,
  `six_dof_translation_limits_at_the_bound_step_finitely`,
  `vehicle_springs_and_anti_roll_bar_at_their_bounds_step_finitely`,
  `motor_springs_at_the_coefficient_bound_drive_finitely`);
- impulses at a lever arm on the lightest bodies: the weight impulse of a character of
  `MAX_MASS` at `MAX_WEIGHT_IMPULSE` on the edge of a 6 cm cube of `MIN_MASS`, the
  cube that turns fastest (`character_weight_impulse_at_a_lever_arm_is_bounded`), and a box
  of `MIN_MASS` pressed on its floor and pushed at the velocity bound by such a character
  (`character_at_its_bounds_pushes_the_lightest_body`);
- query arithmetic: rays, shape casts and collisions from frame corners
  (`query_inputs_are_bounded_by_the_frame`) and a sphere at the extent bound cast across
  the frame and collided with separations up to the extent bound
  (`queries_with_shapes_at_the_extent_bound_stay_finite`);
- world constraints: gears at both ends of `1..=MAX_GEAR_RATIO`, racks and pinions and
  pulleys at both `MAX_RATIO` bounds
  between bodies of both mass extremes, one of them turning or sliding at the velocity bound
  (`ratios_at_the_bound_step_finitely`, `pulleys_at_the_ratio_bound_step_finitely`); motor and
  limit springs accepted at the effective-mass bound and rejected one representable frequency
  above it (`world_constraint_springs_are_bounded_by_the_bodies_effective_mass`,
  `slider_swing_twist_and_six_dof_motor_springs_are_bounded`,
  `path_motor_springs_and_friction_are_bounded`); targets at their bounds
  (`constraint_targets_are_bounded`, `slider_cone_swing_twist_and_six_dof_targets_are_bounded`,
  `path_inputs_are_bounded`); hinge, slider, swing-twist and path friction of `f32::MAX` on
  bodies moving at the velocity bounds (`constraint_friction_at_f32_max_steps_finitely`,
  `slider_and_swing_twist_friction_at_f32_max_steps_finitely`,
  `path_motor_springs_and_friction_are_bounded`); and a path segment at the margin of the
  segment check stepped end to end with every rotation constraint
  (`path_validation_accepts_its_boundary`); points that hold a dynamic body at
  `MAX_LEVER_ARM_RATIO`: a ball joint and a hinge on a static body, and a weld, a six-DOF
  joint with every axis fixed and a swing-twist joint with zero ranges on a static body or
  between two bodies, each holding a 1 g, 6 cm cube or a 1 kg, 1 m cube at the velocity
  bounds under gravity (`constraints_at_the_lever_arm_bound_step_finitely`).

## Not covered

- State the simulation produces itself is not an input and is not checked again: a body Jolt
  carries out of the frame, the positions a rebase computes (only checked to be finite), or a
  character state restored with `CharacterMut::restore_state`,
  which can only come from `CharacterRef::save_state`.
- Bodies with a principal inverse inertia above `√3 · 1e6`, such as a light needle-thin shape
  or a centre of mass far from the shape. `PhysicsWorld::create_body` bounds no inverse
  inertia from above: an exactly diagonal tensor (an unrotated shape, or a centre of mass
  moved along a principal axis) only needs finite inverse moments, and the rigid body
  inertia floor (see [limits.md](limits.md#rigid-body-inertia)) bounds how badly conditioned any
  other tensor is, not how small it is. The boundary tests of that floor step slender
  bodies for two ticks without contacts or impulses, so impulses at a lever arm on such a
  body, including a character's weight impulse, are neither derived nor tested. The same holds
  for the suspension effective mass Jolt forms from a wheel's force point and the chassis's
  inverse inertia (`VehicleConstraint.cpp:448-451`).
- Constraints whose solver diverges for reasons the lever-arm ratio does not measure. In the
  `asserts` build Jolt then asserts that a squared velocity is finite
  (`MotionProperties.inl:28` or `:38`); in the release build the joint tears apart by metres
  and the state can become NaN: Jolt's velocity clamp scales a velocity whose squared length
  overflows to zero, turns an already infinite component into NaN (`inf * 0`) and lets NaN
  through. No impact is needed. Measured with one constraint to a static
  body under gravity unless stated otherwise:
  - a hinge whose pin lies off the least-inertia axis of a slender body, outside its cross
    section (an offset `d` of about twice the half thickness `t` or more). A 2 kg rod of 1 m
    by 2 cm hinged 5 cm off its axis (a lever-arm ratio of 43.5) asserted while swinging
    about the hinge at 10 rad/s; a 10 kg barrier arm of 3 m by 5 cm on a 5 cm bracket
    (ratio 12) asserted at 30 rad/s. In the release build both became NaN within a few
    hundred steps of such a swing (the step depends on the scene); a 1 m rod 1 cm thick with the pin 5 cm beside it
    along the hinge line asserted while falling from horizontal under gravity alone. On 1 m
    rods swung at up to 47 rad/s, `t` = 5 mm failed from `d` = 1 cm, `t` = 1 cm from 2 cm,
    `t` = 2 cm only at 10 cm, and `t` = 5 cm not up to 10 cm;
  - chains of hinges with non-parallel axes, also between cubes: of 100 seeded chains of 3 to
    10 cubes, 3 asserted at kicks of 15 m/s and 1.4 rad/s per body at lever-arm ratios of 10
    to 100, and 18 at ratios of 100 to 500; in the release build 2 of 100 chains became NaN
    at 15 m/s and 31 of 100 at 50 m/s;
  - a light body held by two to four constraints to static anchors, also without a hinge
    (point and fixed, cone, distance and six-DOF, ...): of 300 seeded scenes at 0.3 to 1 of
    the lever-arm bound, 24 asserted at the velocity bounds with every constraint kind and 8
    without hinges, 3 at a tenth of the bounds and none at 3 %.

  The same probes found no assert and no NaN for a 2 m by 1 m by 5 cm door hinged at its
  edge, capsule limbs on hinges, a rod hinged on its axis, or the 12-capsule test ragdoll, at
  kicks up to 499 m/s and 47 rad/s, but their joints still opened by more than 0.5 m: of 24
  kicks at 150 m/s, 8 for the door, 10 for the rod and 9 for a forearm or shin capsule, and
  at 499 m/s 16, 24 and 19, up to 3.3 m for the door, 4.1 m for the rod and 7.3 m for the
  capsules; one of 24 exploded ragdolls opened by 1 m at 150 m/s. A six-DOF joint with the
  hinge's free axis did not assert on the slender bodies but let the joint drift apart by
  0.2 to 3.3 m while swinging and up to 6.9 m under kicks of 50 m/s, so it is no
  workaround. No
  bound in `oxijolt::limits` excludes these cases.
- A slider adds the distance travelled along its axis to the lever of body 1
  (`SliderConstraint.cpp`), which `MAX_LEVER_ARM_RATIO` checks only at creation; with a
  dynamic body 1 and a long travel the lever grows beyond the bound.
- Inputs that are only checked to be finite or ordered, as the table above says: ray
  directions, damping, motor force and torque limits, ragdoll joint friction (world
  constraint friction is probed as above), wheel friction curves
  and the wheel and drivetrain values that only have to give finite step coefficients.
- A soft body with pressure whose volume shrinks after creation (crushed, or its vertices
  moved or recentred by Jolt so that an open mesh encloses less): Jolt divides the pressure
  by the volume of every sub-step (`SoftBodyMotionProperties.cpp:300-307`), and the
  creation check holds only for the start geometry.
- Soft body constraint stability: `MAX_COMPLIANCE` keeps Jolt's compliance terms finite,
  not the solver convergent. Measured: a 2 m cube of 1 g vertices, three of them kinematic,
  with six tetrahedral volume constraints of compliances 0 to 1e20, per-vertex attributes
  with edge and shear compliances up to 1e20 and LRA multipliers up to 1e4, distance bends,
  100 iterations, no damping, restitution 1, a vertex speed limit of 1 m/s, one vertex given
  155 m/s and an accepted force of 3.7e6 N, had NaN vertices after its first step in the
  release build and asserted that a squared velocity is finite (`MotionProperties.inl:28`)
  in the asserts build. With a tenth of the force, 5 iterations, no volume constraints or
  uniform attributes it stayed finite. No bound in `oxijolt::limits` excludes it. Two more
  seeded scenes of the same kind failed in the asserts build: a cube of 1 g vertices with
  six volume constraints, 37 iterations, a step of 0.1 s, friction 1000, a vertex radius of
  2000 m, a force of 3.7e6 N and a vertex unpinned to 1 kg asserted at
  `MotionProperties.inl:28`; a cube with pressure 1e5 under gravity of about 1000 m/s²,
  pushed by repeated forces of 3.8e9 N and resting against rigid bodies (without them it
  did not fail), grew bounds beyond `cLargeFloat` (`QuadTree.cpp:68`).
- `RagdollSettings::new_stabilized` reports Jolt's `Stabilize` failing to decompose an
  inertia tensor as an error, but Jolt asserts on that path first (`Ragdoll.cpp:158`).
- The assertion `errors == EPhysicsUpdateError::None` at the end of every step that drops
  contacts (`PhysicsSystem.cpp:679`) is intentional: `PhysicsWorld::step` returns the same
  errors in its `StepReport`, and oxijolt's assertion handler lets the
  process continue for this assertion only.
