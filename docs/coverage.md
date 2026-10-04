# Coverage

What the safe API binds and which tests check it; then which tests check the magnitude rules of
`oxijolt::limits`, and what they leave open. [limits.md](limits.md) derives the bounds themselves.
Test names refer to the integration tests in `crates/oxijolt/tests` and the unit tests in
`crates/oxijolt/src` (`src/...` below). Line numbers refer to the vendored Jolt 5.6 sources
(`crates/oxijolt-sys/vendor/JoltPhysics/Jolt/Physics/`).

## What is bound and tested

| Feature | Safe API | Tests |
|---|---|---|
| World with settings and validation | `PhysicsWorld::new`, `WorldSettings` | `world.rs`: `default_world_steps`, `invalid_settings_are_rejected`, `gravity_round_trips_including_zero` |
| Step with a report of dropped work, time step bounds | `PhysicsWorld::step`, `StepReport`, `StepError`, `MIN_DELTA_TIME`, `MAX_DELTA_TIME` | `world.rs`: `step_rejects_bad_delta_time`, `step_rejects_delta_time_below_the_bound`, `step_rejects_delta_time_above_the_bound`, `full_contact_constraint_buffer_is_reported_and_the_world_advances`, `full_body_pair_cache_is_reported_and_the_world_advances` |
| Collision layers | `CollisionLayers`, `ObjectLayer`, `BroadPhaseLayer` | `dynamics.rs`: `collision_layers_decide_which_bodies_touch`; `src/layers.rs`: `validate_rejects_inconsistent_tables` |
| Several worlds, threads | `PhysicsWorld: Send + Sync`, `WorldSettings::worker_threads` | `dynamics.rs`: `two_worlds_side_by_side_do_not_interfere`, `worlds_step_in_parallel_threads`; `world.rs`: `world_and_shape_are_send_and_sync`, `world_is_readable_from_many_threads` |
| Caller job system | `JobSystem`, `Job`, `WorldSettings::job_system`, `WorldSettings::MAX_CONCURRENCY` | `job_system.rs`: `rayon_job_system_matches_the_native_pool`, `inline_job_system_matches_the_native_pool`, `step_from_the_only_thread_of_a_rayon_pool_completes`, `jobs_may_outlive_their_world`, `a_deferred_job_may_finish_on_another_thread_while_the_world_drops`, `a_job_system_that_keeps_every_job_does_not_run_out_of_jolt_jobs`, `a_panicking_job_system_panics_out_of_step_and_the_world_stays_usable`, `max_concurrency_is_bounded`, `worlds_sharing_one_job_system_step_in_parallel`, `the_last_of_job_system_and_worker_threads_wins`, `a_sleeping_world_queues_no_jobs`; `src/job_system.rs`: `run_and_drop_each_run_the_job_once`, `jobs_run_inside_a_queue_callback_wait_for_the_end_of_the_update`, `a_job_released_by_its_update_does_nothing_when_run_later`, `concurrent_panics_keep_one_payload_and_drop_the_others`; `job_system_leaks.rs`: `caller_job_systems_do_not_leak`; the Rayon doctest of `JobSystem` |
| Static, kinematic and dynamic bodies, ids | `BodySettings`, `create_body`, `BodyId` | `bodies.rs`: `ids_follow_insertion_order`, `foreign_and_stale_ids_are_rejected`, `invalid_body_settings_are_rejected`, `full_world_rejects_another_body` |
| Pose and velocity, read and write | `BodyRef`, `BodyMut`, `Activation` | `bodies.rs`: `pose_and_velocity_read_back_as_exact_bits`, `activation_decides_whether_a_pose_write_wakes_the_body` |
| Forces, torque, reset | `BodyMut::add_force`, `add_force_at_point`, `add_torque`, `reset_forces` | `bodies.rs`: `forces`, `reset_forces_ignores_static_and_kinematic_bodies` |
| Sleeping and active state | `BodyRef::is_sleeping`, `is_active` | `bodies.rs`: `sleeping_flag_is_readable` |
| Removal wakes the bodies around it | `PhysicsWorld::remove_body` | `bodies.rs`: `removal_wakes_bodies_resting_on_it`, `removal_wakes_the_same_bodies_whether_or_not_the_broad_phase_was_optimized` |
| Mass override, continuous collision, gravity factor | `BodySettings::mass`, `motion_quality`, `gravity_factor` | `bodies.rs`: `forces`, `mass_too_small_to_invert_is_rejected`, `gravity_factor_scales_the_fall`; `dynamics.rs`: `linear_cast_body_does_not_tunnel_through_thin_static` |
| Damping, mass readout | `BodySettings::linear_damping`, `angular_damping`, `BodyRef::mass` | `src/body.rs`: `damping_reaches_the_body`, `invalid_damping_is_rejected`, `mass_is_reported_for_dynamic_bodies_only`; `ragdoll.rs`: `plain_settings_keep_the_masses_and_stabilized_ones_the_total` |
| Contacts of the last step | `PhysicsWorld::were_bodies_in_contact` | `ragdoll.rs`: `the_same_parts_as_plain_bodies_collide`, `two_ragdolls_collide_with_each_other_but_not_with_themselves`; `listeners.rs`: `added_and_persisted_pairs_were_in_contact` |
| Enhanced internal edge removal | `BodySettings::enhanced_internal_edge_removal` | `src/body.rs`: `enhanced_internal_edge_removal_reaches_the_body` (reads the flag back from the Jolt body); `dynamics.rs`: `enhanced_internal_edge_removal_smooths_sliding_over_a_compound` |
| Box, sphere, Y-cylinder, Y-capsule, convex radius | `Shape::new_box`, `new_box_with_convex_radius`, `new_sphere`, `new_cylinder`, `new_capsule` | `shapes.rs`: `sharp_box_edge_ray_hits_the_exact_corner`, `convex_radius_rounds_box_edges_for_contacts`; `src/shape.rs`: `cylinder_and_capsule_dimensions_reach_jolt`, `invalid_dimensions_are_rejected` |
| Heightfield (n = 33, holes, block size, active-edge threshold) | `Shape::new_height_field`, `HeightFieldSettings` | `shapes.rs`: `height_field_33_builds_and_matches_samples_at_nodes`, `height_field_rising_along_z_matches_analytic_surface`, `height_field_of_holes_has_no_collision`, `height_field_with_custom_active_edge_threshold_builds`, `active_edge_threshold_decides_the_normal_on_a_gentle_ridge`, `static_only_shapes_are_rejected_for_moving_bodies` |
| Offset centre of mass | `Shape::new_offset_center_of_mass` | `shapes.rs`: `low_center_of_mass_rights_a_tilted_box`; `src/shape.rs`: `offset_center_of_mass_moves_only_the_center` |
| Compound with per-child pose and user data | `Shape::new_compound`, `CompoundChild`, `CompoundSubShape` | `shapes.rs`: `compound_children_report_their_user_data`, `child_pose_is_applied`, `single_child_compound_keeps_its_user_data` |
| Shapes shared between bodies and worlds | `Shape` | `shapes.rs`: `shapes_are_shared_across_bodies_and_worlds`; `bodies.rs`: `shape_can_be_dropped_after_body_creation`; `ragdoll.rs`: `humanoid_settles_on_a_heightfield_in_a_second_world` (the same terrain and compound shapes in two worlds, identical ray hits) |
| Physics materials with user data | `PhysicsMaterial`, `Shape::new_box_with_material` and its siblings, `new_height_field_with_materials`, `ContactManifold::materials` | `src/material.rs`: `user_data_round_trips`, `convex_constructors_keep_the_plain_shape_and_carry_the_material`, `compound_children_resolve_their_own_materials`, `padded_height_field_cells_resolve_their_materials`, `single_material_height_field_resolves_everywhere`; `listeners.rs`: `materials_are_reported_for_each_side`; `crates/oxijolt-sys/tests/material_smoke.rs` |
| Ray cast | `PhysicsWorld::cast_ray`, `RayCast`, `RayHit` | `queries.rs`: `ray_hits_chunk_and_terrain_from_twenty_metres`, `ray_starting_inside_hits_at_exactly_zero`, `ray_normals_point_out_of_the_hit_surface`; `shapes.rs`: `height_field_is_hit_from_below` |
| Shape cast | `PhysicsWorld::cast_shape`, `ShapeCast`, `ShapeCastHit` | `queries.rs`: `capsule_cast_down_reports_floor_normal_and_distance`, `rotated_capsule_cast_uses_its_rotation`, `target_distance_stops_short`, `start_penetrating_is_reported_moving_in_not_out` |
| Collide shape | `PhysicsWorld::collide_shape`, `CollideShape`, `CollideShapeHit` | `queries.rs`: `collide_capsule_with_floor_reports_geometric_depth`, `collide_ceiling_normal_points_down`, `max_separation_reports_negative_depth` |
| Query filters: layers, child groups, excluded body | `QueryFilter` | `queries.rs`: `spawn_ground_ignores_canopy_by_group`, `object_layer_selection_skips_other_layers`, `excluded_body_is_skipped`, `nested_compound_group_governs_shape_casts_and_collide`, `filter_rejects_unknown_layer_and_foreign_body` |
| Queries without a step, broad-phase optimisation | `PhysicsWorld::optimize_broad_phase` | `queries.rs`: `queries_see_created_moved_and_removed_bodies_without_a_step`, `optimize_broad_phase_keeps_query_results` |
| Queries from many threads | queries on `&PhysicsWorld` | `queries.rs`: `filtered_queries_run_in_parallel`; `world.rs`: `rays_are_cast_from_many_threads` |
| Floating origin | `PhysicsWorld::rebase` | `rebase.rs`: `vehicle_drives_across_a_rotating_rebase`, `resting_item_stays_across_a_rebase`, `sleeping_body_stays_asleep_across_a_rebase`, `drift_rebase_equals_the_scene_built_in_the_new_frame`, `rays_answer_the_same_across_a_rebase`, `invalid_rebase_changes_nothing`; `ragdoll.rs`: `rebase_moves_a_settled_ragdoll_rigidly`; `constraints.rs`: `rebase_moves_constraints_rigidly`, `rebase_with_a_pulley_is_atomic`, `rebase_with_pulleys_is_repeatable` |
| Same results with 1 and 4 worker threads and with caller job systems, for the scenes the tests run ([determinism.md](determinism.md)) | `WorldSettings::worker_threads`, `WorldSettings::job_system` | `determinism.rs`: `stacks_digest_is_identical_across_thread_counts`, `chunk_digest_is_identical_across_thread_counts`, `walker_digest_is_identical_across_thread_counts`, `vehicle_digest_is_identical_across_thread_counts`, `fleet_digest_is_identical_across_thread_counts`, `fleet_digest_detects_a_changed_input`, `ragdoll_pile_digest_is_identical_across_thread_counts`, `constraint_digest_is_identical_across_thread_counts`, `soft_body_digest_is_identical_across_thread_counts`, `permuted_insertion_order_fails_the_gate` and the nine `*_digest_is_identical_with_caller_job_systems` gates; `event_determinism.rs`: `events_are_identical_with_1_and_4_workers`, `events_are_identical_with_caller_job_systems`, `observing_every_event_leaves_the_simulation_unchanged`; `state.rs`: `rollback_replay_is_identical_across_processes`; `vehicle.rs`: `drive_a_route_twice_gives_identical_bits`; `crates/oxijolt-sys/tests/determinism.rs`: `digest_is_identical_across_thread_counts` |
| World state save and restore | `PhysicsWorld::save_state`, `save_state_of`, `restore_state`, `WorldState`, `StateError` | `state.rs`: `rollback_replays_every_tick_bit_for_bit`, `a_state_restores_any_number_of_times`, `restore_undoes_saved_constraint_targets`, `restore_does_not_undo_unsaved_configuration`, `restore_after_a_structural_change_is_refused_and_changes_nothing`, `a_rejected_structural_call_keeps_a_state_restorable`, `a_failed_create_on_a_full_world_keeps_a_state_restorable`, `a_vehicle_in_zero_gravity_keeps_its_world_up_across_a_restore`, `a_state_of_another_world_is_refused`, `bodies_created_after_the_restore_point_get_the_original_ids`, `a_selected_bodies_state_replays_when_the_rest_is_static`, `bodies_left_out_of_a_state_keep_their_current_state`, `soft_bodies_replay_after_a_detour_restore`, `restore_does_not_undo_a_vertex_inverse_mass` and the other tests in the file; `state_leaks.rs`: `world_states_do_not_leak`; `crates/oxijolt-sys/tests/state_smoke.rs` |
| Debug wireframe as line data (`debug-renderer`) | `PhysicsWorld::debug_lines`, `DebugLines`, `DebugLineSettings` | `debug_lines.rs`: `near_colliders_present_far_absent_terrain_present`, `line_cap_gives_exactly_the_cap_and_truncated`, `hidden_groups_vanish`, `compound_child_pose_applies_child_rotation_before_body_rotation`; `debug_lines_leaks.rs`; `crates/oxijolt-sys/tests/debug_renderer_bindings.rs` |
| Leak gate for per-call joltc objects (Windows; bounded memory growth, catches only leaks above its threshold) | queries, rebase | `leaks.rs`: `per_call_joltc_objects_do_not_leak` |
| Character settings with Jolt's defaults, validation | `PhysicsWorld::create_character`, `CharacterSettings`, `CharacterError` | `character.rs`: `invalid_settings_and_poses_are_rejected_without_side_effects`; `src/character/tests.rs`: `default_settings_match_jolt` |
| Character ids per world, removal | `CharacterId`, `character_ids`, `remove_character` | `character.rs`: `ids_count_from_one_are_never_reused_and_belong_to_their_world`, `removing_a_character_mid_run_leaves_the_others_sound` |
| Character update: move by velocity, ground state and normal | `update_character`, `CharacterMut::set_linear_velocity`, `CharacterRef::ground_state`, `ground_normal`, `ground_body`, `GroundState` | `character.rs`: `a_character_lands_on_a_floor_and_reports_it`, `linear_velocity_round_trips_and_moves_a_free_character`, `a_slope_limit_near_zero_turns_the_limit_off`; `src/character/tests.rs`: `ground_states_convert_and_report_support` |
| Stick to floor and walk stairs | `ExtendedUpdateSettings` | `character.rs`: `walk_stairs_climbs_a_step_that_stops_a_character_without_it`; `walker.rs`: `walking_down_a_30_degree_slope_has_no_hops` (stick to floor as the floor snap) |
| Up and rotation per update (radial up) | `CharacterMut::set_up`, `set_rotation`, `set_position` | `character.rs`: `up_and_rotation_set_per_update_give_the_same_walk_in_another_frame`; `walker.rs`: `a_rotated_floor_far_from_the_anchor_keeps_the_path`, `a_chunk_seam_is_crossed` |
| Character contacts: body, layer, compound child group, contact normal; filters | `CharacterRef::active_contacts`, `CharacterContact`, `contact_compound_child`, `contact_object_layer`, `QueryFilter` | `character.rs`: `a_character_lands_on_a_floor_and_reports_it`, `compound_children_report_their_group_and_filters_select_them` |
| Inner body | `CharacterSettings::inner_body`, `InnerBody`, `is_inner_body`, `BodyError::OwnedByCharacter` | `character.rs`: `an_inner_body_is_a_body_of_the_world_owned_by_its_character`, `dropping_a_world_with_characters_releases_them` |
| Collisions between characters | `CharacterSettings::collide_with_characters` | `character.rs`: `characters_that_collide_with_characters_keep_apart`, `removing_a_character_mid_run_leaves_the_others_sound` |
| Character save and restore, chained replay | `CharacterRef::save_state`, `CharacterMut::restore_state`, `CharacterState` | `character.rs`: `a_chained_replay_of_one_character_is_bit_exact`, `a_chained_replay_of_colliding_characters_is_bit_exact`, `a_restored_state_saves_the_same_bytes`, `a_restored_state_moves_the_inner_body_with_the_character`; `crates/oxijolt-sys/tests/character_smoke.rs`: `character_lands_and_its_restored_state_continues_bit_for_bit` |
| Characters across a rebase | `PhysicsWorld::rebase`, `refresh_character_contacts` | `character.rs`: `a_character_crosses_a_rebase_with_the_world` |
| The game's near step on a radial planet (reference controller in test support, `tests/common/walker.rs`) | built on the rows above | `walker.rs`: `step_law_climbs_up_to_045_but_not_05`, `a_step_under_a_low_ceiling_is_not_taken`, `the_autostep_never_climbs_a_dynamic_body_in_the_filter`, `slope_law_climbs_30_degrees_and_slides_back_from_60`, `steep_slopes_always_bring_the_walker_down`, `a_buried_walker_is_put_back_on_the_terrain`, `an_overlapping_wall_pushes_the_walker_out`, `a_jump_reaches_the_ballistic_apex_and_lands`, `rising_into_a_ceiling_resets_vel_up`, `chained_near_step_replay_is_bit_exact` and the other tests in the file |
| Leak gate for characters (Windows) | create, update, save and restore, remove | `character_leaks.rs`: `characters_do_not_leak` |
| Vehicle settings with Jolt's defaults, validation | `VehicleSettings`, `WheelSettings`, `SuspensionSpring`, `VehicleEngineSettings`, `VehicleTransmissionSettings`, `VehicleDifferentialSettings`, `VehicleAntiRollBar` | `src/vehicle/settings.rs`: `wheel_defaults_are_jolts`, `controller_defaults_are_jolts`, `constraint_differential_and_anti_roll_bar_defaults_are_jolts`, `built_settings_reach_jolt`, `step_coefficients_of_wheels_must_be_finite`, `step_coefficients_of_the_drivetrain_must_be_finite` and the other validation tests in the file; `vehicle.rs`: `invalid_vehicles_create_nothing`. Anti-roll bars and the pitch and roll limit are tested for their defaults and validation only, not for their effect |
| Vehicle creation, ids, removal | `PhysicsWorld::create_vehicle`, `remove_vehicle`, `VehicleId`, `vehicle_ids`, `vehicle_of_body`, `VehicleError`, `BodyError::UsedByVehicle` | `vehicle.rs`: `invalid_vehicles_create_nothing`, `vehicle_ids_are_sequential_and_never_reused`, `chassis_cannot_be_removed_while_a_vehicle_uses_it`, `a_removed_vehicle_no_longer_drives`, `dropping_a_world_with_vehicles_is_clean` |
| Wheel collision testers and contact readout | `VehicleCollisionTester` (ray, sphere, cylinder), `VehicleRef::wheels`, `WheelState`, `WheelContact`, `VehicleMut::set_collision_tester` | `vehicle.rs`: `wheel_contacts_match_the_ground_geometry` (every tester on a box floor and a heightfield, against the analytic plane), `start_overlap_reports_a_hard_point_without_depth`, `start_overlap_depth_comes_from_collide_shape`, `collision_tester_can_be_replaced`, `wheels_pass_through_soft_bodies_to_the_ground` |
| Driver input, engine and gear readout | `DriverInput`, `VehicleMut::set_driver_input`, `VehicleRef::engine_rpm`, `current_gear` | `vehicle.rs`: `driver_input_drives_steers_and_brakes`, `braking_at_the_smallest_step_stays_finite` |
| Caller-applied gravity, sleep | `VehicleMut::set_gravity`, `VehicleRef::gravity`, `world_up` | `vehicle.rs`: `gravity_override_replaces_world_gravity`, `gravity_is_bounded_by_max_acceleration`, `sleeping_chassis_gets_no_override_force`, `never_sleeping_chassis_stays_awake` |
| A car drives a route over heightfield terrain | built on the rows above (route and controller in `tests/common/vehicle.rs`) | `vehicle.rs`: `drive_a_route_over_terrain` (four waypoints, never tipped past 60°, two wheels down on 95 % of the ticks), `drive_a_route_twice_gives_identical_bits` |
| Leak gate for vehicles (Windows) | create, drive, read, replace the tester, remove | `vehicle_leaks.rs`: `vehicles_do_not_leak` |
| Constraint settings with Jolt's defaults, validation (ragdoll joints and world constraints) | `SwingTwistConstraintSettings`, `HingeConstraintSettings`, `SixDofConstraintSettings`, `MotorSettings`, `SpringSettings`, `SwingType`, `ConstraintSpace` | `src/constraint/tests.rs`: `constraint_base_is_jolts_default`, `swing_twist_defaults_are_jolts`, `hinge_defaults_are_jolts`, `six_dof_defaults_are_jolts`, `fixed_and_free_map_to_jolts_sentinels`, `frames_are_validated`, `swing_twist_limits_are_validated`, `hinge_limits_are_validated`, `six_dof_limits_are_validated`, `motors_and_springs_are_validated`; `crates/oxijolt-sys/tests/constraint_smoke.rs`: `new_settings_init_to_jolts_defaults` |
| World constraints of twelve kinds | `PhysicsWorld::create_constraint` with `FixedConstraintSettings`, `PointConstraintSettings`, `DistanceConstraintSettings`, `HingeConstraintSettings`, `SliderConstraintSettings`, `ConeConstraintSettings`, `SwingTwistConstraintSettings`, `SixDofConstraintSettings`, `GearConstraintSettings`, `RackAndPinionConstraintSettings`, `PulleyConstraintSettings`, `PathConstraintSettings` | `constraints.rs`: `fixed_constraint_holds_a_box_under_a_static_bar`, `point_constraint_keeps_the_pendulum_pivot`, `distance_constraint_keeps_a_rod_length`, `hinge_limit_holds_a_falling_door`, `slider_limit_holds`, `cone_holds_a_hanging_body_within_its_angle`, `swing_twist_twist_limit_holds`, `six_dof_translation_limit_holds`, `six_dof_pyramid_holds_asymmetric_limits`, `gear_turns_the_second_hinge_at_the_ratio`, `rack_moves_at_the_pinion_rate`, `pulley_lifts_one_body_as_the_other_falls`, `pulley_with_ratio_two_moves_half_as_far`, `path_keeps_a_body_on_its_curve`; `crates/oxijolt-sys/tests/constraint_smoke.rs` |
| Constraint motors, springs, targets and readout | `ConstraintRef`, `ConstraintMut`, `MotorState` | `constraints.rs`: `hinge_velocity_motor_reaches_its_target_speed`, `hinge_position_motor_reaches_its_target_angle`, `soft_distance_spring_oscillates_and_damps`, `slider_position_motor_reaches_its_target`, `slider_soft_limit_spring_oscillates_and_damps`, `swing_twist_motor_reaches_target_orientation_with_a_rotated_parent`, `swing_twist_reads_back_its_motor_settings`, `six_dof_translation_spring_oscillates_and_damps`, `six_dof_rotation_position_motor_reaches_its_target`, `path_motor_drives_to_a_target_fraction`, `path_reads_back_its_motor_looping_and_rotation_impulses`, `looping_path_wraps`, `hinge_setters_check_their_values`, `changed_limits_motors_and_friction_act_on_sleeping_bodies` |
| Constraint ids, guards and removal | `ConstraintId`, `AnyConstraintId`, `remove_constraint`, `constraint_ids`, `constraints_of_body`, `ConstraintError`, `BodyError::UsedByConstraint` | `constraints.rs`: `constraint_ids_are_typed_ordered_and_never_reused`, `wrong_world_and_removed_ids_are_rejected`, `bodies_of_constraints_cannot_be_removed`, `constraints_refuse_ragdoll_parts_inner_bodies_and_one_body`, `gears_racks_and_pulleys_need_two_dynamic_bodies`, `invalid_constraint_creates_nothing`, `removing_a_constraint_wakes_its_bodies`, `a_new_constraint_wakes_its_sleeping_bodies`, `every_setter_wakes_the_constraint_bodies`, `dropping_a_world_with_constraints_is_clean` |
| Gears and racks that reference hinges and sliders | `GearConstraintSettings::hinges`, `RackAndPinionConstraintSettings::constraints`, `ConstraintError::UsedByConstraint` | `constraints.rs`: `gear_references_correct_drift`, `rack_references_correct_drift`, `a_referenced_hinge_cannot_be_removed`, `unrelated_or_reversed_references_are_rejected`, `gear_keeps_its_velocity_relation_at_the_largest_ratio` |
| Hermite paths | `HermitePath`, `HermitePathPoint`, `PathRotationConstraint` | `constraints.rs`: `invalid_paths_are_rejected`, `path_validation_accepts_its_boundary`, `looping_path_wraps` |
| Leak gate for constraints (Windows) | create, step, read, remove | `constraint_leaks.rs`: `constraints_do_not_leak` |
| Skeleton, ragdoll settings, stabilized masses | `Skeleton`, `SkeletonJoint`, `RagdollSettings::new`, `new_stabilized`, `RagdollPart`, `RagdollJoint` | `src/ragdoll/settings.rs`: `skeletons_are_validated`, `ragdoll_settings_are_validated`, `heightfield_parts_are_rejected`, `stabilized_settings_build`; `ragdoll.rs`: `plain_settings_keep_the_masses_and_stabilized_ones_the_total`, `one_settings_value_serves_two_worlds` |
| Ragdoll creation, ids, removal | `PhysicsWorld::create_ragdoll`, `remove_ragdoll`, `RagdollId`, `ragdoll_ids`, `ragdoll_of_body`, `RagdollError`, `BodyError::OwnedByRagdoll` | `ragdoll.rs`: `ragdoll_parts_and_ids_are_guarded` (including a world too small for the parts), `removing_a_ragdoll_drops_what_rests_on_it`, `dropping_a_world_with_ragdolls_is_clean`; `src/ragdoll/mod.rs`: `a_created_chain_reports_its_parts_and_joints` |
| No collisions between a ragdoll's own parts | `RagdollSettings` | `ragdoll.rs`: `two_ragdolls_collide_with_each_other_but_not_with_themselves`, `the_same_parts_as_plain_bodies_collide` (the control), `humanoid_settles_on_a_heightfield_in_a_second_world` (every part pair on every tick while the ragdoll is awake) |
| Pose readout and writing, kinematic drive | `RagdollRef::pose`, `root_transform`, `RagdollMut::set_pose`, `drive_to_pose_using_kinematics`, `set_motion_type`, `SkeletonPose` | `ragdoll.rs`: `poses_read_back_in_world_transforms`, `kinematic_parts_reach_the_driven_pose`; `src/ragdoll/settings.rs`: `poses_are_validated` |
| Motor drive and joint readings | `RagdollMut::drive_to_pose_using_motors`, `stop_motors`, `RagdollRef::joint`, `joint_motors_on`, `JointReading` | `ragdoll.rs`: `motors_drive_each_joint_kind_to_its_target` (a hinge elbow and knee, a six-DOF hip and a swing-twist neck, driven from the bind pose), `motors_drive_joints_of_a_turned_ragdoll_in_the_parent_frame`, `rebase_moves_a_settled_ragdoll_rigidly` (joint readings before and after a rebase), `humanoid_settles_on_a_heightfield_in_a_second_world` (the limit check reads every joint) |
| Settle detection | `SettleDetector`, `RagdollRef::is_calm` | `src/ragdoll/settle.rs`: `calm_updates_in_a_row_settle`, `default_is_the_rest_rule`, `invalid_limits_are_rejected`; `ragdoll.rs`: `humanoid_settles_on_a_heightfield_in_a_second_world` |
| A 12-capsule humanoid settles on a heightfield in a second world (fixture in `tests/common/ragdoll.rs`) | built on the rows above | `ragdoll.rs`: `humanoid_settles_on_a_heightfield_in_a_second_world` (caller-applied gravity, the main world stepping on another thread, settled within 600 ticks (measured: tick 141), joints within 0.01 rad of their limits at rest), `humanoid_joints_overshoot_their_limits_only_boundedly_during_the_drop` (below 0.40 rad on every tick of that drop), `an_environment_ray_passes_through_a_resting_part` |
| Leak gate for ragdolls (Windows) | settings, create, drive, read, remove | `ragdoll_leaks.rs`: `ragdolls_do_not_leak` |
| Soft bodies: shared settings, creation, vertex readout and writes | `SoftBodySharedSettings`, `SoftBodySharedSettingsBuilder`, `SoftBodySettings`, `PhysicsWorld::create_soft_body`, `soft_body`, `soft_body_mut`, `SoftBodyRef`, `SoftBodyMut` | `soft_body.rs`: `a_cloth_pinned_at_two_corners_drapes_over_a_sphere`, `vertices_read_back_in_world_space`, `pinning_and_unpinning_vertices`, `invalid_vertex_writes_change_nothing`, `a_kinematic_vertex_moves_to_its_target_and_keeps_its_velocity`, `a_pressurised_ball_keeps_its_size_on_the_floor`, `a_tetrahedral_cube_keeps_its_volume_on_the_floor`, `explicit_constraints_add_to_the_generated_ones`, `explicit_edges_without_faces_hold_a_pendulum`, `queries_find_a_cloth`, `one_shared_settings_serves_worlds_on_two_threads` and the other tests in the file; `src/soft_body/tests.rs`: `default_settings_match_jolt`, `default_vertex_attributes_match_jolt`, `invalid_vertices_are_rejected`, `invalid_faces_are_rejected`; `crates/oxijolt-sys/tests/soft_body_smoke.rs` |
| Body APIs a soft body refuses | `BodyError::SoftBody`, `BodyMut::add_force` on a soft body | `soft_body.rs`: `body_level_velocity_torque_and_point_force_refuse_soft_bodies`, `constraints_and_vehicles_refuse_soft_bodies`, `add_force_on_a_rotated_soft_body_pushes_along_the_world_force` |
| Leak gate for soft bodies (Windows) | shared settings, create, step, read, write, remove | `soft_body_leaks.rs`: `soft_bodies_do_not_leak` |
| Contact, activation and soft body contact events | `EventSettings`, `PhysicsWorld::set_event_settings`, `take_events`, `WorldEvents`, `ContactEvent`, `ActivationEvent`, `SoftBodyContacts`, `SoftBodyValidation` | `listeners.rs`: `a_default_world_reports_nothing`, `contacts_are_added_persisted_and_removed_with_their_sub_shapes`, `persisted_contacts_are_opt_in`, `removing_a_body_reports_its_contact_removed_with_the_stale_id`, `falling_asleep_removes_contacts_and_deactivates`, `activation_follows_creation_wake_sleep_and_removal_but_not_restore`, `several_steps_queue_in_step_order`, `worlds_stepped_on_two_threads_see_only_their_own_events`, `a_cloth_on_a_table_reports_its_vertex_contacts`, `rotated_cloth_contacts_are_converted_to_world_space`, `a_replay_after_a_detour_reports_the_same_events` and the other tests in the file; `src/listener/tests.rs`: `a_panic_in_any_callback_during_a_step_is_resumed_by_step`, `a_panic_outside_a_step_is_resumed_by_take_events`; `crates/oxijolt-sys/tests/listener_smoke.rs` |
| Contact listener | `ContactListener`, `PhysicsWorld::set_contact_listener`, `ContactSettings`, `SoftBodyContactSettings`, `ContactSettingsRejection`, `StepReport::rejected_contact_settings` | `contact_listener.rs`: `a_listener_makes_ice_slippery`, `a_conveyor_moves_a_resting_cube`, `sensor_contacts_let_a_cube_fall_through_and_are_still_reported`, `rejected_soft_body_contacts_let_a_cloth_fall_through`, `a_panicking_listener_is_resumed_by_step`, `continuous_collision_with_a_listener_steps_cleanly`; `src/listener/tests.rs`: `settings_are_checked_again_against_the_contact_that_takes_them`, `a_rejection_is_recorded_and_the_listener_is_still_called`, `settings_moved_to_a_contact_they_do_not_fit_are_rejected` |
| Leak gate for listeners, materials and worlds (Windows) | listener replacement, material shapes, a world per round | `listener_leaks.rs`: `listeners_materials_and_worlds_do_not_leak` ([events.md](events.md#leaks) describes the gate) |
| Magnitude policy | `oxijolt::limits` | `limits.rs` and the unit tests in `src/limits/tests.rs`; the table [below](#inputs-and-their-boundary-tests) names the test of each input |
| Jolt assertion handler (`asserts` feature) | installed once per process before `JPH_Init` | `assertions.rs`: `a_failed_jolt_assertion_aborts_with_its_message`; `src/jolt_assert.rs`: `only_the_pinned_update_error_continues` |
| Raw bindings | `oxijolt-sys` | `crates/oxijolt-sys/tests/smoke_test.rs`: `box_falls_onto_static_box`, `callback_job_system_steps_a_world`; `character_smoke.rs`: `character_lands_and_its_restored_state_continues_bit_for_bit`, `copying_no_bytes_accepts_a_null_buffer`; `vehicle_smoke.rs`: `vehicle_constraint_steps_and_tears_down`; `ragdoll_smoke.rs`: `ragdoll_parts_and_constraints_reach_jolt`; `constraint_smoke.rs`, `state_smoke.rs`, `soft_body_smoke.rs`, `listener_smoke.rs`, `material_smoke.rs`; layout checks at compile time in `crates/oxijolt-sys/src/layout.rs` |
| Committed bindings and the supported targets | `oxijolt-sys`, `cargo xtask bindings` | `xtask/src/main.rs`: `every_registered_target_is_accepted`, `narrow_and_unregistered_targets_are_refused`, `bindings_files_are_distinct`, `every_selectable_bindings_file_is_committed` and the other tests in the file; CI's `Committed bindings` job compares them with LLVM 18 output |
| Examples in the guides and the README | [guide.md](guide.md) and the other guides in `docs/`, `README.md` | doctests of the `#[cfg(doctest)]` items at the end of `crates/oxijolt/src/lib.rs` (`cargo test -p oxijolt --doc`); the `hello_world` example, which CI runs |

CI runs the whole test suite on Windows (MSVC) and Linux (GCC), each in five configurations:
default, `cross-platform-deterministic`, `double-precision`, `debug-renderer` and `asserts`. The
tests behind the `debug-renderer` feature run only in that configuration. The leak gates read the
process's private bytes through a Windows API and are built only on Windows.

Not in the safe API yet (joltc exposes them, so `oxijolt-sys` has them):
- mesh, convex hull, scaled and tapered shapes;
- for rigid bodies: impulses, `MoveKinematic`, activating and deactivating a body on demand, the
  sensor flag, and changing a body's shape or motion type after creation;
- tracked vehicles, motorcycles and the manual transmission;
- the skeleton mapper, Jolt's rigid-body `Character` and the character contact listener.

Skinned soft bodies (Jolt's skin constraints) are in neither layer.

## Inputs and their boundary tests

Every public setter and constructor that takes a magnitude, the rule it applies, and the tests that
check it at its boundary.

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
- contact and constraint impulses the solver generates: spheres of `MIN_MASS` and `MAX_MASS`
  colliding head-on at the velocity bounds, restitution 1, friction at `MAX_FRICTION` including a
  speculative contact with zero normal impulse, motors at the spring bound, six-DOF translation
  limits at the extent bound between parts of both mass extremes, suspension springs and anti-roll
  bars at their bounds (`bodies_at_every_bound_step_finitely`,
  `friction_at_the_bound_keeps_contacts_finite`,
  `six_dof_translation_limits_at_the_bound_step_finitely`,
  `vehicle_springs_and_anti_roll_bar_at_their_bounds_step_finitely`,
  `motor_springs_at_the_coefficient_bound_drive_finitely`);
- impulses at a lever arm on the lightest bodies: the weight impulse of a character of `MAX_MASS` at
  `MAX_WEIGHT_IMPULSE` on the edge of a 6 cm cube of `MIN_MASS`, the cube that turns fastest
  (`character_weight_impulse_at_a_lever_arm_is_bounded`), and a box of `MIN_MASS` pressed on its
  floor and pushed at the velocity bound by such a character
  (`character_at_its_bounds_pushes_the_lightest_body`);
- query arithmetic: rays, shape casts and collisions from frame corners
  (`query_inputs_are_bounded_by_the_frame`) and a sphere at the extent bound cast across the frame
  and collided with separations up to the extent bound
  (`queries_with_shapes_at_the_extent_bound_stay_finite`);
- world constraints: gears at both ends of `1..=MAX_GEAR_RATIO`, racks and pinions and pulleys at
  both `MAX_RATIO` bounds between bodies of both mass extremes, one of them turning or sliding at
  the velocity bound (`ratios_at_the_bound_step_finitely`,
  `pulleys_at_the_ratio_bound_step_finitely`); motor and limit springs accepted at the
  effective-mass bound and rejected one representable frequency above it
  (`world_constraint_springs_are_bounded_by_the_bodies_effective_mass`,
  `slider_swing_twist_and_six_dof_motor_springs_are_bounded`,
  `path_motor_springs_and_friction_are_bounded`); targets at their bounds
  (`constraint_targets_are_bounded`, `slider_cone_swing_twist_and_six_dof_targets_are_bounded`,
  `path_inputs_are_bounded`); hinge, slider, swing-twist and path friction of `f32::MAX` on bodies
  moving at the velocity bounds (`constraint_friction_at_f32_max_steps_finitely`,
  `slider_and_swing_twist_friction_at_f32_max_steps_finitely`,
  `path_motor_springs_and_friction_are_bounded`); and a path segment at the margin of the segment
  check stepped end to end with every rotation constraint (`path_validation_accepts_its_boundary`);
  points that hold a dynamic body at `MAX_LEVER_ARM_RATIO`: a ball joint and a hinge on a static
  body, and a weld, a six-DOF joint with every axis fixed and a swing-twist joint with zero ranges
  on a static body or between two bodies, each holding a 1 g, 6 cm cube or a 1 kg, 1 m cube at the
  velocity bounds under gravity (`constraints_at_the_lever_arm_bound_step_finitely`).

## Not covered

- State the simulation produces itself is not an input and is not checked again: a body Jolt carries
  out of the frame, the positions a rebase computes (only checked to be finite), or a character
  state restored with `CharacterMut::restore_state`, which can only come from
  `CharacterRef::save_state`.
- Bodies with a principal inverse inertia above `√3 · 1e6`, such as a light needle-thin shape or a
  centre of mass far from the shape. `PhysicsWorld::create_body` bounds no inverse inertia from
  above: an exactly diagonal tensor (an unrotated shape, or a centre of mass moved along a principal
  axis) only needs finite inverse moments, and the rigid body inertia floor (see
  [limits.md](limits.md#rigid-body-inertia)) bounds how badly conditioned any other tensor is, not
  how small it is. The boundary tests of that floor step slender bodies for two ticks without
  contacts or impulses, so impulses at a lever arm on such a body, including a character's weight
  impulse, are neither derived nor tested. The same holds for the suspension effective mass Jolt
  forms from a wheel's force point and the chassis's inverse inertia
  (`VehicleConstraint.cpp:448-451`).
- Constraints whose solver diverges for reasons the lever-arm ratio does not measure. In the
  `asserts` build Jolt then asserts that a squared velocity is finite (`MotionProperties.inl:28` or
  `:38`); in the release build the joint tears apart by metres and the state can become NaN: Jolt's
  velocity clamp scales a velocity whose squared length overflows to zero, turns an already infinite
  component into NaN (`inf * 0`) and lets NaN through. No impact is needed. Measured with one
  constraint to a static body under gravity unless stated otherwise:
  - a hinge whose pin lies off the least-inertia axis of a slender body, outside its cross section
    (an offset `d` of about twice the half thickness `t` or more). A 2 kg rod of 1 m by 2 cm hinged
    5 cm off its axis (a lever-arm ratio of 43.5) asserted while swinging about the hinge at 10
    rad/s; a 10 kg barrier arm of 3 m by 5 cm on a 5 cm bracket (ratio 12) asserted at 30 rad/s. In
    the release build both became NaN within a few hundred steps of such a swing (the step depends
    on the scene); a 1 m rod 1 cm thick with the pin 5 cm beside it along the hinge line asserted
    while falling from horizontal under gravity alone. On 1 m rods swung at up to 47 rad/s, `t` = 5
    mm failed from `d` = 1 cm, `t` = 1 cm from 2 cm, `t` = 2 cm only at 10 cm, and `t` = 5 cm not up
    to 10 cm;
  - chains of hinges with non-parallel axes, also between cubes: of 100 seeded chains of 3 to 10
    cubes, 3 asserted at kicks of 15 m/s and 1.4 rad/s per body at lever-arm ratios of 10 to 100,
    and 18 at ratios of 100 to 500; in the release build 2 of 100 chains became NaN at 15 m/s and 31
    of 100 at 50 m/s;
  - a light body held by two to four constraints to static anchors, also without a hinge (point and
    fixed, cone, distance and six-DOF, ...): of 300 seeded scenes at 0.3 to 1 of the lever-arm
    bound, 24 asserted at the velocity bounds with every constraint kind and 8 without hinges, 3 at
    a tenth of the bounds and none at 3 %.

  The same probes found no assert and no NaN for a 2 m by 1 m by 5 cm door hinged at its edge,
  capsule limbs on hinges, a rod hinged on its axis, or the 12-capsule test ragdoll, at kicks up to
  499 m/s and 47 rad/s, but their joints still opened by more than 0.5 m: of 24 kicks at 150 m/s, 8
  for the door, 10 for the rod and 9 for a forearm or shin capsule, and at 499 m/s 16, 24 and 19, up
  to 3.3 m for the door, 4.1 m for the rod and 7.3 m for the capsules; one of 24 exploded ragdolls
  opened by 1 m at 150 m/s. A six-DOF joint with the hinge's free axis did not assert on the slender
  bodies but let the joint drift apart by 0.2 to 3.3 m while swinging and up to 6.9 m under kicks of
  50 m/s, so it is no workaround. No bound in `oxijolt::limits` excludes these cases.
- A slider adds the distance travelled along its axis to the lever of body 1
  (`SliderConstraint.cpp`), which `MAX_LEVER_ARM_RATIO` checks only at creation; with a dynamic body
  1 and a long travel the lever grows beyond the bound.
- Inputs that are only checked to be finite or ordered, as the table above says: ray directions,
  damping, motor force and torque limits, ragdoll joint friction (world constraint friction is
  probed as above), wheel friction curves and the wheel and drivetrain values that only have to give
  finite step coefficients.
- A soft body with pressure whose volume shrinks after creation (crushed, or its vertices moved or
  recentred by Jolt so that an open mesh encloses less): Jolt divides the pressure by the volume of
  every sub-step (`SoftBodyMotionProperties.cpp:300-307`), and the creation check holds only for the
  start geometry.
- Soft body constraint stability: `MAX_COMPLIANCE` keeps Jolt's compliance terms finite, not the
  solver convergent. Measured: a 2 m cube of 1 g vertices, three of them kinematic, with six
  tetrahedral volume constraints of compliances 0 to 1e20, per-vertex attributes with edge and shear
  compliances up to 1e20 and LRA multipliers up to 1e4, distance bends, 100 iterations, no damping,
  restitution 1, a vertex speed limit of 1 m/s, one vertex given 155 m/s and an accepted force of
  3.7e6 N, had NaN vertices after its first step in the release build and asserted that a squared
  velocity is finite (`MotionProperties.inl:28`) in the asserts build. With a tenth of the force, 5
  iterations, no volume constraints or uniform attributes it stayed finite. No bound in
  `oxijolt::limits` excludes it. Two more seeded scenes of the same kind failed in the asserts
  build: a cube of 1 g vertices with six volume constraints, 37 iterations, a step of 0.1 s,
  friction 1000, a vertex radius of 2000 m, a force of 3.7e6 N and a vertex unpinned to 1 kg
  asserted at `MotionProperties.inl:28`; a cube with pressure 1e5 under gravity of about 1000 m/s²,
  pushed by repeated forces of 3.8e9 N and resting against rigid bodies (without them it did not
  fail), grew bounds beyond `cLargeFloat` (`QuadTree.cpp:68`).
- `RagdollSettings::new_stabilized` reports Jolt's `Stabilize` failing to decompose an inertia
  tensor as an error, but Jolt asserts on that path first (`Ragdoll.cpp:158`).
- The assertion `errors == EPhysicsUpdateError::None` at the end of every step that drops contacts
  (`PhysicsSystem.cpp:679`) is intentional: `PhysicsWorld::step` returns the same errors in its
  `StepReport`, and oxijolt's assertion handler lets the process continue for this assertion only.
