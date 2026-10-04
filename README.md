<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="docs/assets/oxijolt-logo-dark.png">
    <img alt="oxijolt" src="docs/assets/oxijolt-logo-light.png" width="420">
  </picture>
</p>

# oxijolt
[![CI](https://github.com/pockerhead/oxijolt/actions/workflows/ci.yml/badge.svg)](https://github.com/pockerhead/oxijolt/actions/workflows/ci.yml)
[![License: MIT OR Apache-2.0](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue.svg)](#license)
[![Jolt Physics 5.6.0](https://img.shields.io/badge/Jolt%20Physics-5.6.0-orange.svg)](https://github.com/jrouwe/JoltPhysics/releases/tag/v5.6.0)

Rust bindings for [Jolt Physics](https://github.com/jrouwe/JoltPhysics) 5.6.0 through the [joltc] C
wrapper: raw `unsafe` bindings and a safe, idiomatic API on top. The safe API is written for a
deterministic, headless simulation that a game's ECS owns: rigid bodies, terrain heightfields,
compound shapes with per-child collision groups, scene queries, a floating origin, a virtual
character, a wheeled vehicle and ragdolls. Everything runs and is tested without a window.

The project is young. The API changes between versions, and only Windows (MSVC) is built and
tested in CI.

## Origin
The repository started as a fork of jolt-rust. Of its code only the workspace layout, the licences
and a few `build.rs` details (the Android NDK cross toolchain, the switch behind the
`cross-platform-deterministic` feature) are left. The crates were renamed (`joltc-sys` became
`oxijolt-sys`, `rolt` became `oxijolt`; before the first release they were briefly `joltphysics-sys`
and `joltphysics`), the raw layer sits on a different C wrapper ([joltc] instead of JoltC) and the
safe API was written from scratch. What carries over is the design: a raw crate and a safe crate,
Jolt built by CMake from cargo, and JoltC's soundness rules.
[LINEAGE.md](LINEAGE.md) credits the projects this work builds on.

## Crates

### `oxijolt-sys`: raw bindings
`bindgen` output over joltc's `joltc.h` and this repository's extension `native/joltc_ext/joltc_ext.h`,
with joltc's `JPH_*` names. joltc and Jolt are pinned submodules, compiled unmodified; the extension
adds a few functions in joltc's naming (character state save and restore, character updates with
explicit filters and temp allocator, the constraint base of a vehicle, ragdoll parts and joints
that keep every setting, and swing-twist and hinge motor access). The FFI structs in use have
their layout asserted on the C++ side (`native/layout_checks.cpp`) and the Rust side
(`src/layout.rs`). The crate docs list joltc's
own rules for using the raw API.

Features:
- `asserts`: compile Jolt with its debug assertions.
- `double-precision`: world positions in `f64` (`Real`, `JPH_RVec3`).
- `cross-platform-deterministic`: build Jolt with `CROSS_PLATFORM_DETERMINISTIC` (see
  [Determinism](#determinism)).
- `debug-renderer`: compile Jolt's debug renderer and bind joltc's drawing functions.

### `oxijolt`: safe API
A `PhysicsWorld` owns a Jolt physics system with its collision layers, job system and temp
allocator. Changing a world takes `&mut`, reading it (body state, queries) takes `&`, and worlds
are `Send` and `Sync`. Inputs Jolt only checks with debug assertions are validated and reported as
typed errors.

Features: `double-precision`, `cross-platform-deterministic` and `debug-renderer` forward to
`oxijolt-sys`; `debug-renderer` also enables `PhysicsWorld::debug_lines`.

Start with the [guide](docs/guide.md) (a terrain, a chunk compound, an item, queries and a rebase;
a character walking on a planet; a car on terrain and a ragdoll in a second world; all run as
tests), the crate docs (`cargo doc -p oxijolt --open`) and the `hello_world` example
(`cargo run -p oxijolt --example hello_world`).
Timings against the game's budgets are in [docs/benchmarks.md](docs/benchmarks.md)
(`cargo bench -p oxijolt --bench budgets`).

## What is bound and tested
Tests are in `crates/oxijolt/tests/` unless a path says otherwise; `src/...` names a unit test
in `crates/oxijolt/src/`.

| Feature | Safe API | Tests |
|---|---|---|
| World with settings and validation | `PhysicsWorld::new`, `WorldSettings` | `world.rs`: `default_world_steps`, `invalid_settings_are_rejected`, `gravity_round_trips_including_zero` |
| Step with a report of dropped work, time step bounds | `PhysicsWorld::step`, `StepReport`, `StepError`, `MIN_DELTA_TIME`, `MAX_DELTA_TIME` | `world.rs`: `step_rejects_bad_delta_time`, `step_rejects_delta_time_below_the_bound`, `step_rejects_delta_time_above_the_bound`, `full_contact_constraint_buffer_is_reported_and_the_world_advances`, `full_body_pair_cache_is_reported_and_the_world_advances` |
| Collision layers | `CollisionLayers`, `ObjectLayer`, `BroadPhaseLayer` | `dynamics.rs`: `collision_layers_decide_which_bodies_touch`; `src/layers.rs`: `validate_rejects_inconsistent_tables` |
| Several worlds, threads | `PhysicsWorld: Send + Sync`, `WorldSettings::worker_threads` | `dynamics.rs`: `two_worlds_side_by_side_do_not_interfere`, `worlds_step_in_parallel_threads`; `world.rs`: `world_and_shape_are_send_and_sync`, `world_is_readable_from_many_threads` |
| Static, kinematic and dynamic bodies, ids | `BodySettings`, `create_body`, `BodyId` | `bodies.rs`: `ids_follow_insertion_order`, `foreign_and_stale_ids_are_rejected`, `invalid_body_settings_are_rejected`, `full_world_rejects_another_body` |
| Pose and velocity, read and write | `BodyRef`, `BodyMut`, `Activation` | `bodies.rs`: `pose_and_velocity_read_back_as_exact_bits`, `activation_decides_whether_a_pose_write_wakes_the_body` |
| Forces, torque, reset | `BodyMut::add_force`, `add_force_at_point`, `add_torque`, `reset_forces` | `bodies.rs`: `forces`, `reset_forces_ignores_static_and_kinematic_bodies` |
| Sleeping and active state | `BodyRef::is_sleeping`, `is_active` | `bodies.rs`: `sleeping_flag_is_readable` |
| Removal wakes the bodies around it | `PhysicsWorld::remove_body` | `bodies.rs`: `removal_wakes_bodies_resting_on_it`, `removal_wakes_the_same_bodies_whether_or_not_the_broad_phase_was_optimized` |
| Mass override, continuous collision, gravity factor | `BodySettings::mass`, `motion_quality`, `gravity_factor` | `bodies.rs`: `forces`, `mass_too_small_to_invert_is_rejected`, `gravity_factor_scales_the_fall`; `dynamics.rs`: `linear_cast_body_does_not_tunnel_through_thin_static` |
| Damping, mass readout | `BodySettings::linear_damping`, `angular_damping`, `BodyRef::mass` | `src/body.rs`: `damping_reaches_the_body`, `invalid_damping_is_rejected`, `mass_is_reported_for_dynamic_bodies_only`; `ragdoll.rs`: `plain_settings_keep_the_masses_and_stabilized_ones_the_total` |
| Contacts of the last step | `PhysicsWorld::were_bodies_in_contact` | `ragdoll.rs`: `the_same_parts_as_plain_bodies_collide`, `two_ragdolls_collide_with_each_other_but_not_with_themselves` |
| Enhanced internal edge removal | `BodySettings::enhanced_internal_edge_removal` | `src/body.rs`: `enhanced_internal_edge_removal_reaches_the_body` (reads the flag back from the Jolt body); `dynamics.rs`: `enhanced_internal_edge_removal_smooths_sliding_over_a_compound` |
| Box, sphere, Y-cylinder, Y-capsule, convex radius | `Shape::new_box`, `new_box_with_convex_radius`, `new_sphere`, `new_cylinder`, `new_capsule` | `shapes.rs`: `sharp_box_edge_ray_hits_the_exact_corner`, `convex_radius_rounds_box_edges_for_contacts`; `src/shape.rs`: `cylinder_and_capsule_dimensions_reach_jolt`, `invalid_dimensions_are_rejected` |
| Heightfield (n = 33, holes, block size, active-edge threshold) | `Shape::new_height_field`, `HeightFieldSettings` | `shapes.rs`: `height_field_33_builds_and_matches_samples_at_nodes`, `height_field_rising_along_z_matches_analytic_surface`, `height_field_of_holes_has_no_collision`, `height_field_with_custom_active_edge_threshold_builds`, `active_edge_threshold_decides_the_normal_on_a_gentle_ridge`, `static_only_shapes_are_rejected_for_moving_bodies` |
| Offset centre of mass | `Shape::new_offset_center_of_mass` | `shapes.rs`: `low_center_of_mass_rights_a_tilted_box`; `src/shape.rs`: `offset_center_of_mass_moves_only_the_center` |
| Compound with per-child pose and user data | `Shape::new_compound`, `CompoundChild`, `CompoundSubShape` | `shapes.rs`: `compound_children_report_their_user_data`, `child_pose_is_applied`, `single_child_compound_keeps_its_user_data` |
| Shapes shared between bodies and worlds | `Shape` | `shapes.rs`: `shapes_are_shared_across_bodies_and_worlds`; `bodies.rs`: `shape_can_be_dropped_after_body_creation`; `ragdoll.rs`: `humanoid_settles_on_a_heightfield_in_a_second_world` (the same terrain and compound shapes in two worlds, identical ray hits) |
| Ray cast | `PhysicsWorld::cast_ray`, `RayCast`, `RayHit` | `queries.rs`: `ray_hits_chunk_and_terrain_from_twenty_metres`, `ray_starting_inside_hits_at_exactly_zero`, `ray_normals_point_out_of_the_hit_surface`; `shapes.rs`: `height_field_is_hit_from_below` |
| Shape cast | `PhysicsWorld::cast_shape`, `ShapeCast`, `ShapeCastHit` | `queries.rs`: `capsule_cast_down_reports_floor_normal_and_distance`, `rotated_capsule_cast_uses_its_rotation`, `target_distance_stops_short`, `start_penetrating_is_reported_moving_in_not_out` |
| Collide shape | `PhysicsWorld::collide_shape`, `CollideShape`, `CollideShapeHit` | `queries.rs`: `collide_capsule_with_floor_reports_geometric_depth`, `collide_ceiling_normal_points_down`, `max_separation_reports_negative_depth` |
| Query filters: layers, child groups, excluded body | `QueryFilter` | `queries.rs`: `spawn_ground_ignores_canopy_by_group`, `object_layer_selection_skips_other_layers`, `excluded_body_is_skipped`, `nested_compound_group_governs_shape_casts_and_collide`, `filter_rejects_unknown_layer_and_foreign_body` |
| Queries without a step, broad-phase optimisation | `PhysicsWorld::optimize_broad_phase` | `queries.rs`: `queries_see_created_moved_and_removed_bodies_without_a_step`, `optimize_broad_phase_keeps_query_results` |
| Queries from many threads | queries on `&PhysicsWorld` | `queries.rs`: `filtered_queries_run_in_parallel`; `world.rs`: `rays_are_cast_from_many_threads` |
| Floating origin | `PhysicsWorld::rebase` | `rebase.rs`: `vehicle_drives_across_a_rotating_rebase`, `resting_item_stays_across_a_rebase`, `sleeping_body_stays_asleep_across_a_rebase`, `drift_rebase_equals_the_scene_built_in_the_new_frame`, `rays_answer_the_same_across_a_rebase`, `invalid_rebase_changes_nothing`; `ragdoll.rs`: `rebase_moves_a_settled_ragdoll_rigidly` |
| Same results with 1 and 4 worker threads, for the scenes the tests run | `WorldSettings::worker_threads` | `determinism.rs`: `stacks_digest_is_identical_across_thread_counts`, `chunk_digest_is_identical_across_thread_counts`, `walker_digest_is_identical_across_thread_counts`, `vehicle_digest_is_identical_across_thread_counts`, `fleet_digest_is_identical_across_thread_counts`, `fleet_digest_detects_a_changed_input`, `ragdoll_pile_digest_is_identical_across_thread_counts`, `permuted_insertion_order_fails_the_gate`; `vehicle.rs`: `drive_a_route_twice_gives_identical_bits`; `crates/oxijolt-sys/tests/determinism.rs`: `digest_is_identical_across_thread_counts` |
| Debug wireframe as line data (`debug-renderer`) | `PhysicsWorld::debug_lines`, `DebugLines`, `DebugLineSettings` | `debug_lines.rs`: `near_colliders_present_far_absent_terrain_present`, `line_cap_gives_exactly_the_cap_and_truncated`, `hidden_groups_vanish`, `compound_child_pose_applies_child_rotation_before_body_rotation`; `debug_lines_leaks.rs`; `crates/oxijolt-sys/tests/debug_renderer_bindings.rs` |
| Leak gate for per-call joltc objects (Windows; bounded memory growth, catches only leaks above its threshold) | queries, rebase | `leaks.rs`: `per_call_joltc_objects_do_not_leak` |
| Character settings with Jolt's defaults, validation | `PhysicsWorld::create_character`, `CharacterSettings`, `CharacterError` | `character.rs`: `invalid_settings_and_poses_are_rejected_without_side_effects`; `src/character.rs`: `default_settings_match_jolt` |
| Character ids per world, removal | `CharacterId`, `character_ids`, `remove_character` | `character.rs`: `ids_count_from_one_are_never_reused_and_belong_to_their_world`, `removing_a_character_mid_run_leaves_the_others_sound` |
| Character update: move by velocity, ground state and normal | `update_character`, `CharacterMut::set_linear_velocity`, `CharacterRef::ground_state`, `ground_normal`, `ground_body`, `GroundState` | `character.rs`: `a_character_lands_on_a_floor_and_reports_it`, `linear_velocity_round_trips_and_moves_a_free_character`, `a_slope_limit_near_zero_turns_the_limit_off`; `src/character.rs`: `ground_states_convert_and_report_support` |
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
| Wheel collision testers and contact readout | `VehicleCollisionTester` (ray, sphere, cylinder), `VehicleRef::wheels`, `WheelState`, `WheelContact`, `VehicleMut::set_collision_tester` | `vehicle.rs`: `wheel_contacts_match_the_ground_geometry` (every tester on a box floor and a heightfield, against the analytic plane), `start_overlap_reports_a_hard_point_without_depth`, `start_overlap_depth_comes_from_collide_shape`, `collision_tester_can_be_replaced` |
| Driver input, engine and gear readout | `DriverInput`, `VehicleMut::set_driver_input`, `VehicleRef::engine_rpm`, `current_gear` | `vehicle.rs`: `driver_input_drives_steers_and_brakes`, `braking_at_the_smallest_step_stays_finite` |
| Caller-applied gravity, sleep | `VehicleMut::set_gravity`, `VehicleRef::gravity`, `world_up` | `vehicle.rs`: `gravity_override_replaces_world_gravity`, `gravity_whose_force_overflows_is_rejected`, `sleeping_chassis_gets_no_override_force`, `never_sleeping_chassis_stays_awake` |
| A car drives a route over heightfield terrain | built on the rows above (route and controller in `tests/common/vehicle.rs`) | `vehicle.rs`: `drive_a_route_over_terrain` (four waypoints, never tipped past 60°, two wheels down on 95 % of the ticks), `drive_a_route_twice_gives_identical_bits` |
| Leak gate for vehicles (Windows) | create, drive, read, replace the tester, remove | `vehicle_leaks.rs`: `vehicles_do_not_leak` |
| Constraint settings for ragdoll joints, validation | `SwingTwistConstraintSettings`, `HingeConstraintSettings`, `SixDofConstraintSettings`, `MotorSettings`, `SpringSettings`, `SwingType`, `ConstraintSpace` | `src/constraint.rs`: `swing_twist_defaults_are_jolts`, `hinge_defaults_are_jolts`, `six_dof_defaults_are_jolts`, `fixed_and_free_map_to_jolts_sentinels`, `frames_are_validated`, `swing_twist_limits_are_validated`, `hinge_limits_are_validated`, `six_dof_limits_are_validated`, `motors_and_springs_are_validated`; `crates/oxijolt-sys/tests/ragdoll_smoke.rs`: `ragdoll_parts_and_constraints_reach_jolt`. Frames in `ConstraintSpace::LocalToBodyCom` are not tested |
| Skeleton, ragdoll settings, stabilized masses | `Skeleton`, `SkeletonJoint`, `RagdollSettings::new`, `new_stabilized`, `RagdollPart`, `RagdollJoint` | `src/ragdoll/settings.rs`: `skeletons_are_validated`, `ragdoll_settings_are_validated`, `heightfield_parts_are_rejected`, `stabilized_settings_build`; `ragdoll.rs`: `plain_settings_keep_the_masses_and_stabilized_ones_the_total`, `one_settings_value_serves_two_worlds` |
| Ragdoll creation, ids, removal | `PhysicsWorld::create_ragdoll`, `remove_ragdoll`, `RagdollId`, `ragdoll_ids`, `ragdoll_of_body`, `RagdollError`, `BodyError::OwnedByRagdoll` | `ragdoll.rs`: `ragdoll_parts_and_ids_are_guarded` (including a world too small for the parts), `removing_a_ragdoll_drops_what_rests_on_it`, `dropping_a_world_with_ragdolls_is_clean`; `src/ragdoll/mod.rs`: `a_created_chain_reports_its_parts_and_joints` |
| No collisions between a ragdoll's own parts | `RagdollSettings` | `ragdoll.rs`: `two_ragdolls_collide_with_each_other_but_not_with_themselves`, `the_same_parts_as_plain_bodies_collide` (the control), `humanoid_settles_on_a_heightfield_in_a_second_world` (every part pair on every tick while the ragdoll is awake) |
| Pose readout and writing, kinematic drive | `RagdollRef::pose`, `root_transform`, `RagdollMut::set_pose`, `drive_to_pose_using_kinematics`, `set_motion_type`, `SkeletonPose` | `ragdoll.rs`: `poses_read_back_in_world_transforms`, `kinematic_parts_reach_the_driven_pose`; `src/ragdoll/settings.rs`: `poses_are_validated` |
| Motor drive and joint readings | `RagdollMut::drive_to_pose_using_motors`, `stop_motors`, `RagdollRef::joint`, `joint_motors_on`, `JointReading` | `ragdoll.rs`: `motors_drive_each_joint_kind_to_its_target` (a hinge elbow and knee, a six-DOF hip and a swing-twist neck, driven from the bind pose), `rebase_moves_a_settled_ragdoll_rigidly` (joint readings before and after a rebase), `humanoid_settles_on_a_heightfield_in_a_second_world` (the limit check reads every joint) |
| Settle detection | `SettleDetector`, `RagdollRef::is_calm` | `src/ragdoll/settle.rs`: `calm_updates_in_a_row_settle`, `default_is_the_rest_rule`, `invalid_limits_are_rejected`; `ragdoll.rs`: `humanoid_settles_on_a_heightfield_in_a_second_world` |
| A 12-capsule humanoid settles on a heightfield in a second world (fixture in `tests/common/ragdoll.rs`) | built on the rows above | `ragdoll.rs`: `humanoid_settles_on_a_heightfield_in_a_second_world` (caller-applied gravity, the main world stepping on another thread, settled within 600 ticks (measured: tick 141), joints within 0.01 rad of their limits at rest), `humanoid_joints_overshoot_their_limits_only_boundedly_during_the_drop` (below 0.40 rad on every tick of that drop), `an_environment_ray_passes_through_a_resting_part` |
| Leak gate for ragdolls (Windows) | settings, create, drive, read, remove | `ragdoll_leaks.rs`: `ragdolls_do_not_leak` |
| Raw bindings | `oxijolt-sys` | `crates/oxijolt-sys/tests/smoke_test.rs`: `box_falls_onto_static_box`; `crates/oxijolt-sys/tests/character_smoke.rs` (the extension's character functions): `character_lands_and_its_restored_state_continues_bit_for_bit`, `copying_no_bytes_accepts_a_null_buffer`; `crates/oxijolt-sys/tests/vehicle_smoke.rs`: `vehicle_constraint_steps_and_tears_down`; `crates/oxijolt-sys/tests/ragdoll_smoke.rs`: `ragdoll_parts_and_constraints_reach_jolt`; layout checks at compile time in `crates/oxijolt-sys/src/layout.rs` |
| Guide examples (a world; a walking character; a car and a ragdoll) | [docs/guide.md](docs/guide.md) | doctests of `Guide` in `crates/oxijolt/src/lib.rs` (`cargo test -p oxijolt --doc`) |

CI runs the test suite in four configurations: default, `cross-platform-deterministic`,
`double-precision` and `debug-renderer`. The tests behind the `debug-renderer` feature run only in
the last one.

Not in the safe API yet (joltc exposes them, so `oxijolt-sys` has them): mesh and convex hull
shapes, constraints between arbitrary bodies (the swing-twist, hinge and six-DOF settings serve
only as ragdoll joints) and the other constraint kinds, tracked vehicles, motorcycles and the
manual transmission, the skeleton mapper, soft bodies, Jolt's rigid-body `Character`, and
contact, activation and character contact listeners. Saving and restoring the state of bodies is
in neither layer yet; only a character's state can be saved.

## Guarantees and limits
- **Validation.** Values Jolt only checks with debug assertions (non-finite poses, non-unit
  quaternions, zero dimensions, ids of another world, unknown layers) are rejected with a typed
  error before they reach Jolt. Validation checks each input value, not what a step later
  computes from it: an accepted but extreme value can still overflow inside a step (see the
  vehicle and ragdoll limits below). The rule is that an `Err` means the call changed nothing; a step
  that ran but dropped contacts returns `Ok` with a flag in its `StepReport`.
- **Ids.** A `BodyId` is Jolt's index and 8-bit sequence number. The sequence wraps after 255
  reuses of one index, so a very old id can name a new body. Character, vehicle and ragdoll ids
  count from 1 in each world and are never reused.
- **Time step.** `step` accepts 1 µs (`MIN_DELTA_TIME`) to 1 s (`MAX_DELTA_TIME`). Jolt divides by
  the step (kinematic and character velocities, a wheel's brake-lock torque); the lower bound
  keeps that divisor away from subnormal values, where these quotients become infinite. A huge
  numerator can still overflow a quotient, and the bound does not limit every force or velocity
  a step can produce.
- **Queries.** Ray casts see a box's sharp faces whatever its convex radius; contacts and shape
  casts use at most 0.05 m of a convex radius. Heightfields cannot be query shapes, and only
  spheres and capsules take a shape-cast target distance. `collide_shape` hits come in no
  particular order.
- **Characters.** A new character reports `InAir` until its first update or
  `refresh_character_contacts`. Jolt's walk stairs and steep-slope test judge an obstacle by the
  surface normal at the contact. On a box with sharp edges (convex radius 0) the contact sits on
  the top edge, and whether Jolt reports the top face's normal there or the side's is decided by
  float rounding, so walk stairs on a sharp step is unreliable: it climbs some step heights and
  refuses others, differently from scene to scene. A capsule also creeps onto low sharp edges by
  itself (up to about 0.37 m in the walker tests, measured on a sharp dynamic box).
  The [guide](docs/guide.md#sharp-steps-and-the-games-own-autostep) shows the autostep a game builds
  instead. With walk stairs on, the climbable height is not `walk_stairs_step_up` but about it
  plus the padding plus `r (1 - cos max_slope_angle)` for a capsule of radius `r`, so measure it.
  A `max_slope_angle` below about 0.81° turns Jolt's slope limit off. After a rotating rebase,
  refresh each character's contacts before its next update. The game's controller
  (`tests/common/walker.rs`) is test support, not API.
- **Vehicles.** Only the wheeled controller with the automatic transmission is bound. Settings
  are checked against the values Jolt asserts on or divides by, and the coefficients Jolt forms
  from them are checked at the time step bound that is their worst case. What a step computes
  from the vehicle's state afterwards is not bounded: a huge but finite gravity or velocity can
  still overflow inside the step, where Jolt clamps it (with `oxijolt-sys/asserts` its
  assertion fires instead). A gravity override whose force (gravity times the chassis mass)
  would overflow is refused, also when a rotating rebase would produce it. Wheel contacts are
  those of the last step, found at the chassis pose before that step moved it. A wheel whose cast
  starts inside a solid body reports suspension length 0 and `hit_hard_point`, never a depth. Measured on a tilted chassis and on sloped ground, ray and
  sphere testers match the analytic contact within 1e-6 m; the cylinder's suspension length
  matches within 6e-5 m, but its contact point may sit about 2 mm along the rim. Testers apply
  no compound-child filter: give the wheels their own object layer.
- **Ragdolls.** Parts of one ragdoll never collide with each other; different ragdolls do.
  Joints do not stay within their limits on every tick: in each solver iteration Jolt solves
  contacts after constraints, so on impact the contacts win and joints pass their limits for a
  few dozen ticks, and a small error can remain at rest. Check limits at rest, with a tolerance. These numbers are measurements of the tests' 12-part humanoid, not guarantees: on the
  drop the tests run (pelvis 1.5 m above a heightfield) the worst overshoot is 0.29 rad and
  0.0037 rad remain at rest, and the tests bound them at 0.40 rad and 0.01 rad for that drop
  only. Over a sweep of 126 drops from 1 to 2.5 m, 4 overshot by more than 0.40 rad (up to
  0.48 rad), 40 were more than 0.01 rad outside a limit when they came to rest (up to 0.15 rad),
  3 needed more than 600 ticks to settle, and hinges bent about their fixed axes by up to
  0.57 rad on impact. In that sweep the centre of a thin limb dropped from 2.5 m ended up to
  0.15 m below the terrain surface with `MotionQuality::Discrete`; use `LinearCast` on limbs for
  high falls. Motors drive only while the parts are awake, so drive them every tick. Spring values
  are checked for finiteness and sign, not size: an absurd motor spring frequency (about 1e19 Hz
  and up) overflows inside Jolt. `RagdollSettings::new_stabilized` changes the parts' masses (Jolt's
  `Stabilize` bounds every parent/child mass ratio to [0.8, 1.2]).
- **Debug lines.** No level of detail: one capsule draws 6528 lines and one cylinder 768 at any
  distance, and each call builds a new Jolt debug renderer. Calls are serialized process-wide. A
  character is drawn only through its inner body.
- **Global state.** `oxijolt` calls `JPH_Init` once per process and never `JPH_Shutdown`. It
  installs joltc's filter procs (and with `debug-renderer` its debug renderer procs) once and owns
  them; code that also uses `oxijolt-sys` directly must leave them alone.
- **Platforms.** CI builds and tests Windows MSVC only. The build script has code for other hosts
  and for Android cross builds, but nothing checks them.

## Determinism
The requirement: on one machine, the same binary given the same calls in the same order produces
bit-identical body ids, poses, velocities and sleep flags whatever `WorldSettings::worker_threads`
is (`worker_threads(n)` means n workers plus the thread that calls `step`). This rests on Jolt's own
conditions (same binary, simulation-changing calls in the same order). The tests check it with 1 and
4 workers on the scenes listed under "How it is checked", not for every thread count or every call
sequence. The call history includes the
order of body creation and removal, which decides each `BodyId`'s index and sequence number, as well as
rebases, `optimize_broad_phase`, forces and velocity writes. Characters are covered too: each world
numbers its characters from 1 in creation order (Jolt's default id comes from a process-wide
counter and orders contacts between characters, so the world passes its own), and a character's
contacts come in a deterministic order while `max_hits_exceeded` is false. `CharacterRef::save_state`
and `CharacterMut::restore_state` continue a character bit for bit in a world rebuilt the same
way, which a chained replay of the game's near step checks tick by tick. `remove_body` wakes the
bodies whose exact bounds overlap the removed one, in id order, so it adds no hidden state;
`remove_ragdoll` does the same for each part. Vehicles and ragdolls are covered: vehicles run as
Jolt step listeners, and a fleet of 40 is spread over a different number of listener jobs with 1
and 4 workers; a pile of 16 ragdolls forms islands of 128 or more joints and contacts, and Jolt
splits islands of more than 128 for parallel solving. Each world numbers its vehicles and ragdolls from 1 in creation order.

What is not guaranteed by default: equal results across compilers, compiler flags, operating systems or
CPU architectures. The `cross-platform-deterministic` feature (off by default, roughly 8 % slower) builds
Jolt with `CROSS_PLATFORM_DETERMINISTIC`. Jolt then
[claims](https://github.com/jrouwe/JoltPhysics/blob/v5.6.0/Docs/Architecture.md#deterministic-simulation)
equal results across compilers, configurations, operating systems and architectures, as long as the same
source is built with the same defines: never compare a `double-precision` build with a single-precision
one, and FPU rounding and denormal (DAZ/FTZ) modes must match. This repository's gate runs on one
machine and checks thread-count equality in each configuration; it does not check equality across
machines.

Order caveats: narrow-phase query hits (`collide_shape` and friends) come in an unspecified order; sort
them by body id and sub-shape id when order matters. Code that drives the world must not feed hash-map
iteration order, time or thread identity into its calls.

How it is checked: `cargo test -p oxijolt --test determinism` runs each scene in two child
processes, with 1 and with 4 workers, records ticks 0 to 1000 of a chunk, terrain and item scene and
requires them to match byte for byte; the same bodies created in another order must fail the gate.
A walker scene runs the game's reference controller for 600 ticks and is compared the same way, and
so are a car driving the route over terrain, the fleet of 40 vehicles (whose digest must also
change when one input changes) and the ragdoll pile. CI runs it in all four configurations.

## Building
Requirements: a C++ toolchain (MSVC on Windows), CMake 3.20 or newer, and LLVM/libclang for `bindgen`
(see the [bindgen guide](https://rust-lang.github.io/rust-bindgen/requirements.html)).

```bash
git submodule update --init         # joltc and Jolt
cargo build                         # builds joltc + Jolt through CMake (always Release), generates bindings
cargo test --workspace              # everything, headless
cargo test -p oxijolt --features debug-renderer   # with the debug wireframe
```

The first build compiles joltc and Jolt with CMake, always in Release. Later builds of the same target and profile reuse it; a change to the native sources, the pinned
commits, a feature or `build.rs` reruns CMake. Run one cargo process at a time on a shared machine: the C++ build is heavy. On Windows a
very long `CARGO_TARGET_DIR` can make the CMake configure step fail (path length limit).

### Prebuilt native library
Set `JOLTC_LIB_DIR` to skip CMake entirely and link a native library built earlier. It points at an
install prefix:

```
<prefix>/lib/          joltc (or joltc_double) and Jolt static libraries
<prefix>/include/joltc.h
<prefix>/include/joltc_ext.h
<prefix>/oxijolt-sys-manifest.txt
```

A normal build leaves such a prefix in `target/<profile>/build/oxijolt-sys-*/out/joltc`. A prefix is tied to
the target, the C runtime, the crate features and the pinned joltc and Jolt commits. The build script
checks all of them against the manifest and refuses a prefix built for another configuration. CI
links every build after the first through such a prefix.

## Submodules
joltc and Jolt Physics are Git submodules under `crates/oxijolt-sys/vendor`, pinned to exact commits:

```bash
git submodule update --init
```

Checkouts made before the move to joltc or the crate rename run `git submodule sync && git submodule update --init` once.

## License
Licensed under either of

* Apache License, Version 2.0, ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
* MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution
Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.

[joltc]: https://github.com/amerkoleci/joltc
