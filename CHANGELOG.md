# Changelog

All notable changes to this fork. The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## Unreleased

- Tracked vehicles ([guide](docs/vehicles.md)): `PhysicsWorld::create_tracked_vehicle` with
  `TrackedVehicleSettings`, two `VehicleTrackSettings` that own their `TrackedWheelSettings`, and
  Jolt's tracked engine and transmission defaults. `TrackedDriverInput` sets throttle, brake and a
  speed ratio per track, at least `1/limits::MAX_RATIO` in magnitude; `VehicleRef::tracks`,
  `track`, `track_wheels` (`TrackState`, `TrackSide`) read them back. A track's inertia is within
  `limits::MIN_TRACK_INERTIA..=MAX_TRACK_INERTIA` and its wheel radii within a factor
  `limits::MAX_RATIO` of its driven wheel's, the two tracks' inertias within a factor
  `limits::MAX_TRACK_INERTIA_RATIO` (100) of each other, because Jolt's ground contact diverged
  between very unequal tracks (`docs/limits.md`, Track inertia ratio), and the track speeds the
  drivetrain can reach must keep the tracked step finite (`docs/limits.md`, Track drive envelope).
  `create_tracked_vehicle` refuses tracks heavy against their chassis: every wheel's track inertia
  over its radius squared, times the chassis' largest inverse effective mass at the wheel over the
  wheel's forward/up plane, is at most `limits::MAX_TRACK_MASS_RATIO` (0.25), because Jolt's ground
  contact diverged and spun the chassis up to its angular velocity clamp from a ratio of 6 at steps
  up to 0.1 s (`docs/limits.md`, Track mass ratio). Jolt's `TankTest` hull accepts tracks up to 11 kg·m².
- Motorcycles: `PhysicsWorld::create_motorcycle` with `MotorcycleSettings`, which wraps a
  two-wheeled `VehicleSettings` and adds the lean controller: max lean angle up to
  `MotorcycleSettings::MAX_LEAN_ANGLE` (80°), a lean spring bounded against the chassis' inertia,
  smoothing, and the lean controller and lean steering limit switches, fixed at creation.
  `VehicleRef::lean` (`MotorcycleLean`) reads the target lean and the lean angles. A rotating
  `rebase` rotates each motorcycle's target lean.
- `VehicleError::LeanSpringIntegrationNotSaved`: Jolt does not save a motorcycle's integrated lean
  angle, so a nonzero lean spring integration coefficient is refused.
- joltc extension: `JPH_MotorcycleController_GetTargetLean` and `_SetTargetLean`, which check the
  controller's kind.
- Changed (breaking): an engine's torque curve (`VehicleEngineSettings::normalized_torque`, for
  every vehicle kind) needs x within `0..=1`, at least `limits::MIN_TORQUE_CURVE_SPACING` apart
  up to rounding x to `f32` (points at `i / 1000` pass), and y within
  `0..=limits::MAX_NORMALIZED_TORQUE`. Any finite points with increasing x were accepted before,
  and wide spans overflowed Jolt's interpolation (`docs/limits.md`, Torque curves).
- Changed (breaking): vehicle ids are typed by kind. `VehicleId` is now `VehicleId<K>` with `K`
  `WheeledVehicle` (the default), `TrackedVehicle` or `Motorcycle`, and `VehicleRef` and
  `VehicleMut` take the same parameter. `vehicle_ids` and `vehicle_of_body` return `AnyVehicleId`
  (with `kind` and `downcast`), `remove_vehicle` takes either id, and `VehicleError::NotFound` and
  `WrongWorld` carry an `AnyVehicleId`. An id's `Debug` output names its kind:
  `VehicleId(Wheeled, 1)`.

## 0.5.0 — 2026-10-05

- Shapes: convex hulls (`Shape::new_convex_hull`, `new_convex_hull_with_material`, `HullError`),
  triangle meshes with a material per triangle for static and kinematic bodies (`Shape::new_mesh`,
  `new_mesh_with_settings`, `MeshSettings`, `MeshBuildQuality`, `MeshError`), scaled shapes
  (`Shape::scaled`) and tapered capsules and cylinders, cones among them
  (`Shape::new_tapered_capsule`, `new_tapered_cylinder`). Point clouds, scales and radii that Jolt
  cannot build or collide with reliably are refused with typed errors; anything else Jolt refuses
  comes back as `ShapeError::Rejected` with Jolt's message (`JoltMessage`, cut after a whole word).
- Mesh triangles too small or too thin to collide with are dropped and returned as
  `DroppedTriangles` (indices and area). The rule counts the rounding of the triangle in the space
  of the convex shape it collides with, up to `MeshSettings::max_convex_extent` (default 1100 m,
  below `limits::MAX_SHAPE_EXTENT`; see `docs/limits.md`), and does not depend on the order of a
  triangle's corners. `Shape::scaled` checks a mesh inside it for the extent the mesh was built
  with and refuses a scale that leaves its triangles too thin with `ShapeError::ThinTriangles`
  (`ThinTrianglesError`: the scale and the extent).
- With the `asserts` feature Jolt's hull builder can abort on point clouds with many nearly
  coplanar faces that pass the hull rules: about one densely sampled noisy box in 1000 and 1 to 2 %
  of dense flat cones and domes; without it Jolt refuses most of them as `ShapeError::Rejected`
  and builds the rest. See `docs/limits.md`. The
  `HullError::Coplanar` message suggests thickening the cloud or centring it on the shape origin.
- joltc extension: `JPH_ShapeSettings_CreateShapeWithError`, `JPH_MeshShapeSettings_Create3`
  (materials), `JPH_MeshShapeSettings_GetTriangleCount` and `JPH_Shape_GetTriangles`.
- Body controls ([guide](docs/bodies.md)): impulses (`BodyMut::add_impulse`, `add_angular_impulse`,
  `add_impulse_at_point`), bounded by the new `limits::MAX_VELOCITY_CHANGE` and
  `limits::MAX_ANGULAR_VELOCITY_CHANGE`; kinematic moves (`BodyMut::move_kinematic`); waking and
  sleeping on demand (`BodyMut::activate`, `deactivate`) and `PhysicsWorld::activate_bodies_in_box`,
  which picks bodies by their exact bounds in body-id order.
- Configuration fixed at creation, with getters on `BodyRef`: sensor bodies
  (`BodySettings::sensor`, contacts reported as sensor contacts), per-body user data
  (`BodySettings::user_data`), locked axes (`BodySettings::allowed_dofs`, `AllowedDofs`) and the
  movement capability of static bodies (`BodySettings::allow_dynamic_or_kinematic`).
- Structural changes: `BodyMut::set_motion_type` and `BodyMut::set_shape`. Every successful
  `set_shape` call, also one with the body's current shape, and every change of motion type makes
  earlier `WorldState`s unrestorable; `set_motion_type` with the current motion type changes
  nothing. Both refuse bodies that a character, vehicle, ragdoll or constraint holds. A shape
  change wakes the bodies inside the box enclosing the body's old and new bounds.
- New `BodyError` variants: `RestrictedDofs`, `NotKinematic`, `CannotMove`. Constraints, vehicles,
  ragdoll parts and rotating rebases refuse bodies with fewer than six degrees of freedom; ragdoll
  parts refuse user data, which Jolt overwrites.
- `BodyMut` now borrows its world (the signature of `body_mut` is unchanged).
- Fixed: `BodyMut::add_force_at_point` checked the exact torque, while Jolt's `f32` cross product,
  with a product fused into the subtraction, keeps a rounding error. A force parallel to a long lever
  on a very thin body had an exact torque of 0 and was accepted, and the next step overflowed the
  angular velocity (a Jolt assertion, or an angular velocity of exactly 0 in release). The torque
  rule now counts that rounding (`docs/limits.md`, Impulses), as `add_impulse_at_point` does.

## 0.4.0 — 2026-10-04

First release on crates.io.


- Renamed the crates: `joltc-sys` is now `oxijolt-sys` (`crates/oxijolt-sys`, Rust path `oxijolt_sys`)
  and `rolt` is now `oxijolt` (`crates/oxijolt`); the old names belong to the upstream project on crates.io. Before
  the first release the crates were briefly called `joltphysics-sys` and `joltphysics`. The repository is
  now https://github.com/pockerhead/oxijolt. The prebuilt manifest is now `oxijolt-sys-manifest.txt`;
  `JOLTC_LIB_DIR` and the features are unchanged. Existing checkouts run
  `git submodule sync && git submodule update --init` once.
- Project rules (`AGENTS.md`) and lineage (`LINEAGE.md`) for the fork.
- The build script's Android NDK setup and `cross-platform-deterministic` switch were rewritten with
  the same CMake options; no code from jolt-rust remains, and `LICENSE-MIT` now carries this
  project's copyright line.
- The raw layer moved from JoltC to [joltc](https://github.com/amerkoleci/joltc) over Jolt Physics 5.6.0:
  `oxijolt-sys` now exposes joltc's `JPH_*` API. joltc and Jolt are pinned submodules under
  `crates/oxijolt-sys/vendor`, and the native build no longer fetches anything from the network.
- Layout assertions for the FFI types in use, on the C++ side and the Rust side.
- Eight joltc ragdoll and skeleton-mapper functions that cast 4-aligned matrices to 16-aligned ones are
  left out of the bindings.
- Removed the `object-layer-u32` feature: joltc always uses 32-bit object layers.
- `JOLTC_LIB_DIR` links a prebuilt native library and skips CMake; the prefix is validated against a
  manifest. Rust-only changes no longer rerun CMake.
- `oxijolt` is a new safe API on the joltc raw layer: a physics world with collision layers, box and
  sphere shapes and rigid bodies, with a headless `hello_world` example.
- CI on GitHub Actions (Windows MSVC: build, test, clippy, docs, formatting) with a cached native build.
- `PhysicsWorld::step` returns `Result<StepReport, StepError>`. `Err` now means the step was rejected and
  the world did not advance (`StepError::InvalidDeltaTime`); a step that ran but dropped contacts because
  a fixed-size Jolt buffer was full returns `Ok` with the matching `StepReport` flag set
  (`StepReport::is_complete` is false). `StepError::CacheFull` is gone. The rule for the safe API: `Err`
  means nothing happened; anything that happened but was degraded is reported in the `Ok` value.
- Bodies: kinematic bodies, mass override, continuous collision (`MotionQuality::LinearCast`),
  gravity factor, enhanced internal edge removal, forces at a point and torques, `reset_forces`,
  and sleep and active readout. Invalid values are rejected with typed errors before they reach Jolt.
- `WorldSettings::worker_threads` accepts 1 to `WorldSettings::MAX_WORKER_THREADS` (64).
  `PhysicsWorld` is `Send` and `Sync`: many threads may read one world, and separate worlds step in
  parallel.
- `PhysicsWorld::remove_body` wakes the non-static bodies whose bounds overlap the removed one, in
  body-id order, so a stack whose bottom is removed falls.
- Shapes: Y-cylinders and Y-capsules, boxes and cylinders with a chosen convex radius (0 for sharp
  edges), heightfields (`Shape::new_height_field`, `HeightFieldSettings`: holes, block size, bits per
  sample, active-edge threshold) and compounds whose children carry their own pose and user data
  (`Shape::new_compound`, `CompoundChild`). One shape serves many bodies in many worlds.
- Scene queries on `&PhysicsWorld`: `cast_ray`, `cast_shape` (with target distance, start
  penetration and deepest point) and `collide_shape`, all filtered by `QueryFilter` (object layers,
  compound child groups, one excluded body). Hits report the outward normal of the obstacle and the
  compound child that was hit. `optimize_broad_phase` makes queries fast after many single inserts.
- `PhysicsWorld::rebase` moves the whole world into a new frame (floating origin) without waking or
  putting to sleep any body.
- A determinism gate: scenes run in two processes with 1 and 4 worker threads must match bit for
  bit for 1000 ticks, and another creation order must not. CI runs it, and the whole suite, in the
  default, `cross-platform-deterministic` and `double-precision` configurations.
- `debug-renderer` feature: `PhysicsWorld::debug_lines` returns the wireframe of the colliders around
  a point as line data, with layer and group filters and a line cap. Jolt's debug renderer is left
  out of the native libraries without the feature.
- A guide (`docs/guide.md`) with a headless example of a terrain, a chunk compound, an item, queries
  and a rebase, run as a doctest.
- `oxijolt-sys` gains a C extension in joltc's naming (`native/joltc_ext`): a state recorder,
  `JPH_CharacterVirtual_SaveState` and `_RestoreState`, and `JPH_CharacterVirtual_ExtendedUpdate2` and
  `_RefreshContacts2`, which take explicit gravity, filters and temp allocator.
- Virtual characters (Jolt `CharacterVirtual`) owned by the world: `PhysicsWorld::create_character`,
  `update_character` (Jolt `ExtendedUpdate`: move by the velocity, stick to floor, walk stairs),
  `refresh_character_contacts` and `remove_character`, configured by `CharacterSettings` and
  `ExtendedUpdateSettings` with Jolt's defaults; invalid values are rejected with `CharacterError`.
- Characters take up and rotation before every update, so they walk on a spherical planet with
  radial up.
- Character readout: ground state, normal, position, velocity and body, the compound child stood on,
  and the active contacts with their body or character, layer, compound child group and normals.
  Updates and refreshes take the same `QueryFilter` as queries.
- A character can carry a kinematic inner body that bodies and queries see; `remove_body` refuses it
  with `BodyError::OwnedByCharacter`, and it goes with its character.
- Characters can collide with each other (`CharacterSettings::collide_with_characters`).
- Each world numbers its characters from 1 in creation order (`CharacterId`), so contacts between
  characters come in the same order in every run.
- `CharacterRef::save_state` and `CharacterMut::restore_state` continue a character bit for bit in a
  world rebuilt the same way, for chained replays.
- `PhysicsWorld::rebase` moves characters with the bodies.
- The determinism gate also runs the game's reference walker for 600 ticks with 1 and 4 worker
  threads, and a leak gate covers characters.
- The guide has a section on characters, with a character walking on a small planet run as a
  doctest, and explains how a game builds its own autostep for steps with sharp edges.
- Benchmarks against the game's budgets (`cargo bench -p oxijolt --bench budgets`, results in
  `docs/benchmarks.md`); the `character_cost` example moved into this bench.
- `Shape::new_offset_center_of_mass` (Jolt `OffsetCenterOfMassShape`) moves a shape's centre of
  mass without moving its surface, for a vehicle chassis with a low centre of mass.
- Wheeled vehicles (Jolt `VehicleConstraint` with the wheeled controller) on a dynamic chassis
  body: `PhysicsWorld::create_vehicle`, `remove_vehicle`, `vehicle`, `vehicle_mut`, configured by
  `VehicleSettings` with wheels, suspension, engine, automatic transmission, differentials and
  anti-roll bars at Jolt's defaults, and validated with `VehicleError`.
- Vehicle wheels find the ground with a ray, sphere or cylinder cast (`VehicleCollisionTester`)
  on their own object layer, and report contact, suspension length and impulses (`WheelState`).
- Vehicles take driver input, a gravity override for gravity the caller applies, and a pitch and
  roll limit; they report engine rpm and gear. An override whose force would overflow is refused.
- `remove_body` refuses a vehicle's chassis (`BodyError::UsedByVehicle`), and dropping a world
  removes its vehicles and ragdolls first.
- `PhysicsWorld::rebase` rotates each vehicle's gravity override and the up of its ray or sphere
  tester.
- `PhysicsWorld::step`, `update_character` and the kinematic ragdoll drive reject time steps below
  `PhysicsWorld::MIN_DELTA_TIME` (1 µs). Jolt divides by the step; the bound keeps the divisor
  away from subnormal values, where those quotients become infinite.
- `BodySettings::linear_damping` and `angular_damping` (Jolt's default 0.05), and `BodyRef::mass`
  for dynamic bodies, for callers that apply gravity as a force.
- `PhysicsWorld::were_bodies_in_contact` reports whether two bodies touched in the last step.
- Constraint settings for ragdoll joints with Jolt's defaults and validation:
  `SwingTwistConstraintSettings`, `HingeConstraintSettings` with limits and
  `SixDofConstraintSettings` with asymmetric pyramid limits, with `MotorSettings` and
  `SpringSettings`.
- Ragdolls (Jolt `Ragdoll`): `Skeleton`, `RagdollSettings` (and `new_stabilized` for Jolt's
  `Stabilize`), `PhysicsWorld::create_ragdoll` and `remove_ragdoll`. Parts of one ragdoll never
  collide with each other; different ragdolls do. `remove_body` refuses a part
  (`BodyError::OwnedByRagdoll`).
- Ragdolls report their pose, root transform and joint readings, take a pose, are driven to a
  pose with motors (six-DOF joints included) or kinematically, switch motion type and take
  velocities; `SettleDetector` tells when a ragdoll has come to rest.
- Determinism gates for a car driving a route, a fleet of 40 vehicles and a pile of 16 ragdolls
  (1 against 4 worker threads), and leak gates for vehicles and ragdolls.
- `oxijolt-sys` extension: `JPH_VehicleConstraint_AsConstraint`, ragdoll part and joint
  setters that keep every setting (`JPH_RagdollSettings_SetPart`,
  `_SetPartToParentSwingTwist`, `_SetPartToParentHinge`, `_SetPartToParentSixDOF`),
  `JPH_RagdollSettings_CalculateConstraintPriorities`, and swing-twist and hinge motor access.
  Layout assertions now cover the vehicle and constraint settings and every `JPH_PhysicsSettings`
  field.
- The guide has sections on vehicles and ragdolls, with a car on terrain and a ragdoll in a
  second world run as a doctest; the README and the ragdoll docs state how far joints pass their
  limits on impact.
- `PhysicsWorld::step` also rejects time steps above `PhysicsWorld::MAX_DELTA_TIME` (1 s), a guard
  against overflow-scale steps; it is not a Jolt limit.
- A magnitude policy for the safe API: the public `oxijolt::limits` module holds the bounds
  (positions within `MAX_POSITION`, shape extents, velocities, accelerations, masses, friction,
  spring coefficients, ratios, ...), and the setters of those inputs check them before they reach
  Jolt. Some inputs, such as damping and ray directions, are only checked to be finite.
  `docs/limits.md` shows how the bounds follow from Jolt's arithmetic; `docs/coverage.md` says
  which values only tests check and which no bound covers. New checks refuse values that were
  accepted before, among them `WorldSettings::max_contact_constraints` above
  `WorldSettings::MAX_CONTACT_CONSTRAINTS`, body friction above `limits::MAX_FRICTION`, a
  character weight impulse above `limits::MAX_WEIGHT_IMPULSE` and six-DOF translation limits
  beyond `limits::MAX_SHAPE_EXTENT`.
- The `asserts` feature of `oxijolt` (forwarded to `oxijolt-sys/asserts`), and Jolt's assertion
  handler: oxijolt installs it before `JPH_Init`; with `asserts` a failed assertion prints its
  expression, message, file and line and aborts the process, except the physics-update-error
  assertion, whose condition `step` reports in its `StepReport`. CI runs the whole test suite with
  `asserts` as a fifth configuration.
- Behaviour change: the rigid-body inertia floor. A dynamic or kinematic body (also a character's
  inner body and a ragdoll part) whose inertia tensor is not exactly diagonal (a rotated compound
  child, an offset centre of mass) is refused with `InvalidValue` unless its smallest principal
  moment is at least about 4.8e-4 of the tensor's norm, the rule soft bodies use. The earlier rule
  accepted tensors on which Jolt's eigen decomposition asserted (`docs/limits.md`, "Rigid body
  inertia"). In a rotated child this refuses a square needle box
  past about 54 times longer than wide and a capsule or cylinder past about 47 times longer than
  its diameter. Shapes with an exactly diagonal tensor, such as unrotated primitives, are not
  affected.
- World constraints of twelve kinds: `PhysicsWorld::create_constraint` with
  `FixedConstraintSettings`, `PointConstraintSettings`, `DistanceConstraintSettings`,
  `HingeConstraintSettings`, `SliderConstraintSettings`, `ConeConstraintSettings`,
  `SwingTwistConstraintSettings`, `SixDofConstraintSettings`, `GearConstraintSettings`,
  `RackAndPinionConstraintSettings`, `PulleyConstraintSettings` and `PathConstraintSettings`
  (along a `HermitePath`). The typed `ConstraintId<K>` selects the motor, target, limit and
  readout methods of `ConstraintRef` and `ConstraintMut`; `remove_constraint`, `constraint_ids`
  and `constraints_of_body` manage them, and `ConstraintError` reports refusals.
- `remove_body` refuses a body a constraint uses (`BodyError::UsedByConstraint`), and a hinge or
  slider a gear or rack references cannot be removed before the coupling. Gears, racks and
  pulleys need two dynamic bodies. Gear ratios lie within `1..=limits::MAX_GEAR_RATIO` (10),
  because of a defect in Jolt 5.6's gear solver. Every point where a constraint holds a dynamic
  body has a lever-arm ratio of at most `limits::MAX_LEVER_ARM_RATIO`. Creating,
  changing or removing a constraint wakes its bodies. `rebase` recreates pulleys in the new frame.
- Caller job systems: the `JobSystem` trait and `Job` run a world's jobs on the caller's thread
  pool, such as Rayon (`WorldSettings::job_system`, `WorldSettings::MAX_CONCURRENCY`). A job run
  inside `queue_job` is left to the stepping thread, a step finishes even when the pool runs a
  job late or never, and a panic in `queue_job` is resumed from `step` after the world advanced.
  The determinism gates also run with a Rayon pool and an inline job system.
- World state save and restore: `PhysicsWorld::save_state`, `save_state_of` and `restore_state`
  with an opaque `WorldState`, for rollback and replays. A state restores only into the world
  that saved it while its structure is unchanged (`StateError`); configuration Jolt does not save
  stays as it is (`docs/state.md`).
- Soft bodies: `SoftBodySharedSettings` (built and checked by `SoftBodySharedSettingsBuilder`,
  with generated or explicit edge, bend and volume constraints), `SoftBodySettings`,
  `PhysicsWorld::create_soft_body`, vertex readout (`soft_body`) and writes (`soft_body_mut`:
  velocities, pinning, kinematic moves). Body-level velocity, torque and point-force setters,
  constraints and vehicles refuse soft bodies (`BodyError::SoftBody`); vehicle wheels look through
  them. Mass, inertia and pressure rules are checked at creation, and vertex masses and forces
  again in the vertex setters; they refuse some thin or far-from-origin bodies Jolt would simulate
  (`docs/soft-bodies.md`). A pressurised body whose volume shrinks later is not covered
  (`docs/coverage.md`, "Not covered").
- Events: `PhysicsWorld::set_event_settings` and `take_events` record contact, body activation
  and soft body contact events (`EventSettings`, `WorldEvents`), sorted per step into an order
  that does not depend on the thread count. A `ContactListener` (`set_contact_listener`) may change
  each contact's `ContactSettings`; settings that do not fit their contact are not applied and are
  reported in `WorldEvents::rejected_contact_settings` and the new
  `StepReport::rejected_contact_settings`. Panics in callbacks are resumed by `step` or
  `take_events`.
- `PhysicsMaterial` carries the caller's user data; `Shape::new_box_with_material`,
  `new_sphere_with_material`, `new_capsule_with_material`, `new_cylinder_with_material` and
  `new_height_field_with_materials` make shapes of materials, and contacts report each side's
  material.
- `oxijolt-sys` extension: `JPH_PhysicsSystem_SaveState` and `_RestoreState` with a body
  selection, path, pulley and rack-and-pinion constraints and motor accessors, soft body
  functions, vehicle collision testers that skip soft bodies, materials with user data
  (`JPH_PhysicsMaterial_Create2`, `JPH_ConvexShapeSettings_SetMaterial`,
  `JPH_HeightFieldShapeSettings_Create2`), a soft body contact listener and sub-shape pair
  getters.
- Builds without LLVM: the raw bindings are committed under `crates/oxijolt-sys/src/bindings/`,
  one file per ABI family (`msvc`, `gnu`) and configuration, and `cargo xtask bindings`
  regenerates them. The new `bindgen` feature (forwarded by `oxijolt`) generates them at build time
  instead. The supported targets are the 64-bit ones in `build/targets.rs`; 32-bit targets,
  including the Android armv7 and x86 ones accepted before, are refused when the build starts.
  Raw API changes: the bindings no longer carry joltc's header comments, and on `gnu` targets
  joltc's enum typedefs are `c_uint`.
- CI runs on Linux (GCC) as well as Windows (MSVC), and a `Committed bindings` job checks the
  bindings against LLVM 18. A release workflow builds prebuilt native libraries for
  `JOLTC_LIB_DIR` (x86_64 Windows MSVC and Linux GNU, eight feature subsets each) and attaches
  them to a tagged release. A prebuilt prefix's headers are compared without regard to line
  endings.
- A readability pass with no public API change: large modules are split by concern, the limits
  derivations and the per-input audit table moved to `docs/limits.md` and `docs/coverage.md`
  (both new), and 18 validation error messages are shorter; callers that compare exact
  `InvalidValue` texts see the new ones.
- Documentation: the README is a front page; the per-feature test table moved to
  `docs/coverage.md`; new guides for constraints, soft bodies, events, state save and restore,
  caller job systems, determinism and building (`docs/`), whose examples run as doctests.
- `CharacterRef::penetration_recovery_speed` and `CharacterMut::set_penetration_recovery_speed`
  read and set a character's penetration recovery speed while it runs (finite, `0..=1`).
- A study of which character laws CharacterVirtual's built-in mechanisms carry
  ([docs/character-study.md](docs/character-study.md)): law scenarios and pinned results in
  `tests/character_study.rs`, and a bench of every configuration's cost per move
  (`cargo bench -p oxijolt --bench character_study`).
- `PhysicsWorld::active_body_poses` and `active_body_poses_into` return a `BodyPose` (id, position,
  rotation) for every awake body, sorted by `BodyId` and read under one multi-body read lock.
- `oxijolt::error::Error` wraps every area error with a `From` impl, and
  `oxijolt::error::Result<T>` is `Result<T, oxijolt::error::Error>`, so `?` works across areas.
  Both live only in the now public `error` module, so `use oxijolt::*` gains no `Error` or
  `Result`. Existing signatures are unchanged.
- `oxijolt::prelude` holds the commonly used types without the math types, `Error` or `Result`, so
  it can be glob-imported next to `bevy::prelude::*`; the math types are in `oxijolt::prelude::math`.
