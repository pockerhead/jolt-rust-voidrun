# Features

What the safe API, `oxijolt`, covers today. The [roadmap](../README.md#roadmap) in the README lists
what is planned next.

- Rigid bodies: static, kinematic and dynamic; box, sphere, cylinder, capsule, tapered capsule and
  cylinder (cones too), convex hull, triangle mesh (with a material per triangle and a report of
  the triangles too thin to collide with, for static and kinematic bodies), heightfield, plane
  (static ground cut to a square), compound and scaled shapes, compound edits at run time
  (`MutableCompound`); physics materials with user data;
  forces, sleeping, continuous collision; impulses, kinematic moves, waking and sleeping on demand,
  sensors, per-body user data, locked axes (allowed degrees of freedom), shape and motion type
  changes, contact-cache invalidation, and buoyancy and drag in a fluid ([guide](bodies.md)).
- Collision groups: tables of sub groups that say which pairs collide, given to rigid and soft
  bodies at creation, for ragdolls, chains and vehicles that ignore their driver
  ([guide](bodies.md#collision-groups)).
- Scene queries: ray casts, shape casts, collide-shape and point queries (in the world and on a
  shape), filtered by layer, compound child group and body.
- A character controller (Jolt's `CharacterVirtual`) with stair walking, floor sticking and an up
  direction that can change every update, for walking on a planet, and a character contact
  listener for moving platforms, ignored contacts, push settings and added, persisted and removed
  contacts ([guide](events.md#character-contacts-charactercontactlistener)); a humanoid preset,
  `CharacterSettings::humanoid(height, radius)`.
- Wheeled vehicles with suspension, engine, automatic transmission, differentials and anti-roll
  bars; tracked vehicles (tanks) with two tracks; motorcycles with a lean controller
  ([guide](vehicles.md)); presets for a car and a motorcycle (`VehicleSettings::car`,
  `MotorcycleSettings::bike`), and each wheel's pose in world space for drawing
  (`VehicleRef::wheel_world_transform`).
- Ragdolls from a skeleton, posed, motor-driven or kinematic, with settle detection, and a skeleton
  mapper to and from a detailed animation skeleton ([guide](guide.md#ragdolls)).
- Twelve kinds of constraints with motors, springs and limits: fixed, point, distance, hinge,
  slider, cone, swing-twist, six-DOF, gear, rack and pinion, pulley and path
  ([guide](constraints.md)).
- Soft bodies: cloth, pressurised and volume-preserving bodies, with vertex readout and pinning
  ([guide](soft-bodies.md)).
- Contact, activation and soft body contact events in an order that does not depend on the
  thread count, and a contact listener that changes friction, restitution, mass scales or surface
  velocity per contact, or rejects contacts before they form (one-way platforms), and Jolt's
  estimate of each new contact's impulses for impact sounds and damage ([guide](events.md)).
- Saving and restoring a world's state for rollback and replays ([guide](state.md)).
- A floating origin (`PhysicsWorld::rebase`) and optional `f64` world positions.
- Jolt's jobs on Jolt's thread pool or on your own, such as Rayon ([guide](job-system.md)).
- Debug wireframes as line data (feature `debug-renderer`); nothing is drawn.
- Optional `glam` and `mint` features with exact conversions for `Vec3`, `RVec3` and `Quat`.
- The poses of every awake body in one call, in a deterministic order
  (`PhysicsWorld::active_body_poses`).
- `oxijolt::prelude` for the common types, and `oxijolt::error::{Error, Result}` that wraps the
  error of every area.

The [playground](playground.md) shows them in ten small scenes with a window, and runs the same
scenes headless.

The raw layer, `oxijolt-sys`, has the joltc functions for all of them.
[coverage.md](coverage.md) lists every bound feature with the tests that check it, and
the rest of what is not bound yet.
