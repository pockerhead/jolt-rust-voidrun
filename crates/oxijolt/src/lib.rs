//! Safe Rust API for [Jolt Physics](https://github.com/jrouwe/JoltPhysics) over the
//! [joltc](https://github.com/amerkoleci/joltc) raw layer in
//! [`oxijolt-sys`](https://github.com/pockerhead/oxijolt/tree/main/crates/oxijolt-sys).
//!
//! A [`PhysicsWorld`] owns a Jolt physics system with its collision layers
//! ([`CollisionLayers`]), its own job system and temp allocator. Bodies are created from a
//! [`Shape`] and [`BodySettings`] and named by a [`BodyId`]. Shapes are boxes (with a
//! configurable convex radius), spheres, Y-cylinders, Y-capsules, heightfields, compounds
//! whose children carry their own pose and user data, and shapes with a moved centre of mass;
//! one shape may serve many bodies in many worlds. Primitives and heightfields can be made of
//! [`PhysicsMaterial`]s, which carry the caller's user data.
//!
//! Scene queries run on `&PhysicsWorld`: [`PhysicsWorld::cast_ray`] finds the closest body
//! along a ray, [`PhysicsWorld::cast_shape`] the first obstacle a moving shape hits, and
//! [`PhysicsWorld::collide_shape`] every obstacle a shape at a pose overlaps. Each takes a
//! [`QueryFilter`] that selects object layers and compound-child groups and can skip one body.
//! Every normal they report is the outward surface normal of the obstacle.
//!
//! ```
//! use oxijolt::*;
//!
//! # fn main() -> Result<(), Box<dyn std::error::Error>> {
//! let mut world = PhysicsWorld::new(WorldSettings::default())?;
//!
//! let floor_shape = Shape::new_box(Vec3::new(100.0, 1.0, 100.0))?;
//! let floor = BodySettings::new_static().position(RVec3::new(0.0, -1.0, 0.0));
//! world.create_body(&floor_shape, &floor)?;
//!
//! let ball_shape = Shape::new_sphere(0.5)?;
//! let ball = world.create_body(
//!     &ball_shape,
//!     &BodySettings::new_dynamic().position(RVec3::new(0.0, 2.0, 0.0)),
//! )?;
//!
//! for _ in 0..60 {
//!     let report = world.step(1.0 / 60.0)?;
//!     assert!(report.is_complete());
//! }
//! assert!(world.body(ball)?.position().y < 2.0);
//! # Ok(())
//! # }
//! ```
//!
//! # Characters
//! [`PhysicsWorld::create_character`] adds a virtual character (Jolt `CharacterVirtual`): a
//! convex shape, usually a capsule, that [`PhysicsWorld::update_character`] moves by its velocity
//! with collision queries, sliding along walls, stepping up stairs and sticking to the floor. Up
//! and rotation can change every update, so the character works on a spherical planet. A
//! character reports its [`GroundState`], ground normal and [`CharacterContact`]s, can carry a
//! kinematic inner body and can collide with other characters. [`CharacterRef::save_state`] and
//! [`CharacterMut::restore_state`] continue a character bit for bit, for replays.
//!
//! # Vehicles
//! [`PhysicsWorld::create_vehicle`] attaches a wheeled vehicle (Jolt `VehicleConstraint` with
//! the wheeled controller) to a dynamic chassis body: suspension, wheels that find the ground
//! with a ray, sphere or cylinder cast ([`VehicleCollisionTester`]), an engine, an automatic
//! transmission and differentials. The vehicle runs inside [`PhysicsWorld::step`]; the caller
//! sets [`DriverInput`] and, for radial gravity, the gravity at the vehicle every tick
//! ([`VehicleMut::set_gravity`]), and reads each wheel's [`WheelState`].
//!
//! ```
//! use oxijolt::*;
//!
//! # fn main() -> Result<(), Box<dyn std::error::Error>> {
//! let mut world = PhysicsWorld::new(WorldSettings::default().gravity(Vec3::ZERO))?;
//! let floor = Shape::new_box(Vec3::new(50.0, 1.0, 50.0))?;
//! world.create_body(&floor, &BodySettings::new_static().position(RVec3::new(0.0, -1.0, 0.0)))?;
//!
//! // A chassis with a low centre of mass, under the caller's gravity only.
//! let hull = Shape::new_box(Vec3::new(0.9, 0.3, 2.0))?;
//! let chassis_shape = Shape::new_offset_center_of_mass(&hull, Vec3::new(0.0, -0.3, 0.0))?;
//! let chassis = world.create_body(
//!     &chassis_shape,
//!     &BodySettings::new_dynamic()
//!         .position(RVec3::new(0.0, 1.0, 0.0))
//!         .mass(1500.0)
//!         .allow_sleeping(false)
//!         .gravity_factor(0.0),
//! )?;
//! let wheel = |x: f32, z: f32| WheelSettings::new(Vec3::new(x, -0.1, z)).radius(0.35);
//! let settings = VehicleSettings::new(
//!     vec![wheel(0.9, 1.4), wheel(-0.9, 1.4), wheel(0.9, -1.4), wheel(-0.9, -1.4)],
//!     vec![VehicleDifferentialSettings::new(Some(0), Some(1))],
//!     VehicleCollisionTester::cast_sphere(ObjectLayer::MOVING, 0.2),
//! );
//! let car = world.create_vehicle(chassis, &settings)?;
//!
//! for _ in 0..120 {
//!     let mut vehicle = world.vehicle_mut(car)?;
//!     vehicle.set_gravity(Vec3::new(0.0, -9.81, 0.0))?;
//!     vehicle.set_driver_input(DriverInput { forward: 1.0, ..DriverInput::default() })?;
//!     world.step(1.0 / 60.0)?;
//! }
//! let wheels = world.vehicle(car)?.wheels();
//! let contact = wheels[0].contact.expect("the front left wheel is on the floor");
//! assert!(contact.normal.y > 0.99);
//! assert!(world.body(chassis)?.position().z > 1.0);
//! # Ok(())
//! # }
//! ```
//!
//! # Ragdolls
//! [`PhysicsWorld::create_ragdoll`] creates a ragdoll (Jolt `Ragdoll`) from [`RagdollSettings`]:
//! a [`Skeleton`], one body per joint and a [`RagdollJoint`] to each part's parent, a
//! [`SwingTwistConstraintSettings`], [`HingeConstraintSettings`] with limits or
//! [`SixDofConstraintSettings`] with asymmetric limits. Parts of one ragdoll never collide with
//! each other. A ragdoll reports its [`SkeletonPose`] and joint readings, can be posed, driven to
//! a pose with motors or kinematically, and a [`SettleDetector`] tells when it has come to rest.
//!
//! Ragdolls usually live in a second world next to the main one; give both worlds static bodies
//! made from the same [`Shape`]s, which Jolt shares. For the caller's own gravity, create the
//! parts with [`BodySettings::gravity_factor`] 0 and add `g * mass` ([`BodyRef::mass`]) to each
//! part every tick.
//!
//! Joints do not stay within their limits on every tick: in each solver iteration Jolt solves
//! contacts after constraints, so on impact joints pass their limits for a few dozen ticks, and a
//! small error can remain at rest (see [`RagdollSettings`]).
//!
//! ```
//! use oxijolt::*;
//!
//! # fn main() -> Result<(), Box<dyn std::error::Error>> {
//! let mut world = PhysicsWorld::new(WorldSettings::default())?;
//! let floor = Shape::new_box(Vec3::new(20.0, 1.0, 20.0))?;
//! world.create_body(&floor, &BodySettings::new_static().position(RVec3::new(0.0, -1.0, 0.0)))?;
//!
//! // Three capsules stacked along Y, joined by swing-twist joints at their ends.
//! let skeleton = Skeleton::new(&[
//!     SkeletonJoint { name: "root", parent: None },
//!     SkeletonJoint { name: "middle", parent: Some(0) },
//!     SkeletonJoint { name: "top", parent: Some(1) },
//! ])?;
//! let capsule = Shape::new_capsule(0.25, 0.15)?;
//! let up = Vec3::new(0.0, 1.0, 0.0);
//! let side = Vec3::new(1.0, 0.0, 0.0);
//! let parts: Vec<RagdollPart<'_>> = (0..3)
//!     .map(|i| RagdollPart {
//!         shape: &capsule,
//!         body: BodySettings::new_dynamic().position(RVec3::new(0.0, 1.0 + 0.7 * i as Real, 0.0)),
//!         joint: (i > 0).then(|| {
//!             let anchor = RVec3::new(0.0, 0.65 + 0.7 * i as Real, 0.0);
//!             RagdollJoint::SwingTwist(
//!                 SwingTwistConstraintSettings::new(anchor, up, side)
//!                     .half_cone_angles(0.6, 0.6)
//!                     .twist_limits(-0.3, 0.3),
//!             )
//!         }),
//!     })
//!     .collect();
//! let settings = RagdollSettings::new(&skeleton, &parts)?;
//! let ragdoll = world.create_ragdoll(&settings, None, Activation::Activate)?;
//!
//! let mut detector = SettleDetector::default();
//! let settled = (0..600).any(|_| {
//!     world.step(1.0 / 60.0).unwrap();
//!     detector.update(&world.ragdoll(ragdoll).unwrap())
//! });
//! assert!(settled);
//! let pose = world.ragdoll(ragdoll)?.pose();
//! assert!(pose.root_offset.y < 0.5);
//! # Ok(())
//! # }
//! ```
//!
//! # Constraints
//! [`PhysicsWorld::create_constraint`] joins two bodies of a world with a constraint built from
//! settings: [`FixedConstraintSettings`], [`PointConstraintSettings`],
//! [`DistanceConstraintSettings`], [`HingeConstraintSettings`], [`SliderConstraintSettings`],
//! [`ConeConstraintSettings`], [`SwingTwistConstraintSettings`], [`SixDofConstraintSettings`],
//! [`GearConstraintSettings`], [`RackAndPinionConstraintSettings`], [`PulleyConstraintSettings`]
//! and [`PathConstraintSettings`]. The world owns the constraint; the returned [`ConstraintId`]
//! is typed by the kind, which selects the motor, target, limit and readout methods of
//! [`ConstraintRef`] and [`ConstraintMut`]. A body cannot be removed while a constraint uses it.
//!
//! # Soft bodies
//! [`PhysicsWorld::create_soft_body`] creates a soft body (cloth, a pressurised ball) from
//! [`SoftBodySharedSettings`]: vertices, the faces between them and constraints that
//! [`SoftBodySharedSettingsBuilder::create_constraints`] generates (edges, shear edges, bend
//! constraints and long range attachments) or the caller adds (edges, dihedral bends, volume
//! constraints for solid bodies). It is an ordinary body with a [`BodyId`];
//! [`PhysicsWorld::soft_body`] reads its vertices in world space, and
//! [`PhysicsWorld::soft_body_mut`] sets vertex velocities and inverse masses and moves kinematic
//! (pinned) vertices. Soft bodies collide with rigid bodies, not with each other (Jolt does not
//! implement that yet); constraints and vehicles refuse them. So do the body-level velocity and
//! torque setters, which Jolt ignores on soft bodies, and the point-force setter: Jolt applies
//! its force at the centre, but also adds its torque, which a soft body never clears.
//! [`BodyMut::add_force`] works on soft bodies.
//!
//! # Events
//! A world records contact, body activation and soft body contact events once
//! [`PhysicsWorld::set_event_settings`] asks for them; by default it records nothing and installs
//! no listener in Jolt. Jolt reports them from its worker threads during a step; the world sorts
//! each step's events into an order that does not depend on the thread count and queues them
//! until [`PhysicsWorld::take_events`], which should be called after every step. Contacts carry
//! the sub-shapes that touch and the user data of their [`PhysicsMaterial`]s.
//! `docs/events.md` describes where each event comes from.
//!
//! ```
//! use oxijolt::*;
//!
//! # fn main() -> Result<(), Box<dyn std::error::Error>> {
//! let mut world = PhysicsWorld::new(WorldSettings::default())?;
//! world.set_event_settings(EventSettings::default().contacts(true));
//! let floor = Shape::new_box(Vec3::new(10.0, 1.0, 10.0))?;
//! world.create_body(&floor, &BodySettings::new_static().position(RVec3::new(0.0, -1.0, 0.0)))?;
//! let cube_shape = Shape::new_box(Vec3::new(0.5, 0.5, 0.5))?;
//! let cube = world.create_body(
//!     &cube_shape,
//!     &BodySettings::new_dynamic().position(RVec3::new(0.0, 1.0, 0.0)),
//! )?;
//!
//! let mut added = Vec::new();
//! for _ in 0..60 {
//!     world.step(1.0 / 60.0)?;
//!     for event in world.take_events().contacts {
//!         if let ContactEvent::Added { manifold, .. } = event {
//!             added.push(manifold.pair.body2);
//!         }
//!     }
//! }
//! assert_eq!(added, [cube]);
//! # Ok(())
//! # }
//! ```
//!
//! # Threads
//! Changing a world, including [`PhysicsWorld::step`], takes `&mut PhysicsWorld`; reading it
//! takes `&PhysicsWorld`. `PhysicsWorld` is `Send` and `Sync`, so many threads may read one
//! world while nobody steps it, and different worlds may be stepped on different threads at
//! the same time.
//!
//! A world runs Jolt's jobs on Jolt's own thread pool ([`WorldSettings::worker_threads`]) or on
//! the caller's pool, such as Rayon or a game's own, through a [`JobSystem`] set with
//! [`WorldSettings::job_system`].
//!
//! # Determinism
//! The same binary, initial state and calls in the same order give bit-identical results on
//! one machine, for any [`WorldSettings::worker_threads`] or caller [`JobSystem`]. The order of body creation and removal is part of the
//! state: it decides the [`BodyId`]s. Results agree across platforms and compilers only with
//! the `cross-platform-deterministic` feature. The repository README's Determinism section
//! lists what is and is not covered.
//!
//! # Save and restore
//! [`PhysicsWorld::save_state`] saves the world's simulation state (bodies, contacts,
//! constraints, vehicles and characters) as a [`WorldState`], and
//! [`PhysicsWorld::restore_state`] goes back to it, for rollback and replays: after a restore,
//! the same calls give the same results bit for bit. A state restores only into the world that
//! saved it, while no body, character, vehicle, ragdoll or constraint has been created or
//! removed since; configuration Jolt does not save, such as constraint limits, stays as it is.
//! [`WorldState`] lists what is and is not saved.
//!
//! ```
//! use oxijolt::*;
//!
//! # fn main() -> Result<(), Box<dyn std::error::Error>> {
//! let mut world = PhysicsWorld::new(WorldSettings::default())?;
//! let floor = Shape::new_box(Vec3::new(10.0, 1.0, 10.0))?;
//! world.create_body(&floor, &BodySettings::new_static().position(RVec3::new(0.0, -1.0, 0.0)))?;
//! let ball_shape = Shape::new_sphere(0.5)?;
//! let ball = world.create_body(
//!     &ball_shape,
//!     &BodySettings::new_dynamic().position(RVec3::new(0.0, 2.0, 0.0)),
//! )?;
//!
//! let saved = world.save_state();
//! for _ in 0..60 {
//!     world.step(1.0 / 60.0)?;
//! }
//! let first_run = world.body(ball)?.position();
//!
//! world.restore_state(&saved)?;
//! for _ in 0..60 {
//!     world.step(1.0 / 60.0)?;
//! }
//! assert_eq!(world.body(ball)?.position(), first_run);
//! # Ok(())
//! # }
//! ```
//!
//! # Frame
//! [`PhysicsWorld::rebase`] moves the whole world into a new frame (a floating origin) with one
//! rotation and translation, without waking or putting to sleep any body. It must name every
//! body of the world in a stable order.
//!
//! # Debug lines
//! With the `debug-renderer` feature, `PhysicsWorld::debug_lines` produces the wireframe of
//! the colliders around a point as line data (`DebugLines`); it never draws anything. The
//! feature also compiles Jolt's native debug renderer, which is left out of the native
//! libraries without it.
//!
//! # Global state
//! oxijolt calls `JPH_Init` once per process and never calls `JPH_Shutdown`. It also
//! installs joltc's object-layer, body and shape filter procs once per process and owns them;
//! code that uses `oxijolt-sys` directly must leave them alone (see its notes on the raw
//! API). The same holds for the contact, body activation and soft body contact listener procs,
//! which oxijolt installs the first time a world records events. With `debug-renderer`,
//! oxijolt also installs joltc's debug renderer procs once and serializes debug drawing.
//!
//! oxijolt installs Jolt's assertion handler once per process, before `JPH_Init`. With the
//! `asserts` feature, which compiles Jolt with its debug assertions, a failed assertion prints its
//! expression, message, file and line to stderr and aborts the process. The only exception is the
//! physics-update-error assertion, whose condition [`PhysicsWorld::step`] returns in its
//! [`StepReport`]. The handler slot is process-global in joltc: code that calls
//! `JPH_SetAssertFailureHandler` directly replaces it, which oxijolt does not support.
//!
//! # Features
//! - `double-precision`: world positions ([`Real`], [`RVec3`]) use `f64`.
//! - `cross-platform-deterministic`: Jolt's cross-platform deterministic floating point settings.
//! - `debug-renderer`: collider wireframes as line data, see above.
//! - `asserts`: Jolt's debug assertions, reported as above.
#![warn(
    missing_docs,
    unsafe_op_in_unsafe_fn,
    clippy::undocumented_unsafe_blocks,
    clippy::missing_safety_doc
)]

mod body;
mod character;
mod constraint;
#[cfg(feature = "debug-renderer")]
mod debug;
mod error;
mod filter;
mod job_system;
mod jolt_assert;
mod layers;
pub mod limits;
mod listener;
mod material;
mod math;
mod owned;
mod query;
mod ragdoll;
mod shape;
mod soft_body;
mod state;
mod vehicle;
mod world;

pub use body::{Activation, BodyId, BodyMut, BodyRef, BodySettings, MotionQuality, MotionType};
pub use character::{
    CharacterContact, CharacterId, CharacterMut, CharacterRef, CharacterSettings, CharacterState,
    ExtendedUpdateSettings, GroundState, InnerBody,
};
pub use constraint::{
    AnyConstraintId, ConeConstraint, ConeConstraintSettings, ConstraintId, ConstraintKind,
    ConstraintMut, ConstraintRef, ConstraintSettings, ConstraintSpace, ConstraintType,
    DistanceConstraint, DistanceConstraintSettings, DistanceRange, FixedConstraint,
    FixedConstraintSettings, GearConstraint, GearConstraintSettings, HermitePath, HermitePathPoint,
    HingeConstraint, HingeConstraintSettings, MotorSettings, MotorState, PathConstraint,
    PathConstraintSettings, PathRotationConstraint, PointConstraint, PointConstraintSettings,
    PulleyConstraint, PulleyConstraintSettings, PulleyLength, RackAndPinionConstraint,
    RackAndPinionConstraintSettings, SixDofAxis, SixDofConstraint, SixDofConstraintAxis,
    SixDofConstraintSettings, SliderConstraint, SliderConstraintSettings, SpringSettings,
    SwingTwistConstraint, SwingTwistConstraintSettings, SwingType,
};
#[cfg(feature = "debug-renderer")]
pub use debug::{DebugLine, DebugLineSettings, DebugLines};
pub use error::{
    BodyError, CharacterError, ConstraintError, ContactSettingsError, QueryError, RagdollError,
    ShapeError, SoftBodyError, StateError, StepError, VehicleError, WorldError,
};
pub use filter::QueryFilter;
pub use job_system::{Job, JobSystem};
pub use layers::{BroadPhaseLayer, CollisionLayers, ObjectLayer};
pub use listener::{
    ActivationEvent, ContactEvent, ContactListener, ContactManifold, ContactPoint, ContactSettings,
    ContactSettingsRejection, EventSettings, SoftBodyContactSettings, SoftBodyContacts,
    SoftBodyValidateResult, SoftBodyValidation, SoftBodyVertexContact, SubShapeIdPair, WorldEvents,
};
pub use material::PhysicsMaterial;
pub use math::{Quat, RVec3, Real, Vec3};
pub use query::{CollideShape, CollideShapeHit, RayCast, RayHit, ShapeCast, ShapeCastHit};
pub use ragdoll::{
    JointReading, JointTransform, RagdollId, RagdollJoint, RagdollMut, RagdollPart, RagdollRef,
    RagdollSettings, SettleDetector, Skeleton, SkeletonJoint, SkeletonPose,
};
pub use shape::{CompoundChild, CompoundSubShape, HeightFieldSettings, Shape, SubShapeId};
pub use soft_body::{
    LongRangeAttachment, SoftBodyBendType, SoftBodyDihedralBend, SoftBodyEdge, SoftBodyMut,
    SoftBodyRef, SoftBodySettings, SoftBodySharedSettings, SoftBodySharedSettingsBuilder,
    SoftBodyVertex, SoftBodyVertexAttributes, SoftBodyVertexState, SoftBodyVolume,
};
pub use state::WorldState;
pub use vehicle::{
    DriverInput, SuspensionSpring, VehicleAntiRollBar, VehicleCollisionTester,
    VehicleDifferentialSettings, VehicleEngineSettings, VehicleId, VehicleMut, VehicleRef,
    VehicleSettings, VehicleTransmissionSettings, WheelContact, WheelSettings, WheelState,
    DEFAULT_LATERAL_FRICTION, DEFAULT_LONGITUDINAL_FRICTION, DEFAULT_NORMALIZED_TORQUE,
};
pub use world::{PhysicsWorld, StepReport, WorldSettings};

/// The repository's guide, `docs/guide.md`, whose examples run as doctests.
#[cfg(doctest)]
#[doc = include_str!("../../../docs/guide.md")]
pub struct Guide;
