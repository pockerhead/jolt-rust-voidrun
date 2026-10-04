//! Safe Rust API for [Jolt Physics](https://github.com/jrouwe/JoltPhysics) 5.6 over the
//! [joltc](https://github.com/amerkoleci/joltc) raw layer in
//! [`oxijolt-sys`](https://github.com/pockerhead/oxijolt/tree/main/crates/oxijolt-sys).
//!
//! A [`PhysicsWorld`] owns a Jolt physics system with its collision layers
//! ([`CollisionLayers`]), its job system and temp allocator. Bodies are created from a [`Shape`]
//! and [`BodySettings`] and named by a [`BodyId`]; one shape may serve many bodies in many worlds.
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
//! # What a world holds
//! - Rigid bodies of box, sphere, cylinder, capsule, heightfield and compound [`Shape`]s, with
//!   [`PhysicsMaterial`]s that carry the caller's user data.
//! - Scene queries on `&PhysicsWorld`: [`PhysicsWorld::cast_ray`], [`PhysicsWorld::cast_shape`]
//!   and [`PhysicsWorld::collide_shape`], filtered by [`QueryFilter`].
//! - Virtual characters: [`PhysicsWorld::create_character`].
//! - Wheeled vehicles on a chassis body: [`PhysicsWorld::create_vehicle`].
//! - Ragdolls: [`PhysicsWorld::create_ragdoll`].
//! - Constraints of twelve kinds between two bodies: [`PhysicsWorld::create_constraint`]
//!   ([constraints guide]).
//! - Soft bodies: [`PhysicsWorld::create_soft_body`] ([soft body guide]).
//! - Contact, activation and soft body contact events and a [`ContactListener`]:
//!   [`PhysicsWorld::set_event_settings`], [`PhysicsWorld::take_events`] ([events guide]).
//! - Snapshots for rollback: [`PhysicsWorld::save_state`], [`PhysicsWorld::restore_state`]
//!   ([state guide]).
//! - A floating origin: [`PhysicsWorld::rebase`].
//! - With the `debug-renderer` feature, the colliders' wireframe as line data
//!   (`PhysicsWorld::debug_lines`); nothing is drawn.
//!
//! The [guide] builds a game scene with terrain, queries, a character, a vehicle and a ragdoll.
//!
//! # Contract
//! - **Errors.** A call that returns `Err` was refused and changed nothing, unless its
//!   documentation says otherwise. A step that ran but dropped work returns `Ok`, and its
//!   [`StepReport`] says what was dropped.
//! - **Magnitudes.** Inputs with a magnitude are checked against [`limits`] before they reach
//!   Jolt.
//! - **Threads.** Changing a world, [`PhysicsWorld::step`] included, takes `&mut PhysicsWorld`;
//!   reading it takes `&PhysicsWorld`. `PhysicsWorld` is `Send` and `Sync`: many threads may read
//!   one world while nobody steps it, and different worlds may step on different threads at the
//!   same time. A world runs Jolt's jobs on Jolt's thread pool
//!   ([`WorldSettings::worker_threads`]) or on the caller's pool through a [`JobSystem`]
//!   ([job system guide]).
//! - **Determinism.** The order of calls, including the order of body creation and removal, is
//!   part of the state. The tests check bit-identical results with 1 and 4 worker threads and
//!   with caller job systems on one machine; results agree across platforms and compilers only
//!   with the `cross-platform-deterministic` feature ([determinism]).
//!
//! # Global state
//! oxijolt calls `JPH_Init` once per process and never calls `JPH_Shutdown`. It installs
//! joltc's process-global procs once and owns them: the object-layer, body and shape filters,
//! the contact, body activation and soft body contact listeners (the first time a world records
//! events), the debug renderer (with `debug-renderer`, which also serializes debug drawing) and
//! Jolt's assertion handler (before `JPH_Init`). Code that also uses `oxijolt-sys` directly must
//! leave them alone; see the notes on the raw API in `oxijolt-sys`.
//!
//! With the `asserts` feature, which compiles Jolt with its debug assertions, a failed assertion
//! prints its expression, message, file and line to stderr and aborts the process. The only
//! exception is the physics-update-error assertion, whose condition [`PhysicsWorld::step`]
//! returns in its [`StepReport`].
//!
//! # Features
//! - `double-precision`: world positions ([`Real`], [`RVec3`]) use `f64`.
//! - `cross-platform-deterministic`: Jolt's cross-platform deterministic floating point settings.
//! - `debug-renderer`: collider wireframes as line data, see above.
//! - `asserts`: Jolt's debug assertions, reported as above.
//! - `bindgen`: generates the raw bindings with libclang at build time.
//!
//! [guide]: https://github.com/pockerhead/oxijolt/blob/main/docs/guide.md
//! [constraints guide]: https://github.com/pockerhead/oxijolt/blob/main/docs/constraints.md
//! [soft body guide]: https://github.com/pockerhead/oxijolt/blob/main/docs/soft-bodies.md
//! [events guide]: https://github.com/pockerhead/oxijolt/blob/main/docs/events.md
//! [state guide]: https://github.com/pockerhead/oxijolt/blob/main/docs/state.md
//! [job system guide]: https://github.com/pockerhead/oxijolt/blob/main/docs/job-system.md
//! [determinism]: https://github.com/pockerhead/oxijolt/blob/main/docs/determinism.md
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

/// The repository's README, whose example runs as a doctest.
#[cfg(doctest)]
#[doc = include_str!("../../../README.md")]
pub struct Readme;

/// The constraints guide, `docs/constraints.md`.
#[cfg(doctest)]
#[doc = include_str!("../../../docs/constraints.md")]
pub struct ConstraintsGuide;

/// The soft body guide, `docs/soft-bodies.md`.
#[cfg(doctest)]
#[doc = include_str!("../../../docs/soft-bodies.md")]
pub struct SoftBodiesGuide;

/// The events guide, `docs/events.md`.
#[cfg(doctest)]
#[doc = include_str!("../../../docs/events.md")]
pub struct EventsGuide;

/// The save and restore guide, `docs/state.md`.
#[cfg(doctest)]
#[doc = include_str!("../../../docs/state.md")]
pub struct StateGuide;

/// The job system guide, `docs/job-system.md`.
#[cfg(doctest)]
#[doc = include_str!("../../../docs/job-system.md")]
pub struct JobSystemGuide;
