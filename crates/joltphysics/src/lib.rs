//! Safe Rust API for [Jolt Physics](https://github.com/jrouwe/JoltPhysics) over the
//! [joltc](https://github.com/amerkoleci/joltc) raw layer in
//! [`joltphysics-sys`](https://github.com/pockerhead/jolt-rust-voidrun/tree/main/crates/joltphysics-sys).
//!
//! A [`PhysicsWorld`] owns a Jolt physics system with its collision layers
//! ([`CollisionLayers`]), its own job system and temp allocator. Bodies are created from a
//! [`Shape`] and [`BodySettings`] and named by a [`BodyId`]. Shapes are boxes (with a
//! configurable convex radius), spheres, Y-cylinders, Y-capsules, heightfields and compounds
//! whose children carry their own pose and user data, and one shape may serve
//! many bodies in many worlds.
//!
//! Scene queries run on `&PhysicsWorld`: [`PhysicsWorld::cast_ray`] finds the closest body
//! along a ray, [`PhysicsWorld::cast_shape`] the first obstacle a moving shape hits, and
//! [`PhysicsWorld::collide_shape`] every obstacle a shape at a pose overlaps. Each takes a
//! [`QueryFilter`] that selects object layers and compound-child groups and can skip one body.
//! Every normal they report is the outward surface normal of the obstacle.
//!
//! ```
//! use joltphysics::*;
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
//! use joltphysics::*;
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
//! # Threads
//! Changing a world, including [`PhysicsWorld::step`], takes `&mut PhysicsWorld`; reading it
//! takes `&PhysicsWorld`. `PhysicsWorld` is `Send` and `Sync`, so many threads may read one
//! world while nobody steps it, and different worlds may be stepped on different threads at
//! the same time.
//!
//! # Determinism
//! The same calls in the same order give bit-identical results on one machine, for any
//! [`WorldSettings::worker_threads`]. The order of body creation and removal is part of the
//! state: it decides the [`BodyId`]s. Results agree across platforms and compilers only with
//! the `cross-platform-deterministic` feature. The repository README's Determinism section
//! lists what is and is not covered.
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
//! joltphysics calls `JPH_Init` once per process and never calls `JPH_Shutdown`. It also
//! installs joltc's object-layer, body and shape filter procs once per process and owns them;
//! code that uses `joltphysics-sys` directly must leave them alone (see its notes on the raw
//! API). With `debug-renderer`, joltphysics also installs joltc's debug renderer procs once and
//! serializes debug drawing.
#![warn(
    missing_docs,
    unsafe_op_in_unsafe_fn,
    clippy::undocumented_unsafe_blocks,
    clippy::missing_safety_doc
)]

mod body;
mod character;
#[cfg(feature = "debug-renderer")]
mod debug;
mod error;
mod filter;
mod layers;
mod math;
mod owned;
mod query;
mod shape;
mod vehicle;
mod world;

pub use body::{Activation, BodyId, BodyMut, BodyRef, BodySettings, MotionQuality, MotionType};
pub use character::{
    CharacterContact, CharacterId, CharacterMut, CharacterRef, CharacterSettings, CharacterState,
    ExtendedUpdateSettings, GroundState, InnerBody,
};
#[cfg(feature = "debug-renderer")]
pub use debug::{DebugLine, DebugLineSettings, DebugLines};
pub use error::{
    BodyError, CharacterError, QueryError, ShapeError, StepError, VehicleError, WorldError,
};
pub use filter::QueryFilter;
pub use layers::{BroadPhaseLayer, CollisionLayers, ObjectLayer};
pub use math::{Quat, RVec3, Real, Vec3};
pub use query::{CollideShape, CollideShapeHit, RayCast, RayHit, ShapeCast, ShapeCastHit};
pub use shape::{CompoundChild, CompoundSubShape, HeightFieldSettings, Shape, SubShapeId};
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
