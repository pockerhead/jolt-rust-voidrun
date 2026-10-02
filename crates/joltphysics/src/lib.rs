//! Safe Rust API for [Jolt Physics](https://github.com/jrouwe/JoltPhysics) over the
//! [joltc](https://github.com/amerkoleci/joltc) raw layer in
//! [`joltphysics-sys`](https://docs.rs/joltphysics-sys).
//!
//! A [`PhysicsWorld`] owns a Jolt physics system with its collision layers
//! ([`CollisionLayers`]), its own job system and temp allocator. Bodies are created from a
//! [`Shape`] and [`BodySettings`] and named by a [`BodyId`].
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
//!     world.step(1.0 / 60.0)?;
//! }
//! assert!(world.body(ball)?.position().y < 2.0);
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
//! the `cross-platform-deterministic` feature.
//!
//! # Global state
//! joltphysics calls `JPH_Init` once per process and never calls `JPH_Shutdown`.
#![warn(
    missing_docs,
    unsafe_op_in_unsafe_fn,
    clippy::undocumented_unsafe_blocks,
    clippy::missing_safety_doc
)]

mod body;
mod error;
mod layers;
mod math;
mod shape;
mod world;

pub use body::{Activation, BodyId, BodyMut, BodyRef, BodySettings, MotionQuality, MotionType};
pub use error::{BodyError, ShapeError, StepError, WorldError};
pub use layers::{BroadPhaseLayer, CollisionLayers, ObjectLayer};
pub use math::{Quat, RVec3, Real, Vec3};
pub use shape::Shape;
pub use world::{PhysicsWorld, WorldSettings};
