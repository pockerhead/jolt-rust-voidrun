//! Safe Rust API for [Jolt Physics](https://github.com/jrouwe/JoltPhysics) over the
//! [joltc](https://github.com/amerkoleci/joltc) raw layer in
//! [`joltc-sys`](https://docs.rs/joltc-sys).
#![warn(
    missing_docs,
    unsafe_op_in_unsafe_fn,
    clippy::undocumented_unsafe_blocks,
    clippy::missing_safety_doc
)]

mod error;
mod layers;
mod math;
mod world;

pub use error::{ShapeError, StepError, WorldError};
pub use layers::{BroadPhaseLayer, CollisionLayers, ObjectLayer};
pub use math::{Quat, RVec3, Real, Vec3};
