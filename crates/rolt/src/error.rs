//! Error types. Every value Jolt only checks with debug assertions is validated before it
//! reaches Jolt and reported through these.

use std::fmt;

/// Why a [`PhysicsWorld`](crate::PhysicsWorld) could not be created or changed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum WorldError {
    /// Jolt's one-time global initialisation failed.
    InitFailed,
    /// A world setting is out of range; the payload names it.
    InvalidSettings(&'static str),
    /// The collision layer tables are inconsistent; the payload says how.
    InvalidLayers(&'static str),
    /// Jolt or joltc returned null when creating the named object.
    AllocationFailed(&'static str),
}

impl fmt::Display for WorldError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InitFailed => f.write_str("Jolt initialisation failed"),
            Self::InvalidSettings(what) => write!(f, "invalid world setting: {what}"),
            Self::InvalidLayers(what) => write!(f, "invalid collision layers: {what}"),
            Self::AllocationFailed(what) => write!(f, "could not create the {what}"),
        }
    }
}

impl std::error::Error for WorldError {}

/// Why a [`Shape`](crate::Shape) could not be created.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum ShapeError {
    /// Jolt's one-time global initialisation failed.
    InitFailed,
    /// A dimension is not finite or not positive; the payload names it.
    InvalidDimensions(&'static str),
    /// joltc returned null.
    AllocationFailed,
}

impl fmt::Display for ShapeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InitFailed => f.write_str("Jolt initialisation failed"),
            Self::InvalidDimensions(what) => write!(f, "invalid shape dimensions: {what}"),
            Self::AllocationFailed => f.write_str("could not create the shape"),
        }
    }
}

impl std::error::Error for ShapeError {}

/// Why [`PhysicsWorld::step`](crate::PhysicsWorld::step) reported a problem.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum StepError {
    /// The time step is not finite or not positive. The world did not advance.
    InvalidDeltaTime,
    /// The step ran, but Jolt dropped work because a fixed-size buffer was full; the flags say
    /// which. The world has advanced. Raise the matching [`WorldSettings`](crate::WorldSettings)
    /// limit if this happens.
    CacheFull {
        /// The contact manifold cache was full (`max_body_pairs` related).
        manifold_cache: bool,
        /// The body pair cache was full (`max_body_pairs`).
        body_pair_cache: bool,
        /// The contact constraint buffer was full (`max_contact_constraints`).
        contact_constraints: bool,
    },
}

impl fmt::Display for StepError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidDeltaTime => f.write_str("the time step must be finite and positive"),
            Self::CacheFull {
                manifold_cache,
                body_pair_cache,
                contact_constraints,
            } => write!(
                f,
                "the step dropped work: manifold cache full: {manifold_cache}, \
                 body pair cache full: {body_pair_cache}, \
                 contact constraints full: {contact_constraints}"
            ),
        }
    }
}

impl std::error::Error for StepError {}
