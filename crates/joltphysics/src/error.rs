//! Error types. Every value Jolt only checks with debug assertions is validated before it
//! reaches Jolt and reported through these.

use std::fmt;

use crate::{BodyId, ObjectLayer};

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
    /// A setting other than a dimension is out of range; the payload names it.
    InvalidSettings(&'static str),
    /// Jolt refused the shape settings (joltc does not pass on Jolt's message).
    Rejected,
    /// joltc returned null.
    AllocationFailed,
}

impl fmt::Display for ShapeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InitFailed => f.write_str("Jolt initialisation failed"),
            Self::InvalidDimensions(what) => write!(f, "invalid shape dimensions: {what}"),
            Self::InvalidSettings(what) => write!(f, "invalid shape setting: {what}"),
            Self::Rejected => f.write_str("Jolt rejected the shape settings"),
            Self::AllocationFailed => f.write_str("could not create the shape"),
        }
    }
}

impl std::error::Error for ShapeError {}

/// Why a scene query could not run.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum QueryError {
    /// A query input is out of range; the payload names it.
    InvalidValue(&'static str),
}

impl fmt::Display for QueryError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidValue(what) => write!(f, "invalid query value: {what}"),
        }
    }
}

impl std::error::Error for QueryError {}

/// Why [`PhysicsWorld::step`](crate::PhysicsWorld::step) rejected the call. The world did not
/// advance. A step that ran but dropped work is reported in its
/// [`StepReport`](crate::StepReport) instead.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum StepError {
    /// The time step is not finite or not positive.
    InvalidDeltaTime,
}

impl fmt::Display for StepError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidDeltaTime => f.write_str("the time step must be finite and positive"),
        }
    }
}

impl std::error::Error for StepError {}

/// Why a body operation failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum BodyError {
    /// The id names no body in this world: it was removed, or its index was reused.
    NotFound(BodyId),
    /// The id belongs to another world.
    WrongWorld(BodyId),
    /// The object layer does not exist in this world's collision layers.
    UnknownObjectLayer(ObjectLayer),
    /// A value is out of range; the payload names it.
    InvalidValue(&'static str),
    /// The world already holds its maximum number of bodies.
    TooManyBodies,
    /// joltc returned null when creating the body settings.
    AllocationFailed,
}

impl fmt::Display for BodyError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NotFound(id) => write!(f, "no body {id:?} in this world"),
            Self::WrongWorld(id) => write!(f, "body {id:?} belongs to another world"),
            Self::UnknownObjectLayer(layer) => {
                write!(
                    f,
                    "object layer {} does not exist in this world",
                    layer.get()
                )
            }
            Self::InvalidValue(what) => write!(f, "invalid body value: {what}"),
            Self::TooManyBodies => f.write_str("the world is full"),
            Self::AllocationFailed => f.write_str("could not create the body settings"),
        }
    }
}

impl std::error::Error for BodyError {}
