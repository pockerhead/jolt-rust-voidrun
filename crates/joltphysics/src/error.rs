//! Error types. Every value Jolt only checks with debug assertions is validated before it
//! reaches Jolt and reported through these; magnitudes follow [`crate::limits`], which says
//! which assertion paths are derived and which are covered by tests.

use std::fmt;

use crate::{BodyId, CharacterId, ObjectLayer, RagdollId, VehicleId};

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
    /// The time step is not finite, below
    /// [`PhysicsWorld::MIN_DELTA_TIME`](crate::PhysicsWorld::MIN_DELTA_TIME) or above
    /// [`PhysicsWorld::MAX_DELTA_TIME`](crate::PhysicsWorld::MAX_DELTA_TIME).
    InvalidDeltaTime,
}

impl fmt::Display for StepError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidDeltaTime => write!(
                f,
                "the time step must be finite and between {} and {} s",
                crate::PhysicsWorld::MIN_DELTA_TIME,
                crate::PhysicsWorld::MAX_DELTA_TIME
            ),
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
    /// The body is the inner body of a character; remove the character instead.
    OwnedByCharacter(BodyId),
    /// The body is the chassis of a vehicle; remove the vehicle first.
    UsedByVehicle(BodyId),
    /// The body is a part of a ragdoll; remove the ragdoll instead.
    OwnedByRagdoll(BodyId),
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
            Self::OwnedByCharacter(id) => {
                write!(f, "body {id:?} is the inner body of a character")
            }
            Self::UsedByVehicle(id) => write!(f, "body {id:?} is the chassis of a vehicle"),
            Self::OwnedByRagdoll(id) => write!(f, "body {id:?} is a part of a ragdoll"),
        }
    }
}

impl std::error::Error for BodyError {}

/// Why a character operation failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum CharacterError {
    /// The id names no character in this world: it was removed.
    NotFound(CharacterId),
    /// The id belongs to another world.
    WrongWorld(CharacterId),
    /// A value is out of range; the payload names it.
    InvalidValue(&'static str),
    /// The character asked for an inner body and the world already holds its maximum number of
    /// bodies.
    TooManyBodies,
    /// The world has given out every character id.
    TooManyCharacters,
}

impl fmt::Display for CharacterError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NotFound(id) => write!(f, "no character {id:?} in this world"),
            Self::WrongWorld(id) => write!(f, "character {id:?} belongs to another world"),
            Self::InvalidValue(what) => write!(f, "invalid character value: {what}"),
            Self::TooManyBodies => f.write_str("the world is full; no room for the inner body"),
            Self::TooManyCharacters => f.write_str("the world has no character ids left"),
        }
    }
}

impl std::error::Error for CharacterError {}

/// Why a vehicle operation failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum VehicleError {
    /// The id names no vehicle in this world: it was removed.
    NotFound(VehicleId),
    /// The id belongs to another world.
    WrongWorld(VehicleId),
    /// A setting or input is out of range; the payload names it.
    InvalidValue(&'static str),
    /// The chassis body is not usable: not in this world, the inner body of a character or a
    /// part of a ragdoll.
    Body(BodyError),
    /// The chassis body is not dynamic.
    NotDynamic(BodyId),
    /// The chassis body already carries a vehicle.
    AlreadyHasVehicle(BodyId),
    /// The world has given out every vehicle id.
    TooManyVehicles,
}

impl fmt::Display for VehicleError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NotFound(id) => write!(f, "no vehicle {id:?} in this world"),
            Self::WrongWorld(id) => write!(f, "vehicle {id:?} belongs to another world"),
            Self::InvalidValue(what) => write!(f, "invalid vehicle value: {what}"),
            Self::Body(error) => write!(f, "unusable chassis body: {error}"),
            Self::NotDynamic(id) => write!(f, "chassis body {id:?} is not dynamic"),
            Self::AlreadyHasVehicle(id) => write!(f, "body {id:?} already carries a vehicle"),
            Self::TooManyVehicles => f.write_str("the world has no vehicle ids left"),
        }
    }
}

impl std::error::Error for VehicleError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Body(error) => Some(error),
            _ => None,
        }
    }
}

/// Why a skeleton, ragdoll settings or ragdoll operation failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum RagdollError {
    /// Jolt's one-time global initialisation failed.
    InitFailed,
    /// The id names no ragdoll in this world: it was removed.
    NotFound(RagdollId),
    /// The id belongs to another world.
    WrongWorld(RagdollId),
    /// A setting, pose or input is out of range; the payload names it.
    InvalidValue(&'static str),
    /// A part's object layer does not exist in this world's collision layers.
    UnknownObjectLayer(ObjectLayer),
    /// A part's body settings could not be turned into Jolt's.
    Body(BodyError),
    /// The world has no room for every part of the ragdoll.
    TooManyBodies,
    /// The world has given out every ragdoll id.
    TooManyRagdolls,
}

impl fmt::Display for RagdollError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InitFailed => f.write_str("Jolt initialisation failed"),
            Self::NotFound(id) => write!(f, "no ragdoll {id:?} in this world"),
            Self::WrongWorld(id) => write!(f, "ragdoll {id:?} belongs to another world"),
            Self::InvalidValue(what) => write!(f, "invalid ragdoll value: {what}"),
            Self::UnknownObjectLayer(layer) => write!(
                f,
                "object layer {} of a ragdoll part does not exist in this world",
                layer.get()
            ),
            Self::Body(error) => write!(f, "unusable ragdoll part: {error}"),
            Self::TooManyBodies => f.write_str("the world has no room for every ragdoll part"),
            Self::TooManyRagdolls => f.write_str("the world has no ragdoll ids left"),
        }
    }
}

impl std::error::Error for RagdollError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Body(error) => Some(error),
            _ => None,
        }
    }
}
