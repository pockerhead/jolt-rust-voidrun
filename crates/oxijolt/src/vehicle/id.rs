//! Typed vehicle ids and the vehicle kinds.

use std::cmp::Ordering;
use std::fmt;
use std::hash::{Hash, Hasher};
use std::marker::PhantomData;

use crate::world::WorldTag;

/// The kinds of vehicle a world can hold.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[non_exhaustive]
pub enum VehicleType {
    /// [`WheeledVehicle`]: Jolt's `WheeledVehicleController`.
    Wheeled,
    /// [`TrackedVehicle`]: Jolt's `TrackedVehicleController`.
    Tracked,
    /// [`Motorcycle`]: Jolt's `MotorcycleController`.
    Motorcycle,
}

/// The trait users can name but not implement: the vehicle kinds are the crate's own.
pub(crate) mod sealed {
    use super::VehicleType;

    /// A vehicle kind marker.
    pub trait Kind {
        /// The kind as a value.
        const TYPE: VehicleType;
    }
}

/// A kind of vehicle, named by the markers [`WheeledVehicle`], [`TrackedVehicle`] and
/// [`Motorcycle`]. It selects the methods of [`VehicleRef`](crate::VehicleRef) and
/// [`VehicleMut`](crate::VehicleMut).
pub trait VehicleKind: sealed::Kind {}

macro_rules! vehicle_kinds {
    ($($(#[$doc:meta])* $name:ident => $kind:ident;)*) => {
        $(
            $(#[$doc])*
            #[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
            pub enum $name {}

            impl sealed::Kind for $name {
                const TYPE: VehicleType = VehicleType::$kind;
            }

            impl VehicleKind for $name {}
        )*
    };
}

vehicle_kinds! {
    /// A wheeled vehicle, created with
    /// [`PhysicsWorld::create_wheeled_vehicle`](crate::PhysicsWorld::create_wheeled_vehicle).
    WheeledVehicle => Wheeled;
    /// A tracked vehicle such as a tank.
    TrackedVehicle => Tracked;
    /// A two-wheeled motorcycle that leans into turns.
    Motorcycle => Motorcycle;
}

/// Identifies a vehicle of kind `K` in the world that created it.
///
/// The raw value is 1 for the first vehicle of a world, then 2, 3 and so on, whatever the kind;
/// ids are never reused within a world, so the same creation history gives the same ids. Ids
/// order by creation, then by world. Using an id with another world returns
/// [`VehicleError::WrongWorld`](crate::VehicleError::WrongWorld).
pub struct VehicleId<K = WheeledVehicle> {
    pub(super) raw: u32,
    pub(crate) world: WorldTag,
    kind: PhantomData<fn() -> K>,
}

impl<K: VehicleKind> VehicleId<K> {
    pub(super) fn new(raw: u32, world: WorldTag) -> Self {
        Self {
            raw,
            world,
            kind: PhantomData,
        }
    }
}

impl<K> VehicleId<K> {
    /// The id's number within its world.
    pub fn to_raw(self) -> u32 {
        self.raw
    }
}

impl<K> Clone for VehicleId<K> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<K> Copy for VehicleId<K> {}

impl<K> PartialEq for VehicleId<K> {
    fn eq(&self, other: &Self) -> bool {
        (self.raw, self.world) == (other.raw, other.world)
    }
}

impl<K> Eq for VehicleId<K> {}

impl<K> PartialOrd for VehicleId<K> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl<K> Ord for VehicleId<K> {
    fn cmp(&self, other: &Self) -> Ordering {
        (self.raw, self.world).cmp(&(other.raw, other.world))
    }
}

impl<K> Hash for VehicleId<K> {
    fn hash<H: Hasher>(&self, state: &mut H) {
        (self.raw, self.world).hash(state);
    }
}

impl<K: VehicleKind> fmt::Debug for VehicleId<K> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("VehicleId")
            .field(&K::TYPE)
            .field(&self.raw)
            .finish()
    }
}

/// A vehicle id of any kind, for listing and removing vehicles.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct AnyVehicleId {
    // Declared first, so ids order by creation before the world.
    pub(super) raw: u32,
    pub(super) world: WorldTag,
    pub(super) kind: VehicleType,
}

impl AnyVehicleId {
    /// The id's number within its world.
    pub fn to_raw(self) -> u32 {
        self.raw
    }

    /// The kind of vehicle the id names.
    pub fn kind(self) -> VehicleType {
        self.kind
    }

    /// The typed id, if the vehicle is of kind `K`.
    pub fn downcast<K: VehicleKind>(self) -> Option<VehicleId<K>> {
        (self.kind == K::TYPE).then(|| VehicleId::new(self.raw, self.world))
    }
}

impl<K: VehicleKind> From<VehicleId<K>> for AnyVehicleId {
    fn from(id: VehicleId<K>) -> Self {
        Self {
            raw: id.raw,
            world: id.world,
            kind: K::TYPE,
        }
    }
}

impl fmt::Debug for AnyVehicleId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("VehicleId")
            .field(&self.kind)
            .field(&self.raw)
            .finish()
    }
}
