//! Typed constraint ids.

use std::cmp::Ordering;
use std::fmt;
use std::hash::{Hash, Hasher};
use std::marker::PhantomData;

use super::world::{ConstraintKind, ConstraintType};
use crate::world::WorldTag;

/// Identifies a constraint of kind `K` in the world that created it.
///
/// The raw value is 1 for the first constraint of a world, then 2, 3 and so on, whatever the
/// kind; ids are never reused within a world, so the same creation history gives the same ids.
/// Ids order by creation, then by world. Using an id with another world returns
/// [`ConstraintError::WrongWorld`](crate::ConstraintError::WrongWorld).
pub struct ConstraintId<K> {
    pub(super) raw: u32,
    world: WorldTag,
    kind: PhantomData<fn() -> K>,
}

impl<K: ConstraintKind> ConstraintId<K> {
    pub(super) fn new(raw: u32, world: WorldTag) -> Self {
        Self {
            raw,
            world,
            kind: PhantomData,
        }
    }
}

impl<K> ConstraintId<K> {
    /// The id's number within its world.
    pub fn to_raw(self) -> u32 {
        self.raw
    }
}

impl<K> Clone for ConstraintId<K> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<K> Copy for ConstraintId<K> {}

impl<K> PartialEq for ConstraintId<K> {
    fn eq(&self, other: &Self) -> bool {
        (self.raw, self.world) == (other.raw, other.world)
    }
}

impl<K> Eq for ConstraintId<K> {}

impl<K> PartialOrd for ConstraintId<K> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl<K> Ord for ConstraintId<K> {
    fn cmp(&self, other: &Self) -> Ordering {
        (self.raw, self.world).cmp(&(other.raw, other.world))
    }
}

impl<K> Hash for ConstraintId<K> {
    fn hash<H: Hasher>(&self, state: &mut H) {
        (self.raw, self.world).hash(state);
    }
}

impl<K: ConstraintKind> fmt::Debug for ConstraintId<K> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("ConstraintId")
            .field(&K::TYPE)
            .field(&self.raw)
            .finish()
    }
}

/// A constraint id of any kind, for listing and removing constraints.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct AnyConstraintId {
    // Declared first, so ids order by creation before the world.
    pub(super) raw: u32,
    pub(super) world: WorldTag,
    pub(super) kind: ConstraintType,
}

impl AnyConstraintId {
    /// The id's number within its world.
    pub fn to_raw(self) -> u32 {
        self.raw
    }

    /// The kind of constraint the id names.
    pub fn kind(self) -> ConstraintType {
        self.kind
    }

    /// The typed id, if the constraint is of kind `K`.
    pub fn downcast<K: ConstraintKind>(self) -> Option<ConstraintId<K>> {
        (self.kind == K::TYPE).then(|| ConstraintId::new(self.raw, self.world))
    }
}

impl<K: ConstraintKind> From<ConstraintId<K>> for AnyConstraintId {
    fn from(id: ConstraintId<K>) -> Self {
        Self {
            raw: id.raw,
            world: id.world,
            kind: K::TYPE,
        }
    }
}

impl fmt::Debug for AnyConstraintId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("ConstraintId")
            .field(&self.kind)
            .field(&self.raw)
            .finish()
    }
}
