//! Which bodies a world state saves: [`BodySelection`] and the id lists it resolves to.

use crate::{BodyId, PhysicsWorld, StateError};

/// Which bodies [`PhysicsWorld::save_state_of`] saves.
///
/// Global state, contacts, constraints, characters and pending contact-cache invalidations are
/// saved whole whatever the selection.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum BodySelection<'a> {
    /// Every body.
    All,
    /// The listed bodies, in any order, duplicates allowed; no body for an empty slice. Every id
    /// must name a body of this world, or the call fails with [`StateError::Body`].
    Only(&'a [BodyId]),
}

impl PhysicsWorld {
    /// Writes the raw ids of the bodies `selection` names into `ids`, ascending and without
    /// duplicates, and returns them; `None` for [`BodySelection::All`], which leaves `ids` alone.
    /// Checks every listed id before it touches `ids`.
    pub(super) fn select_bodies<'v>(
        &self,
        selection: BodySelection<'_>,
        ids: &'v mut Vec<u32>,
    ) -> Result<Option<&'v [u32]>, StateError> {
        match selection {
            BodySelection::All => Ok(None),
            BodySelection::Only(listed) => {
                for &id in listed {
                    self.check(id).map_err(StateError::Body)?;
                }
                ids.clear();
                ids.extend(listed.iter().map(|id| id.to_raw()));
                ids.sort_unstable();
                ids.dedup();
                Ok(Some(ids))
            }
        }
    }
}
