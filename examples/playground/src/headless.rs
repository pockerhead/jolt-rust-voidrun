//! Running scenes without a window: their scripts for a number of ticks, with a digest of
//! everything they simulated and drew.

use crate::digest::Digest;
use crate::draw::DrawList;
use crate::scene::{Result, SceneConfig, SceneKind};
use crate::session::Session;

/// What a headless run of one scene ended with.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Summary {
    /// The scene.
    pub kind: SceneKind,
    /// Ticks run.
    pub ticks: u32,
    /// Bodies in the world at the end.
    pub bodies: u32,
    /// The digest of the scene state and the draw list after every tick.
    pub digest: u64,
}

impl std::fmt::Display for Summary {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "scene {}: {} ticks, {} bodies, digest {:016x}",
            self.kind.name(),
            self.ticks,
            self.bodies,
            self.digest
        )
    }
}

/// Runs `kind`'s script for `ticks` ticks. Fails on a scene error, on a drawn position that is
/// not finite, and, when the run is at least as long as the scene's clip, on a milestone of the
/// clip that the run did not reach.
pub fn run(kind: SceneKind, ticks: u32, config: SceneConfig) -> Result<Summary> {
    let mut session = Session::new(kind, config)?;
    let mut digest = Digest::default();
    let mut list = DrawList::default();
    for tick in 0..ticks {
        session
            .tick_scripted()
            .map_err(|error| format!("scene {} at tick {tick}: {error}", kind.name()))?;
        session
            .write_state(&mut digest)
            .map_err(|error| format!("scene {} at tick {tick}: {error}", kind.name()))?;
        session.draw(&mut list)?;
        if !list.is_finite() {
            return Err(format!(
                "scene {} drew a non-finite position at tick {tick}",
                kind.name()
            )
            .into());
        }
        digest.draw_list(&list);
    }
    if ticks >= session.scene().record_ticks() {
        let missing = session.missing_milestones();
        if !missing.is_empty() {
            return Err(format!(
                "scene {} did not reach: {}",
                kind.name(),
                missing.join(", ")
            )
            .into());
        }
    }
    Ok(Summary {
        kind,
        ticks,
        bodies: session.scene().world().body_count(),
        digest: digest.finish(),
    })
}
