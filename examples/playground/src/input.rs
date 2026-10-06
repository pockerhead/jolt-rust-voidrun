//! What a scene reads each tick: controls held down, and key presses that act once.
//!
//! The window samples the held controls every tick, while presses arrive whenever a frame sees
//! them; [`InputQueue`] hands each press to exactly one tick, also when a frame runs several
//! ticks or none.

use oxijolt::RayCast;

/// Controls sampled every tick.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Held {
    /// Walking direction relative to the camera: x to the right, y forward, each in `[-1, 1]`.
    pub walk: [f32; 2],
    /// The camera's yaw, radians, which turns [`walk`](Self::walk) into the world.
    pub camera_yaw: f32,
    /// Sprint (Shift).
    pub sprint: bool,
    /// Throttle in `[-1, 1]`: W forward, S backward.
    pub throttle: f32,
    /// Steering in `[-1, 1]`, positive to the right.
    pub steer: f32,
    /// Hand brake (Space while driving).
    pub hand_brake: bool,
    /// A secondary axis in `[-1, 1]` (arrow keys up and down).
    pub up_down: f32,
    /// The ray under the mouse cursor, for highlights.
    pub aim: Option<RayCast>,
}

/// Key presses that act once.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Edges {
    /// Jump (Space).
    pub jump: bool,
    /// Switch what is controlled (Tab).
    pub switch: bool,
    /// The scene's action key (E).
    pub action: bool,
    /// Fire or throw (F).
    pub fire: bool,
    /// Toggle a scene setting (C or T).
    pub toggle: bool,
    /// Motors on or off (M).
    pub motor: bool,
    /// A click: the ray under the cursor when the left button went down.
    pub pick: Option<RayCast>,
    /// Save the world (K).
    pub save: bool,
    /// Restore the world (L).
    pub restore: bool,
    /// Move the origin (B).
    pub rebase: bool,
    /// Show or hide the collider wireframe (G).
    pub wireframe: bool,
}

impl Edges {
    /// Both sets of presses: a press in either counts, and `newer`'s click replaces `self`'s.
    pub fn merged(self, newer: Edges) -> Edges {
        Edges {
            jump: self.jump || newer.jump,
            switch: self.switch || newer.switch,
            action: self.action || newer.action,
            fire: self.fire || newer.fire,
            toggle: self.toggle || newer.toggle,
            motor: self.motor || newer.motor,
            pick: newer.pick.or(self.pick),
            save: self.save || newer.save,
            restore: self.restore || newer.restore,
            rebase: self.rebase || newer.rebase,
            wireframe: self.wireframe || newer.wireframe,
        }
    }
}

/// One tick's input.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Input {
    /// Controls held during the tick.
    pub held: Held,
    /// Presses this tick acts on.
    pub edges: Edges,
}

/// Presses waiting for the next tick.
#[derive(Clone, Copy, Debug, Default)]
pub struct InputQueue {
    pending: Edges,
}

impl InputQueue {
    /// Adds presses seen by a frame.
    pub fn push_edges(&mut self, edges: Edges) {
        self.pending = self.pending.merged(edges);
    }

    /// The input of the next tick: `held`, and every press not handed out yet.
    pub fn next_tick(&mut self, held: Held) -> Input {
        Input {
            held,
            edges: std::mem::take(&mut self.pending),
        }
    }

    /// Drops the presses not handed out yet.
    pub fn clear(&mut self) {
        self.pending = Edges::default();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn jump() -> Edges {
        Edges {
            jump: true,
            ..Edges::default()
        }
    }

    #[test]
    fn input_edges_reach_exactly_one_tick() {
        let mut queue = InputQueue::default();
        // A frame that runs no tick keeps the press for the next frame's first tick.
        queue.push_edges(jump());
        let ticks: Vec<Input> = (0..4).map(|_| queue.next_tick(Held::default())).collect();
        assert!(ticks[0].edges.jump);
        assert!(ticks[1..].iter().all(|input| !input.edges.jump));

        // One press per frame, one tick per frame.
        for _ in 0..3 {
            queue.push_edges(jump());
            assert!(queue.next_tick(Held::default()).edges.jump);
        }

        // While paused no tick runs; the press waits until stepping resumes.
        queue.push_edges(jump());
        queue.push_edges(Edges {
            fire: true,
            ..Edges::default()
        });
        let resumed = queue.next_tick(Held::default());
        assert!(resumed.edges.jump && resumed.edges.fire);
        assert_eq!(queue.next_tick(Held::default()).edges, Edges::default());

        // A reset drops what was pending.
        queue.push_edges(jump());
        queue.clear();
        assert_eq!(queue.next_tick(Held::default()).edges, Edges::default());
    }

    #[test]
    fn a_newer_click_replaces_an_older_one() {
        let ray = |x| {
            oxijolt::RayCast::new(
                oxijolt::RVec3::new(x, 0.0, 0.0),
                oxijolt::Vec3::new(0.0, -1.0, 0.0),
            )
        };
        let mut queue = InputQueue::default();
        queue.push_edges(Edges {
            pick: Some(ray(1.0)),
            ..Edges::default()
        });
        queue.push_edges(Edges {
            pick: Some(ray(2.0)),
            ..Edges::default()
        });
        assert_eq!(queue.next_tick(Held::default()).edges.pick, Some(ray(2.0)));
    }
}
