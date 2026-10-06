//! The bodies a scene draws, with a pose cache kept in sync from the binding's readouts.

use std::collections::BTreeMap;

use oxijolt::{
    ActivationEvent, BodyId, BodyPose, BodySettings, PhysicsWorld, Quat, RVec3, WorldEvents,
};

use crate::digest::Digest;
use crate::draw::{Colour, DrawList, Solid};
use crate::math::position_f32;
use crate::scene::Result;
use crate::visual::{Shaped, VisualKey, Visuals};

/// One drawn body.
#[derive(Clone, Copy, Debug)]
struct Entry {
    body: BodyId,
    visual: VisualKey,
    colour: Colour,
    position: RVec3,
    rotation: Quat,
}

/// Bodies in the order the scene added them, each with its description, colour and last known
/// pose.
///
/// After every step [`sync`](Self::sync) copies the poses of the awake bodies and re-reads the
/// bodies that fell asleep at the end of the step, which the awake readout misses.
#[derive(Debug, Default)]
pub struct Tracked {
    entries: Vec<Entry>,
    index: BTreeMap<BodyId, usize>,
    scratch: Vec<BodyPose>,
}

impl Tracked {
    /// Creates a body of `shaped` with `settings`, describes it in `visuals` and tracks it.
    pub fn spawn(
        &mut self,
        world: &mut PhysicsWorld,
        shaped: &Shaped,
        settings: &BodySettings,
        visuals: &mut Visuals,
        colour: Colour,
    ) -> Result<BodyId> {
        let body = world.create_body(&shaped.shape, settings)?;
        let visual = visuals.add(shaped.visual.clone());
        self.adopt(world, body, visual, colour)?;
        Ok(body)
    }

    /// Creates a body of `shape` with `settings` and tracks it, drawn with `visual`, a
    /// description that several bodies share.
    pub fn spawn_keyed(
        &mut self,
        world: &mut PhysicsWorld,
        shape: &oxijolt::Shape,
        visual: VisualKey,
        settings: &BodySettings,
        colour: Colour,
    ) -> Result<BodyId> {
        let body = world.create_body(shape, settings)?;
        self.adopt(world, body, visual, colour)?;
        Ok(body)
    }

    /// Tracks `body`, which exists, drawn with `visual`.
    pub fn adopt(
        &mut self,
        world: &PhysicsWorld,
        body: BodyId,
        visual: VisualKey,
        colour: Colour,
    ) -> Result<()> {
        let reading = world.body(body)?;
        self.index.insert(body, self.entries.len());
        self.entries.push(Entry {
            body,
            visual,
            colour,
            position: reading.position(),
            rotation: reading.rotation(),
        });
        Ok(())
    }

    /// Removes `body` from the world and stops tracking it.
    pub fn remove(&mut self, world: &mut PhysicsWorld, body: BodyId) -> Result<()> {
        world.remove_body(body)?;
        self.forget(body);
        Ok(())
    }

    /// Stops tracking `body` without touching the world.
    pub fn forget(&mut self, body: BodyId) {
        if let Some(index) = self.index.remove(&body) {
            self.entries.remove(index);
            for entry in &self.entries[index..] {
                *self.index.get_mut(&entry.body).expect("indexed") -= 1;
            }
        }
    }

    /// Whether `body` is tracked.
    pub fn contains(&self, body: BodyId) -> bool {
        self.index.contains_key(&body)
    }

    /// Changes the description `body` is drawn with.
    pub fn set_visual(&mut self, body: BodyId, visual: VisualKey) {
        if let Some(&index) = self.index.get(&body) {
            self.entries[index].visual = visual;
        }
    }

    /// Changes the colour `body` is drawn with.
    pub fn set_colour(&mut self, body: BodyId, colour: Colour) {
        if let Some(&index) = self.index.get(&body) {
            self.entries[index].colour = colour;
        }
    }

    /// The cached pose of `body`.
    pub fn pose(&self, body: BodyId) -> Option<(RVec3, Quat)> {
        self.index.get(&body).map(|&index| {
            let entry = &self.entries[index];
            (entry.position, entry.rotation)
        })
    }

    /// Copies the poses of the awake bodies, then re-reads every tracked body that `events`
    /// reports as having fallen asleep.
    pub fn sync(&mut self, world: &PhysicsWorld, events: &WorldEvents) {
        world.active_body_poses_into(&mut self.scratch);
        for pose in &self.scratch {
            if let Some(&index) = self.index.get(&pose.id) {
                self.entries[index].position = pose.position;
                self.entries[index].rotation = pose.rotation;
            }
        }
        for event in &events.activations {
            if let ActivationEvent::Deactivated(body) = *event {
                self.reread(world, body);
            }
        }
    }

    /// Re-reads every tracked body, after a change that moved bodies without events: a rebase,
    /// a restore.
    pub fn sync_all(&mut self, world: &PhysicsWorld) {
        let bodies: Vec<BodyId> = self.entries.iter().map(|entry| entry.body).collect();
        for body in bodies {
            self.reread(world, body);
        }
    }

    fn reread(&mut self, world: &PhysicsWorld, body: BodyId) {
        let (Some(&index), Ok(reading)) = (self.index.get(&body), world.body(body)) else {
            return;
        };
        self.entries[index].position = reading.position();
        self.entries[index].rotation = reading.rotation();
    }

    /// The tracked bodies in ascending id order.
    pub fn ids_in_order(&self) -> Vec<BodyId> {
        self.index.keys().copied().collect()
    }

    /// The tracked bodies in the order they were added.
    pub fn bodies(&self) -> impl Iterator<Item = BodyId> + '_ {
        self.entries.iter().map(|entry| entry.body)
    }

    /// Number of tracked bodies.
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// Whether no body is tracked.
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// Draws every body at its cached pose, with `colour_of` choosing each body's colour from
    /// its own.
    pub fn draw_with(&self, out: &mut DrawList, colour_of: impl Fn(BodyId, Colour) -> Colour) {
        for entry in &self.entries {
            out.solids.push(Solid {
                visual: entry.visual,
                position: position_f32(entry.position),
                rotation: <[f32; 4]>::from(entry.rotation),
                colour: colour_of(entry.body, entry.colour),
            });
        }
    }

    /// Draws every body at its cached pose in its own colour.
    pub fn draw(&self, out: &mut DrawList) {
        self.draw_with(out, |_, colour| colour);
    }

    /// Folds every tracked body's raw id and cached pose into `digest`.
    pub fn write_state(&self, digest: &mut Digest) {
        digest.u64(self.entries.len() as u64);
        for entry in &self.entries {
            digest.u32(entry.body.to_raw());
            digest.rvec3(entry.position);
            digest.quat(entry.rotation);
        }
    }
}
