//! Constraint impulse readouts in the saved state: a restore brings back every constraint's
//! readouts and enabled flag, and a replay from it repeats them bit for bit, also after a detour
//! that changed loads, a motor target, an enabled flag, gravity and the step length.
//!
//! That a removed constraint makes earlier states unrestorable is checked by
//! `restore_after_a_structural_change_is_refused_and_changes_nothing` in `state.rs`.

mod common;

use std::collections::BTreeSet;

use common::constraint_rigs::{
    all_kinds_rig, assert_every_getter_loaded, is_enabled, readout_bits,
};
use common::{record_body, world, DT};
use oxijolt::*;

const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);
/// Ticks before the save: twice the ticks after which every readout of the rig carries a load.
const BEFORE_SAVE: usize = 60;
/// Ticks recorded after the save and replayed after the restore.
const REPLAY: usize = 60;
/// Ticks of the abandoned detour.
const DETOUR: usize = 40;

/// The world of [`all_kinds_rig`] and what it made.
struct Rig {
    world: PhysicsWorld,
    /// Every body, in raw id order.
    bodies: Vec<BodyId>,
    constraints: Vec<AnyConstraintId>,
}

impl Rig {
    fn new() -> Self {
        let mut world = world(GRAVITY, 1);
        let constraints = all_kinds_rig(&mut world);
        let mut bodies: Vec<BodyId> = world.body_ids().collect();
        bodies.sort_by_key(|id| id.to_raw());
        Self {
            world,
            bodies,
            constraints,
        }
    }

    /// The first constraint of `kind`.
    fn first(&self, kind: ConstraintType) -> AnyConstraintId {
        *self
            .constraints
            .iter()
            .find(|id| id.kind() == kind)
            .unwrap()
    }

    fn weld(&self) -> ConstraintId<FixedConstraint> {
        self.first(ConstraintType::Fixed).downcast().unwrap()
    }

    /// Every body's state, then every constraint's enabled flag and readout bits.
    fn snapshot(&self) -> Vec<u8> {
        let mut bytes = Vec::new();
        for &id in &self.bodies {
            record_body(&self.world, id, &mut bytes);
        }
        for &id in &self.constraints {
            bytes.push(u8::from(is_enabled(&self.world, id)));
            for bits in readout_bits(&self.world, id) {
                bytes.extend_from_slice(&bits.to_le_bytes());
            }
        }
        bytes
    }

    /// Steps `ticks` times with `dt` and returns the snapshot after each step.
    fn run(&mut self, dt: f32, ticks: usize) -> Vec<Vec<u8>> {
        (0..ticks)
            .map(|_| {
                assert!(self.world.step(dt).unwrap().is_complete());
                self.snapshot()
            })
            .collect()
    }

    /// A run the restore abandons: the weld disabled, the hinge motor's target changed,
    /// another gravity, half the step rate and an impulse on a constrained body every tick.
    fn detour(&mut self) {
        let weld = self.weld();
        self.world.constraint_mut(weld).unwrap().set_enabled(false);
        let hinge = self.first(ConstraintType::Hinge).downcast().unwrap();
        self.world
            .constraint_mut::<HingeConstraint>(hinge)
            .unwrap()
            .set_target_angular_velocity(-3.0)
            .unwrap();
        self.world.set_gravity(Vec3::new(2.0, -5.0, 1.0)).unwrap();
        let dynamic: Vec<BodyId> = self
            .bodies
            .iter()
            .copied()
            .filter(|&id| self.world.body(id).unwrap().motion_type() == MotionType::Dynamic)
            .collect();
        for tick in 0..DETOUR {
            let body = dynamic[tick % dynamic.len()];
            let push = 0.5 * (1 + tick % 3) as f32;
            self.world
                .body_mut(body)
                .unwrap()
                .add_impulse(Vec3::new(push, 1.0, -push))
                .unwrap();
            assert!(self.world.step(2.0 * DT).unwrap().is_complete());
        }
    }
}

/// Panics, naming the first tick that differs, unless `replay` repeats `recording`.
#[track_caller]
fn assert_same_ticks(recording: &[Vec<u8>], replay: &[Vec<u8>]) {
    assert_eq!(recording.len(), replay.len());
    if let Some(tick) = (0..recording.len()).find(|&t| recording[t] != replay[t]) {
        panic!("the replay differs from the recording at tick {tick}");
    }
}

#[test]
fn readouts_restore_with_the_state_for_every_kind() {
    let mut rig = Rig::new();
    let kinds: BTreeSet<String> = rig
        .constraints
        .iter()
        .map(|id| format!("{:?}", id.kind()))
        .collect();
    assert_eq!(kinds.len(), 12, "every world constraint kind: {kinds:?}");
    rig.run(DT, BEFORE_SAVE);
    let saved = rig.world.save_state();
    let at_save = rig.snapshot();
    assert_every_getter_loaded(&rig.world, &rig.constraints, "at the save");
    let recording = rig.run(DT, REPLAY);

    rig.world.restore_state(&saved).unwrap();
    assert_eq!(rig.snapshot(), at_save, "first restore");
    rig.detour();
    assert_ne!(rig.snapshot(), at_save, "the detour changes the world");
    rig.world.restore_state(&saved).unwrap();
    assert_eq!(rig.world.gravity(), GRAVITY);
    assert!(rig.world.constraint(rig.weld()).unwrap().is_enabled());
    assert_eq!(rig.snapshot(), at_save, "restore after the detour");
    assert_same_ticks(&recording, &rig.run(DT, REPLAY));
}

#[test]
fn a_disabled_bond_saved_stays_disabled() {
    let mut rig = Rig::new();
    rig.run(DT, BEFORE_SAVE);
    let weld = rig.weld();
    rig.world.constraint_mut(weld).unwrap().set_enabled(false);
    rig.run(DT, 1);
    let saved = rig.world.save_state();
    let at_save = rig.snapshot();
    let recording = rig.run(DT, REPLAY);

    rig.world.constraint_mut(weld).unwrap().set_enabled(true);
    rig.run(DT, DETOUR);
    assert!(rig.world.constraint(weld).unwrap().is_enabled());
    rig.world.restore_state(&saved).unwrap();
    assert!(!rig.world.constraint(weld).unwrap().is_enabled());
    assert_eq!(rig.snapshot(), at_save);
    assert_same_ticks(&recording, &rig.run(DT, REPLAY));
}
