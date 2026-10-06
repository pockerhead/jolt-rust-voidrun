//! A filtered restore of a cloth and a ragdoll's parts: the selected cloth and parts go back to the
//! save, while an unselected cloth, a body with a force and a torque added since the save and a
//! body woken since keep their state and step on exactly as in a world that was not restored.
//! Nothing unselected touches another body, so the contacts a restore brings back from the save
//! do not matter here.

mod common;

use common::ragdoll::{bind_pose, humanoid_settings, ragdoll_layers, transformed_pose};
use common::soft_body::Cloth;
use common::{add_cube, record_body, step};
use oxijolt::*;

struct Scene {
    world: PhysicsWorld,
    cloth: BodyId,
    ragdoll: RagdollId,
    other_cloth: BodyId,
    pushed: BodyId,
    woken: BodyId,
}

/// A pinned cloth swinging at `x`.
fn add_cloth(world: &mut PhysicsWorld, x: Real) -> BodyId {
    let cloth = Cloth::new(6, 0.2);
    let pins = cloth.first_row_corners();
    let at = RVec3::new(x, 40.0, 0.0);
    world
        .create_soft_body(
            &cloth.pin(&pins).settings(),
            &SoftBodySettings::default().position(at),
        )
        .unwrap()
}

/// Every object high above the floor and apart, so that none touches another for the test.
fn scene() -> Scene {
    let (layers, ids) = ragdoll_layers();
    let mut world = PhysicsWorld::new(WorldSettings::default().layers(layers)).unwrap();
    let cloth = add_cloth(&mut world, 0.0);
    let pose = transformed_pose(&bind_pose(), Quat::IDENTITY, [8.0, 40.0, 0.0]);
    let ragdoll = world
        .create_ragdoll(
            &humanoid_settings(ids.ragdoll),
            Some(&pose),
            Activation::Activate,
        )
        .unwrap();
    let other_cloth = add_cloth(&mut world, -8.0);
    let pushed = add_cube(&mut world, RVec3::new(-16.0, 40.0, 0.0));
    let shape = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
    let asleep = BodySettings::new_dynamic()
        .position(RVec3::new(16.0, 40.0, 0.0))
        .activation(Activation::DontActivate);
    let woken = world.create_body(&shape, &asleep).unwrap();
    Scene {
        world,
        cloth,
        ragdoll,
        other_cloth,
        pushed,
        woken,
    }
}

impl Scene {
    fn parts(&self) -> Vec<BodyId> {
        self.world
            .ragdoll(self.ragdoll)
            .unwrap()
            .body_ids()
            .to_vec()
    }

    /// The bits of `bodies` and the vertex states of `cloth`.
    fn bits(&self, bodies: &[BodyId], cloth: BodyId) -> (Vec<u8>, Vec<SoftBodyVertexState>) {
        let mut bits = Vec::new();
        for &id in bodies {
            record_body(&self.world, id, &mut bits);
        }
        (bits, self.world.soft_body(cloth).unwrap().vertices())
    }

    fn unselected(&self) -> (Vec<u8>, Vec<SoftBodyVertexState>) {
        self.bits(
            &[self.other_cloth, self.pushed, self.woken],
            self.other_cloth,
        )
    }

    /// What happens after the save in both worlds: a push on the ragdoll, a force and a torque on
    /// one cube, the other woken.
    fn move_on(&mut self) {
        step(&mut self.world, 20);
        let part = self.parts()[0];
        let mut body = self.world.body_mut(part).unwrap();
        body.add_impulse(Vec3::new(0.0, 0.0, 30.0)).unwrap();
        let mut body = self.world.body_mut(self.pushed).unwrap();
        body.add_force(Vec3::new(400.0, 0.0, 0.0)).unwrap();
        body.add_torque(Vec3::new(0.0, 60.0, 0.0)).unwrap();
        self.world.body_mut(self.woken).unwrap().activate();
        step(&mut self.world, 5);
        let mut body = self.world.body_mut(self.pushed).unwrap();
        body.add_force(Vec3::new(0.0, 0.0, 300.0)).unwrap();
        body.add_torque(Vec3::new(50.0, 0.0, 0.0)).unwrap();
    }
}

#[test]
fn a_filtered_restore_keeps_unselected_cloths_forces_and_wakes() {
    let (mut control, mut restored) = (scene(), scene());
    step(&mut control.world, 30);
    step(&mut restored.world, 30);
    let mut selected = restored.parts();
    selected.push(restored.cloth);
    let saved = restored.world.save_state();
    let at_save = restored.bits(&selected, restored.cloth);
    assert!(restored.world.body(restored.woken).unwrap().is_sleeping());

    control.move_on();
    restored.move_on();
    assert_ne!(restored.bits(&selected, restored.cloth), at_save);
    restored
        .world
        .restore_state_of(&saved, BodySelection::Only(&selected))
        .unwrap();

    assert!(restored.bits(&selected, restored.cloth) == at_save);
    assert!(restored.unselected() == control.unselected());
    assert!(!restored.world.body(restored.woken).unwrap().is_sleeping());
    let pushed = |scene: &Scene| {
        let body = scene.world.body(scene.pushed).unwrap();
        (body.linear_velocity(), body.angular_velocity())
    };
    let before = pushed(&control);
    for tick in 0..30 {
        step(&mut control.world, 1);
        step(&mut restored.world, 1);
        assert!(
            restored.unselected() == control.unselected(),
            "the unselected objects differ at tick {tick}"
        );
    }
    // The force and torque pending at the restore were applied.
    let after = pushed(&control);
    assert!(
        after.0.z > before.0.z && after.1.x > before.1.x,
        "{before:?} {after:?}"
    );
}
