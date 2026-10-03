//! A leak gate for events, contact listeners and materials: in one world that records every
//! event, each round replaces the event settings and the contact listener ten times, makes
//! shapes of new materials, creates bodies and a cloth from them, steps, takes the events and
//! removes the bodies again, against a budget of 100 bytes per round.
//!
//! It measures the private bytes of the process, because the native listeners and materials are
//! allocated by C++, which a Rust global allocator does not see. The world lives through all
//! rounds: creating a world per round alone grew the counter by about 560 bytes per round. The
//! file holds exactly one test, so its binary runs alone and no parallel test disturbs the
//! counter. Locally the counter grew by 120 to 150 KB in total, the same for 2 000 and 6 000
//! rounds. The gate was checked once against local builds that never released a material
//! (about 450 bytes per round) and that never destroyed the native contact listener (about
//! 840 bytes per round); each exceeded the budget.
#![cfg(windows)]

mod common;

use std::sync::Arc;

use common::events::*;
use common::memory::private_bytes;
use common::*;
use joltphysics::*;

const WARM_UP_ROUNDS: usize = 500;
const MEASURED_ROUNDS: usize = 6_000;
/// 100 bytes per measured round.
const MAX_GROWTH: usize = 100 * MEASURED_ROUNDS;

fn every_kind() -> EventSettings {
    EventSettings::default()
        .persisted_contacts(true)
        .body_activation(true)
        .soft_body_contacts(true)
        .soft_body_validations(true)
}

fn listener_round(world: &mut PhysicsWorld, round: usize) {
    for i in 0..10 {
        let settings = if i % 2 == 0 {
            EventSettings::default()
        } else {
            every_kind()
        };
        world.set_event_settings(settings);
        world.set_contact_listener(Some(Arc::new(NoOp)));
    }
    world.set_event_settings(every_kind());
    world.set_contact_listener(Some(Arc::new(RoughContacts)));
    let materials: Vec<PhysicsMaterial> = (0..4)
        .map(|i| PhysicsMaterial::new(round as u64 * 4 + i).unwrap())
        .collect();
    let shapes = [
        Shape::new_box_with_material(Vec3::new(0.25, 0.25, 0.25), 0.05, &materials[0]).unwrap(),
        Shape::new_sphere_with_material(0.25, &materials[1]).unwrap(),
        Shape::new_capsule_with_material(0.2, 0.2, &materials[2]).unwrap(),
        Shape::new_cylinder_with_material(0.25, 0.25, 0.05, &materials[3]).unwrap(),
    ];
    drop(materials);
    let mut ids: Vec<BodyId> = shapes
        .iter()
        .enumerate()
        .map(|(i, shape)| {
            let position = RVec3::new(-1.5 + i as Real, 0.3, 0.0);
            world
                .create_body(shape, &BodySettings::new_dynamic().position(position))
                .unwrap()
        })
        .collect();
    drop(shapes);
    ids.push(add_cloth(world, RVec3::new(10.0, 0.3, 0.0), Quat::IDENTITY));
    for _ in 0..5 {
        step(world, 1);
        assert!(!world.take_events().is_empty());
    }
    for id in ids {
        world.remove_body(id).unwrap();
    }
    step(world, 1);
    world.take_events();
}

#[test]
fn listeners_and_materials_do_not_leak() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    add_two_box_floor(&mut world);
    add_table(&mut world, 10.0);
    for round in 0..WARM_UP_ROUNDS {
        listener_round(&mut world, round);
    }
    let before = private_bytes();
    for round in 0..MEASURED_ROUNDS {
        listener_round(&mut world, round);
    }
    let after = private_bytes();
    let growth = after.saturating_sub(before);
    assert!(
        growth < MAX_GROWTH,
        "private bytes grew by {growth} over {MEASURED_ROUNDS} rounds"
    );
}
