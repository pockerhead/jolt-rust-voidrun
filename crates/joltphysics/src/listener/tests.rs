//! Panic containment of the event callbacks: a test-only switch makes a chosen callback panic
//! on Jolt's worker threads or on the calling thread.

use std::panic::{catch_unwind, AssertUnwindSafe};
use std::sync::atomic::Ordering;

use super::*;
use crate::{
    Activation, BodyId, BodySettings, CompoundChild, PhysicsWorld, Quat, RVec3, Shape,
    SoftBodyBendType, SoftBodySettings, SoftBodySharedSettings, SoftBodyVertex,
    SoftBodyVertexAttributes, Vec3, WorldSettings,
};

pub(super) const NO_PANIC: u8 = 0;
/// Every callback panics with a string payload.
const ALWAYS: u8 = 254;
/// Every callback panics with a payload whose `Drop` panics as well.
const ALWAYS_WITH_PANICKING_DROP: u8 = 255;

const DT: f32 = 1.0 / 60.0;

/// A panic payload whose drop panics again.
struct PanicsOnDrop;

impl Drop for PanicsOnDrop {
    fn drop(&mut self) {
        panic!("panic payload dropped");
    }
}

impl ListenerContext {
    /// Panics when the test switch names `callback` or every callback.
    pub(super) fn panic_if_asked(&self, callback: Callback) {
        match self.panic_in.load(Ordering::Relaxed) {
            ALWAYS_WITH_PANICKING_DROP => std::panic::panic_any(PanicsOnDrop),
            ALWAYS => panic!("listener test panic in every callback"),
            asked if asked == callback as u8 => panic!("listener test panic in {callback:?}"),
            _ => {}
        }
    }
}

fn set_panic(world: &PhysicsWorld, value: u8) {
    let context = world.listeners.context.as_ref().expect("events are on");
    context.panic_in.store(value, Ordering::Relaxed);
}

/// The bodies of the test scene.
struct Scene {
    cube: BodyId,
}

fn every_event() -> EventSettings {
    EventSettings::default()
        .persisted_contacts(true)
        .body_activation(true)
        .soft_body_contacts(true)
        .soft_body_validations(true)
}

/// Four workers; a cube that lands on a two-box compound floor, and a cloth that lands on a
/// table 10 m away.
fn scene() -> (PhysicsWorld, Scene) {
    let mut world = PhysicsWorld::new(WorldSettings::default().worker_threads(4)).unwrap();
    world.set_event_settings(every_event());
    let half = Shape::new_box(Vec3::new(1.0, 0.5, 1.0)).unwrap();
    let child = |x| CompoundChild {
        shape: &half,
        position: Vec3::new(x, 0.0, 0.0),
        rotation: Quat::IDENTITY,
        user_data: 0,
    };
    let floor = Shape::new_compound(&[child(-1.0), child(1.0)]).unwrap();
    let at = |x, y| BodySettings::new_static().position(RVec3::new(x, y, 0.0));
    world.create_body(&floor, &at(0.0, -0.5)).unwrap();
    world.create_body(&half, &at(10.0, -0.5)).unwrap();
    let cube_shape = Shape::new_box(Vec3::new(0.25, 0.25, 0.25)).unwrap();
    let cube = world
        .create_body(
            &cube_shape,
            &BodySettings::new_dynamic().position(RVec3::new(-1.0, 0.3, 0.0)),
        )
        .unwrap();
    let mut vertices = Vec::new();
    let mut faces = Vec::new();
    for z in 0..4 {
        for x in 0..4 {
            let position = Vec3::new(x as f32 * 0.25 - 0.375, 0.0, z as f32 * 0.25 - 0.375);
            vertices.push(SoftBodyVertex::new(position));
            if x < 3 && z < 3 {
                let v = z * 4 + x;
                faces.push([v, v + 4, v + 5]);
                faces.push([v, v + 5, v + 1]);
            }
        }
    }
    let cloth = SoftBodySharedSettings::builder(vertices, faces)
        .create_constraints(SoftBodyBendType::None, SoftBodyVertexAttributes::default())
        .build()
        .unwrap();
    world
        .create_soft_body(
            &cloth,
            &SoftBodySettings::default().position(RVec3::new(10.0, 0.3, 0.0)),
        )
        .unwrap();
    world.take_events();
    (world, Scene { cube })
}

fn message(payload: &(dyn Any + Send)) -> String {
    payload
        .downcast_ref::<String>()
        .cloned()
        .or_else(|| payload.downcast_ref::<&str>().map(|s| (*s).to_owned()))
        .unwrap_or_default()
}

/// Steps until a step panics, lifting the cube after a second so that its contact is
/// removed; returns the payload.
fn step_until_panic(world: &mut PhysicsWorld, scene: &Scene) -> Box<dyn Any + Send> {
    for tick in 0..900 {
        if tick == 60 {
            let mut cube = world.body_mut(scene.cube).unwrap();
            cube.set_position(RVec3::new(-1.0, 3.0, 0.0), Activation::Activate)
                .unwrap();
        }
        if let Err(payload) = catch_unwind(AssertUnwindSafe(|| world.step(DT))) {
            return payload;
        }
    }
    panic!("no step panicked");
}

/// After a resumed panic the world steps, records and removes bodies as before.
fn assert_world_still_works(world: &mut PhysicsWorld, scene: &Scene) {
    set_panic(world, NO_PANIC);
    for _ in 0..5 {
        assert!(world.step(DT).unwrap().is_complete());
    }
    world.take_events();
    world.remove_body(scene.cube).unwrap();
    assert!(world.step(DT).unwrap().is_complete());
    let events = world.take_events();
    assert!(
        events
            .contacts
            .iter()
            .all(|event| event.pair().body1 != scene.cube
                || matches!(event, ContactEvent::Removed(_)))
    );
}

#[test]
fn a_panic_in_any_callback_during_a_step_is_resumed_by_step() {
    for callback in [
        Callback::ContactAdded,
        Callback::ContactPersisted,
        Callback::ContactRemoved,
        Callback::BodyDeactivated,
        Callback::SoftBodyValidate,
        Callback::SoftBodyAdded,
    ] {
        let (mut world, scene) = scene();
        set_panic(&world, callback as u8);
        let payload = step_until_panic(&mut world, &scene);
        assert_eq!(
            message(&*payload),
            format!("listener test panic in {callback:?}")
        );
        assert_world_still_works(&mut world, &scene);
    }
}

#[test]
fn a_panic_outside_a_step_is_resumed_by_take_events() {
    let (mut world, scene) = scene();
    set_panic(&world, Callback::BodyActivated as u8);
    let shape = Shape::new_sphere(0.2).unwrap();
    let ball = world
        .create_body(
            &shape,
            &BodySettings::new_dynamic().position(RVec3::new(5.0, 2.0, 0.0)),
        )
        .unwrap();
    set_panic(&world, NO_PANIC);
    let payload = catch_unwind(AssertUnwindSafe(|| world.take_events())).unwrap_err();
    assert_eq!(message(&*payload), "listener test panic in BodyActivated");
    world.remove_body(ball).unwrap();
    assert_eq!(
        world.take_events().activations,
        vec![ActivationEvent::Deactivated(ball)]
    );
    assert_world_still_works(&mut world, &scene);
}

#[test]
fn an_unresumed_panic_between_steps_is_resumed_by_the_next_step() {
    let (mut world, _) = scene();
    set_panic(&world, Callback::BodyActivated as u8);
    let shape = Shape::new_sphere(0.2).unwrap();
    world
        .create_body(
            &shape,
            &BodySettings::new_dynamic().position(RVec3::new(5.0, 2.0, 0.0)),
        )
        .unwrap();
    set_panic(&world, NO_PANIC);
    let payload = catch_unwind(AssertUnwindSafe(|| world.step(DT))).unwrap_err();
    assert_eq!(message(&*payload), "listener test panic in BodyActivated");
    assert!(world.step(DT).unwrap().is_complete());
}

#[test]
fn always_panicking_callbacks_are_resumed_every_step() {
    let (mut world, scene) = scene();
    set_panic(&world, ALWAYS);
    // The first steps report nothing until the bodies come close.
    step_until_panic(&mut world, &scene);
    // A push keeps the cube awake, so its contact is reported (and panics) every step.
    let pushed_step = |world: &mut PhysicsWorld| {
        let mut cube = world.body_mut(scene.cube).unwrap();
        cube.set_linear_velocity(Vec3::new(0.2, 0.0, 0.0)).unwrap();
        catch_unwind(AssertUnwindSafe(|| world.step(DT))).unwrap_err()
    };
    for _ in 0..60 {
        let payload = pushed_step(&mut world);
        assert_eq!(message(&*payload), "listener test panic in every callback");
    }
    set_panic(&world, ALWAYS_WITH_PANICKING_DROP);
    for _ in 0..60 {
        let payload = pushed_step(&mut world);
        assert!(payload.is::<PanicsOnDrop>());
        // Dropping it would panic in this test.
        std::mem::forget(payload);
    }
    assert_world_still_works(&mut world, &scene);
}

#[test]
fn replacing_the_settings_keeps_events_and_panics() {
    let (mut world, _) = scene();
    set_panic(&world, Callback::BodyActivated as u8);
    let shape = Shape::new_sphere(0.2).unwrap();
    let ball = world
        .create_body(
            &shape,
            &BodySettings::new_dynamic().position(RVec3::new(5.0, 2.0, 0.0)),
        )
        .unwrap();
    world.remove_body(ball).unwrap();
    world.set_event_settings(EventSettings::default());
    assert!(world.listeners.context.is_none(), "nothing is installed");
    let payload = catch_unwind(AssertUnwindSafe(|| world.take_events())).unwrap_err();
    assert_eq!(message(&*payload), "listener test panic in BodyActivated");
    let events = world.take_events();
    assert_eq!(events.activations, vec![ActivationEvent::Deactivated(ball)]);
    assert!(world.step(DT).unwrap().is_complete());
    assert!(world.take_events().is_empty());
}
