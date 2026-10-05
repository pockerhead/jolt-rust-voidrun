//! Contact-cache invalidation (`BodyMut::invalidate_contact_cache`): Jolt reuses a resting
//! pair's contacts without asking `ContactListener::contact_validate` again until the cache is
//! invalidated, and a pending invalidation is part of the world's saved state.

mod common;

use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::Arc;

use common::controls::fall_asleep;
use common::*;
use oxijolt::*;

const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);
/// User data of the bodies [`Switch`] may reject.
const SWITCHED: u64 = 7;

/// Rejects every pair with a [`SWITCHED`] body while `reject` is set, and counts its calls.
#[derive(Default)]
struct Switch {
    reject: AtomicBool,
    calls: AtomicU32,
}

impl Switch {
    fn set_reject(&self, value: bool) {
        self.reject.store(value, Ordering::Relaxed);
    }

    fn take_calls(&self) -> u32 {
        self.calls.swap(0, Ordering::Relaxed)
    }
}

impl ContactListener for Switch {
    fn contact_validate(&self, contact: &ContactCandidate) -> ValidateResult {
        self.calls.fetch_add(1, Ordering::Relaxed);
        if self.reject.load(Ordering::Relaxed) && contact.user_data.contains(&SWITCHED) {
            ValidateResult::RejectAllContactsForThisBodyPair
        } else {
            ValidateResult::AcceptAllContactsForThisBodyPair
        }
    }
}

/// A floor and a unit cube resting on it, with a [`Switch`] that accepts for now.
struct Scene {
    world: PhysicsWorld,
    floor: BodyId,
    cube: BodyId,
    switch: Arc<Switch>,
}

/// The scene after the cube settled; it never sleeps unless `may_sleep`.
fn resting_cube(threads: u32, may_sleep: bool) -> Scene {
    let mut world = world(GRAVITY, threads);
    let switch = Arc::new(Switch::default());
    world.set_contact_listener(Some(switch.clone()));
    let floor = add_floor(&mut world);
    let cube = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_dynamic()
                .position(RVec3::new(0.0, 0.5, 0.0))
                .user_data(SWITCHED)
                .allow_sleeping(may_sleep),
        )
        .unwrap();
    step(&mut world, 60);
    switch.take_calls();
    Scene {
        world,
        floor,
        cube,
        switch,
    }
}

fn invalidate(world: &mut PhysicsWorld, id: BodyId) {
    world.body_mut(id).unwrap().invalidate_contact_cache();
}

/// The cube's position after each of `ticks` steps.
fn trace(scene: &mut Scene, ticks: usize) -> Vec<RVec3> {
    (0..ticks)
        .map(|_| {
            step(&mut scene.world, 1);
            scene.world.body(scene.cube).unwrap().position()
        })
        .collect()
}

fn height(scene: &Scene) -> Real {
    scene.world.body(scene.cube).unwrap().position().y
}

/// How many contact-cache invalidations a state holds, from its `Debug` output.
fn pending_in(state: &WorldState) -> String {
    let debug = format!("{state:?}");
    let start = debug
        .find("cache_invalidations: ")
        .expect("listed by Debug");
    debug[start..]
        .split([',', ' ', '}'])
        .nth(1)
        .unwrap()
        .to_owned()
}

#[test]
fn a_cached_accepted_pair_is_not_validated_until_invalidated() {
    for threads in [1, 4] {
        let mut scene = resting_cube(threads, false);
        step(&mut scene.world, 10);
        assert_eq!(scene.switch.take_calls(), 0, "a resting pair is cached");
        invalidate(&mut scene.world, scene.cube);
        step(&mut scene.world, 1);
        assert!(
            scene.switch.take_calls() > 0,
            "the invalidated pair is asked"
        );

        scene.switch.set_reject(true);
        step(&mut scene.world, 30);
        assert!(height(&scene) > 0.45, "the cached contact holds the cube");
        invalidate(&mut scene.world, scene.cube);
        step(&mut scene.world, 30);
        assert!(height(&scene) < -0.5, "the rejected pair lets it fall");
    }
}

/// A static box overlapping a resting cube's side; while its pair is rejected, Jolt caches "no
/// contact" and keeps that until the cache is invalidated.
#[test]
fn a_cached_rejected_pair_stays_rejected_until_invalidated() {
    for threads in [1, 4] {
        let mut world = world(GRAVITY, threads);
        let switch = Arc::new(Switch::default());
        switch.set_reject(true);
        world.set_contact_listener(Some(switch.clone()));
        add_floor(&mut world);
        let cube = world
            .create_body(
                &cube_shape(),
                &BodySettings::new_dynamic()
                    .position(RVec3::new(0.0, 0.5, 0.0))
                    .allow_sleeping(false),
            )
            .unwrap();
        let ghost_shape = Shape::new_box(Vec3::new(0.2, 0.2, 0.2)).unwrap();
        let ghost = BodySettings::new_static()
            .position(RVec3::new(0.5, 0.5, 0.0))
            .user_data(SWITCHED);
        world.create_body(&ghost_shape, &ghost).unwrap();
        step(&mut world, 60);
        let rested = world.body(cube).unwrap().position();
        switch.set_reject(false);
        step(&mut world, 30);
        let still = world.body(cube).unwrap().position();
        assert!((still.x - rested.x).abs() < 1e-4, "{rested:?} {still:?}");
        invalidate(&mut world, cube);
        step(&mut world, 30);
        let pushed = world.body(cube).unwrap().position();
        assert!(
            pushed.x < rested.x - 0.1,
            "the accepted overlap pushes it out: {pushed:?}"
        );
    }
}

#[test]
fn an_invalidation_saved_before_the_step_is_replayed() {
    for threads in [1, 4] {
        let mut scene = resting_cube(threads, false);
        scene.switch.set_reject(true);
        invalidate(&mut scene.world, scene.cube);
        let saved = scene.world.save_state();
        let first = trace(&mut scene, 30);
        assert!(first[29].y < 0.0, "the cube falls");
        scene.world.restore_state(&saved).unwrap();
        assert_eq!(trace(&mut scene, 30), first);
    }
}

#[test]
fn an_invalidation_before_a_restore_does_not_reach_the_replay() {
    for threads in [1, 4] {
        let mut scene = resting_cube(threads, false);
        scene.switch.set_reject(true);
        let saved = scene.world.save_state();
        let first = trace(&mut scene, 30);
        assert!(first[29].y > 0.45, "the cached contact holds the cube");
        scene.world.restore_state(&saved).unwrap();
        invalidate(&mut scene.world, scene.cube);
        scene.world.restore_state(&saved).unwrap();
        assert_eq!(trace(&mut scene, 30), first);
    }
}

#[test]
fn a_detour_invalidation_is_dropped_by_restoring_a_clean_state() {
    for threads in [1, 4] {
        let mut scene = resting_cube(threads, false);
        scene.switch.set_reject(true);
        let saved = scene.world.save_state();
        let first = trace(&mut scene, 30);
        scene.world.restore_state(&saved).unwrap();
        invalidate(&mut scene.world, scene.cube);
        let detour = trace(&mut scene, 5);
        assert!(detour[4].y < first[4].y - 0.01, "the detour falls");
        scene.world.restore_state(&saved).unwrap();
        assert_eq!(trace(&mut scene, 30), first);
    }
}

#[test]
fn repeated_invalidations_are_one_request() {
    let mut scene = resting_cube(1, false);
    scene.switch.set_reject(true);
    let saved = scene.world.save_state();
    invalidate(&mut scene.world, scene.cube);
    let once = scene.world.save_state();
    assert_eq!(pending_in(&once), "1");
    let single = trace(&mut scene, 20);
    scene.world.restore_state(&saved).unwrap();
    for _ in 0..3 {
        invalidate(&mut scene.world, scene.cube);
    }
    assert_eq!(pending_in(&scene.world.save_state()), "1");
    assert_eq!(trace(&mut scene, 20), single);
}

#[test]
fn invalidating_a_sleeping_body_wakes_it_and_revalidates_its_pairs() {
    for threads in [1, 4] {
        let mut scene = resting_cube(threads, true);
        fall_asleep(&mut scene.world, scene.cube);
        scene.switch.set_reject(true);
        invalidate(&mut scene.world, scene.cube);
        assert!(scene.world.body(scene.cube).unwrap().is_active());
        step(&mut scene.world, 30);
        assert!(height(&scene) < 0.0, "revalidated and rejected");
    }
}

/// Only the static floor is invalidated while nothing is awake, so the steps are idle and leave
/// the request pending until a step simulates again.
#[test]
fn an_idle_step_keeps_the_request_pending() {
    let mut scene = resting_cube(1, true);
    fall_asleep(&mut scene.world, scene.cube);
    scene.switch.set_reject(true);
    invalidate(&mut scene.world, scene.floor);
    step(&mut scene.world, 10);
    assert!(!scene.world.body(scene.cube).unwrap().is_active());
    assert_eq!(pending_in(&scene.world.save_state()), "1");
    scene.world.body_mut(scene.cube).unwrap().activate();
    step(&mut scene.world, 1);
    assert_eq!(pending_in(&scene.world.save_state()), "0", "applied");
    step(&mut scene.world, 30);
    assert!(
        height(&scene) < 0.0,
        "the floor's pair was validated and rejected"
    );
}

#[test]
fn a_partial_state_saves_and_restores_the_requests() {
    let mut scene = resting_cube(4, false);
    scene.switch.set_reject(true);
    invalidate(&mut scene.world, scene.cube);
    let saved = scene.world.save_state_of(&[scene.cube]).unwrap();
    assert_eq!(pending_in(&saved), "1");
    let first = trace(&mut scene, 30);
    assert!(first[29].y < 0.0);
    scene.world.restore_state(&saved).unwrap();
    assert_eq!(trace(&mut scene, 30), first);
}

#[test]
fn a_removed_body_leaves_no_request() {
    let mut scene = resting_cube(1, false);
    invalidate(&mut scene.world, scene.cube);
    scene.world.remove_body(scene.cube).unwrap();
    assert_eq!(pending_in(&scene.world.save_state()), "0");
}

/// A state saved right after `set_shape` must replay with the new shape: the cube grows 0.1 m
/// downward into the floor, and Jolt's fresh contact pushes it out, in the run and in the replay.
/// The cached contact of the old shape would hold it where it was.
#[test]
fn a_state_saved_right_after_set_shape_replays_with_the_new_shape() {
    for threads in [1, 4] {
        let mut scene = resting_cube(threads, false);
        let taller = Shape::new_box(Vec3::new(0.5, 0.6, 0.5)).unwrap();
        scene
            .world
            .body_mut(scene.cube)
            .unwrap()
            .set_shape(&taller, None, Activation::Activate)
            .unwrap();
        let saved = scene.world.save_state();
        let first = trace(&mut scene, 30);
        assert!(
            first[29].y > 0.55,
            "pushed out of the floor: {}",
            first[29].y
        );
        scene.world.restore_state(&saved).unwrap();
        assert_eq!(trace(&mut scene, 30), first);
    }
}
