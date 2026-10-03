//! World state save and restore: a rollback replays bit for bit (1 and 4 worker threads, one
//! process and two), saved and unsaved configuration, structural changes refuse old states and
//! change nothing, and states of other worlds are refused.

mod common;

use common::determinism::{assert_same, child_request, digest_in_child, finish_child, Digest};
use common::ragdoll::{bind_pose, humanoid_settings, transformed_pose};
use common::vehicle::{add_car, car_world, record_vehicle, CarLayers, GRAVITY};
use common::*;
use joltphysics::*;

const X: Vec3 = Vec3::new(1.0, 0.0, 0.0);
const Y: Vec3 = Vec3::new(0.0, 1.0, 0.0);
/// Ticks before the save point.
const BEFORE_SAVE: usize = 30;
/// Ticks of each run after the save point.
const AFTER_SAVE: usize = 90;

/// A floor and the toppling stacks of [`build_stacks`], two cubes joined by a hinge whose
/// velocity motor turns the second, a car with fixed driver input under the vehicle gravity
/// set every tick, a walking character with an inner body, and a ragdoll driven to its bind
/// pose by its motors.
struct Scene {
    world: PhysicsWorld,
    layers: CarLayers,
    /// The bodies the test created itself, in creation order: the floor, the stack cubes, the
    /// two hinge cubes and the chassis.
    bodies: Vec<BodyId>,
    hinge: ConstraintId<HingeConstraint>,
    car: VehicleId,
    character: CharacterId,
    ragdoll: RagdollId,
}

impl Scene {
    fn new(worker_threads: u32) -> Self {
        let (mut world, layers) = car_world(GRAVITY, worker_threads);
        let mut bodies = build_stacks(&mut world);

        let first = add_cube(&mut world, RVec3::new(6.0, 0.5, -4.0));
        let second = add_cube(&mut world, RVec3::new(7.2, 0.5, -4.0));
        bodies.extend([first, second]);
        let hinge = world
            .create_constraint(
                first,
                second,
                &HingeConstraintSettings::new(RVec3::new(6.6, 0.5, -4.0), X, Y),
            )
            .unwrap();
        let mut motor = world.constraint_mut(hinge).unwrap();
        motor.set_target_angular_velocity(1.0).unwrap();
        motor.set_motor_state(MotorState::Velocity);

        let (chassis, car) = add_car(
            &mut world,
            &layers,
            RVec3::new(-6.0, 1.0, -6.0),
            Quat::IDENTITY,
        );
        bodies.push(chassis);

        let capsule = Shape::new_capsule(0.7, 0.4).unwrap();
        let settings = CharacterSettings::new(&capsule)
            .shape_offset(Vec3::new(0.0, 1.1, 0.0))
            .inner_body(Some(InnerBody {
                shape: &capsule,
                object_layer: layers.moving,
            }));
        let character = world
            .create_character(&settings, RVec3::new(4.0, 0.0, 6.0), Quat::IDENTITY)
            .unwrap();
        world
            .refresh_character_contacts(character, &QueryFilter::new())
            .unwrap();

        let pose = transformed_pose(&bind_pose(), Quat::IDENTITY, [-6.0, 1.2, 6.0]);
        let ragdoll = world
            .create_ragdoll(
                &humanoid_settings(layers.moving),
                Some(&pose),
                Activation::Activate,
            )
            .unwrap();
        world
            .ragdoll_mut(ragdoll)
            .unwrap()
            .drive_to_pose_using_motors(&pose)
            .unwrap();

        Self {
            world,
            layers,
            bodies,
            hinge,
            car,
            character,
            ragdoll,
        }
    }

    /// Every body of the world: the test's own, the character's inner body and the ragdoll
    /// parts.
    fn all_bodies(&self) -> Vec<BodyId> {
        let mut ids = self.bodies.clone();
        ids.extend(self.world.character(self.character).unwrap().inner_body());
        ids.extend_from_slice(self.world.ragdoll(self.ragdoll).unwrap().body_ids());
        ids
    }

    /// One tick of the caller's inputs and a step.
    fn tick(&mut self) {
        let mut car = self.world.vehicle_mut(self.car).unwrap();
        car.set_gravity(GRAVITY).unwrap();
        car.set_driver_input(DriverInput {
            forward: 0.6,
            right: 0.3,
            ..DriverInput::default()
        })
        .unwrap();
        self.world
            .character_mut(self.character)
            .unwrap()
            .set_linear_velocity(Vec3::new(0.5, -1.0, 0.25))
            .unwrap();
        self.world
            .update_character(
                self.character,
                DT,
                GRAVITY,
                &ExtendedUpdateSettings::default(),
                &QueryFilter::new(),
            )
            .unwrap();
        step(&mut self.world, 1);
    }

    fn run(&mut self, ticks: usize) {
        for _ in 0..ticks {
            self.tick();
        }
    }

    /// The state after this tick: every body as [`record_body`] writes it, the character's
    /// saved state and the vehicle with its wheels.
    fn record(&self, digest: &mut Digest) {
        let tick = digest.push();
        for id in self.all_bodies() {
            record_body(&self.world, id, &mut tick.state);
        }
        let character = self.world.character(self.character).unwrap().save_state();
        tick.state.extend(character.as_bytes());
        record_vehicle(&self.world, self.car, &mut tick.state);
    }

    /// Runs `ticks` ticks and records each.
    fn recorded_run(&mut self, ticks: usize) -> Digest {
        let mut digest = Digest::new();
        for _ in 0..ticks {
            self.tick();
            self.record(&mut digest);
        }
        digest
    }

    /// What the world reports now: every body as [`record_body`] writes it and the
    /// character's saved state. A [`WorldState`] has no bytes to compare, so tests compare this.
    /// The wheel contacts are left out: a restore empties them until the next step. A removed
    /// character or ragdoll is skipped.
    fn snapshot(&self) -> Vec<u8> {
        let character = self.world.character(self.character).ok();
        let mut ids = self.bodies.clone();
        ids.extend(
            character
                .as_ref()
                .and_then(|character| character.inner_body()),
        );
        if let Ok(ragdoll) = self.world.ragdoll(self.ragdoll) {
            ids.extend_from_slice(ragdoll.body_ids());
        }
        let mut bytes = Vec::new();
        for id in ids {
            record_body(&self.world, id, &mut bytes);
        }
        if let Some(character) = character {
            bytes.extend(character.save_state().as_bytes());
        }
        bytes
    }

    /// Saves the world's state together with its [`snapshot`](Self::snapshot).
    fn save(&self) -> Saved {
        Saved {
            state: self.world.save_state(),
            snapshot: self.snapshot(),
        }
    }

    /// Restores `saved` and checks that the world reports what it reported at the save.
    fn restore(&mut self, saved: &Saved) {
        self.world.restore_state(&saved.state).unwrap();
        assert!(
            self.snapshot() == saved.snapshot,
            "a restore brings back what was saved"
        );
    }
}

/// A saved state and what the world reported when it was saved.
struct Saved {
    state: WorldState,
    snapshot: Vec<u8>,
}

/// A rollback: the run after the save point and the replay after restoring the save.
struct Rollback {
    first: Digest,
    replay: Digest,
}

fn rollback(worker_threads: u32) -> Rollback {
    let mut scene = Scene::new(worker_threads);
    scene.run(BEFORE_SAVE);
    let saved = scene.save();
    let first = scene.recorded_run(AFTER_SAVE);
    scene.restore(&saved);
    let replay = scene.recorded_run(AFTER_SAVE);
    Rollback { first, replay }
}

#[test]
fn rollback_replays_every_tick_bit_for_bit() {
    let runs = [1, 4].map(rollback);
    for (run, threads) in runs.iter().zip([1, 4]) {
        assert_same(
            &format!("replay with {threads} workers"),
            &run.first,
            &run.replay,
        );
    }
    assert_same("1 vs 4 workers", &runs[0].first, &runs[1].first);
    // The scene moves: the run is not trivially equal to itself.
    assert_ne!(runs[0].first.ticks[0], runs[0].first.ticks[AFTER_SAVE - 1]);
}

#[test]
#[ignore = "child process of the rollback gate"]
fn state_child() {
    let Some((scenario, threads, _)) = child_request() else {
        return;
    };
    assert_eq!(scenario, "rollback", "unknown scenario");
    let run = rollback(threads);
    assert_same("replay in the child", &run.first, &run.replay);
    finish_child(&run.replay);
}

#[test]
fn rollback_replay_is_identical_across_processes() {
    let one = digest_in_child("state_child", "rollback", 1, "");
    let four = digest_in_child("state_child", "rollback", 4, "");
    assert_eq!(one.ticks.len(), AFTER_SAVE);
    assert_same("1 vs 4 workers in two processes", &one, &four);
}

// Jolt runs no sleep test on kinematic bodies, so a character's inner body never falls asleep on
// its own; the test checks that the inner body and the character come back after the character
// was moved.
#[test]
fn a_moved_character_and_its_inner_body_restore() {
    let (mut world, layers) = car_world(GRAVITY, 1);
    add_floor(&mut world);
    let capsule = Shape::new_capsule(0.7, 0.4).unwrap();
    let settings = CharacterSettings::new(&capsule)
        .shape_offset(Vec3::new(0.0, 1.1, 0.0))
        .inner_body(Some(InnerBody {
            shape: &capsule,
            object_layer: layers.moving,
        }));
    let character = world
        .create_character(&settings, RVec3::ZERO, Quat::IDENTITY)
        .unwrap();
    let inner = world.character(character).unwrap().inner_body().unwrap();
    step(&mut world, 60);

    let saved = world.save_state();
    let inner_saved = body_bits(&world, inner);
    let character_saved = world.character(character).unwrap().save_state();
    world
        .character_mut(character)
        .unwrap()
        .set_position(RVec3::new(1.0, 0.0, 0.0))
        .unwrap();
    step(&mut world, 1);
    world.restore_state(&saved).unwrap();
    assert_eq!(body_bits(&world, inner), inner_saved);
    assert_eq!(
        world.character(character).unwrap().save_state(),
        character_saved
    );
}

#[test]
fn a_state_saved_before_the_first_step_restores() {
    let mut scene = Scene::new(1);
    let saved = scene.save();
    let first = scene.recorded_run(AFTER_SAVE);
    scene.restore(&saved);
    let replay = scene.recorded_run(AFTER_SAVE);
    assert_same("replay from before the first step", &first, &replay);
}

#[test]
fn a_state_restores_any_number_of_times() {
    let mut scene = Scene::new(1);
    scene.run(BEFORE_SAVE);
    let saved = scene.save();
    let first = scene.recorded_run(20);
    for _ in 0..3 {
        scene.restore(&saved);
        let replay = scene.recorded_run(20);
        assert_same("repeated replay", &first, &replay);
    }
}

#[test]
fn restore_undoes_saved_constraint_targets() {
    let mut scene = Scene::new(1);
    scene.run(BEFORE_SAVE);
    let saved = scene.save();
    let mut motor = scene.world.constraint_mut(scene.hinge).unwrap();
    motor.set_target_angle(0.7).unwrap();
    motor.set_target_angular_velocity(-2.0).unwrap();
    motor.set_motor_state(MotorState::Position);
    scene.run(5);

    scene.restore(&saved);
    let hinge = scene.world.constraint(scene.hinge).unwrap();
    assert_eq!(hinge.motor_state(), MotorState::Velocity);
    assert_eq!(hinge.target_angular_velocity(), 1.0);
    assert_eq!(hinge.target_angle(), 0.0);
}

#[test]
fn restore_does_not_undo_unsaved_configuration() {
    let mut scene = Scene::new(1);
    scene.run(BEFORE_SAVE);
    let saved = scene.save();
    let baseline = scene.recorded_run(AFTER_SAVE);
    scene.restore(&saved);

    let limits = scene.world.constraint(scene.hinge).unwrap().limits();
    scene
        .world
        .constraint_mut(scene.hinge)
        .unwrap()
        .set_limits(-0.5, 0.5)
        .unwrap();
    let low_gravity = Vec3::new(0.0, -5.0, 0.0);
    let mut car = scene.world.vehicle_mut(scene.car).unwrap();
    car.set_gravity(low_gravity).unwrap();
    step(&mut scene.world, 1);

    scene.restore(&saved);
    assert_eq!(
        scene.world.constraint(scene.hinge).unwrap().limits(),
        (-0.5, 0.5)
    );
    assert_eq!(
        scene.world.vehicle(scene.car).unwrap().gravity(),
        Some(low_gravity)
    );

    // Set back by the caller (the vehicle gravity by every tick), the replay is exact.
    scene
        .world
        .constraint_mut(scene.hinge)
        .unwrap()
        .set_limits(limits.0, limits.1)
        .unwrap();
    let replay = scene.recorded_run(AFTER_SAVE);
    assert_same(
        "replay after setting the configuration back",
        &baseline,
        &replay,
    );
}

/// Saves after a few ticks, prepares with `prepare` before the save, steps, makes the change
/// with `change`, and checks that the old state is refused and the refusal changed nothing.
fn assert_refused_after<T>(
    what: &str,
    prepare: impl FnOnce(&mut Scene) -> T,
    change: impl FnOnce(&mut Scene, T),
) {
    let mut scene = Scene::new(1);
    scene.run(5);
    let prepared = prepare(&mut scene);
    let saved = scene.world.save_state();
    scene.tick();
    change(&mut scene, prepared);
    let after = scene.snapshot();
    assert_eq!(
        scene.world.restore_state(&saved),
        Err(StateError::WorldChanged),
        "{what}"
    );
    assert!(
        scene.snapshot() == after,
        "{what}: the refusal changed the world"
    );
}

fn cube_shape() -> Shape {
    Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap()
}

fn extra_cube(scene: &mut Scene, x: Real) -> BodyId {
    let id = add_cube(&mut scene.world, RVec3::new(x, 0.5, 9.0));
    scene.bodies.push(id);
    id
}

#[test]
fn restore_after_a_structural_change_is_refused_and_changes_nothing() {
    assert_refused_after(
        "create body",
        |_| (),
        |scene, ()| {
            extra_cube(scene, 9.0);
        },
    );
    assert_refused_after(
        "remove body",
        |_| (),
        |scene, ()| {
            let cube = scene.bodies.remove(1);
            scene.world.remove_body(cube).unwrap();
        },
    );
    assert_refused_after(
        "create then remove a body",
        |_| (),
        |scene, ()| {
            let count = scene.world.body_count();
            let cube = extra_cube(scene, 9.0);
            scene.world.remove_body(cube).unwrap();
            scene.bodies.pop();
            assert_eq!(scene.world.body_count(), count);
        },
    );
    assert_refused_after(
        "create constraint",
        |_| (),
        |scene, ()| {
            let (a, b) = (scene.bodies[1], scene.bodies[5]);
            let point = scene.world.body(a).unwrap().position();
            scene
                .world
                .create_constraint(a, b, &PointConstraintSettings::new(point))
                .unwrap();
        },
    );
    assert_refused_after(
        "remove constraint",
        |_| (),
        |scene, ()| scene.world.remove_constraint(scene.hinge).unwrap(),
    );
    assert_refused_after(
        "create character",
        |_| (),
        |scene, ()| {
            let capsule = Shape::new_capsule(0.7, 0.4).unwrap();
            scene
                .world
                .create_character(
                    &CharacterSettings::new(&capsule),
                    RVec3::new(9.0, 0.0, 9.0),
                    Quat::IDENTITY,
                )
                .unwrap();
        },
    );
    assert_refused_after(
        "remove character",
        |_| (),
        |scene, ()| scene.world.remove_character(scene.character).unwrap(),
    );
    assert_refused_after(
        "create vehicle",
        |scene| {
            let chassis = scene
                .world
                .create_body(
                    &common::vehicle::chassis_shape(),
                    &common::vehicle::chassis_settings(
                        &scene.layers,
                        RVec3::new(9.0, 1.0, -9.0),
                        Quat::IDENTITY,
                    ),
                )
                .unwrap();
            scene.bodies.push(chassis);
            chassis
        },
        |scene, chassis| {
            let settings =
                common::vehicle::car_settings(VehicleCollisionTester::ray(scene.layers.probe));
            scene.world.create_vehicle(chassis, &settings).unwrap();
        },
    );
    assert_refused_after(
        "remove vehicle",
        |_| (),
        |scene, ()| scene.world.remove_vehicle(scene.car).unwrap(),
    );
    assert_refused_after(
        "create ragdoll",
        |_| (),
        |scene, ()| {
            let pose = transformed_pose(&bind_pose(), Quat::IDENTITY, [9.0, 1.2, 0.0]);
            scene
                .world
                .create_ragdoll(
                    &humanoid_settings(scene.layers.moving),
                    Some(&pose),
                    Activation::Activate,
                )
                .unwrap();
        },
    );
    assert_refused_after(
        "remove ragdoll",
        |_| (),
        |scene, ()| scene.world.remove_ragdoll(scene.ragdoll).unwrap(),
    );
    assert_refused_after(
        "ragdoll motion type",
        |_| (),
        |scene, ()| {
            scene
                .world
                .ragdoll_mut(scene.ragdoll)
                .unwrap()
                .set_motion_type(MotionType::Kinematic, Activation::Activate)
                .unwrap()
        },
    );
    assert_refused_after(
        "rebase",
        |_| (),
        |scene, ()| {
            let bodies = scene.all_bodies();
            scene
                .world
                .rebase(&bodies, Quat::IDENTITY, RVec3::new(1.0, 0.0, 0.0))
                .unwrap();
        },
    );
    assert_refused_after(
        "rebase with a pulley",
        |scene| {
            let (a, b) = (extra_cube(scene, 9.0), extra_cube(scene, 11.0));
            let (pa, pb) = (RVec3::new(9.0, 0.5, 9.0), RVec3::new(11.0, 0.5, 9.0));
            let pulley = PulleyConstraintSettings::new(
                pa,
                RVec3::new(pa.x, pa.y + 2.0, pa.z),
                pb,
                RVec3::new(pb.x, pb.y + 2.0, pb.z),
            );
            scene.world.create_constraint(a, b, &pulley).unwrap();
        },
        |scene, ()| {
            let bodies = scene.all_bodies();
            scene
                .world
                .rebase(&bodies, Quat::IDENTITY, RVec3::new(0.0, 0.0, 1.0))
                .unwrap();
        },
    );
}

#[test]
fn a_noop_rebase_keeps_a_state_restorable() {
    let mut scene = Scene::new(1);
    scene.run(5);
    let saved = scene.save();
    scene.tick();
    let bodies = scene.all_bodies();
    scene
        .world
        .rebase(&bodies, Quat::IDENTITY, RVec3::ZERO)
        .unwrap();
    scene.restore(&saved);
}

#[test]
fn a_rejected_structural_call_keeps_a_state_restorable() {
    let mut scene = Scene::new(1);
    scene.run(5);
    let removed = extra_cube(&mut scene, 9.0);
    scene.world.remove_body(removed).unwrap();
    scene.bodies.pop();
    let saved = scene.save();
    scene.tick();

    assert_eq!(
        scene.world.remove_body(removed),
        Err(BodyError::NotFound(removed))
    );
    let cube = scene.bodies[1];
    let point = PointConstraintSettings::new(RVec3::ZERO);
    assert!(matches!(
        scene.world.create_constraint(cube, cube, &point),
        Err(ConstraintError::InvalidValue(_))
    ));
    let inner = scene
        .world
        .character(scene.character)
        .unwrap()
        .inner_body()
        .unwrap();
    assert_eq!(
        scene.world.remove_body(inner),
        Err(BodyError::OwnedByCharacter(inner))
    );
    let bodies = scene.all_bodies();
    assert!(scene
        .world
        .rebase(&bodies[1..], Quat::IDENTITY, RVec3::new(1.0, 0.0, 0.0))
        .is_err());
    assert!(scene
        .world
        .create_body(&cube_shape(), &BodySettings::new_dynamic().mass(-1.0))
        .is_err());
    scene.restore(&saved);
}

#[test]
fn a_state_of_another_world_is_refused() {
    let mut a = Scene::new(1);
    let mut b = Scene::new(1);
    a.run(5);
    b.run(5);
    let state = a.world.save_state();
    let before = b.snapshot();
    assert_eq!(b.world.restore_state(&state), Err(StateError::WrongWorld));
    assert!(b.snapshot() == before, "the refusal changed the world");
}

/// The structural calls of the id test at fixed ticks of a recorded run: a cube created at tick
/// 5, a stack cube removed at 10, a point constraint created at 12 and a second cube created at
/// 15, which takes the removed cube's index. Returns the digest and the first created, the
/// removed and the second created cube.
fn structural_run(scene: &mut Scene, ticks: usize) -> (Digest, [BodyId; 3]) {
    let mut digest = Digest::new();
    let mut ids = Vec::new();
    for tick in 0..ticks {
        match tick {
            5 => ids.push(extra_cube(scene, 9.0)),
            10 => {
                let cube = scene.bodies.remove(3);
                scene.world.remove_body(cube).unwrap();
                ids.push(cube);
            }
            12 => {
                let (a, b) = (scene.bodies[1], scene.bodies[2]);
                let point = scene.world.body(a).unwrap().position();
                scene
                    .world
                    .create_constraint(a, b, &PointConstraintSettings::new(point))
                    .unwrap();
            }
            15 => ids.push(extra_cube(scene, 11.0)),
            _ => {}
        }
        scene.tick();
        scene.record(&mut digest);
    }
    (digest, [ids[0], ids[1], ids[2]])
}

// This shows how ids are allocated after a restore the world accepted: the same calls as in the
// uninterrupted run give the same ids. It is not a rollback across creation or removal, which
// `restore_state` refuses.
#[test]
fn bodies_created_after_the_restore_point_get_the_original_ids() {
    let mut uninterrupted = Scene::new(1);
    uninterrupted.run(BEFORE_SAVE);
    let (expected, expected_ids) = structural_run(&mut uninterrupted, AFTER_SAVE);

    let mut rolled_back = Scene::new(1);
    rolled_back.run(BEFORE_SAVE);
    let saved = rolled_back.save();
    rolled_back.run(AFTER_SAVE);
    rolled_back.restore(&saved);
    let (replay, ids) = structural_run(&mut rolled_back, AFTER_SAVE);

    assert_same("structural calls after a restore", &expected, &replay);
    assert_eq!(ids.map(BodyId::to_raw), expected_ids.map(BodyId::to_raw));
    let [_, removed, reused] = ids;
    assert_eq!(reused.index(), removed.index());
    assert_ne!(reused.sequence(), removed.sequence());
}

#[test]
fn a_selected_bodies_state_replays_when_the_rest_is_static() {
    let mut scene = Scene::new(1);
    scene.run(BEFORE_SAVE);
    let floor = scene.bodies[0];
    assert_eq!(
        scene.world.body(floor).unwrap().motion_type(),
        MotionType::Static
    );
    let moving: Vec<BodyId> = scene.all_bodies()[1..].to_vec();
    let partial = scene.world.save_state_of(&moving).unwrap();

    let first = scene.recorded_run(AFTER_SAVE);
    scene.world.restore_state(&partial).unwrap();
    let replay = scene.recorded_run(AFTER_SAVE);
    assert_same("replay from a selected bodies state", &first, &replay);
}

fn body_bits(world: &PhysicsWorld, id: BodyId) -> Vec<u8> {
    let mut bits = Vec::new();
    record_body(world, id, &mut bits);
    bits
}

/// Runs the scene, saves the bodies `selection` makes of every body but one, runs on and
/// restores: the saved bodies come back, and the one left out, the second hinge cube that the
/// motor keeps turning, keeps its current state.
fn assert_left_out_body_keeps_its_state(selection: impl FnOnce(&[BodyId]) -> Vec<BodyId>) {
    let mut scene = Scene::new(1);
    scene.run(BEFORE_SAVE);
    let left_out = scene.bodies[10];
    let selected: Vec<BodyId> = scene
        .all_bodies()
        .into_iter()
        .filter(|&id| id != left_out)
        .collect();
    let partial = scene.world.save_state_of(&selection(&selected)).unwrap();
    let saved: Vec<Vec<u8>> = selected
        .iter()
        .map(|&id| body_bits(&scene.world, id))
        .collect();
    let left_out_at_save = body_bits(&scene.world, left_out);

    scene.run(20);
    let current = body_bits(&scene.world, left_out);
    assert_ne!(current, left_out_at_save);
    scene.world.restore_state(&partial).unwrap();
    assert_eq!(body_bits(&scene.world, left_out), current);
    for (&id, saved) in selected.iter().zip(&saved) {
        assert_eq!(&body_bits(&scene.world, id), saved, "{id:?}");
    }
}

#[test]
fn bodies_left_out_of_a_state_keep_their_current_state() {
    assert_left_out_body_keeps_its_state(<[BodyId]>::to_vec);
}

#[test]
fn a_selection_in_any_order_with_duplicates_saves_those_bodies() {
    assert_left_out_body_keeps_its_state(|selected| {
        let mut shuffled: Vec<BodyId> = selected.iter().rev().copied().collect();
        shuffled.extend_from_slice(&selected[..3]);
        shuffled.extend_from_slice(&selected[..3]);
        shuffled
    });
}

#[test]
fn an_empty_selection_saves_no_body() {
    let mut scene = Scene::new(1);
    scene.run(BEFORE_SAVE);
    let empty = scene.world.save_state_of(&[]).unwrap();
    scene.run(20);
    // The character's inner body is left out: the restored character moves it to its pose.
    let inner = scene
        .world
        .character(scene.character)
        .unwrap()
        .inner_body()
        .unwrap();
    let bodies: Vec<BodyId> = scene
        .all_bodies()
        .into_iter()
        .filter(|&id| id != inner)
        .collect();
    let current: Vec<Vec<u8>> = bodies
        .iter()
        .map(|&id| body_bits(&scene.world, id))
        .collect();
    scene.world.restore_state(&empty).unwrap();
    for (&id, current) in bodies.iter().zip(&current) {
        assert_eq!(&body_bits(&scene.world, id), current, "{id:?}");
    }
}

#[test]
fn a_selection_with_an_unknown_body_is_refused() {
    let mut scene = Scene::new(1);
    let removed = extra_cube(&mut scene, 9.0);
    scene.world.remove_body(removed).unwrap();
    assert_eq!(
        scene
            .world
            .save_state_of(&[scene.bodies[1], removed])
            .unwrap_err(),
        BodyError::NotFound(removed)
    );
    let other = Scene::new(1);
    let foreign = other.bodies[1];
    assert_eq!(
        scene.world.save_state_of(&[foreign]).unwrap_err(),
        BodyError::WrongWorld(foreign)
    );
}
