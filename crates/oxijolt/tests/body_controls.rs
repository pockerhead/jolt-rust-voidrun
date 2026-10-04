//! Body controls: locked axes (allowed degrees of freedom) and where they are refused, sensor
//! bodies, user data, impulses and kinematic moves.

mod common;

use std::sync::{Arc, Mutex};

use common::constraint_kinds::create_every_kind;
use common::events::{add_cloth, every_event, soft_contacts};
use common::ragdoll::{conj, humanoid_parts, mul, part_shapes, ragdoll_world, skeleton};
use common::vehicle::{add_car, car_settings, car_world, chassis_settings, chassis_shape};
use common::*;
use oxijolt::*;

const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);
const Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);

/// A dynamic unit cube at `position` with `dofs`.
fn add_locked_cube(world: &mut PhysicsWorld, position: RVec3, dofs: AllowedDofs) -> BodyId {
    world
        .create_body(
            &cube_shape(),
            &BodySettings::new_dynamic()
                .position(position)
                .allowed_dofs(dofs),
        )
        .unwrap()
}

#[test]
fn a_plane_2d_body_stays_in_its_plane() {
    let mut world = world(GRAVITY, 1);
    // A static slab tilted about a diagonal axis, so its normal leans along z as well as x.
    let slab = Shape::new_box(Vec3::new(5.0, 0.5, 5.0)).unwrap();
    let tilt = quat_about(Vec3::new(0.6, 0.0, 0.8), 0.4);
    world
        .create_body(
            &slab,
            &BodySettings::new_static()
                .position(RVec3::new(0.0, -1.0, 0.0))
                .rotation(tilt),
        )
        .unwrap();
    let cube = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_dynamic()
                .position(RVec3::new(0.5, 2.0, 0.0))
                .linear_velocity(Vec3::new(1.0, -2.0, 3.0))
                .angular_velocity(Vec3::new(2.0, -1.0, 0.5))
                .allowed_dofs(AllowedDofs::PLANE_2D),
        )
        .unwrap();
    let start = world.body(cube).unwrap().position();
    let mut turned = false;
    for _ in 0..120 {
        step(&mut world, 1);
        let body = world.body(cube).unwrap();
        let (p, q) = (body.position(), body.rotation());
        let (v, w) = (body.linear_velocity(), body.angular_velocity());
        assert_eq!(p.z, 0.0);
        assert_eq!((q.x, q.y), (0.0, 0.0));
        assert_eq!(v.z, 0.0);
        assert_eq!((w.x, w.y), (0.0, 0.0));
        turned |= q.z != 0.0;
    }
    let end = world.body(cube).unwrap().position();
    assert!(turned, "the cube never turned about z");
    assert!(end.x != start.x && end.y < start.y, "{end:?}");
}

#[test]
fn axes_stay_world_axes_for_a_rotated_body() {
    let mut world = world(Vec3::ZERO, 1);
    // The body's own z axis points along world -y.
    let start = quat_about(Vec3::new(1.0, 0.0, 0.0), std::f32::consts::FRAC_PI_2);
    let translations =
        AllowedDofs::TRANSLATION_X | AllowedDofs::TRANSLATION_Y | AllowedDofs::TRANSLATION_Z;
    let cube = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_dynamic()
                .rotation(start)
                .allowed_dofs(translations | AllowedDofs::ROTATION_Z),
        )
        .unwrap();
    world
        .body_mut(cube)
        .unwrap()
        .set_angular_velocity(Vec3::new(1.0, 1.0, 1.0))
        .unwrap();
    assert_eq!(
        world.body(cube).unwrap().angular_velocity(),
        Vec3::new(0.0, 0.0, 1.0)
    );
    step(&mut world, 30);
    // The turn since the start is about world z, not about the body's own z.
    let q = world.body(cube).unwrap().rotation();
    let turn = mul(q, conj(start));
    assert!(turn.x.abs() < 1.0e-6 && turn.y.abs() < 1.0e-6, "{turn:?}");
    assert!(turn.z.abs() > 0.2, "{turn:?}");
}

#[test]
fn allowed_dofs_read_back() {
    let mut world = world(Vec3::ZERO, 1);
    let plane = add_locked_cube(&mut world, RVec3::ZERO, AllowedDofs::PLANE_2D);
    let rail = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_kinematic()
                .position(RVec3::new(3.0, 0.0, 0.0))
                .allowed_dofs(AllowedDofs::TRANSLATION_Y),
        )
        .unwrap();
    let free = add_cube(&mut world, RVec3::new(6.0, 0.0, 0.0));
    let fixed = world
        .create_body(&cube_shape(), &BodySettings::new_static())
        .unwrap();
    let dofs = |id| world.body(id).unwrap().allowed_dofs();
    assert_eq!(dofs(plane), AllowedDofs::PLANE_2D);
    assert_eq!(dofs(rail), AllowedDofs::TRANSLATION_Y);
    assert_eq!(dofs(free), AllowedDofs::ALL);
    assert_eq!(dofs(fixed), AllowedDofs::ALL);
}

#[test]
fn a_static_body_that_cannot_move_refuses_restricted_dofs() {
    let mut world = world(Vec3::ZERO, 1);
    let refused = world.create_body(
        &cube_shape(),
        &BodySettings::new_static().allowed_dofs(AllowedDofs::PLANE_2D),
    );
    assert!(
        matches!(refused, Err(BodyError::InvalidValue(_))),
        "{refused:?}"
    );
    let without_translation = world.create_body(
        &cube_shape(),
        &BodySettings::new_dynamic().allowed_dofs(AllowedDofs::ROTATION_Y),
    );
    assert!(
        matches!(without_translation, Err(BodyError::InvalidValue(_))),
        "{without_translation:?}"
    );
    assert_eq!(world.body_count(), 0);
}

#[test]
fn bodies_with_restricted_dofs_are_refused_by_every_constraint_kind_vehicles_and_ragdolls() {
    let mut world = world(Vec3::ZERO, 1);
    let locked = add_locked_cube(&mut world, RVec3::ZERO, AllowedDofs::PLANE_2D);
    let free = add_cube(&mut world, RVec3::new(1.5, 0.0, 0.0));
    for (body1, body2) in [(locked, free), (free, locked)] {
        let results = create_every_kind(&mut world, body1, body2);
        assert_eq!(results.len(), 12);
        for result in results {
            assert_eq!(
                result,
                Err(ConstraintError::Body(BodyError::RestrictedDofs(locked)))
            );
        }
    }
    assert_eq!(world.constraint_count(), 0);
    // The same two bodies with all six degrees of freedom take every kind.
    let other = add_cube(&mut world, RVec3::new(0.0, 0.0, 3.0));
    let accepted = create_every_kind(&mut world, other, free);
    assert!(accepted.iter().all(Result::is_ok), "{accepted:?}");
    step(&mut world, 2);

    let (mut world, layers) = car_world(GRAVITY, 1);
    let chassis = world
        .create_body(
            &chassis_shape(),
            &chassis_settings(&layers, RVec3::new(0.0, 2.0, 0.0), Quat::IDENTITY)
                .allowed_dofs(AllowedDofs::PLANE_2D),
        )
        .unwrap();
    let settings = car_settings(VehicleCollisionTester::ray(layers.probe));
    assert_eq!(
        world.create_vehicle(chassis, &settings),
        Err(VehicleError::Body(BodyError::RestrictedDofs(chassis)))
    );
    assert_eq!(world.vehicle_ids().count(), 0);
    add_car(
        &mut world,
        &layers,
        RVec3::new(5.0, 2.0, 0.0),
        Quat::IDENTITY,
    );

    let (_, layers) = ragdoll_world(1);
    let shapes = part_shapes();
    let mut parts = humanoid_parts(&shapes, layers.ragdoll);
    parts[1].body = parts[1].body.clone().allowed_dofs(AllowedDofs::PLANE_2D);
    assert!(matches!(
        RagdollSettings::new(&skeleton(), &parts),
        Err(RagdollError::InvalidValue(_))
    ));
}

/// The pose and velocities of `id`.
fn motion(world: &PhysicsWorld, id: BodyId) -> (RVec3, Quat, Vec3, Vec3) {
    let body = world.body(id).unwrap();
    (
        body.position(),
        body.rotation(),
        body.linear_velocity(),
        body.angular_velocity(),
    )
}

#[test]
fn rotated_rebase_refuses_restricted_dofs_and_changes_nothing() {
    let mut world = world(Vec3::ZERO, 1);
    let free = add_cube(&mut world, RVec3::new(-3.0, 0.0, 0.0));
    let rail = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_dynamic()
                .linear_velocity(Vec3::new(1.0, 0.0, 0.0))
                .allowed_dofs(AllowedDofs::TRANSLATION_X),
        )
        .unwrap();
    step(&mut world, 1);
    let saved = world.save_state();
    let before = [motion(&world, free), motion(&world, rail)];
    let quarter_turn = quat_about(Z, std::f32::consts::FRAC_PI_2);
    assert_eq!(
        world.rebase(&[free, rail], quarter_turn, RVec3::ZERO),
        Err(BodyError::RestrictedDofs(rail))
    );
    assert_eq!([motion(&world, free), motion(&world, rail)], before);
    assert_eq!(world.gravity(), Vec3::ZERO);
    world.restore_state(&saved).unwrap();
}

#[test]
fn translation_rebase_keeps_restricted_dofs() {
    let mut world = world(Vec3::ZERO, 1);
    let rail = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_dynamic()
                .linear_velocity(Vec3::new(1.0, 0.0, 0.0))
                .allowed_dofs(AllowedDofs::TRANSLATION_X),
        )
        .unwrap();
    world
        .rebase(&[rail], Quat::IDENTITY, RVec3::new(10.0, 2.0, -3.0))
        .unwrap();
    let body = world.body(rail).unwrap();
    assert_eq!(body.position(), RVec3::new(10.0, 2.0, -3.0));
    assert_eq!(body.linear_velocity(), Vec3::new(1.0, 0.0, 0.0));
    assert_eq!(body.allowed_dofs(), AllowedDofs::TRANSLATION_X);
    step(&mut world, 10);
    let p = world.body(rail).unwrap().position();
    assert!(p.x > 10.1 && p.y == 2.0 && p.z == -3.0, "{p:?}");
}

#[test]
fn remove_body_guards_still_hold() {
    let (mut world, layers) = ragdoll_world(1);
    let ragdoll = world
        .create_ragdoll(
            &common::ragdoll::humanoid_settings(layers.ragdoll),
            None,
            Activation::Activate,
        )
        .unwrap();
    let part = world.ragdoll(ragdoll).unwrap().body_ids()[0];
    assert_eq!(
        world.remove_body(part),
        Err(BodyError::OwnedByRagdoll(part))
    );

    let capsule = Shape::new_capsule(0.7, 0.4).unwrap();
    let character = world
        .create_character(
            &CharacterSettings::new(&capsule).inner_body(Some(InnerBody {
                shape: &capsule,
                object_layer: layers.ragdoll,
            })),
            RVec3::new(5.0, 1.0, 0.0),
            Quat::IDENTITY,
        )
        .unwrap();
    let inner = world.character(character).unwrap().inner_body().unwrap();
    assert_eq!(
        world.remove_body(inner),
        Err(BodyError::OwnedByCharacter(inner))
    );

    let anchor = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_static().position(RVec3::new(-5.0, 3.0, 0.0)),
        )
        .unwrap();
    let cube = add_cube(&mut world, RVec3::new(-5.0, 1.0, 0.0));
    world
        .create_constraint(
            anchor,
            cube,
            &PointConstraintSettings::new(RVec3::new(-5.0, 2.0, 0.0)),
        )
        .unwrap();
    for body in [anchor, cube] {
        assert_eq!(
            world.remove_body(body),
            Err(BodyError::UsedByConstraint(body))
        );
    }

    let (mut world, layers) = car_world(GRAVITY, 1);
    let (chassis, _) = add_car(
        &mut world,
        &layers,
        RVec3::new(0.0, 2.0, 0.0),
        Quat::IDENTITY,
    );
    assert_eq!(
        world.remove_body(chassis),
        Err(BodyError::UsedByVehicle(chassis))
    );
}

/// A static sensor box of half extent `half` at `position`.
fn add_static_sensor(world: &mut PhysicsWorld, half: f32, position: RVec3) -> BodyId {
    world
        .create_body(
            &Shape::new_box(Vec3::new(half, half, half)).unwrap(),
            &BodySettings::new_static().position(position).sensor(true),
        )
        .unwrap()
}

/// Whether `event` is between `a` and `b`, in either order.
fn is_between(event: &ContactEvent, a: BodyId, b: BodyId) -> bool {
    let pair = event.pair();
    [pair.body1, pair.body2] == [a, b] || [pair.body1, pair.body2] == [b, a]
}

/// The contact events between `a` and `b`, as `a` (added), `p` (persisted) or `r` (removed),
/// with the settings of the added and persisted ones.
fn sensor_contacts(
    events: &[ContactEvent],
    a: BodyId,
    b: BodyId,
) -> Vec<(char, Option<ContactSettings>)> {
    events
        .iter()
        .filter(|event| is_between(event, a, b))
        .map(|event| match event {
            ContactEvent::Added { settings, .. } => ('a', Some(*settings)),
            ContactEvent::Persisted { settings, .. } => ('p', Some(*settings)),
            ContactEvent::Removed(_) => ('r', None),
        })
        .collect()
}

/// The kinds of [`sensor_contacts`] as a string.
fn contact_kinds(events: &[ContactEvent], a: BodyId, b: BodyId) -> String {
    sensor_contacts(events, a, b)
        .iter()
        .map(|(kind, _)| kind)
        .collect()
}

/// Steps `ticks` times and returns every contact event.
fn contact_events(world: &mut PhysicsWorld, ticks: usize) -> Vec<ContactEvent> {
    let mut events = Vec::new();
    for _ in 0..ticks {
        step(world, 1);
        events.extend(world.take_events().contacts);
    }
    events
}

/// Steps until `body` sleeps, at most 600 ticks, and asserts that it does.
fn fall_asleep(world: &mut PhysicsWorld, body: BodyId) {
    for _ in 0..600 {
        if world.body(body).unwrap().is_sleeping() {
            return;
        }
        step(world, 1);
    }
    panic!("the body never fell asleep");
}

#[test]
fn a_sensor_reports_a_falling_cube_without_stopping_it() {
    let mut world = world(GRAVITY, 1);
    world.set_event_settings(every_event());
    let sensor = add_static_sensor(&mut world, 1.0, RVec3::ZERO);
    let cube = add_cube(&mut world, RVec3::new(0.0, 3.0, 0.0));
    assert!(world.body(sensor).unwrap().is_sensor());
    assert!(!world.body(cube).unwrap().is_sensor());
    let events = contact_events(&mut world, 90);
    let contacts = sensor_contacts(&events, sensor, cube);
    let kinds = contact_kinds(&events, sensor, cube);
    assert!(kinds.starts_with("ap") && kinds.ends_with("pr"), "{kinds}");
    assert!(kinds[1..kinds.len() - 1].chars().all(|kind| kind == 'p'));
    assert!(contacts
        .iter()
        .filter_map(|(_, settings)| *settings)
        .all(|settings| settings.is_sensor()));
    // No response: the cube fell through at free-fall speed.
    let body = world.body(cube).unwrap();
    assert!(body.position().y < -5.0, "{:?}", body.position());
    assert!(body.linear_velocity().y < -14.0);
}

/// Keeps the settings of the first ordinary contact, then tries to turn every sensor contact
/// into an ordinary one, by the setter and by replacing the whole value.
struct Desensitizer {
    sensor: BodyId,
    kept: Mutex<Option<ContactSettings>>,
    setter_results: Mutex<Vec<Result<(), ContactSettingsError>>>,
}

impl ContactListener for Desensitizer {
    fn contact_added(&self, manifold: &ContactManifold, settings: &mut ContactSettings) {
        let pair = manifold.pair;
        if pair.body1 != self.sensor && pair.body2 != self.sensor {
            self.kept.lock().unwrap().get_or_insert(*settings);
            return;
        }
        self.setter_results
            .lock()
            .unwrap()
            .push(settings.set_is_sensor(false));
        if let Some(kept) = *self.kept.lock().unwrap() {
            *settings = kept;
        }
    }
}

#[test]
fn a_listener_cannot_turn_a_sensor_contact_into_an_ordinary_one() {
    let mut world = world(GRAVITY, 1);
    world.set_event_settings(EventSettings::default().contacts(true));
    add_floor(&mut world);
    add_cube(&mut world, RVec3::new(-5.0, 0.5, 0.0));
    let sensor = add_static_sensor(&mut world, 1.0, RVec3::new(0.0, 4.0, 0.0));
    let listener = Arc::new(Desensitizer {
        sensor,
        kept: Mutex::default(),
        setter_results: Mutex::default(),
    });
    world.set_contact_listener(Some(listener.clone()));
    step(&mut world, 5);
    assert!(listener.kept.lock().unwrap().is_some());
    world.take_events();

    let cube = add_cube(&mut world, RVec3::new(0.0, 6.0, 0.0));
    let mut contacts = Vec::new();
    let mut rejections = Vec::new();
    for _ in 0..70 {
        step(&mut world, 1);
        let events = world.take_events();
        contacts.extend(events.contacts);
        rejections.extend(events.rejected_contact_settings);
    }
    let results = listener.setter_results.lock().unwrap().clone();
    assert!(!results.is_empty());
    assert!(results
        .iter()
        .all(|result| *result == Err(ContactSettingsError::SensorBody)));
    assert!(!rejections.is_empty());
    for rejection in &rejections {
        assert_eq!(rejection.error, ContactSettingsError::SensorBody);
    }
    assert!(sensor_contacts(&contacts, sensor, cube)
        .iter()
        .filter_map(|(_, settings)| *settings)
        .all(|settings| settings.is_sensor()));
    assert!(world.body(cube).unwrap().position().y < 2.0);
}

#[test]
fn a_kinematic_body_entering_a_static_sensor_is_reported() {
    let mut world = world(Vec3::ZERO, 1);
    world.set_event_settings(EventSettings::default().contacts(true));
    let sensor = add_static_sensor(&mut world, 1.0, RVec3::ZERO);
    let plain = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_static().position(RVec3::new(0.0, 0.0, 6.0)),
        )
        .unwrap();
    let kinematic = |z| {
        BodySettings::new_kinematic()
            .position(RVec3::new(-4.0, 0.0, z))
            .linear_velocity(Vec3::new(3.0, 0.0, 0.0))
    };
    let platform = world.create_body(&cube_shape(), &kinematic(0.0)).unwrap();
    let crossing = world.create_body(&cube_shape(), &kinematic(6.0)).unwrap();
    let events = contact_events(&mut world, 120);
    assert_eq!(contact_kinds(&events, sensor, platform), "ar");
    // A kinematic body passing a static body that is not a sensor makes no contact.
    assert!(sensor_contacts(&events, plain, crossing).is_empty());
}

#[test]
fn a_static_sensor_loses_a_body_that_falls_asleep() {
    let mut world = world(GRAVITY, 1);
    world.set_event_settings(EventSettings::default().contacts(true));
    add_floor(&mut world);
    let sensor = add_static_sensor(&mut world, 2.0, RVec3::new(0.0, 1.0, 0.0));
    let cube = add_cube(&mut world, RVec3::new(0.0, 0.6, 0.0));
    let mut events = Vec::new();
    for _ in 0..600 {
        step(&mut world, 1);
        events.extend(world.take_events().contacts);
        if world.body(cube).unwrap().is_sleeping() {
            break;
        }
    }
    assert!(world.body(cube).unwrap().is_sleeping());
    events.extend(contact_events(&mut world, 30));
    assert_eq!(contact_kinds(&events, sensor, cube), "ar");
    assert!(world.body(cube).unwrap().is_sleeping());
}

#[test]
fn an_active_kinematic_sensor_sees_a_sleeping_body() {
    let mut world = world(GRAVITY, 1);
    world.set_event_settings(EventSettings::default().contacts(true));
    add_floor(&mut world);
    let cube = add_cube(&mut world, RVec3::new(0.0, 0.5, 0.0));
    fall_asleep(&mut world, cube);
    world.take_events();
    let sensor = world
        .create_body(
            &Shape::new_box(Vec3::new(1.0, 1.0, 1.0)).unwrap(),
            &BodySettings::new_kinematic()
                .position(RVec3::new(0.0, 1.0, 0.0))
                .sensor(true),
        )
        .unwrap();
    assert!(world.body(sensor).unwrap().is_active());
    let events = contact_events(&mut world, 30);
    let kinds = contact_kinds(&events, sensor, cube);
    assert!(kinds.starts_with('a') && !kinds.contains('r'), "{kinds}");
    assert!(world.body(cube).unwrap().is_sleeping());
}

#[test]
fn sensors_never_fall_asleep() {
    let mut world = world(GRAVITY, 1);
    let still = |x| {
        BodySettings::new_dynamic()
            .position(RVec3::new(x, 0.0, 0.0))
            .gravity_factor(0.0)
    };
    let plain = world.create_body(&cube_shape(), &still(0.0)).unwrap();
    let sensor = world
        .create_body(&cube_shape(), &still(5.0).sensor(true))
        .unwrap();
    step(&mut world, 300);
    assert!(world.body(plain).unwrap().is_sleeping());
    assert!(world.body(sensor).unwrap().is_active());
}

#[test]
fn a_character_inner_body_triggers_a_sensor() {
    let mut world = world(GRAVITY, 1);
    world.set_event_settings(EventSettings::default().contacts(true));
    add_floor(&mut world);
    let sensor = add_static_sensor(&mut world, 1.0, RVec3::new(3.0, 1.0, 0.0));
    let capsule = Shape::new_capsule(0.5, 0.3).unwrap();
    let settings = CharacterSettings::new(&capsule)
        .shape_offset(Vec3::new(0.0, 0.8, 0.0))
        .inner_body(Some(InnerBody {
            shape: &capsule,
            object_layer: ObjectLayer::MOVING,
        }));
    let character = world
        .create_character(&settings, RVec3::ZERO, Quat::IDENTITY)
        .unwrap();
    let inner = world.character(character).unwrap().inner_body().unwrap();
    let mut events = Vec::new();
    let mut listed_as_sensor = false;
    for _ in 0..180 {
        world
            .character_mut(character)
            .unwrap()
            .set_linear_velocity(Vec3::new(2.0, -1.0, 0.0))
            .unwrap();
        world
            .update_character(
                character,
                DT,
                GRAVITY,
                &ExtendedUpdateSettings::default(),
                &QueryFilter::new(),
            )
            .unwrap();
        listed_as_sensor |= world
            .character(character)
            .unwrap()
            .active_contacts()
            .iter()
            .any(|contact| contact.body == Some(sensor) && contact.is_sensor);
        step(&mut world, 1);
        events.extend(world.take_events().contacts);
    }
    // The character walked through the sensor, which it listed but which never blocked it.
    assert!(listed_as_sensor);
    assert!(world.character(character).unwrap().position().x > 4.5);
    let kinds = contact_kinds(&events, sensor, inner);
    assert!(kinds.starts_with('a') && kinds.ends_with('r'), "{kinds}");
}

#[test]
fn sensor_rules_refuse_static_only_shapes_and_linear_cast() {
    let mut world = world(GRAVITY, 1);
    let terrain = flat_height_field();
    let (vertices, triangles) = common::meshes::grid(2, 1.0, |_, _| 0.0);
    let (mesh, _) = Shape::new_mesh(&vertices, &triangles).unwrap();
    for shape in [&terrain, &mesh] {
        for settings in [
            BodySettings::new_static(),
            BodySettings::new_kinematic().mass(1.0),
        ] {
            let refused = world.create_body(shape, &settings.sensor(true));
            assert!(
                matches!(refused, Err(BodyError::InvalidValue(_))),
                "{refused:?}"
            );
        }
    }
    let fast = BodySettings::new_dynamic()
        .motion_quality(MotionQuality::LinearCast)
        .sensor(true);
    assert!(matches!(
        world.create_body(&cube_shape(), &fast),
        Err(BodyError::InvalidValue(_))
    ));
    assert_eq!(world.body_count(), 0);
    // Without the sensor flag both are ordinary bodies.
    world
        .create_body(&terrain, &BodySettings::new_static())
        .unwrap();
    world
        .create_body(
            &cube_shape(),
            &fast.sensor(false).position(RVec3::new(0.0, 3.0, 0.0)),
        )
        .unwrap();
    step(&mut world, 10);
}

#[test]
fn a_soft_body_reports_a_sensor() {
    let mut world = world(GRAVITY, 1);
    world.set_event_settings(every_event());
    let sensor = add_static_sensor(&mut world, 1.0, RVec3::ZERO);
    let cloth = add_cloth(&mut world, RVec3::new(-0.5, 2.0, -0.5), Quat::IDENTITY);
    let contacts = soft_contacts(&mut world, 90);
    assert!(contacts
        .iter()
        .any(|contacts| contacts.soft_body == cloth && contacts.sensors.contains(&sensor)));
    // The cloth fell through the sensor.
    assert!(world.body(cloth).unwrap().position().y < -1.0);
}

#[test]
fn user_data_round_trips() {
    let mut world = world(GRAVITY, 1);
    let floor = add_floor(&mut world);
    let keys = [0, 7, u64::MAX];
    let bodies: Vec<BodyId> = keys
        .iter()
        .enumerate()
        .map(|(i, &key)| {
            let settings = BodySettings::new_dynamic()
                .position(RVec3::new(2.0 * i as Real, 2.0, 0.0))
                .user_data(key);
            world.create_body(&cube_shape(), &settings).unwrap()
        })
        .collect();
    let read = |world: &PhysicsWorld| -> Vec<u64> {
        bodies
            .iter()
            .map(|&id| world.body(id).unwrap().user_data())
            .collect()
    };
    assert_eq!(read(&world), keys);
    let saved = world.save_state();
    step(&mut world, 30);
    assert_eq!(read(&world), keys);
    world.restore_state(&saved).unwrap();
    assert_eq!(read(&world), keys);
    let mut all = vec![floor];
    all.extend(&bodies);
    world
        .rebase(
            &all,
            quat_about(Vec3::new(0.0, 1.0, 0.0), 0.5),
            RVec3::new(3.0, 0.0, 0.0),
        )
        .unwrap();
    assert_eq!(read(&world), keys);

    let capsule = Shape::new_capsule(0.5, 0.3).unwrap();
    let character = world
        .create_character(
            &CharacterSettings::new(&capsule)
                .user_data(42)
                .inner_body(Some(InnerBody {
                    shape: &capsule,
                    object_layer: ObjectLayer::MOVING,
                })),
            RVec3::new(-5.0, 1.0, 0.0),
            Quat::IDENTITY,
        )
        .unwrap();
    let inner = world.character(character).unwrap().inner_body().unwrap();
    assert_eq!(world.body(inner).unwrap().user_data(), 42);
}

#[test]
fn ragdoll_parts_refuse_user_data() {
    let (mut world, layers) = ragdoll_world(1);
    let shapes = part_shapes();
    let mut parts = humanoid_parts(&shapes, layers.ragdoll);
    parts[2].body = parts[2].body.clone().user_data(5);
    assert!(matches!(
        RagdollSettings::new(&skeleton(), &parts),
        Err(RagdollError::InvalidValue(_))
    ));
    let settings = common::ragdoll::humanoid_settings(layers.ragdoll);
    let ragdoll = world
        .create_ragdoll(&settings, None, Activation::Activate)
        .unwrap();
    for &part in world.ragdoll(ragdoll).unwrap().body_ids() {
        assert_eq!(world.body(part).unwrap().user_data(), 0);
    }
}

#[test]
fn changing_settings_after_creation_changes_no_body() {
    let mut world = world(GRAVITY, 1);
    let mut settings = BodySettings::new_dynamic()
        .sensor(true)
        .user_data(11)
        .allowed_dofs(AllowedDofs::PLANE_2D);
    let first = world.create_body(&cube_shape(), &settings).unwrap();
    settings = settings
        .sensor(false)
        .user_data(12)
        .allowed_dofs(AllowedDofs::ALL)
        .position(RVec3::new(3.0, 0.0, 0.0));
    let second = world.create_body(&cube_shape(), &settings).unwrap();
    let config = |id| {
        let body = world.body(id).unwrap();
        (body.is_sensor(), body.user_data(), body.allowed_dofs())
    };
    assert_eq!(config(first), (true, 11, AllowedDofs::PLANE_2D));
    assert_eq!(config(second), (false, 12, AllowedDofs::ALL));
}

/// Whether `result` is a refusal for an invalid value.
fn invalid<T: std::fmt::Debug>(result: Result<T, BodyError>) -> bool {
    matches!(result, Err(BodyError::InvalidValue(_)))
}

/// The largest positive `f32` in `0..=high` that `accepts` accepts, by bisection on the bits;
/// `accepts` must accept 0 and be monotonic.
fn largest_accepted(high: f32, mut accepts: impl FnMut(f32) -> bool) -> f32 {
    let (mut low, mut high) = (0_u32, high.to_bits());
    assert!(accepts(0.0));
    while low < high {
        let middle = low + (high - low).div_ceil(2);
        if accepts(f32::from_bits(middle)) {
            low = middle;
        } else {
            high = middle - 1;
        }
    }
    f32::from_bits(low)
}

/// Asserts that `id`'s pose and velocities are finite.
fn assert_finite(world: &PhysicsWorld, id: BodyId) {
    let (p, q, v, w) = motion(world, id);
    let p: [Real; 3] = p.into();
    let values = [q.x, q.y, q.z, q.w, v.x, v.y, v.z, w.x, w.y, w.z];
    assert!(p.iter().all(|c| c.is_finite()) && values.iter().all(|c| c.is_finite()));
}

/// A dynamic body of `shape` with `mass` that starts asleep, without gravity.
fn add_sleeping(world: &mut PhysicsWorld, shape: &Shape, mass: f32, position: RVec3) -> BodyId {
    world
        .create_body(
            shape,
            &BodySettings::new_dynamic()
                .position(position)
                .mass(mass)
                .activation(Activation::DontActivate),
        )
        .unwrap()
}

#[test]
fn impulses_change_velocity_at_once_and_wake() {
    let mut world = world(Vec3::ZERO, 1);
    // 2 kg: Jolt's inverse mass 0.5 is exact, so the velocity change is exact too.
    let cube = add_sleeping(&mut world, &cube_shape(), 2.0, RVec3::ZERO);
    assert!(world.body(cube).unwrap().is_sleeping());
    let mut body = world.body_mut(cube).unwrap();
    body.add_impulse(Vec3::new(3.0, -1.0, 0.5)).unwrap();
    assert!(body.is_active());
    assert_eq!(body.linear_velocity(), Vec3::new(1.5, -0.5, 0.25));
    body.add_impulse(Vec3::new(1.0, 1.0, -0.5)).unwrap();
    assert_eq!(body.linear_velocity(), Vec3::new(2.0, 0.0, 0.0));

    // A zero impulse is accepted and still wakes a sleeping body, as in Jolt.
    let other = add_sleeping(&mut world, &cube_shape(), 2.0, RVec3::new(5.0, 0.0, 0.0));
    let mut body = world.body_mut(other).unwrap();
    body.add_impulse(Vec3::ZERO).unwrap();
    body.add_angular_impulse(Vec3::ZERO).unwrap();
    assert!(body.is_active());
    assert_eq!(body.linear_velocity(), Vec3::ZERO);
    step(&mut world, 1);
    assert!(world.body(cube).unwrap().position().x > 0.0);
}

#[test]
fn impulse_at_point_spins_the_body() {
    let mut world = world(Vec3::ZERO, 1);
    // A sphere of 2.5 kg and radius 1 m has an inertia of 1 kg·m² about every axis.
    let ball = add_sleeping(
        &mut world,
        &Shape::new_sphere(1.0).unwrap(),
        2.5,
        RVec3::ZERO,
    );
    let mut body = world.body_mut(ball).unwrap();
    body.add_impulse_at_point(Vec3::new(0.0, 1.0, 0.0), RVec3::new(1.0, 0.0, 0.0))
        .unwrap();
    assert!(body.is_active());
    assert_eq!(body.linear_velocity(), Vec3::new(0.0, 0.4, 0.0));
    let w = body.angular_velocity();
    assert!(w.x.abs() < 1.0e-6 && w.y.abs() < 1.0e-6, "{w:?}");
    assert!((w.z - 1.0).abs() < 1.0e-5, "{w:?}");
}

#[test]
fn impulses_are_bounded_by_the_velocity_change_they_give() {
    let mut world = world(Vec3::ZERO, 1);
    for (i, mass) in [limits::MIN_MASS, 2.0, limits::MAX_MASS]
        .into_iter()
        .enumerate()
    {
        let id = add_sleeping(
            &mut world,
            &cube_shape(),
            mass,
            RVec3::new(0.0, 0.0, 10.0 * i as Real),
        );
        // Jolt's inverse mass of a body whose mass is overridden.
        let inverse_mass = f64::from(1.0 / mass);
        let bound = f64::from(limits::MAX_VELOCITY_CHANGE);
        let largest = largest_accepted(f32::MAX, |j| f64::from(j) * inverse_mass <= bound);
        let mut body = world.body_mut(id).unwrap();
        let before = (body.linear_velocity(), body.is_active());
        assert!(invalid(body.add_impulse(Vec3::new(
            0.0,
            largest.next_up(),
            0.0
        ))));
        assert!(invalid(body.add_impulse_at_point(
            Vec3::new(0.0, -largest.next_up(), 0.0),
            body.position()
        )));
        for value in [f32::NAN, f32::INFINITY] {
            assert!(invalid(body.add_impulse(Vec3::new(value, 0.0, 0.0))));
        }
        assert_eq!((body.linear_velocity(), body.is_active()), before);
        body.add_impulse(Vec3::new(0.0, largest, 0.0)).unwrap();
        // Jolt clamps the new velocity to the speed bound.
        let speed = length(body.linear_velocity());
        assert!(
            (speed - limits::MAX_LINEAR_VELOCITY).abs() < 1.0e-3,
            "{speed}"
        );
        body.add_impulse(Vec3::new(0.0, -largest, 0.0)).unwrap();
        assert!(body.linear_velocity().y < -499.9);
    }
    step(&mut world, 60);
}

#[test]
fn angular_impulses_are_bounded_by_the_angular_velocity_change() {
    let mut world = world(Vec3::ZERO, 1);
    // A 1 g cube of 6 cm: its inertia is just above Jolt's near-zero threshold, which gives the
    // largest inverse inertia a body can have.
    let tiny = Shape::new_box(Vec3::new(0.03, 0.03, 0.03)).unwrap();
    let moment = limits::MIN_MASS * (2.0 * 0.06 * 0.06) / 12.0;
    let rotations = [
        Quat::IDENTITY,
        quat_about(Vec3::new(0.6, 0.0, 0.8), 0.7),
        quat_about(Vec3::new(0.0, 0.28, 0.96), 2.1),
    ];
    let directions = [
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 0.6, 0.8),
        Vec3::new(0.48, 0.6, -0.64),
    ];
    let mut ids = Vec::new();
    for (i, (&rotation, &direction)) in rotations.iter().zip(&directions).enumerate() {
        let id = world
            .create_body(
                &tiny,
                &BodySettings::new_dynamic()
                    .position(RVec3::new(i as Real, 0.0, 0.0))
                    .rotation(rotation)
                    .mass(limits::MIN_MASS)
                    .angular_damping(0.0),
            )
            .unwrap();
        ids.push(id);
        let mut body = world.body_mut(id).unwrap();
        let kick = |body: &mut BodyMut<'_>, size: f32| {
            body.add_angular_impulse(Vec3::new(
                direction.x * size,
                direction.y * size,
                direction.z * size,
            ))
        };
        let largest = largest_accepted(1.0, |size| kick(&mut body, size).is_ok());
        // The bound is the angular velocity change over the largest inverse inertia.
        let expected = limits::MAX_ANGULAR_VELOCITY_CHANGE * moment;
        assert!(
            (largest / expected - 1.0).abs() < 1.0e-3,
            "{largest} {expected}"
        );
        let before = body.angular_velocity();
        assert!(invalid(kick(&mut body, largest.next_up())));
        assert_eq!(body.angular_velocity(), before);
        let point = body.position();
        let lever = RVec3::new(point.x, point.y + 0.03, point.z);
        assert!(invalid(body.add_impulse_at_point(
            Vec3::new(largest.next_up() * 2.0 / 0.03, 0.0, 0.0),
            lever
        )));
        kick(&mut body, largest).unwrap();
        // Jolt clamps the spin to the bound; an overflow would show as an exact zero.
        let spin = length(body.angular_velocity());
        assert!(
            (spin - limits::MAX_ANGULAR_VELOCITY).abs() < 1.0e-3,
            "{spin}"
        );
    }
    step(&mut world, 60);
    for id in ids {
        assert_finite(&world, id);
        let spin = length(world.body(id).unwrap().angular_velocity());
        assert!(spin > 40.0, "{spin}");
    }
}

#[test]
fn point_impulses_are_bounded_by_the_angular_impulse_of_their_lever() {
    let mut world = world(Vec3::ZERO, 1);
    let ball = add_sleeping(
        &mut world,
        &Shape::new_sphere(1.0).unwrap(),
        2.5,
        RVec3::ZERO,
    );
    let mut body = world.body_mut(ball).unwrap();
    // 10 N·s at the centre is accepted; at 50 m from it, it is a 500 N·m·s angular impulse on an
    // inertia of 1 kg·m², beyond the bound.
    let impulse = Vec3::new(0.0, 10.0, 0.0);
    assert!(invalid(
        body.add_impulse_at_point(impulse, RVec3::new(50.0, 0.0, 0.0))
    ));
    let outside = RVec3::new(limits::MAX_POSITION.next_up(), 0.0, 0.0);
    assert!(invalid(body.add_impulse_at_point(impulse, outside)));
    assert!(body.is_sleeping());
    body.add_impulse_at_point(impulse, RVec3::ZERO).unwrap();
    assert_eq!(body.angular_velocity(), Vec3::ZERO);
    body.add_impulse_at_point(impulse, RVec3::new(2.0, 0.0, 0.0))
        .unwrap();
    assert!((body.angular_velocity().z - 20.0).abs() < 1.0e-3);
    step(&mut world, 30);
    assert_finite(&world, ball);
}

#[test]
fn impulses_respect_locked_axes() {
    let mut world = world(Vec3::ZERO, 1);
    let plane = add_locked_cube(&mut world, RVec3::ZERO, AllowedDofs::PLANE_2D);
    let mut body = world.body_mut(plane).unwrap();
    body.add_impulse(Vec3::new(0.0, 0.0, 50.0)).unwrap();
    body.add_angular_impulse(Vec3::new(5.0, 5.0, 0.0)).unwrap();
    assert_eq!(body.linear_velocity(), Vec3::ZERO);
    assert_eq!(body.angular_velocity(), Vec3::ZERO);
    body.add_impulse_at_point(Vec3::new(0.0, 0.0, 10.0), RVec3::new(0.5, 0.5, 0.0))
        .unwrap();
    let (v, w) = (body.linear_velocity(), body.angular_velocity());
    assert_eq!((v.z, w.x, w.y), (0.0, 0.0, 0.0));
    step(&mut world, 10);
    assert_eq!(world.body(plane).unwrap().position(), RVec3::ZERO);
}

#[test]
fn impulses_ignore_static_and_kinematic_bodies_and_refuse_soft_bodies() {
    let mut world = world(Vec3::ZERO, 1);
    let fixed = world
        .create_body(&cube_shape(), &BodySettings::new_static())
        .unwrap();
    let platform = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_kinematic().position(RVec3::new(3.0, 0.0, 0.0)),
        )
        .unwrap();
    for id in [fixed, platform] {
        let mut body = world.body_mut(id).unwrap();
        let huge = Vec3::new(1.0e30, 0.0, 0.0);
        body.add_impulse(huge).unwrap();
        body.add_angular_impulse(huge).unwrap();
        body.add_impulse_at_point(huge, RVec3::new(0.0, 4.0, 0.0))
            .unwrap();
        assert!(invalid(body.add_impulse(Vec3::new(f32::NAN, 0.0, 0.0))));
        assert_eq!(body.linear_velocity(), Vec3::ZERO);
        assert_eq!(body.angular_velocity(), Vec3::ZERO);
    }
    let cloth = add_cloth(&mut world, RVec3::new(0.0, 5.0, 0.0), Quat::IDENTITY);
    let mut body = world.body_mut(cloth).unwrap();
    assert_eq!(
        body.add_impulse(Vec3::ZERO),
        Err(BodyError::SoftBody(cloth))
    );
    assert_eq!(
        body.add_angular_impulse(Vec3::ZERO),
        Err(BodyError::SoftBody(cloth))
    );
    assert_eq!(
        body.add_impulse_at_point(Vec3::ZERO, RVec3::ZERO),
        Err(BodyError::SoftBody(cloth))
    );
    step(&mut world, 5);
}

/// A kinematic body of `shape` at `position`, awake or asleep as `activation` says.
fn add_kinematic(
    world: &mut PhysicsWorld,
    shape: &Shape,
    position: RVec3,
    activation: Activation,
) -> BodyId {
    world
        .create_body(
            shape,
            &BodySettings::new_kinematic()
                .position(position)
                .activation(activation),
        )
        .unwrap()
}

/// The angle in radians between two rotations.
fn angle_between(a: Quat, b: Quat) -> f32 {
    let dot = (a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w)
        .abs()
        .min(1.0);
    2.0 * dot.acos()
}

/// The distance between two points, in `f64`.
fn distance(a: RVec3, b: RVec3) -> f64 {
    let d = [a.x - b.x, a.y - b.y, a.z - b.z].map(f64::from);
    (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
}

#[test]
fn a_kinematic_platform_moves_toward_its_target() {
    let mut world = world(GRAVITY, 1);
    let block = Shape::new_box(Vec3::new(1.0, 0.2, 1.0)).unwrap();
    let off_centre = Shape::new_offset_center_of_mass(&block, Vec3::new(0.3, 0.0, -0.2)).unwrap();
    for (i, shape) in [&block, &off_centre].into_iter().enumerate() {
        let start = RVec3::new(10.0 * i as Real, 0.0, 0.0);
        let platform = add_kinematic(&mut world, shape, start, Activation::Activate);
        let target = RVec3::new(start.x + 0.5, 0.2, -0.1);
        let turn = quat_about(Vec3::new(0.0, 1.0, 0.0), 0.03);
        world
            .body_mut(platform)
            .unwrap()
            .move_kinematic(target, turn, DT)
            .unwrap();
        step(&mut world, 1);
        let body = world.body(platform).unwrap();
        assert!(distance(body.position(), target) < 1.0e-5, "{i}");
        // Jolt's small-angle angular velocity turns by 2 sin(θ/2) instead of θ.
        let error = angle_between(body.rotation(), turn);
        assert!(error < 2.0e-6, "{i}: {error}");
    }
}

#[test]
fn a_kinematic_move_keeps_its_velocity_after_the_step() {
    let mut world = world(Vec3::ZERO, 1);
    let platform = add_kinematic(&mut world, &cube_shape(), RVec3::ZERO, Activation::Activate);
    world
        .body_mut(platform)
        .unwrap()
        .move_kinematic(RVec3::new(0.1, 0.0, 0.0), Quat::IDENTITY, DT)
        .unwrap();
    let velocity = world.body(platform).unwrap().linear_velocity();
    step(&mut world, 2);
    let body = world.body(platform).unwrap();
    assert_eq!(body.linear_velocity(), velocity);
    assert!(distance(body.position(), RVec3::new(0.2, 0.0, 0.0)) < 1.0e-5);
}

#[test]
fn a_tiny_move_does_not_wake_a_sleeping_kinematic_body() {
    let mut world = world(Vec3::ZERO, 1);
    let platform = add_kinematic(
        &mut world,
        &cube_shape(),
        RVec3::ZERO,
        Activation::DontActivate,
    );
    let mut body = world.body_mut(platform).unwrap();
    // 1e-9 m in 1/60 s: a squared speed of 3.6e-15 m²/s², below Jolt's 1e-12.
    body.move_kinematic(RVec3::new(1.0e-9, 0.0, 0.0), Quat::IDENTITY, DT)
        .unwrap();
    assert!(body.is_sleeping());
    step(&mut world, 1);
    assert_eq!(world.body(platform).unwrap().position(), RVec3::ZERO);
    let mut body = world.body_mut(platform).unwrap();
    body.move_kinematic(RVec3::new(1.0e-3, 0.0, 0.0), Quat::IDENTITY, DT)
        .unwrap();
    assert!(body.is_active());
}

#[test]
fn kinematic_moves_respect_locked_axes() {
    let mut world = world(Vec3::ZERO, 1);
    let rail = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_kinematic()
                .allowed_dofs(AllowedDofs::TRANSLATION_X | AllowedDofs::ROTATION_Y),
        )
        .unwrap();
    let turn = quat_about(Vec3::new(0.6, 0.8, 0.0), 0.2);
    world
        .body_mut(rail)
        .unwrap()
        .move_kinematic(RVec3::new(1.0, 2.0, 3.0), turn, DT)
        .unwrap();
    let body = world.body(rail).unwrap();
    let (v, w) = (body.linear_velocity(), body.angular_velocity());
    assert_eq!((v.y, v.z, w.x, w.z), (0.0, 0.0, 0.0, 0.0));
    assert!(v.x > 59.0 && w.y > 0.0, "{v:?} {w:?}");
    step(&mut world, 1);
    let p = world.body(rail).unwrap().position();
    assert_eq!((p.y, p.z), (0.0, 0.0));
}

#[test]
fn a_kinematic_platform_carries_a_resting_cube() {
    let mut world = world(GRAVITY, 1);
    let deck = Shape::new_box(Vec3::new(2.0, 0.2, 2.0)).unwrap();
    let platform = add_kinematic(&mut world, &deck, RVec3::ZERO, Activation::Activate);
    let cube = add_cube(&mut world, RVec3::new(0.0, 0.7, 0.0));
    step(&mut world, 30);
    for tick in 1..=60 {
        let target = RVec3::new(tick as Real / 60.0, 0.0, 0.0);
        world
            .body_mut(platform)
            .unwrap()
            .move_kinematic(target, Quat::IDENTITY, DT)
            .unwrap();
        step(&mut world, 1);
    }
    let platform_x = world.body(platform).unwrap().position().x;
    let cube_x = world.body(cube).unwrap().position().x;
    assert!((platform_x - 1.0).abs() < 1.0e-4, "{platform_x}");
    // Friction accelerates the cube to the platform's 1 m/s within about half a second.
    assert!(cube_x > 0.7, "{cube_x}");
    let cube_speed = world.body(cube).unwrap().linear_velocity().x;
    assert!((cube_speed - 1.0).abs() < 0.05, "{cube_speed}");
}

#[test]
fn kinematic_moves_are_bounded_by_the_velocities_they_imply() {
    let mut world = world(Vec3::ZERO, 1);
    // dt = 1/64: a move of 7.8125 m is exactly 500 m/s.
    let dt = 1.0 / 64.0;
    let platform = add_kinematic(&mut world, &cube_shape(), RVec3::ZERO, Activation::Activate);
    let along = |distance: f32| RVec3::new(Real::from(distance), 0.0, 0.0);
    let mut body = world.body_mut(platform).unwrap();
    assert!(invalid(body.move_kinematic(
        along(7.8125_f32.next_up()),
        Quat::IDENTITY,
        dt
    )));
    body.move_kinematic(along(7.8125), Quat::IDENTITY, dt)
        .unwrap();
    assert_eq!(body.linear_velocity(), Vec3::new(500.0, 0.0, 0.0));
    assert!(world.step(dt).unwrap().is_complete());
    assert_eq!(world.body(platform).unwrap().position(), along(7.8125));

    // The largest turn about z the bound accepts, found on the angle's bits.
    let spinner = add_kinematic(
        &mut world,
        &cube_shape(),
        RVec3::new(0.0, 5.0, 0.0),
        Activation::Activate,
    );
    let mut body = world.body_mut(spinner).unwrap();
    let here = body.position();
    let about_z = |angle: f32| quat_about(Z, angle);
    let largest = largest_accepted(1.5, |angle| {
        body.move_kinematic(here, about_z(angle), dt).is_ok()
    });
    assert!(
        (largest - limits::MAX_ANGULAR_VELOCITY * dt).abs() < 1.0e-3,
        "{largest}"
    );
    assert!(invalid(body.move_kinematic(
        here,
        about_z(largest.next_up()),
        dt
    )));
    body.move_kinematic(here, about_z(largest), dt).unwrap();
    let spin = length(body.angular_velocity());
    assert!(
        spin <= limits::MAX_ANGULAR_VELOCITY * (1.0 + 1.0e-6),
        "{spin}"
    );
    assert!(world.step(dt).unwrap().is_complete());
    assert_finite(&world, spinner);

    // Invalid inputs change nothing.
    let mut body = world.body_mut(platform).unwrap();
    let before = (body.linear_velocity(), body.angular_velocity());
    let far = RVec3::new(limits::MAX_POSITION.next_up(), 0.0, 0.0);
    assert!(invalid(body.move_kinematic(far, Quat::IDENTITY, dt)));
    let skewed = Quat::from_xyzw(0.0, 0.0, 0.0, 2.0);
    assert!(invalid(body.move_kinematic(along(8.0), skewed, dt)));
    for bad_dt in [0.0, -dt, f32::NAN, PhysicsWorld::MAX_DELTA_TIME * 2.0] {
        assert!(invalid(body.move_kinematic(
            along(8.0),
            Quat::IDENTITY,
            bad_dt
        )));
    }
    assert_eq!((body.linear_velocity(), body.angular_velocity()), before);

    let ball = add_cube(&mut world, RVec3::new(0.0, -5.0, 0.0));
    let fixed = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_static().position(RVec3::new(0.0, -10.0, 0.0)),
        )
        .unwrap();
    for id in [ball, fixed] {
        assert_eq!(
            world
                .body_mut(id)
                .unwrap()
                .move_kinematic(RVec3::ZERO, Quat::IDENTITY, dt),
            Err(BodyError::NotKinematic(id))
        );
    }
    let cloth = add_cloth(&mut world, RVec3::new(0.0, 20.0, 0.0), Quat::IDENTITY);
    assert_eq!(
        world
            .body_mut(cloth)
            .unwrap()
            .move_kinematic(RVec3::ZERO, Quat::IDENTITY, dt),
        Err(BodyError::SoftBody(cloth))
    );
    step(&mut world, 1);
}
