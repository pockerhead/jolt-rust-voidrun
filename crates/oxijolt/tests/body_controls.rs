//! Body controls: locked axes (allowed degrees of freedom) and where they are refused, and
//! sensor bodies.

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
