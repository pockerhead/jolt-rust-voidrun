//! Sensor bodies: their contact events, Jolt's pair and sleep rules, and how characters and
//! soft bodies see them.

mod common;

use std::sync::{Arc, Mutex};

use common::controls::*;
use common::events::{add_cloth, every_event, soft_contacts};
use common::*;
use oxijolt::*;

const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);

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
fn a_deactivated_sensor_does_not_see_a_sleeping_body() {
    let mut world = world(GRAVITY, 1);
    world.set_event_settings(EventSettings::default().contacts(true));
    add_floor(&mut world);
    let cube = add_cube(&mut world, RVec3::new(0.0, 0.5, 0.0));
    fall_asleep(&mut world, cube);
    let sensor = world
        .create_body(
            &Shape::new_box(Vec3::new(1.0, 1.0, 1.0)).unwrap(),
            &BodySettings::new_kinematic()
                .position(RVec3::new(0.0, 1.0, 0.0))
                .sensor(true),
        )
        .unwrap();
    world.body_mut(sensor).unwrap().deactivate().unwrap();
    world.take_events();
    let events = contact_events(&mut world, 30);
    assert!(sensor_contacts(&events, sensor, cube).is_empty());
    assert!(world.body(sensor).unwrap().is_sleeping());
    // Woken by hand, it detects the sleeping cube.
    world.body_mut(sensor).unwrap().activate();
    let events = contact_events(&mut world, 2);
    assert_eq!(contact_kinds(&events, sensor, cube), "a");
}

#[test]
fn an_awake_body_still_triggers_a_deactivated_sensor() {
    let mut world = world(GRAVITY, 1);
    world.set_event_settings(every_event());
    let sensor = world
        .create_body(
            &Shape::new_box(Vec3::new(1.0, 1.0, 1.0)).unwrap(),
            &BodySettings::new_kinematic().sensor(true),
        )
        .unwrap();
    world.body_mut(sensor).unwrap().deactivate().unwrap();
    let cube = add_cube(&mut world, RVec3::new(0.0, 3.0, 0.0));
    world.take_events();
    let mut events = Vec::new();
    for _ in 0..90 {
        events.extend(contact_events(&mut world, 1));
        assert!(world.body(sensor).unwrap().is_sleeping());
    }
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
