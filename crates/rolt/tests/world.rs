//! World lifetime, settings validation, gravity and stepping.

use rolt::*;

fn assert_send_sync<T: Send + Sync>() {}

#[test]
fn world_and_shape_are_send_and_sync() {
    assert_send_sync::<PhysicsWorld>();
    assert_send_sync::<Shape>();
}

#[test]
fn default_world_steps() {
    let mut world = PhysicsWorld::new(WorldSettings::default()).unwrap();
    for _ in 0..10 {
        assert_eq!(world.step(1.0 / 60.0), Ok(()));
    }
    assert_eq!(world.body_count(), 0);
}

#[test]
fn invalid_settings_are_rejected() {
    let invalid = [
        WorldSettings::default().worker_threads(0),
        WorldSettings::default().max_bodies(0),
        WorldSettings::default().max_bodies((1 << 23) + 1),
        WorldSettings::default().max_body_pairs(0),
        WorldSettings::default().max_contact_constraints(0),
        WorldSettings::default().temp_allocator_size(0),
        WorldSettings::default().gravity(Vec3::new(0.0, f32::NAN, 0.0)),
        WorldSettings::default().layers(CollisionLayers::new(1)),
    ];
    for settings in invalid {
        let result = PhysicsWorld::new(settings.clone());
        assert!(
            matches!(
                result,
                Err(WorldError::InvalidSettings(_) | WorldError::InvalidLayers(_))
            ),
            "{settings:?} was accepted"
        );
    }
}

fn bits(v: Vec3) -> [u32; 3] {
    <[f32; 3]>::from(v).map(f32::to_bits)
}

#[test]
fn gravity_round_trips_including_zero() {
    for gravity in [Vec3::ZERO, Vec3::new(0.1, -9.81, 3.5)] {
        let mut world = PhysicsWorld::new(WorldSettings::default().gravity(gravity)).unwrap();
        assert_eq!(bits(world.gravity()), bits(gravity));
        world.step(1.0 / 60.0).unwrap();

        let other = Vec3::new(-1.25, 0.0, 7.0);
        world.set_gravity(other).unwrap();
        assert_eq!(bits(world.gravity()), bits(other));

        assert!(world.set_gravity(Vec3::new(f32::NAN, 0.0, 0.0)).is_err());
        assert_eq!(bits(world.gravity()), bits(other));
    }
}

#[test]
fn step_rejects_bad_delta_time() {
    let mut world = PhysicsWorld::new(WorldSettings::default()).unwrap();
    for dt in [0.0, -0.0, -1.0 / 60.0, f32::NAN, f32::INFINITY] {
        assert_eq!(world.step(dt), Err(StepError::InvalidDeltaTime));
    }
}
