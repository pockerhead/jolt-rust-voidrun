//! Drivetrain bounds: overflowing torque curves and tracked drivetrains are refused, and the
//! strongest accepted ones stay finite through a step, with every wheel, track and chassis
//! value read back. Tracks of very unequal inertia are refused, and the widest accepted ratio
//! stays finite on high-friction ground.

mod common;

use common::vehicle::GRAVITY;
use common::vehicle::{car_settings, car_world, chassis_settings, chassis_shape, CarLayers};
use common::vehicle_kinds::*;
use oxijolt::*;

/// A track with one wheel per radius in `radii`, driven at the first, on the side at `x`.
fn track(x: f32, radii: &[f32]) -> VehicleTrackSettings {
    let wheels = radii
        .iter()
        .zip([0.0, 1.0, -1.0])
        .map(|(&radius, z)| TrackedWheelSettings::new(Vec3::new(x, -0.3, z)).radius(radius))
        .collect();
    VehicleTrackSettings::new(wheels, 0)
}

/// A tracked vehicle whose clutch engages at once, with an engine of `max_torque` over `curve`.
fn tracked(
    layers: &CarLayers,
    left: VehicleTrackSettings,
    right: VehicleTrackSettings,
    curve: Vec<(f32, f32)>,
    max_torque: f32,
) -> TrackedVehicleSettings {
    TrackedVehicleSettings::new(left, right, VehicleCollisionTester::ray(layers.probe))
        .engine(
            TrackedVehicleSettings::default_engine()
                .max_torque(max_torque)
                .normalized_torque(curve),
        )
        .transmission(TrackedVehicleSettings::default_transmission().clutch_release_time(0.0))
}

/// The widest torque curve: x over all of `0..=1`, from 0 straight up to the largest fraction.
fn widest_curve() -> Vec<(f32, f32)> {
    let top = limits::MAX_NORMALIZED_TORQUE;
    vec![
        (0.0, 0.0),
        (limits::MIN_TORQUE_CURVE_SPACING, top),
        (1.0, top),
    ]
}

/// Curves whose interpolation overflowed in Jolt (NaN wheel speeds within two steps): a wide x
/// span with a huge y, a wide x span alone, and a wide y span alone.
fn overflowing_curves() -> [Vec<(f32, f32)>; 3] {
    [
        vec![(-1.0e30, 0.0), (1.0e30, 1.0e30)],
        vec![(-1.0e38, 0.0), (1.0e38, 10.0)],
        vec![(0.0, -3.0e38), (1.0, 3.0e38)],
    ]
}

#[track_caller]
fn assert_refused<T: std::fmt::Debug>(result: Result<T, VehicleError>, rule: &str) {
    match result {
        Err(VehicleError::InvalidValue(what)) => {
            assert!(
                what.contains(rule),
                "refused for {what:?}, expected {rule:?}"
            )
        }
        other => panic!("expected a refusal for {rule:?}, got {other:?}"),
    }
}

/// A world with gravity, a tank-sized chassis high in the air, and optionally a ground under it.
fn tank_world(ground: bool) -> (PhysicsWorld, CarLayers, BodyId) {
    let (mut world, layers) = car_world(GRAVITY, 1);
    let height = if ground {
        let shape = Shape::new_box(Vec3::new(100.0, 1.0, 100.0)).unwrap();
        world
            .create_body(
                &shape,
                &BodySettings::new_static()
                    .position(RVec3::new(0.0, -1.0, 0.0))
                    .object_layer(layers.ground),
            )
            .unwrap();
        1.2
    } else {
        100.0
    };
    let chassis = world
        .create_body(
            &tank_chassis_shape(),
            &behaviour_chassis(
                &layers,
                4000.0,
                RVec3::new(0.0, height, 0.0),
                Quat::IDENTITY,
            ),
        )
        .unwrap();
    (world, layers, chassis)
}

/// The largest max torque `settings` accepts on `chassis`: the vehicle is created at it and
/// refused at the next larger `f32`.
fn largest_accepted_torque(
    world: &mut PhysicsWorld,
    chassis: BodyId,
    settings: impl Fn(f32) -> TrackedVehicleSettings,
) -> f32 {
    let mut accepts = |torque: f32| match world.create_tracked_vehicle(chassis, &settings(torque)) {
        Ok(tank) => {
            world.remove_vehicle(tank).unwrap();
            true
        }
        Err(_) => false,
    };
    let (mut low, mut high) = (0_u32, f32::MAX.to_bits());
    assert!(accepts(0.0) && !accepts(f32::MAX));
    while high - low > 1 {
        let middle = low + (high - low) / 2;
        if accepts(f32::from_bits(middle)) {
            low = middle;
        } else {
            high = middle;
        }
    }
    f32::from_bits(low)
}

/// Every value the world reads back from a vehicle and its chassis, which must all be finite:
/// each wheel's angular velocity, rotation and steer angle, suspension length and impulses, the
/// engine rpm, and the chassis pose and velocities.
fn assert_finite<K: VehicleKind>(world: &PhysicsWorld, id: VehicleId<K>, chassis: BodyId) {
    let vehicle = world.vehicle(id).unwrap();
    for (index, wheel) in vehicle.wheels().iter().enumerate() {
        let values = [
            wheel.angular_velocity,
            wheel.rotation_angle,
            wheel.steer_angle,
            wheel.suspension_length,
            wheel.suspension_lambda,
            wheel.longitudinal_lambda,
            wheel.lateral_lambda,
        ];
        assert!(
            values.iter().all(|value| value.is_finite()),
            "wheel {index}: {wheel:?}"
        );
    }
    assert!(vehicle.engine_rpm().is_finite());
    let body = world.body(chassis).unwrap();
    let (p, q, v, w) = (
        body.position(),
        body.rotation(),
        body.linear_velocity(),
        body.angular_velocity(),
    );
    let position = [p.x, p.y, p.z];
    let rest = [q.x, q.y, q.z, q.w, v.x, v.y, v.z, w.x, w.y, w.z];
    assert!(
        position.iter().all(|value| value.is_finite())
            && rest.iter().all(|value| value.is_finite()),
        "chassis {p:?} {q:?} {v:?} {w:?}"
    );
}

/// Drives a tank of `settings` with each input schedule at the shortest common and the longest
/// step, in the air and on the ground, and checks every value after each step, the tracks
/// included. A schedule alternates its two inputs tick by tick; the last one swaps the slow track
/// every tick, so the synchronisation moves speed from one track to the other.
fn drive_every_way(settings: &TrackedVehicleSettings) {
    let slow = 1.0 / limits::MAX_RATIO;
    let constant = |input: TrackedDriverInput| [input, input];
    let schedules = [
        constant(tracks(1.0, 1.0, 1.0)),
        constant(tracks(1.0, 1.0, slow)),
        constant(tracks(1.0, -1.0, 1.0)),
        constant(tracks(-1.0, 1.0, 1.0)),
        constant(tracks(0.0, 1.0, 1.0)),
        [tracks(1.0, 1.0, slow), tracks(1.0, slow, 1.0)],
    ];
    let mut fastest = [0.0_f32; 2];
    for ground in [false, true] {
        for schedule in schedules {
            for (dt, ticks) in [(1.0 / 60.0, 120), (PhysicsWorld::MAX_DELTA_TIME, 10)] {
                let (mut world, _, chassis) = tank_world(ground);
                let tank = world.create_tracked_vehicle(chassis, settings).unwrap();
                for tick in 0..ticks {
                    let input = schedule[tick % 2];
                    world
                        .vehicle_mut(tank)
                        .unwrap()
                        .set_driver_input(input)
                        .unwrap();
                    let _ = world.step(dt).unwrap();
                    assert_finite(&world, tank, chassis);
                    let vehicle = world.vehicle(tank).unwrap();
                    for state in vehicle.tracks() {
                        assert!(
                            state.angular_velocity.is_finite() && state.speed.is_finite(),
                            "{state:?} with {input:?}, dt {dt}, ground {ground}"
                        );
                        fastest[0] = fastest[0].max(state.angular_velocity.abs());
                    }
                    for wheel in vehicle.wheels() {
                        fastest[1] = fastest[1].max(wheel.angular_velocity.abs());
                    }
                }
            }
        }
    }
    eprintln!(
        "fastest track {:e} rad/s, fastest wheel {:e} rad/s",
        fastest[0], fastest[1]
    );
}

#[test]
fn overflowing_tracked_drivetrains_are_refused() {
    let (mut world, layers, chassis) = tank_world(false);
    let constant = vec![(0.0, 1.0)];
    // Unequal tiny track inertias: the synchronisation quotient overflowed on the second step.
    let light = tracked(
        &layers,
        track(1.7, &[0.3]).inertia(1.0e-30),
        track(-1.7, &[0.3]).inertia(2.0e-30),
        constant.clone(),
        500.0,
    );
    assert_refused(
        world.create_tracked_vehicle(chassis, &light),
        "track inertia",
    );
    // A 1e-36 m wheel next to the 0.3 m driven wheel turned at an infinite rate.
    let tiny_wheel = |x: f32| track(x, &[0.3, 1.0e-36]).max_brake_torque(0.0);
    let tiny = tracked(
        &layers,
        tiny_wheel(1.7),
        tiny_wheel(-1.7),
        constant.clone(),
        500.0,
    );
    assert_refused(world.create_tracked_vehicle(chassis, &tiny), "wheel radii");
    // Torque curves whose interpolation overflows.
    for curve in overflowing_curves() {
        let settings = tracked(
            &layers,
            track(1.7, &[0.3]),
            track(-1.7, &[0.3]),
            curve,
            1.0e-30,
        );
        assert_refused(
            world.create_tracked_vehicle(chassis, &settings),
            "torque curve",
        );
    }
    // In the domain, but a torque the synchronisation cannot take.
    let strong = tracked(
        &layers,
        track(1.7, &[0.3]).inertia(limits::MIN_TRACK_INERTIA),
        track(-1.7, &[0.3]).inertia(2.0 * limits::MIN_TRACK_INERTIA),
        constant,
        1.0e34,
    );
    assert_refused(
        world.create_tracked_vehicle(chassis, &strong),
        "drive envelope",
    );
}

#[test]
fn the_strongest_light_tracks_stay_finite() {
    let (mut world, layers, chassis) = tank_world(false);
    let settings = |torque: f32| {
        tracked(
            &layers,
            track(1.7, &[0.3]).inertia(limits::MIN_TRACK_INERTIA),
            track(-1.7, &[0.3]).inertia(2.0 * limits::MIN_TRACK_INERTIA),
            vec![(0.0, 1.0)],
            torque,
        )
    };
    let torque = largest_accepted_torque(&mut world, chassis, settings);
    eprintln!("largest accepted torque with unequal light tracks: {torque:e} N·m");
    drive_every_way(&settings(torque));
}

#[test]
fn the_strongest_tracks_with_the_widest_radius_ratio_stay_finite() {
    let (mut world, layers, chassis) = tank_world(false);
    let wide = |x: f32| track(x, &[1.0, 1.0 / limits::MAX_RATIO]).max_brake_torque(0.0);
    let settings = |torque: f32| tracked(&layers, wide(1.7), wide(-1.7), vec![(0.0, 1.0)], torque);
    let torque = largest_accepted_torque(&mut world, chassis, settings);
    eprintln!("largest accepted torque with radius ratio MAX_RATIO: {torque:e} N·m");
    drive_every_way(&settings(torque));
}

#[test]
fn the_strongest_tracks_on_the_widest_curve_stay_finite() {
    let (mut world, layers, chassis) = tank_world(false);
    let settings = |torque: f32| {
        tracked(
            &layers,
            track(1.7, &[0.3]),
            track(-1.7, &[0.3]),
            widest_curve(),
            torque,
        )
    };
    let torque = largest_accepted_torque(&mut world, chassis, settings);
    eprintln!("largest accepted torque on the widest curve: {torque:e} N·m");
    drive_every_way(&settings(torque));
    drive_every_way(&settings(1.0e-30));
    drive_every_way(&settings(0.0));
}

/// A track of seven wheels of `radius` with tire friction `MAX_FRICTION` and no brake: the driven
/// wheel high in front, the other six carrying the vehicle.
fn high_friction_track(x: f32, radius: f32, inertia: f32) -> VehicleTrackSettings {
    let wheel = |position: Vec3| {
        TrackedWheelSettings::new(position)
            .radius(radius)
            .longitudinal_friction(limits::MAX_FRICTION)
            .lateral_friction(limits::MAX_FRICTION)
    };
    let driven = wheel(Vec3::new(x, 0.4, 0.0))
        .suspension_min_length(0.0)
        .suspension_max_length(0.05);
    let carrying = [2.5, 1.5, 0.5, -0.5, -1.5, -2.5].map(|z| {
        wheel(Vec3::new(x, -0.3, z))
            .suspension_min_length(0.1)
            .suspension_max_length(0.5)
    });
    let wheels = std::iter::once(driven).chain(carrying).collect();
    VehicleTrackSettings::new(wheels, 0)
        .inertia(inertia)
        .max_brake_torque(0.0)
}

/// A 4000 kg tank resting on wheels of `radius` on ground of friction `MAX_FRICTION`, with track
/// inertias `[left, right]`, and the result of creating it.
fn high_friction_tank(
    radius: f32,
    [left, right]: [f32; 2],
) -> (
    PhysicsWorld,
    BodyId,
    Result<VehicleId<TrackedVehicle>, VehicleError>,
) {
    let (mut world, layers) = car_world(GRAVITY, 1);
    let ground = Shape::new_box(Vec3::new(400.0, 1.0, 400.0)).unwrap();
    world
        .create_body(
            &ground,
            &BodySettings::new_static()
                .position(RVec3::new(0.0, -1.0, 0.0))
                .object_layer(layers.ground)
                .friction(limits::MAX_FRICTION),
        )
        .unwrap();
    let height = (0.7 + radius) as Real;
    let chassis = world
        .create_body(
            &tank_chassis_shape(),
            &behaviour_chassis(
                &layers,
                4000.0,
                RVec3::new(0.0, height, 0.0),
                Quat::IDENTITY,
            ),
        )
        .unwrap();
    let settings = tracked(
        &layers,
        high_friction_track(1.7, radius, left),
        high_friction_track(-1.7, radius, right),
        vec![
            (0.0, limits::MAX_NORMALIZED_TORQUE),
            (1.0, limits::MAX_NORMALIZED_TORQUE),
        ],
        500.0,
    )
    .max_pitch_roll_angle(std::f32::consts::PI);
    let tank = world.create_tracked_vehicle(chassis, &settings);
    (world, chassis, tank)
}

#[test]
fn unequal_track_inertias_are_refused() {
    // On 50 m wheels these gave NaN tracks and chassis on the second step, and Jolt's assertion
    // `MotionProperties.inl:28` in the asserts build.
    for inertias in [[1.0e-3, 1.0e6], [1.0e6, 1.0e-3]] {
        let (_, _, tank) = high_friction_tank(50.0, inertias);
        assert_refused(tank, "inertia ratio");
    }
}

#[test]
fn the_widest_track_inertia_ratio_stays_finite_on_high_friction_ground() {
    let slow = 1.0 / limits::MAX_RATIO;
    let braking = TrackedDriverInput {
        brake: 1.0,
        ..tracks(1.0, 1.0, slow)
    };
    let schedules = [
        [tracks(1.0, 1.0, 1.0); 4],
        [
            tracks(1.0, 1.0, -slow),
            tracks(-1.0, -slow, 1.0),
            tracks(1.0, slow, 1.0),
            braking,
        ],
    ];
    let ratio = limits::MAX_TRACK_INERTIA_RATIO;
    for radius in [5.0, 50.0, 1000.0] {
        for lighter in [limits::MIN_TRACK_INERTIA, 1.0] {
            for inertias in [[lighter, ratio * lighter], [ratio * lighter, lighter]] {
                for schedule in schedules {
                    let (mut world, chassis, tank) = high_friction_tank(radius, inertias);
                    let tank = tank.unwrap();
                    let mut contacts = 0;
                    for tick in 0..300 {
                        world
                            .vehicle_mut(tank)
                            .unwrap()
                            .set_driver_input(schedule[tick % 4])
                            .unwrap();
                        let _ = world.step(1.0 / 60.0).unwrap();
                        assert_finite(&world, tank, chassis);
                        let vehicle = world.vehicle(tank).unwrap();
                        for state in vehicle.tracks() {
                            assert!(
                                state.angular_velocity.is_finite(),
                                "{state:?}, radius {radius}, inertias {inertias:?}, tick {tick}"
                            );
                        }
                        contacts += vehicle
                            .wheels()
                            .iter()
                            .filter(|wheel| wheel.contact.is_some())
                            .count();
                    }
                    assert!(
                        contacts > 0,
                        "radius {radius}: the tank never touched the ground"
                    );
                }
            }
        }
    }
}

/// Steps a wheeled vehicle in the air at `forward` throttle and checks every value.
fn drive_wheeled<K: VehicleKind>(
    world: &mut PhysicsWorld,
    id: VehicleId<K>,
    chassis: BodyId,
    set_throttle: impl Fn(&mut PhysicsWorld),
) {
    set_throttle(world);
    for _ in 0..120 {
        let _ = world.step(1.0 / 60.0).unwrap();
        assert_finite(world, id, chassis);
    }
}

#[test]
fn wheeled_engines_take_only_curves_in_the_domain() {
    for (max_torque, forward) in [(1.0e-30, 1.0), (1.0e-30, 0.0), (0.0, 1.0), (500.0, 1.0)] {
        let (mut world, layers) = car_world(GRAVITY, 1);
        let chassis = world
            .create_body(
                &chassis_shape(),
                &chassis_settings(&layers, RVec3::new(0.0, 100.0, 0.0), Quat::IDENTITY),
            )
            .unwrap();
        let car = |curve: Vec<(f32, f32)>| {
            car_settings(VehicleCollisionTester::ray(layers.probe)).engine(
                VehicleEngineSettings::default()
                    .max_torque(max_torque)
                    .normalized_torque(curve),
            )
        };
        for curve in overflowing_curves() {
            assert_refused(world.create_vehicle(chassis, &car(curve)), "torque curve");
        }
        let id = world.create_vehicle(chassis, &car(widest_curve())).unwrap();
        drive_wheeled(&mut world, id, chassis, |world| {
            world
                .vehicle_mut(id)
                .unwrap()
                .set_driver_input(DriverInput {
                    forward,
                    ..DriverInput::default()
                })
                .unwrap()
        });
    }
}

#[test]
fn motorcycle_engines_take_only_curves_in_the_domain() {
    for (max_torque, forward) in [(1.0e-30, 1.0), (1.0e-30, 0.0), (0.0, 1.0), (150.0, 1.0)] {
        let (mut world, layers) = car_world(GRAVITY, 1);
        let chassis = world
            .create_body(
                &bike_chassis_shape(),
                &behaviour_chassis(
                    &layers,
                    BIKE_MASS,
                    RVec3::new(0.0, 100.0, 0.0),
                    Quat::IDENTITY,
                ),
            )
            .unwrap();
        let bike = |curve: Vec<(f32, f32)>| {
            let vehicle = bike_vehicle_settings(bike_tester(&layers));
            MotorcycleSettings::new(
                vehicle.engine(
                    VehicleEngineSettings::default()
                        .max_torque(max_torque)
                        .min_rpm(1000.0)
                        .max_rpm(10000.0)
                        .normalized_torque(curve),
                ),
            )
        };
        for curve in overflowing_curves() {
            assert_refused(
                world.create_motorcycle(chassis, &bike(curve)),
                "torque curve",
            );
        }
        let id = world
            .create_motorcycle(chassis, &bike(widest_curve()))
            .unwrap();
        drive_wheeled(&mut world, id, chassis, |world| {
            world
                .vehicle_mut(id)
                .unwrap()
                .set_driver_input(ride(forward, 0.0))
                .unwrap()
        });
    }
}
