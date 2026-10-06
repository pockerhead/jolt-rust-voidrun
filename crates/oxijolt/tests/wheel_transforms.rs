//! Wheel poses in world space: on the suspension of a resting car, turned by steering and by
//! rolling, on tank tracks and a raked motorcycle fork, past the last wheel, and across a
//! rebase.

mod common;

use common::math::{add, cross, dot, f3, norm, rotate, scale, sub, v3, V3};
use common::vehicle::*;
use common::vehicle_kinds::*;
use common::*;
use oxijolt::*;

/// A static box floor on the ground layer whose top face is the plane y = 0.
fn add_ground(world: &mut PhysicsWorld, layers: &CarLayers) -> BodyId {
    world
        .create_body(
            &Shape::new_box(Vec3::new(100.0, 1.0, 100.0)).unwrap(),
            &BodySettings::new_static()
                .position(RVec3::new(0.0, -1.0, 0.0))
                .object_layer(layers.ground),
        )
        .unwrap()
}

/// The test car turned 20° about +Y on a floor, settled for 60 ticks under the fixture's
/// gravity.
fn settled_car() -> (PhysicsWorld, BodyId, BodyId, VehicleId) {
    let (mut world, layers) = car_world(Vec3::ZERO, 1);
    let ground = add_ground(&mut world, &layers);
    let rotation = quat_about(Vec3::new(0.0, 1.0, 0.0), 20.0_f32.to_radians());
    let (chassis, car) = add_car(&mut world, &layers, RVec3::new(0.0, 0.9, 0.0), rotation);
    drive_car(&mut world, car, DriverInput::default(), 60);
    (world, ground, chassis, car)
}

fn drive_car(world: &mut PhysicsWorld, car: VehicleId, input: DriverInput, ticks: usize) {
    for _ in 0..ticks {
        let mut vehicle = world.vehicle_mut(car).unwrap();
        vehicle.set_gravity(GRAVITY).unwrap();
        vehicle.set_driver_input(input).unwrap();
        assert!(world.step(DT).unwrap().is_complete());
    }
}

/// The chassis pose in `f64`.
fn chassis_pose(world: &PhysicsWorld, chassis: BodyId) -> (V3, Quat) {
    let body = world.body(chassis).unwrap();
    (v3(body.position()), body.rotation())
}

/// `local` in world space for a chassis at `pose`.
fn to_world(pose: (V3, Quat), local: V3) -> V3 {
    add(pose.0, rotate(pose.1, local))
}

fn assert_close(actual: V3, expected: V3, tolerance: f64, what: &str) {
    assert!(
        norm(sub(actual, expected)) <= tolerance,
        "{what}: {actual:?}, expected {expected:?}"
    );
}

/// The wheel centre Jolt computes: the attachment point plus the suspension direction times
/// the suspension length, in the chassis frame.
fn suspended_centre(attachment: Vec3, direction: V3, suspension_length: f32) -> V3 {
    add(
        f3(attachment),
        scale(direction, f64::from(suspension_length)),
    )
}

/// `v` turned by `angle` radians about the unit `axis` (Rodrigues).
fn turned(v: V3, axis: V3, angle: f64) -> V3 {
    let (s, c) = angle.sin_cos();
    add(
        add(scale(v, c), scale(cross(axis, v), s)),
        scale(axis, dot(axis, v) * (1.0 - c)),
    )
}

#[test]
fn a_resting_car_wheel_sits_on_its_suspension() {
    let (world, _, chassis, car) = settled_car();
    let pose = chassis_pose(&world, chassis);
    let vehicle = world.vehicle(car).unwrap();
    for (index, attachment) in WHEEL_POSITIONS.iter().enumerate() {
        let wheel = vehicle.wheel(index as u32).unwrap();
        assert!(wheel.contact.is_some(), "wheel {index} rests on the floor");
        let (position, rotation) = vehicle.wheel_world_transform(index as u32).unwrap();
        let local = suspended_centre(*attachment, [0.0, -1.0, 0.0], wheel.suspension_length);
        assert_close(
            v3(position),
            to_world(pose, local),
            1e-4,
            &format!("wheel {index} centre"),
        );
        if index == 0 {
            // Unsteered and level: the model's Y axle is the chassis' right, −X.
            assert!(wheel.steer_angle.abs() < 1e-6);
            assert_close(
                rotate(rotation, [0.0, 1.0, 0.0]),
                rotate(pose.1, [-1.0, 0.0, 0.0]),
                1e-4,
                "front left axle",
            );
        }
    }
}

#[test]
fn steering_turns_the_front_axles() {
    let (mut world, _, chassis, car) = settled_car();
    let steer = DriverInput {
        right: 1.0,
        ..DriverInput::default()
    };
    drive_car(&mut world, car, steer, 30);
    let pose = chassis_pose(&world, chassis);
    let vehicle = world.vehicle(car).unwrap();
    let chassis_up = rotate(pose.1, [0.0, 1.0, 0.0]);
    let chassis_right = rotate(pose.1, [-1.0, 0.0, 0.0]);
    for index in 0..4 {
        let wheel = vehicle.wheel(index).unwrap();
        let (_, rotation) = vehicle.wheel_world_transform(index).unwrap();
        let axle = rotate(rotation, [0.0, 1.0, 0.0]);
        let expected = turned(chassis_right, chassis_up, f64::from(wheel.steer_angle));
        if index < 2 {
            assert!(wheel.steer_angle < -0.1, "wheel {index} steers right");
        } else {
            assert_eq!(wheel.steer_angle, 0.0);
        }
        assert_close(axle, expected, 1e-4, &format!("wheel {index} axle"));
    }
}

#[test]
fn rolling_turns_the_wheel_about_its_axle() {
    let (mut world, _, chassis, car) = settled_car();
    let throttle = DriverInput {
        forward: 1.0,
        ..DriverInput::default()
    };
    drive_car(&mut world, car, throttle, 47);
    let pose = chassis_pose(&world, chassis);
    let vehicle = world.vehicle(car).unwrap();
    // A rear wheel does not steer: its up turns toward forward by its rotation angle.
    let wheel = vehicle.wheel(2).unwrap();
    assert!(wheel.rotation_angle.abs() > 0.1, "{wheel:?}");
    let (_, rotation) = vehicle.wheel_world_transform(2).unwrap();
    let (s, c) = f64::from(wheel.rotation_angle).sin_cos();
    let expected = rotate(pose.1, [0.0, c, s]);
    assert_close(
        rotate(rotation, [1.0, 0.0, 0.0]),
        expected,
        1e-4,
        "rear left up",
    );
}

#[test]
fn tank_and_bike_wheels_follow_their_suspension() {
    let (mut world, layers) = car_world(GRAVITY, 1);
    add_ground(&mut world, &layers);
    let (tank_chassis, tank) = add_tank(
        &mut world,
        &layers,
        RVec3::new(0.0, 1.2, 0.0),
        quat_about(Vec3::new(0.0, 1.0, 0.0), 0.3),
    );
    let (bike_chassis, bike) = add_bike(
        &mut world,
        &layers,
        RVec3::new(8.0, 1.0, 0.0),
        Quat::IDENTITY,
    );
    step(&mut world, 60);

    let pose = chassis_pose(&world, tank_chassis);
    let vehicle = world.vehicle(tank).unwrap();
    let wheels: Vec<_> = tank_track_wheels(1.7)
        .into_iter()
        .chain(tank_track_wheels(-1.7))
        .collect();
    assert_eq!(vehicle.wheel_count() as usize, wheels.len());
    for (index, x) in (0..wheels.len()).map(|i| (i, if i < 9 { 1.7 } else { -1.7 })) {
        let state = vehicle.wheel(index as u32).unwrap();
        let (position, _) = vehicle.wheel_world_transform(index as u32).unwrap();
        let z = TANK_WHEEL_Z[index % 9];
        let end = index % 9 == 0 || index % 9 == 8;
        let attachment = Vec3::new(x, if end { 0.0 } else { -0.3 }, z);
        let local = suspended_centre(attachment, [0.0, -1.0, 0.0], state.suspension_length);
        assert_close(
            v3(position),
            to_world(pose, local),
            1e-4,
            &format!("tank wheel {index}"),
        );
    }

    let pose = chassis_pose(&world, bike_chassis);
    let vehicle = world.vehicle(bike).unwrap();
    let (position, _) = vehicle.wheel_world_transform(0).unwrap();
    let rake = 30.0_f64.to_radians();
    let raked = [0.0, -rake.cos(), rake.sin()];
    let attachment = to_world(pose, [0.0, -0.27, 0.75]);
    let along = rotate(pose.1, raked);
    let offset = sub(v3(position), attachment);
    let length = f64::from(vehicle.wheel(0).unwrap().suspension_length);
    assert!(length > 0.25, "the fork is loaded: {length}");
    assert_close(offset, scale(along, length), 1e-4, "bike front wheel");
}

#[test]
fn wheel_world_transform_refuses_indices_past_the_last_wheel() {
    let (world, _, _, car) = settled_car();
    let vehicle = world.vehicle(car).unwrap();
    assert!(vehicle.wheel_world_transform(3).is_some());
    assert_eq!(vehicle.wheel_world_transform(vehicle.wheel_count()), None);
    assert_eq!(vehicle.wheel_world_transform(u32::MAX), None);
}

#[test]
fn wheel_transforms_move_with_a_rebase() {
    let (mut world, ground, chassis, car) = settled_car();
    let before: Vec<_> = (0..4)
        .map(|index| {
            world
                .vehicle(car)
                .unwrap()
                .wheel_world_transform(index)
                .unwrap()
        })
        .collect();
    let rotation = quat_about(Vec3::new(0.0, 1.0, 0.0), 1.1);
    let translation = RVec3::new(-30.0, 2.0, 45.0);
    world
        .rebase(&[ground, chassis], rotation, translation)
        .unwrap();
    let vehicle = world.vehicle(car).unwrap();
    for (index, (position, wheel_rotation)) in before.into_iter().enumerate() {
        let (moved, moved_rotation) = vehicle.wheel_world_transform(index as u32).unwrap();
        let expected = add(rotate(rotation, v3(position)), v3(translation));
        assert_close(v3(moved), expected, 1e-4, &format!("wheel {index} centre"));
        for axis in [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]] {
            assert_close(
                rotate(moved_rotation, axis),
                rotate(rotation, rotate(wheel_rotation, axis)),
                1e-4,
                &format!("wheel {index} axis {axis:?}"),
            );
        }
    }
}
