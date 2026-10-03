//! Wheeled vehicles: wheel contacts against known geometry, driver input, gravity override,
//! sleep policy, lifecycle guards, and the acceptance drive over terrain.

mod common;

use common::vehicle::*;
use common::walker::{f3, rotate, v3};
use common::*;
use joltphysics::*;

/// The three tester kinds on the probe layer: ray, sphere of radius 0.2, cylinder.
fn testers(layers: &CarLayers) -> [VehicleCollisionTester; 3] {
    [
        VehicleCollisionTester::ray(layers.probe),
        VehicleCollisionTester::cast_sphere(layers.probe, 0.2),
        VehicleCollisionTester::cast_cylinder(layers.probe),
    ]
}

/// A static ground whose surface is the plane y = 0: a box floor or the flat heightfield.
fn add_ground(world: &mut PhysicsWorld, layers: &CarLayers, height_field: bool) -> BodyId {
    let (shape, position) = if height_field {
        (flat_height_field(), RVec3::ZERO)
    } else {
        (
            Shape::new_box(Vec3::new(20.0, 1.0, 20.0)).unwrap(),
            RVec3::new(0.0, -1.0, 0.0),
        )
    };
    world
        .create_body(
            &shape,
            &BodySettings::new_static()
                .position(position)
                .object_layer(layers.ground),
        )
        .unwrap()
}

/// A level test car whose wheel attachment points are `attachment_height` above y = 0, with
/// `tester`, stepped once in a world without gravity. Returns the world, the chassis, the
/// vehicle and the chassis pose before the step.
fn car_stepped_once(
    tester_index: usize,
    height_field: bool,
    attachment_height: f32,
) -> (PhysicsWorld, BodyId, BodyId, VehicleId, RVec3, Quat) {
    let (mut world, layers) = car_world(Vec3::ZERO, 1);
    let ground = add_ground(&mut world, &layers, height_field);
    let origin_height = attachment_height - WHEEL_POSITIONS[0].y;
    let body = chassis_settings(
        &layers,
        RVec3::new(0.0, origin_height as Real, 0.0),
        Quat::IDENTITY,
    );
    let (chassis, car) = add_car_with(&mut world, &body, testers(&layers)[tester_index]);
    let before = world.body(chassis).unwrap();
    let (position, rotation) = (before.position(), before.rotation());
    assert!(world.step(DT).unwrap().is_complete());
    (world, ground, chassis, car, position, rotation)
}

#[test]
fn wheel_contacts_match_the_ground_geometry() {
    for height_field in [false, true] {
        for tester in 0..3 {
            let case = format!("tester {tester}, height field {height_field}");
            let (world, ground, _, car, p0, rotation) =
                car_stepped_once(tester, height_field, WHEEL_RADIUS + 0.4);
            let vehicle = world.vehicle(car).unwrap();
            assert_eq!(vehicle.wheel_count(), 4);
            assert_eq!(vehicle.wheel(4), None);
            for (index, wheel) in vehicle.wheels().into_iter().enumerate() {
                let contact = wheel
                    .contact
                    .unwrap_or_else(|| panic!("{case}: wheel {index}"));
                // The suspension starts at the attachment point and runs along the rotated
                // suspension direction; the ground is the plane y = 0.
                let offset = rotate(rotation, f3(WHEEL_POSITIONS[index]));
                let origin = [
                    v3(p0)[0] + offset[0],
                    v3(p0)[1] + offset[1],
                    v3(p0)[2] + offset[2],
                ];
                let direction = rotate(rotation, [0.0, -1.0, 0.0]);
                let distance = origin[1] / -direction[1];
                // Ray: ray_length * f - r; sphere: cast_length * f + R - r; cylinder: max * f.
                // For a vertical suspension over a plane each is the distance minus the wheel
                // radius.
                let expected = distance - f64::from(WHEEL_RADIUS);
                assert_eq!(contact.body, ground, "{case}");
                let normal = f3(contact.normal);
                assert!(
                    normal[0].abs() < 1e-4
                        && (normal[1] - 1.0).abs() < 1e-4
                        && normal[2].abs() < 1e-4,
                    "{case}: wheel {index} normal {normal:?}"
                );
                let position = v3(contact.position);
                assert!(
                    position[1].abs() < 1e-3,
                    "{case}: wheel {index} at {position:?}"
                );
                assert!(
                    (f64::from(wheel.suspension_length) - expected).abs() < 1e-3,
                    "{case}: wheel {index} length {} expected {expected}",
                    wheel.suspension_length
                );
                assert!(!wheel.hit_hard_point, "{case}: wheel {index}");
                let horizontal = |axis: usize| (position[axis] - origin[axis]).abs();
                if tester == 2 {
                    // The cylinder touches along its axle, anywhere across its width.
                    assert!(
                        horizontal(0) <= f64::from(WHEEL_WIDTH) / 2.0 + 1e-3,
                        "{case}: wheel {index} at {position:?}, origin {origin:?}"
                    );
                } else {
                    assert!(
                        horizontal(0) < 1e-3 && horizontal(2) < 1e-3,
                        "{case}: wheel {index} at {position:?}, origin {origin:?}"
                    );
                }
            }

            // Out of reach: attachment 1.75 above the ground, the wheels reach 0.85.
            let (world, _, _, car, _, _) = car_stepped_once(tester, height_field, 1.75);
            for wheel in world.vehicle(car).unwrap().wheels() {
                assert_eq!(wheel.contact, None, "{case}");
                assert_eq!(wheel.suspension_length, SUSPENSION_MAX, "{case}");
            }
        }
    }
}

#[test]
fn start_overlap_reports_a_hard_point_without_depth() {
    // A heightfield is a surface, not a solid, so a wheel under it is not inside anything; the
    // start overlap is shown on the box floor.
    for tester in 0..3 {
        let (world, ground, _, car, _, _) = car_stepped_once(tester, false, -0.1);
        for (index, wheel) in world.vehicle(car).unwrap().wheels().into_iter().enumerate() {
            let contact = wheel
                .contact
                .unwrap_or_else(|| panic!("tester {tester}: wheel {index}"));
            assert_eq!(contact.body, ground);
            assert_eq!(
                wheel.suspension_length, 0.0,
                "tester {tester}: wheel {index}"
            );
            assert!(wheel.hit_hard_point, "tester {tester}: wheel {index}");
        }
    }
}

#[test]
fn start_overlap_depth_comes_from_collide_shape() {
    let (world, ground, chassis, car, p0, _) = car_stepped_once(0, false, -0.1);
    let layers = [ObjectLayer::new(0), ObjectLayer::new(1)];
    assert_eq!(
        world
            .vehicle(car)
            .unwrap()
            .collision_tester()
            .object_layer(),
        ObjectLayer::new(2)
    );
    let wheel = Shape::new_sphere(WHEEL_RADIUS).unwrap();
    let attachment = WHEEL_POSITIONS[0];
    let position = RVec3::new(
        p0.x + attachment.x as Real,
        p0.y + attachment.y as Real,
        p0.z + attachment.z as Real,
    );
    // The layers the probe layer collides with, without the chassis: the same bodies the
    // tester sees.
    let filter = QueryFilter::new()
        .object_layers(&layers)
        .exclude_body(chassis);
    let hits = world
        .collide_shape(
            &CollideShape::new(&wheel, position, Quat::IDENTITY),
            &filter,
        )
        .unwrap();
    assert_eq!(hits.len(), 1, "{hits:?}");
    assert_eq!(hits[0].body, ground);
    // The sphere's centre is 0.1 below the floor top, so it overlaps by 0.35 + 0.1.
    assert!(
        (hits[0].penetration_depth - 0.45).abs() < 1e-2,
        "{:?}",
        hits[0]
    );
}

/// A car on a box floor under the override gravity, settled for 30 ticks.
fn settled_car() -> (PhysicsWorld, BodyId, VehicleId) {
    let (mut world, layers) = car_world(Vec3::ZERO, 1);
    add_ground(&mut world, &layers, false);
    let (chassis, car) = add_car(
        &mut world,
        &layers,
        RVec3::new(0.0, 0.9, 0.0),
        Quat::IDENTITY,
    );
    world
        .vehicle_mut(car)
        .unwrap()
        .set_gravity(GRAVITY)
        .unwrap();
    step(&mut world, 30);
    (world, chassis, car)
}

fn drive(world: &mut PhysicsWorld, car: VehicleId, input: DriverInput, ticks: usize) {
    world
        .vehicle_mut(car)
        .unwrap()
        .set_driver_input(input)
        .unwrap();
    step(world, ticks);
}

fn throttle(right: f32) -> DriverInput {
    DriverInput {
        forward: 1.0,
        right,
        ..DriverInput::default()
    }
}

#[test]
fn driver_input_drives_steers_and_brakes() {
    let (mut world, chassis, car) = settled_car();
    drive(&mut world, car, throttle(0.0), 120);
    let body = world.body(chassis).unwrap();
    let position = v3(body.position());
    assert!(position[2] > 2.0, "{position:?}");
    assert!(position[0].abs() < 0.5, "{position:?}");
    assert!(world.vehicle(car).unwrap().current_gear() >= 1);
    assert!(world.vehicle(car).unwrap().engine_rpm() > 1000.0);
    assert_eq!(world.vehicle(car).unwrap().driver_input(), throttle(0.0));

    let brake = DriverInput {
        brake: 1.0,
        ..DriverInput::default()
    };
    world
        .vehicle_mut(car)
        .unwrap()
        .set_driver_input(brake)
        .unwrap();
    let stopped = (0..180).any(|_| {
        step(&mut world, 1);
        length(world.body(chassis).unwrap().linear_velocity()) < 0.1
    });
    assert!(
        stopped,
        "{:?}",
        world.body(chassis).unwrap().linear_velocity()
    );

    // Steering right bends the path toward the vehicle's right, −X for forward +Z.
    let (mut world, chassis, car) = settled_car();
    drive(&mut world, car, throttle(1.0), 120);
    let position = v3(world.body(chassis).unwrap().position());
    assert!(position[0] < -1.0, "{position:?}");
    let steer = world.vehicle(car).unwrap().wheel(0).unwrap().steer_angle;
    assert!(
        steer < 0.0,
        "steering right is a negative steer angle: {steer}"
    );

    let invalid = [
        DriverInput {
            forward: 1.5,
            ..DriverInput::default()
        },
        DriverInput {
            right: f32::NAN,
            ..DriverInput::default()
        },
        DriverInput {
            brake: -0.1,
            ..DriverInput::default()
        },
        DriverInput {
            hand_brake: 2.0,
            ..DriverInput::default()
        },
    ];
    for input in invalid {
        assert!(matches!(
            world.vehicle_mut(car).unwrap().set_driver_input(input),
            Err(VehicleError::InvalidValue(_))
        ));
    }
}

#[test]
fn braking_at_the_smallest_step_stays_finite() {
    let (mut world, chassis, car) = settled_car();
    drive(&mut world, car, throttle(0.0), 30);
    let brake = DriverInput {
        brake: 1.0,
        ..DriverInput::default()
    };
    world
        .vehicle_mut(car)
        .unwrap()
        .set_driver_input(brake)
        .unwrap();
    // The brake-lock torque is `|w| * inertia / dt`, largest at the smallest step.
    assert!(world
        .step(PhysicsWorld::MIN_DELTA_TIME)
        .unwrap()
        .is_complete());
    let body = world.body(chassis).unwrap();
    let position: [Real; 3] = body.position().into();
    let rotation: [f32; 4] = body.rotation().into();
    assert!(
        position.iter().all(|value| value.is_finite()),
        "{position:?}"
    );
    assert!(
        rotation.iter().all(|value| value.is_finite()),
        "{rotation:?}"
    );
    for wheel in world.vehicle(car).unwrap().wheels() {
        assert!(wheel.angular_velocity.is_finite(), "{wheel:?}");
    }
}

#[test]
fn gravity_override_replaces_world_gravity() {
    let (mut world, layers) = car_world(Vec3::new(0.0, -5.0, 0.0), 1);
    let body =
        chassis_settings(&layers, RVec3::new(0.0, 50.0, 0.0), Quat::IDENTITY).gravity_factor(1.0);
    let (chassis, car) = add_car_with(&mut world, &body, VehicleCollisionTester::ray(layers.probe));
    assert_eq!(world.vehicle(car).unwrap().gravity(), None);
    world
        .vehicle_mut(car)
        .unwrap()
        .set_gravity(GRAVITY)
        .unwrap();
    assert!(matches!(
        world
            .vehicle_mut(car)
            .unwrap()
            .set_gravity(Vec3::new(0.0, f32::INFINITY, 0.0)),
        Err(VehicleError::InvalidValue(_))
    ));
    step(&mut world, 30);
    // Jolt adds the force and then damps the velocity by its default linear damping of 0.05/s
    // on every step.
    let mut expected = 0.0_f64;
    for _ in 0..30 {
        expected = (expected + 9.81 * f64::from(DT)) * (1.0 - 0.05 * f64::from(DT));
    }
    let fall = -f64::from(world.body(chassis).unwrap().linear_velocity().y);
    assert!(
        (fall - expected).abs() < 0.01 * expected,
        "fall speed {fall}, expected {expected}"
    );
    let vehicle = world.vehicle(car).unwrap();
    assert_eq!(vehicle.gravity(), Some(GRAVITY));
    assert_eq!(vehicle.world_up(), Vec3::new(0.0, 1.0, 0.0));
}

#[test]
fn gravity_whose_force_overflows_is_rejected() {
    let (mut world, layers) = car_world(Vec3::ZERO, 1);
    let ground = add_ground(&mut world, &layers, false);
    let body = chassis_settings(&layers, RVec3::new(0.0, 0.9, 0.0), Quat::IDENTITY);
    let (chassis, car) = add_car_with(&mut world, &body, VehicleCollisionTester::ray(layers.probe));
    // Jolt adds `gravity / inverse mass` per component: 1500 kg times 2.3e35 m/s² is past
    // f32::MAX, 1500 kg times 2e35 m/s² is not.
    let accepted = Vec3::new(2.0e35, -2.0e35, 0.0);
    world
        .vehicle_mut(car)
        .unwrap()
        .set_gravity(accepted)
        .unwrap();
    for rejected in [Vec3::new(0.0, -2.3e35, 0.0), Vec3::new(0.0, -1.0e36, 0.0)] {
        assert!(matches!(
            world.vehicle_mut(car).unwrap().set_gravity(rejected),
            Err(VehicleError::InvalidValue(_))
        ));
    }
    assert_eq!(world.vehicle(car).unwrap().gravity(), Some(accepted));

    // An eighth turn about z turns that gravity onto the x axis with a component of 2.8e35,
    // whose force would overflow, so the rebase is refused and changes nothing.
    let angle = std::f32::consts::FRAC_PI_8;
    let eighth_turn_about_z = Quat::from_xyzw(0.0, 0.0, angle.sin(), angle.cos());
    assert!(matches!(
        world.rebase(&[ground, chassis], eighth_turn_about_z, RVec3::ZERO),
        Err(BodyError::InvalidValue(_))
    ));
    assert_eq!(world.vehicle(car).unwrap().gravity(), Some(accepted));

    world
        .vehicle_mut(car)
        .unwrap()
        .set_gravity(GRAVITY)
        .unwrap();
    step(&mut world, 10);
    let chassis = world.body(chassis).unwrap();
    let state = [v3(chassis.position()), f3(chassis.linear_velocity())];
    assert!(
        state.iter().flatten().all(|value| value.is_finite()),
        "{state:?}"
    );
}

#[test]
fn sleeping_chassis_gets_no_override_force() {
    let (mut world, layers) = car_world(Vec3::ZERO, 1);
    add_ground(&mut world, &layers, false);
    let body =
        chassis_settings(&layers, RVec3::new(0.0, 0.9, 0.0), Quat::IDENTITY).allow_sleeping(true);
    let (chassis, car) = add_car_with(&mut world, &body, VehicleCollisionTester::ray(layers.probe));
    world
        .vehicle_mut(car)
        .unwrap()
        .set_gravity(GRAVITY)
        .unwrap();
    let asleep = (0..600).any(|_| {
        step(&mut world, 1);
        world.body(chassis).unwrap().is_sleeping()
    });
    assert!(asleep, "the chassis never fell asleep");
    let bits = |world: &PhysicsWorld| <[Real; 3]>::from(world.body(chassis).unwrap().position());
    let before = bits(&world);
    for _ in 0..120 {
        step(&mut world, 1);
        assert!(world.body(chassis).unwrap().is_sleeping());
    }
    assert_eq!(bits(&world).map(Real::to_bits), before.map(Real::to_bits));
}

#[test]
fn never_sleeping_chassis_stays_awake() {
    let (mut world, chassis, car) = settled_car();
    let hand_brake = DriverInput {
        hand_brake: 1.0,
        ..DriverInput::default()
    };
    world
        .vehicle_mut(car)
        .unwrap()
        .set_driver_input(hand_brake)
        .unwrap();
    for _ in 0..300 {
        step(&mut world, 1);
        assert!(!world.body(chassis).unwrap().is_sleeping());
    }
}

#[test]
fn chassis_cannot_be_removed_while_a_vehicle_uses_it() {
    let (mut world, chassis, car) = settled_car();
    assert_eq!(
        world.remove_body(chassis),
        Err(BodyError::UsedByVehicle(chassis))
    );
    assert_eq!(world.vehicle_of_body(chassis), Some(car));
    world.remove_vehicle(car).unwrap();
    assert_eq!(world.vehicle_of_body(chassis), None);
    assert!(matches!(world.vehicle(car), Err(VehicleError::NotFound(_))));
    assert!(matches!(
        world.remove_vehicle(car),
        Err(VehicleError::NotFound(_))
    ));
    world.remove_body(chassis).unwrap();
    step(&mut world, 10);
}

#[test]
fn invalid_vehicles_create_nothing() {
    let (mut world, layers) = car_world(Vec3::ZERO, 1);
    add_ground(&mut world, &layers, false);
    let chassis = world
        .create_body(
            &chassis_shape(),
            &chassis_settings(&layers, RVec3::new(0.0, 0.9, 0.0), Quat::IDENTITY),
        )
        .unwrap();
    let bodies = world.body_count();
    let unchanged = |world: &PhysicsWorld| {
        assert_eq!(world.body_count(), bodies);
        assert_eq!(world.vehicle_ids().count(), 0);
        assert_eq!(world.vehicle_of_body(chassis), None);
    };

    let bad_layer = car_settings(VehicleCollisionTester::ray(ObjectLayer::new(3)));
    assert!(matches!(
        world.create_vehicle(chassis, &bad_layer),
        Err(VehicleError::InvalidValue(_))
    ));
    unchanged(&world);

    let fixed = world
        .create_body(&chassis_shape(), &BodySettings::new_static())
        .unwrap();
    let kinematic = world
        .create_body(&chassis_shape(), &BodySettings::new_kinematic())
        .unwrap();
    let settings = car_settings(VehicleCollisionTester::ray(layers.probe));
    for body in [fixed, kinematic] {
        assert_eq!(
            world.create_vehicle(body, &settings),
            Err(VehicleError::NotDynamic(body))
        );
    }

    let capsule = Shape::new_capsule(0.7, 0.4).unwrap();
    let character = world
        .create_character(
            &CharacterSettings::new(&capsule).inner_body(Some(InnerBody {
                shape: &capsule,
                object_layer: layers.moving,
            })),
            RVec3::new(5.0, 1.0, 0.0),
            Quat::IDENTITY,
        )
        .unwrap();
    let inner = world.character(character).unwrap().inner_body().unwrap();
    assert_eq!(
        world.create_vehicle(inner, &settings),
        Err(VehicleError::Body(BodyError::OwnedByCharacter(inner)))
    );

    let (mut other, _) = car_world(Vec3::ZERO, 1);
    let foreign = other
        .create_body(&chassis_shape(), &BodySettings::new_dynamic())
        .unwrap();
    assert_eq!(
        world.create_vehicle(foreign, &settings),
        Err(VehicleError::Body(BodyError::WrongWorld(foreign)))
    );
    let removed = world
        .create_body(&chassis_shape(), &BodySettings::new_dynamic())
        .unwrap();
    world.remove_body(removed).unwrap();
    assert_eq!(
        world.create_vehicle(removed, &settings),
        Err(VehicleError::Body(BodyError::NotFound(removed)))
    );
    assert_eq!(world.vehicle_ids().count(), 0);

    let car = world.create_vehicle(chassis, &settings).unwrap();
    assert_eq!(
        world.create_vehicle(chassis, &settings),
        Err(VehicleError::AlreadyHasVehicle(chassis))
    );
    assert_eq!(world.vehicle_ids().collect::<Vec<_>>(), vec![car]);

    let other_car = {
        let (mut other_world, other_layers) = car_world(Vec3::ZERO, 1);
        add_car(&mut other_world, &other_layers, RVec3::ZERO, Quat::IDENTITY).1
    };
    assert!(matches!(
        world.vehicle(other_car),
        Err(VehicleError::WrongWorld(_))
    ));
    assert!(matches!(
        world
            .vehicle_mut(car)
            .unwrap()
            .set_max_pitch_roll_angle(4.0),
        Err(VehicleError::InvalidValue(_))
    ));
    assert!(matches!(
        world
            .vehicle_mut(car)
            .unwrap()
            .set_collision_tester(VehicleCollisionTester::cast_sphere(layers.probe, 1.0)),
        Err(VehicleError::InvalidValue(_))
    ));
    assert_eq!(
        *world.vehicle(car).unwrap().collision_tester(),
        VehicleCollisionTester::ray(layers.probe)
    );
}

#[test]
fn vehicle_ids_are_sequential_and_never_reused() {
    let (mut world, layers) = car_world(Vec3::ZERO, 1);
    let mut ids = Vec::new();
    for i in 0..3 {
        let (_, car) = add_car(
            &mut world,
            &layers,
            RVec3::new(10.0 * i as Real, 1.0, 0.0),
            Quat::IDENTITY,
        );
        ids.push(car);
    }
    assert_eq!(
        ids.iter().map(|id| id.to_raw()).collect::<Vec<_>>(),
        [1, 2, 3]
    );
    world.remove_vehicle(ids[1]).unwrap();
    assert_eq!(world.vehicle_ids().collect::<Vec<_>>(), [ids[0], ids[2]]);
    let (_, fourth) = add_car(
        &mut world,
        &layers,
        RVec3::new(40.0, 1.0, 0.0),
        Quat::IDENTITY,
    );
    assert_eq!(fourth.to_raw(), 4);
}

#[test]
fn a_removed_vehicle_no_longer_drives() {
    let (mut world, chassis, car) = settled_car();
    world
        .vehicle_mut(car)
        .unwrap()
        .set_driver_input(throttle(0.0))
        .unwrap();
    world.remove_vehicle(car).unwrap();
    step(&mut world, 60);
    let velocity = world.body(chassis).unwrap().linear_velocity();
    assert!(velocity.z.abs() < 0.01, "{velocity:?}");
}

#[test]
fn dropping_a_world_with_vehicles_is_clean() {
    for _ in 0..20 {
        let (mut world, layers) = car_world(Vec3::ZERO, 2);
        add_ground(&mut world, &layers, false);
        for x in [0.0, 5.0] {
            let (_, car) = add_car(&mut world, &layers, RVec3::new(x, 0.9, 0.0), Quat::IDENTITY);
            world
                .vehicle_mut(car)
                .unwrap()
                .set_gravity(GRAVITY)
                .unwrap();
        }
        step(&mut world, 2);
        drop(world);
    }
}

#[test]
fn collision_tester_can_be_replaced() {
    let (mut world, _, car) = settled_car();
    let sphere = VehicleCollisionTester::cast_sphere(ObjectLayer::new(2), 0.2);
    world
        .vehicle_mut(car)
        .unwrap()
        .set_collision_tester(sphere)
        .unwrap();
    assert_eq!(*world.vehicle(car).unwrap().collision_tester(), sphere);
    step(&mut world, 5);
    assert!(world
        .vehicle(car)
        .unwrap()
        .wheels()
        .iter()
        .all(|wheel| wheel.contact.is_some()));
    // A tester on the ground layer sees only moving bodies, and its own chassis is skipped.
    let blind = VehicleCollisionTester::ray(ObjectLayer::new(0));
    world
        .vehicle_mut(car)
        .unwrap()
        .set_collision_tester(blind)
        .unwrap();
    step(&mut world, 1);
    assert!(world
        .vehicle(car)
        .unwrap()
        .wheels()
        .iter()
        .all(|wheel| wheel.contact.is_none()));
}

#[test]
fn drive_a_route_over_terrain() {
    let report = drive_route(1, |_, _| {});
    assert_eq!(
        report.reached.len(),
        WAYPOINTS.len(),
        "reached {:?} in {} ticks",
        report.reached,
        report.ticks
    );
    assert!(report.min_up_dot >= 0.5, "{report:?}");
    assert!(
        report.ticks_with_two_wheels as f64 >= 0.95 * report.ticks as f64,
        "{report:?}"
    );
    assert!(report.all_complete, "{report:?}");
}

#[test]
fn drive_a_route_twice_gives_identical_bits() {
    let mut first = Vec::new();
    let mut second = Vec::new();
    drive_route(1, |world, car| record_vehicle(world, car, &mut first));
    drive_route(1, |world, car| record_vehicle(world, car, &mut second));
    assert!(!first.is_empty());
    assert!(first == second, "the two drives differ");
}
