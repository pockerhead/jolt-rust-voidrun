//! Tracked vehicles: a tank pivots in place and climbs a slope, its wheels turn with its
//! tracks, input and settings are checked, and the chassis guards, testers, soft bodies and
//! rebase work as for wheeled vehicles.

mod common;

use common::math::{f3, norm, rotate, sub, v3};
use common::vehicle::{car_world, chassis_settings, CarLayers, GRAVITY};
use common::vehicle_kinds::*;
use common::*;
use oxijolt::*;

/// A rotation of `degrees` about +X.
fn about_x(degrees: f32) -> Quat {
    let half = 0.5 * degrees.to_radians();
    Quat::from_xyzw(half.sin(), 0.0, 0.0, half.cos())
}

/// A static box ground on the ground layer whose top face is the plane through the origin
/// rotated by `rotation`.
fn add_ground(world: &mut PhysicsWorld, layers: &CarLayers, rotation: Quat) -> BodyId {
    let shape = Shape::new_box(Vec3::new(100.0, 1.0, 100.0)).unwrap();
    let [x, y, z] = rotate(rotation, [0.0, -1.0, 0.0]);
    world
        .create_body(
            &shape,
            &BodySettings::new_static()
                .position(RVec3::new(x as Real, y as Real, z as Real))
                .rotation(rotation)
                .object_layer(layers.ground),
        )
        .unwrap()
}

fn drive(
    world: &mut PhysicsWorld,
    tank: VehicleId<TrackedVehicle>,
    input: TrackedDriverInput,
    ticks: usize,
) {
    world
        .vehicle_mut(tank)
        .unwrap()
        .set_driver_input(input)
        .unwrap();
    step(world, ticks);
}

/// A tank settled for 120 ticks on flat ground under world gravity, with its world and layers.
fn settled_tank() -> (PhysicsWorld, CarLayers, BodyId, VehicleId<TrackedVehicle>) {
    let (mut world, layers) = car_world(GRAVITY, 1);
    add_ground(&mut world, &layers, Quat::IDENTITY);
    let (chassis, tank) = add_tank(
        &mut world,
        &layers,
        RVec3::new(0.0, 2.0, 0.0),
        Quat::IDENTITY,
    );
    drive(&mut world, tank, tracks(0.0, 1.0, 1.0), 120);
    (world, layers, chassis, tank)
}

fn horizontal_distance(a: RVec3, b: RVec3) -> f64 {
    let d = sub(v3(a), v3(b));
    (d[0] * d[0] + d[2] * d[2]).sqrt()
}

#[test]
fn tank_pivots_in_place() {
    let (mut world, _, chassis, tank) = settled_tank();
    let start = world.body(chassis).unwrap().position();
    drive(&mut world, tank, tracks(1.0, -1.0, 1.0), 300);
    let body = world.body(chassis).unwrap();
    let drift = horizontal_distance(body.position(), start);
    let yaw_rate = body.angular_velocity().y;
    let [left, right] = world.vehicle(tank).unwrap().tracks();
    eprintln!(
        "pivot: drift {drift} m, yaw rate {yaw_rate} rad/s, tracks {} {} m/s",
        left.speed, right.speed
    );
    assert!(drift < 0.05, "drift {drift}");
    assert!(yaw_rate > 2.3, "yaw rate {yaw_rate}");
    assert!(left.speed < 0.0 && right.speed > 0.0);
    let relative = (left.speed + right.speed).abs() / right.speed.abs();
    assert!(
        relative <= 1e-4,
        "tracks {} and {}",
        left.speed,
        right.speed
    );
}

#[test]
fn tank_climbs_a_12_degree_slope() {
    let (mut world, layers) = car_world(GRAVITY, 1);
    let slope = about_x(-12.0);
    add_ground(&mut world, &layers, slope);
    let (chassis, tank) = add_tank(&mut world, &layers, RVec3::new(0.0, 2.0, 0.0), slope);
    drive(&mut world, tank, tracks(0.0, 1.0, 1.0), 120);
    let start = world.body(chassis).unwrap().position();
    drive(&mut world, tank, tracks(1.0, 1.0, 1.0), 300);
    let body = world.body(chassis).unwrap();
    let delta = sub(v3(body.position()), v3(start));
    let yaw_rate = body.angular_velocity().y;
    eprintln!(
        "slope: rise {} m, sideways {} m, along {} m, yaw rate {yaw_rate} rad/s",
        delta[1], delta[0], delta[2]
    );
    assert!(delta[1] > 3.0, "rise {}", delta[1]);
    assert!(delta[0].abs() < 0.1, "sideways {}", delta[0]);
    assert!(yaw_rate.abs() < 1e-3, "yaw rate {yaw_rate}");
}

#[test]
fn track_wheels_turn_with_their_track() {
    let (mut world, _, _, tank) = settled_tank();
    world
        .vehicle_mut(tank)
        .unwrap()
        .set_driver_input(tracks(1.0, -1.0, 1.0))
        .unwrap();
    let mut checked = 0;
    for _ in 0..60 {
        step(&mut world, 1);
        let vehicle = world.vehicle(tank).unwrap();
        let wheels = vehicle.wheels();
        for side in [TrackSide::Left, TrackSide::Right] {
            let track = vehicle.track(side);
            assert_eq!(track.speed, track.angular_velocity * TANK_WHEEL_RADIUS);
            let range = vehicle.track_wheels(side);
            assert!(range.contains(&track.driven_wheel));
            assert_eq!(track.driven_wheel, range.start + TANK_DRIVEN_WHEEL);
            for index in range {
                let wheel = wheels[index as usize];
                if wheel.contact.is_none() {
                    continue;
                }
                // Every wheel has the driven wheel's radius, so it turns at the track's rate.
                let expected = track.angular_velocity;
                assert_eq!(
                    wheel.angular_velocity.signum(),
                    expected.signum(),
                    "wheel {index}"
                );
                let tolerance = 4.0 * f32::EPSILON * expected.abs();
                assert!(
                    (wheel.angular_velocity - expected).abs() <= tolerance,
                    "wheel {index}: {} vs {expected}",
                    wheel.angular_velocity
                );
                checked += 1;
            }
        }
    }
    assert!(checked > 0);
    let vehicle = world.vehicle(tank).unwrap();
    assert_eq!(vehicle.track_wheels(TrackSide::Left), 0..9);
    assert_eq!(vehicle.track_wheels(TrackSide::Right), 9..18);
    assert_eq!(vehicle.tracks().map(|track| track.driven_wheel), [8, 17]);
    assert_eq!(vehicle.wheel_count(), 18);
}

#[test]
fn brake_stops_the_tank() {
    let (mut world, _, chassis, tank) = settled_tank();
    drive(&mut world, tank, tracks(1.0, 1.0, 1.0), 120);
    let moving = world.body(chassis).unwrap().linear_velocity().z;
    assert!(moving > 1.0, "speed {moving}");
    let brake = TrackedDriverInput {
        brake: 1.0,
        ..TrackedDriverInput::default()
    };
    drive(&mut world, tank, brake, 180);
    let body = world.body(chassis).unwrap();
    let speed = norm(f3(body.linear_velocity()));
    assert!(speed < 0.05, "speed {speed}");
    let [left, right] = world.vehicle(tank).unwrap().tracks();
    assert_eq!((left.angular_velocity, right.angular_velocity), (0.0, 0.0));
}

#[test]
fn tracked_input_reads_back() {
    let (mut world, _, _, tank) = settled_tank();
    assert_eq!(
        world.vehicle(tank).unwrap().driver_input(),
        tracks(0.0, 1.0, 1.0)
    );
    let input = TrackedDriverInput {
        forward: -0.5,
        left_ratio: 0.25,
        right_ratio: -1.0,
        brake: 0.75,
    };
    world
        .vehicle_mut(tank)
        .unwrap()
        .set_driver_input(input)
        .unwrap();
    assert_eq!(world.vehicle(tank).unwrap().driver_input(), input);
    let vehicle = world.vehicle(tank).unwrap();
    assert!(vehicle.engine_rpm() >= 500.0);
    assert!((-2..=4).contains(&vehicle.current_gear()));
}

#[test]
fn tracked_inputs_are_validated() {
    let (mut world, _, chassis, tank) = settled_tank();
    let floor = 1.0 / limits::MAX_RATIO;
    let below = f32::from_bits(floor.to_bits() - 1);
    let refused_ratios = [
        0.0,
        1.0e-30,
        -1.0e-30,
        f32::NAN,
        1.0001,
        -1.0001,
        below,
        -below,
    ];
    let before = world.vehicle(tank).unwrap().driver_input();
    for ratio in refused_ratios {
        for input in [tracks(1.0, ratio, 1.0), tracks(1.0, 1.0, ratio)] {
            assert!(
                matches!(
                    world.vehicle_mut(tank).unwrap().set_driver_input(input),
                    Err(VehicleError::InvalidValue(_))
                ),
                "{input:?}"
            );
            assert_eq!(world.vehicle(tank).unwrap().driver_input(), before);
        }
    }
    for input in [
        tracks(1.01, 1.0, 1.0),
        tracks(f32::NAN, 1.0, 1.0),
        TrackedDriverInput {
            brake: -0.1,
            ..TrackedDriverInput::default()
        },
        TrackedDriverInput {
            brake: f32::INFINITY,
            ..TrackedDriverInput::default()
        },
    ] {
        assert!(world
            .vehicle_mut(tank)
            .unwrap()
            .set_driver_input(input)
            .is_err());
        assert_eq!(world.vehicle(tank).unwrap().driver_input(), before);
    }

    // Every sign combination at the floor steps finitely (equal ratios of 1e-30 give NaN in Jolt).
    for (left, right) in [
        (floor, floor),
        (floor, -floor),
        (-floor, floor),
        (-floor, -floor),
    ] {
        drive(&mut world, tank, tracks(1.0, left, right), 120);
        let vehicle = world.vehicle(tank).unwrap();
        for track in vehicle.tracks() {
            assert!(track.angular_velocity.is_finite(), "{left} {right}");
        }
        for wheel in vehicle.wheels() {
            assert!(wheel.angular_velocity.is_finite(), "{left} {right}");
        }
        let body = world.body(chassis).unwrap();
        let [x, y, z] = v3(body.position());
        assert!(x.is_finite() && y.is_finite() && z.is_finite());
        assert!(f3(body.linear_velocity()).iter().all(|v| v.is_finite()));
    }
}

#[test]
fn invalid_tracked_vehicles_create_nothing() {
    let (mut world, layers) = car_world(GRAVITY, 1);
    add_ground(&mut world, &layers, Quat::IDENTITY);
    let body = behaviour_chassis(&layers, 4000.0, RVec3::new(0.0, 2.0, 0.0), Quat::IDENTITY);
    let chassis = world.create_body(&tank_chassis_shape(), &body).unwrap();

    // Every unusable chassis, created before the state is saved.
    let shape = tank_chassis_shape();
    let fixed = world
        .create_body(&shape, &BodySettings::new_static())
        .unwrap();
    let kinematic = world
        .create_body(&shape, &BodySettings::new_kinematic())
        .unwrap();
    let movable_static = world
        .create_body(
            &shape,
            &BodySettings::new_static().allow_dynamic_or_kinematic(true),
        )
        .unwrap();
    let restricted = world
        .create_body(
            &shape,
            &BodySettings::new_dynamic()
                .position(RVec3::new(30.0, 2.0, 0.0))
                .allowed_dofs(AllowedDofs::PLANE_2D),
        )
        .unwrap();
    let capsule = Shape::new_capsule(0.7, 0.4).unwrap();
    let character = world
        .create_character(
            &CharacterSettings::new(&capsule).inner_body(Some(InnerBody {
                shape: &capsule,
                object_layer: layers.moving,
            })),
            RVec3::new(20.0, 1.0, 0.0),
            Quat::IDENTITY,
        )
        .unwrap();
    let inner = world.character(character).unwrap().inner_body().unwrap();
    let ragdoll = world
        .create_ragdoll(
            &common::ragdoll::humanoid_settings(layers.moving),
            None,
            Activation::Activate,
        )
        .unwrap();
    let part = world.ragdoll(ragdoll).unwrap().body_ids()[0];
    let cloth = world
        .create_soft_body(
            &common::soft_body::Cloth::new(4, 0.5).settings(),
            &SoftBodySettings::default().position(RVec3::new(-20.0, 1.0, 0.0)),
        )
        .unwrap();

    let state = world.save_state();
    let bodies = world.body_count();
    let unchanged = |world: &PhysicsWorld| {
        assert_eq!(world.body_count(), bodies);
        assert_eq!(world.vehicle_ids().count(), 0);
        assert_eq!(world.vehicle_of_body(chassis), None);
    };

    let tester = VehicleCollisionTester::ray(layers.probe);
    let settings = tank_settings(tester);
    let refused = [
        TrackedVehicleSettings::new(
            VehicleTrackSettings::new(Vec::new(), 0),
            tank_track(-1.7),
            tester,
        ),
        TrackedVehicleSettings::new(
            tank_track(1.7),
            VehicleTrackSettings::new(tank_track_wheels(-1.7), 9),
            tester,
        ),
        TrackedVehicleSettings::new(tank_track(1.7).inertia(0.0), tank_track(-1.7), tester),
        TrackedVehicleSettings::new(tank_track(1.7), tank_track(-1.7).inertia(1.0e-40), tester),
        TrackedVehicleSettings::new(
            tank_track(1.7),
            tank_track(-1.7).differential_ratio(1.0e5),
            tester,
        )
        .engine(TrackedVehicleSettings::default_engine().max_torque(1.0e30))
        .transmission(TrackedVehicleSettings::default_transmission().gear_ratios(vec![1.0e4])),
        tank_settings(VehicleCollisionTester::ray(ObjectLayer::new(3))),
    ];
    for settings in &refused {
        assert!(matches!(
            world.create_tracked_vehicle(chassis, settings),
            Err(VehicleError::InvalidValue(_))
        ));
        unchanged(&world);
    }
    for body in [fixed, kinematic, movable_static] {
        assert_eq!(
            world.create_tracked_vehicle(body, &settings),
            Err(VehicleError::NotDynamic(body))
        );
    }
    let body_refusals = [
        (restricted, BodyError::RestrictedDofs(restricted)),
        (inner, BodyError::OwnedByCharacter(inner)),
        (part, BodyError::OwnedByRagdoll(part)),
        (cloth, BodyError::NotRigidBody(cloth)),
    ];
    for (body, error) in body_refusals {
        assert_eq!(
            world.create_tracked_vehicle(body, &settings),
            Err(VehicleError::Body(error))
        );
    }
    unchanged(&world);
    assert_eq!(world.restore_state(&state), Ok(()));

    let tank = world.create_tracked_vehicle(chassis, &settings).unwrap();
    assert_eq!(tank.to_raw(), 1, "the refusals took no id");
    assert_eq!(
        world.create_tracked_vehicle(chassis, &settings),
        Err(VehicleError::AlreadyHasVehicle(chassis))
    );
}

#[test]
fn tank_chassis_is_guarded() {
    let (mut world, layers, chassis, tank) = settled_tank();
    assert_eq!(
        world.remove_body(chassis),
        Err(BodyError::UsedByVehicle(chassis))
    );
    let other_shape = Shape::new_box(Vec3::new(1.0, 1.0, 1.0)).unwrap();
    assert_eq!(
        world
            .body_mut(chassis)
            .unwrap()
            .set_shape(&other_shape, None, Activation::Activate),
        Err(BodyError::UsedByVehicle(chassis))
    );
    assert_eq!(
        world
            .body_mut(chassis)
            .unwrap()
            .set_motion_type(MotionType::Kinematic, Activation::Activate),
        Err(BodyError::UsedByVehicle(chassis))
    );

    let any = world.vehicle_of_body(chassis).unwrap();
    assert_eq!(any.kind(), VehicleType::Tracked);
    assert_eq!(any, tank.into());
    assert_eq!(any.downcast::<TrackedVehicle>(), Some(tank));
    assert_eq!(any.downcast::<Motorcycle>(), None);
    assert_eq!(any.downcast::<WheeledVehicle>(), None);
    assert_eq!(format!("{tank:?}"), "VehicleId(Tracked, 1)");
    assert_eq!(format!("{any:?}"), "VehicleId(Tracked, 1)");

    // Ids are sequential across kinds; removal in mixed order works.
    let car_body = chassis_settings(&layers, RVec3::new(10.0, 1.0, 0.0), Quat::IDENTITY);
    let (_, car) = common::vehicle::add_car_with(
        &mut world,
        &car_body,
        VehicleCollisionTester::ray(layers.probe),
    );
    let (_, second_tank) = add_tank(
        &mut world,
        &layers,
        RVec3::new(-10.0, 2.0, 0.0),
        Quat::IDENTITY,
    );
    assert_eq!((car.to_raw(), second_tank.to_raw()), (2, 3));
    assert_eq!(
        world
            .vehicle_ids()
            .map(|id| (id.to_raw(), id.kind()))
            .collect::<Vec<_>>(),
        [
            (1, VehicleType::Tracked),
            (2, VehicleType::Wheeled),
            (3, VehicleType::Tracked)
        ]
    );
    world.remove_vehicle(car).unwrap();
    world.remove_vehicle(tank).unwrap();
    assert_eq!(
        world.vehicle(tank).err(),
        Some(VehicleError::NotFound(tank.into()))
    );
    assert_eq!(
        world.remove_vehicle(tank),
        Err(VehicleError::NotFound(tank.into()))
    );
    world.remove_body(chassis).unwrap();
    step(&mut world, 2);

    let (mut other, other_layers) = car_world(GRAVITY, 1);
    let (_, foreign) = add_tank(
        &mut other,
        &other_layers,
        RVec3::new(0.0, 2.0, 0.0),
        Quat::IDENTITY,
    );
    assert_eq!(
        world.vehicle(foreign).err(),
        Some(VehicleError::WrongWorld(foreign.into()))
    );
    drop(world);
    drop(other);
}

#[test]
fn tank_testers() {
    let (mut world, layers) = car_world(GRAVITY, 1);
    let ground = add_ground(&mut world, &layers, Quat::IDENTITY);
    let testers = [
        VehicleCollisionTester::ray(layers.probe),
        VehicleCollisionTester::cast_sphere(layers.probe, 0.2),
        VehicleCollisionTester::cast_cylinder(layers.probe),
    ];
    let mut tanks = Vec::new();
    for (index, tester) in testers.into_iter().enumerate() {
        let body = behaviour_chassis(
            &layers,
            4000.0,
            RVec3::new(10.0 * index as Real, 2.0, 0.0),
            Quat::IDENTITY,
        );
        let (chassis, tank) = add_tank_with(&mut world, &body, tester);
        world
            .vehicle_mut(tank)
            .unwrap()
            .set_driver_input(tracks(1.0, 1.0, 1.0))
            .unwrap();
        tanks.push((chassis, tank));
    }
    step(&mut world, 120);
    for (index, &(chassis, tank)) in tanks.iter().enumerate() {
        let z = v3(world.body(chassis).unwrap().position())[2];
        assert!(z > 1.0, "tester {index}: z {z}");
        let in_contact = world
            .vehicle(tank)
            .unwrap()
            .wheels()
            .iter()
            .filter(|wheel| wheel.contact.is_some_and(|contact| contact.body == ground))
            .count();
        assert!(in_contact >= 14, "tester {index}: {in_contact} wheels down");
    }
    // A replaced tester takes effect on the next step: one on the ground layer sees no ground.
    let (_, tank) = tanks[0];
    let blind = VehicleCollisionTester::ray(layers.ground);
    world
        .vehicle_mut(tank)
        .unwrap()
        .set_collision_tester(blind)
        .unwrap();
    assert_eq!(world.vehicle(tank).unwrap().collision_tester(), blind);
    step(&mut world, 1);
    assert!(world
        .vehicle(tank)
        .unwrap()
        .wheels()
        .iter()
        .all(|wheel| wheel.contact.is_none()));
    // Removing the ground the wheels stood on leaves them hanging, without a fault.
    world.remove_body(ground).unwrap();
    step(&mut world, 2);
    for &(_, tank) in &tanks {
        assert!(world
            .vehicle(tank)
            .unwrap()
            .wheels()
            .iter()
            .all(|wheel| wheel.contact.is_none()));
    }
}

/// A pressurised ball below the tank's clearance never shows up in a wheel contact: the
/// testers look through soft bodies.
#[test]
fn tank_wheels_pass_through_soft_bodies() {
    let (mut world, layers) = car_world(GRAVITY, 2);
    let ground = add_ground(&mut world, &layers, Quat::IDENTITY);
    let (vertices, faces) = common::soft_body::sphere(0.3, 7, 14);
    let ball = SoftBodySharedSettings::builder(vertices, faces)
        .create_constraints(
            SoftBodyBendType::None,
            SoftBodyVertexAttributes::default()
                .compliance(1.0e-4)
                .shear_compliance(1.0e-4),
        )
        .build()
        .unwrap();
    world
        .create_soft_body(
            &ball,
            &SoftBodySettings::default()
                .object_layer(layers.moving)
                .position(RVec3::new(1.7, 0.3, 4.0))
                .pressure(1000.0)
                .allow_sleeping(false),
        )
        .unwrap();
    let (chassis, tank) = add_tank(
        &mut world,
        &layers,
        RVec3::new(0.0, 1.2, 0.0),
        Quat::IDENTITY,
    );
    world
        .vehicle_mut(tank)
        .unwrap()
        .set_driver_input(tracks(1.0, 1.0, 1.0))
        .unwrap();
    let mut ground_contacts = 0;
    for _ in 0..240 {
        step(&mut world, 1);
        for wheel in world.vehicle(tank).unwrap().wheels() {
            if let Some(contact) = wheel.contact {
                assert_eq!(contact.body, ground);
                ground_contacts += 1;
            }
        }
        let [x, y, z] = v3(world.body(chassis).unwrap().position());
        assert!(x.is_finite() && y.is_finite() && z.is_finite());
    }
    assert!(ground_contacts > 0);
    let z = v3(world.body(chassis).unwrap().position())[2];
    assert!(z > 5.0, "the tank drove over the ball's place: z {z}");
}

#[test]
fn rebase_rotates_a_tanks_gravity_override_and_tester_up() {
    let (mut world, layers) = car_world(Vec3::ZERO, 1);
    let ground = add_ground(&mut world, &layers, Quat::IDENTITY);
    let body = behaviour_chassis(&layers, 4000.0, RVec3::new(0.0, 1.2, 0.0), Quat::IDENTITY)
        .gravity_factor(0.0);
    let (chassis, tank) =
        add_tank_with(&mut world, &body, VehicleCollisionTester::ray(layers.probe));
    let mut vehicle = world.vehicle_mut(tank).unwrap();
    vehicle.set_gravity(GRAVITY).unwrap();
    vehicle.set_driver_input(tracks(1.0, 1.0, 1.0)).unwrap();
    step(&mut world, 60);

    let rotation = about_x(30.0);
    world
        .rebase(&[ground, chassis], rotation, RVec3::new(5.0, 0.0, -3.0))
        .unwrap();
    let vehicle = world.vehicle(tank).unwrap();
    let expected_gravity = rotate(rotation, f3(GRAVITY));
    let gravity = f3(vehicle.gravity().unwrap());
    assert!(norm(sub(gravity, expected_gravity)) < 1e-5, "{gravity:?}");
    let up = f3(vehicle.collision_tester().up().unwrap());
    assert!(
        norm(sub(up, rotate(rotation, [0.0, 1.0, 0.0]))) < 1e-6,
        "{up:?}"
    );
    let gravity = vehicle.gravity().unwrap();
    for _ in 0..30 {
        world
            .vehicle_mut(tank)
            .unwrap()
            .set_gravity(gravity)
            .unwrap();
        step(&mut world, 1);
    }
    let down = world
        .vehicle(tank)
        .unwrap()
        .wheels()
        .iter()
        .filter(|wheel| wheel.contact.is_some())
        .count();
    assert!(down >= 14, "{down} wheels down after the rebase");
}
