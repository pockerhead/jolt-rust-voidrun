//! Motorcycles: the sample bike stays upright at speed and leans into a turn, falls over
//! without its lean controller, gets no lean torque unless both wheels touch, and the settings,
//! chassis guards, testers, soft bodies and rebase work as for the other kinds.

mod common;

use common::math::{add, f3, norm, rotate, sub, v3};
use common::vehicle::{car_world, CarLayers, GRAVITY};
use common::vehicle_kinds::*;
use common::*;
use oxijolt::*;

/// A rotation of `degrees` about the unit `axis`.
fn about(axis: [f32; 3], degrees: f32) -> Quat {
    let half = 0.5 * degrees.to_radians();
    let s = half.sin();
    Quat::from_xyzw(axis[0] * s, axis[1] * s, axis[2] * s, half.cos())
}

/// A static box ground on the ground layer whose top face is the plane y = 0.
fn add_ground(world: &mut PhysicsWorld, layers: &CarLayers) -> BodyId {
    let shape = Shape::new_box(Vec3::new(100.0, 1.0, 100.0)).unwrap();
    world
        .create_body(
            &shape,
            &BodySettings::new_static()
                .position(RVec3::new(0.0, -1.0, 0.0))
                .object_layer(layers.ground),
        )
        .unwrap()
}

fn drive(world: &mut PhysicsWorld, bike: VehicleId<Motorcycle>, input: DriverInput, ticks: usize) {
    for _ in 0..ticks {
        world
            .vehicle_mut(bike)
            .unwrap()
            .set_driver_input(input)
            .unwrap();
        step(world, 1);
    }
}

/// The world y of the chassis' up.
fn up_y(world: &PhysicsWorld, chassis: BodyId) -> f64 {
    rotate(world.body(chassis).unwrap().rotation(), [0.0, 1.0, 0.0])[1]
}

fn both_wheels_down(world: &PhysicsWorld, bike: VehicleId<Motorcycle>) -> bool {
    world
        .vehicle(bike)
        .unwrap()
        .wheels()
        .iter()
        .all(|wheel| wheel.contact.is_some())
}

/// The sample bike dropped at y = 2 on flat ground, with its world and layers.
fn sample_bike(threads: u32) -> (PhysicsWorld, CarLayers, BodyId, VehicleId<Motorcycle>) {
    let (mut world, layers) = car_world(GRAVITY, threads);
    add_ground(&mut world, &layers);
    let (chassis, bike) = add_bike(
        &mut world,
        &layers,
        RVec3::new(0.0, 2.0, 0.0),
        Quat::IDENTITY,
    );
    (world, layers, chassis, bike)
}

#[test]
fn motorcycle_stays_upright_at_speed() {
    let (mut world, _, chassis, bike) = sample_bike(1);
    drive(&mut world, bike, ride(0.4, 0.0), 180);
    let speed = norm(f3(world.body(chassis).unwrap().linear_velocity()));
    let up = up_y(&world, chassis);
    let lean = world.vehicle(bike).unwrap().lean();
    eprintln!("straight: speed {speed} m/s, up.y {up}, lean {lean:?}");
    assert!(speed > 4.0, "speed {speed}");
    assert!(up > 0.99, "up.y {up}");
    assert!(lean.angle.abs() < 0.1, "{lean:?}");
    assert!(both_wheels_down(&world, bike));
}

#[test]
fn motorcycle_leans_into_a_turn() {
    let (mut world, _, chassis, bike) = sample_bike(1);
    drive(&mut world, bike, ride(0.4, 0.0), 180);
    drive(&mut world, bike, ride(0.4, 0.15), 120);
    let body = world.body(chassis).unwrap();
    let yaw_rate = body.angular_velocity().y;
    let up = up_y(&world, chassis);
    let lean = world.vehicle(bike).unwrap().lean();
    eprintln!("turn: yaw rate {yaw_rate} rad/s, up.y {up}, lean {lean:?}");
    assert!(yaw_rate < -0.2, "yaw rate {yaw_rate}");
    assert!(lean.angle > 0.1, "{lean:?}");
    assert!(lean.target_angle > 0.0, "{lean:?}");
    assert!(up > 0.95, "up.y {up}");
    assert!(both_wheels_down(&world, bike));
}

#[test]
fn without_the_lean_controller_it_falls_over() {
    let (mut world, layers) = car_world(GRAVITY, 1);
    add_ground(&mut world, &layers);
    let rolled = about([0.0, 0.0, 1.0], 3.0);
    let mut bikes = Vec::new();
    for (x, enabled) in [(0.0, true), (5.0, false)] {
        let settings = MotorcycleSettings::new(
            bike_vehicle_settings(bike_tester(&layers)).max_pitch_roll_angle(std::f32::consts::PI),
        )
        .lean_controller(enabled);
        let body = behaviour_chassis(&layers, BIKE_MASS, RVec3::new(x, 1.0, 0.0), rolled);
        bikes.push(add_bike_with(&mut world, &body, &settings));
    }
    let [(upright, on), (falling, off)] = [bikes[0], bikes[1]];
    let vehicle = world.vehicle(on).unwrap();
    assert!(vehicle.is_lean_controller_enabled());
    assert!(vehicle.is_lean_steering_limit_enabled());
    assert!(!world.vehicle(off).unwrap().is_lean_controller_enabled());
    // Jolt starts every target lean at the zero vector.
    assert_eq!(world.vehicle(off).unwrap().lean().target, Vec3::ZERO);

    let mut lowest = [1.0_f64; 2];
    for tick in 0..300 {
        step(&mut world, 1);
        lowest[0] = lowest[0].min(up_y(&world, upright));
        lowest[1] = lowest[1].min(up_y(&world, falling));
        let vehicle = world.vehicle(off).unwrap();
        // Without the controller, Jolt sets the target lean to the world up of the step.
        let bits = |v: Vec3| <[f32; 3]>::from(v).map(f32::to_bits);
        assert_eq!(
            bits(vehicle.lean().target),
            bits(vehicle.world_up()),
            "tick {tick}"
        );
    }
    eprintln!(
        "fall over: lowest up.y with the controller {}, without {}",
        lowest[0], lowest[1]
    );
    assert!(lowest[0] > 0.95, "{lowest:?}");
    assert!(up_y(&world, falling) < 0.5, "{lowest:?}");
}

/// The chassis pose and velocities as exact bits.
fn chassis_bits(world: &PhysicsWorld, chassis: BodyId) -> Vec<u8> {
    let mut digest = Vec::new();
    record_body(world, chassis, &mut digest);
    digest
}

#[test]
fn no_lean_torque_unless_both_wheels_touch() {
    // Two worlds, one bike each, the same but for the lean controller. Nose up, so the rear
    // wheel lands first, and rolling.
    let pose = about([1.0, 0.0, 0.0], -8.0);
    let mut worlds = Vec::new();
    for enabled in [true, false] {
        let (mut world, layers) = car_world(GRAVITY, 1);
        add_ground(&mut world, &layers);
        let settings = MotorcycleSettings::new(bike_vehicle_settings(bike_tester(&layers)))
            .lean_controller(enabled);
        let body = behaviour_chassis(&layers, BIKE_MASS, RVec3::new(0.0, 1.5, 0.0), pose)
            .angular_velocity(Vec3::new(0.0, 0.0, 0.5));
        let (chassis, bike) = add_bike_with(&mut world, &body, &settings);
        worlds.push((world, chassis, bike));
    }
    let mut single_contact_ticks = 0;
    let mut first_both = None;
    for tick in 0..240 {
        for (world, _, _) in &mut worlds {
            step(world, 1);
        }
        let (world, chassis, bike) = &worlds[0];
        let wheels = world.vehicle(*bike).unwrap().wheels();
        let both = wheels
            .iter()
            .all(|wheel| wheel.contact.is_some() && wheel.suspension_lambda > 0.0);
        if both {
            first_both.get_or_insert(tick);
        }
        if first_both.is_none() {
            let (other, other_chassis, _) = &worlds[1];
            assert_eq!(
                chassis_bits(world, *chassis),
                chassis_bits(other, *other_chassis),
                "tick {tick}"
            );
            if wheels.iter().any(|wheel| wheel.contact.is_some()) {
                single_contact_ticks += 1;
            }
        }
    }
    let first_both = first_both.expect("both wheels land");
    eprintln!(
        "lean torque: first tick with both wheels down {first_both}, single-contact ticks {single_contact_ticks}"
    );
    assert!(single_contact_ticks > 0, "the rear wheel lands first");
    let [(on, chassis_on, _), (off, chassis_off, _)] = [&worlds[0], &worlds[1]];
    assert_ne!(
        chassis_bits(on, *chassis_on),
        chassis_bits(off, *chassis_off)
    );
}

/// The largest principal inverse inertia Jolt gives a box of `half` extents and `mass`: the
/// box's own, or Jolt's unit-sphere fallback `2.5 / mass` when the inertia diagonal is near
/// zero (`MotionProperties::SetMassProperties`, length squared at most 1e-12).
fn box_inverse_inertia(half: Vec3, mass: f32) -> f64 {
    let [a, b, c] = [half.x, half.y, half.z].map(f64::from);
    let mass = f64::from(mass);
    let inertia = [b * b + c * c, a * a + c * c, a * a + b * b].map(|sum| mass * sum / 3.0);
    if inertia.iter().map(|i| i * i).sum::<f64>() <= 1.0e-12 {
        2.5 / mass
    } else {
        1.0 / inertia.into_iter().fold(f64::INFINITY, f64::min)
    }
}

/// The lean spring constant (with no damping) at `fraction` of the rule's bound for a chassis
/// whose largest principal inverse inertia is `inverse_inertia`.
fn spring_at(fraction: f64, inverse_inertia: f64) -> f32 {
    let bound =
        f64::from(limits::MAX_ANGULAR_ACCELERATION) / inverse_inertia / std::f64::consts::PI;
    (fraction * bound) as f32
}

#[test]
fn motorcycle_settings_are_refused() {
    let (mut world, layers) = car_world(GRAVITY, 1);
    add_ground(&mut world, &layers);
    let chassis = world
        .create_body(
            &bike_chassis_shape(),
            &behaviour_chassis(
                &layers,
                BIKE_MASS,
                RVec3::new(0.0, 2.0, 0.0),
                Quat::IDENTITY,
            ),
        )
        .unwrap();
    // Light chassis of 1 g: cubes around Jolt's near-zero inertia threshold (the 2 cm cube
    // gets Jolt's fallback inertia, the larger ones their own) and an anisotropic plank.
    let light: Vec<(BodyId, f64)> = [
        Vec3::new(0.02, 0.02, 0.02),
        Vec3::new(0.03, 0.03, 0.03),
        Vec3::new(0.035, 0.035, 0.035),
        Vec3::new(0.02, 0.3, 0.4),
    ]
    .into_iter()
    .enumerate()
    .map(|(index, half)| {
        let body = BodySettings::new_dynamic()
            .position(RVec3::new(3.0 * index as Real + 3.0, 2.0, 0.0))
            .object_layer(layers.moving)
            .mass(limits::MIN_MASS);
        let id = world
            .create_body(&Shape::new_box(half).unwrap(), &body)
            .unwrap();
        (id, box_inverse_inertia(half, limits::MIN_MASS))
    })
    .collect();
    let state = world.save_state();
    let bodies = world.body_count();
    let settings = || MotorcycleSettings::new(bike_vehicle_settings(bike_tester(&layers)));
    let unchanged = |world: &PhysicsWorld| {
        assert_eq!(world.body_count(), bodies);
        assert_eq!(world.vehicle_ids().count(), 0);
    };

    assert_eq!(
        world.create_motorcycle(
            chassis,
            &settings().lean_spring_integration_coefficient(1.0e-3)
        ),
        Err(VehicleError::LeanSpringIntegrationNotSaved)
    );
    for refused in [
        settings().max_lean_angle(MotorcycleSettings::MAX_LEAN_ANGLE * 1.01),
        settings().max_lean_angle(-0.1),
        settings().max_lean_angle(f32::NAN),
        settings().lean_smoothing_factor(-0.1),
        settings().lean_smoothing_factor(1.1),
    ] {
        assert!(matches!(
            world.create_motorcycle(chassis, &refused),
            Err(VehicleError::InvalidValue(_))
        ));
    }
    let lean_spring_refused = Err(VehicleError::InvalidValue(
        "lean spring would exceed limits::MAX_ANGULAR_ACCELERATION for this chassis",
    ));
    for &(body, inverse_inertia) in &light {
        // Jolt's default springs, and a spring just above the bound for this chassis.
        assert_eq!(
            world.create_motorcycle(body, &settings()),
            lean_spring_refused
        );
        let strong = settings()
            .lean_spring_constant(spring_at(1.01, inverse_inertia))
            .lean_spring_damping(0.0);
        assert_eq!(world.create_motorcycle(body, &strong), lean_spring_refused);
    }
    unchanged(&world);
    assert_eq!(world.restore_state(&state), Ok(()));

    // Just below the bound is accepted on every light chassis: the rule reads Jolt's inertia,
    // its near-zero fallback included.
    for (index, &(body, inverse_inertia)) in light.iter().enumerate() {
        let weak = settings()
            .lean_spring_constant(spring_at(0.99, inverse_inertia))
            .lean_spring_damping(0.0);
        let bike = world.create_motorcycle(body, &weak).unwrap();
        assert_eq!(bike.to_raw(), index as u32 + 1, "the refusals took no id");
    }
    // The fixture chassis takes Jolt's default springs.
    world.create_motorcycle(chassis, &settings()).unwrap();
}

#[test]
fn lean_spring_at_its_bound_steps_finitely() {
    for dt in [
        PhysicsWorld::MIN_DELTA_TIME,
        1.0 / 60.0,
        PhysicsWorld::MAX_DELTA_TIME,
    ] {
        let (mut world, layers) = car_world(GRAVITY, 1);
        add_ground(&mut world, &layers);
        let half = Vec3::new(0.035, 0.035, 0.035);
        let shape = Shape::new_box(half).unwrap();
        let chassis = world
            .create_body(
                &shape,
                &BodySettings::new_dynamic()
                    .position(RVec3::new(0.0, 0.9, 0.0))
                    .object_layer(layers.moving)
                    .mass(limits::MIN_MASS)
                    .allow_sleeping(false),
            )
            .unwrap();
        // The strongest spring the rule accepts on this chassis.
        let settings = |constant: f32| {
            MotorcycleSettings::new(bike_vehicle_settings(bike_tester(&layers)))
                .lean_spring_constant(constant)
                .lean_spring_damping(0.0)
        };
        let (mut low, mut high) = (0.0_f32, 1.0e12_f32);
        for _ in 0..100 {
            let middle = 0.5 * (low + high);
            match world.create_motorcycle(chassis, &settings(middle)) {
                Ok(id) => {
                    world.remove_vehicle(id).unwrap();
                    low = middle;
                }
                Err(_) => high = middle,
            }
        }
        assert!(low > 0.0 && high / low < 1.0 + 1.0e-6, "{low} {high}");
        let bike = world.create_motorcycle(chassis, &settings(low)).unwrap();
        for tick in 0..120 {
            let right = if tick < 60 { 0.0 } else { 1.0 };
            world
                .vehicle_mut(bike)
                .unwrap()
                .set_driver_input(ride(1.0, right))
                .unwrap();
            assert!(world.step(dt).unwrap().is_complete());
            let body = world.body(chassis).unwrap();
            let [x, y, z] = v3(body.position());
            assert!(
                x.is_finite() && y.is_finite() && z.is_finite(),
                "dt {dt} tick {tick}"
            );
            assert!(f3(body.angular_velocity()).iter().all(|w| w.is_finite()));
            assert!(f3(body.linear_velocity()).iter().all(|v| v.is_finite()));
        }
    }
}

/// `v` rotated by `q` in `f32` with the formula the world's rebase uses,
/// `v + 2w (q × v) + 2 q × (q × v)`.
fn rotate_like_rebase(q: Quat, v: Vec3) -> Vec3 {
    let cross = |a: [f32; 3], b: [f32; 3]| {
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    };
    let u = [q.x, q.y, q.z];
    let v = <[f32; 3]>::from(v);
    let t = cross(u, v).map(|c| c * 2.0);
    let ut = cross(u, t);
    Vec3::new(
        v[0] + t[0] * q.w + ut[0],
        v[1] + t[1] * q.w + ut[1],
        v[2] + t[2] * q.w + ut[2],
    )
}

/// The sample bike riding straight under world gravity for 60 ticks, then, when `frame` is
/// given, moved into that frame (rotation, translation), then riding 60 more ticks. Returns
/// the chassis pose after each later tick and the target lean just before and after the rebase.
fn ride_across(frame: Option<(Quat, RVec3)>) -> (Vec<(RVec3, Quat)>, Vec3, Vec3) {
    let (mut world, layers) = car_world(GRAVITY, 1);
    let ground = add_ground(&mut world, &layers);
    let (chassis, bike) = add_bike(
        &mut world,
        &layers,
        RVec3::new(0.0, 1.2, 0.0),
        Quat::IDENTITY,
    );
    drive(&mut world, bike, ride(0.4, 0.0), 60);
    let before = world.vehicle(bike).unwrap().lean().target;
    if let Some((rotation, translation)) = frame {
        world
            .rebase(&[ground, chassis], rotation, translation)
            .unwrap();
    }
    let after = world.vehicle(bike).unwrap().lean().target;
    let mut poses = Vec::new();
    for _ in 0..60 {
        drive(&mut world, bike, ride(0.4, 0.0), 1);
        let body = world.body(chassis).unwrap();
        poses.push((body.position(), body.rotation()));
    }
    (poses, before, after)
}

#[test]
fn rebase_rotates_the_target_lean() {
    let rotation = about([0.0, 0.0, 1.0], 25.0);
    let translation = RVec3::new(40.0, -12.0, 7.0);
    let (unrebased, before, unmoved) = ride_across(None);
    let (rebased, before_rebase, after) = ride_across(Some((rotation, translation)));
    assert_eq!(before, before_rebase);
    assert_eq!(unmoved, before);
    let bits = |v: Vec3| <[f32; 3]>::from(v).map(f32::to_bits);
    assert_eq!(bits(after), bits(rotate_like_rebase(rotation, before)));

    // The ride continues in the new frame like the unrebased ride mapped into it, after a
    // transient: the first step after the rebase computes the target lean from each wheel's
    // contact normal and impulses of the step before, still in the old frame. Measured on
    // x86_64: at most 2.0 cm and 0.024 in the chassis up, at tick 11, then decaying to half of
    // that by tick 59.
    let differences: Vec<[f64; 2]> = unrebased
        .iter()
        .zip(&rebased)
        .map(|((p, q), (p_new, q_new))| {
            let mapped = add(rotate(rotation, v3(*p)), v3(translation));
            let up = rotate(rotation, rotate(*q, [0.0, 1.0, 0.0]));
            let up_new = rotate(*q_new, [0.0, 1.0, 0.0]);
            [norm(sub(v3(*p_new), mapped)), norm(sub(up, up_new))]
        })
        .collect();
    let peak = differences
        .iter()
        .fold([0.0_f64; 2], |a, d| [a[0].max(d[0]), a[1].max(d[1])]);
    let last = differences[differences.len() - 1];
    eprintln!("rebase: peak {peak:?}, last {last:?}");
    assert!(peak[0] < 0.03 && peak[1] < 0.035, "{peak:?}");
    assert!(
        last[0] < 0.75 * peak[0] && last[1] < 0.75 * peak[1],
        "{last:?} {peak:?}"
    );
}

#[test]
fn motorcycle_testers() {
    let (mut world, layers) = car_world(GRAVITY, 1);
    let ground = add_ground(&mut world, &layers);
    let testers = [
        VehicleCollisionTester::ray(layers.probe),
        VehicleCollisionTester::cast_sphere(layers.probe, 0.2),
        bike_tester(&layers),
    ];
    let mut bikes = Vec::new();
    for (index, tester) in testers.into_iter().enumerate() {
        let body = behaviour_chassis(
            &layers,
            BIKE_MASS,
            RVec3::new(5.0 * index as Real, 1.2, 0.0),
            Quat::IDENTITY,
        );
        let settings = MotorcycleSettings::new(bike_vehicle_settings(tester));
        bikes.push(add_bike_with(&mut world, &body, &settings));
    }
    for _ in 0..120 {
        for &(_, bike) in &bikes {
            world
                .vehicle_mut(bike)
                .unwrap()
                .set_driver_input(ride(0.4, 0.0))
                .unwrap();
        }
        step(&mut world, 1);
    }
    for (index, &(chassis, bike)) in bikes.iter().enumerate() {
        let z = v3(world.body(chassis).unwrap().position())[2];
        assert!(z > 2.0, "tester {index}: z {z}");
        assert!(up_y(&world, chassis) > 0.99, "tester {index}");
        let wheels = world.vehicle(bike).unwrap().wheels();
        assert!(
            wheels
                .iter()
                .all(|wheel| wheel.contact.is_some_and(|c| c.body == ground)),
            "tester {index}: {wheels:?}"
        );
    }
    let (_, bike) = bikes[0];
    let blind = VehicleCollisionTester::ray(layers.ground);
    world
        .vehicle_mut(bike)
        .unwrap()
        .set_collision_tester(blind)
        .unwrap();
    assert_eq!(*world.vehicle(bike).unwrap().collision_tester(), blind);
    step(&mut world, 1);
    assert!(!both_wheels_down(&world, bike));
    world.remove_body(ground).unwrap();
    step(&mut world, 2);
    for &(_, bike) in &bikes {
        let wheels = world.vehicle(bike).unwrap().wheels();
        assert!(wheels.iter().all(|wheel| wheel.contact.is_none()));
    }
}

/// A pressurised ball on the bike's path, below its chassis: the wheels roll through it to
/// the ground, as the testers look through soft bodies.
#[test]
fn motorcycle_wheels_pass_through_soft_bodies() {
    let (mut world, layers) = car_world(GRAVITY, 2);
    let ground = add_ground(&mut world, &layers);
    let (vertices, faces) = common::soft_body::sphere(0.2, 7, 14);
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
                .position(RVec3::new(0.0, 0.2, 2.5))
                .pressure(1000.0)
                .allow_sleeping(false),
        )
        .unwrap();
    let (chassis, bike) = add_bike(
        &mut world,
        &layers,
        RVec3::new(0.0, 1.2, 0.0),
        Quat::IDENTITY,
    );
    let mut ground_contacts = 0;
    for _ in 0..180 {
        drive(&mut world, bike, ride(0.4, 0.0), 1);
        for wheel in world.vehicle(bike).unwrap().wheels() {
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
    assert!(z > 4.0, "the bike rode past the ball: z {z}");
}

#[test]
fn motorcycle_chassis_is_guarded() {
    let (mut world, layers, chassis, bike) = sample_bike(1);
    step(&mut world, 30);
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
    assert_eq!(any.kind(), VehicleType::Motorcycle);
    assert_eq!(any.downcast::<Motorcycle>(), Some(bike));
    assert_eq!(any.downcast::<WheeledVehicle>(), None);
    assert_eq!(any.downcast::<TrackedVehicle>(), None);
    assert_eq!(format!("{bike:?}"), "VehicleId(Motorcycle, 1)");

    let (_, tank) = add_tank(
        &mut world,
        &layers,
        RVec3::new(-10.0, 2.0, 0.0),
        Quat::IDENTITY,
    );
    assert_eq!(tank.to_raw(), 2);
    world.remove_vehicle(bike).unwrap();
    assert_eq!(
        world.vehicle(bike).err(),
        Some(VehicleError::NotFound(bike.into()))
    );
    assert_eq!(
        world.vehicle_mut(bike).err().map(|error| error.to_string()),
        Some("no vehicle VehicleId(Motorcycle, 1) in this world".to_owned())
    );
    world.remove_body(chassis).unwrap();
    step(&mut world, 2);
    let (other, _, _, foreign) = sample_bike(1);
    assert_eq!(
        world.vehicle(foreign).err(),
        Some(VehicleError::WrongWorld(foreign.into()))
    );
    drop(other);
}
