//! Test fixtures of the other vehicle kinds, shaped like Jolt's `TankTest` and `MotorcycleTest`
//! samples; the motorcycle is described at [`bike_vehicle_settings`].
//!
//! The tank: a box chassis with half extents (1.7, 0.5, 3.2), its centre of mass 0.5 below the
//! box centre, 4000 kg; nine wheels per track at x = ±1.7 (left track +X) and z from 2.95 to
//! −2.75, radius 0.3, width 0.1, springs of 1 Hz; the end wheels sit at y = 0 with a fixed
//! suspension length of 0.3, the others at y = −0.3 with travel 0.3 to 0.5. Each track is
//! driven at its rearmost wheel: vehicle wheels 8 (left) and 17 (right).

use oxijolt::*;

use super::vehicle::CarLayers;

/// z of the tank's wheels in each track, front to back.
pub const TANK_WHEEL_Z: [f32; 9] = [2.95, 2.1, 1.4, 0.7, 0.0, -0.7, -1.4, -2.1, -2.75];
/// Radius of every tank wheel.
pub const TANK_WHEEL_RADIUS: f32 = 0.3;
/// Index of the driven wheel within each track.
pub const TANK_DRIVEN_WHEEL: u32 = 8;

/// The tank's chassis shape.
pub fn tank_chassis_shape() -> Shape {
    let hull = Shape::new_box(Vec3::new(1.7, 0.5, 3.2)).unwrap();
    Shape::new_offset_center_of_mass(&hull, Vec3::new(0.0, -0.5, 0.0)).unwrap()
}

/// One of the tank's tracks at `x`.
pub fn tank_track(x: f32) -> VehicleTrackSettings {
    VehicleTrackSettings::new(tank_track_wheels(x), TANK_DRIVEN_WHEEL)
}

/// The wheels of the tank's track at `x`, front to back.
pub fn tank_track_wheels(x: f32) -> Vec<TrackedWheelSettings> {
    let last = TANK_WHEEL_Z.len() - 1;
    TANK_WHEEL_Z
        .iter()
        .enumerate()
        .map(|(index, &z)| {
            let end = index == 0 || index == last;
            TrackedWheelSettings::new(Vec3::new(x, if end { 0.0 } else { -0.3 }, z))
                .radius(TANK_WHEEL_RADIUS)
                .width(0.1)
                .suspension_min_length(0.3)
                .suspension_max_length(if end { 0.3 } else { 0.5 })
                .suspension_spring(SpringSettings::FrequencyAndDamping {
                    frequency: 1.0,
                    damping: 0.5,
                })
        })
        .collect()
}

/// The tank's vehicle settings with `tester`, limited to 60° of pitch and roll.
pub fn tank_settings(tester: VehicleCollisionTester) -> TrackedVehicleSettings {
    TrackedVehicleSettings::new(tank_track(1.7), tank_track(-1.7), tester)
        .max_pitch_roll_angle(60.0_f32.to_radians())
}

/// A chassis body under the world's gravity that never sleeps, for behaviour tests.
pub fn behaviour_chassis(
    layers: &CarLayers,
    mass: f32,
    position: RVec3,
    rotation: Quat,
) -> BodySettings {
    BodySettings::new_dynamic()
        .position(position)
        .rotation(rotation)
        .object_layer(layers.moving)
        .mass(mass)
        .allow_sleeping(false)
}

/// Creates a tank chassis from `body` and attaches a tank with `tester` to it.
pub fn add_tank_with(
    world: &mut PhysicsWorld,
    body: &BodySettings,
    tester: VehicleCollisionTester,
) -> (BodyId, VehicleId<TrackedVehicle>) {
    let chassis = world.create_body(&tank_chassis_shape(), body).unwrap();
    let tank = world
        .create_tracked_vehicle(chassis, &tank_settings(tester))
        .unwrap();
    (chassis, tank)
}

/// A tank under the world's gravity at a pose, with a ray tester on the probe layer.
pub fn add_tank(
    world: &mut PhysicsWorld,
    layers: &CarLayers,
    position: RVec3,
    rotation: Quat,
) -> (BodyId, VehicleId<TrackedVehicle>) {
    add_tank_with(
        world,
        &behaviour_chassis(layers, 4000.0, position, rotation),
        VehicleCollisionTester::ray(layers.probe),
    )
}

/// The tracked input with `forward` throttle and track ratios `left` and `right`, no brake.
pub fn tracks(forward: f32, left: f32, right: f32) -> TrackedDriverInput {
    TrackedDriverInput {
        forward,
        left_ratio: left,
        right_ratio: right,
        brake: 0.0,
    }
}

/// The motorcycle's chassis shape: a box with half extents (0.2, 0.3, 0.4), its centre of mass
/// 0.3 below the box centre.
pub fn bike_chassis_shape() -> Shape {
    let frame = Shape::new_box(Vec3::new(0.2, 0.3, 0.4)).unwrap();
    Shape::new_offset_center_of_mass(&frame, Vec3::new(0.0, -0.3, 0.0)).unwrap()
}

/// Mass of the motorcycle's chassis, kg.
pub const BIKE_MASS: f32 = 240.0;

/// The motorcycle of Jolt's `MotorcycleTest` sample with `tester`: wheels of radius 0.31 at
/// z = ±0.75, the front one steering up to 30° about an axis raked 30° (caster), the rear one
/// driven through a differential of ratio 4.825; a 150 N·m engine up to 10000 rpm, six gears;
/// a pitch and roll limit of 60°.
pub fn bike_vehicle_settings(tester: VehicleCollisionTester) -> WheeledVehicleSettings {
    let rake = 30.0_f32.to_radians().tan();
    let length = (1.0 + rake * rake).sqrt();
    let suspension = Vec3::new(0.0, -1.0 / length, rake / length);
    let wheel = |z: f32, frequency: f32, brake: f32| {
        WheelSettings::new(Vec3::new(0.0, -0.27, z))
            .radius(0.31)
            .width(0.05)
            .suspension_min_length(0.3)
            .suspension_max_length(0.5)
            .suspension_spring(SpringSettings::FrequencyAndDamping {
                frequency,
                damping: 0.5,
            })
            .max_brake_torque(brake)
    };
    let front = wheel(0.75, 1.5, 500.0)
        .max_steer_angle(30.0_f32.to_radians())
        .suspension_direction(suspension)
        .steering_axis(Vec3::new(0.0, 1.0 / length, -rake / length));
    let rear = wheel(-0.75, 2.0, 250.0).max_steer_angle(0.0);
    WheeledVehicleSettings::new(
        vec![front, rear],
        vec![VehicleDifferentialSettings::new(None, Some(1)).differential_ratio(1.93 * 40.0 / 16.0)],
        tester,
    )
    .max_pitch_roll_angle(60.0_f32.to_radians())
    .engine(
        VehicleEngineSettings::default()
            .max_torque(150.0)
            .min_rpm(1000.0)
            .max_rpm(10000.0),
    )
    .transmission(
        VehicleTransmissionSettings::default()
            .gear_ratios(vec![2.27, 1.63, 1.3, 1.09, 0.96, 0.88])
            .reverse_gear_ratios(vec![-4.0])
            .shift_down_rpm(2000.0)
            .shift_up_rpm(8000.0)
            .clutch_strength(2.0),
    )
}

/// The cylinder tester of the motorcycle sample: convex radius fraction 1.
pub fn bike_tester(layers: &CarLayers) -> VehicleCollisionTester {
    VehicleCollisionTester::CastCylinder {
        object_layer: layers.probe,
        convex_radius_fraction: 1.0,
    }
}

/// Creates a motorcycle chassis from `body` and attaches a motorcycle of `settings` to it.
pub fn add_bike_with(
    world: &mut PhysicsWorld,
    body: &BodySettings,
    settings: &MotorcycleSettings,
) -> (BodyId, VehicleId<Motorcycle>) {
    let chassis = world.create_body(&bike_chassis_shape(), body).unwrap();
    let bike = world.create_motorcycle(chassis, settings).unwrap();
    (chassis, bike)
}

/// The sample motorcycle under the world's gravity at a pose.
pub fn add_bike(
    world: &mut PhysicsWorld,
    layers: &CarLayers,
    position: RVec3,
    rotation: Quat,
) -> (BodyId, VehicleId<Motorcycle>) {
    add_bike_with(
        world,
        &behaviour_chassis(layers, BIKE_MASS, position, rotation),
        &MotorcycleSettings::new(bike_vehicle_settings(bike_tester(layers))),
    )
}

/// The motorcycle input with `forward` throttle and `right` steering.
pub fn ride(forward: f32, right: f32) -> DriverInput {
    DriverInput {
        forward,
        right,
        ..DriverInput::default()
    }
}
