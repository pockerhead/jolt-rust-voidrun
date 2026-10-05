//! Test fixtures of the other vehicle kinds, shaped like Jolt's `TankTest` sample.
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
                .suspension_spring(SuspensionSpring::FrequencyAndDamping {
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
