//! A test car on the binding, its world layers, the digest of its state, and the terrain route
//! of the acceptance test.
//!
//! The car: a box chassis with half extents (0.9, 0.3, 2.0) and its centre of mass moved to the
//! bottom face, 1500 kg, under the caller's gravity only (gravity factor 0, set at the vehicle
//! every tick), never sleeping; four wheels of radius 0.35 at (±0.9, −0.1, ±1.4) with suspension
//! travel 0.05 to 0.5, front wheels steering up to 30° and driven, hand brake on the rear wheels.
//! Wheel order: front left (+X), front right, rear left, rear right.

use oxijolt::*;

use super::math::{dot, f3, rotate, v3};
use super::DT;

pub const WHEEL_RADIUS: f32 = 0.35;
pub const WHEEL_WIDTH: f32 = 0.2;
pub const SUSPENSION_MIN: f32 = 0.05;
pub const SUSPENSION_MAX: f32 = 0.5;
/// Wheel attachment points in body space, in wheel order.
pub const WHEEL_POSITIONS: [Vec3; 4] = [
    Vec3::new(0.9, -0.1, 1.4),
    Vec3::new(-0.9, -0.1, 1.4),
    Vec3::new(0.9, -0.1, -1.4),
    Vec3::new(-0.9, -0.1, -1.4),
];
/// Gravity the tests apply at the vehicles, m/s².
pub const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);

/// The object layers of a vehicle world: static ground, moving bodies (chassis, slabs) and the
/// wheel probe the collision testers query as, which sees ground and moving bodies.
#[derive(Clone, Copy, Debug)]
pub struct CarLayers {
    pub ground: ObjectLayer,
    pub moving: ObjectLayer,
    pub probe: ObjectLayer,
}

/// A world with [`CarLayers`].
pub fn car_world(gravity: Vec3, worker_threads: u32) -> (PhysicsWorld, CarLayers) {
    let mut layers = CollisionLayers::new(2);
    let fixed = BroadPhaseLayer::new(0);
    let moving_bp = BroadPhaseLayer::new(1);
    let ground = layers.add_object_layer(fixed);
    let moving = layers.add_object_layer(moving_bp);
    let probe = layers.add_object_layer(moving_bp);
    layers
        .enable_collision(moving, ground)
        .enable_collision(moving, moving)
        .enable_collision(probe, ground)
        .enable_collision(probe, moving);
    let world = PhysicsWorld::new(super::jobs::with_threads(
        WorldSettings::default().gravity(gravity).layers(layers),
        worker_threads,
    ))
    .unwrap();
    (
        world,
        CarLayers {
            ground,
            moving,
            probe,
        },
    )
}

/// The chassis shape of the test car.
pub fn chassis_shape() -> Shape {
    let hull = Shape::new_box(Vec3::new(0.9, 0.3, 2.0)).unwrap();
    Shape::new_offset_center_of_mass(&hull, Vec3::new(0.0, -0.3, 0.0)).unwrap()
}

/// The chassis body settings of the test car at a pose.
pub fn chassis_settings(layers: &CarLayers, position: RVec3, rotation: Quat) -> BodySettings {
    BodySettings::new_dynamic()
        .position(position)
        .rotation(rotation)
        .object_layer(layers.moving)
        .mass(1500.0)
        .allow_sleeping(false)
        .gravity_factor(0.0)
}

/// The vehicle settings of the test car with `tester`.
pub fn car_settings(tester: VehicleCollisionTester) -> WheeledVehicleSettings {
    let wheels = WHEEL_POSITIONS
        .iter()
        .enumerate()
        .map(|(index, &position)| {
            let front = index < 2;
            WheelSettings::new(position)
                .radius(WHEEL_RADIUS)
                .width(WHEEL_WIDTH)
                .suspension_min_length(SUSPENSION_MIN)
                .suspension_max_length(SUSPENSION_MAX)
                .max_steer_angle(if front { 30.0_f32.to_radians() } else { 0.0 })
                .max_hand_brake_torque(if front { 0.0 } else { 4000.0 })
        })
        .collect();
    WheeledVehicleSettings::new(
        wheels,
        vec![VehicleDifferentialSettings::new(Some(0), Some(1))],
        tester,
    )
}

/// Creates a chassis from `body` and attaches the test car with `tester` to it.
pub fn add_car_with(
    world: &mut PhysicsWorld,
    body: &BodySettings,
    tester: VehicleCollisionTester,
) -> (BodyId, VehicleId) {
    let chassis = world.create_body(&chassis_shape(), body).unwrap();
    let vehicle = world
        .create_wheeled_vehicle(chassis, &car_settings(tester))
        .unwrap();
    (chassis, vehicle)
}

/// The test car at a pose with a ray tester on the probe layer.
pub fn add_car(
    world: &mut PhysicsWorld,
    layers: &CarLayers,
    position: RVec3,
    rotation: Quat,
) -> (BodyId, VehicleId) {
    add_car_with(
        world,
        &chassis_settings(layers, position, rotation),
        VehicleCollisionTester::ray(layers.probe),
    )
}

fn push_f32(digest: &mut Vec<u8>, value: f32) {
    digest.extend_from_slice(&value.to_bits().to_le_bytes());
}

/// Appends a vehicle's state to `digest`: the chassis as [`super::record_body`] writes it, then
/// per wheel the contact flag, contact body raw id, sub-shape id, contact position and normal,
/// suspension length, angular velocity, rotation and steer angle, then the engine rpm and the
/// gear, all as exact little-endian bits.
pub fn record_vehicle<K: VehicleKind>(
    world: &PhysicsWorld,
    id: VehicleId<K>,
    digest: &mut Vec<u8>,
) {
    let vehicle = world.vehicle(id).unwrap();
    super::record_body(world, vehicle.body(), digest);
    for wheel in vehicle.wheels() {
        digest.push(u8::from(wheel.contact.is_some()));
        if let Some(contact) = wheel.contact {
            digest.extend_from_slice(&contact.body.to_raw().to_le_bytes());
            digest.extend_from_slice(&contact.sub_shape_id.to_raw().to_le_bytes());
            let position: [Real; 3] = contact.position.into();
            for value in position {
                digest.extend_from_slice(&value.to_bits().to_le_bytes());
            }
            for value in <[f32; 3]>::from(contact.normal) {
                push_f32(digest, value);
            }
        }
        for value in [
            wheel.suspension_length,
            wheel.angular_velocity,
            wheel.rotation_angle,
            wheel.steer_angle,
        ] {
            push_f32(digest, value);
        }
    }
    push_f32(digest, vehicle.engine_rpm());
    digest.extend_from_slice(&vehicle.current_gear().to_le_bytes());
}

/// Side of the route terrain in samples.
pub const TERRAIN_SAMPLES: u32 = 65;
/// World x and z of terrain sample 0.
pub const TERRAIN_OFFSET: f32 = -32.0;

/// Height of the rolling route terrain at world `(x, z)`.
pub fn terrain_height(x: f32, z: f32) -> f32 {
    0.6 * (x / 9.0).sin() * (z / 11.0).cos()
}

/// The rolling 65 × 65 heightfield of the route, one sample per metre, covering x and z in
/// `[-32, 32]` around its body's origin.
pub fn route_terrain() -> Shape {
    let n = TERRAIN_SAMPLES as usize;
    let samples: Vec<f32> = (0..n * n)
        .map(|i| {
            let x = TERRAIN_OFFSET + (i % n) as f32;
            let z = TERRAIN_OFFSET + (i / n) as f32;
            terrain_height(x, z)
        })
        .collect();
    let settings =
        HeightFieldSettings::default().offset(Vec3::new(TERRAIN_OFFSET, 0.0, TERRAIN_OFFSET));
    Shape::new_height_field(TERRAIN_SAMPLES, &samples, &settings).unwrap()
}

/// The route's waypoints in world x and z, about 150 m in total from the start at (−24, −24).
pub const WAYPOINTS: [(f32, f32); 4] = [(-22.0, 20.0), (20.0, 22.0), (22.0, -20.0), (-8.0, -22.0)];
/// Distance at which a waypoint counts as reached, horizontally, metres.
pub const WAYPOINT_RADIUS: f32 = 3.0;
/// Longest a drive may take, ticks.
pub const ROUTE_TICKS: usize = 3600;

/// What a drive along the route saw.
#[derive(Debug, Default)]
pub struct RouteReport {
    /// Ticks at which each waypoint was reached, in order.
    pub reached: Vec<usize>,
    /// Smallest `dot(chassis up, +Y)` over all ticks.
    pub min_up_dot: f64,
    /// Ticks with at least two wheels in contact.
    pub ticks_with_two_wheels: usize,
    /// Ticks driven.
    pub ticks: usize,
    /// Whether every step was complete.
    pub all_complete: bool,
}

/// The driver input of the route controller for the car at `position` with `rotation` and
/// `velocity`, heading for `target`: steer toward the target, hold about 6 m/s.
pub fn route_input(
    position: RVec3,
    rotation: Quat,
    velocity: Vec3,
    target: (f32, f32),
) -> DriverInput {
    let p = v3(position);
    let to_target = [f64::from(target.0) - p[0], 0.0, f64::from(target.1) - p[2]];
    let conjugate = Quat::from_xyzw(-rotation.x, -rotation.y, -rotation.z, rotation.w);
    let local = rotate(conjugate, to_target);
    // Vehicle right is local −X (left wheels at +X, forward +Z).
    let right = (-local[0].atan2(local[2]) / 0.5).clamp(-1.0, 1.0) as f32;
    let speed = dot(f3(velocity), rotate(rotation, [0.0, 0.0, 1.0])) as f32;
    DriverInput {
        forward: ((6.0 - speed) / 2.0).clamp(0.0, 1.0),
        right,
        brake: if speed > 8.0 { 0.3 } else { 0.0 },
        hand_brake: 0.0,
    }
}

/// A world with the route terrain and the test car at the start, facing +Z, with `threads`
/// workers.
pub fn route_world(threads: u32) -> (PhysicsWorld, BodyId, VehicleId) {
    let (mut world, layers) = car_world(Vec3::ZERO, threads);
    world
        .create_body(
            &route_terrain(),
            &BodySettings::new_static().object_layer(layers.ground),
        )
        .unwrap();
    let start = (-24.0, -24.0);
    let height = terrain_height(start.0, start.1) + 1.0;
    let (chassis, car) = add_car(
        &mut world,
        &layers,
        RVec3::new(start.0 as Real, height as Real, start.1 as Real),
        Quat::IDENTITY,
    );
    (world, chassis, car)
}

/// Drives the route with `threads` workers, calling `after_tick` with the world and the vehicle
/// after every tick.
pub fn drive_route(
    threads: u32,
    mut after_tick: impl FnMut(&PhysicsWorld, VehicleId),
) -> RouteReport {
    let (mut world, chassis, car) = route_world(threads);
    let mut report = RouteReport {
        min_up_dot: 1.0,
        all_complete: true,
        ..RouteReport::default()
    };
    for tick in 0..ROUTE_TICKS {
        let Some(&target) = WAYPOINTS.get(report.reached.len()) else {
            break;
        };
        let body = world.body(chassis).unwrap();
        let (position, rotation) = (body.position(), body.rotation());
        let input = route_input(position, rotation, body.linear_velocity(), target);
        let mut vehicle = world.vehicle_mut(car).unwrap();
        vehicle.set_gravity(GRAVITY).unwrap();
        vehicle.set_driver_input(input).unwrap();
        report.all_complete &= world.step(DT).unwrap().is_complete();
        report.ticks += 1;

        let body = world.body(chassis).unwrap();
        let up = rotate(body.rotation(), [0.0, 1.0, 0.0]);
        report.min_up_dot = report.min_up_dot.min(up[1]);
        let p = v3(body.position());
        let (dx, dz) = (f64::from(target.0) - p[0], f64::from(target.1) - p[2]);
        if (dx * dx + dz * dz).sqrt() <= f64::from(WAYPOINT_RADIUS) {
            report.reached.push(tick);
        }
        let in_contact = world
            .vehicle(car)
            .unwrap()
            .wheels()
            .iter()
            .filter(|wheel| wheel.contact.is_some())
            .count();
        if in_contact >= 2 {
            report.ticks_with_two_wheels += 1;
        }
        after_tick(&world, car);
    }
    report
}
