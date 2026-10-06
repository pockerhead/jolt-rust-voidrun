//! A car, a tank and a motorcycle on rolling terrain, and a walker; Tab switches which one the
//! keys drive. Wheels are drawn at the poses the binding reports.

use oxijolt::{
    BodyId, BodySettings, DriverInput, EventSettings, Motorcycle, MotorcycleSettings, PhysicsWorld,
    SuspensionSpring, TrackedDriverInput, TrackedVehicle, TrackedVehicleSettings,
    TrackedWheelSettings, Vec3, VehicleCollisionTester, VehicleId, VehicleKind, VehicleSettings,
    VehicleTrackSettings, VehicleTransmissionSettings,
};

use crate::camera::CameraHint;
use crate::digest::Digest;
use crate::draw::{colours, DrawList, Solid};
use crate::input::{Edges, Input};
use crate::math::{about_axis, glam, glam_quat, position_f32, rvec};
use crate::scene::{new_world, step, Layers, Milestones, Result, Scene, SceneConfig};
use crate::terrain;
use crate::tracked::Tracked;
use crate::visual::{Shaped, Visual, VisualKey, Visuals};
use crate::walker::Walker;

/// What the keys drive, in Tab order.
const DRIVEN: [&str; 4] = ["walker", "car", "tank", "motorcycle"];
/// Where each starts: x, z and the yaw it faces (0 is +Z).
const STARTS: [(f32, f32, f32); 4] = [
    (-4.0, 9.0, 0.0),
    (-16.0, -6.0, std::f32::consts::FRAC_PI_2),
    (9.0, 8.0, 0.0),
    (-12.0, 3.0, std::f32::consts::FRAC_PI_2),
];
/// The ticks at which the script switches to the car, the tank and the motorcycle.
const SWITCHES: [u32; 3] = [60, 390, 540];
/// z of the tank's wheels in each track, front to back.
const TANK_WHEEL_Z: [f32; 9] = [2.95, 2.1, 1.4, 0.7, 0.0, -0.7, -1.4, -2.1, -2.75];

const MILESTONES: &[&str] = &[
    "walker walked",
    "car drove 15 m",
    "car shifted to gear 2",
    "tank turned in place 90°",
    "bike leaned into a turn",
];
const CONTROLS: &[(&str, &str)] = &[
    ("Tab", "switch: walker, car, tank, motorcycle"),
    ("W S", "throttle; against the motion it brakes first"),
    ("A D", "steer; the tank turns on the spot when standing"),
    ("Space", "hand brake, or jump when walking"),
];

/// The terrain height at `(x, z)`.
fn ground_height(x: f32, z: f32) -> f32 {
    terrain::rolling(0.35)(x, z)
}

/// A vehicle's chassis and how to draw its wheels.
struct Chassis {
    body: BodyId,
    wheel: VisualKey,
    /// A spoke from the hub toward the rim, so that the wheel's turning shows, and where it
    /// sits in the wheel's frame.
    spoke: (VisualKey, [f32; 3]),
}

/// The vehicles scene.
pub struct Vehicles {
    world: PhysicsWorld,
    visuals: Visuals,
    tracked: Tracked,
    walker: Walker,
    walker_visual: VisualKey,
    car: VehicleId,
    tank: VehicleId<TrackedVehicle>,
    bike: VehicleId<Motorcycle>,
    chassis: [Chassis; 3],
    driven: usize,
    /// The tank's heading last tick and how far it turned since the scene started.
    tank_heading: (f32, f32),
    milestones: Milestones,
}

impl Vehicles {
    /// Builds the terrain, the three vehicles and the walker.
    pub fn new(config: &SceneConfig, generation: u64) -> Result<Self> {
        let (mut world, layers) = new_world(config, 2, EventSettings::default())?;
        let mut visuals = Visuals::new(generation);
        let mut tracked = Tracked::default();
        let ground = BodySettings::new_static().object_layer(layers.ground);
        let terrain = terrain::height_field(65, 1.0, ground_height)?;
        tracked.spawn(&mut world, &terrain, &ground, &mut visuals, colours::GROUND)?;

        let car_hull = Shaped::cuboid([0.9, 0.3, 2.0])?;
        let car_body = spawn_chassis(
            &mut world,
            &layers,
            &mut visuals,
            &mut tracked,
            &Shaped::offset_center_of_mass(&car_hull, [0.0, -0.3, 0.0])?,
            1500.0,
            STARTS[1],
            1.0,
        )?;
        // The preset keeps Jolt's shift point of 4000 rpm, which this front-wheel-drive car
        // reaches only after about 13 s without spinning its wheels; shifting at 2000 rpm brings
        // second gear into the clip.
        let tester = VehicleCollisionTester::cast_sphere(layers.probe, 0.2);
        let gearbox = VehicleTransmissionSettings::default()
            .shift_up_rpm(2000.0)
            .shift_down_rpm(1100.0);
        let car_settings =
            VehicleSettings::car(Vec3::new(0.9, -0.1, 1.4), 0.35, tester).transmission(gearbox);
        let car = world.create_vehicle(car_body, &car_settings)?;

        let tank_hull = Shaped::cuboid([1.7, 0.5, 3.2])?;
        let tank_body = spawn_chassis(
            &mut world,
            &layers,
            &mut visuals,
            &mut tracked,
            &Shaped::offset_center_of_mass(&tank_hull, [0.0, -0.5, 0.0])?,
            4000.0,
            STARTS[2],
            1.3,
        )?;
        let tank_settings = TrackedVehicleSettings::new(
            tank_track(1.7),
            tank_track(-1.7),
            VehicleCollisionTester::ray(layers.probe),
        )
        .max_pitch_roll_angle(60.0_f32.to_radians());
        let tank = world.create_tracked_vehicle(tank_body, &tank_settings)?;

        let bike_frame = Shaped::cuboid([0.2, 0.3, 0.4])?;
        let bike_body = spawn_chassis(
            &mut world,
            &layers,
            &mut visuals,
            &mut tracked,
            &Shaped::offset_center_of_mass(&bike_frame, [0.0, -0.3, 0.0])?,
            240.0,
            STARTS[3],
            1.0,
        )?;
        let bike_settings = MotorcycleSettings::bike(
            Vec3::new(0.0, -0.27, 0.75),
            0.31,
            VehicleCollisionTester::cast_cylinder(layers.probe),
        );
        let bike = world.create_motorcycle(bike_body, &bike_settings)?;

        // The wheel model's axle is its Y axis and its up is X, as the binding reports it.
        let wheels = |visuals: &mut Visuals, body: BodyId, width: f32, radius: f32| Chassis {
            body,
            wheel: visuals.add(Visual::Cylinder {
                half_height: width / 2.0,
                radius,
            }),
            spoke: (
                visuals.add(Visual::Box {
                    half_extent: [0.4 * radius, width / 2.0 + 0.015, 0.12 * radius],
                }),
                [0.5 * radius, 0.0, 0.0],
            ),
        };
        let chassis = [
            wheels(&mut visuals, car_body, 0.2, 0.35),
            wheels(&mut visuals, tank_body, 0.1, 0.3),
            wheels(&mut visuals, bike_body, 0.05, 0.31),
        ];
        let (x, z, _) = STARTS[0];
        let walker = Walker::new(&mut world, rvec([x, ground_height(x, z), z]), 1.8, 0.3)?;
        let walker_visual = visuals.add(walker.visual());
        let tank_heading = heading(&world, tank_body);
        Ok(Self {
            world,
            visuals,
            tracked,
            walker,
            walker_visual,
            car,
            tank,
            bike,
            chassis,
            driven: 0,
            tank_heading: (tank_heading, 0.0),
            milestones: Milestones::new(MILESTONES),
        })
    }

    /// Sets every vehicle's input: the driven one from `input`, the others braking.
    fn drive(&mut self, input: &Input) -> Result<()> {
        let held = &input.held;
        let hold = DriverInput {
            brake: 1.0,
            ..DriverInput::default()
        };
        let car_input = if self.driven == 1 {
            wheeled_input(
                &self.world,
                self.chassis[0].body,
                held.throttle,
                held.steer,
                held.hand_brake,
            )?
        } else {
            hold
        };
        self.world
            .vehicle_mut(self.car)?
            .set_driver_input(car_input)?;
        let tank_input = if self.driven == 2 {
            tracked_input(held.throttle, held.steer)
        } else {
            TrackedDriverInput {
                brake: 1.0,
                ..TrackedDriverInput::default()
            }
        };
        self.world
            .vehicle_mut(self.tank)?
            .set_driver_input(tank_input)?;
        let bike_input = if self.driven == 3 {
            wheeled_input(
                &self.world,
                self.chassis[2].body,
                held.throttle,
                held.steer,
                held.hand_brake,
            )?
        } else {
            hold
        };
        self.world
            .vehicle_mut(self.bike)?
            .set_driver_input(bike_input)?;
        Ok(())
    }

    fn check_milestones(&mut self) -> Result<()> {
        let moved = |world: &PhysicsWorld, body: BodyId, start: usize| -> Result<f32> {
            let p = position_f32(world.body(body)?.position());
            let (x, z, _) = STARTS[start];
            Ok((p[0] - x).hypot(p[2] - z))
        };
        let walker = position_f32(self.world.character(self.walker.id())?.position());
        if (walker[0] - STARTS[0].0).hypot(walker[2] - STARTS[0].1) >= 2.0 {
            self.milestones.reach("walker walked");
        }
        if moved(&self.world, self.chassis[0].body, 1)? >= 15.0 {
            self.milestones.reach("car drove 15 m");
        }
        if self.world.vehicle(self.car)?.current_gear() >= 2 {
            self.milestones.reach("car shifted to gear 2");
        }
        let now = heading(&self.world, self.chassis[1].body);
        let turn = wrap_angle(now - self.tank_heading.0);
        self.tank_heading = (now, self.tank_heading.1 + turn);
        if self.tank_heading.1.abs() >= 90.0_f32.to_radians()
            && moved(&self.world, self.chassis[1].body, 2)? < 3.0
        {
            self.milestones.reach("tank turned in place 90°");
        }
        if self.world.vehicle(self.bike)?.lean().angle.abs() > 0.1 {
            self.milestones.reach("bike leaned into a turn");
        }
        Ok(())
    }

    /// The chassis the keys drive, or `None` for the walker.
    fn driven_body(&self) -> Option<BodyId> {
        (self.driven > 0).then(|| self.chassis[self.driven - 1].body)
    }

    fn draw_wheels<K: VehicleKind>(
        &self,
        id: VehicleId<K>,
        chassis: &Chassis,
        out: &mut DrawList,
    ) -> Result<()> {
        let vehicle = self.world.vehicle(id)?;
        for index in 0..vehicle.wheel_count() {
            if let Some((centre, rotation)) = vehicle.wheel_world_transform(index) {
                let pose = (position_f32(centre), rotation.into());
                out.solids.push(Solid {
                    visual: chassis.wheel,
                    position: pose.0,
                    rotation: pose.1,
                    colour: [0.2, 0.2, 0.2],
                });
                let (spoke, at) = chassis.spoke;
                out.solids
                    .push(Solid::attached(spoke, pose, at, [1.0, 1.0, 1.0]));
            }
        }
        Ok(())
    }

    fn write_vehicle<K: VehicleKind>(&self, id: VehicleId<K>, digest: &mut Digest) -> Result<()> {
        let vehicle = self.world.vehicle(id)?;
        for wheel in vehicle.wheels() {
            digest.f32s(&[
                wheel.suspension_length,
                wheel.angular_velocity,
                wheel.rotation_angle,
                wheel.steer_angle,
            ]);
            digest.u32(
                wheel
                    .contact
                    .map_or(u32::MAX, |contact| contact.body.to_raw()),
            );
        }
        digest.f32(vehicle.engine_rpm());
        digest.i32(vehicle.current_gear());
        Ok(())
    }
}

impl Scene for Vehicles {
    fn update(&mut self, input: &Input) -> Result<()> {
        if input.edges.switch {
            self.driven = (self.driven + 1) % DRIVEN.len();
        }
        self.drive(input)?;
        let walker_input = if self.driven == 0 {
            input.held
        } else {
            Default::default()
        };
        let jump = self.driven == 0 && input.edges.jump;
        self.walker.tick(&mut self.world, &walker_input, jump)?;
        step(&mut self.world)?;
        let events = self.world.take_events();
        self.tracked.sync(&self.world, &events);
        self.check_milestones()
    }

    fn draw(&self, out: &mut DrawList) -> Result<()> {
        let driven = self.driven_body();
        self.tracked.draw_with(out, |body, colour| {
            if Some(body) == driven {
                colours::PLAYER
            } else {
                colour
            }
        });
        self.draw_wheels(self.car, &self.chassis[0], out)?;
        self.draw_wheels(self.tank, &self.chassis[1], out)?;
        self.draw_wheels(self.bike, &self.chassis[2], out)?;
        let character = self.world.character(self.walker.id())?;
        let (position, rotation) = self.walker.capsule_pose(&character);
        let colour = if self.driven == 0 {
            colours::PLAYER
        } else {
            colours::BODY_ALT
        };
        out.solids.push(Solid {
            visual: self.walker_visual,
            position,
            rotation,
            colour,
        });
        out.hud.push(format!("driving: {}", DRIVEN[self.driven]));
        let car = self.world.vehicle(self.car)?;
        out.hud.push(format!(
            "car: gear {}, {:.0} rpm",
            car.current_gear(),
            car.engine_rpm()
        ));
        let lean = self.world.vehicle(self.bike)?.lean();
        out.hud.push(format!(
            "motorcycle lean {:.0}°, tank turned {:.0}°",
            lean.angle.to_degrees(),
            self.tank_heading.1.to_degrees()
        ));
        Ok(())
    }

    fn write_state(&self, digest: &mut Digest) -> Result<()> {
        self.tracked.write_state(digest);
        self.walker.write_state(&self.world, digest)?;
        self.write_vehicle(self.car, digest)?;
        self.write_vehicle(self.tank, digest)?;
        self.write_vehicle(self.bike, digest)?;
        digest.u32(self.driven as u32);
        digest.f32s(&[self.tank_heading.0, self.tank_heading.1]);
        Ok(())
    }

    fn visuals(&self) -> &Visuals {
        &self.visuals
    }

    fn world(&self) -> &PhysicsWorld {
        &self.world
    }

    fn world_mut(&mut self) -> &mut PhysicsWorld {
        &mut self.world
    }

    fn camera(&self) -> CameraHint {
        let target = match self.driven_body() {
            Some(body) => self.tracked.pose(body).map(|(p, _)| position_f32(p)),
            None => self
                .world
                .character(self.walker.id())
                .ok()
                .map(|character| position_f32(character.position())),
        }
        .unwrap_or([0.0; 3]);
        let distance = if self.driven == 2 { 14.0 } else { 9.0 };
        CameraHint::new([target[0], target[1] + 1.0, target[2]], 0.6, 0.4, distance)
    }

    /// Close beside what the script drives: the walker, the car, the tank, and the motorcycle
    /// from behind, where its lean shows.
    fn record_camera(&self, tick: u32) -> CameraHint {
        let at = |body: BodyId| {
            self.tracked
                .pose(body)
                .map_or([0.0; 3], |(p, _)| position_f32(p))
        };
        if tick < SWITCHES[0] {
            let walker = self
                .world
                .character(self.walker.id())
                .map_or([0.0; 3], |character| position_f32(character.position()));
            CameraHint::new([walker[0], walker[1] + 1.0, walker[2]], 0.5, 0.25, 5.0)
        } else if tick < SWITCHES[1] {
            let [x, y, z] = at(self.chassis[0].body);
            CameraHint::new([x, y + 0.2, z], 0.45, 0.25, 6.0)
        } else if tick < SWITCHES[2] {
            let [x, y, z] = at(self.chassis[1].body);
            CameraHint::new([x, y + 0.5, z], 0.6, 0.45, 10.0)
        } else {
            let bike = self.chassis[2].body;
            let [x, y, z] = at(bike);
            let behind = heading(&self.world, bike) + std::f32::consts::PI - 0.5;
            CameraHint::new([x, y + 0.3, z], behind, 0.2, 4.5)
        }
    }

    fn record_ticks(&self) -> u32 {
        780
    }

    fn script(&self, tick: u32) -> Input {
        let mut input = Input {
            edges: Edges {
                switch: SWITCHES.contains(&tick),
                ..Edges::default()
            },
            ..Input::default()
        };
        if tick < SWITCHES[0] {
            input.held.walk = [1.0, 0.0];
        } else if tick < SWITCHES[1] - 30 {
            input.held.throttle = 0.35;
            // A gentle right turn in the middle of the run, so the front wheels steer.
            if (200..290).contains(&tick) {
                input.held.steer = 0.3;
            }
        } else if tick < SWITCHES[1] {
            input.held.hand_brake = true;
        } else if tick < SWITCHES[2] {
            input.held.steer = 1.0;
        } else {
            input.held.throttle = 0.5;
            if tick >= SWITCHES[2] + 110 {
                input.held.steer = 0.35;
            }
        }
        input
    }

    fn milestones(&self) -> &Milestones {
        &self.milestones
    }

    fn controls(&self) -> &'static [(&'static str, &'static str)] {
        CONTROLS
    }
}

/// A chassis of `shaped` and `mass` kg at a start `(x, z, yaw)`, `lift` metres above the
/// terrain, under the world's gravity and never asleep.
#[allow(clippy::too_many_arguments)]
fn spawn_chassis(
    world: &mut PhysicsWorld,
    layers: &Layers,
    visuals: &mut Visuals,
    tracked: &mut Tracked,
    shaped: &Shaped,
    mass: f32,
    (x, z, yaw): (f32, f32, f32),
    lift: f32,
) -> Result<BodyId> {
    let settings = BodySettings::new_dynamic()
        .position(rvec([x, ground_height(x, z) + lift, z]))
        .rotation(about_axis([0.0, 1.0, 0.0], yaw))
        .object_layer(layers.moving)
        .mass(mass)
        .allow_sleeping(false);
    tracked.spawn(world, shaped, &settings, visuals, colours::BODY)
}

/// The driver input of a car or motorcycle: `throttle` against the motion brakes until it
/// stops, as in Jolt's samples.
fn wheeled_input(
    world: &PhysicsWorld,
    chassis: BodyId,
    throttle: f32,
    steer: f32,
    hand_brake: bool,
) -> Result<DriverInput> {
    let body = world.body(chassis)?;
    let forward = glam_quat(body.rotation()) * glam::Vec3::Z;
    let speed = glam(body.linear_velocity()).dot(forward);
    let reversing = (throttle > 0.0 && speed < -0.1) || (throttle < 0.0 && speed > 0.1);
    Ok(DriverInput {
        forward: if reversing { 0.0 } else { throttle },
        right: steer,
        brake: if reversing { 1.0 } else { 0.0 },
        hand_brake: if hand_brake { 1.0 } else { 0.0 },
    })
}

/// The tank's input: steering slows one track, or turns the tank on the spot when there is no
/// throttle, by running the tracks opposite ways.
fn tracked_input(throttle: f32, steer: f32) -> TrackedDriverInput {
    let (forward, inner) = if throttle == 0.0 && steer != 0.0 {
        (1.0, -1.0)
    } else {
        (throttle, 0.6)
    };
    let (left_ratio, right_ratio) = match steer {
        s if s > 0.0 => (1.0, inner),
        s if s < 0.0 => (inner, 1.0),
        _ => (1.0, 1.0),
    };
    TrackedDriverInput {
        forward,
        left_ratio,
        right_ratio,
        brake: 0.0,
    }
}

/// One of the tank's tracks at `x`: nine wheels of radius 0.3, the end ones fixed at y = 0,
/// the others sprung at y = -0.3, driven at the rearmost.
fn tank_track(x: f32) -> VehicleTrackSettings {
    let last = TANK_WHEEL_Z.len() - 1;
    let wheels = TANK_WHEEL_Z
        .iter()
        .enumerate()
        .map(|(index, &z)| {
            let end = index == 0 || index == last;
            TrackedWheelSettings::new(Vec3::new(x, if end { 0.0 } else { -0.3 }, z))
                .radius(0.3)
                .width(0.1)
                .suspension_min_length(0.3)
                .suspension_max_length(if end { 0.3 } else { 0.5 })
                .suspension_spring(SuspensionSpring::FrequencyAndDamping {
                    frequency: 1.0,
                    damping: 0.5,
                })
        })
        .collect();
    VehicleTrackSettings::new(wheels, last as u32)
}

/// The heading of `body` about +Y, radians from +Z toward +X.
fn heading(world: &PhysicsWorld, body: BodyId) -> f32 {
    world.body(body).map_or(0.0, |reading| {
        let forward = glam_quat(reading.rotation()) * glam::Vec3::Z;
        forward.x.atan2(forward.z)
    })
}

/// `angle` wrapped into `(-π, π]`.
fn wrap_angle(angle: f32) -> f32 {
    let tau = std::f32::consts::TAU;
    let wrapped = angle.rem_euclid(tau);
    if wrapped > std::f32::consts::PI {
        wrapped - tau
    } else {
        wrapped
    }
}
