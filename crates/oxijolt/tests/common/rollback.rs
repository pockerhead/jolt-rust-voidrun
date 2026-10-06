//! A scene for the rollback buffer tests: toppling stacks on a static floor, a static wall, a
//! cube put to sleep at the start, a kinematic platform gliding along +x, two cubes joined by a
//! motor-driven hinge, a car and a walking character with an inner body.

use oxijolt::*;

use super::determinism::Digest;
use super::vehicle::{add_car, car_world, record_vehicle, CarLayers, GRAVITY};
use super::{add_cube, build_stacks, record_body, step, DT};

/// The caller's inputs of one tick.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Inputs {
    pub forward: f32,
    pub right: f32,
    pub walk: Vec3,
}

impl Inputs {
    /// The inputs of the run that counts.
    pub const PLAYED: Inputs = Inputs {
        forward: 0.6,
        right: 0.3,
        walk: Vec3::new(0.5, -1.0, 0.25),
    };
    /// The inputs of a mispredicted run that a rollback abandons.
    pub const PREDICTED: Inputs = Inputs {
        forward: -0.4,
        right: -0.8,
        walk: Vec3::new(-1.5, -1.0, 0.75),
    };
}

pub struct RollbackScene {
    pub world: PhysicsWorld,
    pub layers: CarLayers,
    /// The bodies the scene created itself, in creation order: the floor, the stack cubes, the
    /// wall, the sleeper, the platform, the two hinge cubes and the chassis.
    pub bodies: Vec<BodyId>,
    pub floor: BodyId,
    pub wall: BodyId,
    /// A dynamic cube deactivated at creation; only a detour wakes it.
    pub sleeper: BodyId,
    /// A kinematic body moving at a constant velocity.
    pub platform: BodyId,
    pub car: VehicleId,
    pub character: CharacterId,
}

impl RollbackScene {
    pub fn new(worker_threads: u32) -> Self {
        let (mut world, layers) = car_world(GRAVITY, worker_threads);
        let mut bodies = build_stacks(&mut world);
        let floor = bodies[0];

        let wall_shape = Shape::new_box(Vec3::new(0.2, 1.0, 3.0)).unwrap();
        let wall = world
            .create_body(
                &wall_shape,
                &BodySettings::new_static().position(RVec3::new(-9.0, 1.0, 0.0)),
            )
            .unwrap();
        let sleeper = add_cube(&mut world, RVec3::new(-4.0, 0.5, 4.0));
        world.body_mut(sleeper).unwrap().deactivate().unwrap();
        let platform_shape = Shape::new_box(Vec3::new(1.0, 0.1, 1.0)).unwrap();
        let platform = world
            .create_body(
                &platform_shape,
                &BodySettings::new_kinematic()
                    .position(RVec3::new(0.0, 6.0, -9.0))
                    .linear_velocity(Vec3::new(0.5, 0.0, 0.0)),
            )
            .unwrap();
        bodies.extend([wall, sleeper, platform]);

        let first = add_cube(&mut world, RVec3::new(6.0, 0.5, -4.0));
        let second = add_cube(&mut world, RVec3::new(7.2, 0.5, -4.0));
        bodies.extend([first, second]);
        let hinge = world
            .create_constraint(
                first,
                second,
                &HingeConstraintSettings::new(
                    RVec3::new(6.6, 0.5, -4.0),
                    Vec3::new(1.0, 0.0, 0.0),
                    Vec3::new(0.0, 1.0, 0.0),
                ),
            )
            .unwrap();
        let mut motor = world.constraint_mut(hinge).unwrap();
        motor.set_target_angular_velocity(1.0).unwrap();
        motor.set_motor_state(MotorState::Velocity);

        let (chassis, car) = add_car(
            &mut world,
            &layers,
            RVec3::new(-6.0, 1.0, -6.0),
            Quat::IDENTITY,
        );
        bodies.push(chassis);

        let capsule = Shape::new_capsule(0.7, 0.4).unwrap();
        let settings = CharacterSettings::new(&capsule)
            .shape_offset(Vec3::new(0.0, 1.1, 0.0))
            .inner_body(Some(InnerBody {
                shape: &capsule,
                object_layer: layers.moving,
            }));
        let character = world
            .create_character(&settings, RVec3::new(4.0, 0.0, 6.0), Quat::IDENTITY)
            .unwrap();
        world
            .refresh_character_contacts(character, &QueryFilter::new())
            .unwrap();

        Self {
            world,
            layers,
            bodies,
            floor,
            wall,
            sleeper,
            platform,
            car,
            character,
        }
    }

    /// The character's inner body.
    pub fn inner_body(&self) -> BodyId {
        self.world
            .character(self.character)
            .unwrap()
            .inner_body()
            .unwrap()
    }

    /// Every body of the world: the scene's own and the character's inner body.
    pub fn all_bodies(&self) -> Vec<BodyId> {
        let mut ids = self.bodies.clone();
        ids.push(self.inner_body());
        ids
    }

    /// The static bodies: the floor and the wall.
    pub fn static_bodies(&self) -> [BodyId; 2] {
        [self.floor, self.wall]
    }

    /// One tick of the caller's inputs and a step.
    pub fn tick(&mut self, inputs: Inputs) {
        let mut car = self.world.vehicle_mut(self.car).unwrap();
        car.set_gravity(GRAVITY).unwrap();
        car.set_driver_input(DriverInput {
            forward: inputs.forward,
            right: inputs.right,
            ..DriverInput::default()
        })
        .unwrap();
        self.world
            .character_mut(self.character)
            .unwrap()
            .set_linear_velocity(inputs.walk)
            .unwrap();
        self.world
            .update_character(
                self.character,
                DT,
                GRAVITY,
                &ExtendedUpdateSettings::default(),
                &QueryFilter::new(),
            )
            .unwrap();
        step(&mut self.world, 1);
    }

    /// What a mispredicted run does besides other inputs: wakes the sleeper with a push,
    /// teleports a stack cube and changes the world's gravity for one step. All of it is saved
    /// state, so a restore undoes it.
    pub fn detour(&mut self) {
        let mut sleeper = self.world.body_mut(self.sleeper).unwrap();
        sleeper.add_impulse(Vec3::new(0.0, 4.0, 2.0)).unwrap();
        let cube = self.bodies[3];
        self.world
            .body_mut(cube)
            .unwrap()
            .set_position(RVec3::new(2.0, 4.0, 3.0), Activation::Activate)
            .unwrap();
        self.world.set_gravity(Vec3::new(1.0, -12.0, 0.0)).unwrap();
        self.tick(Inputs::PREDICTED);
        self.world.set_gravity(GRAVITY).unwrap();
    }

    /// The state after this tick: every body as [`record_body`] writes it, the character's
    /// saved state and the vehicle with its wheels.
    pub fn record(&self, digest: &mut Digest) {
        let tick = digest.push();
        for id in self.all_bodies() {
            record_body(&self.world, id, &mut tick.state);
        }
        let character = self.world.character(self.character).unwrap().save_state();
        tick.state.extend(character.to_bytes());
        record_vehicle(&self.world, self.car, &mut tick.state);
    }

    /// Every body as [`record_body`] writes it, one entry per body of [`all_bodies`](Self::all_bodies).
    pub fn body_bits(&self) -> Vec<Vec<u8>> {
        self.all_bodies()
            .into_iter()
            .map(|id| {
                let mut bits = Vec::new();
                record_body(&self.world, id, &mut bits);
                bits
            })
            .collect()
    }
}
