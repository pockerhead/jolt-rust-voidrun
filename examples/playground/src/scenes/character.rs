//! A humanoid `CharacterVirtual` on heightfield terrain: stairs of rounded boxes, a 30° ramp
//! made of a triangle mesh, a 50° slope too steep to stand on, crates to push, a conveyor
//! belt whose speed a character contact listener supplies, and a kinematic ferry.

use std::sync::Arc;

use oxijolt::{
    BodyId, BodySettings, BodyVelocity, CharacterContactListener, CharacterId, EventSettings,
    GroundState, PhysicsWorld, Quat, Vec3,
};

use crate::camera::CameraHint;
use crate::digest::Digest;
use crate::draw::{colours, DrawList, Solid};
use crate::input::{Edges, Input};
use crate::math::{position_f32, rvec};
use crate::scene::{new_world, step, Layers, Milestones, Result, Scene, SceneConfig};
use crate::terrain;
use crate::tracked::Tracked;
use crate::visual::{face_outward, Shaped, VisualKey, Visuals};
use crate::walker::Walker;

/// Height and radius of the humanoid, metres.
const HUMANOID: (f32, f32) = (1.8, 0.3);
/// Where the walker starts.
const START: [f32; 3] = [-10.5, 0.0, 0.0];
/// The belt's user data, by which the listener knows it.
const CONVEYOR: u64 = 7;
/// The belt's speed, m/s along +X.
const BELT_SPEED: f32 = 1.5;
/// The ferry's ends along x, its z, and the height of its centre. Like the belt it is a
/// 0.2 m slab sunk into the ground up to its top 4 cm, low enough to walk onto.
const FERRY: (f32, f32, f32, f32) = (5.0, 10.0, 0.0, -0.06);
/// Ticks of one ferry round trip: a wait at each end and a crossing each way.
const FERRY_PERIOD: u32 = 360;
/// The tick at which the ferry first waits at its near end.
const FERRY_BOARDING: u32 = 250;

const MILESTONES: &[&str] = &[
    "walked 5 m",
    "climbed the stairs",
    "carried by the conveyor",
    "rode the platform",
    "jumped",
];
const CONTROLS: &[(&str, &str)] = &[
    ("W A S D", "walk, relative to the camera"),
    ("Shift", "sprint"),
    ("Space", "jump"),
];

/// Moves characters standing on the belt along it.
struct Conveyor;

impl CharacterContactListener for Conveyor {
    fn adjust_body_velocity(
        &self,
        _character: CharacterId,
        _body: BodyId,
        user_data: u64,
        velocity: &mut BodyVelocity,
    ) {
        if user_data == CONVEYOR {
            velocity
                .set_linear_velocity(Vec3::new(BELT_SPEED, 0.0, 0.0))
                .expect("the belt speed is within the limits");
        }
    }
}

/// The character scene.
pub struct Character {
    world: PhysicsWorld,
    visuals: Visuals,
    tracked: Tracked,
    walker: Walker,
    walker_visual: VisualKey,
    conveyor: BodyId,
    ferry: BodyId,
    tick: u32,
    conveyor_ticks: u32,
    ferry_ticks: u32,
    milestones: Milestones,
}

impl Character {
    /// Builds the terrain, the course and the walker.
    pub fn new(config: &SceneConfig, generation: u64) -> Result<Self> {
        let (mut world, layers) = new_world(config, 2, EventSettings::default())?;
        let mut visuals = Visuals::new(generation);
        let mut tracked = Tracked::default();
        build_course(&mut world, &layers, &mut visuals, &mut tracked)?;
        let ground = BodySettings::new_static().object_layer(layers.ground);
        let conveyor = tracked.spawn(
            &mut world,
            &Shaped::cuboid([2.0, 0.1, 0.8])?,
            &ground
                .clone()
                .position(rvec([0.5, -0.06, 0.0]))
                .user_data(CONVEYOR),
            &mut visuals,
            colours::KINEMATIC,
        )?;
        let ferry = tracked.spawn(
            &mut world,
            &Shaped::cuboid([1.0, 0.1, 1.0])?,
            &BodySettings::new_kinematic()
                .position(rvec(ferry_position(0)))
                .object_layer(layers.moving),
            &mut visuals,
            colours::KINEMATIC,
        )?;
        world.set_character_contact_listener(Some(Arc::new(Conveyor)));
        let walker = Walker::new(&mut world, rvec(START), HUMANOID.0, HUMANOID.1)?;
        let walker_visual = visuals.add(walker.visual());
        Ok(Self {
            world,
            visuals,
            tracked,
            walker,
            walker_visual,
            conveyor,
            ferry,
            tick: 0,
            conveyor_ticks: 0,
            ferry_ticks: 0,
            milestones: Milestones::new(MILESTONES),
        })
    }

    fn check_milestones(&mut self) -> Result<()> {
        let character = self.world.character(self.walker.id())?;
        let p = position_f32(character.position());
        let on_ground = character.ground_state() == GroundState::OnGround;
        let ground = character.ground_body();
        if (p[0] - START[0]).hypot(p[2] - START[2]) >= 5.0 {
            self.milestones.reach("walked 5 m");
        }
        if on_ground && p[1] > 0.95 {
            self.milestones.reach("climbed the stairs");
        }
        if ground == Some(self.conveyor) {
            self.conveyor_ticks += 1;
            if self.conveyor_ticks >= 30 {
                self.milestones.reach("carried by the conveyor");
            }
        }
        if ground == Some(self.ferry) && is_ferry_moving(self.tick) {
            self.ferry_ticks += 1;
            if self.ferry_ticks >= 60 {
                self.milestones.reach("rode the platform");
            }
        }
        if self.walker.has_jumped() && character.ground_state() == GroundState::InAir {
            self.milestones.reach("jumped");
        }
        Ok(())
    }
}

impl Scene for Character {
    fn update(&mut self, input: &Input) -> Result<()> {
        self.tick += 1;
        self.world.body_mut(self.ferry)?.move_kinematic(
            rvec(ferry_position(self.tick)),
            Quat::IDENTITY,
            crate::scene::DT,
        )?;
        self.walker
            .tick(&mut self.world, &input.held, input.edges.jump)?;
        step(&mut self.world)?;
        let events = self.world.take_events();
        self.tracked.sync(&self.world, &events);
        self.check_milestones()
    }

    fn draw(&self, out: &mut DrawList) -> Result<()> {
        self.tracked.draw(out);
        let character = self.world.character(self.walker.id())?;
        let (position, rotation) = self.walker.capsule_pose(&character);
        out.solids.push(Solid {
            visual: self.walker_visual,
            position,
            rotation,
            colour: colours::PLAYER,
        });
        let speed = character.linear_velocity();
        let p = position_f32(character.position());
        out.hud.push(format!(
            "at ({:.1}, {:.1}, {:.1}), ground: {:?}, speed {:.1} m/s",
            p[0],
            p[1],
            p[2],
            character.ground_state(),
            (speed.x * speed.x + speed.z * speed.z).sqrt()
        ));
        let ground_velocity = character.ground_velocity();
        out.hud.push(format!(
            "ground velocity from the listener or the ferry: ({:.1}, {:.1}, {:.1})",
            ground_velocity.x, ground_velocity.y, ground_velocity.z
        ));
        Ok(())
    }

    fn write_state(&self, digest: &mut Digest) {
        self.tracked.write_state(digest);
        self.walker.write_state(&self.world, digest);
        digest.u32(self.tick);
        digest.u32(self.conveyor_ticks);
        digest.u32(self.ferry_ticks);
    }

    fn visuals(&self) -> &Visuals {
        &self.visuals
    }

    fn world(&self) -> &PhysicsWorld {
        &self.world
    }

    fn camera(&self) -> CameraHint {
        let target = self
            .world
            .character(self.walker.id())
            .map(|character| position_f32(character.position()))
            .unwrap_or(START);
        CameraHint::new([target[0], target[1] + 1.0, target[2]], 0.5, 0.45, 9.0)
    }

    fn record_camera(&self, _tick: u32) -> CameraHint {
        CameraHint::new([1.0, 0.9, 0.0], 0.0, 0.26, 14.0)
    }

    fn record_ticks(&self) -> u32 {
        540
    }

    fn script(&self, tick: u32) -> Input {
        let mut input = Input::default();
        // The camera looks along -Z, so walking "right" is +X, along the course.
        input.held.camera_yaw = 0.0;
        let Ok(character) = self.world.character(self.walker.id()) else {
            return input;
        };
        let x = position_f32(character.position())[0];
        let ferry_x = self
            .tracked
            .pose(self.ferry)
            .map_or(FERRY.0, |(p, _)| position_f32(p)[0]);
        let on_ferry = character.ground_body() == Some(self.ferry);
        let ferry_waits = !is_ferry_moving(tick);
        let walk = if on_ferry {
            // Walk to the middle while it waits at the near end, ride, step off at the far end.
            let at_far_end = ferry_x > FERRY.1 - 0.05;
            if ferry_waits && (at_far_end || x < ferry_x - 0.2) {
                1.0
            } else {
                0.0
            }
        } else if (3.4..FERRY.1).contains(&x) && ferry_x > FERRY.0 + 0.05 {
            // Wait at the near end until the ferry is back.
            0.0
        } else if x < 13.0 {
            1.0
        } else {
            0.0
        };
        input.held.walk = [walk, 0.0];
        input.edges = Edges {
            jump: x > 11.5 && !self.walker.has_jumped(),
            ..Edges::default()
        };
        input
    }

    fn milestones(&self) -> &Milestones {
        &self.milestones
    }

    fn controls(&self) -> &'static [(&'static str, &'static str)] {
        CONTROLS
    }
}

/// The ferry's centre at `tick`: a wait at the near end, a smooth crossing, a wait at the far
/// end and a crossing back, every [`FERRY_PERIOD`] ticks.
fn ferry_position(tick: u32) -> [f32; 3] {
    let (near, far, z, y) = FERRY;
    let phase = ferry_phase(tick);
    let quarter = FERRY_PERIOD / 4;
    let smooth = |t: f32| t * t * (3.0 - 2.0 * t);
    let along = match phase / quarter {
        0 => 0.0,
        1 => smooth((phase - quarter) as f32 / quarter as f32),
        2 => 1.0,
        _ => 1.0 - smooth((phase - 3 * quarter) as f32 / quarter as f32),
    };
    [near + (far - near) * along, y, z]
}

/// Whether the ferry is crossing at `tick`.
fn is_ferry_moving(tick: u32) -> bool {
    matches!(ferry_phase(tick) / (FERRY_PERIOD / 4), 1 | 3)
}

/// Ticks into the ferry's round trip, which starts with the wait at the near end.
fn ferry_phase(tick: u32) -> u32 {
    (tick + FERRY_PERIOD - FERRY_BOARDING % FERRY_PERIOD) % FERRY_PERIOD
}

/// The terrain, the stairs with their landing, the ramp, the steep slope and the crates.
fn build_course(
    world: &mut PhysicsWorld,
    layers: &Layers,
    visuals: &mut Visuals,
    tracked: &mut Tracked,
) -> Result<()> {
    let ground = BodySettings::new_static().object_layer(layers.ground);
    let hills = terrain::rolling(0.9);
    // Flat around the course, rolling hills beyond it.
    let terrain = terrain::height_field(65, 1.0, |x, z| {
        let outside = (x.abs() - 15.0).max(z.abs() - 7.0).max(0.0);
        hills(x, z) * (outside / 6.0).min(1.0)
    })?;
    tracked.spawn(world, &terrain, &ground, visuals, colours::GROUND)?;

    // Five steps of 0.2 m, 0.5 m deep, rounded by Jolt's default convex radius, up to a landing
    // 1 m high.
    for step in 0..5 {
        let top = 0.2 * (step + 1) as f32;
        let x = -8.75 + 0.5 * step as f32;
        let settings = ground.clone().position(rvec([x, top / 2.0, 0.0]));
        let shaped = Shaped::cuboid([0.25, top / 2.0, 1.0])?;
        tracked.spawn(world, &shaped, &settings, visuals, colours::STRUCTURE)?;
    }
    let landing = ground.clone().position(rvec([-5.5, 0.5, 0.0]));
    tracked.spawn(
        world,
        &Shaped::cuboid([1.0, 0.5, 1.0])?,
        &landing,
        visuals,
        colours::STRUCTURE,
    )?;

    // A 30° ramp down from the landing, a triangle mesh.
    let run = 1.0 / 30.0_f32.to_radians().tan();
    let (x0, x1) = (-4.5, -4.5 + run);
    let points = [
        [x0, 1.0, -1.0],
        [x0, 1.0, 1.0],
        [x0, 0.0, -1.0],
        [x0, 0.0, 1.0],
        [x1, 0.0, -1.0],
        [x1, 0.0, 1.0],
    ];
    let faces = [
        [0, 1, 5],
        [0, 5, 4],
        [2, 4, 5],
        [2, 5, 3],
        [0, 2, 3],
        [0, 3, 1],
        [0, 4, 2],
        [1, 3, 5],
    ];
    let (ramp, dropped) = Shaped::mesh(&points, &face_outward(&points, &faces))?;
    if dropped > 0 {
        return Err(format!("the ramp mesh lost {dropped} triangles").into());
    }
    tracked.spawn(world, &ramp, &ground, visuals, colours::STRUCTURE)?;

    // A 50° slope beside the course: too steep to stand on, the walker slides off.
    let rise = 1.5 * 50.0_f32.to_radians().tan();
    let wedge = [
        [0.0, 0.0, -1.0],
        [0.0, 0.0, 1.0],
        [1.5, 0.0, -1.0],
        [1.5, 0.0, 1.0],
        [1.5, rise, -1.0],
        [1.5, rise, 1.0],
    ];
    let wedge_faces = [
        [0, 2, 3],
        [0, 3, 1],
        [0, 4, 5],
        [0, 5, 1],
        [2, 4, 5],
        [2, 5, 3],
        [0, 2, 4],
        [1, 5, 3],
    ];
    let slope = Shaped::hull(&wedge, &wedge_faces)?;
    let at = ground.clone().position(rvec([-3.0, 0.0, -5.0]));
    tracked.spawn(world, &slope, &at, visuals, colours::STRUCTURE)?;

    // Crates to push, 20 kg each.
    let crate_shape = Shaped::cuboid([0.35, 0.35, 0.35])?;
    for (i, z) in [-3.0, -4.0, -3.5].into_iter().enumerate() {
        let settings = BodySettings::new_dynamic()
            .position(rvec([2.0 + i as f32 * 0.8, 0.36, z]))
            .object_layer(layers.moving)
            .mass(20.0);
        tracked.spawn(world, &crate_shape, &settings, visuals, colours::BODY)?;
    }
    Ok(())
}
