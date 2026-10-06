//! Buoyancy and drag: crates of different buoyancy in a pool, a raft of planks and balls. Each
//! tick every floating body gets its fluid's buoyancy impulse before the step; C turns on a
//! current that carries what floats.

use oxijolt::{BodyId, BodySettings, BuoyancySettings, EventSettings, PhysicsWorld, Quat, Vec3};

use crate::camera::CameraHint;
use crate::digest::Digest;
use crate::draw::{colours, Colour, DrawList, Surface};
use crate::input::{Edges, Input};
use crate::math::{position_f32, rvec};
use crate::scene::{new_world, step, Layers, Milestones, Result, Scene, SceneConfig, DT};
use crate::tracked::Tracked;
use crate::visual::{Shaped, Visuals};

/// The pool's inside half extents along x and z, and its depth, metres.
const POOL: [f32; 3] = [6.0, 3.5, 3.0];
/// Speed of the current, m/s along +X.
const CURRENT: f32 = 2.0;
/// Buoyancy factors of the crates, from sinking to floating high, and their colours.
const CRATES: [(f32, Colour); 4] = [
    (0.5, colours::ASLEEP),
    (1.0, colours::BODY_THIRD),
    (2.0, colours::BODY),
    (4.0, colours::PLAYER),
];
/// Where the raft starts along x.
const RAFT_X: f32 = -4.5;
/// The most crates dropped with Space.
const MAX_DROPPED: usize = 12;

const MILESTONES: &[&str] = &[
    "light crate floats",
    "heavy crate sank",
    "current carried the raft",
];
const CONTROLS: &[(&str, &str)] = &[("C", "current on or off"), ("Space", "drop another crate")];

/// A body in the water and the buoyancy factor of its fluid.
struct Floater {
    body: BodyId,
    buoyancy: f32,
}

/// The water scene.
pub struct Water {
    world: PhysicsWorld,
    layers: Layers,
    visuals: Visuals,
    tracked: Tracked,
    crate_shape: Shaped,
    floaters: Vec<Floater>,
    dropped: usize,
    raft: BodyId,
    raft_start: f32,
    current: bool,
    tick: u32,
    milestones: Milestones,
}

impl Water {
    /// Builds the pool, four crates, the raft and three balls.
    pub fn new(config: &SceneConfig, generation: u64) -> Result<Self> {
        let (mut world, layers) = new_world(config, 2, EventSettings::default())?;
        let mut visuals = Visuals::new(generation);
        let mut tracked = Tracked::default();
        let ground = BodySettings::new_static().object_layer(layers.ground);
        let [half_x, half_z, depth] = POOL;
        tracked.spawn(
            &mut world,
            &Shaped::plane(20.0)?,
            &ground.clone().position(rvec([0.0, -depth, 0.0])),
            &mut visuals,
            colours::STRUCTURE,
        )?;
        // The ground around the pool: four blocks 4 m wide, their tops 0.6 m above the water.
        let wall_height = depth / 2.0 + 0.3;
        let walls = [
            ([half_x + 4.0, wall_height, 2.0], [0.0, half_z + 2.0]),
            ([half_x + 4.0, wall_height, 2.0], [0.0, -half_z - 2.0]),
            ([2.0, wall_height, half_z], [half_x + 2.0, 0.0]),
            ([2.0, wall_height, half_z], [-half_x - 2.0, 0.0]),
        ];
        for (half_extent, [x, z]) in walls {
            let at = ground.clone().position(rvec([x, 0.6 - wall_height, z]));
            tracked.spawn(
                &mut world,
                &Shaped::cuboid(half_extent)?,
                &at,
                &mut visuals,
                colours::GROUND,
            )?;
        }

        let crate_shape = Shaped::cuboid([0.35; 3])?;
        let raft = spawn_raft(&mut world, &layers, &mut visuals, &mut tracked)?;
        let mut scene = Self {
            world,
            layers,
            visuals,
            tracked,
            crate_shape,
            floaters: Vec::new(),
            dropped: 0,
            raft,
            raft_start: RAFT_X,
            current: false,
            tick: 0,
            milestones: Milestones::new(MILESTONES),
        };
        for (index, (buoyancy, colour)) in CRATES.into_iter().enumerate() {
            scene.add_crate([-3.0 + 2.0 * index as f32, 1.5, -1.5], buoyancy, colour)?;
        }
        scene.floaters.push(Floater {
            body: raft,
            buoyancy: 3.0,
        });
        let ball = Shaped::sphere(0.3)?;
        for (index, x) in [-4.0, -3.2, -2.4].into_iter().enumerate() {
            let settings = BodySettings::new_dynamic()
                .position(rvec([x, 1.0 + 0.4 * index as f32, 1.8]))
                .object_layer(scene.layers.moving)
                .mass(5.0);
            let body = scene.tracked.spawn(
                &mut scene.world,
                &ball,
                &settings,
                &mut scene.visuals,
                colours::BODY_ALT,
            )?;
            scene.floaters.push(Floater {
                body,
                buoyancy: 2.5,
            });
        }
        Ok(scene)
    }

    fn add_crate(&mut self, at: [f32; 3], buoyancy: f32, colour: Colour) -> Result<BodyId> {
        let settings = BodySettings::new_dynamic()
            .position(rvec(at))
            .rotation(crate::math::about_axis(
                [0.2, 1.0, 0.1],
                0.3 * self.floaters.len() as f32,
            ))
            .object_layer(self.layers.moving)
            .mass(60.0);
        let body = self.tracked.spawn(
            &mut self.world,
            &self.crate_shape,
            &settings,
            &mut self.visuals,
            colour,
        )?;
        self.floaters.push(Floater { body, buoyancy });
        Ok(body)
    }

    /// The fluid for a body of `buoyancy`, with the current when it is on.
    fn water(&self, buoyancy: f32) -> BuoyancySettings {
        let flow = if self.current { CURRENT } else { 0.0 };
        BuoyancySettings::default()
            .buoyancy(buoyancy)
            .linear_drag(0.5)
            .angular_drag(0.05)
            .fluid_velocity(Vec3::new(flow, 0.0, 0.0))
    }

    fn check_milestones(&mut self) -> Result<()> {
        let height = |world: &PhysicsWorld, body: BodyId| -> Result<f32> {
            Ok(position_f32(world.body(body)?.position())[1])
        };
        let speed = |world: &PhysicsWorld, body: BodyId| -> Result<f32> {
            Ok(crate::math::glam(world.body(body)?.linear_velocity()).length())
        };
        let light = self.floaters[3].body;
        if self.tick > 120 && height(&self.world, light)? > -0.2 && speed(&self.world, light)? < 0.5
        {
            self.milestones.reach("light crate floats");
        }
        if height(&self.world, self.floaters[0].body)? < 1.0 - POOL[2] {
            self.milestones.reach("heavy crate sank");
        }
        let raft_x = position_f32(self.world.body(self.raft)?.position())[0];
        if self.current && raft_x - self.raft_start > 2.0 {
            self.milestones.reach("current carried the raft");
        }
        Ok(())
    }
}

impl Scene for Water {
    fn update(&mut self, input: &Input) -> Result<()> {
        self.tick += 1;
        if input.edges.toggle {
            self.current = !self.current;
            self.raft_start = position_f32(self.world.body(self.raft)?.position())[0];
        }
        if input.edges.jump && self.dropped < MAX_DROPPED {
            let (buoyancy, colour) = CRATES[self.dropped % CRATES.len()];
            let x = -4.0 + 1.6 * (self.dropped % 6) as f32;
            self.dropped += 1;
            self.add_crate([x, 2.5, 0.0], buoyancy, colour)?;
        }
        let gravity = self.world.gravity();
        for index in 0..self.floaters.len() {
            let water = self.water(self.floaters[index].buoyancy);
            self.world
                .body_mut(self.floaters[index].body)?
                .apply_buoyancy_impulse(&water, gravity, DT)?;
        }
        step(&mut self.world)?;
        let events = self.world.take_events();
        self.tracked.sync(&self.world, &events);
        self.check_milestones()
    }

    fn draw(&self, out: &mut DrawList) -> Result<()> {
        self.tracked.draw(out);
        let [x, z, _] = POOL;
        out.surfaces.push(Surface {
            vertices: vec![[-x, 0.0, -z], [x, 0.0, -z], [x, 0.0, z], [-x, 0.0, z]],
            triangles: vec![[0, 2, 1], [0, 3, 2]],
            colour: colours::WATER,
            translucent: true,
        });
        let current = if self.current {
            format!("current {CURRENT} m/s toward +X")
        } else {
            "still water".to_owned()
        };
        out.hud.push(format!(
            "{current}; crates of buoyancy 0.5 (lavender), 1, 2 and 4 (yellow)"
        ));
        Ok(())
    }

    fn write_state(&self, digest: &mut Digest) -> Result<()> {
        self.tracked.write_state(digest);
        digest.bool(self.current);
        digest.u32(self.tick);
        digest.u64(self.dropped as u64);
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
        CameraHint::new([0.0, -0.3, 0.0], 0.3, 0.55, 13.0)
    }

    /// Low over the crates of different buoyancy, then beside the raft as the current takes
    /// it.
    fn record_camera(&self, tick: u32) -> CameraHint {
        if tick < 120 {
            CameraHint::new([0.0, -0.3, -1.5], 0.0, 0.25, 4.5)
        } else {
            let x = self
                .tracked
                .pose(self.raft)
                .map_or(RAFT_X, |(p, _)| position_f32(p)[0]);
            CameraHint::new([x + 1.0, 0.0, 0.6], 0.25, 0.35, 6.0)
        }
    }

    fn record_ticks(&self) -> u32 {
        360
    }

    fn script(&self, tick: u32) -> Input {
        Input {
            edges: Edges {
                toggle: tick == 120,
                jump: tick == 200,
                ..Edges::default()
            },
            ..Input::default()
        }
    }

    fn milestones(&self) -> &Milestones {
        &self.milestones
    }

    fn controls(&self) -> &'static [(&'static str, &'static str)] {
        CONTROLS
    }
}

/// A raft of five planks on two cross beams, floating at the pool's west end.
fn spawn_raft(
    world: &mut PhysicsWorld,
    layers: &Layers,
    visuals: &mut Visuals,
    tracked: &mut Tracked,
) -> Result<BodyId> {
    let plank = Shaped::cuboid([0.12, 0.05, 0.9])?;
    let beam = Shaped::cuboid([0.7, 0.05, 0.08])?;
    let mut children: Vec<(&Shaped, [f32; 3], Quat, u32)> = (0..5)
        .map(|i| {
            (
                &plank,
                [-0.56 + 0.28 * i as f32, 0.05, 0.0],
                Quat::IDENTITY,
                0,
            )
        })
        .collect();
    children.push((&beam, [0.0, -0.05, -0.6], Quat::IDENTITY, 0));
    children.push((&beam, [0.0, -0.05, 0.6], Quat::IDENTITY, 0));
    let settings = BodySettings::new_dynamic()
        .position(rvec([RAFT_X, 0.3, 0.6]))
        .object_layer(layers.moving)
        .mass(40.0);
    tracked.spawn(
        world,
        &Shaped::compound(&children)?,
        &settings,
        visuals,
        colours::BODY,
    )
}
