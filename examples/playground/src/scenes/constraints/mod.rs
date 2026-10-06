//! Constraints with motors, limits and couplings, in three groups: motorized mechanisms, things
//! that hang, and joints that limit rotation.

mod joints;
mod mechanisms;
mod ropes;

use oxijolt::{BodyId, BodySettings, EventSettings, PhysicsWorld, Quat};

use crate::camera::CameraHint;
use crate::digest::Digest;
use crate::draw::{colours, Colour, DrawList};
use crate::input::{Edges, Input};
use crate::math::rvec;
use crate::scene::{new_world, step, Layers, Milestones, Result, Scene, SceneConfig};
use crate::tracked::Tracked;
use crate::visual::{Shaped, Visuals};

const MILESTONES: &[&str] = &[
    "windmill turning",
    "elevator at top",
    "gear driven",
    "cart moved along path",
    "puck stayed in its plane",
];
const CONTROLS: &[(&str, &str)] = &[
    ("Up, Down", "windmill motor faster or slower"),
    ("E", "elevator up or down"),
];

/// Creates the scene's bodies and keeps their descriptions.
pub struct Parts {
    layers: Layers,
    visuals: Visuals,
    tracked: Tracked,
}

impl Parts {
    /// A static body of `shaped` at `at`.
    fn fixed(&mut self, world: &mut PhysicsWorld, shaped: &Shaped, at: [f32; 3]) -> Result<BodyId> {
        let settings = BodySettings::new_static()
            .position(rvec(at))
            .object_layer(self.layers.ground);
        self.tracked.spawn(
            world,
            shaped,
            &settings,
            &mut self.visuals,
            colours::STRUCTURE,
        )
    }

    /// A dynamic body of `shaped` and `mass` kg at `at`.
    fn dynamic(
        &mut self,
        world: &mut PhysicsWorld,
        shaped: &Shaped,
        at: [f32; 3],
        mass: f32,
        colour: Colour,
    ) -> Result<BodyId> {
        self.dynamic_rotated(world, shaped, at, Quat::IDENTITY, mass, colour)
    }

    /// A dynamic body of `shaped` and `mass` kg at `at` turned by `rotation`.
    fn dynamic_rotated(
        &mut self,
        world: &mut PhysicsWorld,
        shaped: &Shaped,
        at: [f32; 3],
        rotation: Quat,
        mass: f32,
        colour: Colour,
    ) -> Result<BodyId> {
        let settings = BodySettings::new_dynamic()
            .position(rvec(at))
            .rotation(rotation)
            .mass(mass);
        self.add(world, shaped, &settings, colour)
    }

    /// A moving body of `shaped` with `settings`, in the moving layer.
    fn add(
        &mut self,
        world: &mut PhysicsWorld,
        shaped: &Shaped,
        settings: &BodySettings,
        colour: Colour,
    ) -> Result<BodyId> {
        let settings = settings.clone().object_layer(self.layers.moving);
        self.tracked
            .spawn(world, shaped, &settings, &mut self.visuals, colour)
    }
}

/// The constraints scene.
pub struct Constraints {
    world: PhysicsWorld,
    parts: Parts,
    mechanisms: mechanisms::Mechanisms,
    ropes: ropes::Ropes,
    joints: joints::Joints,
    tick: u32,
    milestones: Milestones,
}

impl Constraints {
    /// Builds the floor and the three groups.
    pub fn new(config: &SceneConfig, generation: u64) -> Result<Self> {
        let (mut world, layers) = new_world(config, 2, EventSettings::default())?;
        let mut parts = Parts {
            layers,
            visuals: Visuals::new(generation),
            tracked: Tracked::default(),
        };
        let floor = BodySettings::new_static().object_layer(layers.ground);
        parts.tracked.spawn(
            &mut world,
            &Shaped::plane(30.0)?,
            &floor,
            &mut parts.visuals,
            colours::GROUND,
        )?;
        let mechanisms = mechanisms::Mechanisms::new(&mut world, &mut parts)?;
        let ropes = ropes::Ropes::new(&mut world, &mut parts)?;
        let joints = joints::Joints::new(&mut world, &mut parts)?;
        Ok(Self {
            world,
            parts,
            mechanisms,
            ropes,
            joints,
            tick: 0,
            milestones: Milestones::new(MILESTONES),
        })
    }
}

impl Scene for Constraints {
    fn update(&mut self, input: &Input) -> Result<()> {
        self.tick += 1;
        self.mechanisms.update(&mut self.world, input, self.tick)?;
        step(&mut self.world)?;
        let events = self.world.take_events();
        self.parts.tracked.sync(&self.world, &events);
        self.mechanisms.check(&self.world, &mut self.milestones)?;
        self.joints.check(&self.world, &mut self.milestones)
    }

    fn draw(&self, out: &mut DrawList) -> Result<()> {
        self.parts.tracked.draw(out);
        self.ropes.draw(&self.world, out)?;
        self.mechanisms.draw(&self.world, out)?;
        self.joints.draw(out);
        Ok(())
    }

    fn write_state(&self, digest: &mut Digest) -> Result<()> {
        self.parts.tracked.write_state(digest);
        self.mechanisms.write_state(digest);
        self.joints.write_state(digest);
        digest.u32(self.tick);
        Ok(())
    }

    fn visuals(&self) -> &Visuals {
        &self.parts.visuals
    }

    fn world(&self) -> &PhysicsWorld {
        &self.world
    }

    fn world_mut(&mut self) -> &mut PhysicsWorld {
        &mut self.world
    }

    fn camera(&self) -> CameraHint {
        CameraHint::new([0.5, 1.8, 0.0], 0.0, 0.3, 17.0)
    }

    /// A close shot of each mechanism in turn: the windmill and the elevator, the gears and
    /// the rack, the cart on its path, the hanging things, the joints.
    fn record_camera(&self, tick: u32) -> CameraHint {
        match tick {
            0..70 => CameraHint::new([-9.6, 2.0, 0.0], 0.1, 0.15, 6.0),
            70..150 => CameraHint::new([-5.2, 1.0, 1.0], 0.0, 0.2, 3.8),
            150..210 => CameraHint::new([-8.0, 0.5, 4.0], 0.0, 0.6, 4.5),
            210..285 => CameraHint::new([2.5, 2.6, 0.0], -0.1, 0.25, 8.0),
            _ => CameraHint::new([10.5, 3.6, -2.0], 0.0, 0.1, 4.5),
        }
    }

    fn record_ticks(&self) -> u32 {
        360
    }

    fn script(&self, tick: u32) -> Input {
        let mut input = Input {
            edges: Edges {
                action: tick == 30,
                ..Edges::default()
            },
            ..Input::default()
        };
        // Up for a second speeds the windmill up by 0.5 rad/s.
        if (60..120).contains(&tick) {
            input.held.up_down = 1.0;
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
