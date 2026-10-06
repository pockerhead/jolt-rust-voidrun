//! The interface every scene implements, the list of scenes, and the world setup they share.

use std::collections::BTreeSet;

use oxijolt::{
    BroadPhaseLayer, CollisionLayers, EventSettings, ObjectLayer, PhysicsWorld, Vec3, WorldSettings,
};

use crate::camera::CameraHint;
use crate::digest::Digest;
use crate::draw::DrawList;
use crate::input::Input;
use crate::scenes;
use crate::visual::Visuals;

/// The fixed time step of every scene, seconds.
pub const DT: f32 = 1.0 / 60.0;

/// The playground's error type: any error, with a message for the screen.
pub type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

/// How a scene's world is built.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SceneConfig {
    /// Jolt worker threads; `None` takes the scene's own default.
    pub worker_threads: Option<u32>,
}

/// What a scene's recorded clip must show, and what the scene showed so far.
#[derive(Clone, Debug)]
pub struct Milestones {
    all: &'static [&'static str],
    reached: BTreeSet<&'static str>,
}

impl Milestones {
    /// None of `all` reached yet.
    pub fn new(all: &'static [&'static str]) -> Self {
        Self {
            all,
            reached: BTreeSet::new(),
        }
    }

    /// Marks `name`, one of the scene's milestones, as reached.
    pub fn reach(&mut self, name: &'static str) {
        debug_assert!(self.all.contains(&name), "unknown milestone {name}");
        self.reached.insert(name);
    }

    /// Every milestone of the scene, in the order the scene lists them.
    pub fn all(&self) -> &'static [&'static str] {
        self.all
    }

    /// Whether `name` was reached.
    pub fn is_reached(&self, name: &str) -> bool {
        self.reached.contains(name)
    }

    /// The milestones not reached yet, in the scene's order.
    pub fn missing(&self) -> Vec<&'static str> {
        self.all
            .iter()
            .copied()
            .filter(|name| !self.reached.contains(name))
            .collect()
    }
}

/// One focused scene: a world of its own, the input that drives it and what to draw.
///
/// Every physics change happens in [`update`](Self::update), once per fixed tick, from the
/// tick's input alone, so a scene run is a function of its inputs.
pub trait Scene {
    /// Applies the tick's input and steps the world once by [`DT`].
    fn update(&mut self, input: &Input) -> Result<()>;
    /// Appends what the binding reports (poses, vertices, lines) and the scene's HUD lines.
    fn draw(&self, out: &mut DrawList) -> Result<()>;
    /// Folds the scene's own state into `digest`, with typed values: characters, vehicles,
    /// joints and the script's counters. The session folds every body of the world before it.
    fn write_state(&self, digest: &mut Digest) -> Result<()>;
    /// The shape descriptions the draw list refers to.
    fn visuals(&self) -> &Visuals;
    /// The scene's world.
    fn world(&self) -> &PhysicsWorld;
    /// The scene's world, for reads that need it exclusively, such as its body list.
    fn world_mut(&mut self) -> &mut PhysicsWorld;
    /// Where the interactive camera starts and what it follows.
    fn camera(&self) -> CameraHint;
    /// The scripted input of tick `tick`, for the headless and record modes.
    fn script(&self, tick: u32) -> Input;
    /// The fixed camera of the recorded clip at tick `tick`.
    fn record_camera(&self, tick: u32) -> CameraHint {
        let _ = tick;
        self.camera()
    }
    /// How many ticks the recorded clip lasts.
    fn record_ticks(&self) -> u32 {
        300
    }
    /// What the clip must show and what was shown so far.
    fn milestones(&self) -> &Milestones;
    /// The scene's own keys and what they do, for the help panel.
    fn controls(&self) -> &'static [(&'static str, &'static str)];
    /// Whether the clip shows the collider wireframe, which the session draws.
    fn shows_wireframe(&self) -> bool {
        false
    }
}

/// The scenes, in menu order.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum SceneKind {
    /// A character controller on terrain with stairs, slopes, a conveyor and a ferry.
    Character,
    /// A car, a tank and a motorcycle, and a walker to switch between them.
    Vehicles,
    /// A dynamic body pile with impact estimates and sleeping.
    Pile,
    /// Ragdolls that settle, and a puppet driven through a skeleton mapper.
    Ragdolls,
    /// Constraints with motors, limits and couplings.
    Constraints,
    /// Soft bodies: a cloth, a pressurised balloon and a soft cube.
    SoftBodies,
    /// Buoyancy and drag in a pool, with a current.
    Water,
    /// A wall of bricks in a mutable compound that impacts break.
    Destruction,
    /// Contact listener effects, a sensor, collision groups and materials.
    Contacts,
    /// Scene queries, body controls, save and restore, and a floating origin.
    Queries,
}

impl SceneKind {
    /// Every scene in menu order.
    pub const ALL: [Self; 10] = [
        Self::Character,
        Self::Vehicles,
        Self::Pile,
        Self::Ragdolls,
        Self::Constraints,
        Self::SoftBodies,
        Self::Water,
        Self::Destruction,
        Self::Contacts,
        Self::Queries,
    ];

    /// The command-line name, the number key and the menu title.
    fn about(self) -> (&'static str, char, &'static str) {
        match self {
            Self::Character => ("character", '1', "Character on terrain"),
            Self::Vehicles => ("vehicles", '2', "Car, tank and motorcycle"),
            Self::Pile => ("pile", '3', "Body pile and impacts"),
            Self::Ragdolls => ("ragdolls", '4', "Ragdolls and a mapped puppet"),
            Self::Constraints => ("constraints", '5', "Constraints and motors"),
            Self::SoftBodies => ("soft-bodies", '6', "Cloth, balloon and soft cube"),
            Self::Water => ("water", '7', "Buoyancy and water"),
            Self::Destruction => ("destruction", '8', "Breakable wall"),
            Self::Contacts => ("contacts", '9', "Contact control and sensors"),
            Self::Queries => ("queries", '0', "Queries, state and origin"),
        }
    }

    /// The name used on the command line and for media files.
    pub fn name(self) -> &'static str {
        self.about().0
    }

    /// The scene of a command-line name.
    pub fn from_name(name: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|kind| kind.name() == name)
    }

    /// The number key that switches to the scene.
    pub fn key(self) -> char {
        self.about().1
    }

    /// The title shown in the menu.
    pub fn title(self) -> &'static str {
        self.about().2
    }

    /// Builds the scene fresh, with visual keys of `generation`.
    pub fn build(self, config: &SceneConfig, generation: u64) -> Result<Box<dyn Scene>> {
        Ok(match self {
            Self::Character => Box::new(scenes::character::Character::new(config, generation)?),
            Self::Vehicles => Box::new(scenes::vehicles::Vehicles::new(config, generation)?),
            Self::Pile => Box::new(scenes::pile::Pile::new(config, generation)?),
            Self::Ragdolls => Box::new(scenes::ragdolls::Ragdolls::new(config, generation)?),
            Self::Constraints => {
                Box::new(scenes::constraints::Constraints::new(config, generation)?)
            }
            Self::SoftBodies => Box::new(scenes::soft_bodies::SoftBodies::new(config, generation)?),
            Self::Water => Box::new(scenes::water::Water::new(config, generation)?),
            Self::Destruction => {
                Box::new(scenes::destruction::Destruction::new(config, generation)?)
            }
            Self::Contacts => Box::new(scenes::contacts::Contacts::new(config, generation)?),
            Self::Queries => Box::new(scenes::queries::Queries::new(config, generation)?),
        })
    }
}

/// The object layers every scene world has: static ground, moving bodies, and a probe layer
/// for wheels and queries that sees both.
#[derive(Clone, Copy, Debug)]
pub struct Layers {
    /// Static bodies.
    pub ground: ObjectLayer,
    /// Dynamic and kinematic bodies.
    pub moving: ObjectLayer,
    /// What vehicle wheels query as.
    pub probe: ObjectLayer,
}

/// A world with [`Layers`], Earth gravity, `default_threads` workers unless `config` says
/// otherwise, and `events` plus body activation events, which every scene uses to sync poses.
pub fn new_world(
    config: &SceneConfig,
    default_threads: u32,
    events: EventSettings,
) -> Result<(PhysicsWorld, Layers)> {
    let mut table = CollisionLayers::new(2);
    let fixed = BroadPhaseLayer::new(0);
    let moving = BroadPhaseLayer::new(1);
    let layers = Layers {
        ground: table.add_object_layer(fixed),
        moving: table.add_object_layer(moving),
        probe: table.add_object_layer(moving),
    };
    table
        .enable_collision(layers.moving, layers.ground)
        .enable_collision(layers.moving, layers.moving)
        .enable_collision(layers.probe, layers.ground)
        .enable_collision(layers.probe, layers.moving);
    let settings = WorldSettings::default()
        .gravity(Vec3::new(0.0, -9.81, 0.0))
        .layers(table)
        .worker_threads(config.worker_threads.unwrap_or(default_threads));
    let mut world = PhysicsWorld::new(settings)?;
    world.set_event_settings(events.body_activation(true));
    Ok((world, layers))
}

/// Steps `world` once by [`DT`], as an error when Jolt had to drop contacts.
pub fn step(world: &mut PhysicsWorld) -> Result<()> {
    let report = world.step(DT)?;
    if !report.is_complete() {
        return Err(format!("the step dropped contacts: {report:?}").into());
    }
    Ok(())
}
