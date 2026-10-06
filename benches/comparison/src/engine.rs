//! What every engine adapter provides, and the settings a run passes to it.

use crate::scene::{SceneSpec, V3};

/// The time step of every run, in seconds.
pub const DT: f32 = 1.0 / 60.0;

/// The gravity of every run, m/s².
pub const GRAVITY: V3 = [0.0, -9.81, 0.0];

/// Friction coefficient of every body in the matched profile.
pub const MATCHED_FRICTION: f32 = 0.5;

/// Speculative contact distance of the matched profile, in metres (Jolt's and Rapier's default).
pub const MATCHED_SPECULATIVE_DISTANCE: f32 = 0.02;

/// Which settings an engine runs with besides the scene.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Profile {
    /// The same settings in every engine: friction 0.5, no restitution, no damping, no sleeping,
    /// no continuous collision, a 2 cm speculative distance.
    Matched,
    /// Each engine as shipped, except the time step, gravity and the scene.
    Defaults,
}

impl Profile {
    pub const ALL: [Profile; 2] = [Profile::Matched, Profile::Defaults];

    pub fn name(self) -> &'static str {
        match self {
            Self::Matched => "matched",
            Self::Defaults => "defaults",
        }
    }

    pub fn from_name(name: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|p| p.name() == name)
    }
}

/// How an engine is set up for one run.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Config {
    pub profile: Profile,
    /// Solver iterations per step (Jolt velocity steps, Rapier solver iterations, Avian
    /// substeps); `None` is the engine's choice for the profile.
    pub iterations: Option<u32>,
    /// Threads that run the step, counting the stepping thread.
    pub threads: u32,
    /// Whether joints are created and solved; off only for the fixture checks that must fail
    /// without them.
    pub joints: bool,
}

impl Config {
    pub fn new(profile: Profile, threads: u32) -> Self {
        Self {
            profile,
            iterations: None,
            threads,
            joints: true,
        }
    }
}

/// One body's state as an engine reports it, in the scene's body order.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BodyState {
    pub position: V3,
    /// Unit quaternion `[x, y, z, w]`.
    pub rotation: [f32; 4],
    pub linear_velocity: V3,
    pub angular_velocity: V3,
    pub sleeping: bool,
}

/// A physics engine set up with one scene.
pub trait Engine: Sized {
    /// Builds the world for `spec` with `config`. Everything that the first step would otherwise
    /// set up lazily is part of building where the engine allows it.
    fn build(spec: &SceneSpec, config: &Config) -> Result<Self, String>;

    /// Advances the world by [`DT`]. An error ends the run (dropped contacts, a broken time step).
    fn step(&mut self) -> Result<(), String>;

    /// Every body's state, in the scene's body order, into `out` (cleared first).
    fn read_state(&mut self, out: &mut Vec<BodyState>);

    /// Mass of body `index` in kg; `None` for fixed bodies.
    fn body_mass(&mut self, index: usize) -> Option<f32>;

    /// Principal moments of inertia of body `index` about its centre, when the engine exposes
    /// them.
    fn body_inertia(&mut self, index: usize) -> Option<V3>;

    /// The closest body that the ray from `origin` along the unit `direction` hits within
    /// `max_distance`, as its scene index and distance.
    fn cast_ray(&mut self, origin: V3, direction: V3, max_distance: f32) -> Option<(usize, f32)>;

    /// How many bodies are awake.
    fn awake_bodies(&mut self) -> usize;
}
