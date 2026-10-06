//! Any OBJ or glTF model given with `--model PATH` (by default the committed track tile) as a
//! static mesh, with spheres and boxes raining onto it. The HUD shows what the mesh constructor
//! kept and dropped.

use std::collections::{BTreeSet, VecDeque};
use std::path::{Path, PathBuf};

use oxijolt::{
    ActivationEvent, BodyId, BodySettings, EventSettings, MotionQuality, PhysicsWorld, Shape,
};

use crate::camera::CameraHint;
use crate::digest::Digest;
use crate::draw::{colours, Colour, DrawList};
use crate::input::Input;
use crate::math::rvec;
use crate::scene::{new_world, step, Layers, Milestones, Result, Scene, SceneConfig};
use crate::tracked::Tracked;
use crate::visual::{Shaped, VisualKey, Visuals};

const MILESTONES: &[&str] = &["model built", "40 bodies dropped", "a body asleep"];
const CONTROLS: &[(&str, &str)] = &[("Space", "drop a burst of bodies")];
/// Ticks between two raindrops.
const RAIN_TICKS: u32 = 5;
/// The most raindrops at once; the oldest goes first.
const MAX_RAIN: usize = 120;

/// The model shown when `--model` is not given.
pub fn default_model() -> PathBuf {
    PathBuf::from(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../assets/models/kenney-starter-kit-racing/track-straight.glb"
    ))
}

/// How far above the ground plane models stand, metres, so that a floor at the model's bottom
/// is drawn above the plane rather than through it.
const LIFT: f32 = 0.02;

/// A model's triangles moved so that its bounding box stands [`LIFT`] above y = 0 centred on
/// `anchor` (x, z), and the box's half size.
pub struct Placed {
    pub vertices: Vec<[f32; 3]>,
    pub triangles: Vec<[u32; 3]>,
    pub half_size: [f32; 3],
}

/// Loads `path` and places it at `anchor`, scaled by `scale`.
pub fn place(path: &Path, anchor: [f32; 2], scale: f32) -> Result<Placed> {
    let mesh = mesh_import::load(path).map_err(|error| format!("{}: {error}", path.display()))?;
    let (min, max) = mesh
        .bounds()
        .ok_or_else(|| format!("{}: no vertices", path.display()))?;
    let centre = [(min[0] + max[0]) / 2.0, (min[2] + max[2]) / 2.0];
    let vertices = mesh
        .vertices
        .iter()
        .map(|v| {
            [
                (v[0] - centre[0]) * scale + anchor[0],
                (v[1] - min[1]) * scale + LIFT,
                (v[2] - centre[1]) * scale + anchor[1],
            ]
        })
        .collect();
    let half_size = [0, 1, 2].map(|axis| (max[axis] - min[axis]) * scale / 2.0);
    Ok(Placed {
        vertices,
        triangles: mesh.triangles,
        half_size,
    })
}

/// Spheres and boxes that fall over areas, one every few ticks, removing the oldest beyond a
/// cap and noting which of them sleep.
pub struct Rain {
    sphere: (Shape, VisualKey),
    block: (Shape, VisualKey),
    /// The areas, as (centre x, top y, centre z, half width x, half width z).
    areas: Vec<[f32; 5]>,
    drops: VecDeque<BodyId>,
    asleep: BTreeSet<BodyId>,
    count: u32,
}

impl Rain {
    pub fn new(areas: Vec<[f32; 5]>, visuals: &mut Visuals) -> Result<Self> {
        let Shaped { shape, visual } = Shaped::sphere(0.15)?;
        let sphere = (shape, visuals.add(visual));
        let Shaped { shape, visual } = Shaped::cuboid([0.12; 3])?;
        let block = (shape, visuals.add(visual));
        Ok(Self {
            sphere,
            block,
            areas,
            drops: VecDeque::new(),
            asleep: BTreeSet::new(),
            count: 0,
        })
    }

    /// Drops the next body over the next area, at a place that cycles through a 5 x 5 grid.
    pub fn drop_one(
        &mut self,
        world: &mut PhysicsWorld,
        tracked: &mut Tracked,
        layers: &Layers,
    ) -> Result<()> {
        if self.drops.len() >= MAX_RAIN {
            if let Some(oldest) = self.drops.pop_front() {
                tracked.remove(world, oldest)?;
                self.asleep.remove(&oldest);
            }
        }
        let n = self.count as usize;
        let [x, top, z, half_x, half_z] = self.areas[n % self.areas.len()];
        let cell = n / self.areas.len();
        let (u, v) = (
            (cell % 5) as f32 / 4.0 - 0.5,
            (cell / 5 % 5) as f32 / 4.0 - 0.5,
        );
        let position = [
            x + 1.6 * u * half_x,
            top + 1.5 + (n % 3) as f32 * 0.4,
            z + 1.6 * v * half_z,
        ];
        let ((shape, visual), colour) = if n.is_multiple_of(2) {
            (&self.sphere, colours::BODY_ALT)
        } else {
            (&self.block, colours::BODY)
        };
        let settings = BodySettings::new_dynamic()
            .position(rvec(position))
            .motion_quality(MotionQuality::LinearCast)
            .object_layer(layers.moving);
        let body = tracked.spawn_keyed(world, shape, *visual, &settings, colour)?;
        self.drops.push_back(body);
        self.count += 1;
        Ok(())
    }

    /// Notes the sleep changes of the step's activation events.
    pub fn note(&mut self, events: &[ActivationEvent]) {
        for event in events {
            match *event {
                ActivationEvent::Activated(body) => {
                    self.asleep.remove(&body);
                }
                ActivationEvent::Deactivated(body) => {
                    if self.drops.contains(&body) {
                        self.asleep.insert(body);
                    }
                }
            }
        }
    }

    /// Bodies dropped so far.
    pub fn count(&self) -> u32 {
        self.count
    }

    /// Whether a raindrop sleeps now.
    pub fn any_asleep(&self) -> bool {
        !self.asleep.is_empty()
    }

    /// The colour of `body`: grey while it sleeps.
    pub fn colour(&self, body: BodyId, colour: Colour) -> Colour {
        if self.asleep.contains(&body) {
            colours::ASLEEP
        } else {
            colour
        }
    }

    pub fn write_state(&self, digest: &mut Digest) {
        digest.u32(self.count);
        digest.u64(self.asleep.len() as u64);
        for body in &self.asleep {
            digest.u32(body.to_raw());
        }
    }
}

/// The model scene.
pub struct Model {
    world: PhysicsWorld,
    layers: Layers,
    visuals: Visuals,
    tracked: Tracked,
    rain: Rain,
    name: String,
    triangles: usize,
    dropped: usize,
    half_size: [f32; 3],
    tick: u32,
    milestones: Milestones,
}

impl Model {
    /// Loads the model of `config` (the track tile by default) onto a ground plane.
    pub fn new(config: &SceneConfig, generation: u64) -> Result<Self> {
        let (mut world, layers) = new_world(config, 2, EventSettings::default())?;
        let mut visuals = Visuals::new(generation);
        let mut tracked = Tracked::default();
        let path = config.model.clone().unwrap_or_else(default_model);
        let placed = place(&path, [0.0, 0.0], 1.0)?;
        let ground = BodySettings::new_static().object_layer(layers.ground);
        let extent = placed.half_size[0].max(placed.half_size[2]);
        tracked.spawn(
            &mut world,
            &Shaped::plane(4.0 * extent + 10.0)?,
            &ground,
            &mut visuals,
            colours::GROUND,
        )?;
        let (shaped, dropped) = Shaped::mesh(&placed.vertices, &placed.triangles)?;
        tracked.spawn(
            &mut world,
            &shaped,
            &ground,
            &mut visuals,
            colours::STRUCTURE,
        )?;
        let [half_x, height, half_z] = placed.half_size;
        let rain = Rain::new(vec![[0.0, 2.0 * height, 0.0, half_x, half_z]], &mut visuals)?;
        let name = path.file_name().map_or_else(
            || path.display().to_string(),
            |name| name.to_string_lossy().into_owned(),
        );
        let mut milestones = Milestones::new(MILESTONES);
        milestones.reach("model built");
        Ok(Self {
            world,
            layers,
            visuals,
            tracked,
            rain,
            name,
            triangles: placed.triangles.len(),
            dropped,
            half_size: placed.half_size,
            tick: 0,
            milestones,
        })
    }
}

impl Scene for Model {
    fn update(&mut self, input: &Input) -> Result<()> {
        let burst = if input.edges.jump { 10 } else { 0 };
        for _ in 0..burst {
            self.rain
                .drop_one(&mut self.world, &mut self.tracked, &self.layers)?;
        }
        if self.tick.is_multiple_of(RAIN_TICKS) {
            self.rain
                .drop_one(&mut self.world, &mut self.tracked, &self.layers)?;
        }
        self.tick += 1;
        step(&mut self.world)?;
        let events = self.world.take_events();
        self.tracked.sync(&self.world, &events);
        self.rain.note(&events.activations);
        if self.rain.count() >= 40 {
            self.milestones.reach("40 bodies dropped");
        }
        if self.rain.any_asleep() {
            self.milestones.reach("a body asleep");
        }
        Ok(())
    }

    fn draw(&self, out: &mut DrawList) -> Result<()> {
        self.tracked
            .draw_with(out, |body, colour| self.rain.colour(body, colour));
        out.hud.push(format!(
            "{}: {} triangles, {} dropped as too thin for Jolt",
            self.name, self.triangles, self.dropped
        ));
        out.hud
            .push(format!("{} bodies dropped", self.rain.count()));
        Ok(())
    }

    fn write_state(&self, digest: &mut Digest) -> Result<()> {
        self.tracked.write_state(digest);
        self.rain.write_state(digest);
        digest.u32(self.tick);
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
        let [half_x, height, half_z] = self.half_size;
        let size = half_x.max(half_z).max(height);
        CameraHint::new([0.0, height, 0.0], 0.6, 0.5, 3.0 * size + 2.0)
    }

    fn script(&self, tick: u32) -> Input {
        let mut input = Input::default();
        input.edges.jump = tick == 200;
        input
    }

    fn milestones(&self) -> &Milestones {
        &self.milestones
    }

    fn controls(&self) -> &'static [(&'static str, &'static str)] {
        CONTROLS
    }
}
