//! The real models of `assets/models` as static meshes: a racing track tile, a dungeon corridor
//! whose floor has both faces, and a row of Kenney furniture and Crane's oloid, with spheres and
//! boxes raining onto them. With `--models DIR` (the directory `scripts/fetch_models.py`
//! filled) the scene adds Crane's spot and the Khronos ScatteringSkull as a 2.5 m statue.

use std::path::{Path, PathBuf};

use oxijolt::{BodySettings, EventSettings, PhysicsWorld};

use crate::camera::CameraHint;
use crate::digest::Digest;
use crate::draw::{colours, DrawList};
use crate::input::Input;
use crate::scene::{new_world, step, Layers, Milestones, Result, Scene, SceneConfig};
use crate::scenes::model::{place, Rain};
use crate::tracked::Tracked;
use crate::visual::{Shaped, Visuals};

const MILESTONES: &[&str] = &["every model built", "60 bodies dropped", "a body asleep"];
const CONTROLS: &[(&str, &str)] = &[("Space", "drop a burst of bodies")];
/// Ticks between two raindrops.
const RAIN_TICKS: u32 = 4;

/// A model of the scene: its file under `assets/models` (or the `--models` directory), where it
/// stands (x, z) and its scale.
struct Placement {
    file: &'static str,
    anchor: [f32; 2],
    scale: f32,
}

/// The committed models.
const COMMITTED: [Placement; 7] = [
    Placement {
        file: "kenney-starter-kit-racing/track-straight.glb",
        anchor: [-6.0, 0.0],
        scale: 1.0,
    },
    Placement {
        file: "kenney-modular-dungeon-kit/corridor-wide-corner.glb",
        anchor: [6.0, 0.0],
        scale: 1.0,
    },
    Placement {
        file: "kenney-furniture-kit/radio.glb",
        anchor: [-2.5, -8.0],
        scale: 1.0,
    },
    Placement {
        file: "kenney-furniture-kit/kitchenFridgeLarge.glb",
        anchor: [-1.0, -8.0],
        scale: 1.0,
    },
    Placement {
        file: "kenney-furniture-kit/bathtub.glb",
        anchor: [1.0, -8.0],
        scale: 1.0,
    },
    Placement {
        file: "kenney-furniture-kit/bookcaseOpen.glb",
        anchor: [3.0, -8.0],
        scale: 1.0,
    },
    Placement {
        file: "crane-oloid/oloid256_tri.obj",
        anchor: [-5.0, -8.0],
        scale: 0.5,
    },
];

/// The downloaded models, read from the `--models` directory.
const DOWNLOADED: [Placement; 2] = [
    Placement {
        file: "spot.obj",
        anchor: [-3.0, 9.0],
        scale: 1.0,
    },
    Placement {
        file: "ScatteringSkull.gltf",
        anchor: [3.0, 9.0],
        scale: 10.0,
    },
];

fn committed_dir() -> PathBuf {
    PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/../../assets/models"))
}

/// The meshes scene.
pub struct Meshes {
    world: PhysicsWorld,
    layers: Layers,
    visuals: Visuals,
    tracked: Tracked,
    rain: Rain,
    models: usize,
    triangles: usize,
    dropped: usize,
    tick: u32,
    milestones: Milestones,
}

impl Meshes {
    /// Builds every committed model, and the downloaded ones when `config.models` names their
    /// directory.
    pub fn new(config: &SceneConfig, generation: u64) -> Result<Self> {
        let (mut world, layers) = new_world(config, 3, EventSettings::default())?;
        let mut visuals = Visuals::new(generation);
        let mut tracked = Tracked::default();
        let ground = BodySettings::new_static().object_layer(layers.ground);
        tracked.spawn(
            &mut world,
            &Shaped::plane(40.0)?,
            &ground,
            &mut visuals,
            colours::GROUND,
        )?;
        let mut placements: Vec<(PathBuf, &Placement)> = COMMITTED
            .iter()
            .map(|placement| (committed_dir().join(placement.file), placement))
            .collect();
        if let Some(dir) = &config.models {
            placements.extend(
                DOWNLOADED
                    .iter()
                    .map(|placement| (Path::new(dir).join(placement.file), placement)),
            );
        }
        let (mut triangles, mut dropped, mut areas) = (0, 0, Vec::new());
        for (path, placement) in &placements {
            let placed = place(path, placement.anchor, placement.scale)?;
            let (shaped, count) = Shaped::mesh(&placed.vertices, &placed.triangles)?;
            tracked.spawn(
                &mut world,
                &shaped,
                &ground,
                &mut visuals,
                colours::STRUCTURE,
            )?;
            triangles += placed.triangles.len();
            dropped += count;
            let [half_x, height, half_z] = placed.half_size;
            let [x, z] = placement.anchor;
            areas.push([x, 2.0 * height, z, half_x, half_z]);
        }
        let rain = Rain::new(areas, &mut visuals)?;
        let mut milestones = Milestones::new(MILESTONES);
        milestones.reach("every model built");
        Ok(Self {
            world,
            layers,
            visuals,
            tracked,
            rain,
            models: placements.len(),
            triangles,
            dropped,
            tick: 0,
            milestones,
        })
    }
}

impl Scene for Meshes {
    fn update(&mut self, input: &Input) -> Result<()> {
        let burst = if input.edges.jump { 2 * self.models } else { 0 };
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
        if self.rain.count() >= 60 {
            self.milestones.reach("60 bodies dropped");
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
            "{} models, {} triangles, {} dropped as too thin for Jolt",
            self.models, self.triangles, self.dropped
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
        CameraHint::new([0.0, 0.5, -1.0], 0.5, 0.6, 26.0)
    }

    /// The whole set first, then the furniture row from behind, with the track and the corridor
    /// beyond it.
    fn record_camera(&self, tick: u32) -> CameraHint {
        if tick < 150 {
            CameraHint::new([0.0, 0.5, -2.0], 0.5, 0.6, 22.0)
        } else {
            CameraHint::new([-1.0, 0.4, -8.0], 3.6, 0.45, 8.0)
        }
    }

    fn script(&self, tick: u32) -> Input {
        let mut input = Input::default();
        input.edges.jump = tick == 120;
        input
    }

    fn milestones(&self) -> &Milestones {
        &self.milestones
    }

    fn controls(&self) -> &'static [(&'static str, &'static str)] {
        CONTROLS
    }
}
