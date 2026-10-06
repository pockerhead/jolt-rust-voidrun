//! A bin of a few hundred dynamic bodies of every convex shape kind: impacts flash from Jolt's
//! collision estimates, sleeping bodies turn grey, and the keys add layers and fire balls.

use std::collections::{BTreeMap, BTreeSet, VecDeque};

use oxijolt::{
    ActivationEvent, BodyId, BodySettings, ContactEvent, EventSettings, MotionQuality,
    PhysicsWorld, Vec3,
};

use crate::camera::CameraHint;
use crate::digest::Digest;
use crate::draw::{colours, Colour, DrawList};
use crate::input::{Edges, Input};
use crate::math::{about_axis, glam, position, rvec};
use crate::scene::{new_world, step, Layers, Milestones, Result, Scene, SceneConfig};
use crate::tracked::Tracked;
use crate::visual::{Shaped, VisualKey, Visuals};

/// Bodies per layer: 8 across, 5 deep.
const LAYER: [u32; 2] = [8, 5];
/// Layers in the first pile.
const FIRST_LAYERS: u32 = 10;
/// The most dynamic bodies of the pile, balls not counted.
const MAX_PILE: usize = 800;
/// The most balls at once; the oldest goes first.
const MAX_BALLS: usize = 20;
/// Ticks an impact keeps its bodies lit.
const FLASH_TICKS: u32 = 10;
/// Velocity change, m/s, from which an estimated impact lights its bodies.
const LOUD_IMPACT: f32 = 3.0;
/// Inside half extents of the bin, metres.
const BIN: [f32; 2] = [3.4, 2.4];

const MILESTONES: &[&str] = &["400 bodies", "impact estimate", "bodies asleep"];
const CONTROLS: &[(&str, &str)] = &[
    ("Space", "drop another layer"),
    ("F", "fire a heavy ball at the cursor"),
];

/// One shape kind of the pile, with the description its bodies share.
struct Kind {
    shaped: Shaped,
    visual: VisualKey,
    colour: Colour,
}

/// The pile scene.
pub struct Pile {
    world: PhysicsWorld,
    layers: Layers,
    visuals: Visuals,
    tracked: Tracked,
    kinds: Vec<Kind>,
    ball: Kind,
    pile: Vec<BodyId>,
    /// Crates resting beside the bin, which fall asleep early whatever the pile does.
    crates: Vec<BodyId>,
    balls: VecDeque<BodyId>,
    asleep: BTreeSet<BodyId>,
    flashes: BTreeMap<BodyId, u32>,
    /// The largest estimated velocity change of each of the last 60 ticks.
    loudest: VecDeque<f32>,
    milestones: Milestones,
}

impl Pile {
    /// Builds the bin and the first ten layers.
    pub fn new(config: &SceneConfig, generation: u64) -> Result<Self> {
        let (mut world, layers) = new_world(
            config,
            3,
            EventSettings::default().collision_estimates(true),
        )?;
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
        for (half_extent, position) in [
            ([BIN[0] + 0.2, 1.2, 0.2], [0.0, 1.2, BIN[1] + 0.2]),
            ([BIN[0] + 0.2, 1.2, 0.2], [0.0, 1.2, -BIN[1] - 0.2]),
            ([0.2, 1.2, BIN[1]], [BIN[0] + 0.2, 1.2, 0.0]),
            ([0.2, 1.2, BIN[1]], [-BIN[0] - 0.2, 1.2, 0.0]),
        ] {
            let wall = ground.clone().position(rvec(position));
            tracked.spawn(
                &mut world,
                &Shaped::cuboid(half_extent)?,
                &wall,
                &mut visuals,
                colours::STRUCTURE,
            )?;
        }
        let kinds = pile_kinds()?
            .into_iter()
            .map(|(shaped, colour)| Kind {
                visual: visuals.add(shaped.visual.clone()),
                shaped,
                colour,
            })
            .collect();
        let ball_shape = Shaped::sphere(0.35)?;
        let ball = Kind {
            visual: visuals.add(ball_shape.visual.clone()),
            shaped: ball_shape,
            colour: colours::PLAYER,
        };
        let mut scene = Self {
            world,
            layers,
            visuals,
            tracked,
            kinds,
            ball,
            pile: Vec::new(),
            crates: Vec::new(),
            balls: VecDeque::new(),
            asleep: BTreeSet::new(),
            flashes: BTreeMap::new(),
            loudest: VecDeque::new(),
            milestones: Milestones::new(MILESTONES),
        };
        for layer in 0..FIRST_LAYERS {
            scene.drop_layer(0.8 + layer as f32 * 0.85)?;
        }
        scene.add_crates()?;
        Ok(scene)
    }

    /// Adds a layer of 40 bodies at height `y`, cycling through the shape kinds.
    fn drop_layer(&mut self, y: f32) -> Result<()> {
        let [across, deep] = LAYER;
        for i in 0..across * deep {
            if self.pile.len() >= MAX_PILE {
                return Ok(());
            }
            let kind =
                &self.kinds[(self.pile.len() + i as usize / across as usize) % self.kinds.len()];
            let (row, column) = (i / across, i % across);
            let x = (column as f32 - (across - 1) as f32 / 2.0) * 0.8;
            let z = (row as f32 - (deep - 1) as f32 / 2.0) * 0.85;
            let turn = (self.pile.len() % 7) as f32 * 0.45;
            let settings = BodySettings::new_dynamic()
                .position(rvec([x, y, z]))
                .rotation(about_axis([0.3, 1.0, 0.2], turn))
                .object_layer(self.layers.moving)
                .friction(0.6);
            let body = self.tracked.spawn_keyed(
                &mut self.world,
                &kind.shaped.shape,
                kind.visual,
                &settings,
                kind.colour,
            )?;
            self.pile.push(body);
        }
        Ok(())
    }

    /// Three crates on the floor beside the bin.
    fn add_crates(&mut self) -> Result<()> {
        let crate_kind = &self.kinds[0];
        for z in [-1.2, 0.0, 1.2] {
            let settings = BodySettings::new_dynamic()
                .position(rvec([BIN[0] + 1.6, 0.21, z]))
                .object_layer(self.layers.moving);
            let body = self.tracked.spawn_keyed(
                &mut self.world,
                &crate_kind.shaped.shape,
                crate_kind.visual,
                &settings,
                crate_kind.colour,
            )?;
            self.crates.push(body);
        }
        Ok(())
    }

    /// Fires a heavy ball along the cursor ray, or from the default camera at the bin.
    fn fire(&mut self, input: &Input) -> Result<()> {
        if self.balls.len() >= MAX_BALLS {
            if let Some(oldest) = self.balls.pop_front() {
                self.remove(oldest)?;
            }
        }
        let ray = input
            .held
            .aim
            .unwrap_or_else(|| self.camera().ray([0.0, 0.0], 16.0 / 9.0));
        let direction = glam(ray.direction).normalize();
        let origin = position(ray.origin) + direction;
        let settings = BodySettings::new_dynamic()
            .position(rvec(origin.to_array()))
            .linear_velocity(Vec3::from((direction * 22.0).to_array()))
            .motion_quality(MotionQuality::LinearCast)
            .object_layer(self.layers.moving)
            .mass(80.0);
        let body = self.tracked.spawn_keyed(
            &mut self.world,
            &self.ball.shaped.shape,
            self.ball.visual,
            &settings,
            self.ball.colour,
        )?;
        self.balls.push_back(body);
        Ok(())
    }

    /// Whether `body` is one of the pile's bodies or balls, not the bin.
    fn is_dynamic(&self, body: BodyId) -> bool {
        self.pile.contains(&body) || self.crates.contains(&body) || self.balls.contains(&body)
    }

    fn remove(&mut self, body: BodyId) -> Result<()> {
        self.tracked.remove(&mut self.world, body)?;
        self.asleep.remove(&body);
        self.flashes.remove(&body);
        Ok(())
    }

    /// The largest velocity change an estimated impact gives either dynamic body.
    fn velocity_change(&self, event: &ContactEvent) -> Option<f32> {
        let ContactEvent::Added {
            manifold,
            estimate: Some(estimate),
            ..
        } = event
        else {
            return None;
        };
        let impulse: f32 = estimate.contact_impulses.iter().sum();
        [manifold.pair.body1, manifold.pair.body2]
            .into_iter()
            .filter_map(|body| self.world.body(body).ok()?.mass())
            .map(|mass| impulse / mass)
            .reduce(f32::max)
    }

    fn handle_events(&mut self) {
        let events = self.world.take_events();
        self.tracked.sync(&self.world, &events);
        let mut loudest = 0.0_f32;
        for event in &events.contacts {
            let Some(change) = self.velocity_change(event) else {
                continue;
            };
            loudest = loudest.max(change);
            if change >= LOUD_IMPACT {
                let pair = event.pair();
                for body in [pair.body1, pair.body2] {
                    if self.is_dynamic(body) {
                        self.flashes.insert(body, FLASH_TICKS);
                    }
                }
                self.milestones.reach("impact estimate");
            }
        }
        if self.loudest.len() == 60 {
            self.loudest.pop_front();
        }
        self.loudest.push_back(loudest);
        for event in &events.activations {
            match *event {
                ActivationEvent::Activated(body) => self.asleep.remove(&body),
                ActivationEvent::Deactivated(body) => {
                    self.is_dynamic(body) && self.asleep.insert(body)
                }
            };
        }
        if !self.asleep.is_empty() {
            self.milestones.reach("bodies asleep");
        }
    }
}

impl Scene for Pile {
    fn update(&mut self, input: &Input) -> Result<()> {
        if input.edges.jump {
            self.drop_layer(6.0)?;
        }
        if input.edges.fire {
            self.fire(input)?;
        }
        self.flashes.retain(|_, ticks| {
            *ticks -= 1;
            *ticks > 0
        });
        step(&mut self.world)?;
        self.handle_events();
        if self.pile.len() >= 400 {
            self.milestones.reach("400 bodies");
        }
        Ok(())
    }

    fn draw(&self, out: &mut DrawList) -> Result<()> {
        self.tracked.draw_with(out, |body, colour| {
            if self.flashes.contains_key(&body) {
                colours::HIGHLIGHT
            } else if self.asleep.contains(&body) {
                colours::ASLEEP
            } else {
                colour
            }
        });
        let loudest = self.loudest.iter().copied().fold(0.0, f32::max);
        out.hud.push(format!(
            "{} bodies, {} balls, {} asleep",
            self.pile.len(),
            self.balls.len(),
            self.asleep.len()
        ));
        out.hud.push(format!(
            "loudest impact of the last second: {loudest:.1} m/s (estimated)"
        ));
        Ok(())
    }

    fn write_state(&self, digest: &mut Digest) {
        self.tracked.write_state(digest);
        digest.u64(self.asleep.len() as u64);
        for body in &self.asleep {
            digest.u32(body.to_raw());
        }
        for (body, ticks) in &self.flashes {
            digest.u32(body.to_raw());
            digest.u32(*ticks);
        }
        for value in &self.loudest {
            digest.f32(*value);
        }
    }

    fn visuals(&self) -> &Visuals {
        &self.visuals
    }

    fn world(&self) -> &PhysicsWorld {
        &self.world
    }

    fn camera(&self) -> CameraHint {
        CameraHint::new([0.0, 1.0, 0.0], 0.6, 0.55, 14.0)
    }

    fn record_camera(&self, _tick: u32) -> CameraHint {
        CameraHint::new([0.0, 1.2, 0.0], 0.6, 0.6, 10.5)
    }

    fn script(&self, tick: u32) -> Input {
        let mut input = Input::default();
        // Aimed above the pile: the ball drops about a metre on its way.
        let aim = CameraHint::new([0.0, 2.2, 0.0], 0.6, 0.3, 12.0);
        input.held.aim = Some(aim.ray([0.0, 0.0], 16.0 / 9.0));
        input.edges = Edges {
            fire: matches!(tick, 150 | 200),
            jump: tick == 230,
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

/// The pile's shape kinds and their colours: box, sphere, capsule, cylinder, tapered capsule,
/// tapered cylinder, a wedge hull, a stretched box and a flattened cylinder.
fn pile_kinds() -> Result<Vec<(Shaped, Colour)>> {
    let wedge_points = [
        [-0.3, -0.2, -0.25],
        [0.3, -0.2, -0.25],
        [-0.3, -0.2, 0.25],
        [0.3, -0.2, 0.25],
        [-0.3, 0.2, -0.25],
        [0.3, 0.2, -0.25],
    ];
    let wedge_faces = [
        [0, 1, 3],
        [0, 3, 2],
        [0, 4, 5],
        [0, 5, 1],
        [2, 3, 5],
        [2, 5, 4],
        [0, 2, 4],
        [1, 5, 3],
    ];
    Ok(vec![
        (Shaped::cuboid([0.25, 0.2, 0.3])?, colours::BODY),
        (Shaped::sphere(0.27)?, colours::BODY_ALT),
        (Shaped::capsule(0.2, 0.17)?, colours::BODY_THIRD),
        (Shaped::cylinder(0.2, 0.25)?, colours::BODY),
        (Shaped::tapered_capsule(0.18, 0.12, 0.2)?, colours::BODY_ALT),
        (
            Shaped::tapered_cylinder(0.2, 0.12, 0.27)?,
            colours::BODY_THIRD,
        ),
        (Shaped::hull(&wedge_points, &wedge_faces)?, colours::BODY),
        (
            Shaped::scaled(&Shaped::cuboid([0.2, 0.2, 0.2])?, [1.6, 0.6, 1.0])?,
            colours::BODY_ALT,
        ),
        (
            Shaped::scaled(&Shaped::cylinder(0.2, 0.28)?, [1.0, 0.45, 1.0])?,
            colours::BODY_THIRD,
        ),
    ])
}
