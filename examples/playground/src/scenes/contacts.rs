//! Contact control: a contact listener makes a one-way platform, a conveyor, an ice slab and a
//! trampoline out of plain static boxes; a sensor reports what is inside it; a chain's
//! overlapping links are kept apart by collision groups; floor tiles name their materials.

use std::collections::BTreeSet;
use std::sync::Arc;

use oxijolt::{
    BodyId, BodySettings, CollisionGroup, ContactCandidate, ContactEvent, ContactListener,
    ContactManifold, ContactSettings, EventSettings, GroupFilterTableBuilder, PhysicsMaterial,
    PhysicsWorld, PointConstraintSettings, Shape, ValidateResult, Vec3,
};

use crate::camera::CameraHint;
use crate::digest::Digest;
use crate::draw::{colours, Colour, DrawList};
use crate::input::{Edges, Input};
use crate::math::{about_axis, glam, position_f32, rvec};
use crate::scene::{new_world, step, Layers, Milestones, Result, Scene, SceneConfig};
use crate::tracked::Tracked;
use crate::visual::{Shaped, Visual, Visuals};

/// The body user data of the one-way platform.
const ONE_WAY: u64 = 1;
/// Material user data and their names.
const MATERIALS: [(u64, &str); 3] = [(1, "wood"), (2, "metal"), (3, "rubber")];
/// Stations: one-way platform, conveyor, ice, trampoline, sensor, chain, material tiles.
const STATIONS: [[f32; 2]; 7] = [
    [-6.0, -3.0],
    [-2.0, -3.0],
    [2.0, -3.0],
    [6.0, -3.0],
    [-6.0, 3.0],
    [-2.0, 3.0],
    [3.0, 3.0],
];
/// Height of the one-way platform.
const PLATFORM_HEIGHT: f32 = 2.5;
/// Speed of the conveyor's surface, m/s along +X.
const BELT_SPEED: f32 = 2.0;
/// Links of the chain.
const LINKS: u32 = 6;
/// The most boxes dropped with Space.
const MAX_DROPS: usize = 8;

const MILESTONES: &[&str] = &[
    "passed the one-way platform",
    "carried by the conveyor",
    "slid on ice",
    "bounced",
    "sensor saw a body",
    "material read",
];
const CONTROLS: &[(&str, &str)] = &[
    ("Space", "drop a box over every station"),
    ("E", "swing the chain"),
];

/// The listener: a one-way platform by body user data, and the stations' bodies, whose
/// contacts get a surface velocity, no friction or full restitution. Its decisions depend on
/// the contact alone.
struct Effects {
    conveyor: BodyId,
    ice: BodyId,
    trampoline: BodyId,
}

impl ContactListener for Effects {
    fn contact_validate(&self, contact: &ContactCandidate) -> ValidateResult {
        // Body 2 moves out of body 1 along the penetration axis: accept only pushes upward.
        let pushed_up = match contact.user_data {
            [ONE_WAY, _] => contact.penetration_axis.y > 0.0,
            [_, ONE_WAY] => contact.penetration_axis.y < 0.0,
            _ => return ValidateResult::AcceptAllContactsForThisBodyPair,
        };
        if pushed_up {
            ValidateResult::AcceptContact
        } else {
            ValidateResult::RejectContact
        }
    }

    fn contact_added(&self, manifold: &ContactManifold, settings: &mut ContactSettings) {
        self.contact_persisted(manifold, settings);
    }

    fn contact_persisted(&self, manifold: &ContactManifold, settings: &mut ContactSettings) {
        let pair = manifold.pair;
        let touches = |body: BodyId| pair.body1 == body || pair.body2 == body;
        let result = if touches(self.conveyor) {
            // Jolt wants body 2's surface velocity minus body 1's.
            let sign = if pair.body1 == self.conveyor {
                -1.0
            } else {
                1.0
            };
            settings.set_relative_linear_surface_velocity(Vec3::new(sign * BELT_SPEED, 0.0, 0.0))
        } else if touches(self.ice) {
            settings.set_combined_friction(0.0)
        } else if touches(self.trampoline) {
            settings.set_combined_restitution(1.0)
        } else {
            Ok(())
        };
        result.expect("the stations' values are within the contact limits");
    }
}

/// The contacts scene.
pub struct Contacts {
    world: PhysicsWorld,
    layers: Layers,
    visuals: Visuals,
    tracked: Tracked,
    box_shape: Shaped,
    boxes: Vec<BodyId>,
    chain: Vec<BodyId>,
    sensor: BodyId,
    inside: BTreeSet<BodyId>,
    last_material: Option<&'static str>,
    drops: usize,
    milestones: Milestones,
}

impl Contacts {
    /// Builds the stations, the chain and a box over every station.
    pub fn new(config: &SceneConfig, generation: u64) -> Result<Self> {
        let events = EventSettings::default().contacts(true);
        let (mut world, layers) = new_world(config, 2, events)?;
        let mut visuals = Visuals::new(generation);
        let mut tracked = Tracked::default();
        let ground = BodySettings::new_static().object_layer(layers.ground);
        tracked.spawn(
            &mut world,
            &Shaped::plane(30.0)?,
            &ground,
            &mut visuals,
            colours::GROUND,
        )?;
        let station = |index: usize, y: f32| {
            let [x, z] = STATIONS[index];
            rvec([x, y, z])
        };
        let slab = Shaped::cuboid([1.2, 0.1, 0.8])?;
        let mut fixed = |shaped: &Shaped, settings: BodySettings, colour: Colour| {
            tracked.spawn(&mut world, shaped, &settings, &mut visuals, colour)
        };
        fixed(
            &Shaped::cuboid([1.2, 0.05, 1.0])?,
            ground
                .clone()
                .position(station(0, PLATFORM_HEIGHT))
                .user_data(ONE_WAY),
            colours::KINEMATIC,
        )?;
        let conveyor = fixed(
            &slab,
            ground.clone().position(station(1, 0.1)),
            colours::BODY_ALT,
        )?;
        let ice = fixed(
            &slab,
            ground
                .clone()
                .position(station(2, 0.6))
                .rotation(about_axis([0.0, 0.0, 1.0], 10.0_f32.to_radians())),
            colours::WATER,
        )?;
        let trampoline = fixed(
            &slab,
            ground.clone().position(station(3, 0.1)),
            colours::HIGHLIGHT,
        )?;
        let sensor_shape = Shaped::cuboid([1.0, 0.8, 1.0])?;
        let sensor = world.create_body(
            &sensor_shape.shape,
            &ground.clone().position(station(4, 0.8)).sensor(true),
        )?;
        for (index, (user_data, _)) in MATERIALS.into_iter().enumerate() {
            let material = PhysicsMaterial::new(user_data)?;
            let tile = Shape::new_box_with_material(Vec3::new(0.6, 0.05, 0.8), 0.05, &material)?;
            let [x, z] = STATIONS[6];
            let at = ground
                .clone()
                .position(rvec([x - 1.3 + 1.3 * index as f32, 0.05, z]));
            let body = world.create_body(&tile, &at)?;
            let visual = visuals.add(Visual::Box {
                half_extent: [0.6, 0.05, 0.8],
            });
            let colour = [colours::BODY, colours::STRUCTURE, colours::SOFT][index];
            tracked.adopt(&world, body, visual, colour)?;
        }
        let chain = chain(&mut world, &layers, &mut visuals, &mut tracked)?;
        world.set_contact_listener(Some(Arc::new(Effects {
            conveyor,
            ice,
            trampoline,
        })));

        let box_shape = Shaped::cuboid([0.25; 3])?;
        let mut scene = Self {
            world,
            layers,
            visuals,
            tracked,
            box_shape,
            boxes: Vec::new(),
            chain,
            sensor,
            inside: BTreeSet::new(),
            last_material: None,
            drops: 0,
            milestones: Milestones::new(MILESTONES),
        };
        // One box shot up through the one-way platform, the others dropped over their stations.
        scene.add_box(station(0, 0.3), Vec3::new(0.0, 9.0, 0.0))?;
        scene.drop_boxes()?;
        Ok(scene)
    }

    fn add_box(&mut self, at: oxijolt::RVec3, velocity: Vec3) -> Result<()> {
        let settings = BodySettings::new_dynamic()
            .position(at)
            .linear_velocity(velocity)
            .object_layer(self.layers.moving)
            .mass(5.0);
        let body = self.tracked.spawn(
            &mut self.world,
            &self.box_shape,
            &settings,
            &mut self.visuals,
            colours::BODY,
        )?;
        self.boxes.push(body);
        Ok(())
    }

    /// Drops a box over the conveyor, the ice, the trampoline, the sensor and the tiles.
    fn drop_boxes(&mut self) -> Result<()> {
        if self.drops >= MAX_DROPS {
            return Ok(());
        }
        self.drops += 1;
        for index in [1, 2, 3, 4, 6] {
            let [x, z] = STATIONS[index];
            let x = if index == 2 { x + 0.6 } else { x - 0.6 };
            self.add_box(rvec([x, 2.5, z]), Vec3::ZERO)?;
        }
        Ok(())
    }

    /// Records the sensor's visitors and the materials of new contacts.
    fn handle_contacts(&mut self, contacts: &[ContactEvent]) {
        for event in contacts {
            let pair = event.pair();
            let other = if pair.body1 == self.sensor {
                Some(pair.body2)
            } else if pair.body2 == self.sensor {
                Some(pair.body1)
            } else {
                None
            };
            match (event, other) {
                (ContactEvent::Added { settings, .. }, Some(body)) if settings.is_sensor() => {
                    self.inside.insert(body);
                    self.milestones.reach("sensor saw a body");
                }
                (ContactEvent::Removed(_), Some(body)) => {
                    self.inside.remove(&body);
                }
                (ContactEvent::Added { manifold, .. }, None) => {
                    let named = manifold.materials.iter().flatten().find_map(|&user_data| {
                        MATERIALS
                            .iter()
                            .find(|(id, _)| *id == user_data)
                            .map(|(_, name)| *name)
                    });
                    if let Some(name) = named {
                        self.last_material = Some(name);
                        self.milestones.reach("material read");
                    }
                }
                _ => {}
            }
        }
    }

    fn check_milestones(&mut self) -> Result<()> {
        for &body in &self.boxes {
            let reading = self.world.body(body)?;
            let p = position_f32(reading.position());
            let v = glam(reading.linear_velocity());
            let near = |index: usize, radius: f32| {
                let [x, z] = STATIONS[index];
                (p[0] - x).abs() < radius && (p[2] - z).abs() < 1.0
            };
            if near(0, 1.2) && p[1] > PLATFORM_HEIGHT + 0.1 && v.y.abs() < 0.1 {
                self.milestones.reach("passed the one-way platform");
            }
            if near(1, 1.3) && p[1] < 0.6 && v.x > 1.0 {
                self.milestones.reach("carried by the conveyor");
            }
            if near(2, 1.3) && v.length() > 1.0 && p[1] < 1.2 {
                self.milestones.reach("slid on ice");
            }
            if near(3, 1.3) && v.y > 3.0 && p[1] < 1.5 {
                self.milestones.reach("bounced");
            }
        }
        Ok(())
    }
}

/// A chain of capsule links hanging from a hook, each link overlapping the next; collision
/// groups keep neighbours from colliding.
fn chain(
    world: &mut PhysicsWorld,
    layers: &Layers,
    visuals: &mut Visuals,
    tracked: &mut Tracked,
) -> Result<Vec<BodyId>> {
    let [x, z] = STATIONS[5];
    let top = 4.0;
    let hook = tracked.spawn(
        world,
        &Shaped::cuboid([0.1; 3])?,
        &BodySettings::new_static()
            .position(rvec([x, top + 0.1, z]))
            .object_layer(layers.ground),
        visuals,
        colours::STRUCTURE,
    )?;
    let mut table = GroupFilterTableBuilder::new(LINKS)?;
    for link in 0..LINKS - 1 {
        table.disable_collision(link, link + 1)?;
    }
    let table = table.build();
    let link_shape = Shaped::capsule(0.22, 0.07)?;
    let spacing = 0.45;
    let mut links = Vec::new();
    let mut previous = hook;
    for link in 0..LINKS {
        let centre = [x, top - spacing * (link as f32 + 0.5), z];
        let settings = BodySettings::new_dynamic()
            .position(rvec(centre))
            .object_layer(layers.moving)
            .mass(1.0)
            .collision_group(CollisionGroup::new(&table, 1, link)?);
        let body = tracked.spawn(world, &link_shape, &settings, visuals, colours::BODY_THIRD)?;
        let joint = [x, top - spacing * link as f32, z];
        world.create_constraint(previous, body, &PointConstraintSettings::new(rvec(joint)))?;
        links.push(body);
        previous = body;
    }
    Ok(links)
}

impl Scene for Contacts {
    fn update(&mut self, input: &Input) -> Result<()> {
        if input.edges.jump {
            self.drop_boxes()?;
        }
        if input.edges.action {
            if let Some(&last) = self.chain.last() {
                self.world
                    .body_mut(last)?
                    .add_impulse(Vec3::new(4.0, 0.0, 2.0))?;
            }
        }
        step(&mut self.world)?;
        let events = self.world.take_events();
        self.tracked.sync(&self.world, &events);
        self.handle_contacts(&events.contacts);
        self.check_milestones()
    }

    fn draw(&self, out: &mut DrawList) -> Result<()> {
        let inside = &self.inside;
        self.tracked.draw_with(out, |body, colour| {
            if inside.contains(&body) {
                colours::HIGHLIGHT
            } else {
                colour
            }
        });
        // The sensor as a wireframe box.
        let [x, z] = STATIONS[4];
        let (low, high) = ([x - 1.0, 0.0, z - 1.0], [x + 1.0, 1.6, z + 1.0]);
        for (a, b) in box_edges(low, high) {
            out.line(a, b, colours::QUERY);
        }
        out.hud.push(format!(
            "sensor holds {} bodies; last material touched: {}",
            self.inside.len(),
            self.last_material.unwrap_or("none")
        ));
        out.hud.push(
            "back row: one-way platform, conveyor, ice, trampoline; front: sensor, chain, tiles"
                .to_owned(),
        );
        Ok(())
    }

    fn write_state(&self, digest: &mut Digest) {
        self.tracked.write_state(digest);
        for body in &self.inside {
            digest.u32(body.to_raw());
        }
        digest.u64(self.drops as u64);
    }

    fn visuals(&self) -> &Visuals {
        &self.visuals
    }

    fn world(&self) -> &PhysicsWorld {
        &self.world
    }

    fn camera(&self) -> CameraHint {
        CameraHint::new([0.0, 1.0, 0.0], 0.0, 0.55, 15.0)
    }

    fn record_ticks(&self) -> u32 {
        300
    }

    fn script(&self, tick: u32) -> Input {
        Input {
            edges: Edges {
                jump: tick == 150,
                action: tick == 60,
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

/// The twelve edges of the box from `low` to `high`.
fn box_edges(low: [f32; 3], high: [f32; 3]) -> Vec<([f32; 3], [f32; 3])> {
    let corner = |i: usize| {
        [
            if i & 1 == 0 { low[0] } else { high[0] },
            if i & 2 == 0 { low[1] } else { high[1] },
            if i & 4 == 0 { low[2] } else { high[2] },
        ]
    };
    let mut edges = Vec::new();
    for i in 0..8 {
        for bit in [1, 2, 4] {
            if i & bit == 0 {
                edges.push((corner(i), corner(i | bit)));
            }
        }
    }
    edges
}
