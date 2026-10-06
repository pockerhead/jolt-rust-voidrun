//! Soft bodies: a cloth pinned at its corners on four poles that catches falling boxes, a
//! pressurised balloon that squashes on the floor, and a soft cube held by volume constraints.
//! E releases the cloth's pins; F throws a ball.

use std::collections::{BTreeSet, VecDeque};

use oxijolt::{
    BodyId, BodySettings, EventSettings, LongRangeAttachment, MotionQuality, PhysicsWorld,
    SoftBodyBendType, SoftBodyEdge, SoftBodySettings, SoftBodySharedSettings, SoftBodyVertex,
    SoftBodyVertexAttributes, SoftBodyVolume, Vec3,
};

use crate::camera::CameraHint;
use crate::digest::Digest;
use crate::draw::{colours, Colour, DrawList, Surface};
use crate::input::{Edges, Input};
use crate::math::{glam, position, position_f32, rvec};
use crate::scene::{new_world, step, Layers, Milestones, Result, Scene, SceneConfig};
use crate::tracked::Tracked;
use crate::visual::{Shaped, VisualKey, Visuals};

/// Vertices along each side of the cloth, and their spacing, metres.
const CLOTH_SIDE: u32 = 15;
const CLOTH_SPACING: f32 = 0.2;
/// Height of the cloth's corners.
const CLOTH_HEIGHT: f32 = 2.5;
/// Where the balloon and the cube are dropped from.
const BALLOON: [f32; 3] = [3.6, 3.0, 0.5];
const CUBE: [f32; 3] = [-3.4, 2.5, 0.5];
/// The most balls thrown at once; the oldest goes first.
const MAX_BALLS: usize = 10;

const MILESTONES: &[&str] = &["cloth hangs", "balloon squashed", "pins released"];
const CONTROLS: &[(&str, &str)] = &[
    ("E", "release the cloth's pins"),
    ("F", "throw a ball at the cursor"),
];

/// One soft body, its faces and how it is drawn.
struct Soft {
    body: BodyId,
    faces: Vec<[u32; 3]>,
    colour: Colour,
}

/// The soft bodies scene.
pub struct SoftBodies {
    world: PhysicsWorld,
    layers: Layers,
    visuals: Visuals,
    tracked: Tracked,
    cloth: Soft,
    pins: Vec<u32>,
    pinned: bool,
    balloon: Soft,
    cube: Soft,
    ball: (Shaped, VisualKey),
    balls: VecDeque<BodyId>,
    milestones: Milestones,
}

impl SoftBodies {
    /// Builds the floor, the poles with the cloth, three boxes over it, the balloon and the cube.
    pub fn new(config: &SceneConfig, generation: u64) -> Result<Self> {
        let (mut world, layers) = new_world(config, 2, EventSettings::default())?;
        let mut visuals = Visuals::new(generation);
        let mut tracked = Tracked::default();
        let ground = BodySettings::new_static().object_layer(layers.ground);
        tracked.spawn(
            &mut world,
            &Shaped::plane(20.0)?,
            &ground,
            &mut visuals,
            colours::GROUND,
        )?;

        let (cloth, pins) = cloth(&mut world, &layers)?;
        let half = 0.5 * CLOTH_SPACING * (CLOTH_SIDE - 1) as f32;
        let pole = Shaped::cylinder(CLOTH_HEIGHT / 2.0 - 0.06, 0.06)?;
        for (x, z) in [(-half, -half), (half, -half), (-half, half), (half, half)] {
            let at = ground
                .clone()
                .position(rvec([x, CLOTH_HEIGHT / 2.0 - 0.06, z]));
            tracked.spawn(&mut world, &pole, &at, &mut visuals, colours::STRUCTURE)?;
        }
        let crate_shape = Shaped::cuboid([0.25; 3])?;
        for (i, x) in [-0.5, 0.4, 0.0].into_iter().enumerate() {
            let settings = BodySettings::new_dynamic()
                .position(rvec([x, 4.0 + 0.8 * i as f32, 0.2 * i as f32 - 0.2]))
                .object_layer(layers.moving)
                .mass(5.0);
            tracked.spawn(
                &mut world,
                &crate_shape,
                &settings,
                &mut visuals,
                colours::BODY,
            )?;
        }

        let balloon = balloon(&mut world, &layers)?;
        let cube = cube(&mut world, &layers)?;
        let ball_shape = Shaped::sphere(0.2)?;
        let ball_visual = visuals.add(ball_shape.visual.clone());
        Ok(Self {
            world,
            layers,
            visuals,
            tracked,
            cloth,
            pins,
            pinned: true,
            balloon,
            cube,
            ball: (ball_shape, ball_visual),
            balls: VecDeque::new(),
            milestones: Milestones::new(MILESTONES),
        })
    }

    /// Unpins the cloth's corners: each gets the mass of the others.
    fn release_pins(&mut self) -> Result<()> {
        if !self.pinned {
            return Ok(());
        }
        let mut cloth = self.world.soft_body_mut(self.cloth.body)?;
        for &pin in &self.pins {
            cloth.set_vertex_inverse_mass(pin, CLOTH_INVERSE_MASS)?;
        }
        self.pinned = false;
        Ok(())
    }

    /// Throws a ball along the cursor ray, or from the default camera at the cloth.
    fn throw(&mut self, input: &Input) -> Result<()> {
        if self.balls.len() >= MAX_BALLS {
            if let Some(oldest) = self.balls.pop_front() {
                self.tracked.remove(&mut self.world, oldest)?;
            }
        }
        let ray = input
            .held
            .aim
            .unwrap_or_else(|| self.camera().ray([0.0, 0.0], 16.0 / 9.0));
        let direction = glam(ray.direction).normalize();
        let settings = BodySettings::new_dynamic()
            .position(rvec((position(ray.origin) + direction).to_array()))
            .linear_velocity(Vec3::from((direction * 14.0).to_array()))
            .motion_quality(MotionQuality::LinearCast)
            .object_layer(self.layers.moving)
            .mass(2.0);
        let (shape, visual) = &self.ball;
        let body = self.tracked.spawn_keyed(
            &mut self.world,
            &shape.shape,
            *visual,
            &settings,
            colours::PLAYER,
        )?;
        self.balls.push_back(body);
        Ok(())
    }

    /// The world positions of `soft`'s vertices, in `f32`.
    fn positions(&self, soft: &Soft) -> Result<Vec<[f32; 3]>> {
        let mut states = Vec::new();
        self.world.soft_body(soft.body)?.vertices_into(&mut states);
        Ok(states
            .iter()
            .map(|state| position_f32(state.position))
            .collect())
    }

    fn check_milestones(&mut self) -> Result<()> {
        let cloth = self.positions(&self.cloth)?;
        let lowest = cloth.iter().map(|p| p[1]).fold(f32::INFINITY, f32::min);
        if self.pinned && lowest < CLOTH_HEIGHT - 0.6 {
            self.milestones.reach("cloth hangs");
        }
        if !self.pinned && lowest < 0.3 {
            self.milestones.reach("pins released");
        }
        let balloon = self.positions(&self.balloon)?;
        let extent = |axis: usize| {
            let values = balloon.iter().map(|p| p[axis]);
            values.clone().fold(f32::NEG_INFINITY, f32::max) - values.fold(f32::INFINITY, f32::min)
        };
        if extent(1) < 0.8 * extent(0) {
            self.milestones.reach("balloon squashed");
        }
        Ok(())
    }
}

/// Inverse mass of a cloth vertex: 0.1 kg each, 22.5 kg in all.
const CLOTH_INVERSE_MASS: f32 = 10.0;

/// The cloth: 15 x 15 vertices in a horizontal square at the poles' height, its four corners
/// pinned. Returns the body and the pinned vertices.
fn cloth(world: &mut PhysicsWorld, layers: &Layers) -> Result<(Soft, Vec<u32>)> {
    let n = CLOTH_SIDE;
    let half = 0.5 * CLOTH_SPACING * (n - 1) as f32;
    let corners = [0, n - 1, n * (n - 1), n * n - 1];
    let mut vertices = Vec::new();
    for z in 0..n {
        for x in 0..n {
            let at = Vec3::new(
                x as f32 * CLOTH_SPACING - half,
                0.0,
                z as f32 * CLOTH_SPACING - half,
            );
            let mut vertex = SoftBodyVertex::new(at);
            vertex.inverse_mass = if corners.contains(&(z * n + x)) {
                0.0
            } else {
                CLOTH_INVERSE_MASS
            };
            vertices.push(vertex);
        }
    }
    let mut faces = Vec::new();
    for z in 0..n - 1 {
        for x in 0..n - 1 {
            let i = z * n + x;
            faces.push([i, i + n, i + n + 1]);
            faces.push([i, i + n + 1, i + 1]);
        }
    }
    let attributes = SoftBodyVertexAttributes::default()
        .bend_compliance(Some(1.0))
        .long_range_attachment(LongRangeAttachment::GeodesicDistance, 1.1);
    let shared = SoftBodySharedSettings::builder(vertices, faces.clone())
        .create_constraints(SoftBodyBendType::Dihedral, attributes)
        .build()?;
    let body = world.create_soft_body(
        &shared,
        &SoftBodySettings::default()
            .position(rvec([0.0, CLOTH_HEIGHT, 0.0]))
            .object_layer(layers.moving)
            .vertex_radius(0.02),
    )?;
    let soft = Soft {
        body,
        faces,
        colour: colours::SOFT,
    };
    Ok((soft, corners.to_vec()))
}

/// A pressurised sphere of radius 0.5: 86 vertices of 50 g.
fn balloon(world: &mut PhysicsWorld, layers: &Layers) -> Result<Soft> {
    let (rings, segments, radius) = (8, 12, 0.5_f32);
    let mut points = vec![[0.0, radius, 0.0]];
    for ring in 1..rings {
        let polar = std::f32::consts::PI * ring as f32 / rings as f32;
        for segment in 0..segments {
            let azimuth = std::f32::consts::TAU * segment as f32 / segments as f32;
            points.push([
                radius * polar.sin() * azimuth.cos(),
                radius * polar.cos(),
                radius * polar.sin() * azimuth.sin(),
            ]);
        }
    }
    let bottom = points.len() as u32;
    points.push([0.0, -radius, 0.0]);
    let at = |ring: u32, segment: u32| 1 + (ring - 1) * segments + segment % segments;
    let mut faces = Vec::new();
    for segment in 0..segments {
        faces.push([0, at(1, segment + 1), at(1, segment)]);
        faces.push([bottom, at(rings - 1, segment), at(rings - 1, segment + 1)]);
    }
    for ring in 1..rings - 1 {
        for segment in 0..segments {
            let (a, b) = (at(ring, segment), at(ring, segment + 1));
            let (c, d) = (at(ring + 1, segment), at(ring + 1, segment + 1));
            faces.push([a, b, d]);
            faces.push([a, d, c]);
        }
    }
    let vertices = points
        .iter()
        .map(|&p| SoftBodyVertex {
            inverse_mass: 20.0,
            ..SoftBodyVertex::new(p.into())
        })
        .collect();
    let shared = SoftBodySharedSettings::builder(vertices, faces.clone())
        .create_constraints(
            SoftBodyBendType::None,
            SoftBodyVertexAttributes::default().compliance(1.0e-3),
        )
        .build()?;
    let body = world.create_soft_body(
        &shared,
        &SoftBodySettings::default()
            .position(rvec(BALLOON))
            .object_layer(layers.moving)
            .pressure(1000.0),
    )?;
    Ok(Soft {
        body,
        faces,
        colour: colours::BODY_ALT,
    })
}

/// A soft cube of 0.8 m: eight corners, six tetrahedra with volume constraints, an edge along
/// every tetrahedron edge.
fn cube(world: &mut PhysicsWorld, layers: &Layers) -> Result<Soft> {
    const TETRAHEDRA: [[u32; 4]; 6] = [
        [0, 1, 3, 7],
        [0, 3, 2, 7],
        [0, 2, 6, 7],
        [0, 6, 4, 7],
        [0, 4, 5, 7],
        [0, 5, 1, 7],
    ];
    const FACES: [[u32; 3]; 12] = [
        [0, 4, 6],
        [0, 6, 2],
        [1, 3, 7],
        [1, 7, 5],
        [0, 1, 5],
        [0, 5, 4],
        [2, 6, 7],
        [2, 7, 3],
        [0, 2, 3],
        [0, 3, 1],
        [4, 5, 7],
        [4, 7, 6],
    ];
    let side = 0.8;
    let vertices = (0..8)
        .map(|corner: u32| {
            let at = |bit: u32| side * ((corner >> bit) & 1) as f32 - 0.5 * side;
            SoftBodyVertex::new(Vec3::new(at(0), at(1), at(2)))
        })
        .collect();
    let mut builder = SoftBodySharedSettings::builder(vertices, FACES.to_vec());
    let mut edges = BTreeSet::new();
    for tetrahedron in TETRAHEDRA {
        for i in 0..4 {
            for j in i + 1..4 {
                let (a, b) = (tetrahedron[i], tetrahedron[j]);
                edges.insert((a.min(b), a.max(b)));
            }
        }
        builder = builder.volume(SoftBodyVolume {
            vertices: tetrahedron,
            compliance: 0.0,
        });
    }
    for (a, b) in edges {
        builder = builder.edge(SoftBodyEdge {
            vertices: [a, b],
            compliance: 2.0e-3,
        });
    }
    let body = world.create_soft_body(
        &builder.build()?,
        &SoftBodySettings::default()
            .position(rvec(CUBE))
            .object_layer(layers.moving),
    )?;
    Ok(Soft {
        body,
        faces: FACES.to_vec(),
        colour: colours::BODY_THIRD,
    })
}

impl Scene for SoftBodies {
    fn update(&mut self, input: &Input) -> Result<()> {
        if input.edges.action {
            self.release_pins()?;
        }
        if input.edges.fire {
            self.throw(input)?;
        }
        step(&mut self.world)?;
        let events = self.world.take_events();
        self.tracked.sync(&self.world, &events);
        self.check_milestones()
    }

    fn draw(&self, out: &mut DrawList) -> Result<()> {
        self.tracked.draw(out);
        for soft in [&self.cloth, &self.balloon, &self.cube] {
            out.surfaces.push(Surface {
                vertices: self.positions(soft)?,
                triangles: soft.faces.clone(),
                colour: soft.colour,
                translucent: false,
            });
        }
        let pins = if self.pinned { "pinned" } else { "released" };
        out.hud.push(format!(
            "cloth: 225 vertices, corners {pins}; balloon: 86 vertices under pressure; cube: 6 volume constraints"
        ));
        Ok(())
    }

    fn write_state(&self, digest: &mut Digest) -> Result<()> {
        self.tracked.write_state(digest);
        digest.bool(self.pinned);
        Ok(())
    }

    fn visuals(&self) -> &Visuals {
        &self.visuals
    }

    fn world(&self) -> &PhysicsWorld {
        &self.world
    }

    fn camera(&self) -> CameraHint {
        CameraHint::new([0.0, 1.3, 0.0], 0.25, 0.4, 10.0)
    }

    /// The soft cube landing and hit by a ball, the balloon hit by another, then the cloth
    /// letting go.
    fn record_camera(&self, tick: u32) -> CameraHint {
        match tick {
            0..110 => CameraHint::new([CUBE[0], 0.6, CUBE[2]], 0.3, 0.25, 3.4),
            110..200 => CameraHint::new([BALLOON[0] - 0.4, 0.8, BALLOON[2]], 0.2, 0.25, 4.5),
            _ => CameraHint::new([0.0, 1.3, 0.0], 0.25, 0.35, 6.5),
        }
    }

    fn record_ticks(&self) -> u32 {
        360
    }

    fn script(&self, tick: u32) -> Input {
        let mut input = Input::default();
        // The first ball at the cube, the second at the balloon.
        let thrower = if tick < 100 {
            CameraHint::new([CUBE[0], 0.4, CUBE[2]], 0.3, 0.25, 6.0)
        } else {
            CameraHint::new([3.6, 0.8, 0.5], 0.25, 0.25, 9.0)
        };
        input.held.aim = Some(thrower.ray([0.0, 0.0], 16.0 / 9.0));
        input.edges = Edges {
            fire: matches!(tick, 50 | 150),
            action: tick == 220,
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
