//! The study's scenes: heightfield slopes, crests, ridges, steps, ledges, walls, box ramps and
//! chunk seams, each with the decoded geometry the law predicates measure against.
//!
//! Planar scenes are built in local coordinates (slope rising along +x, up +y) and placed by a
//! [`Frame`]; their up is the frame's. Radial scenes sit at the anchor of the walker fixtures'
//! planet (radius 99) and use radial up.

use std::sync::atomic::{AtomicU32, Ordering};

use oxijolt::*;

use super::frame::{Frame, UpPolicy};
use super::geometry::{Block, Field, Surface};
use crate::common::math::{add, rvec3, scale, vec3, V3};
use crate::common::walker::{
    capsule, chunk_pose, fixture_world, flat_terrain, up_at, Layers, CENTRE_UP, R, RADIUS,
    REST_HEIGHT,
};
use crate::common::{quat_about, Groups};

/// Samples per side of every study heightfield except the continuous seam field.
pub const SAMPLES: usize = 33;

/// When not 0, the bits per sample of every study heightfield built from now on (the survey's
/// precision runs); the walker fixtures' planet chunks keep 16.
pub static FIELD_BITS_OVERRIDE: AtomicU32 = AtomicU32::new(0);

/// The heights of a seam scene, rising along +x.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Profile {
    Flat,
    /// Flat, 30 degrees for `-6 <= x <= 6`, then a plateau.
    Rise30,
    /// As `Rise30` at 44.5 degrees; the runs walk it down.
    Descent44_5,
}

impl Profile {
    pub fn height(self, x: f64) -> f64 {
        let tan = match self {
            Self::Flat => return 0.0,
            Self::Rise30 => 30.0_f64.to_radians().tan(),
            Self::Descent44_5 => 44.5_f64.to_radians().tan(),
        };
        tan * (x.clamp(-6.0, 6.0) + 6.0)
    }

    pub fn name(self) -> &'static str {
        match self {
            Self::Flat => "flat",
            Self::Rise30 => "rise30",
            Self::Descent44_5 => "descent44.5",
        }
    }
}

/// How a seam scene's terrain is split.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Split {
    /// Two adjacent 33-sample bodies meeting at x = 0.
    Pair,
    /// As `Pair` with the second body 0.03 m higher (the law 6 control).
    Raised,
    /// One 65-sample body over the same span.
    Continuous,
}

/// Which scene to build. Angles are in tenths of a degree, heights in millimetres.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum SceneKey {
    /// `h = tan * x` over the whole field.
    Plane { deg10: u16, bits: u8, tilted: bool },
    /// Flat for `x < -6`, the slope for `-6..6`, a plateau beyond: a concave toe and a convex
    /// crest at x = 6.
    Ramp { deg10: u16, tilted: bool },
    /// Up at the angle for `-6..0`, down for `0..6`, flat outside.
    Ridge { deg10: u16, tilted: bool },
    /// A box step with its face at x = 0 on flat terrain, its top `height_mm` high.
    Step { height_mm: u16, rounded: bool },
    /// A platform over `-4 <= x <= 0` whose edge at x = 0 drops `drop_mm` to flat terrain.
    Ledge { drop_mm: u16, rounded: bool },
    /// Flat terrain and a sharp wall whose face is at x = 0.35: half (0.1, 1, 2), or half
    /// (1, 1, 2) when thick.
    Wall { thick: bool },
    /// A tilted sharp box whose top face rises along +x at the angle through the origin.
    BoxRamp { deg10: u16, tilted: bool },
    /// Flat terrain.
    Flat,
    /// Seam terrain over `-32 <= x <= 32`.
    Seam { profile: Profile, split: Split },
    /// The log spiral `r = 99 exp(tan * phi)` at the anchor: a constant angle to radial up.
    RadialSpiral {
        deg10: u16,
        spacing_mm: u16,
        bits: u8,
    },
    /// The planet's surface for `x >= 0` and the spiral for `x < 0`: a convex crest at x = 0.
    RadialCrest { deg10: u16 },
    /// The walker fixtures' block at the anchor: flat chunk, sharp box face at x = 1.4.
    RadialStep { height_mm: u16 },
    /// Two flat chunks meeting at x = 16.
    RadialSeam,
    /// A flat chunk and a sharp wall of half (0.1, 1, 2) centred at x = 0.45.
    RadialWall,
}

impl SceneKey {
    pub fn plane(deg: f64) -> Self {
        Self::Plane {
            deg10: tenths(deg),
            bits: 16,
            tilted: false,
        }
    }

    /// Whether the scene uses radial up.
    pub fn is_radial(self) -> bool {
        matches!(
            self,
            Self::RadialSpiral { .. }
                | Self::RadialCrest { .. }
                | Self::RadialStep { .. }
                | Self::RadialSeam
                | Self::RadialWall
        )
    }
}

/// `deg` in tenths of a degree.
pub fn tenths(deg: f64) -> u16 {
    (deg * 10.0).round() as u16
}

fn degrees(deg10: u16) -> f64 {
    f64::from(deg10) / 10.0
}

/// A built scene.
pub struct Scene {
    pub world: PhysicsWorld,
    pub layers: Layers,
    pub frame: Frame,
    pub up: UpPolicy,
    pub surface: Surface,
    /// The terrain bodies in creation order.
    pub terrain: Vec<BodyId>,
    /// The game's kinematic actor capsule, which follows the character of the case being run
    /// and which the controller filter excludes.
    pub actor: BodyId,
}

impl Scene {
    /// Builds the scene of `key` in a world with one worker thread.
    pub fn build(key: SceneKey) -> Self {
        Self::build_with_threads(key, 1)
    }

    /// Builds the scene of `key` in a world with `worker_threads` worker threads.
    pub fn build_with_threads(key: SceneKey, worker_threads: u32) -> Self {
        let frame = match key {
            SceneKey::Plane { tilted: true, .. }
            | SceneKey::Ramp { tilted: true, .. }
            | SceneKey::Ridge { tilted: true, .. }
            | SceneKey::BoxRamp { tilted: true, .. } => Frame::tilted(),
            _ => Frame::IDENTITY,
        };
        let (world, layers) = fixture_world(worker_threads);
        let up = if key.is_radial() {
            UpPolicy::Radial
        } else {
            UpPolicy::Frame(frame)
        };
        let mut builder = Builder {
            world,
            layers,
            frame,
            surface: Surface::default(),
            terrain: Vec::new(),
        };
        builder.add(key);
        let actor = builder.actor();
        builder.world.optimize_broad_phase();
        Self {
            world: builder.world,
            layers: builder.layers,
            frame,
            up,
            surface: builder.surface,
            terrain: builder.terrain,
            actor,
        }
    }

    /// A scene around an existing fixture world (radial up, no decoded geometry), with an actor
    /// capsule added.
    pub fn from_world(world: PhysicsWorld, layers: Layers) -> Self {
        let mut builder = Builder {
            world,
            layers,
            frame: Frame::IDENTITY,
            surface: Surface::default(),
            terrain: Vec::new(),
        };
        let actor = builder.actor();
        Self {
            world: builder.world,
            layers,
            frame: Frame::IDENTITY,
            up: UpPolicy::Radial,
            surface: builder.surface,
            terrain: builder.terrain,
            actor,
        }
    }

    /// Moves the actor capsule to body origin `origin` with rotation `rotation`.
    pub fn place_actor(&mut self, origin: V3, rotation: Quat) {
        self.world
            .body_mut(self.actor)
            .unwrap()
            .set_position_and_rotation(rvec3(origin), rotation, Activation::DontActivate)
            .unwrap();
    }
}

/// Builds the bodies of a scene and records their geometry.
struct Builder {
    world: PhysicsWorld,
    layers: Layers,
    frame: Frame,
    surface: Surface,
    terrain: Vec<BodyId>,
}

impl Builder {
    fn add(&mut self, key: SceneKey) {
        match key {
            SceneKey::Plane { deg10, bits, .. } => {
                let tan = degrees(deg10).to_radians().tan();
                self.field([0.0; 3], SAMPLES, 1.0, bits.into(), move |x, _| tan * x);
            }
            SceneKey::Ramp { deg10, .. } => {
                let tan = degrees(deg10).to_radians().tan();
                self.field([0.0; 3], SAMPLES, 1.0, 16, move |x, _| {
                    tan * (x.clamp(-6.0, 6.0) + 6.0)
                });
            }
            SceneKey::Ridge { deg10, .. } => {
                let tan = degrees(deg10).to_radians().tan();
                self.field([0.0; 3], SAMPLES, 1.0, 16, move |x, _| {
                    tan * (6.0 - x.abs()).max(0.0)
                });
            }
            SceneKey::Step { height_mm, rounded } => {
                self.flat();
                let h = f64::from(height_mm) / 1000.0;
                let bottom = -1.0;
                self.block(
                    [2.0, 0.5 * (h + bottom), 0.0],
                    [2.0, 0.5 * (h - bottom), 6.0],
                    0.0,
                    radius(rounded),
                );
            }
            SceneKey::Ledge { drop_mm, rounded } => {
                self.flat();
                let h = f64::from(drop_mm) / 1000.0;
                let bottom = -1.0;
                self.block(
                    [-2.0, 0.5 * (h + bottom), 0.0],
                    [2.0, 0.5 * (h - bottom), 6.0],
                    0.0,
                    radius(rounded),
                );
            }
            SceneKey::Wall { thick } => {
                self.flat();
                let half_x = if thick { 1.0 } else { 0.1 };
                let face = f64::from(RADIUS) - 0.05;
                self.block([face + half_x, 1.0, 0.0], [half_x, 1.0, 2.0], 0.0, 0.0);
            }
            SceneKey::BoxRamp { deg10, .. } => {
                let angle = degrees(deg10).to_radians();
                let thickness = 0.5;
                // The top face's middle sits at the local origin.
                let centre = [thickness * angle.sin(), -thickness * angle.cos(), 0.0];
                self.block(centre, [12.0, thickness, 9.0], angle as f32, 0.0);
                // Ground far below, so nothing falls forever.
                self.field([0.0, -20.0, 0.0], SAMPLES, 1.0, 16, |_, _| 0.0);
            }
            SceneKey::Flat => self.flat(),
            SceneKey::Seam { profile, split } => match split {
                Split::Continuous => {
                    self.field([0.0; 3], 65, 1.0, 16, move |x, _| profile.height(x));
                }
                Split::Pair | Split::Raised => {
                    let lift = if split == Split::Raised { 0.03 } else { 0.0 };
                    self.field([-16.0, 0.0, 0.0], SAMPLES, 1.0, 16, move |x, _| {
                        profile.height(x)
                    });
                    self.field([16.0, 0.0, 0.0], SAMPLES, 1.0, 16, move |x, _| {
                        profile.height(x) + lift
                    });
                }
            },
            SceneKey::RadialSpiral {
                deg10,
                spacing_mm,
                bits,
            } => {
                let tan = degrees(deg10).to_radians().tan();
                let spacing = f64::from(spacing_mm) / 1000.0;
                self.field([0.0; 3], SAMPLES, spacing, bits.into(), move |x, _| {
                    spiral_height(tan, x)
                });
            }
            SceneKey::RadialCrest { deg10 } => {
                let tan = degrees(deg10).to_radians().tan();
                self.field([0.0; 3], SAMPLES, 0.5, 16, move |x, _| {
                    if x >= 0.0 {
                        (R * R - x * x).sqrt() - R
                    } else {
                        spiral_height(tan, x)
                    }
                });
            }
            SceneKey::RadialStep { height_mm } => {
                self.planet_chunk(0.0);
                let face = 1.0 + f64::from(RADIUS);
                let top = flat_ground(face, 0.0) + f64::from(height_mm) / 1000.0;
                self.block([face + 2.0, top - 1.5, 0.0], [2.0, 1.5, 2.0], 0.0, 0.0);
            }
            SceneKey::RadialSeam => {
                self.planet_chunk(0.0);
                self.planet_chunk(2.0 * (16.0 / R).asin());
            }
            SceneKey::RadialWall => {
                self.planet_chunk(0.0);
                self.block([0.45, 1.0, 0.0], [0.1, 1.0, 2.0], 0.0, 0.0);
            }
        }
    }

    fn flat(&mut self) {
        self.field([0.0; 3], SAMPLES, 1.0, 16, |_, _| 0.0);
    }

    /// A static terrain heightfield of `samples` per side, `spacing` apart, centred on local
    /// point `centre`, with heights `height(x, z)` of scene-local x and z.
    fn field(
        &mut self,
        centre: V3,
        samples: usize,
        spacing: f64,
        bits: u32,
        height: impl Fn(f64, f64) -> f64,
    ) {
        let half = (samples - 1) as f64 / 2.0 * spacing;
        let mut heights = Vec::with_capacity(samples * samples);
        for iz in 0..samples {
            for ix in 0..samples {
                let (x, z) = (ix as f64 * spacing - half, iz as f64 * spacing - half);
                heights.push(height(centre[0] + x, centre[2] + z));
            }
        }
        let bits = match FIELD_BITS_OVERRIDE.load(Ordering::Relaxed) {
            0 => bits,
            forced => forced,
        };
        let offset = [-half, 0.0, -half];
        let settings = HeightFieldSettings::default()
            .offset(vec3(offset))
            .scale(Vec3::new(spacing as f32, 1.0, spacing as f32))
            .bits_per_sample(bits);
        let samples_f32: Vec<f32> = heights.iter().map(|&h| h as f32).collect();
        let shape = Shape::new_height_field(samples as u32, &samples_f32, &settings).unwrap();
        let frame = Frame {
            origin: self.frame.to_world_point(centre),
            rotation: self.frame.rotation,
        };
        let body = self
            .world
            .create_body(
                &shape,
                &BodySettings::new_static()
                    .position(rvec3(frame.origin))
                    .rotation(frame.rotation)
                    .object_layer(self.layers.terrain),
            )
            .unwrap();
        self.terrain.push(body);
        let intended = heights.iter().map(|&h| f64::from(h as f32)).collect();
        self.surface.fields.push(Field::decode(
            &shape, frame, samples, spacing, offset, intended, bits,
        ));
    }

    /// The flat terrain of the walker fixtures' chunk at `angle` from the anchor.
    fn planet_chunk(&mut self, angle: f64) {
        let shape = flat_terrain();
        let (position, rotation) = chunk_pose(angle);
        let body = self
            .world
            .create_body(
                &shape,
                &BodySettings::new_static()
                    .position(position)
                    .rotation(rotation)
                    .object_layer(self.layers.terrain),
            )
            .unwrap();
        self.terrain.push(body);
        let intended = (0..SAMPLES * SAMPLES)
            .map(|i| {
                let (x, z) = ((i % SAMPLES) as f64 - 16.0, (i / SAMPLES) as f64 - 16.0);
                f64::from(((R * R - x * x - z * z).sqrt() - R) as f32)
            })
            .collect();
        let frame = Frame {
            origin: crate::common::math::v3(position),
            rotation,
        };
        self.surface.fields.push(Field::decode(
            &shape,
            frame,
            SAMPLES,
            1.0,
            [-16.0, 0.0, -16.0],
            intended,
            16,
        ));
    }

    /// A static chunk compound holding one box of half extents `half` at local `centre`,
    /// turned by `tilt_z` radians about local z, with the structure group.
    fn block(&mut self, centre: V3, half: V3, tilt_z: f32, convex_radius: f64) {
        let shape = Shape::new_box_with_convex_radius(vec3(half), convex_radius as f32).unwrap();
        let tilt = quat_about(Vec3::new(0.0, 0.0, 1.0), tilt_z);
        let chunk = Shape::new_compound(&[CompoundChild {
            shape: &shape,
            position: vec3(centre),
            rotation: tilt,
            user_data: Groups::STRUCTURE,
        }])
        .unwrap();
        self.world
            .create_body(
                &chunk,
                &BodySettings::new_static()
                    .position(rvec3(self.frame.origin))
                    .rotation(self.frame.rotation)
                    .object_layer(self.layers.chunk),
            )
            .unwrap();
        self.surface.blocks.push(Block {
            frame: Frame {
                origin: self.frame.to_world_point(centre),
                rotation: self.frame.to_world_rotation(tilt),
            },
            half,
            convex_radius,
        });
    }

    /// The game's kinematic actor capsule, parked far below the scene.
    fn actor(&mut self) -> BodyId {
        add_actor(&mut self.world, &self.layers)
    }
}

/// Adds the game's kinematic actor capsule to `world`, parked far below the scenes.
pub fn add_actor(world: &mut PhysicsWorld, layers: &Layers) -> BodyId {
    let capsule = capsule();
    let shape = Shape::new_compound(&[CompoundChild {
        shape: &capsule,
        position: Vec3::new(0.0, CENTRE_UP, 0.0),
        rotation: Quat::IDENTITY,
        user_data: Groups::ACTOR,
    }])
    .unwrap();
    world
        .create_body(
            &shape,
            &BodySettings::new_kinematic()
                .position(RVec3::new(0.0, -500.0, 0.0))
                .object_layer(layers.actor),
        )
        .unwrap()
}

fn radius(rounded: bool) -> f64 {
    if rounded {
        0.05
    } else {
        0.0
    }
}

/// Height above the anchor plane of the log spiral `r = R exp(tan * phi)` (in the x-y plane
/// around the planet's centre) at chart x: `phi` solved by bisection from `r sin phi = x`.
pub fn spiral_height(tan: f64, x: f64) -> f64 {
    let at = |phi: f64| R * (tan * phi).exp() * phi.sin();
    let (mut low, mut high) = (-0.5, 0.5);
    for _ in 0..200 {
        let mid = 0.5 * (low + high);
        if at(mid) < x {
            low = mid;
        } else {
            high = mid;
        }
    }
    let phi = 0.5 * (low + high);
    R * (tan * phi).exp() * phi.cos() - R
}

/// Ground height of the anchor chunk's flat terrain at `x`, `z`.
pub fn flat_ground(x: f64, z: f64) -> f64 {
    (R * R - x * x - z * z).sqrt() - R
}

/// The body origin resting on the anchor chunk's flat terrain at `(x, z)`.
pub fn resting_on_planet(x: f64, z: f64) -> V3 {
    let ground = [x, flat_ground(x, z), z];
    add(ground, scale(up_at(ground), f64::from(REST_HEIGHT)))
}
