//! Engine-neutral scene data: bodies with a shape, a position, a motion type and a density, and
//! joints between them. Every engine adapter builds its world from a [`SceneSpec`], so all
//! engines simulate the same scene.

use crate::scenes;

/// A vector in metres (or a direction), `[x, y, z]`, y up.
pub type V3 = [f32; 3];

/// A collision shape centred on its body, in the body's frame (all scenes have identity
/// rotations).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Shape {
    /// A sphere.
    Ball { radius: f32 },
    /// A box with sharp edges.
    Cuboid { half_extents: V3 },
    /// A capsule along y: a cylinder of `2 * half_height` capped by two half spheres.
    CapsuleY { half_height: f32, radius: f32 },
}

impl Shape {
    /// Volume in cubic metres.
    pub fn volume(&self) -> f64 {
        use std::f64::consts::PI;
        match *self {
            Self::Ball { radius } => 4.0 / 3.0 * PI * f64::from(radius).powi(3),
            Self::Cuboid { half_extents } => {
                half_extents.iter().map(|&h| 2.0 * f64::from(h)).product()
            }
            Self::CapsuleY {
                half_height,
                radius,
            } => {
                let (h, r) = (f64::from(half_height), f64::from(radius));
                PI * r * r * 2.0 * h + 4.0 / 3.0 * PI * r.powi(3)
            }
        }
    }

    /// Distance from the centre to the surface along each world axis, `[x, y, z]`.
    pub fn half_size(&self) -> V3 {
        match *self {
            Self::Ball { radius } => [radius; 3],
            Self::Cuboid { half_extents } => half_extents,
            Self::CapsuleY {
                half_height,
                radius,
            } => [radius, half_height + radius, radius],
        }
    }

    /// Principal moments of inertia about the centre for `mass`, about x, y and z.
    pub fn principal_inertia(&self, mass: f64) -> [f64; 3] {
        match *self {
            Self::Ball { radius } => [2.0 / 5.0 * mass * f64::from(radius).powi(2); 3],
            Self::Cuboid { half_extents } => {
                let [x, y, z] = half_extents.map(|h| (2.0 * f64::from(h)).powi(2));
                [y + z, x + z, x + y].map(|s| mass * s / 12.0)
            }
            Self::CapsuleY {
                half_height,
                radius,
            } => capsule_inertia(f64::from(half_height), f64::from(radius), mass),
        }
    }
}

/// The inertia of a y capsule of uniform density, from its cylinder and two half spheres.
fn capsule_inertia(h: f64, r: f64, mass: f64) -> [f64; 3] {
    use std::f64::consts::PI;
    let cylinder_volume = PI * r * r * 2.0 * h;
    let sphere_volume = 4.0 / 3.0 * PI * r.powi(3);
    let density = mass / (cylinder_volume + sphere_volume);
    let (mc, ms) = (density * cylinder_volume, density * sphere_volume);
    let axial = mc * r * r / 2.0 + ms * 2.0 / 5.0 * r * r;
    // Each half sphere's centre of mass sits 3r/8 from its flat face, which is h from the centre.
    let cap = ms * (2.0 / 5.0 * r * r + h * h + 3.0 / 4.0 * h * r);
    let lateral = mc * (r * r / 4.0 + (2.0 * h).powi(2) / 12.0) + cap;
    [lateral, axial, lateral]
}

/// Whether a body is fixed in place or simulated.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Motion {
    Fixed,
    Dynamic,
}

/// One body: a shape at `position` with identity rotation, and its density in kg/m³.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BodySpec {
    pub shape: Shape,
    pub position: V3,
    pub motion: Motion,
    pub density: f32,
}

impl BodySpec {
    /// A dynamic body of density 1, Rapier's default.
    pub fn dynamic(shape: Shape, position: V3) -> Self {
        Self {
            shape,
            position,
            motion: Motion::Dynamic,
            density: 1.0,
        }
    }

    /// A fixed body of density 1.
    pub fn fixed(shape: Shape, position: V3) -> Self {
        Self {
            motion: Motion::Fixed,
            ..Self::dynamic(shape, position)
        }
    }

    /// The same body with another density.
    pub fn with_density(self, density: f32) -> Self {
        Self { density, ..self }
    }

    /// Mass in kilograms: density times volume.
    pub fn mass(&self) -> f64 {
        f64::from(self.density) * self.shape.volume()
    }
}

/// What a joint lets body 2 do relative to body 1.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum JointKind {
    /// Rotation about the anchor in every direction (a ball joint).
    Spherical,
    /// No relative motion.
    Fixed,
    /// Rotation about `axis` only (a hinge).
    Revolute { axis: V3 },
    /// Translation along `axis` only, within `limits` metres (a slider).
    Prismatic { axis: V3, limits: [f32; 2] },
}

/// A joint between two bodies, its anchor given in each body's frame. Axes are the same in both
/// frames because every body starts with identity rotation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointSpec {
    pub kind: JointKind,
    pub body1: usize,
    pub body2: usize,
    pub local_anchor1: V3,
    pub local_anchor2: V3,
}

/// A scene: its bodies and joints in creation order.
#[derive(Clone, Debug, PartialEq)]
pub struct SceneSpec {
    pub name: &'static str,
    /// Where the scene comes from.
    pub source: &'static str,
    pub bodies: Vec<BodySpec>,
    pub joints: Vec<JointSpec>,
}

impl SceneSpec {
    /// The world position of `joint`'s anchor on body 1 at creation.
    pub fn anchor(&self, joint: &JointSpec) -> V3 {
        let p = self.bodies[joint.body1].position;
        [0, 1, 2].map(|i| p[i] + joint.local_anchor1[i])
    }

    /// Top of the ground, the fixed body under the scene, if it has one.
    pub fn ground_top(&self) -> Option<f32> {
        let ground = self.bodies.first()?;
        let is_ground = ground.motion == Motion::Fixed
            && matches!(ground.shape, Shape::Cuboid { half_extents } if half_extents[0] >= 50.0);
        is_ground.then(|| ground.position[1] + ground.shape.half_size()[1])
    }

    /// How many bodies are dynamic.
    pub fn dynamic_count(&self) -> usize {
        self.bodies
            .iter()
            .filter(|body| body.motion == Motion::Dynamic)
            .count()
    }
}

/// What kind of quality measures a scene gets.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SceneClass {
    /// Bodies resting on a ground, piled up.
    Stack,
    /// Bodies dropped into a heap on a ground, not stacked: no height or final-speed bound.
    Pile,
    /// Balls on a fixed layer, no ground.
    Balls,
    /// Bodies hanging from joints.
    Joints,
}

/// Every scene the harness knows: Rapier's ten stress scenes, then small fixtures.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Scene {
    Balls,
    Boxes,
    Capsules,
    Pyramid,
    ManyPyramids,
    Keva,
    JointBall,
    JointFixed,
    JointPrismatic,
    JointRevolute,
    /// Five bodies on a fixed anchor joined by ball joints.
    FixtureSpherical,
    /// Five bodies on a fixed anchor joined by fixed joints.
    FixtureFixed,
    /// Five bodies on a fixed anchor joined by hinges.
    FixtureRevolute,
    /// Five bodies on a fixed anchor joined by sliders with limits.
    FixturePrismatic,
    /// Five boxes stacked on a ground.
    FixtureStack,
}

impl Scene {
    /// Rapier's stress scenes, in the order the reports list them.
    pub const RAPIER: [Scene; 10] = [
        Scene::Balls,
        Scene::Boxes,
        Scene::Capsules,
        Scene::Pyramid,
        Scene::ManyPyramids,
        Scene::Keva,
        Scene::JointBall,
        Scene::JointFixed,
        Scene::JointPrismatic,
        Scene::JointRevolute,
    ];

    /// The small fixtures for tests and CI.
    pub const FIXTURES: [Scene; 5] = [
        Scene::FixtureSpherical,
        Scene::FixtureFixed,
        Scene::FixtureRevolute,
        Scene::FixturePrismatic,
        Scene::FixtureStack,
    ];

    /// The name used on the command line and in result files.
    pub fn name(self) -> &'static str {
        match self {
            Self::Balls => "balls",
            Self::Boxes => "boxes",
            Self::Capsules => "capsules",
            Self::Pyramid => "pyramid",
            Self::ManyPyramids => "many_pyramids",
            Self::Keva => "keva",
            Self::JointBall => "joint_ball",
            Self::JointFixed => "joint_fixed",
            Self::JointPrismatic => "joint_prismatic",
            Self::JointRevolute => "joint_revolute",
            Self::FixtureSpherical => "fixture_spherical",
            Self::FixtureFixed => "fixture_fixed",
            Self::FixtureRevolute => "fixture_revolute",
            Self::FixturePrismatic => "fixture_prismatic",
            Self::FixtureStack => "fixture_stack",
        }
    }

    /// The scene called `name`, if there is one.
    pub fn from_name(name: &str) -> Option<Self> {
        Self::RAPIER
            .into_iter()
            .chain(Self::FIXTURES)
            .find(|scene| scene.name() == name)
    }

    /// Which quality measures apply.
    pub fn class(self) -> SceneClass {
        match self {
            Self::Balls => SceneClass::Balls,
            Self::Capsules => SceneClass::Pile,
            Self::Boxes | Self::Pyramid | Self::ManyPyramids | Self::Keva | Self::FixtureStack => {
                SceneClass::Stack
            }
            _ => SceneClass::Joints,
        }
    }

    /// Builds the scene's bodies and joints.
    pub fn build(self) -> SceneSpec {
        match self {
            Self::Balls => scenes::balls::balls(),
            Self::Boxes => scenes::stacks::boxes(),
            Self::Capsules => scenes::stacks::capsules(),
            Self::Pyramid => scenes::stacks::pyramid(),
            Self::ManyPyramids => scenes::stacks::many_pyramids(),
            Self::Keva => scenes::stacks::keva(),
            Self::JointBall => scenes::joints::joint_ball(),
            Self::JointFixed => scenes::joints::joint_fixed(),
            Self::JointPrismatic => scenes::joints::joint_prismatic(),
            Self::JointRevolute => scenes::joints::joint_revolute(),
            Self::FixtureSpherical => scenes::fixtures::chain(self.name(), JointKind::Spherical),
            Self::FixtureFixed => scenes::fixtures::chain(self.name(), JointKind::Fixed),
            Self::FixtureRevolute => scenes::fixtures::chain(
                self.name(),
                JointKind::Revolute {
                    axis: [0.0, 0.0, 1.0],
                },
            ),
            Self::FixturePrismatic => scenes::fixtures::chain(
                self.name(),
                JointKind::Prismatic {
                    axis: scenes::fixtures::SLIDER_AXIS,
                    limits: scenes::joints::PRISMATIC_LIMITS,
                },
            ),
            Self::FixtureStack => scenes::fixtures::stack(self.name()),
        }
    }
}
