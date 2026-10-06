//! oxijolt: a `PhysicsWorld` with the scene's bodies and constraints.

use std::sync::Arc;

use oxijolt::{
    BodyId, BodySettings, BroadPhaseLayer, CollisionLayers, FixedConstraintSettings,
    HingeConstraintSettings, Job, JobSystem, ObjectLayer, PhysicsWorld, PointConstraintSettings,
    QueryFilter, RVec3, RayCast, Real, Shape as JoltShape, SliderConstraintSettings, Vec3,
    WorldSettings,
};

use crate::engine::{BodyState, Config, Engine, Profile, DT, GRAVITY, MATCHED_FRICTION};
use crate::scene::{BodySpec, JointKind, JointSpec, Motion, Scene, SceneSpec, Shape, V3};

/// Jolt's buffers per scene, from `comparison size-jolt` (results in `jolt_sizes.tsv`):
/// `max_body_pairs` and `max_contact_constraints` both get `contacts`, the smallest power of two
/// from 2^14 at which 600 matched ticks at one thread dropped no contact, doubled once, at most
/// 2^20. The temp allocator gets [`temp_allocator_size`] of that, doubled while 120 ticks ran
/// more than 3 % slower than with twice the size.
pub const SIZES: [(Scene, Buffers); 10] = [
    (Scene::Balls, Buffers::sized(1 << 15)),
    (Scene::Boxes, Buffers::sized(1 << 15)),
    (Scene::Capsules, Buffers::sized(1 << 15)),
    (Scene::Pyramid, Buffers::sized(1 << 19)),
    (
        Scene::ManyPyramids,
        Buffers {
            contacts: 1 << 16,
            temp_allocator: 512 << 20,
        },
    ),
    (
        Scene::Keva,
        Buffers {
            contacts: 1 << 19,
            temp_allocator: 1152 << 20,
        },
    ),
    (Scene::JointBall, Buffers::sized(1 << 15)),
    (Scene::JointFixed, Buffers::sized(1 << 15)),
    (Scene::JointPrismatic, Buffers::sized(1 << 15)),
    (Scene::JointRevolute, Buffers::sized(1 << 15)),
];

/// Buffers of the fixtures and of scenes not in [`SIZES`].
const SMALL_BUFFERS: Buffers = Buffers::sized(1 << 14);

/// Temp allocator bytes for `contact_buffers`: Jolt takes about 1 KiB per contact constraint
/// from it each step (`ContactConstraintManager.cpp:762`), plus 64 MiB for everything else.
pub const fn temp_allocator_size(contact_buffers: u32) -> u32 {
    let bytes = contact_buffers as u64 * 1024 + (64 << 20);
    if bytes > u32::MAX as u64 {
        u32::MAX
    } else {
        bytes as u32
    }
}

/// A caller job system of concurrency 1: every job is left to the stepping thread, so a step
/// runs on that thread alone.
struct SteppingThreadOnly;

impl JobSystem for SteppingThreadOnly {
    fn max_concurrency(&self) -> u32 {
        1
    }

    fn queue_job(&self, job: Job) {
        // Dropping a job inside `queue_job` leaves it to the stepping thread.
        drop(job);
    }
}

/// Buffer sizes of a world.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Buffers {
    pub contacts: u32,
    pub temp_allocator: u32,
}

impl Buffers {
    /// `contacts` with the temp allocator [`temp_allocator_size`] gives it.
    pub const fn sized(contacts: u32) -> Self {
        Self {
            contacts,
            temp_allocator: temp_allocator_size(contacts),
        }
    }

    /// The sizes of the scene called `name`.
    pub fn for_scene(name: &str) -> Self {
        SIZES
            .iter()
            .find(|(scene, _)| scene.name() == name)
            .map_or(SMALL_BUFFERS, |&(_, buffers)| buffers)
    }
}

pub struct Jolt {
    world: PhysicsWorld,
    ids: Vec<BodyId>,
}

impl Jolt {
    /// Builds `spec` with explicit buffer sizes.
    pub fn build_with(spec: &SceneSpec, config: &Config, buffers: Buffers) -> Result<Self, String> {
        let mut table = CollisionLayers::new(2);
        let non_moving = table.add_object_layer(BroadPhaseLayer::new(0));
        let moving = table.add_object_layer(BroadPhaseLayer::new(1));
        table
            .enable_collision(moving, non_moving)
            .enable_collision(moving, moving);
        let body_count = u32::try_from(spec.bodies.len()).map_err(|e| e.to_string())?;
        let mut settings = WorldSettings::default()
            .max_bodies(body_count.max(1))
            .max_body_pairs(buffers.contacts)
            .max_contact_constraints(buffers.contacts)
            .temp_allocator_size(buffers.temp_allocator)
            .gravity(Vec3::from(GRAVITY))
            .layers(table)
            .velocity_steps(config.iterations.unwrap_or(10))
            .position_steps(2);
        settings = if config.threads <= 1 {
            settings.job_system(Arc::new(SteppingThreadOnly))
        } else {
            settings.worker_threads(config.threads - 1)
        };
        let mut world = PhysicsWorld::new(settings).map_err(|e| e.to_string())?;

        let mut shapes: Vec<(Shape, JoltShape)> = Vec::new();
        let mut ids = Vec::with_capacity(spec.bodies.len());
        for body in &spec.bodies {
            let shape_index = shared_shape(&mut shapes, body.shape)?;
            let shape = &shapes[shape_index].1;
            let layer = match body.motion {
                Motion::Fixed => non_moving,
                Motion::Dynamic => moving,
            };
            let settings = body_settings(body, layer, config.profile);
            ids.push(
                world
                    .create_body(shape, &settings)
                    .map_err(|e| e.to_string())?,
            );
        }
        if config.joints {
            for joint in &spec.joints {
                create_joint(&mut world, spec, &ids, joint)?;
            }
        }
        world.optimize_broad_phase();
        Ok(Self { world, ids })
    }
}

/// The index in `shapes` of the Jolt shape for `shape`, created once per distinct shape and
/// shared by its bodies.
fn shared_shape(shapes: &mut Vec<(Shape, JoltShape)>, shape: Shape) -> Result<usize, String> {
    if let Some(index) = shapes.iter().position(|(s, _)| *s == shape) {
        return Ok(index);
    }
    let created = match shape {
        Shape::Ball { radius } => JoltShape::new_sphere(radius),
        // Sharp boxes, like the other engines' cuboids.
        Shape::Cuboid { half_extents } => {
            JoltShape::new_box_with_convex_radius(Vec3::from(half_extents), 0.0)
        }
        Shape::CapsuleY {
            half_height,
            radius,
        } => JoltShape::new_capsule(half_height, radius),
    }
    .map_err(|e| e.to_string())?;
    shapes.push((shape, created));
    Ok(shapes.len() - 1)
}

fn body_settings(body: &BodySpec, layer: ObjectLayer, profile: Profile) -> BodySettings {
    let settings = match body.motion {
        Motion::Fixed => BodySettings::new_static(),
        // Jolt scales its shape's inertia to this mass, so density decides as in the others.
        Motion::Dynamic => BodySettings::new_dynamic().mass(body.mass() as f32),
    }
    .position(rvec3(body.position))
    .object_layer(layer);
    match profile {
        Profile::Matched => settings
            .friction(MATCHED_FRICTION)
            .restitution(0.0)
            .linear_damping(0.0)
            .angular_damping(0.0)
            .allow_sleeping(false),
        Profile::Defaults => settings,
    }
}

/// `v` as a world position.
fn rvec3(v: V3) -> RVec3 {
    RVec3::new(v[0] as Real, v[1] as Real, v[2] as Real)
}

/// A world position in `f32`, which it already is unless oxijolt's `double-precision` is on.
#[allow(clippy::unnecessary_cast)]
fn narrow(p: RVec3) -> V3 {
    [p.x as f32, p.y as f32, p.z as f32]
}

/// A unit vector perpendicular to the unit `axis`.
fn perpendicular(axis: V3) -> Vec3 {
    if axis[2].abs() < 0.9 {
        Vec3::new(0.0, 0.0, 1.0)
    } else {
        Vec3::new(1.0, 0.0, 0.0)
    }
}

fn create_joint(
    world: &mut PhysicsWorld,
    spec: &SceneSpec,
    ids: &[BodyId],
    joint: &JointSpec,
) -> Result<(), String> {
    let point = rvec3(spec.anchor(joint));
    let (a, b) = (ids[joint.body1], ids[joint.body2]);
    let created = match joint.kind {
        JointKind::Spherical => world
            .create_constraint(a, b, &PointConstraintSettings::new(point))
            .map(drop),
        JointKind::Fixed => {
            let settings = FixedConstraintSettings::new(
                point,
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            );
            world.create_constraint(a, b, &settings).map(drop)
        }
        JointKind::Revolute { axis } => {
            let settings =
                HingeConstraintSettings::new(point, Vec3::from(axis), perpendicular(axis));
            world.create_constraint(a, b, &settings).map(drop)
        }
        JointKind::Prismatic { axis, limits } => {
            let settings =
                SliderConstraintSettings::new(point, Vec3::from(axis), perpendicular(axis))
                    .limits(limits[0], limits[1]);
            world.create_constraint(a, b, &settings).map(drop)
        }
    };
    created.map_err(|e| e.to_string())
}

impl Engine for Jolt {
    fn build(spec: &SceneSpec, config: &Config) -> Result<Self, String> {
        Self::build_with(spec, config, Buffers::for_scene(spec.name))
    }

    fn step(&mut self) -> Result<(), String> {
        let report = self.world.step(DT).map_err(|e| e.to_string())?;
        if report.is_complete() {
            Ok(())
        } else {
            Err(format!("the step dropped contacts: {report:?}"))
        }
    }

    fn read_state(&mut self, out: &mut Vec<BodyState>) {
        out.clear();
        for &id in &self.ids {
            let body = self.world.body(id).expect("a scene body");
            let q = body.rotation();
            out.push(BodyState {
                position: narrow(body.position()),
                rotation: [q.x, q.y, q.z, q.w],
                linear_velocity: body.linear_velocity().into(),
                angular_velocity: body.angular_velocity().into(),
                sleeping: body.is_sleeping(),
            });
        }
    }

    fn body_mass(&mut self, index: usize) -> Option<f32> {
        self.world.body(self.ids[index]).ok()?.mass()
    }

    fn body_inertia(&mut self, _index: usize) -> Option<V3> {
        None
    }

    fn cast_ray(&mut self, origin: V3, direction: V3, max_distance: f32) -> Option<(usize, f32)> {
        let ray = RayCast::new(
            rvec3(origin),
            Vec3::from(direction.map(|c| c * max_distance)),
        );
        let hit = self.world.cast_ray(&ray, &QueryFilter::new()).ok()??;
        let index = hit.body.index() as usize;
        (self.ids.get(index) == Some(&hit.body)).then_some((index, hit.distance))
    }

    fn awake_bodies(&mut self) -> usize {
        self.ids
            .iter()
            .filter(|&&id| self.world.body(id).is_ok_and(|b| b.is_active()))
            .count()
    }
}

/// The creation order: body `i` of the scene has Jolt body index `i`.
pub fn ids_follow_scene_order(engine: &Jolt) -> bool {
    engine
        .ids
        .iter()
        .enumerate()
        .all(|(i, id)| id.index() as usize == i)
}
