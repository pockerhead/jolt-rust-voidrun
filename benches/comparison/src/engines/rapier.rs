//! Rapier: a `rapier3d::PhysicsWorld` with the scene's bodies, colliders and impulse joints.

use rapier3d::prelude::*;

use crate::engine::{BodyState, Config, Engine, Profile, DT, GRAVITY, MATCHED_FRICTION};
use crate::scene::{BodySpec, JointKind, JointSpec, Motion, SceneSpec, Shape, V3};

/// Rapier's default solver iterations (`IntegrationParameters::num_solver_iterations`).
pub const DEFAULT_ITERATIONS: u32 = 4;

pub struct Rapier {
    pub world: PhysicsWorld,
    handles: Vec<RigidBodyHandle>,
    /// Whether the broad phase has seen the bodies, which happens in a step.
    detected: bool,
}

fn vector(v: V3) -> Vector {
    Vector::new(v[0], v[1], v[2])
}

/// The rigid body and collider of `body`.
pub fn body_builders(body: &BodySpec, profile: Profile) -> (RigidBodyBuilder, ColliderBuilder) {
    let rigid_body = match body.motion {
        Motion::Fixed => RigidBodyBuilder::fixed(),
        Motion::Dynamic => RigidBodyBuilder::dynamic(),
    }
    .translation(vector(body.position));
    let collider = match body.shape {
        Shape::Ball { radius } => ColliderBuilder::ball(radius),
        Shape::Cuboid {
            half_extents: [x, y, z],
        } => ColliderBuilder::cuboid(x, y, z),
        Shape::CapsuleY {
            half_height,
            radius,
        } => ColliderBuilder::capsule_y(half_height, radius),
    }
    .density(body.density);
    match profile {
        Profile::Matched => (
            rigid_body
                .can_sleep(false)
                .linear_damping(0.0)
                .angular_damping(0.0),
            collider.friction(MATCHED_FRICTION).restitution(0.0),
        ),
        Profile::Defaults => (rigid_body, collider),
    }
}

/// The impulse joint of `joint`, anchored as the scene says.
pub fn joint_builder(joint: &JointSpec) -> GenericJoint {
    let (a1, a2) = (vector(joint.local_anchor1), vector(joint.local_anchor2));
    match joint.kind {
        JointKind::Spherical => SphericalJointBuilder::new()
            .local_anchor1(a1)
            .local_anchor2(a2)
            .into(),
        JointKind::Fixed => FixedJointBuilder::new()
            .local_anchor1(a1)
            .local_anchor2(a2)
            .into(),
        JointKind::Revolute { axis } => RevoluteJointBuilder::new(vector(axis))
            .local_anchor1(a1)
            .local_anchor2(a2)
            .into(),
        JointKind::Prismatic { axis, limits } => PrismaticJointBuilder::new(vector(axis))
            .local_anchor1(a1)
            .local_anchor2(a2)
            .limits(limits)
            .into(),
    }
}

/// Gives the world a pool of `threads` and makes the global Rayon pool the same size.
#[cfg(feature = "parallel")]
fn configure_threads(world: &mut PhysicsWorld, threads: u32) -> Result<(), String> {
    world
        .configure_thread_pool(threads as usize)
        .map_err(|e| e.to_string())?;
    // Only the first call in a process sets the global pool; later worlds of the same process
    // (tests) keep it.
    let _ = rayon::ThreadPoolBuilder::new()
        .num_threads(threads as usize)
        .build_global();
    Ok(())
}

#[cfg(not(feature = "parallel"))]
fn configure_threads(_: &mut PhysicsWorld, threads: u32) -> Result<(), String> {
    if threads == 1 {
        Ok(())
    } else {
        Err("Rapier without the parallel feature runs on one thread".to_owned())
    }
}

impl Engine for Rapier {
    fn build(spec: &SceneSpec, config: &Config) -> Result<Self, String> {
        let mut world = PhysicsWorld::new();
        world.gravity = vector(GRAVITY);
        let parameters = &mut world.integration_parameters;
        parameters.dt = DT;
        parameters.num_solver_iterations = config.iterations.unwrap_or(DEFAULT_ITERATIONS) as usize;
        if config.profile == Profile::Matched {
            parameters.max_ccd_substeps = 0;
        }
        configure_threads(&mut world, config.threads)?;
        let mut handles = Vec::with_capacity(spec.bodies.len());
        for body in &spec.bodies {
            let (rigid_body, collider) = body_builders(body, config.profile);
            handles.push(world.insert(rigid_body, collider).0);
        }
        if config.joints {
            for joint in &spec.joints {
                world.insert_impulse_joint(
                    handles[joint.body1],
                    handles[joint.body2],
                    joint_builder(joint),
                );
            }
        }
        Ok(Self {
            world,
            handles,
            detected: false,
        })
    }

    fn step(&mut self) -> Result<(), String> {
        self.world.step();
        self.detected = true;
        let quarantine = self.world.quarantine();
        if quarantine.is_empty() {
            Ok(())
        } else {
            Err(format!(
                "Rapier disabled {} non-finite bodies",
                quarantine.bodies().len()
            ))
        }
    }

    fn read_state(&mut self, out: &mut Vec<BodyState>) {
        out.clear();
        out.extend(self.handles.iter().map(|&handle| {
            let body = &self.world.bodies[handle];
            BodyState {
                position: body.translation().to_array(),
                rotation: body.rotation().to_array(),
                linear_velocity: body.linvel().to_array(),
                angular_velocity: body.angvel().to_array(),
                sleeping: body.is_sleeping(),
            }
        }));
    }

    fn body_mass(&mut self, index: usize) -> Option<f32> {
        let body = &self.world.bodies[self.handles[index]];
        body.is_dynamic().then(|| body.mass())
    }

    fn body_inertia(&mut self, index: usize) -> Option<V3> {
        let body = &self.world.bodies[self.handles[index]];
        Some(
            body.mass_properties()
                .local_mprops
                .principal_inertia()
                .to_array(),
        )
    }

    /// Rapier's queries use its broad phase, which a step fills; before the first step this
    /// runs collision detection once so the bodies are found.
    fn cast_ray(&mut self, origin: V3, direction: V3, max_distance: f32) -> Option<(usize, f32)> {
        if !self.detected {
            self.world.detect_collisions(&(), &());
            self.detected = true;
        }
        let ray = Ray::new(vector(origin), vector(direction));
        let (collider, distance) =
            self.world
                .cast_ray(&ray, max_distance, true, QueryFilter::default())?;
        let parent = self.world.colliders[collider].parent()?;
        let index = parent.into_raw_parts().0 as usize;
        (self.handles.get(index) == Some(&parent)).then_some((index, distance))
    }

    fn awake_bodies(&mut self) -> usize {
        self.handles
            .iter()
            .map(|&h| &self.world.bodies[h])
            .filter(|body| body.is_dynamic() && !body.is_sleeping())
            .count()
    }
}

/// The creation order: body `i` of the scene has Rapier handle index `i`.
pub fn handles_follow_scene_order(engine: &Rapier) -> bool {
    engine
        .handles
        .iter()
        .enumerate()
        .all(|(i, h)| h.into_raw_parts().0 as usize == i)
}
