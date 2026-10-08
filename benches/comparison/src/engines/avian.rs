//! Avian: a headless Bevy app with Avian's physics plugins and one entity per body and joint.
//! A step is one `App::update`, so its time includes Bevy's schedule, time and transform
//! propagation, and Avian's systems outside its physics schedule.

use std::collections::HashMap;
use std::time::Duration;

use avian3d::diagnostics::{PhysicsTotalDiagnostics, PhysicsTotalDiagnosticsPlugin};
use avian3d::prelude::*;
use bevy::app::PluginsState;
use bevy::ecs::system::SystemState;
use bevy::prelude::*;
use bevy::tasks::{ComputeTaskPool, TaskPoolBuilder};
use bevy::time::TimeUpdateStrategy;

use crate::engine::{
    BodyState, Config, Engine, Profile, DT, GRAVITY, MATCHED_FRICTION, MATCHED_SPECULATIVE_DISTANCE,
};
use crate::scene::{JointKind, JointSpec, Motion, SceneSpec, Shape as SceneShape, V3};

/// Avian's substeps in the matched profile, as in Avian's own benchmarks.
pub const MATCHED_SUBSTEPS: u32 = 4;

/// The time step as a `Duration`, the same for Bevy's fixed clock and its manual updates.
fn step_duration() -> Duration {
    Duration::from_secs_f64(f64::from(DT))
}

pub struct Avian {
    app: App,
    entities: Vec<Entity>,
    index_of: HashMap<Entity, usize>,
}

/// Sizes Bevy's compute pool and the global Rayon pool, before any app exists. Only the first
/// call in a process has an effect; a timed or validation run is a process of its own.
fn init_pools(threads: u32) -> Result<(), String> {
    if !cfg!(feature = "parallel") && threads != 1 {
        return Err("Avian without the parallel feature runs on one thread".to_owned());
    }
    ComputeTaskPool::get_or_init(|| TaskPoolBuilder::new().num_threads(threads as usize).build());
    #[cfg(feature = "parallel")]
    let _ = rayon::ThreadPoolBuilder::new()
        .num_threads(threads as usize)
        .build_global();
    Ok(())
}

fn collider(shape: SceneShape) -> Collider {
    match shape {
        SceneShape::Ball { radius } => Collider::sphere(radius),
        // Avian takes full lengths.
        SceneShape::Cuboid {
            half_extents: [x, y, z],
        } => Collider::cuboid(2.0 * x, 2.0 * y, 2.0 * z),
        SceneShape::CapsuleY {
            half_height,
            radius,
        } => Collider::capsule(radius, 2.0 * half_height),
    }
}

fn vector(v: V3) -> Vec3 {
    Vec3::from_array(v)
}

fn spawn_joint(world: &mut World, entities: &[Entity], joint: &JointSpec) {
    let (e1, e2) = (entities[joint.body1], entities[joint.body2]);
    let (a1, a2) = (vector(joint.local_anchor1), vector(joint.local_anchor2));
    match joint.kind {
        JointKind::Spherical => {
            world.spawn(
                SphericalJoint::new(e1, e2)
                    .with_local_anchor1(a1)
                    .with_local_anchor2(a2),
            );
        }
        JointKind::Fixed => {
            world.spawn(
                FixedJoint::new(e1, e2)
                    .with_local_anchor1(a1)
                    .with_local_anchor2(a2),
            );
        }
        JointKind::Revolute { axis } => {
            world.spawn(
                RevoluteJoint::new(e1, e2)
                    .with_hinge_axis(vector(axis))
                    .with_local_anchor1(a1)
                    .with_local_anchor2(a2),
            );
        }
        JointKind::Prismatic { axis, limits } => {
            world.spawn(
                PrismaticJoint::new(e1, e2)
                    .with_slider_axis(vector(axis))
                    .with_local_anchor1(a1)
                    .with_local_anchor2(a2)
                    .with_limits(limits[0], limits[1]),
            );
        }
    }
}

impl Avian {
    /// Builds the app; with `diagnostics`, Avian's total step timer is added for split runs.
    pub fn build_app(spec: &SceneSpec, config: &Config, diagnostics: bool) -> Result<Self, String> {
        init_pools(config.threads)?;
        let mut app = App::new();
        let physics = if config.joints {
            PhysicsPlugins::default().build()
        } else {
            PhysicsPlugins::default()
                .build()
                .disable::<XpbdSolverPlugin>()
        };
        app.add_plugins((MinimalPlugins, TransformPlugin, physics));
        if diagnostics {
            app.add_plugins(PhysicsTotalDiagnosticsPlugin);
        }
        app.insert_resource(Time::<Fixed>::from_duration(step_duration()))
            .insert_resource(TimeUpdateStrategy::ManualDuration(step_duration()))
            .insert_resource(Gravity(vector(GRAVITY)));
        let substeps = config.iterations.unwrap_or(match config.profile {
            Profile::Matched => MATCHED_SUBSTEPS,
            Profile::Defaults => SubstepCount::default().0,
        });
        app.insert_resource(SubstepCount(substeps));
        if config.profile == Profile::Matched {
            app.insert_resource(NarrowPhaseConfig {
                default_speculative_margin: MATCHED_SPECULATIVE_DISTANCE,
                ..default()
            });
        }

        let world = app.world_mut();
        let mut entities = Vec::with_capacity(spec.bodies.len());
        for body in &spec.bodies {
            let [x, y, z] = body.position;
            let rigid_body = match body.motion {
                Motion::Fixed => RigidBody::Static,
                Motion::Dynamic => RigidBody::Dynamic,
            };
            let mut entity = world.spawn((
                rigid_body,
                collider(body.shape),
                ColliderDensity(body.density),
                Transform::from_xyz(x, y, z),
            ));
            if config.profile == Profile::Matched {
                entity.insert((
                    Friction::new(MATCHED_FRICTION),
                    Restitution::new(0.0),
                    LinearDamping(0.0),
                    AngularDamping(0.0),
                    SleepingDisabled,
                ));
            }
            entities.push(entity.id());
        }
        for joint in &spec.joints {
            spawn_joint(world, &entities, joint);
        }

        while app.plugins_state() != PluginsState::Ready {
            bevy::tasks::tick_global_task_pools_on_main_thread();
        }
        app.finish();
        app.cleanup();
        // One update with the physics clock paused sets up colliders, mass and the spatial
        // structures without simulating; tick 1 is then the first simulated step.
        app.world_mut().resource_mut::<Time<Physics>>().pause();
        app.update();
        app.world_mut().resource_mut::<Time<Physics>>().unpause();
        if app.world().resource::<Time<Physics>>().elapsed() != Duration::ZERO {
            return Err("the paused setup update advanced the physics clock".to_owned());
        }

        let index_of = entities.iter().enumerate().map(|(i, &e)| (e, i)).collect();
        Ok(Self {
            app,
            entities,
            index_of,
        })
    }

    /// Avian's own time for the last physics step, from its total diagnostics (split runs only).
    pub fn last_physics_step_time(&self) -> Option<Duration> {
        self.app
            .world()
            .get_resource::<PhysicsTotalDiagnostics>()
            .map(|d| d.step_time)
    }
}

impl Engine for Avian {
    fn build(spec: &SceneSpec, config: &Config) -> Result<Self, String> {
        Self::build_app(spec, config, false)
    }

    fn step(&mut self) -> Result<(), String> {
        let before = self.app.world().resource::<Time<Physics>>().elapsed();
        self.app.update();
        let after = self.app.world().resource::<Time<Physics>>().elapsed();
        if after - before == step_duration() {
            Ok(())
        } else {
            Err(format!(
                "the physics clock advanced {:?} instead of one step",
                after - before
            ))
        }
    }

    fn read_state(&mut self, out: &mut Vec<BodyState>) {
        out.clear();
        let world = self.app.world();
        for &entity in &self.entities {
            let position = world
                .get::<Position>(entity)
                .expect("a body has a position");
            let rotation = world
                .get::<Rotation>(entity)
                .expect("a body has a rotation");
            let linear = world
                .get::<LinearVelocity>(entity)
                .map_or(Vec3::ZERO, |v| v.0);
            let angular = world
                .get::<AngularVelocity>(entity)
                .map_or(Vec3::ZERO, |v| v.0);
            out.push(BodyState {
                position: position.0.to_array(),
                rotation: rotation.0.to_array(),
                linear_velocity: linear.to_array(),
                angular_velocity: angular.to_array(),
                sleeping: world.get::<Sleeping>(entity).is_some(),
            });
        }
    }

    fn body_mass(&mut self, index: usize) -> Option<f32> {
        let world = self.app.world();
        let entity = self.entities[index];
        if world.get::<RigidBody>(entity) != Some(&RigidBody::Dynamic) {
            return None;
        }
        world.get::<ComputedMass>(entity).map(|m| m.value())
    }

    fn body_inertia(&mut self, index: usize) -> Option<V3> {
        let inertia = self
            .app
            .world()
            .get::<ComputedAngularInertia>(self.entities[index])?;
        Some(
            inertia
                .principal_angular_inertia_with_local_frame()
                .0
                .to_array(),
        )
    }

    fn cast_ray(&mut self, origin: V3, direction: V3, max_distance: f32) -> Option<(usize, f32)> {
        let world = self.app.world_mut();
        let mut state = SystemState::<SpatialQuery>::new(world);
        let query = state.get(world).ok()?;
        let hit = query.cast_ray(
            vector(origin),
            Dir3::new(vector(direction)).ok()?,
            max_distance,
            true,
            &SpatialQueryFilter::default(),
        )?;
        Some((*self.index_of.get(&hit.entity)?, hit.distance))
    }

    fn awake_bodies(&mut self) -> usize {
        let world = self.app.world();
        self.entities
            .iter()
            .filter(|&&e| {
                world.get::<RigidBody>(e) == Some(&RigidBody::Dynamic)
                    && world.get::<Sleeping>(e).is_none()
            })
            .count()
    }
}

/// The creation order: body `i` of the scene is the `i`th entity spawned.
pub fn entities_follow_scene_order(engine: &Avian) -> bool {
    engine
        .entities
        .windows(2)
        .all(|pair| pair[0].index() < pair[1].index())
}
