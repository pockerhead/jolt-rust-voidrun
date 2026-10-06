//! Helpers shared by the integration tests.

// Each test file compiles this module on its own and uses a different subset.
#![allow(dead_code)]

pub mod animation;
pub mod constraint_kinds;
pub mod controls;
pub mod determinism;
pub mod events;
pub mod jobs;
pub mod math;
#[cfg(windows)]
pub mod memory;
pub mod meshes;
pub mod ragdoll;
pub mod soft_body;
pub mod vehicle;
pub mod vehicle_kinds;
pub mod walker;

use oxijolt::*;

pub const DT: f32 = 1.0 / 60.0;

/// A world with the default layers.
pub fn world(gravity: Vec3, worker_threads: u32) -> PhysicsWorld {
    PhysicsWorld::new(jobs::with_threads(
        WorldSettings::default().gravity(gravity),
        worker_threads,
    ))
    .unwrap()
}

/// A static floor whose top face is at y = 0.
pub fn add_floor(world: &mut PhysicsWorld) -> BodyId {
    let shape = Shape::new_box(Vec3::new(100.0, 1.0, 100.0)).unwrap();
    world
        .create_body(
            &shape,
            &BodySettings::new_static().position(RVec3::new(0.0, -1.0, 0.0)),
        )
        .unwrap()
}

/// A dynamic unit cube (half extent 0.5) at `position`.
pub fn add_cube(world: &mut PhysicsWorld, position: RVec3) -> BodyId {
    let shape = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
    world
        .create_body(&shape, &BodySettings::new_dynamic().position(position))
        .unwrap()
}

pub fn step(world: &mut PhysicsWorld, ticks: usize) {
    for _ in 0..ticks {
        assert!(world.step(DT).unwrap().is_complete());
    }
}

/// Positions of eight cubes in two side-by-side stacks of four, dropped from a small gap. Each
/// layer is shifted 0.3 m along +x, which puts the centre of mass of the upper layers past the
/// edge of the bottom cube, so both stacks topple, the left one into the right one.
pub fn stacks_scene() -> Vec<RVec3> {
    let mut cubes = Vec::new();
    for column in 0..2 {
        for layer in 0..4 {
            let x = column as Real + 0.3 * layer as Real;
            let y = 0.5 + 1.05 * layer as Real;
            cubes.push(RVec3::new(x, y, 0.0));
        }
    }
    cubes
}

/// Creates the floor and the cubes of [`stacks_scene`], in that order, and returns their ids.
pub fn build_stacks(world: &mut PhysicsWorld) -> Vec<BodyId> {
    let mut ids = vec![add_floor(world)];
    for position in stacks_scene() {
        ids.push(add_cube(world, position));
    }
    ids
}

/// Appends the state of `id` to `digest`: raw id, position, rotation, linear and angular
/// velocity as little-endian bits, and the sleeping flag.
pub fn record_body(world: &PhysicsWorld, id: BodyId, digest: &mut Vec<u8>) {
    let body = world.body(id).unwrap();
    digest.extend_from_slice(&id.to_raw().to_le_bytes());
    let position: [Real; 3] = body.position().into();
    for value in position {
        digest.extend_from_slice(&value.to_bits().to_le_bytes());
    }
    let rotation: [f32; 4] = body.rotation().into();
    let linear: [f32; 3] = body.linear_velocity().into();
    let angular: [f32; 3] = body.angular_velocity().into();
    for value in rotation.into_iter().chain(linear).chain(angular) {
        digest.extend_from_slice(&value.to_bits().to_le_bytes());
    }
    digest.push(u8::from(body.is_sleeping()));
}

/// Steps the world `ticks` times and records every body in `ids` after each tick.
pub fn run_digest(world: &mut PhysicsWorld, ids: &[BodyId], ticks: usize) -> Vec<u8> {
    let mut digest = Vec::new();
    for _ in 0..ticks {
        assert!(world.step(DT).unwrap().is_complete());
        for &id in ids {
            record_body(world, id, &mut digest);
        }
    }
    digest
}

pub fn length(v: Vec3) -> f32 {
    (v.x * v.x + v.y * v.y + v.z * v.z).sqrt()
}

/// Largest linear speed, m/s, of a body that counts as calm under the game's item rest rule.
pub const CALM_SPEED: f32 = 0.05;
/// Largest angular speed, rad/s, of a body that counts as calm under the game's item rest rule.
pub const CALM_ANGULAR_SPEED: f32 = 0.1;

/// Whether `body` is calm under the game's item rest rule: both its linear and its angular
/// speed are below their limits.
pub fn is_calm(body: &BodyRef<'_>) -> bool {
    length(body.linear_velocity()) < CALM_SPEED
        && length(body.angular_velocity()) < CALM_ANGULAR_SPEED
}

/// The rotation by `angle` radians about the unit vector `axis`.
pub fn quat_about(axis: Vec3, angle: f32) -> Quat {
    let (sin, cos) = (angle / 2.0).sin_cos();
    Quat::from_xyzw(axis.x * sin, axis.y * sin, axis.z * sin, cos)
}

/// The game's collision groups, stored as compound child user data.
pub struct Groups;

impl Groups {
    pub const TERRAIN: u32 = 1;
    pub const STRUCTURE: u32 = 2;
    pub const FEATURE: u32 = 3;
    pub const ITEM: u32 = 4;
    pub const ACTOR: u32 = 5;
}

/// A world with one object layer per group, in the order terrain, chunk (structure and feature
/// compounds), feature, item, actor. Queries ignore which layer pairs collide.
pub fn five_layer_world() -> (PhysicsWorld, [ObjectLayer; 5]) {
    let mut layers = CollisionLayers::new(2);
    let fixed = BroadPhaseLayer::new(0);
    let moving = BroadPhaseLayer::new(1);
    let terrain = layers.add_object_layer(fixed);
    let chunk = layers.add_object_layer(fixed);
    let feature = layers.add_object_layer(fixed);
    let item = layers.add_object_layer(moving);
    let actor = layers.add_object_layer(moving);
    layers.enable_collision(item, terrain);
    let world =
        PhysicsWorld::new(WorldSettings::default().gravity(Vec3::ZERO).layers(layers)).unwrap();
    (world, [terrain, chunk, feature, item, actor])
}

/// A flat 33 x 33 heightfield at y = 0 covering x and z in `[-16, 16]` around its body's origin.
pub fn flat_height_field() -> Shape {
    let settings = HeightFieldSettings::default().offset(Vec3::new(-16.0, 0.0, -16.0));
    Shape::new_height_field(33, &[0.0; 33 * 33], &settings).unwrap()
}

/// A static body of `shape` at `position` in `layer`.
pub fn add_static_in(
    world: &mut PhysicsWorld,
    shape: &Shape,
    position: RVec3,
    layer: ObjectLayer,
) -> BodyId {
    world
        .create_body(
            shape,
            &BodySettings::new_static()
                .position(position)
                .object_layer(layer),
        )
        .unwrap()
}

/// Bodies and layers of [`wireframe_scene`].
pub struct WireframeScene {
    pub world: PhysicsWorld,
    pub layers: [ObjectLayer; 5],
    pub terrain: BodyId,
    pub chunk: BodyId,
    pub capsule: BodyId,
    pub far_box: BodyId,
}

/// Rotation of the chunk's structure box about y, in radians.
pub const WIREFRAME_BOX_TURN: f32 = 0.3;

/// A scene with every collider kind the game draws, near the origin: the flat terrain
/// heightfield; at (5, 0, 5) a chunk compound with a unit structure box (half extent 1, centre
/// (5, 1, 5), turned about y by [`WIREFRAME_BOX_TURN`]) and a feature Y-cylinder (half height 1,
/// radius 0.5, centre (9, 1, 5)); an actor capsule (half height 0.7, radius 0.4) at (-5, 1.5, 0);
/// and a far box (half extent 1) at (500, 0, 0).
pub fn wireframe_scene() -> WireframeScene {
    let (mut world, layers) = five_layer_world();
    let [terrain_layer, chunk_layer, feature_layer, _, actor_layer] = layers;
    let structure = Shape::new_box(Vec3::new(1.0, 1.0, 1.0)).unwrap();
    let feature = Shape::new_cylinder(1.0, 0.5).unwrap();
    let compound = Shape::new_compound(&[
        CompoundChild {
            shape: &structure,
            position: Vec3::new(0.0, 1.0, 0.0),
            rotation: quat_about(Vec3::new(0.0, 1.0, 0.0), WIREFRAME_BOX_TURN),
            user_data: Groups::STRUCTURE,
        },
        CompoundChild {
            shape: &feature,
            position: Vec3::new(4.0, 1.0, 0.0),
            rotation: Quat::IDENTITY,
            user_data: Groups::FEATURE,
        },
    ])
    .unwrap();
    let capsule = Shape::new_capsule(0.7, 0.4).unwrap();
    let far_box = Shape::new_box(Vec3::new(1.0, 1.0, 1.0)).unwrap();
    WireframeScene {
        terrain: add_static_in(&mut world, &flat_height_field(), RVec3::ZERO, terrain_layer),
        chunk: add_static_in(
            &mut world,
            &compound,
            RVec3::new(5.0, 0.0, 5.0),
            chunk_layer,
        ),
        capsule: add_static_in(
            &mut world,
            &capsule,
            RVec3::new(-5.0, 1.5, 0.0),
            actor_layer,
        ),
        far_box: add_static_in(
            &mut world,
            &far_box,
            RVec3::new(500.0, 0.0, 0.0),
            feature_layer,
        ),
        world,
        layers,
    }
}

/// A unit cube shape (half extent 0.5).
pub fn cube_shape() -> Shape {
    Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap()
}

/// A compound child at `position` with no rotation.
pub fn child(shape: &Shape, position: Vec3, user_data: u32) -> CompoundChild<'_> {
    CompoundChild {
        shape,
        position,
        rotation: Quat::IDENTITY,
        user_data,
    }
}

/// A bare joltc physics system with one object layer, for the control run.
///
/// # Safety
/// Jolt is initialised, and no other thread creates or destroys a physics system meanwhile.
pub unsafe fn raw_system() -> *mut oxijolt_sys::JPH_PhysicsSystem {
    // SAFETY: Jolt is initialised and system creation is not concurrent (function contract).
    // The three layer tables are consistent and handed to the system, which owns them.
    unsafe {
        use oxijolt_sys::*;

        let pair_filter = JPH_ObjectLayerPairFilterTable_Create(1);
        let broad_phase = JPH_BroadPhaseLayerInterfaceTable_Create(1, 1);
        JPH_BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(broad_phase, 0, 0);
        let object_vs_broad_phase =
            JPH_ObjectVsBroadPhaseLayerFilterTable_Create(broad_phase, 1, pair_filter, 1);
        let settings = JPH_PhysicsSystemSettings {
            maxBodies: 16,
            maxBodyPairs: 16,
            maxContactConstraints: 16,
            broadPhaseLayerInterface: broad_phase,
            objectLayerPairFilter: pair_filter,
            objectVsBroadPhaseLayerFilter: object_vs_broad_phase,
            ..std::mem::zeroed()
        };
        JPH_PhysicsSystem_Create(&settings)
    }
}

/// A bare joltc physics system with one object layer and one dynamic box, for the control run.
///
/// # Safety
/// Jolt is initialised, and no other thread creates or destroys a physics system meanwhile.
pub unsafe fn raw_system_with_body(
) -> (*mut oxijolt_sys::JPH_PhysicsSystem, oxijolt_sys::JPH_BodyID) {
    // SAFETY: Jolt is initialised and system creation is not concurrent (function contract),
    // as `raw_system` requires. The shape and creation settings hold one reference each,
    // released after the body took its own.
    unsafe {
        use oxijolt_sys::*;

        let system = raw_system();
        let half_extent = JPH_Vec3 {
            x: 0.9,
            y: 0.3,
            z: 2.0,
        };
        let shape = JPH_BoxShape_Create(&half_extent, 0.05);
        let position = JPH_RVec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let rotation = JPH_Quat {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        };
        let creation = JPH_BodyCreationSettings_Create3(
            shape.cast(),
            &position,
            &rotation,
            JPH_MotionType_Dynamic,
            0,
        );
        let body = JPH_BodyInterface_CreateAndAddBody(
            JPH_PhysicsSystem_GetBodyInterface(system),
            creation,
            JPH_Activation_DontActivate,
        );
        JPH_BodyCreationSettings_Destroy(creation);
        JPH_Shape_Destroy(shape.cast());
        (system, body)
    }
}
