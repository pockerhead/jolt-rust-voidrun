//! Virtual characters: settings validation and ids, ground and contact readout, filters, a
//! per-update up, velocity, the inner body, collisions between characters, save and restore,
//! and rebase.

mod common;

use common::*;
use joltphysics::*;

const RADIUS: f32 = 0.4;
const HALF_HEIGHT: f32 = 0.70845;
/// Puts the capsule's bottom at the character position (plus the padding).
const FOOT_OFFSET: Vec3 = Vec3::new(0.0, HALF_HEIGHT + RADIUS, 0.0);
const UP: Vec3 = Vec3::new(0.0, 1.0, 0.0);
const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);

fn capsule() -> Shape {
    Shape::new_capsule(HALF_HEIGHT, RADIUS).unwrap()
}

fn settings(shape: &Shape) -> CharacterSettings<'_> {
    CharacterSettings::new(shape).shape_offset(FOOT_OFFSET)
}

fn dot(a: Vec3, b: Vec3) -> f32 {
    a.x * b.x + a.y * b.y + a.z * b.z
}

#[allow(clippy::useless_conversion)]
fn real3(p: RVec3) -> [f64; 3] {
    [f64::from(p.x), f64::from(p.y), f64::from(p.z)]
}

fn distance(a: RVec3, b: RVec3) -> f64 {
    let (a, b) = (real3(a), real3(b));
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

/// One update with `velocity`, gravity along `-up` and the stick-to-floor and stairs vectors
/// along `up`.
fn update(
    world: &mut PhysicsWorld,
    id: CharacterId,
    velocity: Vec3,
    up: Vec3,
    filter: &QueryFilter<'_>,
) {
    world
        .character_mut(id)
        .unwrap()
        .set_linear_velocity(velocity)
        .unwrap();
    let extended = ExtendedUpdateSettings::default()
        .stick_to_floor_step_down(up.scale_for_test(-0.5))
        .walk_stairs_step_up(up.scale_for_test(0.4));
    world
        .update_character(id, DT, up.scale_for_test(-9.81), &extended, filter)
        .unwrap();
}

/// Scaling for the tests; the crate keeps its own vector math private.
trait ScaleForTest {
    fn scale_for_test(self, factor: f32) -> Self;
}

impl ScaleForTest for Vec3 {
    fn scale_for_test(self, factor: f32) -> Self {
        Vec3::new(self.x * factor, self.y * factor, self.z * factor)
    }
}

/// Walks with horizontal `velocity` and 1 m/s downward for `ticks` updates.
fn walk(
    world: &mut PhysicsWorld,
    id: CharacterId,
    velocity: Vec3,
    ticks: usize,
    filter: &QueryFilter<'_>,
) {
    let with_down = Vec3::new(velocity.x, velocity.y - 1.0, velocity.z);
    for _ in 0..ticks {
        update(world, id, with_down, UP, filter);
    }
}

#[test]
fn invalid_settings_and_poses_are_rejected_without_side_effects() {
    let mut world = world(GRAVITY, 1);
    let floor = add_floor(&mut world);
    let shape = capsule();
    let compound = Shape::new_compound(&[
        CompoundChild {
            shape: &shape,
            position: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            user_data: 0,
        },
        CompoundChild {
            shape: &shape,
            position: Vec3::new(2.0, 0.0, 0.0),
            rotation: Quat::IDENTITY,
            user_data: 0,
        },
    ])
    .unwrap();
    let height_field = flat_height_field();
    let base = settings(&shape);
    let unknown_layer = InnerBody {
        shape: &shape,
        object_layer: ObjectLayer::new(7),
    };
    let static_only = InnerBody {
        shape: &height_field,
        object_layer: ObjectLayer::MOVING,
    };
    let cases = [
        base.clone().up(Vec3::new(0.0, 2.0, 0.0)),
        base.clone().up(Vec3::new(f32::NAN, 1.0, 0.0)),
        base.clone()
            .supporting_volume(Vec3::new(0.0, 0.5, 0.0), 0.0),
        base.clone().max_slope_angle(2.0),
        base.clone().max_slope_angle(-0.1),
        base.clone().mass(-1.0),
        base.clone().max_strength(f32::INFINITY),
        base.clone().shape_offset(Vec3::new(0.0, f32::NAN, 0.0)),
        base.clone().predictive_contact_distance(-0.1),
        base.clone().max_collision_iterations(0),
        base.clone().max_constraint_iterations(0),
        base.clone().min_time_remaining(0.0),
        base.clone().collision_tolerance(0.0),
        base.clone().character_padding(-0.01),
        base.clone().max_num_hits(0),
        base.clone().hit_reduction_cos_max_angle(f32::NAN),
        base.clone().penetration_recovery_speed(1.5),
        CharacterSettings::new(&compound),
        CharacterSettings::new(&height_field),
        base.clone().inner_body(Some(unknown_layer)),
        base.clone().inner_body(Some(static_only)),
    ];
    for settings in &cases {
        assert!(
            matches!(
                world.create_character(settings, RVec3::ZERO, Quat::IDENTITY),
                Err(CharacterError::InvalidValue(_))
            ),
            "{settings:?}"
        );
    }
    let bad_poses = [
        (RVec3::new(0.0, Real::NAN, 0.0), Quat::IDENTITY),
        (RVec3::ZERO, Quat::from_xyzw(0.0, 0.0, 0.0, 2.0)),
    ];
    for (position, rotation) in bad_poses {
        assert!(matches!(
            world.create_character(&base, position, rotation),
            Err(CharacterError::InvalidValue(_))
        ));
    }
    assert_eq!(world.body_count(), 1);
    assert!(world.contains(floor));
    assert_eq!(world.character_ids().count(), 0);

    let id = world
        .create_character(&base, RVec3::ZERO, Quat::IDENTITY)
        .unwrap();
    let filter = QueryFilter::new();
    let extended = ExtendedUpdateSettings::default();
    for delta_time in [0.0, -DT, f32::NAN, PhysicsWorld::MAX_DELTA_TIME * 2.0] {
        assert!(matches!(
            world.update_character(id, delta_time, GRAVITY, &extended, &filter),
            Err(CharacterError::InvalidValue(_))
        ));
    }
    let bad_extended = [
        extended.stick_to_floor_step_down(Vec3::new(f32::NAN, 0.0, 0.0)),
        extended.walk_stairs_min_step_forward(-1.0),
        extended.walk_stairs_step_forward_test(f32::INFINITY),
    ];
    for bad in bad_extended {
        assert!(matches!(
            world.update_character(id, DT, GRAVITY, &bad, &filter),
            Err(CharacterError::InvalidValue(_))
        ));
    }
    let unknown = [ObjectLayer::new(9)];
    assert!(matches!(
        world.update_character(
            id,
            DT,
            GRAVITY,
            &extended,
            &QueryFilter::new().object_layers(&unknown)
        ),
        Err(CharacterError::InvalidValue(_))
    ));
    let mut character = world.character_mut(id).unwrap();
    assert!(character.set_up(Vec3::new(0.0, 0.5, 0.0)).is_err());
    assert!(character
        .set_rotation(Quat::from_xyzw(1.0, 1.0, 0.0, 0.0))
        .is_err());
    assert!(character
        .set_position(RVec3::new(Real::INFINITY, 0.0, 0.0))
        .is_err());
    assert!(character
        .set_linear_velocity(Vec3::new(0.0, f32::NAN, 0.0))
        .is_err());
    let character = world.character(id).unwrap();
    assert_eq!(character.position(), RVec3::ZERO);
    assert_eq!(character.up(), UP);
}

#[test]
fn ids_count_from_one_are_never_reused_and_belong_to_their_world() {
    let mut world = world(GRAVITY, 1);
    let (mut other, _) = world_with_floor();
    let shape = capsule();
    let ids: Vec<CharacterId> = (0..3)
        .map(|_| {
            world
                .create_character(&settings(&shape), RVec3::ZERO, Quat::IDENTITY)
                .unwrap()
        })
        .collect();
    assert_eq!(
        ids.iter().map(|id| id.to_raw()).collect::<Vec<_>>(),
        [1, 2, 3]
    );
    assert_eq!(world.character_ids().collect::<Vec<_>>(), ids);

    world.remove_character(ids[1]).unwrap();
    assert!(matches!(
        world.character(ids[1]),
        Err(CharacterError::NotFound(_))
    ));
    assert_eq!(
        world.remove_character(ids[1]),
        Err(CharacterError::NotFound(ids[1]))
    );
    let fourth = world
        .create_character(&settings(&shape), RVec3::ZERO, Quat::IDENTITY)
        .unwrap();
    assert_eq!(fourth.to_raw(), 4);
    assert_eq!(
        world.character_ids().collect::<Vec<_>>(),
        [ids[0], ids[2], fourth]
    );

    assert!(matches!(
        other.character(ids[0]),
        Err(CharacterError::WrongWorld(_))
    ));
    assert!(matches!(
        other.update_character(
            ids[0],
            DT,
            GRAVITY,
            &ExtendedUpdateSettings::default(),
            &QueryFilter::new()
        ),
        Err(CharacterError::WrongWorld(_))
    ));
    assert!(matches!(
        other.remove_character(ids[0]),
        Err(CharacterError::WrongWorld(_))
    ));
}

/// A world with gravity and a floor box whose top is at y = 0.
fn world_with_floor() -> (PhysicsWorld, BodyId) {
    let mut world = world(GRAVITY, 1);
    let floor = add_floor(&mut world);
    (world, floor)
}

#[test]
fn a_character_lands_on_a_floor_and_reports_it() {
    let (mut world, floor) = world_with_floor();
    let shape = capsule();
    let id = world
        .create_character(&settings(&shape), RVec3::new(0.0, 0.3, 0.0), Quat::IDENTITY)
        .unwrap();
    for _ in 0..60 {
        let v = world.character(id).unwrap().linear_velocity();
        let fed = Vec3::new(v.x, v.y + GRAVITY.y * DT, v.z);
        update(&mut world, id, fed, UP, &QueryFilter::new());
    }
    let character = world.character(id).unwrap();
    assert_eq!(character.ground_state(), GroundState::OnGround);
    assert!(character.ground_state().is_supported());
    assert_eq!(character.ground_body(), Some(floor));
    assert!(dot(character.ground_normal(), UP) > 0.99);
    assert!(character.ground_compound_child().is_none());
    // The shape rests on the floor, the position sits the padding below its bottom.
    assert!(
        character.position().y.abs() < 0.01,
        "{:?}",
        character.position()
    );

    let contacts = character.active_contacts();
    let colliding: Vec<&CharacterContact> = contacts
        .iter()
        .filter(|contact| contact.had_collision)
        .collect();
    assert_eq!(colliding.len(), 1, "{contacts:?}");
    let contact = colliding[0];
    assert_eq!(contact.body, Some(floor));
    assert_eq!(contact.character, None);
    assert!(dot(contact.contact_normal, UP) > 0.99, "{contact:?}");
    assert_eq!(contact.motion_type, MotionType::Static);
    assert_eq!(
        character.contact_object_layer(contact),
        Some(ObjectLayer::NON_MOVING)
    );
    assert!(!character.max_hits_exceeded());
}

/// A chunk compound at the origin with a structure box out of the way (user data 2) and a
/// feature cylinder on the path along +x (user data 3), on flat terrain.
struct ChunkScene {
    world: PhysicsWorld,
    terrain_layer: ObjectLayer,
    chunk_layer: ObjectLayer,
    actor_layer: ObjectLayer,
    actor: BodyId,
    id: CharacterId,
}

fn chunk_scene() -> ChunkScene {
    let (mut world, [terrain_layer, chunk_layer, _, _, actor_layer]) = five_layer_world();
    add_static_in(&mut world, &flat_height_field(), RVec3::ZERO, terrain_layer);
    let block = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
    let cylinder = Shape::new_cylinder(1.0, 0.5).unwrap();
    let chunk = Shape::new_compound(&[
        CompoundChild {
            shape: &block,
            position: Vec3::new(-5.0, 0.5, 5.0),
            rotation: Quat::IDENTITY,
            user_data: Groups::STRUCTURE,
        },
        CompoundChild {
            shape: &cylinder,
            position: Vec3::new(3.0, 1.0, 0.0),
            rotation: Quat::IDENTITY,
            user_data: Groups::FEATURE,
        },
    ])
    .unwrap();
    add_static_in(&mut world, &chunk, RVec3::ZERO, chunk_layer);
    let shape = capsule();
    let start = RVec3::new(0.0, 0.0, 0.0);
    let actor = world
        .create_body(
            &shape,
            &BodySettings::new_kinematic()
                .position(RVec3::new(0.0, (0.02 + RADIUS + HALF_HEIGHT) as Real, 0.0))
                .object_layer(actor_layer),
        )
        .unwrap();
    let id = world
        .create_character(&settings(&shape), start, Quat::IDENTITY)
        .unwrap();
    ChunkScene {
        world,
        terrain_layer,
        chunk_layer,
        actor_layer,
        actor,
        id,
    }
}

#[test]
fn compound_children_report_their_group_and_filters_select_them() {
    let along_x = Vec3::new(2.0, 0.0, 0.0);

    // Every layer, every group, the own actor body excluded: the feature cylinder blocks.
    let mut scene = chunk_scene();
    let layers = [scene.terrain_layer, scene.chunk_layer, scene.actor_layer];
    let all = QueryFilter::new()
        .object_layers(&layers)
        .exclude_body(scene.actor);
    walk(&mut scene.world, scene.id, along_x, 120, &all);
    let character = scene.world.character(scene.id).unwrap();
    let x = character.position().x;
    assert!(
        x < 2.2 && x > 1.9,
        "blocked before the cylinder at x 2.5: {x}"
    );
    let feature = character
        .active_contacts()
        .into_iter()
        .filter(|contact| contact.had_collision)
        .find_map(|contact| character.contact_compound_child(&contact))
        .expect("a contact with the cylinder");
    assert_eq!(feature.user_data, Groups::FEATURE);
    assert_eq!(feature.index, 1);

    // Structures only: the character walks through the feature.
    let mut scene = chunk_scene();
    let structures = QueryFilter::new()
        .object_layers(&layers)
        .exclude_body(scene.actor)
        .child_groups(1 << Groups::STRUCTURE);
    walk(&mut scene.world, scene.id, along_x, 120, &structures);
    let x = scene.world.character(scene.id).unwrap().position().x;
    assert!((x - 4.0).abs() < 0.02, "walked through the feature: {x}");

    // Without the terrain layer there is nothing to stand on.
    let mut scene = chunk_scene();
    let no_terrain = [scene.chunk_layer, scene.actor_layer];
    let filter = QueryFilter::new()
        .object_layers(&no_terrain)
        .exclude_body(scene.actor);
    walk(&mut scene.world, scene.id, Vec3::ZERO, 60, &filter);
    let character = scene.world.character(scene.id).unwrap();
    assert!(character.position().y < -0.5, "{:?}", character.position());
    assert_eq!(character.ground_state(), GroundState::InAir);

    // The own actor capsule sits where the character is: excluded it does not hinder the walk,
    // included the character collides with it.
    let mut scene = chunk_scene();
    let own_excluded = QueryFilter::new()
        .object_layers(&layers)
        .exclude_body(scene.actor);
    walk(
        &mut scene.world,
        scene.id,
        Vec3::new(0.0, 0.0, 2.0),
        30,
        &own_excluded,
    );
    let z = scene.world.character(scene.id).unwrap().position().z;
    assert!((z - 1.0).abs() < 0.02, "{z}");
    let mut scene = chunk_scene();
    let with_actor = QueryFilter::new().object_layers(&layers);
    walk(
        &mut scene.world,
        scene.id,
        Vec3::new(0.0, 0.0, 2.0),
        1,
        &with_actor,
    );
    let character = scene.world.character(scene.id).unwrap();
    assert!(character
        .active_contacts()
        .iter()
        .any(|contact| contact.body == Some(scene.actor)));
}

/// The rotation by 90 degrees about +X, which maps +Y to +Z.
fn y_to_z() -> Quat {
    quat_about(Vec3::new(1.0, 0.0, 0.0), std::f32::consts::FRAC_PI_2)
}

#[test]
fn up_and_rotation_set_per_update_give_the_same_walk_in_another_frame() {
    let shape = capsule();
    let run = |up: Vec3, floor_rotation: Quat, floor_position: RVec3, start: RVec3| {
        let mut world = world(Vec3::ZERO, 1);
        let floor = Shape::new_box(Vec3::new(100.0, 1.0, 100.0)).unwrap();
        world
            .create_body(
                &floor,
                &BodySettings::new_static()
                    .position(floor_position)
                    .rotation(floor_rotation),
            )
            .unwrap();
        let id = world
            .create_character(&settings(&shape), start, Quat::IDENTITY)
            .unwrap();
        let mut character = world.character_mut(id).unwrap();
        character.set_up(up).unwrap();
        character.set_rotation(floor_rotation).unwrap();
        let down = up.scale_for_test(-1.0);
        let velocity = Vec3::new(2.0 + down.x, down.y, down.z);
        let mut path = Vec::new();
        for _ in 0..60 {
            update(&mut world, id, velocity, up, &QueryFilter::new());
            let character = world.character(id).unwrap();
            path.push((real3(character.position()), character.ground_state()));
        }
        path
    };
    let along_y = run(
        UP,
        Quat::IDENTITY,
        RVec3::new(0.0, -1.0, 0.0),
        RVec3::new(0.0, 0.05, 0.0),
    );
    let along_z = run(
        Vec3::new(0.0, 0.0, 1.0),
        y_to_z(),
        RVec3::new(0.0, 0.0, -1.0),
        RVec3::new(0.0, 0.0, 0.05),
    );
    for (tick, ((p, ground_y), (q, ground_z))) in along_y.iter().zip(&along_z).enumerate() {
        assert_eq!(ground_y, ground_z, "tick {tick}");
        // The rotation maps (x, y, z) to (x, -z, y).
        let mapped = [p[0], -p[2], p[1]];
        let error = (0..3).map(|i| (mapped[i] - q[i]).abs()).fold(0.0, f64::max);
        assert!(
            error < 1e-5,
            "tick {tick}: {p:?} maps to {mapped:?}, got {q:?}"
        );
    }
    assert_eq!(along_y.last().unwrap().1, GroundState::OnGround);
}

#[test]
fn linear_velocity_round_trips_and_moves_a_free_character() {
    let mut world = world(Vec3::ZERO, 1);
    let shape = capsule();
    let id = world
        .create_character(&settings(&shape), RVec3::ZERO, Quat::IDENTITY)
        .unwrap();
    let velocity = Vec3::new(0.1, -0.7, 1.3);
    world
        .character_mut(id)
        .unwrap()
        .set_linear_velocity(velocity)
        .unwrap();
    let read: [f32; 3] = world.character(id).unwrap().linear_velocity().into();
    let set: [f32; 3] = velocity.into();
    assert_eq!(read.map(f32::to_bits), set.map(f32::to_bits));

    world
        .update_character(
            id,
            DT,
            Vec3::ZERO,
            &ExtendedUpdateSettings::default(),
            &QueryFilter::new(),
        )
        .unwrap();
    let character = world.character(id).unwrap();
    let moved = real3(character.position());
    for (axis, (moved, v)) in moved.iter().zip(set).enumerate() {
        let expected = f64::from(v * DT);
        assert!(
            (moved - expected).abs() < 1e-6,
            "axis {axis}: {moved} vs {expected}"
        );
    }
    assert_eq!(character.ground_state(), GroundState::InAir);
}

/// Walks a character from the origin along +x at 2 m/s for 45 ticks towards a 0.25 m step with
/// Jolt's rounded box edges (face at x 1, top from x 1 to 4), with Jolt's walk stairs stepping
/// up `step_up`; returns the final position.
fn walk_at_a_step(step_up: f32) -> RVec3 {
    let (mut world, _) = world_with_floor();
    let step = Shape::new_box(Vec3::new(1.5, 0.125, 2.0)).unwrap();
    world
        .create_body(
            &step,
            &BodySettings::new_static().position(RVec3::new(2.5, 0.125, 0.0)),
        )
        .unwrap();
    let shape = capsule();
    let id = world
        .create_character(&settings(&shape), RVec3::ZERO, Quat::IDENTITY)
        .unwrap();
    world
        .refresh_character_contacts(id, &QueryFilter::new())
        .unwrap();
    let extended = ExtendedUpdateSettings::default()
        .stick_to_floor_step_down(UP.scale_for_test(-0.5))
        .walk_stairs_step_up(UP.scale_for_test(step_up));
    for _ in 0..45 {
        world
            .character_mut(id)
            .unwrap()
            .set_linear_velocity(Vec3::new(2.0, -1.0, 0.0))
            .unwrap();
        world
            .update_character(id, DT, GRAVITY, &extended, &QueryFilter::new())
            .unwrap();
    }
    world.character(id).unwrap().position()
}

/// The ground state of a character resting on a 10° slope with the given slope limit.
fn ground_state_on_a_gentle_slope(max_slope_angle: f32) -> GroundState {
    let mut world = world(GRAVITY, 1);
    let angle = 10.0_f32.to_radians();
    let (sin, cos) = (angle * 0.5).sin_cos();
    let slope = Shape::new_box(Vec3::new(5.0, 0.5, 5.0)).unwrap();
    world
        .create_body(
            &slope,
            &BodySettings::new_static().rotation(Quat::from_xyzw(0.0, 0.0, sin, cos)),
        )
        .unwrap();
    let shape = capsule();
    let top = 0.5 / angle.cos();
    let id = world
        .create_character(
            &settings(&shape).max_slope_angle(max_slope_angle),
            RVec3::new(0.0, top as Real, 0.0),
            Quat::IDENTITY,
        )
        .unwrap();
    world
        .refresh_character_contacts(id, &QueryFilter::new())
        .unwrap();
    world.character(id).unwrap().ground_state()
}

#[test]
fn a_slope_limit_near_zero_turns_the_limit_off() {
    assert_eq!(
        ground_state_on_a_gentle_slope(1.0_f32.to_radians()),
        GroundState::OnSteepGround
    );
    assert_eq!(
        ground_state_on_a_gentle_slope(0.5_f32.to_radians()),
        GroundState::OnGround
    );
    assert_eq!(ground_state_on_a_gentle_slope(0.0), GroundState::OnGround);
}

#[test]
fn walk_stairs_climbs_a_step_that_stops_a_character_without_it() {
    let climbed = walk_at_a_step(0.4);
    assert!(
        (climbed.y - 0.25).abs() < 0.01 && climbed.x > 1.0,
        "on top of the step: {climbed:?}"
    );
    let stopped = walk_at_a_step(0.0);
    assert!(
        stopped.y.abs() < 0.01 && stopped.x < 1.0,
        "in front of the step: {stopped:?}"
    );
}

#[test]
fn an_inner_body_is_a_body_of_the_world_owned_by_its_character() {
    let (mut world, floor) = world_with_floor();
    let shape = capsule();
    let inner_shape = Shape::new_capsule(HALF_HEIGHT, RADIUS).unwrap();
    let with_inner = settings(&shape).inner_body(Some(InnerBody {
        shape: &inner_shape,
        object_layer: ObjectLayer::MOVING,
    }));
    let id = world
        .create_character(&with_inner, RVec3::new(0.0, 0.0, 0.0), Quat::IDENTITY)
        .unwrap();
    assert_eq!(world.body_count(), 2);
    let inner = world.character(id).unwrap().inner_body().unwrap();
    assert!(world.contains(inner));
    assert!(world.is_inner_body(inner));
    assert!(!world.is_inner_body(floor));
    assert_eq!(
        world.body(inner).unwrap().motion_type(),
        MotionType::Kinematic
    );
    // Queries find it, in its layer.
    let hit = world
        .cast_ray(
            RayCast::new(RVec3::new(0.0, 5.0, 0.0), Vec3::new(0.0, -10.0, 0.0)),
            &QueryFilter::new(),
        )
        .unwrap()
        .unwrap();
    assert_eq!(hit.body, inner);
    assert_eq!(hit.object_layer, ObjectLayer::MOVING);

    // A box dropped on the character lands on its inner body.
    let cube = add_cube(&mut world, RVec3::new(0.0, 4.0, 0.0));
    for _ in 0..120 {
        update(
            &mut world,
            id,
            GRAVITY.scale_for_test(DT),
            UP,
            &QueryFilter::new(),
        );
        assert!(world.step(DT).unwrap().is_complete());
    }
    let cube_y = world.body(cube).unwrap().position().y;
    assert!(cube_y > 2.0, "the cube rests on the inner body: {cube_y}");

    // The character does not collide with its own inner body.
    let start = world.character(id).unwrap().position();
    walk(
        &mut world,
        id,
        Vec3::new(2.0, 0.0, 0.0),
        30,
        &QueryFilter::new().exclude_body(cube),
    );
    let moved = distance(world.character(id).unwrap().position(), start);
    assert!((moved - 1.0).abs() < 0.02, "{moved}");

    assert_eq!(
        world.remove_body(inner),
        Err(BodyError::OwnedByCharacter(inner))
    );
    assert!(world.contains(inner));
    let count = world.body_count();
    world.remove_character(id).unwrap();
    assert_eq!(world.body_count(), count - 1);
    assert!(!world.contains(inner));
    assert!(!world.is_inner_body(inner));

    let without = world
        .create_character(&settings(&shape), RVec3::ZERO, Quat::IDENTITY)
        .unwrap();
    assert_eq!(world.character(without).unwrap().inner_body(), None);
}

#[test]
fn dropping_a_world_with_characters_releases_them() {
    for inner in [false, true] {
        let (mut world, _) = world_with_floor();
        let shape = capsule();
        let mut character_settings = settings(&shape).collide_with_characters(true);
        if inner {
            character_settings = character_settings.inner_body(Some(InnerBody {
                shape: &shape,
                object_layer: ObjectLayer::MOVING,
            }));
        }
        for x in 0..3 {
            let id = world
                .create_character(
                    &character_settings,
                    RVec3::new(x as Real * 2.0, 0.0, 0.0),
                    Quat::IDENTITY,
                )
                .unwrap();
            update(&mut world, id, Vec3::ZERO, UP, &QueryFilter::new());
        }
        drop(world);
    }
}

/// Two characters 4 m apart on a floor walking toward each other for 90 ticks; returns their
/// final positions and whether each reported a contact with the other.
fn approach(collide: bool) -> ([RVec3; 2], [bool; 2]) {
    let (mut world, _) = world_with_floor();
    let shape = capsule();
    let character_settings = settings(&shape).collide_with_characters(collide);
    let ids = [-2.0, 2.0].map(|x| {
        world
            .create_character(&character_settings, RVec3::new(x, 0.0, 0.0), Quat::IDENTITY)
            .unwrap()
    });
    let mut touched = [false; 2];
    for _ in 0..90 {
        for (i, &id) in ids.iter().enumerate() {
            let direction = if i == 0 { 2.0 } else { -2.0 };
            walk(
                &mut world,
                id,
                Vec3::new(direction, 0.0, 0.0),
                1,
                &QueryFilter::new(),
            );
            let other = ids[1 - i];
            touched[i] |= world
                .character(id)
                .unwrap()
                .active_contacts()
                .iter()
                .any(|contact| contact.character == Some(other) && contact.body.is_none());
        }
    }
    let positions = ids.map(|id| world.character(id).unwrap().position());
    (positions, touched)
}

#[test]
fn characters_that_collide_with_characters_keep_apart() {
    let ([a, b], touched) = approach(true);
    let gap = distance(a, b);
    assert!(gap >= f64::from(2.0 * RADIUS) - 0.01, "{gap}");
    assert_eq!(touched, [true, true]);

    let ([a, b], touched) = approach(false);
    assert!(
        a.x > 0.9 && b.x < -0.9,
        "passed through each other: {a:?} {b:?}"
    );
    assert_eq!(touched, [false, false]);
}

#[test]
fn removing_a_character_mid_run_leaves_the_others_sound() {
    let (mut world, _) = world_with_floor();
    let shape = capsule();
    let character_settings = settings(&shape).collide_with_characters(true);
    let ids = [-0.5, 0.5].map(|x| {
        world
            .create_character(&character_settings, RVec3::new(x, 0.0, 0.0), Quat::IDENTITY)
            .unwrap()
    });
    for _ in 0..10 {
        walk(
            &mut world,
            ids[0],
            Vec3::new(2.0, 0.0, 0.0),
            1,
            &QueryFilter::new(),
        );
        walk(
            &mut world,
            ids[1],
            Vec3::new(-2.0, 0.0, 0.0),
            1,
            &QueryFilter::new(),
        );
    }
    world.remove_character(ids[1]).unwrap();
    for _ in 0..30 {
        walk(
            &mut world,
            ids[0],
            Vec3::new(2.0, 0.0, 0.0),
            1,
            &QueryFilter::new(),
        );
    }
    let character = world.character(ids[0]).unwrap();
    assert!(character.position().x > 0.5);
    assert!(character
        .active_contacts()
        .iter()
        .all(|contact| contact.character.is_none()));
}

/// The scripted horizontal velocity of tick `tick`: walk, turn, stop, walk back. A character
/// with a `phase` drifts towards the first one (created at z 0) and then lets it catch up, so
/// colliding characters press on each other.
fn scripted(tick: usize, phase: f32) -> Vec3 {
    match tick {
        0..=39 => Vec3::new(2.0, 0.0, -phase),
        40..=69 => Vec3::new(0.0, 0.0, 2.0 - phase),
        70..=84 => Vec3::ZERO,
        _ => Vec3::new(-1.5, 0.0, 0.5),
    }
}

/// A contact as bits: body, character, sub-shape, contact normal and whether it collided.
type ContactBits = (Option<u32>, Option<u32>, u32, [u32; 3], bool);

/// What a run records per tick, as bits.
#[derive(Debug, PartialEq)]
struct Record {
    position: Vec<u64>,
    velocity: Vec<u32>,
    ground: GroundState,
    state: Vec<u8>,
    contacts: Vec<ContactBits>,
}

fn record(world: &PhysicsWorld, id: CharacterId) -> Record {
    let character = world.character(id).unwrap();
    assert!(!character.max_hits_exceeded());
    let velocity: [f32; 3] = character.linear_velocity().into();
    Record {
        position: real3(character.position()).map(f64::to_bits).to_vec(),
        velocity: velocity.map(f32::to_bits).to_vec(),
        ground: character.ground_state(),
        state: character.save_state().as_bytes(),
        contacts: character
            .active_contacts()
            .iter()
            .map(|contact| {
                let normal: [f32; 3] = contact.contact_normal.into();
                (
                    contact.body.map(BodyId::to_raw),
                    contact.character.map(CharacterId::to_raw),
                    contact.sub_shape_id.to_raw(),
                    normal.map(f32::to_bits),
                    contact.had_collision,
                )
            })
            .collect(),
    }
}

/// A scene for replays: a floor, a step and a ramp, and `count` characters that collide with
/// each other, created in the same order every time.
fn replay_world(count: usize, shape: &Shape) -> (PhysicsWorld, Vec<CharacterId>) {
    let (mut world, _) = world_with_floor();
    let step = Shape::new_box(Vec3::new(0.5, 0.15, 2.0)).unwrap();
    world
        .create_body(
            &step,
            &BodySettings::new_static().position(RVec3::new(1.8, 0.15, 0.0)),
        )
        .unwrap();
    let ramp = Shape::new_box(Vec3::new(2.0, 0.1, 2.0)).unwrap();
    world
        .create_body(
            &ramp,
            &BodySettings::new_static()
                .position(RVec3::new(-2.5, 0.3, 2.0))
                .rotation(quat_about(Vec3::new(0.0, 0.0, 1.0), 0.3)),
        )
        .unwrap();
    let character_settings = settings(shape).collide_with_characters(count > 1);
    let ids = (0..count)
        .map(|i| {
            world
                .create_character(
                    &character_settings,
                    RVec3::new(0.0, 0.3, i as Real * 0.9),
                    Quat::IDENTITY,
                )
                .unwrap()
        })
        .collect();
    (world, ids)
}

fn replay_tick(world: &mut PhysicsWorld, ids: &[CharacterId], tick: usize) {
    for (i, &id) in ids.iter().enumerate() {
        let horizontal = scripted(tick, i as f32 * 0.5);
        let velocity = Vec3::new(horizontal.x, -1.0, horizontal.z);
        update(world, id, velocity, UP, &QueryFilter::new());
    }
}

/// Runs 120 ticks continuously and then chained, one tick per freshly built world restored
/// from the previous tick's states, and compares every tick bit for bit.
fn chained_replay_matches(count: usize) {
    const TICKS: usize = 120;
    let shape = capsule();
    let (mut world, ids) = replay_world(count, &shape);
    let mut continuous = Vec::new();
    for tick in 0..TICKS {
        replay_tick(&mut world, &ids, tick);
        continuous.push(ids.iter().map(|&id| record(&world, id)).collect::<Vec<_>>());
    }
    if count > 1 {
        // Every tick ends at a save, so the replay restores states that hold collisions between
        // the characters.
        let colliding = continuous
            .iter()
            .filter(|records| {
                records.iter().any(|record| {
                    record
                        .contacts
                        .iter()
                        .any(|contact| contact.1.is_some() && contact.4)
                })
            })
            .count();
        assert!(
            colliding >= 20,
            "the characters collided in {colliding} ticks"
        );
    }

    let (world, ids) = replay_world(count, &shape);
    let mut states: Vec<CharacterState> = ids
        .iter()
        .map(|&id| world.character(id).unwrap().save_state())
        .collect();
    drop(world);
    for (tick, expected) in continuous.iter().enumerate() {
        let (mut world, ids) = replay_world(count, &shape);
        for (&id, state) in ids.iter().zip(&states) {
            world
                .character_mut(id)
                .unwrap()
                .restore_state(state)
                .unwrap();
        }
        replay_tick(&mut world, &ids, tick);
        let chained: Vec<Record> = ids.iter().map(|&id| record(&world, id)).collect();
        assert_eq!(&chained, expected, "tick {tick}");
        states = ids
            .iter()
            .map(|&id| world.character(id).unwrap().save_state())
            .collect();
    }
}

#[test]
fn a_chained_replay_of_one_character_is_bit_exact() {
    chained_replay_matches(1);
}

#[test]
fn a_chained_replay_of_colliding_characters_is_bit_exact() {
    chained_replay_matches(2);
}

#[test]
fn a_restored_state_saves_the_same_bytes() {
    let shape = capsule();
    let (mut world, ids) = replay_world(1, &shape);
    for tick in 0..30 {
        replay_tick(&mut world, &ids, tick);
    }
    let state = world.character(ids[0]).unwrap().save_state();
    let (mut other, other_ids) = replay_world(1, &shape);
    other
        .character_mut(other_ids[0])
        .unwrap()
        .restore_state(&state)
        .unwrap();
    assert_eq!(other.character(other_ids[0]).unwrap().save_state(), state);
}

/// The inner body's pose, as bits.
fn inner_pose(world: &PhysicsWorld, id: CharacterId) -> (Vec<u64>, [u32; 4]) {
    let inner = world.character(id).unwrap().inner_body().unwrap();
    let body = world.body(inner).unwrap();
    let rotation = body.rotation();
    (
        real3(body.position()).map(f64::to_bits).to_vec(),
        [rotation.x, rotation.y, rotation.z, rotation.w].map(f32::to_bits),
    )
}

#[test]
fn a_restored_state_moves_the_inner_body_with_the_character() {
    let shape = capsule();
    let inner_shape = capsule();
    let with_inner = settings(&shape).inner_body(Some(InnerBody {
        shape: &inner_shape,
        object_layer: ObjectLayer::MOVING,
    }));
    let start = RVec3::new(0.0, 0.3, 0.0);
    let (mut world, _) = world_with_floor();
    let id = world
        .create_character(&with_inner, start, Quat::IDENTITY)
        .unwrap();
    for tick in 0..60 {
        replay_tick(&mut world, &[id], tick);
    }
    let state = world.character(id).unwrap().save_state();

    let (mut other, _) = world_with_floor();
    let other_id = other
        .create_character(&with_inner, start, Quat::IDENTITY)
        .unwrap();
    other
        .character_mut(other_id)
        .unwrap()
        .restore_state(&state)
        .unwrap();
    // Before any update or step, the restored world's inner body stands where the original's
    // does.
    assert_eq!(inner_pose(&other, other_id), inner_pose(&world, id));
}

/// A rebase frame: a turn about `axis` and a translation.
fn frame() -> (Quat, RVec3) {
    let axis = [0.3_f32, 0.9, 0.3];
    let length = axis.iter().map(|c| c * c).sum::<f32>().sqrt();
    let axis = Vec3::new(axis[0] / length, axis[1] / length, axis[2] / length);
    (quat_about(axis, 0.7), RVec3::new(120.0, -35.0, 60.0))
}

/// `p` mapped through the frame, in f64.
fn map_point(rotation: Quat, translation: RVec3, p: [f64; 3]) -> [f64; 3] {
    let r = rotate(rotation, p);
    let t = real3(translation);
    [r[0] + t[0], r[1] + t[1], r[2] + t[2]]
}

fn rotate(q: Quat, v: [f64; 3]) -> [f64; 3] {
    let [x, y, z, w] = [q.x, q.y, q.z, q.w].map(f64::from);
    let u = [x, y, z];
    let cross = |a: [f64; 3], b: [f64; 3]| {
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    };
    let t = cross(u, v).map(|c| 2.0 * c);
    let s = cross(u, t);
    [
        v[0] + w * t[0] + s[0],
        v[1] + w * t[1] + s[1],
        v[2] + w * t[2] + s[2],
    ]
}

fn to_vec3(v: [f64; 3]) -> Vec3 {
    Vec3::new(v[0] as f32, v[1] as f32, v[2] as f32)
}

#[test]
fn a_character_crosses_a_rebase_with_the_world() {
    const BEFORE: usize = 30;
    const AFTER: usize = 60;
    let (rotation, translation) = frame();
    let shape = capsule();
    let inner_shape = Shape::new_sphere(0.3).unwrap();
    let run = |rebase: bool| {
        let (mut world, floor) = world_with_floor();
        let character_settings = settings(&shape).inner_body(Some(InnerBody {
            shape: &inner_shape,
            object_layer: ObjectLayer::MOVING,
        }));
        let id = world
            .create_character(&character_settings, RVec3::ZERO, Quat::IDENTITY)
            .unwrap();
        let inner = world.character(id).unwrap().inner_body().unwrap();
        let mut up = UP;
        let mut horizontal = Vec3::new(2.0, 0.0, 0.5);
        for _ in 0..BEFORE {
            let v = Vec3::new(
                horizontal.x - up.x,
                horizontal.y - up.y,
                horizontal.z - up.z,
            );
            update(&mut world, id, v, up, &QueryFilter::new());
        }
        if rebase {
            world
                .rebase(&[floor, inner], rotation, translation)
                .unwrap();
            // Jolt places the inner body at position + rotation * shape offset + padding * up
            // (`CharacterVirtual::GetInnerBodyPosition`).
            let character = world.character(id).unwrap();
            let offset = rotate(
                character.rotation(),
                [0.0, f64::from(HALF_HEIGHT + RADIUS), 0.0],
            );
            let new_up = character.up();
            let p = real3(character.position());
            let expected =
                [0, 1, 2].map(|i| p[i] + offset[i] + 0.02 * f64::from(<[f32; 3]>::from(new_up)[i]));
            let actual = real3(world.body(inner).unwrap().position());
            let gap = (0..3)
                .map(|i| (expected[i] - actual[i]).abs())
                .fold(0.0, f64::max);
            assert!(gap < 1e-5, "inner body {actual:?}, expected {expected:?}");
            up = to_vec3(rotate(rotation, [0.0, 1.0, 0.0]));
            horizontal = to_vec3(rotate(rotation, [2.0, 0.0, 0.5]));
            world
                .refresh_character_contacts(id, &QueryFilter::new())
                .unwrap();
        }
        let mut path = Vec::new();
        for _ in 0..AFTER {
            let v = Vec3::new(
                horizontal.x - up.x,
                horizontal.y - up.y,
                horizontal.z - up.z,
            );
            update(&mut world, id, v, up, &QueryFilter::new());
            let character = world.character(id).unwrap();
            assert_eq!(character.ground_state(), GroundState::OnGround);
            path.push(real3(character.position()));
        }
        path
    };
    let plain = run(false);
    let rebased = run(true);
    for (tick, (p, q)) in plain.iter().zip(&rebased).enumerate() {
        let mapped = map_point(rotation, translation, *p);
        let error = (0..3).map(|i| (mapped[i] - q[i]).abs()).fold(0.0, f64::max);
        assert!(error < 1e-3, "tick {tick}: {mapped:?} vs {q:?}");
    }
}
