//! Collision groups (`CollisionGroup`, `GroupFilterTable`): which bodies the solver lets
//! collide, what they do not filter, ragdolls next to caller groups, and the boundaries of the
//! builder and the group.

mod common;

use common::controls::distance;
use common::events::contacts;
use common::ragdoll::{bind_pose, humanoid_settings, ragdoll_world};
use common::soft_body::Cloth;
use common::vehicle::{add_car_with, car_world, chassis_settings, GRAVITY as CAR_GRAVITY};
use common::*;
use oxijolt::*;

const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);

fn table(sub_groups: u32, disabled: &[(u32, u32)]) -> GroupFilterTable {
    let mut builder = GroupFilterTableBuilder::new(sub_groups).unwrap();
    for &(a, b) in disabled {
        builder.disable_collision(a, b).unwrap();
    }
    builder.build()
}

fn group(table: &GroupFilterTable, group_id: u32, sub_group_id: u32) -> CollisionGroup {
    CollisionGroup::new(table, group_id, sub_group_id).unwrap()
}

fn cube_settings(position: RVec3, group: Option<&CollisionGroup>) -> BodySettings {
    let settings = BodySettings::new_dynamic().position(position);
    match group {
        Some(group) => settings.collision_group(group.clone()),
        None => settings,
    }
}

/// A unit cube with `upper` dropped onto a resting unit cube with `lower`; whether it stays on
/// top. Both are checked against `CollisionGroup::can_collide` when both have a group.
fn stacks(lower: Option<&CollisionGroup>, upper: Option<&CollisionGroup>) -> bool {
    let mut world = world(GRAVITY, 2);
    add_floor(&mut world);
    let shape = cube_shape();
    world
        .create_body(&shape, &cube_settings(RVec3::new(0.0, 0.5, 0.0), lower))
        .unwrap();
    let top = world
        .create_body(&shape, &cube_settings(RVec3::new(0.0, 1.6, 0.0), upper))
        .unwrap();
    step(&mut world, 90);
    let stacked = world.body(top).unwrap().position().y > 1.2;
    if let (Some(lower), Some(upper)) = (lower, upper) {
        assert_eq!(lower.can_collide(upper), stacked, "{lower:?} {upper:?}");
        assert_eq!(upper.can_collide(lower), stacked, "symmetric");
    }
    stacked
}

#[test]
fn bodies_of_one_sub_group_pass_through_each_other() {
    let table = table(2, &[]);
    assert!(!stacks(
        Some(&group(&table, 1, 0)),
        Some(&group(&table, 1, 0))
    ));
}

#[test]
fn a_disabled_pair_passes_and_an_enabled_pair_collides() {
    let table = table(3, &[(1, 0)]);
    assert!(!stacks(
        Some(&group(&table, 1, 0)),
        Some(&group(&table, 1, 1))
    ));
    assert!(stacks(
        Some(&group(&table, 1, 0)),
        Some(&group(&table, 1, 2))
    ));
}

#[test]
fn different_group_ids_always_collide() {
    let table = table(1, &[]);
    assert!(stacks(
        Some(&group(&table, 1, 0)),
        Some(&group(&table, 2, 0))
    ));
}

#[test]
fn same_group_id_with_different_tables_never_collides() {
    let (first, second) = (table(2, &[]), table(2, &[]));
    assert_ne!(first, second);
    assert!(!stacks(
        Some(&group(&first, 1, 0)),
        Some(&group(&second, 1, 1))
    ));
}

#[test]
fn grouped_bodies_collide_with_ungrouped_ones() {
    let table = table(1, &[]);
    assert!(stacks(None, Some(&group(&table, 1, 0))));
    assert!(stacks(Some(&group(&table, 1, 0)), None));
}

#[test]
fn can_collide_matches_the_simulation() {
    let table = table(4, &[(0, 3), (2, 1)]);
    let other = self::table(4, &[]);
    let groups = [
        group(&table, 5, 0),
        group(&table, 5, 1),
        group(&table, 5, 2),
        group(&table, 5, 3),
        group(&table, 6, 3),
        group(&other, 5, 1),
    ];
    for lower in &groups {
        for upper in &groups {
            stacks(Some(lower), Some(upper));
        }
    }
}

/// Five boxes in a row without gravity, each overlapping its neighbours by 0.1 m; the largest
/// distance any of them moves.
fn chain_spread(links: &GroupFilterTable) -> f64 {
    let mut world = world(Vec3::ZERO, 2);
    let shape = Shape::new_box(Vec3::new(0.3, 0.1, 0.1)).unwrap();
    let starts: Vec<RVec3> = (0..5)
        .map(|link| RVec3::new(0.5 * link as Real, 0.0, 0.0))
        .collect();
    let ids: Vec<BodyId> = starts
        .iter()
        .zip(0..)
        .map(|(&at, link)| {
            let settings = cube_settings(at, Some(&group(links, 1, link)));
            world.create_body(&shape, &settings).unwrap()
        })
        .collect();
    step(&mut world, 30);
    ids.iter()
        .zip(&starts)
        .map(|(&id, &start)| distance(world.body(id).unwrap().position(), start))
        .fold(0.0, f64::max)
}

#[test]
fn a_chain_without_self_collision() {
    let neighbours: Vec<(u32, u32)> = (0..4).map(|link| (link, link + 1)).collect();
    assert_eq!(chain_spread(&table(5, &neighbours)), 0.0);
    let spread = chain_spread(&table(5, &[]));
    assert!(spread > 0.01, "overlapping links push apart: {spread}");
}

/// A car driving for two seconds with a driver body overlapping the top of its chassis; the
/// chassis position.
fn drive_with_driver(group_driver: bool, driver: bool) -> RVec3 {
    let (mut world, layers) = car_world(Vec3::ZERO, 1);
    let ground = Shape::new_box(Vec3::new(50.0, 1.0, 50.0)).unwrap();
    world
        .create_body(
            &ground,
            &BodySettings::new_static()
                .position(RVec3::new(0.0, -1.0, 0.0))
                .object_layer(layers.ground),
        )
        .unwrap();
    let table = table(1, &[]);
    let crew = group(&table, 1, 0);
    let start = RVec3::new(0.0, 0.9, 0.0);
    let mut chassis = chassis_settings(&layers, start, Quat::IDENTITY);
    if group_driver {
        chassis = chassis.collision_group(crew.clone());
    }
    let (body, car) = add_car_with(
        &mut world,
        &chassis,
        VehicleCollisionTester::ray(layers.probe),
    );
    if driver {
        let mut seat = BodySettings::new_dynamic()
            .position(RVec3::new(0.0, 1.2, 0.0))
            .object_layer(layers.moving)
            .gravity_factor(0.0)
            .mass(80.0);
        if group_driver {
            seat = seat.collision_group(crew);
        }
        world
            .create_body(&Shape::new_box(Vec3::new(0.3, 0.3, 0.3)).unwrap(), &seat)
            .unwrap();
    }
    let mut vehicle = world.vehicle_mut(car).unwrap();
    vehicle.set_gravity(CAR_GRAVITY).unwrap();
    vehicle
        .set_driver_input(DriverInput {
            forward: 1.0,
            ..DriverInput::default()
        })
        .unwrap();
    step(&mut world, 120);
    world.body(body).unwrap().position()
}

#[test]
fn a_vehicle_ignores_its_driver_body() {
    let alone = drive_with_driver(false, false);
    let ignored = drive_with_driver(true, true);
    let collided = drive_with_driver(false, true);
    assert!(alone.z > 2.0, "the wheels drive the car: {alone:?}");
    assert_eq!(ignored, alone, "the driver changes nothing");
    assert_ne!(
        collided, alone,
        "an ungrouped driver is pushed out of the chassis"
    );
}

#[test]
fn a_solver_excluded_body_is_still_found_by_queries_and_blocks_characters() {
    let mut world = world(GRAVITY, 1);
    let table = table(1, &[]);
    let walls = group(&table, 1, 0);
    let floor_shape = Shape::new_box(Vec3::new(20.0, 0.5, 20.0)).unwrap();
    world
        .create_body(
            &floor_shape,
            &BodySettings::new_static()
                .position(RVec3::new(0.0, -0.5, 0.0))
                .collision_group(walls.clone()),
        )
        .unwrap();
    let wall = world
        .create_body(
            &Shape::new_box(Vec3::new(0.5, 2.0, 2.0)).unwrap(),
            &BodySettings::new_static()
                .position(RVec3::new(2.0, 2.0, 0.0))
                .collision_group(walls),
        )
        .unwrap();
    let ray = RayCast::new(RVec3::new(0.0, 1.0, 0.0), Vec3::new(10.0, 0.0, 0.0));
    let hit = world.cast_ray(ray, &QueryFilter::new()).unwrap().unwrap();
    assert_eq!(hit.body, wall);

    let capsule = Shape::new_capsule(0.7, 0.4).unwrap();
    let settings = CharacterSettings::new(&capsule).shape_offset(Vec3::new(0.0, 1.1, 0.0));
    let id = world
        .create_character(&settings, RVec3::ZERO, Quat::IDENTITY)
        .unwrap();
    for _ in 0..90 {
        world
            .character_mut(id)
            .unwrap()
            .set_linear_velocity(Vec3::new(2.0, -1.0, 0.0))
            .unwrap();
        world
            .update_character(
                id,
                DT,
                GRAVITY,
                &ExtendedUpdateSettings::default(),
                &QueryFilter::new(),
            )
            .unwrap();
    }
    assert!(world.character(id).unwrap().position().x < 1.2);
}

/// A cube falling through a static sensor; the sensor's contacts with it.
fn sensed(same_group: bool) -> usize {
    let mut world = world(GRAVITY, 1);
    world.set_event_settings(contacts());
    let table = table(1, &[]);
    let sensor = world
        .create_body(
            &Shape::new_box(Vec3::new(1.0, 1.0, 1.0)).unwrap(),
            &BodySettings::new_static()
                .sensor(true)
                .collision_group(group(&table, 1, 0)),
        )
        .unwrap();
    let cube_group = if same_group {
        group(&table, 1, 0)
    } else {
        group(&table, 2, 0)
    };
    let cube = world
        .create_body(
            &cube_shape(),
            &cube_settings(RVec3::new(0.0, 3.0, 0.0), Some(&cube_group)),
        )
        .unwrap();
    step(&mut world, 60);
    world
        .take_events()
        .contacts
        .iter()
        .filter(|event| {
            let pair = event.pair();
            [pair.body1, pair.body2].contains(&sensor) && [pair.body1, pair.body2].contains(&cube)
        })
        .count()
}

#[test]
fn a_sensor_does_not_see_a_body_its_group_excludes() {
    assert_eq!(sensed(true), 0);
    assert!(sensed(false) > 0);
}

/// A cloth dropped on a table; its height after a second.
fn cloth_height(cloth_group: CollisionGroup, table_group: CollisionGroup) -> Real {
    let mut world = world(GRAVITY, 2);
    world
        .create_body(
            &Shape::new_box(Vec3::new(2.0, 0.5, 2.0)).unwrap(),
            &BodySettings::new_static()
                .position(RVec3::new(0.0, -0.5, 0.0))
                .collision_group(table_group),
        )
        .unwrap();
    let shared = Cloth::new(6, 0.2)
        .builder()
        .create_constraints(SoftBodyBendType::None, SoftBodyVertexAttributes::default())
        .build()
        .unwrap();
    let settings = SoftBodySettings::default()
        .position(RVec3::new(0.0, 0.3, 0.0))
        .collision_group(cloth_group);
    let id = world.create_soft_body(&shared, &settings).unwrap();
    step(&mut world, 60);
    world.body(id).unwrap().position().y
}

#[test]
fn a_cloth_ignores_a_body_of_its_sub_group() {
    let table = table(2, &[]);
    let resting = cloth_height(group(&table, 1, 0), group(&table, 1, 1));
    assert!(resting > -0.2, "{resting}");
    let fallen = cloth_height(group(&table, 1, 0), group(&table, 1, 0));
    assert!(fallen < -1.0, "{fallen}");
}

#[test]
fn ragdoll_parts_refuse_a_collision_group() {
    let skeleton = Skeleton::new(&[SkeletonJoint {
        name: "root",
        parent: None,
    }])
    .unwrap();
    let shape = Shape::new_sphere(0.2).unwrap();
    let table = table(1, &[]);
    let parts = [RagdollPart {
        shape: &shape,
        body: BodySettings::new_dynamic().collision_group(group(&table, 1, 0)),
        joint: None,
    }];
    assert_eq!(
        RagdollSettings::new(&skeleton, &parts).err(),
        Some(RagdollError::InvalidValue(
            "ragdoll parts use the ragdoll's own collision group"
        ))
    );
}

/// A humanoid ragdoll dropped onto a box whose group id is 1, the raw id of the ragdoll; the
/// root's height after two seconds.
#[test]
fn caller_groups_never_alias_a_ragdoll() {
    let (mut world, layers) = ragdoll_world(2);
    world.set_gravity(GRAVITY).unwrap();
    let table = table(1, &[]);
    world
        .create_body(
            &Shape::new_box(Vec3::new(5.0, 0.5, 5.0)).unwrap(),
            &BodySettings::new_static()
                .position(RVec3::new(0.0, -0.5, 0.0))
                .object_layer(layers.fixed)
                .collision_group(group(&table, 1, 0)),
        )
        .unwrap();
    let settings = humanoid_settings(layers.ragdoll);
    let id = world
        .create_ragdoll(&settings, None, Activation::Activate)
        .unwrap();
    assert_eq!(id.to_raw(), 1);
    step(&mut world, 120);
    let (root, _) = world.ragdoll(id).unwrap().root_transform();
    assert!(root.y > 0.0, "the ragdoll lies on the box: {root:?}");
}

/// Two ragdolls of one settings object created 0.1 m apart without gravity: their parts share a
/// table but not a group id, so they push each other apart.
#[test]
fn two_ragdolls_from_one_settings_collide_with_each_other() {
    let (mut world, layers) = ragdoll_world(2);
    let settings = humanoid_settings(layers.ragdoll);
    let first = world
        .create_ragdoll(&settings, None, Activation::Activate)
        .unwrap();
    let mut shifted = bind_pose();
    shifted.root_offset = RVec3::new(0.1, 0.0, 0.0);
    let second = world
        .create_ragdoll(&settings, Some(&shifted), Activation::Activate)
        .unwrap();
    step(&mut world, 60);
    let a = world.ragdoll(first).unwrap().root_transform().0;
    let b = world.ragdoll(second).unwrap().root_transform().0;
    let apart = ((b.x - a.x).powi(2) + (b.z - a.z).powi(2)).sqrt();
    assert!(apart > 0.15, "{a:?} {b:?}");
}

#[test]
fn a_grouped_ragdoll_replays_after_a_restore() {
    let (mut world, layers) = ragdoll_world(4);
    world.set_gravity(GRAVITY).unwrap();
    let table = table(1, &[]);
    world
        .create_body(
            &Shape::new_box(Vec3::new(5.0, 0.5, 5.0)).unwrap(),
            &BodySettings::new_static()
                .position(RVec3::new(0.0, -0.5, 0.0))
                .object_layer(layers.fixed)
                .collision_group(group(&table, 1, 0)),
        )
        .unwrap();
    let id = world
        .create_ragdoll(
            &humanoid_settings(layers.ragdoll),
            None,
            Activation::Activate,
        )
        .unwrap();
    step(&mut world, 20);
    let saved = world.save_state();
    let run = |world: &mut PhysicsWorld| {
        (0..60)
            .map(|_| {
                step(world, 1);
                world.ragdoll(id).unwrap().pose()
            })
            .collect::<Vec<_>>()
    };
    let first = run(&mut world);
    world.restore_state(&saved).unwrap();
    assert_eq!(run(&mut world), first);
}

/// Every Rust handle of a table and its groups is dropped while bodies of two worlds use it.
#[test]
fn tables_outlive_their_handles() {
    let mut worlds = [world(GRAVITY, 1), world(GRAVITY, 1)];
    let mut bodies = Vec::new();
    {
        let table = table(3, &[(0, 1)]);
        let groups: Vec<CollisionGroup> = (0..3).map(|sub| group(&table, 1, sub)).collect();
        for world in &mut worlds {
            add_floor(world);
            let mut ids = Vec::new();
            for (index, group) in groups.iter().enumerate() {
                let at = RVec3::new(2.0 * index as Real, 0.5, 0.0);
                ids.push(
                    world
                        .create_body(&cube_shape(), &cube_settings(at, Some(group)))
                        .unwrap(),
                );
            }
            let shared = Cloth::new(4, 0.2)
                .builder()
                .create_constraints(SoftBodyBendType::None, SoftBodyVertexAttributes::default())
                .build()
                .unwrap();
            let settings = SoftBodySettings::default()
                .position(RVec3::new(-3.0, 0.5, 0.0))
                .collision_group(groups[2].clone());
            ids.push(world.create_soft_body(&shared, &settings).unwrap());
            bodies.push(ids);
        }
    }
    for (world, ids) in worlds.iter_mut().zip(&bodies) {
        step(world, 30);
        for &id in ids {
            world.remove_body(id).unwrap();
        }
        step(world, 2);
    }
    drop(worlds);
}

#[test]
fn builder_and_group_boundaries() {
    use CollisionGroupError::*;
    assert_eq!(GroupFilterTableBuilder::new(0).err(), Some(NoSubGroups));
    let max = GroupFilterTable::MAX_SUB_GROUPS;
    assert_eq!(
        GroupFilterTableBuilder::new(max + 1).err(),
        Some(TooManySubGroups(max + 1))
    );
    let mut largest = GroupFilterTableBuilder::new(max).unwrap();
    largest.disable_collision(max - 1, max - 2).unwrap();
    let largest = largest.build();
    assert_eq!(largest.sub_groups(), max);
    assert!(!largest.is_collision_enabled(max - 2, max - 1).unwrap());
    assert!(largest.is_collision_enabled(0, max - 1).unwrap());

    let mut single = GroupFilterTableBuilder::new(1).unwrap();
    assert_eq!(single.disable_collision(0, 0), Err(SameSubGroup(0)));
    assert_eq!(single.disable_collision(0, 1), Err(SubGroupOutOfRange(1)));
    let single = single.build();
    assert_eq!(single.is_collision_enabled(0, 0), Ok(false));

    let mut builder = GroupFilterTableBuilder::new(4).unwrap();
    builder.disable_collision(3, 1).unwrap();
    assert_eq!(builder.enable_collision(2, 2), Err(SameSubGroup(2)));
    assert_eq!(
        builder.disable_collision(u32::MAX, 0),
        Err(SubGroupOutOfRange(u32::MAX))
    );
    assert_eq!(
        builder.disable_collision(9, 9),
        Err(SubGroupOutOfRange(9)),
        "equal out-of-range ids are out of range first"
    );
    let table = builder.build();
    assert!(!table.is_collision_enabled(1, 3).unwrap(), "either order");
    assert_eq!(table.is_collision_enabled(4, 0), Err(SubGroupOutOfRange(4)));
    assert_eq!(
        table.is_collision_enabled(0, u32::MAX),
        Err(SubGroupOutOfRange(u32::MAX))
    );

    let top = CollisionGroup::MAX_GROUP_ID;
    assert_eq!(CollisionGroup::new(&table, top, 3).unwrap().group_id(), top);
    assert_eq!(
        CollisionGroup::new(&table, top + 1, 0).err(),
        Some(GroupIdOutOfRange(top + 1))
    );
    assert_eq!(
        CollisionGroup::new(&table, 0, 4).err(),
        Some(SubGroupOutOfRange(4))
    );
    let group = CollisionGroup::new(&table, 0, 3).unwrap();
    assert_eq!((group.sub_group_id(), group.table()), (3, &table));
}

#[test]
fn enabling_a_disabled_pair_makes_it_collide_again() {
    use CollisionGroupError::*;
    let mut builder = GroupFilterTableBuilder::new(4).unwrap();
    builder.disable_collision(0, 2).unwrap();
    builder.disable_collision(1, 3).unwrap();
    builder.enable_collision(2, 0).unwrap();
    assert_eq!(builder.enable_collision(1, 1), Err(SameSubGroup(1)));
    assert_eq!(builder.enable_collision(4, 1), Err(SubGroupOutOfRange(4)));
    assert_eq!(
        builder.enable_collision(1, u32::MAX),
        Err(SubGroupOutOfRange(u32::MAX))
    );
    let table = builder.build();
    assert!(table.is_collision_enabled(0, 2).unwrap());
    assert!(table.is_collision_enabled(2, 0).unwrap());
    assert!(
        !table.is_collision_enabled(1, 3).unwrap(),
        "refused calls change nothing"
    );
    let group = |sub_group| CollisionGroup::new(&table, 7, sub_group).unwrap();
    assert!(group(0).can_collide(&group(2)));
    assert!(!group(3).can_collide(&group(1)));
}
