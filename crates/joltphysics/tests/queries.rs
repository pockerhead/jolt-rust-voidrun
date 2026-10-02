//! Scene queries: ray casts with normals and filters.

mod common;

use common::*;
use joltphysics::*;

fn down_from(x: Real, y: Real, z: Real, length: f32) -> RayCast {
    RayCast::new(RVec3::new(x, y, z), Vec3::new(0.0, -length, 0.0))
}

fn child(shape: &Shape, position: Vec3, user_data: u32) -> CompoundChild<'_> {
    CompoundChild {
        shape,
        position,
        rotation: Quat::IDENTITY,
        user_data,
    }
}

const ALL: QueryFilter<'static> = QueryFilter::new();

fn dot(a: Vec3, b: Vec3) -> f32 {
    a.x * b.x + a.y * b.y + a.z * b.z
}

#[test]
fn ray_hits_chunk_and_terrain_from_twenty_metres() {
    let (mut world, [terrain, chunk, ..]) = five_layer_world();
    let block = Shape::new_box(Vec3::new(1.0, 1.0, 1.0)).unwrap();
    let pillar = Shape::new_cylinder(1.5, 0.4).unwrap();
    let chunk_shape = Shape::new_compound(&[
        child(&block, Vec3::new(0.0, 1.0, 0.0), Groups::STRUCTURE),
        child(&block, Vec3::new(3.0, 1.0, 0.0), Groups::STRUCTURE),
        child(&pillar, Vec3::new(0.0, 1.5, 4.0), Groups::FEATURE),
    ])
    .unwrap();
    add_static_in(&mut world, &chunk_shape, RVec3::new(6.0, 0.0, 0.0), chunk);
    let ground = add_static_in(&mut world, &flat_height_field(), RVec3::ZERO, terrain);

    let hit = world
        .cast_ray(down_from(-3.0, 20.0, 2.0, 40.0), &ALL)
        .unwrap()
        .expect("the terrain is hit");
    assert_eq!(hit.body, ground);
    assert!((hit.distance - 20.0).abs() <= 0.5, "{hit:?}");
    assert!(hit.normal.y > 0.99, "{hit:?}");
    assert_eq!(hit.object_layer, terrain);
    assert_eq!(hit.compound_child, None);

    let hit = world
        .cast_ray(down_from(6.0, 20.0, 0.0, 40.0), &ALL)
        .unwrap()
        .expect("the chunk is hit");
    assert!((hit.distance - 18.0).abs() <= 1.0e-4, "{hit:?}");
    assert_eq!(hit.object_layer, chunk);
    assert_eq!(
        hit.compound_child,
        Some(CompoundSubShape {
            index: 0,
            user_data: Groups::STRUCTURE
        })
    );
}

#[test]
fn wall_blocks_line_of_sight_until_removed() {
    let mut world = world(Vec3::ZERO, 1);
    let wall_shape = Shape::new_box(Vec3::new(0.25, 2.0, 2.0)).unwrap();
    let wall = world
        .create_body(
            &wall_shape,
            &BodySettings::new_static().position(RVec3::new(5.0, 1.0, 0.0)),
        )
        .unwrap();
    let sight = RayCast::new(RVec3::new(0.0, 1.0, 0.0), Vec3::new(10.0, 0.0, 0.0));
    let hit = world
        .cast_ray(sight, &ALL)
        .unwrap()
        .expect("the wall blocks");
    assert_eq!(hit.body, wall);
    assert!(hit.fraction < 1.0);
    assert!((hit.distance - 4.75).abs() <= 1.0e-5, "{hit:?}");

    world.remove_body(wall).unwrap();
    assert_eq!(world.cast_ray(sight, &ALL), Ok(None));
}

#[test]
fn ray_starting_inside_hits_at_exactly_zero() {
    let mut world = world(Vec3::ZERO, 1);
    let unit_box = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
    let block = world
        .create_body(
            &unit_box,
            &BodySettings::new_static().position(RVec3::new(0.0, 0.0, 0.0)),
        )
        .unwrap();
    let compound = Shape::new_compound(&[
        child(&unit_box, Vec3::ZERO, Groups::STRUCTURE),
        child(&unit_box, Vec3::new(2.0, 0.0, 0.0), Groups::FEATURE),
    ])
    .unwrap();
    let chunk = world
        .create_body(
            &compound,
            &BodySettings::new_static().position(RVec3::new(10.0, 0.0, 0.0)),
        )
        .unwrap();

    let structure_only = QueryFilter::new().child_groups(1 << Groups::STRUCTURE);
    for filter in [ALL, structure_only] {
        let hit = world
            .cast_ray(down_from(0.1, 0.2, 0.0, 5.0), &filter)
            .unwrap()
            .expect("the box is hit");
        assert_eq!(hit.body, block);
        assert_eq!(hit.fraction, 0.0);
        assert_eq!(hit.distance, 0.0);

        let hit = world
            .cast_ray(down_from(10.1, 0.2, 0.0, 5.0), &filter)
            .unwrap()
            .expect("the compound child is hit");
        assert_eq!(hit.body, chunk);
        assert_eq!(hit.fraction, 0.0);
        assert_eq!(
            hit.compound_child.map(|c| c.user_data),
            Some(Groups::STRUCTURE)
        );
    }
}

#[test]
fn spawn_ground_ignores_canopy_by_group() {
    let (mut world, [terrain, chunk, ..]) = five_layer_world();
    let porch = Shape::new_box(Vec3::new(2.0, 0.5, 2.0)).unwrap();
    let canopy = Shape::new_cylinder(0.5, 2.0).unwrap();
    let house = Shape::new_compound(&[
        child(&porch, Vec3::new(0.0, 0.5, 0.0), Groups::STRUCTURE),
        child(&canopy, Vec3::new(0.0, 3.0, 0.0), Groups::FEATURE),
    ])
    .unwrap();
    let house_body = add_static_in(&mut world, &house, RVec3::ZERO, chunk);
    add_static_in(&mut world, &flat_height_field(), RVec3::ZERO, terrain);

    let ray = down_from(0.0, 5.0, 0.0, 55.0);
    let layers = [terrain, chunk];
    let ground = QueryFilter::new()
        .object_layers(&layers)
        .child_groups(1 << Groups::TERRAIN | 1 << Groups::STRUCTURE);
    let hit = world.cast_ray(ray, &ground).unwrap().expect("ground");
    let y = ray.point_at(hit.fraction).y;
    assert!((y - 1.0).abs() <= 0.01, "porch top expected, hit at {y}");
    assert_eq!(hit.body, house_body);
    assert_eq!(
        hit.compound_child.map(|c| c.user_data),
        Some(Groups::STRUCTURE)
    );

    let hit = world.cast_ray(ray, &ALL).unwrap().expect("canopy");
    let y = ray.point_at(hit.fraction).y;
    assert!((y - 3.5).abs() <= 0.01, "canopy top expected, hit at {y}");
    assert_eq!(
        hit.compound_child.map(|c| c.user_data),
        Some(Groups::FEATURE)
    );
}

#[test]
fn group_filter_keeps_root_call_for_power_of_two_children() {
    let mut world = world(Vec3::ZERO, 1);
    let unit_box = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
    let compound = Shape::new_compound(&[
        child(&unit_box, Vec3::new(0.0, 0.0, 0.0), Groups::STRUCTURE),
        child(&unit_box, Vec3::new(2.0, 0.0, 0.0), Groups::STRUCTURE),
        child(&unit_box, Vec3::new(4.0, 0.0, 0.0), Groups::STRUCTURE),
        child(&unit_box, Vec3::new(6.0, 0.0, 0.0), Groups::FEATURE),
    ])
    .unwrap();
    world
        .create_body(&compound, &BodySettings::new_static())
        .unwrap();
    let structure = QueryFilter::new().child_groups(1 << Groups::STRUCTURE);
    let hit = world
        .cast_ray(down_from(0.0, 5.0, 0.0, 10.0), &structure)
        .unwrap()
        .expect("child 0 is hit");
    assert_eq!(
        hit.compound_child,
        Some(CompoundSubShape {
            index: 0,
            user_data: Groups::STRUCTURE
        })
    );
    assert_eq!(
        world.cast_ray(down_from(6.0, 5.0, 0.0, 10.0), &structure),
        Ok(None)
    );
}

#[test]
fn group_filter_on_one_child_and_nested_compounds() {
    let mut world = world(Vec3::ZERO, 1);
    let unit_box = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
    let single = Shape::new_compound(&[child(&unit_box, Vec3::ZERO, Groups::STRUCTURE)]).unwrap();
    world
        .create_body(&single, &BodySettings::new_static())
        .unwrap();
    let structure = QueryFilter::new().child_groups(1 << Groups::STRUCTURE);
    let feature = QueryFilter::new().child_groups(1 << Groups::FEATURE);
    let ray = down_from(0.0, 5.0, 0.0, 10.0);
    assert!(world.cast_ray(ray, &structure).unwrap().is_some());
    assert_eq!(world.cast_ray(ray, &feature), Ok(None));

    let inner = Shape::new_compound(&[
        child(&unit_box, Vec3::new(-0.5, 0.0, 0.0), Groups::FEATURE),
        child(&unit_box, Vec3::new(0.5, 0.0, 0.0), Groups::FEATURE),
    ])
    .unwrap();
    let outer = Shape::new_compound(&[
        child(&inner, Vec3::ZERO, Groups::STRUCTURE),
        child(&unit_box, Vec3::new(4.0, 0.0, 0.0), Groups::FEATURE),
    ])
    .unwrap();
    world
        .create_body(
            &outer,
            &BodySettings::new_static().position(RVec3::new(20.0, 0.0, 0.0)),
        )
        .unwrap();
    let hit = world
        .cast_ray(down_from(20.5, 5.0, 0.0, 10.0), &structure)
        .unwrap()
        .expect("the top-level group governs the nested children");
    assert_eq!(
        hit.compound_child,
        Some(CompoundSubShape {
            index: 0,
            user_data: Groups::STRUCTURE
        })
    );
    assert_eq!(
        world.cast_ray(down_from(20.5, 5.0, 0.0, 10.0), &feature),
        Ok(None)
    );
    assert!(world
        .cast_ray(down_from(24.0, 5.0, 0.0, 10.0), &feature)
        .unwrap()
        .is_some());
}

#[test]
fn object_layer_selection_skips_other_layers() {
    let (mut world, [terrain, chunk, feature, ..]) = five_layer_world();
    let slab = Shape::new_box(Vec3::new(1.0, 0.25, 1.0)).unwrap();
    let upper = add_static_in(&mut world, &slab, RVec3::new(0.0, 3.0, 0.0), feature);
    let lower = add_static_in(&mut world, &slab, RVec3::new(0.0, 1.0, 0.0), chunk);
    let ray = down_from(0.0, 10.0, 0.0, 20.0);

    let hit = world.cast_ray(ray, &ALL).unwrap().unwrap();
    assert_eq!((hit.body, hit.object_layer), (upper, feature));
    let chunk_only = [chunk];
    let hit = world
        .cast_ray(ray, &QueryFilter::new().object_layers(&chunk_only))
        .unwrap()
        .unwrap();
    assert_eq!((hit.body, hit.object_layer), (lower, chunk));
    let terrain_only = [terrain];
    assert_eq!(
        world.cast_ray(ray, &QueryFilter::new().object_layers(&terrain_only)),
        Ok(None)
    );
    assert_eq!(
        world.cast_ray(ray, &QueryFilter::new().object_layers(&[])),
        Ok(None)
    );
}

#[test]
fn excluded_body_is_skipped() {
    let (mut world, [terrain, .., actor]) = five_layer_world();
    let floor_shape = Shape::new_box(Vec3::new(10.0, 1.0, 10.0)).unwrap();
    let floor = add_static_in(
        &mut world,
        &floor_shape,
        RVec3::new(0.0, -1.0, 0.0),
        terrain,
    );
    let capsule = Shape::new_capsule(0.70845, 0.4).unwrap();
    let own = world
        .create_body(
            &capsule,
            &BodySettings::new_kinematic()
                .position(RVec3::new(0.0, 1.2, 0.0))
                .object_layer(actor),
        )
        .unwrap();
    let ray = down_from(0.0, 1.2, 0.0, 10.0);

    let hit = world.cast_ray(ray, &ALL).unwrap().unwrap();
    assert_eq!((hit.body, hit.fraction), (own, 0.0));
    let hit = world
        .cast_ray(ray, &QueryFilter::new().exclude_body(own))
        .unwrap()
        .unwrap();
    assert_eq!(hit.body, floor);
    assert!((hit.distance - 1.2).abs() <= 1.0e-5, "{hit:?}");
}

#[test]
fn filter_rejects_unknown_layer_and_foreign_body() {
    let (world, _) = five_layer_world();
    let mut other = common::world(Vec3::ZERO, 1);
    let foreign = add_floor(&mut other);
    let ray = down_from(0.0, 5.0, 0.0, 10.0);
    let unknown = [ObjectLayer::new(5)];
    for filter in [
        QueryFilter::new().object_layers(&unknown),
        QueryFilter::new().exclude_body(foreign),
    ] {
        assert!(matches!(
            world.cast_ray(ray, &filter),
            Err(QueryError::InvalidValue(_))
        ));
    }
}

#[test]
fn ray_normals_point_out_of_the_hit_surface() {
    let mut world = world(Vec3::ZERO, 1);
    add_floor(&mut world);
    let wall_shape = Shape::new_box(Vec3::new(0.5, 2.0, 2.0)).unwrap();
    world
        .create_body(
            &wall_shape,
            &BodySettings::new_static().position(RVec3::new(0.5, 2.0, 0.0)),
        )
        .unwrap();
    let terrain = world
        .create_body(
            &flat_height_field(),
            &BodySettings::new_static().position(RVec3::new(300.0, 0.0, 0.0)),
        )
        .unwrap();

    let hit = world
        .cast_ray(down_from(-5.0, 5.0, 0.0, 10.0), &ALL)
        .unwrap()
        .unwrap();
    assert!(hit.normal.y > 0.99, "floor: {hit:?}");

    let from_minus_x = RayCast::new(RVec3::new(-5.0, 1.0, 0.0), Vec3::new(10.0, 0.0, 0.0));
    let hit = world.cast_ray(from_minus_x, &ALL).unwrap().unwrap();
    assert!(hit.normal.x < -0.99, "wall: {hit:?}");

    let up = Vec3::new(0.0, 10.0, 0.0);
    let from_below = RayCast::new(RVec3::new(300.5, -5.0, 0.5), up);
    let hit = world.cast_ray(from_below, &ALL).unwrap().unwrap();
    assert_eq!(hit.body, terrain);
    assert!(dot(hit.normal, up) > 0.0, "heightfield underside: {hit:?}");
}

#[test]
fn queries_see_created_moved_and_removed_bodies_without_a_step() {
    let mut world = world(Vec3::ZERO, 1);
    let unit_box = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
    let at_origin = down_from(0.0, 5.0, 0.0, 10.0);
    let at_ten = down_from(10.0, 5.0, 0.0, 10.0);
    assert_eq!(world.cast_ray(at_origin, &ALL), Ok(None));

    let body = world
        .create_body(&unit_box, &BodySettings::new_static())
        .unwrap();
    assert_eq!(world.cast_ray(at_origin, &ALL).unwrap().unwrap().body, body);

    world
        .body_mut(body)
        .unwrap()
        .set_position(RVec3::new(10.0, 0.0, 0.0), Activation::DontActivate)
        .unwrap();
    assert_eq!(world.cast_ray(at_origin, &ALL), Ok(None));
    assert_eq!(world.cast_ray(at_ten, &ALL).unwrap().unwrap().body, body);

    world.remove_body(body).unwrap();
    assert_eq!(world.cast_ray(at_ten, &ALL), Ok(None));
}
