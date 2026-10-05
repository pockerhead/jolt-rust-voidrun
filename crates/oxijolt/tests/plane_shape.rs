//! Plane shapes: validation, the bodies and uses that refuse them, and what collides with them.

mod common;

use common::events::add_cloth;
use common::ragdoll::{humanoid_parts, part_shapes, skeleton};
use common::*;
use oxijolt::*;

const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);
const UP: Vec3 = Vec3::new(0.0, 1.0, 0.0);

fn ground(half_extent: f32) -> Shape {
    Shape::new_plane(UP, 0.0, half_extent).unwrap()
}

/// A world with a static plane floor at y = 0 of half extent 50 m.
fn plane_world() -> (PhysicsWorld, BodyId) {
    let mut world = world(GRAVITY, 1);
    let floor = world
        .create_body(&ground(50.0), &BodySettings::new_static())
        .unwrap();
    (world, floor)
}

#[test]
fn plane_inputs_are_validated() {
    let invalid_dimensions = [
        (UP, 2000.5, 1.0),
        (UP, f32::NAN, 1.0),
        (UP, f32::INFINITY, 1.0),
        (UP, 0.0, 0.0),
        (UP, 0.0, -1.0),
        (UP, 0.0, f32::NAN),
        (UP, 0.0, 2000.5),
        // With normal +Y the bounds reach from -c - h to -c along y.
        (UP, 1.0, 2000.0),
    ];
    for (normal, constant, half_extent) in invalid_dimensions {
        assert!(
            matches!(
                Shape::new_plane(normal, constant, half_extent),
                Err(ShapeError::InvalidDimensions(_))
            ),
            "{normal:?} {constant} {half_extent}"
        );
    }
    for normal in [
        Vec3::ZERO,
        Vec3::new(0.0, 2.0, 0.0),
        Vec3::new(0.0, 0.999, 0.0),
        Vec3::new(f32::NAN, 1.0, 0.0),
    ] {
        assert!(
            matches!(
                Shape::new_plane(normal, 0.0, 1.0),
                Err(ShapeError::InvalidSettings(_))
            ),
            "{normal:?}"
        );
    }
    let material = PhysicsMaterial::new(7).unwrap();
    assert!(matches!(
        Shape::new_plane_with_material(UP, 0.0, 0.0, &material),
        Err(ShapeError::InvalidDimensions(_))
    ));

    // The plane at y = 1 with a half extent of 2000 m reaches down to y = -1999; the bounds
    // themselves are checked in the crate's unit tests.
    Shape::new_plane(UP, -1.0, 2000.0).unwrap();
    Shape::new_plane(Vec3::new(0.0, -1.0, 0.0), 1.0, 1999.0).unwrap();
    assert!(matches!(
        Shape::new_plane(Vec3::new(0.0, -1.0, 0.0), 1.0, 2000.0),
        Err(ShapeError::InvalidDimensions(_))
    ));
}

#[test]
fn a_tilted_plane_is_checked_against_jolts_bounds() {
    // A 45 degree normal about z: the square's corners and its depth reach h * sqrt 2 along x
    // and y.
    let s = std::f32::consts::FRAC_1_SQRT_2;
    let normal = Vec3::new(s, s, 0.0);
    let largest = 2000.0 * s;
    Shape::new_plane(normal, 0.0, largest * (1.0 - 1.0e-5)).unwrap();
    assert!(matches!(
        Shape::new_plane(normal, 0.0, largest * (1.0 + 1.0e-5)),
        Err(ShapeError::InvalidDimensions(_))
    ));
}

#[test]
fn only_static_bodies_take_a_plane() {
    let mut world = world(GRAVITY, 1);
    let plane = ground(10.0);
    let cube = cube_shape();
    let compound = Shape::new_compound(&[
        CompoundChild {
            shape: &plane,
            position: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            user_data: 0,
        },
        CompoundChild {
            shape: &cube,
            position: Vec3::new(1.0, 0.5, 0.0),
            rotation: Quat::IDENTITY,
            user_data: 1,
        },
    ])
    .unwrap();
    let offset = Shape::new_offset_center_of_mass(&plane, Vec3::new(0.0, -0.5, 0.0)).unwrap();
    let refused = [
        BodySettings::new_dynamic(),
        BodySettings::new_kinematic(),
        BodySettings::new_kinematic().mass(1.0),
        BodySettings::new_static()
            .object_layer(ObjectLayer::MOVING)
            .allow_dynamic_or_kinematic(true),
        BodySettings::new_static().sensor(true),
    ];
    for shape in [&plane, &compound, &offset] {
        for settings in &refused {
            assert!(
                matches!(
                    world.create_body(shape, settings),
                    Err(BodyError::InvalidValue(_))
                ),
                "{settings:?}"
            );
        }
        world
            .create_body(shape, &BodySettings::new_static())
            .unwrap();
    }
    assert_eq!(world.body_count(), 3);

    let dynamic = add_cube(&mut world, RVec3::new(0.0, 5.0, 0.0));
    let kinematic = world
        .create_body(&cube, &BodySettings::new_kinematic())
        .unwrap();
    for id in [dynamic, kinematic] {
        for shape in [&plane, &compound, &offset] {
            assert!(matches!(
                world
                    .body_mut(id)
                    .unwrap()
                    .set_shape(shape, Some(1.0), Activation::Activate),
                Err(BodyError::InvalidValue(_))
            ));
        }
    }
}

#[test]
fn planes_are_refused_where_shapes_must_move_or_query() {
    let mut world = world(GRAVITY, 1);
    let plane = ground(10.0);
    assert!(matches!(
        Shape::scaled(&plane, Vec3::new(2.0, 2.0, 2.0)),
        Err(ShapeError::InvalidSettings(_))
    ));

    let capsule = Shape::new_capsule(0.5, 0.3).unwrap();
    let inner = InnerBody {
        shape: &plane,
        object_layer: ObjectLayer::MOVING,
    };
    for settings in [
        CharacterSettings::new(&plane),
        CharacterSettings::new(&capsule).inner_body(Some(inner)),
    ] {
        assert!(matches!(
            world.create_character(&settings, RVec3::ZERO, Quat::IDENTITY),
            Err(CharacterError::InvalidValue(_))
        ));
    }

    let filter = QueryFilter::new();
    assert!(matches!(
        world.collide_shape(
            &CollideShape::new(&plane, RVec3::ZERO, Quat::IDENTITY),
            &filter
        ),
        Err(QueryError::InvalidValue(_))
    ));
    assert!(matches!(
        world.cast_shape(
            &ShapeCast::new(&plane, RVec3::ZERO, Quat::IDENTITY, UP),
            &filter
        ),
        Err(QueryError::InvalidValue(_))
    ));

    let mut shapes = part_shapes();
    shapes[3] = ground(10.0);
    assert!(matches!(
        RagdollSettings::new(&skeleton(), &humanoid_parts(&shapes, ObjectLayer::MOVING)),
        Err(RagdollError::InvalidValue(_))
    ));
}

/// The number of pending contact-cache invalidations a state holds, read from its `Debug`.
fn pending_in(state: &WorldState) -> String {
    let debug = format!("{state:?}");
    let start = debug
        .find("cache_invalidations: ")
        .expect("listed by Debug");
    debug[start..]
        .split([',', ' ', '}'])
        .nth(1)
        .unwrap()
        .to_owned()
}

#[test]
fn a_static_body_may_change_to_a_plane() {
    let mut world = world(GRAVITY, 1);
    world.set_event_settings(EventSettings::default().body_activation(true));
    let slab = Shape::new_box(Vec3::new(5.0, 0.5, 5.0)).unwrap();
    let floor = world
        .create_body(
            &slab,
            &BodySettings::new_static().position(RVec3::new(0.0, -0.5, 0.0)),
        )
        .unwrap();
    let cubes = [
        add_cube(&mut world, RVec3::new(-1.5, 0.5, 0.0)),
        add_cube(&mut world, RVec3::new(1.5, 0.5, 0.0)),
    ];
    for _ in 0..600 {
        if cubes
            .iter()
            .all(|&id| world.body(id).unwrap().is_sleeping())
        {
            break;
        }
        step(&mut world, 1);
    }
    assert!(cubes
        .iter()
        .all(|&id| world.body(id).unwrap().is_sleeping()));
    world.take_events();

    // A refused replacement keeps an earlier state restorable.
    let saved = world.save_state();
    let plane = Shape::new_plane(UP, -0.5, 20.0).unwrap();
    assert!(world
        .body_mut(cubes[0])
        .unwrap()
        .set_shape(&plane, Some(1.0), Activation::Activate)
        .is_err());
    world.restore_state(&saved).unwrap();

    // The plane's surface sits where the slab's top was (y = 0 in the world).
    world
        .body_mut(floor)
        .unwrap()
        .set_shape(&plane, None, Activation::Activate)
        .unwrap();
    assert_eq!(world.restore_state(&saved), Err(StateError::WorldChanged));
    assert_eq!(pending_in(&world.save_state()), "1");
    assert_eq!(
        world.take_events().activations,
        cubes.map(ActivationEvent::Activated)
    );
    step(&mut world, 60);
    for id in cubes {
        let y = world.body(id).unwrap().position().y;
        assert!((0.48..=0.501).contains(&y), "{y}");
    }
}

/// Where a body of `shape` dropped from `y` at x = z = 0 onto the plane floor is after 2 s.
fn dropped(shape: &Shape, y: Real) -> RVec3 {
    let (mut world, _) = plane_world();
    let id = world
        .create_body(
            shape,
            &BodySettings::new_dynamic().position(RVec3::new(0.0, y, 0.0)),
        )
        .unwrap();
    step(&mut world, 120);
    world.body(id).unwrap().position()
}

#[test]
fn bodies_rest_on_a_plane() {
    let resting = [
        (cube_shape(), 0.5),
        (Shape::new_sphere(0.3).unwrap(), 0.3),
        // Dropped upright, a capsule lands on its end.
        (Shape::new_capsule(0.5, 0.25).unwrap(), 0.75),
    ];
    for (shape, height) in &resting {
        let y = dropped(shape, 2.0).y;
        assert!(
            (height - 0.02..=height + 0.001).contains(&y),
            "{height}: {y}"
        );
    }
}

#[test]
fn a_box_slides_down_a_tilted_plane() {
    // A 20 degree slope falling toward +x: steeper than Jolt's default friction of 0.2 holds.
    let angle: f32 = 20.0_f32.to_radians();
    let normal = Vec3::new(angle.sin(), angle.cos(), 0.0);
    let mut world = world(GRAVITY, 1);
    world
        .create_body(
            &Shape::new_plane(normal, 0.0, 50.0).unwrap(),
            &BodySettings::new_static(),
        )
        .unwrap();
    let tilt = quat_about(Vec3::new(0.0, 0.0, 1.0), -angle);
    let start = RVec3::new(
        Real::from(0.52 * angle.sin()),
        Real::from(0.52 * angle.cos()),
        0.0,
    );
    let id = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_dynamic().position(start).rotation(tilt),
        )
        .unwrap();
    step(&mut world, 60);
    let p = world.body(id).unwrap().position();
    // Jolt's default friction of 0.2 leaves about 1.5 m/s^2 along the slope: 0.75 m in 1 s.
    assert!(p.x > start.x + 0.5, "{p:?}");
    // Still on the slope: the centre stays about half a box above the surface.
    let height = Real::from(normal.x) * p.x + Real::from(normal.y) * p.y;
    assert!((height - 0.5).abs() < 0.03, "{height}");
}

#[test]
fn a_box_outside_the_half_extent_falls_past() {
    let (mut world, _) = plane_world();
    let id = add_cube(&mut world, RVec3::new(60.0, 1.0, 0.0));
    step(&mut world, 120);
    assert!(world.body(id).unwrap().position().y < -5.0);
}

#[test]
fn a_compound_with_a_plane_child_catches_a_body() {
    let mut world = world(GRAVITY, 1);
    let plane = ground(10.0);
    let post = Shape::new_box(Vec3::new(0.5, 1.0, 0.5)).unwrap();
    let compound = Shape::new_compound(&[
        CompoundChild {
            shape: &plane,
            position: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            user_data: 0,
        },
        CompoundChild {
            shape: &post,
            position: Vec3::new(4.0, 1.0, 0.0),
            rotation: Quat::IDENTITY,
            user_data: 1,
        },
    ])
    .unwrap();
    world
        .create_body(&compound, &BodySettings::new_static())
        .unwrap();
    let on_plane = add_cube(&mut world, RVec3::new(0.0, 2.0, 0.0));
    let on_post = add_cube(&mut world, RVec3::new(4.0, 3.5, 0.0));
    step(&mut world, 120);
    let y = world.body(on_plane).unwrap().position().y;
    assert!((0.48..=0.501).contains(&y), "{y}");
    let y = world.body(on_post).unwrap().position().y;
    assert!((2.479..=2.501).contains(&y), "{y}");
}

#[test]
fn cloth_lands_on_a_plane() {
    let (mut world, _) = plane_world();
    let cloth = add_cloth(&mut world, RVec3::new(0.0, 1.0, 0.0), Quat::IDENTITY);
    step(&mut world, 180);
    let lowest = world
        .soft_body(cloth)
        .unwrap()
        .vertices()
        .into_iter()
        .map(|vertex| vertex.position.y)
        .fold(Real::INFINITY, Real::min);
    assert!((-0.05..0.2).contains(&lowest), "{lowest}");
}

#[test]
fn a_character_stands_and_walks_on_a_plane() {
    let (mut world, _) = plane_world();
    let capsule = Shape::new_capsule(0.7, 0.4).unwrap();
    let settings = CharacterSettings::new(&capsule).shape_offset(Vec3::new(0.0, 1.1, 0.0));
    let id = world
        .create_character(&settings, RVec3::new(0.0, 0.05, 0.0), Quat::IDENTITY)
        .unwrap();
    let filter = QueryFilter::new();
    let extended = ExtendedUpdateSettings::default()
        .stick_to_floor_step_down(Vec3::new(0.0, -0.5, 0.0))
        .walk_stairs_step_up(Vec3::new(0.0, 0.4, 0.0));
    let walk = |world: &mut PhysicsWorld, velocity: Vec3, ticks| {
        for _ in 0..ticks {
            world
                .character_mut(id)
                .unwrap()
                .set_linear_velocity(velocity)
                .unwrap();
            world
                .update_character(id, DT, GRAVITY, &extended, &filter)
                .unwrap();
        }
    };
    walk(&mut world, Vec3::new(0.0, -1.0, 0.0), 30);
    let character = world.character(id).unwrap();
    assert_eq!(character.ground_state(), GroundState::OnGround);
    let normal = character.ground_normal();
    assert!((normal.y - 1.0).abs() < 1.0e-5, "{normal:?}");
    let start = character.position();

    walk(&mut world, Vec3::new(1.0, -1.0, 0.0), 60);
    let end = world.character(id).unwrap().position();
    assert!((end.x - start.x - 1.0).abs() < 0.05, "{start:?} {end:?}");
    assert!(end.y.abs() < 0.05, "{end:?}");
    assert_eq!(
        world.character(id).unwrap().ground_state(),
        GroundState::OnGround
    );
}

#[test]
fn rays_hit_a_plane_from_above_and_start_inside_it_from_below() {
    let (world, floor) = plane_world();
    let filter = QueryFilter::new();
    let down = RayCast::new(RVec3::new(1.0, 2.0, -3.0), Vec3::new(0.0, -4.0, 0.0));
    let hit = world.cast_ray(down, &filter).unwrap().unwrap();
    assert_eq!(hit.body, floor);
    assert_eq!(hit.fraction, 0.5);
    assert!((hit.normal.y - 1.0).abs() < 1.0e-6, "{:?}", hit.normal);
    let up = RayCast::new(RVec3::new(1.0, -1.0, -3.0), Vec3::new(0.0, 4.0, 0.0));
    let hit = world.cast_ray(up, &filter).unwrap().unwrap();
    assert_eq!((hit.body, hit.fraction), (floor, 0.0));
}

#[test]
fn contacts_report_the_planes_material() {
    let mut world = world(GRAVITY, 1);
    world.set_event_settings(EventSettings::default().contacts(true));
    let material = PhysicsMaterial::new(42).unwrap();
    let floor = world
        .create_body(
            &Shape::new_plane_with_material(UP, 0.0, 10.0, &material).unwrap(),
            &BodySettings::new_static(),
        )
        .unwrap();
    add_cube(&mut world, RVec3::new(0.0, 0.6, 0.0));
    step(&mut world, 10);
    let events = world.take_events();
    let added = events
        .contacts
        .iter()
        .find_map(|event| match event {
            ContactEvent::Added { manifold, .. } => Some(manifold),
            _ => None,
        })
        .expect("the cube lands");
    let floor_side = usize::from(added.pair.body2 == floor);
    assert_eq!(added.materials[floor_side], Some(42));
}
