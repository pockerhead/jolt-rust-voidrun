//! Scaled shapes as bodies: contacts, mirrored meshes, kinematic scaled meshes and the bodies
//! that may not use them.

mod common;

use common::meshes::*;
use common::*;
use oxijolt::*;

// `Real` is `f32` without the `double-precision` feature.
#[allow(clippy::unnecessary_cast)]
fn f32_of(value: Real) -> f32 {
    value as f32
}

#[test]
fn non_uniformly_scaled_hull_and_box_rest_on_a_mesh() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    world
        .create_body(&flat_grid(16, 1.0), &BodySettings::new_static())
        .unwrap();
    let hull =
        Shape::new_convex_hull_with_convex_radius(&box_corners(Vec3::new(0.5, 0.5, 0.5)), 0.05)
            .unwrap();
    let block = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
    // Shape and resting height of the body origin: half of the scaled y extent.
    let cases = [
        (
            Shape::new_scaled(&hull, Vec3::new(2.0, 0.6, 1.0)).unwrap(),
            0.3,
        ),
        (
            Shape::new_scaled(&block, Vec3::new(0.5, 1.6, 3.0)).unwrap(),
            0.8,
        ),
    ];
    let ids: Vec<BodyId> = cases
        .iter()
        .enumerate()
        .map(|(i, (shape, _))| {
            let position = RVec3::new(-3.0 + 6.0 * i as Real, 2.0, 0.0);
            let settings = BodySettings::new_dynamic().position(position);
            world.create_body(shape, &settings).unwrap()
        })
        .collect();
    step(&mut world, 240);
    for (id, (_, rest)) in ids.iter().zip(&cases) {
        let body = world.body(*id).unwrap();
        assert!(is_calm(&body), "{id:?}");
        let y = f32_of(body.position().y);
        assert!(
            y >= rest - 0.0201 && y < rest + 0.01,
            "{id:?}: {y} vs {rest}"
        );
    }
}

#[test]
fn scaled_mesh_with_a_far_centre_of_mass_carries_a_cube() {
    // A small mesh whose centre of mass sits 1950 m away once scaled: Jolt collides the mesh's
    // own coordinates and folds the offset into the transform, so the triangles keep their
    // precision and a cube rests on them as on the plain mesh.
    let (vertices, triangles) = grid(4, 0.5, |_, _| 0.0);
    let (mesh, _) = Shape::new_mesh(&vertices, &triangles).unwrap();
    let away = Shape::new_offset_center_of_mass(&mesh, Vec3::new(-1300.0, 0.0, 0.0)).unwrap();
    let ground = Shape::new_scaled(&away, Vec3::new(1.5, 1.0, 1.5)).unwrap();
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    world
        .create_body(&ground, &BodySettings::new_static())
        .unwrap();
    let cube = add_cube(&mut world, RVec3::new(0.2, 1.0, -0.1));
    step(&mut world, 180);
    let body = world.body(cube).unwrap();
    assert!(is_calm(&body));
    let y = f32_of(body.position().y);
    assert!((0.4799..0.51).contains(&y), "{y}");
}

#[test]
fn mirrored_mesh_faces_the_other_way() {
    let mirrored = Shape::new_scaled(&flat_grid(4, 1.0), Vec3::new(1.0, -1.0, 1.0)).unwrap();
    let mut world = world(Vec3::ZERO, 1);
    let ground = world
        .create_body(&mirrored, &BodySettings::new_static())
        .unwrap();
    let sphere = Shape::new_sphere(0.25).unwrap();
    let cast = |from: Real, direction: f32| {
        let cast = ShapeCast::new(
            &sphere,
            RVec3::new(0.3, from, 0.2),
            Quat::IDENTITY,
            Vec3::new(0.0, direction, 0.0),
        );
        world.cast_shape(&cast, &QueryFilter::new()).unwrap()
    };
    // Casts ignore back faces: the mirrored front faces down, so only a cast from below hits.
    let up = cast(-2.0, 4.0).expect("the mirrored front face is hit from below");
    assert_eq!(up.body, ground);
    assert!(up.normal.y < -0.99, "{up:?}");
    assert!((up.distance - 1.75).abs() < 1.0e-3, "{up:?}");
    assert_eq!(cast(2.0, -4.0), None);
}

#[test]
fn kinematic_scaled_mesh_carries_a_box() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    let platform_shape = Shape::new_scaled(&flat_grid(2, 1.0), Vec3::new(2.0, 2.0, 2.0)).unwrap();
    let platform = world
        .create_body(
            &platform_shape,
            &BodySettings::new_kinematic().mass(100.0).friction(0.8),
        )
        .unwrap();
    let cube = Shape::new_box(Vec3::new(0.25, 0.25, 0.25)).unwrap();
    let rider = world
        .create_body(
            &cube,
            &BodySettings::new_dynamic()
                .position(RVec3::new(0.0, 0.26, 0.0))
                .friction(0.8),
        )
        .unwrap();
    step(&mut world, 30);
    let start = world.body(rider).unwrap().position().x;
    for tick in 1..=120 {
        let speed = 0.5 * tick as f32 * DT;
        world
            .body_mut(platform)
            .unwrap()
            .set_linear_velocity(Vec3::new(speed, 0.0, 0.0))
            .unwrap();
        assert!(world.step(DT).unwrap().is_complete());
    }
    let moved = world.body(platform).unwrap().position().x;
    let carried = world.body(rider).unwrap().position().x - start;
    assert!(
        (carried - moved).abs() < 0.05,
        "box {carried} vs platform {moved}"
    );
}

#[test]
fn scaled_mesh_is_refused_as_dynamic() {
    let scaled = Shape::new_scaled(&flat_grid(2, 1.0), Vec3::new(2.0, 2.0, 2.0)).unwrap();
    let mut world = world(Vec3::ZERO, 1);
    assert_eq!(
        world.create_body(&scaled, &BodySettings::new_dynamic().mass(5.0)),
        Err(BodyError::InvalidValue(
            "mesh shapes cannot be used by dynamic bodies"
        ))
    );
}

/// Where a sphere dropped from 1 m onto a static scaled 5 x 5 heightfield ends after two
/// seconds.
fn sphere_on_scaled_field(scale: Vec3) -> f32 {
    let field = Shape::new_height_field(5, &[0.0; 25], &HeightFieldSettings::default()).unwrap();
    let scaled = Shape::new_scaled(&field, scale).unwrap();
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    world
        .create_body(&scaled, &BodySettings::new_static())
        .unwrap();
    let ball = Shape::new_sphere(0.3).unwrap();
    let id = world
        .create_body(
            &ball,
            &BodySettings::new_dynamic().position(RVec3::new(1.5, 1.0, 1.5)),
        )
        .unwrap();
    step(&mut world, 120);
    assert!(is_finite_body(&world, id), "{scale:?}");
    f32_of(world.body(id).unwrap().position().y)
}

#[test]
fn scaled_height_field_catches_a_falling_sphere() {
    let y = sphere_on_scaled_field(Vec3::new(2.0, 2.0, 2.0));
    assert!((0.27..0.32).contains(&y), "{y}");
}

#[test]
fn mirrored_height_field_faces_down() {
    // Mirroring in y turns the surface's front faces down; contacts ignore back faces, so a
    // sphere from above falls through.
    assert!(sphere_on_scaled_field(Vec3::new(1.0, -1.0, 1.0)) < -1.0);
}
