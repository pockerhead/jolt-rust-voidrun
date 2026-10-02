//! Shapes and the closest-hit ray.

mod common;

use common::*;
use joltphysics::*;

fn down_from(x: Real, y: Real, z: Real, length: f32) -> RayCast {
    RayCast::new(RVec3::new(x, y, z), Vec3::new(0.0, -length, 0.0))
}

#[test]
fn ray_hits_a_body_right_after_creation() {
    let mut world = world(Vec3::ZERO, 1);
    let floor = add_floor(&mut world);
    let hit = world
        .cast_ray(down_from(0.0, 5.0, 0.0, 10.0))
        .unwrap()
        .expect("the floor is hit");
    assert_eq!(hit.body, floor);
    assert!((hit.fraction - 0.5).abs() <= 1.0e-6, "{hit:?}");
}

#[test]
fn ray_misses_return_none() {
    let mut world = world(Vec3::ZERO, 1);
    add_floor(&mut world);
    let up = RayCast::new(RVec3::new(0.0, 5.0, 0.0), Vec3::new(0.0, 10.0, 0.0));
    assert_eq!(world.cast_ray(up), Ok(None));
}

#[test]
fn ray_starting_inside_a_box_hits_at_zero() {
    let mut world = world(Vec3::ZERO, 1);
    let floor = add_floor(&mut world);
    let hit = world
        .cast_ray(down_from(0.0, -0.5, 0.0, 10.0))
        .unwrap()
        .expect("the floor is hit");
    assert_eq!(hit.body, floor);
    assert_eq!(hit.fraction, 0.0);
}

#[test]
fn invalid_rays_are_rejected() {
    let world = world(Vec3::ZERO, 1);
    let invalid = [
        RayCast::new(RVec3::new(Real::NAN, 0.0, 0.0), Vec3::new(0.0, -1.0, 0.0)),
        RayCast::new(RVec3::new(0.0, 0.0, 0.0), Vec3::ZERO),
        RayCast::new(
            RVec3::new(0.0, 0.0, 0.0),
            Vec3::new(f32::INFINITY, 0.0, 0.0),
        ),
    ];
    for ray in invalid {
        assert!(
            matches!(world.cast_ray(ray), Err(QueryError::InvalidValue(_))),
            "{ray:?} was accepted"
        );
    }
}

/// Converts a column-major height grid (`heights_zx[j * n + i]` is the height at row `i` along
/// Z and column `j` along X) into Jolt's row-major samples (`samples[i * n + j]`).
fn column_major_to_jolt(heights_zx: &[f32], n: usize) -> Vec<f32> {
    let mut samples = vec![0.0; n * n];
    for i in 0..n {
        for j in 0..n {
            samples[i * n + j] = heights_zx[j * n + i];
        }
    }
    samples
}

fn add_static(world: &mut PhysicsWorld, shape: &Shape, position: RVec3) -> BodyId {
    world
        .create_body(shape, &BodySettings::new_static().position(position))
        .unwrap()
}

/// World height where a downward ray from `y = 20` at `(x, z)` hits, if it does.
fn surface_height(world: &PhysicsWorld, x: Real, z: Real) -> Option<Real> {
    let ray = down_from(x, 20.0, z, 40.0);
    let hit = world.cast_ray(ray).unwrap()?;
    Some(ray.point_at(hit.fraction).y)
}

#[test]
fn height_field_rising_along_z_matches_analytic_surface() {
    const N: usize = 33;
    // Column-major source: row `i` lies at local z = -16 + i and rises 0.2 m per metre.
    let mut heights_zx = vec![0.0; N * N];
    for i in 0..N {
        for j in 0..N {
            heights_zx[j * N + i] = 0.2 * i as f32;
        }
    }
    let samples = column_major_to_jolt(&heights_zx, N);
    let settings = HeightFieldSettings::default().offset(Vec3::new(-16.0, 0.0, -16.0));
    let shape = Shape::new_height_field(N as u32, &samples, &settings).unwrap();

    let mut world = world(Vec3::ZERO, 1);
    let origin = RVec3::new(100.0, -3.0, 50.0);
    let terrain = add_static(&mut world, &shape, origin);

    // Every point has |x - z| >= 1, so a mapping without the transpose is off by 0.2 m or more.
    let points: [(Real, Real); 5] = [
        (-10.3, -12.7),
        (3.25, 0.5),
        (12.9, 9.1),
        (-0.4, 14.6),
        (7.7, -5.2),
    ];
    for (x, z) in points {
        let ray = RayCast::new(
            RVec3::new(origin.x + x, 17.0, origin.z + z),
            Vec3::new(0.0, -30.0, 0.0),
        );
        let hit = world.cast_ray(ray).unwrap().expect("the terrain is hit");
        assert_eq!(hit.body, terrain);
        let expected = -3.0 + 0.2 * (z + 16.0);
        let actual = ray.point_at(hit.fraction).y;
        assert!(
            (actual - expected).abs() <= 0.02,
            "at ({x}, {z}): hit {actual}, expected {expected}"
        );
    }
}

#[test]
fn height_field_33_builds_and_matches_samples_at_nodes() {
    const N: u32 = 33;
    let height = |x: u32, y: u32| {
        let (x, y) = (x as f32, y as f32);
        1.5 * (0.4 * x).sin() * (0.3 * y).cos() + 0.05 * y
    };
    let samples: Vec<f32> = (0..N)
        .flat_map(|y| (0..N).map(move |x| height(x, y)))
        .collect();
    let settings = HeightFieldSettings::default()
        .bits_per_sample(16)
        .scale(Vec3::new(0.5, 1.0, 0.5));
    let shape = Shape::new_height_field(N, &samples, &settings).unwrap();

    for y in 0..N {
        for x in 0..N {
            let p = shape.height_field_position(x, y).expect("a stored node");
            assert!(
                (p.y - height(x, y)).abs() <= 1.0e-4,
                "node ({x}, {y}): {} vs {}",
                p.y,
                height(x, y)
            );
            assert_eq!(p.x, 0.5 * x as f32);
            assert_eq!(p.z, 0.5 * y as f32);
        }
    }
    // n = 33 is padded to 34 with holes; 34 is outside the stored grid.
    assert_eq!(shape.height_field_position(33, 0), None);
    assert_eq!(shape.height_field_position(0, 33), None);
    assert_eq!(shape.height_field_position(34, 0), None);

    let mut world = world(Vec3::ZERO, 1);
    add_static(&mut world, &shape, RVec3::new(0.0, 0.0, 0.0));
    assert!(
        surface_height(&world, 15.75, 15.75).is_some(),
        "cell (31, 31)"
    );
    assert!(surface_height(&world, 16.01, 8.0).is_none(), "padding cell");
}

#[test]
fn height_field_cell_diagonal_runs_from_x_y_to_x1_y1() {
    let mut samples = vec![0.0; 9];
    samples[3 + 1] = 1.0;
    let settings = HeightFieldSettings::default().bits_per_sample(16);
    let shape = Shape::new_height_field(3, &samples, &settings).unwrap();
    let mut world = world(Vec3::ZERO, 1);
    add_static(&mut world, &shape, RVec3::new(0.0, 0.0, 0.0));
    // Along the other diagonal both points would lie on triangles of height 0.
    for (x, z) in [(0.75, 0.25), (0.25, 0.75)] {
        let y = surface_height(&world, x, z).expect("the cell is hit");
        assert!((y - 0.25).abs() <= 1.0e-3, "at ({x}, {z}): {y}");
    }
}

#[test]
fn height_field_is_hit_from_below() {
    let shape = Shape::new_height_field(3, &[0.0; 9], &HeightFieldSettings::default()).unwrap();
    let mut world = world(Vec3::ZERO, 1);
    let terrain = add_static(&mut world, &shape, RVec3::new(0.0, 0.0, 0.0));
    let ray = RayCast::new(RVec3::new(0.5, -1.0, 0.5), Vec3::new(0.0, 2.0, 0.0));
    let hit = world.cast_ray(ray).unwrap().expect("the underside is hit");
    assert_eq!(hit.body, terrain);
    assert!(ray.point_at(hit.fraction).y.abs() <= 1.0e-3);
}

#[test]
fn invalid_height_fields_are_rejected() {
    let defaults = HeightFieldSettings::default;
    let flat = vec![0.0; 9];
    let with_samples = |changes: &[(usize, f32)]| {
        let mut samples = flat.clone();
        for &(index, value) in changes {
            samples[index] = value;
        }
        samples
    };
    let dimension_cases = [
        (1, vec![0.0], defaults()),
        (3, vec![0.0; 8], defaults()),
        (3, with_samples(&[(4, f32::NAN)]), defaults()),
        (3, with_samples(&[(4, f32::INFINITY)]), defaults()),
        (3, with_samples(&[(0, -3.0e38), (4, 3.0e38)]), defaults()),
        (3, flat.clone(), defaults().scale(Vec3::new(0.0, 1.0, 1.0))),
    ];
    for (count, samples, settings) in dimension_cases {
        let result = Shape::new_height_field(count, &samples, &settings);
        assert!(
            matches!(result, Err(ShapeError::InvalidDimensions(_))),
            "n = {count}, {settings:?}"
        );
    }
    let settings_cases = [
        (3, flat.clone(), defaults().block_size(1)),
        (3, flat.clone(), defaults().block_size(9)),
        (3, flat.clone(), defaults().bits_per_sample(0)),
        (3, flat.clone(), defaults().bits_per_sample(17)),
        (
            3,
            flat.clone(),
            defaults().active_edge_cos_threshold_angle(-0.1),
        ),
        (
            3,
            flat.clone(),
            defaults().active_edge_cos_threshold_angle(1.5),
        ),
        (
            3,
            flat.clone(),
            defaults().active_edge_cos_threshold_angle(f32::NAN),
        ),
        (2, vec![0.0; 4], defaults()),
        // The size check runs before the length check, so no huge buffer is needed.
        (40000, Vec::new(), defaults()),
    ];
    for (count, samples, settings) in settings_cases {
        let result = Shape::new_height_field(count, &samples, &settings);
        assert!(
            matches!(result, Err(ShapeError::InvalidSettings(_))),
            "n = {count}, {settings:?}"
        );
    }
}

#[test]
fn height_field_of_holes_has_no_collision() {
    let shape =
        Shape::new_height_field(3, &[f32::MAX; 9], &HeightFieldSettings::default()).unwrap();
    let mut world = world(Vec3::ZERO, 1);
    add_static(&mut world, &shape, RVec3::new(0.0, 0.0, 0.0));
    assert_eq!(surface_height(&world, 1.0, 1.0), None);
    assert_eq!(shape.height_field_position(1, 1), None);
}

#[test]
fn static_only_shapes_are_rejected_for_moving_bodies() {
    let shape = Shape::new_height_field(3, &[0.0; 9], &HeightFieldSettings::default()).unwrap();
    let mut world = world(Vec3::ZERO, 1);
    for settings in [BodySettings::new_dynamic(), BodySettings::new_kinematic()] {
        assert!(matches!(
            world.create_body(&shape, &settings),
            Err(BodyError::InvalidValue(_))
        ));
    }
    assert_eq!(world.body_count(), 0);
    assert!(world
        .create_body(&shape, &BodySettings::new_static())
        .is_ok());
}

#[test]
fn height_field_with_custom_active_edge_threshold_builds() {
    const N: u32 = 9;
    let samples: Vec<f32> = (0..N)
        .flat_map(|y| (0..N).map(move |x| 0.1 * ((x + y) % 3) as f32))
        .collect();
    let settings = HeightFieldSettings::default()
        .offset(Vec3::new(-4.0, 0.0, -4.0))
        .active_edge_cos_threshold_angle(30.0_f32.to_radians().cos());
    let shape = Shape::new_height_field(N, &samples, &settings).unwrap();
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    add_static(&mut world, &shape, RVec3::new(0.0, 0.0, 0.0));
    let ball_shape = Shape::new_sphere(0.5).unwrap();
    let ball = world
        .create_body(
            &ball_shape,
            &BodySettings::new_dynamic()
                .position(RVec3::new(0.3, 2.0, -0.2))
                .enhanced_internal_edge_removal(true),
        )
        .unwrap();
    step(&mut world, 60);
    let body = world.body(ball).unwrap();
    let position: [Real; 3] = body.position().into();
    assert!(position.iter().all(|value| value.is_finite()));
    assert!(length(body.linear_velocity()).is_finite());
    assert!(
        position[1] > 0.3,
        "the ball stays above the surface: {position:?}"
    );
}
