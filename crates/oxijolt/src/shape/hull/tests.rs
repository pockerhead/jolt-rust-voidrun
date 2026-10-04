use super::*;
use crate::material::shape_material;
use crate::{Quat, SubShapeId};

fn cube(half: f32) -> Vec<Vec3> {
    let mut points = Vec::new();
    for x in [-half, half] {
        for y in [-half, half] {
            for z in [-half, half] {
                points.push(Vec3::new(x, y, z));
            }
        }
    }
    points
}

fn point_count(shape: &Shape) -> u32 {
    // SAFETY: the shape is a live convex hull; the getter only reads it.
    unsafe { JPH_ConvexHullShape_GetNumPoints(shape.as_ptr().cast()) }
}

fn face_count(shape: &Shape) -> u32 {
    // SAFETY: the shape is a live convex hull; the getter only reads it.
    unsafe { JPH_ConvexHullShape_GetNumFaces(shape.as_ptr().cast()) }
}

fn hull_error(points: &[Vec3]) -> Option<HullError> {
    match Shape::new_convex_hull(points, 0.05) {
        Err(ShapeError::ConvexHull(error)) => Some(error),
        _ => None,
    }
}

/// Ten points on a circle of radius 1 in the plane through `origin` spanned by the rotated X
/// and Z axes.
fn plane(origin: Vec3, rotation: Quat) -> Vec<Vec3> {
    (0..10)
        .map(|i| {
            let angle = i as f32 * std::f32::consts::TAU / 10.0;
            let on_circle = rotation.rotate(Vec3::new(angle.cos(), 0.0, angle.sin()));
            Vec3::new(
                origin.x + on_circle.x,
                origin.y + on_circle.y,
                origin.z + on_circle.z,
            )
        })
        .collect()
}

#[test]
fn cube_corners_make_a_six_faced_hull() {
    let hull = Shape::new_convex_hull(&cube(0.5), 0.05).unwrap();
    assert_eq!(hull.sub_type(), JPH_ShapeSubType_ConvexHull);
    assert_eq!(point_count(&hull), 8);
    assert_eq!(face_count(&hull), 6);
}

#[test]
fn irregular_tetrahedron_and_duplicate_points_build() {
    let tetrahedron = [
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(1.3, 0.1, 0.0),
        Vec3::new(0.2, 0.9, 0.1),
        Vec3::new(0.3, 0.2, 1.7),
    ];
    let hull = Shape::new_convex_hull(&tetrahedron, 0.0).unwrap();
    assert_eq!(point_count(&hull), 4);
    assert_eq!(face_count(&hull), 4);

    let mut doubled = cube(1.0);
    doubled.extend(cube(1.0));
    doubled.push(Vec3::ZERO);
    let hull = Shape::new_convex_hull(&doubled, 0.05).unwrap();
    assert_eq!(point_count(&hull), 8);
}

#[test]
fn too_few_points_are_refused() {
    assert_eq!(hull_error(&[]), Some(HullError::TooFewPoints));
    assert_eq!(hull_error(&cube(1.0)[..3]), Some(HullError::TooFewPoints));
}

#[test]
fn points_without_a_triangle_are_degenerate() {
    let collinear: Vec<Vec3> = (0..5).map(|i| Vec3::new(i as f32, 0.0, 0.0)).collect();
    assert_eq!(hull_error(&collinear), Some(HullError::Degenerate));
    assert_eq!(
        hull_error(&[Vec3::new(1.0, 2.0, 3.0); 6]),
        Some(HullError::Degenerate)
    );
    let micro: Vec<Vec3> = [
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0e-6, 0.0, 0.0),
        Vec3::new(0.0, 1.0e-6, 0.0),
        Vec3::new(0.0, 0.0, 1.0e-6),
    ]
    .to_vec();
    assert_eq!(hull_error(&micro), Some(HullError::Degenerate));
}

#[test]
fn points_in_one_plane_are_coplanar() {
    // A rotation of 0.7 rad about the normalised axis (1, 2, 0.5).
    let (sin, cos) = 0.35f32.sin_cos();
    let axis = Vec3::new(1.0, 2.0, 0.5).scale(sin / 5.25f32.sqrt());
    let tilt = Quat::from_xyzw(axis.x, axis.y, axis.z, cos);
    for points in [
        plane(Vec3::ZERO, Quat::IDENTITY),
        plane(Vec3::ZERO, tilt),
        plane(Vec3::new(1000.0, -1000.0, 1000.0), Quat::IDENTITY),
    ] {
        assert_eq!(hull_error(&points), Some(HullError::Coplanar));
    }
}

#[test]
fn points_must_be_finite_and_within_the_extent() {
    for bad in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
        let mut points = cube(1.0);
        points[3].y = bad;
        assert!(matches!(
            Shape::new_convex_hull(&points, 0.05),
            Err(ShapeError::InvalidDimensions(_))
        ));
    }
    let max = limits::MAX_SHAPE_EXTENT;
    let hull = Shape::new_convex_hull(&cube(max), 0.05).unwrap();
    assert_eq!(point_count(&hull), 8);
    let mut beyond = cube(max);
    beyond[0].x = -max.next_up();
    assert!(matches!(
        Shape::new_convex_hull(&beyond, 0.05),
        Err(ShapeError::InvalidDimensions(_))
    ));
}

#[test]
fn convex_radius_must_be_finite_and_not_negative() {
    for bad in [-0.01, f32::NAN, f32::INFINITY] {
        assert!(matches!(
            Shape::new_convex_hull(&cube(1.0), bad),
            Err(ShapeError::InvalidDimensions(_))
        ));
    }
}

#[test]
fn hull_carries_its_material() {
    let material = PhysicsMaterial::new(33).unwrap();
    let hull = Shape::new_convex_hull_with_material(&cube(1.0), 0.05, &material).unwrap();
    drop(material);
    // SAFETY: the hull is live and the root id names the hull itself.
    let found = unsafe { shape_material(hull.as_ptr(), SubShapeId::new(u32::MAX)) };
    assert_eq!(found, Some(33));
}
