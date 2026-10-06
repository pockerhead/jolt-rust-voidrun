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

fn hull_error(points: &[Vec3]) -> Option<ConvexHullError> {
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
    assert_eq!(hull_error(&[]), Some(ConvexHullError::TooFewPoints));
    assert_eq!(
        hull_error(&cube(1.0)[..3]),
        Some(ConvexHullError::TooFewPoints)
    );
}

#[test]
fn points_without_a_triangle_are_degenerate() {
    let collinear: Vec<Vec3> = (0..5).map(|i| Vec3::new(i as f32, 0.0, 0.0)).collect();
    assert_eq!(hull_error(&collinear), Some(ConvexHullError::Degenerate));
    assert_eq!(
        hull_error(&[Vec3::new(1.0, 2.0, 3.0); 6]),
        Some(ConvexHullError::Degenerate)
    );
    let micro: Vec<Vec3> = [
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0e-6, 0.0, 0.0),
        Vec3::new(0.0, 1.0e-6, 0.0),
        Vec3::new(0.0, 0.0, 1.0e-6),
    ]
    .to_vec();
    assert_eq!(hull_error(&micro), Some(ConvexHullError::Degenerate));
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
        assert_eq!(hull_error(&points), Some(ConvexHullError::Coplanar));
    }
}

#[test]
fn points_must_be_finite_and_within_the_extent() {
    for bad in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
        let mut points = cube(1.0);
        points[3].y = bad;
        assert!(matches!(
            Shape::new_convex_hull(&points, 0.05),
            Err(ShapeError::InvalidValue(_))
        ));
    }
    let max = limits::MAX_SHAPE_EXTENT;
    let hull = Shape::new_convex_hull(&cube(max), 0.05).unwrap();
    assert_eq!(point_count(&hull), 8);
    let mut beyond = cube(max);
    beyond[0].x = -max.next_up();
    assert!(matches!(
        Shape::new_convex_hull(&beyond, 0.05),
        Err(ShapeError::InvalidValue(_))
    ));
}

#[test]
fn convex_radius_must_be_finite_and_not_negative() {
    for bad in [-0.01, f32::NAN, f32::INFINITY] {
        assert!(matches!(
            Shape::new_convex_hull(&cube(1.0), bad),
            Err(ShapeError::InvalidValue(_))
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

fn simplex(width: f64, thickness: f64, coplanar_distance: f64) -> InitialSimplex {
    InitialSimplex {
        area_sq: 1.0,
        length: 10.0,
        width,
        thickness,
        coplanar_distance,
    }
}

#[test]
fn needle_rule_compares_the_rounding_lever_with_the_tolerance() {
    // Near the origin the tolerance is Jolt's 1 mm: the bound on the width is
    // 0.25 * length * coplanar / 1e-3.
    let coplanar = 1.0e-6;
    let bound = MIN_NEEDLE_LEVER * 10.0 * coplanar / HULL_TOLERANCE;
    assert!(simplex(bound, 1.0, coplanar).classify().is_ok());
    assert_eq!(
        simplex(bound * 0.999, 1.0, coplanar).classify(),
        Err(ShapeError::ConvexHull(ConvexHullError::Degenerate))
    );
    // Far out the coplanar distance exceeds the tolerance and the bound is a pure ratio.
    let coplanar = 2.0e-3;
    assert!(simplex(2.5, 100.0, coplanar).classify().is_ok());
    assert_eq!(
        simplex(2.49, 100.0, coplanar).classify(),
        Err(ShapeError::ConvexHull(ConvexHullError::Degenerate))
    );
}

#[test]
fn slab_rule_counts_coplanar_distances() {
    let coplanar = 1.0e-6;
    assert!(simplex(1.0, 200.0 * coplanar, coplanar).classify().is_ok());
    assert_eq!(
        simplex(1.0, 199.0 * coplanar, coplanar).classify(),
        Err(ShapeError::ConvexHull(ConvexHullError::Coplanar))
    );
}

#[test]
fn thin_slabs_near_the_origin_build_down_to_a_fraction_of_a_millimetre() {
    // A 2 m plank: 200 coplanar distances are about 0.14 mm.
    let plank = |thickness: f32| cube_with(Vec3::new(1.0, thickness / 2.0, 1.0));
    assert!(Shape::new_convex_hull(&plank(2.0e-4), 0.0).is_ok());
    assert_eq!(hull_error(&plank(1.0e-4)), Some(ConvexHullError::Coplanar));
}

#[test]
fn far_clouds_need_more_thickness() {
    // Near (-450, -1209, -1308) the coplanar distance is about 1.06 mm, so 200 of them are
    // about 0.21 m: a 2 m plank there must be thicker than that.
    let plank = |thickness: f32| -> Vec<Vec3> {
        cube_with(Vec3::new(1.0, thickness / 2.0, 1.0))
            .into_iter()
            .map(|p| Vec3::new(p.x - 450.0, p.y - 1209.0, p.z - 1308.0))
            .collect()
    };
    assert!(Shape::new_convex_hull(&plank(0.25), 0.0).is_ok());
    assert_eq!(hull_error(&plank(0.2)), Some(ConvexHullError::Coplanar));
}

fn cube_with(half: Vec3) -> Vec<Vec3> {
    cube(1.0)
        .into_iter()
        .map(|p| Vec3::new(p.x * half.x, p.y * half.y, p.z * half.z))
        .collect()
}
