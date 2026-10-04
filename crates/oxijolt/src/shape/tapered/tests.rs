use super::*;
use crate::Vec3;

/// The sub type Jolt gives a tapered cylinder (`EShapeSubType::TaperedCylinder`), which joltc's
/// `JPH_ShapeSubType` does not name.
const TAPERED_CYLINDER: i64 = 32;

fn dimensions(result: Result<Shape, ShapeError>) -> bool {
    matches!(result, Err(ShapeError::InvalidDimensions(_)))
}

#[test]
fn radius_floor_is_two_to_the_minus_63() {
    assert_eq!(MIN_TAPERED_CYLINDER_RADIUS, 2f32.powi(-63));
}

#[test]
fn tapered_capsule_reaches_jolt() {
    let shape = Shape::new_tapered_capsule(0.6, 0.2, 0.4).unwrap();
    assert_eq!(shape.sub_type(), JPH_ShapeSubType_TaperedCapsule);
    let capsule: *const JPH_TaperedCapsuleShape = shape.as_ptr().cast();
    // SAFETY: the shape is a live tapered capsule; the getters only read it.
    unsafe {
        assert_eq!(JPH_TaperedCapsuleShape_GetHalfHeight(capsule), 0.6);
        assert_eq!(JPH_TaperedCapsuleShape_GetTopRadius(capsule), 0.2);
        assert_eq!(JPH_TaperedCapsuleShape_GetBottomRadius(capsule), 0.4);
    }
}

#[test]
fn tapered_capsule_boundaries() {
    for bad in [0.0, -1.0, f32::NAN, f32::INFINITY] {
        assert!(dimensions(Shape::new_tapered_capsule(bad, 0.2, 0.3)));
        assert!(dimensions(Shape::new_tapered_capsule(1.0, bad, 0.3)));
        assert!(dimensions(Shape::new_tapered_capsule(1.0, 0.2, bad)));
    }
    let max = limits::MAX_SHAPE_EXTENT;
    assert!(Shape::new_tapered_capsule(max - 2.0, 1.0, 2.0).is_ok());
    assert!(dimensions(Shape::new_tapered_capsule(max - 1.0, 1.0, 2.0)));
    // Jolt's sphere predicate at h = 1, top = 0.5: bottom 2.5 makes a sphere.
    assert!(dimensions(Shape::new_tapered_capsule(1.0, 0.5, 2.5)));
    // The taper bound 2 * h * (1 - 2^-21) = 2 - 2^-20 is exact in f32 here.
    let at_bound = 2.5 - 2f32.powi(-20);
    assert_eq!(at_bound - 0.5, 2.0 - 2f32.powi(-20));
    assert!(Shape::new_tapered_capsule(1.0, 0.5, at_bound).is_ok());
    assert!(Shape::new_tapered_capsule(1.0, at_bound, 0.5).is_ok());
    assert!(dimensions(Shape::new_tapered_capsule(
        1.0,
        0.5,
        at_bound.next_up()
    )));
    assert!(dimensions(Shape::new_tapered_capsule(
        1.0,
        at_bound.next_up(),
        0.5
    )));
}

#[test]
fn tapered_cylinder_and_cone_reach_jolt() {
    let shape = Shape::new_tapered_cylinder(0.5, 0.2, 0.4, 0.05).unwrap();
    assert_eq!(shape.sub_type() as i64, TAPERED_CYLINDER);
    let cylinder: *const JPH_TaperedCylinderShape = shape.as_ptr().cast();
    // SAFETY: the shape is a live tapered cylinder; the getters only read it.
    unsafe {
        assert_eq!(JPH_TaperedCylinderShape_GetHalfHeight(cylinder), 0.5);
        assert_eq!(JPH_TaperedCylinderShape_GetTopRadius(cylinder), 0.2);
        assert_eq!(JPH_TaperedCylinderShape_GetBottomRadius(cylinder), 0.4);
        assert_eq!(JPH_TaperedCylinderShape_GetConvexRadius(cylinder), 0.05);
    }
    let cone = Shape::new_tapered_cylinder(0.5, 0.0, 0.4, 0.0).unwrap();
    assert_eq!(cone.sub_type() as i64, TAPERED_CYLINDER);
    // A cone's centre of mass is a quarter of the height above its base.
    let center = cone.center_of_mass();
    assert!((center.y - (0.25 - 0.5)).abs() < 1.0e-6, "{center:?}");
}

#[test]
fn tapered_cylinder_boundaries() {
    for bad in [0.0, -1.0, f32::NAN, f32::INFINITY] {
        assert!(dimensions(Shape::new_tapered_cylinder(bad, 0.2, 0.3, 0.0)));
    }
    for bad in [-0.1, f32::NAN, f32::INFINITY] {
        assert!(dimensions(Shape::new_tapered_cylinder(1.0, bad, 0.3, 0.0)));
        assert!(dimensions(Shape::new_tapered_cylinder(1.0, 0.2, bad, 0.0)));
        assert!(dimensions(Shape::new_tapered_cylinder(1.0, 0.2, 0.3, bad)));
    }
    assert!(dimensions(Shape::new_tapered_cylinder(1.0, 0.3, 0.3, 0.0)));
    let max = limits::MAX_SHAPE_EXTENT;
    assert!(Shape::new_tapered_cylinder(max / 2.0, 0.0, max, 0.0).is_ok());
    // Bounds count from the centre of mass, a quarter of the height above a cone's base.
    assert!(dimensions(Shape::new_tapered_cylinder(max, 0.0, 1.0, 0.0)));
    assert!(dimensions(Shape::new_tapered_cylinder(
        max.next_up(),
        0.0,
        1.0,
        0.0
    )));
    assert!(dimensions(Shape::new_tapered_cylinder(
        1.0,
        0.0,
        max.next_up(),
        0.0
    )));
    let floor = MIN_TAPERED_CYLINDER_RADIUS;
    assert!(Shape::new_tapered_cylinder(1.0, 0.0, floor, 0.0).is_ok());
    assert!(dimensions(Shape::new_tapered_cylinder(
        1.0,
        0.0,
        floor.next_down(),
        0.0
    )));
}

#[test]
fn tapered_shapes_follow_jolts_scale_rules() {
    let capsule = Shape::new_tapered_capsule(0.6, 0.2, 0.4).unwrap();
    assert!(Shape::scaled(&capsule, Vec3::new(2.0, 2.0, -2.0)).is_ok());
    assert!(matches!(
        Shape::scaled(&capsule, Vec3::new(1.0, 2.0, 1.0)),
        Err(ShapeError::InvalidSettings(_))
    ));
    let cylinder = Shape::new_tapered_cylinder(0.5, 0.2, 0.4, 0.0).unwrap();
    assert!(Shape::scaled(&cylinder, Vec3::new(2.0, 3.0, 2.0)).is_ok());
    assert!(matches!(
        Shape::scaled(&cylinder, Vec3::new(2.0, 1.0, 1.0)),
        Err(ShapeError::InvalidSettings(_))
    ));
}
