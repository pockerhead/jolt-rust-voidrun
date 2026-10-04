use super::*;
use crate::body::mass_properties;
use crate::material::shape_material;
use crate::{CompoundChild, HeightFieldSettings, PhysicsMaterial, Quat, SubShapeId};

fn unit_box() -> Shape {
    Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap()
}

/// The shape's local bounds as `[min, max]`.
fn bounds(shape: &Shape) -> [[f32; 3]; 2] {
    let mut bounds = JPH_AABox {
        min: Vec3::ZERO.to_jph(),
        max: Vec3::ZERO.to_jph(),
    };
    // SAFETY: the shape is live; `bounds` is a live local.
    unsafe { JPH_Shape_GetLocalBounds(shape.as_ptr(), &mut bounds) };
    [
        Vec3::from_jph(bounds.min).into(),
        Vec3::from_jph(bounds.max).into(),
    ]
}

fn invalid_settings(result: Result<Shape, ShapeError>) -> bool {
    matches!(result, Err(ShapeError::InvalidSettings(_)))
}

fn invalid_dimensions(result: Result<Shape, ShapeError>) -> bool {
    matches!(result, Err(ShapeError::InvalidDimensions(_)))
}

fn child(shape: &Shape, position: Vec3, rotation: Quat) -> CompoundChild<'_> {
    CompoundChild {
        shape,
        position,
        rotation,
        user_data: 0,
    }
}

/// A 1 m square in the y = 0 plane, two triangles facing up.
fn quad_mesh() -> Shape {
    let vertices = [
        Vec3::new(-0.5, 0.0, -0.5),
        Vec3::new(-0.5, 0.0, 0.5),
        Vec3::new(0.5, 0.0, 0.5),
        Vec3::new(0.5, 0.0, -0.5),
    ];
    Shape::new_mesh(&vertices, &[[0, 1, 2], [0, 2, 3]]).unwrap()
}

#[test]
fn box_bounds_follow_the_scale() {
    let scaled = Shape::scaled(&unit_box(), Vec3::new(1.0, 2.0, 3.0)).unwrap();
    assert_eq!(scaled.sub_type(), JPH_ShapeSubType_Scaled);
    assert_eq!(bounds(&scaled), [[-0.5, -1.0, -1.5], [0.5, 1.0, 1.5]]);
    let nested = Shape::scaled(&scaled, Vec3::new(2.0, 1.0, 1.0)).unwrap();
    assert_eq!(bounds(&nested), [[-1.0, -1.0, -1.5], [1.0, 1.0, 1.5]]);
}

#[test]
fn jolt_scale_rules_per_shape_kind() {
    let sphere = Shape::new_sphere(0.5).unwrap();
    assert!(invalid_settings(Shape::scaled(
        &sphere,
        Vec3::new(1.0, 2.0, 1.0)
    )));
    assert!(Shape::scaled(&sphere, Vec3::new(2.0, 2.0, 2.0)).is_ok());
    assert!(Shape::scaled(&sphere, Vec3::new(-2.0, 2.0, 2.0)).is_ok());
    let cylinder = Shape::new_cylinder(1.0, 0.5).unwrap();
    assert!(Shape::scaled(&cylinder, Vec3::new(2.0, 5.0, 2.0)).is_ok());
    assert!(invalid_settings(Shape::scaled(
        &cylinder,
        Vec3::new(2.0, 1.0, 3.0)
    )));

    let block = unit_box();
    let turned = Quat::from_xyzw(
        0.0,
        (22.5f32).to_radians().sin(),
        0.0,
        (22.5f32).to_radians().cos(),
    );
    let compound = Shape::new_compound(&[
        child(&block, Vec3::ZERO, Quat::IDENTITY),
        child(&block, Vec3::new(2.0, 0.0, 0.0), turned),
    ])
    .unwrap();
    assert!(invalid_settings(Shape::scaled(
        &compound,
        Vec3::new(2.0, 1.0, 1.0)
    )));
    // Scaling along the turn's axis keeps the child's axes.
    assert!(Shape::scaled(&compound, Vec3::new(1.0, 2.0, 1.0)).is_ok());
    assert!(Shape::scaled(&compound, Vec3::new(2.0, 2.0, 2.0)).is_ok());
}

#[test]
fn scale_components_are_checked() {
    let block = unit_box();
    assert!(Shape::scaled(&block, Vec3::new(1.0e-6, 1.0, 1.0)).is_ok());
    assert!(invalid_settings(Shape::scaled(
        &block,
        Vec3::new(0.99e-6, 1.0, 1.0)
    )));
    for bad in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
        assert!(invalid_dimensions(Shape::scaled(
            &block,
            Vec3::new(1.0, bad, 1.0)
        )));
    }
}

#[test]
fn scaled_bounds_and_centre_of_mass_stay_within_the_extent() {
    let max = limits::MAX_SHAPE_EXTENT;
    let block = Shape::new_box(Vec3::new(1.0, 1.0, 1.0)).unwrap();
    assert!(Shape::scaled(&block, Vec3::new(max, 1.0, 1.0)).is_ok());
    assert!(invalid_dimensions(Shape::scaled(
        &block,
        Vec3::new(max.next_up(), 1.0, 1.0)
    )));

    // A one-child compound keeps its centre of mass at the child, 1500 m out.
    let far = Shape::new_compound(&[child(
        &unit_box(),
        Vec3::new(1500.0, 0.0, 0.0),
        Quat::IDENTITY,
    )])
    .unwrap();
    assert!(Shape::scaled(&far, Vec3::new(1.2, 1.2, 1.2)).is_ok());
    assert!(invalid_dimensions(Shape::scaled(
        &far,
        Vec3::new(2.0, 2.0, 2.0)
    )));

    // A field of holes has no surface; its bounds are a point, which the scale moves away.
    let holes =
        Shape::new_height_field(3, &[f32::MAX; 9], &HeightFieldSettings::default()).unwrap();
    assert!(invalid_dimensions(Shape::scaled(
        &holes,
        Vec3::new(1.0e30, 1.0, 1.0e30)
    )));
}

#[test]
fn scaled_box_has_the_mass_of_the_equivalent_box() {
    let scaled = Shape::scaled(&unit_box(), Vec3::new(1.0, 2.0, 3.0)).unwrap();
    let equivalent = Shape::new_box(Vec3::new(0.5, 1.0, 1.5)).unwrap();
    let (a, b) = (
        mass_properties(&scaled, None),
        mass_properties(&equivalent, None),
    );
    let close = |x: f32, y: f32| (x - y).abs() <= 8.0 * f32::EPSILON * y.abs().max(1.0e-30);
    assert!(close(a.mass, b.mass), "{} vs {}", a.mass, b.mass);
    for column in 0..3 {
        let (p, q) = (a.inertia.column[column], b.inertia.column[column]);
        for (x, y) in [(p.x, q.x), (p.y, q.y), (p.z, q.z)] {
            assert!(x == y || close(x, y), "{x} vs {y}");
        }
    }
}

#[test]
fn materials_survive_scaling() {
    let material = PhysicsMaterial::new(44).unwrap();
    let inner = Shape::new_box_with_material(Vec3::new(0.5, 0.5, 0.5), 0.05, &material).unwrap();
    let scaled = Shape::scaled(&inner, Vec3::new(2.0, 1.0, 1.0)).unwrap();
    drop((inner, material));
    let (origin, direction) = (
        Vec3::new(0.0, 2.0, 0.0).to_jph(),
        Vec3::new(0.0, -4.0, 0.0).to_jph(),
    );
    let mut hit = JPH_RayCastResult {
        bodyID: 0,
        fraction: 2.0,
        subShapeID2: 0,
    };
    // SAFETY: the shape is live; every argument is a live local.
    assert!(unsafe { JPH_Shape_CastRay(scaled.as_ptr(), &origin, &direction, &mut hit) });
    // SAFETY: the shape is live and the id came from a hit on it.
    let found = unsafe { shape_material(scaled.as_ptr(), SubShapeId::new(hit.subShapeID2)) };
    assert_eq!(found, Some(44));
}

#[test]
fn scaled_meshes_stay_kinematic_and_collidable() {
    let mesh = quad_mesh();
    let doubled = Shape::scaled(&mesh, Vec3::new(2.0, 2.0, 2.0)).unwrap();
    assert!(doubled.static_only_leaves_are_meshes());
    assert!(Shape::scaled(&mesh, Vec3::new(1.0, -1.0, 1.0)).is_ok());
    // Twice the area of a 1 m triangle scaled by s is s^2: 1e-4 at s = 0.01, 1e-6 at s = 0.001.
    assert!(Shape::scaled(&mesh, Vec3::new(0.01, 1.0, 0.01)).is_ok());
    assert!(invalid_settings(Shape::scaled(
        &mesh,
        Vec3::new(0.001, 1.0, 0.001)
    )));
    // The check reaches meshes inside compounds and other decorators.
    let compound = Shape::new_compound(&[
        child(&mesh, Vec3::ZERO, Quat::IDENTITY),
        child(&unit_box(), Vec3::new(3.0, 0.0, 0.0), Quat::IDENTITY),
    ])
    .unwrap();
    let offset = Shape::new_offset_center_of_mass(&compound, Vec3::new(0.2, 0.0, 0.0)).unwrap();
    assert!(invalid_settings(Shape::scaled(
        &offset,
        Vec3::new(0.001, 0.001, 0.001)
    )));
    assert!(Shape::scaled(&offset, Vec3::new(0.5, 0.5, 0.5)).is_ok());
    let field = Shape::new_height_field(3, &[0.0; 9], &HeightFieldSettings::default()).unwrap();
    let scaled_field = Shape::scaled(&field, Vec3::new(2.0, 1.0, 2.0)).unwrap();
    assert!(!scaled_field.static_only_leaves_are_meshes());
    assert!(invalid_settings(Shape::scaled(
        &field,
        Vec3::new(0.001, 1.0, 0.001)
    )));
}
