use super::*;

#[test]
fn many_shapes_are_created_and_released() {
    for i in 0..1000 {
        let size = 0.1 + i as f32 * 0.001;
        drop(Shape::new_box(Vec3::new(size, size, size)).unwrap());
        drop(Shape::new_sphere(size).unwrap());
        drop(Shape::new_cylinder(size, size).unwrap());
        let capsule = Shape::new_capsule(size, size).unwrap();
        drop(Shape::new_offset_center_of_mass(&capsule, Vec3::new(0.0, -size, 0.0)).unwrap());
    }
}

#[test]
fn offset_center_of_mass_moves_only_the_center() {
    let offset = Vec3::new(0.1, -0.3, 0.25);
    let shape = Shape::new_offset_center_of_mass(&unit_box(), offset).unwrap();
    assert_eq!(shape.sub_type(), JPH_ShapeSubType_OffsetCenterOfMass);
    let bits = |v: Vec3| <[f32; 3]>::from(v).map(f32::to_bits);
    assert_eq!(bits(shape.center_of_mass()), bits(offset));
    for bad in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
        assert!(matches!(
            Shape::new_offset_center_of_mass(&unit_box(), Vec3::new(0.0, bad, 0.0)),
            Err(ShapeError::InvalidDimensions(_))
        ));
    }
    let terrain = Shape::new_height_field(3, &[0.0; 9], &HeightFieldSettings::default()).unwrap();
    let offset_terrain = Shape::new_offset_center_of_mass(&terrain, offset).unwrap();
    assert!(offset_terrain.must_be_static());
}

#[test]
fn invalid_dimensions_are_rejected() {
    let invalid =
        |result: Result<Shape, ShapeError>| matches!(result, Err(ShapeError::InvalidDimensions(_)));
    for bad in [0.0, -1.0, f32::NAN, f32::INFINITY] {
        assert!(invalid(Shape::new_box(Vec3::new(1.0, bad, 1.0))));
        assert!(invalid(Shape::new_sphere(bad)));
        assert!(invalid(Shape::new_cylinder(bad, 1.0)));
        assert!(invalid(Shape::new_cylinder(1.0, bad)));
        assert!(invalid(Shape::new_capsule(bad, 1.0)));
        assert!(invalid(Shape::new_capsule(1.0, bad)));
    }
    for bad in [-0.1, f32::NAN, f32::INFINITY] {
        assert!(invalid(Shape::new_box_with_convex_radius(
            Vec3::new(1.0, 1.0, 1.0),
            bad
        )));
        assert!(invalid(Shape::new_cylinder_with_convex_radius(
            1.0, 1.0, bad
        )));
    }
    assert!(Shape::new_box_with_convex_radius(Vec3::new(1.0, 1.0, 1.0), 0.0).is_ok());
    assert!(Shape::new_cylinder_with_convex_radius(1.0, 1.0, 0.0).is_ok());
}

#[test]
fn primitive_extents_are_bounded() {
    let invalid =
        |result: Result<Shape, ShapeError>| matches!(result, Err(ShapeError::InvalidDimensions(_)));
    let bound = limits::MAX_SHAPE_EXTENT;
    let beyond = bound.next_up();
    assert!(Shape::new_box(Vec3::new(1.0, bound, 1.0)).is_ok());
    assert!(invalid(Shape::new_box(Vec3::new(1.0, beyond, 1.0))));
    assert!(Shape::new_sphere(bound).is_ok());
    assert!(invalid(Shape::new_sphere(beyond)));
    assert!(Shape::new_cylinder(bound, bound).is_ok());
    assert!(invalid(Shape::new_cylinder(beyond, 1.0)));
    assert!(invalid(Shape::new_cylinder(1.0, beyond)));
    assert!(Shape::new_capsule(bound - 1.0, 1.0).is_ok());
    let just_beyond = beyond - (bound - 1.0);
    assert!(invalid(Shape::new_capsule(bound - 1.0, just_beyond)));
    assert!(invalid(Shape::new_capsule(1.0, f32::MAX)));
}

#[test]
fn decorated_and_compound_extents_are_bounded() {
    let unit_box = unit_box();
    let bound = limits::MAX_SHAPE_EXTENT;
    // The bounds around the moved centre of mass reach `offset + 0.5`.
    let edge = Vec3::new(0.0, bound - 1.0, 0.0);
    assert!(Shape::new_offset_center_of_mass(&unit_box, edge).is_ok());
    assert!(matches!(
        Shape::new_offset_center_of_mass(&unit_box, Vec3::new(0.0, bound, 0.0)),
        Err(ShapeError::InvalidDimensions(_))
    ));
    let large = Shape::new_box(Vec3::new(bound, 1.0, 1.0)).unwrap();
    let centred = [child(&large, 0.0, 1), child(&large, 0.0, 2)];
    assert!(Shape::new_compound(&centred).is_ok());
    let beside = [child(&large, 0.0, 1), child(&large, 2.0, 2)];
    assert!(matches!(
        Shape::new_compound(&beside),
        Err(ShapeError::InvalidDimensions(_))
    ));
    let far = [child(&unit_box, bound.next_up(), 1)];
    assert!(matches!(
        Shape::new_compound(&far),
        Err(ShapeError::InvalidSettings(_))
    ));
}

fn box_convex_radius(shape: &Shape) -> f32 {
    assert_eq!(shape.sub_type(), JPH_ShapeSubType_Box);
    // SAFETY: the shape is live and a box (checked above); the getter only reads it.
    unsafe { JPH_BoxShape_GetConvexRadius(shape.as_ptr().cast()) }
}

#[test]
fn box_convex_radius_reaches_jolt() {
    let unit = Vec3::new(1.0, 1.0, 1.0);
    let sharp = Shape::new_box_with_convex_radius(unit, 0.0).unwrap();
    assert_eq!(box_convex_radius(&sharp), 0.0);
    let default = Shape::new_box(unit).unwrap();
    assert_eq!(box_convex_radius(&default), 0.05);
    let clamped = Shape::new_box_with_convex_radius(Vec3::new(0.1, 1.0, 1.0), 0.5).unwrap();
    assert_eq!(box_convex_radius(&clamped), 0.1);
}

#[test]
fn cylinder_and_capsule_dimensions_reach_jolt() {
    let cylinder = Shape::new_cylinder(0.75, 0.3).unwrap();
    let capsule = Shape::new_capsule(0.70845, 0.4).unwrap();
    // SAFETY: both shapes are live and of the type each getter expects (their subtypes are
    // checked in `new_shapes_have_jolt_subtypes`); the getters only read them.
    unsafe {
        assert_eq!(
            JPH_CylinderShape_GetHalfHeight(cylinder.as_ptr().cast()),
            0.75
        );
        assert_eq!(JPH_CylinderShape_GetRadius(cylinder.as_ptr().cast()), 0.3);
        assert_eq!(
            JPH_CapsuleShape_GetHalfHeightOfCylinder(capsule.as_ptr().cast()),
            0.70845
        );
        assert_eq!(JPH_CapsuleShape_GetRadius(capsule.as_ptr().cast()), 0.4);
    }
}

#[test]
fn inflated_grows_spheres_and_capsules_only() {
    let sphere = Shape::new_sphere(0.05)
        .unwrap()
        .inflated(0.1)
        .unwrap()
        .unwrap();
    assert_eq!(sphere.sub_type(), JPH_ShapeSubType_Sphere);
    // SAFETY: the shape is live and a sphere; the getter only reads it.
    let radius = unsafe { JPH_SphereShape_GetRadius(sphere.as_ptr().cast()) };
    assert!((radius - 0.15).abs() < 1e-6);

    let capsule = Shape::new_capsule(0.7, 0.4)
        .unwrap()
        .inflated(0.1)
        .unwrap()
        .unwrap();
    // SAFETY: the shape is live and a capsule; the getters only read it.
    unsafe {
        assert_eq!(
            JPH_CapsuleShape_GetHalfHeightOfCylinder(capsule.as_ptr().cast()),
            0.7
        );
        assert!((JPH_CapsuleShape_GetRadius(capsule.as_ptr().cast()) - 0.5).abs() < 1e-6);
    }

    let unit_box = unit_box();
    assert!(unit_box.inflated(0.1).unwrap().is_none());
    // A query shape may grow beyond the extent bound; a non-finite size is still rejected.
    let largest = Shape::new_capsule(limits::MAX_SHAPE_EXTENT - 1.0, 1.0).unwrap();
    assert!(largest.inflated(1000.0).unwrap().is_some());
    assert!(largest.inflated(f32::INFINITY).is_err());
}

#[test]
fn static_only_and_center_of_mass_are_read_from_jolt() {
    assert!(!unit_box().must_be_static());
    let terrain = Shape::new_height_field(3, &[0.0; 9], &HeightFieldSettings::default());
    assert!(terrain.unwrap().must_be_static());
    assert_eq!(unit_box().center_of_mass(), Vec3::ZERO);
    let unit_box = unit_box();
    let pair = Shape::new_compound(&[child(&unit_box, 0.0, 1), child(&unit_box, 2.0, 2)]).unwrap();
    assert_eq!(pair.center_of_mass(), Vec3::new(1.0, 0.0, 0.0));
}

#[test]
fn new_shapes_have_jolt_subtypes() {
    let unit = Vec3::new(1.0, 1.0, 1.0);
    assert_eq!(
        Shape::new_box(unit).unwrap().sub_type(),
        JPH_ShapeSubType_Box
    );
    assert_eq!(
        Shape::new_cylinder(1.0, 0.5).unwrap().sub_type(),
        JPH_ShapeSubType_Cylinder
    );
    assert_eq!(
        Shape::new_capsule(1.0, 0.5).unwrap().sub_type(),
        JPH_ShapeSubType_Capsule
    );
    assert_eq!(
        Shape::new_sphere(1.0).unwrap().sub_type(),
        JPH_ShapeSubType_Sphere
    );
}

#[test]
fn height_field_settings_reach_jolt() {
    let settings = HeightFieldSettings::default()
        .block_size(4)
        .bits_per_sample(12)
        .active_edge_cos_threshold_angle(0.5);
    assert_ne!(settings, HeightFieldSettings::default());
    let samples = [0.0; 81];
    assert_eq!(settings.validate_layout(9), Ok(12));
    assert!(ensure_initialized());
    // SAFETY: Jolt is initialised and the inputs are valid (checked above).
    let jolt_settings = unsafe { height_field_settings(9, &samples, &settings, None) }.unwrap();
    let ptr = jolt_settings.as_ptr();
    // SAFETY: the settings are live and were created as heightfield settings; getters only
    // read them.
    unsafe {
        assert_eq!(JPH_HeightFieldShapeSettings_GetBlockSize(ptr), 4);
        assert_eq!(JPH_HeightFieldShapeSettings_GetBitsPerSample(ptr), 12);
        assert_eq!(
            JPH_HeightFieldShapeSettings_GetActiveEdgeCosThresholdAngle(ptr),
            0.5
        );
    }
    let shape = Shape::new_height_field(9, &samples, &settings).unwrap();
    // SAFETY: the shape is live and a heightfield; the getter only reads it.
    let block_size = unsafe { JPH_HeightFieldShape_GetBlockSize(shape.as_ptr().cast()) };
    assert_eq!(block_size, 4);
}

fn unit_box() -> Shape {
    Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap()
}

fn child(shape: &Shape, x: f32, user_data: u32) -> CompoundChild<'_> {
    CompoundChild {
        shape,
        position: Vec3::new(x, 0.0, 0.0),
        rotation: Quat::IDENTITY,
        user_data,
    }
}

#[test]
fn compound_subtypes_depend_on_the_child_count() {
    let unit_box = unit_box();
    let single = Shape::new_compound(&[child(&unit_box, 1.0, 7)]).unwrap();
    assert_eq!(single.sub_type(), JPH_ShapeSubType_MutableCompound);
    let pair = Shape::new_compound(&[child(&unit_box, 0.0, 1), child(&unit_box, 2.0, 2)]).unwrap();
    assert_eq!(pair.sub_type(), JPH_ShapeSubType_StaticCompound);
}

#[test]
fn out_of_range_sub_shape_ids_are_rejected() {
    let unit_box = unit_box();
    let children = [
        child(&unit_box, 0.0, 10),
        child(&unit_box, 2.0, 11),
        child(&unit_box, 4.0, 12),
    ];
    let compound = Shape::new_compound(&children).unwrap();
    // Three children need two bits; the remaining bits are the path below the child.
    for index in 0..3 {
        let id = SubShapeId::new(0xffff_fffc | index);
        assert_eq!(
            compound.compound_sub_shape(id),
            Some(CompoundSubShape {
                index,
                user_data: 10 + index
            })
        );
    }
    for raw in [3, 7, u32::MAX] {
        assert_eq!(compound.compound_sub_shape(SubShapeId::new(raw)), None);
    }
    assert_eq!(unit_box.compound_sub_shape(SubShapeId::new(0)), None);
}
