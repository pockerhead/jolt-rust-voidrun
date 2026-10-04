use super::*;
use crate::material::shape_material;
use crate::{CompoundChild, HeightFieldSettings, Quat, SubShapeId};

/// A 1 m square in the y = 0 plane, facing up, as two triangles.
fn quad() -> (Vec<Vec3>, Vec<[u32; 3]>) {
    (
        vec![
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        ],
        vec![[0, 1, 2], [0, 2, 3]],
    )
}

fn invalid_settings(result: Result<Shape, ShapeError>) -> bool {
    matches!(result, Err(ShapeError::InvalidSettings(_)))
}

fn invalid_dimensions(result: Result<Shape, ShapeError>) -> bool {
    matches!(result, Err(ShapeError::InvalidDimensions(_)))
}

fn no_triangles(result: Result<Shape, ShapeError>) -> bool {
    matches!(result, Err(ShapeError::Mesh(MeshError::NoTriangles)))
}

#[test]
fn unit_quad_builds_a_mesh() {
    let (vertices, triangles) = quad();
    let mesh = Shape::new_mesh(&vertices, &triangles).unwrap();
    assert_eq!(mesh.sub_type(), JPH_ShapeSubType_Mesh);
    assert!(mesh.must_be_static());
}

#[test]
fn indices_beyond_the_vertices_are_refused() {
    let (vertices, _) = quad();
    for index in [4, u32::MAX] {
        assert!(invalid_settings(Shape::new_mesh(
            &vertices,
            &[[0, 1, index]]
        )));
    }
    // Refused although the triangle is degenerate and Jolt would drop it.
    assert!(invalid_settings(Shape::new_mesh(&vertices, &[[0, 0, 4]])));
}

#[test]
fn empty_geometry_is_refused() {
    let (vertices, triangles) = quad();
    assert!(invalid_settings(Shape::new_mesh(&[], &triangles)));
    assert!(invalid_settings(Shape::new_mesh(&vertices, &[])));
}

#[test]
fn counts_must_fit_jolt_int() {
    let max = i32::MAX as usize;
    assert!(mesh_counts_fit(max, max));
    assert!(!mesh_counts_fit(max + 1, 1));
    assert!(!mesh_counts_fit(1, max + 1));
}

#[test]
fn vertices_must_be_finite_and_within_the_extent() {
    let (mut vertices, triangles) = quad();
    vertices[2].x = f32::NAN;
    assert!(invalid_dimensions(Shape::new_mesh(&vertices, &triangles)));

    let (mut vertices, triangles) = quad();
    vertices.push(Vec3::new(0.0, f32::NAN, 0.0));
    assert!(
        invalid_dimensions(Shape::new_mesh(&vertices, &triangles)),
        "an unreferenced vertex is checked too"
    );

    let max = limits::MAX_SHAPE_EXTENT;
    let (mut vertices, triangles) = quad();
    vertices[2] = Vec3::new(max, 0.0, max);
    vertices[0] = Vec3::new(-max, 0.0, -max);
    assert!(Shape::new_mesh(&vertices, &triangles).is_ok());
    vertices[2].x = max.next_up();
    assert!(invalid_dimensions(Shape::new_mesh(&vertices, &triangles)));
}

#[test]
fn settings_out_of_range_are_refused() {
    let (vertices, triangles) = quad();
    let build = |settings: MeshSettings<'_>| {
        Shape::new_mesh_with_settings(&vertices, &triangles, &settings)
    };
    for per_leaf in [0, 9] {
        assert!(invalid_settings(build(
            MeshSettings::default().max_triangles_per_leaf(per_leaf)
        )));
    }
    for per_leaf in [1, 8] {
        assert!(build(MeshSettings::default().max_triangles_per_leaf(per_leaf)).is_ok());
    }
    for threshold in [f32::NAN, 1.0001, -1.0001, f32::INFINITY] {
        assert!(invalid_settings(build(
            MeshSettings::default().active_edge_cos_threshold_angle(threshold)
        )));
    }
    for threshold in [-1.0, 1.0] {
        assert!(build(MeshSettings::default().active_edge_cos_threshold_angle(threshold)).is_ok());
    }
}

#[test]
fn material_lists_are_checked() {
    let (vertices, triangles) = quad();
    let list: Vec<PhysicsMaterial> = (0..33).map(|i| PhysicsMaterial::new(i).unwrap()).collect();
    let refs: Vec<&PhysicsMaterial> = list.iter().collect();
    let build = |materials: &[&PhysicsMaterial], indices: &[u8]| {
        let settings = MeshSettings::default().materials(materials, indices);
        Shape::new_mesh_with_settings(&vertices, &triangles, &settings)
    };
    assert!(invalid_settings(build(&[], &[0, 0])));
    assert!(invalid_settings(build(&refs, &[0, 0])), "33 materials");
    assert!(build(&refs[..32], &[0, 31]).is_ok(), "32 materials");
    assert!(invalid_settings(build(&refs[..2], &[0])), "one index short");
    assert!(
        invalid_settings(build(&refs[..2], &[0, 1, 1])),
        "one index too many"
    );
    assert!(
        invalid_settings(build(&refs[..2], &[0, 2])),
        "index beyond the list"
    );
}

/// The user data of the material a down-ray at `(x, z)` hits on `mesh` (shape space).
fn material_under(mesh: &Shape, x: f32, z: f32) -> Option<u64> {
    let (origin, direction) = (
        Vec3::new(x, 1.0, z).to_jph(),
        Vec3::new(0.0, -2.0, 0.0).to_jph(),
    );
    let mut hit = JPH_RayCastResult {
        bodyID: 0,
        fraction: 2.0,
        subShapeID2: 0,
    };
    // SAFETY: the mesh is live; every argument is a live local.
    assert!(unsafe { JPH_Shape_CastRay(mesh.as_ptr(), &origin, &direction, &mut hit) });
    // SAFETY: the mesh is live and the id came from a hit on it.
    unsafe { shape_material(mesh.as_ptr(), SubShapeId::new(hit.subShapeID2)) }
}

#[test]
fn triangles_carry_their_materials() {
    let (a, b) = (
        PhysicsMaterial::new(5).unwrap(),
        PhysicsMaterial::new(6).unwrap(),
    );
    let (vertices, triangles) = quad();
    let mesh = Shape::new_mesh_with_settings(
        &vertices,
        &triangles,
        &MeshSettings::default().materials(&[&a, &b], &[1, 0]),
    )
    .unwrap();
    drop((a, b));
    // Triangle [0, 1, 2] covers x < z, triangle [0, 2, 3] covers x > z.
    assert_eq!(material_under(&mesh, 0.2, 0.8), Some(6));
    assert_eq!(material_under(&mesh, 0.8, 0.2), Some(5));
    let plain = Shape::new_mesh(&vertices, &triangles).unwrap();
    assert_eq!(material_under(&plain, 0.2, 0.8), None);
}

#[test]
fn meshes_without_usable_triangles_are_refused() {
    let (vertices, _) = quad();
    assert!(no_triangles(Shape::new_mesh(
        &vertices,
        &[[0, 0, 1], [2, 3, 3]]
    )));
    let collinear = [
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(2.0, 0.0, 0.0),
    ];
    assert!(no_triangles(Shape::new_mesh(&collinear, &[[0, 1, 2]])));

    // Two slivers 1000 m apart: each has a 2e-4 m edge, below the 21-bit quantization step of
    // the mesh's 1000 m bounds, while its area is above Jolt's float degeneracy test.
    let sliver = |at: f32| {
        [
            Vec3::new(at, at, at),
            Vec3::new(at + 2.0e-4, at, at),
            Vec3::new(at, at + 0.1, at),
        ]
    };
    let vertices: Vec<Vec3> = sliver(0.0).into_iter().chain(sliver(1000.0)).collect();
    let slivers = [[0, 1, 2], [3, 4, 5]];
    assert!(no_triangles(Shape::new_mesh(&vertices, &slivers)));

    // A large triangle keeps the mesh alive next to a collapsing sliver.
    let mut with_floor = vertices.clone();
    with_floor.extend([
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1000.0),
        Vec3::new(1000.0, 0.0, 0.0),
    ]);
    assert!(Shape::new_mesh(&with_floor, &[[0, 1, 2], [6, 7, 8]]).is_ok());
}

#[test]
fn duplicate_triangles_build() {
    let (vertices, _) = quad();
    assert!(Shape::new_mesh(&vertices, &[[0, 1, 2], [0, 1, 2], [1, 2, 0]]).is_ok());
}

#[test]
fn mesh_settings_reach_jolt() {
    let settings = MeshSettings::default()
        .active_edge_cos_threshold_angle(0.5)
        .max_triangles_per_leaf(3)
        .build_quality(MeshBuildQuality::FavorBuildSpeed);
    let (vertices, triangles) = quad();
    initialize().unwrap();
    let jolt_settings = mesh_settings(&vertices, &triangles, &settings).unwrap();
    let ptr: *mut JPH_MeshShapeSettings = jolt_settings.as_ptr();
    // SAFETY: the settings are live and were created as mesh settings; getters only read them.
    unsafe {
        assert_eq!(
            JPH_MeshShapeSettings_GetActiveEdgeCosThresholdAngle(ptr),
            0.5
        );
        assert_eq!(JPH_MeshShapeSettings_GetMaxTrianglesPerLeaf(ptr), 3);
        assert_eq!(
            JPH_MeshShapeSettings_GetBuildQuality(ptr),
            JPH_Mesh_Shape_BuildQuality_FavorBuildSpeed
        );
        assert_eq!(JPH_MeshShapeSettings_GetTriangleCount(ptr), 2);
    }
    let defaults = mesh_settings(&vertices, &triangles, &MeshSettings::default()).unwrap();
    let ptr: *mut JPH_MeshShapeSettings = defaults.as_ptr();
    // SAFETY: as above.
    unsafe {
        assert_eq!(
            JPH_MeshShapeSettings_GetActiveEdgeCosThresholdAngle(ptr),
            0.996195
        );
        assert_eq!(JPH_MeshShapeSettings_GetMaxTrianglesPerLeaf(ptr), 8);
        assert_eq!(
            JPH_MeshShapeSettings_GetBuildQuality(ptr),
            JPH_Mesh_Shape_BuildQuality_FavorRuntimePerformance
        );
    }
}

fn child(shape: &Shape, x: f32) -> CompoundChild<'_> {
    CompoundChild {
        shape,
        position: Vec3::new(x, 0.0, 0.0),
        rotation: Quat::IDENTITY,
        user_data: 0,
    }
}

#[test]
fn kinematic_eligibility_looks_through_compounds_and_decorators() {
    let (vertices, triangles) = quad();
    let mesh = Shape::new_mesh(&vertices, &triangles).unwrap();
    let block = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
    let field = Shape::new_height_field(3, &[0.0; 9], &HeightFieldSettings::default()).unwrap();
    let offset_mesh = Shape::new_offset_center_of_mass(&mesh, Vec3::new(0.1, 0.0, 0.0)).unwrap();
    let mesh_and_box = Shape::new_compound(&[child(&mesh, 0.0), child(&block, 3.0)]).unwrap();
    let mesh_and_field = Shape::new_compound(&[child(&mesh, 0.0), child(&field, 5.0)]).unwrap();
    let inner = Shape::new_compound(&[child(&field, 0.0)]).unwrap();
    let nested_field = Shape::new_compound(&[child(&inner, 0.0), child(&block, 4.0)]).unwrap();
    for (shape, expected) in [
        (&block, true),
        (&mesh, true),
        (&offset_mesh, true),
        (&mesh_and_box, true),
        (&field, false),
        (&mesh_and_field, false),
        (&nested_field, false),
    ] {
        assert_eq!(shape.static_only_leaves_are_meshes(), expected);
    }
}

#[test]
fn sanitizing_runs_until_no_triangle_is_left_to_drop() {
    // The repeated-index triangle widens the bounds of Jolt's first clean-up pass to x = -1000.
    // Within those bounds the sliver's 3e-4 m edge spans two quantization cells; once the
    // degenerate triangle is gone the bounds start at x = 0 and the edge collapses, which
    // Jolt's shape constructor would refuse ("Triangle 1 is degenerate!").
    let x = f32::from_bits(0x43c8_0055);
    let vertices = [
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1000.0),
        Vec3::new(1000.0, 0.0, 0.0),
        Vec3::new(-1000.0, 0.0, 0.0),
        Vec3::new(x, 0.5, 500.0),
        Vec3::new(x + 3.0e-4, 0.5, 500.0),
        Vec3::new(x, 0.6, 500.0),
    ];
    let triangles = [[0, 1, 2], [3, 3, 0], [4, 5, 6]];
    assert!(Shape::new_mesh(&vertices, &triangles).is_ok());
}
