use super::*;
use crate::material::shape_material;
use crate::shape::geometry::{mul_vec, rotation};
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

fn invalid_settings<T>(result: Result<T, ShapeError>) -> bool {
    matches!(result, Err(ShapeError::InvalidSettings(_)))
}

fn invalid_dimensions<T>(result: Result<T, ShapeError>) -> bool {
    matches!(result, Err(ShapeError::InvalidDimensions(_)))
}

fn no_triangles<T>(result: Result<T, ShapeError>) -> bool {
    matches!(result, Err(ShapeError::Mesh(MeshError::NoTriangles)))
}

/// The input indices of the triangles `collidable_triangles` keeps.
fn kept(vertices: &[Vec3], triangles: &[[u32; 3]]) -> Vec<usize> {
    collidable_triangles(vertices, triangles, MeshSettings::DEFAULT_MAX_CONVEX_EXTENT).0
}

#[test]
fn unit_quad_builds_a_mesh() {
    let (vertices, triangles) = quad();
    let (mesh, dropped) = Shape::new_mesh(&vertices, &triangles).unwrap();
    assert!(dropped.is_empty());
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
    let largest = 2.0 * limits::MAX_SHAPE_EXTENT;
    for extent in [f32::NAN, -0.0001, largest.next_up(), f32::INFINITY] {
        assert!(invalid_settings(build(
            MeshSettings::default().max_convex_extent(extent)
        )));
    }
    for extent in [0.0, largest] {
        assert!(build(MeshSettings::default().max_convex_extent(extent)).is_ok());
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
    .unwrap()
    .0;
    drop((a, b));
    // Triangle [0, 1, 2] covers x < z, triangle [0, 2, 3] covers x > z.
    assert_eq!(material_under(&mesh, 0.2, 0.8), Some(6));
    assert_eq!(material_under(&mesh, 0.8, 0.2), Some(5));
    let (plain, _) = Shape::new_mesh(&vertices, &triangles).unwrap();
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

    // Two 1 mm by 10 cm slivers, one at the origin and one 1000 m out on every axis, where
    // rounding moves its corners by about as much as its width.
    let sliver = |at: f32| {
        [
            Vec3::new(at, at, at),
            Vec3::new(at + 1.0e-3, at, at),
            Vec3::new(at, at + 0.1, at),
        ]
    };
    let vertices: Vec<Vec3> = sliver(0.0).into_iter().chain(sliver(1000.0)).collect();
    assert!(no_triangles(Shape::new_mesh(&vertices, &[[3, 4, 5]])));
    // The far sliver does not widen the bounds, so the near one keeps a fine quantization.
    let (_, dropped) = Shape::new_mesh(&vertices, &[[0, 1, 2], [3, 4, 5]]).unwrap();
    assert_eq!(dropped.indices(), [1]);

    // A 1000 m floor makes the x quantization step 0.5 mm, two steps across the near sliver.
    let mut with_floor = vertices.clone();
    with_floor.extend([
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1000.0),
        Vec3::new(1000.0, 0.0, 0.0),
    ]);
    let (_, dropped) = Shape::new_mesh(&with_floor, &[[0, 1, 2], [6, 7, 8]]).unwrap();
    assert_eq!(dropped.indices(), [0]);
}

#[test]
fn duplicate_triangles_build() {
    let (vertices, _) = quad();
    let (_, dropped) = Shape::new_mesh(&vertices, &[[0, 1, 2], [0, 1, 2], [1, 2, 0]]).unwrap();
    assert!(dropped.is_empty(), "Jolt keeps one copy; nothing is lost");
}

#[test]
fn mesh_settings_reach_jolt() {
    let settings = MeshSettings::default()
        .active_edge_cos_threshold_angle(0.5)
        .max_triangles_per_leaf(3)
        .build_quality(MeshBuildQuality::FavorBuildSpeed);
    let (vertices, triangles) = quad();
    initialize().unwrap();
    let jolt_settings = mesh_settings(&vertices, &triangles, &[0, 1], &settings).unwrap();
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
    let defaults = mesh_settings(&vertices, &triangles, &[0, 1], &MeshSettings::default()).unwrap();
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
    let (mesh, _) = Shape::new_mesh(&vertices, &triangles).unwrap();
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
fn slivers_that_collapse_under_quantization_are_dropped() {
    // Without the Rust filter, the repeated-index triangle widens the bounds of Jolt's first
    // clean-up pass to x = -1000, where the sliver's 3e-4 m edge spans two quantization cells;
    // once the degenerate triangle is gone the bounds start at x = 0, the edge collapses and
    // Jolt's shape constructor refuses the mesh ("Triangle 1 is degenerate!"). The filter
    // drops both: the sliver is thinner than the rounding of its 640 m distance.
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
    assert_eq!(kept(&vertices, &triangles), [0]);
    let (_, dropped) = Shape::new_mesh(&vertices, &triangles).unwrap();
    assert_eq!(dropped.indices(), [1, 2]);
}

#[test]
fn small_and_thin_triangles_are_dropped() {
    // Twice the area of a right triangle with legs `leg` is `leg^2`; the threshold is 1e-5 m²
    // plus the rounding margin, here mostly the rounding of coordinates 1100 m out in the space
    // of the largest convex shape of the default settings.
    let triangle = |leg: f32| {
        [
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, leg),
            Vec3::new(leg, 0.0, 0.0),
        ]
    };
    assert_eq!(kept(&triangle(0.0038), &[[0, 1, 2]]), [0]);
    assert!(kept(&triangle(0.0037), &[[0, 1, 2]]).is_empty());
    assert!(no_triangles(Shape::new_mesh(
        &triangle(0.0037),
        &[[0, 1, 2]]
    )));
    // A 10 m sliver 1e-5 m wide has a cross product of 1e-4, above the floor, but each corner
    // can move by about 1e-4 m across it, which changes the cross product by up to 10 m times
    // the width change.
    let sliver = [
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(10.0, 0.0, 0.0),
        Vec3::new(5.0, 0.0, 1.0e-5),
    ];
    assert!(kept(&sliver, &[[0, 1, 2]]).is_empty());
    let wide = [
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(10.0, 0.0, 0.0),
        Vec3::new(5.0, 0.0, 1.0e-3),
    ];
    assert_eq!(kept(&wide, &[[0, 1, 2]]), [0]);
}

/// A 1 m by 1 cm patch at the origin and a 10 m triangle 1500 m out along x: the mixed scales of
/// a level mesh. The far triangle makes the x quantization step 0.7 mm and the z step 5 um.
fn patch_and_far_triangle() -> Vec<Vec3> {
    vec![
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 0.01),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(1500.0, 0.0, 0.0),
        Vec3::new(1500.0, 0.0, 10.0),
        Vec3::new(1510.0, 0.0, 0.0),
    ]
}

#[test]
fn thin_patches_survive_far_geometry() {
    let vertices = patch_and_far_triangle();
    // Each axis has its own step: the patch's 1 cm lies along z, where the step is fine.
    let (_, dropped) = Shape::new_mesh(&vertices, &[[0, 1, 2], [3, 4, 5]]).unwrap();
    assert!(dropped.is_empty());
    // A degenerate far triangle does not widen the bounds.
    let (_, dropped) = Shape::new_mesh(&vertices, &[[0, 1, 2], [3, 3, 3]]).unwrap();
    assert_eq!(dropped.indices(), [1]);
    // A 2 mm wide patch spans 400 steps of z but under 3 of x (0.7 mm): kept along z, dropped
    // when turned across x.
    let mut narrow = vertices.clone();
    narrow[1] = Vec3::new(0.0, 0.0, 0.002);
    let (_, dropped) = Shape::new_mesh(&narrow, &[[0, 1, 2], [3, 4, 5]]).unwrap();
    assert!(dropped.is_empty());
    narrow[1] = Vec3::new(0.002, 0.0, 0.0);
    narrow[2] = Vec3::new(0.0, 0.0, 1.0);
    let (_, dropped) = Shape::new_mesh(&narrow, &[[0, 2, 1], [3, 4, 5]]).unwrap();
    assert_eq!(dropped.indices(), [0]);
}

#[test]
fn dropped_triangles_are_reported_with_their_area() {
    let (mut vertices, _) = quad();
    // A right triangle with 3 mm legs: twice its area, 9e-6 m², is below the 1e-5 floor.
    vertices.extend([
        Vec3::new(2.0, 0.0, 0.0),
        Vec3::new(2.0, 0.0, 0.003),
        Vec3::new(2.003, 0.0, 0.0),
    ]);
    let triangles = [[0, 1, 2], [0, 0, 3], [4, 5, 6], [0, 2, 3]];
    let (_, dropped) = Shape::new_mesh(&vertices, &triangles).unwrap();
    assert_eq!(dropped.count(), 2);
    assert_eq!(dropped.indices(), [1, 2]);
    assert!(
        (dropped.area() - 4.5e-6).abs() < 1.0e-9,
        "{}",
        dropped.area()
    );
    assert!(!dropped.is_empty());
}

#[test]
fn materials_follow_the_input_triangles_past_dropped_ones() {
    let list: Vec<PhysicsMaterial> = (0..3)
        .map(|i| PhysicsMaterial::new(20 + i).unwrap())
        .collect();
    let refs: Vec<&PhysicsMaterial> = list.iter().collect();
    let (vertices, _) = quad();
    // The repeated-index triangle in the middle carries its own material and is dropped.
    let triangles = [[0, 1, 2], [1, 1, 3], [0, 2, 3]];
    let settings = MeshSettings::default().materials(&refs, &[1, 2, 0]);
    let (mesh, dropped) = Shape::new_mesh_with_settings(&vertices, &triangles, &settings).unwrap();
    assert_eq!(dropped.indices(), [1]);
    assert_eq!(material_under(&mesh, 0.2, 0.8), Some(21));
    assert_eq!(material_under(&mesh, 0.8, 0.2), Some(20));

    // The mixed-scale patch keeps its material next to the far triangle.
    let vertices = patch_and_far_triangle();
    let settings = MeshSettings::default().materials(&refs, &[2, 1]);
    let (mesh, dropped) =
        Shape::new_mesh_with_settings(&vertices, &[[0, 1, 2], [3, 4, 5]], &settings).unwrap();
    assert!(dropped.is_empty());
    assert_eq!(material_under(&mesh, 0.5, 0.002), Some(22));
    assert_eq!(material_under(&mesh, 1501.0, 1.0), Some(21));
}

/// A strip 1 m long and `width` wide at the origin as two triangles, turned by the unit
/// quaternion `turn`.
fn turned_strip(width: f64, turn: [f64; 4]) -> Vec<Vec3> {
    let turn = rotation(turn);
    [
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
        [width, 0.0, 1.0],
        [width, 0.0, 0.0],
    ]
    .map(|corner| {
        let [x, y, z] = mul_vec(&turn, corner);
        Vec3::new(x as f32, y as f32, z as f32)
    })
    .to_vec()
}

const STRIP: [[u32; 3]; 2] = [[0, 1, 2], [0, 2, 3]];

/// Seeded unit quaternions, the identity first.
fn turns(count: usize) -> Vec<[f64; 4]> {
    let mut state = 0x1234_5678_u64;
    let mut unit = move || {
        state = state
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        (state >> 11) as f64 / (1u64 << 53) as f64
    };
    let tau = std::f64::consts::TAU;
    let mut turns = vec![[0.0, 0.0, 0.0, 1.0]];
    turns.extend((1..count).map(|_| {
        let (u1, u2, u3) = (unit(), unit(), unit());
        let (s1, s2) = ((1.0 - u1).sqrt(), u1.sqrt());
        [
            s1 * (tau * u2).sin(),
            s1 * (tau * u2).cos(),
            s2 * (tau * u3).sin(),
            s2 * (tau * u3).cos(),
        ]
    }));
    turns
}

#[test]
fn the_default_convex_extent_keeps_millimetre_bevels() {
    let keeps = |extent: f32, turn| {
        collidable_triangles(&turned_strip(1.0e-3, turn), &STRIP, extent)
            .1
            .is_empty()
    };
    let turns = turns(2000);
    // Every orientation keeps the strip at the default; in the worst ones it goes at 1200 m.
    assert!(turns
        .iter()
        .all(|&turn| keeps(MeshSettings::DEFAULT_MAX_CONVEX_EXTENT, turn)));
    assert!(turns.iter().any(|&turn| !keeps(1200.0, turn)));
    // Along the axes it survives up to about 2050 m.
    assert!(keeps(2000.0, turns[0]) && !keeps(2100.0, turns[0]));
}

#[test]
fn the_rule_does_not_depend_on_the_corner_order() {
    // Strips 1 m long and 0.5 to 1 mm wide, split along either diagonal, at the default
    // extent near the thinnest they keep: each triangle gets the same verdict from each corner.
    let mut verdicts = 0;
    for turn in turns(200) {
        for width in [5.0e-4, 5.4e-4, 6.0e-4, 9.3e-4, 1.0e-3] {
            let strip = turned_strip(width, turn);
            for [i, j, k] in [[0, 1, 2], [0, 2, 3], [0, 1, 3], [1, 2, 3]] {
                let corners = [i, j, k].map(|index| v3(strip[index]));
                let keep = |[a, b, c]: [V3; 3]| {
                    is_collidable([a, b, c], [0.0; 3], MeshSettings::DEFAULT_MAX_CONVEX_EXTENT)
                };
                let [a, b, c] = corners;
                let first = keep([a, b, c]);
                assert_eq!(keep([b, c, a]), first, "{width} {turn:?}");
                assert_eq!(keep([c, a, b]), first, "{width} {turn:?}");
                assert_eq!(keep([a, c, b]), first, "{width} {turn:?}");
                verdicts += usize::from(first);
            }
        }
    }
    // Both verdicts occur.
    assert!(verdicts > 0 && verdicts < 200 * 5 * 4, "{verdicts}");
}

#[test]
fn slivers_too_thin_for_large_convex_shapes_are_dropped() {
    // Jolt collides a triangle in the convex shape's space, 300 m from this sliver's corners for
    // a box of half extent 300 m; the 14 um width rounds away there.
    let sliver = |width: f32| {
        [
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(width, 0.0, 0.5),
        ]
    };
    assert!(no_triangles(Shape::new_mesh(&sliver(1.4e-5), &[[0, 1, 2]])));
    let largest = MeshSettings::default().max_convex_extent(2.0 * limits::MAX_SHAPE_EXTENT);
    assert!(no_triangles(Shape::new_mesh_with_settings(
        &sliver(2.0e-5),
        &[[0, 1, 2]],
        &largest
    )));
    // A smaller extent keeps thinner triangles.
    let small = MeshSettings::default().max_convex_extent(2.0);
    assert!(no_triangles(Shape::new_mesh(&sliver(1.0e-4), &[[0, 1, 2]])));
    assert!(Shape::new_mesh_with_settings(&sliver(1.0e-4), &[[0, 1, 2]], &small).is_ok());
    let thinnest = |extent: f32| {
        let settings = MeshSettings::default().max_convex_extent(extent);
        let (mut low, mut high) = (0.0f32, 0.01f32);
        for _ in 0..40 {
            let middle = 0.5 * (low + high);
            if Shape::new_mesh_with_settings(&sliver(middle), &[[0, 1, 2]], &settings).is_ok() {
                high = middle;
            } else {
                low = middle;
            }
        }
        high
    };
    let (default, largest) = (
        thinnest(MeshSettings::DEFAULT_MAX_CONVEX_EXTENT),
        thinnest(2.0 * limits::MAX_SHAPE_EXTENT),
    );
    assert!((5.3e-4..5.5e-4).contains(&default), "{default}");
    assert!((1.9e-3..2.0e-3).contains(&largest), "{largest}");
}
