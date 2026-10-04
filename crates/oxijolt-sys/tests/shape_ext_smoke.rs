//! Smoke tests for the shape functions of the joltc additions: shape creation with Jolt's error
//! message, mesh settings with a material list, the sanitized triangle count and the triangle
//! readback.

mod framework;

use std::ffi::CStr;
use std::ptr::{null, null_mut};

use framework::*;
use oxijolt_sys::*;

/// Jolt's `Color::sGrey`.
const GREY: u32 = 0xFF80_8080;
/// A byte the creation must not write.
const SENTINEL: u8 = 0xA5;

/// Convex hull settings of `points` with a 0.05 m convex radius; the caller owns them.
fn hull_settings(points: &[JPH_Vec3]) -> *mut JPH_ShapeSettings {
    init();
    // SAFETY: Jolt is initialised; `points` is live and holds `points.len()` points, which Jolt
    // copies. The caller owns the returned reference.
    let settings =
        unsafe { JPH_ConvexHullShapeSettings_Create(points.as_ptr(), points.len() as u32, 0.05) };
    assert!(!settings.is_null());
    settings.cast()
}

fn cube() -> Vec<JPH_Vec3> {
    let mut points = Vec::new();
    for x in [-1.0, 1.0] {
        for y in [-1.0, 1.0] {
            for z in [-1.0, 1.0] {
                points.push(vec3(x, y, z));
            }
        }
    }
    points
}

fn collinear() -> Vec<JPH_Vec3> {
    vec![
        vec3(0.0, 0.0, 0.0),
        vec3(1.0, 0.0, 0.0),
        vec3(2.0, 0.0, 0.0),
    ]
}

/// Creates the shape of `settings` with an error buffer of `capacity` bytes filled with
/// [`SENTINEL`]; returns the shape (null on failure) and the buffer.
fn create(settings: *const JPH_ShapeSettings, capacity: usize) -> (*mut JPH_Shape, Vec<u8>) {
    let mut buffer = vec![SENTINEL; capacity.max(1)];
    // SAFETY: the settings are live; `buffer` is live and holds at least `capacity` bytes.
    let shape = unsafe {
        JPH_ShapeSettings_CreateShapeWithError(
            settings,
            buffer.as_mut_ptr().cast(),
            capacity as u32,
        )
    };
    (shape, buffer)
}

/// The NUL-terminated text at the start of `buffer`.
fn text(buffer: &[u8]) -> &str {
    CStr::from_bytes_until_nul(buffer)
        .expect("the message is NUL-terminated")
        .to_str()
        .expect("Jolt's message is UTF-8")
}

#[test]
fn valid_hull_is_created_with_an_empty_message() {
    let settings = hull_settings(&cube());
    let (shape, buffer) = create(settings, 256);
    assert!(!shape.is_null());
    assert_eq!(text(&buffer), "");
    // SAFETY: both are live and this test holds one reference to each.
    unsafe {
        JPH_Shape_Destroy(shape);
        JPH_ShapeSettings_Destroy(settings);
    }
}

#[test]
fn refused_hull_reports_jolt_message() {
    let settings = hull_settings(&collinear());
    let (shape, buffer) = create(settings, 256);
    assert!(shape.is_null());
    assert!(!text(&buffer).is_empty());
    // SAFETY: the settings are live and this test holds one reference.
    unsafe { JPH_ShapeSettings_Destroy(settings) };
}

#[test]
fn message_respects_the_buffer_capacity() {
    let settings = hull_settings(&collinear());
    let (shape, full) = create(settings, 256);
    assert!(shape.is_null());
    let message = text(&full).to_owned();
    assert!(
        message.len() > 7,
        "the message is longer than the small buffer: {message}"
    );

    let (shape, zero) = create(settings, 0);
    assert!(shape.is_null());
    assert_eq!(zero, [SENTINEL], "capacity 0 writes nothing");

    let (shape, one) = create(settings, 1);
    assert!(shape.is_null());
    assert_eq!(one, [0], "capacity 1 holds only the NUL");

    let (shape, eight) = create(settings, 8);
    assert!(shape.is_null());
    assert_eq!(&eight[..7], &message.as_bytes()[..7]);
    assert_eq!(eight[7], 0);

    // SAFETY: the settings are live; a null buffer is allowed with any capacity.
    let shape = unsafe { JPH_ShapeSettings_CreateShapeWithError(settings, null_mut(), 256) };
    assert!(shape.is_null());
    // SAFETY: the settings are live and this test holds one reference.
    unsafe { JPH_ShapeSettings_Destroy(settings) };
}

#[test]
fn second_create_returns_the_cached_shape_with_another_reference() {
    let settings = hull_settings(&cube());
    let (first, _) = create(settings, 0);
    let (second, _) = create(settings, 0);
    assert!(!first.is_null());
    assert_eq!(first, second);
    // SAFETY: the settings and the shape are live; this test holds one settings reference and
    // two shape references, the cache one more until the settings go.
    unsafe {
        JPH_ShapeSettings_Destroy(settings);
        JPH_Shape_Destroy(first);
        let mut hit = JPH_RayCastResult {
            bodyID: 0,
            fraction: 2.0,
            subShapeID2: 0,
        };
        let (origin, direction) = (vec3(0.0, 5.0, 0.0), vec3(0.0, -10.0, 0.0));
        assert!(JPH_Shape_CastRay(second, &origin, &direction, &mut hit));
        JPH_Shape_Destroy(second);
    }
}

fn material(user_data: u64) -> *mut JPH_PhysicsMaterial {
    init();
    // SAFETY: Jolt is initialised; the name is a NUL-terminated literal. The caller owns the
    // returned reference.
    let material = unsafe { JPH_PhysicsMaterial_Create2(c"mesh".as_ptr(), GREY, user_data) };
    assert!(!material.is_null());
    material
}

fn triangle(i1: u32, i2: u32, i3: u32, material_index: u32) -> JPH_IndexedTriangle {
    JPH_IndexedTriangle {
        i1,
        i2,
        i3,
        materialIndex: material_index,
        userData: 0,
    }
}

/// Two horizontal triangles at y = 0, one over x in 0..1 and one over x in 10..11, both facing
/// up (counter-clockwise seen from above).
fn two_triangle_vertices() -> [JPH_Vec3; 6] {
    [
        vec3(0.0, 0.0, 0.0),
        vec3(0.0, 0.0, 1.0),
        vec3(1.0, 0.0, 0.0),
        vec3(10.0, 0.0, 0.0),
        vec3(10.0, 0.0, 1.0),
        vec3(11.0, 0.0, 0.0),
    ]
}

/// Mesh settings of `vertices` and `triangles` with `materials`; the caller owns them.
fn mesh_settings(
    vertices: &[JPH_Vec3],
    triangles: &[JPH_IndexedTriangle],
    materials: &[*const JPH_PhysicsMaterial],
) -> *mut JPH_MeshShapeSettings {
    init();
    // SAFETY: Jolt is initialised; every array is live and holds the count passed, every index
    // is below `vertices.len()` and every material index addresses `materials` (0 without).
    let settings = unsafe {
        JPH_MeshShapeSettings_Create3(
            vertices.as_ptr(),
            vertices.len() as u32,
            triangles.as_ptr(),
            triangles.len() as u32,
            if materials.is_empty() {
                null()
            } else {
                materials.as_ptr()
            },
            materials.len() as u32,
        )
    };
    assert!(!settings.is_null());
    settings
}

/// The user data of the material a down-ray at `(x, 0.5)` hits on `shape`.
fn material_under(shape: *const JPH_Shape, x: f32) -> Option<u64> {
    let (origin, direction) = (vec3(x, 1.0, 0.25), vec3(0.0, -2.0, 0.0));
    let mut hit = JPH_RayCastResult {
        bodyID: 0,
        fraction: 2.0,
        subShapeID2: 0,
    };
    // SAFETY: the shape is live; every argument is a live local.
    assert!(unsafe { JPH_Shape_CastRay(shape, &origin, &direction, &mut hit) });
    // SAFETY: the shape is live and the sub-shape id came from a hit on it.
    let material = unsafe { JPH_Shape_GetMaterial(shape, hit.subShapeID2) };
    let mut value = 0;
    // SAFETY: `material` is null or live; `value` is a live local.
    unsafe { JPH_PhysicsMaterial_GetUserData(material, &mut value) }.then_some(value)
}

#[test]
fn mesh_triangles_keep_their_materials() {
    let (a, b) = (material(20), material(21));
    let settings = mesh_settings(
        &two_triangle_vertices(),
        &[triangle(0, 1, 2, 0), triangle(3, 4, 5, 1)],
        &[a, b],
    );
    // SAFETY: the materials are live; the settings' list holds its own references, and the
    // shape holds its own reference after the settings are released.
    let shape = unsafe {
        JPH_PhysicsMaterial_Destroy(a);
        JPH_PhysicsMaterial_Destroy(b);
        let (shape, buffer) = create(settings.cast(), 256);
        assert_eq!(text(&buffer), "");
        JPH_ShapeSettings_Destroy(settings.cast());
        shape
    };
    assert!(!shape.is_null());
    assert_eq!(material_under(shape, 0.25), Some(20));
    assert_eq!(material_under(shape, 10.25), Some(21));
    // SAFETY: the shape is live and this test holds one reference.
    unsafe { JPH_Shape_Destroy(shape) };
}

/// The triangle count of mesh settings built from `triangles` without materials.
fn sanitized_count(triangles: &[JPH_IndexedTriangle]) -> u32 {
    let settings = mesh_settings(&two_triangle_vertices(), triangles, &[]);
    // SAFETY: the settings are live; this test holds one reference, released after the read.
    unsafe {
        let count = JPH_MeshShapeSettings_GetTriangleCount(settings);
        JPH_ShapeSettings_Destroy(settings.cast());
        count
    }
}

#[test]
fn mesh_without_materials_builds_and_counts_its_triangles() {
    let settings = mesh_settings(
        &two_triangle_vertices(),
        &[triangle(0, 1, 2, 0), triangle(3, 4, 5, 0)],
        &[],
    );
    let (shape, _) = create(settings.cast(), 0);
    assert!(!shape.is_null());
    // SAFETY: both are live and this test holds one reference to each.
    unsafe {
        assert_eq!(JPH_MeshShapeSettings_GetTriangleCount(settings), 2);
        JPH_Shape_Destroy(shape);
        JPH_ShapeSettings_Destroy(settings.cast());
    }
}

#[test]
fn sanitized_count_drops_degenerate_and_duplicate_triangles() {
    assert_eq!(
        sanitized_count(&[triangle(0, 1, 2, 0), triangle(3, 4, 5, 0)]),
        2
    );
    assert_eq!(
        sanitized_count(&[triangle(0, 1, 2, 0), triangle(3, 3, 5, 0)]),
        1,
        "a repeated index makes a degenerate triangle"
    );
    assert_eq!(
        sanitized_count(&[triangle(0, 1, 2, 0), triangle(0, 1, 2, 0)]),
        1,
        "a copy of a triangle is a duplicate"
    );
}

/// The triangles of `shape` read with room for `capacity` of them, and the count returned.
fn triangles(shape: *const JPH_Shape, capacity: usize) -> (u32, Vec<JPH_Vec3>) {
    let sentinel = vec3(f32::NAN, f32::NAN, f32::NAN);
    let mut vertices = vec![sentinel; 3 * capacity];
    let buffer = if capacity == 0 {
        null_mut()
    } else {
        vertices.as_mut_ptr()
    };
    // SAFETY: the shape is live; `buffer` is null with capacity 0 or holds 3 * capacity vertices.
    let count = unsafe { JPH_Shape_GetTriangles(shape, buffer, capacity as u32) };
    (count, vertices)
}

#[test]
fn mesh_triangles_read_back_in_shape_space() {
    let settings = mesh_settings(
        &two_triangle_vertices(),
        &[triangle(0, 1, 2, 0), triangle(3, 4, 5, 0)],
        &[],
    );
    let (shape, _) = create(settings.cast(), 0);
    assert!(!shape.is_null());
    let (count, _) = triangles(shape, 0);
    assert_eq!(count, 2);
    let (count, vertices) = triangles(shape, 2);
    assert_eq!(count, 2);
    let input = two_triangle_vertices();
    // Every read vertex is an input vertex, up to Jolt's 21-bit quantization over 11 m.
    for vertex in &vertices {
        assert!(
            input.iter().any(|v| (v.x - vertex.x).abs() < 1.0e-4
                && (v.y - vertex.y).abs() < 1.0e-4
                && (v.z - vertex.z).abs() < 1.0e-4),
            "{vertex:?}"
        );
    }
    // A buffer of one triangle gets the first one; the count is still the total.
    let (count, partial) = triangles(shape, 1);
    assert_eq!(count, 2);
    assert!(partial.iter().all(|v| v.x.is_finite()));
    // SAFETY: both are live and this test holds one reference to each.
    unsafe {
        JPH_Shape_Destroy(shape);
        JPH_ShapeSettings_Destroy(settings.cast());
    }
}

#[test]
fn convex_shapes_give_a_triangulated_surface() {
    init();
    let half_extent = vec3(0.5, 1.0, 1.5);
    // SAFETY: Jolt is initialised; `half_extent` is a live local. This test owns the reference.
    let shape = unsafe { JPH_BoxShape_Create(&half_extent, 0.0) }.cast::<JPH_Shape>();
    let (count, vertices) = triangles(shape, 64);
    assert_eq!(count, 12, "two triangles per box face");
    for vertex in &vertices[..36] {
        assert_eq!(vertex.x.abs(), 0.5);
        assert_eq!(vertex.y.abs(), 1.0);
        assert_eq!(vertex.z.abs(), 1.5);
    }
    // SAFETY: the shape is live and this test holds one reference.
    unsafe { JPH_Shape_Destroy(shape) };
}

/// A flat mesh of `columns` x `rows` unit cells at y = 0, two triangles per cell; the caller owns
/// the returned shape's reference.
fn grid_mesh(columns: u32, rows: u32) -> *mut JPH_Shape {
    let mut vertices = Vec::new();
    for k in 0..=rows {
        for i in 0..=columns {
            vertices.push(vec3(i as f32, 0.0, k as f32));
        }
    }
    let mut triangles = Vec::new();
    for k in 0..rows {
        for i in 0..columns {
            let v = k * (columns + 1) + i;
            triangles.push(triangle(v, v + columns + 1, v + columns + 2, 0));
            triangles.push(triangle(v, v + columns + 2, v + 1, 0));
        }
    }
    let settings = mesh_settings(&vertices, &triangles, &[]);
    let (shape, _) = create(settings.cast(), 0);
    // SAFETY: the settings are live; this test holds one reference, and the shape its own.
    unsafe { JPH_ShapeSettings_Destroy(settings.cast()) };
    assert!(!shape.is_null());
    shape
}

/// A NaN with a payload the readback never writes.
fn canary() -> JPH_Vec3 {
    let value = f32::from_bits(0x7FC0_1234);
    vec3(value, value, value)
}

fn bits(vertex: &JPH_Vec3) -> [u32; 3] {
    [vertex.x.to_bits(), vertex.y.to_bits(), vertex.z.to_bits()]
}

#[test]
fn triangles_are_read_across_batches_up_to_the_capacity() {
    // 70 triangles take three of Jolt's 32-triangle batches.
    let shape = grid_mesh(5, 7);
    let (count, all) = triangles(shape, 70);
    assert_eq!(count, 70);
    assert!(all
        .iter()
        .all(|v| v.x.is_finite() && v.y == 0.0 && v.z.is_finite()));
    for capacity in [0, 1, 31, 32, 33, 64, 69, 70, 71] {
        // One triangle more than the capacity, which must stay untouched.
        let mut vertices = vec![canary(); 3 * (capacity + 1)];
        let buffer = if capacity == 0 {
            null_mut()
        } else {
            vertices.as_mut_ptr()
        };
        // SAFETY: the shape is live; `buffer` is null with capacity 0 or holds more than
        // 3 * capacity vertices.
        let count = unsafe { JPH_Shape_GetTriangles(shape, buffer, capacity as u32) };
        assert_eq!(count, 70, "capacity {capacity}");
        let written = capacity.min(70);
        for (read, expected) in vertices[..3 * written].iter().zip(&all) {
            assert_eq!(bits(read), bits(expected), "capacity {capacity}");
        }
        for untouched in &vertices[3 * written..] {
            assert_eq!(bits(untouched), bits(&canary()), "capacity {capacity}");
        }
    }
    // SAFETY: the shape is live and this test holds one reference.
    unsafe { JPH_Shape_Destroy(shape) };
}

#[test]
fn height_field_holes_have_no_triangles() {
    init();
    // 4 x 4 samples, 3 x 3 cells of two triangles; f32::MAX marks a hole.
    let mut samples = [0.5_f32; 16];
    let read = |samples: &[f32; 16]| {
        let (offset, scale) = (vec3(0.0, 0.0, 0.0), vec3(1.0, 1.0, 1.0));
        // SAFETY: Jolt is initialised; `samples` holds 4 x 4 samples and the vectors are live
        // locals. This test owns the settings' reference and releases it after creating the
        // shape, which holds its own.
        let shape = unsafe {
            let settings =
                JPH_HeightFieldShapeSettings_Create(samples.as_ptr(), &offset, &scale, 4, null());
            assert!(!settings.is_null());
            let (shape, _) = create(settings.cast(), 0);
            JPH_ShapeSettings_Destroy(settings.cast());
            shape
        };
        assert!(!shape.is_null());
        let result = triangles(shape, 18);
        // SAFETY: the shape is live and this test holds one reference.
        unsafe { JPH_Shape_Destroy(shape) };
        result
    };
    let (full, _) = read(&samples);
    assert_eq!(full, 18);
    samples[5] = f32::MAX;
    let (count, vertices) = read(&samples);
    assert!(count > 0 && count < 18, "{count}");
    for vertex in &vertices[..3 * count as usize] {
        assert!((vertex.y - 0.5).abs() < 1.0e-3, "{vertex:?}");
    }
}

#[test]
fn compound_and_decorated_shapes_have_no_triangles() {
    let mesh = grid_mesh(1, 1);
    let (scale, offset) = (vec3(2.0, 2.0, 2.0), vec3(0.1, 0.0, 0.0));
    let turn = JPH_Quat {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0,
    };
    // SAFETY: Jolt is initialised; the mesh is live and each decorator or compound takes its
    // own reference to it. This test owns the reference of every returned shape and settings
    // object and releases each once.
    unsafe {
        let compound_settings = JPH_MutableCompoundShapeSettings_Create();
        JPH_CompoundShapeSettings_AddShape2(compound_settings.cast(), &offset, &turn, mesh, 0);
        let (compound, _) = create(compound_settings.cast(), 0);
        JPH_ShapeSettings_Destroy(compound_settings.cast());
        let parents: [*mut JPH_Shape; 4] = [
            JPH_ScaledShape_Create(mesh, &scale).cast(),
            JPH_OffsetCenterOfMassShape_Create(&offset, mesh).cast(),
            JPH_RotatedTranslatedShape_Create(&offset, &turn, mesh).cast(),
            compound,
        ];
        for parent in parents {
            assert!(!parent.is_null());
            let mut vertices = vec![canary(); 3];
            assert_eq!(JPH_Shape_GetTriangles(parent, vertices.as_mut_ptr(), 1), 0);
            assert_eq!(bits(&vertices[0]), bits(&canary()));
            JPH_Shape_Destroy(parent);
        }
        assert_eq!(triangles(mesh, 2).0, 2, "the leaf itself is read");
        JPH_Shape_Destroy(mesh);
    }
}
