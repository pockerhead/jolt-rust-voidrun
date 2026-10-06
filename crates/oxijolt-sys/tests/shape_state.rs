//! Tests of the shape binary state functions of the joltc additions: round trips of a box and of a
//! compound of a mesh and a hull with materials, and each check of the restore refusing a damaged
//! payload by its own message before Jolt reads it.

mod framework;

use std::ffi::CStr;
use std::ptr::{null, null_mut};

use framework::*;
use oxijolt_sys::*;

/// Jolt's `Color::sGrey`.
const GREY: u32 = 0xFF80_8080;
/// The sub-shape id of a shape without sub-shapes.
const ROOT_SUB_SHAPE: JPH_SubShapeID = 0xFFFF_FFFF;
/// Jolt's `EShapeSubType::Triangle`, which the functions do not support.
const TRIANGLE_SUB_TYPE: u8 = 2;
/// Jolt's `EShapeSubType::Empty` (joltc's enum stops at `SoftBody`).
const EMPTY_SUB_TYPE: u8 = 33;

/// Saves `shape`; panics with Jolt's message when the save is refused.
fn save(shape: *const JPH_Shape) -> Vec<u8> {
    let mut error = [0u8; 256];
    // SAFETY: `shape` is live; `error` holds 256 bytes.
    let state =
        unsafe { JPH_Shape_SaveBinaryState(shape, error.as_mut_ptr().cast(), error.len() as u32) };
    assert!(!state.is_null(), "save refused: {}", text(&error));
    // SAFETY: `state` is live until the destroy below; `bytes` holds `size` bytes.
    unsafe {
        let size = JPH_ShapeBinaryState_GetSize(state);
        let mut bytes = vec![0u8; size];
        JPH_ShapeBinaryState_CopyData(state, bytes.as_mut_ptr().cast(), size);
        JPH_ShapeBinaryState_Destroy(state);
        bytes
    }
}

/// Restores `bytes`: the shape (null on refusal) and the message.
fn restore(bytes: &[u8]) -> (*mut JPH_Shape, String) {
    init();
    let mut error = [0xA5u8; 256];
    let data = if bytes.is_empty() {
        null()
    } else {
        bytes.as_ptr().cast()
    };
    // SAFETY: `data` is null for no bytes or points at `bytes.len()` live bytes; every test feeds
    // bytes whose Jolt records were written by Jolt's SaveBinaryState of this build (the contract).
    let shape = unsafe {
        JPH_Shape_RestoreBinaryState(
            data,
            bytes.len(),
            error.as_mut_ptr().cast(),
            error.len() as u32,
        )
    };
    (shape, text(&error).to_owned())
}

/// Restores `bytes`, expecting a refusal whose message contains `phrase`.
fn assert_refused(bytes: &[u8], phrase: &str) {
    let (shape, message) = restore(bytes);
    assert!(
        shape.is_null(),
        "restored, expected a refusal with {phrase:?}"
    );
    assert!(
        message.contains(phrase),
        "message {message:?} lacks {phrase:?}"
    );
}

fn text(buffer: &[u8]) -> &str {
    CStr::from_bytes_until_nul(buffer)
        .expect("the message is NUL-terminated")
        .to_str()
        .expect("the message is UTF-8")
}

fn destroy(shape: *mut JPH_Shape) {
    // SAFETY: the caller holds one reference to the live `shape`.
    unsafe { JPH_Shape_Destroy(shape) };
}

fn material(user_data: u64) -> *mut JPH_PhysicsMaterial {
    init();
    // SAFETY: Jolt is initialised; the name is a NUL-terminated literal.
    unsafe { JPH_PhysicsMaterial_Create2(c"stone".as_ptr(), GREY, user_data) }
}

fn material_user_data(material: *const JPH_PhysicsMaterial) -> Option<u64> {
    let mut value = 0;
    // SAFETY: `material` is null or live; `value` is a live local.
    unsafe { JPH_PhysicsMaterial_GetUserData(material, &mut value) }.then_some(value)
}

/// A box with a user-data material and shape user data 77.
fn box_shape() -> *mut JPH_Shape {
    let material = material(7);
    let half_extent = vec3(0.5, 1.0, 1.5);
    // SAFETY: Jolt is initialised; the settings and the material are released after the shape
    // took its own references.
    unsafe {
        let settings = JPH_BoxShapeSettings_Create(&half_extent, 0.05);
        JPH_ConvexShapeSettings_SetMaterial(settings.cast(), material);
        let shape: *mut JPH_Shape = JPH_BoxShapeSettings_CreateShape(settings).cast();
        JPH_ShapeSettings_Destroy(settings.cast());
        JPH_PhysicsMaterial_Destroy(material);
        assert!(!shape.is_null());
        JPH_Shape_SetUserData(shape, 77);
        shape
    }
}

/// A two-triangle mesh with two materials, a hull, and the same hull again, in a static compound
/// with child user data 1, 2 and 3.
fn compound_shape() -> *mut JPH_Shape {
    let materials = [material(10), material(20)];
    let vertices = [
        vec3(-2.0, 0.0, -2.0),
        vec3(2.0, 0.0, -2.0),
        vec3(2.0, 0.0, 2.0),
        vec3(-2.0, 0.0, 2.0),
    ];
    let triangles = [
        JPH_IndexedTriangle {
            i1: 0,
            i2: 2,
            i3: 1,
            materialIndex: 0,
            userData: 0,
        },
        JPH_IndexedTriangle {
            i1: 0,
            i2: 3,
            i3: 2,
            materialIndex: 1,
            userData: 0,
        },
    ];
    let points = [
        vec3(0.0, 0.0, 0.0),
        vec3(1.0, 0.0, 0.0),
        vec3(0.0, 1.0, 0.0),
        vec3(0.0, 0.0, 1.0),
    ];
    let identity = quat_identity();
    // SAFETY: Jolt is initialised; every array is live for its call and holds the count passed;
    // settings, materials and children are released after their users took their own references.
    unsafe {
        let mesh_settings = JPH_MeshShapeSettings_Create3(
            vertices.as_ptr(),
            vertices.len() as u32,
            triangles.as_ptr(),
            triangles.len() as u32,
            materials.as_ptr().cast(),
            materials.len() as u32,
        );
        let mesh: *mut JPH_Shape = JPH_MeshShapeSettings_CreateShape(mesh_settings).cast();
        JPH_ShapeSettings_Destroy(mesh_settings.cast());
        let hull_settings =
            JPH_ConvexHullShapeSettings_Create(points.as_ptr(), points.len() as u32, 0.05);
        let hull: *mut JPH_Shape = JPH_ConvexHullShapeSettings_CreateShape(hull_settings).cast();
        JPH_ShapeSettings_Destroy(hull_settings.cast());
        assert!(!mesh.is_null() && !hull.is_null());

        let settings = JPH_StaticCompoundShapeSettings_Create();
        JPH_CompoundShapeSettings_AddShape2(
            settings.cast(),
            &vec3(0.0, -1.0, 0.0),
            &identity,
            mesh,
            1,
        );
        JPH_CompoundShapeSettings_AddShape2(
            settings.cast(),
            &vec3(0.0, 1.0, 0.0),
            &identity,
            hull,
            2,
        );
        JPH_CompoundShapeSettings_AddShape2(
            settings.cast(),
            &vec3(0.0, 3.0, 0.0),
            &identity,
            hull,
            3,
        );
        let compound: *mut JPH_Shape = JPH_StaticCompoundShape_Create(settings).cast();
        JPH_ShapeSettings_Destroy(settings.cast());
        destroy(mesh);
        destroy(hull);
        for material in materials {
            JPH_PhysicsMaterial_Destroy(material);
        }
        assert!(!compound.is_null());
        compound
    }
}

fn bounds(shape: *const JPH_Shape) -> [f32; 6] {
    // SAFETY: `shape` is live; the box is a live local.
    let b = unsafe {
        let mut b: JPH_AABox = std::mem::zeroed();
        JPH_Shape_GetLocalBounds(shape, &mut b);
        b
    };
    [b.min.x, b.min.y, b.min.z, b.max.x, b.max.y, b.max.z]
}

/// The fraction and sub-shape id of a ray down through `(x, z)`, if it hits.
fn ray_down(shape: *const JPH_Shape, x: f32, z: f32) -> Option<(u32, JPH_SubShapeID)> {
    let origin = vec3(x, 10.0, z);
    let direction = vec3(0.0, -20.0, 0.0);
    // SAFETY: `shape` is live; the arguments are live locals.
    unsafe {
        let mut hit: JPH_RayCastResult = std::mem::zeroed();
        JPH_Shape_CastRay(shape, &origin, &direction, &mut hit)
            .then_some((hit.fraction.to_bits(), hit.subShapeID2))
    }
}

#[test]
fn a_box_round_trips_with_its_material_and_user_data() {
    let shape = box_shape();
    let bytes = save(shape);
    let (restored, message) = restore(&bytes);
    assert!(!restored.is_null(), "{message}");
    assert_eq!(message, "");
    // SAFETY: both shapes are live.
    unsafe {
        assert_eq!(JPH_Shape_GetSubType(restored), JPH_Shape_GetSubType(shape));
        assert_eq!(JPH_Shape_GetUserData(restored), 77);
        assert_eq!(
            JPH_Shape_GetVolume(restored).to_bits(),
            JPH_Shape_GetVolume(shape).to_bits()
        );
        assert_eq!(
            material_user_data(JPH_Shape_GetMaterial(restored, ROOT_SUB_SHAPE)),
            Some(7)
        );
    }
    assert_eq!(bounds(restored), bounds(shape));
    assert_eq!(
        save(restored),
        bytes,
        "save, restore and save gives the first bytes"
    );
    destroy(restored);
    destroy(shape);
}

#[test]
fn a_compound_of_a_mesh_and_a_shared_hull_round_trips() {
    let shape = compound_shape();
    let bytes = save(shape);
    let (restored, message) = restore(&bytes);
    assert!(!restored.is_null(), "{message}");
    assert_eq!(bounds(restored), bounds(shape));
    // SAFETY: both shapes are live compounds.
    unsafe {
        assert_eq!(JPH_CompoundShape_GetNumSubShapes(restored.cast()), 3);
    }
    let mut hits = 0;
    for i in -8..=8 {
        for j in -8..=8 {
            let (x, z) = (i as f32 * 0.24 + 0.01, j as f32 * 0.24 + 0.02);
            let original = ray_down(shape, x, z);
            assert_eq!(ray_down(restored, x, z), original, "ray at ({x}, {z})");
            if let Some((_, sub_shape)) = original {
                hits += 1;
                // SAFETY: both shapes are live and the id came from a hit on the original.
                let (a, b) = unsafe {
                    (
                        JPH_Shape_GetMaterial(shape, sub_shape),
                        JPH_Shape_GetMaterial(restored, sub_shape),
                    )
                };
                assert_eq!(material_user_data(b), material_user_data(a));
            }
        }
    }
    assert!(hits > 100, "the ray grid hits the shape ({hits} hits)");
    assert_eq!(save(restored), bytes);
    destroy(restored);
    destroy(shape);
}

#[test]
fn an_empty_shape_keeps_its_center_of_mass() {
    init();
    let center = vec3(1.0, 2.0, 3.0);
    // SAFETY: Jolt is initialised; the settings are released after the shape was created.
    let shape: *mut JPH_Shape = unsafe {
        let settings = JPH_EmptyShapeSettings_Create(&center);
        let shape = JPH_EmptyShapeSettings_CreateShape(settings).cast();
        JPH_ShapeSettings_Destroy(settings.cast());
        JPH_Shape_SetUserData(shape, 5);
        shape
    };
    let (restored, message) = restore(&save(shape));
    assert!(!restored.is_null(), "{message}");
    let mut restored_center = vec3(0.0, 0.0, 0.0);
    // SAFETY: `restored` is live; the vector is a live local.
    unsafe {
        JPH_Shape_GetCenterOfMass(restored, &mut restored_center);
        assert_eq!(JPH_Shape_GetUserData(restored), 5);
    }
    assert_eq!(
        (restored_center.x, restored_center.y, restored_center.z),
        (1.0, 2.0, 3.0)
    );
    destroy(restored);
    destroy(shape);
}

#[test]
fn a_shape_type_outside_the_supported_set_is_not_saved() {
    init();
    // SAFETY: Jolt is initialised; the vectors are live locals.
    let triangle: *mut JPH_Shape = unsafe {
        JPH_TriangleShape_Create(
            &vec3(0.0, 0.0, 0.0),
            &vec3(1.0, 0.0, 0.0),
            &vec3(0.0, 0.0, 1.0),
            0.0,
        )
        .cast()
    };
    let mut error = [0u8; 256];
    // SAFETY: `triangle` is live; `error` holds 256 bytes.
    let state = unsafe {
        JPH_Shape_SaveBinaryState(triangle, error.as_mut_ptr().cast(), error.len() as u32)
    };
    assert!(state.is_null());
    assert!(
        text(&error).contains("type that cannot be saved"),
        "{}",
        text(&error)
    );
    // SAFETY: `null_mut` is a null shape, which the function refuses before reading it.
    let state = unsafe { JPH_Shape_SaveBinaryState(null_mut(), null_mut(), 0) };
    assert!(state.is_null());
    destroy(triangle);
}

/// A payload split into its records, to build damaged payloads from a valid one.
#[derive(Clone)]
struct Payload {
    materials: Vec<u8>,
    material_count: u32,
    records: Vec<Record>,
}

#[derive(Clone)]
struct Record {
    sub_type: u8,
    children: Vec<u32>,
    materials: Vec<u32>,
    extra: Vec<u8>,
    jolt: Vec<u8>,
}

struct Cursor<'a>(&'a [u8]);

impl Cursor<'_> {
    fn take(&mut self, n: usize) -> Vec<u8> {
        let (head, tail) = self.0.split_at(n);
        self.0 = tail;
        head.to_vec()
    }
    fn u32(&mut self) -> u32 {
        u32::from_le_bytes(self.take(4).try_into().unwrap())
    }
    fn indices(&mut self) -> Vec<u32> {
        let count = self.u32();
        (0..count).map(|_| self.u32()).collect()
    }
}

impl Payload {
    fn parse(bytes: &[u8]) -> Self {
        let mut cursor = Cursor(bytes);
        let material_count = cursor.u32();
        let start = cursor.0;
        for _ in 0..material_count {
            cursor.take(1 + 8 + 4);
            let name_length = cursor.u32() as usize;
            cursor.take(name_length);
        }
        let materials = start[..start.len() - cursor.0.len()].to_vec();
        let record_count = cursor.u32();
        let records = (0..record_count)
            .map(|_| {
                let length = cursor.u32() as usize;
                let mut record = Cursor(&cursor.0[..length]);
                cursor.take(length);
                let sub_type = record.take(1)[0];
                let children = record.indices();
                let materials = record.indices();
                let extra = if sub_type == EMPTY_SUB_TYPE {
                    record.take(12)
                } else {
                    Vec::new()
                };
                let jolt_length = record.u32() as usize;
                let jolt = record.take(jolt_length);
                Record {
                    sub_type,
                    children,
                    materials,
                    extra,
                    jolt,
                }
            })
            .collect();
        assert!(cursor.0.is_empty());
        Payload {
            materials,
            material_count,
            records,
        }
    }

    fn encode(&self) -> Vec<u8> {
        let mut out = self.material_count.to_le_bytes().to_vec();
        out.extend(&self.materials);
        out.extend((self.records.len() as u32).to_le_bytes());
        for record in &self.records {
            let body = record.encode();
            out.extend((body.len() as u32).to_le_bytes());
            out.extend(body);
        }
        out
    }
}

impl Record {
    fn encode(&self) -> Vec<u8> {
        let mut out = vec![self.sub_type];
        for list in [&self.children, &self.materials] {
            out.extend((list.len() as u32).to_le_bytes());
            for index in list {
                out.extend(index.to_le_bytes());
            }
        }
        out.extend(&self.extra);
        out.extend((self.jolt.len() as u32).to_le_bytes());
        out.extend(&self.jolt);
        out
    }
}

fn box_payload() -> Payload {
    let shape = box_shape();
    let payload = Payload::parse(&save(shape));
    destroy(shape);
    payload
}

fn compound_payload() -> Payload {
    let shape = compound_shape();
    let payload = Payload::parse(&save(shape));
    destroy(shape);
    payload
}

#[test]
fn the_parsed_payloads_encode_to_the_saved_bytes() {
    let shape = compound_shape();
    let bytes = save(shape);
    destroy(shape);
    let payload = Payload::parse(&bytes);
    assert_eq!(
        payload.records.len(),
        3,
        "mesh, the shared hull once, compound"
    );
    assert_eq!(
        payload.material_count, 3,
        "two mesh materials and the hull's null one"
    );
    assert_eq!(payload.records[2].children, [0, 1, 1]);
    assert_eq!(payload.encode(), bytes);
    let (restored, message) = restore(&payload.encode());
    assert!(!restored.is_null(), "{message}");
    destroy(restored);
}

#[test]
fn an_unsupported_shape_type_is_refused() {
    let mut payload = box_payload();
    payload.records[0].sub_type = TRIANGLE_SUB_TYPE;
    payload.records[0].jolt[0] = TRIANGLE_SUB_TYPE;
    assert_refused(&payload.encode(), "unsupported shape type");
}

#[test]
fn a_type_that_differs_from_joltc_bytes_is_refused() {
    let mut payload = box_payload();
    payload.records[0].sub_type = JPH_ShapeSubType_Sphere as u8;
    assert_refused(&payload.encode(), "does not match Jolt's data");
}

#[test]
fn a_child_that_is_not_an_earlier_record_is_refused() {
    let mut payload = compound_payload();
    payload.records[2].children[0] = 2;
    assert_refused(&payload.encode(), "does not name an earlier record");
}

#[test]
fn a_wrong_child_count_is_refused() {
    let mut payload = compound_payload();
    payload.records[2].children.pop();
    assert_refused(&payload.encode(), "wrong number of children");
}

#[test]
fn a_wrong_material_count_is_refused() {
    let mut payload = box_payload();
    payload.records[0].materials.clear();
    assert_refused(&payload.encode(), "wrong number of materials");
}

#[test]
fn a_material_index_out_of_range_is_refused() {
    let mut payload = box_payload();
    payload.records[0].materials[0] = payload.material_count;
    assert_refused(&payload.encode(), "material index is out of range");
}

#[test]
fn a_malformed_material_is_refused() {
    let mut payload = box_payload();
    payload.materials[0] = 9;
    assert_refused(&payload.encode(), "material record is malformed");
}

#[test]
fn a_truncated_record_is_refused() {
    let bytes = box_payload().encode();
    for length in [bytes.len() - 1, bytes.len() - 20] {
        assert_refused(&bytes[..length], "runs past the end of the payload");
    }
    assert_refused(&[], "ends before the material count");
    assert_refused(&bytes[..2], "ends before the material count");
}

#[test]
fn trailing_bytes_are_refused() {
    let mut bytes = box_payload().encode();
    bytes.push(0);
    assert_refused(&bytes, "bytes after the last record");
}

#[test]
fn a_record_longer_than_its_contents_is_refused() {
    let payload = box_payload();
    let mut bytes = payload.material_count.to_le_bytes().to_vec();
    bytes.extend(&payload.materials);
    bytes.extend(1u32.to_le_bytes());
    let mut body = payload.records[0].encode();
    body.push(0);
    bytes.extend((body.len() as u32).to_le_bytes());
    bytes.extend(body);
    assert_refused(&bytes, "length does not match its contents");
}

#[test]
fn jolt_bytes_jolt_does_not_read_are_refused() {
    let mut payload = box_payload();
    payload.records[0].jolt.push(0);
    assert_refused(&payload.encode(), "Jolt read fewer bytes");
}

#[test]
fn jolt_bytes_that_end_early_are_refused_by_jolt() {
    let mut payload = box_payload();
    payload.records[0].jolt.pop();
    assert_refused(&payload.encode(), "Jolt refused a record");
}

#[test]
fn a_record_outside_the_root_is_refused() {
    let mut payload = box_payload();
    payload.records.push(payload.records[0].clone());
    assert_refused(&payload.encode(), "not part of the root shape");
}

#[test]
fn a_payload_without_shapes_is_refused() {
    let mut payload = box_payload();
    payload.records.clear();
    assert_refused(&payload.encode(), "holds no shape");
}

#[test]
fn the_version_is_set() {
    // SAFETY: a plain function without arguments.
    assert!(unsafe { JPH_Shape_GetBinaryStateVersion() } >= 1);
}
