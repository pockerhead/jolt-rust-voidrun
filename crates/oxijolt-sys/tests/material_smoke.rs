//! Smoke tests for the material functions of the joltc additions: materials with user data,
//! convex shape settings with a material and heightfield settings with a material list.

mod framework;

use std::ptr::{null, null_mut};

use framework::*;
use oxijolt_sys::*;

/// Jolt's `Color::sGrey`, the colour of a material without its own.
const GREY: u32 = 0xFF80_8080;
/// `JPH::SubShapeID` of a shape's root: every bit set.
const ROOT_SUB_SHAPE: JPH_SubShapeID = 0xFFFF_FFFF;

fn material(user_data: u64) -> *mut JPH_PhysicsMaterial {
    init();
    // SAFETY: Jolt is initialised; the name is a NUL-terminated literal. The caller owns the
    // returned reference.
    let material = unsafe { JPH_PhysicsMaterial_Create2(c"material".as_ptr(), GREY, user_data) };
    assert!(!material.is_null());
    material
}

/// The user data of `material`, or `None` when it has none.
fn user_data(material: *const JPH_PhysicsMaterial) -> Option<u64> {
    let mut value = 7;
    // SAFETY: `material` is null or live; `value` is a live local.
    let found = unsafe { JPH_PhysicsMaterial_GetUserData(material, &mut value) };
    if found {
        Some(value)
    } else {
        assert_eq!(value, 7, "the out value is left untouched");
        None
    }
}

/// A box created through its settings with `material` (null: the default material).
fn box_with_material(material: *const JPH_PhysicsMaterial) -> *mut JPH_Shape {
    let half_extent = vec3(0.5, 0.5, 0.5);
    // SAFETY: Jolt is initialised (`material` or `init`); `half_extent` is a live local. Box
    // settings derive from convex shape settings with single inheritance, as joltc's own casts
    // assume. The settings' reference is released after the shape took its own.
    unsafe {
        init();
        let settings = JPH_BoxShapeSettings_Create(&half_extent, 0.05);
        JPH_ConvexShapeSettings_SetMaterial(settings.cast(), material);
        let shape = JPH_BoxShapeSettings_CreateShape(settings);
        JPH_ShapeSettings_Destroy(settings.cast());
        assert!(!shape.is_null());
        shape.cast()
    }
}

#[test]
fn user_data_round_trips() {
    for value in [0, 42, u64::MAX] {
        let material = material(value);
        assert_eq!(user_data(material), Some(value));
        // SAFETY: the material is live; this releases the test's reference.
        unsafe { JPH_PhysicsMaterial_Destroy(material) };
    }
}

#[test]
fn other_materials_have_no_user_data() {
    init();
    // SAFETY: Jolt is initialised; the name is a NUL-terminated literal.
    let plain = unsafe { JPH_PhysicsMaterial_Create(c"plain".as_ptr(), GREY) };
    assert_eq!(user_data(plain), None);
    assert_eq!(user_data(null()), None);
    let shape = box_with_material(null());
    // SAFETY: the shape is live; the getter reads its material.
    let default = unsafe { JPH_Shape_GetMaterial(shape, ROOT_SUB_SHAPE) };
    assert!(!default.is_null(), "Jolt returns PhysicsMaterial::sDefault");
    assert_eq!(user_data(default), None);
    // SAFETY: both are live and this test holds one reference to each.
    unsafe {
        JPH_PhysicsMaterial_Destroy(plain);
        JPH_Shape_Destroy(shape);
    }
}

#[test]
fn convex_shape_settings_take_the_material() {
    let material = material(5);
    let shape = box_with_material(material);
    // SAFETY: the shape is live.
    let found = unsafe { JPH_Shape_GetMaterial(shape, ROOT_SUB_SHAPE) };
    assert_eq!(found, material.cast_const());
    // The shape keeps its own reference: the material survives the test's release.
    // SAFETY: the material is live; this releases the test's reference.
    unsafe { JPH_PhysicsMaterial_Destroy(material) };
    // SAFETY: the shape is live and holds a reference to the material.
    let found = unsafe { JPH_Shape_GetMaterial(shape, ROOT_SUB_SHAPE) };
    assert_eq!(user_data(found), Some(5));
    // SAFETY: the shape is live and this test holds one reference.
    unsafe { JPH_Shape_Destroy(shape) };
}

/// A flat 3 x 3 heightfield with two materials and indices `[0, 1, 1, 0]`.
fn two_material_field(
    materials: &[*const JPH_PhysicsMaterial],
) -> *mut JPH_HeightFieldShapeSettings {
    init();
    let samples = [0.0f32; 9];
    let indices = [0u8, 1, 1, 0];
    let (offset, scale) = (vec3(0.0, 0.0, 0.0), vec3(1.0, 1.0, 1.0));
    // SAFETY: Jolt is initialised; every array is live for the call and holds the counts Jolt
    // reads (9 samples, 4 indices, `materials.len()` materials).
    unsafe {
        JPH_HeightFieldShapeSettings_Create2(
            samples.as_ptr(),
            &offset,
            &scale,
            3,
            indices.as_ptr(),
            materials.as_ptr(),
            materials.len() as u32,
        )
    }
}

#[test]
fn height_field_cells_resolve_their_materials() {
    let (a, b) = (material(10), material(11));
    let settings = two_material_field(&[a, b]);
    assert!(!settings.is_null());
    // SAFETY: the settings are live; the materials list holds its own references.
    let shape = unsafe {
        JPH_PhysicsMaterial_Destroy(a);
        JPH_PhysicsMaterial_Destroy(b);
        let shape = JPH_HeightFieldShapeSettings_CreateShape(settings);
        JPH_ShapeSettings_Destroy(settings.cast());
        shape
    };
    assert!(!shape.is_null());
    for (x, y, expected) in [(0, 0, 10), (1, 0, 11), (0, 1, 11), (1, 1, 10)] {
        // SAFETY: the shape is live and (x, y) is a cell of the field.
        let found = unsafe { JPH_HeightFieldShape_GetMaterial(shape, x, y) };
        assert_eq!(user_data(found), Some(expected), "cell ({x}, {y})");
    }
    // SAFETY: the shape is live and this test holds one reference.
    unsafe { JPH_Shape_Destroy(shape.cast()) };
}

#[test]
fn height_field_without_materials_is_refused() {
    assert!(two_material_field(&[]).is_null());
    let a = material(1);
    let samples = [0.0f32; 9];
    let (offset, scale) = (vec3(0.0, 0.0, 0.0), vec3(1.0, 1.0, 1.0));
    let materials = [a.cast_const()];
    // SAFETY: Jolt is initialised; the arrays are live; null indices and a null list are refused
    // before Jolt reads anything.
    unsafe {
        let no_indices = JPH_HeightFieldShapeSettings_Create2(
            samples.as_ptr(),
            &offset,
            &scale,
            3,
            null(),
            materials.as_ptr(),
            1,
        );
        assert!(no_indices.is_null());
        let no_list = JPH_HeightFieldShapeSettings_Create2(
            samples.as_ptr(),
            &offset,
            &scale,
            3,
            [0u8; 4].as_ptr(),
            null_mut(),
            1,
        );
        assert!(no_list.is_null());
        JPH_PhysicsMaterial_Destroy(a);
    }
}
