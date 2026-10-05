//! Smoke tests for the buoyancy functions of the joltc additions: the submerged volume and the
//! volume overload of Jolt's buoyancy impulse.

mod framework;

use std::ptr::null;

use framework::*;
use oxijolt_sys::*;

/// What `JPH_Body_GetSubmergedVolume` returned: the result, total and submerged volume and the
/// centre of buoyancy relative to the centre of mass.
#[derive(Debug)]
struct Submerged {
    found: bool,
    total: f32,
    submerged: f32,
    center: JPH_Vec3,
}

/// Runs `f` with body `id` of `world` locked for writing.
fn with_body<R>(world: &TestWorld, id: JPH_BodyID, f: impl FnOnce(*mut JPH_Body) -> R) -> R {
    // SAFETY: the lock interface belongs to the live system and joltc copies the id. The body
    // exists, so the lock yields it, locked until the lock is destroyed after `f`.
    unsafe {
        let lock_interface = JPH_PhysicsSystem_GetBodyLockInterface(world.system());
        let lock = JPH_BodyLockInterface_LockMultiWrite(lock_interface, &id, 1);
        let body = JPH_BodyLockMultiWrite_GetBody(lock, 0);
        assert!(!body.is_null());
        let result = f(body);
        JPH_BodyLockMultiWrite_Destroy(lock);
        result
    }
}

/// The submerged volume of body `id` under a surface at height `surface_y` with normal +Y.
/// The outputs start as a canary so the test sees that every one is written.
fn submerged(world: &TestWorld, id: JPH_BodyID, surface_y: Real) -> Submerged {
    let surface = rvec3(0.0, surface_y, 0.0);
    let normal = vec3(0.0, 1.0, 0.0);
    with_body(world, id, |body| {
        let mut result = Submerged {
            found: false,
            total: -1.0,
            submerged: -1.0,
            center: vec3(-1.0, -1.0, -1.0),
        };
        // SAFETY: `body` is locked for the call; the inputs and outputs are live locals.
        result.found = unsafe {
            JPH_Body_GetSubmergedVolume(
                body,
                &surface,
                &normal,
                &mut result.total,
                &mut result.submerged,
                &mut result.center,
            )
        };
        result
    })
}

/// Creates a body of `shape` at `position`, adds it awake to `world` and returns its id. The
/// caller keeps its own reference to `shape`.
fn create_body(
    world: &TestWorld,
    shape: *const JPH_Shape,
    position: JPH_RVec3,
    motion_type: JPH_MotionType,
) -> JPH_BodyID {
    let rotation = quat_identity();
    let layer = if motion_type == JPH_MotionType_Static {
        OL_NON_MOVING
    } else {
        OL_MOVING
    };
    // SAFETY: the shape is live; the creation settings take their own reference to it and are
    // destroyed after the body took its own. The vectors are live locals.
    let id = unsafe {
        let settings =
            JPH_BodyCreationSettings_Create3(shape, &position, &rotation, motion_type, layer);
        let id = JPH_BodyInterface_CreateAndAddBody(
            world.body_interface(),
            settings,
            JPH_Activation_Activate,
        );
        JPH_BodyCreationSettings_Destroy(settings);
        id
    };
    assert_ne!(id, u32::MAX, "body creation failed");
    id
}

/// A box shape of `half_extent` holding one reference for the caller.
fn box_shape(half_extent: JPH_Vec3) -> *mut JPH_Shape {
    init();
    // SAFETY: Jolt is initialised and `half_extent` is a live local.
    let shape = unsafe { JPH_BoxShape_Create(&half_extent, JPH_DEFAULT_CONVEX_RADIUS as f32) };
    assert!(!shape.is_null());
    shape.cast()
}

/// A flat 4 x 4 heightfield shape holding one reference for the caller.
fn height_field_shape() -> *mut JPH_Shape {
    init();
    let samples = [0.0_f32; 16];
    let (offset, scale) = (vec3(0.0, 0.0, 0.0), vec3(1.0, 1.0, 1.0));
    // SAFETY: Jolt is initialised; `samples` holds 4 x 4 samples and the vectors are live
    // locals. The settings' reference is released after the shape took its own.
    let shape = unsafe {
        let settings =
            JPH_HeightFieldShapeSettings_Create(samples.as_ptr(), &offset, &scale, 4, null());
        let shape = JPH_HeightFieldShapeSettings_CreateShape(settings);
        JPH_ShapeSettings_Destroy(settings.cast());
        shape
    };
    assert!(!shape.is_null());
    shape.cast()
}

/// A static compound of `children` (shape and position), holding one reference for the caller;
/// the caller keeps its references to the children.
fn static_compound(children: &[(*mut JPH_Shape, JPH_Vec3)]) -> *mut JPH_Shape {
    let rotation = quat_identity();
    // SAFETY: the children are live; the settings take their own references and are released
    // after the compound took its own.
    let shape = unsafe {
        let settings = JPH_StaticCompoundShapeSettings_Create();
        for (child, position) in children {
            JPH_CompoundShapeSettings_AddShape2(settings.cast(), position, &rotation, *child, 0);
        }
        let mut error = [0_u8; 1];
        let shape =
            JPH_ShapeSettings_CreateShapeWithError(settings.cast(), error.as_mut_ptr().cast(), 0);
        JPH_ShapeSettings_Destroy(settings.cast());
        shape
    };
    assert!(!shape.is_null());
    shape
}

/// An offset centre of mass decorator around a static compound of a box and a nested static
/// compound of a box and `leaf`; holds one reference for the caller, who keeps its own to `leaf`.
fn decorated_nested_compound(leaf: *mut JPH_Shape) -> *mut JPH_Shape {
    let cube = box_shape(vec3(0.5, 0.5, 0.5));
    let inner = static_compound(&[(cube, vec3(0.0, 0.0, 0.0)), (leaf, vec3(2.0, 0.0, 0.0))]);
    let outer = static_compound(&[(cube, vec3(0.0, 0.0, 0.0)), (inner, vec3(0.0, 0.0, 3.0))]);
    let offset = vec3(0.1, 0.0, 0.0);
    // SAFETY: `outer` is live; the decorator settings take their own reference and are released
    // after the decorator took its own. This function's references are released once each.
    let shape = unsafe {
        let decorator = JPH_OffsetCenterOfMassShapeSettings_Create2(&offset, outer);
        let shape = JPH_OffsetCenterOfMassShapeSettings_CreateShape(decorator);
        JPH_ShapeSettings_Destroy(decorator.cast());
        shape
    };
    [outer, inner, cube].into_iter().for_each(release);
    assert!(!shape.is_null());
    let shape: *mut JPH_Shape = shape.cast();
    assert_nested_compound(shape);
    shape
}

/// Asserts that `shape` is the structure `decorated_nested_compound` builds: a decorator around
/// a compound of two children, the second a compound of two children.
fn assert_nested_compound(shape: *const JPH_Shape) {
    // SAFETY: `shape` is live; each getter is called on a shape of the subtype it takes, checked
    // just before, and the child pointers stay valid while their parents live.
    unsafe {
        assert_eq!(
            JPH_Shape_GetSubType(shape),
            JPH_ShapeSubType_OffsetCenterOfMass
        );
        let outer = JPH_DecoratedShape_GetInnerShape(shape.cast());
        assert_eq!(JPH_Shape_GetSubType(outer), JPH_ShapeSubType_StaticCompound);
        assert_eq!(JPH_CompoundShape_GetNumSubShapes(outer.cast()), 2);
        let mut inner = null();
        JPH_CompoundShape_GetSubShape(
            outer.cast(),
            1,
            &mut inner,
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
        );
        assert_eq!(JPH_Shape_GetSubType(inner), JPH_ShapeSubType_StaticCompound);
        assert_eq!(JPH_CompoundShape_GetNumSubShapes(inner.cast()), 2);
    }
}

/// A plane shape through the origin with normal +Y, holding one reference for the caller.
fn plane_shape() -> *mut JPH_Shape {
    init();
    let plane = JPH_Plane {
        normal: vec3(0.0, 1.0, 0.0),
        distance: 0.0,
    };
    // SAFETY: Jolt is initialised and `plane` is a live local. The settings' reference is
    // released after the shape took its own.
    let shape = unsafe {
        let settings = JPH_PlaneShapeSettings_Create(&plane, null(), 100.0);
        let shape = JPH_PlaneShapeSettings_CreateShape(settings);
        JPH_ShapeSettings_Destroy(settings.cast());
        shape
    };
    assert!(!shape.is_null());
    shape.cast()
}

/// Releases the caller's reference to `shape`.
fn release(shape: *mut JPH_Shape) {
    // SAFETY: the caller holds one reference, released exactly once here.
    unsafe { JPH_Shape_Destroy(shape) };
}

/// Calls `JPH_Body_ApplyBuoyancyImpulse2` with a half-submerged unit volume on body `id`.
fn apply_volume_buoyancy(world: &TestWorld, id: JPH_BodyID, total_volume: f32) -> bool {
    let (center, fluid, gravity) = (
        vec3(0.0, -0.25, 0.0),
        vec3(0.0, 0.0, 0.0),
        vec3(0.0, -9.81, 0.0),
    );
    with_body(world, id, |body| {
        // SAFETY: `body` is locked for the call; the vectors are live locals.
        unsafe {
            JPH_Body_ApplyBuoyancyImpulse2(
                body,
                total_volume,
                total_volume / 2.0,
                &center,
                2.0,
                0.5,
                0.01,
                &fluid,
                &gravity,
                1.0 / 60.0,
            )
        }
    })
}

/// Linear and angular velocity of body `id`.
fn velocities(world: &TestWorld, id: JPH_BodyID) -> (JPH_Vec3, JPH_Vec3) {
    let (mut linear, mut angular) = (vec3(0.0, 0.0, 0.0), vec3(0.0, 0.0, 0.0));
    // SAFETY: the body interface belongs to the live world; the outputs are live locals.
    unsafe {
        JPH_BodyInterface_GetLinearVelocity(world.body_interface(), id, &mut linear);
        JPH_BodyInterface_GetAngularVelocity(world.body_interface(), id, &mut angular);
    }
    (linear, angular)
}

#[test]
fn a_box_reports_its_volume_cut_by_the_surface() {
    let world = TestWorld::new(1);
    let shape = box_shape(vec3(1.0, 0.5, 1.0));
    let id = create_body(&world, shape, rvec3(0.0, 0.0, 0.0), JPH_MotionType_Dynamic);
    release(shape);

    let half = submerged(&world, id, 0.0);
    assert!(half.found);
    assert!((half.total - 4.0).abs() <= 1.0e-5, "{half:?}");
    assert!((half.submerged - 2.0).abs() <= 1.0e-5, "{half:?}");
    assert!((half.center.y + 0.25).abs() <= 1.0e-5, "{half:?}");
    assert!(
        half.center.x.abs() <= 1.0e-5 && half.center.z.abs() <= 1.0e-5,
        "{half:?}"
    );

    let under = submerged(&world, id, 1.0);
    assert!(under.found);
    assert_eq!(under.submerged, under.total);
    assert_eq!(
        [under.center.x, under.center.y, under.center.z],
        [0.0, 0.0, 0.0]
    );

    let above = submerged(&world, id, -1.0);
    assert!(above.found);
    assert_eq!(above.submerged, 0.0);
}

#[test]
fn static_and_kinematic_rigid_bodies_report_volumes() {
    let world = TestWorld::new(1);
    let shape = box_shape(vec3(1.0, 0.5, 1.0));
    for motion_type in [JPH_MotionType_Static, JPH_MotionType_Kinematic] {
        let id = create_body(&world, shape, rvec3(0.0, 0.0, 0.0), motion_type);
        let result = submerged(&world, id, 0.0);
        assert!(result.found, "{motion_type:?}");
        assert!((result.total - 4.0).abs() <= 1.0e-5, "{result:?}");
        assert!((result.submerged - 2.0).abs() <= 1.0e-5, "{result:?}");
    }
    release(shape);
}

#[test]
fn shapes_without_a_volume_and_soft_bodies_report_nothing() {
    let world = TestWorld::new(1);
    let (height_field, cube) = (height_field_shape(), box_shape(vec3(0.5, 0.5, 0.5)));
    // The same nested compound with a box in place of the heightfield has a volume, so the
    // refusal below comes from the heightfield alone.
    let solid = decorated_nested_compound(cube);
    let solid_id = create_body(&world, solid, rvec3(0.0, 0.0, 0.0), JPH_MotionType_Static);
    let solid_volume = submerged(&world, solid_id, 10.0);
    assert!(solid_volume.found, "{solid_volume:?}");
    assert!(
        (solid_volume.total - 3.0).abs() <= 1.0e-5,
        "{solid_volume:?}"
    );

    let static_only = [
        height_field_shape(),
        plane_shape(),
        decorated_nested_compound(height_field),
    ];
    let mut ids: Vec<JPH_BodyID> = static_only
        .iter()
        .map(|&shape| create_body(&world, shape, rvec3(0.0, 0.0, 0.0), JPH_MotionType_Static))
        .collect();
    [solid, cube, height_field]
        .into_iter()
        .chain(static_only)
        .for_each(release);
    ids.push(soft_body(&world));

    for id in ids {
        let result = submerged(&world, id, 10.0);
        assert!(!result.found, "{result:?}");
        assert_eq!(result.total, 0.0);
        assert_eq!(result.submerged, 0.0);
        assert_eq!(
            [result.center.x, result.center.y, result.center.z],
            [0.0, 0.0, 0.0]
        );
    }
}

/// A dynamic soft body of four vertices at the origin, awake in `world`.
fn soft_body(world: &TestWorld) -> JPH_BodyID {
    let vertex = |x, y, z| JPH_SoftVertex {
        position: vec3(x, y, z),
        velocity: vec3(0.0, 0.0, 0.0),
        invMass: 1.0,
    };
    let vertices = [
        vertex(0.0, 0.0, 0.0),
        vertex(1.0, 0.0, 0.0),
        vertex(0.0, 0.0, 1.0),
        vertex(0.0, 1.0, 0.0),
    ];
    let (position, rotation) = (rvec3(0.0, 0.0, 0.0), quat_identity());
    // SAFETY: Jolt is initialised (the world exists); `vertices` holds the count passed. The
    // creation settings take their own reference to the shared settings, the body its own; both
    // of this function's references are released before it returns.
    let id = unsafe {
        let settings = JPH_SoftBodySharedSettings_Create();
        JPH_SoftBodySharedSettings_AddVertices(settings, vertices.as_ptr(), vertices.len() as u32);
        JPH_SoftBodySharedSettings_Optimize(settings);
        let creation =
            JPH_SoftBodyCreationSettings_Create2(settings, &position, &rotation, OL_MOVING);
        let id = JPH_BodyInterface_CreateAndAddSoftBody(
            world.body_interface(),
            creation,
            JPH_Activation_Activate,
        );
        JPH_SoftBodyCreationSettings_Destroy(creation);
        JPH_SoftBodySharedSettings_Destroy(settings);
        id
    };
    assert_ne!(id, u32::MAX, "soft body creation failed");
    id
}

#[test]
fn the_volume_overload_refuses_bodies_it_cannot_move() {
    let world = TestWorld::new(1);
    let shape = box_shape(vec3(0.5, 0.5, 0.5));
    let static_body = create_body(&world, shape, rvec3(0.0, 0.0, 0.0), JPH_MotionType_Static);
    let kinematic = create_body(
        &world,
        shape,
        rvec3(3.0, 0.0, 0.0),
        JPH_MotionType_Kinematic,
    );
    let dynamic = create_body(&world, shape, rvec3(6.0, 0.0, 0.0), JPH_MotionType_Dynamic);
    release(shape);
    let soft = soft_body(&world);

    for (id, total_volume) in [
        (static_body, 1.0),
        (kinematic, 1.0),
        (soft, 1.0),
        (dynamic, 0.0),
        (dynamic, -1.0),
    ] {
        let before = velocities(&world, id);
        assert!(
            !apply_volume_buoyancy(&world, id, total_volume),
            "{id} {total_volume}"
        );
        let after = velocities(&world, id);
        assert_eq!(format!("{before:?}"), format!("{after:?}"));
    }
    assert!(apply_volume_buoyancy(&world, dynamic, 1.0));
    assert!(velocities(&world, dynamic).0.y > 0.0);
}

#[test]
fn the_two_part_call_matches_joltcs_surface_call() {
    let pair = TestWorldPair::new();
    let shape = box_shape(vec3(0.5, 0.25, 0.4));
    let rotation = JPH_Quat {
        x: 0.0,
        y: 0.0,
        z: 0.247_403_96,
        w: 0.968_912_4,
    };
    let ids = [&pair.first, &pair.second].map(|world| {
        let id = create_body(world, shape, rvec3(0.3, 0.1, -0.2), JPH_MotionType_Dynamic);
        let (mut rotation, linear, mut angular) =
            (rotation, vec3(1.0, -2.0, 0.5), vec3(0.3, 1.5, -0.7));
        // SAFETY: the body interface belongs to the live world; joltc only reads the inputs,
        // live locals.
        unsafe {
            JPH_BodyInterface_SetRotation(
                world.body_interface(),
                id,
                &mut rotation,
                JPH_Activation_Activate,
            );
            JPH_BodyInterface_SetLinearVelocity(world.body_interface(), id, &linear);
            JPH_BodyInterface_SetAngularVelocity(world.body_interface(), id, &mut angular);
        }
        id
    });
    release(shape);

    let surface = rvec3(0.0, 0.2, 0.0);
    let normal = vec3(0.0, 1.0, 0.0);
    let (fluid, gravity) = (vec3(0.4, 0.0, -0.3), vec3(0.0, -9.81, 0.0));
    let (buoyancy, linear_drag, angular_drag, dt) = (1.7, 0.5, 0.05, 1.0 / 60.0);

    let surface_call = with_body(&pair.first, ids[0], |body| {
        // SAFETY: `body` is locked for the call; the inputs are live locals.
        unsafe {
            JPH_Body_ApplyBuoyancyImpulse(
                body,
                &surface,
                &normal,
                buoyancy,
                linear_drag,
                angular_drag,
                &fluid,
                &gravity,
                dt,
            )
        }
    });
    let volume = submerged(&pair.second, ids[1], 0.2);
    assert!(volume.found && volume.submerged > 0.0 && volume.submerged < volume.total);
    let volume_call = with_body(&pair.second, ids[1], |body| {
        // SAFETY: `body` is locked for the call; the inputs are live locals.
        unsafe {
            JPH_Body_ApplyBuoyancyImpulse2(
                body,
                volume.total,
                volume.submerged,
                &volume.center,
                buoyancy,
                linear_drag,
                angular_drag,
                &fluid,
                &gravity,
                dt,
            )
        }
    });
    assert!(surface_call && volume_call);

    let (first, second) = (
        velocities(&pair.first, ids[0]),
        velocities(&pair.second, ids[1]),
    );
    let components = |(l, a): (JPH_Vec3, JPH_Vec3)| [l.x, l.y, l.z, a.x, a.y, a.z];
    for (a, b) in components(first).into_iter().zip(components(second)) {
        assert!((a - b).abs() <= 1.0e-6, "{first:?} vs {second:?}");
    }
    assert!(components(first) != [1.0, -2.0, 0.5, 0.3, 1.5, -0.7]);
}
