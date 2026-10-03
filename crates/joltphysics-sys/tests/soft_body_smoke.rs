//! Smoke tests for the soft body functions of the joltc additions: constraint generation with
//! every vertex attribute, explicit constraints, and the vertex readout and setters of a body.

mod framework;

use framework::*;
use joltphysics_sys::*;

const DT: f32 = 1.0 / 60.0;
/// Vertices per side of the test cloth.
const SIDE: u32 = 3;
/// Distance between neighbouring cloth vertices in metres.
const SPACING: f32 = 0.5;

/// Index of the cloth vertex in column `x` and row `z`.
fn vertex_index(x: u32, z: u32) -> u32 {
    z * SIDE + x
}

/// A flat `SIDE` x `SIDE` cloth in the XZ plane; the vertices listed in `kinematic` get inverse
/// mass 0, the others 1. Two triangles per grid cell. The caller owns the returned reference.
fn cloth(kinematic: &[u32]) -> *mut JPH_SoftBodySharedSettings {
    let mut vertices = Vec::new();
    for z in 0..SIDE {
        for x in 0..SIDE {
            let index = vertex_index(x, z);
            vertices.push(JPH_SoftVertex {
                position: vec3(x as f32 * SPACING, 0.0, z as f32 * SPACING),
                velocity: vec3(0.0, 0.0, 0.0),
                invMass: if kinematic.contains(&index) { 0.0 } else { 1.0 },
            });
        }
    }
    let mut faces = Vec::new();
    for z in 0..SIDE - 1 {
        for x in 0..SIDE - 1 {
            let face = |a, b, c| JPH_SoftFace {
                vertex1: a,
                vertex2: b,
                vertex3: c,
                materialIndex: 0,
            };
            let (v00, v10) = (vertex_index(x, z), vertex_index(x + 1, z));
            let (v01, v11) = (vertex_index(x, z + 1), vertex_index(x + 1, z + 1));
            faces.push(face(v00, v01, v11));
            faces.push(face(v00, v11, v10));
        }
    }
    // SAFETY: Jolt is initialised (`init`); both arrays are live for the calls and hold the
    // counts passed. The settings are returned holding the one reference `_Create` added.
    unsafe {
        init();
        let settings = JPH_SoftBodySharedSettings_Create();
        JPH_SoftBodySharedSettings_AddVertices(settings, vertices.as_ptr(), vertices.len() as u32);
        JPH_SoftBodySharedSettings_AddFaces(settings, faces.as_ptr(), faces.len() as u32);
        settings
    }
}

fn default_attributes() -> JPH_SoftBodyVertexAttributes {
    let mut attributes = JPH_SoftBodyVertexAttributes {
        compliance: -1.0,
        shearCompliance: -1.0,
        bendCompliance: -1.0,
        lraType: JPH_SoftBodyLRAType_GeodesicDistance,
        lraMaxDistanceMultiplier: -1.0,
    };
    // SAFETY: `attributes` is a live local that the call overwrites.
    unsafe { JPH_SoftBodyVertexAttributes_Init(&mut attributes) };
    attributes
}

/// Generates constraints for `settings` with one attribute set for every vertex.
fn create_constraints(
    settings: *mut JPH_SoftBodySharedSettings,
    attributes: JPH_SoftBodyVertexAttributes,
    bend: JPH_SoftBodyBendType,
) {
    // SAFETY: `settings` is live; `attributes` is one live attribute set (count 1).
    unsafe { JPH_SoftBodySharedSettings_CreateConstraints2(settings, &attributes, 1, bend) };
}

/// Constraint counts: edge, dihedral bend, volume, LRA.
fn counts(settings: *const JPH_SoftBodySharedSettings) -> [u32; 4] {
    // SAFETY: `settings` is live; the getters only read it.
    unsafe {
        [
            JPH_SoftBodySharedSettings_GetEdgeConstraintCount(settings),
            JPH_SoftBodySharedSettings_GetDihedralBendConstraintCount(settings),
            JPH_SoftBodySharedSettings_GetVolumeConstraintCount(settings),
            JPH_SoftBodySharedSettings_GetLRAConstraintCount(settings),
        ]
    }
}

#[test]
fn vertex_attributes_init_returns_jolts_defaults() {
    init();
    let attributes = default_attributes();
    assert_eq!(attributes.compliance, 0.0);
    assert_eq!(attributes.shearCompliance, 0.0);
    assert_eq!(attributes.bendCompliance, f32::MAX);
    assert_eq!(attributes.lraType, JPH_SoftBodyLRAType_None);
    assert_eq!(attributes.lraMaxDistanceMultiplier, 1.0);
}

#[test]
fn create_constraints2_makes_bend_constraints_that_joltcs_call_does_not() {
    let joltc = cloth(&[]);
    // SAFETY: `joltc` is live.
    unsafe {
        JPH_SoftBodySharedSettings_CreateConstraints(joltc, 0.0, JPH_SoftBodyBendType_Dihedral)
    };
    let [edges, bends, _, _] = counts(joltc);
    assert!(edges > 0);
    assert_eq!(
        bends, 0,
        "joltc's call leaves the bend compliance at FLT_MAX"
    );

    let ours = cloth(&[]);
    let attributes = JPH_SoftBodyVertexAttributes {
        bendCompliance: 0.0,
        ..default_attributes()
    };
    create_constraints(ours, attributes, JPH_SoftBodyBendType_Dihedral);
    let [edges, bends, _, lras] = counts(ours);
    assert!(edges > 0);
    assert!(bends > 0);
    assert_eq!(lras, 0);
    // SAFETY: each settings object holds the one reference `_Create` added, released here.
    unsafe {
        JPH_SoftBodySharedSettings_Destroy(joltc);
        JPH_SoftBodySharedSettings_Destroy(ours);
    }
}

#[test]
fn long_range_attachments_need_kinematic_vertices() {
    let attributes = JPH_SoftBodyVertexAttributes {
        lraType: JPH_SoftBodyLRAType_GeodesicDistance,
        ..default_attributes()
    };
    let free = cloth(&[]);
    create_constraints(free, attributes, JPH_SoftBodyBendType_None);
    assert_eq!(counts(free)[3], 0);

    let pinned = cloth(&[vertex_index(0, 0), vertex_index(SIDE - 1, 0)]);
    create_constraints(pinned, attributes, JPH_SoftBodyBendType_None);
    assert!(counts(pinned)[3] > 0);
    // SAFETY: each settings object holds the one reference `_Create` added, released here.
    unsafe {
        JPH_SoftBodySharedSettings_Destroy(free);
        JPH_SoftBodySharedSettings_Destroy(pinned);
    }
}

#[test]
fn explicit_constraints_are_added_and_prepared() {
    let settings = cloth(&[]);
    let before = counts(settings);
    assert_eq!(before, [0; 4]);
    // SAFETY: `settings` is live; every index names one of its vertices, and the four vertices
    // of the volume constraint are not coplanar after the lift below.
    unsafe {
        JPH_SoftBodySharedSettings_AddEdgeConstraint(settings, 0, 1, 0.0);
        JPH_SoftBodySharedSettings_AddEdgeConstraint(settings, 1, 2, 0.0);
        JPH_SoftBodySharedSettings_AddDihedralBendConstraint(settings, 0, 4, 1, 3, 0.0);
        JPH_SoftBodySharedSettings_CalculateEdgeLengths(settings);
        JPH_SoftBodySharedSettings_CalculateBendConstraintConstants(settings);
    }
    assert_eq!(counts(settings), [2, 1, 0, 0]);

    let tetrahedron = tetrahedron_settings();
    // SAFETY: `tetrahedron` is live and has the four vertices the constraint names.
    unsafe {
        JPH_SoftBodySharedSettings_AddVolumeConstraint(tetrahedron, 0, 1, 2, 3, 0.0);
        JPH_SoftBodySharedSettings_CalculateVolumeConstraintVolumes(tetrahedron);
        JPH_SoftBodySharedSettings_Optimize(tetrahedron);
    }
    assert_eq!(counts(tetrahedron)[2], 1);
    // SAFETY: each settings object holds the one reference `_Create` added, released here.
    unsafe {
        JPH_SoftBodySharedSettings_Destroy(settings);
        JPH_SoftBodySharedSettings_Destroy(tetrahedron);
    }
}

/// Four vertices of a unit tetrahedron, no faces.
fn tetrahedron_settings() -> *mut JPH_SoftBodySharedSettings {
    init();
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
    // SAFETY: Jolt is initialised; `vertices` is live and holds the count passed.
    unsafe {
        let settings = JPH_SoftBodySharedSettings_Create();
        JPH_SoftBodySharedSettings_AddVertices(settings, vertices.as_ptr(), vertices.len() as u32);
        settings
    }
}

/// Creates a soft body from `settings` (which are optimised first) at `position` and adds it,
/// active, to the world. The creation settings are destroyed before returning; the body keeps
/// its own reference to the shared settings.
fn create_soft_body(
    world: &TestWorld,
    settings: *mut JPH_SoftBodySharedSettings,
    position: JPH_RVec3,
) -> JPH_BodyID {
    let rotation = quat_identity();
    // SAFETY: `settings` is live; the creation settings take their own reference to it and are
    // deleted after the body took its own. `position` and `rotation` are live locals.
    let id = unsafe {
        JPH_SoftBodySharedSettings_Optimize(settings);
        let creation =
            JPH_SoftBodyCreationSettings_Create2(settings, &position, &rotation, OL_MOVING);
        let id = JPH_BodyInterface_CreateAndAddSoftBody(
            world.body_interface(),
            creation,
            JPH_Activation_Activate,
        );
        JPH_SoftBodyCreationSettings_Destroy(creation);
        id
    };
    assert_ne!(id, u32::MAX, "soft body creation failed");
    id
}

/// Runs `f` with body `id` locked for writing.
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

/// World positions, world velocities and inverse masses of the vertices of soft body `id`.
fn vertices(world: &TestWorld, id: JPH_BodyID) -> (Vec<JPH_RVec3>, Vec<JPH_Vec3>, Vec<f32>) {
    with_body(world, id, |body| {
        // SAFETY: `body` is locked for the call; each output holds `count` elements.
        unsafe {
            let count = JPH_Body_GetSoftBodyVertexCount(body) as usize;
            let mut positions = vec![rvec3(0.0, 0.0, 0.0); count];
            let mut velocities = vec![vec3(0.0, 0.0, 0.0); count];
            let mut inverse_masses = vec![0.0; count];
            JPH_Body_GetSoftBodyVertices(
                body,
                positions.as_mut_ptr(),
                velocities.as_mut_ptr(),
                inverse_masses.as_mut_ptr(),
                count as u32,
            );
            (positions, velocities, inverse_masses)
        }
    })
}

fn inverse_mass(world: &TestWorld, id: JPH_BodyID) -> f32 {
    with_body(world, id, |body| {
        // SAFETY: `body` is locked and dynamic (soft bodies are), so it has motion properties.
        unsafe { JPH_MotionProperties_GetInverseMassUnchecked(JPH_Body_GetMotionProperties(body)) }
    })
}

#[test]
fn soft_body_vertices_read_back_and_accept_writes() {
    let world = TestWorld::new(2);
    let pinned = vertex_index(0, 0);
    let free = vertex_index(SIDE - 1, SIDE - 1);
    let settings = cloth(&[pinned]);
    let attributes = JPH_SoftBodyVertexAttributes {
        bendCompliance: 0.0,
        ..default_attributes()
    };
    create_constraints(settings, attributes, JPH_SoftBodyBendType_Distance);
    let id = create_soft_body(&world, settings, rvec3(0.0, 2.0, 0.0));
    // SAFETY: the body keeps its own reference; this releases the test's.
    unsafe { JPH_SoftBodySharedSettings_Destroy(settings) };

    let (start, _, inverse_masses) = vertices(&world, id);
    assert_eq!(start.len(), (SIDE * SIDE) as usize);
    assert_eq!(inverse_masses[pinned as usize], 0.0);
    assert_eq!(inverse_masses[free as usize], 1.0);
    assert_eq!(
        inverse_mass(&world, id),
        0.0,
        "a kinematic vertex pins the mass"
    );

    world.step(DT);
    let (after, _, inverse_masses) = vertices(&world, id);
    let (p0, p1) = (start[pinned as usize], after[pinned as usize]);
    let moved =
        |a: JPH_RVec3, b: JPH_RVec3| (a.x - b.x).abs() + (a.y - b.y).abs() + (a.z - b.z).abs();
    assert!(moved(p0, p1) < 1.0e-5, "the kinematic vertex stays");
    assert_eq!(inverse_masses[pinned as usize], 0.0);
    assert!(
        after[free as usize].y < start[free as usize].y,
        "a free vertex falls"
    );

    let velocity = vec3(0.25, -1.5, 3.0);
    with_body(&world, id, |body| {
        // SAFETY: `body` is locked for writing; `velocity` is a live local.
        unsafe { JPH_Body_SetSoftBodyVertexVelocity(body, free, &velocity) }
    });
    let (_, velocities, _) = vertices(&world, id);
    let read = velocities[free as usize];
    // The body rotation is identity, so the world-local conversion is exact.
    assert_eq!(
        [read.x, read.y, read.z].map(f32::to_bits),
        [velocity.x, velocity.y, velocity.z].map(f32::to_bits)
    );

    with_body(&world, id, |body| {
        // SAFETY: `body` is locked for writing and `pinned` is a vertex index.
        unsafe { JPH_Body_SetSoftBodyVertexInvMass(body, pinned, 0.5) }
    });
    let total_mass = 2.0 + (SIDE * SIDE - 1) as f32;
    let expected = 1.0 / total_mass;
    let actual = inverse_mass(&world, id);
    assert!(
        (actual - expected).abs() <= expected * 1.0e-5,
        "{actual} vs {expected}"
    );
    with_body(&world, id, |body| {
        // SAFETY: as above.
        unsafe { JPH_Body_SetSoftBodyVertexInvMass(body, pinned, 0.0) }
    });
    assert_eq!(inverse_mass(&world, id), 0.0);

    // SAFETY: the body exists and nothing else removes it.
    unsafe { JPH_BodyInterface_RemoveAndDestroyBody(world.body_interface(), id) };
}

#[test]
fn soft_body_calls_on_a_rigid_body_do_nothing() {
    let world = TestWorld::new(1);
    let id = create_box(
        world.body_interface(),
        vec3(0.5, 0.5, 0.5),
        rvec3(0.0, 0.0, 0.0),
        JPH_MotionType_Dynamic,
        OL_MOVING,
        JPH_Activation_Activate,
    );
    let marker = rvec3(7.0, 8.0, 9.0);
    let mut positions = [marker; 2];
    let mut velocities = [vec3(1.0, 2.0, 3.0); 2];
    let mut inverse_masses = [42.0_f32; 2];
    let velocity = vec3(1.0, 1.0, 1.0);
    let mass_before = inverse_mass(&world, id);
    with_body(&world, id, |body| {
        // SAFETY: `body` is locked for writing; every output holds two elements.
        unsafe {
            assert_eq!(JPH_Body_GetSoftBodyVertexCount(body), 0);
            JPH_Body_GetSoftBodyVertices(
                body,
                positions.as_mut_ptr(),
                velocities.as_mut_ptr(),
                inverse_masses.as_mut_ptr(),
                2,
            );
            JPH_Body_SetSoftBodyVertexVelocity(body, 0, &velocity);
            JPH_Body_SetSoftBodyVertexInvMass(body, 0, 0.0);
        }
    });
    assert_eq!(positions[0].x, marker.x);
    assert_eq!(velocities[1].z, 3.0);
    assert_eq!(inverse_masses, [42.0; 2]);
    assert_eq!(inverse_mass(&world, id), mass_before);
}

#[test]
fn out_of_range_vertex_writes_do_nothing() {
    let world = TestWorld::new(1);
    let settings = cloth(&[]);
    let id = create_soft_body(&world, settings, rvec3(0.0, 0.0, 0.0));
    // SAFETY: the body keeps its own reference; this releases the test's.
    unsafe { JPH_SoftBodySharedSettings_Destroy(settings) };
    let before = inverse_mass(&world, id);
    let velocity = vec3(1.0, 0.0, 0.0);
    with_body(&world, id, |body| {
        // SAFETY: `body` is locked for writing; the index is past the last vertex.
        unsafe {
            JPH_Body_SetSoftBodyVertexVelocity(body, SIDE * SIDE, &velocity);
            JPH_Body_SetSoftBodyVertexInvMass(body, SIDE * SIDE, 0.0);
        }
    });
    assert_eq!(inverse_mass(&world, id), before);
    // Fewer outputs than vertices: only that many are written.
    let mut one = [rvec3(-1.0, -1.0, -1.0); 2];
    with_body(&world, id, |body| {
        // SAFETY: `body` is locked; `one` holds the one position asked for (and a spare).
        unsafe {
            JPH_Body_GetSoftBodyVertices(
                body,
                one.as_mut_ptr(),
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                1,
            )
        }
    });
    assert_eq!(one[0].x, 0.0);
    assert_eq!(one[1].x, -1.0);
}

#[test]
fn a_soft_body_outlives_the_settings_it_was_made_from() {
    let world = TestWorld::new(2);
    let settings = cloth(&[]);
    create_constraints(settings, default_attributes(), JPH_SoftBodyBendType_None);
    let id = create_soft_body(&world, settings, rvec3(0.0, 1.0, 0.0));
    // SAFETY: the body keeps its own reference; this releases the test's, which was the last
    // other one (the creation settings are already gone).
    unsafe { JPH_SoftBodySharedSettings_Destroy(settings) };
    for _ in 0..5 {
        world.step(DT);
    }
    assert!(vertices(&world, id).0.iter().all(|p| p.y.is_finite()));
    // SAFETY: the body exists and nothing else removes it.
    unsafe { JPH_BodyInterface_RemoveAndDestroyBody(world.body_interface(), id) };
}

/// joltc's own `JPH_Body_GetSoftBodyVertexPositions` rounds world positions to `float`; the
/// extension keeps `Real`.
#[cfg(feature = "double-precision")]
#[test]
fn vertex_positions_keep_double_precision() {
    let world = TestWorld::new(1);
    let settings = cloth(&[0]);
    let x = 33_554_432.5; // 2^25 + 0.5, not representable as f32
    let id = create_soft_body(&world, settings, rvec3(x, 0.0, 0.0));
    // SAFETY: the body keeps its own reference; this releases the test's.
    unsafe { JPH_SoftBodySharedSettings_Destroy(settings) };
    let (positions, _, _) = vertices(&world, id);
    assert!((positions[0].x - x).abs() <= 1.0e-6, "{}", positions[0].x);
}
