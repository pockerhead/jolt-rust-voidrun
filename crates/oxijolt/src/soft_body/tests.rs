use super::shared::{is_edge_length, EDGE_LENGTH_RULE, TOTAL_MASS_RULE, VERTEX_INVERSE_MASS_RULE};
use super::*;
use crate::limits::LINEAR_VELOCITY_RULE;
use crate::owned::Owned;
use crate::world::ensure_initialized;
use crate::{ObjectLayer, Quat, RVec3};

fn triangle() -> Vec<SoftBodyVertex> {
    vec![
        SoftBodyVertex::new(Vec3::new(0.0, 0.0, 0.0)),
        SoftBodyVertex::new(Vec3::new(1.0, 0.0, 0.0)),
        SoftBodyVertex::new(Vec3::new(0.0, 0.0, 1.0)),
    ]
}

fn rejected(builder: SoftBodySharedSettingsBuilder) -> &'static str {
    match builder.build() {
        Err(SoftBodyError::InvalidValue(what)) => what,
        Err(other) => panic!("unexpected error {other:?}"),
        Ok(_) => panic!("accepted"),
    }
}

#[test]
fn default_settings_match_jolt() {
    assert!(ensure_initialized());
    // SAFETY: Jolt is initialised, and the handle takes over the new settings.
    let jolt = unsafe { Owned::from_raw(JPH_SoftBodyCreationSettings_Create()) }.unwrap();
    let ours = SoftBodySettings::default();
    let ptr = jolt.as_ptr();
    let mut position = RVec3::new(1.0, 1.0, 1.0).to_jph();
    let mut rotation = Quat::from_xyzw(1.0, 0.0, 0.0, 0.0).to_jph();
    // SAFETY: `ptr` is the live settings object owned by `jolt`; getters only read it and
    // write live locals.
    unsafe {
        JPH_SoftBodyCreationSettings_GetPosition(ptr, &mut position);
        JPH_SoftBodyCreationSettings_GetRotation(ptr, &mut rotation);
        assert_eq!(RVec3::from_jph(position), ours.position);
        assert_eq!(Quat::from_jph(rotation), ours.rotation);
        assert_eq!(
            JPH_SoftBodyCreationSettings_GetNumIterations(ptr),
            ours.num_iterations
        );
        assert_eq!(
            JPH_SoftBodyCreationSettings_GetLinearDamping(ptr),
            ours.linear_damping
        );
        assert_eq!(
            JPH_SoftBodyCreationSettings_GetMaxLinearVelocity(ptr),
            ours.max_linear_velocity
        );
        assert_eq!(
            JPH_SoftBodyCreationSettings_GetRestitution(ptr),
            ours.restitution
        );
        assert_eq!(JPH_SoftBodyCreationSettings_GetFriction(ptr), ours.friction);
        assert_eq!(JPH_SoftBodyCreationSettings_GetPressure(ptr), ours.pressure);
        assert_eq!(
            JPH_SoftBodyCreationSettings_GetGravityFactor(ptr),
            ours.gravity_factor
        );
        assert_eq!(
            JPH_SoftBodyCreationSettings_GetVertexRadius(ptr),
            ours.vertex_radius
        );
        assert_eq!(
            JPH_SoftBodyCreationSettings_GetUpdatePosition(ptr),
            ours.update_position
        );
        assert_eq!(
            JPH_SoftBodyCreationSettings_GetMakeRotationIdentity(ptr),
            ours.make_rotation_identity
        );
        assert_eq!(
            JPH_SoftBodyCreationSettings_GetAllowSleeping(ptr),
            ours.allow_sleeping
        );
        assert_eq!(
            JPH_SoftBodyCreationSettings_GetFacesDoubleSided(ptr),
            ours.faces_double_sided
        );
        // The one documented difference: Jolt's default layer is 0.
        assert_eq!(JPH_SoftBodyCreationSettings_GetObjectLayer(ptr), 0);
    }
    assert_eq!(ours.object_layer, ObjectLayer::MOVING);
    assert_eq!(ours.max_linear_velocity, limits::MAX_LINEAR_VELOCITY);
}

#[test]
fn default_vertex_attributes_match_jolt() {
    let mut jolt = SoftBodyVertexAttributes::default()
        .compliance(5.0)
        .long_range_attachment(LongRangeAttachment::GeodesicDistance, 3.0)
        .to_jph();
    // SAFETY: `jolt` is a live local that the call overwrites.
    unsafe { JPH_SoftBodyVertexAttributes_Init(&mut jolt) };
    let ours = SoftBodyVertexAttributes::default().to_jph();
    assert_eq!(jolt.compliance, ours.compliance);
    assert_eq!(jolt.shearCompliance, ours.shearCompliance);
    assert_eq!(jolt.bendCompliance, ours.bendCompliance);
    assert_eq!(jolt.lraType, ours.lraType);
    assert_eq!(jolt.lraMaxDistanceMultiplier, ours.lraMaxDistanceMultiplier);
    assert_eq!(SoftBodyVertex::new(Vec3::ZERO).inverse_mass, 1.0);
}

#[test]
fn a_triangle_without_constraints_builds() {
    let settings = SoftBodySharedSettings::builder(triangle(), vec![[0, 2, 1]])
        .build()
        .unwrap();
    assert_eq!(settings.vertex_count(), 3);
    assert_eq!(settings.face_count(), 1);
    assert_eq!(settings.edge_constraint_count(), 0);
}

#[test]
fn invalid_vertices_are_rejected() {
    let build = |vertices| rejected(SoftBodySharedSettings::builder(vertices, Vec::new()));
    assert_eq!(build(Vec::new()), "a soft body needs at least one vertex");
    let mut far = triangle();
    far[1].position.x = limits::MAX_SHAPE_EXTENT.next_up();
    assert!(build(far).contains("vertex position"));
    let mut fast = triangle();
    fast[1].velocity.y = limits::MAX_LINEAR_VELOCITY.next_up();
    assert_eq!(build(fast), LINEAR_VELOCITY_RULE);
    for inverse_mass in [-1.0, f32::NAN, limits::MAX_VERTEX_INVERSE_MASS.next_up()] {
        let mut heavy = triangle();
        heavy[2].inverse_mass = inverse_mass;
        assert_eq!(build(heavy), VERTEX_INVERSE_MASS_RULE);
    }
}

#[test]
fn total_movable_mass_is_bounded() {
    let at_bound = |w| {
        vec![
            SoftBodyVertex {
                inverse_mass: w,
                ..SoftBodyVertex::new(Vec3::ZERO)
            },
            SoftBodyVertex {
                inverse_mass: w,
                ..SoftBodyVertex::new(Vec3::new(1.0, 0.0, 0.0))
            },
        ]
    };
    let heavy = 1.0 / limits::MAX_MASS;
    assert_eq!(
        rejected(SoftBodySharedSettings::builder(at_bound(heavy), Vec::new())),
        TOTAL_MASS_RULE
    );
    // Two vertices of 400 t each, below the bound together.
    let lighter = 2.5 / limits::MAX_MASS;
    assert!(
        SoftBodySharedSettings::builder(at_bound(lighter), Vec::new())
            .build()
            .is_ok()
    );
    let pinned = vec![
        SoftBodyVertex::kinematic(Vec3::ZERO),
        SoftBodyVertex::kinematic(Vec3::new(1.0, 0.0, 0.0)),
    ];
    assert!(SoftBodySharedSettings::builder(pinned, Vec::new())
        .build()
        .is_ok());
}

#[test]
fn invalid_faces_are_rejected() {
    let build = |vertices, face| rejected(SoftBodySharedSettings::builder(vertices, vec![face]));
    assert_eq!(
        build(triangle(), [0, 1, 3]),
        "a face index must name a vertex"
    );
    assert_eq!(
        build(triangle(), [0, 1, 1]),
        "a face must name three different vertices"
    );
    let mut short = triangle();
    short[1].position.x = limits::MIN_SOFT_BODY_EDGE_LENGTH.next_down();
    assert_eq!(build(short, [0, 2, 1]), EDGE_LENGTH_RULE);
    // Three vertices on a line: every edge is long enough, the face has no area.
    let mut collinear = triangle();
    collinear[2].position = Vec3::new(2.0, 0.0, 0.0);
    assert_eq!(build(collinear, [0, 2, 1]), "a face must have an area");
}

#[test]
fn edge_lengths_are_measured_in_f32_like_jolt() {
    // The positions differ, but by less than f32 resolves at 1000 m: the difference is 0.
    let a = Vec3::new(1000.0, 0.0, 0.0);
    let b = Vec3::new(1000.0 + 1.0e-5, 0.0, 0.0);
    assert!(!is_edge_length(a, b));
    // A subnormal difference squares to 0 in f32.
    let tiny = Vec3::new(f32::MIN_POSITIVE / 4.0, 0.0, 0.0);
    assert!(!is_edge_length(Vec3::ZERO, tiny));
    let bound = Vec3::new(limits::MIN_SOFT_BODY_EDGE_LENGTH, 0.0, 0.0);
    assert!(is_edge_length(Vec3::ZERO, bound));
}

#[test]
fn distance_bends_need_separate_opposite_vertices() {
    // Two triangles folded onto each other: the vertices opposite the shared edge 0-1
    // coincide.
    let mut vertices = triangle();
    vertices.push(SoftBodyVertex::new(Vec3::new(0.0, 0.0, 1.0)));
    let faces = vec![[0, 2, 1], [0, 1, 3]];
    let folded = SoftBodySharedSettings::builder(vertices.clone(), faces.clone())
        .create_constraints(SoftBodyBendType::Distance, Default::default());
    assert!(rejected(folded).contains("opposite a shared edge"));
    let dihedral = SoftBodySharedSettings::builder(vertices, faces)
        .create_constraints(SoftBodyBendType::Dihedral, Default::default());
    assert!(dihedral.build().is_ok());
}

#[test]
fn attributes_are_validated() {
    let build = |attributes: SoftBodyVertexAttributes| {
        SoftBodySharedSettings::builder(triangle(), vec![[0, 2, 1]])
            .create_constraints(SoftBodyBendType::Dihedral, attributes)
            .build()
    };
    let default = SoftBodyVertexAttributes::default();
    for compliance in [-1.0, f32::NAN, limits::MAX_COMPLIANCE.next_up()] {
        for attributes in [
            default.compliance(compliance),
            default.shear_compliance(compliance),
            default.bend_compliance(Some(compliance)),
        ] {
            assert_eq!(
                build(attributes).err(),
                Some(SoftBodyError::InvalidValue(COMPLIANCE_RULE))
            );
        }
    }
    let lra = LongRangeAttachment::EuclideanDistance;
    for multiplier in [1.0_f32.next_down(), limits::MAX_RATIO.next_up(), f32::NAN] {
        assert!(build(default.long_range_attachment(lra, multiplier)).is_err());
    }
    assert!(build(default.long_range_attachment(lra, limits::MAX_RATIO)).is_ok());
    assert!(build(default.bend_compliance(Some(limits::MAX_COMPLIANCE))).is_ok());
    let per_vertex = SoftBodySharedSettings::builder(triangle(), vec![[0, 2, 1]])
        .create_constraints_per_vertex(SoftBodyBendType::None, vec![default; 2]);
    assert_eq!(
        rejected(per_vertex),
        "per-vertex attributes need one set per vertex"
    );
}
