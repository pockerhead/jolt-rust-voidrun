use std::error::Error as _;

use super::*;

const WORLD: WorldError = WorldError::InitFailed;
const SHAPE: ShapeError = ShapeError::ConvexHull(ConvexHullError::Coplanar);
const BODY: BodyError = BodyError::TooManyBodies;
const STEP: StepError = StepError::InvalidDeltaTime;
const QUERY: QueryError = QueryError::InvalidValue("ray direction");
const CONTACT_SETTINGS: ContactSettingsError = ContactSettingsError::Friction;
const CHARACTER: CharacterError = CharacterError::TooManyCharacters;
const VEHICLE: VehicleError = VehicleError::TooManyVehicles;
const RAGDOLL: RagdollError = RagdollError::TooManyRagdolls;
const CONSTRAINT: ConstraintError = ConstraintError::TooManyConstraints;
const SOFT_BODY: SoftBodyError = SoftBodyError::InitFailed;
const STATE: StateError = StateError::WrongWorld;
const COLLISION_GROUP: CollisionGroupError = CollisionGroupError::SameSubGroup(3);

/// One error of every area, converted, with the wrapped error's `Display` and `source`.
fn every_area() -> [(Error, String, bool); 13] {
    fn entry<E: std::error::Error + Into<Error>>(error: E) -> (Error, String, bool) {
        let (text, has_source) = (error.to_string(), error.source().is_some());
        (error.into(), text, has_source)
    }
    [
        entry(WORLD),
        entry(SHAPE),
        entry(BODY),
        entry(STEP),
        entry(QUERY),
        entry(CONTACT_SETTINGS),
        entry(CHARACTER),
        entry(VEHICLE),
        entry(RAGDOLL),
        entry(CONSTRAINT),
        entry(SOFT_BODY),
        entry(STATE),
        entry(COLLISION_GROUP),
    ]
}

#[test]
fn every_area_error_converts_into_its_variant() {
    assert_eq!(Error::from(WORLD), Error::World(WORLD));
    assert_eq!(Error::from(SHAPE), Error::Shape(SHAPE));
    assert_eq!(Error::from(BODY), Error::Body(BODY));
    assert_eq!(Error::from(STEP), Error::Step(STEP));
    assert_eq!(Error::from(QUERY), Error::Query(QUERY));
    assert_eq!(
        Error::from(CONTACT_SETTINGS),
        Error::ContactSettings(CONTACT_SETTINGS)
    );
    assert_eq!(Error::from(CHARACTER), Error::Character(CHARACTER));
    assert_eq!(Error::from(VEHICLE), Error::Vehicle(VEHICLE));
    assert_eq!(Error::from(RAGDOLL), Error::Ragdoll(RAGDOLL));
    assert_eq!(Error::from(CONSTRAINT), Error::Constraint(CONSTRAINT));
    assert_eq!(Error::from(SOFT_BODY), Error::SoftBody(SOFT_BODY));
    assert_eq!(Error::from(STATE), Error::State(STATE));
    assert_eq!(
        Error::from(COLLISION_GROUP),
        Error::CollisionGroup(COLLISION_GROUP)
    );
}

#[test]
fn display_is_the_wrapped_errors() {
    for (error, wrapped, _) in every_area() {
        assert_eq!(error.to_string(), wrapped, "{error:?}");
    }
}

#[test]
fn source_is_the_wrapped_errors_source() {
    for (error, _, wrapped_has_source) in every_area() {
        assert_eq!(error.source().is_some(), wrapped_has_source, "{error:?}");
    }
    assert!(Error::from(BodyError::TooManyBodies).source().is_none());
    let body = BodyError::InvalidValue("mass");
    let source = Error::from(VehicleError::Body(body))
        .source()
        .map(|s| s.to_string());
    assert_eq!(source, Some(body.to_string()));
}

/// Every area error that wraps another one returns it from `source`.
#[test]
fn sources_are_reported() {
    fn source_of<E: std::error::Error, S: std::error::Error + Copy + 'static>(
        error: E,
    ) -> Option<S> {
        error.source()?.downcast_ref::<S>().copied()
    }
    let body = BodyError::InvalidValue("mass");
    assert_eq!(source_of(VehicleError::Body(body)), Some(body));
    assert_eq!(source_of(RagdollError::Body(body)), Some(body));
    assert_eq!(source_of(ConstraintError::Body(body)), Some(body));
    assert_eq!(source_of(StateError::Body(body)), Some(body));
    let query = QueryError::AllocationFailed;
    assert_eq!(source_of(CharacterError::Query(query)), Some(query));
    let hull = ConvexHullError::Coplanar;
    assert_eq!(source_of(ShapeError::ConvexHull(hull)), Some(hull));
    let mesh = MeshError::NoTriangles;
    assert_eq!(source_of(ShapeError::Mesh(mesh)), Some(mesh));
    let thin = ThinTrianglesError {
        scale: Vec3::new(1.0, 0.1, 1.0),
        max_convex_extent: 2.0,
    };
    assert_eq!(source_of(ShapeError::ThinTriangles(thin)), Some(thin));
    let binary = BinaryStateError::Corrupt;
    assert_eq!(source_of(ShapeError::BinaryState(binary)), Some(binary));
    assert_eq!(
        source_of::<_, BodyError>(ShapeError::AllocationFailed),
        None
    );
}

#[test]
fn error_is_send_sync_static_and_copy() {
    fn assert_bounds<T: std::error::Error + Send + Sync + Copy + 'static>() {}
    assert_bounds::<Error>();
    let boxed = Box::<dyn std::error::Error + Send + Sync>::from(Error::from(STEP));
    assert_eq!(boxed.to_string(), STEP.to_string());
}

#[test]
fn errors_stay_small() {
    // clippy's `result_large_err` fires at 128 bytes; the margin keeps `Result<_, Error>` cheap.
    assert!(size_of::<ShapeError>() <= 88, "{}", size_of::<ShapeError>());
    assert!(size_of::<Error>() <= 96, "{}", size_of::<Error>());
}

#[test]
fn shape_error_variants_display_their_payload() {
    assert_eq!(
        ShapeError::ConvexHull(ConvexHullError::TooFewPoints).to_string(),
        "invalid convex hull: a convex hull needs at least 4 points"
    );
    assert_eq!(
        ShapeError::ConvexHull(ConvexHullError::Degenerate).to_string(),
        "invalid convex hull: the points lie on or close to a line"
    );
    assert_eq!(
        ShapeError::ConvexHull(ConvexHullError::Coplanar).to_string(),
        "invalid convex hull: the points lie on or close to a plane; thicken the cloud or centre \
         it on the shape origin"
    );
    assert_eq!(
        ShapeError::Mesh(MeshError::NoTriangles).to_string(),
        "invalid triangle mesh: no triangle is left after dropping small, thin and degenerate ones"
    );
    let message = JoltMessage::from_c_buffer(b"Too few points\0garbage");
    assert_eq!(
        ShapeError::Rejected(message).to_string(),
        "Jolt rejected the shape settings: Too few points"
    );
    assert_eq!(format!("{message:?}"), "\"Too few points\"");
    assert_eq!(
        ShapeError::NoSubShape { index: 4, count: 3 }.to_string(),
        "compound has no sub-shape 4 (it has 3)"
    );
    assert_eq!(
        ShapeError::TooManySubShapes {
            expanded: 2_000_000
        }
        .to_string(),
        "compound expands to 2000000 shapes, above limits::MAX_EXPANDED_SUB_SHAPES"
    );
}

#[test]
fn thin_triangles_name_the_scale_and_the_extent() {
    let error = ThinTrianglesError {
        scale: Vec3::new(0.2, 1.0, 0.5),
        max_convex_extent: 1100.0,
    };
    assert_eq!(
        ShapeError::ThinTriangles(error).to_string(),
        "invalid scale: mesh or heightfield triangles scaled by (0.2, 1, 0.5) are too thin for \
         convex shapes up to 1100 m"
    );
    assert_eq!(error, error);
    let other = ThinTrianglesError {
        max_convex_extent: 1.0,
        ..error
    };
    assert_ne!(error, other);
}

#[test]
fn jolt_message_reads_up_to_the_nul() {
    assert_eq!(JoltMessage::from_c_buffer(b"\0rest").as_str(), "");
    assert_eq!(JoltMessage::from_c_buffer(b"").as_str(), "");
    let exact = [b'a'; JoltMessage::CAPACITY];
    let mut buffer = exact.to_vec();
    buffer.push(0);
    assert_eq!(
        JoltMessage::from_c_buffer(&buffer).as_str().as_bytes(),
        exact
    );
}

#[test]
fn long_jolt_messages_are_cut_after_a_whole_word() {
    let long = b"Hull building failed, point 1010 had an error of 0.0934095 (relative to tolerance: 0.001)\0";
    assert_eq!(
        JoltMessage::from_c_buffer(long).as_str(),
        "Hull building failed, point 1010 had an error of 0.0934095 (relative to..."
    );
    // Without a space the cut falls inside the word.
    let buffer = [b'b'; JoltMessage::CAPACITY + 1];
    let message = JoltMessage::from_c_buffer(&buffer);
    assert_eq!(message.as_str().len(), JoltMessage::CAPACITY);
    assert!(message.as_str().ends_with("bb..."));
}

#[test]
fn jolt_message_keeps_whole_characters() {
    // 75 ASCII bytes, a two-byte character and more: the cut 76 bytes in, before `...`,
    // splits the character, which is dropped.
    let mut buffer = vec![b'c'; JoltMessage::CAPACITY - 4];
    buffer.extend_from_slice("\u{e9}tail".as_bytes());
    buffer.push(0);
    let message = JoltMessage::from_c_buffer(&buffer);
    assert_eq!(
        message.as_str(),
        format!("{}...", "c".repeat(JoltMessage::CAPACITY - 4))
    );
    // An invalid byte inside the text keeps the valid part before it.
    assert_eq!(JoltMessage::from_c_buffer(b"ok\xffno\0").as_str(), "ok");
}

#[test]
fn mapper_error_variants_display_their_joint() {
    assert_eq!(
        RagdollError::UnmappedJoint(3).to_string(),
        "ragdoll joint 3 has no animation joint of its name"
    );
    assert_eq!(
        RagdollError::HierarchyMismatch(4).to_string(),
        "the animation joint of ragdoll joint 4 is not below its parent's"
    );
    assert_eq!(
        RagdollError::DegenerateChain(5).to_string(),
        "the mapped chain ending at ragdoll joint 5 is too short to turn"
    );
}
