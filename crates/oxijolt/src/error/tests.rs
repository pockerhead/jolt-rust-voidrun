use std::error::Error as _;

use super::*;

const WORLD: WorldError = WorldError::InitFailed;
const SHAPE: ShapeError = ShapeError::Rejected;
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

/// One error of every area, converted, with the wrapped error's `Display` and `source`.
fn every_area() -> [(Error, String, bool); 12] {
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
        assert!(!wrapped_has_source, "{error:?}");
        assert!(error.source().is_none(), "{error:?}");
    }
    let body = BodyError::InvalidValue("mass");
    for error in [
        Error::from(VehicleError::Body(body)),
        Error::from(RagdollError::Body(body)),
        Error::from(ConstraintError::Body(body)),
    ] {
        let source = error.source().expect("the wrapped error has a source");
        assert_eq!(source.downcast_ref::<BodyError>(), Some(&body), "{error:?}");
    }
}

#[test]
fn error_is_send_sync_static_and_copy() {
    fn assert_bounds<T: std::error::Error + Send + Sync + Copy + 'static>() {}
    assert_bounds::<Error>();
    let boxed = Box::<dyn std::error::Error + Send + Sync>::from(Error::from(STEP));
    assert_eq!(boxed.to_string(), STEP.to_string());
}
