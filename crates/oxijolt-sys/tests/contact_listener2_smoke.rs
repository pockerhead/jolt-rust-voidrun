//! Smoke tests for the extension's contact listener (`JPH_ContactListener2_*`): its validate
//! callback gets a collide result without faces, every validate result reaches Jolt, and added,
//! persisted and removed contacts are forwarded as joltc forwards them.
//!
//! The proc table is process-global, so this binary installs it once and every test keeps its
//! state in the listener's `userData`. Every `TestWorld` holds the framework's world lock for its
//! whole life, so the tests of this binary run one at a time, which lets
//! [`a_null_validate_proc_accepts`] swap the table while its world exists.

mod framework;

use std::ffi::c_void;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::ptr::null_mut;
use std::sync::{Mutex, OnceLock, PoisonError};

use framework::*;
use oxijolt_sys::*;

const DT: f32 = 1.0 / 60.0;
const INVALID_BODY: JPH_BodyID = u32::MAX;
/// The sub-shape id of child 0 of a two-child compound: one bit, 0, pushed onto the empty id
/// (all bits set). Child 1's id has every bit set.
const FIRST_OF_TWO_CHILDREN: JPH_SubShapeID = u32::MAX - 1;

/// What the validate callback answers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ValidatePolicy {
    AcceptAll,
    AcceptEach,
    RejectAll,
    /// Rejects the hits of body 1's first compound child, accepts the others.
    RejectFirstChild,
}

/// What the added and persisted callbacks write to the contact settings.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum SettingsPolicy {
    Keep,
    Conveyor,
    Spin,
    Frictionless,
    Bouncy,
    ImmovableBody1,
    ImmovableBody2,
    Sensor,
}

/// A contact pair: body and sub-shape ids of both sides.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Pair {
    body1: JPH_BodyID,
    sub_shape1: JPH_SubShapeID,
    body2: JPH_BodyID,
    sub_shape2: JPH_SubShapeID,
}

/// What one validate call received and answered.
#[derive(Clone, Copy, Debug)]
struct Validated {
    body1: JPH_BodyID,
    body2: JPH_BodyID,
    sub_shape1: JPH_SubShapeID,
    base_offset: JPH_RVec3,
    point_on1: JPH_Vec3,
    /// Both face counts 0 and both face pointers null.
    no_faces: bool,
    result: JPH_ValidateResult,
}

#[derive(Clone, Copy, Debug)]
enum Event {
    Validated(Validated),
    /// A contact and the settings Jolt passed in, before the policy wrote them.
    Added(Pair, JPH_ContactSettings),
    Persisted(Pair, JPH_ContactSettings),
    Removed(Pair),
}

/// Everything one test's listener records, reached through `userData`.
struct Recorder {
    events: Mutex<Vec<Event>>,
    validate: ValidatePolicy,
    settings: SettingsPolicy,
}

impl Recorder {
    fn push(&self, event: Event) {
        self.events
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .push(event);
    }

    fn take(&self) -> Vec<Event> {
        std::mem::take(&mut *self.events.lock().unwrap_or_else(PoisonError::into_inner))
    }
}

/// Runs `f` with the recorder of `user_data`; aborts instead of unwinding into C++.
///
/// # Safety
/// `user_data` is the `userData` of a listener of this file: a `Recorder` that outlives the call.
unsafe fn with_recorder<R>(user_data: *mut c_void, f: impl FnOnce(&Recorder) -> R) -> R {
    // SAFETY: the caller guarantees a live recorder; it is only read through `&`.
    let recorder = unsafe { &*user_data.cast::<Recorder>() };
    match catch_unwind(AssertUnwindSafe(|| f(recorder))) {
        Ok(result) => result,
        Err(_) => std::process::abort(),
    }
}

/// The extension's `OnContactValidate`.
///
/// # Safety
/// `user_data` is a live `Recorder` (see [`with_recorder`]); the bodies, `base_offset` and
/// `result` are the non-null ones the extension passes for the duration of the callback.
unsafe extern "C" fn on_contact_validate(
    user_data: *mut c_void,
    body1: *const JPH_Body,
    body2: *const JPH_Body,
    base_offset: *const JPH_RVec3,
    result: *const JPH_CollideShapeResult,
) -> JPH_ValidateResult {
    // SAFETY: the extension passes the listener's `userData` and live arguments; Jolt holds the
    // bodies, and the getters only read them.
    unsafe {
        let (body1, body2, base, hit) = (
            JPH_Body_GetID(body1),
            JPH_Body_GetID(body2),
            *base_offset,
            *result,
        );
        with_recorder(user_data, |r| {
            let answer = match r.validate {
                ValidatePolicy::AcceptAll => JPH_ValidateResult_AcceptAllContactsForThisBodyPair,
                ValidatePolicy::AcceptEach => JPH_ValidateResult_AcceptContact,
                ValidatePolicy::RejectAll => JPH_ValidateResult_RejectAllContactsForThisBodyPair,
                ValidatePolicy::RejectFirstChild if hit.subShapeID1 == FIRST_OF_TWO_CHILDREN => {
                    JPH_ValidateResult_RejectContact
                }
                ValidatePolicy::RejectFirstChild => JPH_ValidateResult_AcceptContact,
            };
            r.push(Event::Validated(Validated {
                body1,
                body2,
                sub_shape1: hit.subShapeID1,
                base_offset: base,
                point_on1: hit.contactPointOn1,
                no_faces: hit.shape1FaceCount == 0
                    && hit.shape2FaceCount == 0
                    && hit.shape1Faces.is_null()
                    && hit.shape2Faces.is_null(),
                result: answer,
            }));
            answer
        })
    }
}

/// The pair Jolt reports with an added or persisted contact.
///
/// # Safety
/// The bodies and manifold are the ones Jolt passes to a contact callback.
unsafe fn manifold_pair(
    body1: *const JPH_Body,
    body2: *const JPH_Body,
    manifold: *const JPH_ContactManifold,
) -> Pair {
    // SAFETY: the callback arguments are live for the callback (contract); the getters read.
    unsafe {
        Pair {
            body1: JPH_Body_GetID(body1),
            sub_shape1: JPH_ContactManifold_GetSubShapeID1(manifold),
            body2: JPH_Body_GetID(body2),
            sub_shape2: JPH_ContactManifold_GetSubShapeID2(manifold),
        }
    }
}

/// Writes the recorder's settings policy to `settings`.
fn apply(policy: SettingsPolicy, settings: &mut JPH_ContactSettings) {
    match policy {
        SettingsPolicy::Keep => {}
        SettingsPolicy::Conveyor => settings.relativeLinearSurfaceVelocity = vec3(2.0, 0.0, 0.0),
        SettingsPolicy::Spin => settings.relativeAngularSurfaceVelocity = vec3(0.0, 3.0, 0.0),
        SettingsPolicy::Frictionless => settings.combinedFriction = 0.0,
        SettingsPolicy::Bouncy => settings.combinedRestitution = 1.0,
        SettingsPolicy::ImmovableBody1 => {
            settings.invMassScale1 = 0.0;
            settings.invInertiaScale1 = 0.0;
        }
        SettingsPolicy::ImmovableBody2 => {
            settings.invMassScale2 = 0.0;
            settings.invInertiaScale2 = 0.0;
        }
        SettingsPolicy::Sensor => settings.isSensor = 1,
    }
}

/// The extension's `OnContactAdded` and `OnContactPersisted`.
///
/// # Safety
/// `user_data` is a live `Recorder` (see [`with_recorder`]); the bodies, the manifold and
/// `settings` are the non-null ones the extension passes for the duration of the callback, and
/// `settings` may be written.
unsafe fn record_manifold(
    user_data: *mut c_void,
    body1: *const JPH_Body,
    body2: *const JPH_Body,
    manifold: *const JPH_ContactManifold,
    settings: *mut JPH_ContactSettings,
    event: fn(Pair, JPH_ContactSettings) -> Event,
) {
    // SAFETY: as the caller guarantees.
    unsafe {
        let pair = manifold_pair(body1, body2, manifold);
        let incoming = *settings;
        let policy = with_recorder(user_data, |r| {
            r.push(event(pair, incoming));
            r.settings
        });
        apply(policy, &mut *settings);
    }
}

/// # Safety
/// As for [`record_manifold`].
unsafe extern "C" fn on_contact_added(
    user_data: *mut c_void,
    body1: *const JPH_Body,
    body2: *const JPH_Body,
    manifold: *const JPH_ContactManifold,
    settings: *mut JPH_ContactSettings,
) {
    // SAFETY: the extension passes live callback arguments (contract).
    unsafe { record_manifold(user_data, body1, body2, manifold, settings, Event::Added) };
}

/// # Safety
/// As for [`record_manifold`].
unsafe extern "C" fn on_contact_persisted(
    user_data: *mut c_void,
    body1: *const JPH_Body,
    body2: *const JPH_Body,
    manifold: *const JPH_ContactManifold,
    settings: *mut JPH_ContactSettings,
) {
    // SAFETY: the extension passes live callback arguments (contract).
    unsafe {
        record_manifold(
            user_data,
            body1,
            body2,
            manifold,
            settings,
            Event::Persisted,
        )
    };
}

/// The extension's `OnContactRemoved`.
///
/// # Safety
/// `user_data` is a live `Recorder`; `pair` is the extension's non-null cast of the Jolt pair,
/// live for the duration of the callback.
unsafe extern "C" fn on_contact_removed(user_data: *mut c_void, pair: *const JPH_SubShapeIDPair) {
    // SAFETY: the extension passes the listener's `userData` and the live Jolt pair, which the
    // getters read through Jolt's accessors.
    unsafe {
        let pair = Pair {
            body1: JPH_SubShapeIDPair_GetBody1ID(pair),
            sub_shape1: JPH_SubShapeIDPair_GetSubShapeID1(pair),
            body2: JPH_SubShapeIDPair_GetBody2ID(pair),
            sub_shape2: JPH_SubShapeIDPair_GetSubShapeID2(pair),
        };
        with_recorder(user_data, |r| r.push(Event::Removed(pair)));
    }
}

static PROCS: JPH_ContactListener_Procs = JPH_ContactListener_Procs {
    OnContactValidate: Some(on_contact_validate),
    OnContactAdded: Some(on_contact_added),
    OnContactPersisted: Some(on_contact_persisted),
    OnContactRemoved: Some(on_contact_removed),
};

static PROCS_WITHOUT_VALIDATE: JPH_ContactListener_Procs = JPH_ContactListener_Procs {
    OnContactValidate: None,
    ..PROCS
};

fn install_procs() {
    static INSTALLED: OnceLock<()> = OnceLock::new();
    INSTALLED.get_or_init(|| {
        // SAFETY: the table is an immutable static; `OnceLock` orders this one write before
        // every listener this binary creates.
        unsafe { JPH_ContactListener2_SetProcs(&PROCS) };
    });
}

/// One world's listener, recording into its recorder. Detached and destroyed on drop, before
/// the recorder (field order).
struct Listener {
    system: *mut JPH_PhysicsSystem,
    listener: *mut JPH_ContactListener2,
    recorder: Box<Recorder>,
}

impl Listener {
    fn attach(world: &TestWorld, validate: ValidatePolicy, settings: SettingsPolicy) -> Self {
        install_procs();
        let recorder = Box::new(Recorder {
            events: Mutex::new(Vec::new()),
            validate,
            settings,
        });
        let user_data: *mut c_void = (&*recorder as *const Recorder).cast_mut().cast();
        // SAFETY: the system is live; the listener keeps `user_data`, which the boxed recorder
        // keeps valid until `drop` has detached and destroyed the listener.
        unsafe {
            let listener = JPH_ContactListener2_Create(user_data);
            assert!(!listener.is_null());
            JPH_PhysicsSystem_SetContactListener2(world.system(), listener);
            Self {
                system: world.system(),
                listener,
                recorder,
            }
        }
    }

    fn take(&self) -> Vec<Event> {
        self.recorder.take()
    }
}

impl Drop for Listener {
    fn drop(&mut self) {
        // SAFETY: the system outlives the listener (each test drops it first); after the setter
        // nothing calls it, so it is destroyed once.
        unsafe {
            JPH_PhysicsSystem_SetContactListener2(self.system, null_mut());
            JPH_ContactListener2_Destroy(self.listener);
        }
    }
}

/// A static 20 x 1 x 20 box with its top face at y = `top`, centred on `(x, z)`.
fn add_floor(world: &TestWorld, x: Real, top: Real, z: Real) -> JPH_BodyID {
    create_box(
        world.body_interface(),
        vec3(10.0, 0.5, 10.0),
        rvec3(x, top - 0.5, z),
        JPH_MotionType_Static,
        OL_NON_MOVING,
        JPH_Activation_DontActivate,
    )
}

fn add_cube(world: &TestWorld, position: JPH_RVec3) -> JPH_BodyID {
    create_box(
        world.body_interface(),
        vec3(0.25, 0.25, 0.25),
        position,
        JPH_MotionType_Dynamic,
        OL_MOVING,
        JPH_Activation_Activate,
    )
}

/// A dynamic compound of two 0.5 m cubes centred at x = -0.5 and x = 0.5 around `position`.
fn add_two_cube_compound(world: &TestWorld, position: JPH_RVec3) -> JPH_BodyID {
    let half = vec3(0.25, 0.25, 0.25);
    let rotation = quat_identity();
    // SAFETY: Jolt is initialised (the world exists); every pointer comes from the `_Create`
    // call before its use and every reference this function holds is released after the
    // object that keeps its own took it.
    let body = unsafe {
        let child = JPH_BoxShape_Create(&half, 0.05);
        let settings = JPH_StaticCompoundShapeSettings_Create();
        for x in [-0.5, 0.5] {
            let offset = vec3(x, 0.0, 0.0);
            JPH_CompoundShapeSettings_AddShape2(
                settings.cast(),
                &offset,
                &rotation,
                child.cast(),
                0,
            );
        }
        let shape = JPH_StaticCompoundShape_Create(settings);
        let creation = JPH_BodyCreationSettings_Create3(
            shape.cast(),
            &position,
            &rotation,
            JPH_MotionType_Dynamic,
            OL_MOVING,
        );
        let body = JPH_BodyInterface_CreateAndAddBody(
            world.body_interface(),
            creation,
            JPH_Activation_Activate,
        );
        JPH_BodyCreationSettings_Destroy(creation);
        JPH_Shape_Destroy(shape.cast());
        JPH_ShapeSettings_Destroy(settings.cast());
        JPH_Shape_Destroy(child.cast());
        body
    };
    assert_ne!(body, INVALID_BODY);
    body
}

fn position(world: &TestWorld, id: JPH_BodyID) -> JPH_RVec3 {
    let mut position = rvec3(0.0, 0.0, 0.0);
    // SAFETY: the body exists; `position` is a live local.
    unsafe { JPH_BodyInterface_GetCenterOfMassPosition(world.body_interface(), id, &mut position) };
    position
}

fn linear_velocity(world: &TestWorld, id: JPH_BodyID) -> JPH_Vec3 {
    let mut velocity = vec3(0.0, 0.0, 0.0);
    // SAFETY: the body exists; `velocity` is a live local.
    unsafe { JPH_BodyInterface_GetLinearVelocity(world.body_interface(), id, &mut velocity) };
    velocity
}

fn angular_velocity(world: &TestWorld, id: JPH_BodyID) -> JPH_Vec3 {
    let mut velocity = vec3(0.0, 0.0, 0.0);
    // SAFETY: the body exists; `velocity` is a live local.
    unsafe { JPH_BodyInterface_GetAngularVelocity(world.body_interface(), id, &mut velocity) };
    velocity
}

fn set_linear_velocity(world: &TestWorld, id: JPH_BodyID, velocity: JPH_Vec3) {
    // SAFETY: the body exists; `velocity` is a live local that joltc only reads.
    unsafe { JPH_BodyInterface_SetLinearVelocity(world.body_interface(), id, &velocity) };
}

fn validated(events: &[Event]) -> Vec<Validated> {
    events
        .iter()
        .filter_map(|event| match *event {
            Event::Validated(v) => Some(v),
            _ => None,
        })
        .collect()
}

fn manifolds(events: &[Event]) -> Vec<(Pair, JPH_ContactSettings)> {
    events
        .iter()
        .filter_map(|event| match *event {
            Event::Added(pair, settings) | Event::Persisted(pair, settings) => {
                Some((pair, settings))
            }
            _ => None,
        })
        .collect()
}

/// Steps `steps` times and returns the events of each step.
fn run(world: &TestWorld, listener: &Listener, steps: usize) -> Vec<Vec<Event>> {
    (0..steps)
        .map(|_| {
            world.step(DT);
            listener.take()
        })
        .collect()
}

#[test]
fn validate_gets_no_faces_and_a_base_offset() {
    let world = TestWorld::new(2);
    let listener = Listener::attach(&world, ValidatePolicy::AcceptAll, SettingsPolicy::Keep);
    let floor = add_floor(&world, 100.0, 0.0, 200.0);
    let cube = add_cube(&world, rvec3(100.0, 0.4, 200.0));
    let events: Vec<Event> = run(&world, &listener, 30).concat();
    let calls = validated(&events);
    assert!(!calls.is_empty(), "the cube lands on the floor");
    for call in &calls {
        assert!(call.no_faces, "{call:?}");
        assert_eq!(
            [call.body1, call.body2],
            [cube, floor],
            "the dynamic body is body 1"
        );
        // The base offset is body 1's centre of mass; the point on body 1 lies on the cube's
        // bottom face (half extent 0.25), relative to it.
        assert!((call.base_offset.x - 100.0).abs() < 0.01, "{call:?}");
        assert!((call.base_offset.z - 200.0).abs() < 0.01, "{call:?}");
        assert!((call.point_on1.y + 0.25).abs() < 0.02, "{call:?}");
        assert!(call.point_on1.x.abs() <= 0.26 && call.point_on1.z.abs() <= 0.26);
    }
    drop(listener);
}

#[test]
fn each_validate_result_is_obeyed() {
    let per_step = |policy| {
        let world = TestWorld::new(1);
        let listener = Listener::attach(&world, policy, SettingsPolicy::Keep);
        add_floor(&world, 0.0, 0.0, 0.0);
        let body = add_two_cube_compound(&world, rvec3(0.0, 0.3, 0.0));
        let steps = run(&world, &listener, 90);
        (steps, body, position(&world, body))
    };

    // Accept all: at most one call per collision pass of the pair.
    let (steps, _, rest) = per_step(ValidatePolicy::AcceptAll);
    let counts: Vec<usize> = steps.iter().map(|s| validated(s).len()).collect();
    assert!(counts.iter().all(|&n| n <= 1), "{counts:?}");
    assert!(counts.contains(&1));
    assert!(rest.y > 0.2, "the compound rests on the floor: {}", rest.y);

    // Accept each: Jolt asks again for the other child's hit in the same pass.
    let (steps, _, _) = per_step(ValidatePolicy::AcceptEach);
    assert!(
        steps.iter().any(|s| {
            let calls = validated(s);
            calls.len() == 2 && calls[0].sub_shape1 != calls[1].sub_shape1
        }),
        "both children are validated in one pass"
    );

    // Reject one child's hits: only the other child's contacts reach the manifolds.
    let (steps, compound, _) = per_step(ValidatePolicy::RejectFirstChild);
    let events = steps.concat();
    assert!(validated(&events)
        .iter()
        .any(|v| v.result == JPH_ValidateResult_RejectContact));
    let held = manifolds(&events);
    assert!(!held.is_empty(), "the right child holds");
    // Added and persisted contacts order their bodies by id, validation by motion type.
    for (pair, _) in &held {
        let child = if pair.body1 == compound {
            pair.sub_shape1
        } else {
            pair.sub_shape2
        };
        assert_ne!(child, FIRST_OF_TWO_CHILDREN, "{pair:?}");
    }

    // Reject all: the compound falls through the floor.
    let (_, _, fallen) = per_step(ValidatePolicy::RejectAll);
    assert!(fallen.y < -2.0, "fell through: {}", fallen.y);
}

/// Steps a cube on a floor with `settings` and returns the cube's last linear and angular
/// velocity, its height and the events.
fn cube_on_floor(
    settings: SettingsPolicy,
    start: JPH_RVec3,
    velocity: JPH_Vec3,
    steps: usize,
) -> (JPH_Vec3, JPH_Vec3, Real, Vec<Event>) {
    let world = TestWorld::new(1);
    let listener = Listener::attach(&world, ValidatePolicy::AcceptAll, settings);
    add_floor(&world, 0.0, 0.0, 0.0);
    let cube = add_cube(&world, start);
    set_linear_velocity(&world, cube, velocity);
    let events = run(&world, &listener, steps).concat();
    (
        linear_velocity(&world, cube),
        angular_velocity(&world, cube),
        position(&world, cube).y,
        events,
    )
}

#[test]
fn added_and_persisted_forward_settings_field_by_field() {
    let resting = rvec3(0.0, 0.25, 0.0);
    let still = vec3(0.0, 0.0, 0.0);

    // Jolt's settings reach the callback: friction sqrt(f1 f2), restitution max(r1, r2).
    let world = TestWorld::new(1);
    let listener = Listener::attach(&world, ValidatePolicy::AcceptAll, SettingsPolicy::Keep);
    let floor = add_floor(&world, 0.0, 0.0, 0.0);
    let cube = add_cube(&world, resting);
    // SAFETY: both bodies exist.
    unsafe {
        JPH_BodyInterface_SetFriction(world.body_interface(), cube, 0.5);
        JPH_BodyInterface_SetFriction(world.body_interface(), floor, 0.8);
        JPH_BodyInterface_SetRestitution(world.body_interface(), cube, 0.3);
        JPH_BodyInterface_SetRestitution(world.body_interface(), floor, 0.1);
    }
    let events = run(&world, &listener, 5).concat();
    let (_, settings) = manifolds(&events)[0];
    assert!((settings.combinedFriction - 0.4f32.sqrt()).abs() < 1e-6);
    assert_eq!(settings.combinedRestitution, 0.3);
    assert_eq!(
        [
            settings.invMassScale1,
            settings.invInertiaScale1,
            settings.invMassScale2,
            settings.invInertiaScale2
        ],
        [1.0; 4]
    );
    assert_eq!(settings.isSensor, 0);
    assert_eq!(settings.relativeLinearSurfaceVelocity.x, 0.0);
    assert_eq!(settings.relativeAngularSurfaceVelocity.y, 0.0);
    // SAFETY: the floor exists.
    unsafe { JPH_BodyInterface_SetIsSensor(world.body_interface(), floor, true) };
    let events = run(&world, &listener, 2).concat();
    assert!(manifolds(&events).iter().all(|(_, s)| s.isSensor == 1));
    drop(listener);
    drop(world);

    // The written settings reach Jolt.
    // Body 1 is the floor (the lower id), so its surface moves at -2 m/s.
    let (v, _, _, _) = cube_on_floor(SettingsPolicy::Conveyor, resting, still, 60);
    assert!(v.x < -1.5, "the conveyor drags the cube: {v:?}");
    let (_, w, _, _) = cube_on_floor(SettingsPolicy::Spin, resting, still, 60);
    assert!(w.y.abs() > 1.0, "the surface spins the cube: {w:?}");
    let sliding = vec3(3.0, 0.0, 0.0);
    let (v, _, _, _) = cube_on_floor(SettingsPolicy::Keep, resting, sliding, 90);
    assert!(v.x < 0.5, "friction stops the cube: {v:?}");
    let (v, _, _, _) = cube_on_floor(SettingsPolicy::Frictionless, resting, sliding, 90);
    // Only Jolt's linear damping (0.05 / s) slows it: 3 exp(-0.075) = 2.78.
    assert!(v.x > 2.7, "without friction it slides on: {v:?}");
    let dropped = rvec3(0.0, 1.0, 0.0);
    let (v, _, _, _) = cube_on_floor(SettingsPolicy::Bouncy, dropped, still, 30);
    assert!(v.y > 2.0, "restitution 1 bounces: {v:?}");
    let (v, _, _, _) = cube_on_floor(SettingsPolicy::Keep, dropped, still, 30);
    assert!(v.y.abs() < 0.5, "restitution 0 does not: {v:?}");
    let (_, _, y, _) = cube_on_floor(SettingsPolicy::Sensor, resting, still, 60);
    assert!(y < -1.0, "a sensor contact does not hold the cube: {y}");

    // Mass scales: two cubes without gravity, the lower id (body 1) moving into the other.
    let collide = |policy| {
        let world = TestWorld::new(1);
        let listener = Listener::attach(&world, ValidatePolicy::AcceptAll, policy);
        // SAFETY: the system is live; the gravity is a live temporary that joltc only reads.
        unsafe { JPH_PhysicsSystem_SetGravity(world.system(), &vec3(0.0, 0.0, 0.0)) };
        let first = add_cube(&world, rvec3(-1.0, 0.0, 0.0));
        let second = add_cube(&world, rvec3(1.0, 0.0, 0.0));
        set_linear_velocity(&world, first, vec3(2.0, 0.0, 0.0));
        let events = run(&world, &listener, 120).concat();
        let (pair, _) = manifolds(&events)[0];
        assert_eq!([pair.body1, pair.body2], [first, second]);
        (
            linear_velocity(&world, first).x,
            linear_velocity(&world, second).x,
        )
    };
    let (first, second) = collide(SettingsPolicy::ImmovableBody1);
    // Damping alone leaves 2 exp(-0.1) = 1.81; an even exchange would leave about 0.9 each.
    assert!(
        first > 1.75 && second > 1.75,
        "body 1 is not slowed: {first} {second}"
    );
    let (first, second) = collide(SettingsPolicy::ImmovableBody2);
    assert!(
        first.abs() < 0.1 && second.abs() < 1e-3,
        "body 2 is not moved: {first} {second}"
    );
}

#[test]
fn removed_forwards_the_pair() {
    let world = TestWorld::new(2);
    let listener = Listener::attach(&world, ValidatePolicy::AcceptAll, SettingsPolicy::Keep);
    let floor = add_floor(&world, 0.0, 0.0, 0.0);
    let cube = add_cube(&world, rvec3(0.0, 0.3, 0.0));
    let events = run(&world, &listener, 30).concat();
    let added = events
        .iter()
        .find_map(|event| match *event {
            Event::Added(pair, _) => Some(pair),
            _ => None,
        })
        .expect("the cube lands");
    assert_eq!([added.body1, added.body2], [floor, cube], "ordered by id");

    let mut up = rvec3(0.0, 5.0, 0.0);
    // SAFETY: the body exists; `up` is a live local that joltc only reads.
    unsafe {
        JPH_BodyInterface_SetPosition(
            world.body_interface(),
            cube,
            &mut up,
            JPH_Activation_Activate,
        )
    };
    let removed: Vec<Pair> = run(&world, &listener, 1)
        .concat()
        .iter()
        .filter_map(|event| match *event {
            Event::Removed(pair) => Some(pair),
            _ => None,
        })
        .collect();
    assert_eq!(removed, vec![added]);
    drop(listener);
}

#[test]
fn a_null_validate_proc_accepts() {
    let world = TestWorld::new(1);
    let listener = Listener::attach(&world, ValidatePolicy::RejectAll, SettingsPolicy::Keep);
    // SAFETY: the world holds the framework's world lock, so no other test of this binary has a
    // world that could call the table; the original table is back before the world is dropped.
    unsafe { JPH_ContactListener2_SetProcs(&PROCS_WITHOUT_VALIDATE) };
    add_floor(&world, 0.0, 0.0, 0.0);
    let cube = add_cube(&world, rvec3(0.0, 0.3, 0.0));
    let events = run(&world, &listener, 60).concat();
    // SAFETY: as above.
    unsafe { JPH_ContactListener2_SetProcs(&PROCS) };
    assert!(validated(&events).is_empty());
    assert!(!manifolds(&events).is_empty());
    assert!(position(&world, cube).y > 0.2, "the floor holds the cube");
    drop(listener);
}
