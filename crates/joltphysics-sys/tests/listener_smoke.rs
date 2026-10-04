//! Smoke tests for the listener functions: joltc's contact and body activation listeners with
//! the extension's sub-shape pair getters, and the extension's soft body contact listener.
//!
//! The proc tables are process-global, so this binary installs them once and every test keeps
//! its state in the listener's `userData`.

mod framework;

use std::ffi::c_void;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::ptr::null_mut;
use std::sync::{Mutex, OnceLock, PoisonError};

use framework::*;
use joltphysics_sys::*;

const DT: f32 = 1.0 / 60.0;
const INVALID_BODY: JPH_BodyID = u32::MAX;
const EMPTY_SUB_SHAPE: JPH_SubShapeID = u32::MAX;

/// A contact pair: body and sub-shape ids of both sides.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Pair {
    body1: JPH_BodyID,
    sub_shape1: JPH_SubShapeID,
    body2: JPH_BodyID,
    sub_shape2: JPH_SubShapeID,
}

/// One vertex contact of a soft body, in the soft body's centre-of-mass frame.
#[derive(Clone, Copy, Debug)]
struct VertexContact {
    body: JPH_BodyID,
    normal: JPH_Vec3,
}

#[derive(Clone, Debug)]
enum Event {
    Added(Pair),
    Persisted(Pair),
    Removed(Pair),
    Activated(JPH_BodyID),
    Deactivated(JPH_BodyID),
    SoftValidated { soft: JPH_BodyID, other: JPH_BodyID },
    SoftAdded(SoftContacts),
}

#[derive(Clone, Debug)]
struct SoftContacts {
    soft: JPH_BodyID,
    vertices: Vec<VertexContact>,
    sensors: Vec<JPH_BodyID>,
    /// Whether every out-of-range and no-contact getter behaved as the header states.
    getters_guarded: bool,
}

/// What the soft body validate callback answers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum SoftPolicy {
    Accept,
    Reject,
    Sensor,
}

/// Everything one test's listeners record, reached through `userData`.
struct Recorder {
    events: Mutex<Vec<Event>>,
    soft_policy: SoftPolicy,
}

impl Recorder {
    fn new(soft_policy: SoftPolicy) -> Self {
        Self {
            events: Mutex::new(Vec::new()),
            soft_policy,
        }
    }

    fn push(&self, event: Event) {
        self.events
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .push(event);
    }

    fn take(&self) -> Vec<Event> {
        std::mem::take(&mut *self.events.lock().unwrap_or_else(PoisonError::into_inner))
    }

    fn as_user_data(&self) -> *mut c_void {
        (self as *const Self).cast_mut().cast()
    }
}

/// Runs `f` with the recorder of `user_data`; aborts instead of unwinding into C++.
///
/// # Safety
/// `user_data` is the `as_user_data` of a recorder that outlives the call.
unsafe fn with_recorder<R>(user_data: *mut c_void, f: impl FnOnce(&Recorder) -> R) -> R {
    // SAFETY: the caller guarantees a live recorder; it is only read through `&`.
    let recorder = unsafe { &*user_data.cast::<Recorder>() };
    match catch_unwind(AssertUnwindSafe(|| f(recorder))) {
        Ok(result) => result,
        Err(_) => std::process::abort(),
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

/// joltc's `OnContactAdded`.
///
/// # Safety
/// `user_data` is the `userData` of a contact listener of this file, a live `Recorder`; the
/// bodies and the manifold are the non-null ones Jolt passes for the duration of the callback.
/// `_settings` is not read.
unsafe extern "C" fn on_contact_added(
    user_data: *mut c_void,
    body1: *const JPH_Body,
    body2: *const JPH_Body,
    manifold: *const JPH_ContactManifold,
    _settings: *mut JPH_ContactSettings,
) {
    // SAFETY: joltc passes the listener's `userData` and live callback arguments.
    unsafe {
        let pair = manifold_pair(body1, body2, manifold);
        with_recorder(user_data, |r| r.push(Event::Added(pair)));
    }
}

/// joltc's `OnContactPersisted`.
///
/// # Safety
/// As for [`on_contact_added`].
unsafe extern "C" fn on_contact_persisted(
    user_data: *mut c_void,
    body1: *const JPH_Body,
    body2: *const JPH_Body,
    manifold: *const JPH_ContactManifold,
    _settings: *mut JPH_ContactSettings,
) {
    // SAFETY: as for `on_contact_added`.
    unsafe {
        let pair = manifold_pair(body1, body2, manifold);
        with_recorder(user_data, |r| r.push(Event::Persisted(pair)));
    }
}

/// joltc's `OnContactRemoved`.
///
/// # Safety
/// `user_data` is a live `Recorder` as for [`on_contact_added`]; `pair` is joltc's non-null cast
/// of the Jolt pair, live for the duration of the callback.
unsafe extern "C" fn on_contact_removed(user_data: *mut c_void, pair: *const JPH_SubShapeIDPair) {
    // SAFETY: joltc passes the listener's `userData` and the live Jolt pair; the extension's
    // getters read it through Jolt's accessors.
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

/// joltc's `OnBodyActivated`.
///
/// # Safety
/// `user_data` is the `userData` of an activation listener of this file, a live `Recorder`.
unsafe extern "C" fn on_body_activated(user_data: *mut c_void, id: JPH_BodyID, _: u64) {
    // SAFETY: joltc passes the listener's `userData`.
    unsafe { with_recorder(user_data, |r| r.push(Event::Activated(id))) };
}

/// joltc's `OnBodyDeactivated`.
///
/// # Safety
/// As for [`on_body_activated`].
unsafe extern "C" fn on_body_deactivated(user_data: *mut c_void, id: JPH_BodyID, _: u64) {
    // SAFETY: joltc passes the listener's `userData`.
    unsafe { with_recorder(user_data, |r| r.push(Event::Deactivated(id))) };
}

/// The extension's `OnSoftBodyContactValidate`.
///
/// # Safety
/// `user_data` is the `userData` of a soft body contact listener of this file, a live
/// `Recorder`; both bodies and `settings` are the non-null ones the extension passes for the
/// duration of the callback, and `settings` may be written.
unsafe extern "C" fn on_soft_body_contact_validate(
    user_data: *mut c_void,
    soft_body: *const JPH_Body,
    other_body: *const JPH_Body,
    settings: *mut JPH_SoftBodyContactSettings,
) -> JPH_SoftBodyValidateResult {
    // SAFETY: the extension passes the listener's `userData`, live bodies and live settings.
    unsafe {
        let (soft, other) = (JPH_Body_GetID(soft_body), JPH_Body_GetID(other_body));
        let policy = with_recorder(user_data, |r| {
            r.push(Event::SoftValidated { soft, other });
            r.soft_policy
        });
        match policy {
            SoftPolicy::Accept => JPH_SoftBodyValidateResult_AcceptContact,
            SoftPolicy::Reject => JPH_SoftBodyValidateResult_RejectContact,
            SoftPolicy::Sensor => {
                (*settings).isSensor = true;
                JPH_SoftBodyValidateResult_AcceptContact
            }
        }
    }
}

/// The extension's `OnSoftBodyContactAdded`.
///
/// # Safety
/// `user_data` is a live `Recorder` as for [`on_soft_body_contact_validate`]; the soft body and
/// the manifold are the non-null ones the extension passes for the duration of the callback.
unsafe extern "C" fn on_soft_body_contact_added(
    user_data: *mut c_void,
    soft_body: *const JPH_Body,
    manifold: *const JPH_SoftBodyManifold,
) {
    // SAFETY: the extension passes the listener's `userData`, the live soft body and the live
    // manifold; every getter checks its index.
    unsafe {
        let count = JPH_SoftBodyManifold_GetVertexCount(manifold);
        let mut vertices = Vec::new();
        let mut guarded = true;
        for index in 0..count {
            let mut normal = vec3(9.0, 9.0, 9.0);
            let mut point = vec3(9.0, 9.0, 9.0);
            let has_point = JPH_SoftBodyManifold_GetLocalContactPoint(manifold, index, &mut point);
            let has_normal = JPH_SoftBodyManifold_GetContactNormal(manifold, index, &mut normal);
            if JPH_SoftBodyManifold_HasContact(manifold, index) {
                guarded &= has_point && has_normal;
                vertices.push(VertexContact {
                    body: JPH_SoftBodyManifold_GetContactBodyID(manifold, index),
                    normal,
                });
            } else {
                guarded &= !has_point && !has_normal && point.x == 9.0 && normal.x == 9.0;
                guarded &= JPH_SoftBodyManifold_GetContactBodyID(manifold, index) == INVALID_BODY;
            }
        }
        let mut beyond = vec3(9.0, 9.0, 9.0);
        guarded &= !JPH_SoftBodyManifold_HasContact(manifold, count);
        guarded &= !JPH_SoftBodyManifold_GetLocalContactPoint(manifold, count, &mut beyond);
        guarded &= !JPH_SoftBodyManifold_GetContactNormal(manifold, count, &mut beyond);
        guarded &= beyond.x == 9.0;
        guarded &= JPH_SoftBodyManifold_GetContactBodyID(manifold, count) == INVALID_BODY;
        let sensor_count = JPH_SoftBodyManifold_GetNumSensorContacts(manifold);
        let sensors = (0..sensor_count)
            .map(|i| JPH_SoftBodyManifold_GetSensorContactBodyID(manifold, i))
            .collect();
        guarded &=
            JPH_SoftBodyManifold_GetSensorContactBodyID(manifold, sensor_count) == INVALID_BODY;
        let contacts = SoftContacts {
            soft: JPH_Body_GetID(soft_body),
            vertices,
            sensors,
            getters_guarded: guarded,
        };
        with_recorder(user_data, |r| r.push(Event::SoftAdded(contacts)));
    }
}

static CONTACT_PROCS: JPH_ContactListener_Procs = JPH_ContactListener_Procs {
    OnContactValidate: None,
    OnContactAdded: Some(on_contact_added),
    OnContactPersisted: Some(on_contact_persisted),
    OnContactRemoved: Some(on_contact_removed),
};

static ACTIVATION_PROCS: JPH_BodyActivationListener_Procs = JPH_BodyActivationListener_Procs {
    OnBodyActivated: Some(on_body_activated),
    OnBodyDeactivated: Some(on_body_deactivated),
};

static SOFT_PROCS: JPH_SoftBodyContactListener_Procs = JPH_SoftBodyContactListener_Procs {
    OnSoftBodyContactValidate: Some(on_soft_body_contact_validate),
    OnSoftBodyContactAdded: Some(on_soft_body_contact_added),
};

fn install_procs() {
    static INSTALLED: OnceLock<()> = OnceLock::new();
    INSTALLED.get_or_init(|| {
        // SAFETY: the tables are immutable statics; `OnceLock` orders this one write before
        // every listener this binary creates.
        unsafe {
            JPH_ContactListener_SetProcs(&CONTACT_PROCS);
            JPH_BodyActivationListener_SetProcs(&ACTIVATION_PROCS);
            JPH_SoftBodyContactListener_SetProcs(&SOFT_PROCS);
        }
    });
}

/// The three listeners of one world, recording into one recorder. Detached and destroyed on
/// drop, before the recorder (field order).
struct Listeners {
    system: *mut JPH_PhysicsSystem,
    contact: *mut JPH_ContactListener,
    activation: *mut JPH_BodyActivationListener,
    soft: *mut JPH_SoftBodyContactListener,
    recorder: Box<Recorder>,
}

impl Listeners {
    fn attach(world: &TestWorld, soft_policy: SoftPolicy) -> Self {
        install_procs();
        let recorder = Box::new(Recorder::new(soft_policy));
        let user_data = recorder.as_user_data();
        // SAFETY: the system is live; the listeners keep `user_data`, which the boxed recorder
        // keeps valid until `drop` has detached and destroyed them.
        unsafe {
            let contact = JPH_ContactListener_Create(user_data);
            let activation = JPH_BodyActivationListener_Create(user_data);
            let soft = JPH_SoftBodyContactListener_Create(user_data);
            JPH_PhysicsSystem_SetContactListener(world.system(), contact);
            JPH_PhysicsSystem_SetBodyActivationListener(world.system(), activation);
            JPH_PhysicsSystem_SetSoftBodyContactListener(world.system(), soft);
            Self {
                system: world.system(),
                contact,
                activation,
                soft,
                recorder,
            }
        }
    }
}

impl Drop for Listeners {
    fn drop(&mut self) {
        // SAFETY: the system outlives the listeners (each test drops them first); after the
        // three setters nothing calls them, so they are destroyed once.
        unsafe {
            JPH_PhysicsSystem_SetContactListener(self.system, null_mut());
            JPH_PhysicsSystem_SetBodyActivationListener(self.system, null_mut());
            JPH_PhysicsSystem_SetSoftBodyContactListener(self.system, null_mut());
            JPH_ContactListener_Destroy(self.contact);
            JPH_BodyActivationListener_Destroy(self.activation);
            JPH_SoftBodyContactListener_Destroy(self.soft);
        }
    }
}

/// A static compound of two 2 x 1 x 2 boxes side by side along X, top face at y = 0.
fn add_two_box_floor(world: &TestWorld) -> JPH_BodyID {
    let half = vec3(1.0, 0.5, 1.0);
    let rotation = quat_identity();
    let position = rvec3(0.0, -0.5, 0.0);
    // SAFETY: Jolt is initialised (the world exists); every pointer comes from the `_Create`
    // call before its use and every reference this function holds is released after the
    // object that keeps its own took it.
    let body = unsafe {
        let child = JPH_BoxShape_Create(&half, 0.05);
        let settings = JPH_StaticCompoundShapeSettings_Create();
        for x in [-1.0, 1.0] {
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
            JPH_MotionType_Static,
            OL_NON_MOVING,
        );
        let body = JPH_BodyInterface_CreateAndAddBody(
            world.body_interface(),
            creation,
            JPH_Activation_DontActivate,
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

fn contact_pairs(events: &[Event]) -> Vec<(char, Pair)> {
    events
        .iter()
        .filter_map(|event| match *event {
            Event::Added(pair) => Some(('a', pair)),
            Event::Persisted(pair) => Some(('p', pair)),
            Event::Removed(pair) => Some(('r', pair)),
            _ => None,
        })
        .collect()
}

#[test]
fn rigid_contacts_are_added_persisted_and_removed_with_their_sub_shapes() {
    let world = TestWorld::new(2);
    let listeners = Listeners::attach(&world, SoftPolicy::Accept);
    let floor = add_two_box_floor(&world);
    // Over child 0: with two children, child 1's one-bit sub-shape id has every bit set, like
    // an empty id.
    let cube = add_cube(&world, rvec3(-1.0, 0.3, 0.0));

    let mut added = None;
    for _ in 0..30 {
        world.step(DT);
        for (kind, pair) in contact_pairs(&listeners.recorder.take()) {
            match kind {
                'a' => added = added.or(Some(pair)),
                'p' => assert_eq!(Some(pair), added, "persisted before added"),
                _ => panic!("removed while resting: {pair:?}"),
            }
        }
    }
    let added = added.expect("the cube touches the floor");
    assert!(
        added.body1 < added.body2,
        "body 1 has the lower id: {added:?}"
    );
    assert_eq!(
        [added.body1, added.body2],
        [floor.min(cube), floor.max(cube)]
    );
    let (cube_sub, floor_sub) = if added.body1 == cube {
        (added.sub_shape1, added.sub_shape2)
    } else {
        (added.sub_shape2, added.sub_shape1)
    };
    assert_eq!(cube_sub, EMPTY_SUB_SHAPE);
    assert_ne!(
        floor_sub, EMPTY_SUB_SHAPE,
        "the floor reports its compound child"
    );

    let mut up = rvec3(-1.0, 5.0, 0.0);
    // SAFETY: the body exists; `up` is a live local that joltc only reads.
    unsafe {
        JPH_BodyInterface_SetPosition(
            world.body_interface(),
            cube,
            &mut up,
            JPH_Activation_Activate,
        )
    };
    world.step(DT);
    let removed: Vec<Pair> = contact_pairs(&listeners.recorder.take())
        .into_iter()
        .filter_map(|(kind, pair)| (kind == 'r').then_some(pair))
        .collect();
    assert_eq!(
        removed,
        vec![added],
        "the pair getters return the added pair"
    );
    drop(listeners);
}

#[test]
fn bodies_report_activation_and_deactivation() {
    let world = TestWorld::new(1);
    let listeners = Listeners::attach(&world, SoftPolicy::Accept);
    add_two_box_floor(&world);
    let cube = add_cube(&world, rvec3(1.0, 0.25, 0.0));
    let mut events = listeners.recorder.take();
    for _ in 0..300 {
        world.step(DT);
        events.extend(listeners.recorder.take());
    }
    let activations: Vec<String> = events
        .iter()
        .filter_map(|event| match event {
            Event::Activated(id) if *id == cube => Some("activated".to_owned()),
            Event::Deactivated(id) if *id == cube => Some("deactivated".to_owned()),
            _ => None,
        })
        .collect();
    assert_eq!(activations, ["activated", "deactivated"], "{events:?}");
    drop(listeners);
}

/// A flat 4 x 4 cloth of 1 kg vertices 0.25 m apart in the XZ plane, centred on its origin,
/// with edge constraints. The caller owns the returned reference.
fn cloth() -> *mut JPH_SoftBodySharedSettings {
    init();
    const SIDE: u32 = 4;
    let index = |x: u32, z: u32| z * SIDE + x;
    let mut vertices = Vec::new();
    for z in 0..SIDE {
        for x in 0..SIDE {
            vertices.push(JPH_SoftVertex {
                position: vec3(x as f32 * 0.25 - 0.375, 0.0, z as f32 * 0.25 - 0.375),
                velocity: vec3(0.0, 0.0, 0.0),
                invMass: 1.0,
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
            faces.push(face(index(x, z), index(x, z + 1), index(x + 1, z + 1)));
            faces.push(face(index(x, z), index(x + 1, z + 1), index(x + 1, z)));
        }
    }
    // SAFETY: Jolt is initialised; both arrays are live for the calls and hold the counts
    // passed.
    unsafe {
        let settings = JPH_SoftBodySharedSettings_Create();
        JPH_SoftBodySharedSettings_AddVertices(settings, vertices.as_ptr(), vertices.len() as u32);
        JPH_SoftBodySharedSettings_AddFaces(settings, faces.as_ptr(), faces.len() as u32);
        JPH_SoftBodySharedSettings_CreateConstraints(settings, 0.0, JPH_SoftBodyBendType_None);
        JPH_SoftBodySharedSettings_Optimize(settings);
        settings
    }
}

fn add_cloth(world: &TestWorld, position: JPH_RVec3) -> JPH_BodyID {
    let settings = cloth();
    let rotation = quat_identity();
    // SAFETY: `settings` is live; the creation settings and the body take their own references
    // before the test's is released. `position` and `rotation` are live locals.
    let id = unsafe {
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
    assert_ne!(id, INVALID_BODY);
    id
}

/// A static 2 x 1 x 2 box with its top face at y = 0.
fn add_table(world: &TestWorld) -> JPH_BodyID {
    create_box(
        world.body_interface(),
        vec3(1.0, 0.5, 1.0),
        rvec3(0.0, -0.5, 0.0),
        JPH_MotionType_Static,
        OL_NON_MOVING,
        JPH_Activation_DontActivate,
    )
}

fn cloth_height(world: &TestWorld, id: JPH_BodyID) -> Real {
    let mut position = rvec3(0.0, 0.0, 0.0);
    // SAFETY: the body exists; `position` is a live local.
    unsafe { JPH_BodyInterface_GetCenterOfMassPosition(world.body_interface(), id, &mut position) };
    position.y
}

fn soft_contacts(events: &[Event]) -> Vec<SoftContacts> {
    events
        .iter()
        .filter_map(|event| match event {
            Event::SoftAdded(contacts) => Some(contacts.clone()),
            _ => None,
        })
        .collect()
}

#[test]
fn soft_body_contacts_name_the_body_and_its_normal() {
    let world = TestWorld::new(2);
    let listeners = Listeners::attach(&world, SoftPolicy::Accept);
    let table = add_table(&world);
    let cloth = add_cloth(&world, rvec3(0.0, 0.3, 0.0));
    let mut events = Vec::new();
    for _ in 0..60 {
        world.step(DT);
        events.extend(listeners.recorder.take());
    }
    assert!(events.iter().any(
        |e| matches!(e, Event::SoftValidated { soft, other } if *soft == cloth && *other == table)
    ));
    let contacts = soft_contacts(&events);
    let touching = contacts
        .iter()
        .find(|c| !c.vertices.is_empty())
        .expect("the cloth lands on the table");
    assert_eq!(touching.soft, cloth);
    assert!(touching.sensors.is_empty());
    for contact in &contacts {
        assert!(contact.getters_guarded, "{contact:?}");
    }
    for vertex in &touching.vertices {
        assert_eq!(vertex.body, table);
        // The cloth's rotation is identity, so the normal is in world space: Jolt's contact
        // normal is minus the collision plane normal, which points out of the table.
        assert!(vertex.normal.y < -0.99, "{vertex:?}");
    }
    assert!(
        cloth_height(&world, cloth) > -0.1,
        "the cloth rests on the table"
    );
    drop(listeners);
}

#[test]
fn rejected_soft_body_contacts_let_the_cloth_fall_through() {
    let world = TestWorld::new(1);
    let listeners = Listeners::attach(&world, SoftPolicy::Reject);
    add_table(&world);
    let cloth = add_cloth(&world, rvec3(0.0, 0.3, 0.0));
    for _ in 0..60 {
        world.step(DT);
    }
    let events = listeners.recorder.take();
    assert!(events
        .iter()
        .any(|e| matches!(e, Event::SoftValidated { .. })));
    assert!(soft_contacts(&events).iter().all(|c| c.vertices.is_empty()));
    assert!(
        cloth_height(&world, cloth) < -1.5,
        "the cloth fell through the table"
    );
    drop(listeners);
}

#[test]
fn soft_body_sensor_contacts_list_the_sensor_and_no_vertex() {
    let world = TestWorld::new(1);
    let listeners = Listeners::attach(&world, SoftPolicy::Sensor);
    let table = add_table(&world);
    add_cloth(&world, rvec3(0.0, 0.3, 0.0));
    let mut events = Vec::new();
    for _ in 0..30 {
        world.step(DT);
        events.extend(listeners.recorder.take());
    }
    let contacts = soft_contacts(&events);
    assert!(
        contacts.iter().any(|c| c.sensors == [table]),
        "{contacts:?}"
    );
    assert!(contacts
        .iter()
        .all(|c| c.vertices.is_empty() && c.getters_guarded));
    drop(listeners);
}

#[test]
fn two_systems_record_only_their_own_events() {
    let pair = TestWorldPair::new();
    let worlds = [&pair.first, &pair.second];
    let listeners = worlds.map(|world| Listeners::attach(world, SoftPolicy::Accept));
    let mut cubes = Vec::new();
    for (i, world) in worlds.iter().enumerate() {
        add_two_box_floor(world);
        add_table(world);
        // Static bodies first, a different number per world, so the cubes' ids differ and no
        // body of one world has the other world's cube id.
        for _ in 0..5 * i {
            create_box(
                world.body_interface(),
                vec3(0.5, 0.5, 0.5),
                rvec3(30.0, 0.0, 0.0),
                JPH_MotionType_Static,
                OL_NON_MOVING,
                JPH_Activation_DontActivate,
            );
        }
        cubes.push(add_cube(world, rvec3(1.0, 0.3, 0.0)));
        add_cloth(world, rvec3(0.0, 0.6, 0.0));
    }
    for _ in 0..40 {
        for world in worlds {
            world.step(DT);
        }
    }
    for (i, listeners) in listeners.iter().enumerate() {
        let events = listeners.recorder.take();
        let own_cube = cubes[i];
        let other_cube = cubes[1 - i];
        assert_ne!(own_cube, other_cube);
        assert!(events
            .iter()
            .any(|e| matches!(e, Event::Activated(id) if *id == own_cube)));
        assert!(!events
            .iter()
            .any(|e| matches!(e, Event::Activated(id) if *id == other_cube)));
        assert!(!contact_pairs(&events).is_empty());
        assert!(!soft_contacts(&events).is_empty());
    }
    drop(listeners);
}
