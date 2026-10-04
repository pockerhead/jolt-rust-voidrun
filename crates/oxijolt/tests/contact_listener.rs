//! Contact listeners that change how Jolt resolves contacts.

mod common;

use std::panic::{catch_unwind, AssertUnwindSafe};
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::Arc;

use common::events::*;
use common::*;
use oxijolt::*;

/// Material user data of ice.
const ICE: u64 = 1;

/// Frictionless contacts with ice.
struct IceIsSlippery;

impl ContactListener for IceIsSlippery {
    fn contact_added(&self, manifold: &ContactManifold, settings: &mut ContactSettings) {
        self.contact_persisted(manifold, settings);
    }

    fn contact_persisted(&self, manifold: &ContactManifold, settings: &mut ContactSettings) {
        if manifold.materials.contains(&Some(ICE)) {
            settings.set_combined_friction(0.0).unwrap();
        }
    }
}

/// How far an ice cube slides down a 20 degree ramp of high friction in one second.
fn ramp_slide(listener: Option<Arc<dyn ContactListener>>) -> Real {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    world.set_contact_listener(listener);
    let ramp = Shape::new_box(Vec3::new(10.0, 0.5, 2.0)).unwrap();
    let tilt = quat_about(Vec3::new(0.0, 0.0, 1.0), 20f32.to_radians());
    world
        .create_body(
            &ramp,
            &BodySettings::new_static().rotation(tilt).friction(1.0),
        )
        .unwrap();
    let ice = PhysicsMaterial::new(ICE).unwrap();
    let cube_shape = Shape::new_box_with_material(Vec3::new(0.25, 0.25, 0.25), 0.05, &ice).unwrap();
    let start = RVec3::new(0.0, 0.85, 0.0);
    let cube = world
        .create_body(
            &cube_shape,
            &BodySettings::new_dynamic()
                .position(start)
                .rotation(tilt)
                .friction(1.0),
        )
        .unwrap();
    step(&mut world, 60);
    let end = world.body(cube).unwrap().position();
    let (dx, dy) = (end.x - start.x, end.y - start.y);
    (dx * dx + dy * dy).sqrt()
}

#[test]
fn a_listener_makes_ice_slippery() {
    let sticky = ramp_slide(None);
    let slippery = ramp_slide(Some(Arc::new(IceIsSlippery)));
    assert!(sticky < 0.1, "{sticky}");
    assert!(slippery > 1.0, "{slippery}");
}

/// A conveyor belt: the floor's surface moves along +x under body 2.
struct Conveyor;

impl ContactListener for Conveyor {
    fn contact_added(&self, manifold: &ContactManifold, settings: &mut ContactSettings) {
        self.contact_persisted(manifold, settings);
    }

    fn contact_persisted(&self, _: &ContactManifold, settings: &mut ContactSettings) {
        settings
            .set_relative_linear_surface_velocity(Vec3::new(2.0, 0.0, 0.0))
            .unwrap();
    }
}

#[test]
fn a_conveyor_moves_a_resting_cube() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    add_floor(&mut world);
    world.set_contact_listener(Some(Arc::new(Conveyor)));
    let cube = add_small_cube(&mut world, RVec3::new(0.0, 0.25, 0.0));
    let before = world.body(cube).unwrap().position();
    step(&mut world, 60);
    let after = world.body(cube).unwrap().position();
    assert!((after.x - before.x).abs() > 0.5, "{before:?} -> {after:?}");
    assert!((after.y - before.y).abs() < 0.01, "{before:?} -> {after:?}");
}

/// Turns every contact into a sensor contact.
struct Ghost;

impl ContactListener for Ghost {
    fn contact_added(&self, _: &ContactManifold, settings: &mut ContactSettings) {
        settings.set_is_sensor(true).unwrap();
    }

    fn contact_persisted(&self, _: &ContactManifold, settings: &mut ContactSettings) {
        settings.set_is_sensor(true).unwrap();
    }
}

#[test]
fn sensor_contacts_let_a_cube_fall_through_and_are_still_reported() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 2);
    world.set_event_settings(contacts());
    world.set_contact_listener(Some(Arc::new(Ghost)));
    add_two_box_floor(&mut world);
    let cube = add_small_cube(&mut world, RVec3::new(-1.0, 0.3, 0.0));
    step(&mut world, 60);
    assert!(world.body(cube).unwrap().position().y < -1.0);
    let events = world.take_events();
    let ContactEvent::Added { settings, .. } = &events.contacts[0] else {
        panic!("{events:?}");
    };
    assert!(
        settings.is_sensor(),
        "the event holds the settings Jolt used"
    );
}

/// Rejects every soft body contact.
struct NoSoftContacts;

impl ContactListener for NoSoftContacts {
    fn soft_body_contact_validate(
        &self,
        _: BodyId,
        _: BodyId,
        _: &mut SoftBodyContactSettings,
    ) -> SoftBodyValidateResult {
        SoftBodyValidateResult::RejectContact
    }
}

#[test]
fn rejected_soft_body_contacts_let_a_cloth_fall_through() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    world.set_event_settings(EventSettings::default().soft_body_validations(true));
    world.set_contact_listener(Some(Arc::new(NoSoftContacts)));
    let table = add_table(&mut world, 0.0);
    let cloth = add_cloth(&mut world, RVec3::new(0.0, 0.3, 0.0), Quat::IDENTITY);
    step(&mut world, 60);
    assert!(world.body(cloth).unwrap().position().y < -1.0);
    let validations = world.take_events().soft_body_validations;
    assert!(validations
        .iter()
        .any(|v| v.other == table && v.result == SoftBodyValidateResult::RejectContact));
}

/// Panics in every contact callback, or only once.
struct Panicking {
    always: bool,
    panicked: std::sync::atomic::AtomicBool,
}

impl ContactListener for Panicking {
    fn contact_persisted(&self, _: &ContactManifold, settings: &mut ContactSettings) {
        settings.set_combined_friction(0.0).unwrap();
        if self.always
            || !self
                .panicked
                .swap(true, std::sync::atomic::Ordering::Relaxed)
        {
            panic!("contact listener panic");
        }
    }
}

#[test]
fn a_panicking_listener_is_resumed_by_step() {
    for always in [false, true] {
        let mut world = world(Vec3::new(0.0, -9.81, 0.0), 4);
        world.set_contact_listener(Some(Arc::new(Panicking {
            always,
            panicked: Default::default(),
        })));
        add_floor(&mut world);
        let cube = world
            .create_body(
                &Shape::new_box(Vec3::new(0.25, 0.25, 0.25)).unwrap(),
                &BodySettings::new_dynamic()
                    .position(RVec3::new(0.0, 0.3, 0.0))
                    .allow_sleeping(false),
            )
            .unwrap();
        let mut panics = 0;
        for _ in 0..60 {
            match catch_unwind(AssertUnwindSafe(|| world.step(DT))) {
                Ok(report) => assert!(report.unwrap().is_complete()),
                Err(payload) => {
                    assert_eq!(
                        payload.downcast_ref::<&str>(),
                        Some(&"contact listener panic")
                    );
                    panics += 1;
                }
            }
        }
        if always {
            assert!(
                panics > 50,
                "every step with a persisted contact panics: {panics}"
            );
        } else {
            assert_eq!(panics, 1);
        }
        world.remove_body(cube).unwrap();
        step(&mut world, 1);
    }
}

#[test]
fn continuous_collision_with_a_listener_steps_cleanly() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 2);
    world.set_event_settings(EventSettings::default().persisted_contacts(true));
    world.set_contact_listener(Some(Arc::new(RoughContacts)));
    add_floor(&mut world);
    let shape = Shape::new_box(Vec3::new(0.1, 0.1, 0.1)).unwrap();
    let bullet = world
        .create_body(
            &shape,
            &BodySettings::new_dynamic()
                .position(RVec3::new(0.0, 3.0, 0.0))
                .linear_velocity(Vec3::new(5.0, -200.0, 0.0))
                .motion_quality(MotionQuality::LinearCast),
        )
        .unwrap();
    let mut contacts = 0;
    for _ in 0..30 {
        step(&mut world, 1);
        contacts += world.take_events().contacts.len();
    }
    let position = world.body(bullet).unwrap().position();
    assert!(
        position.y > -0.05,
        "the bullet did not tunnel: {position:?}"
    );
    assert!(contacts > 0);
}

/// Puts fixed inverse mass and inertia scales on every contact, rigid or soft, and counts the
/// calls.
struct Scales {
    mass: f32,
    inertia: f32,
    vertex: f32,
    calls: AtomicU32,
}

impl Scales {
    fn new(mass: f32, inertia: f32, vertex: f32) -> Arc<Self> {
        Arc::new(Self {
            mass,
            inertia,
            vertex,
            calls: AtomicU32::new(0),
        })
    }

    fn calls(&self) -> u32 {
        self.calls.load(Ordering::Relaxed)
    }
}

impl ContactListener for Scales {
    fn contact_added(&self, manifold: &ContactManifold, settings: &mut ContactSettings) {
        self.contact_persisted(manifold, settings);
    }

    fn contact_persisted(&self, _: &ContactManifold, settings: &mut ContactSettings) {
        settings.set_inv_mass_scale1(self.mass).unwrap();
        settings.set_inv_mass_scale2(self.mass).unwrap();
        settings.set_inv_inertia_scale1(self.inertia).unwrap();
        settings.set_inv_inertia_scale2(self.inertia).unwrap();
        self.calls.fetch_add(1, Ordering::Relaxed);
    }

    fn soft_body_contact_validate(
        &self,
        _: BodyId,
        _: BodyId,
        settings: &mut SoftBodyContactSettings,
    ) -> SoftBodyValidateResult {
        settings.set_inv_mass_scale1(self.vertex).unwrap();
        settings.set_inv_mass_scale2(self.mass).unwrap();
        settings.set_inv_inertia_scale2(self.inertia).unwrap();
        self.calls.fetch_add(1, Ordering::Relaxed);
        SoftBodyValidateResult::AcceptContact
    }
}

#[allow(clippy::unnecessary_cast)] // `Real` is `f32` without the `double-precision` feature.
fn is_finite_point(p: RVec3) -> bool {
    [p.x as f64, p.y as f64, p.z as f64]
        .iter()
        .all(|c| c.is_finite())
}

fn is_finite_vector(v: Vec3) -> bool {
    [v.x, v.y, v.z].iter().all(|c| c.is_finite())
}

fn is_finite_body(world: &PhysicsWorld, id: BodyId) -> bool {
    let body = world.body(id).unwrap();
    is_finite_point(body.position())
        && is_finite_vector(body.linear_velocity())
        && is_finite_vector(body.angular_velocity())
}

/// A floor 40 m thick, so that a body at the velocity bound sinks deep into it in one step.
fn add_thick_floor(world: &mut PhysicsWorld) -> BodyId {
    let shape = Shape::new_box(Vec3::new(100.0, 20.0, 100.0)).unwrap();
    world
        .create_body(
            &shape,
            &BodySettings::new_static().position(RVec3::new(0.0, -20.0, 0.0)),
        )
        .unwrap()
}

/// Bodies of `limits::MIN_MASS` and `limits::MAX_MASS` (a cube, and a sphere whose contact
/// normal passes through its centre), discrete and continuous, thrown at a floor at the
/// velocity bounds, and a light cube thrown at a heavy one, with the mass and inertia scales
/// at `limits::MIN_CONTACT_SCALE` or 0: every body stays finite. CI runs this in the asserts
/// build too.
#[test]
fn contact_scales_at_their_floor_step_finitely() {
    let floor = limits::MIN_CONTACT_SCALE;
    let thrown = Vec3::new(0.0, -limits::MAX_LINEAR_VELOCITY, 0.0);
    let spin = Vec3::new(limits::MAX_ANGULAR_VELOCITY, 0.0, 0.0);
    let light_cube = Shape::new_box(Vec3::new(0.03, 0.03, 0.03)).unwrap();
    let light_ball = Shape::new_sphere(0.03).unwrap();
    let heavy_cube = Shape::new_box(Vec3::new(5.0, 5.0, 5.0)).unwrap();
    let heavy_ball = Shape::new_sphere(5.0).unwrap();
    // (shape, mass, start height of the centre)
    let bodies = [
        (&light_cube, limits::MIN_MASS, 5.03),
        (&light_ball, limits::MIN_MASS, 5.03),
        (&heavy_cube, limits::MAX_MASS, 10.0),
        (&heavy_ball, limits::MAX_MASS, 10.0),
    ];
    for (mass_scale, inertia_scale) in [(floor, floor), (0.0, floor), (floor, 0.0)] {
        for (shape, mass, height) in bodies {
            for quality in [MotionQuality::Discrete, MotionQuality::LinearCast] {
                let mut world = world(Vec3::new(0.0, -9.81, 0.0), 2);
                let listener = Scales::new(mass_scale, inertia_scale, 1.0);
                world.set_contact_listener(Some(listener.clone()));
                add_thick_floor(&mut world);
                let settings = BodySettings::new_dynamic()
                    .position(RVec3::new(0.0, height, 0.0))
                    .mass(mass)
                    .linear_velocity(thrown)
                    .angular_velocity(spin)
                    .motion_quality(quality);
                let id = world.create_body(shape, &settings).unwrap();
                for tick in 0..30 {
                    assert!(world.step(DT).unwrap().is_complete());
                    let case = (mass_scale, inertia_scale, mass, quality, tick);
                    assert!(is_finite_body(&world, id), "{case:?}");
                }
                assert!(listener.calls() > 0, "the body touched the floor");
            }
        }

        let mut world = world(Vec3::new(0.0, -9.81, 0.0), 2);
        let listener = Scales::new(mass_scale, inertia_scale, 1.0);
        world.set_contact_listener(Some(listener.clone()));
        add_thick_floor(&mut world);
        let heavy = world
            .create_body(
                &heavy_cube,
                &BodySettings::new_dynamic()
                    .position(RVec3::new(0.0, 5.0, 0.0))
                    .mass(limits::MAX_MASS),
            )
            .unwrap();
        let light = world
            .create_body(
                &light_cube,
                &BodySettings::new_dynamic()
                    .position(RVec3::new(1.0, 15.0, 1.0))
                    .mass(limits::MIN_MASS)
                    .linear_velocity(thrown)
                    .angular_velocity(spin),
            )
            .unwrap();
        for tick in 0..30 {
            assert!(world.step(DT).unwrap().is_complete());
            for id in [heavy, light] {
                let case = (mass_scale, inertia_scale, tick);
                assert!(is_finite_body(&world, id), "{case:?}");
            }
        }
        assert!(listener.calls() > 0, "the light cube hit the heavy one");
    }
}

/// A cloth thrown at the velocity bound onto blocks of `limits::MIN_MASS` and
/// `limits::MAX_MASS`, with the vertex and block scales at `limits::MIN_CONTACT_SCALE` or 0:
/// the cloth and the block stay finite.
#[test]
fn soft_body_contact_scales_at_their_floor_step_finitely() {
    let floor = limits::MIN_CONTACT_SCALE;
    let cases = [
        (floor, floor, floor),
        (0.0, floor, floor),
        (0.0, floor, 0.0),
        (floor, 0.0, 0.0),
    ];
    let block_shape = Shape::new_box(Vec3::new(1.0, 0.25, 1.0)).unwrap();
    let down = Vec3::new(0.0, -limits::MAX_LINEAR_VELOCITY, 0.0);
    for (vertex, mass_scale, inertia_scale) in cases {
        for mass in [limits::MIN_MASS, limits::MAX_MASS] {
            let mut world = world(Vec3::new(0.0, -9.81, 0.0), 2);
            let listener = Scales::new(mass_scale, inertia_scale, vertex);
            world.set_contact_listener(Some(listener.clone()));
            add_thick_floor(&mut world);
            let block = world
                .create_body(
                    &block_shape,
                    &BodySettings::new_dynamic()
                        .position(RVec3::new(0.0, 0.25, 0.0))
                        .mass(mass)
                        .allow_sleeping(false),
                )
                .unwrap();
            let cloth = add_cloth(&mut world, RVec3::new(0.0, 1.0, 0.0), Quat::IDENTITY);
            let mut soft = world.soft_body_mut(cloth).unwrap();
            for index in 0..soft.vertex_count() as u32 {
                soft.set_vertex_velocity(index, down).unwrap();
            }
            for tick in 0..60 {
                assert!(world.step(DT).unwrap().is_complete());
                let case = (vertex, mass_scale, inertia_scale, mass, tick);
                assert!(is_finite_body(&world, block), "{case:?}");
                let vertices = world.soft_body(cloth).unwrap().vertices();
                let finite = vertices
                    .iter()
                    .all(|v| is_finite_point(v.position) && is_finite_vector(v.velocity));
                assert!(finite, "{case:?}");
            }
            assert!(listener.calls() > 0, "the cloth reached the block");
        }
    }
}
