//! Collision estimates on added contacts: Jolt's own scenario against the stepped result, a
//! bounce, the continuous stage, when estimates are absent, which settings they use, and what
//! they leave out.

mod common;

use std::panic::{catch_unwind, AssertUnwindSafe};
use std::sync::{Arc, Mutex};

use common::*;
use oxijolt::*;

fn estimates() -> EventSettings {
    EventSettings::default().collision_estimates(true)
}

/// The estimates of the Added events of the pair `a`-`b` in `events`.
fn estimates_of(events: &WorldEvents, a: BodyId, b: BodyId) -> Vec<Option<CollisionEstimate>> {
    events
        .contacts
        .iter()
        .filter_map(|event| match event {
            ContactEvent::Added {
                manifold, estimate, ..
            } => {
                let bodies = [manifold.pair.body1, manifold.pair.body2];
                (bodies == [a, b] || bodies == [b, a]).then(|| estimate.clone())
            }
            _ => None,
        })
        .collect()
}

fn close(a: Vec3, b: Vec3, tolerance: f32) -> bool {
    (a.x - b.x).abs() <= tolerance
        && (a.y - b.y).abs() <= tolerance
        && (a.z - b.z).abs() <= tolerance
}

fn dot(a: Vec3, b: Vec3) -> f32 {
    a.x * b.x + a.y * b.y + a.z * b.z
}

/// Jolt's own test (`EstimateCollisionResponseTest.cpp`): a thin box moving at (1, 1, 0) m/s
/// against a larger box it overlaps by 0.1 mm, without gravity or damping; the estimate equals
/// the velocities after one step within 1e-3.
#[test]
fn estimates_match_one_step_of_an_isolated_pair() {
    let thin = Vec3::new(0.1, 1.0, 2.0);
    let large = Vec3::new(0.2, 3.0, 4.0);
    let (thin_shape, large_shape) = (
        Shape::new_box(thin).unwrap(),
        Shape::new_box(large).unwrap(),
    );
    let mut friction_seen = false;
    for motion_type in [
        MotionType::Static,
        MotionType::Kinematic,
        MotionType::Dynamic,
    ] {
        for restitution in [0.0, 0.3, 1.0] {
            for friction in [0.0, 0.3, 1.0] {
                for (y, z) in [(0.0, 0.0), (0.5, 0.5)] {
                    for spin in [0.0, 1.0] {
                        let mut world = world(Vec3::ZERO, 1);
                        world.set_event_settings(estimates());
                        let base = RVec3::new(1.0, 2.0, 3.0);
                        let box1 = world
                            .create_body(
                                &thin_shape,
                                &BodySettings::new_dynamic()
                                    .position(base)
                                    .friction(friction)
                                    .restitution(restitution)
                                    .linear_damping(0.0)
                                    .angular_damping(0.0)
                                    .linear_velocity(Vec3::new(1.0, 1.0, 0.0))
                                    .angular_velocity(Vec3::new(0.0, spin, 0.0)),
                            )
                            .unwrap();
                        let at = RVec3::new(
                            base.x + Real::from(thin.x + large.x) - 1.0e-4,
                            base.y + y,
                            base.z + z,
                        );
                        let other = match motion_type {
                            MotionType::Static => BodySettings::new_static(),
                            MotionType::Kinematic => BodySettings::new_kinematic()
                                .linear_velocity(Vec3::new(-1.0, 0.0, 0.0)),
                            MotionType::Dynamic => BodySettings::new_dynamic()
                                .linear_velocity(Vec3::new(-1.0, 0.0, 0.0)),
                        };
                        let box2 = world
                            .create_body(
                                &large_shape,
                                &other
                                    .position(at)
                                    .friction(friction)
                                    .restitution(restitution)
                                    .linear_damping(0.0)
                                    .angular_damping(0.0),
                            )
                            .unwrap();
                        let before = world.body(box1).unwrap().linear_velocity();
                        step(&mut world, 1);
                        let events = world.take_events();
                        let found = estimates_of(&events, box1, box2);
                        assert_eq!(found.len(), 1, "{motion_type:?}");
                        let estimate = found[0].clone().expect("an estimate");
                        let what = format!(
                            "{motion_type:?} e {restitution} f {friction} y {y} z {z} w {spin}"
                        );
                        let (b1, b2) = (world.body(box1).unwrap(), world.body(box2).unwrap());
                        assert!(
                            close(estimate.linear_velocity1, b1.linear_velocity(), 1.0e-3),
                            "{what}"
                        );
                        assert!(
                            close(estimate.angular_velocity1, b1.angular_velocity(), 1.0e-3),
                            "{what}"
                        );
                        assert!(
                            close(estimate.linear_velocity2, b2.linear_velocity(), 1.0e-3),
                            "{what}"
                        );
                        assert!(
                            close(estimate.angular_velocity2, b2.angular_velocity(), 1.0e-3),
                            "{what}"
                        );
                        // Against a static box the normal impulses are what box 1's momentum lost
                        // along the normal (+x, from box 1 to box 2).
                        if motion_type == MotionType::Static {
                            let mass = b1.mass().unwrap();
                            let lost = mass
                                * dot(
                                    Vec3::new(before.x - estimate.linear_velocity1.x, 0.0, 0.0),
                                    Vec3::new(1.0, 0.0, 0.0),
                                );
                            let total: f32 = estimate.contact_impulses.iter().sum();
                            assert!(
                                (lost - total).abs() <= 1.0e-3 * mass,
                                "{what}: {lost} vs {total}"
                            );
                            if friction > 0.0 {
                                assert!(
                                    estimate.friction_impulse1 != 0.0
                                        || estimate.friction_impulse2 != 0.0,
                                    "{what}"
                                );
                                friction_seen = true;
                            }
                        }
                    }
                }
            }
        }
    }
    assert!(friction_seen);
}

#[test]
fn a_bouncing_box_is_estimated_to_leave_at_half_speed() {
    let mut world = world(Vec3::ZERO, 1);
    world.set_event_settings(estimates());
    let floor = world
        .create_body(
            &Shape::new_box(Vec3::new(10.0, 0.5, 10.0)).unwrap(),
            &BodySettings::new_static()
                .position(RVec3::new(0.0, -0.5, 0.0))
                .restitution(0.5),
        )
        .unwrap();
    let cube = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_dynamic()
                .position(RVec3::new(0.0, 0.505, 0.0))
                .restitution(0.5)
                .linear_damping(0.0)
                .linear_velocity(Vec3::new(0.0, -5.0, 0.0)),
        )
        .unwrap();
    step(&mut world, 1);
    let found = estimates_of(&world.take_events(), floor, cube);
    let estimate = found[0].clone().unwrap();
    assert!(
        (estimate.linear_velocity2.y - 2.5).abs() < 1.0e-3,
        "{estimate:?}"
    );
    assert_eq!(estimate.contact_impulses.len(), 4);
}

#[test]
fn a_contact_found_by_the_continuous_stage_is_estimated() {
    let mut world = world(Vec3::ZERO, 1);
    world.set_event_settings(estimates());
    let wall = world
        .create_body(
            &Shape::new_box(Vec3::new(0.01, 2.0, 2.0)).unwrap(),
            &BodySettings::new_static(),
        )
        .unwrap();
    let ball = world
        .create_body(
            &Shape::new_sphere(0.05).unwrap(),
            &BodySettings::new_dynamic()
                .position(RVec3::new(-2.0, 0.0, 0.0))
                .motion_quality(MotionQuality::LinearCast)
                .restitution(0.0)
                .linear_velocity(Vec3::new(200.0, 0.0, 0.0)),
        )
        .unwrap();
    step(&mut world, 1);
    let found = estimates_of(&world.take_events(), wall, ball);
    assert_eq!(found.len(), 1);
    let estimate = found[0]
        .clone()
        .expect("the continuous contact is estimated");
    // Without restitution the ball is estimated to stop along the normal; the continuous stage
    // estimates from velocities after the solve, so the bound is loose.
    assert!(estimate.linear_velocity2.x.abs() < 1.0, "{estimate:?}");
    assert!(world.body(ball).unwrap().position().x < 0.0);
}

/// A cube landing on a floor with `settings` and an optional listener; the estimates of its
/// Added contacts with the floor during 30 steps.
fn landing(
    settings: EventSettings,
    listener: Option<Arc<dyn ContactListener>>,
) -> (Vec<Option<CollisionEstimate>>, Vec<u8>) {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    world.set_event_settings(settings);
    world.set_contact_listener(listener);
    let floor = add_floor(&mut world);
    let cube = add_cube(&mut world, RVec3::new(0.3, 0.8, -0.2));
    world
        .body_mut(cube)
        .unwrap()
        .set_angular_velocity(Vec3::new(1.0, 0.5, -2.0))
        .unwrap();
    let mut found = Vec::new();
    let mut digest = Vec::new();
    for _ in 0..30 {
        // A panicking listener's panic is resumed by `step`; the step itself completed.
        if let Ok(report) = catch_unwind(AssertUnwindSafe(|| world.step(DT))) {
            assert!(report.unwrap().is_complete());
        }
        found.extend(estimates_of(&world.take_events(), floor, cube));
        record_body(&world, cube, &mut digest);
    }
    (found, digest)
}

#[test]
fn estimates_are_off_by_default_and_follow_the_contacts_switch() {
    let (found, _) = landing(EventSettings::default().contacts(true), None);
    assert!(!found.is_empty());
    assert!(found.iter().all(Option::is_none));
    let settings = estimates().contacts(false);
    assert!(!settings.reports_collision_estimates() && !settings.reports_contacts());
    let settings = EventSettings::default().collision_estimates(true);
    assert!(settings.reports_collision_estimates() && settings.reports_contacts());
}

#[test]
fn estimating_does_not_change_the_simulation() {
    let (with, with_digest) = landing(estimates(), None);
    let (_, without_digest) = landing(EventSettings::default(), None);
    assert!(with.iter().all(Option::is_some));
    assert_eq!(with_digest, without_digest);
}

#[test]
fn sensor_contacts_have_no_estimate() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    world.set_event_settings(estimates());
    let trigger = world
        .create_body(
            &Shape::new_box(Vec3::new(2.0, 2.0, 2.0)).unwrap(),
            &BodySettings::new_static().sensor(true),
        )
        .unwrap();
    let cube = add_cube(&mut world, RVec3::new(0.0, 1.0, 0.0));
    step(&mut world, 1);
    let found = estimates_of(&world.take_events(), trigger, cube);
    assert_eq!(found, [None]);
}

/// Sets friction 0 on every added contact.
struct Frictionless;

impl ContactListener for Frictionless {
    fn contact_added(&self, _: &ContactManifold, settings: &mut ContactSettings) {
        settings.set_combined_friction(0.0).unwrap();
    }
}

/// Panics in every added contact.
struct Panicking;

impl ContactListener for Panicking {
    fn contact_added(&self, _: &ContactManifold, _: &mut ContactSettings) {
        panic!("listener panic");
    }
}

/// Halves both inverse mass scales of every added contact.
struct HalfMass;

impl ContactListener for HalfMass {
    fn contact_added(&self, _: &ContactManifold, settings: &mut ContactSettings) {
        settings.set_inv_mass_scale1(0.5).unwrap();
        settings.set_inv_mass_scale2(0.5).unwrap();
    }
}

#[test]
fn estimates_use_the_settings_jolt_resolves_the_contact_with() {
    let (jolts, _) = landing(estimates(), None);
    let jolts = jolts[0].clone().unwrap();
    assert!(jolts.friction_impulse1 != 0.0 || jolts.friction_impulse2 != 0.0);

    let (frictionless, _) = landing(estimates(), Some(Arc::new(Frictionless)));
    let frictionless = frictionless[0].clone().unwrap();
    assert_eq!(
        (
            frictionless.friction_impulse1,
            frictionless.friction_impulse2
        ),
        (0.0, 0.0)
    );

    // A panicking listener changes nothing: the estimate is that of Jolt's own settings.
    let (panicked, _) = landing(estimates(), Some(Arc::new(Panicking)));
    assert_eq!(panicked[0].clone().unwrap(), jolts);

    // Mass scales are not part of the estimate.
    let (scaled, _) = landing(estimates(), Some(Arc::new(HalfMass)));
    assert_eq!(scaled[0].clone().unwrap(), jolts);
}

/// Keeps the settings of the donor's first contact after giving them an angular surface
/// velocity, and returns them for every later added contact of another body.
struct Transplant {
    donor: BodyId,
    kept: Mutex<Option<ContactSettings>>,
}

impl ContactListener for Transplant {
    fn contact_added(&self, manifold: &ContactManifold, settings: &mut ContactSettings) {
        let pair = manifold.pair;
        let mut kept = self.kept.lock().unwrap();
        if pair.body1 == self.donor || pair.body2 == self.donor {
            settings
                .set_relative_angular_surface_velocity(Vec3::new(0.0, 20.0, 0.0))
                .unwrap();
            *kept = Some(*settings);
        } else if let Some(value) = *kept {
            *settings = value;
        }
    }
}

#[test]
fn a_rejected_transplant_is_estimated_with_jolts_settings() {
    let run = |transplant: bool| {
        let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
        world.set_event_settings(estimates());
        let floor = world
            .create_body(
                &Shape::new_box(Vec3::new(100.0, 0.5, 100.0)).unwrap(),
                &BodySettings::new_static().position(RVec3::new(0.0, -0.5, 0.0)),
            )
            .unwrap();
        let small = Shape::new_box(Vec3::new(0.25, 0.25, 0.25)).unwrap();
        let donor = world
            .create_body(
                &small,
                &BodySettings::new_dynamic().position(RVec3::new(0.0, 0.3, 0.0)),
            )
            .unwrap();
        // 60 m from the floor's centre of mass: the kept spin does not fit its lever.
        let recipient = world
            .create_body(
                &small,
                &BodySettings::new_dynamic().position(RVec3::new(60.0, 0.3, 0.0)),
            )
            .unwrap();
        if transplant {
            world.set_contact_listener(Some(Arc::new(Transplant {
                donor,
                kept: Mutex::default(),
            })));
        }
        let mut rejected = 0;
        let mut found = Vec::new();
        for _ in 0..10 {
            step(&mut world, 1);
            let events = world.take_events();
            rejected += events.rejected_contact_settings.len();
            found.extend(estimates_of(&events, floor, recipient));
        }
        (rejected, found)
    };
    let (_, jolts) = run(false);
    let (rejected, transplanted) = run(true);
    assert!(rejected > 0);
    assert!(!jolts.is_empty() && jolts.iter().all(Option::is_some));
    assert_eq!(transplanted, jolts);
}

#[test]
fn locked_axes_are_not_part_of_the_estimate() {
    // A floor tilted about x: its normal has a z component, which a `PLANE_2D` body cannot
    // follow. Jolt's estimate does not know that.
    let mut world = world(Vec3::ZERO, 1);
    world.set_event_settings(estimates());
    let tilt = quat_about(Vec3::new(1.0, 0.0, 0.0), 0.3);
    let floor = world
        .create_body(
            &Shape::new_box(Vec3::new(10.0, 0.5, 10.0)).unwrap(),
            &BodySettings::new_static()
                .position(RVec3::new(0.0, -0.5, 0.0))
                .rotation(tilt),
        )
        .unwrap();
    let cube = world
        .create_body(
            &Shape::new_sphere(0.5).unwrap(),
            &BodySettings::new_dynamic()
                .position(RVec3::new(0.0, 0.52, 0.0))
                .allowed_dofs(AllowedDofs::PLANE_2D)
                .restitution(0.0)
                .linear_velocity(Vec3::new(0.0, -5.0, 0.0)),
        )
        .unwrap();
    step(&mut world, 1);
    let estimate = estimates_of(&world.take_events(), floor, cube)[0]
        .clone()
        .unwrap();
    assert!(estimate.linear_velocity2.z.abs() > 0.1, "{estimate:?}");
    assert_eq!(world.body(cube).unwrap().linear_velocity().z, 0.0);
}
