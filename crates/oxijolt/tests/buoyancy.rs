//! Buoyancy: floating, drag, the velocity clamp, return values and waking, the input rules and
//! a seeded stress of accepted and refused calls.

mod common;

use common::events::add_cloth;
use common::*;
use oxijolt::*;

const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);
const UP: Vec3 = Vec3::new(0.0, 1.0, 0.0);

fn water() -> BuoyancySettings {
    BuoyancySettings::default()
}

/// Water whose surface is at height `y`.
fn water_at(y: Real) -> BuoyancySettings {
    water().surface(RVec3::new(0.0, y, 0.0), UP)
}

fn buoy(
    world: &mut PhysicsWorld,
    id: BodyId,
    settings: &BuoyancySettings,
    gravity: Vec3,
) -> Result<bool, BodyError> {
    world
        .body_mut(id)
        .unwrap()
        .apply_buoyancy_impulse(settings, gravity, DT)
}

/// Steps `ticks` times, applying `settings` with the world's gravity to `id` before each step.
fn float(world: &mut PhysicsWorld, id: BodyId, settings: &BuoyancySettings, ticks: usize) {
    let gravity = world.gravity();
    for _ in 0..ticks {
        buoy(world, id, settings, gravity).unwrap();
        step(world, 1);
    }
}

/// Finiteness of every component; the crate keeps its own vector checks private.
trait AllFinite {
    fn all_finite(self) -> bool;
}

impl AllFinite for Vec3 {
    fn all_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }
}

fn speed(body: &BodyRef<'_>) -> f32 {
    length(body.linear_velocity())
}

/// The linear and angular velocity of `id` as bits.
fn velocity_bits(world: &PhysicsWorld, id: BodyId) -> [u32; 6] {
    let body = world.body(id).unwrap();
    let (v, w) = (body.linear_velocity(), body.angular_velocity());
    [v.x, v.y, v.z, w.x, w.y, w.z].map(f32::to_bits)
}

#[test]
fn a_light_box_floats_at_its_density_line() {
    let mut world = world(GRAVITY, 1);
    // Half extents 0.5, 0.25, 0.5: 0.5 m³. Four times lighter than the water, a quarter of its
    // 0.5 m height sinks: the centre rests 0.25 - 0.125 = 0.125 m above the surface. Quadratic
    // drag alone barely damps slow bobbing, so the body has linear damping too.
    let id = world
        .create_body(
            &Shape::new_box(Vec3::new(0.5, 0.25, 0.5)).unwrap(),
            &BodySettings::new_dynamic()
                .position(RVec3::new(0.0, 0.5, 0.0))
                .linear_damping(1.0),
        )
        .unwrap();
    let settings = water().buoyancy(4.0).linear_drag(2.0).angular_drag(0.05);
    float(&mut world, id, &settings, 600);
    let body = world.body(id).unwrap();
    assert!(
        (body.position().y - 0.125).abs() < 0.01,
        "{:?}",
        body.position()
    );
    assert!(speed(&body) < 0.01, "{}", speed(&body));
}

#[test]
fn a_sphere_twice_as_light_floats_half_under() {
    let mut world = world(GRAVITY, 1);
    let id = world
        .create_body(
            &Shape::new_sphere(0.5).unwrap(),
            &BodySettings::new_dynamic()
                .position(RVec3::new(0.0, 0.5, 0.0))
                .linear_damping(1.0),
        )
        .unwrap();
    float(&mut world, id, &water().buoyancy(2.0).linear_drag(2.0), 600);
    let y = world.body(id).unwrap().position().y;
    assert!(y.abs() < 0.01, "{y}");
}

#[test]
fn a_dense_box_sinks_to_the_floor() {
    let mut world = world(GRAVITY, 1);
    world
        .create_body(
            &Shape::new_plane(UP, 3.0, 20.0).unwrap(),
            &BodySettings::new_static(),
        )
        .unwrap();
    let id = add_cube(&mut world, RVec3::new(0.0, 0.0, 0.0));
    float(&mut world, id, &water().buoyancy(0.5), 600);
    let y = world.body(id).unwrap().position().y;
    assert!((y + 2.5).abs() < 0.03, "{y}");
}

/// A world without gravity and a submerged unit cube moving at `velocity`, spinning at
/// `spin`, with no damping of its own.
fn submerged_cube(velocity: Vec3, spin: Vec3) -> (PhysicsWorld, BodyId) {
    let mut world = world(Vec3::ZERO, 1);
    let id = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_dynamic()
                .linear_velocity(velocity)
                .angular_velocity(spin)
                .linear_damping(0.0)
                .angular_damping(0.0),
        )
        .unwrap();
    (world, id)
}

/// The speed and spin of the submerged cube after `ticks` of `settings` without lift.
fn after_drag(settings: &BuoyancySettings, velocity: Vec3, spin: Vec3, ticks: usize) -> (f32, f32) {
    let (mut world, id) = submerged_cube(velocity, spin);
    let settings = settings.surface(RVec3::new(0.0, 10.0, 0.0), UP);
    for _ in 0..ticks {
        assert!(buoy(&mut world, id, &settings, Vec3::ZERO).unwrap());
        step(&mut world, 1);
    }
    let body = world.body(id).unwrap();
    (speed(&body), length(body.angular_velocity()))
}

#[test]
fn drag_slows_a_submerged_body() {
    let velocity = Vec3::new(5.0, 0.0, 0.0);
    let (free, _) = after_drag(&water().linear_drag(0.0), velocity, Vec3::ZERO, 60);
    let (dragged, _) = after_drag(&water().linear_drag(2.0), velocity, Vec3::ZERO, 60);
    assert_eq!(free, 5.0);
    assert!(dragged < 1.0, "{dragged}");

    let spin = Vec3::new(0.0, 5.0, 0.0);
    let (_, free) = after_drag(&water().angular_drag(0.0), Vec3::ZERO, spin, 60);
    let (_, dragged) = after_drag(&water().angular_drag(1.0), Vec3::ZERO, spin, 60);
    assert!((free - 5.0).abs() < 1.0e-5, "{free}");
    assert!(dragged < 4.0, "{dragged}");
}

#[test]
fn drag_never_reverses_a_body_in_one_call() {
    let (mut world, id) = submerged_cube(Vec3::new(5.0, 0.0, 0.0), Vec3::new(0.0, 3.0, 0.0));
    let settings = water_at(10.0).linear_drag(1.0e6).angular_drag(1.0e6);
    assert!(buoy(&mut world, id, &settings, Vec3::ZERO).unwrap());
    let body = world.body(id).unwrap();
    let (v, w) = (body.linear_velocity(), body.angular_velocity());
    // Stopped up to rounding, not reversed.
    assert!(v.x.abs() < 1.0e-5 && w.y.abs() < 1.0e-5, "{v:?} {w:?}");
}

#[test]
fn a_current_does_not_carry_a_body_at_rest() {
    let (mut world, id) = submerged_cube(Vec3::ZERO, Vec3::ZERO);
    let settings = water_at(10.0)
        .fluid_velocity(Vec3::new(3.0, 0.0, 1.0))
        .linear_drag(2.0);
    for _ in 0..10 {
        assert!(buoy(&mut world, id, &settings, Vec3::ZERO).unwrap());
        step(&mut world, 1);
    }
    let body = world.body(id).unwrap();
    assert_eq!(body.linear_velocity(), Vec3::ZERO);
    assert_eq!(body.position(), RVec3::ZERO);
}

#[test]
fn the_buoyant_lever_matches_the_geometry() {
    // A unit cube (half extent a = 0.5) cut through its centre by a surface at 45 degrees: the
    // submerged half is a prism whose centre lies at (-a/3, -a/3, 0). The lift
    // `b · m / 2 · g · dt` at that lever turns the cube by `-b · g · dt / (4 a)` about z.
    let (mut world, id) = submerged_cube(Vec3::ZERO, Vec3::ZERO);
    let s = std::f32::consts::FRAC_1_SQRT_2;
    let settings = water()
        .surface(RVec3::ZERO, Vec3::new(s, s, 0.0))
        .angular_drag(0.0);
    let gravity = Vec3::new(0.0, -10.0, 0.0);
    assert!(buoy(&mut world, id, &settings, gravity).unwrap());
    let body = world.body(id).unwrap();
    let (v, w) = (body.linear_velocity(), body.angular_velocity());
    let expected = 10.0 * DT / 2.0;
    assert!((v.y - expected).abs() < 1.0e-5 * expected, "{v:?}");
    assert!((w.z + expected).abs() < 1.0e-4 * expected, "{w:?}");
    assert!(w.x.abs() < 1.0e-6 && w.y.abs() < 1.0e-6, "{w:?}");
}

#[test]
fn a_fully_submerged_offset_body_turns() {
    // A sphere of radius 0.5 whose centre of mass is moved by (0.25, 0, 0): fully under water,
    // Jolt's centre of buoyancy is the sphere's centre, r = (-0.25, 0, 0) from the centre of
    // mass. The lift `J = b · m · g · dt` at that lever turns it about z by
    // `-0.25 · J / I_zz`, with `I_zz = (0.4 · 0.25 + 0.25²) · m`.
    let sphere = Shape::new_sphere(0.5).unwrap();
    let offset = Shape::new_offset_center_of_mass(&sphere, Vec3::new(0.25, 0.0, 0.0)).unwrap();
    let gravity = Vec3::new(0.0, -10.0, 0.0);
    let settings = water_at(10.0).linear_drag(0.0).angular_drag(0.0);
    let mut turns = Vec::new();
    for shape in [&sphere, &offset] {
        let mut world = world(Vec3::ZERO, 1);
        let id = world
            .create_body(shape, &BodySettings::new_dynamic())
            .unwrap();
        assert!(buoy(&mut world, id, &settings, gravity).unwrap());
        let body = world.body(id).unwrap();
        let (v, w) = (body.linear_velocity(), body.angular_velocity());
        let lift = 10.0 * DT;
        assert!((v.y - lift).abs() < 1.0e-5 * lift, "{v:?}");
        assert!(w.x.abs() < 1.0e-6 && w.y.abs() < 1.0e-6, "{w:?}");
        turns.push(w.z);
    }
    let expected = -0.25 * 10.0 * DT / (0.4 * 0.25 + 0.25 * 0.25);
    assert!(turns[0].abs() < 1.0e-6, "{turns:?}");
    assert!(
        (turns[1] - expected).abs() < 1.0e-4 * expected.abs(),
        "{turns:?}"
    );
}

#[test]
fn velocities_are_clamped_right_after_the_call() {
    // A small cube fully under water with a buoyant velocity change just below the policy.
    let mut world = world(Vec3::ZERO, 1);
    let small = Shape::new_box(Vec3::new(0.1, 0.1, 0.1)).unwrap();
    let rising = world
        .create_body(&small, &BodySettings::new_dynamic())
        .unwrap();
    let factor = 999.0 / (9.81 * DT);
    let deep = water_at(10.0).buoyancy(factor).linear_drag(0.0);
    assert!(buoy(&mut world, rising, &deep, GRAVITY).unwrap());
    let v = world.body(rising).unwrap().linear_velocity();
    assert!(length(v) <= limits::MAX_LINEAR_VELOCITY, "{v:?}");
    assert!(length(v) > 0.99 * limits::MAX_LINEAR_VELOCITY, "{v:?}");

    // A 10 cm rod of 10 g falling at 20 m/s, tilted so that only its lower end is in the water:
    // the drag on that end, 4 cm from the centre of mass, would spin it at several hundred rad/s,
    // and the clamp brings it back.
    let rod = Shape::new_box(Vec3::new(0.05, 0.005, 0.005)).unwrap();
    let falling = world
        .create_body(
            &rod,
            &BodySettings::new_dynamic()
                .position(RVec3::new(3.0, 0.0, 0.0))
                .rotation(quat_about(Vec3::new(0.0, 0.0, 1.0), 0.5))
                .linear_velocity(Vec3::new(0.0, -20.0, 0.0)),
        )
        .unwrap();
    let surface = water_at(-0.018).buoyancy(1.5).linear_drag(0.5);
    assert!(buoy(&mut world, falling, &surface, GRAVITY).unwrap());
    let w = world.body(falling).unwrap().angular_velocity();
    assert!(length(w) <= limits::MAX_ANGULAR_VELOCITY * 1.0001, "{w:?}");
    assert!(length(w) > 0.99 * limits::MAX_ANGULAR_VELOCITY, "{w:?}");
    step(&mut world, 1);
}

#[test]
fn locked_axes_stay_still() {
    let mut world = world(Vec3::ZERO, 1);
    let id = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_dynamic()
                .rotation(quat_about(Vec3::new(1.0, 0.0, 0.0), 0.4))
                .allowed_dofs(AllowedDofs::PLANE_2D),
        )
        .unwrap();
    let settings = water_at(0.1).fluid_velocity(Vec3::new(1.0, 0.0, 3.0));
    assert!(buoy(&mut world, id, &settings, GRAVITY).unwrap());
    let body = world.body(id).unwrap();
    let (v, w) = (body.linear_velocity(), body.angular_velocity());
    assert_eq!((v.z, w.x, w.y), (0.0, 0.0, 0.0), "{v:?} {w:?}");
    assert!(v.y > 0.0, "{v:?}");
}

#[test]
fn only_submerged_dynamic_bodies_are_pushed() {
    let mut world = world(GRAVITY, 1);
    world.set_event_settings(EventSettings::default().body_activation(true));
    let above = add_cube(&mut world, RVec3::new(0.0, 5.0, 0.0));
    let fixed = world
        .create_body(&cube_shape(), &BodySettings::new_static())
        .unwrap();
    let moved = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_kinematic().position(RVec3::new(3.0, 0.0, 0.0)),
        )
        .unwrap();
    let cloth = add_cloth(&mut world, RVec3::new(-5.0, -1.0, 0.0), Quat::IDENTITY);
    let surface = water_at(1.0);
    for id in [above, fixed, moved] {
        let before = velocity_bits(&world, id);
        assert_eq!(buoy(&mut world, id, &surface, GRAVITY), Ok(false));
        assert_eq!(velocity_bits(&world, id), before);
    }
    assert_eq!(
        buoy(&mut world, cloth, &surface, GRAVITY),
        Err(BodyError::SoftBody(cloth))
    );
    world.take_events();

    // A sleeping body under water wakes, also when the factors make the impulse zero.
    for buoyancy in [1.5, 0.0] {
        world.take_events();
        let sleeper = world
            .create_body(
                &cube_shape(),
                &BodySettings::new_dynamic()
                    .position(RVec3::new(10.0, -2.0, 0.0))
                    .activation(Activation::DontActivate),
            )
            .unwrap();
        let still = water_at(1.0)
            .buoyancy(buoyancy)
            .linear_drag(0.0)
            .angular_drag(0.0);
        assert_eq!(buoy(&mut world, sleeper, &still, Vec3::ZERO), Ok(true));
        assert!(!world.body(sleeper).unwrap().is_sleeping());
        assert_eq!(
            world.take_events().activations,
            [ActivationEvent::Activated(sleeper)]
        );
        world.remove_body(sleeper).unwrap();
    }
    world.take_events();
    // A dry sleeping body stays asleep.
    let dry = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_dynamic()
                .position(RVec3::new(10.0, 5.0, 0.0))
                .activation(Activation::DontActivate),
        )
        .unwrap();
    assert_eq!(buoy(&mut world, dry, &surface, GRAVITY), Ok(false));
    assert!(world.body(dry).unwrap().is_sleeping());
    assert!(world.take_events().activations.is_empty());
}

#[test]
fn invalid_inputs_change_nothing() {
    let mut world = world(GRAVITY, 1);
    let id = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_dynamic()
                .linear_velocity(Vec3::new(1.0, 2.0, 3.0))
                .activation(Activation::DontActivate),
        )
        .unwrap();
    let saved = world.save_state();
    let before = velocity_bits(&world, id);
    let base = water_at(10.0);
    let bad_settings = [
        base.surface(RVec3::new(0.0, Real::NAN, 0.0), UP),
        base.surface(RVec3::new(limits::MAX_POSITION * 2.0, 0.0, 0.0), UP),
        base.surface(RVec3::ZERO, Vec3::new(0.0, 2.0, 0.0)),
        base.surface(RVec3::ZERO, Vec3::ZERO),
        base.buoyancy(-1.0),
        base.buoyancy(f32::NAN),
        base.buoyancy(f32::INFINITY),
        base.linear_drag(-0.1),
        base.angular_drag(f32::NAN),
        base.fluid_velocity(Vec3::new(limits::MAX_LINEAR_VELOCITY * 1.01, 0.0, 0.0)),
        base.fluid_velocity(Vec3::new(f32::NAN, 0.0, 0.0)),
    ];
    for settings in &bad_settings {
        assert!(
            matches!(
                buoy(&mut world, id, settings, GRAVITY),
                Err(BodyError::InvalidValue(_))
            ),
            "{settings:?}"
        );
    }
    let mut body = world.body_mut(id).unwrap();
    for gravity in [
        Vec3::new(0.0, f32::NAN, 0.0),
        Vec3::new(0.0, -limits::MAX_ACCELERATION * 1.01, 0.0),
    ] {
        assert!(matches!(
            body.apply_buoyancy_impulse(&base, gravity, DT),
            Err(BodyError::InvalidValue(_))
        ));
    }
    for delta_time in [0.0, -DT, f32::NAN, 2.0, PhysicsWorld::MIN_DELTA_TIME / 2.0] {
        assert!(matches!(
            body.apply_buoyancy_impulse(&base, GRAVITY, delta_time),
            Err(BodyError::InvalidValue(_))
        ));
    }
    // Policy: fully under water a buoyant velocity change of 1001 m/s is refused.
    let factor = 1001.0 / (9.81 * DT);
    assert!(matches!(
        body.apply_buoyancy_impulse(&base.buoyancy(factor), GRAVITY, DT),
        Err(BodyError::InvalidValue(_))
    ));
    assert_eq!(velocity_bits(&world, id), before);
    assert!(world.body(id).unwrap().is_sleeping());
    world.restore_state(&saved).unwrap();

    // The same factor half under water changes the velocity by 500.5 m/s and is accepted.
    let half = water_at(0.0).buoyancy(factor);
    assert_eq!(buoy(&mut world, id, &half, GRAVITY), Ok(true));
    // Buoyancy changes only state Jolt saves: an earlier snapshot still restores.
    world.restore_state(&saved).unwrap();
}

#[test]
fn bodies_at_the_bounds_stay_finite() {
    let mut world = world(Vec3::ZERO, 1);
    let finite_after = |world: &mut PhysicsWorld,
                        id: BodyId,
                        settings: &BuoyancySettings,
                        gravity: Vec3,
                        delta_time: f32| {
        let result = world
            .body_mut(id)
            .unwrap()
            .apply_buoyancy_impulse(settings, gravity, delta_time);
        assert!(result.is_ok(), "{result:?}");
        step(world, 1);
        let body = world.body(id).unwrap();
        assert!(body.linear_velocity().all_finite() && body.angular_velocity().all_finite());
        assert!(length(body.linear_velocity()) <= limits::MAX_LINEAR_VELOCITY);
    };
    let half = water_at(0.0);
    for gravity_factor in [1000.0, -1000.0, 0.0] {
        let id = world
            .create_body(
                &cube_shape(),
                &BodySettings::new_dynamic().gravity_factor(gravity_factor),
            )
            .unwrap();
        finite_after(&mut world, id, &half.buoyancy(0.1), GRAVITY, DT);
        world.remove_body(id).unwrap();
    }
    let id = add_cube(&mut world, RVec3::ZERO);
    finite_after(&mut world, id, &half, Vec3::ZERO, DT);
    finite_after(&mut world, id, &half, GRAVITY, PhysicsWorld::MIN_DELTA_TIME);
    finite_after(&mut world, id, &half, GRAVITY, PhysicsWorld::MAX_DELTA_TIME);
    // The largest gravity, with a buoyancy small enough for the velocity change bound.
    let strongest = Vec3::new(0.0, -limits::MAX_ACCELERATION, 0.0);
    finite_after(
        &mut world,
        id,
        &half.buoyancy(1.0e-7),
        strongest,
        PhysicsWorld::MIN_DELTA_TIME,
    );
    let current = Vec3::new(limits::MAX_LINEAR_VELOCITY, 0.0, 0.0);
    finite_after(
        &mut world,
        id,
        &half.fluid_velocity(current).linear_drag(1.0e6),
        GRAVITY,
        DT,
    );
    world.remove_body(id).unwrap();

    // The largest drag the rules accept for a fast body in a fast current.
    let fast = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_dynamic().linear_velocity(Vec3::new(
                -limits::MAX_LINEAR_VELOCITY,
                0.0,
                0.0,
            )),
        )
        .unwrap();
    let flow = half.fluid_velocity(current);
    let accepts = |world: &mut PhysicsWorld, drag: f32| {
        let saved = world.save_state();
        let ok = world
            .body_mut(fast)
            .unwrap()
            .apply_buoyancy_impulse(&flow.linear_drag(drag).angular_drag(drag), GRAVITY, DT)
            .is_ok();
        world.restore_state(&saved).unwrap();
        ok
    };
    let (mut low, mut high) = (1.0_f32, f32::MAX);
    assert!(accepts(&mut world, low) && !accepts(&mut world, high));
    for _ in 0..200 {
        let middle = (low * 0.5 + high * 0.5).max(low.next_up());
        if middle >= high {
            break;
        }
        if accepts(&mut world, middle) {
            low = middle;
        } else {
            high = middle;
        }
    }
    finite_after(
        &mut world,
        fast,
        &flow.linear_drag(low).angular_drag(low),
        GRAVITY,
        DT,
    );
}

#[test]
fn small_light_and_far_buoyancy_centres_stay_finite() {
    let mut world = world(GRAVITY, 1);
    // A tiny shape given the largest mass: its density is out of reach for any buoyancy.
    let speck = world
        .create_body(
            &Shape::new_box(Vec3::new(0.001, 0.001, 0.001)).unwrap(),
            &BodySettings::new_dynamic().mass(limits::MAX_MASS),
        )
        .unwrap();
    assert!(matches!(
        buoy(
            &mut world,
            speck,
            &water_at(1.0).buoyancy(1.0e23),
            Vec3::ZERO
        ),
        Err(BodyError::InvalidValue(_))
    ));
    // A 1 g, 6 cm cube near Jolt's unit-sphere inertia fallback, half under water and spinning.
    let light = world
        .create_body(
            &Shape::new_box(Vec3::new(0.03, 0.03, 0.03)).unwrap(),
            &BodySettings::new_dynamic()
                .position(RVec3::new(2.0, 0.0, 0.0))
                .mass(limits::MIN_MASS)
                .rotation(quat_about(Vec3::new(0.0, 0.6, 0.8), 0.9))
                .angular_velocity(Vec3::new(10.0, -20.0, 5.0))
                .linear_velocity(Vec3::new(0.0, -50.0, 0.0)),
        )
        .unwrap();
    // A thin plate.
    let plate = world
        .create_body(
            &Shape::new_box(Vec3::new(2.0, 0.005, 1.0)).unwrap(),
            &BodySettings::new_dynamic()
                .position(RVec3::new(-4.0, 0.0, 0.0))
                .rotation(quat_about(Vec3::new(1.0, 0.0, 0.0), 0.2)),
        )
        .unwrap();
    // A scaled compound whose submerged part is far from its centre of mass.
    let cube = cube_shape();
    let inner = Shape::new_compound(&[
        child(&cube, Vec3::new(-6.0, 0.0, 0.0), 1),
        child(&cube, Vec3::new(6.0, 4.0, 0.0), 2),
    ])
    .unwrap();
    let nested = Shape::new_compound(&[
        child(&inner, Vec3::ZERO, 1),
        child(&cube, Vec3::new(0.0, 8.0, 0.0), 2),
    ])
    .unwrap();
    let scaled = Shape::scaled(&nested, Vec3::new(1.5, -1.0, 1.0)).unwrap();
    let long = world
        .create_body(
            &scaled,
            &BodySettings::new_dynamic().position(RVec3::new(0.0, 6.0, 20.0)),
        )
        .unwrap();
    let surface = water_at(0.0)
        .buoyancy(1.2)
        .linear_drag(1.0)
        .angular_drag(0.5);
    for id in [light, plate, long] {
        assert!(buoy(&mut world, id, &surface, GRAVITY).unwrap(), "{id:?}");
    }
    step(&mut world, 1);
    for id in [light, plate, long] {
        let body = world.body(id).unwrap();
        assert!(body.linear_velocity().all_finite() && body.angular_velocity().all_finite());
    }

    // A waterline grazing a face: the surface within 1e-6 m of the cube's bottom.
    for offset in [-1.0e-6, 0.0, 1.0e-6] {
        let id = add_cube(&mut world, RVec3::new(30.0, 0.5, 0.0));
        let grazing = water_at(Real::from(offset as f32)).buoyancy(3.0);
        assert!(buoy(&mut world, id, &grazing, GRAVITY).is_ok());
        let v = world.body(id).unwrap().linear_velocity();
        assert!(v.all_finite() && length(v) < 1.0, "{offset}: {v:?}");
        world.remove_body(id).unwrap();
    }
}

/// SplitMix64, for seeded cases.
struct SplitMix64(u64);

impl SplitMix64 {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    fn unit(&mut self) -> f32 {
        (self.next() >> 40) as f32 / (1_u64 << 24) as f32
    }

    fn between(&mut self, low: f32, high: f32) -> f32 {
        low + (high - low) * self.unit()
    }

    fn vector(&mut self, scale: f32) -> Vec3 {
        Vec3::new(
            self.between(-scale, scale),
            self.between(-scale, scale),
            self.between(-scale, scale),
        )
    }

    fn rotation(&mut self) -> Quat {
        let axis = self.vector(1.0);
        let len = length(axis).max(1.0e-3);
        quat_about(
            Vec3::new(axis.x / len, axis.y / len, axis.z / len),
            self.between(0.0, std::f32::consts::TAU),
        )
    }
}

fn stress_shapes() -> Vec<Shape> {
    let cube = cube_shape();
    let ball = Shape::new_sphere(0.4).unwrap();
    let compound = Shape::new_compound(&[
        child(&cube, Vec3::ZERO, 0),
        child(&ball, Vec3::new(0.8, 0.3, 0.0), 1),
    ])
    .unwrap();
    vec![
        Shape::new_box(Vec3::new(0.6, 0.2, 0.3)).unwrap(),
        Shape::new_sphere(0.3).unwrap(),
        Shape::new_capsule(0.4, 0.2).unwrap(),
        Shape::new_cylinder(0.3, 0.25).unwrap(),
        Shape::new_convex_hull(&common::meshes::irregular_points(), 0.02).unwrap(),
        Shape::new_offset_center_of_mass(&compound, Vec3::new(0.2, -0.1, 0.0)).unwrap(),
        compound,
    ]
}

#[test]
fn seeded_calls_are_accepted_or_refused_and_stay_finite() {
    let shapes = stress_shapes();
    let mut rng = SplitMix64(0xB0A7_5EED);
    let (mut accepted, mut refused) = (0, 0);
    let mut world = world(GRAVITY, 1);
    for case in 0..500_usize {
        let gravity_factor = match case % 10 {
            0 => 0.0,
            _ => rng.between(-1000.0, 1000.0),
        };
        let still = case.is_multiple_of(7);
        let id = world
            .create_body(
                &shapes[case % shapes.len()],
                &BodySettings::new_dynamic()
                    .position(RVec3::new(0.0, Real::from(rng.between(-0.6, 0.6)), 0.0))
                    .rotation(rng.rotation())
                    .gravity_factor(gravity_factor)
                    .linear_velocity(if still { Vec3::ZERO } else { rng.vector(250.0) })
                    .angular_velocity(if still { Vec3::ZERO } else { rng.vector(25.0) }),
            )
            .unwrap();
        let huge = case.is_multiple_of(13);
        let settings = water()
            .buoyancy(rng.between(0.0, 10.0))
            .linear_drag(if huge {
                1.0e30
            } else {
                rng.between(0.0, 100.0)
            })
            .angular_drag(if huge {
                1.0e30
            } else {
                rng.between(0.0, 100.0)
            })
            .fluid_velocity(if still { Vec3::ZERO } else { rng.vector(250.0) });
        let gravity = match case % 11 {
            0 => Vec3::ZERO,
            _ => rng.vector(100.0),
        };
        let gravity = if length(gravity) > 100.0 {
            GRAVITY
        } else {
            gravity
        };
        let delta_time = match case % 5 {
            0 => PhysicsWorld::MIN_DELTA_TIME,
            1 => PhysicsWorld::MAX_DELTA_TIME,
            _ => DT,
        };
        let before = velocity_bits(&world, id);
        match world
            .body_mut(id)
            .unwrap()
            .apply_buoyancy_impulse(&settings, gravity, delta_time)
        {
            Ok(_) => accepted += 1,
            Err(BodyError::InvalidValue(_)) => {
                refused += 1;
                assert_eq!(velocity_bits(&world, id), before, "case {case}");
            }
            Err(error) => panic!("case {case}: {error}"),
        }
        for when in ["after the call", "after a step"] {
            let body = world.body(id).unwrap();
            let (v, w) = (body.linear_velocity(), body.angular_velocity());
            assert!(
                v.all_finite() && w.all_finite(),
                "case {case} {when}: {v:?} {w:?}"
            );
            assert!(
                length(v) <= limits::MAX_LINEAR_VELOCITY * 1.0001,
                "case {case} {when}"
            );
            assert!(
                length(w) <= limits::MAX_ANGULAR_VELOCITY * 1.0001,
                "case {case} {when}"
            );
            if when == "after the call" {
                step(&mut world, 1);
            }
        }
        world.remove_body(id).unwrap();
    }
    assert!(accepted > 100 && refused > 10, "{accepted} {refused}");
}
