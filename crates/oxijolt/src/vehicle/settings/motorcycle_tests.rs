use super::super::{VehicleCollisionTester, VehicleDifferentialSettings, WheelSettings};
use super::*;
use crate::world::ensure_initialized;
use crate::{
    limits, BodySettings, MotorcycleSettings, ObjectLayer, PhysicsWorld, RVec3, Shape,
    WorldSettings,
};

const LAYERS: u32 = 2;

/// A valid motorcycle: front wheel at +Z, rear wheel at −Z, rear-wheel drive, a ray tester on
/// layer 1.
fn bike() -> VehicleSettings {
    let wheel = |z: f32| WheelSettings::new(Vec3::new(0.0, -0.27, z)).radius(0.31);
    VehicleSettings::new(
        vec![wheel(0.75), wheel(-0.75).max_steer_angle(0.0)],
        vec![VehicleDifferentialSettings::new(None, Some(1))],
        VehicleCollisionTester::ray(ObjectLayer::new(1)),
    )
}

fn with_wheels(wheels: Vec<WheelSettings>) -> MotorcycleSettings {
    let mut vehicle = bike();
    vehicle.wheels = wheels;
    vehicle.differentials = vec![VehicleDifferentialSettings::new(None, Some(0))];
    MotorcycleSettings::new(vehicle)
}

#[track_caller]
fn assert_refused(settings: MotorcycleSettings, rule: &str) {
    match settings.validate(LAYERS) {
        Err(VehicleError::InvalidValue(what)) => {
            assert!(
                what.contains(rule),
                "refused for {what:?}, expected {rule:?}"
            )
        }
        other => panic!("expected a refusal for {rule:?}, got {other:?}"),
    }
}

/// A world with a 240 kg box chassis and the motorcycle of `settings` attached through the
/// shared attach path alone, without the motorcycle's switches.
fn attached(settings: &MotorcycleSettings) -> (PhysicsWorld, *mut JPH_MotorcycleController) {
    let mut world = PhysicsWorld::new(WorldSettings::default()).unwrap();
    let shape = Shape::new_box(Vec3::new(0.2, 0.3, 0.4)).unwrap();
    let chassis = world
        .create_body(
            &shape,
            &BodySettings::new_dynamic()
                .position(RVec3::new(0.0, 1.0, 0.0))
                .mass(240.0),
        )
        .unwrap();
    let id = world.attach_vehicle(chassis, settings.build()).unwrap();
    let constraint = world.vehicles[&id.raw].constraint.as_ptr();
    // SAFETY: the world owns the constraint; the getter returns a member, a motorcycle
    // controller, which derives from `VehicleController` with single inheritance.
    let controller = unsafe { JPH_VehicleConstraint_GetController(constraint) }.cast();
    (world, controller)
}

#[test]
fn motorcycle_defaults_are_jolts() {
    assert!(ensure_initialized());
    let ours = MotorcycleSettings::new(bike());
    // SAFETY: Jolt is initialised; the guard owns the one reference joltc returns.
    let jolt = unsafe { Owned::from_raw(JPH_MotorcycleControllerSettings_Create()) }.unwrap();
    let ptr = jolt.as_ptr();
    // SAFETY: the settings are live and only read.
    unsafe {
        assert_eq!(
            JPH_MotorcycleControllerSettings_GetMaxLeanAngle(ptr),
            ours.max_lean_angle
        );
        assert_eq!(
            JPH_MotorcycleControllerSettings_GetLeanSpringConstant(ptr),
            ours.lean_spring_constant
        );
        assert_eq!(
            JPH_MotorcycleControllerSettings_GetLeanSpringDamping(ptr),
            ours.lean_spring_damping
        );
        assert_eq!(
            JPH_MotorcycleControllerSettings_GetLeanSpringIntegrationCoefficient(ptr),
            ours.lean_spring_integration_coefficient
        );
        assert_eq!(
            JPH_MotorcycleControllerSettings_GetLeanSmoothingFactor(ptr),
            ours.lean_smoothing_factor
        );
    }
    // Jolt's controller starts with both switches on, as the settings do.
    let (world, controller) = attached(&ours);
    // SAFETY: `world` keeps the controller alive; the getters read members.
    unsafe {
        assert_eq!(
            JPH_MotorcycleController_IsLeanControllerEnabled(controller),
            ours.lean_controller
        );
        assert_eq!(
            JPH_MotorcycleController_IsLeanSteeringLimitEnabled(controller),
            ours.lean_steering_limit
        );
    }
    drop(world);
}

#[test]
fn built_lean_settings_reach_jolt() {
    assert!(ensure_initialized());
    let settings = MotorcycleSettings::new(bike())
        .max_lean_angle(0.5)
        .lean_spring_constant(4000.0)
        .lean_spring_damping(800.0)
        .lean_smoothing_factor(0.6);
    let (world, controller) = attached(&settings);
    // SAFETY: `world` keeps the controller alive; the getters read members.
    unsafe {
        assert_eq!(
            JPH_MotorcycleController_GetLeanSpringConstant(controller),
            4000.0
        );
        assert_eq!(
            JPH_MotorcycleController_GetLeanSpringDamping(controller),
            800.0
        );
        assert_eq!(
            JPH_MotorcycleController_GetLeanSpringIntegrationCoefficient(controller),
            0.0
        );
        assert_eq!(
            JPH_MotorcycleController_GetLeanSpringIntegrationCoefficientDecay(controller),
            4.0
        );
        assert_eq!(
            JPH_MotorcycleController_GetLeanSmoothingFactor(controller),
            0.6
        );
        assert_eq!(JPH_MotorcycleController_GetWheelBase(controller), 1.5);
    }
    drop(world);

    let mut world = PhysicsWorld::new(WorldSettings::default()).unwrap();
    let shape = Shape::new_box(Vec3::new(0.2, 0.3, 0.4)).unwrap();
    let chassis = world
        .create_body(&shape, &BodySettings::new_dynamic().mass(240.0))
        .unwrap();
    let off = settings.lean_controller(false).lean_steering_limit(false);
    let bike = world.create_motorcycle(chassis, &off).unwrap();
    let vehicle = world.vehicle(bike).unwrap();
    assert!(!vehicle.is_lean_controller_enabled());
    assert!(!vehicle.is_lean_steering_limit_enabled());
}

#[test]
fn motorcycle_settings_are_validated() {
    assert_eq!(MotorcycleSettings::new(bike()).validate(LAYERS), Ok(()));
    let wheel = |z: f32| WheelSettings::new(Vec3::new(0.0, -0.27, z));

    for count in [1, 3] {
        let wheels = (0..count).map(|i| wheel(i as f32)).collect();
        assert_refused(with_wheels(wheels), "exactly two wheels");
    }
    // No wheels at all fails the vehicle's own rule first.
    assert_refused(with_wheels(Vec::new()), "at least one wheel");

    // Side by side, the wheels have no base along the forward.
    assert_refused(
        with_wheels(vec![
            WheelSettings::new(Vec3::new(0.3, -0.27, 0.0)),
            WheelSettings::new(Vec3::new(-0.3, -0.27, 0.0)),
        ]),
        "apart along its forward",
    );
    // Force points decide where Jolt measures, not the attachment points.
    let force_points = |front: f32, rear: f32| {
        with_wheels(vec![
            wheel(0.75).suspension_force_point(Some(Vec3::new(0.0, -0.3, front))),
            wheel(-0.75).suspension_force_point(Some(Vec3::new(0.0, -0.3, rear))),
        ])
    };
    assert_refused(force_points(0.1, 0.1), "apart along its forward");
    assert_eq!(force_points(0.1, -0.1).validate(LAYERS), Ok(()));
    // Coincident attachment points, with the front suspension raked forward: apart.
    let raked = Vec3::new(0.0, -0.8, 0.6);
    assert_eq!(
        with_wheels(vec![wheel(0.0).suspension_direction(raked), wheel(0.0)]).validate(LAYERS),
        Ok(())
    );
    // A forward of +X: wheels apart along Z are side by side.
    let mut sideways = MotorcycleSettings::new(bike().forward(Vec3::new(1.0, 0.0, 0.0)));
    assert_refused(sideways.clone(), "apart along its forward");
    sideways.vehicle.wheels = vec![
        WheelSettings::new(Vec3::new(0.75, -0.27, 0.0)),
        WheelSettings::new(Vec3::new(-0.75, -0.27, 0.0)),
    ];
    assert_eq!(sideways.validate(LAYERS), Ok(()));
    // An up of +Z with forward +Y.
    let mut upright = bike()
        .up(Vec3::new(0.0, 0.0, 1.0))
        .forward(Vec3::new(0.0, 1.0, 0.0));
    upright.wheels = vec![
        WheelSettings::new(Vec3::new(0.0, 0.75, -0.27))
            .suspension_direction(Vec3::new(0.0, 0.0, -1.0)),
        WheelSettings::new(Vec3::new(0.0, -0.75, -0.27))
            .suspension_direction(Vec3::new(0.0, 0.0, -1.0)),
    ];
    assert_eq!(MotorcycleSettings::new(upright).validate(LAYERS), Ok(()));

    let settings = || MotorcycleSettings::new(bike());
    for angle in [
        -0.01,
        MotorcycleSettings::MAX_LEAN_ANGLE.next_up(),
        f32::NAN,
        f32::INFINITY,
    ] {
        assert_refused(settings().max_lean_angle(angle), "max lean angle");
    }
    for angle in [0.0, MotorcycleSettings::MAX_LEAN_ANGLE] {
        assert_eq!(settings().max_lean_angle(angle).validate(LAYERS), Ok(()));
    }
    for value in [-1.0, f32::NAN, f32::INFINITY] {
        assert_refused(settings().lean_spring_constant(value), "lean spring");
        assert_refused(settings().lean_spring_damping(value), "lean spring");
    }
    for factor in [-0.1, 1.1, f32::NAN] {
        assert_refused(settings().lean_smoothing_factor(factor), "smoothing");
    }
    for factor in [0.0, 1.0] {
        assert_eq!(
            settings().lean_smoothing_factor(factor).validate(LAYERS),
            Ok(())
        );
    }
    for coefficient in [1.0e-3, -1.0e-3, f32::MIN_POSITIVE, f32::NAN, f32::INFINITY] {
        assert_eq!(
            settings()
                .lean_spring_integration_coefficient(coefficient)
                .validate(LAYERS),
            Err(VehicleError::LeanSpringIntegrationNotSaved),
            "{coefficient}"
        );
    }
    // The vehicle's rules still apply.
    assert_refused(
        MotorcycleSettings::new(bike().max_pitch_roll_angle(-0.1)),
        "pitch roll",
    );
}

#[test]
fn max_lean_angle_is_80_degrees() {
    let exact = 80.0_f32.to_radians();
    let ulps = (MotorcycleSettings::MAX_LEAN_ANGLE.to_bits() as i64 - exact.to_bits() as i64).abs();
    assert!(
        ulps <= 1,
        "{} vs {exact}",
        MotorcycleSettings::MAX_LEAN_ANGLE
    );
}

#[test]
fn lean_spring_rule_boundary() {
    let inverse_inertia = 0.05;
    let damping = 1000.0;
    // The bound in f64, from the rule's own formula.
    let headroom = 1.0 + 4.0 * f64::from(crate::math::UNIT_TOLERANCE);
    let bound = (f64::from(limits::MAX_ANGULAR_ACCELERATION)
        / (f64::from(inverse_inertia) * headroom)
        - f64::from(damping) * f64::from(limits::MAX_ANGULAR_VELOCITY_CHANGE))
        / std::f64::consts::PI;
    let mut constant = bound as f32;
    while !limits::is_lean_spring(constant, damping, inverse_inertia) {
        constant = constant.next_down();
    }
    assert!(f64::from(constant) <= bound);
    assert!((bound - f64::from(constant)) / bound < 1e-6);
    assert!(!limits::is_lean_spring(
        constant.next_up(),
        damping,
        inverse_inertia
    ));
    // Jolt's default springs fit the fixture chassis and are refused above about 430 1/(kg·m²).
    assert!(limits::is_lean_spring(5000.0, 1000.0, 0.1));
    assert!(limits::is_lean_spring(5000.0, 1000.0, 428.0));
    assert!(!limits::is_lean_spring(5000.0, 1000.0, 430.0));
    assert!(!limits::is_lean_spring(f32::MAX, 0.0, 1.0));
}
