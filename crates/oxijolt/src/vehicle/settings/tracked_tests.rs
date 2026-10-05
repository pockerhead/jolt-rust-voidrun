use super::super::tests::{bits3, curve_points, jolt_vec};
use super::*;
use crate::world::ensure_initialized;
use crate::ObjectLayer;

const LAYERS: u32 = 2;

/// A valid tank: three wheels per track, each track driven at its first wheel, a ray tester on
/// layer 1.
fn tank() -> TrackedVehicleSettings {
    let track = |x: f32| {
        let wheels = [1.0, 0.0, -1.0]
            .map(|z| TrackedWheelSettings::new(Vec3::new(x, -0.3, z)))
            .to_vec();
        VehicleTrackSettings::new(wheels, 0)
    };
    TrackedVehicleSettings::new(
        track(1.0),
        track(-1.0),
        VehicleCollisionTester::ray(ObjectLayer::new(1)),
    )
}

fn with_left(
    edit: impl Fn(VehicleTrackSettings) -> VehicleTrackSettings,
) -> TrackedVehicleSettings {
    let mut settings = tank();
    settings.left = edit(settings.left.clone());
    settings
}

fn with_wheel(
    edit: impl Fn(TrackedWheelSettings) -> TrackedWheelSettings,
) -> TrackedVehicleSettings {
    let mut settings = tank();
    settings.right.wheels[1] = edit(settings.right.wheels[1].clone());
    settings
}

#[track_caller]
fn assert_refused(settings: TrackedVehicleSettings, rule: &str) {
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

#[test]
fn tracked_wheel_defaults_are_jolts() {
    assert!(ensure_initialized());
    let ours = TrackedWheelSettings::new(Vec3::ZERO);
    // SAFETY: Jolt is initialised; the guard owns the one reference joltc returns.
    let jolt = unsafe { Owned::from_raw(JPH_WheelSettingsTV_Create()) }.unwrap();
    let tv = jolt.as_ptr();
    let base: *mut JPH_WheelSettings = tv.cast();
    // SAFETY: the settings are live and only read; every output is a live local.
    unsafe {
        let direction = jolt_vec(|v| JPH_WheelSettings_GetSuspensionDirection(base, v));
        assert_eq!(bits3(direction), bits3(ours.base.suspension_direction));
        let axis = jolt_vec(|v| JPH_WheelSettings_GetSteeringAxis(base, v));
        assert_eq!(bits3(axis), bits3(ours.base.steering_axis));
        let up = jolt_vec(|v| JPH_WheelSettings_GetWheelUp(base, v));
        assert_eq!(bits3(up), bits3(ours.base.wheel_up));
        let forward = jolt_vec(|v| JPH_WheelSettings_GetWheelForward(base, v));
        assert_eq!(bits3(forward), bits3(ours.base.wheel_forward));
        assert!(!JPH_WheelSettings_GetEnableSuspensionForcePoint(base));
        assert_eq!(
            JPH_WheelSettings_GetSuspensionMinLength(base),
            ours.base.suspension_min_length
        );
        assert_eq!(
            JPH_WheelSettings_GetSuspensionMaxLength(base),
            ours.base.suspension_max_length
        );
        assert_eq!(
            JPH_WheelSettings_GetSuspensionPreloadLength(base),
            ours.base.suspension_preload_length
        );
        let mut spring = JPH_SpringSettings {
            mode: JPH_SpringMode_StiffnessAndDamping,
            frequencyOrStiffness: 0.0,
            damping: 0.0,
        };
        JPH_WheelSettings_GetSuspensionSpring(base, &mut spring);
        let expected = ours.base.suspension_spring.to_jph();
        assert_eq!(spring.mode, expected.mode);
        assert_eq!(spring.frequencyOrStiffness, expected.frequencyOrStiffness);
        assert_eq!(spring.damping, expected.damping);
        assert_eq!(JPH_WheelSettings_GetRadius(base), ours.base.radius);
        assert_eq!(JPH_WheelSettings_GetWidth(base), ours.base.width);
        assert_eq!(
            JPH_WheelSettingsTV_GetLongitudinalFriction(tv),
            ours.longitudinal_friction
        );
        assert_eq!(
            JPH_WheelSettingsTV_GetLateralFriction(tv),
            ours.lateral_friction
        );
    }
}

#[test]
fn track_defaults_are_jolts() {
    let ours = VehicleTrackSettings::new(Vec::new(), 0);
    // SAFETY: an all-zero `JPH_VehicleTrackSettings` is valid (integers, floats and a null
    // pointer); `_Init` fills it with Jolt's defaults and allocates nothing.
    let jolt = unsafe {
        let mut jolt: JPH_VehicleTrackSettings = std::mem::zeroed();
        JPH_VehicleTrackSettings_Init(&mut jolt);
        jolt
    };
    assert_eq!(jolt.drivenWheel, ours.driven_wheel);
    assert_eq!(jolt.inertia, ours.inertia);
    assert_eq!(jolt.angularDamping, ours.angular_damping);
    assert_eq!(jolt.maxBrakeTorque, ours.max_brake_torque);
    assert_eq!(jolt.differentialRatio, ours.differential_ratio);
}

#[test]
fn tracked_controller_defaults_are_jolts() {
    assert!(ensure_initialized());
    let engine = TrackedVehicleSettings::default_engine();
    let transmission = TrackedVehicleSettings::default_transmission();
    // SAFETY: Jolt is initialised; the guard owns the one reference joltc returns.
    let jolt = unsafe { Owned::from_raw(JPH_TrackedVehicleControllerSettings_Create()) }.unwrap();
    let ptr = jolt.as_ptr();
    // SAFETY: the settings are live and only read. `GetEngine` lends the settings' own torque
    // curve and `GetTransmission` the settings' own transmission; both are read while the guard
    // keeps the settings alive and never destroyed here.
    unsafe {
        let mut jolt_engine: JPH_VehicleEngineSettings = std::mem::zeroed();
        JPH_TrackedVehicleControllerSettings_GetEngine(ptr, &mut jolt_engine);
        assert_eq!(jolt_engine.maxTorque, engine.max_torque);
        assert_eq!(jolt_engine.minRPM, engine.min_rpm);
        assert_eq!(jolt_engine.maxRPM, engine.max_rpm);
        assert_eq!(jolt_engine.inertia, engine.inertia);
        assert_eq!(jolt_engine.angularDamping, engine.angular_damping);
        assert_eq!(
            curve_points(jolt_engine.normalizedTorque),
            engine.normalized_torque
        );

        let jolt_transmission = JPH_TrackedVehicleControllerSettings_GetTransmission(ptr);
        assert_eq!(
            JPH_VehicleTransmissionSettings_GetMode(jolt_transmission),
            JPH_TransmissionMode_Auto
        );
        let forward: Vec<f32> =
            (0..JPH_VehicleTransmissionSettings_GetGearRatioCount(jolt_transmission))
                .map(|i| JPH_VehicleTransmissionSettings_GetGearRatio(jolt_transmission, i))
                .collect();
        assert_eq!(forward, transmission.gear_ratios);
        let reverse: Vec<f32> =
            (0..JPH_VehicleTransmissionSettings_GetReverseGearRatioCount(jolt_transmission))
                .map(|i| JPH_VehicleTransmissionSettings_GetReverseGearRatio(jolt_transmission, i))
                .collect();
        assert_eq!(reverse, transmission.reverse_gear_ratios);
        assert_eq!(
            JPH_VehicleTransmissionSettings_GetSwitchTime(jolt_transmission),
            transmission.switch_time
        );
        assert_eq!(
            JPH_VehicleTransmissionSettings_GetClutchReleaseTime(jolt_transmission),
            transmission.clutch_release_time
        );
        assert_eq!(
            JPH_VehicleTransmissionSettings_GetSwitchLatency(jolt_transmission),
            transmission.switch_latency
        );
        assert_eq!(
            JPH_VehicleTransmissionSettings_GetShiftUpRPM(jolt_transmission),
            transmission.shift_up_rpm
        );
        assert_eq!(
            JPH_VehicleTransmissionSettings_GetShiftDownRPM(jolt_transmission),
            transmission.shift_down_rpm
        );
        assert_eq!(
            JPH_VehicleTransmissionSettings_GetClutchStrength(jolt_transmission),
            transmission.clutch_strength
        );
    }
}

#[test]
fn built_tracked_wheels_reach_jolt() {
    assert!(ensure_initialized());
    let wheel = TrackedWheelSettings::new(Vec3::new(1.0, -0.3, 0.5))
        .radius(0.4)
        .longitudinal_friction(3.0)
        .lateral_friction(1.5);
    let jolt = wheel.create();
    let tv = jolt.as_ptr();
    // SAFETY: the settings are live and only read; outputs are live locals.
    unsafe {
        let position = jolt_vec(|v| JPH_WheelSettings_GetPosition(tv.cast(), v));
        assert_eq!(bits3(position), bits3(wheel.base.position));
        assert_eq!(JPH_WheelSettings_GetRadius(tv.cast()), 0.4);
        assert_eq!(JPH_WheelSettingsTV_GetLongitudinalFriction(tv), 3.0);
        assert_eq!(JPH_WheelSettingsTV_GetLateralFriction(tv), 1.5);
    }
    let built = tank().build();
    assert_eq!(built.wheels.len(), 6);
    assert_eq!(built.tracks, Some([0..3, 3..6]));
    assert!(matches!(built.controller, ControllerGuard::Tracked(_)));
}

#[test]
fn tracked_settings_are_validated() {
    assert!(ensure_initialized());
    assert_eq!(tank().validate(LAYERS), Ok(()));

    assert_refused(
        with_left(|t| VehicleTrackSettings::new(Vec::new(), 0).inertia(t.inertia)),
        "at least one wheel",
    );
    assert_refused(
        with_left(|t| VehicleTrackSettings {
            driven_wheel: 3,
            ..t
        }),
        "driven wheel",
    );
    assert_refused(
        with_left(|t| VehicleTrackSettings {
            driven_wheel: u32::MAX,
            ..t
        }),
        "driven wheel",
    );
    for inertia in [0.0, -1.0, f32::NAN, f32::INFINITY] {
        assert_refused(with_left(|t| t.inertia(inertia)), "track inertia");
    }
    // Positive and finite, but too small to synchronise the tracks without underflow.
    assert_refused(with_left(|t| t.inertia(1.0e-40)), "synchronise");
    assert_refused(with_left(|t| t.angular_damping(-0.1)), "angular damping");
    assert_refused(
        with_left(|t| t.max_brake_torque(f32::NAN)),
        "max brake torque",
    );
    for ratio in [0.0, -6.0, f32::INFINITY] {
        assert_refused(
            with_left(|t| t.differential_ratio(ratio)),
            "differential ratio",
        );
    }
    for friction in [-0.1, limits::MAX_FRICTION * 1.001, f32::NAN] {
        assert_refused(
            with_wheel(|w| w.longitudinal_friction(friction)),
            "friction",
        );
        assert_refused(with_wheel(|w| w.lateral_friction(friction)), "friction");
    }
    assert_eq!(
        with_wheel(|w| w
            .longitudinal_friction(limits::MAX_FRICTION)
            .lateral_friction(0.0))
        .validate(LAYERS),
        Ok(())
    );
    assert_refused(tank().up(Vec3::new(0.0, 2.0, 0.0)), "unit");
    assert_refused(tank().forward(Vec3::new(0.0, 0.6, 0.8)), "perpendicular");
    assert_refused(tank().max_pitch_roll_angle(-0.1), "pitch roll");
    assert_refused(
        tank().engine(TrackedVehicleSettings::default_engine().max_rpm(3000.0)),
        "shift up rpm",
    );
    assert_refused(
        tank().transmission(TrackedVehicleSettings::default_transmission().gear_ratios(vec![])),
        "gear ratios",
    );
    let mut far_layer = tank();
    far_layer.collision_tester = VehicleCollisionTester::ray(ObjectLayer::new(LAYERS));
    assert_refused(far_layer, "object layer");
}

#[test]
fn tracked_wheels_share_the_wheel_rules() {
    assert_refused(with_wheel(|w| w.radius(0.0)), "wheel radius");
    assert_refused(with_wheel(|w| w.width(-0.1)), "wheel width");
    assert_refused(
        with_wheel(|w| w.suspension_direction(Vec3::new(0.0, -1.1, 0.0))),
        "wheel directions",
    );
    assert_refused(
        with_wheel(|w| w.wheel_up(Vec3::new(0.0, 0.0, 1.0))),
        "perpendicular",
    );
    assert_refused(with_wheel(|w| w.suspension_min_length(-0.1)), "min length");
    assert_refused(with_wheel(|w| w.suspension_max_length(0.2)), "max length");
    assert_refused(with_wheel(|w| w.suspension_preload_length(-0.1)), "preload");
    assert_refused(
        with_wheel(|w| w.suspension_force_point(Some(Vec3::new(f32::NAN, 0.0, 0.0)))),
        "force point",
    );
    assert_refused(
        with_wheel(|w| {
            w.suspension_spring(SuspensionSpring::FrequencyAndDamping {
                frequency: 0.0,
                damping: 0.5,
            })
        }),
        "suspension spring",
    );
    let extent = limits::MAX_SHAPE_EXTENT;
    assert_refused(
        with_wheel(|_| TrackedWheelSettings::new(Vec3::new(extent * 1.001, 0.0, 0.0))),
        "wheel position",
    );
    // The cylinder tester needs positive widths, as for wheeled vehicles.
    let mut settings = with_wheel(|w| w.width(0.0));
    assert_eq!(settings.validate(LAYERS), Ok(()));
    settings.collision_tester = VehicleCollisionTester::cast_cylinder(ObjectLayer::new(1));
    assert_refused(settings, "cylinder tester");
}

#[test]
fn tracked_step_coefficients_follow_jolts_intermediates() {
    const STEP: &str = "step coefficient";
    let engine = TrackedVehicleSettings::default_engine;
    let transmission = TrackedVehicleSettings::default_transmission;
    // Valid one by one: a huge torque curve and max torque, a heavy engine, gear and
    // differential ratios of 100. The differential torque overflows.
    let counter_example = with_left(|t| t.inertia(1.0e6).differential_ratio(100.0))
        .engine(
            engine()
                .max_torque(1.0e20)
                .normalized_torque(vec![(0.0, 1.0e15)])
                .inertia(1.0e36),
        )
        .transmission(transmission().gear_ratios(vec![100.0]));
    assert_refused(counter_example, STEP);

    // The transmission torque: largest gear times the engine's largest torque.
    let torque = |value: f32| engine().max_torque(value);
    assert_eq!(tank().engine(torque(1.0e37)).validate(LAYERS), Ok(()));
    assert_refused(
        tank()
            .engine(torque(1.0e37))
            .transmission(transmission().gear_ratios(vec![100.0])),
        STEP,
    );
    // A reverse gear counts as much as a forward one.
    assert_refused(
        tank()
            .engine(torque(1.0e37))
            .transmission(transmission().reverse_gear_ratios(vec![-100.0])),
        STEP,
    );
    // The differential torque, then its impulse over the track inertia, of either track.
    assert_refused(
        with_left(|t| t.differential_ratio(1.0e5))
            .engine(torque(1.0e30))
            .transmission(transmission().gear_ratios(vec![1.0e4])),
        STEP,
    );
    let mut unequal = tank().engine(torque(1.0e36));
    unequal.right = unequal.right.clone().inertia(1.0e-3);
    assert_eq!(
        tank().engine(torque(1.0e36)).validate(LAYERS),
        Ok(()),
        "the left track alone is fine"
    );
    assert_refused(unequal, STEP);
    // The brake: its impulse over the inertia, the lock torque at the smallest step, and the
    // torque per wheel radius.
    assert_refused(with_left(|t| t.max_brake_torque(3.0e38).inertia(0.5)), STEP);
    assert_eq!(with_left(|t| t.inertia(1.0e30)).validate(LAYERS), Ok(()));
    assert_refused(with_left(|t| t.inertia(1.0e33)), STEP);
    let mut small_wheel = with_left(|t| t.max_brake_torque(1.0e36).inertia(1.0e30));
    assert_eq!(small_wheel.validate(LAYERS), Ok(()));
    small_wheel.left.wheels[2] = small_wheel.left.wheels[2].clone().radius(1.0e-3);
    assert_refused(small_wheel, STEP);
    // The speed limit at the engine's rpm: its divisor must not underflow, its value must not
    // overflow.
    assert_refused(
        tank().transmission(
            transmission()
                .gear_ratios(vec![1.0e-36])
                .reverse_gear_ratios(vec![-1.0e-36]),
        ),
        "underflows",
    );
    assert_eq!(
        tank()
            .transmission(transmission().gear_ratios(vec![1.0e-30, 1.0]))
            .validate(LAYERS),
        Ok(())
    );
    assert_refused(
        tank().transmission(transmission().gear_ratios(vec![1.0e-33, 1.0])),
        STEP,
    );
    // A wheel much smaller than the driven wheel turns too fast.
    let mut tiny = with_left(|t| t.max_brake_torque(0.0));
    tiny.left.wheels[0] = tiny.left.wheels[0].clone().radius(limits::MAX_SHAPE_EXTENT);
    tiny.left.wheels[1] = tiny.left.wheels[1].clone().radius(1.0e-36);
    assert_refused(tiny, STEP);
}
