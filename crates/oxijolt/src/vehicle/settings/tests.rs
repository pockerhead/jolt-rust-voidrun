use super::*;
use crate::world::ensure_initialized;
use crate::{limits, ObjectLayer, SpringSettings};

pub(super) fn bits3(v: Vec3) -> [u32; 3] {
    <[f32; 3]>::from(v).map(f32::to_bits)
}

pub(super) fn jolt_vec(get: impl FnOnce(*mut JPH_Vec3)) -> Vec3 {
    let mut value = Vec3::ZERO.to_jph();
    get(&mut value);
    Vec3::from_jph(value)
}

/// The points of a curve that joltc lends, read before its owner is released.
///
/// # Safety
/// `curve` points to a live curve.
pub(super) unsafe fn curve_points(curve: *const JPH_LinearCurve) -> Vec<(f32, f32)> {
    // SAFETY: the curve is live (caller contract); the getters only read it, with indices
    // below its point count.
    unsafe {
        (0..JPH_LinearCurve_GetPointCount(curve))
            .map(|index| {
                let mut point = JPH_Point { x: 0.0, y: 0.0 };
                JPH_LinearCurve_GetPoint(curve, index, &mut point);
                (point.x, point.y)
            })
            .collect()
    }
}

#[test]
fn wheel_defaults_are_jolts() {
    assert!(ensure_initialized());
    let ours = WheelSettings::new(Vec3::ZERO);
    // SAFETY: Jolt is initialised; the guard owns the one reference joltc returns.
    let jolt = unsafe { Owned::from_raw(JPH_WheelSettingsWV_Create()) }.unwrap();
    let wv = jolt.as_ptr();
    let base: *mut JPH_WheelSettings = wv.cast();
    // SAFETY: the settings are live and only read; the friction curves are members of the
    // settings, read while the guard keeps them alive. Every output is a live local.
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
        assert_eq!(ours.base.suspension_force_point, None);
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
        assert_eq!(JPH_WheelSettingsWV_GetInertia(wv), ours.inertia);
        assert_eq!(
            JPH_WheelSettingsWV_GetAngularDamping(wv),
            ours.angular_damping
        );
        assert_eq!(
            JPH_WheelSettingsWV_GetMaxSteerAngle(wv),
            ours.max_steer_angle
        );
        assert_eq!(
            JPH_WheelSettingsWV_GetMaxBrakeTorque(wv),
            ours.max_brake_torque
        );
        assert_eq!(
            JPH_WheelSettingsWV_GetMaxHandBrakeTorque(wv),
            ours.max_hand_brake_torque
        );
        assert_eq!(
            curve_points(JPH_WheelSettingsWV_GetLongitudinalFriction(wv)),
            ours.longitudinal_friction
        );
        assert_eq!(
            curve_points(JPH_WheelSettingsWV_GetLateralFriction(wv)),
            ours.lateral_friction
        );
    }
}

#[test]
fn controller_defaults_are_jolts() {
    assert!(ensure_initialized());
    let engine = VehicleEngineSettings::default();
    let transmission = VehicleTransmissionSettings::default();
    let vehicle = car();
    // SAFETY: Jolt is initialised; the guard owns the one reference joltc returns.
    let jolt = unsafe { Owned::from_raw(JPH_WheeledVehicleControllerSettings_Create()) }.unwrap();
    let ptr = jolt.as_ptr();
    // SAFETY: the settings are live and only read. `GetEngine` lends the settings' own
    // torque curve and `GetTransmission` the settings' own transmission; both are read
    // while the guard keeps the settings alive and never destroyed here.
    unsafe {
        assert_eq!(
            JPH_WheeledVehicleControllerSettings_GetDifferentialLimitedSlipRatio(ptr),
            vehicle.differential_limited_slip_ratio
        );
        let mut jolt_engine: JPH_VehicleEngineSettings = std::mem::zeroed();
        JPH_WheeledVehicleControllerSettings_GetEngine(ptr, &mut jolt_engine);
        assert_eq!(jolt_engine.maxTorque, engine.max_torque);
        assert_eq!(jolt_engine.minRPM, engine.min_rpm);
        assert_eq!(jolt_engine.maxRPM, engine.max_rpm);
        assert_eq!(jolt_engine.inertia, engine.inertia);
        assert_eq!(jolt_engine.angularDamping, engine.angular_damping);
        assert_eq!(
            curve_points(jolt_engine.normalizedTorque),
            engine.normalized_torque
        );

        let jolt_transmission = JPH_WheeledVehicleControllerSettings_GetTransmission(ptr);
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
fn constraint_differential_and_anti_roll_bar_defaults_are_jolts() {
    assert!(ensure_initialized());
    let vehicle = car();
    // SAFETY: all-zero values of these plain C structs are valid (integers, floats, `false`
    // and null pointers); the `_Init` calls fill them with Jolt's defaults and allocate
    // nothing.
    let (constraint, differential, bar) = unsafe {
        let mut constraint: JPH_VehicleConstraintSettings = std::mem::zeroed();
        let mut differential: JPH_VehicleDifferentialSettings = std::mem::zeroed();
        let mut bar: JPH_VehicleAntiRollBar = std::mem::zeroed();
        JPH_VehicleConstraintSettings_Init(&mut constraint);
        JPH_VehicleDifferentialSettings_Init(&mut differential);
        JPH_VehicleAntiRollBar_Init(&mut bar);
        (constraint, differential, bar)
    };
    assert_eq!(
        bits3(Vec3::from_jph(constraint.up)),
        bits3(vehicle.frame.up)
    );
    assert_eq!(
        bits3(Vec3::from_jph(constraint.forward)),
        bits3(vehicle.frame.forward)
    );
    assert_eq!(
        constraint.maxPitchRollAngle,
        vehicle.frame.max_pitch_roll_angle
    );

    let ours = VehicleDifferentialSettings::new(None, None).to_jph();
    assert_eq!(ours.leftWheel, differential.leftWheel);
    assert_eq!(ours.rightWheel, differential.rightWheel);
    assert_eq!(ours.differentialRatio, differential.differentialRatio);
    assert_eq!(ours.leftRightSplit, differential.leftRightSplit);
    assert_eq!(ours.limitedSlipRatio, differential.limitedSlipRatio);
    assert_eq!(ours.engineTorqueRatio, differential.engineTorqueRatio);

    let ours = VehicleAntiRollBar::new(0, 1);
    assert_eq!(bar.leftWheel, ours.left_wheel as i32);
    assert_eq!(bar.rightWheel, ours.right_wheel as i32);
    assert_eq!(bar.stiffness, ours.stiffness);
}

#[test]
fn built_settings_reach_jolt() {
    assert!(ensure_initialized());
    let wheel = WheelSettings::new(Vec3::new(0.9, -0.1, 1.4))
        .suspension_force_point(Some(Vec3::new(0.9, -0.3, 1.4)))
        .radius(0.35)
        .width(0.2)
        .inertia(1.5)
        .max_steer_angle(0.5)
        .longitudinal_friction(vec![(0.0, 0.0), (0.1, 1.5)])
        .suspension_spring(SpringSettings::StiffnessAndDamping {
            stiffness: 30000.0,
            damping: 2000.0,
        });
    let jolt = wheel.create();
    let wv = jolt.as_ptr();
    let base: *mut JPH_WheelSettings = wv.cast();
    // SAFETY: the settings are live and only read; outputs are live locals.
    unsafe {
        let position = jolt_vec(|v| JPH_WheelSettings_GetPosition(base, v));
        assert_eq!(bits3(position), bits3(wheel.base.position));
        assert!(JPH_WheelSettings_GetEnableSuspensionForcePoint(base));
        assert_eq!(JPH_WheelSettings_GetRadius(base), 0.35);
        assert_eq!(JPH_WheelSettings_GetWidth(base), 0.2);
        assert_eq!(JPH_WheelSettingsWV_GetInertia(wv), 1.5);
        assert_eq!(JPH_WheelSettingsWV_GetMaxSteerAngle(wv), 0.5);
        let mut spring = JPH_SpringSettings {
            mode: JPH_SpringMode_FrequencyAndDamping,
            frequencyOrStiffness: 0.0,
            damping: 0.0,
        };
        JPH_WheelSettings_GetSuspensionSpring(base, &mut spring);
        assert_eq!(spring.mode, JPH_SpringMode_StiffnessAndDamping);
        assert_eq!(spring.frequencyOrStiffness, 30000.0);
        assert_eq!(
            curve_points(JPH_WheelSettingsWV_GetLongitudinalFriction(wv)),
            vec![(0.0, 0.0), (0.1, 1.5)]
        );
    }

    let vehicle = car()
        .engine(VehicleEngineSettings::default().max_torque(800.0))
        .transmission(VehicleTransmissionSettings::default().gear_ratios(vec![3.0, 1.5]))
        .differential_limited_slip_ratio(f32::MAX);
    let controller = vehicle.create_controller();
    // SAFETY: as in `controller_defaults_are_jolts`.
    unsafe {
        let ptr = controller.as_ptr();
        let mut engine: JPH_VehicleEngineSettings = std::mem::zeroed();
        JPH_WheeledVehicleControllerSettings_GetEngine(ptr, &mut engine);
        assert_eq!(engine.maxTorque, 800.0);
        let transmission = JPH_WheeledVehicleControllerSettings_GetTransmission(ptr);
        assert_eq!(
            JPH_VehicleTransmissionSettings_GetGearRatioCount(transmission),
            2
        );
        assert_eq!(
            JPH_WheeledVehicleControllerSettings_GetDifferentialsCount(ptr),
            1
        );
        assert_eq!(
            JPH_WheeledVehicleControllerSettings_GetDifferentialLimitedSlipRatio(ptr),
            f32::MAX
        );
    }
    let tester = VehicleCollisionTester::ray(ObjectLayer::new(1)).create(0);
    // SAFETY: the tester is live and only read.
    let layer = unsafe { JPH_VehicleCollisionTester_GetObjectLayer(tester.as_ptr()) };
    assert_eq!(layer, 1);
}

/// A valid car: four wheels, front-wheel drive, a ray tester on layer 1.
fn car() -> WheeledVehicleSettings {
    let wheel = |x: f32, z: f32| WheelSettings::new(Vec3::new(x, -0.1, z)).radius(0.35);
    WheeledVehicleSettings::new(
        vec![
            wheel(0.9, 1.4),
            wheel(-0.9, 1.4),
            wheel(0.9, -1.4),
            wheel(-0.9, -1.4),
        ],
        vec![VehicleDifferentialSettings::new(Some(0), Some(1))],
        VehicleCollisionTester::ray(ObjectLayer::new(1)),
    )
}

const LAYERS: u32 = 2;

#[track_caller]
fn assert_rejected(settings: WheeledVehicleSettings) {
    assert!(
        matches!(
            settings.validate(LAYERS),
            Err(VehicleError::InvalidValue(_))
        ),
        "{settings:?}"
    );
}

fn with_wheel(edit: impl Fn(WheelSettings) -> WheelSettings) -> WheeledVehicleSettings {
    let mut settings = car();
    settings.wheels[2] = edit(settings.wheels[2].clone());
    settings
}

#[test]
fn valid_settings_pass() {
    assert_eq!(car().validate(LAYERS), Ok(()));
    let two_differentials = WheeledVehicleSettings {
        differentials: vec![
            VehicleDifferentialSettings::new(Some(0), Some(1)).engine_torque_ratio(0.5),
            VehicleDifferentialSettings::new(Some(2), Some(3)).engine_torque_ratio(0.5),
        ],
        ..car()
    }
    .anti_roll_bars(vec![VehicleAntiRollBar::new(0, 1)])
    .differential_limited_slip_ratio(f32::MAX);
    assert_eq!(two_differentials.validate(LAYERS), Ok(()));
    let cylinder = WheeledVehicleSettings {
        collision_tester: VehicleCollisionTester::cast_cylinder(ObjectLayer::new(0)),
        ..car()
    };
    assert_eq!(cylinder.validate(LAYERS), Ok(()));
}

#[test]
fn vehicle_values_are_validated() {
    assert_rejected(WheeledVehicleSettings {
        wheels: Vec::new(),
        differentials: Vec::new(),
        ..car()
    });
    assert_rejected(car().up(Vec3::new(0.0, 2.0, 0.0)));
    assert_rejected(car().forward(Vec3::new(0.0, f32::NAN, 1.0)));
    assert_rejected(car().forward(Vec3::new(0.0, 0.6, 0.8)));
    assert_rejected(car().max_pitch_roll_angle(-0.1));
    assert_rejected(car().max_pitch_roll_angle(PI + 0.001));
    assert_rejected(car().differential_limited_slip_ratio(1.0));
    assert_rejected(car().differential_limited_slip_ratio(f32::INFINITY));
}

#[test]
fn unit_vectors_are_checked_more_strictly_than_jolt() {
    // |v|² - 1 = 8e-7: inside Jolt's `IsNormalized` tolerance of 1e-6, outside ours.
    let almost = Vec3::new(0.0, -1.000_000_4, 0.0);
    assert!((almost.dot(almost) - 1.0).abs() < 1.0e-6);
    assert!(!is_unit(almost));
    assert_rejected(with_wheel(|w| w.suspension_direction(almost)));
    assert!(is_unit(Vec3::new(0.6, 0.0, 0.8)));
}

#[test]
fn wheel_values_are_validated() {
    let y = Vec3::new(0.0, 1.0, 0.0);
    assert_rejected(with_wheel(|w| {
        w.suspension_direction(Vec3::new(0.0, -1.1, 0.0))
    }));
    assert_rejected(with_wheel(|w| w.steering_axis(Vec3::ZERO)));
    assert_rejected(with_wheel(|w| w.wheel_up(Vec3::new(0.0, 0.0, 1.0))));
    assert_rejected(with_wheel(|w| w.wheel_forward(y)));
    assert_rejected(with_wheel(|w| {
        w.suspension_force_point(Some(Vec3::new(f32::NAN, 0.0, 0.0)))
    }));
    assert_rejected(with_wheel(|w| w.suspension_min_length(-0.1)));
    assert_rejected(with_wheel(|w| w.suspension_max_length(0.2)));
    assert_rejected(with_wheel(|w| w.suspension_preload_length(-0.1)));
    assert_rejected(with_wheel(|w| {
        w.suspension_spring(SpringSettings::FrequencyAndDamping {
            frequency: 0.0,
            damping: 0.5,
        })
    }));
    assert_rejected(with_wheel(|w| {
        w.suspension_spring(SpringSettings::StiffnessAndDamping {
            stiffness: 1000.0,
            damping: -1.0,
        })
    }));
    assert_rejected(with_wheel(|w| w.radius(0.0)));
    assert_rejected(with_wheel(|w| w.width(-0.1)));
    assert_rejected(with_wheel(|w| w.angular_damping(-0.1)));
    assert_rejected(with_wheel(|w| w.max_steer_angle(0.5 * PI + 0.001)));
    assert_rejected(with_wheel(|w| w.max_brake_torque(-1.0)));
    assert_rejected(with_wheel(|w| w.max_hand_brake_torque(f32::NAN)));
    assert_rejected(with_wheel(|w| w.longitudinal_friction(Vec::new())));
    assert_rejected(with_wheel(|w| {
        w.lateral_friction(vec![(0.0, 0.0), (3.0, 1.2), (3.0, 1.0)])
    }));
    assert_eq!(with_wheel(|w| w.width(0.0)).validate(LAYERS), Ok(()));
    assert_eq!(
        with_wheel(|w| w.max_steer_angle(-0.5 * PI)).validate(LAYERS),
        Ok(())
    );
}

#[test]
fn wheel_magnitudes_are_bounded_by_the_policy() {
    let extent = limits::MAX_SHAPE_EXTENT;
    let beyond = extent.next_up();
    let at = |x: f32| Vec3::new(x, -0.1, 0.0);
    let placed = |p: Vec3| move |_: WheelSettings| WheelSettings::new(p).radius(0.35);
    assert_eq!(with_wheel(placed(at(extent))).validate(LAYERS), Ok(()));
    assert_rejected(with_wheel(placed(at(beyond))));
    assert_eq!(
        with_wheel(|w| w.suspension_force_point(Some(at(-extent)))).validate(LAYERS),
        Ok(())
    );
    assert_rejected(with_wheel(|w| w.suspension_force_point(Some(at(-beyond)))));
    type Edit = fn(WheelSettings, f32) -> WheelSettings;
    let lengths: [Edit; 5] = [
        |w, v| {
            w.suspension_min_length(v)
                .suspension_max_length(limits::MAX_SHAPE_EXTENT)
        },
        |w, v| w.suspension_max_length(v),
        |w, v| w.suspension_preload_length(v),
        |w, v| w.radius(v),
        |w, v| w.width(v),
    ];
    for edit in lengths {
        assert_eq!(with_wheel(|w| edit(w, extent)).validate(LAYERS), Ok(()));
        assert_rejected(with_wheel(|w| edit(w, beyond)));
    }
}

#[test]
fn suspension_springs_are_bounded_by_the_coefficient() {
    let bound = limits::MAX_SPRING_COEFFICIENT;
    let stiffness = |stiffness, damping| SpringSettings::StiffnessAndDamping { stiffness, damping };
    let spring = |spring: SpringSettings| with_wheel(move |w| w.suspension_spring(spring));
    assert_eq!(spring(stiffness(bound, bound)).validate(LAYERS), Ok(()));
    assert_rejected(spring(stiffness(bound.next_up(), 0.0)));
    assert_rejected(spring(stiffness(1.0, bound.next_up())));
    // Frequency mode with the effective mass at most `MAX_MASS`: `MAX_MASS * ω² <= bound`
    // gives the largest frequency, `2 * MAX_MASS * ζ * ω <= bound` the largest damping ratio
    // at 1 Hz.
    let mass = f64::from(limits::MAX_MASS);
    let two_pi = 2.0 * std::f64::consts::PI;
    let max_frequency = (f64::from(bound) / mass).sqrt() / two_pi;
    let max_damping = f64::from(bound) / (2.0 * mass * two_pi);
    let frequency = |frequency: f64, damping: f64| SpringSettings::FrequencyAndDamping {
        frequency: frequency as f32,
        damping: damping as f32,
    };
    for (factor, accepted) in [(0.999, true), (1.001, false)] {
        for candidate in [
            frequency(factor * max_frequency, 0.0),
            frequency(1.0, factor * max_damping),
        ] {
            assert_eq!(
                spring(candidate).validate(LAYERS).is_ok(),
                accepted,
                "{candidate:?}"
            );
        }
    }
}

#[test]
fn wheel_inertia_must_be_positive() {
    // Jolt asserts only `>= 0`, but divides by the wheel inertia.
    assert_rejected(with_wheel(|w| w.inertia(0.0)));
}

const SMALLEST_SUBNORMAL: f32 = f32::from_bits(1);

#[test]
fn step_coefficients_of_wheels_must_be_finite() {
    // `delta_time / inertia` overflows for a subnormal inertia, driven wheel or not.
    assert_rejected(with_wheel(|w| w.inertia(SMALLEST_SUBNORMAL)));
    assert_rejected(with_wheel(|w| w.radius(SMALLEST_SUBNORMAL)));
    assert_rejected(with_wheel(|w| w.radius(f32::MAX).inertia(1.0e-3)));
    assert_rejected(with_wheel(|w| {
        w.max_brake_torque(f32::MAX).max_hand_brake_torque(f32::MAX)
    }));
    assert_rejected(with_wheel(|w| w.max_brake_torque(f32::MAX).inertia(0.5)));
    // The brake-lock torque per rad/s, `inertia / MIN_DELTA_TIME`, overflows; with radius 1
    // every other coefficient of the wheel stays finite.
    assert_rejected(with_wheel(|w| w.radius(1.0).inertia(f32::MAX / 2.0)));
    assert_eq!(
        with_wheel(|w| w.radius(1.0).inertia(1.0e30)).validate(LAYERS),
        Ok(())
    );
    assert_eq!(with_wheel(|w| w.inertia(1.0e-30)).validate(LAYERS), Ok(()));
    assert_eq!(
        with_wheel(|w| w.max_brake_torque(1.0e30)).validate(LAYERS),
        Ok(())
    );
}

#[test]
fn step_coefficients_of_the_drivetrain_must_be_finite() {
    let engine = VehicleEngineSettings::default();
    assert_rejected(car().engine(engine.clone().inertia(SMALLEST_SUBNORMAL)));
    assert_rejected(car().engine(engine.clone().inertia(1.0e-38)));
    assert_rejected(car().engine(engine.clone().max_torque(f32::MAX)));
    assert_rejected(car().engine(engine.clone().normalized_torque(vec![(0.0, f32::MAX)])));
    let transmission = VehicleTransmissionSettings::default();
    assert_rejected(car().transmission(transmission.clone().clutch_strength(f32::MAX)));
    assert_rejected(car().transmission(transmission.gear_ratios(vec![f32::MAX])));
    assert_rejected(WheeledVehicleSettings {
        differentials: vec![
            VehicleDifferentialSettings::new(Some(0), Some(1)).differential_ratio(f32::MAX)
        ],
        ..car()
    });
    // A driven wheel with a tiny inertia overflows `delta_time * S * R / inertia`; the same
    // inertia on an undriven wheel does not.
    let tiny = |w: WheelSettings| {
        w.inertia(1.0e-37)
            .max_brake_torque(0.0)
            .max_hand_brake_torque(0.0)
    };
    let mut tiny_driven = car();
    tiny_driven.wheels[0] = tiny(tiny_driven.wheels[0].clone());
    assert_rejected(tiny_driven);
    assert_eq!(with_wheel(tiny).validate(LAYERS), Ok(()));
    assert_eq!(
        car().engine(engine.max_torque(1.0e30)).validate(LAYERS),
        Ok(())
    );
}

#[test]
fn engine_values_are_validated() {
    let engine = |edit: fn(VehicleEngineSettings) -> VehicleEngineSettings| {
        car().engine(edit(VehicleEngineSettings::default()))
    };
    assert_rejected(engine(|e| e.max_torque(-1.0)));
    assert_rejected(engine(|e| e.min_rpm(-1.0)));
    assert_rejected(engine(|e| e.min_rpm(7000.0)));
    assert_rejected(engine(|e| e.max_rpm(f32::INFINITY)));
    assert_rejected(engine(|e| e.angular_damping(-0.1)));
    assert_rejected(engine(|e| e.normalized_torque(Vec::new())));
    assert_rejected(engine(|e| {
        e.normalized_torque(vec![(0.5, 1.0), (0.2, 0.8)])
    }));
}

#[test]
fn engine_torque_curves_stay_in_their_domain() {
    let curve = |points: Vec<(f32, f32)>| {
        car().engine(VehicleEngineSettings::default().normalized_torque(points))
    };
    let refused = |points: Vec<(f32, f32)>| match curve(points.clone()).validate(LAYERS) {
        Err(VehicleError::InvalidValue(what)) => assert_eq!(what, limits::TORQUE_CURVE_RULE),
        other => panic!("{points:?} gave {other:?}"),
    };
    let spacing = limits::MIN_TORQUE_CURVE_SPACING;
    let top = limits::MAX_NORMALIZED_TORQUE;
    // The widest curve: both ends of x, the closest spacing, y from 0 to the top.
    assert_eq!(
        curve(vec![(0.0, 0.0), (spacing, top), (1.0, top)]).validate(LAYERS),
        Ok(())
    );
    refused(vec![(-f32::EPSILON, 1.0)]);
    refused(vec![(0.5, 1.0), (1.0 + f32::EPSILON, 1.0)]);
    refused(vec![(0.5, -f32::MIN_POSITIVE)]);
    refused(vec![(0.5, f32::from_bits(top.to_bits() + 1))]);
    refused(vec![(0.5, f32::NAN)]);
    refused(vec![(0.5, 1.0), (0.5 + 0.99 * spacing, 1.0)]);
    // Rounding x to `f32` may take up to one epsilon off the spacing: a curve sampled at
    // `i / 1000` passes, and above 0.5 the closest accepted neighbour is one ulp from refused.
    let sampled = (0..=1000).map(|i| (i as f32 / 1000.0, 1.0)).collect();
    assert_eq!(curve(sampled).validate(LAYERS), Ok(()));
    let closest = 0.500_999_9_f32;
    assert_eq!(
        curve(vec![(0.5, 1.0), (closest, 1.0)]).validate(LAYERS),
        Ok(())
    );
    refused(vec![
        (0.5, 1.0),
        (f32::from_bits(closest.to_bits() - 1), 1.0),
    ]);
    // Curves whose interpolation overflows in Jolt: a wide x span with a huge y, a wide x span
    // alone, a wide y span alone.
    refused(vec![(-1.0e30, 0.0), (1.0e30, 1.0e30)]);
    refused(vec![(-1.0e38, 0.0), (1.0e38, top)]);
    refused(vec![(0.0, -3.0e38), (1.0, 3.0e38)]);
}

#[test]
fn engine_inertia_must_be_positive() {
    assert_rejected(car().engine(VehicleEngineSettings::default().inertia(0.0)));
}

#[test]
fn transmission_values_are_validated() {
    let transmission = |edit: fn(VehicleTransmissionSettings) -> VehicleTransmissionSettings| {
        car().transmission(edit(VehicleTransmissionSettings::default()))
    };
    assert_rejected(transmission(|t| t.gear_ratios(vec![2.0, 0.0])));
    assert_rejected(transmission(|t| t.reverse_gear_ratios(vec![0.0])));
    assert_rejected(transmission(|t| t.switch_time(-0.1)));
    assert_rejected(transmission(|t| t.clutch_release_time(f32::NAN)));
    assert_rejected(transmission(|t| t.switch_latency(-0.1)));
    assert_rejected(transmission(|t| t.shift_down_rpm(0.0)));
    assert_rejected(transmission(|t| t.shift_up_rpm(2000.0)));
    assert_rejected(transmission(|t| t.shift_up_rpm(6000.0)));
    assert_rejected(transmission(|t| t.clutch_strength(0.0)));
}

#[test]
fn gear_lists_must_not_be_empty() {
    let transmission = VehicleTransmissionSettings::default();
    assert_rejected(car().transmission(transmission.clone().gear_ratios(Vec::new())));
    assert_rejected(car().transmission(transmission.reverse_gear_ratios(Vec::new())));
}

#[test]
fn differentials_are_required() {
    assert_rejected(WheeledVehicleSettings {
        differentials: Vec::new(),
        ..car()
    });
}

#[test]
fn differential_values_are_validated() {
    let with = |differential: VehicleDifferentialSettings| WheeledVehicleSettings {
        differentials: vec![differential],
        ..car()
    };
    let front = VehicleDifferentialSettings::new(Some(0), Some(1));
    assert_rejected(with(VehicleDifferentialSettings::new(Some(0), Some(4))));
    assert_rejected(with(VehicleDifferentialSettings::new(None, None)));
    assert_rejected(with(front.differential_ratio(0.0)));
    assert_rejected(with(front.left_right_split(1.5)));
    assert_rejected(with(front.limited_slip_ratio(1.0)));
    assert_rejected(with(front.engine_torque_ratio(-0.5)));
    assert_rejected(with(front.engine_torque_ratio(0.9)));
    assert_eq!(
        with(VehicleDifferentialSettings::new(None, Some(3))).validate(LAYERS),
        Ok(())
    );
    let uneven = WheeledVehicleSettings {
        differentials: vec![
            front.engine_torque_ratio(0.5),
            VehicleDifferentialSettings::new(Some(2), Some(3)).engine_torque_ratio(0.4999),
        ],
        ..car()
    };
    assert_rejected(uneven);
}

#[test]
fn anti_roll_bars_are_validated() {
    assert_rejected(car().anti_roll_bars(vec![VehicleAntiRollBar::new(0, 4)]));
    assert_rejected(car().anti_roll_bars(vec![VehicleAntiRollBar::new(1, 1)]));
    assert_rejected(car().anti_roll_bars(vec![VehicleAntiRollBar::new(0, 1).stiffness(-1.0)]));
    let bar =
        |stiffness| car().anti_roll_bars(vec![VehicleAntiRollBar::new(0, 1).stiffness(stiffness)]);
    let bound = VehicleAntiRollBar::MAX_STIFFNESS;
    assert_eq!(bar(bound).validate(LAYERS), Ok(()));
    for stiffness in [bound.next_up(), f32::MAX, f32::NAN] {
        assert_rejected(bar(stiffness));
    }
}

#[test]
fn collision_testers_are_validated() {
    let with = |tester: VehicleCollisionTester| WheeledVehicleSettings {
        collision_tester: tester,
        ..car()
    };
    let layer = ObjectLayer::new(1);
    assert_rejected(with(VehicleCollisionTester::ray(ObjectLayer::new(LAYERS))));
    assert_rejected(with(VehicleCollisionTester::Ray {
        object_layer: layer,
        up: Vec3::new(0.0, 0.5, 0.0),
        max_slope_angle: 1.0,
    }));
    assert_rejected(with(VehicleCollisionTester::Ray {
        object_layer: layer,
        up: Vec3::new(0.0, 1.0, 0.0),
        max_slope_angle: PI + 0.01,
    }));
    assert_rejected(with(VehicleCollisionTester::cast_sphere(layer, 0.0)));
    // Max suspension length 0.5 plus wheel radius 0.35 leaves no cast for a 0.85 sphere.
    assert_rejected(with(VehicleCollisionTester::cast_sphere(layer, 0.85)));
    assert_eq!(
        with(VehicleCollisionTester::cast_sphere(layer, 0.84)).validate(LAYERS),
        Ok(())
    );
    // A sphere as large as a wheel at the length bound leaves no cast.
    let largest = |w: WheelSettings| {
        w.suspension_max_length(limits::MAX_SHAPE_EXTENT)
            .radius(limits::MAX_SHAPE_EXTENT)
            .inertia(1.0e30)
    };
    assert_eq!(with_wheel(largest).validate(LAYERS), Ok(()));
    let mut largest_sphere = with_wheel(largest);
    largest_sphere.collision_tester =
        VehicleCollisionTester::cast_sphere(layer, 2.0 * limits::MAX_SHAPE_EXTENT);
    assert_rejected(largest_sphere);
    assert_rejected(with(VehicleCollisionTester::CastCylinder {
        object_layer: layer,
        convex_radius_fraction: 1.5,
    }));
    let cylinder = VehicleCollisionTester::cast_cylinder(layer);
    let mut no_width = with_wheel(|w| w.width(0.0));
    no_width.collision_tester = cylinder;
    assert_rejected(no_width);
    let mut no_travel = with_wheel(|w| w.suspension_min_length(0.0).suspension_max_length(0.0));
    assert_eq!(no_travel.validate(LAYERS), Ok(()));
    no_travel.collision_tester = cylinder;
    assert_rejected(no_travel);
}
