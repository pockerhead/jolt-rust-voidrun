use super::super::tests::{bits3, curve_points, jolt_vec};
use super::*;
use crate::world::ensure_initialized;
use crate::{ObjectLayer, SpringSettings};

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

fn with_inertias(left: f32, right: f32) -> TrackedVehicleSettings {
    let mut settings = with_left(|t| t.inertia(left));
    settings.right = settings.right.clone().inertia(right);
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
    let below_min = f32::from_bits(limits::MIN_TRACK_INERTIA.to_bits() - 1);
    let above_max = f32::from_bits(limits::MAX_TRACK_INERTIA.to_bits() + 1);
    for inertia in [0.0, -1.0, f32::NAN, f32::INFINITY, below_min, above_max] {
        assert_refused(with_left(|t| t.inertia(inertia)), "track inertia");
    }
    for inertia in [limits::MIN_TRACK_INERTIA, limits::MAX_TRACK_INERTIA] {
        assert_eq!(with_inertias(inertia, inertia).validate(LAYERS), Ok(()));
    }
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
            w.suspension_spring(SpringSettings::FrequencyAndDamping {
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
fn track_wheel_radii_stay_within_max_ratio_of_the_driven_wheel() {
    const RULE: &str = "wheel radii";
    let radii = |driven: f32, other: f32| {
        let mut settings = tank();
        settings.left.wheels[0] = settings.left.wheels[0].clone().radius(driven);
        settings.left.wheels[2] = settings.left.wheels[2].clone().radius(other);
        settings
    };
    assert_eq!(radii(1.0, 1.0e-4).validate(LAYERS), Ok(()));
    assert_refused(radii(1.0, 0.99e-4), RULE);
    assert_eq!(radii(1.0e-3, 10.0).validate(LAYERS), Ok(()));
    assert_refused(radii(1.0e-3, 10.1), RULE);
    // The review's counter-example: a 1e-36 m wheel next to a 0.3 m driven wheel would turn at
    // 1200 rad/s · 0.3 / 1e-36, which overflows.
    assert_refused(radii(0.3, 1.0e-36), RULE);
}

#[test]
fn track_inertias_stay_within_max_track_inertia_ratio() {
    let ratio = limits::MAX_TRACK_INERTIA_RATIO;
    for lighter in [limits::MIN_TRACK_INERTIA, 10.0, 1.0e3] {
        let heavier = ratio * lighter;
        let above = f32::from_bits(heavier.to_bits() + 1);
        assert_eq!(with_inertias(lighter, heavier).validate(LAYERS), Ok(()));
        assert_eq!(with_inertias(heavier, lighter).validate(LAYERS), Ok(()));
        assert_refused(with_inertias(lighter, above), "inertia ratio");
        assert_refused(with_inertias(above, lighter), "inertia ratio");
    }
}

#[test]
fn tracked_drive_envelope_bounds_the_drivetrain() {
    const ENVELOPE: &str = "drive envelope";
    let engine = TrackedVehicleSettings::default_engine;
    let transmission = TrackedVehicleSettings::default_transmission;
    let torque = |value: f32| engine().max_torque(value);
    // Valid one by one: a huge max torque, heavy tracks, gear and differential ratios of 100.
    let mut counter_example = with_inertias(1.0e6, 1.0e6)
        .engine(torque(1.0e20).normalized_torque(vec![(0.0, limits::MAX_NORMALIZED_TORQUE)]))
        .transmission(transmission().gear_ratios(vec![100.0]));
    counter_example.left = counter_example.left.clone().differential_ratio(100.0);
    assert_refused(counter_example, ENVELOPE);

    // Each factor alone, a decade inside and a decade outside.
    assert_eq!(tank().engine(torque(1.0e18)).validate(LAYERS), Ok(()));
    assert_refused(tank().engine(torque(1.0e19)), ENVELOPE);
    let tenfold = vec![(0.0, limits::MAX_NORMALIZED_TORQUE)];
    assert_refused(
        tank().engine(torque(1.0e18).normalized_torque(tenfold)),
        ENVELOPE,
    );
    let gear = |ratio: f32| transmission().gear_ratios(vec![ratio, 1.0]);
    assert_eq!(
        tank()
            .engine(torque(1.0e17))
            .transmission(gear(100.0))
            .validate(LAYERS),
        Ok(())
    );
    assert_refused(
        tank().engine(torque(1.0e18)).transmission(gear(100.0)),
        ENVELOPE,
    );
    // A reverse gear counts as much as a forward one.
    assert_refused(
        tank()
            .engine(torque(1.0e18))
            .transmission(transmission().reverse_gear_ratios(vec![-100.0])),
        ENVELOPE,
    );
    // The speed limit grows as the smallest gear shrinks.
    assert_eq!(tank().transmission(gear(1.0e-13)).validate(LAYERS), Ok(()));
    assert_refused(tank().transmission(gear(1.0e-14)), ENVELOPE);
    assert_refused(
        tank().transmission(
            transmission()
                .gear_ratios(vec![1.0e-36])
                .reverse_gear_ratios(vec![-1.0e-36]),
        ),
        "underflows",
    );
    // Two light tracks of unequal inertia: the synchronisation bounds the torque.
    let light = |torque_value: f32| {
        let mut settings = tank().engine(torque(torque_value));
        settings.left = settings.left.clone().inertia(limits::MIN_TRACK_INERTIA);
        settings.right = settings
            .right
            .clone()
            .inertia(2.0 * limits::MIN_TRACK_INERTIA);
        settings
    };
    assert_eq!(light(1.0e11).validate(LAYERS), Ok(()));
    assert_refused(light(1.0e34), ENVELOPE);
    // The brake: its impulse over the inertia and its torque over the smallest wheel radius.
    assert_eq!(
        with_left(|t| t.max_brake_torque(1.0e29)).validate(LAYERS),
        Ok(())
    );
    assert_refused(with_left(|t| t.max_brake_torque(1.0e30)), ENVELOPE);
    let mut small_wheel = with_left(|t| t.max_brake_torque(1.0e25));
    small_wheel.left.wheels[2] = small_wheel.left.wheels[2].clone().radius(3.1e-5);
    assert_eq!(small_wheel.validate(LAYERS), Ok(()));
    small_wheel.left.max_brake_torque = 1.0e26;
    assert_refused(small_wheel, ENVELOPE);
    // The max rpm sets the speed limit too.
    assert_refused(tank().engine(engine().max_rpm(1.0e20)), ENVELOPE);
}

/// A 4000 kg chassis with principal inverse inertia `inverse_inertia`, its principal frame
/// turned from body space by `body_to_principal`, its centre of mass at `center`.
fn chassis(inverse_inertia: [f64; 3], body_to_principal: Quat, center: Vec3) -> ChassisMass {
    ChassisMass {
        mass: PrincipalMass {
            inverse_mass: 1.0 / 4000.0,
            inverse_inertia,
        },
        body_to_principal,
        center_of_mass: center,
    }
}

/// The largest `√((r × d)ᵀ M (r × d))` over 3600 directions `d` in the y/z plane, for the
/// symmetric `M` given by its rows.
fn sampled_plane_lever(m: [[f64; 3]; 3], r: [f64; 3]) -> f64 {
    (0..3600)
        .map(|k| {
            let (sin, cos) = (f64::from(k) * std::f64::consts::PI / 1800.0).sin_cos();
            let c = limits::cross(r, [0.0, sin, cos]);
            (0..3)
                .flat_map(|i| (0..3).map(move |j| (i, j)))
                .map(|(i, j)| c[i] * m[i][j] * c[j])
                .sum::<f64>()
                .sqrt()
        })
        .fold(0.0, f64::max)
}

#[test]
fn track_mass_ratio_takes_the_chassis_inertia_at_the_farthest_contact() {
    let (a, b, c): (f64, f64, f64) = (1.0 / 1500.0, 1.0 / 4000.0, 1.0 / 2000.0);
    let center = Vec3::new(0.3, -0.5, 0.0);
    // `tank()`'s wheels: x = ±1, y = −0.3, z = 1, 0, −1, up +Y and forward +Z, so every ground
    // normal gives a longitudinal direction in the y/z plane. Jolt's 10 kg·m² tracks on 0.3 m
    // wheels, every contact within 0.5 + √(0.3² + 0.05²) of the attachment, where a unit lever
    // weighs at most the largest inverse inertia.
    let reach = 0.5 + 0.3_f64.hypot(0.05);
    let per_metre = a.max(b).max(c).sqrt();
    let track_mass = 10.0 / (0.3_f64 * 0.3);
    for angle in [0.0_f64, 0.4, -0.4] {
        let (half_sin, half_cos) = (0.5 * angle).sin_cos();
        let to_principal = Quat::from_xyzw(0.0, 0.0, half_sin as f32, half_cos as f32);
        // The body-space inverse inertia Rᵀ D R, with R the turn by `angle` about z.
        let (sin, cos) = angle.sin_cos();
        let xx = a * cos * cos + b * sin * sin;
        let yy = a * sin * sin + b * cos * cos;
        let xy = sin * cos * (b - a);
        let body = [[xx, xy, 0.0], [xy, yy, 0.0], [0.0, 0.0, c]];
        let expected = [1.0_f64, -1.0]
            .into_iter()
            .flat_map(|x| [1.0_f64, 0.0, -1.0].map(|z| [x - 0.3, -0.3 + 0.5, z]))
            .map(|r| {
                let lever = sampled_plane_lever(body, r) + reach * per_metre;
                track_mass * (1.0 / 4000.0 + lever * lever)
            })
            .fold(0.0, f64::max);
        let ratio = tank().largest_track_mass_ratio(&chassis([a, b, c], to_principal, center));
        assert!(
            (ratio - expected).abs() <= 1e-5 * expected,
            "angle {angle}: {ratio} vs {expected}"
        );
    }
    // A suspension force point far below the contacts sets the lever.
    let mut settings = tank();
    settings.left.wheels[0] = settings.left.wheels[0]
        .clone()
        .suspension_force_point(Some(Vec3::new(1.0, -5.0, 0.0)));
    let principal = [[a, 0.0, 0.0], [0.0, b, 0.0], [0.0, 0.0, c]];
    let lever = sampled_plane_lever(principal, [0.7, -4.5, 0.0]);
    let expected = track_mass * (1.0 / 4000.0 + lever * lever);
    let ratio = settings.largest_track_mass_ratio(&chassis([a, b, c], Quat::IDENTITY, center));
    assert!(
        (ratio - expected).abs() <= 1e-5 * expected,
        "{ratio} vs {expected}"
    );
}
