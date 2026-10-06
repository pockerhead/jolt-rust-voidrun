use super::rebase::{BodyFrameState, FrameChange};
use super::*;
use crate::{Quat, RVec3, Real};

#[test]
fn any_full_buffer_makes_a_report_incomplete() {
    let complete = StepReport {
        manifold_cache_full: false,
        body_pair_cache_full: false,
        contact_constraints_full: false,
        rejected_contact_settings: 0,
    };
    assert!(complete.is_complete());
    for report in [
        StepReport {
            manifold_cache_full: true,
            ..complete
        },
        StepReport {
            body_pair_cache_full: true,
            ..complete
        },
        StepReport {
            contact_constraints_full: true,
            ..complete
        },
    ] {
        assert!(!report.is_complete(), "{report:?}");
    }
}

fn moving_state() -> BodyFrameState {
    BodyFrameState {
        position: RVec3::new(1.0, 0.0, 0.0),
        rotation: Quat::from_xyzw(0.0, 0.0, 0.6, 0.8),
        linear_velocity: Vec3::new(1.0, 0.0, 0.0),
        angular_velocity: Vec3::new(1.0, 0.0, 0.0),
    }
}

#[test]
fn translation_only_frame_change_keeps_rotation_and_velocity_bits() {
    let frame = FrameChange {
        rotation: Quat::IDENTITY,
        translation: RVec3::new(1.0, 2.0, 3.0),
    };
    let state = moving_state();
    let moved = frame.body(state).unwrap();
    assert_eq!(moved.position, RVec3::new(2.0, 2.0, 3.0));
    let bits = |q: Quat| <[f32; 4]>::from(q).map(f32::to_bits);
    assert_eq!(bits(moved.rotation), bits(state.rotation));
    assert_eq!(moved.linear_velocity, state.linear_velocity);
    assert_eq!(moved.angular_velocity, state.angular_velocity);
}

#[test]
fn quarter_turn_about_y_maps_x_to_minus_z() {
    let half = std::f32::consts::FRAC_1_SQRT_2;
    let frame = FrameChange {
        rotation: Quat::from_xyzw(0.0, half, 0.0, half),
        translation: RVec3::ZERO,
    };
    let moved = frame.body(moving_state()).unwrap();
    let p = moved.position;
    let near =
        |a: [f32; 3]| (a[0].abs() < 1e-6) && (a[1].abs() < 1e-6) && ((a[2] + 1.0).abs() < 1e-6);
    assert!(p.x.abs() < 1e-6 && p.y.abs() < 1e-6, "{moved:?}");
    assert!((p.z + 1.0).abs() < 1e-6, "{moved:?}");
    assert!(near(moved.linear_velocity.into()), "{moved:?}");
    assert!(near(moved.angular_velocity.into()), "{moved:?}");
    assert!(moved.rotation.is_valid_rotation(), "{moved:?}");
}

#[test]
fn frame_change_rejects_overflow_and_nan() {
    let frame = FrameChange {
        rotation: Quat::IDENTITY,
        translation: RVec3::new(Real::MAX, 0.0, 0.0),
    };
    let mut state = moving_state();
    state.position.x = Real::MAX / 2.0;
    assert_eq!(frame.body(state), None);

    let frame = FrameChange {
        rotation: Quat::IDENTITY,
        translation: RVec3::ZERO,
    };
    let mut state = moving_state();
    state.linear_velocity.y = f32::NAN;
    assert_eq!(frame.body(state), None);
}

#[test]
fn contact_constraint_capacity_is_bounded() {
    let settings = |value| WorldSettings::default().max_contact_constraints(value);
    for valid in [1, WorldSettings::MAX_CONTACT_CONSTRAINTS] {
        assert_eq!(settings(valid).validate(), Ok(()));
    }
    for invalid in [0, WorldSettings::MAX_CONTACT_CONSTRAINTS + 1, u32::MAX] {
        assert!(matches!(
            settings(invalid).validate(),
            Err(WorldError::InvalidValue(_))
        ));
    }
}

#[test]
fn worker_thread_bounds_are_validated() {
    for valid in [1, WorldSettings::MAX_WORKER_THREADS] {
        assert_eq!(
            WorldSettings::default().worker_threads(valid).validate(),
            Ok(())
        );
    }
    for invalid in [0, WorldSettings::MAX_WORKER_THREADS + 1, u32::MAX] {
        assert!(matches!(
            WorldSettings::default().worker_threads(invalid).validate(),
            Err(WorldError::InvalidValue(_))
        ));
    }
}

/// Jolt's `PhysicsSettings` defaults (`PhysicsSettings.h:36-126`) with the given step counts.
fn jolt_default_physics_settings(velocity_steps: u32, position_steps: u32) -> JPH_PhysicsSettings {
    JPH_PhysicsSettings {
        maxInFlightBodyPairs: 16384,
        stepListenersBatchSize: 8,
        stepListenerBatchesPerJob: 1,
        baumgarte: 0.2,
        speculativeContactDistance: 0.02,
        penetrationSlop: 0.02,
        linearCastThreshold: 0.75,
        linearCastMaxPenetration: 0.25,
        manifoldTolerance: 1.0e-3,
        maxPenetrationDistance: 0.2,
        bodyPairCacheMaxDeltaPositionSq: 0.001 * 0.001,
        bodyPairCacheCosMaxDeltaRotationDiv2: 1.0f64.to_radians().cos() as f32,
        contactNormalCosMaxDeltaRotation: 5.0f64.to_radians().cos() as f32,
        contactPointPreserveLambdaMaxDistSq: 0.01 * 0.01,
        numVelocitySteps: velocity_steps,
        numPositionSteps: position_steps,
        minVelocityForRestitution: 1.0,
        timeBeforeSleep: 0.5,
        pointVelocitySleepThreshold: 0.03,
        deterministicSimulation: true,
        constraintWarmStart: true,
        useBodyPairContactCache: true,
        useManifoldReduction: true,
        useLargeIslandSplitter: true,
        allowSleeping: true,
        checkActiveEdges: true,
    }
}

#[test]
fn solver_steps_leave_every_other_physics_setting_at_jolts_default() {
    for (velocity, position) in [(10, 2), (4, 0), (255, 255)] {
        let settings = WorldSettings::default()
            .velocity_steps(velocity)
            .position_steps(position);
        let world = PhysicsWorld::new(settings).unwrap();
        // SAFETY: the world's system is live and nothing steps it.
        let read = unsafe { physics_settings(world.system.as_ptr()) };
        assert_eq!(
            format!("{read:?}"),
            format!("{:?}", jolt_default_physics_settings(velocity, position))
        );
    }
}
