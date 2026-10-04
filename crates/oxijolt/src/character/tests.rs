use super::*;
use crate::world::ensure_initialized;

#[test]
fn default_settings_match_jolt() {
    assert!(ensure_initialized());
    let capsule = Shape::new_capsule(0.5, 0.3).unwrap();
    let ours = CharacterSettings::new(&capsule).to_jph(1);
    // SAFETY: an all-zero struct is valid input for `_Init`, which overwrites it.
    let mut jolt: JPH_CharacterVirtualSettings = unsafe { std::mem::zeroed() };
    // SAFETY: `jolt` is a live local. `_Init` creates an empty shape holding one reference,
    // released below, and empty shape settings whose one reference joltc never releases:
    // one small object per test process.
    unsafe { JPH_CharacterVirtualSettings_Init(&mut jolt) };
    let bits = |v: JPH_Vec3| [v.x, v.y, v.z].map(f32::to_bits);
    assert_eq!(bits(jolt.base.up), bits(ours.base.up));
    assert_eq!(
        bits(jolt.base.supportingVolume.normal),
        bits(ours.base.supportingVolume.normal)
    );
    assert_eq!(
        jolt.base.supportingVolume.distance,
        ours.base.supportingVolume.distance
    );
    assert_eq!(
        jolt.base.maxSlopeAngle.to_bits(),
        ours.base.maxSlopeAngle.to_bits()
    );
    assert_eq!(
        jolt.base.enhancedInternalEdgeRemoval,
        ours.base.enhancedInternalEdgeRemoval
    );
    assert_eq!(jolt.mass, ours.mass);
    assert_eq!(jolt.maxStrength, ours.maxStrength);
    assert_eq!(bits(jolt.shapeOffset), bits(ours.shapeOffset));
    assert_eq!(jolt.backFaceMode, ours.backFaceMode);
    assert_eq!(
        jolt.predictiveContactDistance,
        ours.predictiveContactDistance
    );
    assert_eq!(jolt.maxCollisionIterations, ours.maxCollisionIterations);
    assert_eq!(jolt.maxConstraintIterations, ours.maxConstraintIterations);
    assert_eq!(jolt.minTimeRemaining, ours.minTimeRemaining);
    assert_eq!(jolt.collisionTolerance, ours.collisionTolerance);
    assert_eq!(jolt.characterPadding, ours.characterPadding);
    assert_eq!(jolt.maxNumHits, ours.maxNumHits);
    assert_eq!(jolt.hitReductionCosMaxAngle, ours.hitReductionCosMaxAngle);
    assert_eq!(jolt.penetrationRecoverySpeed, ours.penetrationRecoverySpeed);
    assert!(jolt.innerBodyShape.is_null());
    assert_eq!(jolt.innerBodyLayer, ours.innerBodyLayer);
    // SAFETY: `_Init` returned the empty shape holding one reference, released once here.
    unsafe { JPH_Shape_Destroy(jolt.base.shape.cast_mut()) };
}

#[test]
fn ground_states_convert_and_report_support() {
    let states = [
        (JPH_GroundState_OnGround, GroundState::OnGround, true),
        (
            JPH_GroundState_OnSteepGround,
            GroundState::OnSteepGround,
            true,
        ),
        (
            JPH_GroundState_NotSupported,
            GroundState::NotSupported,
            false,
        ),
        (JPH_GroundState_InAir, GroundState::InAir, false),
    ];
    for (raw, state, supported) in states {
        assert_eq!(GroundState::from_jph(raw), state);
        assert_eq!(state.is_supported(), supported, "{state:?}");
    }
}
