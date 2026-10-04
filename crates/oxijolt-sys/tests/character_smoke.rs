//! Smoke test for the fork's joltc additions: a CharacterVirtual updated with explicit gravity,
//! filters and temp allocator lands on a floor, and its saved state restored into a character of
//! an identically built world continues bit for bit.

mod framework;

use std::ptr::null;

use framework::*;
use oxijolt_sys::*;

const DT: f32 = 1.0 / 60.0;
const RADIUS: f32 = 0.4;
const HALF_HEIGHT: f32 = 0.70845;

/// Jolt's `CharacterVirtual::ExtendedUpdateSettings` defaults.
fn default_extended_update_settings() -> JPH_ExtendedUpdateSettings {
    JPH_ExtendedUpdateSettings {
        stickToFloorStepDown: vec3(0.0, -0.5, 0.0),
        walkStairsStepUp: vec3(0.0, 0.4, 0.0),
        walkStairsMinStepForward: 0.02,
        walkStairsStepForwardTest: 0.15,
        walkStairsCosAngleForwardContact: 75.0_f32.to_radians().cos(),
        walkStairsStepDownExtra: vec3(0.0, 0.0, 0.0),
    }
}

/// A character in a world with a floor box whose top is at y = 0, plus its own temp allocator.
struct Scene {
    character: *mut JPH_CharacterVirtual,
    temp_allocator: *mut JPH_TempAllocator,
    // Last field: the world is destroyed after the character (see `Drop`).
    _world: TestWorld,
}

impl Scene {
    fn new() -> Self {
        let world = TestWorld::new(1);
        create_box(
            world.body_interface(),
            vec3(100.0, 1.0, 100.0),
            rvec3(0.0, -1.0, 0.0),
            JPH_MotionType_Static,
            OL_NON_MOVING,
            JPH_Activation_DontActivate,
        );
        // SAFETY: Jolt is initialised (`TestWorld::new`).
        let temp_allocator = unsafe { JPH_TempAllocator_Create(1024 * 1024) };
        assert!(!temp_allocator.is_null());
        // SAFETY: the shape is created holding one reference. The settings are filled field by
        // field (`JPH_CharacterVirtualSettings_Init` would leak an empty shape); the character
        // takes its own reference to the shape, so this one is released right after creation.
        // `position` and `rotation` are live locals, and the system is live.
        let character = unsafe {
            let shape = JPH_CapsuleShape_Create(HALF_HEIGHT, RADIUS) as *mut JPH_Shape;
            let settings = JPH_CharacterVirtualSettings {
                base: JPH_CharacterBaseSettings {
                    up: vec3(0.0, 1.0, 0.0),
                    supportingVolume: JPH_Plane {
                        normal: vec3(0.0, 1.0, 0.0),
                        distance: -1.0e10,
                    },
                    maxSlopeAngle: 50.0_f32.to_radians(),
                    enhancedInternalEdgeRemoval: false,
                    shape,
                },
                ID: 1,
                mass: 70.0,
                maxStrength: 100.0,
                shapeOffset: vec3(0.0, 0.0, 0.0),
                backFaceMode: JPH_BackFaceMode_CollideWithBackFaces,
                predictiveContactDistance: 0.1,
                maxCollisionIterations: 5,
                maxConstraintIterations: 15,
                minTimeRemaining: 1.0e-4,
                collisionTolerance: 1.0e-3,
                characterPadding: 0.02,
                maxNumHits: 256,
                hitReductionCosMaxAngle: 0.999,
                penetrationRecoverySpeed: 1.0,
                innerBodyShape: null(),
                innerBodyIDOverride: u32::MAX,
                innerBodyLayer: 0,
            };
            let position = rvec3(0.0, 1.2, 0.0);
            let rotation = quat_identity();
            let character =
                JPH_CharacterVirtual_Create(&settings, &position, &rotation, 0, world.system());
            JPH_Shape_Destroy(shape);
            character
        };
        assert!(!character.is_null());
        Self {
            character,
            temp_allocator,
            _world: world,
        }
    }

    /// One update falling at 2 m/s, with gravity, Jolt's default extended settings and null
    /// (accept-all) filters.
    fn update(&self) {
        let velocity = vec3(0.0, -2.0, 0.0);
        let gravity = vec3(0.0, -9.81, 0.0);
        let settings = default_extended_update_settings();
        // SAFETY: the character and allocator are live; the vectors and settings are live
        // locals. Null filters accept everything. No other thread uses the world.
        unsafe {
            JPH_CharacterVirtual_SetLinearVelocity(self.character, &velocity);
            JPH_CharacterVirtual_ExtendedUpdate2(
                self.character,
                DT,
                &gravity,
                &settings,
                null(),
                null(),
                null(),
                null(),
                self.temp_allocator,
            );
        }
    }

    fn position(&self) -> JPH_RVec3 {
        let mut position = rvec3(0.0, 0.0, 0.0);
        // SAFETY: the character is live and `position` is a live local.
        unsafe { JPH_CharacterVirtual_GetPosition(self.character, &mut position) };
        position
    }

    fn save(&self) -> Vec<u8> {
        // SAFETY: Jolt is initialised; the recorder is destroyed before returning, and `bytes`
        // has exactly the size `CopyData` is told.
        unsafe {
            let recorder = JPH_StateRecorder_Create();
            JPH_CharacterVirtual_SaveState(self.character, recorder);
            let mut bytes = vec![0_u8; JPH_StateRecorder_GetDataSize(recorder)];
            JPH_StateRecorder_CopyData(recorder, bytes.as_mut_ptr().cast(), bytes.len());
            assert!(!JPH_StateRecorder_IsFailed(recorder));
            JPH_StateRecorder_Destroy(recorder);
            bytes
        }
    }

    fn restore(&self, bytes: &[u8]) {
        // SAFETY: the bytes are a complete stream written by `SaveState`, so `RestoreState`
        // reads exactly what was written. The recorder is destroyed before returning.
        unsafe {
            let recorder = JPH_StateRecorder_Create();
            JPH_StateRecorder_WriteBytes(recorder, bytes.as_ptr().cast(), bytes.len());
            JPH_StateRecorder_Rewind(recorder);
            JPH_CharacterVirtual_RestoreState(self.character, recorder);
            assert!(!JPH_StateRecorder_IsFailed(recorder));
            JPH_StateRecorder_Destroy(recorder);
        }
    }

    /// Positions after each of `ticks` updates, as bits, and the saved state after the last.
    // `Real` is already `f64` with the `double-precision` feature.
    #[allow(clippy::useless_conversion)]
    fn run(&self, ticks: usize) -> (Vec<[u64; 3]>, Vec<u8>) {
        let positions = (0..ticks)
            .map(|_| {
                self.update();
                let p = self.position();
                [p.x, p.y, p.z].map(|value| f64::from(value).to_bits())
            })
            .collect();
        (positions, self.save())
    }
}

impl Drop for Scene {
    fn drop(&mut self) {
        // SAFETY: both were created in `new` and are destroyed once, in reverse order of
        // creation, while the world they use is still alive (it drops after this function).
        unsafe {
            JPH_CharacterBase_Destroy(self.character.cast());
            JPH_TempAllocator_Destroy(self.temp_allocator);
        }
    }
}

#[test]
fn character_lands_and_its_restored_state_continues_bit_for_bit() {
    let (saved, continued) = {
        let scene = Scene::new();
        for _ in 0..60 {
            scene.update();
        }
        // SAFETY: the character is live.
        let ground = unsafe { JPH_CharacterBase_GetGroundState(scene.character.cast()) };
        assert_eq!(ground, JPH_GroundState_OnGround);
        let saved = scene.save();
        assert!(!saved.is_empty());
        (saved, scene.run(10))
    };

    // Worlds are built one after the other: the test framework holds a global lock per world.
    let scene = Scene::new();
    scene.restore(&saved);
    assert_eq!(scene.save(), saved, "a restored state saves the same bytes");
    assert_eq!(scene.run(10), continued);
}

#[test]
fn copying_no_bytes_accepts_a_null_buffer() {
    let _world = TestWorld::new(1);
    // SAFETY: Jolt is initialised (the world exists); the recorder is destroyed before
    // returning. An empty recorder copies no bytes, which allows a null buffer.
    unsafe {
        let recorder = JPH_StateRecorder_Create();
        assert_eq!(JPH_StateRecorder_GetDataSize(recorder), 0);
        JPH_StateRecorder_CopyData(recorder, std::ptr::null_mut(), 0);
        JPH_StateRecorder_CopyData(recorder, std::ptr::null_mut(), 16);
        assert!(!JPH_StateRecorder_IsFailed(recorder));
        JPH_StateRecorder_Destroy(recorder);
    }
}
