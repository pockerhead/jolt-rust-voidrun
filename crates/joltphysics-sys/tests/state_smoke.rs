//! Smoke tests for saving and restoring a whole physics system through the joltc additions
//! (`JPH_PhysicsSystem_SaveState`, `JPH_PhysicsSystem_RestoreState`).

mod framework;

use framework::*;
use joltphysics_sys::*;

const DT: f32 = 1.0 / 60.0;

/// A static floor and two spheres falling onto it, side by side.
struct Scene {
    world: TestWorld,
    spheres: [JPH_BodyID; 2],
}

impl Scene {
    fn new() -> Self {
        let world = TestWorld::new(2);
        let bodies = world.body_interface();
        create_box(
            bodies,
            vec3(100.0, 1.0, 100.0),
            rvec3(0.0, -1.0, 0.0),
            JPH_MotionType_Static,
            OL_NON_MOVING,
            JPH_Activation_DontActivate,
        );
        let spheres = [
            create_sphere(bodies, rvec3(-1.0, 3.0, 0.0)),
            create_sphere(bodies, rvec3(1.0, 5.0, 0.0)),
        ];
        Self { world, spheres }
    }

    fn step(&self, ticks: usize) {
        for _ in 0..ticks {
            self.world.step(DT);
        }
    }

    /// Saves every part of the system's state; `bodies` selects the bodies, `None` all.
    fn save(&self, bodies: Option<&[JPH_BodyID]>) -> Vec<u8> {
        let (ids, count) = match bodies {
            Some(ids) => (ids.as_ptr(), ids.len() as u32),
            None => (std::ptr::null(), 0),
        };
        // SAFETY: the system is live and not stepping; `ids` is null or readable for `count`
        // ids. The recorder is destroyed before returning, and `bytes` has exactly the size
        // `CopyData` is told.
        unsafe {
            let recorder = JPH_StateRecorder_Create();
            JPH_PhysicsSystem_SaveState(
                self.world.system(),
                recorder,
                JPH_StateRecorderState_All,
                ids,
                count,
            );
            let mut bytes = vec![0_u8; JPH_StateRecorder_GetDataSize(recorder)];
            JPH_StateRecorder_CopyData(recorder, bytes.as_mut_ptr().cast(), bytes.len());
            assert!(!JPH_StateRecorder_IsFailed(recorder));
            JPH_StateRecorder_Destroy(recorder);
            bytes
        }
    }

    /// Restores `bytes` from [`save`](Self::save) and returns what Jolt reported.
    fn restore(&self, bytes: &[u8]) -> bool {
        // SAFETY: the bytes are a complete stream written by `SaveState` of this system, so
        // `RestoreState` reads exactly what was written; the system is not stepping. The
        // recorder is destroyed before returning.
        unsafe {
            let recorder = JPH_StateRecorder_Create();
            JPH_StateRecorder_WriteBytes(recorder, bytes.as_ptr().cast(), bytes.len());
            JPH_StateRecorder_Rewind(recorder);
            let restored = JPH_PhysicsSystem_RestoreState(self.world.system(), recorder);
            if restored {
                assert!(!JPH_StateRecorder_IsFailed(recorder));
            }
            JPH_StateRecorder_Destroy(recorder);
            restored
        }
    }

    /// Position, rotation, linear and angular velocity of `body`, as bits.
    // `Real` is already `f64` with the `double-precision` feature.
    #[allow(clippy::useless_conversion)]
    fn body_bits(&self, body: JPH_BodyID) -> Vec<u64> {
        let bodies = self.world.body_interface();
        let mut position = rvec3(0.0, 0.0, 0.0);
        let mut rotation = quat_identity();
        let mut linear = vec3(0.0, 0.0, 0.0);
        let mut angular = vec3(0.0, 0.0, 0.0);
        // SAFETY: `body` is a body of the live world and every out pointer is a live local.
        unsafe {
            JPH_BodyInterface_GetPosition(bodies, body, &mut position);
            JPH_BodyInterface_GetRotation(bodies, body, &mut rotation);
            JPH_BodyInterface_GetLinearVelocity(bodies, body, &mut linear);
            JPH_BodyInterface_GetAngularVelocity(bodies, body, &mut angular);
        }
        let mut bits: Vec<u64> = [position.x, position.y, position.z]
            .into_iter()
            .map(|value| f64::from(value).to_bits())
            .collect();
        let floats = [
            rotation.x, rotation.y, rotation.z, rotation.w, linear.x, linear.y, linear.z,
            angular.x, angular.y, angular.z,
        ];
        bits.extend(floats.into_iter().map(|value| u64::from(value.to_bits())));
        bits
    }

    fn all_bits(&self) -> Vec<Vec<u64>> {
        self.spheres.iter().map(|&id| self.body_bits(id)).collect()
    }
}

/// Creates a dynamic sphere of radius 0.5 at `position`, adds it awake and returns its id.
fn create_sphere(body_interface: *mut JPH_BodyInterface, position: JPH_RVec3) -> JPH_BodyID {
    let rotation = quat_identity();
    // SAFETY: `body_interface` belongs to a live `TestWorld`; the arguments are live locals.
    // The shape is created holding one reference; the body takes its own, so releasing ours
    // keeps the shape alive for the body.
    let body = unsafe {
        let shape = JPH_SphereShape_Create(0.5);
        let settings = JPH_BodyCreationSettings_Create3(
            shape as *const JPH_Shape,
            &position,
            &rotation,
            JPH_MotionType_Dynamic,
            OL_MOVING,
        );
        let body =
            JPH_BodyInterface_CreateAndAddBody(body_interface, settings, JPH_Activation_Activate);
        JPH_BodyCreationSettings_Destroy(settings);
        JPH_Shape_Destroy(shape as *mut JPH_Shape);
        body
    };
    assert_ne!(body, u32::MAX, "body creation failed");
    body
}

#[test]
fn full_state_restores_bodies_bit_for_bit() {
    let scene = Scene::new();
    scene.step(10);
    let saved = scene.save(None);

    scene.step(20);
    let first_run = scene.all_bits();

    assert!(scene.restore(&saved));
    scene.step(20);
    assert_eq!(scene.all_bits(), first_run);
}

#[test]
fn a_state_restores_any_number_of_times() {
    let scene = Scene::new();
    scene.step(10);
    let saved = scene.save(None);
    let saved_bits = scene.all_bits();

    for _ in 0..3 {
        scene.step(15);
        assert!(scene.restore(&saved));
        assert_eq!(scene.all_bits(), saved_bits);
        assert_eq!(scene.save(None), saved);
    }
}

#[test]
fn a_selected_body_state_restores_only_that_body() {
    let scene = Scene::new();
    scene.step(10);
    let [first, second] = scene.spheres;
    let partial = scene.save(Some(&[first]));
    assert!(partial.len() < scene.save(None).len());
    let first_saved = scene.body_bits(first);

    scene.step(10);
    let second_now = scene.body_bits(second);
    assert_ne!(scene.body_bits(first), first_saved);

    assert!(scene.restore(&partial));
    assert_eq!(scene.body_bits(first), first_saved);
    assert_eq!(scene.body_bits(second), second_now);
}

// Jolt asserts on this path ("Restoring state for non-existing body"), and the asserts build must
// stay assert-free.
#[cfg(not(feature = "asserts"))]
#[test]
fn a_state_with_a_missing_body_is_not_restored() {
    let scene = Scene::new();
    scene.step(10);
    let saved = scene.save(None);

    // SAFETY: the sphere is a body of the live world, removed once.
    unsafe {
        JPH_BodyInterface_RemoveAndDestroyBody(scene.world.body_interface(), scene.spheres[1])
    };
    assert!(!scene.restore(&saved));
}
