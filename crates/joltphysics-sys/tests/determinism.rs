//! Same-machine determinism witness: the per-tick state of a scene must be
//! bit-identical whether Jolt steps it with 1 or 4 worker threads, and the
//! order in which bodies are created must decide their BodyIDs.
//!
//! Each run happens in its own child process (this test binary, running the
//! ignored `determinism_child` test), so no state leaks between runs.

mod framework;

use std::path::Path;
use std::process::Command;

use framework::*;
use joltphysics_sys::*;

const CHILD_ENV: &str = "JOLTPHYSICS_SYS_DIGEST_CHILD";
const TICKS: usize = 120;
const COLUMNS: usize = 5;
const LAYERS: usize = 4;

/// Bytes recorded per body per tick: id, position, rotation, linear and
/// angular velocity, and the active flag.
const BODY_RECORD_SIZE: usize = 4 + 3 * size_of::<Real>() + 4 * 4 + 3 * 4 + 3 * 4 + 1;
/// Bodies in the scene: the floor plus the boxes.
const BODY_COUNT: usize = 1 + COLUMNS * LAYERS;

/// What a body is created as.
struct BoxSpec {
    half_extent: JPH_Vec3,
    position: JPH_RVec3,
    motion_type: JPH_MotionType,
    layer: JPH_ObjectLayer,
    activation: JPH_Activation,
}

/// The floor and five columns of four boxes, 1.5 m apart. Each layer is shifted 0.15 m, less
/// than the half extent, so the columns stand and settle as five independent islands that the
/// job system may solve on different threads.
fn scene() -> Vec<BoxSpec> {
    let mut specs = vec![BoxSpec {
        half_extent: vec3(100.0, 1.0, 100.0),
        position: rvec3(0.0, -1.0, 0.0),
        motion_type: JPH_MotionType_Static,
        layer: OL_NON_MOVING,
        activation: JPH_Activation_DontActivate,
    }];
    for column in 0..COLUMNS {
        for layer in 0..LAYERS {
            let x = 1.5 * column as Real + 0.15 * layer as Real;
            let y = 0.5 + 1.05 * layer as Real;
            specs.push(BoxSpec {
                half_extent: vec3(0.5, 0.5, 0.5),
                position: rvec3(x, y, 0.0),
                motion_type: JPH_MotionType_Dynamic,
                layer: OL_MOVING,
                activation: JPH_Activation_Activate,
            });
        }
    }
    specs
}

/// Runs the scene and returns the little-endian state of every body after
/// every tick. Bodies are created in scene order, or in reverse when
/// `reversed`, and always recorded in scene order.
fn run_scene(worker_threads: i32, reversed: bool) -> Vec<u8> {
    let world = TestWorld::new(worker_threads);
    let bodies = world.body_interface();

    let mut specs = scene();
    if reversed {
        specs.reverse();
    }
    let mut ids: Vec<JPH_BodyID> = specs
        .iter()
        .map(|spec| {
            create_box(
                bodies,
                spec.half_extent,
                spec.position,
                spec.motion_type,
                spec.layer,
                spec.activation,
            )
        })
        .collect();
    if reversed {
        ids.reverse();
    }

    let mut digest = Vec::with_capacity(TICKS * BODY_COUNT * BODY_RECORD_SIZE);
    for _ in 0..TICKS {
        world.step(1.0 / 60.0);
        for &id in &ids {
            record_body(bodies, id, &mut digest);
        }
    }

    for &id in &ids {
        // SAFETY: `id` is a body of the live `world`, removed once.
        unsafe { JPH_BodyInterface_RemoveAndDestroyBody(bodies, id) };
    }
    digest
}

/// Appends one body's state to `digest`.
fn record_body(bodies: *mut JPH_BodyInterface, id: JPH_BodyID, digest: &mut Vec<u8>) {
    let mut position = rvec3(0.0, 0.0, 0.0);
    let mut rotation = quat_identity();
    let mut linear = vec3(0.0, 0.0, 0.0);
    let mut angular = vec3(0.0, 0.0, 0.0);
    // SAFETY: `bodies` belongs to a live world that contains `id`, and every
    // out-pointer refers to a live local.
    let active = unsafe {
        JPH_BodyInterface_GetPosition(bodies, id, &mut position);
        JPH_BodyInterface_GetRotation(bodies, id, &mut rotation);
        JPH_BodyInterface_GetLinearVelocity(bodies, id, &mut linear);
        JPH_BodyInterface_GetAngularVelocity(bodies, id, &mut angular);
        JPH_BodyInterface_IsActive(bodies, id)
    };

    digest.extend_from_slice(&id.to_le_bytes());
    for value in [position.x, position.y, position.z] {
        digest.extend_from_slice(&value.to_bits().to_le_bytes());
    }
    for value in [
        rotation.x, rotation.y, rotation.z, rotation.w, linear.x, linear.y, linear.z, angular.x,
        angular.y, angular.z,
    ] {
        digest.extend_from_slice(&value.to_bits().to_le_bytes());
    }
    digest.push(u8::from(active));
}

/// The BodyIDs of the first tick, in scene order.
fn first_tick_ids(digest: &[u8]) -> Vec<JPH_BodyID> {
    digest[..BODY_COUNT * BODY_RECORD_SIZE]
        .as_chunks::<BODY_RECORD_SIZE>()
        .0
        .iter()
        .map(|record| JPH_BodyID::from_le_bytes(record[..4].try_into().unwrap()))
        .collect()
}

/// The BodyID Jolt gives the `n`-th body added to a fresh system: index `n`
/// in the low 23 bits and sequence number 1 above them (Jolt `BodyID.h`).
fn nth_body_id(n: usize) -> JPH_BodyID {
    (1 << 23) | n as JPH_BodyID
}

#[test]
#[ignore = "child process of digest_is_identical_across_thread_counts"]
fn determinism_child() {
    let Ok(request) = std::env::var(CHILD_ENV) else {
        return;
    };
    let mut parts = request.splitn(3, ',');
    let threads: i32 = parts.next().unwrap().parse().unwrap();
    let reversed = match parts.next().unwrap() {
        "forward" => false,
        "reversed" => true,
        order => panic!("unknown order {order}"),
    };
    let output = parts.next().unwrap();
    std::fs::write(output, run_scene(threads, reversed)).unwrap();
}

/// Runs the scene in a child process and returns its digest.
fn digest_in_child(threads: i32, order: &str, output: &Path) -> Vec<u8> {
    let status = Command::new(std::env::current_exe().unwrap())
        .args([
            "determinism_child",
            "--exact",
            "--ignored",
            "--test-threads=1",
        ])
        .env(CHILD_ENV, format!("{threads},{order},{}", output.display()))
        .status()
        .unwrap();
    assert!(status.success(), "child {threads},{order} failed: {status}");
    std::fs::read(output).unwrap()
}

#[test]
fn digest_is_identical_across_thread_counts() {
    let dir = std::env::temp_dir().join(format!("joltphysics-sys-digest-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();

    let one_thread = digest_in_child(1, "forward", &dir.join("a"));
    let four_threads = digest_in_child(4, "forward", &dir.join("b"));
    let reversed = digest_in_child(1, "reversed", &dir.join("c"));
    std::fs::remove_dir_all(&dir).unwrap();

    assert_eq!(one_thread.len(), TICKS * BODY_COUNT * BODY_RECORD_SIZE);
    if let Some(offset) = one_thread
        .iter()
        .zip(&four_threads)
        .position(|(a, b)| a != b)
    {
        let tick = offset / (BODY_COUNT * BODY_RECORD_SIZE);
        panic!("1-thread and 4-thread digests differ at byte {offset} (tick {tick})");
    }
    assert_eq!(one_thread.len(), four_threads.len());

    // Insertion order is part of the state: the n-th created body gets the
    // n-th BodyID, so reversing creation reverses the IDs of the same bodies.
    let created_forward: Vec<JPH_BodyID> = (0..BODY_COUNT).map(nth_body_id).collect();
    let created_reversed: Vec<JPH_BodyID> = (0..BODY_COUNT).rev().map(nth_body_id).collect();
    assert_eq!(first_tick_ids(&one_thread), created_forward);
    assert_eq!(first_tick_ids(&reversed), created_reversed);
}
