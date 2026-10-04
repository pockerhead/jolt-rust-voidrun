//! A leak gate for the debug renderer that `PhysicsWorld::debug_lines` creates and destroys on
//! every call: 2000 measured calls into one reused buffer against a 4 MiB threshold. Each
//! renderer builds Jolt's shared debug geometry (spheres, boxes, capsules, cylinders), so one
//! leaked renderer per call would grow the process far past the threshold. The scene holds a
//! heightfield and a mesh, whose own debug geometry Jolt caches in the shape on the first draw.
//!
//! It measures the private bytes of the process (Windows `K32GetProcessMemoryInfo`), because
//! the renderer is allocated by C++, which a Rust global allocator does not see. The file holds
//! exactly one test, so its binary runs alone and no parallel test disturbs the counter.
#![cfg(all(windows, feature = "debug-renderer"))]

mod common;

use common::memory::private_bytes;
use common::*;
use oxijolt::*;

/// Enough to build the heightfield's and the mesh's cached debug geometry and settle the
/// allocator.
const WARM_UP_CALLS: usize = 200;
const MEASURED_CALLS: usize = 2000;
const MAX_GROWTH: usize = 4 * 1024 * 1024;

#[test]
fn debug_lines_do_not_leak() {
    let mut scene = wireframe_scene();
    // A static mesh near the origin: Jolt builds its debug geometry once, on the first draw.
    let (vertices, triangles) = common::meshes::grid(8, 1.0, |x, z| 0.1 * (x + z).sin());
    let mesh = Shape::new_mesh(&vertices, &triangles).unwrap();
    scene
        .world
        .create_body(
            &mesh,
            &BodySettings::new_static().position(RVec3::new(-8.0, 0.5, -8.0)),
        )
        .unwrap();
    let settings = DebugLineSettings::new(RVec3::ZERO, 20.0);
    let filter = QueryFilter::new();
    let mut lines = DebugLines::new();
    let mut run_calls = |calls: usize| {
        let mut total = 0;
        for _ in 0..calls {
            scene
                .world
                .debug_lines(&settings, &filter, &mut lines)
                .unwrap();
            total += lines.lines().len();
        }
        total
    };

    assert!(run_calls(WARM_UP_CALLS) > 0);
    let before = private_bytes();
    assert!(run_calls(MEASURED_CALLS) > 0);
    let after = private_bytes();
    let growth = after.saturating_sub(before);
    eprintln!("private bytes before {before}, after {after}, growth {growth}");
    assert!(
        growth < MAX_GROWTH,
        "private bytes grew by {growth} (before {before}, after {after}): the debug renderer leaks"
    );
}
