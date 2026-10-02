//! A leak gate for the joltc objects joltphysics creates per call: the three query filters and
//! the inflated shape of a shape cast with a target distance, created every round (100 000
//! measured rounds against a 4 MiB threshold), and the body write lock a rebase takes for every
//! moving body. Rebases run only every 100th round, about 1000 times, so a per-rebase leak below
//! about 4 KiB, a leaked body lock included, stays under the threshold; the gate catches only
//! larger ones.
//!
//! It measures the private bytes of the process (Windows `K32GetProcessMemoryInfo`), because
//! these objects are allocated by C++, which a Rust global allocator does not see. The file
//! holds exactly one test, so its binary runs alone and no parallel test disturbs the counter.
#![cfg(windows)]

mod common;

use common::memory::private_bytes;
use common::*;
use joltphysics::*;

fn child(shape: &Shape, position: Vec3, user_data: u32) -> CompoundChild<'_> {
    CompoundChild {
        shape,
        position,
        rotation: Quat::IDENTITY,
        user_data,
    }
}

const WARM_UP_ROUNDS: usize = 20_000;
const MEASURED_ROUNDS: usize = 100_000;
const MAX_GROWTH: usize = 4 * 1024 * 1024;

#[test]
fn per_call_joltc_objects_do_not_leak() {
    let (mut world, [terrain, chunk, _, item, actor]) = five_layer_world();
    let porch = Shape::new_box(Vec3::new(2.0, 0.5, 2.0)).unwrap();
    let canopy = Shape::new_cylinder(0.5, 2.0).unwrap();
    let house = Shape::new_compound(&[
        child(&porch, Vec3::new(0.0, 0.5, 0.0), Groups::STRUCTURE),
        child(&canopy, Vec3::new(0.0, 3.0, 0.0), Groups::FEATURE),
    ])
    .unwrap();
    let capsule = Shape::new_capsule(0.70845, 0.4).unwrap();
    let ball = Shape::new_sphere(0.5).unwrap();
    let ids = [
        add_static_in(&mut world, &house, RVec3::ZERO, chunk),
        add_static_in(&mut world, &flat_height_field(), RVec3::ZERO, terrain),
        add_static_in(&mut world, &capsule, RVec3::new(0.0, 1.5, 0.0), actor),
        // A spinning body, so every rebase also writes a velocity under a body lock.
        world
            .create_body(
                &ball,
                &BodySettings::new_dynamic()
                    .position(RVec3::new(50.0, 10.0, 50.0))
                    .angular_velocity(Vec3::new(0.0, 1.0, 0.0))
                    .object_layer(item),
            )
            .unwrap(),
    ];
    let actor_body = ids[2];
    // Object layers, child groups and an excluded body: every query creates all three filters.
    let layers = [terrain, chunk, actor];
    let filter = QueryFilter::new()
        .object_layers(&layers)
        .child_groups(1 << Groups::TERRAIN | 1 << Groups::STRUCTURE | 1 << Groups::FEATURE)
        .exclude_body(actor_body);
    let down = Vec3::new(0.0, -10.0, 0.0);
    let turn_about_y = |angle: f32| quat_about(Vec3::new(0.0, 1.0, 0.0), angle);

    let mut rebases = 0;
    let mut run_rounds = |world: &mut PhysicsWorld, rounds: usize| {
        let mut hits = 0;
        for i in 0..rounds {
            let x = (i % 7) as Real * 0.1;
            let ray = RayCast::new(RVec3::new(x, 5.0, 0.0), down);
            hits += usize::from(world.cast_ray(ray, &filter).unwrap().is_some());
            let cast = ShapeCast::new(&capsule, RVec3::new(x, 6.0, 0.0), Quat::IDENTITY, down)
                .target_distance(0.02);
            hits += usize::from(world.cast_shape(&cast, &filter).unwrap().is_some());
            let overlap = CollideShape::new(&capsule, RVec3::new(x, 1.2, 0.0), Quat::IDENTITY);
            hits += world.collide_shape(&overlap, &filter).unwrap().len();
            if i % 100 == 0 {
                // A real, small turn that the next rebase undoes, so the scene stays in place.
                // Only about 1000 rebases run, too few to catch a leak below about 4 KiB each,
                // such as a body lock.
                let angle = if rebases % 2 == 0 { 0.1 } else { -0.1 };
                world
                    .rebase(&ids, turn_about_y(angle), RVec3::ZERO)
                    .unwrap();
                rebases += 1;
            }
        }
        hits
    };

    assert!(run_rounds(&mut world, WARM_UP_ROUNDS) > 0);
    let before = private_bytes();
    assert!(run_rounds(&mut world, MEASURED_ROUNDS) > 0);
    let after = private_bytes();
    let growth = after.saturating_sub(before);
    eprintln!("private bytes before {before}, after {after}, growth {growth}");
    assert!(
        growth < MAX_GROWTH,
        "private bytes grew by {growth} (before {before}, after {after}): a per-call joltc object leaks"
    );
}
