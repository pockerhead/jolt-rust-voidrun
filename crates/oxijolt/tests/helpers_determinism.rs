//! Determinism of the physics helpers: a scene on a plane floor runs bit for bit the same with 1
//! and 4 worker threads, in one process and in two.

mod common;

use common::determinism::{assert_same, child_request, digest_in_child, finish_child, Digest};
use common::*;
use oxijolt::*;

const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);
const TICKS: usize = 240;

/// A plane floor and a grid of mixed bodies dropped onto it.
struct Scene {
    world: PhysicsWorld,
    bodies: Vec<BodyId>,
}

/// The shapes the grid cycles through: box, sphere, hull and compound.
fn mixed_shapes() -> Vec<Shape> {
    let block = Shape::new_box(Vec3::new(0.3, 0.2, 0.25)).unwrap();
    let ball = Shape::new_sphere(0.25).unwrap();
    let hull = Shape::new_convex_hull(
        &[
            Vec3::new(-0.3, 0.0, -0.3),
            Vec3::new(0.3, 0.0, -0.3),
            Vec3::new(0.0, 0.0, 0.3),
            Vec3::new(0.0, 0.5, 0.0),
        ],
        0.02,
    )
    .unwrap();
    let compound = Shape::new_compound(&[
        CompoundChild {
            shape: &block,
            position: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            user_data: 0,
        },
        CompoundChild {
            shape: &ball,
            position: Vec3::new(0.4, 0.2, 0.0),
            rotation: Quat::IDENTITY,
            user_data: 1,
        },
    ])
    .unwrap();
    vec![block, ball, hull, compound]
}

impl Scene {
    fn new(threads: u32) -> Self {
        let mut world = world(GRAVITY, threads);
        let floor = world
            .create_body(
                &Shape::new_plane(Vec3::new(0.0, 1.0, 0.0), 0.0, 30.0).unwrap(),
                &BodySettings::new_static(),
            )
            .unwrap();
        let mut bodies = vec![floor];
        let shapes = mixed_shapes();
        for i in 0..16 {
            let (column, row) = ((i % 4) as Real, (i / 4) as Real);
            let tilt = quat_about(Vec3::new(0.6, 0.0, 0.8), 0.3 * i as f32);
            bodies.push(
                world
                    .create_body(
                        &shapes[i % shapes.len()],
                        &BodySettings::new_dynamic()
                            .position(RVec3::new(
                                1.2 * column - 1.8,
                                1.0 + 0.4 * row,
                                1.2 * row - 1.8,
                            ))
                            .rotation(tilt)
                            .angular_velocity(Vec3::new(0.0, 1.0, 0.0)),
                    )
                    .unwrap(),
            );
        }
        Self { world, bodies }
    }

    fn tick(&mut self) {
        step(&mut self.world, 1);
    }

    fn record(&mut self, digest: &mut Digest) {
        let tick = digest.push();
        for &id in &self.bodies {
            record_body(&self.world, id, &mut tick.state);
        }
    }
}

/// The scene run for [`TICKS`] ticks with `threads` workers.
fn helpers(threads: u32) -> Digest {
    let mut scene = Scene::new(threads);
    let mut digest = Digest::new();
    for _ in 0..TICKS {
        scene.tick();
        scene.record(&mut digest);
    }
    digest
}

#[test]
#[ignore = "child process of the helpers determinism gate"]
fn helpers_determinism_child() {
    let Some((scenario, threads, _)) = child_request() else {
        return;
    };
    assert_eq!(scenario, "helpers");
    finish_child(&helpers(threads));
}

#[test]
fn helpers_match_with_1_and_4_workers() {
    let one = helpers(1);
    assert_eq!(one.ticks.len(), TICKS);
    assert_same("1 vs 4 workers in one process", &one, &helpers(4));
    assert_ne!(one.ticks[0].state, one.ticks[TICKS - 1].state);
}

#[test]
fn helpers_match_across_processes() {
    let one = digest_in_child("helpers_determinism_child", "helpers", 1, "");
    let four = digest_in_child("helpers_determinism_child", "helpers", 4, "");
    assert_eq!(one.ticks.len(), TICKS);
    assert_same("1 vs 4 workers in two processes", &one, &four);
    assert_same("one process vs a child", &helpers(1), &one);
}
