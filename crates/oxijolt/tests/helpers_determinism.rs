//! Determinism of the physics helpers: a scene on a plane floor, with point queries every tick,
//! runs bit for bit the same with 1 and 4 worker threads, in one process and in two.

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
    /// How many point query hits the run recorded.
    point_hits: usize,
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
        Self {
            world,
            bodies,
            point_hits: 0,
        }
    }

    fn tick(&mut self) {
        step(&mut self.world, 1);
    }

    fn record(&mut self, digest: &mut Digest) {
        let tick = digest.push();
        for &id in &self.bodies {
            record_body(&self.world, id, &mut tick.state);
        }
        // Points where the first two grid rows land, and the last one below the floor.
        for i in 0..8 {
            let y = if i == 7 { -0.5 } else { 0.15 };
            let point = RVec3::new(1.2 * (i % 4) as Real - 1.8, y, 1.2 * (i / 4) as Real - 1.8);
            let hits = self
                .world
                .collide_point(point, &QueryFilter::new())
                .unwrap();
            self.point_hits += hits.len();
            for hit in hits {
                tick.state.extend(
                    format!("{}:{};", hit.body.to_raw(), hit.sub_shape_id.to_raw()).bytes(),
                );
            }
            tick.state.push(b'|');
        }
    }
}

/// The scene run for [`TICKS`] ticks with `threads` workers, and the point query hits it saw.
fn run(threads: u32) -> (Digest, usize) {
    let mut scene = Scene::new(threads);
    let mut digest = Digest::new();
    for _ in 0..TICKS {
        scene.tick();
        scene.record(&mut digest);
    }
    (digest, scene.point_hits)
}

fn helpers(threads: u32) -> Digest {
    run(threads).0
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
    let (one, point_hits) = run(1);
    assert_eq!(one.ticks.len(), TICKS);
    assert_same("1 vs 4 workers in one process", &one, &helpers(4));
    assert_ne!(one.ticks[0].state, one.ticks[TICKS - 1].state);
    // More than the floor below every tick: the points find landed bodies too.
    assert!(point_hits > TICKS, "{point_hits}");
}

#[test]
fn helpers_match_across_processes() {
    let one = digest_in_child("helpers_determinism_child", "helpers", 1, "");
    let four = digest_in_child("helpers_determinism_child", "helpers", 4, "");
    assert_eq!(one.ticks.len(), TICKS);
    assert_same("1 vs 4 workers in two processes", &one, &four);
    assert_same("one process vs a child", &helpers(1), &one);
}
