//! Seeded stress of the shape constructors: random inputs per shape family, a named expectation
//! where the outcome is defined, and collision use of a share of the accepted shapes.
//!
//! The cases run in a child process (the ignored test `shape_stress_child`), so a Jolt assertion
//! (asserts builds abort on it) or a crash fails the parent test, which names the last case the
//! child started. Every case is independent of the worker count and of the other cases' outcomes.

mod common;

use std::io::Write as _;
use std::process::Command;

use common::*;
use oxijolt::*;

const CHILD: &str = "shape_stress_child";

#[test]
fn shapes_survive_seeded_stress() {
    let output = Command::new(std::env::current_exe().unwrap())
        .args([
            CHILD,
            "--exact",
            "--ignored",
            "--nocapture",
            "--test-threads=1",
        ])
        .output()
        .unwrap();
    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr);
        let last = stderr
            .lines()
            .rev()
            .find(|line| line.starts_with("case "))
            .unwrap_or("before the first case");
        // With --nocapture the child's panic message is on stderr between the case lines.
        let messages: Vec<&str> = stderr
            .lines()
            .filter(|line| !line.starts_with("case "))
            .collect();
        let messages = messages.join("\n");
        let tail = &messages[messages.len().saturating_sub(4000)..];
        panic!(
            "the stress child failed ({}) at {last}\nstderr:\n{tail}",
            output.status
        );
    }
}

#[test]
#[ignore = "run in its own process by shapes_survive_seeded_stress"]
fn shape_stress_child() {
    let mut arena = Arena::new();
    hull_family(&mut arena);
}

/// Announces a case on stderr before it runs, so a crash can be traced to it.
fn announce(family: &str, index: usize) {
    eprintln!("case {family} {index}");
    std::io::stderr().flush().unwrap();
}

/// SplitMix64: a fixed-seed generator, so every run sees the same cases.
struct Rng(u64);

impl Rng {
    fn new(seed: u64) -> Self {
        Self(seed)
    }

    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Uniform in `[0, 1)`.
    fn unit(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    fn range(&mut self, low: f64, high: f64) -> f64 {
        low + (high - low) * self.unit()
    }

    /// Log-uniform in `[low, high)`, both positive.
    fn log_range(&mut self, low: f64, high: f64) -> f64 {
        self.range(low.ln(), high.ln()).exp()
    }

    /// Uniform in `low..high`.
    fn below(&mut self, low: usize, high: usize) -> usize {
        low + (self.next_u64() % (high - low) as u64) as usize
    }

    /// A uniformly random rotation (Shoemake's method) as a row-major matrix.
    fn rotation(&mut self) -> [[f64; 3]; 3] {
        let (u1, u2, u3) = (self.unit(), self.unit(), self.unit());
        let tau = std::f64::consts::TAU;
        let (a, b) = ((1.0 - u1).sqrt(), u1.sqrt());
        let (x, y, z, w) = (
            a * (tau * u2).sin(),
            a * (tau * u2).cos(),
            b * (tau * u3).sin(),
            b * (tau * u3).cos(),
        );
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - w * z),
                2.0 * (x * z + w * y),
            ],
            [
                2.0 * (x * y + w * z),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - w * x),
            ],
            [
                2.0 * (x * z - w * y),
                2.0 * (y * z + w * x),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ]
    }
}

fn rotate(m: &[[f64; 3]; 3], p: [f64; 3]) -> [f64; 3] {
    [0, 1, 2].map(|row| m[row][0] * p[0] + m[row][1] * p[1] + m[row][2] * p[2])
}

// `Real` is `f32` without the `double-precision` feature.
#[allow(clippy::useless_conversion)]
fn real(value: f32) -> Real {
    Real::from(value)
}

fn to_vec3(p: [f64; 3]) -> Vec3 {
    Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32)
}

/// Where a shape's input geometry lies in shape space: lowest and highest y and the centroid.
#[derive(Clone, Copy, Debug)]
struct Extent {
    min_y: f32,
    max_y: f32,
    centroid: Vec3,
}

impl Extent {
    fn of(points: &[Vec3]) -> Self {
        let count = points.len() as f64;
        let sum = points.iter().fold([0.0f64; 3], |s, p| {
            [
                s[0] + f64::from(p.x),
                s[1] + f64::from(p.y),
                s[2] + f64::from(p.z),
            ]
        });
        Self {
            min_y: points.iter().map(|p| p.y).fold(f32::INFINITY, f32::min),
            max_y: points.iter().map(|p| p.y).fold(f32::NEG_INFINITY, f32::max),
            centroid: to_vec3(sum.map(|c| c / count)),
        }
    }

    fn height(&self) -> f32 {
        self.max_y - self.min_y
    }
}

/// One world reused by every case, with a static floor whose top face is at y = 0.
struct Arena {
    world: PhysicsWorld,
}

impl Arena {
    fn new() -> Self {
        let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
        let floor = Shape::new_box(Vec3::new(2000.0, 1.0, 2000.0)).unwrap();
        world
            .create_body(
                &floor,
                &BodySettings::new_static().position(RVec3::new(0.0, -1.0, 0.0)),
            )
            .unwrap();
        Self { world }
    }

    /// Drops `shape` as a dynamic body with its lowest point 0.5 m above the floor, when the
    /// world accepts it as dynamic, and steps `ticks` times; the body must stay finite.
    fn drop_dynamic(&mut self, shape: &Shape, extent: Extent, ticks: usize, what: &str) {
        let position = RVec3::new(
            -real(extent.centroid.x),
            real(0.5 - extent.min_y),
            -real(extent.centroid.z),
        );
        let settings = BodySettings::new_dynamic().position(position);
        let Ok(id) = self.world.create_body(shape, &settings) else {
            return;
        };
        for _ in 0..ticks {
            assert!(self.world.step(DT).unwrap().is_complete());
        }
        assert_finite_body(&self.world, id, what);
        self.world.remove_body(id).unwrap();
    }

    /// Adds `shape` as a static body at the origin and runs a down-ray and a capsule overlap
    /// through the centroid of its input geometry; both must report finite results.
    fn query_static(&mut self, shape: &Shape, extent: Extent, what: &str) {
        let id = self
            .world
            .create_body(shape, &BodySettings::new_static())
            .unwrap();
        let c = extent.centroid;
        let top = real(extent.max_y) + 1.0;
        let ray = RayCast::new(
            RVec3::new(real(c.x), top, real(c.z)),
            Vec3::new(0.0, -(extent.height() + 2.0), 0.0),
        );
        if let Some(hit) = self.world.cast_ray(ray, &QueryFilter::new()).unwrap() {
            let y = ray.point_at(hit.fraction).y;
            assert!(
                y.is_finite() && hit.normal.y.is_finite(),
                "{what}: ray hit {hit:?}"
            );
        }
        let capsule = Shape::new_capsule(0.5, 0.2).unwrap();
        let centre = RVec3::new(real(c.x), real(c.y), real(c.z));
        let query = CollideShape::new(&capsule, centre, Quat::IDENTITY);
        for hit in self
            .world
            .collide_shape(&query, &QueryFilter::new())
            .unwrap()
        {
            assert!(
                hit.penetration_depth.is_finite() && hit.normal.x.is_finite(),
                "{what}: overlap {hit:?}"
            );
        }
        self.world.remove_body(id).unwrap();
    }
}

fn assert_finite_body(world: &PhysicsWorld, id: BodyId, what: &str) {
    let body = world.body(id).unwrap();
    let position: [Real; 3] = body.position().into();
    let rotation: [f32; 4] = body.rotation().into();
    let linear: [f32; 3] = body.linear_velocity().into();
    let angular: [f32; 3] = body.angular_velocity().into();
    assert!(
        position.iter().all(|v| v.is_finite())
            && rotation
                .iter()
                .chain(&linear)
                .chain(&angular)
                .all(|v| v.is_finite()),
        "{what}: body state is not finite"
    );
}

/// The kinds of point clouds the hull family draws from.
#[derive(Clone, Copy, Debug)]
enum Cloud {
    /// Fewer than four points.
    TooFew,
    /// A box of random aspect ratios.
    Box,
    /// Points on a sphere surface, more than Jolt keeps as hull vertices.
    Sphere,
    /// Points in an axis plane, exactly.
    ExactPlane,
    /// Points close to a plane.
    NearPlane,
    /// Points on an axis line, exactly.
    ExactLine,
    /// Points close to a line.
    NearLine,
    /// A few points repeated many times.
    Duplicates,
}

const CLOUDS: [Cloud; 7] = [
    Cloud::Box,
    Cloud::Sphere,
    Cloud::ExactPlane,
    Cloud::NearPlane,
    Cloud::ExactLine,
    Cloud::NearLine,
    Cloud::Duplicates,
];

/// A hull point cloud of `kind`, in shape space.
fn cloud(rng: &mut Rng, kind: Cloud) -> Vec<Vec3> {
    let size = rng.log_range(1.0e-4, 1.0e3);
    let translation = [0; 3].map(|_| rng.range(-1000.0, 1000.0));
    let rotation = rng.rotation();
    let place = |p: [f64; 3]| {
        let r = rotate(&rotation, p);
        to_vec3([0, 1, 2].map(|i| r[i] * size + translation[i]))
    };
    match kind {
        Cloud::TooFew => (0..rng.below(0, 4))
            .map(|_| place([0; 3].map(|_| rng.range(-1.0, 1.0))))
            .collect(),
        Cloud::Box => {
            let aspect = [0; 3].map(|_| rng.log_range(1.0e-4, 1.0));
            (0..rng.below(4, 301))
                .map(|_| place([0, 1, 2].map(|i| aspect[i] * rng.range(-1.0, 1.0))))
                .collect()
        }
        Cloud::Sphere => (0..rng.below(260, 301))
            .map(|_| {
                let z = rng.range(-1.0, 1.0);
                let angle = rng.range(0.0, std::f64::consts::TAU);
                let r = (1.0 - z * z).sqrt();
                place([r * angle.cos(), r * angle.sin(), z])
            })
            .collect(),
        Cloud::ExactPlane => {
            // No rotation or scaling across the plane, so the y values stay exactly equal.
            let y = rng.range(-1000.0, 1000.0) as f32;
            let size = size as f32;
            (0..rng.below(4, 60))
                .map(|_| {
                    Vec3::new(
                        size * rng.range(-1.0, 1.0) as f32,
                        y,
                        size * rng.range(-1.0, 1.0) as f32,
                    )
                })
                .collect()
        }
        Cloud::NearPlane => {
            let offset = rng.log_range(1.0e-7, 1.0e-2);
            (0..rng.below(4, 120))
                .map(|_| {
                    place([
                        rng.range(-1.0, 1.0),
                        offset * rng.range(-1.0, 1.0),
                        rng.range(-1.0, 1.0),
                    ])
                })
                .collect()
        }
        Cloud::ExactLine => {
            let (y, z) = (
                rng.range(-1000.0, 1000.0) as f32,
                rng.range(-1000.0, 1000.0) as f32,
            );
            let size = size as f32;
            (0..rng.below(4, 40))
                .map(|_| Vec3::new(size * rng.range(-1.0, 1.0) as f32, y, z))
                .collect()
        }
        Cloud::NearLine => {
            let offset = rng.log_range(1.0e-9, 1.0e-3);
            (0..rng.below(4, 60))
                .map(|_| {
                    place([
                        rng.range(-1.0, 1.0),
                        offset * rng.range(-1.0, 1.0),
                        offset * rng.range(-1.0, 1.0),
                    ])
                })
                .collect()
        }
        Cloud::Duplicates => {
            let distinct: Vec<[f64; 3]> = (0..rng.below(1, 7))
                .map(|_| [0; 3].map(|_| rng.range(-1.0, 1.0)))
                .collect();
            (0..rng.below(4, 80))
                .map(|_| place(distinct[rng.below(0, distinct.len())]))
                .collect()
        }
    }
}

/// The outcome the hull rules define for `kind`, when they define one.
fn expected_hull_error(kind: Cloud, points: &[Vec3]) -> Option<&'static [HullError]> {
    let within = points.iter().all(|p| {
        [p.x, p.y, p.z]
            .iter()
            .all(|c| c.abs() <= limits::MAX_SHAPE_EXTENT)
    });
    match kind {
        Cloud::TooFew => Some(&[HullError::TooFewPoints]),
        Cloud::ExactLine if within => Some(&[HullError::Degenerate]),
        // A tiny plane is also smaller than Jolt's minimum initial triangle.
        Cloud::ExactPlane if within => Some(&[HullError::Coplanar, HullError::Degenerate]),
        _ => None,
    }
}

const HULL_CASES: usize = 600;

fn hull_family(arena: &mut Arena) {
    let mut rng = Rng::new(0x5EED_0001);
    let mut accepted = 0;
    for index in 0..HULL_CASES {
        announce("hull", index);
        let kind = if index % 50 == 0 {
            Cloud::TooFew
        } else {
            CLOUDS[index % CLOUDS.len()]
        };
        let points = cloud(&mut rng, kind);
        let radius = if index % 3 == 0 { 0.0 } else { 0.05 };
        let what = format!("hull {index} ({kind:?}, {} points)", points.len());
        let result = Shape::new_convex_hull(&points, radius);
        if let Some(allowed) = expected_hull_error(kind, &points) {
            assert!(
                matches!(result, Err(ShapeError::ConvexHull(error)) if allowed.contains(&error)),
                "{what}: {:?}, expected one of {allowed:?}",
                result.err()
            );
            continue;
        }
        let shape = match result {
            Ok(shape) => shape,
            Err(error) => {
                eprintln!("{what}: {error}");
                continue;
            }
        };
        accepted += 1;
        if accepted % 6 == 0 {
            let extent = Extent::of(&points);
            arena.drop_dynamic(&shape, extent, 30, &what);
            arena.query_static(&shape, extent, &what);
        }
    }
    eprintln!("hull: {accepted} of {HULL_CASES} accepted");
    assert!(
        accepted > HULL_CASES / 5,
        "only {accepted} hulls were accepted"
    );
}
