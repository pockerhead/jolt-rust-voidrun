//! Thin dynamic hulls on a floor (`docs/limits.md`, "Thin dynamic hulls on a floor"): flat
//! cones and domes 0.4 to 7 cm thick and up to 14 m wide rest and sleep when they are set down;
//! dropped tilted, the edge that slaps down after they tip over sinks into a box floor and
//! passes through a mesh floor, because Jolt's discrete step does not sweep rotation.

mod common;

use common::meshes::flat_grid;
use common::*;
use oxijolt::*;

/// A flat cone of radius `radius` and height `height`: 400 points on its rim at y = 0 and its
/// apex; a dome adds nine rings of 100 points under a paraboloid cap.
fn flat_cone(radius: f32, height: f32, dome: bool) -> Vec<Vec3> {
    let ring = |r: f32, y: f32, count: usize| {
        (0..count).map(move |i| {
            let angle = i as f32 / count as f32 * std::f32::consts::TAU;
            Vec3::new(r * angle.cos(), y, r * angle.sin())
        })
    };
    let mut points: Vec<Vec3> = ring(radius, 0.0, 400).collect();
    if dome {
        for step in 1..10 {
            let r = radius * (1.0 - step as f32 / 10.0);
            points.extend(ring(r, height * (1.0 - (r / radius).powi(2)), 100));
        }
    }
    points.push(Vec3::new(0.0, height, 0.0));
    points
}

/// `v` turned by the unit quaternion `q`.
fn rotate(q: Quat, v: Vec3) -> Vec3 {
    let t = [
        2.0 * (q.y * v.z - q.z * v.y),
        2.0 * (q.z * v.x - q.x * v.z),
        2.0 * (q.x * v.y - q.y * v.x),
    ];
    Vec3::new(
        v.x + q.w * t[0] + (q.y * t[2] - q.z * t[1]),
        v.y + q.w * t[1] + (q.z * t[0] - q.x * t[2]),
        v.z + q.w * t[2] + (q.x * t[1] - q.y * t[0]),
    )
}

// `Real` is `f32` without the `double-precision` feature.
#[allow(clippy::unnecessary_cast)]
fn f32_of(value: Real) -> f32 {
    value as f32
}

/// The lowest of `points` (shape space) on body `id`, world y.
fn lowest(world: &PhysicsWorld, id: BodyId, points: &[Vec3]) -> f32 {
    let body = world.body(id).unwrap();
    let (position, rotation) = (body.position(), body.rotation());
    points
        .iter()
        .map(|&p| f32_of(position.y) + rotate(rotation, p).y)
        .fold(f32::MAX, f32::min)
}

/// Twelve uniformly random rotations from a SplitMix64 sequence seeded with 7.
fn tilts() -> Vec<Quat> {
    let mut state = 7u64;
    let mut unit = || {
        state = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        ((z ^ (z >> 31)) >> 40) as f32 / (1u64 << 24) as f32
    };
    (0..12)
        .map(|_| {
            let (u1, u2, u3) = (
                unit(),
                unit() * std::f32::consts::TAU,
                unit() * std::f32::consts::TAU,
            );
            let (a, b) = ((1.0 - u1).sqrt(), u1.sqrt());
            Quat::from_xyzw(a * u2.sin(), a * u2.cos(), b * u3.sin(), b * u3.cos())
        })
        .collect()
}

/// Drops the hull of `points` turned by `tilt` from 3 m onto `floor`, a static body placed at
/// `floor_y` so that its top is at y = 0, for `seconds`, and returns the lowest any point got
/// and the lowest point at the end, metres.
fn drop_tilted(
    points: &[Vec3],
    tilt: Quat,
    (floor, floor_y): (&Shape, Real),
    seconds: usize,
) -> (f32, f32) {
    let turned: Vec<Vec3> = points.iter().map(|&p| rotate(tilt, p)).collect();
    let hull = Shape::new_convex_hull(&turned, 0.0).unwrap();
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    world
        .create_body(
            floor,
            &BodySettings::new_static().position(RVec3::new(0.0, floor_y, 0.0)),
        )
        .unwrap();
    let id = world
        .create_body(
            &hull,
            &BodySettings::new_dynamic().position(RVec3::new(0.0, 3.0, 0.0)),
        )
        .unwrap();
    let mut deepest = f32::MAX;
    for _ in 0..seconds * 60 {
        step(&mut world, 1);
        deepest = deepest.min(lowest(&world, id, &turned));
    }
    (deepest, lowest(&world, id, &turned))
}

#[test]
fn thin_hulls_set_down_rest_and_sleep() {
    let half_turn = Quat::from_xyzw(1.0, 0.0, 0.0, 0.0);
    for (radius, height) in [(0.5, 0.004), (2.0, 0.01), (7.0, 0.03), (7.0, 0.07)] {
        for dome in [false, true] {
            for (turn, lift) in [(Quat::IDENTITY, 0.01), (half_turn, height + 0.01)] {
                let points: Vec<Vec3> = flat_cone(radius, height, dome)
                    .into_iter()
                    .map(|p| rotate(turn, p))
                    .collect();
                let hull = Shape::new_convex_hull(&points, 0.05).unwrap();
                let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
                add_floor(&mut world);
                let position = RVec3::new(0.0, lift as Real, 0.0);
                let id = world
                    .create_body(&hull, &BodySettings::new_dynamic().position(position))
                    .unwrap();
                let mut deepest = f32::MAX;
                for _ in 0..360 {
                    step(&mut world, 1);
                    deepest = deepest.min(lowest(&world, id, &points));
                }
                let case = format!("radius {radius}, height {height}, dome {dome}, turn {turn:?}");
                assert!(deepest >= -1.0e-3, "{case}: sank to {deepest} m");
                assert!(world.body(id).unwrap().is_sleeping(), "{case}: still awake");
            }
        }
    }
}

/// A 1 cm thick dome 4 m wide, dropped from 3 m at twelve tilts, lands on its rim, tips over
/// and slaps down. On a box floor the slapping edge sinks far deeper than the hull is thick
/// (to 24 cm in the measurement) and some drops stay wedged 10 cm deep after 6 s (6 of 12
/// measured on Windows); a flat mesh floor lets most through (11 of 12). The counts depend on
/// the platform's rounding, so the test asserts only that each outcome occurs.
#[test]
fn a_thin_dome_slapping_down_sinks_into_a_box_and_through_a_mesh() {
    let dome = flat_cone(2.0, 0.01, true);
    let floor = Shape::new_box(Vec3::new(100.0, 0.5, 100.0)).unwrap();
    let mesh_floor = flat_grid(40, 1.0);
    let (mut deep, mut wedged, mut through) = (0, 0, 0);
    for tilt in tilts() {
        let (deepest, end) = drop_tilted(&dome, tilt, (&floor, -0.5), 6);
        deep += usize::from(deepest < -0.1);
        wedged += usize::from(end < -0.05);
        let (_, end) = drop_tilted(&dome, tilt, (&mesh_floor, 0.0), 2);
        through += usize::from(end < -1.0);
    }
    println!(
        "box floor: {deep} of 12 sank below 10 cm, {wedged} still below 5 cm after 6 s; mesh floor: {through} of 12 fell through"
    );
    assert!(deep > 0 && wedged > 0 && through > 0);
}
