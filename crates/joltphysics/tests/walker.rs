//! The game's character laws (spec G.4 and G.5) on the reference near step of
//! `common::walker`, which runs on CharacterVirtual.
//!
//! The spec's scenarios were written against another engine. Each test asserts the law the
//! scenario protects; where a number belonged to that engine, the test says what it asserts
//! instead and why.

mod common;

use common::walker::*;
use common::*;
use joltphysics::*;

const SPEED: f64 = 2.0;

fn dt() -> f64 {
    f64::from(DT)
}

/// Ground height of the anchor chunk's flat terrain at `x`, `z`.
fn flat_ground(x: f64, z: f64) -> f64 {
    (R * R - x * x - z * z).sqrt() - R
}

/// The body origin resting on the anchor chunk's flat terrain at `(x, z)`.
fn resting_at(x: f64, z: f64) -> V3 {
    let ground = [x, flat_ground(x, z), z];
    add(ground, scale(up_at(ground), f64::from(REST_HEIGHT)))
}

/// The anchor chunk with flat terrain and a walker resting at `(x, z)`.
fn flat_scene(x: f64, z: f64) -> (PhysicsWorld, Layers, Walker) {
    let (mut world, layers) = fixture_world(1);
    flat_chunk(&mut world, &layers, 0.0);
    let walker = add_walker(&mut world, &layers, resting_at(x, z));
    (world, layers, walker)
}

/// Runs `ticks` near steps from rest, wanting `desired(tick, origin)` each tick.
fn run(
    world: &mut PhysicsWorld,
    walker: &Walker,
    ticks: usize,
    mut desired: impl FnMut(usize, V3) -> V3,
) -> Vec<NearOutput> {
    let mut carry = Carry::RESTING;
    (0..ticks)
        .map(|tick| {
            let wanted = desired(tick, origin(world, walker));
            near_tick(world, walker, &mut carry, wanted)
        })
        .collect()
}

/// Walking along world direction `direction` at `speed` m/s.
fn walking(direction: V3, speed: f64) -> impl FnMut(usize, V3) -> V3 {
    move |_, origin| tangent(origin, direction, speed * dt())
}

fn still(_: usize, _: V3) -> V3 {
    [0.0; 3]
}

/// Height of the body origin above the anchor's flat terrain, along up.
fn height_above_flat(p: V3) -> f64 {
    let ground = [p[0], flat_ground(p[0], p[2]), p[2]];
    dot(sub(p, ground), up_at(ground))
}

/// Length of the polyline through `points`.
fn path_length(points: impl IntoIterator<Item = V3>) -> f64 {
    let points: Vec<V3> = points.into_iter().collect();
    points
        .windows(2)
        .map(|pair| norm(sub(pair[1], pair[0])))
        .sum()
}

/// The error of converting `p` to the world's `Real` positions: one unit in the last place.
fn conversion_error(p: V3) -> f64 {
    let largest = p.iter().fold(0.0_f64, |m, c| m.max(c.abs()));
    largest * f64::from(f32::EPSILON)
}

#[test]
fn walker_rest_height_is_foot_plus_radius_plus_padding() {
    let (mut world, _, walker) = flat_scene(0.0, 0.0);
    let outputs = run(&mut world, &walker, 30, still);
    let last = outputs.last().unwrap();
    assert!(last.grounded);
    // Measured 0.42000001668930054.
    let height = height_above_flat(last.pos);
    assert!((height - f64::from(REST_HEIGHT)).abs() < 1e-3, "{height}");
}

/// G.4 #10, flat chunk: one step along chart x moves exactly speed · dt along x, nothing across,
/// and stays grounded; on the anchor chunk and on a chunk with another orientation.
#[test]
fn a_step_moves_exactly_the_desired_displacement() {
    for angle in [0.0, 0.6] {
        let (mut world, layers) = fixture_world(1);
        flat_chunk(&mut world, &layers, angle);
        let (centre, rotation) = chunk_pose(angle);
        let start = add(v3(centre), scale(up_at(v3(centre)), f64::from(REST_HEIGHT)));
        let walker = add_walker(&mut world, &layers, start);
        let mut carry = Carry::RESTING;
        near_tick(&mut world, &walker, &mut carry, [0.0; 3]);
        let before = origin(&world, &walker);
        // Chart x is the chunk's local x.
        let chart_x = rotate(rotation, [1.0, 0.0, 0.0]);
        let desired = tangent(before, chart_x, SPEED * dt());
        let out = near_tick(&mut world, &walker, &mut carry, desired);
        let moved = sub(out.pos, before);
        let along = normalize(desired);
        let tolerance = 2.0 * conversion_error(before) + 1e-4;
        let error = norm(sub(moved, desired));
        assert!(
            error < tolerance,
            "angle {angle}: moved {moved:?}, wanted {desired:?} ({error} > {tolerance})"
        );
        assert!((dot(moved, along) - SPEED * dt()).abs() < tolerance);
        assert!(out.grounded, "angle {angle}");
    }
}

/// G.4 #10, airborne far from the anchor: a tick in the air falls by g · dt² along -up and is not
/// grounded.
#[test]
fn an_airborne_step_falls_radially() {
    let (mut world, layers) = fixture_world(1);
    let angle: f64 = 1.2;
    let surface = add(CENTRE, [R * angle.sin(), R * angle.cos(), 0.0]);
    let start = add(surface, scale(up_at(surface), 5.0));
    let walker = add_walker(&mut world, &layers, start);
    let mut carry = Carry {
        vel_up: 0.0,
        grounded: false,
    };
    let out = near_tick(&mut world, &walker, &mut carry, [0.0; 3]);
    let expected = scale(up_at(start), -f64::from(G) * dt() * dt());
    let error = norm(sub(sub(out.pos, start), expected));
    assert!(error < 2.0 * conversion_error(start) + 1e-6, "{error}");
    assert!(!out.grounded);
}

/// G.4 #11: a walker dropped onto a floor box whose top is at 0.8 stands on it.
#[test]
fn a_walker_lands_on_a_floor_box() {
    let (mut world, layers, _) = flat_scene(5.0, 5.0);
    structure_box(&mut world, &layers, [0.0, 0.4, 0.0], [2.0, 0.4, 2.0], 0.0);
    let walker = add_walker(&mut world, &layers, [0.0, 0.9 + f64::from(RADIUS), 0.0]);
    let mut carry = Carry {
        vel_up: 0.0,
        grounded: false,
    };
    let mut last = None;
    for _ in 0..30 {
        last = Some(near_tick(&mut world, &walker, &mut carry, [0.0; 3]));
    }
    let out = last.unwrap();
    let feet = out.pos[1] - f64::from(RADIUS);
    assert!((feet - 0.8).abs() < 0.05, "feet at {feet}");
    assert!(out.grounded);
}

/// Walks at 2 m/s from 1 m before a block of `height` (above the ground at its face) for 120
/// ticks; returns the last output and the x of the block's face.
fn walk_into_block(height: f64, step_up: f32) -> (NearOutput, f64) {
    let (mut world, layers) = fixture_world(1);
    flat_chunk(&mut world, &layers, 0.0);
    let face = 1.0 + f64::from(RADIUS);
    let top = flat_ground(face, 0.0) + height;
    structure_box(
        &mut world,
        &layers,
        [face + 2.0, top - 1.5, 0.0],
        [2.0, 1.5, 2.0],
        0.0,
    );
    let mut walker = add_walker(&mut world, &layers, resting_at(0.0, 0.0));
    walker.step_up = step_up;
    let outputs = run(&mut world, &walker, 120, walking([1.0, 0.0, 0.0], SPEED));
    (*outputs.last().unwrap(), face)
}

/// Whether the walker of `walk_into_block` stands on the block.
fn climbed(out: &NearOutput, height: f64, face: f64) -> bool {
    out.pos[1] - f64::from(RADIUS) > flat_ground(face, 0.0) + height - 0.05
}

/// The highest block, to 1 mm, that a walker with walk-stairs step-up `step_up` climbs.
fn max_climbable_block(step_up: f32) -> f64 {
    let (mut low, mut high) = (0.0, 1.5);
    while high - low > 1e-3 {
        let mid = 0.5 * (low + high);
        let (out, face) = walk_into_block(mid, step_up);
        if climbed(&out, mid, face) {
            low = mid;
        } else {
            high = mid;
        }
    }
    low
}

/// G.4 #12, the step law: 0.4 m and 0.45 m are climbed, 0.5 m is not, and the walker stops more
/// than 0.3 m before the face.
///
/// Jolt's step-up is not the game's autostep height: the rounded capsule bottom climbs an edge
/// whose contact normal is within 45° of up on its own, so the highest block climbed is about the
/// step-up plus 0.14 m. `STEP_UP` 0.33 was chosen by measuring that height (0.47 m here).
#[test]
fn step_law_climbs_up_to_045_but_not_05() {
    for height in [0.40, 0.45] {
        let (out, face) = walk_into_block(height, STEP_UP);
        assert!(climbed(&out, height, face), "{height}: {out:?}");
        assert!(out.grounded, "{height}");
    }
    let (out, face) = walk_into_block(0.5, STEP_UP);
    assert!(!climbed(&out, 0.5, face), "{out:?}");
    assert!(
        face - out.pos[0] > 0.3,
        "stopped at {} before the face at {face}",
        out.pos[0]
    );
    assert_eq!(out.blocker.and_then(|b| b.group), Some(Groups::STRUCTURE));

    let highest = max_climbable_block(STEP_UP);
    assert!((0.45..0.5).contains(&highest), "climbs up to {highest}");
}

/// The anchor chunk with `sloped(0, tan)` terrain.
fn slope_scene(tan: f64) -> (PhysicsWorld, Layers) {
    let (mut world, layers) = fixture_world(1);
    add_terrain(
        &mut world,
        &layers,
        &sloped(0.0, tan),
        (RVec3::ZERO, Quat::IDENTITY),
    );
    (world, layers)
}

/// The slope plane of `sloped(0, tan)`: through the origin with this unit normal.
fn slope_normal(tan: f64) -> V3 {
    normalize([-tan, 1.0, 0.0])
}

/// The body origin resting on the slope of `sloped(0, tan)` at `x` (on the slope part).
fn resting_on_slope(x: f64, tan: f64) -> V3 {
    add(
        [x, tan * x, 0.0],
        scale(slope_normal(tan), f64::from(REST_HEIGHT)),
    )
}

/// Distance of the capsule's lower sphere from the slope plane, minus the rest gap.
fn off_slope(p: V3, tan: f64) -> f64 {
    dot(p, slope_normal(tan)) - f64::from(REST_HEIGHT)
}

/// G.4 #13, the slope law: on 30° the walker climbs, projected onto the slope (run at least
/// v · t · cos²30° − 0.05), stays on the slope within 0.03 and grounded; on 60° it makes no
/// headway and slides back.
#[test]
fn slope_law_climbs_30_degrees_and_slides_back_from_60() {
    let tan30 = 30.0_f64.to_radians().tan();
    let (mut world, layers) = slope_scene(tan30);
    let start = resting_on_slope(1.0, tan30);
    let walker = add_walker(&mut world, &layers, start);
    let outputs = run(&mut world, &walker, 30, walking([1.0, 0.0, 0.0], SPEED));
    for (tick, out) in outputs.iter().enumerate() {
        assert!(out.grounded, "tick {tick}");
        let off = off_slope(out.pos, tan30);
        assert!(off.abs() < 0.03, "tick {tick}: {off} off the slope");
    }
    let run_x = outputs.last().unwrap().pos[0] - start[0];
    let cos2 = 30.0_f64.to_radians().cos().powi(2);
    assert!(run_x >= SPEED * 0.5 * cos2 - 0.05, "run {run_x}");

    let tan60 = 60.0_f64.to_radians().tan();
    let (mut world, layers) = slope_scene(tan60);
    let start = resting_on_slope(0.3, tan60);
    let walker = add_walker(&mut world, &layers, start);
    let outputs = run(&mut world, &walker, 60, walking([1.0, 0.0, 0.0], SPEED));
    let highest = outputs
        .iter()
        .map(|out| out.pos[1])
        .fold(f64::MIN, f64::max);
    assert!(
        highest <= start[1] + 1e-3,
        "rose to {highest} from {}",
        start[1]
    );
    let last = outputs.last().unwrap();
    assert!(last.pos[0] < start[0], "slid back: {:?}", last.pos);
}

/// G.4 #14: walking down a 30° slope the walker stays grounded and on the slope, without hops.
#[test]
fn walking_down_a_30_degree_slope_has_no_hops() {
    let tan30 = 30.0_f64.to_radians().tan();
    let (mut world, layers) = slope_scene(tan30);
    let walker = add_walker(&mut world, &layers, resting_on_slope(4.0, tan30));
    let outputs = run(&mut world, &walker, 40, walking([-1.0, 0.0, 0.0], SPEED));
    for (tick, out) in outputs.iter().enumerate() {
        assert!(out.grounded, "tick {tick}: {out:?}");
        let off = off_slope(out.pos, tan30);
        assert!(off.abs() < 0.03, "tick {tick}: {off} off the slope");
    }
}

/// G.4 #15: walking across the seam between two chunks' heightfields, the walker keeps its rest
/// height and walks the full path.
#[test]
fn a_chunk_seam_is_crossed() {
    let (mut world, layers) = fixture_world(1);
    flat_chunk(&mut world, &layers, 0.0);
    // The neighbour's edge meets the anchor's edge exactly on the sphere.
    flat_chunk(&mut world, &layers, 2.0 * (16.0 / R).asin());
    let start = resting_at(14.0, 0.3);
    let walker = add_walker(&mut world, &layers, start);
    let outputs = run(&mut world, &walker, 120, walking([1.0, 0.0, 0.0], SPEED));
    let crossed = outputs.last().unwrap().pos;
    assert!(crossed[0] > 17.0, "{crossed:?}");
    for (tick, out) in outputs.iter().enumerate() {
        let height = dot(sub(out.pos, CENTRE), up_at(out.pos)) - R;
        assert!(
            (height - f64::from(REST_HEIGHT)).abs() < 0.05,
            "tick {tick}: height {height}"
        );
        assert!(out.grounded, "tick {tick}");
    }
    let path = path_length(std::iter::once(start).chain(outputs.iter().map(|out| out.pos)));
    assert!((path - 4.0).abs() < 0.08, "path {path}");
}

/// G.4 #16: a walker buried 1, 3 or 10 m under the terrain is put back on it in one still tick;
/// one on the surface stays put.
#[test]
fn a_buried_walker_is_put_back_on_the_terrain() {
    for depth in [1.0, 3.0, 10.0] {
        let (mut world, layers) = fixture_world(1);
        flat_chunk(&mut world, &layers, 0.0);
        let buried = add(
            resting_at(0.5, 0.5),
            [0.0, -depth - f64::from(REST_HEIGHT), 0.0],
        );
        let walker = add_walker(&mut world, &layers, buried);
        let mut carry = Carry {
            vel_up: -3.0,
            grounded: false,
        };
        let out = near_tick(&mut world, &walker, &mut carry, [0.0; 3]);
        assert!(out.recovered, "depth {depth}");
        let height = height_above_flat(out.pos);
        assert!(
            (height - f64::from(REST_HEIGHT)).abs() < 0.05,
            "depth {depth}: height {height}"
        );
        assert!(out.grounded, "depth {depth}");
    }

    let (mut world, _, walker) = flat_scene(0.5, 0.5);
    let start = origin(&world, &walker);
    let mut carry = Carry::RESTING;
    let out = near_tick(&mut world, &walker, &mut carry, [0.0; 3]);
    assert!(!out.recovered);
    assert!(norm(sub(out.pos, start)) < 0.01);
}

/// A walker at the anchor overlapping a wall of half extents (0.1, 1, 2) centred at x 0.45 by
/// 0.05 m; returns its x after `ticks` near steps wanting `desired` each tick.
fn pushed_from_wall(desired: V3, ticks: usize) -> Vec<f64> {
    let (mut world, layers, walker) = flat_scene(0.0, 0.0);
    structure_box(&mut world, &layers, [0.45, 1.0, 0.0], [0.1, 1.0, 2.0], 0.0);
    let start = origin(&world, &walker)[0];
    run(&mut world, &walker, ticks, move |_, _| desired)
        .iter()
        .map(|out| out.pos[0] - start)
        .collect()
}

/// G.4 #17: a walker overlapping a wall is pushed out by more than 0.04 m, with and without
/// motion.
///
/// The spec's engine pushed out before the move and only without motion; here Jolt's
/// penetration recovery pushes within the move (recovery speed 1 resolves the overlap at once).
/// The test asserts the push, not when it happens: within the first tick.
#[test]
fn an_overlapping_wall_pushes_the_walker_out() {
    for desired in [[0.0, 0.0, SPEED * dt()], [0.0; 3]] {
        let xs = pushed_from_wall(desired, 10);
        assert!(xs[0] < -0.04, "{desired:?}: {xs:?}");
        assert!(xs.iter().all(|&x| x < -0.04), "{desired:?}: {xs:?}");
    }
}

/// G.4 #18: on a floor rotated with the planet far from the anchor, the walker walks the full
/// path along it.
#[test]
fn a_rotated_floor_far_from_the_anchor_keeps_the_path() {
    let (mut world, layers) = fixture_world(1);
    let angle = 1.0;
    let (centre, rotation) = chunk_pose(angle);
    let floor = Shape::new_box(Vec3::new(8.0, 0.5, 8.0)).unwrap();
    let floor_centre = sub(v3(centre), scale(up_at(v3(centre)), 0.5));
    world
        .create_body(
            &floor,
            &BodySettings::new_static()
                .position(rvec3(floor_centre))
                .rotation(rotation)
                .object_layer(layers.chunk),
        )
        .unwrap();
    let start = add(v3(centre), scale(up_at(v3(centre)), f64::from(REST_HEIGHT)));
    let walker = add_walker(&mut world, &layers, start);
    let chart_x = rotate(rotation, [1.0, 0.0, 0.0]);
    let outputs = run(&mut world, &walker, 120, walking(chart_x, SPEED));
    assert!(outputs.iter().all(|out| out.grounded));
    let path = path_length(std::iter::once(start).chain(outputs.iter().map(|out| out.pos)));
    assert!((path - 4.0).abs() < 0.08, "path {path}");
}

/// G.4 #19: a walker that only touches a floor is not pushed.
#[test]
fn touching_a_floor_is_not_overlapping() {
    let (mut world, layers, _) = flat_scene(5.0, 5.0);
    structure_box(&mut world, &layers, [0.0, 0.4, 0.0], [2.0, 0.4, 2.0], 0.0);
    let start = [0.0, 0.8 + f64::from(REST_HEIGHT), 0.0];
    let walker = add_walker(&mut world, &layers, start);
    let mut carry = Carry::RESTING;
    let out = near_tick(&mut world, &walker, &mut carry, [0.0; 3]);
    let push = norm(sub(out.pos, start));
    assert!(push < 0.05, "pushed {push}");
}

/// G.4 #20: with no input, on 60° terrain the walker is not grounded and slides every tick,
/// faster and faster; on 30° it stands.
#[test]
fn steep_terrain_slides_and_gentle_terrain_holds() {
    let tan60 = 60.0_f64.to_radians().tan();
    let (mut world, layers) = slope_scene(tan60);
    let walker = add_walker(&mut world, &layers, resting_on_slope(6.0, tan60));
    let mut positions = vec![origin(&world, &walker)];
    let outputs = run(&mut world, &walker, 40, still);
    positions.extend(outputs.iter().map(|out| out.pos));
    let mut last_step = 0.0;
    for (tick, (out, pair)) in outputs.iter().zip(positions.windows(2)).enumerate() {
        assert!(!out.grounded && out.sliding, "tick {tick}: {out:?}");
        let downhill = pair[0][0] - pair[1][0];
        assert!(
            downhill >= last_step - 1e-4,
            "tick {tick}: {downhill} after {last_step}"
        );
        last_step = downhill;
        if tick == 30 {
            assert!(downhill > 0.02, "tick {tick}: {downhill}");
            assert!(out.vel_up < -1.0, "tick {tick}: {}", out.vel_up);
        }
    }

    let tan30 = 30.0_f64.to_radians().tan();
    let (mut world, layers) = slope_scene(tan30);
    let start = resting_on_slope(3.0, tan30);
    let walker = add_walker(&mut world, &layers, start);
    let outputs = run(&mut world, &walker, 60, still);
    for (tick, out) in outputs.iter().enumerate() {
        assert!(out.grounded && !out.sliding, "tick {tick}");
        let moved = norm(sub(out.pos, start));
        assert!(moved < 1e-3, "tick {tick}: moved {moved}");
    }
}

/// G.4 #21: on 50°, 60° and 75° terrain the walker never climbs, never sticks high and reaches
/// the bottom, from four start points each.
#[test]
fn steep_slopes_always_bring_the_walker_down() {
    for degrees in [50.0_f64, 60.0, 75.0] {
        let tan = degrees.to_radians().tan();
        for (x, z) in [(1.0, -3.0), (2.0, -1.0), (3.0, 1.0), (4.0, 3.0)] {
            let (mut world, layers) = slope_scene(tan);
            let start = add(resting_on_slope(x, tan), [0.0, 0.0, z]);
            let walker = add_walker(&mut world, &layers, start);
            let mut previous = start;
            let outputs = run(&mut world, &walker, 600, still);
            for (tick, out) in outputs.iter().enumerate() {
                let rise = dot(sub(out.pos, previous), up_at(previous));
                assert!(
                    rise <= 0.02,
                    "{degrees}° from x {x}, tick {tick}: rose {rise}"
                );
                previous = out.pos;
            }
            let last = outputs.last().unwrap();
            assert!(
                last.pos[0] < 0.5,
                "{degrees}° from x {x}: stuck at {:?}",
                last.pos
            );
            let height = height_above_flat(last.pos);
            assert!(
                height < f64::from(REST_HEIGHT) + 0.05,
                "{degrees}° from x {x}: {height}"
            );
            assert!(last.grounded, "{degrees}° from x {x}");
        }
    }
}

/// G.5 #22: jogging at 3.5 m/s for 60 ticks covers a chord of 3.5 m and stays grounded.
#[test]
fn jogging_covers_the_chord() {
    let (mut world, _, walker) = flat_scene(0.0, 0.0);
    let start = origin(&world, &walker);
    let outputs = run(&mut world, &walker, 60, walking([1.0, 0.0, 0.0], 3.5));
    assert!(outputs.iter().all(|out| out.grounded));
    let end = outputs.last().unwrap().pos;
    let chord = norm(sub(end, start));
    let tolerance = 2.0 * conversion_error(end) + 1e-3;
    assert!(
        (chord - 3.5).abs() < tolerance,
        "chord {chord} (tolerance {tolerance})"
    );
}

/// G.5 #23: a jump's apex is the discrete ballistic sum and the walker lands.
#[test]
fn a_jump_reaches_the_ballistic_apex_and_lands() {
    let (mut world, _, walker) = flat_scene(0.0, 0.0);
    let start = origin(&world, &walker);
    let mut carry = Carry {
        vel_up: 4.5,
        grounded: true,
    };
    let mut apex = 0.0_f64;
    let mut landed = false;
    for _ in 0..90 {
        let out = near_tick(&mut world, &walker, &mut carry, [0.0; 3]);
        apex = apex.max(dot(sub(out.pos, start), up_at(start)));
        landed = out.grounded;
    }
    // The feed subtracts g · dt before the move, so the first tick rises by (4.5 − g · dt) · dt:
    // k starts at 1.
    let g_dt = f64::from(G) * dt();
    let ballistic: f64 = (1..)
        .map(|k| (4.5 - k as f64 * g_dt) * dt())
        .take_while(|&rise| rise > 0.0)
        .sum();
    assert!(
        (apex - ballistic).abs() < 0.005,
        "apex {apex}, ballistic {ballistic}"
    );
    assert!(landed);
}

/// G.5 #24: released in the air, the horizontal velocity decays by air friction only.
#[test]
fn released_input_decays_by_air_friction() {
    let (mut world, _, walker) = flat_scene(0.0, 0.0);
    let mut player = Player::new(&mut world, &walker);
    let ahead = [1.0, 0.0, 0.0];
    for _ in 0..20 {
        player.tick(&mut world, &walker, ahead, 3.5, false);
    }
    // Jump while running, then release the input.
    player.tick(&mut world, &walker, ahead, 3.5, true);
    let mut horizontal = Vec::new();
    for _ in 0..40 {
        let out = player.tick(&mut world, &walker, [0.0; 3], 3.5, false);
        if out.grounded {
            break;
        }
        horizontal.push(norm(player.horizontal_velocity()));
    }
    assert!(horizontal.len() > 20, "{horizontal:?}");
    for pair in horizontal.windows(2) {
        let ratio = pair[1] / pair[0];
        assert!((ratio - 0.98).abs() < 1e-3, "{ratio} in {horizontal:?}");
    }
}

/// G.5 #25: a running jump at a 60° rise after flat ground reaches beyond the rise's start and
/// never sinks into the terrain.
#[test]
fn a_running_jump_at_a_steep_rise_gets_past_its_start() {
    let tan60 = 60.0_f64.to_radians().tan();
    let (mut world, layers) = slope_scene(tan60);
    let walker = add_walker(&mut world, &layers, [-4.0, f64::from(REST_HEIGHT), 0.0]);
    let mut player = Player::new(&mut world, &walker);
    let mut furthest = f64::MIN;
    let mut jumped = false;
    for _ in 0..120 {
        let jump = !jumped && player.last.pos[0] > -1.2;
        let out = player.tick(&mut world, &walker, [1.0, 0.0, 0.0], 3.5, jump);
        jumped |= jump;
        furthest = furthest.max(out.pos[0]);
        let feet = sub(out.pos, scale(up_at(out.pos), f64::from(RADIUS)));
        let terrain = tan60 * feet[0].clamp(0.0, 12.0);
        assert!(
            feet[1] >= terrain - 1e-3,
            "feet {feet:?} below terrain {terrain}"
        );
    }
    assert!(jumped);
    assert!(furthest > 0.0, "reached x {furthest}");
}

/// D.2 rule 8: rising into a ceiling resets vel_up, and the walker falls back and lands.
#[test]
fn rising_into_a_ceiling_resets_vel_up() {
    let (mut world, layers, walker) = flat_scene(0.0, 0.0);
    let start = origin(&world, &walker);
    // The capsule top is CENTRE_UP + HALF_HEIGHT + RADIUS above the origin.
    let head = start[1] + f64::from(CENTRE_UP + HALF_HEIGHT + RADIUS);
    structure_box(
        &mut world,
        &layers,
        [0.0, head + 0.3 + 0.25, 0.0],
        [2.0, 0.25, 2.0],
        0.0,
    );
    let mut carry = Carry {
        vel_up: 4.5,
        grounded: true,
    };
    let mut hit = None;
    let mut landed = false;
    for tick in 0..90 {
        let out = near_tick(&mut world, &walker, &mut carry, [0.0; 3]);
        if out.ceiling && hit.is_none() {
            assert_eq!(out.vel_up, 0.0, "tick {tick}");
            hit = Some(tick);
        }
        if hit.is_some() && out.grounded {
            landed = true;
            break;
        }
    }
    assert!(hit.is_some(), "no ceiling contact");
    assert!(landed);
}

/// Runs the script for `ticks` ticks in blocks of `block` ticks. Each block builds the scene
/// afresh (same bodies, same ids), creates the walker, restores the character state and the
/// game's carried state of the previous block, and runs on. `block == ticks` is the continuous
/// run. Returns the digest of every tick.
fn scripted_run(ticks: usize, block: usize) -> Vec<Vec<u8>> {
    let (mut world, layers) = script_scene(1);
    let walker = add_walker(&mut world, &layers, script_start());
    let mut player = Player::new(&mut world, &walker);
    let mut state = world.character(walker.id).unwrap().save_state();
    drop(world);

    let mut records = Vec::with_capacity(ticks);
    for first in (0..ticks).step_by(block) {
        let (mut world, layers) = script_scene(1);
        let walker = add_walker(&mut world, &layers, player.last.pos);
        world
            .character_mut(walker.id)
            .unwrap()
            .restore_state(&state)
            .unwrap();
        for tick in first..(first + block).min(ticks) {
            let out = script_tick(&mut world, &walker, &mut player, tick);
            let mut record = Vec::new();
            record_walker(&world, &walker, &out, &mut record);
            records.push(record);
        }
        state = world.character(walker.id).unwrap().save_state();
    }
    records
}

/// Chaining the game's near steps from saved state equals one continuous run, bit for bit, for
/// blocks of 1 and of 7 ticks: position, velocity, vel_up, grounded, sliding, the character
/// state and the contacts of every tick.
#[test]
fn chained_near_step_replay_is_bit_exact() {
    const TICKS: usize = 300;
    let continuous = scripted_run(TICKS, TICKS);
    for block in [1, 7] {
        let chained = scripted_run(TICKS, block);
        for (tick, (a, b)) in continuous.iter().zip(&chained).enumerate() {
            assert!(a == b, "blocks of {block}: tick {tick} differs");
        }
        assert_eq!(chained.len(), TICKS);
    }
}
