//! Moving the world into a new frame: dropped items fall and rest the same with and without a
//! rebase and land on the ground after one, queries answer the same, and nothing wakes or changes on a no-op or an
//! invalid rebase.
//!
//! Every scenario runs twice with the same calls, A without and B with a rebase, and compares
//! B against A mapped through the same frame change. The mapping is computed here in `f64`,
//! independently of the crate's own math.

mod common;

use common::math::*;
use common::*;
use oxijolt::*;

fn quat4(q: Quat) -> [f64; 4] {
    [q.x, q.y, q.z, q.w].map(f64::from)
}

fn normalize4(q: [f64; 4]) -> [f64; 4] {
    let length = q.iter().map(|c| c * c).sum::<f64>().sqrt();
    q.map(|c| c / length)
}

/// The angle in radians between two rotations, computed stably near zero.
fn angle_between(a: Quat, b: Quat) -> f64 {
    let (a, b) = (normalize4(quat4(a)), normalize4(quat4(b)));
    let dot: f64 = a.iter().zip(&b).map(|(x, y)| x * y).sum();
    let sign = if dot < 0.0 { -1.0 } else { 1.0 };
    let difference: f64 = (0..4).map(|i| (a[i] - sign * b[i]).powi(2)).sum();
    let sum: f64 = (0..4).map(|i| (a[i] + sign * b[i]).powi(2)).sum();
    // The 4D angle between the quaternions is twice this arc tangent, the rotation angle twice
    // the 4D angle.
    4.0 * difference.sqrt().atan2(sum.sqrt())
}

/// A rigid change of frame: `p -> rotation * p + translation`.
#[derive(Clone, Copy, Debug)]
struct Frame {
    rotation: Quat,
    translation: RVec3,
}

impl Frame {
    /// A pure drift of the origin, as a floating origin does most of the time.
    fn drift() -> Self {
        Self {
            rotation: Quat::IDENTITY,
            translation: RVec3::new(-444.0, 0.0, 0.0),
        }
    }

    /// 30 degrees about a tilted axis plus a translation.
    fn tilted() -> Self {
        Self {
            rotation: tilted_rotation(),
            translation: RVec3::new(12.5, -3.0, 7.25),
        }
    }

    fn map_vector_f64(&self, v: V3) -> V3 {
        let [x, y, z, w] = quat4(self.rotation);
        let q = [x, y, z];
        let t = cross(q, v).map(|c| 2.0 * c);
        let u = cross(q, t);
        [
            v[0] + w * t[0] + u[0],
            v[1] + w * t[1] + u[1],
            v[2] + w * t[2] + u[2],
        ]
    }

    fn map_point_f64(&self, p: V3) -> V3 {
        let rotated = self.map_vector_f64(p);
        let t = v3(self.translation);
        [rotated[0] + t[0], rotated[1] + t[1], rotated[2] + t[2]]
    }

    fn map_point(&self, p: RVec3) -> RVec3 {
        rvec3(self.map_point_f64(v3(p)))
    }

    fn map_vector(&self, v: Vec3) -> Vec3 {
        vec3(self.map_vector_f64(f3(v)))
    }

    fn map_rotation(&self, q: Quat) -> Quat {
        let [ax, ay, az, aw] = quat4(self.rotation);
        let [bx, by, bz, bw] = quat4(q);
        let product = normalize4([
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ]);
        Quat::from_xyzw(
            product[0] as f32,
            product[1] as f32,
            product[2] as f32,
            product[3] as f32,
        )
    }
}

fn tilted_rotation() -> Quat {
    let axis = Vec3::new(1.0, 2.0, 0.5);
    let axis = Vec3::new(
        axis.x / length(axis),
        axis.y / length(axis),
        axis.z / length(axis),
    );
    quat_about(axis, 30.0_f32.to_radians())
}

/// Each frame change with the offset its scene is built at: the drifted scene sits far from
/// the origin and is brought back by the rebase.
fn frames() -> [(&'static str, Frame, RVec3); 2] {
    [
        ("drift", Frame::drift(), RVec3::new(444.0, 0.0, 0.0)),
        ("tilted", Frame::tilted(), RVec3::ZERO),
    ]
}

const ALL: QueryFilter<'static> = QueryFilter::new();
const ITEM_MASS: f32 = 1.2;
const GRAVITY: f32 = 9.8;
/// Radius of the planet whose surface the terrain is, centred below the scene.
const PLANET_RADIUS: f64 = 99.0;
/// Height of the terrain surface above the scene origin.
const SURFACE: f64 = 2.0;

/// A dropped item over a heightfield terrain and a chunk, in a five-layer world with no world
/// gravity: the item is pulled towards `centre` by the caller every tick.
struct Scene {
    world: PhysicsWorld,
    terrain: BodyId,
    chunk: BodyId,
    item: BodyId,
    centre: V3,
}

impl Scene {
    /// Every body in key order: terrain, chunk, item.
    fn ids(&self) -> [BodyId; 3] {
        [self.terrain, self.chunk, self.item]
    }

    fn item(&self) -> BodyRef<'_> {
        self.world.body(self.item).unwrap()
    }

    /// Distance of the item origin from the planet centre.
    fn item_radius(&self) -> f64 {
        norm(sub(v3(self.item().position()), self.centre))
    }

    fn item_speed(&self) -> f64 {
        norm(f3(self.item().linear_velocity()))
    }

    /// Pulls the item towards the planet centre with its weight, then steps.
    fn tick(&mut self) {
        let position = v3(self.item().position());
        let towards = sub(self.centre, position);
        let pull = towards.map(|c| f64::from(ITEM_MASS * GRAVITY) * c / norm(towards));
        let mut item = self.world.body_mut(self.item).unwrap();
        item.reset_forces();
        item.add_force(vec3(pull)).unwrap();
        assert!(self.world.step(DT).unwrap().is_complete());
    }

    fn rebase(&mut self, frame: &Frame) {
        let ids = self.ids();
        self.world
            .rebase(&ids, frame.rotation, frame.translation)
            .unwrap();
        self.centre = frame.map_point_f64(self.centre);
    }
}

fn scene(offset: RVec3, item_y: Real) -> Scene {
    let (mut world, [terrain_layer, chunk_layer, _, item_layer, _]) = five_layer_world();
    let samples = [SURFACE as f32; 33 * 33];
    let settings = HeightFieldSettings::default().offset(Vec3::new(-16.0, 0.0, -16.0));
    let height_field = Shape::new_height_field(33, &samples, &settings).unwrap();
    let terrain = add_static_in(&mut world, &height_field, offset, terrain_layer);

    let block = Shape::new_box(Vec3::new(1.0, 0.5, 1.0)).unwrap();
    let pillar = Shape::new_cylinder(1.0, 0.5).unwrap();
    let chunk_shape = Shape::new_compound(&[
        child(&block, Vec3::new(6.0, 2.5, 0.0), Groups::STRUCTURE),
        child(&pillar, Vec3::new(-6.0, 3.0, 0.0), Groups::FEATURE),
    ])
    .unwrap();
    let chunk = world
        .create_body(
            &chunk_shape,
            &BodySettings::new_static()
                .position(offset)
                .rotation(quat_about(Vec3::new(0.0, 1.0, 0.0), 90.0_f32.to_radians()))
                .object_layer(chunk_layer),
        )
        .unwrap();

    let item_shape = Shape::new_box(Vec3::new(0.35, 0.06, 0.04)).unwrap();
    let item = world
        .create_body(
            &item_shape,
            &BodySettings::new_dynamic()
                .position(RVec3::new(offset.x, offset.y + item_y, offset.z))
                .mass(ITEM_MASS)
                .friction(0.8)
                .restitution(0.1)
                .motion_quality(MotionQuality::LinearCast)
                .object_layer(item_layer),
        )
        .unwrap();

    let centre = v3(offset);
    Scene {
        world,
        terrain,
        chunk,
        item,
        centre: [centre[0], centre[1] + SURFACE - PLANET_RADIUS, centre[2]],
    }
}

/// How far B's item is from A's item mapped through `frame`: position, rotation angle, linear
/// and angular velocity.
fn differences(a: &Scene, b: &Scene, frame: &Frame) -> [f64; 4] {
    let (a, b) = (a.item(), b.item());
    [
        norm(sub(v3(b.position()), frame.map_point_f64(v3(a.position())))),
        angle_between(b.rotation(), frame.map_rotation(a.rotation())),
        norm(sub(
            f3(b.linear_velocity()),
            frame.map_vector_f64(f3(a.linear_velocity())),
        )),
        norm(sub(
            f3(b.angular_velocity()),
            frame.map_vector_f64(f3(a.angular_velocity())),
        )),
    ]
}

/// Asserts that B's item equals A's item mapped through `frame`.
fn assert_matches(
    tag: &str,
    tick: usize,
    a: &Scene,
    b: &Scene,
    frame: &Frame,
    tolerances: [f64; 2],
) {
    let [position_tolerance, velocity_tolerance] = tolerances;
    let [position, angle, linear, angular] = differences(a, b, frame);
    let context = format!(
        "{tag}, tick {tick}: dpos {position:e}, angle {angle:e}, dv {linear:e}, dw {angular:e}"
    );
    assert!(position <= position_tolerance, "{context}");
    assert!(angle <= 1e-3, "{context}");
    assert!(linear <= velocity_tolerance, "{context}");
    assert!(angular <= velocity_tolerance, "{context}");
}

/// Ten metres above the surface.
const HIGH: Real = 12.06;
/// One centimetre above the surface.
const LOW: Real = 2.07;

#[test]
fn falling_item_crosses_a_rebase_unchanged() {
    for (tag, frame, offset) in frames() {
        let mut a = scene(offset, HIGH);
        let mut b = scene(offset, HIGH);
        for _ in 0..30 {
            a.tick();
            b.tick();
        }
        assert!(a.item_speed() > 0.5, "{tag}: {}", a.item_speed());
        b.rebase(&frame);
        assert_matches(tag, 30, &a, &b, &frame, [1e-3, 1e-3]);
        for tick in 31..=60 {
            a.tick();
            b.tick();
            assert_matches(tag, tick, &a, &b, &frame, [1e-3, 1e-3]);
        }
    }
}

#[test]
fn resting_item_stays_across_a_rebase() {
    for (tag, frame, offset) in frames() {
        let mut a = scene(offset, LOW);
        let mut b = scene(offset, LOW);
        for _ in 0..180 {
            a.tick();
            b.tick();
        }
        let resting_radius = b.item_radius();
        b.rebase(&frame);
        for tick in 181..=210 {
            a.tick();
            b.tick();
            let speed = b.item_speed();
            let sink = resting_radius - b.item_radius();
            assert!(speed < 0.05, "{tag}, tick {tick}: speed {speed}");
            assert!(sink <= 1e-3, "{tag}, tick {tick}: sink {sink}");
            assert_matches(tag, tick, &a, &b, &frame, [1e-3, 5e-3]);
        }
    }
}

/// The tick after which the landing scenarios rebase, with the item in mid-air.
const LANDING_REBASE_TICK: usize = 30;
/// Ticks the item gets to come to rest after its first contact.
const SETTLE_TICKS: usize = 120;
/// Ticks at the end of the settling window in which the item must stay at rest.
const REST_TICKS: usize = 30;
/// Jolt's default `PhysicsSettings::mPenetrationSlop`, which oxijolt does not change: a
/// resting contact may sink this deep by design.
const PENETRATION_SLOP: f32 = 0.02;

/// The scene without terrain and chunk: an item with nothing to hit.
fn free_item(offset: RVec3, item_y: Real) -> Scene {
    let mut scene = scene(offset, item_y);
    scene.world.remove_body(scene.terrain).unwrap();
    scene.world.remove_body(scene.chunk).unwrap();
    scene
}

/// Asserts that the item rests on the terrain, sunk into it no deeper than the slop.
fn assert_on_the_ground(tag: &str, tick: usize, scene: &Scene) {
    let item = scene.item();
    assert!(
        is_calm(&item),
        "{tag}, tick {tick}: the item is not at rest, speed {}, angular speed {}",
        length(item.linear_velocity()),
        length(item.angular_velocity())
    );
    let shape = Shape::new_box(Vec3::new(0.35, 0.06, 0.04)).unwrap();
    let query =
        CollideShape::new(&shape, item.position(), item.rotation()).max_separation_distance(0.01);
    let hits = scene
        .world
        .collide_shape(&query, &QueryFilter::new().exclude_body(scene.item))
        .unwrap();
    let depth = hits
        .iter()
        .filter(|hit| hit.body == scene.terrain)
        .map(|hit| hit.penetration_depth)
        .fold(f32::NEG_INFINITY, f32::max);
    assert!(
        depth > -0.01,
        "{tag}, tick {tick}: the item is not on the terrain"
    );
    assert!(
        depth <= PENETRATION_SLOP,
        "{tag}, tick {tick}: the item sinks {depth} m into the terrain"
    );
}

/// Translation-only rebases add no error of their own: the rebased run equals, bit for bit,
/// the same scene built directly in the new frame.
#[test]
fn drift_rebase_equals_the_scene_built_in_the_new_frame() {
    let frame = Frame::drift();
    let offset = RVec3::new(444.0, 0.0, 0.0);
    let mut rebased = scene(offset, HIGH);
    let mut built_there = scene(frame.map_point(offset), HIGH);
    for _ in 0..LANDING_REBASE_TICK {
        rebased.tick();
        built_there.tick();
    }
    rebased.rebase(&frame);
    assert_eq!(rebased.centre, built_there.centre);
    for tick in LANDING_REBASE_TICK..=LANDING_REBASE_TICK + 180 {
        if tick > LANDING_REBASE_TICK {
            rebased.tick();
            built_there.tick();
        }
        assert!(
            digest(&rebased.world, &rebased.ids())
                == digest(&built_there.world, &built_there.ids()),
            "tick {tick}: the rebased scene differs from the scene built in the new frame"
        );
    }
}

/// An item rebased in mid-air falls like the unrebased one until it first touches the ground;
/// the flat impact of the thin box is chaotic, so afterwards both runs only have to land and
/// come to rest on the terrain.
#[test]
fn landing_after_a_rebase_matches_the_unrebased_run() {
    for (tag, frame, offset) in frames() {
        let mut a = scene(offset, HIGH);
        let mut b = scene(offset, HIGH);
        let mut free = free_item(offset, HIGH);
        for _ in 0..LANDING_REBASE_TICK {
            a.tick();
            b.tick();
            free.tick();
        }
        b.rebase(&frame);
        assert_matches(tag, LANDING_REBASE_TICK, &a, &b, &frame, [1e-3, 5e-3]);

        // Until its first contact, A's item moves exactly like an item with nothing to hit.
        let mut tick = LANDING_REBASE_TICK;
        let first_contact = loop {
            tick += 1;
            a.tick();
            b.tick();
            free.tick();
            if digest(&a.world, &[a.item]) != digest(&free.world, &[free.item]) {
                break tick;
            }
            assert_matches(tag, tick, &a, &b, &frame, [1e-3, 5e-3]);
            assert!(tick < 300, "{tag}: the item never touches the ground");
        };
        assert!(
            first_contact > LANDING_REBASE_TICK + 30,
            "{tag}: first contact at tick {first_contact}, too close to the rebase"
        );

        let mut divergence = [0.0_f64; 4];
        for tick in first_contact..=first_contact + SETTLE_TICKS {
            if tick > first_contact {
                a.tick();
                b.tick();
            }
            for (worst, now) in divergence.iter_mut().zip(differences(&a, &b, &frame)) {
                *worst = worst.max(now);
            }
            if tick > first_contact + SETTLE_TICKS - REST_TICKS {
                assert_on_the_ground(&format!("{tag}, unrebased"), tick, &a);
                assert_on_the_ground(&format!("{tag}, rebased"), tick, &b);
            }
        }
        let [position, angle, linear, angular] = divergence;
        println!(
            "{tag}: first contact at tick {first_contact}; largest divergence after it: dpos {position:e}, angle {angle:e}, dv {linear:e}, dw {angular:e}"
        );
    }
}

fn down(origin: RVec3, x: Real, z: Real) -> RayCast {
    RayCast::new(
        RVec3::new(origin.x + x, origin.y + 20.0, origin.z + z),
        Vec3::new(0.0, -40.0, 0.0),
    )
}

#[test]
fn rays_answer_the_same_across_a_rebase() {
    for (tag, frame, offset) in frames() {
        let mut scene = scene(offset, LOW);
        for _ in 0..60 {
            scene.tick();
        }
        let rays = [
            ("terrain", down(offset, 3.0, 3.0)),
            ("chunk box", down(offset, 0.0, -6.0)),
            ("chunk cylinder", down(offset, 0.0, 6.0)),
            ("item", down(offset, 0.0, 0.0)),
            (
                "oblique terrain",
                RayCast::new(
                    RVec3::new(offset.x - 5.0, offset.y + 10.0, offset.z - 5.0),
                    Vec3::new(4.0, -12.0, 2.0),
                ),
            ),
        ];
        let before: Vec<_> = rays
            .iter()
            .map(|(name, ray)| {
                let hit = scene.world.cast_ray(ray, &ALL).unwrap();
                hit.unwrap_or_else(|| panic!("{tag}, {name}: no hit before the rebase"))
            })
            .collect();
        let expected_bodies = [
            scene.terrain,
            scene.chunk,
            scene.chunk,
            scene.item,
            scene.terrain,
        ];
        for ((name, _), (hit, body)) in rays.iter().zip(before.iter().zip(expected_bodies)) {
            assert_eq!(hit.body, body, "{tag}, {name}");
        }

        scene.rebase(&frame);
        for optimized in [false, true] {
            if optimized {
                scene.world.optimize_broad_phase();
            }
            for ((name, ray), old) in rays.iter().zip(&before) {
                let mapped = RayCast::new(
                    frame.map_point(ray.origin()),
                    frame.map_vector(ray.direction()),
                );
                let context = format!("{tag}, {name}, optimized {optimized}");
                let hit = scene
                    .world
                    .cast_ray(&mapped, &ALL)
                    .unwrap()
                    .unwrap_or_else(|| panic!("{context}: no hit after the rebase"));
                assert_eq!(hit.body, old.body, "{context}");
                assert_eq!(hit.compound_child, old.compound_child, "{context}");
                assert!(
                    (hit.distance - old.distance).abs() <= 1e-3,
                    "{context}: {hit:?} {old:?}"
                );
                let point = norm(sub(
                    v3(mapped.point_at(hit.fraction)),
                    frame.map_point_f64(v3(ray.point_at(old.fraction))),
                ));
                assert!(point <= 1e-3, "{context}: point off by {point}");
                let normal = norm(sub(f3(hit.normal), frame.map_vector_f64(f3(old.normal))));
                assert!(normal <= 1e-3, "{context}: normal off by {normal}");
            }
        }
    }
}

fn assert_vector_near(what: &str, actual: Vec3, expected: Vec3, tolerance: f64) {
    let error = norm(sub(f3(actual), f3(expected)));
    assert!(error <= tolerance, "{what}: {actual:?} vs {expected:?}");
}

fn assert_pose_mapped(
    what: &str,
    world: &PhysicsWorld,
    id: BodyId,
    before: (RVec3, Quat),
    frame: &Frame,
) {
    let body = world.body(id).unwrap();
    let position = norm(sub(v3(body.position()), frame.map_point_f64(v3(before.0))));
    assert!(position <= 1e-4, "{what}: position off by {position}");
    let angle = angle_between(body.rotation(), frame.map_rotation(before.1));
    assert!(angle <= 1e-5, "{what}: rotation off by {angle}");
}

fn pose(world: &PhysicsWorld, id: BodyId) -> (RVec3, Quat) {
    let body = world.body(id).unwrap();
    (body.position(), body.rotation())
}

#[test]
fn rebase_maps_static_and_kinematic_bodies_and_rotates_gravity() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    let block = Shape::new_box(Vec3::new(1.0, 0.5, 2.0)).unwrap();
    let static_box = world
        .create_body(
            &block,
            &BodySettings::new_static()
                .position(RVec3::new(1.0, 2.0, 3.0))
                .rotation(quat_about(Vec3::new(1.0, 0.0, 0.0), 20.0_f32.to_radians())),
        )
        .unwrap();
    let sphere = Shape::new_sphere(0.5).unwrap();
    let kinematic = world
        .create_body(
            &sphere,
            &BodySettings::new_kinematic()
                .position(RVec3::new(5.0, 5.0, 5.0))
                .linear_velocity(Vec3::new(1.0, 0.0, 0.5))
                .angular_velocity(Vec3::new(0.0, 2.0, 0.0)),
        )
        .unwrap();
    let cube = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
    let dynamic = world
        .create_body(
            &cube,
            &BodySettings::new_dynamic()
                .position(RVec3::new(-5.0, 5.0, 0.0))
                .linear_velocity(Vec3::new(0.5, 0.0, 0.0))
                .angular_velocity(Vec3::new(0.3, -1.2, 2.0)),
        )
        .unwrap();
    let ids = [static_box, kinematic, dynamic];
    let poses = ids.map(|id| pose(&world, id));
    let velocities = ids.map(|id| {
        let body = world.body(id).unwrap();
        (body.linear_velocity(), body.angular_velocity())
    });
    let gravity = world.gravity();

    let frame = Frame::tilted();
    world
        .rebase(&ids, frame.rotation, frame.translation)
        .unwrap();

    for ((id, before), name) in ids
        .iter()
        .zip(poses)
        .zip(["static", "kinematic", "dynamic"])
    {
        assert_pose_mapped(name, &world, *id, before, &frame);
    }
    for (index, name) in [(1, "kinematic"), (2, "dynamic")] {
        let body = world.body(ids[index]).unwrap();
        let (linear, angular) = velocities[index];
        assert_vector_near(name, body.linear_velocity(), frame.map_vector(linear), 1e-5);
        assert_vector_near(
            name,
            body.angular_velocity(),
            frame.map_vector(angular),
            1e-5,
        );
        assert!(body.is_active(), "{name}");
    }
    assert_vector_near("gravity", world.gravity(), frame.map_vector(gravity), 1e-5);
}

#[test]
fn sleeping_body_stays_asleep_across_a_rebase() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    let floor = add_floor(&mut world);
    let cube = add_cube(&mut world, RVec3::new(0.0, 0.5, 0.0));
    let mut ticks = 0;
    while !world.body(cube).unwrap().is_sleeping() {
        assert!(ticks < 600, "the cube never fell asleep");
        step(&mut world, 1);
        ticks += 1;
    }
    let before = [floor, cube].map(|id| pose(&world, id));

    let frame = Frame::tilted();
    world
        .rebase(&[floor, cube], frame.rotation, frame.translation)
        .unwrap();
    {
        let body = world.body(cube).unwrap();
        assert!(body.is_sleeping());
        assert_eq!(body.linear_velocity(), Vec3::ZERO);
        assert_eq!(body.angular_velocity(), Vec3::ZERO);
    }
    assert_pose_mapped("floor", &world, floor, before[0], &frame);
    assert_pose_mapped("cube", &world, cube, before[1], &frame);

    let mut after_rebase = Vec::new();
    record_body(&world, cube, &mut after_rebase);
    step(&mut world, 1);
    let mut after_step = Vec::new();
    record_body(&world, cube, &mut after_step);
    assert!(world.body(cube).unwrap().is_sleeping());
    assert_eq!(after_step, after_rebase);
}

#[test]
fn inactive_body_velocity_is_rotated_without_waking() {
    let mut world = world(Vec3::ZERO, 1);
    let shape = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
    let linear = Vec3::new(1.0, 0.0, 0.0);
    let angular = Vec3::new(0.0, 0.0, 1.0);
    let cube = world
        .create_body(
            &shape,
            &BodySettings::new_dynamic()
                .activation(Activation::DontActivate)
                .linear_velocity(linear)
                .angular_velocity(angular),
        )
        .unwrap();
    assert!(!world.body(cube).unwrap().is_active());

    let frame = Frame::tilted();
    world
        .rebase(&[cube], frame.rotation, frame.translation)
        .unwrap();
    let body = world.body(cube).unwrap();
    assert!(!body.is_active());
    assert_vector_near(
        "linear",
        body.linear_velocity(),
        frame.map_vector(linear),
        1e-6,
    );
    assert_vector_near(
        "angular",
        body.angular_velocity(),
        frame.map_vector(angular),
        1e-6,
    );
}

fn digest(world: &PhysicsWorld, ids: &[BodyId]) -> Vec<u8> {
    let mut digest = Vec::new();
    for &id in ids {
        record_body(world, id, &mut digest);
    }
    digest
}

fn gravity_bits(world: &PhysicsWorld) -> [u32; 3] {
    <[f32; 3]>::from(world.gravity()).map(f32::to_bits)
}

#[test]
fn identity_rebase_changes_nothing() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    let ids = build_stacks(&mut world);
    step(&mut world, 30);
    let before = digest(&world, &ids);
    let gravity = gravity_bits(&world);
    world.rebase(&ids, Quat::IDENTITY, RVec3::ZERO).unwrap();
    assert_eq!(digest(&world, &ids), before);
    assert_eq!(gravity_bits(&world), gravity);

    let states = |world: &PhysicsWorld| {
        ids.iter()
            .map(|&id| {
                let body = world.body(id).unwrap();
                let bits = <[f32; 4]>::from(body.rotation())
                    .into_iter()
                    .chain(<[f32; 3]>::from(body.linear_velocity()))
                    .chain(<[f32; 3]>::from(body.angular_velocity()))
                    .map(f32::to_bits)
                    .collect::<Vec<_>>();
                (body.position(), bits, body.is_sleeping())
            })
            .collect::<Vec<_>>()
    };
    let before = states(&world);
    let translation = RVec3::new(5.0, 0.0, -2.0);
    world.rebase(&ids, Quat::IDENTITY, translation).unwrap();
    for ((position, bits, sleeping), (old_position, old_bits, old_sleeping)) in
        states(&world).into_iter().zip(before)
    {
        assert_eq!(bits, old_bits);
        assert_eq!(sleeping, old_sleeping);
        let [x, y, z] = v3(old_position);
        let expected = [x + 5.0, y, z - 2.0];
        let error = norm(sub(v3(position), expected));
        assert!(error <= 1e-4, "{position:?} vs {expected:?}");
    }
    assert_eq!(gravity_bits(&world), gravity);
}

#[test]
fn invalid_rebase_changes_nothing() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    let floor = add_floor(&mut world);
    let first = add_cube(&mut world, RVec3::new(0.0, 0.5, 0.0));
    let second = add_cube(&mut world, RVec3::new(3.0, 2.0, 0.0));
    step(&mut world, 10);
    let ids = [floor, first, second];

    let assert_rejected = |world: &mut PhysicsWorld,
                           ids: &[BodyId],
                           list: &[BodyId],
                           rotation: Quat,
                           translation: RVec3,
                           expected: fn(&BodyError) -> bool| {
        let before = digest(world, ids);
        let gravity = gravity_bits(world);
        let result = world.rebase(list, rotation, translation);
        assert!(
            result.as_ref().is_err_and(expected),
            "{list:?} {rotation:?} {translation:?}: {result:?}"
        );
        assert_eq!(digest(world, ids), before, "{result:?}");
        assert_eq!(gravity_bits(world), gravity, "{result:?}");
    };
    let invalid_value: fn(&BodyError) -> bool = |e| matches!(e, BodyError::InvalidValue(_));
    let tilted = Frame::tilted();

    for rotation in [
        Quat::from_xyzw(0.0, 0.0, 0.0, 2.0),
        Quat::from_xyzw(f32::NAN, 0.0, 0.0, 1.0),
    ] {
        assert_rejected(&mut world, &ids, &ids, rotation, RVec3::ZERO, invalid_value);
    }
    for translation in [
        RVec3::new(Real::NAN, 0.0, 0.0),
        RVec3::new(0.0, Real::INFINITY, 0.0),
    ] {
        assert_rejected(
            &mut world,
            &ids,
            &ids,
            Quat::IDENTITY,
            translation,
            invalid_value,
        );
    }
    let missing_one = [floor, first];
    let twice = [floor, first, first];
    for list in [&missing_one[..], &twice[..]] {
        assert_rejected(
            &mut world,
            &ids,
            list,
            tilted.rotation,
            tilted.translation,
            invalid_value,
        );
    }

    let mut other = common::world(Vec3::ZERO, 1);
    let foreign = add_floor(&mut other);
    assert_rejected(
        &mut world,
        &ids,
        &[floor, first, foreign],
        tilted.rotation,
        tilted.translation,
        |e| matches!(e, BodyError::WrongWorld(_)),
    );

    world.remove_body(second).unwrap();
    let replacement = add_cube(&mut world, RVec3::new(3.0, 2.0, 0.0));
    assert_eq!(replacement.index(), second.index());
    let ids = [floor, first, replacement];
    assert_rejected(
        &mut world,
        &ids,
        &[floor, first, second],
        tilted.rotation,
        tilted.translation,
        |e| matches!(e, BodyError::NotFound(_)),
    );

    // A gravity whose rotation could overflow is no longer accepted, so a rebase cannot meet
    // one.
    let gravity = world.gravity();
    assert!(matches!(
        world.set_gravity(Vec3::new(f32::MAX, f32::MAX, 0.0)),
        Err(WorldError::InvalidValue(_))
    ));
    let bits = |v: Vec3| <[f32; 3]>::from(v).map(f32::to_bits);
    assert_eq!(bits(world.gravity()), bits(gravity));
}

#[test]
fn empty_world_rebases() {
    let (mut world, _) = five_layer_world();
    let frame = Frame::tilted();
    assert_eq!(world.rebase(&[], frame.rotation, frame.translation), Ok(()));
}

/// How a vehicle run treats its collision tester after the rebase.
#[derive(Clone, Copy, PartialEq)]
enum TesterAfterRebase {
    /// Keep the tester the rebase rotated.
    Rotated,
    /// Set back a tester with the up of the old frame.
    OldUp,
}

/// A car driving straight on a box floor under the override gravity, with a ray tester whose
/// slope limit is only 10 degrees: a tester up that did not follow a tilting rebase rejects the
/// floor. Drives 60 ticks, then rebases with `frame` when given, then drives 60 more ticks.
/// Returns the world, the chassis, the vehicle and how many wheels had contact on each tick
/// after the rebase.
fn drive_across(
    frame: Option<&Frame>,
    tester_after: TesterAfterRebase,
) -> (PhysicsWorld, BodyId, VehicleId, Vec<usize>) {
    let (mut world, layers) = common::vehicle::car_world(Vec3::ZERO, 1);
    let floor = Shape::new_box(Vec3::new(100.0, 1.0, 100.0)).unwrap();
    let floor = world
        .create_body(
            &floor,
            &BodySettings::new_static()
                .position(RVec3::new(0.0, -1.0, 0.0))
                .object_layer(layers.ground),
        )
        .unwrap();
    let tester = VehicleCollisionTester::Ray {
        object_layer: layers.probe,
        up: Vec3::new(0.0, 1.0, 0.0),
        max_slope_angle: 10.0_f32.to_radians(),
    };
    let body =
        common::vehicle::chassis_settings(&layers, RVec3::new(0.0, 0.9, 0.0), Quat::IDENTITY);
    let (chassis, car) = common::vehicle::add_car_with(&mut world, &body, tester);
    let input = DriverInput {
        forward: 0.5,
        ..DriverInput::default()
    };
    let mut gravity = common::vehicle::GRAVITY;
    for _ in 0..60 {
        let mut vehicle = world.vehicle_mut(car).unwrap();
        vehicle.set_gravity(gravity).unwrap();
        vehicle.set_driver_input(input).unwrap();
        step(&mut world, 1);
    }
    if let Some(frame) = frame {
        world
            .rebase(&[floor, chassis], frame.rotation, frame.translation)
            .unwrap();
        gravity = frame.map_vector(gravity);
        let vehicle = world.vehicle(car).unwrap();
        assert_vector_near(
            "gravity override",
            vehicle.gravity().unwrap(),
            gravity,
            1e-5,
        );
        let up = vehicle.collision_tester().up().unwrap();
        assert_vector_near(
            "tester up",
            up,
            frame.map_vector(Vec3::new(0.0, 1.0, 0.0)),
            1e-6,
        );
        if tester_after == TesterAfterRebase::OldUp {
            world
                .vehicle_mut(car)
                .unwrap()
                .set_collision_tester(tester)
                .unwrap();
        }
    }
    let mut wheels_down = Vec::new();
    for _ in 0..60 {
        let mut vehicle = world.vehicle_mut(car).unwrap();
        vehicle.set_gravity(gravity).unwrap();
        vehicle.set_driver_input(input).unwrap();
        step(&mut world, 1);
        let vehicle = world.vehicle(car).unwrap();
        wheels_down.push(
            vehicle
                .wheels()
                .iter()
                .filter(|wheel| wheel.contact.is_some())
                .count(),
        );
    }
    (world, chassis, car, wheels_down)
}

#[test]
fn vehicle_drives_across_a_rotating_rebase() {
    let frame = Frame::tilted();
    let (unrebased, chassis_a, car_a, wheels_down_a) =
        drive_across(None, TesterAfterRebase::Rotated);
    assert!(wheels_down_a.iter().all(|&count| count == 4));
    let (rebased, chassis_b, car_b, wheels_down_b) =
        drive_across(Some(&frame), TesterAfterRebase::Rotated);
    assert!(
        wheels_down_b.iter().all(|&count| count == 4),
        "a wheel lost the floor after the rebase: {wheels_down_b:?}"
    );

    let a = unrebased.body(chassis_a).unwrap();
    let b = rebased.body(chassis_b).unwrap();
    // The car moved: the comparison is not between two resting cars.
    assert!(v3(a.position())[2] > 1.0);
    let position = norm(sub(v3(b.position()), frame.map_point_f64(v3(a.position()))));
    assert!(position <= 1e-2, "position off by {position}");
    let velocity = norm(sub(
        f3(b.linear_velocity()),
        f3(frame.map_vector(a.linear_velocity())),
    ));
    assert!(velocity <= 1e-2, "velocity off by {velocity}");
    assert_eq!(
        unrebased.vehicle(car_a).unwrap().current_gear(),
        rebased.vehicle(car_b).unwrap().current_gear()
    );

    // The control: the same rebase with the tester's old up loses the floor at once.
    let (_, _, _, wheels_down_c) = drive_across(Some(&frame), TesterAfterRebase::OldUp);
    assert_eq!(wheels_down_c[0], 0, "{wheels_down_c:?}");
}

#[test]
fn a_resting_cloth_moves_with_a_rebase_and_stays_at_rest() {
    use common::soft_body::Cloth;

    let mut world = world(Vec3::new(0.0, -GRAVITY, 0.0), 1);
    let floor = add_floor(&mut world);
    let cloth = Cloth::new(6, 0.2);
    let shared = cloth.settings();
    let resting = world
        .create_soft_body(
            &shared,
            &SoftBodySettings::default().position(RVec3::new(0.0, 0.05, 0.0)),
        )
        .unwrap();
    let free = world
        .create_soft_body(
            &shared,
            &SoftBodySettings::default()
                .position(RVec3::new(5.0, 3.0, 0.0))
                .gravity_factor(0.0),
        )
        .unwrap();
    let mut ticks = 0;
    while !world.body(resting).unwrap().is_sleeping() {
        step(&mut world, 1);
        ticks += 1;
        assert!(ticks < 600, "the cloth does not come to rest");
    }
    let before = world.soft_body(resting).unwrap().vertices();

    let rotation = tilted_rotation();
    let translation = RVec3::new(100.0, -50.0, 30.0);
    world
        .rebase(&[floor, resting, free], rotation, translation)
        .unwrap();
    assert!(world.body(resting).unwrap().is_sleeping(), "nothing wakes");
    let after = world.soft_body(resting).unwrap().vertices();
    for (old, new) in before.iter().zip(&after) {
        let expected = rotate(rotation, v3(old.position));
        let expected = [
            expected[0] + v3(translation)[0],
            expected[1] + v3(translation)[1],
            expected[2] + v3(translation)[2],
        ];
        let error = norm(sub(v3(new.position), expected));
        assert!(
            error < 1.0e-4,
            "vertex moved {error} m off the mapped position"
        );
        let velocity = rotate(rotation, f3(old.velocity));
        assert!(norm(sub(f3(new.velocity), velocity)) < 1.0e-5);
    }

    step(&mut world, 30);
    let body = world.body(resting).unwrap();
    let speed = common::soft_body::max_vertex_speed(&world, resting);
    assert!(
        body.is_sleeping() || speed < 0.05,
        "{speed} m/s after the rebase"
    );

    // The free cloth's body is turned by the rebase; a world force still pushes along itself.
    let force = Vec3::new(3.0, 4.0, 0.0);
    world.body_mut(free).unwrap().add_force(force).unwrap();
    step(&mut world, 1);
    let vertices = world.soft_body(free).unwrap().vertices();
    let mean = vertices.iter().fold([0.0; 3], |m, v| {
        let v = f3(v.velocity);
        [m[0] + v[0], m[1] + v[1], m[2] + v[2]]
    });
    let direction = mean.map(|c| c / norm(mean));
    let along = (direction[0] * 3.0 + direction[1] * 4.0) / 5.0;
    assert!(along > 0.99, "{direction:?}");
}
