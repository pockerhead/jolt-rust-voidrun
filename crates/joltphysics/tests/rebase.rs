//! Moving the world into a new frame: dropped items fall, rest and land the same with and
//! without a rebase, queries answer the same, and nothing wakes or changes on a no-op or an
//! invalid rebase.
//!
//! Every scenario runs twice with the same calls, A without and B with a rebase, and compares
//! B against A mapped through the same frame change. The mapping is computed here in `f64`,
//! independently of the crate's own math.

mod common;

use common::*;
use joltphysics::*;

type V = [f64; 3];

fn real3(p: RVec3) -> V {
    [f64::from(p.x), f64::from(p.y), f64::from(p.z)]
}

fn vec3(v: Vec3) -> V {
    [f64::from(v.x), f64::from(v.y), f64::from(v.z)]
}

fn to_rvec3(p: V) -> RVec3 {
    RVec3::new(p[0] as Real, p[1] as Real, p[2] as Real)
}

fn to_vec3(v: V) -> Vec3 {
    Vec3::new(v[0] as f32, v[1] as f32, v[2] as f32)
}

fn sub(a: V, b: V) -> V {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn norm(a: V) -> f64 {
    (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt()
}

fn cross(a: V, b: V) -> V {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

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

    fn map_vector_f64(&self, v: V) -> V {
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

    fn map_point_f64(&self, p: V) -> V {
        let rotated = self.map_vector_f64(p);
        let t = real3(self.translation);
        [rotated[0] + t[0], rotated[1] + t[1], rotated[2] + t[2]]
    }

    fn map_point(&self, p: RVec3) -> RVec3 {
        to_rvec3(self.map_point_f64(real3(p)))
    }

    fn map_vector(&self, v: Vec3) -> Vec3 {
        to_vec3(self.map_vector_f64(vec3(v)))
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

fn child(shape: &Shape, position: Vec3, user_data: u32) -> CompoundChild<'_> {
    CompoundChild {
        shape,
        position,
        rotation: Quat::IDENTITY,
        user_data,
    }
}

const ALL: QueryFilter<'static> = QueryFilter::new();
const ITEM_MASS: f32 = 1.2;
const GRAVITY: f32 = 9.8;
/// Radius of the planet whose surface the terrain is, centred below the scene.
const PLANET_RADIUS: f64 = 99.0;
/// Height of the terrain surface above the scene origin.
const SURFACE: f64 = 2.0;
const ITEM_HALF_HEIGHT: f64 = 0.06;

/// A dropped item over a heightfield terrain and a chunk, in a five-layer world with no world
/// gravity: the item is pulled towards `centre` by the caller every tick.
struct Scene {
    world: PhysicsWorld,
    terrain: BodyId,
    chunk: BodyId,
    item: BodyId,
    centre: V,
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
        norm(sub(real3(self.item().position()), self.centre))
    }

    fn item_speed(&self) -> f64 {
        norm(vec3(self.item().linear_velocity()))
    }

    /// Pulls the item towards the planet centre with its weight, then steps.
    fn tick(&mut self) {
        let position = real3(self.item().position());
        let towards = sub(self.centre, position);
        let pull = towards.map(|c| f64::from(ITEM_MASS * GRAVITY) * c / norm(towards));
        let mut item = self.world.body_mut(self.item).unwrap();
        item.reset_forces();
        item.add_force(to_vec3(pull)).unwrap();
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

    let centre = real3(offset);
    Scene {
        world,
        terrain,
        chunk,
        item,
        centre: [centre[0], centre[1] + SURFACE - PLANET_RADIUS, centre[2]],
    }
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
    let (a, b) = (a.item(), b.item());
    let position = norm(sub(
        real3(b.position()),
        frame.map_point_f64(real3(a.position())),
    ));
    let angle = angle_between(b.rotation(), frame.map_rotation(a.rotation()));
    let linear = norm(sub(
        vec3(b.linear_velocity()),
        frame.map_vector_f64(vec3(a.linear_velocity())),
    ));
    let angular = norm(sub(
        vec3(b.angular_velocity()),
        frame.map_vector_f64(vec3(a.angular_velocity())),
    ));
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

#[test]
fn landing_after_a_rebase_matches_the_unrebased_run() {
    for (tag, frame, offset) in frames() {
        let mut a = scene(offset, HIGH);
        let mut b = scene(offset, HIGH);
        for _ in 0..30 {
            a.tick();
            b.tick();
        }
        b.rebase(&frame);
        for tick in 31..=210 {
            a.tick();
            b.tick();
            assert_matches(tag, tick, &a, &b, &frame, [1e-3, 5e-3]);
        }
        let speed = b.item_speed();
        let height = b.item_radius() - (PLANET_RADIUS + ITEM_HALF_HEIGHT);
        assert!(
            speed < 0.05,
            "{tag}: the item has not landed, speed {speed}"
        );
        assert!(
            height.abs() < 0.05,
            "{tag}: the item is not on the surface, {height}"
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
                let hit = scene.world.cast_ray(*ray, &ALL).unwrap();
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
                let mapped =
                    RayCast::new(frame.map_point(ray.origin), frame.map_vector(ray.direction));
                let context = format!("{tag}, {name}, optimized {optimized}");
                let hit = scene
                    .world
                    .cast_ray(mapped, &ALL)
                    .unwrap()
                    .unwrap_or_else(|| panic!("{context}: no hit after the rebase"));
                assert_eq!(hit.body, old.body, "{context}");
                assert_eq!(hit.compound_child, old.compound_child, "{context}");
                assert!(
                    (hit.distance - old.distance).abs() <= 1e-3,
                    "{context}: {hit:?} {old:?}"
                );
                let point = norm(sub(
                    real3(mapped.point_at(hit.fraction)),
                    frame.map_point_f64(real3(ray.point_at(old.fraction))),
                ));
                assert!(point <= 1e-3, "{context}: point off by {point}");
                let normal = norm(sub(
                    vec3(hit.normal),
                    frame.map_vector_f64(vec3(old.normal)),
                ));
                assert!(normal <= 1e-3, "{context}: normal off by {normal}");
            }
        }
    }
}

fn assert_vector_near(what: &str, actual: Vec3, expected: Vec3, tolerance: f64) {
    let error = norm(sub(vec3(actual), vec3(expected)));
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
    let position = norm(sub(
        real3(body.position()),
        frame.map_point_f64(real3(before.0)),
    ));
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
        let expected = [
            f64::from(old_position.x) + 5.0,
            f64::from(old_position.y),
            f64::from(old_position.z) - 2.0,
        ];
        let error = norm(sub(real3(position), expected));
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

    world
        .set_gravity(Vec3::new(f32::MAX, f32::MAX, 0.0))
        .unwrap();
    let eighth_turn_about_z = quat_about(Vec3::new(0.0, 0.0, 1.0), 45.0_f32.to_radians());
    assert_rejected(
        &mut world,
        &ids,
        &ids,
        eighth_turn_about_z,
        RVec3::ZERO,
        invalid_value,
    );
}

#[test]
fn empty_world_rebases() {
    let (mut world, _) = five_layer_world();
    let frame = Frame::tilted();
    assert_eq!(world.rebase(&[], frame.rotation, frame.translation), Ok(()));
}
