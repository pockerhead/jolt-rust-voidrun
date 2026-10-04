//! Triangle mesh bodies: queries, contacts, per-triangle materials, kinematic meshes and the
//! bodies that may not use them.

mod common;

use common::meshes::*;
use common::ragdoll::{humanoid_parts, part_shapes, skeleton};
use common::*;
use oxijolt::*;

/// Cells per side and cell size of the bumpy grid.
const CELLS: u32 = 64;
const CELL: f32 = 0.5;

fn bump(x: f32, z: f32) -> f32 {
    0.5 * (0.3 * x).sin() * (0.2 * z).cos()
}

/// The height of the bumpy grid's triangle surface at `(x, z)`, interpolated on the triangle
/// that holds the point (the grid's diagonal runs from the lowest x and z corner of a cell).
fn grid_surface(x: f32, z: f32) -> f32 {
    let half = CELLS as f32 * CELL / 2.0;
    let (i, k) = (((x + half) / CELL).floor(), ((z + half) / CELL).floor());
    let (x0, z0) = (i * CELL - half, k * CELL - half);
    let (u, v) = ((x - x0) / CELL, (z - z0) / CELL);
    let h = |di: f32, dk: f32| bump(x0 + di * CELL, z0 + dk * CELL);
    if v >= u {
        h(0.0, 0.0) + v * (h(0.0, 1.0) - h(0.0, 0.0)) + u * (h(1.0, 1.0) - h(0.0, 1.0))
    } else {
        h(0.0, 0.0) + u * (h(1.0, 0.0) - h(0.0, 0.0)) + v * (h(1.0, 1.0) - h(1.0, 0.0))
    }
}

// `Real` is `f32` without the `double-precision` feature.
#[allow(clippy::unnecessary_cast)]
fn f32_of(value: Real) -> f32 {
    value as f32
}

fn bumpy_mesh() -> Shape {
    let (vertices, triangles) = grid(CELLS, CELL, bump);
    let (mesh, dropped) = Shape::new_mesh(&vertices, &triangles).unwrap();
    assert!(dropped.is_empty());
    mesh
}

fn add_static(world: &mut PhysicsWorld, shape: &Shape) -> BodyId {
    world
        .create_body(shape, &BodySettings::new_static())
        .unwrap()
}

#[test]
fn rays_follow_the_triangle_surface() {
    let mut world = world(Vec3::ZERO, 1);
    let ground = add_static(&mut world, &bumpy_mesh());
    // Jolt stores vertices quantized to 21 bits over the mesh bounds (32 m wide).
    let tolerance = 32.0 / (1 << 21) as f32 + 1.0e-4;
    for (x, z) in [
        (-12.3, 7.9),
        (0.1, 0.2),
        (5.55, -14.4),
        (13.0, 13.0),
        (-3.7, -8.25),
    ] {
        let ray = RayCast::new(
            RVec3::new(x as Real, 5.0, z as Real),
            Vec3::new(0.0, -10.0, 0.0),
        );
        let hit = world.cast_ray(ray, &QueryFilter::new()).unwrap().unwrap();
        assert_eq!(hit.body, ground);
        let y = f32_of(ray.point_at(hit.fraction).y);
        let expected = grid_surface(x, z);
        assert!(
            (y - expected).abs() <= tolerance,
            "({x}, {z}): {y} vs {expected}"
        );
        assert!(hit.normal.y > 0.8, "{hit:?}");
    }
    // Closest-hit rays hit back faces: from below the mesh is hit too.
    let up = RayCast::new(RVec3::new(0.1, -5.0, 0.2), Vec3::new(0.0, 10.0, 0.0));
    let hit = world.cast_ray(up, &QueryFilter::new()).unwrap().unwrap();
    assert_eq!(hit.body, ground);
    let y = f32_of(up.point_at(hit.fraction).y);
    assert!((y - grid_surface(0.1, 0.2)).abs() <= tolerance, "{y}");
}

#[test]
fn shape_queries_see_the_mesh_from_above() {
    let mut world = world(Vec3::ZERO, 1);
    let ground = add_static(&mut world, &flat_grid(8, 1.0));
    let sphere = Shape::new_sphere(0.5).unwrap();
    let cast = ShapeCast::new(
        &sphere,
        RVec3::new(0.3, 3.0, -0.2),
        Quat::IDENTITY,
        Vec3::new(0.0, -5.0, 0.0),
    );
    let hit = world
        .cast_shape(&cast, &QueryFilter::new())
        .unwrap()
        .unwrap();
    assert_eq!(hit.body, ground);
    assert!((hit.distance - 2.5).abs() < 1.0e-3, "{hit:?}");
    assert!(hit.normal.y > 0.99, "{hit:?}");

    let capsule = Shape::new_capsule(0.5, 0.25).unwrap();
    let query = CollideShape::new(&capsule, RVec3::new(1.2, 0.65, 0.4), Quat::IDENTITY);
    let hits = world.collide_shape(&query, &QueryFilter::new()).unwrap();
    // Every triangle under the capsule reports its own overlap; the deepest is the capsule's.
    let deepest = hits
        .iter()
        .map(|hit| hit.penetration_depth)
        .fold(f32::MIN, f32::max);
    assert!((deepest - 0.1).abs() < 1.0e-3, "{hits:?}");
    for hit in &hits {
        assert_eq!(hit.body, ground);
        assert!(hit.normal.y > 0.99, "{hit:?}");
    }
}

#[test]
fn dynamic_shapes_rest_on_a_mesh() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    add_static(&mut world, &flat_grid(16, 1.0));
    let lying = quat_about(Vec3::new(0.0, 0.0, 1.0), std::f32::consts::FRAC_PI_2);
    // Shape, rotation, resting height of the body origin.
    let cases = [
        (
            Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap(),
            Quat::IDENTITY,
            0.5,
        ),
        (Shape::new_sphere(0.5).unwrap(), Quat::IDENTITY, 0.5),
        (Shape::new_capsule(0.5, 0.3).unwrap(), lying, 0.3),
        (
            Shape::new_convex_hull(&box_corners(Vec3::new(0.4, 0.4, 0.4)), 0.05).unwrap(),
            Quat::IDENTITY,
            0.4,
        ),
    ];
    let ids: Vec<BodyId> = cases
        .iter()
        .enumerate()
        .map(|(i, (shape, rotation, _))| {
            let settings = BodySettings::new_dynamic()
                .position(RVec3::new(-4.5 + 3.0 * i as Real, 1.5, 0.3))
                .rotation(*rotation);
            world.create_body(shape, &settings).unwrap()
        })
        .collect();
    step(&mut world, 240);
    for (id, (_, _, rest)) in ids.iter().zip(&cases) {
        let body = world.body(*id).unwrap();
        assert!(is_calm(&body), "{id:?}");
        let y = f32_of(body.position().y);
        // Resting contacts may sink up to Jolt's penetration slop of 0.02 m.
        assert!(
            y >= rest - 0.0201 && y < rest + 0.01,
            "{id:?}: {y} vs {rest}"
        );
    }
}

#[test]
fn contacts_report_the_triangle_material() {
    const LEFT: u64 = 71;
    const RIGHT: u64 = 72;
    let (left, right) = (
        PhysicsMaterial::new(LEFT).unwrap(),
        PhysicsMaterial::new(RIGHT).unwrap(),
    );
    let (vertices, triangles) = grid(8, 1.0, |_, _| 0.0);
    // A triangle's material follows the side of x = 0 its cell lies on.
    let indices: Vec<u8> = triangles
        .iter()
        .map(|triangle| u8::from(vertices[triangle[1] as usize].x >= 0.5))
        .collect();
    let refs = [&left, &right];
    let settings = MeshSettings::default().materials(&refs, &indices);
    let (ground, _) = Shape::new_mesh_with_settings(&vertices, &triangles, &settings).unwrap();
    drop(left);
    drop(right);

    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    world.set_event_settings(EventSettings::default().contacts(true));
    let floor = add_static(&mut world, &ground);
    let west = add_cube(&mut world, RVec3::new(-2.3, 0.8, 0.4));
    let east = add_cube(&mut world, RVec3::new(2.3, 0.8, -0.4));
    let mut seen = Vec::new();
    for _ in 0..60 {
        assert!(world.step(DT).unwrap().is_complete());
        for event in world.take_events().contacts {
            if let ContactEvent::Added { manifold, .. } = event {
                let pair = manifold.pair;
                assert_eq!(pair.body1, floor, "the floor has the lower id");
                seen.push((pair.body2, manifold.materials[0]));
            }
        }
    }
    assert!(seen.contains(&(west, Some(LEFT))), "{seen:?}");
    assert!(seen.contains(&(east, Some(RIGHT))), "{seen:?}");
    assert!(!seen.contains(&(west, Some(RIGHT))), "{seen:?}");
    assert!(!seen.contains(&(east, Some(LEFT))), "{seen:?}");
}

#[test]
fn thin_strips_of_a_wide_level_mesh_carry_a_cube() {
    const STRIP: u64 = 81;
    const FAR: u64 = 82;
    // A 2 m floor of 2 m by 1 cm strips at the origin and a 10 m triangle 1500 m out along x:
    // the far triangle makes the x quantization step 0.7 mm, while each strip is 1 cm wide
    // along z, where the step is 5 um.
    let mut vertices = Vec::new();
    let mut triangles = Vec::new();
    for k in 0..200 {
        let z = k as f32 * 0.01 - 1.0;
        let base = vertices.len() as u32;
        vertices.extend([
            Vec3::new(-1.0, 0.0, z),
            Vec3::new(-1.0, 0.0, z + 0.01),
            Vec3::new(1.0, 0.0, z + 0.01),
            Vec3::new(1.0, 0.0, z),
        ]);
        triangles.extend([[base, base + 1, base + 2], [base, base + 2, base + 3]]);
    }
    let far = vertices.len() as u32;
    vertices.extend([
        Vec3::new(1500.0, 0.0, -5.0),
        Vec3::new(1500.0, 0.0, 5.0),
        Vec3::new(1510.0, 0.0, 0.0),
    ]);
    triangles.push([far, far + 1, far + 2]);
    let (strip, far_material) = (
        PhysicsMaterial::new(STRIP).unwrap(),
        PhysicsMaterial::new(FAR).unwrap(),
    );
    let mut indices = vec![0; triangles.len()];
    indices[triangles.len() - 1] = 1;
    let refs = [&strip, &far_material];
    let settings = MeshSettings::default().materials(&refs, &indices);
    let (ground, dropped) =
        Shape::new_mesh_with_settings(&vertices, &triangles, &settings).unwrap();
    assert!(dropped.is_empty(), "{dropped:?}");

    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    world.set_event_settings(EventSettings::default().contacts(true));
    let floor = add_static(&mut world, &ground);
    for (x, z) in [(0.0, 0.0), (-0.73, 0.4049), (0.9, -0.9951), (1505.0, 0.0)] {
        let ray = RayCast::new(
            RVec3::new(x as Real, 1.0, z as Real),
            Vec3::new(0.0, -2.0, 0.0),
        );
        let hit = world.cast_ray(ray, &QueryFilter::new()).unwrap().unwrap();
        assert_eq!(hit.body, floor);
        assert!(
            f32_of(ray.point_at(hit.fraction).y).abs() < 1.0e-4,
            "({x}, {z})"
        );
    }
    let cube = add_cube(&mut world, RVec3::new(0.1, 0.8, -0.2));
    let mut materials = Vec::new();
    for _ in 0..120 {
        assert!(world.step(DT).unwrap().is_complete());
        for event in world.take_events().contacts {
            if let ContactEvent::Added { manifold, .. } = event {
                assert_eq!(manifold.pair.body1, floor, "the floor has the lower id");
                materials.push(manifold.materials[0]);
            }
        }
    }
    let body = world.body(cube).unwrap();
    assert!(is_calm(&body));
    let y = f32_of(body.position().y);
    assert!((0.4799..0.51).contains(&y), "{y}");
    assert!(!materials.is_empty());
    assert!(materials.iter().all(|&m| m == Some(STRIP)), "{materials:?}");
}

/// A sliver 1 m long along z and `width` wide along x at the mesh origin, front face up.
fn sliver(width: f32) -> [Vec3; 3] {
    [
        Vec3::ZERO,
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(width, 0.0, 0.5),
    ]
}

/// The narrowest sliver a mesh for convex shapes up to `extent` keeps, within 1 %.
fn thinnest_kept_sliver(extent: f32) -> Shape {
    let settings = MeshSettings::default().max_convex_extent(extent);
    let build = |width| Shape::new_mesh_with_settings(&sliver(width), &[[0, 1, 2]], &settings);
    let (mut dropped, mut kept) = (0.0f32, 0.01f32);
    while kept > 1.01 * dropped {
        let middle = 0.5 * (dropped + kept);
        if build(middle).is_ok() {
            kept = middle;
        } else {
            dropped = middle;
        }
    }
    build(kept).unwrap().0
}

/// `v` turned by the unit quaternion `q`, in `f64`.
fn turned(q: Quat, v: Vec3) -> [f64; 3] {
    let [x, y, z, w]: [f32; 4] = q.into();
    let (q, v, w) = (
        [x, y, z].map(f64::from),
        [v.x, v.y, v.z].map(f64::from),
        f64::from(w),
    );
    let cross = |a: [f64; 3], b: [f64; 3]| {
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    };
    let t = cross(q, v).map(|c| 2.0 * c);
    let u = cross(q, t);
    [0, 1, 2].map(|i| v[i] + w * t[i] + u[i])
}

fn finite(v: Vec3) -> bool {
    v.x.is_finite() && v.y.is_finite() && v.z.is_finite()
}

/// A box of half extent `(half, 1, half)` turned by `turn`, placed so the sliver lies 0.1 m
/// inside its bottom face near a far corner: in the box's space the sliver's coordinates are
/// close to the extent.
// `Real` is `f64` with the `double-precision` feature.
#[allow(clippy::unnecessary_cast)]
fn box_over_sliver(half: f32, turn: Quat) -> (Shape, RVec3) {
    let boxed = Shape::new_box(Vec3::new(half, 1.0, half)).unwrap();
    let in_box = turned(turn, Vec3::new(1.0 - half, -0.9, 1.0 - half));
    let centre = [-in_box[0], -in_box[1], 0.5 - in_box[2]].map(|c| c as Real);
    (boxed, RVec3::from(centre))
}

#[test]
fn the_thinnest_kept_slivers_collide_with_convex_shapes_up_to_the_extent() {
    let turns = [
        Quat::IDENTITY,
        quat_about(Vec3::new(0.0, 1.0, 0.0), 0.6),
        quat_about(Vec3::new(0.6, 0.8, 0.0), 2.0),
    ];
    // The two largest boxes are those that tripped Jolt's assertions on slivers the rule
    // kept before it counted the convex shape's space.
    for (extent, halves) in [
        (MeshSettings::DEFAULT_MAX_CONVEX_EXTENT, [300.0, 750.0]),
        (limits::MAX_SHAPE_EXTENT, [1500.0, 2000.0]),
    ] {
        let mesh = thinnest_kept_sliver(extent);
        let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
        add_static(&mut world, &mesh);
        for half in halves {
            for turn in turns {
                let (boxed, centre) = box_over_sliver(half, turn);
                let query = CollideShape::new(&boxed, centre, turn);
                let hits = world.collide_shape(&query, &QueryFilter::new()).unwrap();
                assert!(!hits.is_empty(), "{extent} m, box {half} m");
                for hit in hits {
                    assert!(finite(hit.normal) && hit.penetration_depth.is_finite());
                }
            }
            // A heavy box resting on the sliver; the speculative contact distance, 0.02 m,
            // counts toward the extent.
            let (boxed, centre) = box_over_sliver(half - 0.02, Quat::IDENTITY);
            let resting = RVec3::new(centre.x, centre.y + 0.09, centre.z);
            let settings = BodySettings::new_dynamic().position(resting).mass(1.0e5);
            let id = world.create_body(&boxed, &settings).unwrap();
            for _ in 0..30 {
                assert!(world.step(DT).unwrap().is_complete());
            }
            let body = world.body(id).unwrap();
            assert!(finite(body.linear_velocity()) && finite(body.angular_velocity()));
            world.remove_body(id).unwrap();
        }
    }
}

/// A kinematic flat mesh platform, 4 m wide, with a mass so a kinematic body may use it.
fn add_platform(world: &mut PhysicsWorld, mesh: &Shape) -> BodyId {
    world
        .create_body(
            mesh,
            &BodySettings::new_kinematic().mass(100.0).friction(0.8),
        )
        .unwrap()
}

#[test]
fn kinematic_mesh_platform_carries_a_box() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    let platform = add_platform(&mut world, &flat_grid(4, 1.0));
    let cube = Shape::new_box(Vec3::new(0.25, 0.25, 0.25)).unwrap();
    let rider = world
        .create_body(
            &cube,
            &BodySettings::new_dynamic()
                .position(RVec3::new(0.0, 0.26, 0.0))
                .friction(0.8),
        )
        .unwrap();
    step(&mut world, 30);
    let start = world.body(rider).unwrap().position().x;
    // Accelerate at 0.5 m/s², below what friction can pass on, for two seconds.
    for tick in 1..=120 {
        let speed = 0.5 * tick as f32 * DT;
        world
            .body_mut(platform)
            .unwrap()
            .set_linear_velocity(Vec3::new(speed, 0.0, 0.0))
            .unwrap();
        assert!(world.step(DT).unwrap().is_complete());
    }
    let moved = world.body(platform).unwrap().position().x;
    let carried = world.body(rider).unwrap().position().x - start;
    assert!(moved > 0.9, "{moved}");
    assert!(
        (carried - moved).abs() < 0.05,
        "box {carried} vs platform {moved}"
    );
}

#[test]
fn kinematic_pure_mesh_steps_without_inertia() {
    let mut world = world(Vec3::ZERO, 1);
    let platform = add_platform(&mut world, &flat_grid(2, 1.0));
    {
        let mut body = world.body_mut(platform).unwrap();
        body.set_linear_velocity(Vec3::new(0.5, 0.1, 0.0)).unwrap();
        body.set_angular_velocity(Vec3::new(0.0, 0.7, 0.2)).unwrap();
    }
    step(&mut world, 60);
    assert!(is_finite_body(&world, platform));
    assert!(world.body(platform).unwrap().position().x > 0.4);
}

#[test]
fn bodies_that_may_not_use_a_mesh_are_refused() {
    let mesh = flat_grid(2, 1.0);
    let block = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
    let field = Shape::new_height_field(3, &[0.0; 9], &HeightFieldSettings::default()).unwrap();
    let child = |shape, x| CompoundChild {
        shape,
        position: Vec3::new(x, 0.0, 0.0),
        rotation: Quat::IDENTITY,
        user_data: 0,
    };
    let mesh_and_field = Shape::new_compound(&[child(&mesh, 0.0), child(&field, 4.0)]).unwrap();
    let mesh_and_box = Shape::new_compound(&[child(&mesh, 0.0), child(&block, 4.0)]).unwrap();
    let mut world = world(Vec3::ZERO, 1);
    let refused = |world: &mut PhysicsWorld, shape: &Shape, settings: BodySettings| match world
        .create_body(shape, &settings)
    {
        Err(BodyError::InvalidValue(rule)) => rule,
        other => panic!("accepted: {other:?}"),
    };
    assert_eq!(
        refused(&mut world, &mesh, BodySettings::new_kinematic()),
        "a kinematic body with a mesh shape needs a mass"
    );
    for settings in [
        BodySettings::new_dynamic(),
        BodySettings::new_dynamic().mass(5.0),
    ] {
        assert_eq!(
            refused(&mut world, &mesh, settings),
            "mesh shapes cannot be used by dynamic bodies"
        );
    }
    assert_eq!(
        refused(
            &mut world,
            &mesh_and_field,
            BodySettings::new_kinematic().mass(5.0)
        ),
        "this shape can only be used by static bodies"
    );
    assert_eq!(world.body_count(), 0);
    world
        .create_body(&mesh_and_box, &BodySettings::new_kinematic().mass(5.0))
        .unwrap();

    let query = CollideShape::new(&mesh, RVec3::new(0.0, 0.0, 0.0), Quat::IDENTITY);
    assert!(matches!(
        world.collide_shape(&query, &QueryFilter::new()),
        Err(QueryError::InvalidValue(_))
    ));
    let character = CharacterSettings::new(&mesh);
    assert!(matches!(
        world.create_character(&character, RVec3::new(0.0, 5.0, 0.0), Quat::IDENTITY),
        Err(CharacterError::InvalidValue(_))
    ));
    let mut shapes = part_shapes();
    shapes[0] = flat_grid(2, 0.2);
    let parts = humanoid_parts(&shapes, ObjectLayer::MOVING);
    assert!(matches!(
        RagdollSettings::new(&skeleton(), &parts),
        Err(RagdollError::InvalidValue(_))
    ));
}

#[test]
fn meshes_and_heightfields_never_collide_with_each_other() {
    let mesh = flat_grid(4, 1.0);
    let block = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
    let field = Shape::new_height_field(5, &[0.0; 25], &HeightFieldSettings::default()).unwrap();
    let mesh_and_box = Shape::new_compound(&[
        CompoundChild {
            shape: &mesh,
            position: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            user_data: 0,
        },
        CompoundChild {
            shape: &block,
            position: Vec3::new(0.0, 0.5, 0.0),
            rotation: Quat::IDENTITY,
            user_data: 1,
        },
    ])
    .unwrap();
    let mut world = world(Vec3::ZERO, 4);
    world.set_event_settings(EventSettings::default().contacts(true));
    let mut ids = vec![
        add_static(&mut world, &mesh),
        world
            .create_body(
                &field,
                &BodySettings::new_static().position(RVec3::new(-2.0, 0.0, -2.0)),
            )
            .unwrap(),
    ];
    for (shape, x, speed) in [
        (&mesh, 0.5, 1.0),
        (&mesh, -0.5, -1.0),
        (&mesh_and_box, 0.0, 0.5),
    ] {
        let settings = BodySettings::new_kinematic()
            .mass(10.0)
            .position(RVec3::new(x, 0.1, 0.0))
            .linear_velocity(Vec3::new(speed, -0.2, 0.0))
            .angular_velocity(Vec3::new(0.0, 0.5, 0.0));
        ids.push(world.create_body(shape, &settings).unwrap());
    }
    for _ in 0..60 {
        assert!(world.step(DT).unwrap().is_complete());
        let events = world.take_events();
        assert!(
            events.contacts.is_empty(),
            "contact between meshes or heightfields: {:?}",
            events.contacts[0].pair()
        );
    }
    for id in ids {
        assert!(is_finite_body(&world, id));
    }
}
