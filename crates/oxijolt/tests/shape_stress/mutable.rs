//! The compound editor family: seeded edit sequences over every shape kind at random poses,
//! including the extent edge, invalid indices and poses and a child that uses all 32 sub-shape
//! id bits. Each case checks every edit against a model, publishes the result and uses it on a
//! static and a dynamic body.

use super::common::meshes::grid;
use super::*;

const CASES: usize = 300;
const EDITS: usize = 20;

/// The shapes children are made of, with what the world allows them.
struct Palette {
    shapes: Vec<Shape>,
    /// Whether the shape at the same index may only be used by static bodies.
    static_only: Vec<bool>,
    /// The index of the child that uses all 32 id bits: it fits only alone.
    deep: usize,
}

fn palette() -> Palette {
    let cube = Shape::new_box(Vec3::new(0.4, 0.3, 0.5)).unwrap();
    let nested = Shape::new_compound(&[
        CompoundChild {
            shape: &cube,
            position: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            user_data: 0,
        },
        CompoundChild {
            shape: &cube,
            position: Vec3::new(1.0, 0.0, 0.0),
            rotation: Quat::IDENTITY,
            user_data: 1,
        },
    ])
    .unwrap();
    let (vertices, triangles) = grid(4, 0.5, |x, z| 0.1 * (x + z).sin());
    let (mesh, _) = Shape::new_mesh(&vertices, &triangles).unwrap();
    let mut shapes = vec![
        cube,
        Shape::new_sphere(0.35).unwrap(),
        Shape::new_capsule(0.4, 0.2).unwrap(),
        Shape::new_cylinder(0.3, 0.4).unwrap(),
        Shape::new_convex_hull(
            &[
                Vec3::new(-0.3, -0.2, -0.3),
                Vec3::new(0.3, -0.25, -0.2),
                Vec3::new(0.0, -0.3, 0.35),
                Vec3::new(0.05, 0.35, 0.0),
            ],
            0.05,
        )
        .unwrap(),
        Shape::new_tapered_capsule(0.4, 0.1, 0.3).unwrap(),
        // A thin box: rotated, it tests the inertia floor of dynamic bodies.
        Shape::new_box(Vec3::new(1.0, 0.01, 0.01)).unwrap(),
        nested,
        mesh,
        flat_height_field(),
    ];
    let mut static_only = vec![false; shapes.len() - 2];
    static_only.extend([true, true]);
    shapes.push(height_field_at_32_bits());
    static_only.push(true);
    let deep = shapes.len() - 1;
    Palette {
        shapes,
        static_only,
        deep,
    }
}

/// A 33 x 33 heightfield (13 id bits) inside compounds of 64, 64 and 128 children: 32 bits.
fn height_field_at_32_bits() -> Shape {
    let cube = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
    let mut shape = flat_height_field();
    for count in [64, 64, 128] {
        let mut children = vec![CompoundChild {
            shape: &shape,
            position: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            user_data: 0,
        }];
        children.extend((1..count).map(|i| CompoundChild {
            shape: &cube,
            position: Vec3::new(20.0 + 1.5 * i as f32, 0.0, 0.0),
            rotation: Quat::IDENTITY,
            user_data: i,
        }));
        shape = Shape::new_compound(&children).unwrap();
    }
    shape
}

/// A uniformly random rotation (Shoemake's method).
fn random_rotation(rng: &mut Rng) -> Quat {
    let (u1, u2, u3) = (rng.unit(), rng.unit(), rng.unit());
    let tau = std::f64::consts::TAU;
    let (a, b) = ((1.0 - u1).sqrt(), u1.sqrt());
    let [x, y, z, w] = [
        a * (tau * u2).sin(),
        a * (tau * u2).cos(),
        b * (tau * u3).sin(),
        b * (tau * u3).cos(),
    ]
    .map(|c| c as f32);
    Quat::from_xyzw(x, y, z, w)
}

/// A child pose: mostly near the origin, sometimes at the extent edge, sometimes invalid.
/// Returns the pose and whether the rules of `new_compound` accept it.
fn random_pose(rng: &mut Rng) -> (Vec3, Quat, bool) {
    let edge = limits::MAX_SHAPE_EXTENT;
    let near = |rng: &mut Rng| rng.range(-5.0, 5.0) as f32;
    let roll = rng.below(0, 20);
    let position = match roll {
        0 => Vec3::new(edge, near(rng), near(rng)),
        1 => Vec3::new(near(rng), -edge, near(rng)),
        2 => Vec3::new(near(rng), near(rng), edge * 1.001),
        3 => Vec3::new(f32::NAN, 0.0, 0.0),
        _ => Vec3::new(near(rng), near(rng), near(rng)),
    };
    let turn = rng.below(0, 20);
    let rotation = match turn {
        0 => Quat::from_xyzw(0.0, 0.0, 0.0, 1.5),
        1 => Quat::from_xyzw(f32::INFINITY, 0.0, 0.0, 1.0),
        _ => random_rotation(rng),
    };
    (position, rotation, !matches!(roll, 2 | 3) && turn > 1)
}

/// The editor's children as the model keeps them: palette index and user data.
type Model = Vec<(usize, u32)>;

/// Whether the model holds the 32-bit child.
fn holds(model: &Model, shape: usize) -> bool {
    model.iter().any(|&(held, _)| held == shape)
}

fn assert_same_children(editor: &MutableCompound, model: &Model, what: &str) {
    assert_eq!(editor.sub_shape_count() as usize, model.len(), "{what}");
    for (index, &(_, user_data)) in model.iter().enumerate() {
        assert_eq!(
            editor.sub_shape_user_data(index as u32),
            Some(user_data),
            "{what}: child {index}"
        );
    }
}

/// One random edit, checked against the model.
fn edit(
    rng: &mut Rng,
    palette: &Palette,
    editor: &mut MutableCompound,
    model: &mut Model,
    what: &str,
) {
    let len = model.len() as u32;
    let (position, rotation, valid_pose) = random_pose(rng);
    // Usually a shape of the palette, rarely the 32-bit child.
    let shape = if rng.below(0, 15) == 0 {
        palette.deep
    } else {
        rng.below(0, palette.deep)
    };
    let user_data = rng.next_u64() as u32;
    let index = if rng.below(0, 8) == 0 {
        len + rng.below(0, 3) as u32
    } else {
        rng.below(0, model.len().max(1)) as u32
    };
    let deep = palette.deep;
    match rng.below(0, 20) {
        0..=10 => {
            let child = CompoundChild {
                shape: &palette.shapes[shape],
                position,
                rotation,
                user_data,
            };
            let result = editor.add_shape(&child);
            let ids_fit = !(holds(model, deep) || (shape == deep && len >= 1));
            if !valid_pose || !ids_fit {
                assert!(
                    matches!(result, Err(ShapeError::InvalidSettings(_))),
                    "{what}: add {result:?}"
                );
            } else {
                assert_eq!(result, Ok(len), "{what}: add");
                model.push((shape, user_data));
            }
        }
        11..=14 => {
            let result = editor.remove_shape(index);
            if index < len {
                assert_eq!(result, Ok(()), "{what}: remove");
                model.remove(index as usize);
            } else {
                assert_eq!(
                    result,
                    Err(ShapeError::NoSubShape { index, count: len }),
                    "{what}: remove"
                );
            }
        }
        _ => {
            let replace = rng.below(0, 2) == 0;
            let new_shape = replace.then_some(&palette.shapes[shape]);
            let result = editor.modify_shape(index, position, rotation, new_shape);
            if index >= len {
                assert_eq!(
                    result,
                    Err(ShapeError::NoSubShape { index, count: len }),
                    "{what}: modify"
                );
            } else if !valid_pose || (replace && shape == deep && len >= 2) {
                assert!(
                    matches!(result, Err(ShapeError::InvalidSettings(_))),
                    "{what}: modify {result:?}"
                );
            } else {
                assert_eq!(result, Ok(()), "{what}: modify");
                if replace {
                    model[index as usize].0 = shape;
                }
            }
        }
    }
    assert_same_children(editor, model, what);
}

/// Uses `published` on a static body with a cube dropped onto it, as a query target, and on a
/// dynamic body, which takes it unless a child is static-only or the mass rules refuse it.
/// Returns whether the dynamic body took it.
fn use_publication(arena: &mut Arena, published: &Shape, static_only: bool, what: &str) -> bool {
    let world = &mut arena.world;
    let ground = world
        .create_body(published, &BodySettings::new_static())
        .unwrap();
    let ray = RayCast::new(RVec3::new(0.0, 2100.0, 0.0), Vec3::new(0.0, -2200.0, 0.0));
    let top = world
        .cast_ray(ray, &QueryFilter::new())
        .unwrap()
        .map_or(0.0, |hit| ray.point_at(hit.fraction).y);
    let cube = world
        .create_body(
            &arena.probes[1],
            &BodySettings::new_dynamic().position(RVec3::new(0.0, top + 1.0, 0.0)),
        )
        .unwrap();
    let mover = world
        .create_body(
            &arena.probes[0],
            &BodySettings::new_dynamic().position(RVec3::new(0.0, 2100.0, 3000.0)),
        )
        .unwrap();
    let taken = world
        .body_mut(mover)
        .unwrap()
        .set_shape(published, None, Activation::Activate)
        .is_ok();
    if static_only {
        assert!(!taken, "{what}: a dynamic body took a static-only child");
    }
    for _ in 0..3 {
        assert!(world.step(DT).unwrap().is_complete());
    }
    assert_finite_body(world, cube, what);
    assert_finite_body(world, mover, what);
    let capsule = Shape::new_capsule(0.5, 0.2).unwrap();
    let query = CollideShape::new(&capsule, RVec3::new(0.0, top, 0.0), Quat::IDENTITY);
    for hit in world.collide_shape(&query, &QueryFilter::new()).unwrap() {
        assert!(hit.penetration_depth.is_finite(), "{what}: {hit:?}");
    }
    for id in [ground, cube, mover] {
        world.remove_body(id).unwrap();
    }
    taken
}

pub fn mutable_family(arena: &mut Arena) {
    let palette = palette();
    let mut rng = Rng::new(0x5EED_0005);
    let (mut published_count, mut taken) = (0, 0);
    for case in 0..CASES {
        announce("mutable", case);
        let what = format!("mutable {case}");
        let mut editor = MutableCompound::new().unwrap();
        let mut model = Model::new();
        for _ in 0..EDITS {
            edit(&mut rng, &palette, &mut editor, &mut model, &what);
        }
        let published = match editor.to_shape() {
            Ok(shape) => shape,
            Err(ShapeError::InvalidSettings(_)) if model.is_empty() => continue,
            // A child at the extent edge can end up beyond it once the centre of mass moves.
            Err(ShapeError::InvalidDimensions(_)) => continue,
            Err(error) => panic!("{what}: to_shape {error}"),
        };
        published_count += 1;
        let static_only = model.iter().any(|&(shape, _)| palette.static_only[shape]);
        if use_publication(arena, &published, static_only, &what) {
            taken += 1;
        }
    }
    eprintln!("mutable: {published_count} of {CASES} published, {taken} taken by dynamic bodies");
    assert!(published_count > CASES / 2, "{published_count}");
    assert!(taken > CASES / 10, "{taken}");
}
