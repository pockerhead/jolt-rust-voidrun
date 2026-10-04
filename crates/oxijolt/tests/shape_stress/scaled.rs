//! The scaled family: every kind of shape under random per-axis scales.

use super::*;

/// How a case draws its scale.
#[derive(Clone, Copy, Debug)]
enum ScaleKind {
    /// One magnitude on every axis, random signs.
    Uniform,
    /// Independent magnitudes and signs.
    Free,
    /// Equal magnitudes on X and Z.
    UniformXz,
    /// One axis squashed to `1e-5..1e-3`, the others near 1.
    Flattened,
}

const SCALE_KINDS: [ScaleKind; 4] = [
    ScaleKind::Uniform,
    ScaleKind::Free,
    ScaleKind::UniformXz,
    ScaleKind::Flattened,
];

fn signed(rng: &mut Rng, magnitude: f64) -> f32 {
    let sign = if rng.below(0, 2) == 0 { -1.0 } else { 1.0 };
    (sign * magnitude) as f32
}

fn scale(rng: &mut Rng, kind: ScaleKind) -> Vec3 {
    let mut magnitude = || rng.log_range(1.0e-5, 1.0e3);
    let m = match kind {
        ScaleKind::Uniform => {
            let m = magnitude();
            [m; 3]
        }
        ScaleKind::Free => [magnitude(), magnitude(), magnitude()],
        ScaleKind::UniformXz => {
            let (xz, y) = (magnitude(), magnitude());
            [xz, y, xz]
        }
        ScaleKind::Flattened => {
            let mut m = [0; 3].map(|_| rng.log_range(0.5, 2.0));
            m[rng.below(0, 3)] = rng.log_range(1.0e-5, 1.0e-3);
            m
        }
    };
    Vec3::new(signed(rng, m[0]), signed(rng, m[1]), signed(rng, m[2]))
}

/// A base shape and the corners of its local bounds.
struct Base {
    name: &'static str,
    shape: Shape,
    corners: Vec<Vec3>,
}

fn corners(half: Vec3, centre: Vec3) -> Vec<Vec3> {
    let mut points = Vec::new();
    for x in [-half.x, half.x] {
        for y in [-half.y, half.y] {
            for z in [-half.z, half.z] {
                points.push(Vec3::new(centre.x + x, centre.y + y, centre.z + z));
            }
        }
    }
    points
}

fn bases() -> Vec<Base> {
    let block = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
    let ball = Shape::new_sphere(0.5).unwrap();
    let turned = Quat::from_xyzw(0.0, 0.258_819, 0.0, 0.965_926);
    let compound = Shape::new_compound(&[
        CompoundChild {
            shape: &block,
            position: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            user_data: 0,
        },
        CompoundChild {
            shape: &ball,
            position: Vec3::new(1.5, 0.0, 0.0),
            rotation: Quat::IDENTITY,
            user_data: 1,
        },
        CompoundChild {
            shape: &block,
            position: Vec3::new(-1.5, 0.0, 0.0),
            rotation: turned,
            user_data: 2,
        },
    ])
    .unwrap();
    let hull_points = vec![
        Vec3::new(-0.5, -0.45, -0.4),
        Vec3::new(0.55, -0.4, -0.35),
        Vec3::new(0.45, -0.45, 0.5),
        Vec3::new(-0.4, -0.35, 0.45),
        Vec3::new(0.0, 0.6, 0.0),
        Vec3::new(0.4, 0.35, -0.25),
    ];
    let (vertices, triangles) = grid_mesh();
    vec![
        Base {
            name: "box",
            corners: corners(Vec3::new(0.5, 0.5, 0.5), Vec3::ZERO),
            shape: Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap(),
        },
        Base {
            name: "sphere",
            corners: corners(Vec3::new(0.5, 0.5, 0.5), Vec3::ZERO),
            shape: Shape::new_sphere(0.5).unwrap(),
        },
        Base {
            name: "capsule",
            corners: corners(Vec3::new(0.3, 0.8, 0.3), Vec3::ZERO),
            shape: Shape::new_capsule(0.5, 0.3).unwrap(),
        },
        Base {
            name: "cylinder",
            corners: corners(Vec3::new(0.4, 0.5, 0.4), Vec3::ZERO),
            shape: Shape::new_cylinder(0.5, 0.4).unwrap(),
        },
        Base {
            name: "hull",
            shape: Shape::new_convex_hull(&hull_points, 0.05).unwrap(),
            corners: hull_points,
        },
        Base {
            name: "mesh",
            shape: Shape::new_mesh(&vertices, &triangles).unwrap().0,
            corners: vertices,
        },
        Base {
            name: "compound",
            corners: corners(Vec3::new(2.2, 0.75, 0.75), Vec3::ZERO),
            shape: compound,
        },
    ]
}

/// An 8 x 8 grid of 0.5 m cells with gentle bumps.
fn grid_mesh() -> (Vec<Vec3>, Vec<[u32; 3]>) {
    let mut vertices = Vec::new();
    for k in 0..9 {
        for i in 0..9 {
            let (x, z) = (0.5 * i as f32 - 2.0, 0.5 * k as f32 - 2.0);
            vertices.push(Vec3::new(x, 0.2 * (x * 1.3).sin() * (z * 0.7).cos(), z));
        }
    }
    let mut triangles = Vec::new();
    for k in 0..8 {
        for i in 0..8 {
            let v = k * 9 + i;
            triangles.push([v, v + 9, v + 10]);
            triangles.push([v, v + 10, v + 1]);
        }
    }
    (vertices, triangles)
}

/// Jolt's `ScaleHelpers::IsUniformScale` of the absolute scale: the components differ by a
/// squared length of at most `1e-8` (an absolute tolerance, so tiny scales count as uniform).
fn uniform(a: f32, b: f32, c: f32) -> bool {
    let [a, b, c] = [a, b, c].map(|v| f64::from(v.abs()));
    (b - a).powi(2) + (c - b).powi(2) + (a - c).powi(2) <= 1.0e-8
}

/// Whether Jolt's per-kind rule refuses `scale` for the base `name`.
fn refused_by_kind(name: &str, scale: Vec3) -> bool {
    match name {
        "sphere" | "capsule" => !uniform(scale.x, scale.y, scale.z),
        "cylinder" => !uniform(scale.x, scale.x, scale.z),
        _ => false,
    }
}

const SCALED_CASES: usize = 1200;

pub fn scaled_family(arena: &mut Arena) {
    let mut rng = Rng::new(0x5EED_0003);
    let bases = bases();
    let mut accepted = 0;
    for index in 0..SCALED_CASES {
        announce("scaled", index);
        let base = &bases[index % bases.len()];
        let kind = SCALE_KINDS[(index / bases.len()) % SCALE_KINDS.len()];
        let scale = scale(&mut rng, kind);
        let what = format!("scaled {index} ({} {kind:?} {scale:?})", base.name);
        let result = Shape::scaled(&base.shape, scale);
        if refused_by_kind(base.name, scale) {
            assert!(
                matches!(result, Err(ShapeError::InvalidSettings(_))),
                "{what}: {:?}",
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
        if accepted % 5 == 0 {
            let points: Vec<Vec3> = base
                .corners
                .iter()
                .map(|p| Vec3::new(p.x * scale.x, p.y * scale.y, p.z * scale.z))
                .collect();
            let extent = Extent::of(&points);
            if !arena.drop_dynamic(&shape, extent, 30, &what) {
                arena.drop_probes_on(&shape, extent, 30, &what);
            }
        }
    }
    eprintln!("scaled: {accepted} of {SCALED_CASES} accepted");
    assert!(
        accepted > SCALED_CASES / 3,
        "only {accepted} scaled shapes were accepted"
    );
}
