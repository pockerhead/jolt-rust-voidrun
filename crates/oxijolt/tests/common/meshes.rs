//! Triangle mesh fixtures shared by the integration tests.

use oxijolt::*;

/// A square grid mesh of `cells` x `cells` cells of `cell` metres, centred on the origin, with
/// vertex heights `height(x, z)`. Every cell is split along the diagonal from its lowest x and
/// z corner to its highest, and every triangle faces up (+y).
pub fn grid(cells: u32, cell: f32, height: impl Fn(f32, f32) -> f32) -> (Vec<Vec3>, Vec<[u32; 3]>) {
    let side = cells + 1;
    let half = cells as f32 * cell / 2.0;
    let mut vertices = Vec::with_capacity((side * side) as usize);
    for k in 0..side {
        for i in 0..side {
            let (x, z) = (i as f32 * cell - half, k as f32 * cell - half);
            vertices.push(Vec3::new(x, height(x, z), z));
        }
    }
    let mut triangles = Vec::with_capacity((2 * cells * cells) as usize);
    for k in 0..cells {
        for i in 0..cells {
            let v00 = k * side + i;
            let (v10, v01, v11) = (v00 + 1, v00 + side, v00 + side + 1);
            triangles.push([v00, v01, v11]);
            triangles.push([v00, v11, v10]);
        }
    }
    (vertices, triangles)
}

/// A flat grid mesh at y = 0, `cells` x `cells` cells of `cell` metres.
pub fn flat_grid(cells: u32, cell: f32) -> Shape {
    let (vertices, triangles) = grid(cells, cell, |_, _| 0.0);
    Shape::new_mesh(&vertices, &triangles).unwrap()
}

/// The eight corners of a box with the given half extents.
pub fn box_corners(half: Vec3) -> Vec<Vec3> {
    let mut points = Vec::new();
    for x in [-half.x, half.x] {
        for y in [-half.y, half.y] {
            for z in [-half.z, half.z] {
                points.push(Vec3::new(x, y, z));
            }
        }
    }
    points
}

/// An irregular hull of eleven points, roughly 1 m across, with its lowest point at y = -0.45.
pub fn irregular_points() -> Vec<Vec3> {
    vec![
        Vec3::new(-0.5, -0.45, -0.4),
        Vec3::new(0.55, -0.4, -0.35),
        Vec3::new(0.45, -0.45, 0.5),
        Vec3::new(-0.4, -0.35, 0.45),
        Vec3::new(0.0, 0.6, 0.0),
        Vec3::new(-0.45, 0.3, -0.3),
        Vec3::new(0.4, 0.35, -0.25),
        Vec3::new(0.3, 0.25, 0.45),
        Vec3::new(-0.3, 0.2, 0.4),
        Vec3::new(0.1, -0.1, 0.0),
        Vec3::new(0.0, 0.0, -0.55),
    ]
}

/// Whether every component of `id`'s pose and velocities is finite.
pub fn is_finite_body(world: &PhysicsWorld, id: BodyId) -> bool {
    let body = world.body(id).unwrap();
    let position: [Real; 3] = body.position().into();
    let rotation: [f32; 4] = body.rotation().into();
    let linear: [f32; 3] = body.linear_velocity().into();
    let angular: [f32; 3] = body.angular_velocity().into();
    position.iter().all(|v| v.is_finite())
        && rotation
            .iter()
            .chain(&linear)
            .chain(&angular)
            .all(|v| v.is_finite())
}
