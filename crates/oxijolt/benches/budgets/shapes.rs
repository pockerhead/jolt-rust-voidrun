//! The shape creation case: triangle meshes of growing size built with both build qualities,
//! the same meshes scaled, and convex hulls of growing random clouds. One sample is one
//! constructor call, the shape dropped after it.

use std::error::Error;
use std::time::Instant;

use crate::report::{micros, Limit, Row};
use crate::Lcg;
use oxijolt::*;

/// Cells per side of the height grids: 1 058, 10 082, 100 352 and 1 002 528 triangles.
const GRID_CELLS: [u32; 4] = [23, 71, 224, 708];
/// Points of the hull clouds.
const HULL_POINTS: [usize; 3] = [100, 10_000, 100_000];
/// Seed of the hull clouds.
const HULL_SEED: u32 = 0x1234_5678;
/// Untimed calls before each row measures.
const WARMUP: usize = 1;
/// Measured calls per row.
const SAMPLES: usize = 5;

/// A height grid of `cells` x `cells` cells of 0.5 m with gentle waves, two triangles per cell.
fn grid(cells: u32) -> (Vec<Vec3>, Vec<[u32; 3]>) {
    let side = cells + 1;
    let mut vertices = Vec::with_capacity((side * side) as usize);
    for k in 0..side {
        for i in 0..side {
            let (x, z) = (i as f32 * 0.5, k as f32 * 0.5);
            vertices.push(Vec3::new(x, 0.3 * (0.2 * x).sin() * (0.3 * z).cos(), z));
        }
    }
    let mut triangles = Vec::with_capacity((2 * cells * cells) as usize);
    for k in 0..cells {
        for i in 0..cells {
            let v = k * side + i;
            triangles.push([v, v + side, v + side + 1]);
            triangles.push([v, v + side + 1, v + 1]);
        }
    }
    (vertices, triangles)
}

/// The microseconds of [`SAMPLES`] calls of `create` after [`WARMUP`] untimed ones; each result
/// is dropped outside the timing.
fn time<T>(mut create: impl FnMut() -> Result<T, ShapeError>) -> Result<Vec<f64>, ShapeError> {
    let mut us = Vec::with_capacity(SAMPLES);
    for call in 0..WARMUP + SAMPLES {
        let start = Instant::now();
        let shape = create()?;
        let elapsed = micros(start);
        drop(shape);
        if call >= WARMUP {
            us.push(elapsed);
        }
    }
    Ok(us)
}

pub fn run_shape_creation() -> Result<Vec<Row>, Box<dyn Error>> {
    let mut rows = Vec::new();
    for cells in GRID_CELLS {
        let (vertices, triangles) = grid(cells);
        let count = triangles.len();
        for (quality, name) in [
            (
                MeshBuildQuality::FavorRuntimePerformance,
                "FavorRuntimePerformance",
            ),
            (MeshBuildQuality::FavorBuildSpeed, "FavorBuildSpeed"),
        ] {
            let settings = MeshSettings::default().build_quality(quality);
            let us = time(|| Shape::new_mesh_with_settings(&vertices, &triangles, &settings))?;
            rows.push(Row::new(
                &format!("shape creation: mesh of {count} triangles, {name}"),
                "one creation",
                us,
                Limit::None,
                "",
            ));
        }
        let (mesh, _) = Shape::new_mesh(&vertices, &triangles)?;
        let us = time(|| Shape::scaled(&mesh, Vec3::new(2.0, 2.0, 2.0)))?;
        rows.push(Row::new(
            &format!("shape creation: Shape::scaled by 2, mesh of {count} triangles"),
            "one creation",
            us,
            Limit::None,
            "reads every stored triangle back from Jolt",
        ));
    }
    let mut random = Lcg(HULL_SEED);
    for points in HULL_POINTS {
        let mut coordinate = || (random.next() - 0.5) as f32;
        let cloud: Vec<Vec3> = (0..points)
            .map(|_| Vec3::new(coordinate(), coordinate(), coordinate()))
            .collect();
        let us = time(|| Shape::new_convex_hull(&cloud, 0.05))?;
        rows.push(Row::new(
            &format!("shape creation: convex hull of {points} random points"),
            "one creation",
            us,
            Limit::None,
            "points uniform in a 1 m cube",
        ));
    }
    Ok(rows)
}
