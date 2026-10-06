//! The large real models that `scripts/fetch_models.py` downloads: Keenan Crane's spot (5856
//! triangles) and the Khronos ScatteringSkull (188 871 triangles). The tests are ignored by
//! default; run them with `OXIJOLT_MODELS=<dir> cargo test --test real_meshes_downloaded --
//! --include-ignored`. They fail when a file is missing or its SHA-256 differs from
//! `assets/models/models.tsv`. Results are in `docs/real-meshes.md`.

#[path = "real_meshes_checks/mod.rs"]
mod checks;
mod common;

use checks::*;
use oxijolt::*;

/// The skull model is 0.25 m tall; the checks use it as a 2.5 m statue (see
/// [`the_skull_at_its_own_size_is_too_fine_for_jolt`]).
const SKULL_SCALE: f32 = 10.0;

#[test]
#[ignore = "needs scripts/fetch_models.py"]
fn spot() {
    let model = Model::load("spot");
    let built = build(&model);
    rain_bodies(&model, &built.mesh, SMALL_DROP_SIZES);
}

#[test]
#[ignore = "needs scripts/fetch_models.py"]
fn scattering_skull() {
    let model = Model::load("ScatteringSkull").scaled(SKULL_SCALE);
    let built = build(&model);
    rain_bodies(&model, &built.mesh, DROP_SIZES);
}

/// At its own size the skull's 188 871 triangles average 1.5 mm² (twice the area, the length
/// of Jolt's cross product, 3e-6 m²), against the 1e-5 m² floor the mesh rule keeps above
/// Jolt's sliver assertion at 1e-6 m² (`EPAPenetrationDepth.h`): every triangle is dropped and
/// the constructor refuses the mesh instead of building an empty one.
#[test]
#[ignore = "needs scripts/fetch_models.py"]
fn the_skull_at_its_own_size_is_too_fine_for_jolt() {
    let model = Model::load("ScatteringSkull");
    let refused = Shape::new_mesh(&model.vertices, &model.triangles).err();
    assert_eq!(refused, Some(ShapeError::Mesh(MeshError::NoTriangles)));
}
