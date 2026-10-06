//! The committed real models of `assets/models`: mesh and convex hull creation, the share of
//! surface area the sliver rule drops, cooking, bodies dropped on each mesh and a character
//! walking across the level pieces. Results are in `docs/real-meshes.md`.

#[path = "real_meshes_checks/mod.rs"]
mod checks;
mod common;

use checks::*;
use oxijolt::*;

/// A prop built with the default convex extent, and bodies dropped on it.
fn prop(name: &str, sizes: DropSizes) {
    let model = Model::load(name);
    let built = build(&model);
    drop_bodies(&model, &built.mesh, sizes);
}

#[test]
fn radio() {
    prop("radio", TINY_DROP_SIZES);
}

#[test]
fn kitchen_fridge() {
    prop("kitchenFridgeLarge", SMALL_DROP_SIZES);
}

#[test]
fn bathtub() {
    prop("bathtub", SMALL_DROP_SIZES);
}

#[test]
fn bookcase() {
    prop("bookcaseOpen", SMALL_DROP_SIZES);
}

#[test]
fn track_tile() {
    let model = Model::load("track-straight");
    let built = build(&model);
    drop_bodies(&model, &built.mesh, DROP_SIZES);
    // The road runs along z between kerbs at x = +-3.75.
    walk_across(
        "track-straight",
        &built.mesh,
        RVec3::new(0.0, 0.0, -4.0),
        RVec3::new(0.0, 0.0, 4.0),
    );
}

#[test]
fn dungeon_corridor() {
    let model = Model::load("corridor-wide-corner");
    let built = build(&model);
    drop_bodies(&model, &built.mesh, DROP_SIZES);
    // The floor (both faces of every triangle) spans x -3.75..0.75, z -3.75..3.25.
    walk_across(
        "corridor-wide-corner",
        &built.mesh,
        RVec3::new(-1.5, 0.0, -3.25),
        RVec3::new(-1.5, 0.0, 3.0),
    );
}

#[test]
fn oloid_mesh_and_hull() {
    let model = Model::load("oloid");
    // The oloid has no flat place to rest on; it is here for its hull.
    build(&model);
    hull_holds_every_vertex(&model);
}

/// Every vertex of the oloid lies on its surface, so its convex hull passes through each of
/// them: a ray from outside towards the centre, aimed at a vertex, meets the hull within 2 mm
/// of it. Jolt keeps at most 256 of the 258 points and lets a point it leaves out lie up to its
/// hull tolerance (1 mm, more for large hulls) outside.
fn hull_holds_every_vertex(model: &Model) {
    let hull = Shape::new_convex_hull_with_convex_radius(&model.vertices, 0.0).unwrap();
    let mut world = common::world(Vec3::ZERO, 1);
    world
        .create_body(&hull, &BodySettings::new_static())
        .unwrap();
    let centre = Vec3::new(0.0, 0.5, 0.0);
    let mut worst = 0.0f32;
    for vertex in &model.vertices {
        let out = Vec3::new(
            vertex.x - centre.x,
            vertex.y - centre.y,
            vertex.z - centre.z,
        );
        let start = RVec3::new(
            real(centre.x + 2.0 * out.x),
            real(centre.y + 2.0 * out.y),
            real(centre.z + 2.0 * out.z),
        );
        let ray = RayCast::new(start, Vec3::new(-2.0 * out.x, -2.0 * out.y, -2.0 * out.z));
        let hit = world.cast_ray(&ray, &QueryFilter::new()).unwrap().unwrap();
        let length = (out.x * out.x + out.y * out.y + out.z * out.z).sqrt();
        let gap = (hit.fraction - 0.5).abs() * 2.0 * length;
        worst = worst.max(gap);
    }
    println!("oloid: hull surface within {worst} m of every vertex");
    assert!(worst <= 2.0e-3, "a vertex lies {worst} m from the hull");
}
