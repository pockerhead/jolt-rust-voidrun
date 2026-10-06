//! The checks the real-mesh tests run on each model of `assets/models/models.tsv`: the mesh and
//! the convex hull build, the share of surface area the sliver rule drops stays below
//! [`MAX_DROPPED_SHARE`], cooking round-trips, spheres and boxes dropped on the mesh come to rest
//! on it without passing through, and a character walks across level pieces. Every check prints
//! its numbers, which `docs/real-meshes.md` quotes.

// Each test file uses a different subset.
#![allow(dead_code)]

mod sha256;

use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use oxijolt::*;

use crate::common::{world, DT};

/// Largest share of a model's surface area the mesh constructor may drop.
pub const MAX_DROPPED_SHARE: f64 = 0.001;
/// Jolt's `PhysicsSettings::mPenetrationSlop`, metres: resting contacts may sink this far.
pub const PENETRATION_SLOP: f32 = 0.02;
/// Variable naming the directory `scripts/fetch_models.py --out` filled.
pub const MODELS_ENV: &str = "OXIJOLT_MODELS";
/// Speed below which a dropped body counts as at rest, m/s.
const REST_SPEED: f32 = 0.05;

/// One row of `models.tsv`.
struct Row {
    file: String,
    sha256: String,
    committed: bool,
}

fn models_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/models")
}

/// The rows of model `name`: its own first, then the files listed next to it under names that
/// start with `name` and a space (a glTF buffer).
fn rows(name: &str) -> Vec<Row> {
    let table = std::fs::read_to_string(models_dir().join("models.tsv")).unwrap();
    let mut lines = table.lines();
    let header: Vec<&str> = lines.next().unwrap().split('\t').collect();
    let column = |name: &str| header.iter().position(|&c| c == name).unwrap();
    let (name_at, file_at, sha_at, where_at) = (
        column("name"),
        column("file"),
        column("sha256"),
        column("where"),
    );
    let part = format!("{name} ");
    let mut rows: Vec<(bool, Row)> = lines
        .map(|line| line.split('\t').collect::<Vec<_>>())
        .filter(|cells| cells[name_at] == name || cells[name_at].starts_with(&part))
        .map(|cells| {
            let row = Row {
                file: cells[file_at].to_owned(),
                sha256: cells[sha_at].to_owned(),
                committed: cells[where_at] == "commit",
            };
            (cells[name_at] != name, row)
        })
        .collect();
    rows.sort_by_key(|&(listed_next, _)| listed_next);
    assert!(
        rows.first().is_some_and(|&(listed_next, _)| !listed_next),
        "{name} is not in models.tsv"
    );
    rows.into_iter().map(|(_, row)| row).collect()
}

/// A model's triangles as the tests build them.
pub struct Model {
    pub name: String,
    pub vertices: Vec<Vec3>,
    pub triangles: Vec<[u32; 3]>,
}

impl Model {
    /// The model `name` of `models.tsv`, read from `assets/models` when it is committed and
    /// from the [`MODELS_ENV`] directory otherwise. Fails when the variable is not set, or as
    /// [`try_load`](Self::try_load) does.
    pub fn load(name: &str) -> Self {
        let downloaded = std::env::var_os(MODELS_ENV).map(PathBuf::from);
        Self::try_load(name, downloaded.as_deref()).unwrap_or_else(|error| panic!("{error}"))
    }

    /// The model `name` of `models.tsv`, its downloaded files read from `downloaded`. Fails
    /// before importing anything when a file of the model (the files listed next to it, such as
    /// a glTF buffer, too) is missing or its SHA-256 differs from the table's.
    pub fn try_load(name: &str, downloaded: Option<&Path>) -> Result<Self, String> {
        let mut paths = Vec::new();
        for row in rows(name) {
            let path = if row.committed {
                models_dir().join(&row.file)
            } else {
                let dir = downloaded.ok_or_else(|| {
                    format!("{MODELS_ENV} must name the directory scripts/fetch_models.py filled")
                })?;
                dir.join(&row.file)
            };
            let bytes =
                std::fs::read(&path).map_err(|error| format!("{}: {error}", path.display()))?;
            if sha256::hex(&bytes) != row.sha256 {
                return Err(format!(
                    "{}: SHA-256 differs from models.tsv",
                    path.display()
                ));
            }
            paths.push(path);
        }
        let mesh = mesh_import::load(&paths[0]).map_err(|error| format!("{error:?}"))?;
        Ok(Self {
            name: name.to_owned(),
            vertices: mesh
                .vertices
                .iter()
                .map(|&[x, y, z]| Vec3::new(x, y, z))
                .collect(),
            triangles: mesh.triangles,
        })
    }

    /// The model scaled uniformly by `factor`.
    pub fn scaled(mut self, factor: f32) -> Self {
        for vertex in &mut self.vertices {
            *vertex = Vec3::new(vertex.x * factor, vertex.y * factor, vertex.z * factor);
        }
        self
    }

    /// The smallest box holding every vertex.
    pub fn bounds(&self) -> (Vec3, Vec3) {
        self.vertices.iter().fold(
            (
                Vec3::new(f32::MAX, f32::MAX, f32::MAX),
                Vec3::new(f32::MIN, f32::MIN, f32::MIN),
            ),
            |(min, max), v| {
                (
                    Vec3::new(min.x.min(v.x), min.y.min(v.y), min.z.min(v.z)),
                    Vec3::new(max.x.max(v.x), max.y.max(v.y), max.z.max(v.z)),
                )
            },
        )
    }

    /// The area of every triangle, m².
    pub fn surface_area(&self) -> f64 {
        self.triangles
            .iter()
            .map(|&[a, b, c]| {
                let [a, b, c] = [a, b, c].map(|i| self.vertices[i as usize]);
                let u = [b.x - a.x, b.y - a.y, b.z - a.z].map(f64::from);
                let w = [c.x - a.x, c.y - a.y, c.z - a.z].map(f64::from);
                let cross = [
                    u[1] * w[2] - u[2] * w[1],
                    u[2] * w[0] - u[0] * w[2],
                    u[0] * w[1] - u[1] * w[0],
                ];
                0.5 * cross.iter().map(|x| x * x).sum::<f64>().sqrt()
            })
            .sum()
    }
}

/// What building and cooking a model gave.
pub struct Built {
    pub mesh: Shape,
    pub dropped: usize,
    pub dropped_share: f64,
}

/// Builds the mesh (default settings) and the convex hull of `model`, checks that the dropped
/// share is at most [`MAX_DROPPED_SHARE`] and cooking, and prints the numbers.
pub fn build(model: &Model) -> Built {
    build_with(model, &MeshSettings::default(), MAX_DROPPED_SHARE)
}

/// [`build`] with mesh `settings` and a largest dropped share of `max_dropped_share`.
pub fn build_with(model: &Model, settings: &MeshSettings, max_dropped_share: f64) -> Built {
    let started = Instant::now();
    let (mesh, dropped) =
        Shape::new_mesh_with_settings(&model.vertices, &model.triangles, settings)
            .unwrap_or_else(|error| panic!("{}: {error}", model.name));
    let build_time = started.elapsed();
    let total = model.surface_area();
    let dropped_share = f64::from(dropped.area()) / total;

    let started = Instant::now();
    let hull = Shape::new_convex_hull(&model.vertices);
    let hull_time = started.elapsed();
    if let Err(error) = hull {
        panic!("{}: convex hull of every vertex: {error}", model.name);
    }

    let started = Instant::now();
    let bytes = mesh.save_binary_state().unwrap();
    let save_time = started.elapsed();
    let started = Instant::now();
    // SAFETY: the bytes were saved just above by this build, unchanged.
    let restored = unsafe { Shape::restore_binary_state(&bytes) }.unwrap();
    let restore_time = started.elapsed();
    assert_eq!(
        restored.save_binary_state().unwrap(),
        bytes,
        "{}",
        model.name
    );

    println!(
        "{}: {} triangles, {} dropped, {:.4} % of {:.4} m² dropped; mesh {}, hull {}, \
         save {} ({} bytes), restore {}",
        model.name,
        model.triangles.len(),
        dropped.count(),
        100.0 * dropped_share,
        total,
        millis(build_time),
        millis(hull_time),
        millis(save_time),
        bytes.len(),
        millis(restore_time),
    );
    assert!(
        dropped_share <= max_dropped_share,
        "{}: the sliver rule drops {:.4} % of the surface area, above {} %",
        model.name,
        100.0 * dropped_share,
        100.0 * max_dropped_share
    );
    Built {
        mesh: restored,
        dropped: dropped.count(),
        dropped_share,
    }
}

fn millis(time: Duration) -> String {
    format!("{:.3} ms", time.as_secs_f64() * 1000.0)
}

// `Real` is `f32` without the `double-precision` feature.
#[allow(clippy::unnecessary_cast)]
pub fn real(value: f32) -> Real {
    value as Real
}

// `Real` is `f32` without the `double-precision` feature.
#[allow(clippy::unnecessary_cast)]
pub fn f32_of(value: Real) -> f32 {
    value as f32
}

/// Sizes of the bodies [`drop_bodies`] drops, metres.
#[derive(Clone, Copy)]
pub struct DropSizes {
    pub sphere_radius: f32,
    pub box_half_extent: f32,
}

/// A 0.1 m radius sphere and a 0.1 m box.
pub const DROP_SIZES: DropSizes = DropSizes {
    sphere_radius: 0.1,
    box_half_extent: 0.05,
};

/// Hand-sized items for props: a 3 cm radius sphere and a 4 cm box.
pub const SMALL_DROP_SIZES: DropSizes = DropSizes {
    sphere_radius: 0.03,
    box_half_extent: 0.02,
};

/// Small items for the radio, whose top is two 4 cm strips beside its handle: a 1 cm radius
/// sphere and a 1.4 cm box.
pub const TINY_DROP_SIZES: DropSizes = DropSizes {
    sphere_radius: 0.01,
    box_half_extent: 0.007,
};

/// The first surface of `mesh_body` under `(x, z)`, seen from above `top`: its height and the
/// normal's y component.
fn surface_at(
    world: &PhysicsWorld,
    mesh_body: BodyId,
    x: f32,
    z: f32,
    top: f32,
    depth: f32,
) -> Option<(f32, f32)> {
    let ray = RayCast::new(
        RVec3::new(real(x), real(top), real(z)),
        Vec3::new(0.0, -depth, 0.0),
    );
    let hit = world.cast_ray(&ray, &QueryFilter::new()).unwrap()?;
    (hit.body == mesh_body).then_some((top - hit.distance, hit.normal.y))
}

/// Up to `count` places on `mesh_body` where a body of horizontal radius `footprint` can rest:
/// the surface faces up (within 11.5 degrees) at the place and at four points `footprint` +
/// 5 mm away along x and z, which lie within 0.5 mm of its height. Highest first, at least
/// `spacing` apart and away from `taken`.
fn landing_spots(
    world: &PhysicsWorld,
    mesh_body: BodyId,
    (min, max): (Vec3, Vec3),
    footprint: f32,
    spacing: f32,
    taken: &[Vec3],
    count: usize,
) -> Vec<Vec3> {
    let (top, depth) = (max.y + 1.0, max.y - min.y + 2.0);
    let reach = footprint + 0.005;
    let mut spots = Vec::new();
    for i in 0..32 {
        for j in 0..32 {
            let x = min.x + (max.x - min.x) * (i as f32 + 0.5) / 32.0;
            let z = min.z + (max.z - min.z) * (j as f32 + 0.5) / 32.0;
            let Some((y, up)) = surface_at(world, mesh_body, x, z, top, depth) else {
                continue;
            };
            let around = [(reach, 0.0), (-reach, 0.0), (0.0, reach), (0.0, -reach)];
            let fits = up >= 0.98
                && around.iter().all(|&(dx, dz)| {
                    surface_at(world, mesh_body, x + dx, z + dz, top, depth)
                        .is_some_and(|(height, up)| up >= 0.98 && (height - y).abs() <= 5.0e-4)
                });
            if fits {
                spots.push(Vec3::new(x, y, z));
            }
        }
    }
    spots.sort_by(|a, b| b.y.total_cmp(&a.y));
    let mut chosen: Vec<Vec3> = Vec::new();
    for spot in spots {
        let apart = |other: &Vec3| {
            let (dx, dz) = (spot.x - other.x, spot.z - other.z);
            dx * dx + dz * dz >= spacing * spacing
        };
        if chosen.len() < count && chosen.iter().chain(taken).all(apart) {
            chosen.push(spot);
        }
    }
    chosen
}

/// A world with `mesh` on a static body at the origin and a static floor whose top is at the
/// bottom of `model`, and the mesh body's id.
fn world_with_mesh(model: &Model, mesh: &Shape) -> (PhysicsWorld, BodyId) {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    let mesh_body = world
        .create_body(mesh, &BodySettings::new_static())
        .unwrap();
    let floor = Shape::new_box(Vec3::new(50.0, 0.5, 50.0)).unwrap();
    let bottom = model.bounds().0.y;
    world
        .create_body(
            &floor,
            &BodySettings::new_static().position(RVec3::new(0.0, real(bottom - 0.5), 0.0)),
        )
        .unwrap();
    (world, mesh_body)
}

/// A dropped body: its id and the lowest and highest height of its centre above a surface it
/// rests on, metres.
struct Dropped {
    id: BodyId,
    low: f32,
    high: f32,
}

/// Creates a dynamic body of `shape` with `quality` at `position`.
fn drop_at(
    world: &mut PhysicsWorld,
    shape: &Shape,
    quality: MotionQuality,
    position: Vec3,
    (low, high): (f32, f32),
) -> Dropped {
    let position = RVec3::new(real(position.x), real(position.y), real(position.z));
    let settings = BodySettings::new_dynamic()
        .position(position)
        .motion_quality(quality);
    let id = world.create_body(shape, &settings).unwrap();
    Dropped { id, low, high }
}

/// Steps `world` for 3 s and fails when the centre of a body crosses a surface of
/// `mesh_body` between two ticks: a ray along each tick's move hits the mesh.
fn step_watching_crossings(
    world: &mut PhysicsWorld,
    mesh_body: BodyId,
    bodies: &[Dropped],
    name: &str,
) {
    let mut previous: Vec<RVec3> = bodies
        .iter()
        .map(|body| world.body(body.id).unwrap().position())
        .collect();
    for tick in 0..180 {
        assert!(world.step(DT).unwrap().is_complete());
        for (body, before) in bodies.iter().zip(&mut previous) {
            let now = world.body(body.id).unwrap().position();
            let path = Vec3::new(
                f32_of(now.x - before.x),
                f32_of(now.y - before.y),
                f32_of(now.z - before.z),
            );
            if path.x != 0.0 || path.y != 0.0 || path.z != 0.0 {
                let filter = QueryFilter::new().exclude_body(body.id);
                let crossed = world
                    .cast_ray(&RayCast::new(*before, path), &filter)
                    .unwrap();
                assert!(
                    crossed.is_none_or(|hit| hit.body != mesh_body),
                    "{name}: body {:?} passed through the mesh at tick {tick}",
                    body.id
                );
            }
            *before = now;
        }
    }
}

/// The body a ray straight down from `body`'s centre hits first, after checking that it lies
/// within the body's size plus [`PENETRATION_SLOP`] and 1 mm.
fn resting_on(world: &PhysicsWorld, body: &Dropped, name: &str) -> BodyId {
    let position = world.body(body.id).unwrap().position();
    let filter = QueryFilter::new().exclude_body(body.id);
    let down = RayCast::new(position, Vec3::new(0.0, -1.0, 0.0));
    let under = world
        .cast_ray(&down, &filter)
        .unwrap()
        .unwrap_or_else(|| panic!("{name}: nothing under {:?} at {position:?}", body.id));
    // Resting contacts sink up to the slop; 1 mm more covers the solver's last iteration.
    let margin = PENETRATION_SLOP + 1.0e-3;
    let (low, high) = (body.low - margin, body.high + margin);
    assert!(
        (low..=high).contains(&under.distance),
        "{name}: {:?} at {position:?} moving at {:?} lies {} m above a surface (shape {}..{} m)",
        body.id,
        world.body(body.id).unwrap().linear_velocity(),
        under.distance,
        body.low,
        body.high
    );
    under.body
}

/// Drops up to three spheres and three boxes of `sizes` onto flat places of the static `mesh`
/// of `model`, with a floor under the model, and checks that no body's centre crosses a mesh
/// surface, and after 3 s that each is at rest on the mesh: slower than 0.05 m/s, with a ray
/// down from it hitting the mesh within its size plus [`PENETRATION_SLOP`].
pub fn drop_bodies(model: &Model, mesh: &Shape, sizes: DropSizes) {
    let (mut world, mesh_body) = world_with_mesh(model, mesh);
    let bounds = model.bounds();
    let DropSizes {
        sphere_radius,
        box_half_extent,
    } = sizes;
    let box_footprint = box_half_extent * 2f32.sqrt();
    let spacing = 4.0 * sphere_radius.max(box_footprint);
    let sphere_spots = landing_spots(&world, mesh_body, bounds, sphere_radius, spacing, &[], 3);
    let box_spots = landing_spots(
        &world,
        mesh_body,
        bounds,
        box_footprint,
        spacing,
        &sphere_spots,
        3,
    );
    assert!(
        !sphere_spots.is_empty() && !box_spots.is_empty(),
        "{}: no flat place for a sphere ({}) or a box ({})",
        model.name,
        sphere_spots.len(),
        box_spots.len()
    );

    let sphere = Shape::new_sphere(sphere_radius).unwrap();
    let block =
        Shape::new_box(Vec3::new(box_half_extent, box_half_extent, box_half_extent)).unwrap();
    let box_high = box_half_extent * 3f32.sqrt();
    let mut bodies = Vec::new();
    for spot in &sphere_spots {
        let above = Vec3::new(spot.x, spot.y + sphere_radius + 0.05, spot.z);
        let size = (sphere_radius, sphere_radius);
        bodies.push(drop_at(
            &mut world,
            &sphere,
            MotionQuality::Discrete,
            above,
            size,
        ));
    }
    for spot in &box_spots {
        let above = Vec3::new(spot.x, spot.y + box_high + 0.05, spot.z);
        let size = (box_half_extent, box_high);
        bodies.push(drop_at(
            &mut world,
            &block,
            MotionQuality::Discrete,
            above,
            size,
        ));
    }
    step_watching_crossings(&mut world, mesh_body, &bodies, &model.name);
    for body in &bodies {
        let v = world.body(body.id).unwrap().linear_velocity();
        let speed = (v.x * v.x + v.y * v.y + v.z * v.z).sqrt();
        assert!(
            speed < REST_SPEED,
            "{}: {:?} still moves at {speed} m/s",
            model.name,
            body.id
        );
        let under = resting_on(&world, body, &model.name);
        assert_eq!(
            under, mesh_body,
            "{}: {:?} left the mesh",
            model.name, body.id
        );
    }
    println!(
        "{}: {} bodies at rest on the mesh",
        model.name,
        bodies.len()
    );
}

/// For models without flat places (closed curved meshes): drops a 3 x 3 grid of spheres and
/// boxes of `sizes` over the middle of `model`'s static `mesh`, with a floor under it, and
/// checks that no body's centre crosses a mesh surface and that after 3 s each lies on the
/// mesh or the floor within its size plus [`PENETRATION_SLOP`]. Bodies roll and slide off
/// curved tops, so rest is not required. They fall up to 2 m and use
/// [`MotionQuality::LinearCast`]: in the discrete mode a 3 cm ball reaching 5 m/s passed into
/// the closed spot mesh between two ticks (`docs/real-meshes.md`).
pub fn rain_bodies(model: &Model, mesh: &Shape, sizes: DropSizes) {
    let (mut world, mesh_body) = world_with_mesh(model, mesh);
    let (min, max) = model.bounds();
    let sphere = Shape::new_sphere(sizes.sphere_radius).unwrap();
    let half = sizes.box_half_extent;
    let block = Shape::new_box(Vec3::new(half, half, half)).unwrap();
    let mut bodies = Vec::new();
    for i in 0..9 {
        let (u, v) = ((i % 3) as f32 / 2.0, (i / 3) as f32 / 2.0);
        let above = Vec3::new(
            min.x + (max.x - min.x) * (0.2 + 0.6 * u),
            max.y + 0.3 + 0.1 * i as f32,
            min.z + (max.z - min.z) * (0.2 + 0.6 * v),
        );
        let (shape, size) = if i % 2 == 0 {
            (&sphere, (sizes.sphere_radius, sizes.sphere_radius))
        } else {
            (&block, (half, half * 3f32.sqrt()))
        };
        bodies.push(drop_at(
            &mut world,
            shape,
            MotionQuality::LinearCast,
            above,
            size,
        ));
    }
    step_watching_crossings(&mut world, mesh_body, &bodies, &model.name);
    let on_mesh = bodies
        .iter()
        .filter(|body| resting_on(&world, body, &model.name) == mesh_body)
        .count();
    println!(
        "{}: {} bodies dropped, none through the mesh; {on_mesh} lie on it, the rest on the floor",
        model.name,
        bodies.len()
    );
}

/// Walks a humanoid character (1.8 m, radius 0.3 m) on `mesh` from `from` to `to` (feet
/// positions) at 2 m/s, and checks that it keeps ground contact and ends within 0.5 m of `to`.
pub fn walk_across(name: &str, mesh: &Shape, from: RVec3, to: RVec3) {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    world
        .create_body(mesh, &BodySettings::new_static())
        .unwrap();
    let settings = CharacterSettings::humanoid(1.8, 0.3).unwrap();
    let id = world
        .create_character(&settings, from, Quat::IDENTITY)
        .unwrap();
    let filter = QueryFilter::new();
    world.refresh_character_contacts(id, &filter).unwrap();
    let gravity = Vec3::new(0.0, -9.81, 0.0);
    let span = Vec3::new(f32_of(to.x - from.x), 0.0, f32_of(to.z - from.z));
    let length = (span.x * span.x + span.z * span.z).sqrt();
    let speed = 2.0;
    let ticks = (length / speed / DT).ceil() as usize;
    let mut airborne = 0;
    for _ in 0..ticks {
        let velocity = Vec3::new(span.x / length * speed, -1.0, span.z / length * speed);
        world
            .character_mut(id)
            .unwrap()
            .set_linear_velocity(velocity)
            .unwrap();
        world
            .update_character(id, DT, gravity, &ExtendedUpdateSettings::default(), &filter)
            .unwrap();
        if world.character(id).unwrap().ground_state() != GroundState::OnGround {
            airborne += 1;
        }
    }
    let end = world.character(id).unwrap().position();
    let miss = Vec3::new(
        f32_of(end.x - to.x),
        f32_of(end.y - to.y),
        f32_of(end.z - to.z),
    );
    let miss = (miss.x * miss.x + miss.y * miss.y + miss.z * miss.z).sqrt();
    println!(
        "{name}: walked {length} m, {airborne} ticks off the ground, ends {miss} m from the goal"
    );
    assert_eq!(airborne, 0, "{name}: the character lost the ground");
    assert!(
        miss <= 0.5,
        "{name}: the character ends {miss} m from the goal"
    );
}
