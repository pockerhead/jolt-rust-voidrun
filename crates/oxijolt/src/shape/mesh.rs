//! Triangle meshes.

use std::ptr::null;

use oxijolt_sys::*;

use super::geometry::{cross, length, length_sq, sub, v3, V3};
use super::{initialize, Shape, ShapeSettings};
use crate::{limits, MeshError, PhysicsMaterial, ShapeError, Vec3};

/// Most materials Jolt accepts for one mesh (`MeshShape::FLAGS_MATERIAL_BITS` = 5).
const MAX_MESH_MATERIALS: usize = 32;
/// Most triangles Jolt stores per leaf of a mesh's tree (`MeshShape::MaxTrianglesPerLeaf`).
const MAX_TRIANGLES_PER_LEAF: u32 = 8;
/// Smallest `|(v1 - v0) x (v2 - v0)|` (twice the area, m²) of a triangle given to Jolt: ten
/// times the cross product below which Jolt's collision detection asserts on a triangle
/// (`EPAPenetrationDepth::GetPenetrationDepthStepGJK`, `IsNearZero` at 1e-12 squared).
const MIN_TRIANGLE_CROSS: f64 = 1.0e-5;
/// How many times the largest change Jolt's rounding can make to a triangle's cross product the
/// cross product must keep above [`MIN_TRIANGLE_CROSS`]; see [`is_collidable`].
const CROSS_ERROR_MARGIN: f64 = 2.0;
/// Steps of Jolt's 21-bit vertex quantization across a mesh's bounds on each axis
/// (`TriangleCodecIndexed8BitPackSOA4Flags::COMPONENT_MASK`).
const QUANTIZATION_STEPS: f64 = ((1u32 << 21) - 1) as f64;

/// How Jolt builds a mesh's bounding volume tree (Jolt `MeshShapeSettings::EBuildQuality`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum MeshBuildQuality {
    /// A tree that is slower to build and faster to query. Jolt's default.
    #[default]
    FavorRuntimePerformance,
    /// A tree that is faster to build and slower to query.
    FavorBuildSpeed,
}

impl MeshBuildQuality {
    fn to_jph(self) -> JPH_Mesh_Shape_BuildQuality {
        match self {
            Self::FavorRuntimePerformance => JPH_Mesh_Shape_BuildQuality_FavorRuntimePerformance,
            Self::FavorBuildSpeed => JPH_Mesh_Shape_BuildQuality_FavorBuildSpeed,
        }
    }
}

/// Settings of [`Shape::new_mesh_with_settings`] other than the geometry. The defaults are
/// Jolt's (`MeshShapeSettings`): no materials, an active-edge threshold of 5 degrees, 8
/// triangles per leaf, [`MeshBuildQuality::FavorRuntimePerformance`]; and convex shapes up to
/// [`MeshSettings::DEFAULT_MAX_CONVEX_EXTENT`].
#[derive(Clone, Debug)]
pub struct MeshSettings<'a> {
    materials: Option<(&'a [&'a PhysicsMaterial], &'a [u8])>,
    active_edge_cos_threshold_angle: f32,
    max_triangles_per_leaf: u32,
    build_quality: MeshBuildQuality,
    max_convex_extent: f32,
}

impl Default for MeshSettings<'_> {
    fn default() -> Self {
        Self {
            materials: None,
            active_edge_cos_threshold_angle: 0.996195,
            max_triangles_per_leaf: 8,
            build_quality: MeshBuildQuality::default(),
            max_convex_extent: Self::DEFAULT_MAX_CONVEX_EXTENT,
        }
    }
}

impl<'a> MeshSettings<'a> {
    /// The default of [`max_convex_extent`](Self::max_convex_extent), metres: the largest
    /// extent at which the triangle rule still keeps a strip 1 m long and 1 mm wide, made of two
    /// triangles, near the mesh origin in any orientation, rounded down. It is below
    /// [`limits::MAX_SHAPE_EXTENT`]; see [docs/limits.md#convex-shapes-against-meshes].
    ///
    /// [docs/limits.md#convex-shapes-against-meshes]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#convex-shapes-against-meshes
    pub const DEFAULT_MAX_CONVEX_EXTENT: f32 = 1100.0;

    /// Gives triangle `i` the material `list[indices[i]]`. `list` holds 1 to 32 materials,
    /// `indices` one entry per triangle, each naming a material of the list. Without materials
    /// every triangle uses Jolt's default material. The shape holds its own reference to each
    /// material.
    #[must_use]
    pub fn materials(mut self, list: &'a [&'a PhysicsMaterial], indices: &'a [u8]) -> Self {
        self.materials = Some((list, indices));
        self
    }

    /// Cosine of the angle between two triangles above which their shared edge counts as
    /// active, in `[-1, 1]`; a negative value makes every edge active. Concave edges are never
    /// active. Smaller values give more ghost collisions with edges, larger ones slower
    /// depenetration (Jolt's wording). Default `0.996195`, the cosine of 5 degrees.
    #[must_use]
    pub fn active_edge_cos_threshold_angle(mut self, value: f32) -> Self {
        self.active_edge_cos_threshold_angle = value;
        self
    }

    /// Most triangles per leaf of the tree, in `1..=8`. Default 8.
    #[must_use]
    pub fn max_triangles_per_leaf(mut self, value: u32) -> Self {
        self.max_triangles_per_leaf = value;
        self
    }

    /// How the tree is built. Default [`MeshBuildQuality::FavorRuntimePerformance`].
    #[must_use]
    pub fn build_quality(mut self, value: MeshBuildQuality) -> Self {
        self.build_quality = value;
        self
    }

    /// The size of the largest convex shape the mesh must collide with reliably, metres: the
    /// largest absolute coordinate of the convex shape's local bounds (relative to its centre of
    /// mass, after scaling) plus the separation distance of the collide query. Jolt collides a
    /// triangle in the convex shape's space, so its rounding grows with this extent, and the
    /// mesh drops the triangles too thin for it. In `0..=2 * limits::MAX_SHAPE_EXTENT`; default
    /// [`Self::DEFAULT_MAX_CONVEX_EXTENT`]. A larger convex shape against a thin triangle can
    /// trip Jolt's assertions in an asserts build and gets a distorted contact otherwise; see
    /// [docs/limits.md#convex-shapes-against-meshes].
    ///
    /// [docs/limits.md#convex-shapes-against-meshes]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#convex-shapes-against-meshes
    #[must_use]
    pub fn max_convex_extent(mut self, metres: f32) -> Self {
        self.max_convex_extent = metres;
        self
    }

    /// Checks everything but the geometry against `triangle_count`.
    fn validate(&self, triangle_count: usize) -> Result<(), ShapeError> {
        let invalid = |what| Err(ShapeError::InvalidSettings(what));
        let threshold = self.active_edge_cos_threshold_angle;
        if !(threshold.is_finite() && (-1.0..=1.0).contains(&threshold)) {
            return invalid("active_edge_cos_threshold_angle must be between -1 and 1");
        }
        if !(1..=MAX_TRIANGLES_PER_LEAF).contains(&self.max_triangles_per_leaf) {
            return invalid("max_triangles_per_leaf must be between 1 and 8");
        }
        if !(0.0..=2.0 * limits::MAX_SHAPE_EXTENT).contains(&self.max_convex_extent) {
            return invalid("max_convex_extent must be between 0 and 2 * limits::MAX_SHAPE_EXTENT");
        }
        if let Some((list, indices)) = self.materials {
            if !(1..=MAX_MESH_MATERIALS).contains(&list.len()) {
                return invalid("mesh material list must hold 1 to 32 materials");
            }
            if indices.len() != triangle_count {
                return invalid("mesh material indices must hold one value per triangle");
            }
            if indices
                .iter()
                .any(|&index| usize::from(index) >= list.len())
            {
                return invalid("material indices must name a material of the list");
            }
        }
        Ok(())
    }
}

/// Whether Jolt can index `vertices` vertices and `triangles` triangles: its mesh code stores
/// both as `int`.
fn mesh_counts_fit(vertices: usize, triangles: usize) -> bool {
    let max = i32::MAX as usize;
    vertices <= max && triangles <= max
}

/// The geometry checks of [`Shape::new_mesh_with_settings`].
fn validate_geometry(vertices: &[Vec3], triangles: &[[u32; 3]]) -> Result<(), ShapeError> {
    if vertices.is_empty() || triangles.is_empty() {
        return Err(ShapeError::InvalidSettings(
            "a mesh needs vertices and triangles",
        ));
    }
    if !mesh_counts_fit(vertices.len(), triangles.len()) {
        return Err(ShapeError::InvalidSettings(
            "a mesh has at most i32::MAX vertices and triangles",
        ));
    }
    if !vertices
        .iter()
        .all(|&vertex| limits::is_local_offset(vertex))
    {
        return Err(ShapeError::InvalidDimensions(
            "mesh vertices must be finite and within limits::MAX_SHAPE_EXTENT",
        ));
    }
    // Jolt reads `vertices[index]` without a range check when it sanitizes the triangles.
    if triangles
        .iter()
        .flatten()
        .any(|&index| index as usize >= vertices.len())
    {
        return Err(ShapeError::InvalidSettings(
            "triangle index beyond the vertex list",
        ));
    }
    Ok(())
}

/// The triangles [`Shape::new_mesh`] left out because they are too small or too thin for Jolt to
/// collide with reliably (degenerate triangles among them).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct DroppedTriangles {
    indices: Vec<usize>,
    area: f64,
}

impl DroppedTriangles {
    /// How many triangles were dropped.
    pub fn count(&self) -> usize {
        self.indices.len()
    }

    /// Whether every triangle was kept.
    pub fn is_empty(&self) -> bool {
        self.indices.is_empty()
    }

    /// The positions of the dropped triangles in the `triangles` given to the constructor, in
    /// ascending order.
    pub fn indices(&self) -> &[usize] {
        &self.indices
    }

    /// The total area of the dropped triangles, m².
    pub fn area(&self) -> f32 {
        self.area as f32
    }

    fn push(&mut self, index: usize, [a, b, c]: [V3; 3]) {
        self.indices.push(index);
        self.area += 0.5 * length(cross(sub(b, a), sub(c, a)));
    }
}

impl Shape {
    /// A triangle mesh of `vertices` (shape space, metres) and `triangles` (three indices into
    /// `vertices` each) with [`MeshSettings::default`], and the triangles it dropped; see
    /// [`new_mesh_with_settings`](Self::new_mesh_with_settings).
    pub fn new_mesh(
        vertices: &[Vec3],
        triangles: &[[u32; 3]],
    ) -> Result<(Self, DroppedTriangles), ShapeError> {
        Self::new_mesh_with_settings(vertices, triangles, &MeshSettings::default())
    }

    /// A triangle mesh of `vertices` (shape space, metres) and `triangles` (three indices into
    /// `vertices` each), and the triangles it dropped.
    ///
    /// A triangle's front face is the side from which its vertices run counter-clockwise.
    /// Triangles too small or too thin for Jolt to collide with reliably are dropped and
    /// reported in [`DroppedTriangles`]: twice a triangle's area must be at least 1e-5 m² plus
    /// twice the largest change Jolt's 21-bit vertex quantization and `f32` rounding can make
    /// to it. That margin follows the triangle's own shape and distance from the shape origin,
    /// the quantization step of the mesh's bounds on each axis and the size of the convex shapes
    /// it collides with ([`MeshSettings::max_convex_extent`]); with the defaults a strip near
    /// the origin is kept from about 0.54 mm wide along the axes and from 0.93 mm in any
    /// orientation ([docs/limits.md#triangle-meshes]). Jolt itself
    /// keeps one copy of duplicate triangles and reorders the rest, so sub-shape ids do not
    /// follow the input order. Closest-hit rays hit back faces too.
    ///
    /// ```
    /// # use oxijolt::{Shape, Vec3};
    /// let vertices = [Vec3::ZERO, Vec3::new(0.0, 0.0, 1.0), Vec3::new(1.0, 0.0, 0.0)];
    /// // The second triangle repeats a vertex.
    /// let (mesh, dropped) = Shape::new_mesh(&vertices, &[[0, 1, 2], [0, 0, 1]])?;
    /// assert_eq!(dropped.indices(), [1]);
    /// # drop(mesh);
    /// # Ok::<(), oxijolt::ShapeError>(())
    /// ```
    ///
    /// Meshes have no volume. They suit static bodies, and kinematic bodies with an explicit
    /// [`BodySettings::mass`](crate::BodySettings::mass);
    /// [`PhysicsWorld::create_body`](crate::PhysicsWorld::create_body) refuses them for
    /// dynamic bodies.
    ///
    /// # Errors
    /// - [`ShapeError::InvalidSettings`]: no vertices or no triangles, more than `i32::MAX` of
    ///   either, an index beyond `vertices`, or a setting out of range (see [`MeshSettings`]);
    /// - [`ShapeError::InvalidDimensions`]: a vertex (referenced or not) that is not finite or
    ///   has a component beyond [`limits::MAX_SHAPE_EXTENT`] in absolute value;
    /// - [`ShapeError::Mesh`]: no triangle is left after dropping small, thin and degenerate
    ///   ones;
    /// - [`ShapeError::Rejected`]: anything else Jolt refuses.
    ///
    /// Building cost grows with the triangle count; see [docs/benchmarks.md] and
    /// [docs/limits.md#triangle-meshes].
    ///
    /// [docs/benchmarks.md]: https://github.com/pockerhead/oxijolt/blob/main/docs/benchmarks.md
    /// [docs/limits.md#triangle-meshes]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#triangle-meshes
    pub fn new_mesh_with_settings(
        vertices: &[Vec3],
        triangles: &[[u32; 3]],
        settings: &MeshSettings<'_>,
    ) -> Result<(Self, DroppedTriangles), ShapeError> {
        validate_geometry(vertices, triangles)?;
        settings.validate(triangles.len())?;
        initialize()?;
        let (kept, dropped) = collidable_triangles(vertices, triangles, settings.max_convex_extent);
        if kept.is_empty() {
            return Err(ShapeError::Mesh(MeshError::NoTriangles));
        }
        let jolt_settings = mesh_settings(vertices, triangles, &kept, settings)?;
        let mesh: *mut JPH_MeshShapeSettings = jolt_settings.as_ptr();
        // Jolt's own clean-up has run. After `collidable_triangles` it can only drop duplicates,
        // which leaves a copy; the count is Jolt's own verdict on what is left.
        // SAFETY: the settings are live, owned by the guard and were created as mesh settings.
        if unsafe { JPH_MeshShapeSettings_GetTriangleCount(mesh) } == 0 {
            return Err(ShapeError::Mesh(MeshError::NoTriangles));
        }
        let shape = jolt_settings.create()?.within_extent_bounds()?;
        Ok((shape, dropped))
    }
}

/// The indices of the triangles Jolt can collide with reliably, in input order, and the rest.
///
/// Jolt stores vertices quantized to 21 bits over the bounds of the triangles it keeps, one step
/// per axis. A triangle that fails [`is_collidable`] without quantization fails with any, so the
/// bounds leave it out; the triangles kept then lie within bounds no larger than those Jolt
/// quantizes over. See [docs/limits.md#triangle-meshes].
///
/// [docs/limits.md#triangle-meshes]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#triangle-meshes
fn collidable_triangles(
    vertices: &[Vec3],
    triangles: &[[u32; 3]],
    convex_extent: f32,
) -> (Vec<usize>, DroppedTriangles) {
    let corners = |triangle: &[u32; 3]| triangle.map(|index| v3(vertices[index as usize]));
    let candidates = triangles
        .iter()
        .map(corners)
        .filter(|&corners| is_collidable(corners, [0.0; 3], convex_extent));
    let step = quantization_step(candidates);
    let mut kept = Vec::new();
    let mut dropped = DroppedTriangles::default();
    for (index, triangle) in triangles.iter().enumerate() {
        let corners = corners(triangle);
        if is_collidable(corners, step, convex_extent) {
            kept.push(index);
        } else {
            dropped.push(index, corners);
        }
    }
    (kept, dropped)
}

/// The step of Jolt's vertex quantization on each axis over the bounds of `triangles`; zero for
/// none.
fn quantization_step(triangles: impl Iterator<Item = [V3; 3]>) -> V3 {
    let mut low = [f64::INFINITY; 3];
    let mut high = [f64::NEG_INFINITY; 3];
    for vertex in triangles.flatten() {
        for axis in 0..3 {
            low[axis] = low[axis].min(vertex[axis]);
            high[axis] = high[axis].max(vertex[axis]);
        }
    }
    [0, 1, 2].map(|axis| ((high[axis] - low[axis]) / QUANTIZATION_STEPS).max(0.0))
}

/// How far Jolt's `f32` arithmetic can move a corner of a triangle on each axis when it collides
/// the triangle with a convex shape (`CollideConvexVsTriangles::Collide`): the transform of the
/// triangle's own coordinates, at most `distance` from their origin, and the rounding of the
/// result in the convex shape's space. There every coordinate is at most `convex_extent` plus
/// the triangle's `longest_edge`, because Jolt only goes on with triangles whose bounds overlap
/// the convex shape's.
fn rounding_displacement(distance: f64, convex_extent: f64, longest_edge: f64) -> f64 {
    let epsilon = f64::from(f32::EPSILON);
    4.0 * epsilon * distance + epsilon * (convex_extent + longest_edge)
}

/// Whether the triangle with `corners` keeps a cross product above [`MIN_TRIANGLE_CROSS`] under
/// Jolt's rounding against convex shapes up to `convex_extent`
/// ([`MeshSettings::max_convex_extent`]), with a factor [`CROSS_ERROR_MARGIN`] to spare.
///
/// Each corner can move by up to `step[axis]` on each axis (the quantization of a mesh being
/// built, zero for triangles Jolt has stored) plus [`rounding_displacement`]; an edge moves by
/// the difference of two corner moves. With edges `ab`, `ac` from one corner moved by `e1`,
/// `e2`, the cross product changes by `ab × e2 + e1 × ac + e1 × e2`. Its length is at least its
/// component along the unit normal `n`, which changes by `e2 · (n × ab) + e1 · (ac × n) + n ·
/// (e1 × e2)`: moves within the triangle's plane across an edge shrink it, moves out of the
/// plane do not. The cross product is the same from every corner, so the smallest of the three
/// corners' bounds holds, and the rule does not depend on the order of the corners. Jolt's `f32`
/// cross product, from whichever corner it starts, rounds it once more.
pub(super) fn is_collidable(corners: [V3; 3], step: V3, convex_extent: f32) -> bool {
    let [a, b, c] = corners;
    let normal = cross(sub(b, a), sub(c, a));
    let twice_area = length(normal);
    if twice_area < MIN_TRIANGLE_CROSS {
        return false;
    }
    let unit = normal.map(|component| component / twice_area);
    let distance = corners.map(length).into_iter().fold(0.0, f64::max);
    // The two edges leaving each corner, in the order that gives the same normal.
    let edge_pairs = [
        (sub(b, a), sub(c, a)),
        (sub(c, b), sub(a, b)),
        (sub(a, c), sub(b, c)),
    ];
    let longest_edge = edge_pairs
        .iter()
        .map(|&(edge, _)| length(edge))
        .fold(0.0, f64::max);
    let rounding = rounding_displacement(distance, f64::from(convex_extent), longest_edge);
    let edge_move = step.map(|axis_step| 2.0 * (axis_step + rounding));
    let shrink = edge_pairs
        .iter()
        .map(|&(first, second)| {
            reach(cross(unit, first), edge_move) + reach(cross(second, unit), edge_move)
        })
        .fold(f64::INFINITY, f64::min);
    let cross_rounding = edge_pairs
        .iter()
        .map(|&(first, second)| 2.0 * f64::from(f32::EPSILON) * length(first) * length(second))
        .fold(0.0, f64::max);
    let change = shrink + length_sq(edge_move) + cross_rounding;
    twice_area >= MIN_TRIANGLE_CROSS + CROSS_ERROR_MARGIN * change
}

/// The largest `|e · direction|` over every `e` with `|e[axis]| <= bound[axis]`.
fn reach(direction: V3, bound: V3) -> f64 {
    (0..3).map(|axis| bound[axis] * direction[axis].abs()).sum()
}

/// Jolt mesh settings holding the validated geometry, of the triangles `kept`, and `settings`.
fn mesh_settings(
    vertices: &[Vec3],
    triangles: &[[u32; 3]],
    kept: &[usize],
    settings: &MeshSettings<'_>,
) -> Result<ShapeSettings, ShapeError> {
    let jolt_vertices: Vec<JPH_Vec3> = vertices.iter().map(|vertex| vertex.to_jph()).collect();
    let material_indices = settings.materials.map(|(_, indices)| indices);
    let jolt_triangles: Vec<JPH_IndexedTriangle> = kept
        .iter()
        .map(|&i| {
            let [i1, i2, i3] = triangles[i];
            JPH_IndexedTriangle {
                i1,
                i2,
                i3,
                materialIndex: material_indices.map_or(0, |indices| u32::from(indices[i])),
                userData: 0,
            }
        })
        .collect();
    let list: Vec<*const JPH_PhysicsMaterial> = settings
        .materials
        .map(|(list, _)| list.iter().map(|material| material.as_ptr()).collect())
        .unwrap_or_default();
    // SAFETY: Jolt is initialised. Every array is live for the call and holds the count passed;
    // the counts fit `i32` and every index is below the vertex count (`validate_geometry`), and
    // every material index addresses `list`, or is 0 without one (`MeshSettings::validate`).
    // The materials are live, borrowed by `settings`; Jolt's list takes its own references.
    // The returned settings hold one reference, which the guard takes over.
    let jolt_settings = unsafe {
        ShapeSettings::from_raw(
            JPH_MeshShapeSettings_Create3(
                jolt_vertices.as_ptr(),
                jolt_vertices.len() as u32,
                jolt_triangles.as_ptr(),
                jolt_triangles.len() as u32,
                if list.is_empty() {
                    null()
                } else {
                    list.as_ptr()
                },
                list.len() as u32,
            )
            .cast(),
        )
    }?;
    let mesh: *mut JPH_MeshShapeSettings = jolt_settings.as_ptr();
    // SAFETY: the settings are live, owned by the guard, were created as mesh settings and have
    // not created a shape yet; every value was validated.
    unsafe {
        JPH_MeshShapeSettings_SetActiveEdgeCosThresholdAngle(
            mesh,
            settings.active_edge_cos_threshold_angle,
        );
        JPH_MeshShapeSettings_SetMaxTrianglesPerLeaf(mesh, settings.max_triangles_per_leaf);
        JPH_MeshShapeSettings_SetBuildQuality(mesh, settings.build_quality.to_jph());
    }
    Ok(jolt_settings)
}

#[cfg(test)]
mod tests;
