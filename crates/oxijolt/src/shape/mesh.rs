//! Triangle meshes.

use std::ptr::null;

use oxijolt_sys::*;

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
/// How many vertex displacements times the longest edge a triangle's cross product must keep
/// above [`MIN_TRIANGLE_CROSS`]; see [`collidable_triangles`].
const CROSS_ROUNDING_FACTOR: f64 = 8.0;

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
/// triangles per leaf, [`MeshBuildQuality::FavorRuntimePerformance`].
#[derive(Clone, Debug)]
pub struct MeshSettings<'a> {
    materials: Option<(&'a [&'a PhysicsMaterial], &'a [u8])>,
    active_edge_cos_threshold_angle: f32,
    max_triangles_per_leaf: u32,
    build_quality: MeshBuildQuality,
}

impl Default for MeshSettings<'_> {
    fn default() -> Self {
        Self {
            materials: None,
            active_edge_cos_threshold_angle: 0.996195,
            max_triangles_per_leaf: 8,
            build_quality: MeshBuildQuality::default(),
        }
    }
}

impl<'a> MeshSettings<'a> {
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

impl Shape {
    /// A triangle mesh of `vertices` (shape space, metres) and `triangles` (three indices into
    /// `vertices` each) with [`MeshSettings::default`]; see
    /// [`new_mesh_with_settings`](Self::new_mesh_with_settings).
    pub fn new_mesh(vertices: &[Vec3], triangles: &[[u32; 3]]) -> Result<Self, ShapeError> {
        Self::new_mesh_with_settings(vertices, triangles, &MeshSettings::default())
    }

    /// A triangle mesh of `vertices` (shape space, metres) and `triangles` (three indices into
    /// `vertices` each).
    ///
    /// A triangle's front face is the side from which its vertices run counter-clockwise.
    /// Triangles too small or too thin for Jolt to collide with reliably are dropped: twice
    /// their area must be at least 1e-5 m² plus a margin for Jolt's 21-bit vertex quantization
    /// and `f32` rounding, which grows with the mesh's size and distance from the shape origin
    /// ([docs/limits.md#triangle-meshes]). Jolt itself drops degenerate triangles (also those
    /// that become degenerate under its quantization) and duplicates, and reorders the rest,
    /// so sub-shape ids do not follow the input order. Closest-hit rays hit back faces too.
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
    /// - [`ShapeError::Mesh`]: no triangle is left after dropping small, thin, degenerate and
    ///   duplicate ones;
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
    ) -> Result<Self, ShapeError> {
        validate_geometry(vertices, triangles)?;
        settings.validate(triangles.len())?;
        initialize()?;
        let kept = collidable_triangles(vertices, triangles);
        if kept.is_empty() {
            return Err(ShapeError::Mesh(MeshError::NoTriangles));
        }
        let jolt_settings = mesh_settings(vertices, triangles, &kept, settings)?;
        let mesh: *mut JPH_MeshShapeSettings = jolt_settings.as_ptr();
        // Jolt's own clean-up has run; after `collidable_triangles` it can only drop duplicates.
        // SAFETY: the settings are live, owned by the guard and were created as mesh settings.
        if unsafe { JPH_MeshShapeSettings_GetTriangleCount(mesh) } == 0 {
            return Err(ShapeError::Mesh(MeshError::NoTriangles));
        }
        jolt_settings.create()?.within_extent_bounds()
    }
}

/// The indices of the triangles Jolt can collide with reliably, in input order.
///
/// Jolt stores vertices quantized to 21 bits over the mesh's bounds and transforms them into
/// the other shape's space in `f32` when it collides, so each vertex can move by about one
/// quantization step plus a few `f32` roundings of the largest coordinate. A triangle whose
/// cross product could shrink below Jolt's collision threshold under such moves is dropped like
/// a degenerate one: it is too small or too thin to collide with. See
/// [docs/limits.md#triangle-meshes].
///
/// [docs/limits.md#triangle-meshes]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#triangle-meshes
fn collidable_triangles(vertices: &[Vec3], triangles: &[[u32; 3]]) -> Vec<usize> {
    let corner = |index: u32| {
        let v = vertices[index as usize];
        [v.x, v.y, v.z].map(f64::from)
    };
    let mut low = [f64::INFINITY; 3];
    let mut high = [f64::NEG_INFINITY; 3];
    for &index in triangles.iter().flatten() {
        let v = corner(index);
        for axis in 0..3 {
            low[axis] = low[axis].min(v[axis]);
            high[axis] = high[axis].max(v[axis]);
        }
    }
    let step = (0..3)
        .map(|axis| (high[axis] - low[axis]) / f64::from((1u32 << 21) - 1))
        .fold(0.0, f64::max);
    let largest = (0..3)
        .map(|axis| low[axis].abs().max(high[axis].abs()))
        .fold(0.0, f64::max);
    let displacement = step + 4.0 * f64::from(f32::EPSILON) * largest;
    triangles
        .iter()
        .enumerate()
        .filter(|(_, triangle)| {
            let [a, b, c] = triangle.map(corner);
            let (ab, ac, bc) = (sub(b, a), sub(c, a), sub(c, b));
            let longest = [ab, ac, bc]
                .map(|edge| dot(edge, edge).sqrt())
                .into_iter()
                .fold(0.0, f64::max);
            let normal = cross(ab, ac);
            dot(normal, normal).sqrt()
                >= MIN_TRIANGLE_CROSS + CROSS_ROUNDING_FACTOR * displacement * longest
        })
        .map(|(index, _)| index)
        .collect()
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
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
