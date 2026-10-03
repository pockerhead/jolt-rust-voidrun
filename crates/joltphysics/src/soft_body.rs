//! Soft bodies: shared particle settings, creation, and read and write access to vertices.
//!
//! A soft body is an ordinary body of its world: [`PhysicsWorld::create_soft_body`] returns a
//! [`BodyId`], and the body counts against the world's body limit, is removed with
//! [`PhysicsWorld::remove_body`], saved by [`PhysicsWorld::save_state`], moved by
//! [`PhysicsWorld::rebase`] and found by queries. Its particles are described by
//! [`SoftBodySharedSettings`], which one or many soft bodies in any number of worlds share.

use std::collections::BTreeMap;
use std::marker::PhantomData;
use std::ops::Deref;
use std::ptr::NonNull;

use joltphysics_sys::*;

use crate::body::{
    with_locked_body, with_read_locked_body, Activation, INVALID_BODY_ID, LINEAR_VELOCITY_RULE,
};
use crate::limits::{
    self, is_compliance, is_friction, is_gravity_factor, is_in_frame, is_linear_velocity,
    is_local_distance, is_local_offset, is_soft_body_force, is_soft_body_pressure,
    is_vertex_inverse_mass, SoftBodyPressureGeometry,
};
use crate::math::{is_finite_non_negative, jolt_length};
use crate::owned::{JoltObject, Owned};
use crate::world::ensure_initialized;
use crate::{BodyError, BodyId, ObjectLayer, PhysicsWorld, Quat, RVec3, Real, SoftBodyError, Vec3};

/// One particle of a soft body as [`SoftBodySharedSettings`] describe it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SoftBodyVertex {
    /// Position relative to the body origin, in metres, every component at most
    /// [`limits::MAX_SHAPE_EXTENT`] in absolute value.
    pub position: Vec3,
    /// Initial velocity relative to the body, in m/s, at most
    /// [`limits::MAX_LINEAR_VELOCITY`] long.
    pub velocity: Vec3,
    /// Inverse mass in 1/kg: 0 for a kinematic vertex, which only moves by its velocity, or the
    /// inverse of a mass within [`limits::MIN_MASS`]`..=`[`limits::MAX_MASS`].
    pub inverse_mass: f32,
}

impl SoftBodyVertex {
    /// A vertex of 1 kg at rest at `position`, Jolt's default vertex.
    pub fn new(position: Vec3) -> Self {
        Self {
            position,
            velocity: Vec3::ZERO,
            inverse_mass: 1.0,
        }
    }

    /// A kinematic vertex (inverse mass 0) at rest at `position`: it keeps its place unless
    /// it is given a velocity.
    pub fn kinematic(position: Vec3) -> Self {
        Self {
            inverse_mass: 0.0,
            ..Self::new(position)
        }
    }

    fn to_jph(self) -> JPH_SoftVertex {
        JPH_SoftVertex {
            position: self.position.to_jph(),
            velocity: self.velocity.to_jph(),
            invMass: self.inverse_mass,
        }
    }
}

/// Which bend constraints [`SoftBodySharedSettingsBuilder::create_constraints`] creates between
/// two faces that share an edge (Jolt `SoftBodySharedSettings::EBendType`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum SoftBodyBendType {
    /// No bend constraints.
    None,
    /// An edge between the two vertices opposite the shared edge, the cheapest.
    Distance,
    /// A dihedral angle constraint, which also keeps a fold between faces that start out of
    /// one plane; the most expensive.
    Dihedral,
}

impl SoftBodyBendType {
    fn to_jph(self) -> JPH_SoftBodyBendType {
        match self {
            Self::None => JPH_SoftBodyBendType_None,
            Self::Distance => JPH_SoftBodyBendType_Distance,
            Self::Dihedral => JPH_SoftBodyBendType_Dihedral,
        }
    }
}

/// Which long range attachment (LRA) constraint ties a movable vertex to the closest kinematic
/// vertex (Jolt `SoftBodySharedSettings::ELRAType`). An LRA constraint keeps the vertex within
/// its rest distance from that kinematic vertex times a multiplier, so a hanging cloth does not
/// stretch under its own weight. The anchors are chosen when the settings are built.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum LongRangeAttachment {
    /// No LRA constraint.
    None,
    /// The rest distance is the straight-line distance to the closest kinematic vertex.
    EuclideanDistance,
    /// The rest distance is measured along the edges to the closest kinematic vertex.
    GeodesicDistance,
}

impl LongRangeAttachment {
    fn to_jph(self) -> JPH_SoftBodyLRAType {
        match self {
            Self::None => JPH_SoftBodyLRAType_None,
            Self::EuclideanDistance => JPH_SoftBodyLRAType_EuclideanDistance,
            Self::GeodesicDistance => JPH_SoftBodyLRAType_GeodesicDistance,
        }
    }
}

/// How [`SoftBodySharedSettingsBuilder::create_constraints`] builds the constraints at a vertex
/// (Jolt `SoftBodySharedSettings::VertexAttributes`).
///
/// Compliances are inverse stiffnesses in the units of each constraint's own equation (an edge:
/// metres per newton; bend and volume constraints differ); 0 is rigid. Each must be finite and
/// within `0..=`[`limits::MAX_COMPLIANCE`]. An edge or shear edge uses the average of its two
/// vertices' compliances, a bend constraint the average over the shared edge.
///
/// The defaults are Jolt's: rigid edges and shear edges, no bend constraints, no LRA
/// constraint and an LRA multiplier of 1.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SoftBodyVertexAttributes {
    compliance: f32,
    shear_compliance: f32,
    bend_compliance: Option<f32>,
    long_range_attachment: LongRangeAttachment,
    lra_multiplier: f32,
}

impl Default for SoftBodyVertexAttributes {
    fn default() -> Self {
        Self {
            compliance: 0.0,
            shear_compliance: 0.0,
            bend_compliance: None,
            long_range_attachment: LongRangeAttachment::None,
            lra_multiplier: 1.0,
        }
    }
}

impl SoftBodyVertexAttributes {
    /// Compliance of the edges along the faces. Default 0.
    #[must_use]
    pub fn compliance(mut self, value: f32) -> Self {
        self.compliance = value;
        self
    }

    /// Compliance of the shear edges, the diagonals of two faces that form a quad. Default 0.
    #[must_use]
    pub fn shear_compliance(mut self, value: f32) -> Self {
        self.shear_compliance = value;
        self
    }

    /// Compliance of the bend constraints; `None` creates none for edges at this vertex (Jolt's
    /// `FLT_MAX`). Default `None`.
    #[must_use]
    pub fn bend_compliance(mut self, value: Option<f32>) -> Self {
        self.bend_compliance = value;
        self
    }

    /// The LRA constraint of this vertex and its multiplier of the rest distance, finite and
    /// within `1..=`[`limits::MAX_RATIO`] (1.01 lets the vertex move 1 % further away).
    /// Default [`LongRangeAttachment::None`] with multiplier 1.
    ///
    /// The bound keeps Jolt's arithmetic finite: an LRA rest distance is at most the sum of all
    /// edge lengths, below `2³² · 2√3 ·` [`limits::MAX_SHAPE_EXTENT`] (about 3e13 m); times the
    /// multiplier it is at most 3e17 m, and Jolt's square of it (`SoftBodyMotionProperties.cpp:695`)
    /// stays below 1e35.
    #[must_use]
    pub fn long_range_attachment(mut self, kind: LongRangeAttachment, multiplier: f32) -> Self {
        self.long_range_attachment = kind;
        self.lra_multiplier = multiplier;
        self
    }

    fn validate(&self) -> Result<(), SoftBodyError> {
        let compliances = [self.compliance, self.shear_compliance];
        require(
            compliances
                .into_iter()
                .chain(self.bend_compliance)
                .all(is_compliance),
            COMPLIANCE_RULE,
        )?;
        require(
            (1.0..=limits::MAX_RATIO).contains(&self.lra_multiplier),
            "an LRA multiplier must be finite and within 1..=limits::MAX_RATIO",
        )
    }

    fn to_jph(self) -> JPH_SoftBodyVertexAttributes {
        JPH_SoftBodyVertexAttributes {
            compliance: self.compliance,
            shearCompliance: self.shear_compliance,
            bendCompliance: self.bend_compliance.unwrap_or(f32::MAX),
            lraType: self.long_range_attachment.to_jph(),
            lraMaxDistanceMultiplier: self.lra_multiplier,
        }
    }
}

/// What a constraint compliance must satisfy.
const COMPLIANCE_RULE: &str = "a compliance must be finite and within 0..=limits::MAX_COMPLIANCE";

/// What [`SoftBodySharedSettingsBuilder::create_constraints`] or
/// [`create_constraints_per_vertex`](SoftBodySharedSettingsBuilder::create_constraints_per_vertex)
/// asked for.
#[derive(Clone, Debug)]
enum GeneratedConstraints {
    Uniform(SoftBodyBendType, SoftBodyVertexAttributes),
    PerVertex(SoftBodyBendType, Vec<SoftBodyVertexAttributes>),
}

/// An edge constraint the caller adds explicitly (Jolt `SoftBodySharedSettings::Edge`): keeps
/// two vertices at their rest distance, measured when the settings are built.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SoftBodyEdge {
    /// The two vertices, different, at least [`limits::MIN_SOFT_BODY_EDGE_LENGTH`] apart.
    pub vertices: [u32; 2],
    /// Compliance in m/N, `0..=`[`limits::MAX_COMPLIANCE`]; 0 is rigid.
    pub compliance: f32,
}

/// A dihedral bend constraint the caller adds explicitly (Jolt
/// `SoftBodySharedSettings::DihedralBend`): keeps the angle between two triangles that share an
/// edge at its rest angle.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SoftBodyDihedralBend {
    /// Four different vertices; the first two are the shared edge, at least
    /// [`limits::MIN_SOFT_BODY_EDGE_LENGTH`] long, the last two the vertices opposite it.
    pub vertices: [u32; 4],
    /// Compliance (inverse stiffness of the angle constraint), `0..=`[`limits::MAX_COMPLIANCE`];
    /// 0 is rigid.
    pub compliance: f32,
}

/// A volume constraint the caller adds explicitly (Jolt `SoftBodySharedSettings::Volume`):
/// keeps the volume of a tetrahedron at its rest volume.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SoftBodyVolume {
    /// Four different vertices that span a tetrahedron with a volume.
    pub vertices: [u32; 4],
    /// Compliance (inverse stiffness of the volume constraint), `0..=`[`limits::MAX_COMPLIANCE`];
    /// 0 is rigid.
    pub compliance: f32,
}

/// The particles of a soft body and the constraints between them (Jolt
/// `SoftBodySharedSettings`), built and checked by [`SoftBodySharedSettings::builder`].
///
/// Owns one Jolt reference. Every soft body created from the settings holds its own, so the
/// settings may be dropped while bodies use them. They cannot change after they are built, and
/// one value may serve many bodies in many worlds on many threads.
pub struct SoftBodySharedSettings {
    settings: Owned<JPH_SoftBodySharedSettings>,
    pressure_geometry: SoftBodyPressureGeometry,
}

// SAFETY: the settings are never changed after `build` returns (no method takes `&mut self`, and
// Jolt requires shared settings to stay constant while bodies use them, `Docs/Architecture.md:427`),
// no Jolt member of `SoftBodySharedSettings` is `mutable`, and `RefTarget` counts references
// atomically, so the settings may be used and released from any thread.
unsafe impl Send for SoftBodySharedSettings {}
// SAFETY: as for `Send`; `&SoftBodySharedSettings` only lets bodies take further references and
// read the immutable settings.
unsafe impl Sync for SoftBodySharedSettings {}

/// Shared settings, of which the owner holds one reference.
impl JoltObject for JPH_SoftBodySharedSettings {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner holds the one reference `JPH_SoftBodySharedSettings_Create` added
        // (trait contract), released here once; creation settings and bodies keep their own
        // (`SoftBodyCreationSettings::mSettings` is a `RefConst`).
        unsafe { JPH_SoftBodySharedSettings_Destroy(ptr) };
    }
}

impl SoftBodySharedSettings {
    /// Starts settings with `vertices` and the triangle `faces` between them, given as vertex
    /// indices. Faces are what other bodies collide with and what queries hit; without a call
    /// to [`create_constraints`](SoftBodySharedSettingsBuilder::create_constraints) or explicit
    /// constraints ([`edge`](SoftBodySharedSettingsBuilder::edge),
    /// [`dihedral_bend`](SoftBodySharedSettingsBuilder::dihedral_bend),
    /// [`volume`](SoftBodySharedSettingsBuilder::volume)) the particles are not tied together.
    ///
    /// ```
    /// use joltphysics::*;
    ///
    /// # fn main() -> Result<(), Box<dyn std::error::Error>> {
    /// // A square of two triangles, pinned at one corner.
    /// let vertices = vec![
    ///     SoftBodyVertex::kinematic(Vec3::new(0.0, 0.0, 0.0)),
    ///     SoftBodyVertex::new(Vec3::new(1.0, 0.0, 0.0)),
    ///     SoftBodyVertex::new(Vec3::new(0.0, 0.0, 1.0)),
    ///     SoftBodyVertex::new(Vec3::new(1.0, 0.0, 1.0)),
    /// ];
    /// let settings = SoftBodySharedSettings::builder(vertices, vec![[0, 2, 3], [0, 3, 1]])
    ///     .create_constraints(
    ///         SoftBodyBendType::Distance,
    ///         SoftBodyVertexAttributes::default().bend_compliance(Some(1.0e-4)),
    ///     )
    ///     .build()?;
    /// assert_eq!(settings.vertex_count(), 4);
    /// assert!(settings.edge_constraint_count() >= 5);
    /// # Ok(())
    /// # }
    /// ```
    pub fn builder(
        vertices: Vec<SoftBodyVertex>,
        faces: Vec<[u32; 3]>,
    ) -> SoftBodySharedSettingsBuilder {
        SoftBodySharedSettingsBuilder {
            vertices,
            faces,
            generated: None,
            edges: Vec::new(),
            dihedral_bends: Vec::new(),
            volumes: Vec::new(),
        }
    }

    pub(crate) fn as_ptr(&self) -> *const JPH_SoftBodySharedSettings {
        self.settings.as_ptr()
    }

    /// Number of vertices.
    pub fn vertex_count(&self) -> usize {
        // SAFETY: the settings are live and immutable; the getter only reads them.
        unsafe { JPH_SoftBodySharedSettings_GetVertexCount(self.as_ptr()) as usize }
    }

    /// Number of faces.
    pub fn face_count(&self) -> usize {
        // SAFETY: as in `vertex_count`.
        unsafe { JPH_SoftBodySharedSettings_GetFaceCount(self.as_ptr()) as usize }
    }

    /// Number of edge constraints, the generated edges, shear edges and distance bends
    /// included.
    pub fn edge_constraint_count(&self) -> usize {
        // SAFETY: as in `vertex_count`.
        unsafe { JPH_SoftBodySharedSettings_GetEdgeConstraintCount(self.as_ptr()) as usize }
    }

    /// Number of dihedral bend constraints.
    pub fn dihedral_bend_constraint_count(&self) -> usize {
        // SAFETY: as in `vertex_count`.
        unsafe { JPH_SoftBodySharedSettings_GetDihedralBendConstraintCount(self.as_ptr()) as usize }
    }

    /// Number of volume constraints.
    pub fn volume_constraint_count(&self) -> usize {
        // SAFETY: as in `vertex_count`.
        unsafe { JPH_SoftBodySharedSettings_GetVolumeConstraintCount(self.as_ptr()) as usize }
    }

    /// Number of long range attachment constraints.
    pub fn lra_constraint_count(&self) -> usize {
        // SAFETY: as in `vertex_count`.
        unsafe { JPH_SoftBodySharedSettings_GetLRAConstraintCount(self.as_ptr()) as usize }
    }
}

/// Collects the vertices, faces and constraints of [`SoftBodySharedSettings`]; made by
/// [`SoftBodySharedSettings::builder`].
#[derive(Clone, Debug)]
#[must_use]
pub struct SoftBodySharedSettingsBuilder {
    vertices: Vec<SoftBodyVertex>,
    faces: Vec<[u32; 3]>,
    generated: Option<GeneratedConstraints>,
    edges: Vec<SoftBodyEdge>,
    dihedral_bends: Vec<SoftBodyDihedralBend>,
    volumes: Vec<SoftBodyVolume>,
}

impl SoftBodySharedSettingsBuilder {
    /// Creates edge, shear, bend and LRA constraints from the faces, every vertex with the
    /// same `attributes` (Jolt `SoftBodySharedSettings::CreateConstraints`, with its default
    /// angle tolerance of 8° for detecting quads). Replaces an earlier call.
    pub fn create_constraints(
        mut self,
        bend: SoftBodyBendType,
        attributes: SoftBodyVertexAttributes,
    ) -> Self {
        self.generated = Some(GeneratedConstraints::Uniform(bend, attributes));
        self
    }

    /// Like [`create_constraints`](Self::create_constraints) with one attribute set per vertex,
    /// in vertex order; there must be exactly as many as vertices.
    pub fn create_constraints_per_vertex(
        mut self,
        bend: SoftBodyBendType,
        attributes: Vec<SoftBodyVertexAttributes>,
    ) -> Self {
        self.generated = Some(GeneratedConstraints::PerVertex(bend, attributes));
        self
    }

    /// Adds an edge constraint, kept in addition to the generated ones. Long range attachments
    /// follow the generated edges only.
    pub fn edge(mut self, edge: SoftBodyEdge) -> Self {
        self.edges.push(edge);
        self
    }

    /// Adds a dihedral bend constraint, kept in addition to the generated ones.
    pub fn dihedral_bend(mut self, bend: SoftBodyDihedralBend) -> Self {
        self.dihedral_bends.push(bend);
        self
    }

    /// Adds a volume constraint, for example one per tetrahedron of a solid soft body.
    pub fn volume(mut self, volume: SoftBodyVolume) -> Self {
        self.volumes.push(volume);
        self
    }

    /// Checks everything and builds the settings; nothing is allocated when a check fails.
    ///
    /// Fails with [`SoftBodyError::InvalidValue`] when there is no vertex; when a vertex
    /// position is beyond [`limits::MAX_SHAPE_EXTENT`], a velocity beyond
    /// [`limits::MAX_LINEAR_VELOCITY`] or an inverse mass neither 0 nor the inverse of a mass
    /// within [`limits::MIN_MASS`]`..=`[`limits::MAX_MASS`]; when the masses of the movable
    /// vertices add up to more than [`limits::MAX_MASS`] (Jolt gives the body their sum as its
    /// mass); when a face names a vertex that does not exist, names one vertex twice, has an
    /// edge shorter than [`limits::MIN_SOFT_BODY_EDGE_LENGTH`] or no area; with
    /// [`SoftBodyBendType::Distance`], when the vertices opposite a shared edge are closer than
    /// that length (the bend edge between them would have no length); when the
    /// attributes are out of range or, per vertex, not one per vertex; and when an explicit
    /// constraint names a vertex that does not exist or one vertex twice, has a compliance out
    /// of range, an edge (or a bend's shared edge) shorter than
    /// [`limits::MIN_SOFT_BODY_EDGE_LENGTH`], or a tetrahedron without volume.
    pub fn build(self) -> Result<SoftBodySharedSettings, SoftBodyError> {
        if !ensure_initialized() {
            return Err(SoftBodyError::InitFailed);
        }
        self.validate()?;
        let positions: Vec<Vec3> = self.vertices.iter().map(|v| v.position).collect();
        let pressure_geometry = SoftBodyPressureGeometry::new(&positions, &self.faces);
        // SAFETY: Jolt is initialised. The handle takes over the one reference joltc's
        // `_Create` adds.
        let owned = unsafe { Owned::from_raw(JPH_SoftBodySharedSettings_Create()) }
            .unwrap_or_else(|| unreachable!("`new` does not return null"));
        let settings = SoftBodySharedSettings {
            settings: owned,
            pressure_geometry,
        };
        let ptr = settings.settings.as_ptr();
        let vertices: Vec<JPH_SoftVertex> = self.vertices.iter().map(|v| v.to_jph()).collect();
        let faces: Vec<JPH_SoftFace> = self
            .faces
            .iter()
            .map(|&[vertex1, vertex2, vertex3]| JPH_SoftFace {
                vertex1,
                vertex2,
                vertex3,
                // joltc's `_Create` adds Jolt's default material at index 0.
                materialIndex: 0,
            })
            .collect();
        // SAFETY: `ptr` is the live settings object owned by `settings`, which nothing else
        // uses yet. Both arrays are live for the calls and hold the counts passed, which
        // `validate` checked to fit `u32`; every face index names a vertex.
        unsafe {
            JPH_SoftBodySharedSettings_AddVertices(ptr, vertices.as_ptr(), vertices.len() as u32);
            JPH_SoftBodySharedSettings_AddFaces(ptr, faces.as_ptr(), faces.len() as u32);
        }
        if let Some(generated) = &self.generated {
            let (bend, attributes) = match generated {
                GeneratedConstraints::Uniform(bend, attributes) => {
                    (bend, vec![attributes.to_jph()])
                }
                GeneratedConstraints::PerVertex(bend, attributes) => {
                    (bend, attributes.iter().map(|a| a.to_jph()).collect())
                }
            };
            // SAFETY: `ptr` is live and holds the checked vertices and faces; `attributes` holds
            // at least one set (`validate`) and lives for the call.
            unsafe {
                JPH_SoftBodySharedSettings_CreateConstraints2(
                    ptr,
                    attributes.as_ptr(),
                    attributes.len() as u32,
                    bend.to_jph(),
                );
            }
        }
        // Generating constraints replaces the edges, so explicit ones come after it.
        self.add_explicit_constraints(ptr);
        // SAFETY: `ptr` is live; Jolt reorders the constraints for its solver, the last change
        // the settings get.
        unsafe { JPH_SoftBodySharedSettings_Optimize(ptr) };
        Ok(settings)
    }

    /// Adds the explicit constraints to `ptr` and computes their rest values.
    fn add_explicit_constraints(&self, ptr: *mut JPH_SoftBodySharedSettings) {
        // SAFETY: `ptr` is the live settings object `build` owns, which holds the checked
        // vertices; `validate` checked every index and the geometry the rest values need.
        unsafe {
            for edge in &self.edges {
                let [a, b] = edge.vertices;
                JPH_SoftBodySharedSettings_AddEdgeConstraint(ptr, a, b, edge.compliance);
            }
            for bend in &self.dihedral_bends {
                let [a, b, c, d] = bend.vertices;
                JPH_SoftBodySharedSettings_AddDihedralBendConstraint(
                    ptr,
                    a,
                    b,
                    c,
                    d,
                    bend.compliance,
                );
            }
            for volume in &self.volumes {
                let [a, b, c, d] = volume.vertices;
                JPH_SoftBodySharedSettings_AddVolumeConstraint(ptr, a, b, c, d, volume.compliance);
            }
            if !self.edges.is_empty() {
                JPH_SoftBodySharedSettings_CalculateEdgeLengths(ptr);
            }
            if !self.dihedral_bends.is_empty() {
                JPH_SoftBodySharedSettings_CalculateBendConstraintConstants(ptr);
            }
            if !self.volumes.is_empty() {
                JPH_SoftBodySharedSettings_CalculateVolumeConstraintVolumes(ptr);
            }
        }
    }

    fn validate(&self) -> Result<(), SoftBodyError> {
        require(
            !self.vertices.is_empty(),
            "a soft body needs at least one vertex",
        )?;
        require(
            u32::try_from(self.vertices.len()).is_ok() && u32::try_from(self.faces.len()).is_ok(),
            "vertex and face counts must fit u32",
        )?;
        let mut total_mass = 0.0_f64;
        for vertex in &self.vertices {
            require(
                is_local_offset(vertex.position),
                "a vertex position must be finite and within limits::MAX_SHAPE_EXTENT",
            )?;
            require(is_linear_velocity(vertex.velocity), LINEAR_VELOCITY_RULE)?;
            if vertex.inverse_mass != 0.0 {
                require(
                    is_vertex_inverse_mass(vertex.inverse_mass),
                    VERTEX_INVERSE_MASS_RULE,
                )?;
                total_mass += vertex_mass(vertex.inverse_mass);
            }
        }
        require(total_mass <= f64::from(limits::MAX_MASS), TOTAL_MASS_RULE)?;
        for &face in &self.faces {
            self.validate_face(face)?;
        }
        if let Some(generated) = &self.generated {
            let bend = match generated {
                GeneratedConstraints::Uniform(bend, attributes) => {
                    attributes.validate()?;
                    *bend
                }
                GeneratedConstraints::PerVertex(bend, attributes) => {
                    require(
                        attributes.len() == self.vertices.len(),
                        "per-vertex attributes need one set per vertex",
                    )?;
                    for attributes in attributes {
                        attributes.validate()?;
                    }
                    *bend
                }
            };
            if bend == SoftBodyBendType::Distance {
                self.validate_distance_bends()?;
            }
        }
        self.validate_explicit_constraints()
    }

    /// Whether `indices` name different vertices.
    fn names_different_vertices(&self, indices: &[u32]) -> Result<(), SoftBodyError> {
        let count = self.vertices.len();
        require(
            indices.iter().all(|&index| (index as usize) < count),
            "a constraint index must name a vertex",
        )?;
        let distinct = indices
            .iter()
            .enumerate()
            .all(|(i, a)| indices[i + 1..].iter().all(|b| a != b));
        require(distinct, "a constraint must name different vertices")
    }

    fn validate_explicit_constraints(&self) -> Result<(), SoftBodyError> {
        for edge in &self.edges {
            self.names_different_vertices(&edge.vertices)?;
            require(is_compliance(edge.compliance), COMPLIANCE_RULE)?;
            let [a, b] = edge.vertices.map(|index| self.position(index));
            require(is_edge_length(a, b), EDGE_LENGTH_RULE)?;
        }
        for bend in &self.dihedral_bends {
            self.names_different_vertices(&bend.vertices)?;
            require(is_compliance(bend.compliance), COMPLIANCE_RULE)?;
            let [a, b, _, _] = bend.vertices.map(|index| self.position(index));
            require(
                is_edge_length(a, b),
                "the shared edge of a bend must be at least limits::MIN_SOFT_BODY_EDGE_LENGTH long",
            )?;
        }
        for volume in &self.volumes {
            self.names_different_vertices(&volume.vertices)?;
            require(is_compliance(volume.compliance), COMPLIANCE_RULE)?;
            let [x1, x2, x3, x4] = volume.vertices.map(|index| self.position(index));
            // Jolt's six times the rest volume, in f32 (`CalculateVolumeConstraintVolumes`).
            let six_volume = dot(
                cross(difference(x1, x2), difference(x1, x3)),
                difference(x1, x4),
            );
            require(
                six_volume.is_finite() && six_volume != 0.0,
                "a volume constraint must span a tetrahedron with a volume",
            )?;
        }
        Ok(())
    }

    fn position(&self, index: u32) -> Vec3 {
        self.vertices[index as usize].position
    }

    fn validate_face(&self, face: [u32; 3]) -> Result<(), SoftBodyError> {
        let count = self.vertices.len();
        require(
            face.iter().all(|&index| (index as usize) < count),
            "a face index must name a vertex",
        )?;
        let [a, b, c] = face;
        require(
            a != b && b != c && a != c,
            "a face must name three different vertices",
        )?;
        let [pa, pb, pc] = face.map(|index| self.position(index));
        for (from, to) in [(pa, pb), (pb, pc), (pc, pa)] {
            require(is_edge_length(from, to), EDGE_LENGTH_RULE)?;
        }
        let area = jolt_length(cross(difference(pa, pb), difference(pa, pc)));
        require(area.is_finite() && area > 0.0, "a face must have an area")
    }

    /// With distance bends, Jolt joins the two vertices opposite each shared edge by an edge
    /// (`SoftBodySharedSettings.cpp:295-302`), which must have a length like every edge.
    fn validate_distance_bends(&self) -> Result<(), SoftBodyError> {
        let mut opposite: BTreeMap<(u32, u32), Vec<u32>> = BTreeMap::new();
        for face in &self.faces {
            for i in 0..3 {
                let (v0, v1) = (face[i], face[(i + 1) % 3]);
                opposite
                    .entry((v0.min(v1), v0.max(v1)))
                    .or_default()
                    .push(face[(i + 2) % 3]);
            }
        }
        for vertices in opposite.values() {
            for (i, &first) in vertices.iter().enumerate() {
                for &second in &vertices[i + 1..] {
                    if first != second {
                        require(
                            is_edge_length(self.position(first), self.position(second)),
                            "with distance bends, the vertices opposite a shared edge must be \
                             at least limits::MIN_SOFT_BODY_EDGE_LENGTH apart",
                        )?;
                    }
                }
            }
        }
        Ok(())
    }
}

/// What the inverse mass of a soft body vertex must satisfy.
const VERTEX_INVERSE_MASS_RULE: &str =
    "a vertex inverse mass must be 0 or the inverse of a mass within limits::MIN_MASS..=limits::MAX_MASS";
/// What the vertex masses of a soft body must satisfy together.
const TOTAL_MASS_RULE: &str =
    "the masses of the movable vertices must add up to at most limits::MAX_MASS";
/// What an edge between two vertices must satisfy.
const EDGE_LENGTH_RULE: &str = "an edge must be at least limits::MIN_SOFT_BODY_EDGE_LENGTH long";

/// The mass of a movable vertex as Jolt computes it (`1.0f / mInvMass`,
/// `SoftBodyMotionProperties.cpp:33`), for summing in `f64`.
fn vertex_mass(inverse_mass: f32) -> f64 {
    f64::from(1.0 / inverse_mass)
}

fn require(valid: bool, what: &'static str) -> Result<(), SoftBodyError> {
    if valid {
        Ok(())
    } else {
        Err(SoftBodyError::InvalidValue(what))
    }
}

/// `to - from` in `f32`, as Jolt subtracts vertex positions.
fn difference(from: Vec3, to: Vec3) -> Vec3 {
    Vec3::new(to.x - from.x, to.y - from.y, to.z - from.z)
}

fn dot(a: Vec3, b: Vec3) -> f32 {
    a.x * b.x + a.y * b.y + a.z * b.z
}

fn cross(a: Vec3, b: Vec3) -> Vec3 {
    Vec3::new(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    )
}

/// Whether `from` and `to` are at least [`limits::MIN_SOFT_BODY_EDGE_LENGTH`] apart, measured
/// as Jolt measures a rest length: an `f32` difference and Jolt's own `f32` length.
fn is_edge_length(from: Vec3, to: Vec3) -> bool {
    jolt_length(difference(from, to)) >= limits::MIN_SOFT_BODY_EDGE_LENGTH
}

/// How to create a soft body from [`SoftBodySharedSettings`].
///
/// The defaults are those of Jolt's `SoftBodyCreationSettings`, except the object layer: Jolt
/// uses layer 0, joltphysics [`ObjectLayer::MOVING`].
#[derive(Clone, Debug, PartialEq)]
pub struct SoftBodySettings {
    position: RVec3,
    rotation: Quat,
    object_layer: ObjectLayer,
    num_iterations: u32,
    linear_damping: f32,
    max_linear_velocity: f32,
    restitution: f32,
    friction: f32,
    pressure: f32,
    gravity_factor: f32,
    vertex_radius: f32,
    update_position: bool,
    make_rotation_identity: bool,
    allow_sleeping: bool,
    faces_double_sided: bool,
    activation: Activation,
}

impl Default for SoftBodySettings {
    fn default() -> Self {
        Self {
            position: RVec3::ZERO,
            rotation: Quat::IDENTITY,
            object_layer: ObjectLayer::MOVING,
            num_iterations: 5,
            linear_damping: 0.1,
            max_linear_velocity: 500.0,
            restitution: 0.0,
            friction: 0.2,
            pressure: 0.0,
            gravity_factor: 1.0,
            vertex_radius: 0.0,
            update_position: true,
            make_rotation_identity: true,
            allow_sleeping: true,
            faces_double_sided: false,
            activation: Activation::Activate,
        }
    }
}

impl SoftBodySettings {
    /// Largest number of solver iterations per step.
    ///
    /// Crate policy: Jolt divides the step into this many sub-steps, which bounds the smallest
    /// sub-step that [`limits::MAX_COMPLIANCE`] is derived for.
    pub const MAX_ITERATIONS: u32 = 100;

    /// Initial position of the body origin in metres, every component at most
    /// [`limits::MAX_POSITION`] in absolute value. Default the origin.
    #[must_use]
    pub fn position(mut self, value: RVec3) -> Self {
        self.position = value;
        self
    }

    /// Initial rotation, a unit quaternion. Default identity.
    #[must_use]
    pub fn rotation(mut self, value: Quat) -> Self {
        self.rotation = value;
        self
    }

    /// The object layer, which must exist in the world's
    /// [`CollisionLayers`](crate::CollisionLayers). Default [`ObjectLayer::MOVING`].
    #[must_use]
    pub fn object_layer(mut self, value: ObjectLayer) -> Self {
        self.object_layer = value;
        self
    }

    /// Solver iterations per step, `1..=`[`MAX_ITERATIONS`](Self::MAX_ITERATIONS). Default 5.
    #[must_use]
    pub fn num_iterations(mut self, value: u32) -> Self {
        self.num_iterations = value;
        self
    }

    /// Linear damping of every vertex, finite and at least 0: Jolt scales the vertex velocity by
    /// `max(0, 1 - c * dt)` every sub-step. Default 0.1.
    #[must_use]
    pub fn linear_damping(mut self, value: f32) -> Self {
        self.linear_damping = value;
        self
    }

    /// Largest speed of a vertex in m/s, above 0 and at most [`limits::MAX_LINEAR_VELOCITY`].
    /// Default 500.
    #[must_use]
    pub fn max_linear_velocity(mut self, value: f32) -> Self {
        self.max_linear_velocity = value;
        self
    }

    /// Restitution (bounciness) of collisions, between 0 and 1. Default 0.
    #[must_use]
    pub fn restitution(mut self, value: f32) -> Self {
        self.restitution = value;
        self
    }

    /// Friction coefficient of collisions, between 0 and [`limits::MAX_FRICTION`]. Default 0.2.
    #[must_use]
    pub fn friction(mut self, value: f32) -> Self {
        self.friction = value;
        self
    }

    /// Pressure coefficient of a closed body (`n · R · T`), finite and within
    /// `0..=`[`limits::MAX_SOFT_BODY_PRESSURE`]; 0 applies no pressure. Jolt pushes the faces
    /// outwards with it divided by the enclosed volume. Default 0.
    ///
    /// Above 0, [`PhysicsWorld::create_soft_body`] also needs faces wound counter-clockwise
    /// seen from outside that enclose a volume large enough for the pressure: the pressure
    /// force Jolt computes for a vertex at the start geometry may give a vertex of
    /// [`limits::MIN_MASS`] at most [`limits::MAX_ACCELERATION`] (see
    /// [Derived bounds](crate::limits#derived-bounds)). Jolt computes the volume in `f32` from
    /// the vertex positions about the body origin, so the vertices of a pressurised body
    /// belong around that origin: far from it the rounding of that volume can exceed the
    /// volume itself, and the body is refused.
    #[must_use]
    pub fn pressure(mut self, value: f32) -> Self {
        self.pressure = value;
        self
    }

    /// Multiplier for the world's gravity on every vertex, at most
    /// [`limits::MAX_GRAVITY_FACTOR`] in absolute value. Default 1.
    #[must_use]
    pub fn gravity_factor(mut self, value: f32) -> Self {
        self.gravity_factor = value;
        self
    }

    /// Radius of every particle in metres, `0..=`[`limits::MAX_SHAPE_EXTENT`]: vertices keep
    /// this distance from the surfaces they collide with. Default 0.
    #[must_use]
    pub fn vertex_radius(mut self, value: f32) -> Self {
        self.vertex_radius = value;
        self
    }

    /// Whether Jolt moves the body origin to the centre of the vertices' bounds every step.
    /// Default true; false suits a body attached to the static world.
    #[must_use]
    pub fn update_position(mut self, value: bool) -> Self {
        self.update_position = value;
        self
    }

    /// Whether the initial rotation is baked into the vertices, leaving the body rotation at
    /// identity (Jolt simulates slightly more accurately that way). Default true.
    #[must_use]
    pub fn make_rotation_identity(mut self, value: bool) -> Self {
        self.make_rotation_identity = value;
        self
    }

    /// Whether the body may fall asleep when it comes to rest. Default true.
    #[must_use]
    pub fn allow_sleeping(mut self, value: bool) -> Self {
        self.allow_sleeping = value;
        self
    }

    /// Whether queries (ray casts, shape casts and collisions) hit the faces from both sides.
    /// Default false: only from the side their counter-clockwise winding faces.
    #[must_use]
    pub fn faces_double_sided(mut self, value: bool) -> Self {
        self.faces_double_sided = value;
        self
    }

    /// Whether the body starts awake. Default [`Activation::Activate`].
    #[must_use]
    pub fn activation(mut self, value: Activation) -> Self {
        self.activation = value;
        self
    }

    fn validate(&self, object_layer_count: u32) -> Result<(), BodyError> {
        if self.object_layer.get() >= object_layer_count {
            return Err(BodyError::UnknownObjectLayer(self.object_layer));
        }
        let check = |valid: bool, what| {
            if valid {
                Ok(())
            } else {
                Err(BodyError::InvalidValue(what))
            }
        };
        check(
            is_in_frame(self.position),
            "position must be finite and within limits::MAX_POSITION",
        )?;
        check(
            self.rotation.is_valid_rotation(),
            "rotation must be a finite unit quaternion",
        )?;
        check(
            (1..=Self::MAX_ITERATIONS).contains(&self.num_iterations),
            "iterations must be within 1..=SoftBodySettings::MAX_ITERATIONS",
        )?;
        check(
            is_finite_non_negative(self.linear_damping),
            "linear damping must be finite and not negative",
        )?;
        check(
            self.max_linear_velocity > 0.0
                && self.max_linear_velocity <= limits::MAX_LINEAR_VELOCITY,
            "max linear velocity must be above 0 and at most limits::MAX_LINEAR_VELOCITY",
        )?;
        check(
            (0.0..=1.0).contains(&self.restitution),
            "restitution must be between 0 and 1",
        )?;
        check(
            is_friction(self.friction),
            "friction must be finite and between 0 and limits::MAX_FRICTION",
        )?;
        check(
            (0.0..=limits::MAX_SOFT_BODY_PRESSURE).contains(&self.pressure),
            "pressure must be finite and within 0..=limits::MAX_SOFT_BODY_PRESSURE",
        )?;
        check(
            is_gravity_factor(self.gravity_factor),
            "gravity factor must be finite and within limits::MAX_GRAVITY_FACTOR",
        )?;
        check(
            is_local_distance(self.vertex_radius),
            "vertex radius must be finite and within 0..=limits::MAX_SHAPE_EXTENT",
        )
    }
}

/// Soft body creation settings, owned whole by their owner.
impl JoltObject for JPH_SoftBodyCreationSettings {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner owns the settings (trait contract), which joltc deletes; their
        // `RefConst` releases the reference to the shared settings, and bodies created from
        // them keep their own.
        unsafe { JPH_SoftBodyCreationSettings_Destroy(ptr) };
    }
}

/// Jolt's creation settings for a body of `shared` made from `settings`, which the caller
/// validated.
fn creation_settings(
    shared: &SoftBodySharedSettings,
    settings: &SoftBodySettings,
) -> Result<Owned<JPH_SoftBodyCreationSettings>, BodyError> {
    let position = settings.position.to_jph();
    let rotation = settings.rotation.to_jph();
    // SAFETY: `shared` is live for the call and the creation settings take their own reference
    // to it; `position` and `rotation` are live locals. The handle takes over the result.
    let creation = unsafe {
        Owned::from_raw(JPH_SoftBodyCreationSettings_Create2(
            shared.as_ptr(),
            &position,
            &rotation,
            settings.object_layer.get(),
        ))
    }
    .ok_or(BodyError::AllocationFailed)?;
    let ptr = creation.as_ptr();
    // SAFETY: `ptr` is the live settings object owned by `creation`; every input is a value.
    unsafe {
        JPH_SoftBodyCreationSettings_SetNumIterations(ptr, settings.num_iterations);
        JPH_SoftBodyCreationSettings_SetLinearDamping(ptr, settings.linear_damping);
        JPH_SoftBodyCreationSettings_SetMaxLinearVelocity(ptr, settings.max_linear_velocity);
        JPH_SoftBodyCreationSettings_SetRestitution(ptr, settings.restitution);
        JPH_SoftBodyCreationSettings_SetFriction(ptr, settings.friction);
        JPH_SoftBodyCreationSettings_SetPressure(ptr, settings.pressure);
        JPH_SoftBodyCreationSettings_SetGravityFactor(ptr, settings.gravity_factor);
        JPH_SoftBodyCreationSettings_SetVertexRadius(ptr, settings.vertex_radius);
        JPH_SoftBodyCreationSettings_SetUpdatePosition(ptr, settings.update_position);
        JPH_SoftBodyCreationSettings_SetMakeRotationIdentity(ptr, settings.make_rotation_identity);
        JPH_SoftBodyCreationSettings_SetAllowSleeping(ptr, settings.allow_sleeping);
        JPH_SoftBodyCreationSettings_SetFacesDoubleSided(ptr, settings.faces_double_sided);
    }
    Ok(creation)
}

impl PhysicsWorld {
    /// Creates a soft body from `shared` and adds it to the world. The body keeps its own
    /// reference to the shared settings, so they may be dropped afterwards.
    ///
    /// The body is an ordinary body of the world (see [Soft bodies](crate#soft-bodies)): it has
    /// a [`BodyId`], is removed with [`remove_body`](Self::remove_body) and read with
    /// [`body`](Self::body); [`soft_body`](Self::soft_body) and
    /// [`soft_body_mut`](Self::soft_body_mut) give access to its vertices. With
    /// [`SoftBodySettings::make_rotation_identity`] (the default) the vertices start at
    /// `position + rotation · vertex position`.
    ///
    /// Fails with [`BodyError::UnknownObjectLayer`] or [`BodyError::InvalidValue`] when a
    /// setting is out of range or the pressure is too high for the volume the faces of
    /// `shared` enclose (see [`SoftBodySettings::pressure`]), and with
    /// [`BodyError::TooManyBodies`] when the world is full; then nothing changes.
    ///
    /// ```
    /// use joltphysics::*;
    ///
    /// # fn main() -> Result<(), Box<dyn std::error::Error>> {
    /// let mut world = PhysicsWorld::new(WorldSettings::default())?;
    /// // A 3 x 3 cloth hanging from two corners.
    /// let mut vertices = Vec::new();
    /// for z in 0..3 {
    ///     for x in 0..3 {
    ///         let position = Vec3::new(0.5 * x as f32, 0.0, 0.5 * z as f32);
    ///         vertices.push(if z == 0 && x != 1 {
    ///             SoftBodyVertex::kinematic(position)
    ///         } else {
    ///             SoftBodyVertex::new(position)
    ///         });
    ///     }
    /// }
    /// let mut faces = Vec::new();
    /// for z in 0..2 {
    ///     for x in 0..2 {
    ///         let i = z * 3 + x;
    ///         faces.push([i, i + 3, i + 4]);
    ///         faces.push([i, i + 4, i + 1]);
    ///     }
    /// }
    /// let cloth = SoftBodySharedSettings::builder(vertices, faces)
    ///     .create_constraints(SoftBodyBendType::Distance, SoftBodyVertexAttributes::default())
    ///     .build()?;
    /// let id = world.create_soft_body(
    ///     &cloth,
    ///     &SoftBodySettings::default().position(RVec3::new(0.0, 2.0, 0.0)),
    /// )?;
    /// for _ in 0..30 {
    ///     world.step(1.0 / 60.0)?;
    /// }
    /// let vertices = world.soft_body(id)?.vertices();
    /// assert_eq!(vertices[0].inverse_mass, 0.0);
    /// assert!(vertices[8].position.y < 2.0);
    /// # Ok(())
    /// # }
    /// ```
    pub fn create_soft_body(
        &mut self,
        shared: &SoftBodySharedSettings,
        settings: &SoftBodySettings,
    ) -> Result<BodyId, BodyError> {
        settings.validate(self.object_layer_count)?;
        if !is_soft_body_pressure(settings.pressure, &shared.pressure_geometry) {
            return Err(BodyError::InvalidValue(
                "pressure needs faces around the body origin that enclose a volume large \
                 enough for it, wound counter-clockwise seen from outside (see limits)",
            ));
        }
        let creation = creation_settings(shared, settings)?;
        if !self.has_room_for_bodies(1) {
            return Err(BodyError::TooManyBodies);
        }
        self.note_structure_change();
        // SAFETY: the body interface belongs to this live world, borrowed mutably; `creation` is
        // a fully set up settings object whose layer exists in this world.
        let raw = unsafe {
            JPH_BodyInterface_CreateAndAddSoftBody(
                self.body_interface.as_ptr(),
                creation.as_ptr(),
                settings.activation.to_jph(),
            )
        };
        if raw == INVALID_BODY_ID {
            return Err(BodyError::TooManyBodies);
        }
        Ok(BodyId::new(raw, self.tag))
    }

    /// Whether `id` names a soft body; `Ok` if so, [`BodyError::NotSoftBody`] for a rigid one.
    fn check_soft_body(&self, id: BodyId) -> Result<(), BodyError> {
        self.check(id)?;
        let is_soft = with_read_locked_body(self.body_lock_interface, id, |body| {
            // SAFETY: `body` is locked for reading for the duration of the closure; the getter
            // only reads it.
            unsafe { JPH_Body_IsSoftBody(body.as_ptr()) }
        })
        .ok_or(BodyError::NotFound(id))?;
        if is_soft {
            Ok(())
        } else {
            Err(BodyError::NotSoftBody(id))
        }
    }

    /// Read access to the vertices of a soft body; [`BodyError::NotSoftBody`] for a rigid body.
    pub fn soft_body(&self, id: BodyId) -> Result<SoftBodyRef<'_>, BodyError> {
        self.check_soft_body(id)?;
        Ok(SoftBodyRef {
            body_interface: self.body_interface,
            body_lock_interface: self.body_lock_interface,
            id,
            _world: PhantomData,
        })
    }

    /// Read and write access to the vertices of a soft body; [`BodyError::NotSoftBody`] for a
    /// rigid body.
    pub fn soft_body_mut(&mut self, id: BodyId) -> Result<SoftBodyMut<'_>, BodyError> {
        self.check_soft_body(id)?;
        Ok(SoftBodyMut {
            inner: SoftBodyRef {
                body_interface: self.body_interface,
                body_lock_interface: self.body_lock_interface,
                id,
                _world: PhantomData,
            },
            _world: PhantomData,
        })
    }
}

/// One vertex of a soft body in the world, as [`SoftBodyRef::vertices`] reads it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SoftBodyVertexState {
    /// Position in world space, in metres.
    pub position: RVec3,
    /// Velocity in world space, in m/s.
    pub velocity: Vec3,
    /// Inverse mass in 1/kg; 0 for a kinematic vertex.
    pub inverse_mass: f32,
}

/// Read access to the vertices of one soft body, borrowed from its world.
///
/// Each method copies what it reads under Jolt's body read lock, so a readout is consistent.
/// The view borrows the world, so the world cannot step while it exists. Not `Send` or `Sync`;
/// share the world instead.
pub struct SoftBodyRef<'w> {
    body_interface: NonNull<JPH_BodyInterface>,
    body_lock_interface: NonNull<JPH_BodyLockInterface>,
    id: BodyId,
    _world: PhantomData<&'w PhysicsWorld>,
}

/// World positions, world velocities and inverse masses of a soft body's vertices.
type VertexArrays = (Vec<JPH_RVec3>, Vec<JPH_Vec3>, Vec<f32>);

impl SoftBodyRef<'_> {
    /// The body's id.
    pub fn id(&self) -> BodyId {
        self.id
    }

    /// Number of vertices.
    pub fn vertex_count(&self) -> usize {
        with_read_locked_body(self.body_lock_interface, self.id, |body| {
            // SAFETY: `body` is locked for reading for the duration of the closure; the getter
            // only reads it.
            unsafe { JPH_Body_GetSoftBodyVertexCount(body.as_ptr()) as usize }
        })
        .unwrap_or(0)
    }

    /// Every vertex in world space, in vertex order.
    pub fn vertices(&self) -> Vec<SoftBodyVertexState> {
        let mut vertices = Vec::new();
        self.vertices_into(&mut vertices);
        vertices
    }

    /// Like [`vertices`](Self::vertices), into `out`, which is cleared first; reusing it
    /// avoids an allocation per call.
    pub fn vertices_into(&self, out: &mut Vec<SoftBodyVertexState>) {
        out.clear();
        let (positions, velocities, inverse_masses) = self.arrays();
        out.extend(
            positions
                .into_iter()
                .zip(velocities)
                .zip(inverse_masses)
                .map(|((position, velocity), inverse_mass)| SoftBodyVertexState {
                    position: RVec3::from_jph(position),
                    velocity: Vec3::from_jph(velocity),
                    inverse_mass,
                }),
        );
    }

    /// Copies every vertex under one body read lock.
    fn arrays(&self) -> VertexArrays {
        with_read_locked_body(self.body_lock_interface, self.id, |body| {
            // SAFETY: `body` is locked for reading for the duration of the closure. joltc writes
            // at most `count` elements to each output, and each holds exactly `count`.
            unsafe {
                let count = JPH_Body_GetSoftBodyVertexCount(body.as_ptr());
                let mut positions = vec![RVec3::ZERO.to_jph(); count as usize];
                let mut velocities = vec![Vec3::ZERO.to_jph(); count as usize];
                let mut inverse_masses = vec![0.0; count as usize];
                JPH_Body_GetSoftBodyVertices(
                    body.as_ptr(),
                    positions.as_mut_ptr(),
                    velocities.as_mut_ptr(),
                    inverse_masses.as_mut_ptr(),
                    count,
                );
                (positions, velocities, inverse_masses)
            }
        })
        .unwrap_or_default()
    }
}

/// Read and write access to the vertices of one soft body, borrowed mutably from its world.
///
/// Dereferences to [`SoftBodyRef`] for reads. Every setter validates its input before it
/// reaches Jolt, changes nothing when it fails, and wakes the body when it succeeds.
///
/// Vertex inverse masses set here are configuration that a [`WorldState`](crate::WorldState)
/// does not save, and LRA anchors stay those chosen when the shared settings were built.
pub struct SoftBodyMut<'w> {
    inner: SoftBodyRef<'w>,
    _world: PhantomData<&'w mut PhysicsWorld>,
}

impl<'w> Deref for SoftBodyMut<'w> {
    type Target = SoftBodyRef<'w>;

    fn deref(&self) -> &SoftBodyRef<'w> {
        &self.inner
    }
}

impl SoftBodyMut<'_> {
    /// Sets the world-space velocity of vertex `index` in m/s, finite and at most
    /// [`limits::MAX_LINEAR_VELOCITY`] long. A kinematic vertex keeps this velocity until it is
    /// set again.
    pub fn set_vertex_velocity(&mut self, index: u32, velocity: Vec3) -> Result<(), BodyError> {
        self.check_index(index, self.vertex_count())?;
        require_body(is_linear_velocity(velocity), LINEAR_VELOCITY_RULE)?;
        self.write_velocity(index, velocity)
    }

    /// Sets the inverse mass of vertex `index` in 1/kg: 0 pins the vertex (it becomes
    /// kinematic), otherwise the inverse of a mass within
    /// [`limits::MIN_MASS`]`..=`[`limits::MAX_MASS`]. The masses of the movable vertices must
    /// still add up to at most [`limits::MAX_MASS`], and the force added to the body this step
    /// ([`BodyMut::add_force`](crate::BodyMut::add_force)) must stay within its bound for the
    /// new inverse masses, so unpinning a vertex cannot release a force that was accepted
    /// while every vertex was pinned. Jolt recomputes the body's mass from the
    /// vertices; while any vertex is kinematic the body's mass is infinite
    /// ([`BodyRef::mass`](crate::BodyRef::mass) is `None`).
    pub fn set_vertex_inverse_mass(
        &mut self,
        index: u32,
        inverse_mass: f32,
    ) -> Result<(), BodyError> {
        require_body(
            inverse_mass == 0.0 || is_vertex_inverse_mass(inverse_mass),
            VERTEX_INVERSE_MASS_RULE,
        )?;
        let (_, _, mut inverse_masses) = self.arrays();
        self.check_index(index, inverse_masses.len())?;
        inverse_masses[index as usize] = inverse_mass;
        let total_mass: f64 = inverse_masses
            .iter()
            .filter(|&&w| w > 0.0)
            .map(|&w| vertex_mass(w))
            .sum();
        require_body(total_mass <= f64::from(limits::MAX_MASS), TOTAL_MASS_RULE)?;
        let largest_inverse_mass = inverse_masses.iter().copied().fold(0.0, f32::max);
        let vertex_count = inverse_masses.len() as u32;
        let written = with_locked_body(self.body_lock_interface, self.id, |body| {
            let mut force = Vec3::ZERO.to_jph();
            // SAFETY: `body` is locked for writing for the duration of the closure. A soft body
            // is always dynamic, so it has the force accumulator the getter reads; `force` is a
            // live local.
            unsafe { JPH_Body_GetAccumulatedForce(body.as_ptr(), &mut force) };
            let force = Vec3::from_jph(force);
            let force = [force.x, force.y, force.z].map(f64::from);
            if !is_soft_body_force(force, largest_inverse_mass, vertex_count) {
                return false;
            }
            // SAFETY: as above; `index` names one of the body's vertices.
            unsafe { JPH_Body_SetSoftBodyVertexInvMass(body.as_ptr(), index, inverse_mass) };
            true
        })
        .ok_or(BodyError::NotFound(self.id))?;
        require_body(
            written,
            "the force added this step would exceed the soft body force bound of limits with \
             this inverse mass; step or reset forces first",
        )?;
        self.activate();
        Ok(())
    }

    /// Moves the kinematic vertex `index` to the world position `target` over the next step of
    /// `delta_time` seconds, by setting its velocity to `(target - position) / delta_time`.
    ///
    /// The vertex must be kinematic (inverse mass 0), `target` within
    /// [`limits::MAX_POSITION`], `delta_time` one that [`PhysicsWorld::step`] accepts, and the
    /// velocity at most [`limits::MAX_LINEAR_VELOCITY`] long. The velocity stays after the
    /// step, as for a kinematic body Jolt moves with `MoveKinematic`: the vertex keeps moving
    /// until it is moved again or stopped with
    /// [`set_vertex_velocity`](Self::set_vertex_velocity)`(index, Vec3::ZERO)`.
    pub fn move_kinematic_vertex(
        &mut self,
        index: u32,
        target: RVec3,
        delta_time: f32,
    ) -> Result<(), BodyError> {
        require_body(
            is_in_frame(target),
            "target must be finite and within limits::MAX_POSITION",
        )?;
        require_body(
            PhysicsWorld::is_valid_delta_time(delta_time),
            "delta time must be one that PhysicsWorld::step accepts",
        )?;
        let (positions, _, inverse_masses) = self.arrays();
        self.check_index(index, positions.len())?;
        require_body(
            inverse_masses[index as usize] == 0.0,
            "only a kinematic vertex (inverse mass 0) can be moved",
        )?;
        let position = RVec3::from_jph(positions[index as usize]);
        let delta_time = Real::from(delta_time);
        // `Real` is `f32` without the `double-precision` feature, so the casts are no-ops there.
        #[allow(clippy::unnecessary_cast)]
        let velocity = Vec3::new(
            ((target.x - position.x) / delta_time) as f32,
            ((target.y - position.y) / delta_time) as f32,
            ((target.z - position.z) / delta_time) as f32,
        );
        require_body(
            is_linear_velocity(velocity),
            "the move needs a velocity above limits::MAX_LINEAR_VELOCITY",
        )?;
        self.write_velocity(index, velocity)
    }

    fn check_index(&self, index: u32, count: usize) -> Result<(), BodyError> {
        require_body(
            (index as usize) < count,
            "vertex index must name a vertex of the soft body",
        )
    }

    fn write_velocity(&mut self, index: u32, velocity: Vec3) -> Result<(), BodyError> {
        let velocity = velocity.to_jph();
        with_locked_body(self.body_lock_interface, self.id, |body| {
            // SAFETY: `body` is locked for writing for the duration of the closure, `index` names
            // one of its vertices and `velocity` is a live local.
            unsafe { JPH_Body_SetSoftBodyVertexVelocity(body.as_ptr(), index, &velocity) }
        })
        .ok_or(BodyError::NotFound(self.id))?;
        self.activate();
        Ok(())
    }

    /// Wakes the body. Called after the write lock is released: `ActivateBody` locks the body
    /// again, and Jolt's body mutexes are not recursive.
    fn activate(&mut self) {
        // SAFETY: the world is borrowed mutably through this view and holds the body; this
        // thread holds no body lock.
        unsafe { JPH_BodyInterface_ActivateBody(self.body_interface.as_ptr(), self.id.to_raw()) };
    }
}

fn require_body(valid: bool, what: &'static str) -> Result<(), BodyError> {
    if valid {
        Ok(())
    } else {
        Err(BodyError::InvalidValue(what))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn triangle() -> Vec<SoftBodyVertex> {
        vec![
            SoftBodyVertex::new(Vec3::new(0.0, 0.0, 0.0)),
            SoftBodyVertex::new(Vec3::new(1.0, 0.0, 0.0)),
            SoftBodyVertex::new(Vec3::new(0.0, 0.0, 1.0)),
        ]
    }

    fn rejected(builder: SoftBodySharedSettingsBuilder) -> &'static str {
        match builder.build() {
            Err(SoftBodyError::InvalidValue(what)) => what,
            Err(other) => panic!("unexpected error {other:?}"),
            Ok(_) => panic!("accepted"),
        }
    }

    #[test]
    fn default_settings_match_jolt() {
        assert!(ensure_initialized());
        // SAFETY: Jolt is initialised, and the handle takes over the new settings.
        let jolt = unsafe { Owned::from_raw(JPH_SoftBodyCreationSettings_Create()) }.unwrap();
        let ours = SoftBodySettings::default();
        let ptr = jolt.as_ptr();
        let mut position = RVec3::new(1.0, 1.0, 1.0).to_jph();
        let mut rotation = Quat::from_xyzw(1.0, 0.0, 0.0, 0.0).to_jph();
        // SAFETY: `ptr` is the live settings object owned by `jolt`; getters only read it and
        // write live locals.
        unsafe {
            JPH_SoftBodyCreationSettings_GetPosition(ptr, &mut position);
            JPH_SoftBodyCreationSettings_GetRotation(ptr, &mut rotation);
            assert_eq!(RVec3::from_jph(position), ours.position);
            assert_eq!(Quat::from_jph(rotation), ours.rotation);
            assert_eq!(
                JPH_SoftBodyCreationSettings_GetNumIterations(ptr),
                ours.num_iterations
            );
            assert_eq!(
                JPH_SoftBodyCreationSettings_GetLinearDamping(ptr),
                ours.linear_damping
            );
            assert_eq!(
                JPH_SoftBodyCreationSettings_GetMaxLinearVelocity(ptr),
                ours.max_linear_velocity
            );
            assert_eq!(
                JPH_SoftBodyCreationSettings_GetRestitution(ptr),
                ours.restitution
            );
            assert_eq!(JPH_SoftBodyCreationSettings_GetFriction(ptr), ours.friction);
            assert_eq!(JPH_SoftBodyCreationSettings_GetPressure(ptr), ours.pressure);
            assert_eq!(
                JPH_SoftBodyCreationSettings_GetGravityFactor(ptr),
                ours.gravity_factor
            );
            assert_eq!(
                JPH_SoftBodyCreationSettings_GetVertexRadius(ptr),
                ours.vertex_radius
            );
            assert_eq!(
                JPH_SoftBodyCreationSettings_GetUpdatePosition(ptr),
                ours.update_position
            );
            assert_eq!(
                JPH_SoftBodyCreationSettings_GetMakeRotationIdentity(ptr),
                ours.make_rotation_identity
            );
            assert_eq!(
                JPH_SoftBodyCreationSettings_GetAllowSleeping(ptr),
                ours.allow_sleeping
            );
            assert_eq!(
                JPH_SoftBodyCreationSettings_GetFacesDoubleSided(ptr),
                ours.faces_double_sided
            );
            // The one documented difference: Jolt's default layer is 0.
            assert_eq!(JPH_SoftBodyCreationSettings_GetObjectLayer(ptr), 0);
        }
        assert_eq!(ours.object_layer, ObjectLayer::MOVING);
        assert_eq!(ours.max_linear_velocity, limits::MAX_LINEAR_VELOCITY);
    }

    #[test]
    fn default_vertex_attributes_match_jolt() {
        let mut jolt = SoftBodyVertexAttributes::default()
            .compliance(5.0)
            .long_range_attachment(LongRangeAttachment::GeodesicDistance, 3.0)
            .to_jph();
        // SAFETY: `jolt` is a live local that the call overwrites.
        unsafe { JPH_SoftBodyVertexAttributes_Init(&mut jolt) };
        let ours = SoftBodyVertexAttributes::default().to_jph();
        assert_eq!(jolt.compliance, ours.compliance);
        assert_eq!(jolt.shearCompliance, ours.shearCompliance);
        assert_eq!(jolt.bendCompliance, ours.bendCompliance);
        assert_eq!(jolt.lraType, ours.lraType);
        assert_eq!(jolt.lraMaxDistanceMultiplier, ours.lraMaxDistanceMultiplier);
        assert_eq!(SoftBodyVertex::new(Vec3::ZERO).inverse_mass, 1.0);
    }

    #[test]
    fn a_triangle_without_constraints_builds() {
        let settings = SoftBodySharedSettings::builder(triangle(), vec![[0, 2, 1]])
            .build()
            .unwrap();
        assert_eq!(settings.vertex_count(), 3);
        assert_eq!(settings.face_count(), 1);
        assert_eq!(settings.edge_constraint_count(), 0);
    }

    #[test]
    fn invalid_vertices_are_rejected() {
        let build = |vertices| rejected(SoftBodySharedSettings::builder(vertices, Vec::new()));
        assert_eq!(build(Vec::new()), "a soft body needs at least one vertex");
        let mut far = triangle();
        far[1].position.x = limits::MAX_SHAPE_EXTENT.next_up();
        assert!(build(far).contains("vertex position"));
        let mut fast = triangle();
        fast[1].velocity.y = limits::MAX_LINEAR_VELOCITY.next_up();
        assert_eq!(build(fast), LINEAR_VELOCITY_RULE);
        for inverse_mass in [-1.0, f32::NAN, (1.0 / limits::MIN_MASS).next_up()] {
            let mut heavy = triangle();
            heavy[2].inverse_mass = inverse_mass;
            assert_eq!(build(heavy), VERTEX_INVERSE_MASS_RULE);
        }
    }

    #[test]
    fn total_movable_mass_is_bounded() {
        let at_bound = |w| {
            vec![
                SoftBodyVertex {
                    inverse_mass: w,
                    ..SoftBodyVertex::new(Vec3::ZERO)
                },
                SoftBodyVertex {
                    inverse_mass: w,
                    ..SoftBodyVertex::new(Vec3::new(1.0, 0.0, 0.0))
                },
            ]
        };
        let heavy = 1.0 / limits::MAX_MASS;
        assert_eq!(
            rejected(SoftBodySharedSettings::builder(at_bound(heavy), Vec::new())),
            TOTAL_MASS_RULE
        );
        // Two vertices of 400 t each, below the bound together.
        let lighter = 2.5 / limits::MAX_MASS;
        assert!(
            SoftBodySharedSettings::builder(at_bound(lighter), Vec::new())
                .build()
                .is_ok()
        );
        let pinned = vec![
            SoftBodyVertex::kinematic(Vec3::ZERO),
            SoftBodyVertex::kinematic(Vec3::new(1.0, 0.0, 0.0)),
        ];
        assert!(SoftBodySharedSettings::builder(pinned, Vec::new())
            .build()
            .is_ok());
    }

    #[test]
    fn invalid_faces_are_rejected() {
        let build =
            |vertices, face| rejected(SoftBodySharedSettings::builder(vertices, vec![face]));
        assert_eq!(
            build(triangle(), [0, 1, 3]),
            "a face index must name a vertex"
        );
        assert_eq!(
            build(triangle(), [0, 1, 1]),
            "a face must name three different vertices"
        );
        let mut short = triangle();
        short[1].position.x = limits::MIN_SOFT_BODY_EDGE_LENGTH.next_down();
        assert_eq!(build(short, [0, 2, 1]), EDGE_LENGTH_RULE);
        // Three vertices on a line: every edge is long enough, the face has no area.
        let mut collinear = triangle();
        collinear[2].position = Vec3::new(2.0, 0.0, 0.0);
        assert_eq!(build(collinear, [0, 2, 1]), "a face must have an area");
    }

    #[test]
    fn edge_lengths_are_measured_in_f32_like_jolt() {
        // The positions differ, but by less than f32 resolves at 1000 m: the difference is 0.
        let a = Vec3::new(1000.0, 0.0, 0.0);
        let b = Vec3::new(1000.0 + 1.0e-5, 0.0, 0.0);
        assert!(!is_edge_length(a, b));
        // A subnormal difference squares to 0 in f32.
        let tiny = Vec3::new(f32::MIN_POSITIVE / 4.0, 0.0, 0.0);
        assert!(!is_edge_length(Vec3::ZERO, tiny));
        let bound = Vec3::new(limits::MIN_SOFT_BODY_EDGE_LENGTH, 0.0, 0.0);
        assert!(is_edge_length(Vec3::ZERO, bound));
    }

    #[test]
    fn distance_bends_need_separate_opposite_vertices() {
        // Two triangles folded onto each other: the vertices opposite the shared edge 0-1
        // coincide.
        let mut vertices = triangle();
        vertices.push(SoftBodyVertex::new(Vec3::new(0.0, 0.0, 1.0)));
        let faces = vec![[0, 2, 1], [0, 1, 3]];
        let folded = SoftBodySharedSettings::builder(vertices.clone(), faces.clone())
            .create_constraints(SoftBodyBendType::Distance, Default::default());
        assert!(rejected(folded).contains("opposite a shared edge"));
        let dihedral = SoftBodySharedSettings::builder(vertices, faces)
            .create_constraints(SoftBodyBendType::Dihedral, Default::default());
        assert!(dihedral.build().is_ok());
    }

    #[test]
    fn attributes_are_validated() {
        let build = |attributes: SoftBodyVertexAttributes| {
            SoftBodySharedSettings::builder(triangle(), vec![[0, 2, 1]])
                .create_constraints(SoftBodyBendType::Dihedral, attributes)
                .build()
        };
        let default = SoftBodyVertexAttributes::default();
        for compliance in [-1.0, f32::NAN, limits::MAX_COMPLIANCE.next_up()] {
            for attributes in [
                default.compliance(compliance),
                default.shear_compliance(compliance),
                default.bend_compliance(Some(compliance)),
            ] {
                assert_eq!(
                    build(attributes).err(),
                    Some(SoftBodyError::InvalidValue(COMPLIANCE_RULE))
                );
            }
        }
        let lra = LongRangeAttachment::EuclideanDistance;
        for multiplier in [1.0_f32.next_down(), limits::MAX_RATIO.next_up(), f32::NAN] {
            assert!(build(default.long_range_attachment(lra, multiplier)).is_err());
        }
        assert!(build(default.long_range_attachment(lra, limits::MAX_RATIO)).is_ok());
        assert!(build(default.bend_compliance(Some(limits::MAX_COMPLIANCE))).is_ok());
        let per_vertex = SoftBodySharedSettings::builder(triangle(), vec![[0, 2, 1]])
            .create_constraints_per_vertex(SoftBodyBendType::None, vec![default; 2]);
        assert_eq!(
            rejected(per_vertex),
            "per-vertex attributes need one set per vertex"
        );
    }
}
