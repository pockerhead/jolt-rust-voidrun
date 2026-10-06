//! Triangle meshes of the shape descriptions, built on the CPU once per description, and the
//! frame batch that poses, shades and splits them into draw calls a renderer can take.

use glam::{Quat, Vec3};

use crate::draw::Colour;
use crate::visual::{Visual, VisualKey, Visuals};

/// Segments around the axis of round shapes.
const SEGMENTS: u32 = 16;
/// Rings from the pole to the equator of a sphere or a capsule's end.
const HALF_RINGS: u32 = 6;

/// A flat-shaded triangle mesh: three positions and one normal per triangle.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct FlatMesh {
    /// Corner positions, three per triangle.
    pub positions: Vec<Vec3>,
    /// Unit normals, one per triangle.
    pub normals: Vec<Vec3>,
}

impl FlatMesh {
    /// The mesh of `visual` in the shape's frame.
    pub fn of(visual: &Visual) -> Self {
        let mut mesh = Self::default();
        mesh.append(visual, Vec3::ZERO, Quat::IDENTITY);
        mesh
    }

    /// Number of triangles.
    pub fn triangle_count(&self) -> usize {
        self.normals.len()
    }

    /// Appends `visual` placed at `position` and `rotation`.
    fn append(&mut self, visual: &Visual, position: Vec3, rotation: Quat) {
        let place = |p: Vec3| position + rotation * p;
        match visual {
            Visual::Box { half_extent } => {
                let (vertices, triangles) = box_mesh(Vec3::from(*half_extent));
                self.append_indexed(&vertices, &triangles, place);
            }
            Visual::Sphere { radius } => {
                let profile = arc(*radius, 0.0, 1.0, -1.0);
                self.append_lathe(&profile, place);
            }
            Visual::Capsule {
                half_height,
                radius,
            } => {
                let mut profile = arc(*radius, *half_height, 1.0, 0.0);
                profile.extend(arc(*radius, -half_height, 0.0, -1.0));
                self.append_lathe(&profile, place);
            }
            Visual::TaperedCapsule {
                half_height,
                top_radius,
                bottom_radius,
            } => {
                let mut profile = arc(*top_radius, *half_height, 1.0, 0.0);
                profile.extend(arc(*bottom_radius, -half_height, 0.0, -1.0));
                self.append_lathe(&profile, place);
            }
            Visual::Cylinder {
                half_height,
                radius,
            } => {
                let h = *half_height;
                self.append_lathe(&[(0.0, h), (*radius, h), (*radius, -h), (0.0, -h)], place);
            }
            Visual::TaperedCylinder {
                half_height,
                top_radius,
                bottom_radius,
            } => {
                let h = *half_height;
                let profile = [(0.0, h), (*top_radius, h), (*bottom_radius, -h), (0.0, -h)];
                self.append_lathe(&profile, place);
            }
            Visual::Scaled(inner, scale) => {
                let inner = Self::of(inner);
                let scale = Vec3::from(*scale);
                let corners: Vec<Vec3> = inner.positions.iter().map(|&p| p * scale).collect();
                for triangle in corners.chunks_exact(3) {
                    self.push_triangle([triangle[0], triangle[1], triangle[2]].map(place));
                }
            }
            Visual::Triangles {
                vertices,
                triangles,
            } => {
                let vertices: Vec<Vec3> = vertices.iter().map(|&v| Vec3::from(v)).collect();
                self.append_indexed(&vertices, triangles, place);
            }
            Visual::Compound(children) => {
                for (child, child_position, child_rotation) in children {
                    let child_rotation = Quat::from_array(*child_rotation);
                    self.append(
                        child,
                        place(Vec3::from(*child_position)),
                        rotation * child_rotation,
                    );
                }
            }
        }
    }

    fn append_indexed(
        &mut self,
        vertices: &[Vec3],
        triangles: &[[u32; 3]],
        place: impl Fn(Vec3) -> Vec3,
    ) {
        for triangle in triangles {
            self.push_triangle(triangle.map(|index| place(vertices[index as usize])));
        }
    }

    /// The surface of revolution about Y of `profile`, `(radius, y)` points from top to bottom.
    fn append_lathe(&mut self, profile: &[(f32, f32)], place: impl Fn(Vec3) -> Vec3) {
        let ring = |(radius, y): (f32, f32), segment: u32| {
            let angle = segment as f32 / SEGMENTS as f32 * std::f32::consts::TAU;
            Vec3::new(radius * angle.sin(), y, radius * angle.cos())
        };
        for pair in profile.windows(2) {
            for segment in 0..SEGMENTS {
                let upper = [ring(pair[0], segment), ring(pair[0], segment + 1)];
                let lower = [ring(pair[1], segment), ring(pair[1], segment + 1)];
                self.push_triangle([upper[0], lower[0], lower[1]].map(&place));
                self.push_triangle([upper[0], lower[1], upper[1]].map(&place));
            }
        }
    }

    /// Adds one triangle; a degenerate one (no normal) is left out.
    fn push_triangle(&mut self, corners: [Vec3; 3]) {
        let normal = (corners[1] - corners[0]).cross(corners[2] - corners[0]);
        if normal.length_squared() <= 1.0e-14 {
            return;
        }
        self.positions.extend(corners);
        self.normals.push(normal.normalize());
    }
}

/// The 12 triangles of a box.
fn box_mesh(half: Vec3) -> (Vec<Vec3>, Vec<[u32; 3]>) {
    let vertices = (0..8)
        .map(|i| {
            Vec3::new(
                if i & 1 == 0 { -half.x } else { half.x },
                if i & 2 == 0 { -half.y } else { half.y },
                if i & 4 == 0 { -half.z } else { half.z },
            )
        })
        .collect();
    let faces = [
        [0, 4, 6, 2], // -X
        [1, 3, 7, 5], // +X
        [0, 1, 5, 4], // -Y
        [2, 6, 7, 3], // +Y
        [0, 2, 3, 1], // -Z
        [4, 5, 7, 6], // +Z
    ];
    let triangles = faces
        .iter()
        .flat_map(|&[a, b, c, d]| [[a, b, c], [a, c, d]])
        .collect();
    (vertices, triangles)
}

/// Points `(radius, y)` of a circle of `radius` around `(0, centre)` from `sin` angle `from` to
/// `to` (1 at the top pole, 0 at the equator, -1 at the bottom pole), top to bottom.
fn arc(radius: f32, centre: f32, from: f32, to: f32) -> Vec<(f32, f32)> {
    let rings = ((from - to) * HALF_RINGS as f32).round() as u32;
    (0..=rings)
        .map(|ring| {
            let s = from + (to - from) * ring as f32 / rings as f32;
            let angle = s * std::f32::consts::FRAC_PI_2;
            (radius * angle.cos(), centre + radius * angle.sin())
        })
        .collect()
}

/// The meshes of one scene build's descriptions, each built once per version.
#[derive(Debug, Default)]
pub struct MeshStore {
    generation: Option<u64>,
    meshes: Vec<Option<(u32, FlatMesh)>>,
}

impl MeshStore {
    /// The mesh of `key`, built from `visuals` when the store has no mesh of that version. A key
    /// of another scene build empties the store first.
    pub fn get(&mut self, key: VisualKey, visuals: &Visuals) -> &FlatMesh {
        if self.generation != Some(key.generation) {
            self.generation = Some(key.generation);
            self.meshes.clear();
        }
        let index = key.id as usize;
        if self.meshes.len() <= index {
            self.meshes.resize_with(index + 1, || None);
        }
        let slot = &mut self.meshes[index];
        if slot
            .as_ref()
            .is_none_or(|(version, _)| *version != key.version)
        {
            *slot = Some((key.version, FlatMesh::of(visuals.get(key))));
        }
        &slot.as_ref().expect("filled above").1
    }

    /// How many meshes the store holds.
    pub fn len(&self) -> usize {
        self.meshes.iter().flatten().count()
    }

    /// Whether the store holds no mesh.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

/// The most triangles one draw call holds: macroquad truncates a draw call of `capacity` or
/// more vertices or indices, and the window sets both capacities to 60 000.
pub const TRIANGLES_PER_DRAW_CALL: usize = 19_999;

/// One corner of a shaded triangle.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ShadedVertex {
    /// World position.
    pub position: [f32; 3],
    /// Red, green, blue, alpha.
    pub colour: [u8; 4],
}

/// The direction toward the light, normalized.
fn light() -> Vec3 {
    Vec3::new(0.35, 1.0, 0.55).normalize()
}

/// `base` lit from [`light`] on a surface with `normal`, in four flat steps of brightness.
pub fn shade(base: Colour, normal: Vec3, alpha: u8) -> [u8; 4] {
    let lambert = normal.dot(light()).max(0.0);
    let step = (lambert * 4.0).floor().min(3.0) / 3.0;
    let brightness = 0.4 + 0.6 * step;
    let channel = |c: f32| (c * brightness * 255.0).round().clamp(0.0, 255.0) as u8;
    [channel(base[0]), channel(base[1]), channel(base[2]), alpha]
}

/// Shaded triangles of one frame, split into draw calls of at most
/// [`TRIANGLES_PER_DRAW_CALL`] triangles; corners `3 i..3 i + 3` of a call form triangle `i`.
#[derive(Debug, Default)]
pub struct TriangleBatch {
    calls: Vec<Vec<ShadedVertex>>,
    used: usize,
}

impl TriangleBatch {
    /// Empties the batch, keeping its allocations.
    pub fn clear(&mut self) {
        for call in &mut self.calls {
            call.clear();
        }
        self.used = 0;
    }

    /// The filled draw calls.
    pub fn calls(&self) -> impl Iterator<Item = &[ShadedVertex]> {
        self.calls[..self.used.min(self.calls.len())]
            .iter()
            .map(Vec::as_slice)
            .filter(|call| !call.is_empty())
    }

    /// Adds one triangle of one colour.
    pub fn push(&mut self, corners: [Vec3; 3], colour: [u8; 4]) {
        if self.used == 0 || self.calls[self.used - 1].len() >= 3 * TRIANGLES_PER_DRAW_CALL {
            if self.calls.len() == self.used {
                self.calls
                    .push(Vec::with_capacity(3 * TRIANGLES_PER_DRAW_CALL));
            }
            self.used += 1;
        }
        let call = &mut self.calls[self.used - 1];
        call.extend(corners.map(|p| ShadedVertex {
            position: p.to_array(),
            colour,
        }));
    }

    /// Adds `mesh` at `position` and `rotation`, shaded from `base`.
    pub fn push_mesh(
        &mut self,
        mesh: &FlatMesh,
        position: Vec3,
        rotation: Quat,
        base: Colour,
        alpha: u8,
    ) {
        for (corners, normal) in mesh.positions.chunks_exact(3).zip(&mesh.normals) {
            let colour = shade(base, rotation * *normal, alpha);
            self.push(
                [corners[0], corners[1], corners[2]].map(|p| position + rotation * p),
                colour,
            );
        }
    }

    /// Adds triangles over `vertices`, each shaded by its own normal from `base`.
    pub fn push_surface(
        &mut self,
        vertices: &[[f32; 3]],
        triangles: &[[u32; 3]],
        base: Colour,
        alpha: u8,
    ) {
        for triangle in triangles {
            let corners = triangle.map(|index| Vec3::from(vertices[index as usize]));
            let normal = (corners[1] - corners[0]).cross(corners[2] - corners[0]);
            if normal.length_squared() <= 1.0e-14 {
                continue;
            }
            self.push(corners, shade(base, normal.normalize(), alpha));
        }
    }

    /// Total number of triangles.
    pub fn triangle_count(&self) -> usize {
        self.calls().map(|call| call.len() / 3).sum()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::visual::Shaped;
    use oxijolt::HeightFieldSettings;

    #[test]
    fn closed_meshes_face_outward() {
        let visuals = [
            Visual::Box {
                half_extent: [0.5, 1.0, 1.5],
            },
            Visual::Sphere { radius: 0.7 },
            Visual::Capsule {
                half_height: 0.5,
                radius: 0.3,
            },
            Visual::Cylinder {
                half_height: 0.4,
                radius: 0.6,
            },
            Visual::TaperedCylinder {
                half_height: 0.4,
                top_radius: 0.2,
                bottom_radius: 0.6,
            },
        ];
        for visual in &visuals {
            let mesh = FlatMesh::of(visual);
            assert!(mesh.triangle_count() >= 12, "{visual:?}");
            for (corners, normal) in mesh.positions.chunks_exact(3).zip(&mesh.normals) {
                let centre = (corners[0] + corners[1] + corners[2]) / 3.0;
                assert!(centre.dot(*normal) > 0.0, "{visual:?}: {corners:?}");
            }
        }
    }

    #[test]
    fn visual_keys_change_with_generation_and_version() {
        let mut store = MeshStore::default();
        let mut first = Visuals::new(1);
        let small = first.add(Visual::Sphere { radius: 0.5 });
        let mut second = Visuals::new(2);
        let same_id = second.add(Visual::Box {
            half_extent: [1.0; 3],
        });
        assert_eq!(small.id, same_id.id);
        let sphere = store.get(small, &first).clone();
        let cube = store.get(same_id, &second).clone();
        assert_ne!(sphere, cube, "another build's key must not reuse the mesh");
        assert_eq!(cube.triangle_count(), 12);
        assert_eq!(store.len(), 1);

        let replaced = second.replace(
            same_id,
            Visual::Box {
                half_extent: [2.0; 3],
            },
        );
        let bigger = store.get(replaced, &second).clone();
        assert_ne!(bigger, cube, "a new version rebuilds the mesh");
        assert_eq!(store.len(), 1);
    }

    #[test]
    fn chunks_respect_the_draw_call_capacity() {
        let n = 65;
        let samples: Vec<f32> = (0..n * n).map(|i| ((i * 7) % 11) as f32 * 0.05).collect();
        let terrain = Shaped::height_field(n, &samples, &HeightFieldSettings::default()).unwrap();
        let child = Visual::Box {
            half_extent: [0.1; 3],
        };
        let wall = Visual::Compound(
            (0..2000)
                .map(|i| {
                    (
                        child.clone(),
                        [i as f32 * 0.3, 0.0, 0.0],
                        [0.0, 0.0, 0.0, 1.0],
                    )
                })
                .collect(),
        );
        let mut batch = TriangleBatch::default();
        let mut expected = 0;
        for visual in [&terrain.visual, &wall] {
            let mesh = FlatMesh::of(visual);
            expected += mesh.triangle_count();
            batch.push_mesh(&mesh, Vec3::ZERO, Quat::IDENTITY, [1.0; 3], 255);
        }
        assert_eq!(expected, 2 * 64 * 64 + 2000 * 12);
        assert_eq!(batch.triangle_count(), expected);
        assert!(batch.calls().count() >= 2);
        for call in batch.calls() {
            // Vertices and indices both stay below macroquad's capacity of 60 000.
            assert!(call.len() < 60_000 && call.len() % 3 == 0);
        }

        // A cleared batch reuses its calls.
        batch.clear();
        assert_eq!(batch.calls().count(), 0);
        let mesh = FlatMesh::of(&child);
        batch.push_mesh(&mesh, Vec3::ZERO, Quat::IDENTITY, [1.0; 3], 255);
        assert_eq!(batch.triangle_count(), 12);
    }

    #[test]
    fn shading_has_four_levels() {
        let mut levels: Vec<[u8; 4]> = (0..=100)
            .map(|i| {
                let angle = i as f32 / 100.0 * std::f32::consts::PI;
                shade([1.0; 3], Vec3::new(angle.cos(), angle.sin(), 0.0), 255)
            })
            .collect();
        levels.sort();
        levels.dedup();
        assert_eq!(levels.len(), 4);
    }
}
