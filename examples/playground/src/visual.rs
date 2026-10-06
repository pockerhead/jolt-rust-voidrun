//! The playground's own descriptions of the shapes it creates, so it can draw them: the binding
//! has no shape introspection, so each scene describes what it built next to the `Shape`.

use oxijolt::{CompoundChild, HeightFieldSettings, Quat, Shape, Vec3};

use crate::scene::Result;

/// A shape description in the shape's own frame, with Jolt's conventions: cylinders and
/// capsules along local Y, tapered shapes with their top at +Y.
#[derive(Clone, Debug, PartialEq)]
pub enum Visual {
    /// A box of half extents.
    Box { half_extent: [f32; 3] },
    /// A sphere.
    Sphere { radius: f32 },
    /// A capsule: a cylinder of half height with hemispheres of radius on both ends.
    Capsule { half_height: f32, radius: f32 },
    /// A capsule whose ends have different radii.
    TaperedCapsule {
        half_height: f32,
        top_radius: f32,
        bottom_radius: f32,
    },
    /// A cylinder.
    Cylinder { half_height: f32, radius: f32 },
    /// A cylinder whose ends have different radii.
    TaperedCylinder {
        half_height: f32,
        top_radius: f32,
        bottom_radius: f32,
    },
    /// Another description scaled per axis.
    Scaled(Box<Visual>, [f32; 3]),
    /// Triangles, counter-clockwise seen from outside.
    Triangles {
        vertices: Vec<[f32; 3]>,
        triangles: Vec<[u32; 3]>,
    },
    /// Children at their positions and rotations (`[x, y, z, w]`).
    Compound(Vec<(Visual, [f32; 3], [f32; 4])>),
}

/// Names one description of one scene build: the build's generation, the description's id in
/// that build, and its version, which grows each time the description is replaced.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct VisualKey {
    /// The scene build the description belongs to.
    pub generation: u64,
    /// The description's index in the build.
    pub id: u32,
    /// How often the description was replaced.
    pub version: u32,
}

/// The descriptions of one scene build.
#[derive(Clone, Debug)]
pub struct Visuals {
    generation: u64,
    entries: Vec<(Visual, u32)>,
}

impl Visuals {
    /// An empty set for the scene build `generation`.
    pub fn new(generation: u64) -> Self {
        Self {
            generation,
            entries: Vec::new(),
        }
    }

    /// Adds a description.
    pub fn add(&mut self, visual: Visual) -> VisualKey {
        self.entries.push((visual, 0));
        self.key(self.entries.len() as u32 - 1)
    }

    /// Replaces the description of `key`, returning the key of the new version.
    pub fn replace(&mut self, key: VisualKey, visual: Visual) -> VisualKey {
        let entry = &mut self.entries[key.id as usize];
        *entry = (visual, entry.1 + 1);
        self.key(key.id)
    }

    /// The current description of `key`'s id.
    pub fn get(&self, key: VisualKey) -> &Visual {
        &self.entries[key.id as usize].0
    }

    fn key(&self, id: u32) -> VisualKey {
        VisualKey {
            generation: self.generation,
            id,
            version: self.entries[id as usize].1,
        }
    }
}

/// A Jolt shape together with the description it was built from.
pub struct Shaped {
    /// The shape, for bodies.
    pub shape: Shape,
    /// The description, for drawing.
    pub visual: Visual,
}

impl Shaped {
    /// A box with Jolt's default convex radius.
    pub fn cuboid(half_extent: [f32; 3]) -> Result<Self> {
        Ok(Self {
            shape: Shape::new_box(half_extent.into())?,
            visual: Visual::Box { half_extent },
        })
    }

    /// A sphere.
    pub fn sphere(radius: f32) -> Result<Self> {
        Ok(Self {
            shape: Shape::new_sphere(radius)?,
            visual: Visual::Sphere { radius },
        })
    }

    /// A capsule along Y.
    pub fn capsule(half_height: f32, radius: f32) -> Result<Self> {
        Ok(Self {
            shape: Shape::new_capsule(half_height, radius)?,
            visual: Visual::Capsule {
                half_height,
                radius,
            },
        })
    }

    /// A cylinder along Y.
    pub fn cylinder(half_height: f32, radius: f32) -> Result<Self> {
        Ok(Self {
            shape: Shape::new_cylinder(half_height, radius)?,
            visual: Visual::Cylinder {
                half_height,
                radius,
            },
        })
    }

    /// A tapered capsule along Y.
    pub fn tapered_capsule(half_height: f32, top_radius: f32, bottom_radius: f32) -> Result<Self> {
        Ok(Self {
            shape: Shape::new_tapered_capsule(half_height, top_radius, bottom_radius)?,
            visual: Visual::TaperedCapsule {
                half_height,
                top_radius,
                bottom_radius,
            },
        })
    }

    /// A tapered cylinder along Y with Jolt's default convex radius.
    pub fn tapered_cylinder(half_height: f32, top_radius: f32, bottom_radius: f32) -> Result<Self> {
        Ok(Self {
            shape: Shape::new_tapered_cylinder(half_height, top_radius, bottom_radius, 0.05)?,
            visual: Visual::TaperedCylinder {
                half_height,
                top_radius,
                bottom_radius,
            },
        })
    }

    /// `of` scaled per axis.
    pub fn scaled(of: &Shaped, scale: [f32; 3]) -> Result<Self> {
        Ok(Self {
            shape: Shape::scaled(&of.shape, scale.into())?,
            visual: Visual::Scaled(Box::new(of.visual.clone()), scale),
        })
    }

    /// The convex hull of `points`, drawn with `faces`, the hull's triangles, which the caller
    /// knows for its few fixed hulls; each face is turned to face outward.
    pub fn hull(points: &[[f32; 3]], faces: &[[u32; 3]]) -> Result<Self> {
        let jolt_points: Vec<Vec3> = points.iter().map(|&p| p.into()).collect();
        Ok(Self {
            shape: Shape::new_convex_hull(&jolt_points, 0.05)?,
            visual: Visual::Triangles {
                vertices: points.to_vec(),
                triangles: face_outward(points, faces),
            },
        })
    }

    /// Static ground: the plane y = 0 of the body, solid below, `half_extent` metres to each
    /// side.
    pub fn plane(half_extent: f32) -> Result<Self> {
        let h = half_extent;
        Ok(Self {
            shape: Shape::new_plane(Vec3::new(0.0, 1.0, 0.0), 0.0, h)?,
            visual: Visual::Triangles {
                vertices: vec![[-h, 0.0, -h], [h, 0.0, -h], [h, 0.0, h], [-h, 0.0, h]],
                triangles: vec![[0, 2, 1], [0, 3, 2]],
            },
        })
    }

    /// A static triangle mesh. Jolt drops triangles too thin to collide with; the scenes' meshes
    /// have none, which their tests check, so the description draws the triangles Jolt kept.
    pub fn mesh(vertices: &[[f32; 3]], triangles: &[[u32; 3]]) -> Result<(Self, usize)> {
        let jolt_vertices: Vec<Vec3> = vertices.iter().map(|&p| p.into()).collect();
        let (shape, dropped) = Shape::new_mesh(&jolt_vertices, triangles)?;
        let kept = triangles
            .iter()
            .enumerate()
            .filter(|(index, _)| dropped.indices().binary_search(index).is_err())
            .map(|(_, &triangle)| triangle)
            .collect();
        let shaped = Self {
            shape,
            visual: Visual::Triangles {
                vertices: vertices.to_vec(),
                triangles: kept,
            },
        };
        Ok((shaped, dropped.count()))
    }

    /// A heightfield of `n` x `n` samples, drawn from the positions the built shape reports.
    pub fn height_field(n: u32, samples: &[f32], settings: &HeightFieldSettings) -> Result<Self> {
        let shape = Shape::new_height_field(n, samples, settings)?;
        let mut vertices = Vec::with_capacity((n * n) as usize);
        let mut present = Vec::with_capacity((n * n) as usize);
        for y in 0..n {
            for x in 0..n {
                let position = shape.height_field_position(x, y);
                present.push(position.is_some());
                vertices.push(position.map_or([0.0; 3], <[f32; 3]>::from));
            }
        }
        let mut triangles = Vec::new();
        for y in 0..n - 1 {
            for x in 0..n - 1 {
                let i = y * n + x;
                let corners = [i, i + 1, i + n, i + n + 1];
                if corners.iter().all(|&c| present[c as usize]) {
                    // Jolt splits each cell along the diagonal from (x, y) to (x + 1, y + 1).
                    triangles.push([i, i + n + 1, i + 1]);
                    triangles.push([i, i + n, i + n + 1]);
                }
            }
        }
        Ok(Self {
            shape,
            visual: Visual::Triangles {
                vertices,
                triangles,
            },
        })
    }

    /// A compound of `children`: each a shape, its position, rotation and user data.
    pub fn compound(children: &[(&Shaped, [f32; 3], Quat, u32)]) -> Result<Self> {
        let jolt_children: Vec<CompoundChild<'_>> = children
            .iter()
            .map(|&(child, position, rotation, user_data)| CompoundChild {
                shape: &child.shape,
                position: position.into(),
                rotation,
                user_data,
            })
            .collect();
        Ok(Self {
            shape: Shape::new_compound(&jolt_children)?,
            visual: Visual::Compound(
                children
                    .iter()
                    .map(|&(child, position, rotation, _)| {
                        (child.visual.clone(), position, rotation.into())
                    })
                    .collect(),
            ),
        })
    }
}

/// The triangles of a convex solid of `points`, each turned to face away from the points'
/// centre.
pub fn face_outward(points: &[[f32; 3]], faces: &[[u32; 3]]) -> Vec<[u32; 3]> {
    let point = |index: u32| glam::Vec3::from(points[index as usize]);
    let centre = points
        .iter()
        .map(|&p| glam::Vec3::from(p))
        .sum::<glam::Vec3>()
        / points.len() as f32;
    faces
        .iter()
        .map(|&[a, b, c]| {
            let normal = (point(b) - point(a)).cross(point(c) - point(a));
            let face_centre = (point(a) + point(b) + point(c)) / 3.0;
            if normal.dot(face_centre - centre) < 0.0 {
                [a, c, b]
            } else {
                [a, b, c]
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn replacing_a_visual_bumps_its_version_only() {
        let mut visuals = Visuals::new(7);
        let a = visuals.add(Visual::Sphere { radius: 1.0 });
        let b = visuals.add(Visual::Sphere { radius: 2.0 });
        let a2 = visuals.replace(a, Visual::Sphere { radius: 3.0 });
        assert_eq!((a.generation, a.id, a.version), (7, 0, 0));
        assert_eq!((a2.id, a2.version), (0, 1));
        assert_eq!(b.version, 0);
        assert_eq!(visuals.get(a2), &Visual::Sphere { radius: 3.0 });
    }

    #[test]
    fn a_height_field_visual_follows_the_shape() {
        let n = 5;
        let samples: Vec<f32> = (0..n * n).map(|i| (i % 3) as f32 * 0.1).collect();
        let settings = HeightFieldSettings::default().offset(Vec3::new(-2.0, 0.0, -2.0));
        let shaped = Shaped::height_field(n, &samples, &settings).unwrap();
        let Visual::Triangles {
            vertices,
            triangles,
        } = &shaped.visual
        else {
            panic!("a heightfield is drawn as triangles");
        };
        assert_eq!(triangles.len(), 2 * 4 * 4);
        let sample = |x: u32, y: u32| shaped.shape.height_field_position(x, y).unwrap();
        assert_eq!(
            vertices[(3 * n + 2) as usize],
            <[f32; 3]>::from(sample(2, 3))
        );
    }
}
