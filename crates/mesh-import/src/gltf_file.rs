//! glTF 2.0: `.glb` with its binary chunk, or `.gltf` with buffers in files next to it.

use std::path::Path;

use gltf::buffer::Source;
use gltf::mesh::Mode;

use crate::{read, Error, TriangleMesh};

/// A column-major 4 x 4 matrix, as glTF stores node transforms.
type Matrix = [[f32; 4]; 4];

const IDENTITY: Matrix = [
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
];

/// Every triangle primitive of every mesh instance of the default scene (the first scene when
/// none is marked default), in world space.
pub(crate) fn load(path: &Path) -> Result<TriangleMesh, Error> {
    let document = gltf::Gltf::from_slice(&read(path)?).map_err(Error::Gltf)?;
    let buffers = document
        .buffers()
        .map(|buffer| match buffer.source() {
            Source::Bin => document
                .blob
                .clone()
                .ok_or_else(|| Error::Unsupported("a GLB buffer without a binary chunk".into())),
            Source::Uri(uri) if uri.starts_with("data:") => {
                Err(Error::Unsupported("buffers embedded as data URIs".into()))
            }
            Source::Uri(uri) => read(&path.with_file_name(uri)),
        })
        .collect::<Result<Vec<_>, _>>()?;

    let scene = document
        .default_scene()
        .or_else(|| document.scenes().next())
        .ok_or_else(|| Error::Unsupported("a file without scenes".into()))?;
    let mut mesh = TriangleMesh::default();
    let mut stack: Vec<_> = scene.nodes().map(|node| (node, IDENTITY)).collect();
    while let Some((node, parent)) = stack.pop() {
        let world = multiply(&parent, &node.transform().matrix());
        if let Some(node_mesh) = node.mesh() {
            for primitive in node_mesh.primitives() {
                add_primitive(&primitive, &buffers, &world, &mut mesh)?;
            }
        }
        stack.extend(node.children().map(|child| (child, world)));
    }
    Ok(mesh)
}

fn add_primitive(
    primitive: &gltf::Primitive<'_>,
    buffers: &[Vec<u8>],
    world: &Matrix,
    mesh: &mut TriangleMesh,
) -> Result<(), Error> {
    if primitive.mode() != Mode::Triangles {
        return Err(Error::Unsupported(format!(
            "primitive mode {:?} (only triangle lists)",
            primitive.mode()
        )));
    }
    let reader = primitive.reader(|buffer| buffers.get(buffer.index()).map(Vec::as_slice));
    let positions: Vec<[f32; 3]> = reader
        .read_positions()
        .ok_or_else(|| Error::Unsupported("a primitive without positions".into()))?
        .map(|p| transform(world, p))
        .collect();
    let indices: Vec<u32> = match reader.read_indices() {
        Some(indices) => indices.into_u32().collect(),
        None => (0..positions.len() as u32).collect(),
    };
    if !indices.len().is_multiple_of(3) || indices.iter().any(|&i| i as usize >= positions.len()) {
        return Err(Error::Unsupported(
            "a triangle list whose indices are incomplete or out of range".into(),
        ));
    }
    // A mirroring transform turns counter-clockwise triangles clockwise; swap two corners back.
    let mirrored = determinant(world) < 0.0;
    let base = u32::try_from(mesh.vertices.len())
        .map_err(|_| Error::Unsupported("more than u32::MAX vertices".into()))?;
    mesh.vertices.extend(positions);
    let (triangles, _) = indices.as_chunks::<3>();
    mesh.triangles.extend(triangles.iter().map(|t| {
        let [a, b, c] = t.map(|i| i + base);
        if mirrored {
            [a, c, b]
        } else {
            [a, b, c]
        }
    }));
    Ok(())
}

fn multiply(a: &Matrix, b: &Matrix) -> Matrix {
    let mut out = [[0.0; 4]; 4];
    for (column, out_column) in out.iter_mut().enumerate() {
        for (row, value) in out_column.iter_mut().enumerate() {
            *value = (0..4).map(|k| a[k][row] * b[column][k]).sum();
        }
    }
    out
}

fn transform(m: &Matrix, p: [f32; 3]) -> [f32; 3] {
    let mut out = [0.0; 3];
    for (row, value) in out.iter_mut().enumerate() {
        *value = m[0][row] * p[0] + m[1][row] * p[1] + m[2][row] * p[2] + m[3][row];
    }
    out
}

/// Determinant of the upper-left 3 x 3 part.
fn determinant(m: &Matrix) -> f32 {
    m[0][0] * (m[1][1] * m[2][2] - m[2][1] * m[1][2])
        - m[1][0] * (m[0][1] * m[2][2] - m[2][1] * m[0][2])
        + m[2][0] * (m[0][1] * m[1][2] - m[1][1] * m[0][2])
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A glTF file with one triangle, as a JSON document with its buffer as `data`, two nodes
    /// (a parent translated by +10 x and a child scaled by -1 in x holding the mesh) and `mode`.
    fn document(mode: u32) -> (String, Vec<u8>) {
        let mut data = Vec::new();
        for value in [0.0f32, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0] {
            data.extend(value.to_le_bytes());
        }
        for index in [0u16, 1, 2] {
            data.extend(index.to_le_bytes());
        }
        data.extend([0, 0]);
        let json = format!(
            r#"{{
  "asset": {{ "version": "2.0" }},
  "scene": 0,
  "scenes": [ {{ "nodes": [0] }} ],
  "nodes": [
    {{ "translation": [10, 0, 0], "children": [1] }},
    {{ "scale": [-1, 1, 1], "mesh": 0 }}
  ],
  "meshes": [ {{ "primitives": [ {{ "attributes": {{ "POSITION": 0 }}, "indices": 1, "mode": {mode} }} ] }} ],
  "buffers": [ {{ "byteLength": {len}, "uri": "triangle.bin" }} ],
  "bufferViews": [
    {{ "buffer": 0, "byteOffset": 0, "byteLength": 36 }},
    {{ "buffer": 0, "byteOffset": 36, "byteLength": 6 }}
  ],
  "accessors": [
    {{ "bufferView": 0, "componentType": 5126, "count": 3, "type": "VEC3", "min": [0, 0, 0], "max": [1, 1, 0] }},
    {{ "bufferView": 1, "componentType": 5123, "count": 3, "type": "SCALAR" }}
  ]
}}"#,
            len = data.len()
        );
        (json, data)
    }

    fn write(name: &str, mode: u32) -> std::path::PathBuf {
        let dir = std::env::temp_dir().join(format!("mesh-import-{}-{name}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let (json, data) = document(mode);
        std::fs::write(dir.join("triangle.bin"), data).unwrap();
        let path = dir.join("triangle.gltf");
        std::fs::write(&path, json).unwrap();
        path
    }

    #[test]
    fn node_transforms_apply_and_mirroring_keeps_the_front_face() {
        let path = write("transforms", 4);
        let mesh = crate::load(&path).unwrap();
        std::fs::remove_dir_all(path.parent().unwrap()).unwrap();
        assert_eq!(
            mesh.vertices,
            [[10.0, 0.0, 0.0], [9.0, 0.0, 0.0], [10.0, 1.0, 0.0]]
        );
        assert_eq!(mesh.triangles, [[0, 2, 1]]);
    }

    #[test]
    fn primitives_that_are_not_triangle_lists_are_refused() {
        let path = write("strips", 5);
        let result = crate::load(&path);
        std::fs::remove_dir_all(path.parent().unwrap()).unwrap();
        assert!(matches!(result, Err(Error::Unsupported(_))), "{result:?}");
    }

    #[test]
    fn a_composed_transform_is_parent_then_child() {
        let mut translate = IDENTITY;
        translate[3] = [1.0, 2.0, 3.0, 1.0];
        let mut scale = IDENTITY;
        scale[0][0] = 2.0;
        let m = multiply(&translate, &scale);
        assert_eq!(transform(&m, [1.0, 1.0, 1.0]), [3.0, 3.0, 4.0]);
        assert_eq!(determinant(&m), 2.0);
    }
}
