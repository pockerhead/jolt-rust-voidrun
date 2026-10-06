//! Triangle meshes from Wavefront OBJ and glTF 2.0 files (`.gltf` with external buffers, `.glb`),
//! for oxijolt's real-mesh tests and the playground. Not published.
//!
//! Every mesh comes back as one vertex list and one triangle list, with glTF node transforms
//! applied, so the result goes straight into `Shape::new_mesh`. Texture coordinates, normals,
//! materials and images are ignored.

use std::fmt;
use std::path::{Path, PathBuf};

mod gltf_file;
mod obj;

pub use obj::parse_obj;

/// Vertices (metres, in the file's frame) and triangles (three indices into the vertices each,
/// counter-clockwise seen from the front as in the file).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct TriangleMesh {
    /// Vertex positions.
    pub vertices: Vec<[f32; 3]>,
    /// Vertex index triples.
    pub triangles: Vec<[u32; 3]>,
}

impl TriangleMesh {
    /// The smallest box holding every vertex, as `(min, max)`; `None` without vertices.
    pub fn bounds(&self) -> Option<([f32; 3], [f32; 3])> {
        let first = *self.vertices.first()?;
        Some(self.vertices.iter().fold((first, first), |(min, max), v| {
            (
                [min[0].min(v[0]), min[1].min(v[1]), min[2].min(v[2])],
                [max[0].max(v[0]), max[1].max(v[1]), max[2].max(v[2])],
            )
        }))
    }
}

/// Why a file could not be read as a triangle mesh.
#[derive(Debug)]
pub enum Error {
    /// The file (or a glTF buffer file) could not be read.
    Io(PathBuf, std::io::Error),
    /// A line of an OBJ file is malformed.
    Obj {
        /// Line number, from 1.
        line: usize,
        /// What is wrong.
        message: String,
    },
    /// The glTF parser refused the file.
    Gltf(gltf::Error),
    /// The file uses something this reader does not support; the text says what.
    Unsupported(String),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(path, error) => write!(f, "cannot read {}: {error}", path.display()),
            Self::Obj { line, message } => write!(f, "OBJ line {line}: {message}"),
            Self::Gltf(error) => write!(f, "glTF: {error}"),
            Self::Unsupported(what) => write!(f, "unsupported: {what}"),
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Io(_, error) => Some(error),
            Self::Gltf(error) => Some(error),
            _ => None,
        }
    }
}

/// Reads the mesh in `path`, chosen by extension: `.obj`, `.gltf` or `.glb` (any case).
pub fn load(path: impl AsRef<Path>) -> Result<TriangleMesh, Error> {
    let path = path.as_ref();
    let extension = path
        .extension()
        .and_then(|e| e.to_str())
        .map(str::to_ascii_lowercase);
    match extension.as_deref() {
        Some("obj") => {
            let text = read(path)?;
            parse_obj(&String::from_utf8_lossy(&text))
        }
        Some("gltf") | Some("glb") => gltf_file::load(path),
        _ => Err(Error::Unsupported(format!(
            "{}: expected .obj, .gltf or .glb",
            path.display()
        ))),
    }
}

fn read(path: &Path) -> Result<Vec<u8>, Error> {
    std::fs::read(path).map_err(|error| Error::Io(path.to_owned(), error))
}
