//! Wavefront OBJ: `v` and `f` statements; everything else is skipped.

use crate::{Error, TriangleMesh};

/// Parses OBJ text. Faces may use `v`, `v/vt`, `v/vt/vn` and `v//vn` references, negative
/// (relative) indices and any number of corners (fan-triangulated from the first corner).
/// Positions with a fourth (`w`) component keep x, y and z.
pub fn parse_obj(text: &str) -> Result<TriangleMesh, Error> {
    let mut mesh = TriangleMesh::default();
    for (number, line) in text.lines().enumerate() {
        let line_number = number + 1;
        let error = |message: String| Error::Obj {
            line: line_number,
            message,
        };
        let line = line.split('#').next().unwrap_or_default();
        let mut words = line.split_whitespace();
        match words.next() {
            Some("v") => {
                let mut position = [0.0f32; 3];
                for coordinate in &mut position {
                    let word = words
                        .next()
                        .ok_or_else(|| error("a vertex needs three coordinates".into()))?;
                    *coordinate = word
                        .parse()
                        .map_err(|_| error(format!("{word:?} is not a number")))?;
                }
                mesh.vertices.push(position);
            }
            Some("f") => {
                let corners = words
                    .map(|word| corner(word, mesh.vertices.len()).map_err(error))
                    .collect::<Result<Vec<u32>, Error>>()?;
                if corners.len() < 3 {
                    return Err(error("a face needs at least three corners".into()));
                }
                for pair in corners[1..].windows(2) {
                    mesh.triangles.push([corners[0], pair[0], pair[1]]);
                }
            }
            _ => {}
        }
    }
    Ok(mesh)
}

/// The zero-based vertex index of a face corner such as `7`, `7/2`, `7/2/3`, `7//3` or `-1`,
/// with `count` vertices read so far.
fn corner(word: &str, count: usize) -> Result<u32, String> {
    let index = word.split('/').next().unwrap_or_default();
    let value: i64 = index
        .parse()
        .map_err(|_| format!("{word:?} is not a vertex reference"))?;
    let zero_based = match value {
        0 => return Err("vertex index 0 does not exist (indices start at 1)".into()),
        v if v > 0 => v - 1,
        v => count as i64 + v,
    };
    if zero_based < 0 || zero_based >= count as i64 {
        return Err(format!(
            "vertex {value} is out of range ({count} vertices so far)"
        ));
    }
    u32::try_from(zero_based).map_err(|_| format!("vertex {value} does not fit 32 bits"))
}

#[cfg(test)]
mod tests {
    use super::*;

    const SQUARE: &str = "v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\n";

    fn triangles(faces: &str) -> Vec<[u32; 3]> {
        parse_obj(&format!("{SQUARE}{faces}")).unwrap().triangles
    }

    #[test]
    fn reads_vertices_and_skips_other_statements() {
        let mesh = parse_obj(
            "# comment\nmtllib a.mtl\no square\nv 0 0 0\nv 1.5 -2 3e-1 1.0\nvt 0 1\nvn 0 0 1\ns off\n",
        )
        .unwrap();
        assert_eq!(mesh.vertices, [[0.0, 0.0, 0.0], [1.5, -2.0, 0.3]]);
        assert!(mesh.triangles.is_empty());
    }

    #[test]
    fn every_corner_form_names_the_vertex() {
        let expected = [[0, 1, 2]];
        assert_eq!(triangles("f 1 2 3\n"), expected);
        assert_eq!(triangles("f 1/1 2/2 3/3\n"), expected);
        assert_eq!(triangles("f 1/1/1 2/2/2 3/3/3\n"), expected);
        assert_eq!(triangles("f 1//1 2//2 3//3\n"), expected);
        assert_eq!(triangles("f -4 -3 -2\n"), expected);
        assert_eq!(triangles("f 1 2 3 # trailing comment\n"), expected);
    }

    #[test]
    fn polygons_are_fanned_from_the_first_corner() {
        assert_eq!(triangles("f 1 2 3 4\n"), [[0, 1, 2], [0, 2, 3]]);
        let pentagon = parse_obj("v 0 0 0\nv 1 0 0\nv 2 1 0\nv 1 2 0\nv 0 1 0\nf 1 2 3 4 5\n")
            .unwrap()
            .triangles;
        assert_eq!(pentagon, [[0, 1, 2], [0, 2, 3], [0, 3, 4]]);
    }

    #[test]
    fn negative_indices_count_back_from_the_last_vertex_read() {
        let mesh =
            parse_obj("v 0 0 0\nv 1 0 0\nv 0 1 0\nf -3 -2 -1\nv 5 5 5\nf -4 -3 -1\n").unwrap();
        assert_eq!(mesh.triangles, [[0, 1, 2], [0, 1, 3]]);
    }

    #[test]
    fn malformed_lines_are_refused_with_their_number() {
        for (text, line) in [
            ("v 0 0\n", 1),
            ("v 0 x 0\n", 1),
            ("v 0 0 0\nv 1 0 0\nf 1 2\n", 3),
            ("v 0 0 0\nf 0 1 1\n", 2),
            ("v 0 0 0\nf 1 1 2\n", 2),
            ("v 0 0 0\nf -2 1 1\n", 2),
            ("v 0 0 0\nf a 1 1\n", 2),
        ] {
            match parse_obj(text) {
                Err(Error::Obj { line: found, .. }) => assert_eq!(found, line, "{text:?}"),
                other => panic!("{text:?}: expected an OBJ error, got {other:?}"),
            }
        }
    }
}
