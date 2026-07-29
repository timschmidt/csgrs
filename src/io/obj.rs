//! Wavefront OBJ import and export for native Hypermesh triangles.

use super::{
    IoError, finite_f64, parse_real_decimal, single_line_metadata, triangulate_planar_face,
};
use hyperlattice::{Point3, Real};
use hypermesh::{Triangle, TriangleMesh};
#[cfg(feature = "attributed")]
use std::collections::HashMap;
use std::io::{BufRead, Write};

fn malformed(line: usize, message: impl std::fmt::Display) -> IoError {
    IoError::MalformedInput(format!("OBJ line {line}: {message}"))
}

fn parse_real(text: &str, line: usize) -> Result<Real, IoError> {
    parse_real_decimal(text).map_err(|error| malformed(line, error))
}

fn parse_index(text: &str, position_count: usize, line: usize) -> Result<usize, IoError> {
    let head = text.split('/').next().unwrap_or_default();
    let raw = head
        .parse::<isize>()
        .map_err(|error| malformed(line, format!("invalid face index: {error}")))?;
    let index = if raw > 0 {
        usize::try_from(raw - 1).ok()
    } else if raw < 0 {
        position_count.checked_sub(raw.unsigned_abs())
    } else {
        None
    };
    index
        .filter(|index| *index < position_count)
        .ok_or_else(|| malformed(line, "face index is outside the position array"))
}

#[cfg(feature = "attributed")]
fn parse_optional_normal_index(
    text: &str,
    normal_count: usize,
    line: usize,
) -> Result<Option<usize>, IoError> {
    let mut fields = text.split('/');
    let _ = fields.next();
    let _ = fields.next();
    let normal = fields.next();
    if fields.next().is_some() {
        return Err(malformed(line, "face token has too many index fields"));
    }
    let Some(raw) = normal.filter(|raw| !raw.is_empty()) else {
        return Ok(None);
    };
    let raw = raw
        .parse::<isize>()
        .map_err(|error| malformed(line, format!("invalid normal index: {error}")))?;
    let index = if raw > 0 {
        usize::try_from(raw - 1).ok()
    } else if raw < 0 {
        normal_count.checked_sub(raw.unsigned_abs())
    } else {
        None
    };
    index
        .filter(|index| *index < normal_count)
        .map(Some)
        .ok_or_else(|| malformed(line, "normal index is outside the normal array"))
}

fn triangulate_face(
    positions: &[Point3],
    face: &[usize],
    line: usize,
) -> Result<Vec<Triangle>, IoError> {
    triangulate_planar_face(positions, face, "OBJ").map_err(|error| malformed(line, error))
}

fn retain_import_facts(mesh: &TriangleMesh) {
    let _ = mesh.exact_bounds();
    let _ = mesh.adjacency();
    let _ = mesh.connectivity_counts();
    let _ = mesh.is_closed_manifold();
}

/// Import OBJ vertex and face records into native triangle geometry.
pub fn from_obj<R: BufRead>(reader: R) -> Result<TriangleMesh, IoError> {
    let mut positions = Vec::new();
    let mut triangles = Vec::new();
    for (line_index, line) in reader.lines().enumerate() {
        let line_number = line_index + 1;
        let line = line?;
        let line = line.split('#').next().unwrap_or_default().trim();
        let mut fields = line.split_whitespace();
        match fields.next() {
            Some("v") => {
                let values = fields.collect::<Vec<_>>();
                if values.len() < 3 {
                    return Err(malformed(line_number, "vertex needs x y z"));
                }
                positions.push(Point3::new(
                    parse_real(values[0], line_number)?,
                    parse_real(values[1], line_number)?,
                    parse_real(values[2], line_number)?,
                ));
            },
            Some("f") => {
                let face = fields
                    .map(|field| parse_index(field, positions.len(), line_number))
                    .collect::<Result<Vec<_>, _>>()?;
                triangles.extend(triangulate_face(&positions, &face, line_number)?);
            },
            _ => {},
        }
    }
    let mesh = TriangleMesh::new(positions, triangles);
    retain_import_facts(&mesh);
    Ok(mesh)
}

/// Imports OBJ geometry with an optional per-position authored-normal sidecar.
///
/// Position/normal pairs are indexed independently at the file boundary and
/// collapsed into the thin [`crate::AttributedMesh`] carrier. Core modeling
/// continues to consume only the returned native geometry.
#[cfg(feature = "attributed")]
pub fn from_obj_attributed<R: BufRead>(
    reader: R,
) -> Result<crate::AttributedMesh<()>, IoError> {
    let mut source_positions = Vec::new();
    let mut source_normals = Vec::new();
    let mut positions = Vec::new();
    let mut authored_normals = Vec::new();
    let mut triangles = Vec::new();
    let mut position_normal_slots = HashMap::<(usize, Option<usize>), usize>::new();
    let mut all_corners_have_normals = true;

    for (line_index, line) in reader.lines().enumerate() {
        let line_number = line_index + 1;
        let line = line?;
        let line = line.split('#').next().unwrap_or_default().trim();
        let mut fields = line.split_whitespace();
        match fields.next() {
            Some("v") => {
                let values = fields.collect::<Vec<_>>();
                if values.len() < 3 {
                    return Err(malformed(line_number, "vertex needs x y z"));
                }
                source_positions.push(Point3::new(
                    parse_real(values[0], line_number)?,
                    parse_real(values[1], line_number)?,
                    parse_real(values[2], line_number)?,
                ));
            },
            Some("vn") => {
                let values = fields.collect::<Vec<_>>();
                if values.len() != 3 {
                    return Err(malformed(line_number, "normal needs x y z"));
                }
                source_normals.push(hyperlattice::Vector3::from_xyz(
                    parse_real(values[0], line_number)?,
                    parse_real(values[1], line_number)?,
                    parse_real(values[2], line_number)?,
                ));
            },
            Some("f") => {
                let corners = fields
                    .map(|field| {
                        Ok((
                            parse_index(field, source_positions.len(), line_number)?,
                            parse_optional_normal_index(
                                field,
                                source_normals.len(),
                                line_number,
                            )?,
                        ))
                    })
                    .collect::<Result<Vec<_>, IoError>>()?;
                all_corners_have_normals &= corners.iter().all(|(_, normal)| normal.is_some());
                let face = corners
                    .iter()
                    .map(|(position, _)| *position)
                    .collect::<Vec<_>>();
                let face_triangles = triangulate_face(&source_positions, &face, line_number)?;
                let normal_for_position = |position| {
                    corners
                        .iter()
                        .find_map(|(candidate, normal)| {
                            (*candidate == position).then_some(*normal)
                        })
                        .flatten()
                };
                for triangle in face_triangles {
                    let mut remapped = [0; 3];
                    for (corner, position) in triangle.indices().into_iter().enumerate() {
                        let normal = normal_for_position(position);
                        remapped[corner] = *position_normal_slots
                            .entry((position, normal))
                            .or_insert_with(|| {
                                let slot = positions.len();
                                positions.push(source_positions[position].clone());
                                authored_normals.push(
                                    normal
                                        .map(|index| source_normals[index].clone())
                                        .unwrap_or_else(hyperlattice::Vector3::z),
                                );
                                slot
                            });
                    }
                    triangles.push(Triangle::new(remapped[0], remapped[1], remapped[2]));
                }
            },
            _ => {},
        }
    }

    let geometry = TriangleMesh::new(positions, triangles);
    retain_import_facts(&geometry);
    let metadata = vec![(); geometry.triangles.len()];
    let attributed = crate::AttributedMesh::with_authored_normals(
        geometry,
        metadata,
        all_corners_have_normals.then_some(authored_normals),
    )
    .map_err(|error| IoError::Geometry {
        format: "OBJ",
        detail: error.to_string(),
    })?;
    let _ = attributed.exact_gpu_mesh_buffers();
    Ok(attributed)
}

/// Export native triangle geometry as Wavefront OBJ.
pub fn to_obj(mesh: &TriangleMesh, object_name: &str) -> Result<String, IoError> {
    let object_name = single_line_metadata(object_name, "OBJ", "object name")?;
    if object_name.contains('#') {
        return Err(IoError::InvalidMetadata {
            format: "OBJ",
            field: "object name",
        });
    }
    let mut output = format!("# Generated by csgrs\no {object_name}\n");
    for point in mesh.positions.iter() {
        output.push_str(&format!(
            "v {:.17} {:.17} {:.17}\n",
            finite_f64(&point.x, "OBJ", "vertex x")?,
            finite_f64(&point.y, "OBJ", "vertex y")?,
            finite_f64(&point.z, "OBJ", "vertex z")?,
        ));
    }
    for triangle in mesh.triangles.iter() {
        let [a, b, c] = triangle.indices();
        output.push_str(&format!("f {} {} {}\n", a + 1, b + 1, c + 1));
    }
    Ok(output)
}

/// Write OBJ text to an output stream.
pub fn write_obj<W: Write>(
    mesh: &TriangleMesh,
    writer: &mut W,
    object_name: &str,
) -> Result<(), IoError> {
    writer.write_all(to_obj(mesh, object_name)?.as_bytes())?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    const QUAD_CUBE: &[u8] = br#"v -1 -1 -1
v 1 -1 -1
v 1 1 -1
v -1 1 -1
v -1 -1 1
v 1 -1 1
v 1 1 1
v -1 1 1
vn -1 -1 -1
vn 1 -1 -1
vn 1 1 -1
vn -1 1 -1
vn -1 -1 1
vn 1 -1 1
vn 1 1 1
vn -1 1 1
f 1//1 4//4 3//3 2//2
f 5//5 6//6 7//7 8//8
f 1//1 2//2 6//6 5//5
f 2//2 3//3 7//7 6//6
f 3//3 4//4 8//8 7//7
f 4//4 1//1 5//5 8//8
"#;

    #[test]
    fn cube_round_trips() {
        let cube = crate::solid::cube(Real::from(2_u8));
        let text = to_obj(&cube, "cube").unwrap();
        let imported = from_obj(text.as_bytes()).unwrap();
        assert_eq!(imported.triangles.len(), cube.triangles.len());
    }

    #[test]
    fn scientific_notation_is_promoted_exactly() {
        let imported =
            from_obj(b"v 7.78437e-005 0 0\nv 0 1e+0 0\nv 0 0 1E0\nf 1 2 3\n".as_slice())
                .expect("scientific OBJ coordinates");
        assert_eq!(
            imported.positions[0].x,
            "0.0000778437".parse::<Real>().unwrap()
        );
        assert_eq!(imported.triangles.len(), 1);
    }

    #[test]
    fn arbitrary_polygon_fixture_triangulates_closed() {
        let imported = from_obj(QUAD_CUBE).expect("quad cube OBJ");
        assert_eq!(imported.positions.len(), 8);
        assert_eq!(imported.triangles.len(), 12);
        assert!(imported.is_closed_manifold());
    }

    #[test]
    #[cfg(feature = "attributed")]
    fn authored_normals_stay_in_the_optional_sidecar() {
        let imported = from_obj_attributed(QUAD_CUBE).expect("attributed quad cube OBJ");
        assert_eq!(imported.geometry().positions.len(), 8);
        assert_eq!(imported.geometry().triangles.len(), 12);
        assert_eq!(
            imported
                .authored_normals()
                .expect("quad cube has authored normals")
                .len(),
            8
        );
        assert_eq!(
            imported
                .exact_gpu_mesh_buffers()
                .expect("authored graphics rows")
                .indices
                .len(),
            3 * 12
        );
    }
}
