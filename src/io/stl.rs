//! STL import and export helpers for triangulated 3D geometry.

use crate::io::{IoError, finite_f32, finite_f64, single_line_metadata};
use crate::mesh::Mesh;
use crate::polygon::Polygon;
use crate::triangulated::Triangulated3D;
use crate::vertex::Vertex;
use hyperlattice::{Point3, Real, Vector3};
use std::fmt::Debug;
use std::io::Cursor;

fn triangle_normal(triangle: &[Vertex; 3]) -> Result<Vector3, IoError> {
    let first = &triangle[1].position - &triangle[0].position;
    let second = &triangle[2].position - &triangle[0].position;
    first
        .unit_cross_checked(&second)
        .map_err(|error| IoError::Geometry {
            format: "STL",
            detail: format!("degenerate triangle: {error}"),
        })
}

/// Serialize a triangulated shape as ASCII STL.
pub fn to_stl_ascii<T: Triangulated3D>(shape: &T, name: &str) -> Result<String, IoError> {
    let name = single_line_metadata(name, "STL", "solid name")?;
    let mut out = format!("solid {name}\n");
    let mut failure = None;
    shape.visit_triangles(|triangle| {
        if failure.is_some() {
            return;
        }
        let result = (|| {
            let normal = triangle_normal(&triangle)?;
            out.push_str(&format!(
                "  facet normal {:.17} {:.17} {:.17}\n",
                finite_f64(&normal.0[0], "STL", "normal x")?,
                finite_f64(&normal.0[1], "STL", "normal y")?,
                finite_f64(&normal.0[2], "STL", "normal z")?,
            ));
            out.push_str("    outer loop\n");
            for vertex in &triangle {
                out.push_str(&format!(
                    "      vertex {:.17} {:.17} {:.17}\n",
                    finite_f64(&vertex.position.x, "STL", "vertex x")?,
                    finite_f64(&vertex.position.y, "STL", "vertex y")?,
                    finite_f64(&vertex.position.z, "STL", "vertex z")?,
                ));
            }
            out.push_str("    endloop\n  endfacet\n");
            Ok::<_, IoError>(())
        })();
        if let Err(error) = result {
            failure = Some(error);
        }
    });
    if let Some(error) = failure {
        return Err(error);
    }
    out.push_str(&format!("endsolid {name}\n"));
    Ok(out)
}

/// Serialize a triangulated shape as binary STL.
pub fn to_stl_binary<T: Triangulated3D>(shape: &T, name: &str) -> Result<Vec<u8>, IoError> {
    use stl_io::{Normal, Triangle, Vertex as StlVertex, write_stl};

    single_line_metadata(name, "STL", "solid name")?;
    let mut triangles = Vec::<Triangle>::new();
    let mut failure = None;
    shape.visit_triangles(|triangle| {
        if failure.is_some() {
            return;
        }
        let result = (|| {
            let normal = triangle_normal(&triangle)?;
            Ok::<_, IoError>(Triangle {
                normal: Normal::new([
                    finite_f32(&normal.0[0], "STL", "normal x")?,
                    finite_f32(&normal.0[1], "STL", "normal y")?,
                    finite_f32(&normal.0[2], "STL", "normal z")?,
                ]),
                vertices: [
                    stl_vertex(&triangle[0])?,
                    stl_vertex(&triangle[1])?,
                    stl_vertex(&triangle[2])?,
                ],
            })
        })();
        match result {
            Ok(triangle) => triangles.push(triangle),
            Err(error) => failure = Some(error),
        }
    });
    if let Some(error) = failure {
        return Err(error);
    }

    fn stl_vertex(vertex: &Vertex) -> Result<stl_io::Vertex, IoError> {
        Ok(StlVertex::new([
            finite_f32(&vertex.position.x, "STL", "vertex x")?,
            finite_f32(&vertex.position.y, "STL", "vertex y")?,
            finite_f32(&vertex.position.z, "STL", "vertex z")?,
        ]))
    }

    let mut cursor = Cursor::new(Vec::new());
    write_stl(&mut cursor, triangles.iter())?;
    Ok(cursor.into_inner())
}

/// Import a closed, consistently oriented STL mesh.
pub fn from_stl<M>(data: &[u8], metadata: M) -> Result<Mesh<M>, IoError>
where
    M: Clone + Debug + Send + Sync,
{
    let mut cursor = Cursor::new(data);
    let indexed = stl_io::read_stl(&mut cursor)?;
    indexed.validate()?;

    let positions = indexed
        .vertices
        .iter()
        .map(|vertex| {
            Ok(Point3::new(
                Real::try_from(vertex[0]).map_err(|error| {
                    IoError::MalformedInput(format!("STL vertex x is invalid: {error}"))
                })?,
                Real::try_from(vertex[1]).map_err(|error| {
                    IoError::MalformedInput(format!("STL vertex y is invalid: {error}"))
                })?,
                Real::try_from(vertex[2]).map_err(|error| {
                    IoError::MalformedInput(format!("STL vertex z is invalid: {error}"))
                })?,
            ))
        })
        .collect::<Result<Vec<_>, IoError>>()?;

    let mut polygons = Vec::with_capacity(indexed.faces.len());
    for face in indexed.faces {
        let points = face.vertices.map(|index| positions[index].clone());
        let normal = (&points[1] - &points[0])
            .unit_cross_checked(&(&points[2] - &points[0]))
            .map_err(|error| IoError::Geometry {
                format: "STL",
                detail: format!("degenerate imported triangle: {error}"),
            })?;
        polygons.push(Polygon::new(
            points
                .map(|position| Vertex::new(position, normal.clone()))
                .to_vec(),
            metadata.clone(),
        ));
    }
    Ok(Mesh::from_polygons(&polygons, metadata))
}

impl<M: Clone + Debug + Send + Sync> Mesh<M> {
    pub fn to_stl_ascii(&self, name: &str) -> Result<String, IoError> {
        to_stl_ascii(self, name)
    }

    pub fn to_stl_binary(&self, name: &str) -> Result<Vec<u8>, IoError> {
        to_stl_binary(self, name)
    }

    pub fn from_stl(data: &[u8], metadata: M) -> Result<Self, IoError> {
        from_stl(data, metadata)
    }
}

#[cfg(feature = "sketch")]
impl<M: Clone + Debug + Send + Sync> crate::sketch::Profile<M> {
    pub fn to_stl_ascii(&self, name: &str) -> Result<String, IoError> {
        to_stl_ascii(self, name)
    }

    pub fn to_stl_binary(&self, name: &str) -> Result<Vec<u8>, IoError> {
        to_stl_binary(self, name)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn binary_cube_round_trips_through_validated_import() {
        let cube = Mesh::<()>::cube(Real::from(2_u8), ());
        let bytes = cube.to_stl_binary("cube").unwrap();
        let imported = Mesh::from_stl(&bytes, ()).unwrap();

        assert_eq!(imported.polygons.len(), 12);
    }

    #[test]
    fn ascii_name_rejects_record_injection() {
        let cube = Mesh::<()>::cube(Real::one(), ());
        assert!(cube.to_stl_ascii("safe\nendsolid forged").is_err());
    }
}
