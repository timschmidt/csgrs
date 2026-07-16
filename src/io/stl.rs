//! STL import and export helpers for triangulated 3D geometry.

use crate::io::{IoError, finite_f32, finite_f64, single_line_metadata};
use crate::mesh::Mesh;
use crate::mesh::Polygon;
use crate::triangulated::Triangulated3D;
use crate::vertex::Vertex;
use hyperlattice::{Point3, Real, Vector3};
use std::fmt::Debug;
use std::io::Cursor;

fn require_triangle_normal(normal: Option<Vector3>) -> Result<Vector3, IoError> {
    normal.ok_or_else(|| IoError::Geometry {
        format: "STL",
        detail: "degenerate triangle: no certified facet normal".to_owned(),
    })
}

fn validate_binary_triangle_count(count: usize) -> Result<(), IoError> {
    u32::try_from(count)
        .map(|_| ())
        .map_err(|_| IoError::SizeOverflow {
            format: "STL",
            limit: "32-bit binary triangle count",
        })
}

fn finite_stl_position_f64(vertex: &Vertex) -> Result<[f64; 3], IoError> {
    if let Some(position) = vertex.position_f64_lossy() {
        return Ok(position);
    }
    Ok([
        finite_f64(&vertex.position.x, "STL", "vertex x")?,
        finite_f64(&vertex.position.y, "STL", "vertex y")?,
        finite_f64(&vertex.position.z, "STL", "vertex z")?,
    ])
}

fn finite_stl_position(vertex: &Vertex) -> Result<[f32; 3], IoError> {
    let position = finite_stl_position_f64(vertex)?.map(|component| component as f32);
    if position.iter().all(|component| component.is_finite()) {
        return Ok(position);
    }
    Ok([
        finite_f32(&vertex.position.x, "STL", "vertex x")?,
        finite_f32(&vertex.position.y, "STL", "vertex y")?,
        finite_f32(&vertex.position.z, "STL", "vertex z")?,
    ])
}

fn mesh_stl_normal<M: Clone + Send + Sync>(
    polygon: &Polygon<M>,
    vertices: &[Vertex],
) -> Result<[f32; 3], IoError> {
    match polygon.certified_nondegenerate() {
        Some(false) => {
            return Err(IoError::Geometry {
                format: "STL",
                detail: "degenerate triangle: no certified facet normal".to_owned(),
            });
        },
        None => {
            let normal = require_triangle_normal(polygon.plane().unit_hreal_normal())?;
            return Ok([
                finite_f32(&normal.0[0], "STL", "normal x")?,
                finite_f32(&normal.0[1], "STL", "normal y")?,
                finite_f32(&normal.0[2], "STL", "normal z")?,
            ]);
        },
        Some(true) => {},
    }
    let points = if vertices.len() == 3 {
        [
            finite_stl_position_f64(&vertices[0]),
            finite_stl_position_f64(&vertices[1]),
            finite_stl_position_f64(&vertices[2]),
        ]
    } else {
        let plane = polygon.plane();
        [&plane.point_a, &plane.point_b, &plane.point_c].map(|point| {
            Ok([
                finite_f64(&point.x, "STL", "normal support x")?,
                finite_f64(&point.y, "STL", "normal support y")?,
                finite_f64(&point.z, "STL", "normal support z")?,
            ])
        })
    };
    if let [Ok(a), Ok(b), Ok(c)] = points {
        let first = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let second = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
        let cross = [
            first[1] * second[2] - first[2] * second[1],
            first[2] * second[0] - first[0] * second[2],
            first[0] * second[1] - first[1] * second[0],
        ];
        let magnitude_squared =
            cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2];
        if magnitude_squared.is_finite() && magnitude_squared > 0.0 {
            let inverse_magnitude = magnitude_squared.sqrt().recip();
            let normal = cross.map(|component| (component * inverse_magnitude) as f32);
            if normal.iter().all(|component| component.is_finite()) {
                return Ok(normal);
            }
        }
    }

    let normal = require_triangle_normal(polygon.plane().unit_hreal_normal())?;
    Ok([
        finite_f32(&normal.0[0], "STL", "normal x")?,
        finite_f32(&normal.0[1], "STL", "normal y")?,
        finite_f32(&normal.0[2], "STL", "normal z")?,
    ])
}

fn push_binary_triangle(output: &mut Vec<u8>, normal: [f32; 3], vertices: [[f32; 3]; 3]) {
    let mut record = [0_u8; 50];
    let mut offset = 0;
    for component in normal.into_iter().chain(vertices.into_iter().flatten()) {
        record[offset..offset + 4].copy_from_slice(&component.to_le_bytes());
        offset += 4;
    }
    output.extend_from_slice(&record);
}

fn mesh_to_stl_binary<M>(mesh: &Mesh<M>, name: &str) -> Result<Vec<u8>, IoError>
where
    M: Clone + Debug + Send + Sync,
{
    single_line_metadata(name, "STL", "solid name")?;
    let triangle_count = mesh
        .polygons
        .iter()
        .map(|polygon| polygon.vertices().len().saturating_sub(2))
        .sum::<usize>();
    validate_binary_triangle_count(triangle_count)?;
    let mut output = Vec::with_capacity(84 + triangle_count.saturating_mul(50));
    output.resize(80, 0);
    output.extend_from_slice(&(triangle_count as u32).to_le_bytes());

    for polygon in &mesh.polygons {
        let vertices = polygon.vertices();
        let normal = mesh_stl_normal(polygon, vertices)?;
        if vertices.len() == 3 {
            push_binary_triangle(
                &mut output,
                normal,
                [
                    finite_stl_position(&vertices[0])?,
                    finite_stl_position(&vertices[1])?,
                    finite_stl_position(&vertices[2])?,
                ],
            );
        } else {
            let positions = vertices
                .iter()
                .map(finite_stl_position)
                .collect::<Result<Vec<_>, _>>()?;
            for indices in polygon.triangulate_indices() {
                push_binary_triangle(
                    &mut output,
                    normal,
                    [
                        positions[indices[0]],
                        positions[indices[1]],
                        positions[indices[2]],
                    ],
                );
            }
        }
    }
    Ok(output)
}

/// Serialize a triangulated shape as ASCII STL.
pub fn to_stl_ascii<T: Triangulated3D>(shape: &T, name: &str) -> Result<String, IoError> {
    let name = single_line_metadata(name, "STL", "solid name")?;
    let mut out = format!("solid {name}\n");
    let mut failure = None;
    shape.visit_triangle_facets(|triangle, normal| {
        if failure.is_some() {
            return;
        }
        let result = (|| {
            let normal = require_triangle_normal(normal)?;
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
    shape.visit_triangle_facets(|triangle, normal| {
        if failure.is_some() {
            return;
        }
        let result = (|| {
            let normal = require_triangle_normal(normal)?;
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
    validate_binary_triangle_count(triangles.len())?;

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
    Ok(Mesh::from_polygons(polygons))
}

impl<M: Clone + Debug + Send + Sync> Mesh<M> {
    pub fn to_stl_ascii(&self, name: &str) -> Result<String, IoError> {
        to_stl_ascii(self, name)
    }

    pub fn to_stl_binary(&self, name: &str) -> Result<Vec<u8>, IoError> {
        mesh_to_stl_binary(self, name)
    }

    pub fn from_stl(data: &[u8], metadata: M) -> Result<Self, IoError> {
        from_stl(data, metadata)
    }
}

#[cfg(feature = "sketch")]
impl crate::sketch::Profile {
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

    struct DegenerateTriangle;

    impl Triangulated3D for DegenerateTriangle {
        fn visit_triangles<F>(&self, mut visit: F)
        where
            F: FnMut([Vertex; 3]),
        {
            let point = Point3::new(Real::zero(), Real::zero(), Real::zero());
            let vertex = Vertex::new(point, Vector3::z());
            visit([vertex.clone(), vertex.clone(), vertex]);
        }
    }

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

    #[test]
    fn rejects_malformed_degenerate_and_overflowing_inputs() {
        assert!(Mesh::<()>::from_stl(b"not an STL", ()).is_err());
        assert!(matches!(
            to_stl_ascii(&DegenerateTriangle, "degenerate"),
            Err(IoError::Geometry { format: "STL", .. })
        ));
        assert!(matches!(
            validate_binary_triangle_count(usize::MAX),
            Err(IoError::SizeOverflow { format: "STL", .. })
        ));
    }
}
