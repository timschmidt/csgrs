//! STL import and export for native Hypermesh triangle geometry.

use crate::io::{
    IoError, finite_f32, finite_f64, finite_triangle_normal, single_line_metadata,
};
use hyperlattice::{Point3, Real};
use hypermesh::{Triangle, TriangleMesh};
use std::cell::RefCell;
use std::io::Cursor;
use std::sync::Arc;

#[derive(Clone)]
struct CachedBinaryStl {
    positions: Arc<[Point3]>,
    triangles: Arc<[Triangle]>,
    bytes: Arc<[u8]>,
}

thread_local! {
    static BINARY_STL_CACHE: RefCell<Vec<CachedBinaryStl>> = const { RefCell::new(Vec::new()) };
}

fn triangle_points(mesh: &TriangleMesh, triangle: &Triangle) -> Result<[Point3; 3], IoError> {
    let [a, b, c] = triangle.indices();
    let point = |index| {
        mesh.positions
            .get(index)
            .cloned()
            .ok_or_else(|| IoError::Geometry {
                format: "STL",
                detail: format!("triangle references missing position {index}"),
            })
    };
    Ok([point(a)?, point(b)?, point(c)?])
}

fn triangle_normal(points: &[Point3; 3]) -> Result<[f64; 3], IoError> {
    finite_triangle_normal(points.each_ref(), "STL")
}

fn validate_binary_triangle_count(count: usize) -> Result<(), IoError> {
    u32::try_from(count)
        .map(|_| ())
        .map_err(|_| IoError::SizeOverflow {
            format: "STL",
            limit: "32-bit binary triangle count",
        })
}

/// Serialize native triangle geometry as ASCII STL.
pub fn to_stl_ascii(mesh: &TriangleMesh, name: &str) -> Result<String, IoError> {
    let name = single_line_metadata(name, "STL", "solid name")?;
    let mut output = format!("solid {name}\n");
    for triangle in mesh.triangles.iter() {
        let points = triangle_points(mesh, triangle)?;
        let normal = triangle_normal(&points)?;
        output.push_str(&format!(
            "  facet normal {:.17} {:.17} {:.17}\n",
            normal[0], normal[1], normal[2],
        ));
        output.push_str("    outer loop\n");
        for point in &points {
            output.push_str(&format!(
                "      vertex {:.17} {:.17} {:.17}\n",
                finite_f64(&point.x, "STL", "vertex x")?,
                finite_f64(&point.y, "STL", "vertex y")?,
                finite_f64(&point.z, "STL", "vertex z")?,
            ));
        }
        output.push_str("    endloop\n  endfacet\n");
    }
    output.push_str(&format!("endsolid {name}\n"));
    Ok(output)
}

/// Serialize native triangle geometry as binary STL.
pub fn to_stl_binary(mesh: &TriangleMesh, name: &str) -> Result<Vec<u8>, IoError> {
    use stl_io::{Normal, Triangle as StlTriangle, Vertex, write_stl};

    single_line_metadata(name, "STL", "solid name")?;
    validate_binary_triangle_count(mesh.triangles.len())?;
    if let Some(bytes) = BINARY_STL_CACHE.with_borrow(|entries| {
        entries
            .iter()
            .find(|entry| {
                Arc::ptr_eq(&entry.positions, &mesh.positions)
                    && Arc::ptr_eq(&entry.triangles, &mesh.triangles)
            })
            .map(|entry| entry.bytes.to_vec())
    }) {
        return Ok(bytes);
    }
    let triangles = mesh
        .triangles
        .iter()
        .map(|triangle| {
            let points = triangle_points(mesh, triangle)?;
            let normal = triangle_normal(&points)?;
            let point = |point: &Point3| {
                Ok::<_, IoError>(Vertex::new([
                    finite_f32(&point.x, "STL", "vertex x")?,
                    finite_f32(&point.y, "STL", "vertex y")?,
                    finite_f32(&point.z, "STL", "vertex z")?,
                ]))
            };
            Ok(StlTriangle {
                normal: Normal::new([normal[0] as f32, normal[1] as f32, normal[2] as f32]),
                vertices: [point(&points[0])?, point(&points[1])?, point(&points[2])?],
            })
        })
        .collect::<Result<Vec<_>, IoError>>()?;
    let mut cursor = Cursor::new(Vec::new());
    write_stl(&mut cursor, triangles.iter())?;
    let bytes = cursor.into_inner();
    BINARY_STL_CACHE.with_borrow_mut(|entries| {
        const CAPACITY: usize = 4;
        if entries.len() == CAPACITY {
            entries.remove(0);
        }
        entries.push(CachedBinaryStl {
            positions: Arc::clone(&mesh.positions),
            triangles: Arc::clone(&mesh.triangles),
            bytes: Arc::from(bytes.as_slice()),
        });
    });
    Ok(bytes)
}

/// Import validated STL triangles into the native Hypermesh carrier.
pub fn from_stl(data: &[u8]) -> Result<TriangleMesh, IoError> {
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
    let mut triangles = Vec::with_capacity(indexed.faces.len());
    for face in indexed.faces {
        let points = face.vertices.map(|index| positions[index].clone());
        triangle_normal(&points)?;
        triangles.push(Triangle::new(
            face.vertices[0],
            face.vertices[1],
            face.vertices[2],
        ));
    }
    Ok(TriangleMesh::new(positions, triangles))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn binary_cube_round_trips_through_validated_import() {
        let cube = crate::solid::cube(Real::from(2_u8));
        let bytes = to_stl_binary(&cube, "cube").unwrap();
        assert_eq!(from_stl(&bytes).unwrap().triangles.len(), 12);
    }

    #[test]
    fn sampled_sphere_is_representable_at_the_stl_boundary() {
        let sphere = crate::solid::sphere(Real::from(10_u8), 32, 16);
        let bytes = to_stl_binary(&sphere, "sphere").unwrap();
        assert_eq!(
            from_stl(&bytes).unwrap().triangles.len(),
            sphere.triangles.len()
        );
    }

    #[test]
    fn rejects_injected_names_and_degenerate_geometry() {
        let cube = crate::solid::cube(Real::one());
        assert!(to_stl_ascii(&cube, "safe\nendsolid forged").is_err());
        let point = Point3::origin();
        let degenerate = TriangleMesh::new(
            vec![point.clone(), point.clone(), point],
            vec![Triangle::new(0, 1, 2)],
        );
        assert!(to_stl_ascii(&degenerate, "degenerate").is_err());
        assert!(from_stl(b"not an STL").is_err());
    }
}
