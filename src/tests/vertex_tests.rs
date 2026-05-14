//! Tests for vertex interpolation and export helpers.

use super::support::*;

#[test]
fn test_vertex_flip() {
    let mut v = Vertex::new(Point3::new(1.0, 2.0, 3.0), Vector3::x());
    v.flip();
    // Position remains the same
    assert_eq!(v.position, Point3::new(1.0, 2.0, 3.0));
    // Normal should be negated
    assert_eq!(v.normal, -Vector3::x());
}

// --------------------------------------------------------
//   Polygon Tests
// --------------------------------------------------------

#[test]
fn test_polygon_construction() {
    let v1 = Vertex::new(Point3::origin(), Vector3::y());
    let v2 = Vertex::new(Point3::new(1.0, 0.0, 1.0), Vector3::y());
    let v3 = Vertex::new(Point3::new(1.0, 0.0, -1.0), Vector3::y());

    let poly: Polygon<()> = Polygon::new(vec![v1, v2, v3], None);
    assert_eq!(poly.vertices.len(), 3);
    // Plane should be defined by these three points. We expect a normal near ±Y.
    assert!(
        approx_eq(poly.plane.normal().dot(&Vector3::y()).abs(), 1.0, 1e-8),
        "Expected plane normal to match ±Y"
    );
}

// --------------------------------------------------------
//   CSG: STL Export
// --------------------------------------------------------

#[test]
#[cfg(feature = "stl-io")]
fn test_to_stl_ascii() {
    let cube: Mesh<()> = Mesh::cube(2.0, None);
    let stl_str = cube.to_stl_ascii("test_cube");
    // Basic checks
    assert!(stl_str.contains("solid test_cube"));
    assert!(stl_str.contains("endsolid test_cube"));

    // Should contain some facet normals
    assert!(stl_str.contains("facet normal"));
    // Should contain some vertex lines
    assert!(stl_str.contains("vertex"));
}

// --------------------------------------------------------
//   Node & Clipping Tests
//   (Optional: these get more into internal details)
// --------------------------------------------------------

#[test]
fn test_degenerate_polygon_after_clipping() {
    let vertices = vec![
        Vertex::new(Point3::origin(), Vector3::y()),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::y()),
        Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::y()),
    ];

    let polygon: Polygon<()> = Polygon::new(vertices.clone(), None);
    let plane = Plane::from_normal(Vector3::new(0.0, 0.0, 1.0), 0.0);

    eprintln!("Original polygon: {:?}", polygon);
    eprintln!("Clipping plane: {:?}", plane);

    let (_coplanar_front, _coplanar_back, front, back) = plane.split_polygon(&polygon);

    eprintln!("Front polygons: {:?}", front);
    eprintln!("Back polygons: {:?}", back);

    assert!(front.is_empty(), "Front should be empty for this test");
    assert!(back.is_empty(), "Back should be empty for this test");
}

#[test]
fn test_valid_polygon_clipping() {
    let vertices = vec![
        Vertex::new(Point3::origin(), Vector3::y()),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::y()),
        Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::y()),
    ];

    let polygon: Polygon<()> = Polygon::new(vertices, None);

    let plane = Plane::from_normal(-Vector3::y(), -0.5);

    eprintln!("Polygon before clipping: {:?}", polygon);
    eprintln!("Clipping plane: {:?}", plane);

    let (_coplanar_front, _coplanar_back, front, back) = plane.split_polygon(&polygon);

    eprintln!("Front polygons: {:?}", front);
    eprintln!("Back polygons: {:?}", back);

    assert!(!front.is_empty(), "Front should not be empty");
    assert!(!back.is_empty(), "Back should not be empty");
}

// ------------------------------------------------------------
// Vertex tests
// ------------------------------------------------------------
#[test]
fn test_vertex_new() {
    let pos = Point3::new(1.0, 2.0, 3.0);
    let normal = Vector3::new(0.0, 1.0, 0.0);
    let v = Vertex::new(pos, normal);
    assert_eq!(v.position, pos);
    assert_eq!(v.normal, normal);
}

#[test]
fn test_vertex_interpolate() {
    let v1 = Vertex::new(Point3::origin(), Vector3::x());
    let v2 = Vertex::new(Point3::new(2.0, 2.0, 2.0), Vector3::y());
    let v_mid = v1.interpolate(&v2, 0.5);
    assert!(approx_eq(v_mid.position.x, 1.0, tolerance()));
    assert!(approx_eq(v_mid.position.y, 1.0, tolerance()));
    assert!(approx_eq(v_mid.position.z, 1.0, tolerance()));
    assert!(approx_eq(v_mid.normal.x, 0.5, tolerance()));
    assert!(approx_eq(v_mid.normal.y, 0.5, tolerance()));
    assert!(approx_eq(v_mid.normal.z, 0.0, tolerance()));
}

// ------------------------------------------------------------
// Plane tests
