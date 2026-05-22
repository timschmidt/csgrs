//! Tests for polygon construction and manipulation.

use super::support::*;
use crate::polygon::build_orthonormal_basis;

#[test]
fn test_polygon_new() {
    let vertices = vec![
        Vertex::new(Point3::origin(), Vector3::z()),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
    ];
    let poly: Polygon<()> = Polygon::new(vertices.clone(), ());
    assert_eq!(poly.vertices.len(), 3);
    assert_eq!(poly.metadata, ());
    // Plane normal should be +Z for the above points
    assert!(approx_eq(poly.plane.normal().x, 0.0, tolerance()));
    assert!(approx_eq(poly.plane.normal().y, 0.0, tolerance()));
    assert!(approx_eq(poly.plane.normal().z, 1.0, tolerance()));
}

#[test]
fn test_polygon_flip() {
    let mut poly: Polygon<()> = Polygon::new(
        vec![
            Vertex::new(Point3::origin(), Vector3::z()),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
        ],
        (),
    );
    let plane_normal_before = poly.plane.normal();
    poly.flip();
    // The vertices should be reversed, and normal flipped
    assert_eq!(poly.vertices.len(), 3);
    assert!(approx_eq(
        poly.plane.normal().x,
        -plane_normal_before.x,
        tolerance()
    ));
    assert!(approx_eq(
        poly.plane.normal().y,
        -plane_normal_before.y,
        tolerance()
    ));
    assert!(approx_eq(
        poly.plane.normal().z,
        -plane_normal_before.z,
        tolerance()
    ));
}

#[test]
fn test_polygon_triangulate() {
    // A quad:
    let poly: Polygon<()> = Polygon::new(
        vec![
            Vertex::new(Point3::origin(), Vector3::z()),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(1.0, 1.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
        ],
        (),
    );
    let triangles = poly.triangulate();
    // We expect 2 triangles from a quad
    assert_eq!(
        triangles.len(),
        2,
        "A quad should triangulate into 2 triangles"
    );
}

#[test]
fn test_polygon_subdivide_triangles() {
    // A single triangle (level=1 should produce 4 sub-triangles)
    let poly: Polygon<()> = Polygon::new(
        vec![
            Vertex::new(Point3::origin(), Vector3::z()),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
        ],
        (),
    );
    let subs = poly.subdivide_triangles(1.try_into().expect("not 0"));
    // One triangle subdivided once => 4 smaller triangles
    assert_eq!(subs.len(), 4);

    // If we subdivide the same single tri 2 levels, we expect 16 sub-triangles.
    let subs2 = poly.subdivide_triangles(2.try_into().expect("not 0"));
    assert_eq!(subs2.len(), 16);
}

#[test]
fn test_polygon_recalc_plane_and_normals() {
    let mut poly: Polygon<()> = Polygon::new(
        vec![
            Vertex::new(Point3::origin(), Vector3::zeros()),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::zeros()),
            Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::zeros()),
        ],
        (),
    );
    poly.set_new_normal();
    assert!(approx_eq(poly.plane.normal().z, 1.0, tolerance()));
    for v in &poly.vertices {
        assert!(approx_eq(v.normal.x, 0.0, tolerance()));
        assert!(approx_eq(v.normal.y, 0.0, tolerance()));
        assert!(approx_eq(v.normal.z, 1.0, tolerance()));
    }
}

#[test]
fn polygon_newell_normal_and_basis_use_hyperreal_checked_normalization() {
    let mut poly: Polygon<()> = Polygon::new(
        vec![
            Vertex::new(Point3::new(0.0, 0.0, 2.0), Vector3::zeros()),
            Vertex::new(Point3::new(3.0, 0.0, 2.0), Vector3::zeros()),
            Vertex::new(Point3::new(3.0, 4.0, 2.0), Vector3::zeros()),
            Vertex::new(Point3::new(0.0, 4.0, 2.0), Vector3::zeros()),
        ],
        (),
    );
    poly.set_new_normal();

    for vertex in &poly.vertices {
        assert!((vertex.normal.norm() - 1.0).abs() < tolerance());
        assert!(vertex.normal.dot(&Vector3::z()) > 1.0 - tolerance());
    }

    let (u, v) = build_orthonormal_basis(Vector3::new(2.0, 3.0, 6.0));
    let n = Vector3::new(2.0, 3.0, 6.0).normalize();
    assert!((u.norm() - 1.0).abs() < tolerance());
    assert!((v.norm() - 1.0).abs() < tolerance());
    assert!(u.dot(&v).abs() < tolerance());
    assert!(u.dot(&n).abs() < tolerance());
    assert!(v.dot(&n).abs() < tolerance());
}

// ------------------------------------------------------------
// Plane split tests
// ------------------------------------------------------------
