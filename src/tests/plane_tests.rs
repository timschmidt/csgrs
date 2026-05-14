//! Tests for plane classification and splitting.

use super::support::*;

#[test]
fn test_plane_flip() {
    let mut plane = Plane::from_normal(Vector3::y(), 2.0);
    plane.flip();
    assert_eq!(plane.normal(), Vector3::new(0.0, -1.0, 0.0));
    assert_eq!(plane.offset(), -2.0);
}

#[test]
fn test_plane_split_polygon() {
    // Define a plane that splits the XY plane at y=0
    let plane = Plane::from_normal(Vector3::new(0.0, 1.0, 0.0), 0.0);

    // A polygon that crosses y=0 line: a square from ( -1, -1 ) to (1, 1 )
    let poly: Polygon<()> = Polygon::new(
        vec![
            Vertex::new(Point3::new(-1.0, -1.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(1.0, -1.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(1.0, 1.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(-1.0, 1.0, 0.0), Vector3::z()),
        ],
        (),
    );

    let (cf, cb, f, b) = plane.split_polygon(&poly);
    // This polygon is spanning across y=0 plane => we expect no coplanar, but front/back polygons.
    assert_eq!(cf.len(), 0);
    assert_eq!(cb.len(), 0);
    assert_eq!(f.len(), 1);
    assert_eq!(b.len(), 1);

    // Check that each part has at least 3 vertices and is "above" or "below" the plane
    // in rough terms
    let front_poly = &f[0];
    let back_poly = &b[0];
    assert!(front_poly.vertices.len() >= 3);
    assert!(back_poly.vertices.len() >= 3);

    // Quick check: all front vertices should have y >= 0 (within a tolerance).
    for v in &front_poly.vertices {
        assert!(v.position.y >= -tolerance());
    }
    // All back vertices should have y <= 0 (within a tolerance).
    for v in &back_poly.vertices {
        assert!(v.position.y <= tolerance());
    }
}

// ------------------------------------------------------------
// Polygon tests
// ------------------------------------------------------------
