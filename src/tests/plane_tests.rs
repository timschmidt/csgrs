//! Tests for plane classification and splitting.

use super::support::*;
use crate::polygon_mesh::Polygon as PlanarPolygon;

#[test]
fn test_plane_flip() {
    let mut plane = Plane::from_normal(Vector3::y(), r(2.0));
    plane.flip();
    assert_eq!(plane.normal(), v3(0.0, -1.0, 0.0));
    assert_eq!(plane.offset(), r(-2.0));
}

#[test]
fn test_plane_split_polygon() {
    // Define a plane that splits the XY plane at y=0
    let plane = Plane::from_normal(v3(0.0, 1.0, 0.0), r(0.0));

    // A polygon that crosses y=0 line: a square from ( -1, -1 ) to (1, 1 )
    let poly: PlanarPolygon<()> = PlanarPolygon::new(
        vec![
            Vertex::new(p3(-1.0, -1.0, 0.0), Vector3::z()),
            Vertex::new(p3(1.0, -1.0, 0.0), Vector3::z()),
            Vertex::new(p3(1.0, 1.0, 0.0), Vector3::z()),
            Vertex::new(p3(-1.0, 1.0, 0.0), Vector3::z()),
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
    assert_eq!(front_poly.plane_id, poly.plane_id);
    assert_eq!(back_poly.plane_id, poly.plane_id);
    assert!(front_poly.vertices().len() >= 3);
    assert!(back_poly.vertices().len() >= 3);

    // Quick check: all front vertices should have y >= 0.
    for v in front_poly.vertices() {
        assert!(v.position.y >= r(0.0));
    }
    // All back vertices should have y <= 0.
    for v in back_poly.vertices() {
        assert!(v.position.y <= r(0.0));
    }
}

#[test]
fn test_plane_orient_point_uses_exact_hyperreal_sign() {
    let plane = Plane::from_normal(Vector3::y(), r(0.0));
    let on_plane = p3(0.0, 0.0, 0.0);
    let near_front = p3(0.0, tolerance() * 0.25, 0.0);
    let front = p3(0.0, tolerance() * 2.0, 0.0);
    let back = p3(0.0, -tolerance() * 2.0, 0.0);

    assert_eq!(plane.orient_point(&on_plane), crate::mesh::plane::COPLANAR);
    assert_eq!(plane.orient_point(&near_front), crate::mesh::plane::FRONT);
    assert_eq!(plane.orient_point(&front), crate::mesh::plane::FRONT);
    assert_eq!(plane.orient_point(&back), crate::mesh::plane::BACK);
}

#[test]
fn plane_from_normal_uses_hyperreal_scale_and_rejects_invalid_normals() {
    let plane = Plane::from_normal(v3(2.0, 0.0, 0.0), r(4.0));

    assert!((plane.offset() - r(2.0)).abs() < tolerance());
    assert_eq!(
        plane.orient_point(&p3(2.0, 0.0, 0.0)),
        crate::mesh::plane::COPLANAR
    );
    assert_eq!(
        plane.orient_point(&p3(2.0 + tolerance() * 0.25, 0.0, 0.0)),
        crate::mesh::plane::FRONT
    );

    assert!(Plane::try_from_normal(Vector3::zeros(), r(0.0)).is_none());
    assert_eq!(
        Plane::from_normal(Vector3::zeros(), r(0.0)).normal(),
        Vector3::z()
    );
    let tiny_normal_plane =
        Plane::try_from_normal(v3(tolerance() * r(0.25), 0.0, 0.0), r(0.0))
            .expect("nonzero normals are valid under exact hyperreal predicates");
    assert_eq!(
        tiny_normal_plane.orient_point(&p3(tolerance() * 0.25, 0.0, 0.0)),
        crate::mesh::plane::FRONT
    );
}

#[test]
fn plane_normal_uses_hyperreal_cross_product_and_filters_degenerate_support() {
    let plane = Plane {
        point_a: p3(2.0, 3.0, 5.0),
        point_b: p3(8.0, 3.0, 5.0),
        point_c: p3(2.0, 9.0, 5.0),
    };

    let normal = plane.normal();
    assert!((normal.norm() - r(1.0)).abs() < tolerance());
    assert!(normal.dot(&Vector3::z()) > r(1.0) - tolerance());

    let degenerate = Plane {
        point_a: p3(1.0, 1.0, 1.0),
        point_b: p3(2.0, 2.0, 2.0),
        point_c: p3(3.0, 3.0, 3.0),
    };
    assert_eq!(degenerate.normal(), Vector3::zeros());
}

#[test]
fn plane_to_xy_transform_fails_closed_for_degenerate_normals() {
    let degenerate = Plane {
        point_a: p3(1.0, 1.0, 1.0),
        point_b: p3(2.0, 2.0, 2.0),
        point_c: p3(3.0, 3.0, 3.0),
    };
    let (to_xy, from_xy) = degenerate.to_xy_transform();
    assert_eq!(to_xy, Matrix4::identity());
    assert_eq!(from_xy, Matrix4::identity());
}

#[test]
fn plane_from_vertices_uses_hyperreal_support_triangle_ranking() {
    let vertices = vec![
        Vertex::new(p3(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(p3(1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(p3(2.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(p3(2.0, 3.0, 0.0), Vector3::z()),
        Vertex::new(p3(0.0, 3.0, 0.0), Vector3::z()),
    ];

    let plane = Plane::from_vertices(&vertices);
    let normal = plane.normal();

    assert!((normal.norm() - r(1.0)).abs() < tolerance());
    assert!(normal.dot(&Vector3::z()) > r(1.0) - tolerance());
    for vertex in &vertices {
        assert_eq!(
            plane.orient_point(&vertex.position),
            crate::mesh::plane::COPLANAR
        );
    }
}

// ------------------------------------------------------------
// Polygon tests
// ------------------------------------------------------------
