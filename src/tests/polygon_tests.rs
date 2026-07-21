//! Tests for polygon construction and manipulation.

use super::support::*;

#[test]
fn test_polygon_new() {
    let vertices = vec![
        Vertex::new(Point3::origin(), Vector3::z()),
        Vertex::new(p3(1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(p3(0.0, 1.0, 0.0), Vector3::z()),
    ];
    let poly: Polygon<()> = Polygon::from_planar_vertices(vertices.clone(), ());
    assert_eq!(poly.vertices.len(), 3);
    assert_eq!(poly.metadata, ());
    // Plane normal should be +Z for the above points
    let normal = poly.plane.normal();
    assert!(approx_eq(&normal.0[0], 0.0, tolerance()));
    assert!(approx_eq(&normal.0[1], 0.0, tolerance()));
    assert!(approx_eq(&normal.0[2], 1.0, tolerance()));
}

#[test]
fn test_polygon_flip() {
    let mut poly: Polygon<()> = Polygon::from_planar_vertices(
        vec![
            Vertex::new(Point3::origin(), Vector3::z()),
            Vertex::new(p3(1.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(p3(0.0, 1.0, 0.0), Vector3::z()),
        ],
        (),
    );
    let plane_normal_before = poly.plane.normal();
    poly.flip();
    // The vertices should be reversed, and normal flipped
    assert_eq!(poly.vertices.len(), 3);
    let plane_normal_after = poly.plane.normal();
    assert!(approx_eq(
        &plane_normal_after.0[0],
        -plane_normal_before.0[0].clone(),
        tolerance()
    ));
    assert!(approx_eq(
        &plane_normal_after.0[1],
        -plane_normal_before.0[1].clone(),
        tolerance()
    ));
    assert!(approx_eq(
        &plane_normal_after.0[2],
        -plane_normal_before.0[2].clone(),
        tolerance()
    ));
}

#[test]
fn test_polygon_triangulate() {
    // A quad:
    let poly: Polygon<()> = Polygon::from_planar_vertices(
        vec![
            Vertex::new(Point3::origin(), Vector3::z()),
            Vertex::new(p3(1.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(p3(1.0, 1.0, 0.0), Vector3::z()),
            Vertex::new(p3(0.0, 1.0, 0.0), Vector3::z()),
        ],
        (),
    );
    let triangles = poly.triangulate();
    let triangle_indices = poly.triangulate_indices();
    // We expect 2 triangles from a quad
    assert_eq!(
        triangles.len(),
        2,
        "A quad should triangulate into 2 triangles"
    );
    assert_eq!(triangle_indices.len(), triangles.len());
    assert_eq!(triangle_indices, [[0, 1, 2], [0, 2, 3]]);
    for (indices, triangle) in triangle_indices.iter().zip(&triangles) {
        for (index, vertex) in indices.iter().zip(triangle) {
            assert_eq!(poly.vertices()[*index], *vertex);
        }
    }
}

#[test]
fn test_polygon_subdivide_triangles() {
    // A single triangle (level=1 should produce 4 sub-triangles)
    let poly: Polygon<()> = Polygon::from_planar_vertices(
        vec![
            Vertex::new(Point3::origin(), Vector3::z()),
            Vertex::new(p3(1.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(p3(0.0, 1.0, 0.0), Vector3::z()),
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
    let mut poly: Polygon<()> = Polygon::from_planar_vertices(
        vec![
            Vertex::new(Point3::origin(), Vector3::zeros()),
            Vertex::new(p3(1.0, 0.0, 0.0), Vector3::zeros()),
            Vertex::new(p3(0.0, 1.0, 0.0), Vector3::zeros()),
        ],
        (),
    );
    poly.set_new_normal();
    assert!(approx_eq(&poly.plane.normal().0[2], 1.0, tolerance()));
    for v in &poly.vertices {
        assert!(approx_eq(&v.normal.0[0], 0.0, tolerance()));
        assert!(approx_eq(&v.normal.0[1], 0.0, tolerance()));
        assert!(approx_eq(&v.normal.0[2], 1.0, tolerance()));
    }
}

#[test]
fn polygon_newell_normal_and_basis_use_hyperreal_checked_normalization() {
    let mut poly: Polygon<()> = Polygon::from_planar_vertices(
        vec![
            Vertex::new(p3(0.0, 0.0, 2.0), Vector3::zeros()),
            Vertex::new(p3(3.0, 0.0, 2.0), Vector3::zeros()),
            Vertex::new(p3(3.0, 4.0, 2.0), Vector3::zeros()),
            Vertex::new(p3(0.0, 4.0, 2.0), Vector3::zeros()),
        ],
        (),
    );
    poly.set_new_normal();

    for vertex in &poly.vertices {
        assert!((vertex.normal.norm() - r(1.0)).abs() < tolerance());
        assert!(vertex.normal.dot(&Vector3::z()) > r(1.0) - tolerance());
    }

    let n = v3(2.0, 3.0, 6.0).normalize().unwrap();
    let (u, v) = n.orthonormal_basis_checked().unwrap();
    assert!((u.norm() - r(1.0)).abs() < tolerance());
    assert!((v.norm() - r(1.0)).abs() < tolerance());
    assert!(u.dot(&v).abs() < tolerance());
    assert!(u.dot(&n).abs() < tolerance());
    assert!(v.dot(&n).abs() < tolerance());
}

#[test]
fn triangulation_preserves_exact_coordinates_beyond_f64_resolution() {
    let base = Real::from(1_i64 << 60);
    let one = Real::one();
    let zero = Real::zero();
    let point = |x: Real, y: Real| Point3::new(x, y, zero.clone());
    let polygon = Polygon::from_planar_vertices(
        vec![
            Vertex::new(point(base.clone(), zero.clone()), Vector3::z()),
            Vertex::new(point(base.clone() + one.clone(), zero.clone()), Vector3::z()),
            Vertex::new(point(base.clone() + one.clone(), one.clone()), Vector3::z()),
            Vertex::new(point(base, one), Vector3::z()),
        ],
        (),
    );

    assert_eq!(polygon.triangulate().len(), 2);
}

#[test]
fn mutable_vertex_access_refreshes_plane_and_bounds() {
    let mut polygon = Polygon::from_planar_vertices(
        vec![
            Vertex::new(Point3::origin(), Vector3::z()),
            Vertex::new(p3(1.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(p3(0.0, 1.0, 0.0), Vector3::z()),
        ],
        (),
    );
    let old_bounds = polygon.bounding_box();
    let old_position_ids = polygon
        .vertices()
        .iter()
        .map(|vertex| vertex.position_id)
        .collect::<Vec<_>>();

    {
        let mut vertices = polygon.vertices_mut();
        for vertex in vertices.iter_mut() {
            vertex.position.z += r(2.0);
        }
    }

    assert_eq!(old_bounds.mins.z, Real::zero());
    assert_eq!(polygon.bounding_box().mins.z, r(2.0));
    assert_eq!(polygon.plane().point_a.z, r(2.0));
    assert!(
        polygon
            .vertices()
            .iter()
            .zip(old_position_ids)
            .all(|(vertex, old_id)| vertex.position_id != old_id)
    );
}

// ------------------------------------------------------------
// Plane split tests
// ------------------------------------------------------------
