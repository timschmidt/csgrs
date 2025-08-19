use csgrs::{
    float_types::EPSILON,
    mesh::{polygon::Polygon, vertex::Vertex},
};
use nalgebra::{Point3, Vector3};

mod support;

use crate::support::approx_eq;

#[test]
fn construction() {
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

#[test]
fn new() {
    let vertices = vec![
        Vertex::new(Point3::origin(), Vector3::z()),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
    ];
    let poly: Polygon<()> = Polygon::new(vertices.clone(), None);
    assert_eq!(poly.vertices.len(), 3);
    assert_eq!(poly.metadata, None);
    // Plane normal should be +Z for the above points
    assert!(approx_eq(poly.plane.normal().x, 0.0, EPSILON));
    assert!(approx_eq(poly.plane.normal().y, 0.0, EPSILON));
    assert!(approx_eq(poly.plane.normal().z, 1.0, EPSILON));
}

#[test]
fn flip() {
    let mut poly: Polygon<()> = Polygon::new(
        vec![
            Vertex::new(Point3::origin(), Vector3::z()),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
        ],
        None,
    );
    let plane_normal_before = poly.plane.normal();
    poly.flip();
    // The vertices should be reversed, and normal flipped
    assert_eq!(poly.vertices.len(), 3);
    assert!(approx_eq(
        poly.plane.normal().x,
        -plane_normal_before.x,
        EPSILON
    ));
    assert!(approx_eq(
        poly.plane.normal().y,
        -plane_normal_before.y,
        EPSILON
    ));
    assert!(approx_eq(
        poly.plane.normal().z,
        -plane_normal_before.z,
        EPSILON
    ));
}

#[test]
fn triangulate() {
    // A quad:
    let poly: Polygon<()> = Polygon::new(
        vec![
            Vertex::new(Point3::origin(), Vector3::z()),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(1.0, 1.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
        ],
        None,
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
fn subdivide_triangles() {
    // A single triangle (level=1 should produce 4 sub-triangles)
    let poly: Polygon<()> = Polygon::new(
        vec![
            Vertex::new(Point3::origin(), Vector3::z()),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
        ],
        None,
    );
    let subs = poly.subdivide_triangles(1.try_into().expect("not 0"));
    // One triangle subdivided once => 4 smaller triangles
    assert_eq!(subs.len(), 4);

    // If we subdivide the same single tri 2 levels, we expect 16 sub-triangles.
    let subs2 = poly.subdivide_triangles(2.try_into().expect("not 0"));
    assert_eq!(subs2.len(), 16);
}

#[test]
fn recalc_plane_and_normals() {
    let mut poly: Polygon<()> = Polygon::new(
        vec![
            Vertex::new(Point3::origin(), Vector3::zeros()),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::zeros()),
            Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::zeros()),
        ],
        None,
    );
    poly.set_new_normal();
    assert!(approx_eq(poly.plane.normal().z, 1.0, EPSILON));
    for v in &poly.vertices {
        assert!(approx_eq(v.normal.x, 0.0, EPSILON));
        assert!(approx_eq(v.normal.y, 0.0, EPSILON));
        assert!(approx_eq(v.normal.z, 1.0, EPSILON));
    }
}
