mod support;

use csgrs::{
    float_types::EPSILON,
    mesh::{bsp::Node, plane::Plane, polygon::Polygon, vertex::Vertex},
};
use nalgebra::{Point3, Vector3};

use crate::support::approx_eq;

#[test]
fn new_and_build() {
    // A simple triangle:
    let p: Polygon<()> = Polygon::new(
        vec![
            Vertex::new(Point3::origin(), Vector3::z()),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
        ],
        None,
    );
    let node: Node<()> = Node::from_polygons(&[p.clone()]);
    // The node should have built a tree with plane = p.plane, polygons = [p], no front/back children
    assert!(node.plane.is_some());
    assert_eq!(node.polygons.len(), 1);
    assert!(node.front.is_none());
    assert!(node.back.is_none());
}

#[test]
fn invert() {
    let p: Polygon<()> = Polygon::new(
        vec![
            Vertex::new(Point3::origin(), Vector3::z()),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
        ],
        None,
    );
    let mut node: Node<()> = Node::from_polygons(&[p.clone()]);
    let original_count = node.polygons.len();
    let original_normal = node.plane.as_ref().unwrap().normal();
    node.invert();
    // The plane normal should be flipped, polygons should be flipped, and front/back swapped (they were None).
    let flipped_normal = node.plane.as_ref().unwrap().normal();
    assert!(approx_eq(flipped_normal.x, -original_normal.x, EPSILON));
    assert!(approx_eq(flipped_normal.y, -original_normal.y, EPSILON));
    assert!(approx_eq(flipped_normal.z, -original_normal.z, EPSILON));
    // We shouldn't lose polygons by inverting
    assert_eq!(node.polygons.len(), original_count);
    // If we invert back, we should get the same geometry
    node.invert();
    assert_eq!(node.polygons.len(), original_count);
}

#[test]
fn clip_polygons2() {
    // A node with a single plane normal to +Z, passing through z=0
    let plane = Plane::from_normal(Vector3::z(), 0.0);
    let mut node: Node<()> = Node {
        plane: Some(plane),
        front: None,
        back: None,
        polygons: Vec::new(),
    };
    // Build the node with some polygons
    // We'll put a polygon in the plane exactly (z=0) and one above, one below
    let poly_in_plane: Polygon<()> = Polygon::new(
        vec![
            Vertex::new(Point3::origin(), Vector3::z()),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
        ],
        None,
    );
    let poly_above: Polygon<()> = Polygon::new(
        vec![
            Vertex::new(Point3::new(0.0, 0.0, 1.0), Vector3::z()),
            Vertex::new(Point3::new(1.0, 0.0, 1.0), Vector3::z()),
            Vertex::new(Point3::new(0.0, 1.0, 1.0), Vector3::z()),
        ],
        None,
    );
    let poly_below: Polygon<()> = Polygon::new(
        vec![
            Vertex::new(Point3::new(0.0, 0.0, -1.0), Vector3::z()),
            Vertex::new(Point3::new(1.0, 0.0, -1.0), Vector3::z()),
            Vertex::new(Point3::new(0.0, 1.0, -1.0), Vector3::z()),
        ],
        None,
    );

    node.build(&[
        poly_in_plane.clone(),
        poly_above.clone(),
        poly_below.clone(),
    ]);
    // Now node has polygons: [poly_in_plane], front child with poly_above, back child with poly_below

    // Clip a polygon that crosses from z=-0.5 to z=0.5
    let crossing_poly: Polygon<()> = Polygon::new(
        vec![
            Vertex::new(Point3::new(-1.0, -1.0, -0.5), Vector3::z()),
            Vertex::new(Point3::new(2.0, -1.0, 0.5), Vector3::z()),
            Vertex::new(Point3::new(-1.0, 2.0, 0.5), Vector3::z()),
        ],
        None,
    );
    let clipped = node.clip_polygons(&[crossing_poly.clone()]);
    // The crossing polygon should be clipped against z=0 plane and any sub-planes from front/back nodes
    // For a single-plane node, we expect either one or two polygons left (front part & back part).
    // But we built subtrees, so let's just check if clipped is not empty.
    assert!(!clipped.is_empty());
}

#[test]
fn clip_to() {
    // Basic test: if we clip a node to another that encloses it fully, we keep everything
    let poly: Polygon<()> = Polygon::new(
        vec![
            Vertex::new(Point3::new(-0.5, -0.5, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.5, -0.5, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.0, 0.5, 0.0), Vector3::z()),
        ],
        None,
    );
    let mut node_a: Node<()> = Node::from_polygons(&[poly]);
    // Another polygon that fully encloses the above
    let big_poly: Polygon<()> = Polygon::new(
        vec![
            Vertex::new(Point3::new(-1.0, -1.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(1.0, -1.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(1.0, 1.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(-1.0, 1.0, 0.0), Vector3::z()),
        ],
        None,
    );
    let node_b: Node<()> = Node::from_polygons(&[big_poly]);
    node_a.clip_to(&node_b);
    // We expect nodeA's polygon to be present
    let all_a = node_a.all_polygons();
    assert_eq!(all_a.len(), 1);
}

#[test]
fn all_polygons() {
    // Build a node with multiple polygons
    let poly1: Polygon<()> = Polygon::new(
        vec![
            Vertex::new(Point3::origin(), Vector3::z()),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
        ],
        None,
    );
    let poly2: Polygon<()> = Polygon::new(
        vec![
            Vertex::new(Point3::new(0.0, 0.0, 1.0), Vector3::z()),
            Vertex::new(Point3::new(1.0, 0.0, 1.0), Vector3::z()),
            Vertex::new(Point3::new(0.0, 1.0, 1.0), Vector3::z()),
        ],
        None,
    );

    let node: Node<()> = Node::from_polygons(&[poly1.clone(), poly2.clone()]);
    let all_polys = node.all_polygons();
    // We expect to retrieve both polygons
    assert_eq!(all_polys.len(), 2);
}
