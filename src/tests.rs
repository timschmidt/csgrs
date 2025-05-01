use crate::bsp::Node;
use crate::csg::CSG;
use crate::float_types::{EPSILON, FRAC_PI_2, Real};
use crate::plane::Plane;
use crate::polygon::Polygon;
use crate::vertex::Vertex;
use nalgebra::{Point3, Vector3};

// --------------------------------------------------------
//   Helpers
// --------------------------------------------------------

/// Returns the approximate bounding box `[min_x, min_y, min_z, max_x, max_y, max_z]`
/// for a set of polygons.
fn bounding_box(polygons: &[Polygon<()>]) -> [Real; 6] {
    let mut min_x = Real::MAX;
    let mut min_y = Real::MAX;
    let mut min_z = Real::MAX;
    let mut max_x = Real::MIN;
    let mut max_y = Real::MIN;
    let mut max_z = Real::MIN;

    for poly in polygons {
        for v in &poly.vertices {
            let p = v.pos;
            if p.x < min_x {
                min_x = p.x;
            }
            if p.y < min_y {
                min_y = p.y;
            }
            if p.z < min_z {
                min_z = p.z;
            }
            if p.x > max_x {
                max_x = p.x;
            }
            if p.y > max_y {
                max_y = p.y;
            }
            if p.z > max_z {
                max_z = p.z;
            }
        }
    }

    [min_x, min_y, min_z, max_x, max_y, max_z]
}

/// Quick helper to compare floating-point results with an acceptable tolerance.
fn approx_eq(a: Real, b: Real, eps: Real) -> bool {
    (a - b).abs() < eps
}

// --------------------------------------------------------
//   Vertex Tests
// --------------------------------------------------------

#[test]
fn test_vertex_flip() {
    let mut v = Vertex::new(Point3::new(1.0, 2.0, 3.0), Vector3::x());
    v.flip();
    // Position remains the same
    assert_eq!(v.pos, Point3::new(1.0, 2.0, 3.0));
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

    let poly: Polygon<()> = Polygon::new(vec![v1.clone(), v2.clone(), v3.clone()], None);
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
fn test_to_stl_ascii() {
    let cube: CSG<()> = CSG::cube(2.0, 2.0, 2.0, None);
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
    assert_eq!(v.pos, pos);
    assert_eq!(v.normal, normal);
}

#[test]
fn test_vertex_interpolate() {
    let v1 = Vertex::new(Point3::origin(), Vector3::x());
    let v2 = Vertex::new(Point3::new(2.0, 2.0, 2.0), Vector3::y());
    let v_mid = v1.interpolate(&v2, 0.5);
    assert!(approx_eq(v_mid.pos.x, 1.0, EPSILON));
    assert!(approx_eq(v_mid.pos.y, 1.0, EPSILON));
    assert!(approx_eq(v_mid.pos.z, 1.0, EPSILON));
    assert!(approx_eq(v_mid.normal.x, 0.5, EPSILON));
    assert!(approx_eq(v_mid.normal.y, 0.5, EPSILON));
    assert!(approx_eq(v_mid.normal.z, 0.0, EPSILON));
}

// ------------------------------------------------------------
// Plane tests
// ------------------------------------------------------------
#[test]
fn test_plane_from_points() {
    let a = Point3::origin();
    let b = Point3::new(1.0, 0.0, 0.0);
    let c = Point3::new(0.0, 1.0, 0.0);
    let plane = Plane::from_points(&a, &b, &c);
    assert!(approx_eq(plane.normal().x, 0.0, EPSILON));
    assert!(approx_eq(plane.normal().y, 0.0, EPSILON));
    assert!(approx_eq(plane.normal().z, 1.0, EPSILON));
    assert!(approx_eq(plane.offset(), 0.0, EPSILON));
}

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
        None,
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

    // Quick check: all front vertices should have y >= 0 (within an epsilon).
    for v in &front_poly.vertices {
        assert!(v.pos.y >= -EPSILON);
    }
    // All back vertices should have y <= 0 (within an epsilon).
    for v in &back_poly.vertices {
        assert!(v.pos.y <= EPSILON);
    }
}

// ------------------------------------------------------------
// Polygon tests
// ------------------------------------------------------------
#[test]
fn test_polygon_new() {
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
fn test_polygon_flip() {
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
fn test_polygon_triangulate() {
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
    let triangles = poly.tessellate();
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
        None,
    );
    let subs = poly.subdivide_triangles(1);
    // One triangle subdivided once => 4 smaller triangles
    assert_eq!(subs.len(), 4);

    // If we subdivide the same single tri 2 levels, we expect 16 sub-triangles.
    let subs2 = poly.subdivide_triangles(2);
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

// ------------------------------------------------------------
// Node tests
// ------------------------------------------------------------
#[test]
fn test_node_new_and_build() {
    // A simple triangle:
    let p: Polygon<()> = Polygon::new(
        vec![
            Vertex::new(Point3::origin(), Vector3::z()),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
        ],
        None,
    );
    let node: Node<()> = Node::new(&[p.clone()]);
    // The node should have built a tree with plane = p.plane, polygons = [p], no front/back children
    assert!(node.plane.is_some());
    assert_eq!(node.polygons.len(), 1);
    assert!(node.front.is_none());
    assert!(node.back.is_none());
}

#[test]
fn test_node_invert() {
    let p: Polygon<()> = Polygon::new(
        vec![
            Vertex::new(Point3::origin(), Vector3::z()),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
        ],
        None,
    );
    let mut node: Node<()> = Node::new(&[p.clone()]);
    let original_count = node.polygons.len();
    let original_normal = node.plane.as_ref().unwrap().normal();
    node.invert();
    // The plane normal should be flipped, polygons should be flipped, and front/back swapped (they were None).
    let flipped_normal = node.plane.as_ref().unwrap().normal();
    assert!(approx_eq(flipped_normal.x, -original_normal.x, EPSILON));
    assert!(approx_eq(flipped_normal.y, -original_normal.y, EPSILON));
    assert!(approx_eq(flipped_normal.z, -original_normal.z, EPSILON));
    // We shouldn’t lose polygons by inverting
    assert_eq!(node.polygons.len(), original_count);
    // If we invert back, we should get the same geometry
    node.invert();
    assert_eq!(node.polygons.len(), original_count);
}

#[test]
fn test_node_clip_polygons2() {
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
fn test_node_clip_to() {
    // Basic test: if we clip a node to another that encloses it fully, we keep everything
    let poly: Polygon<()> = Polygon::new(
        vec![
            Vertex::new(Point3::new(-0.5, -0.5, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.5, -0.5, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.0, 0.5, 0.0), Vector3::z()),
        ],
        None,
    );
    let mut node_a: Node<()> = Node::new(&[poly]);
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
    let node_b: Node<()> = Node::new(&[big_poly]);
    node_a.clip_to(&node_b);
    // We expect nodeA's polygon to be present
    let all_a = node_a.all_polygons();
    assert_eq!(all_a.len(), 1);
}

#[test]
fn test_node_all_polygons() {
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

    let node: Node<()> = Node::new(&[poly1.clone(), poly2.clone()]);
    let all_polys = node.all_polygons();
    // We expect to retrieve both polygons
    assert_eq!(all_polys.len(), 2);
}

// ------------------------------------------------------------
// CSG tests
// ------------------------------------------------------------
#[test]
fn test_csg_from_polygons_and_to_polygons() {
    let poly: Polygon<()> = Polygon::new(
        vec![
            Vertex::new(Point3::origin(), Vector3::z()),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
        ],
        None,
    );
    let csg: CSG<()> = CSG::from_polygons(&[poly.clone()]);
    let polys = csg.to_polygons();
    assert_eq!(polys.len(), 1);
    assert_eq!(polys[0].vertices.len(), 3);
}

#[test]
fn test_csg_union() {
    let cube1: CSG<()> = CSG::cube(2.0, 2.0, 2.0, None).translate(-1.0, -1.0, -1.0); // from -1 to +1 in all coords
    let cube2: CSG<()> = CSG::cube(1.0, 1.0, 1.0, None).translate(0.5, 0.5, 0.5);

    let union_csg = cube1.union(&cube2);
    let polys = union_csg.to_polygons();
    assert!(
        !polys.is_empty(),
        "Union of two cubes should produce polygons"
    );

    // Check bounding box => should now at least range from -1 to (0.5+1) = 1.5
    let bb = bounding_box(&polys);
    assert!(approx_eq(bb[0], -1.0, 1e-8));
    assert!(approx_eq(bb[1], -1.0, 1e-8));
    assert!(approx_eq(bb[2], -1.0, 1e-8));
    assert!(approx_eq(bb[3], 1.5, 1e-8));
    assert!(approx_eq(bb[4], 1.5, 1e-8));
    assert!(approx_eq(bb[5], 1.5, 1e-8));
}

#[test]
fn test_csg_difference() {
    // Subtract a smaller cube from a bigger one
    let big_cube: CSG<()> = CSG::cube(4.0, 4.0, 4.0, None).translate(-2.0, -2.0, -2.0); // radius=2 => spans [-2,2]
    let small_cube: CSG<()> = CSG::cube(2.0, 2.0, 2.0, None).translate(-1.0, -1.0, -1.0); // radius=1 => spans [-1,1]

    let result = big_cube.difference(&small_cube);
    let polys = result.to_polygons();
    assert!(
        !polys.is_empty(),
        "Subtracting a smaller cube should leave polygons"
    );

    // Check bounding box => should still be [-2,-2,-2, 2,2,2], but with a chunk removed
    let bb = bounding_box(&polys);
    // At least the bounding box remains the same
    assert!(approx_eq(bb[0], -2.0, 1e-8));
    assert!(approx_eq(bb[3], 2.0, 1e-8));
}

#[test]
fn test_csg_union2() {
    let c1: CSG<()> = CSG::cube(2.0, 2.0, 2.0, None); // cube from (-1..+1) if that's how you set radius=1 by default
    let c2: CSG<()> = CSG::sphere(1.0, 16, 8, None); // default sphere radius=1
    let unioned = c1.union(&c2);
    // We can check bounding box is bigger or at least not smaller than either shape’s box
    let bb_union = unioned.bounding_box();
    let bb_cube = c1.bounding_box();
    let bb_sphere = c2.bounding_box();
    assert!(bb_union.mins.x <= bb_cube.mins.x.min(bb_sphere.mins.x));
    assert!(bb_union.maxs.x >= bb_cube.maxs.x.max(bb_sphere.maxs.x));
}

#[test]
fn test_csg_intersect() {
    let c1: CSG<()> = CSG::cube(2.0, 2.0, 2.0, None);
    let c2: CSG<()> = CSG::sphere(1.0, 16, 8, None);
    let isect = c1.intersection(&c2);
    let bb_isect = isect.bounding_box();
    // The intersection bounding box should be smaller than or equal to each
    let bb_cube = c1.bounding_box();
    let bb_sphere = c2.bounding_box();
    assert!(bb_isect.mins.x >= bb_cube.mins.x - EPSILON);
    assert!(bb_isect.mins.x >= bb_sphere.mins.x - EPSILON);
    assert!(bb_isect.maxs.x <= bb_cube.maxs.x + EPSILON);
    assert!(bb_isect.maxs.x <= bb_sphere.maxs.x + EPSILON);
}

#[test]
fn test_csg_intersect2() {
    let sphere: CSG<()> = CSG::sphere(1.0, 16, 8, None);
    let cube: CSG<()> = CSG::cube(2.0, 2.0, 2.0, None);

    let intersection = sphere.intersection(&cube);
    let polys = intersection.to_polygons();
    assert!(
        !polys.is_empty(),
        "Sphere ∩ Cube should produce the portion of the sphere inside the cube"
    );

    // Check bounding box => intersection is roughly a sphere clipped to [-1,1]^3
    let bb = bounding_box(&polys);
    // Should be a region inside the [-1,1] box
    for &val in &bb[..3] {
        assert!(val >= -1.0 - 1e-1);
    }
    for &val in &bb[3..] {
        assert!(val <= 1.0 + 1e-1);
    }
}

#[test]
fn test_csg_inverse() {
    let c1: CSG<()> = CSG::cube(2.0, 2.0, 2.0, None);
    let inv = c1.inverse();
    // The polygons are flipped
    // We can check just that the polygon planes are reversed, etc.
    // A full check might compare the polygons, but let's do a quick check on one polygon.
    let orig_poly = &c1.polygons[0];
    let inv_poly = &inv.polygons[0];
    assert!(approx_eq(
        orig_poly.plane.normal().x,
        -inv_poly.plane.normal().x,
        EPSILON
    ));
    assert!(approx_eq(
        orig_poly.plane.normal().y,
        -inv_poly.plane.normal().y,
        EPSILON
    ));
    assert!(approx_eq(
        orig_poly.plane.normal().z,
        -inv_poly.plane.normal().z,
        EPSILON
    ));
    assert_eq!(
        c1.to_polygons().len(),
        inv.to_polygons().len(),
        "Inverse should keep the same polygon count, but flip them"
    );
}

#[test]
fn test_csg_cube() {
    let c: CSG<()> = CSG::cube(2.0, 2.0, 2.0, None);
    // By default, corner at (0,0,0)
    // We expect 6 faces, each 4 vertices = 6 polygons
    assert_eq!(c.polygons.len(), 6);
    // Check bounding box
    let bb = c.bounding_box();
    assert!(approx_eq(bb.mins.x, 0.0, EPSILON));
    assert!(approx_eq(bb.maxs.x, 2.0, EPSILON));
}

// --------------------------------------------------------
//   CSG: Basic Shape Generation
// --------------------------------------------------------

#[test]
fn test_csg_sphere() {
    // Default sphere => radius=1, slices=16, stacks=8
    let sphere: CSG<()> = CSG::sphere(1.0, 16, 8, None);
    let polys = sphere.to_polygons();
    assert!(!polys.is_empty(), "Sphere should generate polygons");

    let bb = bounding_box(&polys);
    // Should roughly be [-1, -1, -1, 1, 1, 1]
    assert!(approx_eq(bb[0], -1.0, 1e-1));
    assert!(approx_eq(bb[1], -1.0, 1e-1));
    assert!(approx_eq(bb[2], -1.0, 1e-1));
    assert!(approx_eq(bb[3], 1.0, 1e-1));
    assert!(approx_eq(bb[4], 1.0, 1e-1));
    assert!(approx_eq(bb[5], 1.0, 1e-1));

    // We expect 16 * 8 polygons = 128 polygons
    // each stack band is 16 polys, times 8 => 128.
    assert_eq!(sphere.polygons.len(), 16 * 8);
}

#[test]
fn test_csg_cylinder() {
    // Default cylinder => from (0,0,0) to (0,2,0) with radius=1
    let cylinder: CSG<()> = CSG::cylinder(1.0, 2.0, 16, None);
    let polys = cylinder.to_polygons();
    assert!(!polys.is_empty(), "Cylinder should generate polygons");

    let bb = bounding_box(&polys);
    // Expect x in [-1,1], y in [-1,1], z in [-1,1].
    assert!(approx_eq(bb[0], -1.0, 1e-8), "min X");
    assert!(approx_eq(bb[1], -1.0, 1e-8), "min Y");
    assert!(approx_eq(bb[2], 0.0, 1e-8), "min Z");
    assert!(approx_eq(bb[3], 1.0, 1e-8), "max X");
    assert!(approx_eq(bb[4], 1.0, 1e-8), "max Y");
    assert!(approx_eq(bb[5], 2.0, 1e-8), "max Z");

    // We have slices = 16, plus 16*2 polygons for the end caps
    assert_eq!(cylinder.polygons.len(), 48);
}

#[test]
fn test_csg_polyhedron() {
    // A simple tetrahedron
    let pts = &[
        [0.0, 0.0, 0.0], // 0
        [1.0, 0.0, 0.0], // 1
        [0.0, 1.0, 0.0], // 2
        [0.0, 0.0, 1.0], // 3
    ];
    let faces = vec![vec![0, 1, 2], vec![0, 1, 3], vec![1, 2, 3], vec![2, 0, 3]];
    let csg_tetra: CSG<()> = CSG::polyhedron(pts, &faces, None);
    // We should have exactly 4 triangular faces
    assert_eq!(csg_tetra.polygons.len(), 4);
}

#[test]
fn test_csg_transform_translate_rotate_scale() {
    let c: CSG<()> = CSG::cube(2.0, 2.0, 2.0, None).center();
    let translated = c.translate(1.0, 2.0, 3.0);
    let rotated = c.rotate(90.0, 0.0, 0.0); // 90 deg about X
    let scaled = c.scale(2.0, 1.0, 1.0);

    // Quick bounding box checks
    let bb_t = translated.bounding_box();
    assert!(approx_eq(bb_t.mins.x, -1.0 + 1.0, EPSILON));
    assert!(approx_eq(bb_t.mins.y, -1.0 + 2.0, EPSILON));
    assert!(approx_eq(bb_t.mins.z, -1.0 + 3.0, EPSILON));

    let bb_s = scaled.bounding_box();
    assert!(approx_eq(bb_s.mins.x, -2.0, EPSILON)); // scaled by 2 in X
    assert!(approx_eq(bb_s.maxs.x, 2.0, EPSILON));
    assert!(approx_eq(bb_s.mins.y, -1.0, EPSILON));
    assert!(approx_eq(bb_s.maxs.y, 1.0, EPSILON));

    // For rotated, let's just check one polygon's vertices to see if z got mapped to y, etc.
    // (A thorough check would be more geometry-based.)
    let poly0 = &rotated.polygons[0];
    for v in &poly0.vertices {
        // After a 90° rotation around X, the old Y should become old Z,
        // and the old Z should become -old Y.
        // We can’t trivially guess each vertex's new coordinate but can do a sanity check:
        // The bounding box in Y might be [-1..1], but let's check we have differences in Y from original.
        assert_ne!(v.pos.y, 0.0); // Expect something was changed if originally it was ±1 in Z
    }
}

#[test]
fn test_csg_mirror() {
    let c: CSG<()> = CSG::cube(2.0, 2.0, 2.0, None);
    let plane_x = Plane::from_normal(Vector3::x(), 0.0); // x=0 plane
    let mirror_x = c.mirror(plane_x);
    let bb_mx = mirror_x.bounding_box();
    // The original cube was from x=0..2, so mirrored across X=0 should be -2..0
    assert!(approx_eq(bb_mx.mins.x, -2.0, EPSILON));
    assert!(approx_eq(bb_mx.maxs.x, 0.0, EPSILON));
}

#[test]
fn test_csg_convex_hull() {
    // If we take a shape with some random points, the hull should just enclose them
    let c1: CSG<()> = CSG::sphere(1.0, 16, 8, None);
    // The convex_hull of a sphere's sampling is basically that same shape, but let's see if it runs.
    let hull = c1.convex_hull();
    // The hull should have some polygons
    assert!(!hull.polygons.is_empty());
}

#[test]
fn test_csg_minkowski_sum() {
    // Minkowski sum of two cubes => bigger cube offset by edges
    let c1: CSG<()> = CSG::cube(2.0, 2.0, 2.0, None).center();
    let c2: CSG<()> = CSG::cube(1.0, 1.0, 1.0, None).center();
    let sum = c1.minkowski_sum(&c2);
    let bb_sum = sum.bounding_box();
    // Expect bounding box from -1.5..+1.5 in each axis if both cubes were centered at (0,0,0).
    assert!(approx_eq(bb_sum.mins.x, -1.5, 0.01));
    assert!(approx_eq(bb_sum.maxs.x, 1.5, 0.01));
}

#[test]
fn test_csg_subdivide_triangles() {
    let cube: CSG<()> = CSG::cube(2.0, 2.0, 2.0, None);
    // subdivide_triangles(1) => each polygon (quad) is triangulated => 2 triangles => each tri subdivides => 4
    // So each face with 4 vertices => 2 triangles => each becomes 4 => total 8 per face => 6 faces => 48
    let subdiv = cube.subdivide_triangles(1);
    assert_eq!(subdiv.polygons.len(), 6 * 8);
}

#[test]
fn test_csg_renormalize() {
    let mut cube: CSG<()> = CSG::cube(2.0, 2.0, 2.0, None);
    // After we do some transforms, normals might be changed. We can artificially change them:
    for poly in &mut cube.polygons {
        for v in &mut poly.vertices {
            v.normal = Vector3::x(); // just set to something
        }
    }
    cube.renormalize();
    // Now each polygon's vertices should match the plane's normal
    for poly in &cube.polygons {
        for v in &poly.vertices {
            assert!(approx_eq(v.normal.x, poly.plane.normal().x, EPSILON));
            assert!(approx_eq(v.normal.y, poly.plane.normal().y, EPSILON));
            assert!(approx_eq(v.normal.z, poly.plane.normal().z, EPSILON));
        }
    }
}

#[test]
fn test_csg_ray_intersections() {
    let cube: CSG<()> = CSG::cube(2.0, 2.0, 2.0, None).center();
    // Ray from (-2,0,0) toward +X
    let origin = Point3::new(-2.0, 0.0, 0.0);
    let direction = Vector3::new(1.0, 0.0, 0.0);
    let hits = cube.ray_intersections(&origin, &direction);
    // Expect 2 intersections with the cube's side at x=-1 and x=1
    assert_eq!(hits.len(), 2);
    // The distances should be 1 unit from -2.0 -> -1 => t=1, and from -2.0 -> +1 => t=3
    assert!(approx_eq(hits[0].1, 1.0, EPSILON));
    assert!(approx_eq(hits[1].1, 3.0, EPSILON));
}

#[test]
fn test_csg_square() {
    let sq: CSG<()> = CSG::square(2.0, 2.0, None);
    // Single polygon, 4 vertices
    assert_eq!(sq.polygons.len(), 1);
    let poly = &sq.polygons[0];
    assert_eq!(poly.vertices.len(), 4);
}

#[test]
fn test_csg_circle() {
    let circle: CSG<()> = CSG::circle(2.0, 32, None);
    // Single polygon with 32 segments => 32 vertices
    assert_eq!(circle.polygons.len(), 1);
    let poly = &circle.polygons[0];
    assert_eq!(poly.vertices.len(), 32);
}

#[test]
fn test_csg_polygon_2d() {
    let points = &[[0.0, 0.0], [2.0, 0.0], [2.0, 1.0], [0.0, 1.0]];
    let poly2d: CSG<()> = CSG::polygon(points, None);
    assert_eq!(poly2d.polygons.len(), 1);
    assert_eq!(poly2d.polygons[0].vertices.len(), 4);
}

#[test]
fn test_csg_extrude() {
    let sq: CSG<()> = CSG::square(2.0, 2.0, None); // default 1x1 square at XY plane
    let extruded = sq.extrude(5.0);
    // We expect:
    //   bottom polygon: 1
    //   top polygon (translated): 1
    //   side polygons: 4 for a square (one per edge)
    // => total 6 polygons
    assert_eq!(extruded.polygons.len(), 6);
    // Check bounding box
    let bb = extruded.bounding_box();
    assert!(approx_eq(bb.mins.z, 0.0, EPSILON));
    assert!(approx_eq(bb.maxs.z, 5.0, EPSILON));
}

#[test]
fn test_csg_rotate_extrude() {
    // Default square is from (0,0) to (1,1) in XY.
    // Shift it so it’s from (1,0) to (2,1) — i.e. at least 1.0 unit away from the Z-axis.
    // and rotate it 90 degrees so that it can be swept around Z
    let square: CSG<()> = CSG::square(2.0, 2.0, None)
        .translate(1.0, 0.0, 0.0)
        .rotate(90.0, 0.0, 0.0);

    // Now revolve this translated square around the Z-axis, 360° in 16 segments.
    let revolve = square.rotate_extrude(360.0, 16);

    // We expect a ring-like “tube” instead of a degenerate shape.
    assert!(!revolve.polygons.is_empty());
}

#[test]
fn test_csg_bounding_box() {
    let sphere: CSG<()> = CSG::sphere(1.0, 16, 8, None);
    let bb = sphere.bounding_box();
    // center=(2,-1,3), radius=2 => bounding box min=(0,-3,1), max=(4,1,5)
    assert!(approx_eq(bb.mins.x, -1.0, 0.1));
    assert!(approx_eq(bb.mins.y, -1.0, 0.1));
    assert!(approx_eq(bb.mins.z, -1.0, 0.1));
    assert!(approx_eq(bb.maxs.x, 1.0, 0.1));
    assert!(approx_eq(bb.maxs.y, 1.0, 0.1));
    assert!(approx_eq(bb.maxs.z, 1.0, 0.1));
}

#[test]
fn test_csg_vertices() {
    let cube: CSG<()> = CSG::cube(2.0, 2.0, 2.0, None);
    let verts = cube.vertices();
    // 6 faces x 4 vertices each = 24
    assert_eq!(verts.len(), 24);
}

#[test]
fn test_csg_offset_2d() {
    let square: CSG<()> = CSG::square(2.0, 2.0, None);
    let grown = square.offset(0.5);
    let shrunk = square.offset(-0.5);
    let bb_square = square.bounding_box();
    let bb_grown = grown.bounding_box();
    let bb_shrunk = shrunk.bounding_box();

    println!("Square bb: {:#?}", bb_square);
    println!("Grown bb: {:#?}", bb_grown);
    println!("Shrunk bb: {:#?}", bb_shrunk);

    // Should be bigger
    assert!(bb_grown.maxs.x > bb_square.maxs.x + 0.4);

    // Should be smaller
    assert!(bb_shrunk.maxs.x < bb_square.maxs.x + 0.1);
}

#[test]
fn test_csg_text() {
    // We can’t easily test visually, but we can at least test that it doesn’t panic
    // and returns some polygons for normal ASCII letters.
    let font_data = include_bytes!("../asar.ttf");
    let text_csg: CSG<()> = CSG::text("ABC", font_data, 10.0, None);
    assert!(!text_csg.polygons.is_empty());
}

#[test]
fn test_csg_to_trimesh() {
    let cube: CSG<()> = CSG::cube(2.0, 2.0, 2.0, None);
    let shape = cube.to_trimesh();
    // Should be a TriMesh with 12 triangles
    if let Some(trimesh) = shape.as_trimesh() {
        assert_eq!(trimesh.indices().len(), 12); // 6 faces => 2 triangles each => 12
    } else {
        panic!("Expected a TriMesh");
    }
}

#[test]
fn test_csg_mass_properties() {
    let cube: CSG<()> = CSG::cube(2.0, 2.0, 2.0, None).center(); // side=2 => volume=8. If density=1 => mass=8
    let (mass, com, _frame) = cube.mass_properties(1.0);
    println!("{:#?}", mass);
    // For a centered cube with side 2, volume=8 => mass=8 => COM=(0,0,0)
    assert!(approx_eq(mass, 8.0, 0.1));
    assert!(approx_eq(com.x, 0.0, 0.001));
    assert!(approx_eq(com.y, 0.0, 0.001));
    assert!(approx_eq(com.z, 0.0, 0.001));
}

#[test]
fn test_csg_to_rigid_body() {
    use crate::float_types::rapier3d::prelude::*;
    let cube: CSG<()> = CSG::cube(2.0, 2.0, 2.0, None);
    let mut rb_set = RigidBodySet::new();
    let mut co_set = ColliderSet::new();
    let handle = cube.to_rigid_body(
        &mut rb_set,
        &mut co_set,
        Vector3::new(10.0, 0.0, 0.0),
        Vector3::new(0.0, 0.0, FRAC_PI_2), // 90 deg around Z
        1.0,
    );
    let rb = rb_set.get(handle).unwrap();
    let pos = rb.translation();
    assert!(approx_eq(pos.x, 10.0, EPSILON));
}

#[test]
fn test_csg_to_stl_and_from_stl_file() -> Result<(), Box<dyn std::error::Error>> {
    // We'll create a small shape, write to an STL, read it back.
    // You can redirect to a temp file or do an in-memory test.
    let tmp_path = "test_csg_output.stl";

    let cube: CSG<()> = CSG::cube(2.0, 2.0, 2.0, None);
    let res = cube.to_stl_binary("A cube");
    let _ = std::fs::write(tmp_path, res.as_ref().unwrap());
    assert!(res.is_ok());

    let stl_data: Vec<u8> = std::fs::read(tmp_path)?;
    let csg_in: CSG<()> = CSG::from_stl(&stl_data, None)?;
    // We expect to read the same number of triangular faces as the cube originally had
    // (though the orientation/normals might differ).
    // The default cube -> 6 polygons x 1 polygon each with 4 vertices => 12 triangles in STL.
    // So from_stl_file => we get 12 triangles as 12 polygons (each is a tri).
    assert_eq!(csg_in.polygons.len(), 12);

    // Cleanup the temp file if desired
    let _ = std::fs::remove_file(tmp_path);
    Ok(())
}

/// A small, custom metadata type to demonstrate usage.
/// We derive PartialEq so we can assert equality in tests.
#[derive(Debug, Clone, PartialEq)]
struct MyMetaData {
    id: u32,
    label: String,
}

#[test]
fn test_polygon_metadata_string() {
    // Create a simple triangle polygon with shared data = Some("triangle".to_string()).
    let verts = vec![
        Vertex::new(Point3::origin(), Vector3::z()),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
    ];
    let mut poly = Polygon::new(verts, Some("triangle".to_string()));

    // Check getter
    assert_eq!(poly.metadata(), Some(&"triangle".to_string()));

    // Check setter
    poly.set_metadata("updated".to_string());
    assert_eq!(poly.metadata(), Some(&"updated".to_string()));

    // Check mutable getter
    if let Some(data) = poly.metadata_mut() {
        data.push_str("_appended");
    }
    assert_eq!(poly.metadata(), Some(&"updated_appended".to_string()));
}

#[test]
fn test_polygon_metadata_integer() {
    // Create a polygon with integer shared data
    let verts = vec![
        Vertex::new(Point3::origin(), Vector3::z()),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
    ];
    let poly = Polygon::new(verts, Some(42u32));

    // Confirm data
    assert_eq!(poly.metadata(), Some(&42));
}

#[test]
fn test_polygon_metadata_custom_struct() {
    // Create a polygon with our custom struct
    let my_data = MyMetaData {
        id: 999,
        label: "MyLabel".into(),
    };
    let verts = vec![
        Vertex::new(Point3::origin(), Vector3::z()),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
    ];
    let poly = Polygon::new(verts, Some(my_data.clone()));

    assert_eq!(poly.metadata(), Some(&my_data));
}

#[test]
fn test_csg_construction_with_metadata() {
    // Build a CSG of two polygons, each with distinct shared data.
    let poly_a = Polygon::new(
        vec![
            Vertex::new(Point3::origin(), Vector3::z()),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(1.0, 1.0, 0.0), Vector3::z()),
        ],
        Some("PolyA".to_string()),
    );
    let poly_b = Polygon::new(
        vec![
            Vertex::new(Point3::new(2.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(3.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(3.0, 1.0, 0.0), Vector3::z()),
        ],
        Some("PolyB".to_string()),
    );
    let csg = CSG::from_polygons(&[poly_a.clone(), poly_b.clone()]);

    // We expect two polygons with the same shared data as the originals.
    assert_eq!(csg.polygons.len(), 2);
    assert_eq!(csg.polygons[0].metadata(), Some(&"PolyA".to_string()));
    assert_eq!(csg.polygons[1].metadata(), Some(&"PolyB".to_string()));
}

#[test]
fn test_union_metadata() {
    // Let's union two squares in the XY plane, each with different shared data.
    // So after union, we typically get polygons from each original shape.
    // If there's any overlap, new polygons might be formed, but in CSG
    // each new polygon inherits the shared data from whichever polygon it came from.

    // Square1 from (0,0) to (1,1) => label "Square1"
    let sq1 = CSG::square(1.0, 1.0, None); // bottom-left at (0,0), top-right at (1,1)
    let mut sq1 = sq1; // now let us set shared data for each polygon
    for p in &mut sq1.polygons {
        p.set_metadata("Square1".to_string());
    }

    // Translate Square2 so it partially overlaps. => label "Square2"
    let sq2 = CSG::square(1.0, 1.0, None).translate(0.5, 0.0, 0.0);
    let mut sq2 = sq2;
    for p in &mut sq2.polygons {
        p.set_metadata("Square2".to_string());
    }

    // Union
    let union_csg = sq1.union(&sq2);

    // Depending on the library's polygon splitting, we often end up with multiple polygons.
    // We can at least confirm that each polygon's shared data is EITHER "Square1" or "Square2",
    // and never mixed or lost.
    for poly in &union_csg.polygons {
        let data = poly.metadata().unwrap();
        assert!(
            data == "Square1" || data == "Square2",
            "Union polygon has unexpected shared data = {:?}",
            data
        );
    }
}

#[test]
fn test_difference_metadata() {
    // Difference two cubes, each with different shared data. The resulting polygons
    // come from the *minuend* (the first shape) with *some* portion clipped out.
    // So the differenced portion from the second shape won't appear in the final.

    let mut cube1 = CSG::cube(2.0, 2.0, 2.0, None);
    for p in &mut cube1.polygons {
        p.set_metadata("Cube1".to_string());
    }

    let mut cube2 = CSG::cube(2.0, 2.0, 2.0, None).translate(0.5, 0.5, 0.5);
    for p in &mut cube2.polygons {
        p.set_metadata("Cube2".to_string());
    }

    let result = cube1.difference(&cube2);

    // All polygons in the result should come from "Cube1" only.
    for poly in &result.polygons {
        assert_eq!(poly.metadata(), Some(&"Cube1".to_string()));
    }
}

#[test]
fn test_intersect_metadata() {
    // Intersection: the resulting polygons should come from polygons that are inside both.
    // Typically, the library picks polygons from the first shape, then clips them
    // against the second. Depending on exact implementation, the polygons that remain
    // carry the first shape's shared data. In many CSG implementations, the final polygons
    // keep the "side" from whichever shape is relevant. That might be shape A or B or both.
    // We'll check that we only see "Cube1" or "Cube2" but not random data.

    let mut cube1 = CSG::cube(2.0, 2.0, 2.0, None);
    for p in &mut cube1.polygons {
        p.set_metadata("Cube1".to_string());
    }

    let mut cube2 = CSG::cube(2.0, 2.0, 2.0, None).translate(0.5, 0.5, 0.5);
    for p in &mut cube2.polygons {
        p.set_metadata("Cube2".to_string());
    }

    let result = cube1.intersection(&cube2);

    // Depending on the implementation, it's common that intersection polygons are
    // actually from both shapes or from shape A. Let's check that if they do have shared data,
    // it must be from either "Cube1" or "Cube2".
    for poly in &result.polygons {
        let data = poly.metadata().unwrap();
        assert!(
            data == "Cube1" || data == "Cube2",
            "Intersection polygon has unexpected shared data = {:?}",
            data
        );
    }
}

#[test]
fn test_flip_invert_metadata() {
    // Flipping or inverting a shape should NOT change the shared data;
    // it only flips normals/polygons.

    let mut csg = CSG::cube(2.0, 2.0, 2.0, None);
    for p in &mut csg.polygons {
        p.set_metadata("MyCube".to_string());
    }

    // Invert
    let inverted = csg.inverse();
    for poly in &inverted.polygons {
        assert_eq!(poly.metadata(), Some(&"MyCube".to_string()));
    }
}

#[test]
fn test_subdivide_metadata() {
    // Subdivide a polygon with shared data, ensure all new subdivided polygons
    // preserve that data.

    let poly = Polygon::new(
        vec![
            Vertex::new(Point3::origin(), Vector3::z()),
            Vertex::new(Point3::new(2.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(2.0, 2.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.0, 2.0, 0.0), Vector3::z()),
        ],
        Some("LargeQuad".to_string()),
    );
    let csg = CSG::from_polygons(&[poly]);
    let subdivided = csg.subdivide_triangles(1); // one level of subdivision

    // Now it's split into multiple triangles. Each should keep "LargeQuad" as metadata.
    assert!(subdivided.polygons.len() > 1);
    for spoly in &subdivided.polygons {
        assert_eq!(spoly.metadata(), Some(&"LargeQuad".to_string()));
    }
}

#[test]
fn test_transform_metadata() {
    // Make sure that transform does not lose or change shared data.
    let poly = Polygon::new(
        vec![
            Vertex::new(Point3::origin(), Vector3::z()),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
        ],
        Some("Tri".to_string()),
    );
    let csg = CSG::from_polygons(&[poly]);
    let csg_trans = csg.translate(10.0, 5.0, 0.0);
    let csg_scale = csg_trans.scale(2.0, 2.0, 1.0);
    let csg_rot = csg_scale.rotate(0.0, 0.0, 45.0);

    for poly in &csg_rot.polygons {
        assert_eq!(poly.metadata(), Some(&"Tri".to_string()));
    }
}

#[test]
fn test_complex_metadata_struct_in_boolean_ops() {
    // We'll do an operation using a custom struct to verify it remains intact.
    // We'll do a union for instance.

    #[derive(Debug, Clone, PartialEq)]
    struct Color(u8, u8, u8);

    let mut csg1 = CSG::cube(2.0, 2.0, 2.0, None);
    for p in &mut csg1.polygons {
        p.set_metadata(Color(255, 0, 0));
    }
    let mut csg2 = CSG::cube(2.0, 2.0, 2.0, None).translate(0.5, 0.5, 0.5);
    for p in &mut csg2.polygons {
        p.set_metadata(Color(0, 255, 0));
    }

    let unioned = csg1.union(&csg2);
    // Now polygons are either from csg1 (red) or csg2 (green).
    for poly in &unioned.polygons {
        let col = poly.metadata().unwrap();
        assert!(
            *col == Color(255, 0, 0) || *col == Color(0, 255, 0),
            "Unexpected color in union: {:?}",
            col
        );
    }
}

/// Helper function to calculate the signed area of a polygon.
/// Positive area indicates CCW ordering.
fn signed_area(polygon: &Polygon<()>) -> Real {
    let mut area = 0.0;
    let verts = &polygon.vertices;
    for i in 0..verts.len() {
        let j = (i + 1) % verts.len();
        area += (verts[i].pos.x * verts[j].pos.y) - (verts[j].pos.x * verts[i].pos.y);
    }
    area / 2.0
}

#[test]
fn test_square_ccw_ordering() {
    let square = CSG::square(2.0, 2.0, None);
    assert_eq!(square.polygons.len(), 1);
    let poly = &square.polygons[0];
    let area = signed_area(poly);
    assert!(area > 0.0, "Square vertices are not CCW ordered");
}

#[test]
fn test_offset_2d_positive_distance_grows() {
    let square = CSG::square(2.0, 2.0, None); // Centered square with size 2x2
    let offset = square.offset(0.5); // Positive offset should grow the square

    // The original square has area 4.0
    // The offset square should have area greater than 4.0
    assert_eq!(offset.polygons.len(), 1);
    let poly = &offset.polygons[0];
    let area = signed_area(poly);
    assert!(
        area > 4.0,
        "Offset with positive distance did not grow the square"
    );
}

#[test]
fn test_offset_2d_negative_distance_shrinks() {
    let square = CSG::square(2.0, 2.0, None); // Centered square with size 2x2
    let offset = square.offset(-0.5); // Negative offset should shrink the square

    // The original square has area 4.0
    // The offset square should have area less than 4.0
    assert_eq!(offset.polygons.len(), 1);
    let poly = &offset.polygons[0];
    let area = signed_area(poly);
    assert!(
        area < 4.0,
        "Offset with negative distance did not shrink the square"
    );
}

#[test]
fn test_polygon_2d_enforce_ccw_ordering() {
    // Define a triangle in CW order
    let points_cw = vec![[0.0, 0.0], [1.0, 0.0], [0.5, 1.0]];
    let mut csg_cw = CSG::polygon(&points_cw, None);
    // Enforce CCW ordering
    csg_cw.renormalize();
    let poly = &csg_cw.polygons[0];
    let area = signed_area(poly);
    assert!(area > 0.0, "Polygon ordering was not corrected to CCW");
}

#[test]
fn test_circle_offset_2d() {
    let circle = CSG::circle(1.0, 32, None);
    let offset_grow = circle.offset(0.2); // Should grow the circle
    let offset_shrink = circle.offset(-0.2); // Should shrink the circle

    // Original circle has area ~3.1416
    let original_area = 3.141592653589793;
    let grow_area = signed_area(&offset_grow.polygons[0]);
    let shrink_area = signed_area(&offset_shrink.polygons[0]);

    assert!(
        grow_area > original_area,
        "Offset with positive distance did not grow the circle"
    );
    assert!(
        shrink_area < original_area,
        "Offset with negative distance did not shrink the circle"
    );
}

/// Helper to make a simple Polygon in 3D with given vertices.
fn make_polygon_3d(points: &[[Real; 3]]) -> Polygon<()> {
    let mut verts = Vec::new();
    for p in points {
        let pos = Point3::new(p[0], p[1], p[2]);
        // For simplicity, just store an arbitrary normal; Polygon::new re-computes the plane anyway.
        let normal = Vector3::z();
        verts.push(Vertex::new(pos, normal));
    }
    Polygon::new(verts, None)
}

#[test]
fn test_same_number_of_vertices() {
    // "Bottom" is a triangle in 3D
    let bottom = make_polygon_3d(&[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.5, 0.5, 0.0]]);
    // "Top" is the same triangle, shifted up in Z
    let top = make_polygon_3d(&[[0.0, 0.0, 1.0], [1.0, 0.0, 1.0], [0.5, 0.5, 1.0]]);

    // This should succeed with no panic:
    let csg = CSG::extrude_between(&bottom, &top, true);

    // Expect:
    //  - bottom polygon
    //  - top polygon
    //  - 3 side polygons (one for each edge of the triangle)
    assert_eq!(
        csg.polygons.len(),
        1 /*bottom*/ + 1 /*top*/ + 3 /*sides*/
    );
}

#[test]
#[should_panic(expected = "same number of vertices")]
fn test_different_number_of_vertices_panics() {
    // Bottom has 3 vertices
    let bottom = make_polygon_3d(&[[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [1.0, 1.0, 0.0]]);
    // Top has 4 vertices
    let top = make_polygon_3d(&[
        [0.0, 0.0, 2.0],
        [2.0, 0.0, 2.0],
        [2.0, 2.0, 2.0],
        [0.0, 2.0, 2.0],
    ]);

    // This should panic due to unequal vertex counts
    let _ = CSG::extrude_between(&bottom, &top, true);
}

#[test]
fn test_consistent_winding() {
    // Make a square in the XY plane (bottom)
    let bottom = make_polygon_3d(&[
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [1.0, 1.0, 0.0],
        [0.0, 1.0, 0.0],
    ]);
    // Make the same square, shifted up in Z, with the same winding direction
    let top = make_polygon_3d(&[
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 1.0],
        [1.0, 1.0, 1.0],
        [0.0, 1.0, 1.0],
    ]);

    let csg = CSG::extrude_between(&bottom, &top, false);

    // Expect 1 bottom + 1 top + 4 side faces = 6 polygons
    assert_eq!(csg.polygons.len(), 6);

    // Optionally check that each polygon has at least 3 vertices
    for poly in &csg.polygons {
        assert!(poly.vertices.len() >= 3);
    }
}

#[test]
fn test_inverted_orientation() {
    // Bottom square
    let bottom = make_polygon_3d(&[
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [1.0, 1.0, 0.0],
        [0.0, 1.0, 0.0],
    ]);
    // Top square, but with vertices in opposite order => "flipped" winding
    let mut top = make_polygon_3d(&[
        [0.0, 1.0, 1.0],
        [1.0, 1.0, 1.0],
        [1.0, 0.0, 1.0],
        [0.0, 0.0, 1.0],
    ]);

    // We can fix by flipping `top`:
    top.flip();

    let csg = CSG::extrude_between(&bottom, &top, false);

    // Expect 1 bottom + 1 top + 4 sides = 6 polygons
    assert_eq!(csg.polygons.len(), 6);

    // Check bounding box for sanity
    let bbox = csg.bounding_box();
    assert!(
        bbox.mins.z < bbox.maxs.z,
        "Should have a non-zero height in the Z dimension"
    );
}

#[test]
fn test_union_of_extruded_shapes() {
    // We'll extrude two shapes that partially overlap, then union them.

    // First shape: triangle
    let bottom1 = make_polygon_3d(&[[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [1.0, 1.0, 0.0]]);
    let top1 = make_polygon_3d(&[[0.0, 0.0, 1.0], [2.0, 0.0, 1.0], [1.0, 1.0, 1.0]]);
    let csg1 = CSG::extrude_between(&bottom1, &top1, true);

    // Second shape: small shifted square
    let bottom2 = make_polygon_3d(&[
        [1.0, -0.2, 0.5],
        [2.0, -0.2, 0.5],
        [2.0, 0.8, 0.5],
        [1.0, 0.8, 0.5],
    ]);
    let top2 = make_polygon_3d(&[
        [1.0, -0.2, 1.5],
        [2.0, -0.2, 1.5],
        [2.0, 0.8, 1.5],
        [1.0, 0.8, 1.5],
    ]);
    let csg2 = CSG::extrude_between(&bottom2, &top2, true);

    // Union them
    let unioned = csg1.union(&csg2);

    // Sanity check: union shouldn’t be empty
    assert!(!unioned.polygons.is_empty());

    // Its bounding box should span at least from z=0 to z=1.5
    let bbox = unioned.bounding_box();
    assert!(bbox.mins.z <= 0.0 + EPSILON);
    assert!(bbox.maxs.z >= 1.5 - EPSILON);
}

#[test]
fn test_flatten_cube() {
    // 1) Create a cube from (-1,-1,-1) to (+1,+1,+1)
    //    (By default, CSG::cube(None) is from -1..+1 if the "radius" is [1,1,1].)
    let cube = CSG::<()>::cube(2.0, 2.0, 2.0, None);
    // 2) Flatten into the XY plane
    let flattened = cube.flatten();

    // The flattened cube should have 1 polygon1, now in z=0
    assert_eq!(
        flattened.polygons.len(),
        1,
        "Flattened cube should have 1 face in z=0"
    );

    // Check that all vertices lie at z=0
    for poly in &flattened.polygons {
        for v in &poly.vertices {
            assert!(
                (v.pos.z - 0.0).abs() < EPSILON,
                "Flattened vertex must have z=0, found z={}",
                v.pos.z
            );
        }
    }

    // Optional: we can check the bounding box in z-dimension is effectively zero
    let bbox = flattened.bounding_box();
    let thickness = bbox.maxs.z - bbox.mins.z;
    assert!(
        thickness.abs() < EPSILON,
        "Flattened shape should have negligible thickness in z"
    );
}

#[test]
#[cfg(feature = "hashmap")]
fn test_slice_cylinder() {
    // 1) Create a cylinder (start=-1, end=+1) with radius=1, 32 slices
    let cyl = CSG::<()>::cylinder(1.0, 2.0, 32, None).center();
    // 2) Slice at z=0
    let cross_section = cyl.slice(Plane::from_normal(Vector3::z(), 0.0));

    // For a simple cylinder, the cross-section is typically 1 circle polygon
    // (unless the top or bottom also exactly intersect z=0, which they do not in this scenario).
    // So we expect exactly 1 polygon.
    assert_eq!(
        cross_section.polygons.len(),
        1,
        "Slicing a cylinder at z=0 should yield exactly 1 cross-section polygon"
    );

    let poly = &cross_section.polygons[0];
    let vcount = poly.vertices.len();

    // We used 32 slices for the cylinder, so we expect up to 32 edges
    // in the cross-section circle. Some slight differences might occur
    // if the slicing logic merges or sorts vertices.
    // Typically, you might see vcount = 32 or vcount = 34, etc.
    // Let's just check it's > 3 and in a plausible range:
    assert!(
        vcount >= 3 && vcount <= 40,
        "Expected cross-section circle to have a number of edges ~32, got {}",
        vcount
    );

    // Check all vertices are on z=0
    for v in &poly.vertices {
        assert!(
            (v.pos.z - 0.0).abs() < EPSILON,
            "Sliced vertex must have z=0, found z={}",
            v.pos.z
        );
    }

    // Optional: check approximate radius
    // The cross-section should be at radius ~1 around x=0,y=some, z=0
    // (Actually, the cylinder's axis is along Y, so we expect x^2+z^2=1.
    //  But since z=0 now, effectively we expect x^2=1 => x=±1 if we're ignoring any bulge,
    //  or we'll see a circle in the plane y=?? Actually, wait—
    //  the cylinder we built is from (0,-1,0) to (0,1,0) with radius=1 around that axis,
    //  so the cross-section plane is z=0, meaning x^2 + y^2 = 1.
    //  We can check a couple of sample vertices.)
    if !poly.vertices.is_empty() {
        let first_v = &poly.vertices[0].pos;
        let r_approx = (first_v.x.powi(2) + first_v.y.powi(2)).sqrt();
        // We expect something close to radius=1:
        assert!(
            (r_approx - 1.0).abs() < 1e-3,
            "Cross-section radius should be ~1, got {:?}",
            r_approx
        );
    }
}

/// Helper to create a `Polygon` in the XY plane from an array of (x,y) points,
/// with z=0 and normal=+Z.
fn polygon_from_xy_points(xy_points: &[[Real; 2]]) -> Polygon<()> {
    assert!(
        xy_points.len() >= 3,
        "Need at least 3 points for a polygon."
    );

    let normal = Vector3::z();
    let vertices: Vec<Vertex> = xy_points
        .iter()
        .map(|&[x, y]| Vertex::new(Point3::new(x, y, 0.0), normal))
        .collect();

    Polygon::new(vertices, None)
}

/// Test a simple case of `flatten_and_union` with a single square in the XY plane.
/// We expect the same shape back.
#[test]
fn test_flatten_and_union_single_polygon() {
    // Create a CSG with one polygon (a unit square).
    let square_poly = polygon_from_xy_points(&[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]);
    let csg = CSG::from_polygons(&[square_poly]);

    // Flatten & union it
    let flat_csg = csg.flatten();

    // Expect the same bounding box
    assert!(!flat_csg.polygons.is_empty(), "Result should not be empty");
    let bb = flat_csg.bounding_box();
    assert_eq!(bb.mins.x, 0.0);
    assert_eq!(bb.mins.y, 0.0);
    assert_eq!(bb.maxs.x, 1.0);
    assert_eq!(bb.maxs.y, 1.0);
}

/// Test `flatten_and_union` with two overlapping squares.
/// The result should be a single unioned polygon covering [0..2, 0..1].
#[test]
fn test_flatten_and_union_two_overlapping_squares() {
    // First square from (0,0) to (1,1)
    let square1 = polygon_from_xy_points(&[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]);
    // Second square from (1,0) to (2,1)
    let square2 = polygon_from_xy_points(&[[1.0, 0.0], [2.0, 0.0], [2.0, 1.0], [1.0, 1.0]]);
    let csg = CSG::from_polygons(&[square1, square2]);

    let flat_csg = csg.flatten();
    assert!(!flat_csg.polygons.is_empty(), "Union should not be empty");

    // The bounding box should now span x=0..2, y=0..1
    let bb = flat_csg.bounding_box();
    assert_eq!(bb.mins.x, 0.0);
    assert_eq!(bb.maxs.x, 2.0);
    assert_eq!(bb.mins.y, 0.0);
    assert_eq!(bb.maxs.y, 1.0);

    // We can also check that there's exactly 1 polygon if the union merges them fully
    // (Though sometimes the union might produce multiple polygons if the library doesn't merge edges,
    //  but with cavalier_contours "Or" we should get a single merged loop.)
    assert_eq!(
        flat_csg.polygons.len(),
        1,
        "Expected one merged polygon after flatten+union"
    );
}

/// Test `flatten_and_union` with two disjoint squares.
/// The result should have two separate polygons.
#[test]
fn test_flatten_and_union_two_disjoint_squares() {
    // Square A at (0..1, 0..1)
    let square_a = polygon_from_xy_points(&[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]);
    // Square B at (2..3, 2..3)
    let square_b = polygon_from_xy_points(&[[2.0, 2.0], [3.0, 2.0], [3.0, 3.0], [2.0, 3.0]]);
    let csg = CSG::from_polygons(&[square_a, square_b]);

    let flat_csg = csg.flatten();
    assert!(!flat_csg.polygons.is_empty());

    // Expect 2 disjoint polygons in the result
    assert_eq!(
        flat_csg.polygons.len(),
        2,
        "Expected two separate polygons after union"
    );
}

/// Test `flatten_and_union` when polygons are not perfectly in the XY plane,
/// but very close to z=0. This checks sensitivity to floating errors.
#[test]
fn test_flatten_and_union_near_xy_plane() {
    let normal = Vector3::z();
    // Slightly "tilted" or with z=1e-6
    let poly1 = Polygon::<()>::new(
        vec![
            Vertex::new(Point3::new(0.0, 0.0, 1e-6), normal),
            Vertex::new(Point3::new(1.0, 0.0, 1e-6), normal),
            Vertex::new(Point3::new(1.0, 1.0, 1e-6), normal),
            Vertex::new(Point3::new(0.0, 1.0, 1e-6), normal),
        ],
        None,
    );

    let csg = CSG::from_polygons(&[poly1]);
    let flat_csg = csg.flatten();

    assert!(
        !flat_csg.polygons.is_empty(),
        "Should flatten to a valid polygon"
    );
    let bb = flat_csg.bounding_box();
    assert_eq!(bb.mins.x, 0.0);
    assert_eq!(bb.maxs.x, 1.0);
    assert_eq!(bb.mins.y, 0.0);
    assert_eq!(bb.maxs.y, 1.0);
}

/// Test with multiple polygons that share edges or have nearly collinear edges
/// to ensure numeric tolerances (`eps_area`, `eps_pos`) don't remove them incorrectly.
#[test]
fn test_flatten_and_union_collinear_edges() {
    // Two rectangles sharing a long horizontal edge
    let rect1 = polygon_from_xy_points(&[[0.0, 0.0], [2.0, 0.0], [2.0, 1.0], [0.0, 1.0]]);
    let rect2 = polygon_from_xy_points(&[
        [2.0, 0.0],
        [4.0, 0.0],
        [4.0, 1.001], // slightly off
        [2.0, 1.0],
    ]);

    let csg = CSG::<()>::from_polygons(&[rect1, rect2]);
    let flat_csg = csg.flatten();

    // Expect 1 polygon from x=0..4, y=0..~1.0ish
    assert!(!flat_csg.polygons.is_empty());
    let bb = flat_csg.bounding_box();
    assert!((bb.maxs.x - 4.0).abs() < 1e-5, "Should span up to x=4.0");
    // Also check the y-range is ~1.001
    assert!((bb.maxs.y - 1.001).abs() < 1e-3);
}

/// If you suspect `flatten_and_union` is returning no polygons, this test
/// ensures we get at least one polygon for a simple shape. If it fails,
/// you can println! debug info in `flatten_and_union`.
#[test]
fn test_flatten_and_union_debug() {
    let csg_square = CSG::<()>::square(2.0, 2.0, None); // a 1×1 square at [0..1, 0..1]
    let flattened = csg_square.flatten();
    assert!(
        !flattened.polygons.is_empty(),
        "Flattened square should not be empty"
    );
    assert!(
        flattened.polygons[0].vertices.len() >= 3,
        "Should form at least a triangle"
    );
}
