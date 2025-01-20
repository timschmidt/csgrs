// tests

#[cfg(test)]
use super::*;
use nalgebra::{Vector3, Point3, Matrix4};

// --------------------------------------------------------
//   Helpers
// --------------------------------------------------------

/// Returns the approximate bounding box `[min_x, min_y, min_z, max_x, max_y, max_z]`
/// for a set of polygons.
fn bounding_box(polygons: &[Polygon]) -> [f64; 6] {
    let mut min_x = f64::MAX;
    let mut min_y = f64::MAX;
    let mut min_z = f64::MAX;
    let mut max_x = f64::MIN;
    let mut max_y = f64::MIN;
    let mut max_z = f64::MIN;

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
fn approx_eq(a: f64, b: f64, eps: f64) -> bool {
    (a - b).abs() < eps
}

// --------------------------------------------------------
//   Vertex Tests
// --------------------------------------------------------

#[test]
fn test_vertex_flip() {
    let mut v = Vertex::new(
        Point3::new(1.0, 2.0, 3.0),
        Vector3::new(1.0, 0.0, 0.0),
    );
    v.flip();
    // Position remains the same
    assert_eq!(v.pos, Point3::new(1.0, 2.0, 3.0));
    // Normal should be negated
    assert_eq!(v.normal, Vector3::new(-1.0, 0.0, 0.0));
}

// --------------------------------------------------------
//   Polygon Tests
// --------------------------------------------------------

#[test]
fn test_polygon_construction() {
    let v1 = Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::y());
    let v2 = Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::y());
    let v3 = Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::y());

    let poly = Polygon::new(vec![v1.clone(), v2.clone(), v3.clone()], None);
    assert_eq!(poly.vertices.len(), 3);
    // Plane should be defined by these three points. We expect a normal near ±Y.
    assert!(
        approx_eq(poly.plane.normal.dot(&Vector3::y()).abs(), 1.0, 1e-8),
        "Expected plane normal to match ±Y"
    );
}

// --------------------------------------------------------
//   CSG: STL Export
// --------------------------------------------------------

#[test]
fn test_to_stl() {
    let cube = CSG::cube(None);
    let stl_str = cube.to_stl("test_cube");
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
#[should_panic(expected = "Polygon::new requires at least 3 vertices")]
fn test_polygon_creation_with_fewer_than_three_vertices() {
    let vertices = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)),
    ];

    // This should panic due to insufficient vertices
    let _polygon = Polygon::new(vertices, None);
}

#[test]
fn test_degenerate_polygon_after_clipping() {
    let vertices = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0)),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0)),
        Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 1.0, 0.0)),
    ];

    let polygon = Polygon::new(vertices.clone(), None);
    let plane = Plane {
        normal: Vector3::new(0.0, 1.0, 0.0),
        w: 1.5,
    };

    let mut coplanar_front = Vec::new();
    let mut coplanar_back = Vec::new();
    let mut front = Vec::new();
    let mut back = Vec::new();

    eprintln!("Original polygon: {:?}", polygon);
    eprintln!("Clipping plane: {:?}", plane);

    plane.split_polygon(
        &polygon,
        &mut coplanar_front,
        &mut coplanar_back,
        &mut front,
        &mut back,
    );

    eprintln!("Front polygons: {:?}", front);
    eprintln!("Back polygons: {:?}", back);

    assert!(front.is_empty(), "Front should be empty for this test");
    assert!(back.is_empty(), "Back should be empty for this test");
}

#[test]
fn test_valid_polygon_clipping() {
    let vertices = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0)),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0)),
        Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 1.0, 0.0)),
    ];

    let polygon = Polygon::new(vertices, None);

    let plane = Plane {
        normal: Vector3::new(0.0, -1.0, 0.0),
        w: -0.5,
    };

    let mut coplanar_front = Vec::new();
    let mut coplanar_back = Vec::new();
    let mut front = Vec::new();
    let mut back = Vec::new();

    eprintln!("Polygon before clipping: {:?}", polygon);
    eprintln!("Clipping plane: {:?}", plane);

    plane.split_polygon(
        &polygon,
        &mut coplanar_front,
        &mut coplanar_back,
        &mut front,
        &mut back,
    );

    eprintln!("Front polygons: {:?}", front);
    eprintln!("Back polygons: {:?}", back);

    assert!(!front.is_empty(), "Front should not be empty");
    assert!(!back.is_empty(), "Back should not be empty");
}

#[test]
#[should_panic(expected = "Polygon::new requires at least 3 vertices")]
fn test_polygon_with_insufficient_vertices() {
    let vertices = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)),
    ];
    let _polygon = Polygon::new(vertices, None); // Should panic
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
    let v1 = Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0));
    let v2 = Vertex::new(Point3::new(2.0, 2.0, 2.0), Vector3::new(0.0, 1.0, 0.0));
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
    let a = Point3::new(0.0, 0.0, 0.0);
    let b = Point3::new(1.0, 0.0, 0.0);
    let c = Point3::new(0.0, 1.0, 0.0);
    let plane = Plane::from_points(&a, &b, &c);
    assert!(approx_eq(plane.normal.x, 0.0, EPSILON));
    assert!(approx_eq(plane.normal.y, 0.0, EPSILON));
    assert!(approx_eq(plane.normal.z, 1.0, EPSILON));
    assert!(approx_eq(plane.w, 0.0, EPSILON));
}

#[test]
fn test_plane_flip() {
    let mut plane = Plane {
        normal: Vector3::new(0.0, 1.0, 0.0),
        w: 2.0,
    };
    plane.flip();
    assert_eq!(plane.normal, Vector3::new(0.0, -1.0, 0.0));
    assert_eq!(plane.w, -2.0);
}

#[test]
fn test_plane_split_polygon() {
    // Define a plane that splits the XY plane at y=0
    let plane = Plane {
        normal: Vector3::new(0.0, 1.0, 0.0),
        w: 0.0,
    };

    // A polygon that crosses y=0 line: a square from ( -1, -1 ) to (1, 1 )
    let poly = Polygon::new(
        vec![
            Vertex::new(Point3::new(-1.0, -1.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(1.0, -1.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(1.0, 1.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(-1.0, 1.0, 0.0), Vector3::z()),
        ],
        None,
    );

    let mut cf = Vec::new(); // coplanar_front
    let mut cb = Vec::new(); // coplanar_back
    let mut f = Vec::new();  // front
    let mut b = Vec::new();  // back

    plane.split_polygon(&poly, &mut cf, &mut cb, &mut f, &mut b);
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
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
    ];
    let poly = Polygon::new(vertices.clone(), Some("triangle".to_string()));
    assert_eq!(poly.vertices.len(), 3);
    assert_eq!(poly.shared, Some("triangle".to_string()));
    // Plane normal should be +Z for the above points
    assert!(approx_eq(poly.plane.normal.x, 0.0, EPSILON));
    assert!(approx_eq(poly.plane.normal.y, 0.0, EPSILON));
    assert!(approx_eq(poly.plane.normal.z, 1.0, EPSILON));
}

#[test]
fn test_polygon_flip() {
    let mut poly = Polygon::new(
        vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
        ],
        None,
    );
    let plane_normal_before = poly.plane.normal;
    poly.flip();
    // The vertices should be reversed, and normal flipped
    assert_eq!(poly.vertices.len(), 3);
    assert!(approx_eq(poly.plane.normal.x, -plane_normal_before.x, EPSILON));
    assert!(approx_eq(poly.plane.normal.y, -plane_normal_before.y, EPSILON));
    assert!(approx_eq(poly.plane.normal.z, -plane_normal_before.z, EPSILON));
}

#[test]
fn test_polygon_triangulate() {
    // A quad:
    let poly = Polygon::new(
        vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
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
fn test_polygon_subdivide_triangles() {
    // A single triangle (level=1 should produce 4 sub-triangles)
    let poly = Polygon::new(
        vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
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
    let mut poly = Polygon::new(
        vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::zeros()),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::zeros()),
            Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::zeros()),
        ],
        None,
    );
    poly.recalc_plane_and_normals();
    assert!(approx_eq(poly.plane.normal.z, 1.0, EPSILON));
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
    let p = Polygon::new(
        vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
        ],
        None,
    );
    let node = Node::new(vec![p.clone()]);
    // The node should have built a tree with plane = p.plane, polygons = [p], no front/back children
    assert!(node.plane.is_some());
    assert_eq!(node.polygons.len(), 1);
    assert!(node.front.is_none());
    assert!(node.back.is_none());
}

#[test]
fn test_node_invert() {
    let p = Polygon::new(
        vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
        ],
        None,
    );
    let mut node = Node::new(vec![p.clone()]);
    let original_count = node.polygons.len();
    let original_normal = node.plane.as_ref().unwrap().normal;
    node.invert();
    // The plane normal should be flipped, polygons should be flipped, and front/back swapped (they were None).
    let flipped_normal = node.plane.as_ref().unwrap().normal;
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
fn test_node_clip_polygons() {
    // Build a simple BSP from a cube
    let cube = CSG::cube(None);
    let mut flipped_cube = cube.clone();
    for p in &mut flipped_cube.polygons {
        p.flip(); // now plane normals match the Node-building logic
    }
    let node = Node::new(flipped_cube.polygons.clone());
    let clipped = node.clip_polygons(&flipped_cube.polygons);
    assert_eq!(clipped.len(), flipped_cube.polygons.len());
}

#[test]
fn test_node_clip_polygons2() {
    // A node with a single plane normal to +Z, passing through z=0
    let plane = Plane {
        normal: Vector3::z(),
        w: 0.0,
    };
    let mut node = Node {
        plane: Some(plane),
        front: None,
        back: None,
        polygons: Vec::new(),
    };
    // Build the node with some polygons
    // We'll put a polygon in the plane exactly (z=0) and one above, one below
    let poly_in_plane = Polygon::new(
        vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
        ],
        None,
    );
    let poly_above = Polygon::new(
        vec![
            Vertex::new(Point3::new(0.0, 0.0, 1.0), Vector3::z()),
            Vertex::new(Point3::new(1.0, 0.0, 1.0), Vector3::z()),
            Vertex::new(Point3::new(0.0, 1.0, 1.0), Vector3::z()),
        ],
        None,
    );
    let poly_below = Polygon::new(
        vec![
            Vertex::new(Point3::new(0.0, 0.0, -1.0), Vector3::z()),
            Vertex::new(Point3::new(1.0, 0.0, -1.0), Vector3::z()),
            Vertex::new(Point3::new(0.0, 1.0, -1.0), Vector3::z()),
        ],
        None,
    );

    node.build(&[poly_in_plane.clone(), poly_above.clone(), poly_below.clone()]);
    // Now node has polygons: [poly_in_plane], front child with poly_above, back child with poly_below

    // Clip a polygon that crosses from z=-0.5 to z=0.5
    let crossing_poly = Polygon::new(
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
    let poly = Polygon::new(
        vec![
            Vertex::new(Point3::new(-0.5, -0.5, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.5, -0.5, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.0, 0.5, 0.0), Vector3::z()),
        ],
        None,
    );
    let mut nodeA = Node::new(vec![poly.clone()]);
    // Another polygon that fully encloses the above
    let big_poly = Polygon::new(
        vec![
            Vertex::new(Point3::new(-1.0, -1.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(1.0, -1.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(1.0, 1.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(-1.0, 1.0, 0.0), Vector3::z()),
        ],
        None,
    );
    let nodeB = Node::new(vec![big_poly]);
    nodeA.clip_to(&nodeB);
    // We expect nodeA's polygon to remain
    let allA = nodeA.all_polygons();
    assert_eq!(allA.len(), 1);
}

#[test]
fn test_node_all_polygons() {
    // Build a node with multiple polygons
    let poly1 = Polygon::new(
        vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
        ],
        None,
    );
    let poly2 = Polygon::new(
        vec![
            Vertex::new(Point3::new(0.0, 0.0, 1.0), Vector3::z()),
            Vertex::new(Point3::new(1.0, 0.0, 1.0), Vector3::z()),
            Vertex::new(Point3::new(0.0, 1.0, 1.0), Vector3::z()),
        ],
        None,
    );

    let node = Node::new(vec![poly1.clone(), poly2.clone()]);
    let all_polys = node.all_polygons();
    // We expect to retrieve both polygons
    assert_eq!(all_polys.len(), 2);
}

// ------------------------------------------------------------
// CSG tests
// ------------------------------------------------------------
#[test]
fn test_csg_from_polygons_and_to_polygons() {
    let poly = Polygon::new(
        vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
        ],
        None,
    );
    let csg = CSG::from_polygons(vec![poly.clone()]);
    let polys = csg.to_polygons();
    assert_eq!(polys.len(), 1);
    assert_eq!(polys[0].vertices.len(), 3);
}

#[test]
fn test_csg_union() {
    let cube1 = CSG::cube(None); // from -1 to +1 in all coords
    let cube2 = CSG::cube(Some((&[0.5, 0.5, 0.5], &[1.0, 1.0, 1.0])));

    let union_csg = cube1.union(&cube2);
    let polys = union_csg.to_polygons();
    assert!(!polys.is_empty(), "Union of two cubes should produce polygons");

    // Check bounding box => should now at least range from -1 to (0.5+1) = 1.5
    let bb = bounding_box(polys);
    assert!(approx_eq(bb[0], -1.0, 1e-8));
    assert!(approx_eq(bb[1], -1.0, 1e-8));
    assert!(approx_eq(bb[2], -1.0, 1e-8));
    assert!(approx_eq(bb[3], 1.5, 1e-8));
    assert!(approx_eq(bb[4], 1.5, 1e-8));
    assert!(approx_eq(bb[5], 1.5, 1e-8));
}

#[test]
fn test_csg_subtract() {
    // Subtract a smaller cube from a bigger one
    let big_cube = CSG::cube(Some((&[0.0, 0.0, 0.0], &[2.0, 2.0, 2.0]))); // radius=2 => spans [-2,2]
    let small_cube = CSG::cube(None); // radius=1 => spans [-1,1]

    let result = big_cube.subtract(&small_cube);
    let polys = result.to_polygons();
    assert!(!polys.is_empty(), "Subtracting a smaller cube should leave polygons");

    // Check bounding box => should still be [-2,-2,-2, 2,2,2], but with a chunk removed
    let bb = bounding_box(polys);
    // At least the bounding box remains the same
    assert!(approx_eq(bb[0], -2.0, 1e-8));
    assert!(approx_eq(bb[3],  2.0, 1e-8));
}

#[test]
fn test_csg_union2() {
    let c1 = CSG::cube(None); // cube from (-1..+1) if that's how you set radius=1 by default
    let c2 = CSG::sphere(None); // default sphere radius=1
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
    let c1 = CSG::cube(None);
    let c2 = CSG::sphere(None);
    let isect = c1.intersect(&c2);
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
    let sphere = CSG::sphere(None);
    let cube = CSG::cube(None);

    let intersection = sphere.intersect(&cube);
    let polys = intersection.to_polygons();
    assert!(
        !polys.is_empty(),
        "Sphere ∩ Cube should produce the portion of the sphere inside the cube"
    );

    // Check bounding box => intersection is roughly a sphere clipped to [-1,1]^3
    let bb = bounding_box(polys);
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
    let c1 = CSG::cube(None);
    let inv = c1.inverse();
    // The polygons are flipped
    // We can check just that the polygon planes are reversed, etc. 
    // A full check might compare the polygons, but let's do a quick check on one polygon.
    let orig_poly = &c1.polygons[0];
    let inv_poly = &inv.polygons[0];
    assert!(approx_eq(orig_poly.plane.normal.x, -inv_poly.plane.normal.x, EPSILON));
    assert!(approx_eq(orig_poly.plane.normal.y, -inv_poly.plane.normal.y, EPSILON));
    assert!(approx_eq(orig_poly.plane.normal.z, -inv_poly.plane.normal.z, EPSILON));
    assert_eq!(
        c1.to_polygons().len(),
        inv.to_polygons().len(),
        "Inverse should keep the same polygon count, but flip them"
    );
}

#[test]
fn test_csg_cube() {
    let c = CSG::cube(None); 
    // By default, center=(0,0,0) radius=(1,1,1) => corners at ±1
    // We expect 6 faces, each 4 vertices = 6 polygons
    assert_eq!(c.polygons.len(), 6);
    // Check bounding box
    let bb = c.bounding_box();
    assert!(approx_eq(bb.mins.x, -1.0, EPSILON));
    assert!(approx_eq(bb.maxs.x, 1.0, EPSILON));
}

// --------------------------------------------------------
//   CSG: Basic Shape Generation
// --------------------------------------------------------

#[test]
fn test_csg_sphere() {
    // Default sphere => radius=1, slices=16, stacks=8
    let sphere = CSG::sphere(None);
    let polys = sphere.to_polygons();
    assert!(!polys.is_empty(), "Sphere should generate polygons");

    let bb = bounding_box(polys);
    // Should roughly be [-1, -1, -1, 1, 1, 1]
    assert!(approx_eq(bb[0], -1.0, 1e-1));
    assert!(approx_eq(bb[1], -1.0, 1e-1));
    assert!(approx_eq(bb[2], -1.0, 1e-1));
    assert!(approx_eq(bb[3],  1.0, 1e-1));
    assert!(approx_eq(bb[4],  1.0, 1e-1));
    assert!(approx_eq(bb[5],  1.0, 1e-1));
    
    // We expect 16 * 8 polygons = 128 polygons
    // each stack band is 16 polys, times 8 => 128. 
    assert_eq!(sphere.polygons.len(), 16 * 8);
}

#[test]
fn test_csg_cylinder() {
    // Default cylinder => from (0,-1,0) to (0,1,0) with radius=1
    let cylinder = CSG::cylinder(None);
    let polys = cylinder.to_polygons();
    assert!(!polys.is_empty(), "Cylinder should generate polygons");

    let bb = bounding_box(polys);
    // Expect x in [-1,1], y in [-1,1], z in [-1,1].
    assert!(approx_eq(bb[0], -1.0, 1e-8), "min X");
    assert!(approx_eq(bb[1], -1.0, 1e-8), "min Y");
    assert!(approx_eq(bb[2], -1.0, 1e-8), "min Z");
    assert!(approx_eq(bb[3],  1.0, 1e-8), "max X");
    assert!(approx_eq(bb[4],  1.0, 1e-8), "max Y");
    assert!(approx_eq(bb[5],  1.0, 1e-8), "max Z");
    
    // We have slices = 16, each slice has 3 polygons (bottom cap tri, tube quad, top cap tri) => 16 * 3 = 48
    // But the tube quad is actually a single polygon with 4 vertices, not 2 triangles. 
    // So total polygons = 16 * 3 = 48
    assert_eq!(cylinder.polygons.len(), 16 * 3);
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
    let faces = vec![
        vec![0, 1, 2],
        vec![0, 1, 3],
        vec![1, 2, 3],
        vec![2, 0, 3],
    ];
    let csg_tetra = CSG::polyhedron(pts, &faces);
    // We should have exactly 4 triangular faces
    assert_eq!(csg_tetra.polygons.len(), 4);
}

#[test]
fn test_csg_transform_translate_rotate_scale() {
    let c = CSG::cube(None);
    let translated = c.translate(Vector3::new(1.0, 2.0, 3.0));
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
    let c = CSG::cube(None);
    let mirror_x = c.mirror(Axis::X);
    let bb_mx = mirror_x.bounding_box();
    // The original cube was from x=-1..1, so mirrored across X=0 is the same bounding box
    assert!(approx_eq(bb_mx.mins.x, -1.0, EPSILON));
    assert!(approx_eq(bb_mx.maxs.x, 1.0, EPSILON));
}

#[test]
fn test_csg_convex_hull() {
    // If we take a shape with some random points, the hull should just enclose them
    let c1 = CSG::sphere(Some((&[0.0, 0.0, 0.0], 1.0, 8, 4)));
    // The convex_hull of a sphere's sampling is basically that same shape, but let's see if it runs.
    let hull = c1.convex_hull();
    // The hull should have some polygons
    assert!(!hull.polygons.is_empty());
}

#[test]
fn test_csg_minkowski_sum() {
    // Minkowski sum of two cubes => bigger cube offset by edges
    let c1 = CSG::cube(Some((&[0.0, 0.0, 0.0], &[1.0, 1.0, 1.0])));
    let c2 = CSG::cube(Some((&[0.0, 0.0, 0.0], &[0.5, 0.5, 0.5])));
    let sum = c1.minkowski_sum(&c2);
    let bb_sum = sum.bounding_box();
    // Expect bounding box from -1.5..+1.5 in each axis if both cubes were centered at (0,0,0).
    assert!(approx_eq(bb_sum.mins.x, -1.5, 0.01));
    assert!(approx_eq(bb_sum.maxs.x, 1.5, 0.01));
}

#[test]
fn test_csg_subdivide_triangles() {
    let cube = CSG::cube(None);
    // subdivide_triangles(1) => each polygon (quad) is triangulated => 2 triangles => each tri subdivides => 4
    // So each face with 4 vertices => 2 triangles => each becomes 4 => total 8 per face => 6 faces => 48
    let subdiv = cube.subdivide_triangles(1);
    assert_eq!(subdiv.polygons.len(), 6 * 8);
}

#[test]
fn test_csg_renormalize() {
    let mut cube = CSG::cube(None);
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
            assert!(approx_eq(v.normal.x, poly.plane.normal.x, EPSILON));
            assert!(approx_eq(v.normal.y, poly.plane.normal.y, EPSILON));
            assert!(approx_eq(v.normal.z, poly.plane.normal.z, EPSILON));
        }
    }
}

#[test]
fn test_csg_ray_intersections() {
    let cube = CSG::cube(None);
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
    let sq = CSG::square(None);
    // Single polygon, 4 vertices
    assert_eq!(sq.polygons.len(), 1);
    let poly = &sq.polygons[0];
    assert_eq!(poly.vertices.len(), 4);
}

#[test]
fn test_csg_circle() {
    let circle = CSG::circle(None);
    // Single polygon with 32 segments => 32 vertices
    assert_eq!(circle.polygons.len(), 1);
    let poly = &circle.polygons[0];
    assert_eq!(poly.vertices.len(), 32);
}

#[test]
fn test_csg_polygon_2d() {
    let points = &[
        [0.0, 0.0],
        [2.0, 0.0],
        [2.0, 1.0],
        [0.0, 1.0],
    ];
    let poly2d = CSG::polygon_2d(points);
    assert_eq!(poly2d.polygons.len(), 1);
    assert_eq!(poly2d.polygons[0].vertices.len(), 4);
}

#[test]
fn test_csg_extrude() {
    let sq = CSG::square(None); // default 1x1 square at XY plane
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
    // A line from (1,0,0) to (1,1,0) in XY => rotate_extrude => forms a cylindrical shell.
    let line_pts = &[
        [1.0, 0.0],
        [1.0, 1.0],
    ];
    let line_csg = CSG::polygon_2d(line_pts);
    let revolve = line_csg.rotate_extrude(360.0, 16);
    // We expect some ring-like shape with polygons
    assert!(!revolve.polygons.is_empty());
}

#[test]
fn test_csg_bounding_box() {
    let sphere = CSG::sphere(Some((&[2.0, -1.0, 3.0], 2.0, 8, 4)));
    let bb = sphere.bounding_box();
    // center=(2,-1,3), radius=2 => bounding box min=(0,-3,1), max=(4,1,5)
    assert!(approx_eq(bb.mins.x, 0.0, 0.1));
    assert!(approx_eq(bb.mins.y, -3.0, 0.1));
    assert!(approx_eq(bb.mins.z, 1.0, 0.1));
    assert!(approx_eq(bb.maxs.x, 4.0, 0.1));
    assert!(approx_eq(bb.maxs.y, 1.0, 0.1));
    assert!(approx_eq(bb.maxs.z, 5.0, 0.1));
}

#[test]
fn test_csg_vertices() {
    let cube = CSG::cube(None);
    let verts = cube.vertices();
    // 6 faces x 4 vertices each = 24
    assert_eq!(verts.len(), 24);
}

#[test]
fn test_csg_grow_and_shrink() {
    let sq = CSG::square(Some(([1.0, 1.0], true))); // center-based square
    // Grow by 0.5
    let grown = sq.grow(0.5);
    // bounding box should be bigger
    let bb_sq = sq.bounding_box();
    let bb_gr = grown.bounding_box();
    assert!(bb_gr.mins.x < bb_sq.mins.x - 0.1);

    // Similarly test shrink
    let shr = sq.shrink(0.5);
    let bb_sh = shr.bounding_box();
    // bounding box should be smaller
    assert!(bb_sh.mins.x > bb_sq.mins.x - 0.1);
}

#[test]
fn test_csg_grow_2d_and_shrink_2d() {
    let sq = CSG::square(None); 
    let grown = sq.grow_2d(0.5);
    let bb_sq = sq.bounding_box();
    let bb_gr = grown.bounding_box();
    // Should be bigger
    assert!(bb_gr.maxs.x > bb_sq.maxs.x + 0.4);

    let shr = sq.shrink_2d(0.5);
    let bb_sh = shr.bounding_box();
    // Should be smaller
    assert!(bb_sh.maxs.x < bb_sq.maxs.x + 0.1);
}

#[test]
fn test_csg_text_mesh() {
    // We can’t easily test visually, but we can at least test that it doesn’t panic
    // and returns some polygons for normal ASCII letters.
    let font_data = include_bytes!("../asar.ttf");
    let text_csg = CSG::text_mesh("ABC", font_data, Some(10.0));
    assert!(!text_csg.polygons.is_empty());
}

#[test]
fn test_csg_to_trimesh() {
    let cube = CSG::cube(None);
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
    let cube = CSG::cube(None); // side=2 => volume=8. If density=1 => mass=8
    let (mass, com, _frame) = cube.mass_properties(1.0);
    // For a centered cube with side 2, volume=8 => mass=8 => COM=(0,0,0)
    assert!(approx_eq(mass, 8.0, 0.1));
    assert!(approx_eq(com.x, 0.0, 0.001));
    assert!(approx_eq(com.y, 0.0, 0.001));
    assert!(approx_eq(com.z, 0.0, 0.001));
}

#[test]
fn test_csg_to_rigid_body() {
    use rapier3d_f64::prelude::*;
    let cube = CSG::cube(None);
    let mut rb_set = RigidBodySet::new();
    let mut co_set = ColliderSet::new();
    let handle = cube.to_rigid_body(
        &mut rb_set,
        &mut co_set,
        Vector3::new(10.0, 0.0, 0.0),
        Vector3::new(0.0, 0.0, std::f64::consts::FRAC_PI_2), // 90 deg around Z
        1.0,
    );
    let rb = rb_set.get(handle).unwrap();
    let pos = rb.translation();
    assert!(approx_eq(pos.x, 10.0, EPSILON));
}

#[test]
fn test_csg_to_stl_and_from_stl_file() {
    // We'll create a small shape, write to an STL, read it back.
    // You can redirect to a temp file or do an in-memory test.
    let tmp_path = "test_csg_output.stl";

    let cube = CSG::cube(None);
    let res = cube.to_stl_file(tmp_path);
    assert!(res.is_ok());

    let csg_in = CSG::from_stl_file(tmp_path).unwrap();
    // We expect to read the same number of triangular faces as the cube originally had
    // (though the orientation/normals might differ).
    // The default cube -> 6 polygons x 1 polygon each with 4 vertices => 12 triangles in STL.
    // So from_stl_file => we get 12 triangles as 12 polygons (each is a tri).
    assert_eq!(csg_in.polygons.len(), 12);

    // Cleanup the temp file if desired
    let _ = std::fs::remove_file(tmp_path);
}
