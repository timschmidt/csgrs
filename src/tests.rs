// tests

#[cfg(test)]
use super::*;
use nalgebra::Vector3;

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
fn test_vertex_interpolate() {
    let v1 = Vertex::new(
        Point3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
    );
    let v2 = Vertex::new(
        Point3::new(10.0, 10.0, 10.0),
        Vector3::new(0.0, 1.0, 0.0),
    );

    let v_mid = v1.interpolate(&v2, 0.5);
    assert!(approx_eq(v_mid.pos.x, 5.0, 1e-8));
    assert!(approx_eq(v_mid.pos.y, 5.0, 1e-8));
    assert!(approx_eq(v_mid.pos.z, 5.0, 1e-8));

    // Normal should also interpolate
    assert!(approx_eq(v_mid.normal.x, 0.5, 1e-8));
    assert!(approx_eq(v_mid.normal.y, 0.5, 1e-8));
    assert!(approx_eq(v_mid.normal.z, 0.0, 1e-8));
}

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

#[test]
fn test_polygon_flip() {
    let v1 = Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::y());
    let v2 = Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::y());
    let v3 = Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::y());
    let mut poly = Polygon::new(vec![v1, v2, v3], None);

    let original_normal = poly.plane.normal;
    poly.flip();
    // The vertex order should be reversed
    assert_eq!(poly.vertices.len(), 3);
    // The plane’s normal should be reversed
    let flipped_normal = poly.plane.normal;
    assert_eq!(flipped_normal, -original_normal);
}

#[test]
fn test_polygon_triangulate() {
    let v1 = Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z());
    let v2 = Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z());
    let v3 = Vertex::new(Point3::new(1.0, 1.0, 0.0), Vector3::z());
    let v4 = Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z());
    let poly = Polygon::new(vec![v1, v2, v3, v4], None);

    let triangles = poly.triangulate();
    assert_eq!(
        triangles.len(),
        2,
        "A quad should triangulate into 2 triangles"
    );
}

// --------------------------------------------------------
//   CSG: Basic Shape Generation
// --------------------------------------------------------

#[test]
fn test_csg_cube() {
    // Default cube is centered at (0,0,0) with radius (1,1,1)
    let cube = CSG::cube(None);
    let polys = cube.to_polygons();
    assert_eq!(polys.len(), 6, "Cube should have 6 faces (polygons)");

    // Check bounding box => from (-1,-1,-1) to (1,1,1)
    let bb = bounding_box(polys);
    for &val in &bb[..3] {
        assert!(approx_eq(val, -1.0, 1e-8));
    }
    for &val in &bb[3..] {
        assert!(approx_eq(val, 1.0, 1e-8));
    }
}

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
}

// --------------------------------------------------------
//   CSG: Operations (union, subtract, intersect)
// --------------------------------------------------------

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
fn test_csg_intersect() {
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
    let cube = CSG::cube(None);
    let inv_cube = cube.inverse();
    assert_eq!(
        inv_cube.to_polygons().len(),
        cube.to_polygons().len(),
        "Inverse should keep the same polygon count, but flip them"
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
fn test_node_invert() {
    let cube = CSG::cube(None);
    let mut node = Node::new(cube.polygons.clone());
    let original_count = node.polygons.len();
    // Invert them
    node.invert();
    // We shouldn’t lose polygons by inverting
    assert_eq!(node.polygons.len(), original_count);
    // If we invert back, we should get the same geometry
    node.invert();
    assert_eq!(node.polygons.len(), original_count);
}

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
