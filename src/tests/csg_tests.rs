use super::support::*;

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
    let csg: Mesh<()> = Mesh::from_polygons(&[poly.clone()], None);
    assert_eq!(csg.polygons.len(), 1);
    assert_eq!(csg.polygons[0].vertices.len(), 3);
}

#[test]
fn test_csg_union() {
    let cube1: Mesh<()> = Mesh::cube(2.0, None).translate(-1.0, -1.0, -1.0); // from -1 to +1 in all coords
    let cube2: Mesh<()> = Mesh::cube(1.0, None).translate(0.5, 0.5, 0.5);

    let union_csg = cube1.union(&cube2);
    assert!(
        !union_csg.polygons.is_empty(),
        "Union of two cubes should produce polygons"
    );

    // Check bounding box => should now at least range from -1 to (0.5+1) = 1.5
    let bb = bounding_box(&union_csg.polygons);
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
    let big_cube: Mesh<()> = Mesh::cube(4.0, None).translate(-2.0, -2.0, -2.0); // radius=2 => spans [-2,2]
    let small_cube: Mesh<()> = Mesh::cube(2.0, None).translate(-1.0, -1.0, -1.0); // radius=1 => spans [-1,1]

    let result = big_cube.difference(&small_cube);
    assert!(
        !result.polygons.is_empty(),
        "Subtracting a smaller cube should leave polygons"
    );

    // Check bounding box => should still be [-2,-2,-2, 2,2,2], but with a chunk removed
    let bb = bounding_box(&result.polygons);
    // At least the bounding box remains the same
    assert!(approx_eq(bb[0], -2.0, 1e-8));
    assert!(approx_eq(bb[3], 2.0, 1e-8));
}

#[test]
fn test_csg_union2() {
    let c1: Mesh<()> = Mesh::cube(2.0, None); // cube from (-1..+1) if that's how you set radius=1 by default
    let c2: Mesh<()> = Mesh::sphere(1.0, 16, 8, None); // default sphere radius=1
    let unioned = c1.union(&c2);
    // We can check bounding box is bigger or at least not smaller than either shape's box
    let bb_union = unioned.bounding_box();
    let bb_cube = c1.bounding_box();
    let bb_sphere = c2.bounding_box();
    assert!(bb_union.mins.x <= bb_cube.mins.x.min(bb_sphere.mins.x));
    assert!(bb_union.maxs.x >= bb_cube.maxs.x.max(bb_sphere.maxs.x));
}

#[test]
fn test_csg_intersect() {
    let c1: Mesh<()> = Mesh::cube(2.0, None);
    let c2: Mesh<()> = Mesh::sphere(1.0, 16, 8, None);
    let isect = c1.intersection(&c2);
    let bb_isect = isect.bounding_box();
    // The intersection bounding box should be smaller than or equal to each
    let bb_cube = c1.bounding_box();
    let bb_sphere = c2.bounding_box();
    assert!(bb_isect.mins.x >= bb_cube.mins.x - tolerance());
    assert!(bb_isect.mins.x >= bb_sphere.mins.x - tolerance());
    assert!(bb_isect.maxs.x <= bb_cube.maxs.x + tolerance());
    assert!(bb_isect.maxs.x <= bb_sphere.maxs.x + tolerance());
}

#[test]
fn test_csg_intersect2() {
    let sphere: Mesh<()> = Mesh::sphere(1.0, 16, 8, None);
    let cube: Mesh<()> = Mesh::cube(2.0, None);

    let intersection = sphere.intersection(&cube);
    assert!(
        !intersection.polygons.is_empty(),
        "Sphere ∩ Cube should produce the portion of the sphere inside the cube"
    );

    // Check bounding box => intersection is roughly a sphere clipped to [-1,1]^3
    let bb = bounding_box(&intersection.polygons);
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
    let c1: Mesh<()> = Mesh::cube(2.0, None);
    let inv = c1.inverse();
    // The polygons are flipped
    // We can check just that the polygon planes are reversed, etc.
    // A full check might compare the polygons, but let's do a quick check on one polygon.
    let orig_poly = &c1.polygons[0];
    let inv_poly = &inv.polygons[0];
    assert!(approx_eq(
        orig_poly.plane.normal().x,
        -inv_poly.plane.normal().x,
        tolerance()
    ));
    assert!(approx_eq(
        orig_poly.plane.normal().y,
        -inv_poly.plane.normal().y,
        tolerance()
    ));
    assert!(approx_eq(
        orig_poly.plane.normal().z,
        -inv_poly.plane.normal().z,
        tolerance()
    ));
    assert_eq!(
        c1.polygons.len(),
        inv.polygons.len(),
        "Inverse should keep the same polygon count, but flip them"
    );
}

#[test]
fn test_csg_cube() {
    let c: Mesh<()> = Mesh::cube(2.0, None);
    // By default, corner at (0,0,0)
    // We expect 6 faces, each 4 vertices = 6 polygons
    assert_eq!(c.polygons.len(), 6);
    // Check bounding box
    let bb = c.bounding_box();
    assert!(approx_eq(bb.mins.x, 0.0, tolerance()));
    assert!(approx_eq(bb.maxs.x, 2.0, tolerance()));
}

// --------------------------------------------------------
//   CSG: Basic Shape Generation
// --------------------------------------------------------

#[test]
fn test_csg_sphere() {
    // Default sphere => radius=1, slices=16, stacks=8
    let sphere: Mesh<()> = Mesh::sphere(1.0, 16, 8, None);
    assert!(!sphere.polygons.is_empty(), "Sphere should generate polygons");

    let bb = bounding_box(&sphere.polygons);
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
    let cylinder: Mesh<()> = Mesh::cylinder(1.0, 2.0, 16, None);
    assert!(
        !cylinder.polygons.is_empty(),
        "Cylinder should generate polygons"
    );

    let bb = bounding_box(&cylinder.polygons);
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
    let faces: [&[usize]; 4] = [&[0, 1, 2], &[0, 1, 3], &[1, 2, 3], &[2, 0, 3]];
    let csg_tetra: Mesh<()> = Mesh::polyhedron(pts, &faces, None).unwrap();
    // We should have exactly 4 triangular faces
    assert_eq!(csg_tetra.polygons.len(), 4);
}

#[test]
fn test_csg_transform_translate_rotate_scale() {
    let c: Mesh<()> = Mesh::cube(2.0, None).center();
    let translated = c.translate(1.0, 2.0, 3.0);
    let rotated = c.rotate(90.0, 0.0, 0.0); // 90 deg about X
    let scaled = c.scale(2.0, 1.0, 1.0);

    // Quick bounding box checks
    let bb_t = translated.bounding_box();
    assert!(approx_eq(bb_t.mins.x, -1.0 + 1.0, tolerance()));
    assert!(approx_eq(bb_t.mins.y, -1.0 + 2.0, tolerance()));
    assert!(approx_eq(bb_t.mins.z, -1.0 + 3.0, tolerance()));

    let bb_s = scaled.bounding_box();
    assert!(approx_eq(bb_s.mins.x, -2.0, tolerance())); // scaled by 2 in X
    assert!(approx_eq(bb_s.maxs.x, 2.0, tolerance()));
    assert!(approx_eq(bb_s.mins.y, -1.0, tolerance()));
    assert!(approx_eq(bb_s.maxs.y, 1.0, tolerance()));

    // For rotated, let's just check one polygon's vertices to see if z got mapped to y, etc.
    // (A thorough check would be more geometry-based.)
    let poly0 = &rotated.polygons[0];
    for v in &poly0.vertices {
        // After a 90° rotation around X, the old Y should become old Z,
        // and the old Z should become -old Y.
        // We can't trivially guess each vertex's new coordinate but can do a sanity check:
        // The bounding box in Y might be [-1..1], but let's check we have differences in Y from original.
        assert_ne!(v.position.y, 0.0); // Expect something was changed if originally it was ±1 in Z
    }
}

#[test]
fn test_csg_mirror() {
    let c: Mesh<()> = Mesh::cube(2.0, None);
    let plane_x = Plane::from_normal(Vector3::x(), 0.0); // x=0 plane
    let mirror_x = c.mirror(plane_x);
    let bb_mx = mirror_x.bounding_box();
    // The original cube was from x=0..2, so mirrored across X=0 should be -2..0
    assert!(approx_eq(bb_mx.mins.x, -2.0, tolerance()));
    assert!(approx_eq(bb_mx.maxs.x, 0.0, tolerance()));
}

#[test]
#[cfg(feature = "chull-io")]
fn test_csg_convex_hull() {
    // If we take a shape with some random points, the hull should just enclose them
    let c1: Mesh<()> = Mesh::sphere(1.0, 16, 8, None);
    // The convex_hull of a sphere's sampling is basically that same shape, but let's see if it runs.
    let hull = c1.convex_hull();
    // The hull should have some polygons
    assert!(!hull.polygons.is_empty());
}

#[test]
#[cfg(feature = "chull-io")]
fn test_csg_minkowski_sum() {
    // Minkowski sum of two cubes => bigger cube offset by edges
    let c1: Mesh<()> = Mesh::cube(2.0, None).center();
    let c2: Mesh<()> = Mesh::cube(1.0, None).center();
    let sum = c1.minkowski_sum(&c2);
    let bb_sum = sum.bounding_box();
    // Expect bounding box from -1.5..+1.5 in each axis if both cubes were centered at (0,0,0).
    assert!(approx_eq(bb_sum.mins.x, -1.5, 0.01));
    assert!(approx_eq(bb_sum.maxs.x, 1.5, 0.01));
}

#[test]
fn test_csg_subdivide_triangles() {
    let cube: Mesh<()> = Mesh::cube(2.0, None);
    // subdivide_triangles(1) => each polygon (quad) is triangulated => 2 triangles => each tri subdivides => 4
    // So each face with 4 vertices => 2 triangles => each becomes 4 => total 8 per face => 6 faces => 48
    let subdiv = cube.subdivide_triangles(1.try_into().expect("not 0"));
    assert_eq!(subdiv.polygons.len(), 6 * 8);
}

#[test]
fn test_csg_renormalize() {
    let mut cube: Mesh<()> = Mesh::cube(2.0, None);
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
            assert!(approx_eq(v.normal.x, poly.plane.normal().x, tolerance()));
            assert!(approx_eq(v.normal.y, poly.plane.normal().y, tolerance()));
            assert!(approx_eq(v.normal.z, poly.plane.normal().z, tolerance()));
        }
    }
}

#[test]
fn test_csg_ray_intersections() {
    let cube: Mesh<()> = Mesh::cube(2.0, None).center();
    // Ray from (-2,0,0) toward +X
    let origin = Point3::new(-2.0, 0.0, 0.0);
    let direction = Vector3::new(1.0, 0.0, 0.0);
    let hits = cube.ray_intersections(&origin, &direction);
    // Expect 2 intersections with the cube's side at x=-1 and x=1
    assert_eq!(hits.len(), 2);
    // The distances should be 1 unit from -2.0 -> -1 => t=1, and from -2.0 -> +1 => t=3
    assert!(approx_eq(hits[0].1, 1.0, tolerance()));
    assert!(approx_eq(hits[1].1, 3.0, tolerance()));
}

#[test]
fn test_csg_square() {
    let sq: Sketch<()> = Sketch::square(2.0, None);
    let mesh_2d: Mesh<()> = sq.into();
    // Single polygon, 4 vertices
    assert_eq!(mesh_2d.polygons.len(), 1);
    let poly = &mesh_2d.polygons[0];
    assert!(
        matches!(poly.vertices.len(), 4 | 5),
        "Expected 4 or 5 vertices, got {}",
        poly.vertices.len()
    );
}

#[test]
fn test_csg_circle() {
    let circle: Sketch<()> = Sketch::circle(2.0, 32, None);
    let mesh_2d: Mesh<()> = circle.into();
    // Single polygon with 32 segments => 32 or 33 vertices if closed
    assert_eq!(mesh_2d.polygons.len(), 1);
    let poly = &mesh_2d.polygons[0];
    assert!(
        matches!(poly.vertices.len(), 32 | 33),
        "Expected 32 or 33 vertices, got {}",
        poly.vertices.len()
    );
}

#[test]
fn test_csg_extrude() {
    let sq: Sketch<()> = Sketch::square(2.0, None);
    let extruded = sq.extrude(5.0);
    // We expect:
    //   bottom polygon: 2 (square triangulated)
    //   top polygon 2 (square triangulated)
    //   side polygons: 4 for a square (one per edge)
    // => total 8 polygons
    assert_eq!(extruded.polygons.len(), 8);
    // Check bounding box
    let bb = extruded.bounding_box();
    assert!(approx_eq(bb.mins.z, 0.0, tolerance()));
    assert!(approx_eq(bb.maxs.z, 5.0, tolerance()));
}

#[test]
fn test_csg_revolve() {
    // Default square is from (0,0) to (1,1) in XY.
    // Shift it so it's from (1,0) to (2,1) — i.e. at least 1.0 unit away from the Z-axis.
    // and rotate it 90 degrees so that it can be swept around Z
    let square: Sketch<()> = Sketch::square(2.0, None)
        .translate(1.0, 0.0, 0.0)
        .rotate(90.0, 0.0, 0.0);

    // Now revolve this translated square around the Z-axis, 360° in 16 segments.
    let revolve = square.revolve(360.0, 16).unwrap();

    // We expect a ring-like “tube” instead of a degenerate shape.
    assert!(!revolve.polygons.is_empty());
}

#[test]
fn test_csg_bounding_box() {
    let sphere: Mesh<()> = Mesh::sphere(1.0, 16, 8, None);
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
    let cube: Mesh<()> = Mesh::cube(2.0, None);
    let verts = cube.vertices();
    // 6 faces x 4 vertices each = 24
    assert_eq!(verts.len(), 24);
}

#[test]
#[cfg(feature = "offset")]
fn test_csg_offset_2d() {
    let square: Sketch<()> = Sketch::square(2.0, None);
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

#[cfg(feature = "truetype-text")]
#[test]
fn test_csg_text() {
    // We can't easily test visually, but we can at least test that it doesn't panic
    // and returns some polygons for normal ASCII letters.
    let font_data = include_bytes!("../../asar.ttf");
    let text_csg: Sketch<()> = Sketch::text("ABC", font_data, 10.0, None);
    assert!(!text_csg.geometry.is_empty());
}

#[test]
fn test_csg_to_trimesh() {
    let cube: Mesh<()> = Mesh::cube(2.0, None);
    let shape = cube.to_trimesh();
    // Should be a TriMesh with 12 triangles
    if let Some(trimesh) = shape {
        assert_eq!(trimesh.indices().len(), 12); // 6 faces => 2 triangles each => 12
    } else {
        panic!("Expected a TriMesh");
    }
}

#[test]
fn test_csg_mass_properties() {
    let cube: Mesh<()> = Mesh::cube(2.0, None).center(); // side=2 => volume=8. If density=1 => mass=8
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
    let cube: Mesh<()> = Mesh::cube(2.0, None);
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
    assert!(approx_eq(pos.x, 10.0, tolerance()));
}

#[ignore = "TODO: impl Mesh::from_stl"] #[cfg(any())]
#[test]
#[cfg(feature = "stl-io")]
fn test_csg_to_stl_and_from_stl_file() -> Result<(), Box<dyn std::error::Error>> {
    // We'll create a small shape, write to an STL, read it back.
    // You can redirect to a temp file or do an in-memory test.
    let tmp_path = "test_csg_output.stl";

    let cube: Mesh<()> = Mesh::cube(2.0, None);
    let res = cube.to_stl_binary("A cube");
    let _ = std::fs::write(tmp_path, res.as_ref().unwrap());
    assert!(res.is_ok());

    let stl_data: Vec<u8> = std::fs::read(tmp_path)?;
    let csg_in: Mesh<()> = Mesh::from_stl(&stl_data, None)?;
    // We expect to read the same number of triangular faces as the cube originally had
    // (though the orientation/normals might differ).
    // The default cube -> 6 polygons x 1 polygon each with 4 vertices => 12 triangles in STL.
    // So from_stl_file => we get 12 triangles as 12 polygons (each is a tri).
    assert_eq!(csg_in.polygons.len(), 12);

    // Cleanup the temp file if desired
    let _ = std::fs::remove_file(tmp_path);
    Ok(())
}
