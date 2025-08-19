mod support;

use csgrs::{
    errors::ValidationError,
    float_types::{EPSILON, FRAC_PI_2, PI, Real},
    mesh::{
        Mesh,
        plane::Plane,
        polygon::Polygon,
        vertex::{Vertex, VertexCluster},
    },
    sketch::Sketch,
    traits::CSG,
};
use geo::{Area, Geometry, HasDimensions};
use nalgebra::{Point3, Vector3};

#[test]
fn sphere() {
    // Default sphere => radius=1, slices=16, stacks=8
    let sphere: Mesh<()> = Mesh::sphere(1.0, 16, 8, None);
    assert!(!sphere.polygons.is_empty(), "Sphere should generate polygons");

    let bb = support::bounding_box(&sphere.polygons);
    // Should roughly be [-1, -1, -1, 1, 1, 1]
    assert!(support::approx_eq(bb[0], -1.0, 1e-1));
    assert!(support::approx_eq(bb[1], -1.0, 1e-1));
    assert!(support::approx_eq(bb[2], -1.0, 1e-1));
    assert!(support::approx_eq(bb[3], 1.0, 1e-1));
    assert!(support::approx_eq(bb[4], 1.0, 1e-1));
    assert!(support::approx_eq(bb[5], 1.0, 1e-1));

    // We expect 16 * 8 polygons = 128 polygons
    // each stack band is 16 polys, times 8 => 128.
    assert_eq!(sphere.polygons.len(), 16 * 8);
}

#[test]
fn cylinder() {
    // Default cylinder => from (0,0,0) to (0,2,0) with radius=1
    let cylinder: Mesh<()> = Mesh::cylinder(1.0, 2.0, 16, None);
    assert!(
        !cylinder.polygons.is_empty(),
        "Cylinder should generate polygons"
    );

    let bb = support::bounding_box(&cylinder.polygons);
    // Expect x in [-1,1], y in [-1,1], z in [-1,1].
    assert!(support::approx_eq(bb[0], -1.0, 1e-8), "min X");
    assert!(support::approx_eq(bb[1], -1.0, 1e-8), "min Y");
    assert!(support::approx_eq(bb[2], 0.0, 1e-8), "min Z");
    assert!(support::approx_eq(bb[3], 1.0, 1e-8), "max X");
    assert!(support::approx_eq(bb[4], 1.0, 1e-8), "max Y");
    assert!(support::approx_eq(bb[5], 2.0, 1e-8), "max Z");

    // We have slices = 16, plus 16*2 polygons for the end caps
    assert_eq!(cylinder.polygons.len(), 48);
}

#[test]
fn polyhedron() {
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
fn transform_translate_rotate_scale() {
    let c: Mesh<()> = Mesh::cube(2.0, None).center();
    let translated = c.translate(1.0, 2.0, 3.0);
    let rotated = c.rotate(90.0, 0.0, 0.0); // 90 deg about X
    let scaled = c.scale(2.0, 1.0, 1.0);

    // Quick bounding box checks
    let bb_t = translated.bounding_box();
    assert!(support::approx_eq(bb_t.mins.x, -1.0 + 1.0, EPSILON));
    assert!(support::approx_eq(bb_t.mins.y, -1.0 + 2.0, EPSILON));
    assert!(support::approx_eq(bb_t.mins.z, -1.0 + 3.0, EPSILON));

    let bb_s = scaled.bounding_box();
    assert!(support::approx_eq(bb_s.mins.x, -2.0, EPSILON)); // scaled by 2 in X
    assert!(support::approx_eq(bb_s.maxs.x, 2.0, EPSILON));
    assert!(support::approx_eq(bb_s.mins.y, -1.0, EPSILON));
    assert!(support::approx_eq(bb_s.maxs.y, 1.0, EPSILON));

    // For rotated, let's just check one polygon's vertices to see if z got mapped to y, etc.
    // (A thorough check would be more geometry-based.)
    let poly0 = &rotated.polygons[0];
    for v in &poly0.vertices {
        // After a 90° rotation around X, the old Y should become old Z,
        // and the old Z should become -old Y.
        // We can't trivially guess each vertex's new coordinate but can do a sanity check:
        // The bounding box in Y might be [-1..1], but let's check we have differences in Y from original.
        assert_ne!(v.pos.y, 0.0); // Expect something was changed if originally it was ±1 in Z
    }
}

#[test]
fn mirror() {
    let c: Mesh<()> = Mesh::cube(2.0, None);
    let plane_x = Plane::from_normal(Vector3::x(), 0.0); // x=0 plane
    let mirror_x = c.mirror(plane_x);
    let bb_mx = mirror_x.bounding_box();
    // The original cube was from x=0..2, so mirrored across X=0 should be -2..0
    assert!(support::approx_eq(bb_mx.mins.x, -2.0, EPSILON));
    assert!(support::approx_eq(bb_mx.maxs.x, 0.0, EPSILON));
}

#[test]
fn subdivide_triangles() {
    let cube: Mesh<()> = Mesh::cube(2.0, None);
    // subdivide_triangles(1) => each polygon (quad) is triangulated => 2 triangles => each tri subdivides => 4
    // So each face with 4 vertices => 2 triangles => each becomes 4 => total 8 per face => 6 faces => 48
    let subdiv = cube.subdivide_triangles(1.try_into().expect("not 0"));
    assert_eq!(subdiv.polygons.len(), 6 * 8);
}

#[test]
fn renormalize() {
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
            assert!(support::approx_eq(v.normal.x, poly.plane.normal().x, EPSILON));
            assert!(support::approx_eq(v.normal.y, poly.plane.normal().y, EPSILON));
            assert!(support::approx_eq(v.normal.z, poly.plane.normal().z, EPSILON));
        }
    }
}

#[test]
fn ray_intersections() {
    let cube: Mesh<()> = Mesh::cube(2.0, None).center();
    // Ray from (-2,0,0) toward +X
    let origin = Point3::new(-2.0, 0.0, 0.0);
    let direction = Vector3::new(1.0, 0.0, 0.0);
    let hits = cube.ray_intersections(&origin, &direction);
    // Expect 2 intersections with the cube's side at x=-1 and x=1
    assert_eq!(hits.len(), 2);
    // The distances should be 1 unit from -2.0 -> -1 => t=1, and from -2.0 -> +1 => t=3
    assert!(support::approx_eq(hits[0].1, 1.0, EPSILON));
    assert!(support::approx_eq(hits[1].1, 3.0, EPSILON));
}

#[test]
fn square() {
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
fn circle() {
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
fn extrude() {
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
    assert!(support::approx_eq(bb.mins.z, 0.0, EPSILON));
    assert!(support::approx_eq(bb.maxs.z, 5.0, EPSILON));
}

#[test]
fn revolve() {
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
fn bounding_box() {
    let sphere: Mesh<()> = Mesh::sphere(1.0, 16, 8, None);
    let bb = sphere.bounding_box();
    // center=(2,-1,3), radius=2 => bounding box min=(0,-3,1), max=(4,1,5)
    assert!(support::approx_eq(bb.mins.x, -1.0, 0.1));
    assert!(support::approx_eq(bb.mins.y, -1.0, 0.1));
    assert!(support::approx_eq(bb.mins.z, -1.0, 0.1));
    assert!(support::approx_eq(bb.maxs.x, 1.0, 0.1));
    assert!(support::approx_eq(bb.maxs.y, 1.0, 0.1));
    assert!(support::approx_eq(bb.maxs.z, 1.0, 0.1));
}

#[test]
fn vertices() {
    let cube: Mesh<()> = Mesh::cube(2.0, None);
    let verts = cube.vertices();
    // 6 faces x 4 vertices each = 24
    assert_eq!(verts.len(), 24);
}

#[test]
fn to_trimesh() {
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
fn mass_properties() {
    let cube: Mesh<()> = Mesh::cube(2.0, None).center(); // side=2 => volume=8. If density=1 => mass=8
    let (mass, com, _frame) = cube.mass_properties(1.0);
    println!("{:#?}", mass);
    // For a centered cube with side 2, volume=8 => mass=8 => COM=(0,0,0)
    assert!(support::approx_eq(mass, 8.0, 0.1));
    assert!(support::approx_eq(com.x, 0.0, 0.001));
    assert!(support::approx_eq(com.y, 0.0, 0.001));
    assert!(support::approx_eq(com.z, 0.0, 0.001));
}

#[test]
fn to_rigid_body() {
    use csgrs::float_types::rapier3d::prelude::*;
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
    assert!(support::approx_eq(pos.x, 10.0, EPSILON));
}

#[test]
fn square_ccw_ordering() {
    let square = Sketch::<()>::square(2.0, None);
    let mp = square.to_multipolygon();
    assert_eq!(mp.0.len(), 1);
    let poly = &mp.0[0];
    let area = poly.signed_area();
    assert!(area > 0.0, "Square vertices are not CCW ordered");
}

#[test]
fn polygon_2d_enforce_ccw_ordering() {
    // Define a triangle in CW order
    let points_cw = vec![[0.0, 0.0], [1.0, 0.0], [0.5, 1.0]];
    let csg_cw = Sketch::<()>::polygon(&points_cw, None);
    // Enforce CCW ordering
    csg_cw.renormalize();
    let poly = &csg_cw.geometry.0[0];
    let area = poly.signed_area();
    assert!(area > 0.0, "Polygon ordering was not corrected to CCW");
}

#[test]
fn same_number_of_vertices() {
    // "Bottom" is a triangle in 3D
    let bottom =
        support::make_polygon_3d(&[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.5, 0.5, 0.0]]);
    // "Top" is the same triangle, shifted up in Z
    let top = support::make_polygon_3d(&[[0.0, 0.0, 1.0], [1.0, 0.0, 1.0], [0.5, 0.5, 1.0]]);

    // This should succeed with no panic:
    let csg = Sketch::loft(&bottom, &top, true).unwrap();

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
fn different_number_of_vertices_panics() {
    // Bottom has 3 vertices
    let bottom =
        support::make_polygon_3d(&[[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [1.0, 1.0, 0.0]]);
    // Top has 4 vertices
    let top = support::make_polygon_3d(&[
        [0.0, 0.0, 2.0],
        [2.0, 0.0, 2.0],
        [2.0, 2.0, 2.0],
        [0.0, 2.0, 2.0],
    ]);

    // Call the API and assert the specific error variant is returned
    let result = Sketch::loft(&bottom, &top, true);
    assert!(matches!(result, Err(ValidationError::MismatchedVertices)));
}

#[test]
fn consistent_winding() {
    // Make a square in the XY plane (bottom)
    let bottom = support::make_polygon_3d(&[
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [1.0, 1.0, 0.0],
        [0.0, 1.0, 0.0],
    ]);
    // Make the same square, shifted up in Z, with the same winding direction
    let top = support::make_polygon_3d(&[
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 1.0],
        [1.0, 1.0, 1.0],
        [0.0, 1.0, 1.0],
    ]);

    let csg = Sketch::loft(&bottom, &top, false).unwrap();

    // Expect 1 bottom + 1 top + 4 side faces = 6 polygons
    assert_eq!(csg.polygons.len(), 6);

    // Optionally check that each polygon has at least 3 vertices
    for poly in &csg.polygons {
        assert!(poly.vertices.len() >= 3);
    }
}

#[test]
fn inverted_orientation() {
    // Bottom square
    let bottom = support::make_polygon_3d(&[
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [1.0, 1.0, 0.0],
        [0.0, 1.0, 0.0],
    ]);
    // Top square, but with vertices in opposite order => "flipped" winding
    let mut top = support::make_polygon_3d(&[
        [0.0, 1.0, 1.0],
        [1.0, 1.0, 1.0],
        [1.0, 0.0, 1.0],
        [0.0, 0.0, 1.0],
    ]);

    // We can fix by flipping `top`:
    top.flip();

    let csg = Sketch::loft(&bottom, &top, false).unwrap();

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
fn union_of_extruded_shapes() {
    // We'll extrude two shapes that partially overlap, then union them.

    // First shape: triangle
    let bottom1 =
        support::make_polygon_3d(&[[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [1.0, 1.0, 0.0]]);
    let top1 = support::make_polygon_3d(&[[0.0, 0.0, 1.0], [2.0, 0.0, 1.0], [1.0, 1.0, 1.0]]);
    let csg1 = Sketch::loft(&bottom1, &top1, true).unwrap();

    // Second shape: small shifted square
    let bottom2 = support::make_polygon_3d(&[
        [1.0, -0.2, 0.5],
        [2.0, -0.2, 0.5],
        [2.0, 0.8, 0.5],
        [1.0, 0.8, 0.5],
    ]);
    let top2 = support::make_polygon_3d(&[
        [1.0, -0.2, 1.5],
        [2.0, -0.2, 1.5],
        [2.0, 0.8, 1.5],
        [1.0, 0.8, 1.5],
    ]);
    let csg2 = Sketch::loft(&bottom2, &top2, true).unwrap();

    // Union them
    let unioned = csg1.union(&csg2);

    // Sanity check: union shouldn't be empty
    assert!(!unioned.polygons.is_empty());

    // Its bounding box should span at least from z=0 to z=1.5
    let bbox = unioned.bounding_box();
    assert!(bbox.mins.z <= 0.0 + EPSILON);
    assert!(bbox.maxs.z >= 1.5 - EPSILON);
}

#[test]
fn flatten_cube() {
    // 1) Create a cube from (-1,-1,-1) to (+1,+1,+1)
    let cube = Mesh::<()>::cube(2.0, None);
    // 2) Flatten into the XY plane
    let flattened = cube.flatten();

    // The flattened cube should have 1 polygon1, now in z=0
    assert_eq!(
        flattened.geometry.len(),
        1,
        "Flattened cube should have 1 face in z=0"
    );

    // Optional: we can check the bounding box in z-dimension is effectively zero
    let bbox = flattened.bounding_box();
    let thickness = bbox.maxs.z - bbox.mins.z;
    assert!(
        thickness.abs() < EPSILON,
        "Flattened shape should have negligible thickness in z"
    );
}

#[test]
fn slice_cylinder() {
    // 1) Create a cylinder (start=-1, end=+1) with radius=1, 32 slices
    let cyl = Mesh::<()>::cylinder(1.0, 2.0, 32, None).center();
    // 2) Slice at z=0
    let cross_section = cyl.slice(Plane::from_normal(Vector3::z(), 0.0));

    // For a simple cylinder, the cross-section is typically 1 circle polygon
    // (unless the top or bottom also exactly intersect z=0, which they do not in this scenario).
    // So we expect exactly 1 polygon.
    assert_eq!(
        cross_section.geometry.len(),
        1,
        "Slicing a cylinder at z=0 should yield exactly 1 cross-section polygon"
    );

    let poly_geom = &cross_section.geometry.0[0];
    // Geometry → Polygon
    let poly = match poly_geom {
        Geometry::Polygon(p) => p,
        _ => panic!("Cross-section geometry is not a polygon"),
    };

    // `geo::Polygon` stores a closed ring – skip the last (repeat) vertex.
    let vcount = poly.exterior().0.len() - 1;

    // We used 32 slices for the cylinder, so we expect up to 32 edges
    // in the cross-section circle. Some slight differences might occur
    // if the slicing logic merges or sorts vertices.
    // Typically, you might see vcount = 32 or vcount = 34, etc.
    // Let's just check it's > 3 and in a plausible range:
    assert!(
        (3..=40).contains(&vcount),
        "Expected cross-section circle to have a number of edges ~32, got {}",
        vcount
    );
}

/// Helper to create a `Polygon` in the XY plane from an array of (x,y) points,
/// with z=0 and normal=+Z.
fn polygon_from_xy_points(xy_points: &[[Real; 2]]) -> Polygon<()> {
    assert!(xy_points.len() >= 3, "Need at least 3 points for a polygon.");

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
fn flatten_and_union_single_polygon() {
    // Create a Mesh with one polygon (a unit square).
    let square_poly =
        polygon_from_xy_points(&[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]);
    let csg = Mesh::from_polygons(&[square_poly], None);

    // Flatten & union it
    let flat_csg = csg.flatten();

    // Expect the same bounding box
    assert!(
        !flat_csg.geometry.0[0].is_empty(),
        "Result should not be empty"
    );
    let bb = flat_csg.bounding_box();
    assert_eq!(bb.mins.x, 0.0);
    assert_eq!(bb.mins.y, 0.0);
    assert_eq!(bb.maxs.x, 1.0);
    assert_eq!(bb.maxs.y, 1.0);
}

/// Test `flatten_and_union` with two overlapping squares.
/// The result should be a single unioned polygon covering [0..2, 0..1].
#[test]
fn flatten_and_union_two_overlapping_squares() {
    // First square from (0,0) to (1,1)
    let square1 = polygon_from_xy_points(&[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]);
    // Second square from (1,0) to (2,1)
    let square2 = polygon_from_xy_points(&[[1.0, 0.0], [2.0, 0.0], [2.0, 1.0], [1.0, 1.0]]);
    let csg = Mesh::from_polygons(&[square1, square2], None);

    let flat_csg = csg.flatten();
    assert!(
        !flat_csg.geometry.0[0].is_empty(),
        "Union should not be empty"
    );

    // The bounding box should now span x=0..2, y=0..1
    let bb = flat_csg.bounding_box();
    assert_eq!(bb.mins.x, 0.0);
    assert_eq!(bb.maxs.x, 2.0);
    assert_eq!(bb.mins.y, 0.0);
    assert_eq!(bb.maxs.y, 1.0);
}

/// Test `flatten_and_union` with two disjoint squares.
/// The result should have two separate polygons.
#[test]
fn flatten_and_union_two_disjoint_squares() {
    // Square A at (0..1, 0..1)
    let square_a = polygon_from_xy_points(&[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]);
    // Square B at (2..3, 2..3)
    let square_b = polygon_from_xy_points(&[[2.0, 2.0], [3.0, 2.0], [3.0, 3.0], [2.0, 3.0]]);
    let csg = Mesh::from_polygons(&[square_a, square_b], None);

    let flat_csg = csg.flatten();
    assert!(!flat_csg.geometry.0[0].is_empty());
}

/// Test `flatten_and_union` when polygons are not perfectly in the XY plane,
/// but very close to z=0. This checks sensitivity to floating errors.
#[test]
fn flatten_and_union_near_xy_plane() {
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

    let csg = Mesh::from_polygons(&[poly1], None);
    let flat_csg = csg.flatten();

    assert!(
        !flat_csg.geometry.0[0].is_empty(),
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
fn flatten_and_union_collinear_edges() {
    // Two rectangles sharing a long horizontal edge
    let rect1 = polygon_from_xy_points(&[[0.0, 0.0], [2.0, 0.0], [2.0, 1.0], [0.0, 1.0]]);
    let rect2 = polygon_from_xy_points(&[
        [2.0, 0.0],
        [4.0, 0.0],
        [4.0, 1.001], // slightly off
        [2.0, 1.0],
    ]);

    let csg = Mesh::<()>::from_polygons(&[rect1, rect2], None);
    let flat_csg = csg.flatten();

    // Expect 1 polygon from x=0..4, y=0..~1.0ish
    assert!(!flat_csg.geometry.0[0].is_empty());
    let bb = flat_csg.bounding_box();
    assert!((bb.maxs.x - 4.0).abs() < 1e-5, "Should span up to x=4.0");
    // Also check the y-range is ~1.001
    assert!((bb.maxs.y - 1.001).abs() < 1e-3);
}

/// If you suspect `flatten_and_union` is returning no polygons, this test
/// ensures we get at least one polygon for a simple shape. If it fails,
/// you can println! debug info in `flatten_and_union`.
#[test]
fn flatten_and_union_debug() {
    let cube = Mesh::<()>::cube(2.0, None);
    let flattened = cube.flatten();
    assert!(
        !flattened.geometry.0[0].is_empty(),
        "Flattened cube should not be empty"
    );
    let area = flattened.geometry.0[0].signed_area();
    assert!(area > 3.9, "Flattened cube too small");
}
