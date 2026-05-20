//! Tests for extrusion and slicing behavior.

use super::support::*;

#[test]
fn test_same_number_of_vertices() {
    // "Bottom" is a triangle in 3D
    let bottom = make_polygon_3d(&[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.5, 0.5, 0.0]]);
    // "Top" is the same triangle, shifted up in Z
    let top = make_polygon_3d(&[[0.0, 0.0, 1.0], [1.0, 0.0, 1.0], [0.5, 0.5, 1.0]]);

    // This should succeed with no panic:
    let csg = Profile::loft(&bottom, &top, true).unwrap();

    // Expect:
    //  - bottom polygon
    //  - top polygon
    //  - 3 side polygons (one for each edge of the triangle)
    assert_eq!(
        csg.polygons.len(),
        1 /*bottom*/ + 1 /*top*/ + 3 // sides
    );
}

#[test]
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

    // Call the API and assert the specific error variant is returned
    let result = Profile::loft(&bottom, &top, true);
    assert!(matches!(
        result,
        Err(ValidationError::MismatchedVertexCount { left: 3, right: 4 })
    ));
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

    let csg = Profile::loft(&bottom, &top, false).unwrap();

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

    let csg = Profile::loft(&bottom, &top, false).unwrap();

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
    let csg1 = Profile::loft(&bottom1, &top1, true).unwrap();

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
    let csg2 = Profile::loft(&bottom2, &top2, true).unwrap();

    // Union them
    let unioned = csg1.union(&csg2);

    // Sanity check: union shouldn't be empty
    assert!(!unioned.polygons.is_empty());

    // Its bounding box should span at least from z=0 to z=1.5
    let bbox = unioned.bounding_box();
    assert!(bbox.mins.z <= 0.0 + tolerance());
    assert!(bbox.maxs.z >= 1.5 - tolerance());
}

#[test]
fn test_flatten_cube() {
    // 1) Create a cube from (-1,-1,-1) to (+1,+1,+1)
    let cube = Mesh::<()>::cube(2.0, ());
    // 2) Flatten into the XY plane
    let flattened = cube.flatten();

    // The flattened cube should have 1 polygon1, now in z=0
    assert_eq!(
        flattened.region_profiles().len(),
        1,
        "Flattened cube should have 1 face in z=0"
    );

    // Optional: we can check the bounding box in z-dimension is effectively zero
    let bbox = flattened.bounding_box();
    let thickness = bbox.maxs.z - bbox.mins.z;
    assert!(
        thickness.abs() < tolerance(),
        "Flattened shape should have negligible thickness in z"
    );
}

#[test]
fn test_slice_cylinder() {
    // 1) Create a cylinder (start=-1, end=+1) with radius=1, 32 slices
    let cyl = Mesh::<()>::cylinder(1.0, 2.0, 32, ()).center();
    // 2) Slice at z=0
    let cross_section = cyl.slice(Plane::from_normal(Vector3::z(), 0.0));

    // For a simple cylinder, the cross-section is typically 1 circle polygon
    // (unless the top or bottom also exactly intersect z=0, which they do not in this scenario).
    // So we expect exactly 1 polygon.
    assert_eq!(
        cross_section.region_profiles().len(),
        1,
        "Slicing a cylinder at z=0 should yield exactly 1 cross-section polygon"
    );
    assert_eq!(cross_section.material_contour_count(), 1);

    let profiles = cross_section.region_profiles();
    let profile = profiles
        .first()
        .expect("Cross-section region is not an area shell");
    let vcount = profile.material().points().len().saturating_sub(1);

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
