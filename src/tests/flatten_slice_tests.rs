use super::support::*;

#[test]
fn test_flatten_and_union_single_polygon() {
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
fn test_flatten_and_union_two_overlapping_squares() {
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
fn test_flatten_and_union_two_disjoint_squares() {
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
fn test_flatten_and_union_collinear_edges() {
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
fn test_flatten_and_union_debug() {
    let cube = Mesh::<()>::cube(2.0, None);
    let flattened = cube.flatten();
    assert!(
        !flattened.geometry.0[0].is_empty(),
        "Flattened cube should not be empty"
    );
    let area = flattened.geometry.0[0].signed_area();
    assert!(area > 3.9, "Flattened cube too small");
}
