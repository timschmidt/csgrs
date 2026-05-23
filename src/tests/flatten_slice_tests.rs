//! Tests for flattening and slice conversion helpers.

use super::support::*;

#[test]
fn test_flatten_and_union_single_polygon() {
    // Create a Mesh with one polygon (a unit square).
    let square_poly =
        polygon_from_xy_points(&[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]);
    let csg = Mesh::from_polygons(&[square_poly], ());

    // Flatten & union it
    let flat_csg = csg.flatten();

    // Expect the same bounding box
    assert!(!flat_csg.as_region().is_empty(), "Result should not be empty");
    let bb = flat_csg.bounding_box();
    assert_eq!(bb.mins.x, r(0.0));
    assert_eq!(bb.mins.y, r(0.0));
    assert_eq!(bb.maxs.x, r(1.0));
    assert_eq!(bb.maxs.y, r(1.0));
}

#[test]
fn flatten_promotes_union_projection_to_hypercurve_region() {
    let square_poly =
        polygon_from_xy_points(&[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]);
    let csg = Mesh::from_polygons(&[square_poly], ());

    let flat = csg.flatten();

    assert!(!flat.as_region().is_empty());
    assert_eq!(flat.material_contour_count(), 1);
    assert!(flat.contains_xy(hr(0.5), hr(0.5)).unwrap());

    assert!(
        !flat.region_profiles().is_empty(),
        "flattened projection should regenerate finite output from native Region2"
    );
    assert!(
        !flat.triangulate().is_empty(),
        "flattened projection should triangulate from native Region2 after cache removal"
    );
}

#[test]
#[cfg(feature = "mesh")]
fn sketch_from_mesh_uses_same_hypercurve_flatten_path() {
    let square_poly =
        polygon_from_xy_points(&[[0.0, 0.0], [2.0, 0.0], [2.0, 1.0], [0.0, 1.0]]);
    let csg = Mesh::from_polygons(&[square_poly], ());

    let sketch = Profile::from(csg);

    assert!(!sketch.as_region().is_empty());
    assert_eq!(sketch.material_contour_count(), 1);
    assert!(sketch.contains_xy(hr(1.0), hr(0.5)).unwrap());

    let bbox = sketch.bounding_box();
    assert_eq!(bbox.mins.x, r(0.0));
    assert_eq!(bbox.maxs.x, r(2.0));
    assert!(
        !sketch.region_profiles().is_empty(),
        "Profile::from(mesh) should not depend on retained finite union geometry"
    );
}

#[test]
fn slice_open_intersection_chain_is_native_hypercurve_wire() {
    let polygon = Polygon::new(
        vec![
            Vertex::new(p3(-1.0, 0.0, -1.0), Vector3::z()),
            Vertex::new(p3(1.0, 0.0, 1.0), Vector3::z()),
            Vertex::new(p3(0.0, 1.0, 1.0), Vector3::z()),
        ],
        (),
    );
    let mesh = Mesh::from_polygons(&[polygon], ());

    let section = mesh.slice(Plane::from_normal(Vector3::z(), r(0.0)));

    assert!(section.as_region().is_empty());
    assert_eq!(section.wires().len(), 1);
    assert_eq!(section.wire_polylines()[0].len(), 2);

    assert!(
        !section.wire_polylines().is_empty()
            && section.wire_polylines().iter().all(|wire| wire.len() >= 2),
        "open slice should regenerate finite output from native CurveString2 wires"
    );
}

/// Test `flatten_and_union` with two overlapping squares.
/// The result should be a single unioned polygon covering [0..2, 0..1].
#[test]
fn test_flatten_and_union_two_overlapping_squares() {
    // First square from (0,0) to (1,1)
    let square1 = polygon_from_xy_points(&[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]);
    // Second square from (1,0) to (2,1)
    let square2 = polygon_from_xy_points(&[[1.0, 0.0], [2.0, 0.0], [2.0, 1.0], [1.0, 1.0]]);
    let csg = Mesh::from_polygons(&[square1, square2], ());

    let flat_csg = csg.flatten();
    assert!(!flat_csg.as_region().is_empty(), "Union should not be empty");

    // The bounding box should now span x=0..2, y=0..1
    let bb = flat_csg.bounding_box();
    assert_eq!(bb.mins.x, r(0.0));
    assert_eq!(bb.maxs.x, r(2.0));
    assert_eq!(bb.mins.y, r(0.0));
    assert_eq!(bb.maxs.y, r(1.0));
}

#[test]
fn flatten_union_normalizes_opposite_projected_winding() {
    let ccw = polygon_from_xy_points(&[[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]]);
    let cw = polygon_from_xy_points(&[[0.0, 0.0], [0.0, 2.0], [2.0, 2.0], [2.0, 0.0]]);
    let mesh = Mesh::from_polygons(&[ccw, cw], ());

    let flat = mesh.flatten();

    assert!(flat.contains_xy(hr(1.0), hr(1.0)).unwrap());
    assert_eq!(flat.material_contour_count(), 1);
    let bounds = flat.bounding_box();
    assert_eq!(bounds.mins.x, r(0.0));
    assert_eq!(bounds.mins.y, r(0.0));
    assert_eq!(bounds.maxs.x, r(2.0));
    assert_eq!(bounds.maxs.y, r(2.0));
}

/// Test `flatten_and_union` with two disjoint squares.
/// The result should have two separate polygons.
#[test]
fn test_flatten_and_union_two_disjoint_squares() {
    // Square A at (0..1, 0..1)
    let square_a = polygon_from_xy_points(&[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]);
    // Square B at (2..3, 2..3)
    let square_b = polygon_from_xy_points(&[[2.0, 2.0], [3.0, 2.0], [3.0, 3.0], [2.0, 3.0]]);
    let csg = Mesh::from_polygons(&[square_a, square_b], ());

    let flat_csg = csg.flatten();
    assert!(!flat_csg.as_region().is_empty());
}

/// Test `flatten_and_union` when polygons are not perfectly in the XY plane,
/// but very close to z=0. This checks sensitivity to floating errors.
#[test]
fn test_flatten_and_union_near_xy_plane() {
    let normal = Vector3::z();
    // Slightly "tilted" or with z=1e-6
    let poly1 = Polygon::<()>::new(
        vec![
            Vertex::new(p3(0.0, 0.0, 1e-6), normal.clone()),
            Vertex::new(p3(1.0, 0.0, 1e-6), normal.clone()),
            Vertex::new(p3(1.0, 1.0, 1e-6), normal.clone()),
            Vertex::new(p3(0.0, 1.0, 1e-6), normal),
        ],
        (),
    );

    let csg = Mesh::from_polygons(&[poly1], ());
    let flat_csg = csg.flatten();

    assert!(
        !flat_csg.as_region().is_empty(),
        "Should flatten to a valid polygon"
    );
    let bb = flat_csg.bounding_box();
    assert_eq!(bb.mins.x, r(0.0));
    assert_eq!(bb.maxs.x, r(1.0));
    assert_eq!(bb.mins.y, r(0.0));
    assert_eq!(bb.maxs.y, r(1.0));
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

    let csg = Mesh::<()>::from_polygons(&[rect1, rect2], ());
    let flat_csg = csg.flatten();

    // Expect 1 polygon from x=0..4, y=0..~1.0ish
    assert!(!flat_csg.as_region().is_empty());
    let bb = flat_csg.bounding_box();
    assert!(
        (bb.maxs.x - r(4.0)).abs() < r(1e-5),
        "Should span up to x=4.0"
    );
    // Also check the y-range is ~1.001
    assert!((bb.maxs.y - r(1.001)).abs() < r(1e-3));
}

/// If you suspect `flatten_and_union` is returning no polygons, this test
/// ensures we get at least one polygon for a simple shape. If it fails,
/// you can println! debug info in `flatten_and_union`.
#[test]
fn test_flatten_and_union_debug() {
    let cube = Mesh::<()>::cube(r(2.0), ());
    let flattened = cube.flatten();
    assert!(
        !flattened.as_region().is_empty(),
        "Flattened cube should not be empty"
    );
    let area = flattened
        .region_profiles()
        .iter()
        .map(|profile| {
            r(hypercurve::finite_ring_signed_area(profile.material().points()).abs())
        })
        .sum::<Real>();
    assert!(area > r(3.9), "Flattened cube too small");
}
