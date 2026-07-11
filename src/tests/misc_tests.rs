//! Miscellaneous regression and integration tests.

use super::support::*;

#[test]
fn test_cube_basics() {
    let cube: Mesh<()> = Mesh::cube(r(2.0), ());

    // A cube should have 6 faces
    assert_eq!(cube.polygons.len(), 6);

    // Each face should have 4 vertices
    for poly in &cube.polygons {
        assert_eq!(poly.vertices.len(), 4);
    }

    // Check bounding box dimensions (cube should be 2x2x2)
    let bbox = cube.bounding_box();
    let width = bbox.maxs.x - bbox.mins.x;
    let height = bbox.maxs.y - bbox.mins.y;
    let depth = bbox.maxs.z - bbox.mins.z;

    assert!((width - r(2.0)).abs() < r(1e-10), "Width should be 2.0");
    assert!((height - r(2.0)).abs() < r(1e-10), "Height should be 2.0");
    assert!((depth - r(2.0)).abs() < r(1e-10), "Depth should be 2.0");
}

#[test]
fn test_cube_intersection() {
    let cube1: Mesh<()> = Mesh::cube(r(2.0), ());
    let cube2: Mesh<()> = Mesh::cube(r(2.0), ()).translate(r(1.0), r(0.0), r(0.0));

    let intersection = cube1.intersection(&cube2);

    // The intersection should have some polygons
    assert!(
        !intersection.polygons.is_empty(),
        "Intersection should produce some polygons"
    );

    // Check that intersection bounding box is reasonable
    let bbox = intersection.bounding_box();
    let width = bbox.maxs.x - bbox.mins.x;
    assert!(
        width > r(0.0) && width < r(2.0),
        "Intersection width should be between 0 and 2"
    );
}

#[test]
fn test_mesh_intersect_polyline_hits_cube() {
    let cube: Mesh<()> = Mesh::cube(r(2.0), ());
    let hits = cube.intersect_polyline(&[p3(-2.0, 0.0, 0.0), p3(2.0, 0.0, 0.0)]);

    assert!(
        hits.len() >= 2,
        "Polyline crossing cube should hit the entry and exit surfaces"
    );
}

#[test]
fn test_negative_extrude_has_downward_top_normal() {
    let square = Profile::square(r(2.0));
    let mesh = square.extrude_vector(v3(0.0, 0.0, -1.0), ());

    assert!(
        mesh.polygons
            .iter()
            .any(|poly| poly.plane.normal().dot(&v3(0.0, 0.0, -1.0)) > r(0.9)),
        "Negative extrusion should produce a downward-facing translated cap"
    );
}

#[test]
fn test_taubin_smoothing() {
    let mesh: Mesh<()> = Mesh::cube(r(1.0), ());
    let original_positions: Vec<_> = mesh
        .polygons
        .iter()
        .flat_map(|poly| poly.vertices.iter().map(|v| v.position.clone()))
        .collect();

    // Apply Taubin smoothing
    let smoothed = mesh.taubin_smooth(r(0.1), r(-0.105), 2, false);

    // Mesh should have same number of polygons
    assert_eq!(
        smoothed.polygons.len(),
        mesh.polygons.len(),
        "Smoothing should preserve polygon count"
    );

    // At least some vertices should have moved
    let smoothed_positions: Vec<_> = smoothed
        .polygons
        .iter()
        .flat_map(|poly| poly.vertices.iter().map(|v| v.position.clone()))
        .collect();

    let mut moved_count = 0;
    for (orig, smooth) in original_positions.iter().zip(smoothed_positions.iter()) {
        if (orig - smooth).norm() > r(1e-10) {
            moved_count += 1;
        }
    }
    assert!(
        moved_count > 0,
        "Taubin smoothing should change vertex positions"
    );
}
