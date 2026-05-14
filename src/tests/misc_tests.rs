use super::support::*;

#[test]
fn test_cube_basics() {
    let cube: Mesh<()> = Mesh::cube(2.0, None);

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

    assert!((width - 2.0).abs() < 1e-10, "Width should be 2.0");
    assert!((height - 2.0).abs() < 1e-10, "Height should be 2.0");
    assert!((depth - 2.0).abs() < 1e-10, "Depth should be 2.0");
}

#[test]
fn test_cube_intersection() {
    let cube1: Mesh<()> = Mesh::cube(2.0, None);
    let cube2: Mesh<()> = Mesh::cube(2.0, None).translate(1.0, 0.0, 0.0);

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
        width > 0.0 && width < 2.0,
        "Intersection width should be between 0 and 2"
    );
}

#[test]
fn test_mesh_intersect_polyline_hits_cube() {
    let cube: Mesh<()> = Mesh::cube(2.0, None);
    let hits = cube.intersect_polyline(&[
        Point3::new(-2.0, 0.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
    ]);

    assert!(
        hits.len() >= 2,
        "Polyline crossing cube should hit the entry and exit surfaces"
    );
}

#[test]
fn test_negative_extrude_has_downward_top_normal() {
    let square = Sketch::<()>::square(2.0, None);
    let mesh = square.extrude_vector(Vector3::new(0.0, 0.0, -1.0));

    assert!(
        mesh.polygons
            .iter()
            .any(|poly| poly.plane.normal().dot(&Vector3::new(0.0, 0.0, -1.0)) > 0.9),
        "Negative extrusion should produce a downward-facing translated cap"
    );
}

#[test]
#[cfg(feature = "nurbs")]
fn test_nurbs_rectangle_bbox_and_transform() {
    let rect = crate::nurbs::Nurbs::<()>::rectangle(2.0, 4.0, None);
    let bbox = rect.bounding_box();
    assert!((bbox.mins.x + 1.0).abs() < tolerance());
    assert!((bbox.maxs.y - 2.0).abs() < tolerance());

    let moved = rect.translate(3.0, -1.0, 0.0);
    let moved_bbox = moved.bounding_box();
    assert!((moved_bbox.mins.x - 2.0).abs() < tolerance());
    assert!((moved_bbox.maxs.y - 1.0).abs() < tolerance());
}

#[test]
#[cfg(all(feature = "nurbs", feature = "sketch"))]
fn test_nurbs_to_sketch_tessellates_region() {
    let circle = crate::nurbs::Nurbs::<()>::circle(1.0, None).unwrap();
    let sketch = circle.to_sketch(Some(1e-3));
    let mp = sketch.to_multipolygon();

    assert_eq!(mp.0.len(), 1);
    assert!(mp.0[0].exterior().coords().count() > 4);
}

#[test]
#[cfg(feature = "nurbs")]
fn test_nurbs_boolean_intersection() {
    let a = crate::nurbs::Nurbs::<()>::rectangle(2.0, 2.0, None);
    let b = crate::nurbs::Nurbs::<()>::rectangle(2.0, 2.0, None).translate(1.0, 0.0, 0.0);
    let intersection = a.try_intersection(&b).unwrap();
    let bbox = intersection.bounding_box();

    assert!(bbox.maxs.x > bbox.mins.x);
    assert!(bbox.maxs.x <= 1.0 + tolerance());
    assert!(bbox.mins.x >= 0.0 - tolerance());
}

#[test]
#[cfg(all(feature = "nurbs", feature = "sketch", feature = "mesh"))]
fn test_nurbs_extrudes_to_mesh() {
    let rect = crate::nurbs::Nurbs::<()>::rectangle(2.0, 2.0, None);
    let mesh = rect.extrude_vector(Vector3::new(0.0, 0.0, 1.0), Some(1e-3));

    assert!(!mesh.polygons.is_empty());
    assert!(mesh.bounding_box().maxs.z > 0.9);
}

#[test]
fn test_taubin_smoothing() {
    let sphere: Mesh<()> = Mesh::sphere(1.0, 16, 16, None);
    let original_positions: Vec<_> = sphere
        .polygons
        .iter()
        .flat_map(|poly| poly.vertices.iter().map(|v| v.position))
        .collect();

    // Apply Taubin smoothing
    let smoothed = sphere.taubin_smooth(0.1, -0.105, 2, false);

    // Mesh should have same number of polygons
    assert_eq!(
        smoothed.polygons.len(),
        sphere.polygons.len(),
        "Smoothing should preserve polygon count"
    );

    // At least some vertices should have moved
    let smoothed_positions: Vec<_> = smoothed
        .polygons
        .iter()
        .flat_map(|poly| poly.vertices.iter().map(|v| v.position))
        .collect();

    let mut moved_count = 0;
    for (orig, smooth) in original_positions.iter().zip(smoothed_positions.iter()) {
        if (orig - smooth).norm() > 1e-10 {
            moved_count += 1;
        }
    }
    assert!(
        moved_count > 0,
        "Taubin smoothing should change vertex positions"
    );
}
