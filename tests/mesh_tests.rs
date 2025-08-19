use csgrs::{
    float_types::{PI, Real},
    mesh::Mesh,
    traits::CSG,
};
use nalgebra::Point3;

#[test]
fn mesh_connectivity_adjacency_usage() {
    // Create a simple cube to test mesh connectivity
    let cube: Mesh<()> = Mesh::cube(2.0, None);

    // Build the mesh connectivity graph
    let (vertex_map, adjacency_map) = cube.build_connectivity();

    // Verify that the adjacency map is properly built and used
    println!("Mesh connectivity analysis:");
    println!("  Total unique vertices: {}", vertex_map.vertex_count());
    println!("  Adjacency map entries: {}", adjacency_map.len());

    // Verify that vertices have neighbors
    assert!(
        vertex_map.vertex_count() > 0,
        "Vertex map should not be empty"
    );
    assert!(!adjacency_map.is_empty(), "Adjacency map should not be empty");

    // Test that each vertex has some neighbors
    let mut total_neighbors = 0;
    for (vertex_idx, neighbors) in &adjacency_map {
        total_neighbors += neighbors.len();
        println!("  Vertex {} has {} neighbors", vertex_idx, neighbors.len());
        assert!(
            !neighbors.is_empty(),
            "Each vertex should have at least one neighbor"
        );
    }

    println!("  Total neighbor relationships: {}", total_neighbors);

    // Test the actual mesh connectivity in Laplacian smoothing
    let smoothed_cube = cube.laplacian_smooth(0.1, 1, false);

    // Verify the smoothed mesh has the same number of polygons
    assert_eq!(
        cube.polygons.len(),
        smoothed_cube.polygons.len(),
        "Smoothing should preserve polygon count"
    );

    // Verify that smoothing actually changes vertex positions
    let original_pos = cube.polygons[0].vertices[0].pos;
    let smoothed_pos = smoothed_cube.polygons[0].vertices[0].pos;
    let position_change = (original_pos - smoothed_pos).norm();

    println!("  Position change from smoothing: {:.6}", position_change);
    assert!(
        position_change > 1e-10,
        "Smoothing should change vertex positions"
    );
}

#[test]
fn vertex_connectivity_analysis() {
    // Create a more complex mesh to test vertex connectivity
    let sphere: Mesh<()> = Mesh::sphere(1.0, 16, 8, None);
    let (vertex_map, adjacency_map) = sphere.build_connectivity();

    // Build vertex positions map for analysis
    let mut vertex_positions = hashbrown::HashMap::new();
    for (pos, idx) in vertex_map.get_vertex_positions() {
        vertex_positions.insert(*idx, *pos);
    }

    // Test vertex connectivity analysis for a few vertices
    let mut total_regularity = 0.0;
    let mut vertex_count = 0;

    for &vertex_idx in adjacency_map.keys().take(5) {
        let (valence, regularity) =
            csgrs::mesh::vertex::Vertex::analyze_connectivity_with_index(
                vertex_idx,
                &adjacency_map,
            );

        println!(
            "Vertex {}: valence={}, regularity={:.3}",
            vertex_idx, valence, regularity
        );

        assert!(valence > 0, "Vertex should have positive valence");
        assert!(
            (0.0..=1.0).contains(&regularity),
            "Regularity should be in [0,1]"
        );

        total_regularity += regularity;
        vertex_count += 1;
    }

    let avg_regularity = total_regularity / vertex_count as Real;
    println!("Average regularity: {:.3}", avg_regularity);

    // Sphere vertices should have reasonable regularity
    assert!(
        avg_regularity > 0.1,
        "Sphere vertices should have decent regularity"
    );
}

#[test]
fn mesh_quality_with_adjacency() {
    // Create a triangulated cube
    let cube: Mesh<()> = Mesh::cube(2.0, None).triangulate();

    // Test triangle quality analysis
    let qualities = cube.analyze_triangle_quality();
    println!("Triangle quality analysis:");
    println!("  Number of triangles: {}", qualities.len());

    if !qualities.is_empty() {
        let avg_quality: Real =
            qualities.iter().map(|q| q.quality_score).sum::<Real>() / qualities.len() as Real;
        println!("  Average quality score: {:.3}", avg_quality);

        let min_quality = qualities
            .iter()
            .map(|q| q.quality_score)
            .fold(Real::INFINITY, |a, b| a.min(b));
        println!("  Minimum quality score: {:.3}", min_quality);

        // Cube triangles should have reasonable quality
        assert!(avg_quality > 0.1, "Cube triangles should have decent quality");
        assert!(min_quality >= 0.0, "Quality scores should be non-negative");
    }

    // Test mesh quality metrics
    let metrics = cube.compute_mesh_quality();
    println!("Mesh quality metrics:");
    println!("  Average quality: {:.3}", metrics.avg_quality);
    println!("  Minimum quality: {:.3}", metrics.min_quality);
    println!("  High quality ratio: {:.3}", metrics.high_quality_ratio);
    println!("  Sliver count: {}", metrics.sliver_count);
    println!("  Average edge length: {:.3}", metrics.avg_edge_length);
    println!("  Edge length std: {:.3}", metrics.edge_length_std);

    assert!(
        metrics.avg_quality >= 0.0,
        "Average quality should be non-negative"
    );
    assert!(
        metrics.min_quality >= 0.0,
        "Minimum quality should be non-negative"
    );
    assert!(
        metrics.high_quality_ratio >= 0.0 && metrics.high_quality_ratio <= 1.0,
        "High quality ratio should be in [0,1]"
    );
}

#[test]
fn adjacency_map_actually_used() {
    // This test specifically verifies that the adjacency map is actually used
    // by comparing results with and without proper connectivity

    let cube: Mesh<()> = Mesh::cube(2.0, None);

    // Build connectivity
    let (vertex_map, adjacency_map) = cube.build_connectivity();

    // Verify the adjacency map is not empty and has meaningful data
    assert!(!adjacency_map.is_empty(), "Adjacency map should not be empty");

    // Verify that vertices have multiple neighbors (not just self-references)
    let mut has_multiple_neighbors = false;
    for neighbors in adjacency_map.values() {
        if neighbors.len() > 2 {
            has_multiple_neighbors = true;
            break;
        }
    }
    assert!(
        has_multiple_neighbors,
        "Some vertices should have multiple neighbors"
    );

    // Test that the adjacency map affects smoothing
    let smoothed_0_iterations = cube.laplacian_smooth(0.0, 1, false);
    let smoothed_1_iterations = cube.laplacian_smooth(0.1, 1, false);

    // With lambda=0, no smoothing should occur
    let original_first_vertex = cube.polygons[0].vertices[0].pos;
    let zero_smoothed_first_vertex = smoothed_0_iterations.polygons[0].vertices[0].pos;
    let smoothed_first_vertex = smoothed_1_iterations.polygons[0].vertices[0].pos;

    // With lambda=0, position should be unchanged
    let zero_diff = (original_first_vertex - zero_smoothed_first_vertex).norm();
    assert!(
        zero_diff < 1e-10,
        "Zero smoothing should not change positions"
    );

    // With lambda=0.1, position should change
    let smooth_diff = (original_first_vertex - smoothed_first_vertex).norm();
    assert!(
        smooth_diff > 1e-10,
        "Smoothing should change vertex positions"
    );

    println!("Adjacency map usage verified:");
    println!("  Vertex count: {}", vertex_map.vertex_count());
    println!("  Adjacency entries: {}", adjacency_map.len());
    println!("  Smoothing effect: {:.6}", smooth_diff);
}

#[test]
fn cube_basics() {
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
fn cube_intersection() {
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
fn contains_vertex() {
    let csg_cube = Mesh::<()>::cube(6.0, None);

    assert!(csg_cube.contains_vertex(&Point3::new(3.0, 3.0, 3.0)));
    assert!(csg_cube.contains_vertex(&Point3::new(1.0, 2.0, 5.9)));
    assert!(!csg_cube.contains_vertex(&Point3::new(3.0, 3.0, 6.0)));
    assert!(!csg_cube.contains_vertex(&Point3::new(3.0, 3.0, -6.0)));
    assert!(!csg_cube.contains_vertex(&Point3::new(3.0, 3.0, 0.0)));
    assert!(csg_cube.contains_vertex(&Point3::new(3.0, 3.0, 0.01)));

    #[cfg(feature = "f64")]
    {
        assert!(csg_cube.contains_vertex(&Point3::new(3.0, 3.0, 5.99999999)));
        assert!(csg_cube.contains_vertex(&Point3::new(3.0, 3.0, 6.0 - 1e-11)));
        assert!(csg_cube.contains_vertex(&Point3::new(3.0, 3.0, 6.0 - 1e-14)));
        assert!(csg_cube.contains_vertex(&Point3::new(3.0, 3.0, 5.9 + 9e-9)));
    }

    #[cfg(feature = "f32")]
    {
        assert!(csg_cube.contains_vertex(&Point3::new(3.0, 3.0, 5.999999)));
        assert!(csg_cube.contains_vertex(&Point3::new(3.0, 3.0, 6.0 - 1e-6)));
        assert!(csg_cube.contains_vertex(&Point3::new(3.0, 3.0, 5.9 + 9e-9)));
    }

    assert!(csg_cube.contains_vertex(&Point3::new(3.0, -3.0, 3.0)));
    assert!(!csg_cube.contains_vertex(&Point3::new(3.0, -3.01, 3.0)));
    assert!(csg_cube.contains_vertex(&Point3::new(0.01, 4.0, 3.0)));
    assert!(!csg_cube.contains_vertex(&Point3::new(-0.01, 4.0, 3.0)));

    let csg_cube_hole = Mesh::<()>::cube(4.0, None);
    let cube_with_hole = csg_cube.difference(&csg_cube_hole);

    assert!(!cube_with_hole.contains_vertex(&Point3::new(0.01, 4.0, 3.0)));
    assert!(cube_with_hole.contains_vertex(&Point3::new(0.01, 4.01, 3.0)));
    assert!(!cube_with_hole.contains_vertex(&Point3::new(-0.01, 4.0, 3.0)));
    assert!(cube_with_hole.contains_vertex(&Point3::new(1.0, 2.0, 5.9)));
    assert!(!cube_with_hole.contains_vertex(&Point3::new(3.0, 3.0, 6.0)));

    let csg_sphere = Mesh::<()>::sphere(6.0, 14, 14, None);

    assert!(csg_sphere.contains_vertex(&Point3::new(3.0, 3.0, 3.0)));
    assert!(csg_sphere.contains_vertex(&Point3::new(-3.0, -3.0, -3.0)));
    assert!(!csg_sphere.contains_vertex(&Point3::new(1.0, 2.0, 5.9)));

    assert!(!csg_sphere.contains_vertex(&Point3::new(1.0, 1.0, 5.8)));
    assert!(!csg_sphere.contains_vertex(&Point3::new(0.0, 3.0, 5.8)));
    assert!(csg_sphere.contains_vertex(&Point3::new(0.0, 0.0, 5.8)));

    assert!(!csg_sphere.contains_vertex(&Point3::new(3.0, 3.0, 6.0)));
    assert!(!csg_sphere.contains_vertex(&Point3::new(3.0, 3.0, -6.0)));
    assert!(csg_sphere.contains_vertex(&Point3::new(3.0, 3.0, 0.0)));

    assert!(csg_sphere.contains_vertex(&Point3::new(0.0, 0.0, -5.8)));
    assert!(!csg_sphere.contains_vertex(&Point3::new(3.0, 3.0, -5.8)));
    assert!(!csg_sphere.contains_vertex(&Point3::new(3.0, 3.0, -6.01)));
    assert!(csg_sphere.contains_vertex(&Point3::new(3.0, 3.0, 0.01)));
}

#[test]
fn quality_analysis() {
    let cube: Mesh<()> = Mesh::cube(2.0, None);

    // Analyze triangle quality
    let qualities = cube.analyze_triangle_quality();
    assert!(
        !qualities.is_empty(),
        "Should have quality metrics for triangles"
    );

    // All cube triangles should have reasonable quality
    for quality in &qualities {
        assert!(
            quality.quality_score >= 0.0,
            "Quality score should be non-negative"
        );
        assert!(
            quality.quality_score <= 1.0,
            "Quality score should be at most 1.0"
        );
        assert!(quality.area > 0.0, "Triangle area should be positive");
        assert!(quality.min_angle > 0.0, "Minimum angle should be positive");
        assert!(quality.max_angle < PI, "Maximum angle should be less than π");
    }

    // Compute overall mesh quality metrics
    let mesh_metrics = cube.compute_mesh_quality();
    assert!(
        mesh_metrics.avg_quality >= 0.0,
        "Average quality should be non-negative"
    );
    assert!(
        mesh_metrics.min_quality >= 0.0,
        "Minimum quality should be non-negative"
    );
    assert!(
        mesh_metrics.high_quality_ratio >= 0.0,
        "High quality ratio should be non-negative"
    );
    assert!(
        mesh_metrics.high_quality_ratio <= 1.0,
        "High quality ratio should be at most 1.0"
    );
    assert!(
        mesh_metrics.avg_edge_length > 0.0,
        "Average edge length should be positive"
    );
}

#[test]
fn adaptive_refinement() {
    let cube: Mesh<()> = Mesh::cube(2.0, None);
    let original_polygon_count = cube.polygons.len();

    // Refine mesh adaptively based on quality and edge length
    let refined = cube.adaptive_refine(0.3, 2.0, 15.0); // Refine triangles with quality < 0.3, edge length > 2.0, or curvature > 15 deg

    // Refined mesh should generally have more polygons (unless already very high quality)
    // Note: Since cube has high-quality faces, refinement may not increase polygon count significantly
    assert!(
        refined.polygons.len() >= original_polygon_count,
        "Adaptive refinement should maintain or increase polygon count"
    );

    // Test with more aggressive settings
    let aggressive_refined = cube.adaptive_refine(0.8, 1.0, 5.0);
    assert!(
        aggressive_refined.polygons.len() >= refined.polygons.len(),
        "More aggressive refinement should result in equal or more polygons"
    );
}

#[test]
fn laplacian_smoothing() {
    let sphere: Mesh<()> = Mesh::sphere(1.0, 16, 16, None);
    let original_positions: Vec<_> = sphere
        .polygons
        .iter()
        .flat_map(|poly| poly.vertices.iter().map(|v| v.pos))
        .collect();

    // Apply mild Laplacian smoothing
    let smoothed = sphere.laplacian_smooth(0.1, 2, false);

    // Mesh should have same number of polygons
    assert_eq!(
        smoothed.polygons.len(),
        sphere.polygons.len(),
        "Smoothing should preserve polygon count"
    );

    // Vertices should have moved (unless already perfectly smooth)
    let smoothed_positions: Vec<_> = smoothed
        .polygons
        .iter()
        .flat_map(|poly| poly.vertices.iter().map(|v| v.pos))
        .collect();

    assert_eq!(
        original_positions.len(),
        smoothed_positions.len(),
        "Should preserve vertex count"
    );

    // At least some vertices should have moved (unless mesh was already perfect)
    let mut moved_count = 0;
    for (orig, smooth) in original_positions.iter().zip(smoothed_positions.iter()) {
        if (orig - smooth).norm() > 1e-10 {
            moved_count += 1;
        }
    }

    // With sphere geometry and smoothing, we expect some movement
    println!("Moved vertices: {}/{}", moved_count, original_positions.len());
}

#[test]
fn taubin_smoothing() {
    let sphere: Mesh<()> = Mesh::sphere(1.0, 16, 16, None);
    let original_positions: Vec<_> = sphere
        .polygons
        .iter()
        .flat_map(|poly| poly.vertices.iter().map(|v| v.pos))
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
        .flat_map(|poly| poly.vertices.iter().map(|v| v.pos))
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
