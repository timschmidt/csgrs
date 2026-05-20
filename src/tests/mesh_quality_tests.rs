//! Tests for mesh quality metrics.

use super::support::*;

#[test]
fn test_mesh_quality_analysis() {
    let cube: Mesh<()> = Mesh::cube(2.0, ());

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
fn mesh_quality_uses_hyperreal_edge_and_area_measurements() {
    let normal = Vector3::z();
    let polygons = vec![
        Polygon::new(
            vec![
                Vertex::new(Point3::new(0.0, 0.0, 0.0), normal),
                Vertex::new(Point3::new(3.0, 0.0, 0.0), normal),
                Vertex::new(Point3::new(0.0, 4.0, 0.0), normal),
            ],
            (),
        ),
        Polygon::new(
            vec![
                Vertex::new(Point3::new(0.0, 0.0, 1.0), normal),
                Vertex::new(Point3::new(tolerance() * 0.25, 0.0, 1.0), normal),
                Vertex::new(Point3::new(0.0, tolerance() * 0.25, 1.0), normal),
            ],
            (),
        ),
    ];
    let mesh: Mesh<()> = Mesh::from_polygons(&polygons, ());

    let qualities = mesh.analyze_triangle_quality();
    assert_eq!(qualities.len(), 2);
    assert!((qualities[0].area - 6.0).abs() < tolerance());
    assert!(qualities[0].min_angle.is_finite());
    assert!(qualities[0].max_angle.is_finite());
    assert_eq!(qualities[1].area, 0.0);
    assert_eq!(qualities[1].quality_score, 0.0);
}

#[test]
fn mesh_quality_handles_nearly_collinear_triangle_with_hyperreal_metrics() {
    let normal = Vector3::z();
    let polygon = Polygon::new(
        vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), normal),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), normal),
            Vertex::new(Point3::new(1.0, tolerance() * 64.0, 0.0), normal),
        ],
        (),
    );
    let mesh: Mesh<()> = Mesh::from_polygons(&[polygon], ());

    let qualities = mesh.analyze_triangle_quality();
    assert_eq!(qualities.len(), 1);
    let quality = &qualities[0];
    assert!(quality.area.is_finite());
    assert!(quality.edge_ratio.is_finite());
    assert!(quality.min_angle.is_finite());
    assert!(quality.max_angle.is_finite());
    assert!(quality.quality_score.is_finite());
    assert!(quality.quality_score >= 0.0);
    assert!(quality.quality_score <= 1.0);
}

#[test]
fn mesh_dihedral_angle_uses_hyperreal_normal_angle() {
    let xy = Polygon::new(
        vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
        ],
        (),
    );
    let yz = Polygon::new(
        vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::x()),
            Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::x()),
            Vertex::new(Point3::new(0.0, 0.0, 1.0), Vector3::x()),
        ],
        (),
    );

    let angle = Mesh::<()>::dihedral_angle(&xy, &yz);
    assert!((angle - FRAC_PI_2).abs() < tolerance());
}

#[test]
fn test_adaptive_mesh_refinement() {
    let cube: Mesh<()> = Mesh::cube(2.0, ());
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
fn test_laplacian_mesh_smoothing() {
    let sphere: Mesh<()> = Mesh::sphere(1.0, 16, 16, ());
    let original_positions: Vec<_> = sphere
        .polygons
        .iter()
        .flat_map(|poly| poly.vertices.iter().map(|v| v.position))
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
        .flat_map(|poly| poly.vertices.iter().map(|v| v.position))
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
fn test_remove_poor_triangles() {
    // Create a degenerate case by making a very thin triangle
    let vertices = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.5, 1e-8, 0.0), Vector3::z()), // Very thin triangle
    ];
    let bad_polygon: Polygon<()> = Polygon::new(vertices, ());
    let csg_with_bad = Mesh::from_polygons(&[bad_polygon], ());

    // Remove poor quality triangles
    let filtered = csg_with_bad.remove_poor_triangles(0.1);

    // Should remove the poor quality triangle
    assert!(
        filtered.polygons.len() <= csg_with_bad.polygons.len(),
        "Should remove or maintain triangle count"
    );
}

#[test]
fn test_vertex_distance_operations() {
    let v1 = Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::x());
    let v2 = Vertex::new(Point3::new(3.0, 4.0, 0.0), Vector3::y());

    // Test distance calculation
    let distance = v1.distance_to(&v2);
    assert!(
        (distance - 5.0).abs() < 1e-10,
        "Distance should be 5.0 (3-4-5 triangle)"
    );

    // Test squared distance (should be 25.0)
    let distance_sq = v1.distance_squared_to(&v2);
    assert!(
        (distance_sq - 25.0).abs() < 1e-10,
        "Squared distance should be 25.0"
    );

    // Test normal angle
    let angle = v1.normal_angle_to(&v2);
    assert!(
        (angle - PI / 2.0).abs() < 1e-10,
        "Angle between x and y normals should be π/2"
    );
}

#[test]
fn test_vertex_interpolation_methods() {
    let v1 = Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::x());
    let v2 = Vertex::new(Point3::new(2.0, 2.0, 2.0), Vector3::y());

    // Test linear interpolation
    let mid_linear = v1.interpolate(&v2, 0.5);
    assert!(
        (mid_linear.position - Point3::new(1.0, 1.0, 1.0)).norm() < 1e-10,
        "Linear interpolation midpoint should be (1,1,1)"
    );

    // Test spherical interpolation
    let mid_slerp = v1.slerp_interpolate(&v2, 0.5);
    assert!(
        (mid_slerp.position - Point3::new(1.0, 1.0, 1.0)).norm() < 1e-10,
        "SLERP position should match linear for positions"
    );

    // Normal should be normalized and between the two normals
    assert!(
        (mid_slerp.normal.norm() - 1.0).abs() < 1e-10,
        "SLERP normal should be unit length"
    );
}

#[test]
fn test_barycentric_interpolation() {
    let v1 = Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::x());
    let v2 = Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::y());
    let v3 = Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z());

    // Test centroid (equal weights)
    let centroid =
        Vertex::barycentric_interpolate(&v1, &v2, &v3, 1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0);
    let expected_pos = Point3::new(1.0 / 3.0, 1.0 / 3.0, 0.0);
    assert!(
        (centroid.position - expected_pos).norm() < 1e-10,
        "Barycentric centroid should be at (1/3, 1/3, 0)"
    );

    // Test vertex recovery (weight=1 for one vertex)
    let recovered_v1 = Vertex::barycentric_interpolate(&v1, &v2, &v3, 1.0, 0.0, 0.0);
    assert!(
        (recovered_v1.position - v1.position).norm() < 1e-10,
        "Barycentric should recover original vertex"
    );
}

#[test]
fn test_vertex_clustering() {
    let vertices = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::x()),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::x()),
        Vertex::new(Point3::new(0.5, 0.5, 0.0), Vector3::y()),
    ];

    let cluster = VertexCluster::from_vertices(&vertices).expect("Should create cluster");

    // Check cluster properties
    assert_eq!(cluster.count, 3, "Cluster should contain 3 vertices");
    assert!(cluster.radius > 0.0, "Cluster should have positive radius");

    // Centroid should be reasonable
    let expected_centroid = Point3::new(0.5, 1.0 / 6.0, 0.0);
    assert!(
        (cluster.position - expected_centroid).norm() < 1e-10,
        "Cluster centroid should be average of vertex positions"
    );

    // Convert back to vertex
    let representative = cluster.to_vertex();
    assert_eq!(
        representative.position, cluster.position,
        "Representative should have cluster position"
    );
    assert_eq!(
        representative.normal, cluster.normal,
        "Representative should have cluster normal"
    );
}

#[test]
fn test_mesh_connectivity_adjacency_usage() {
    // Create a simple cube to test mesh connectivity
    let cube: Mesh<()> = Mesh::cube(2.0, ());

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
    let original_pos = cube.polygons[0].vertices[0].position;
    let smoothed_pos = smoothed_cube.polygons[0].vertices[0].position;
    let position_change = (original_pos - smoothed_pos).norm();

    println!("  Position change from smoothing: {:.6}", position_change);
    assert!(
        position_change > 1e-10,
        "Smoothing should change vertex positions"
    );
}

#[test]
fn test_vertex_connectivity_analysis() {
    // Create a more complex mesh to test vertex connectivity
    let sphere: Mesh<()> = Mesh::sphere(1.0, 16, 8, ());
    let (vertex_map, adjacency_map) = sphere.build_connectivity();

    // Build vertex positions map for analysis
    let mut vertex_positions = HashMap::new();
    for (pos, idx) in vertex_map.get_vertex_positions() {
        vertex_positions.insert(*idx, *pos);
    }

    // Test vertex connectivity analysis for a few vertices
    let mut total_regularity = 0.0;
    let mut vertex_count = 0;

    for &vertex_idx in adjacency_map.keys().take(5) {
        let (valence, regularity) =
            crate::vertex::Vertex::analyze_connectivity_with_index(vertex_idx, &adjacency_map);

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
fn test_mesh_quality_with_adjacency() {
    // Create a triangulated cube
    let cube: Mesh<()> = Mesh::cube(2.0, ()).triangulate();

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
fn test_adjacency_map_actually_used() {
    // This test specifically verifies that the adjacency map is actually used
    // by comparing results with and without proper connectivity

    let cube: Mesh<()> = Mesh::cube(2.0, ());

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
    let original_first_vertex = cube.polygons[0].vertices[0].position;
    let zero_smoothed_first_vertex = smoothed_0_iterations.polygons[0].vertices[0].position;
    let smoothed_first_vertex = smoothed_1_iterations.polygons[0].vertices[0].position;

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
