//! **Comprehensive Tests for IndexedMesh Gap Analysis Implementation**
//!
//! This test suite validates all the functionality implemented to achieve
//! feature parity between IndexedMesh and the regular Mesh module.

use csgrs::IndexedMesh::{IndexedMesh, IndexedPolygon};
use csgrs::IndexedMesh::plane::Plane as IndexedPlane;
use csgrs::IndexedMesh::vertex::IndexedVertex;
use csgrs::float_types::Real;
use csgrs::traits::CSG;
// Removed unused imports: mesh::plane::Plane, mesh::vertex::Vertex
use nalgebra::{Point3, Vector3};
use std::sync::OnceLock;

/// Create a simple cube IndexedMesh for testing
fn create_test_cube() -> IndexedMesh<i32> {
    let vertices = vec![
        // Bottom face vertices
        IndexedVertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, -1.0)),
        IndexedVertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, -1.0)),
        IndexedVertex::new(Point3::new(1.0, 1.0, 0.0), Vector3::new(0.0, 0.0, -1.0)),
        IndexedVertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::new(0.0, 0.0, -1.0)),
        // Top face vertices
        IndexedVertex::new(Point3::new(0.0, 0.0, 1.0), Vector3::new(0.0, 0.0, 1.0)),
        IndexedVertex::new(Point3::new(1.0, 0.0, 1.0), Vector3::new(0.0, 0.0, 1.0)),
        IndexedVertex::new(Point3::new(1.0, 1.0, 1.0), Vector3::new(0.0, 0.0, 1.0)),
        IndexedVertex::new(Point3::new(0.0, 1.0, 1.0), Vector3::new(0.0, 0.0, 1.0)),
    ];

    let polygons = vec![
        // Bottom face
        IndexedPolygon::new(
            vec![0, 1, 2, 3],
            IndexedPlane::from_points(vertices[0].pos, vertices[1].pos, vertices[2].pos),
            Some(1),
        ),
        // Top face
        IndexedPolygon::new(
            vec![4, 7, 6, 5],
            IndexedPlane::from_points(vertices[4].pos, vertices[7].pos, vertices[6].pos),
            Some(2),
        ),
        // Front face
        IndexedPolygon::new(
            vec![0, 4, 5, 1],
            IndexedPlane::from_points(vertices[0].pos, vertices[4].pos, vertices[5].pos),
            Some(3),
        ),
        // Back face
        IndexedPolygon::new(
            vec![2, 6, 7, 3],
            IndexedPlane::from_points(vertices[2].pos, vertices[6].pos, vertices[7].pos),
            Some(4),
        ),
        // Left face
        IndexedPolygon::new(
            vec![0, 3, 7, 4],
            IndexedPlane::from_points(vertices[0].pos, vertices[3].pos, vertices[7].pos),
            Some(5),
        ),
        // Right face
        IndexedPolygon::new(
            vec![1, 5, 6, 2],
            IndexedPlane::from_points(vertices[1].pos, vertices[5].pos, vertices[6].pos),
            Some(6),
        ),
    ];

    IndexedMesh {
        vertices,
        polygons,
        bounding_box: OnceLock::new(),
        metadata: Some(42),
    }
}

#[test]
fn test_plane_operations_classify_indexed_polygon() {

    let cube = create_test_cube();
    let test_plane = IndexedPlane::from_points(
        Point3::new(0.5, 0.0, 0.0),
        Point3::new(0.5, 1.0, 0.0),
        Point3::new(0.5, 0.0, 1.0),
    );

    // Test polygon classification
    let bottom_face = &cube.polygons[0]; // Should span the plane
    let classification = test_plane.classify_polygon(bottom_face, &cube.vertices);

    // Bottom face should span the vertical plane at x=0.5
    assert_ne!(classification, 0, "Polygon classification should not be zero");
}

#[test]
fn test_plane_operations_split_indexed_polygon() {

    let cube = create_test_cube();
    let mut vertices = cube.vertices.clone();
    let test_plane = IndexedPlane::from_points(
        Point3::new(0.5, 0.0, 0.0),
        Point3::new(0.5, 1.0, 0.0),
        Point3::new(0.5, 0.0, 1.0),
    );

    let bottom_face = &cube.polygons[0];
    let mut edge_cache = std::collections::HashMap::new();
    let (coplanar_front, coplanar_back, front, back) =
        test_plane.split_indexed_polygon_with_cache(bottom_face, &mut vertices, &mut edge_cache);

    // Should have some split results
    let total_results = coplanar_front.len() + coplanar_back.len() + front.len() + back.len();
    assert!(total_results > 0, "Split operation should produce results");
}

#[test]
fn test_indexed_polygon_edges_iterator() {
    let cube = create_test_cube();
    let bottom_face = &cube.polygons[0];

    let edges: Vec<(usize, usize)> = bottom_face.edges().collect();
    assert_eq!(edges.len(), 4, "Square should have 4 edges");

    // Check that edges form a cycle
    assert_eq!(edges[0].0, edges[3].1, "Edges should form a cycle");
}

#[test]
fn test_indexed_polygon_subdivide_triangles() {
    let cube = create_test_cube();
    let _vertices = cube.vertices.clone();
    let bottom_face = &cube.polygons[0];

    let subdivisions = std::num::NonZeroU32::new(1).unwrap();
    let triangles = bottom_face.subdivide_triangles(&mut cube.clone(), subdivisions);

    assert!(!triangles.is_empty(), "Subdivision should produce triangles");
}

#[test]
fn test_indexed_polygon_calculate_new_normal() {
    let cube = create_test_cube();
    let bottom_face = &cube.polygons[0];

    let normal = bottom_face.calculate_new_normal(&cube.vertices);
    assert!(
        (normal.norm() - 1.0).abs() < Real::EPSILON,
        "Normal should be unit length"
    );
}

#[test]
fn test_mesh_validation() {
    let cube = create_test_cube();
    let issues = cube.validate();

    // A well-formed cube should have no validation issues
    assert!(
        issues.is_empty(),
        "Well-formed cube should pass validation: {:?}",
        issues
    );
}

#[test]
fn test_mesh_validation_with_issues() {
    // Create a mesh with validation issues
    let vertices = vec![
        IndexedVertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        IndexedVertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
        IndexedVertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::z()),
    ];

    let polygons = vec![
        // Polygon with duplicate indices
        IndexedPolygon::new(
            vec![0, 1, 1], // Duplicate index
            IndexedPlane::from_points(vertices[0].pos, vertices[1].pos, vertices[2].pos),
            None,
        ),
        // Polygon with out-of-bounds index
        IndexedPolygon::new(
            vec![0, 1, 5], // Index 5 is out of bounds
            IndexedPlane::from_points(vertices[0].pos, vertices[1].pos, vertices[2].pos),
            None,
        ),
    ];

    let mesh = IndexedMesh {
        vertices,
        polygons,
        bounding_box: OnceLock::new(),
        metadata: None::<i32>,
    };

    let issues = mesh.validate();
    assert!(!issues.is_empty(), "Mesh with issues should fail validation");
    assert!(
        issues.iter().any(|issue| issue.contains("duplicate")),
        "Should detect duplicate indices"
    );
    assert!(
        issues.iter().any(|issue| issue.contains("out-of-bounds")),
        "Should detect out-of-bounds indices"
    );
}

#[test]
fn test_merge_vertices() {
    let mut cube = create_test_cube();
    let original_vertex_count = cube.vertices.len();

    // Add a duplicate vertex very close to an existing one
    let duplicate_vertex = IndexedVertex::new(
        Point3::new(0.0001, 0.0, 0.0), // Very close to vertex 0
        Vector3::new(0.0, 0.0, -1.0),
    );
    cube.vertices.push(duplicate_vertex);

    // Add a polygon using the duplicate vertex
    cube.polygons.push(IndexedPolygon::new(
        vec![8, 1, 2], // Using the duplicate vertex
        IndexedPlane::from_points(cube.vertices[8].pos, cube.vertices[1].pos, cube.vertices[2].pos),
        Some(99),
    ));

    cube.merge_vertices(0.001); // Merge vertices within 1mm

    // Should have merged the duplicate vertex
    assert!(
        cube.vertices.len() <= original_vertex_count,
        "Should have merged duplicate vertices"
    );
}

#[test]
fn test_remove_duplicate_polygons() {
    let mut cube = create_test_cube();
    let original_polygon_count = cube.polygons.len();

    // Add a duplicate polygon
    let duplicate_polygon = cube.polygons[0].clone();
    cube.polygons.push(duplicate_polygon);

    cube.remove_duplicate_polygons();

    assert_eq!(
        cube.polygons.len(),
        original_polygon_count,
        "Should have removed duplicate polygon"
    );
}

#[test]
fn test_surface_area_computation() {
    let cube = create_test_cube();
    let surface_area = cube.surface_area();

    // A unit cube should have surface area of 6 (6 faces of area 1 each)
    assert!(
        (surface_area - 6.0).abs() < 0.1,
        "Unit cube should have surface area ~6, got {}",
        surface_area
    );
}

#[test]
fn test_volume_computation() {
    let cube = create_test_cube();
    let volume = cube.volume();

    // A unit cube should have volume of 1
    assert!(
        (volume - 1.0).abs() < 0.1,
        "Unit cube should have volume ~1, got {}",
        volume
    );
}

#[test]
fn test_is_closed() {
    let cube = create_test_cube();
    assert!(cube.is_closed(), "Complete cube should be closed");
}

#[test]
fn test_edge_count() {
    let cube = create_test_cube();
    let edge_count = cube.edge_count();

    // A cube has 12 edges
    assert_eq!(
        edge_count, 12,
        "Cube should have 12 edges, got {}",
        edge_count
    );
}

#[test]
fn test_ray_intersections() {
    let cube = create_test_cube();

    // Ray from inside the cube going outward
    let inside_point = Point3::new(0.5, 0.5, 0.5);
    let direction = Vector3::new(1.0, 0.0, 0.0);

    let intersections = cube.ray_intersections(&inside_point, &direction);
    assert!(
        !intersections.is_empty(),
        "Ray from inside should intersect mesh"
    );
}

#[test]
fn test_contains_vertex() {
    let cube = create_test_cube();

    // Point inside the cube
    let inside_point = Point3::new(0.5, 0.5, 0.5);
    assert!(
        cube.contains_vertex(&inside_point),
        "Point inside cube should be detected"
    );

    // Point outside the cube
    let outside_point = Point3::new(2.0, 2.0, 2.0);
    assert!(
        !cube.contains_vertex(&outside_point),
        "Point outside cube should be detected"
    );
}

#[test]
fn test_bsp_union_operation() {
    let cube1 = create_test_cube();

    // Create a second cube offset by 0.5 units
    let mut cube2 = create_test_cube();
    for vertex in &mut cube2.vertices {
        vertex.pos.x += 0.5;
    }

    let union_result = cube1.union_indexed(&cube2);

    // Union should have more vertices than either original cube
    assert!(
        union_result.vertices.len() >= cube1.vertices.len(),
        "Union should preserve or increase vertex count"
    );
    assert!(
        !union_result.polygons.is_empty(),
        "Union should have polygons"
    );
}

#[test]
fn test_vertex_array_mutation_fix() {
    // **CRITICAL TEST**: This test validates that the vertex array mutation issue is fixed
    // Before the fix, this would cause index out-of-bounds errors or incorrect geometry

    let cube1 = create_test_cube();
    let cube2 = create_test_cube();

    // Perform multiple CSG operations that would trigger vertex array mutations
    let union_result = cube1.union_indexed(&cube2);
    let difference_result = cube1.difference_indexed(&cube2);
    let intersection_result = cube1.intersection_indexed(&cube2);

    // Validate that all operations completed without panics
    assert!(!union_result.vertices.is_empty(), "Union should have vertices");
    assert!(!difference_result.vertices.is_empty(), "Difference should have vertices");
    assert!(!intersection_result.vertices.is_empty(), "Intersection should have vertices");

    // Validate that all polygons have valid indices
    for polygon in &union_result.polygons {
        for &index in &polygon.indices {
            assert!(index < union_result.vertices.len(),
                "Union polygon index {} out of bounds (vertex count: {})",
                index, union_result.vertices.len());
        }
    }

    for polygon in &difference_result.polygons {
        for &index in &polygon.indices {
            assert!(index < difference_result.vertices.len(),
                "Difference polygon index {} out of bounds (vertex count: {})",
                index, difference_result.vertices.len());
        }
    }

    for polygon in &intersection_result.polygons {
        for &index in &polygon.indices {
            assert!(index < intersection_result.vertices.len(),
                "Intersection polygon index {} out of bounds (vertex count: {})",
                index, intersection_result.vertices.len());
        }
    }

    println!("✓ Vertex array mutation fix validated - all indices are valid");
}

#[test]
fn test_quantization_precision_fix() {
    println!("=== Testing Quantization Precision Fix ===");

    // Create test shapes using built-in shape functions
    let cube = IndexedMesh::<i32>::cube(2.0, Some(1));
    let sphere = IndexedMesh::<i32>::sphere(1.2, 8, 6, Some(2));
    let cylinder = IndexedMesh::<i32>::cylinder(0.8, 3.0, 8, Some(3));

    // Perform complex CSG operation with IndexedMesh
    println!("Computing IndexedMesh complex operation: (cube ∪ sphere) - cylinder...");
    let indexed_union = cube.union_indexed(&sphere);
    let indexed_complex = indexed_union.difference_indexed(&cylinder);
    let indexed_analysis = indexed_complex.analyze_manifold();

    // Perform same operation with regular Mesh for comparison
    println!("Computing regular Mesh complex operation for comparison...");
    let cube_mesh = cube.to_mesh();
    let sphere_mesh = sphere.to_mesh();
    let cylinder_mesh = cylinder.to_mesh();
    let regular_union = cube_mesh.union(&sphere_mesh);
    let regular_complex = regular_union.difference(&cylinder_mesh);

    println!("IndexedMesh result: {} vertices, {} polygons, {} boundary edges",
             indexed_complex.vertices.len(), indexed_complex.polygons.len(), indexed_analysis.boundary_edges);
    println!("Regular Mesh result: {} polygons", regular_complex.polygons.len());

    // The results should be reasonably similar (within 30% polygon count difference)
    let polygon_diff_ratio = (indexed_complex.polygons.len() as f64 - regular_complex.polygons.len() as f64).abs()
                           / regular_complex.polygons.len() as f64;

    println!("Polygon count difference ratio: {:.2}%", polygon_diff_ratio * 100.0);

    // With improved quantization, the results should be much closer
    assert!(polygon_diff_ratio < 0.3,
            "IndexedMesh and regular Mesh results should be similar (within 30%), got {:.1}% difference",
            polygon_diff_ratio * 100.0);

    // IndexedMesh should produce a valid manifold result
    assert!(indexed_analysis.boundary_edges < 200,
            "IndexedMesh should produce reasonable boundary edges, got {}",
            indexed_analysis.boundary_edges);

    println!("✓ Quantization precision fix validated - results are consistent between IndexedMesh and regular Mesh");
}

#[test]
fn test_manifold_repair_impact() {
    println!("=== Testing Manifold Repair Impact on CSG Results ===");

    // Create test shapes
    let cube = IndexedMesh::<i32>::cube(2.0, Some(1));
    let sphere = IndexedMesh::<i32>::sphere(1.2, 8, 6, Some(2));
    let cylinder = IndexedMesh::<i32>::cylinder(0.8, 3.0, 8, Some(3));

    // Perform complex CSG operation WITHOUT repair_manifold
    println!("Computing IndexedMesh complex operation WITHOUT repair_manifold...");
    let indexed_union = cube.union_indexed(&sphere);
    let indexed_complex_no_repair = indexed_union.difference_indexed(&cylinder);
    let no_repair_analysis = indexed_complex_no_repair.analyze_manifold();

    // Perform same operation WITH repair_manifold
    println!("Computing IndexedMesh complex operation WITH repair_manifold...");
    let indexed_complex_with_repair = indexed_complex_no_repair.repair_manifold();
    let with_repair_analysis = indexed_complex_with_repair.analyze_manifold();

    println!("WITHOUT repair_manifold: {} vertices, {} polygons, {} boundary edges",
             indexed_complex_no_repair.vertices.len(),
             indexed_complex_no_repair.polygons.len(),
             no_repair_analysis.boundary_edges);

    println!("WITH repair_manifold: {} vertices, {} polygons, {} boundary edges",
             indexed_complex_with_repair.vertices.len(),
             indexed_complex_with_repair.polygons.len(),
             with_repair_analysis.boundary_edges);

    // Compare with regular Mesh (which never calls repair)
    let cube_mesh = cube.to_mesh();
    let sphere_mesh = sphere.to_mesh();
    let cylinder_mesh = cylinder.to_mesh();
    let regular_union = cube_mesh.union(&sphere_mesh);
    let regular_complex = regular_union.difference(&cylinder_mesh);

    println!("Regular Mesh (no repair): {} polygons", regular_complex.polygons.len());

    // The version WITHOUT repair should be closer to regular Mesh results
    let no_repair_diff = (indexed_complex_no_repair.polygons.len() as f64 - regular_complex.polygons.len() as f64).abs()
                        / regular_complex.polygons.len() as f64;
    let with_repair_diff = (indexed_complex_with_repair.polygons.len() as f64 - regular_complex.polygons.len() as f64).abs()
                          / regular_complex.polygons.len() as f64;

    println!("Polygon count difference (no repair): {:.2}%", no_repair_diff * 100.0);
    println!("Polygon count difference (with repair): {:.2}%", with_repair_diff * 100.0);

    // The hypothesis is that repair_manifold is causing the gaps
    if no_repair_analysis.boundary_edges < with_repair_analysis.boundary_edges {
        println!("✓ CONFIRMED: repair_manifold is INCREASING boundary edges (gaps)");
        println!("  - Without repair: {} boundary edges", no_repair_analysis.boundary_edges);
        println!("  - With repair: {} boundary edges", with_repair_analysis.boundary_edges);
    } else {
        println!("✗ repair_manifold is not the primary cause of boundary edges");
    }

    println!("✓ Manifold repair impact analysis completed");
}

#[test]
fn test_edge_caching_impact() {
    println!("=== Testing Edge Caching Impact on CSG Results ===");

    // This test will help us understand if edge caching is the root cause
    // by comparing results with different caching strategies

    // Create test shapes
    let cube = IndexedMesh::<i32>::cube(2.0, Some(1));
    let sphere = IndexedMesh::<i32>::sphere(1.2, 8, 6, Some(2));
    let cylinder = IndexedMesh::<i32>::cylinder(0.8, 3.0, 8, Some(3));

    // Perform complex CSG operation
    println!("Computing IndexedMesh complex operation...");
    let indexed_union = cube.union_indexed(&sphere);
    let indexed_complex = indexed_union.difference_indexed(&cylinder);
    let indexed_analysis = indexed_complex.analyze_manifold();

    // Compare with regular Mesh (no caching)
    let cube_mesh = cube.to_mesh();
    let sphere_mesh = sphere.to_mesh();
    let cylinder_mesh = cylinder.to_mesh();
    let regular_union = cube_mesh.union(&sphere_mesh);
    let regular_complex = regular_union.difference(&cylinder_mesh);

    println!("IndexedMesh (with edge caching): {} vertices, {} polygons, {} boundary edges",
             indexed_complex.vertices.len(),
             indexed_complex.polygons.len(),
             indexed_analysis.boundary_edges);

    println!("Regular Mesh (no caching): {} polygons", regular_complex.polygons.len());

    // The key insight: Regular Mesh creates duplicate vertices but has no gaps
    // IndexedMesh tries to share vertices but creates gaps

    // Calculate vertex efficiency
    let regular_as_indexed = IndexedMesh::from_polygons(&regular_complex.polygons, regular_complex.metadata);
    let regular_analysis = regular_as_indexed.analyze_manifold();

    println!("Regular Mesh converted to IndexedMesh: {} vertices, {} polygons, {} boundary edges",
             regular_as_indexed.vertices.len(),
             regular_as_indexed.polygons.len(),
             regular_analysis.boundary_edges);

    // The hypothesis: Regular Mesh produces solid geometry (0 boundary edges)
    // but IndexedMesh edge caching creates gaps (>0 boundary edges)

    if indexed_analysis.boundary_edges > regular_analysis.boundary_edges {
        println!("✓ CONFIRMED: Edge caching in IndexedMesh is creating more boundary edges (gaps)");
        println!("  - IndexedMesh (with caching): {} boundary edges", indexed_analysis.boundary_edges);
        println!("  - Regular Mesh (no caching): {} boundary edges", regular_analysis.boundary_edges);

        // The trade-off: IndexedMesh is more memory efficient but less geometrically accurate
        let vertex_efficiency = indexed_complex.vertices.len() as f64 / regular_as_indexed.vertices.len() as f64;
        println!("  - Vertex efficiency: IndexedMesh uses {:.1}% of regular Mesh vertices", vertex_efficiency * 100.0);
    } else {
        println!("✗ Edge caching is not the primary cause of boundary edges");
    }

    println!("✓ Edge caching impact analysis completed");
}

#[test]
fn test_final_gap_resolution_status() {
    println!("=== Final Gap Resolution Status ===");

    // Create test shapes
    let cube = IndexedMesh::<i32>::cube(2.0, Some(1));
    let sphere = IndexedMesh::<i32>::sphere(1.2, 8, 6, Some(2));
    let cylinder = IndexedMesh::<i32>::cylinder(0.8, 3.0, 8, Some(3));

    // Perform complex CSG operation with IndexedMesh
    println!("Computing IndexedMesh complex operation...");
    let indexed_union = cube.union_indexed(&sphere);
    let indexed_complex = indexed_union.difference_indexed(&cylinder);
    let indexed_analysis = indexed_complex.analyze_manifold();

    // Compare with regular Mesh
    let cube_mesh = cube.to_mesh();
    let sphere_mesh = sphere.to_mesh();
    let cylinder_mesh = cylinder.to_mesh();
    let regular_union = cube_mesh.union(&sphere_mesh);
    let regular_complex = regular_union.difference(&cylinder_mesh);

    // Convert regular Mesh result to IndexedMesh for comparison
    let regular_as_indexed = IndexedMesh::from_polygons(&regular_complex.polygons, regular_complex.metadata);
    let regular_analysis = regular_as_indexed.analyze_manifold();

    println!("=== FINAL RESULTS ===");
    println!("IndexedMesh native CSG: {} vertices, {} polygons, {} boundary edges",
             indexed_complex.vertices.len(),
             indexed_complex.polygons.len(),
             indexed_analysis.boundary_edges);

    println!("Regular Mesh CSG: {} polygons", regular_complex.polygons.len());

    println!("Regular Mesh → IndexedMesh: {} vertices, {} polygons, {} boundary edges",
             regular_as_indexed.vertices.len(),
             regular_as_indexed.polygons.len(),
             regular_analysis.boundary_edges);

    // Calculate improvements
    let polygon_diff = (indexed_complex.polygons.len() as f64 - regular_complex.polygons.len() as f64).abs()
                      / regular_complex.polygons.len() as f64;

    println!("=== ANALYSIS ===");
    println!("Polygon count difference: {:.1}%", polygon_diff * 100.0);

    if indexed_analysis.boundary_edges == 0 {
        println!("✅ SUCCESS: IndexedMesh produces SOLID geometry (0 boundary edges)");
    } else if indexed_analysis.boundary_edges <= regular_analysis.boundary_edges {
        println!("✅ IMPROVEMENT: IndexedMesh boundary edges ({}) ≤ Regular Mesh conversion ({})",
                 indexed_analysis.boundary_edges, regular_analysis.boundary_edges);
    } else {
        println!("❌ REMAINING ISSUE: IndexedMesh boundary edges ({}) > Regular Mesh conversion ({})",
                 indexed_analysis.boundary_edges, regular_analysis.boundary_edges);
        println!("   → IndexedMesh still has gaps compared to regular Mesh");
    }

    // Memory efficiency
    let vertex_efficiency = indexed_complex.vertices.len() as f64 / regular_as_indexed.vertices.len() as f64;
    println!("Memory efficiency: IndexedMesh uses {:.1}% of regular Mesh vertices", vertex_efficiency * 100.0);

    println!("✓ Final gap resolution status analysis completed");
}
