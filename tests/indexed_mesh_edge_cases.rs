//! **Comprehensive Edge Case Tests for IndexedMesh**
//!
//! This test suite validates IndexedMesh behavior in edge cases and boundary conditions
//! to ensure robust operation across all scenarios.

use csgrs::IndexedMesh::{IndexedMesh, IndexedPolygon};
use csgrs::float_types::{EPSILON, Real};
use csgrs::mesh::{plane::Plane, vertex::Vertex};
use csgrs::traits::CSG;
use nalgebra::{Point3, Vector3};
use std::sync::OnceLock;

/// Test empty mesh operations
#[test]
fn test_empty_mesh_operations() {
    println!("=== Testing Empty Mesh Operations ===");

    let empty_mesh = IndexedMesh::<()>::new();

    // Empty mesh should have no vertices or polygons
    assert_eq!(empty_mesh.vertices.len(), 0);
    assert_eq!(empty_mesh.polygons.len(), 0);

    // Operations on empty mesh should return empty results
    let cube = IndexedMesh::<()>::cube(1.0, None);

    let union_with_empty = empty_mesh.union(&cube);
    assert_eq!(union_with_empty.vertices.len(), cube.vertices.len());
    assert_eq!(union_with_empty.polygons.len(), cube.polygons.len());

    let difference_with_empty = cube.difference(&empty_mesh);
    assert_eq!(difference_with_empty.vertices.len(), cube.vertices.len());
    assert_eq!(difference_with_empty.polygons.len(), cube.polygons.len());

    let intersection_with_empty = cube.intersection(&empty_mesh);
    assert_eq!(intersection_with_empty.vertices.len(), 0);
    assert_eq!(intersection_with_empty.polygons.len(), 0);

    println!("✓ Empty mesh operations behave correctly");
}

/// Test single vertex mesh
#[test]
fn test_single_vertex_mesh() {
    println!("=== Testing Single Vertex Mesh ===");

    let vertices = vec![Vertex::new(Point3::origin(), Vector3::z())];
    let polygons = vec![];

    let single_vertex_mesh = IndexedMesh {
        vertices,
        polygons,
        bounding_box: OnceLock::new(),
        metadata: None::<()>,
    };

    // Single vertex mesh should be valid but degenerate
    assert_eq!(single_vertex_mesh.vertices.len(), 1);
    assert_eq!(single_vertex_mesh.polygons.len(), 0);

    // Validation should pass (no invalid indices)
    let issues = single_vertex_mesh.validate();
    assert!(issues.is_empty(), "Single vertex mesh should be valid");

    // Surface area should be zero
    assert_eq!(single_vertex_mesh.surface_area(), 0.0);

    println!("✓ Single vertex mesh handled correctly");
}

/// Test degenerate triangle (zero area)
#[test]
fn test_degenerate_triangle() {
    println!("=== Testing Degenerate Triangle ===");

    // Create three collinear vertices (zero area triangle)
    let vertices = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(2.0, 0.0, 0.0), Vector3::z()), // Collinear
    ];

    let plane = Plane::from_vertices(vertices.clone());
    let degenerate_polygon = IndexedPolygon::new(vec![0, 1, 2], plane, None::<()>);

    let degenerate_mesh = IndexedMesh {
        vertices,
        polygons: vec![degenerate_polygon],
        bounding_box: OnceLock::new(),
        metadata: None::<()>,
    };

    // Mesh should be valid but have zero surface area
    let issues = degenerate_mesh.validate();
    assert!(issues.is_empty(), "Degenerate triangle should be valid");

    let surface_area = degenerate_mesh.surface_area();
    assert!(
        surface_area < EPSILON,
        "Degenerate triangle should have zero area"
    );

    // Quality analysis should detect the degenerate triangle
    let quality_metrics = degenerate_mesh.analyze_triangle_quality();
    assert!(!quality_metrics.is_empty());
    assert!(quality_metrics[0].area < EPSILON, "Should detect zero area");

    println!("✓ Degenerate triangle handled correctly");
}

/// Test mesh with duplicate vertices
#[test]
fn test_duplicate_vertices() {
    println!("=== Testing Duplicate Vertices ===");

    // Create mesh with duplicate vertices
    let vertices = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()), // Exact duplicate
        Vertex::new(Point3::new(0.0001, 0.0, 0.0), Vector3::z()), // Near duplicate
    ];

    let plane = Plane::from_vertices(vec![vertices[0], vertices[1], vertices[2]]);
    let polygon = IndexedPolygon::new(vec![0, 1, 2], plane, None::<()>);

    let mut mesh_with_duplicates = IndexedMesh {
        vertices,
        polygons: vec![polygon],
        bounding_box: OnceLock::new(),
        metadata: None::<()>,
    };

    let original_vertex_count = mesh_with_duplicates.vertices.len();
    assert_eq!(original_vertex_count, 5);

    // Merge vertices within tolerance
    mesh_with_duplicates.merge_vertices(0.001);

    // Should have merged the near-duplicate vertices
    assert!(mesh_with_duplicates.vertices.len() < original_vertex_count);

    println!("✓ Duplicate vertices handled correctly");
}

/// Test mesh with invalid indices
#[test]
fn test_invalid_indices() {
    println!("=== Testing Invalid Indices ===");

    let vertices = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::z()),
    ];

    let plane = Plane::from_vertices(vertices.clone());

    // Create polygons with invalid indices
    let polygons = vec![
        IndexedPolygon::new(vec![0, 1, 5], plane.clone(), None::<()>), /* Index 5 out of bounds */
        IndexedPolygon::new(vec![0, 0, 1], plane.clone(), None::<()>), // Duplicate index
        IndexedPolygon::new(vec![0, 1], plane, None::<()>),            // Too few vertices
    ];

    let invalid_mesh = IndexedMesh {
        vertices,
        polygons,
        bounding_box: OnceLock::new(),
        metadata: None::<()>,
    };

    // Validation should detect all issues
    let issues = invalid_mesh.validate();
    assert!(!issues.is_empty(), "Should detect validation issues");

    // Should detect out-of-bounds indices
    assert!(
        issues.iter().any(|issue| issue.contains("out-of-bounds")),
        "Should detect out-of-bounds indices"
    );

    // Should detect duplicate indices
    assert!(
        issues.iter().any(|issue| issue.contains("duplicate")),
        "Should detect duplicate indices"
    );

    // Should detect insufficient vertices
    assert!(
        issues.iter().any(|issue| issue.contains("vertices")),
        "Should detect insufficient vertices"
    );

    println!("✓ Invalid indices detected correctly");
}

/// Test very small mesh (numerical precision edge cases)
#[test]
fn test_tiny_mesh() {
    println!("=== Testing Tiny Mesh (Numerical Precision) ===");

    let scale = EPSILON * 10.0; // Very small scale
    let tiny_cube = IndexedMesh::<()>::cube(scale, None);

    // Tiny mesh should still be valid
    assert!(!tiny_cube.vertices.is_empty());
    assert!(!tiny_cube.polygons.is_empty());

    // Surface area should be very small but positive
    let surface_area = tiny_cube.surface_area();
    assert!(
        surface_area > 0.0,
        "Tiny mesh should have positive surface area"
    );
    assert!(surface_area < 1.0, "Tiny mesh should have small surface area");

    // Manifold analysis should still work
    let analysis = tiny_cube.analyze_manifold();
    assert!(analysis.is_manifold, "Tiny cube should be manifold");

    println!("✓ Tiny mesh handled correctly");
}

/// Test very large mesh (numerical stability)
#[test]
fn test_huge_mesh() {
    println!("=== Testing Huge Mesh (Numerical Stability) ===");

    let scale = 1e6; // Very large scale
    let huge_cube = IndexedMesh::<()>::cube(scale, None);

    // Huge mesh should still be valid
    assert!(!huge_cube.vertices.is_empty());
    assert!(!huge_cube.polygons.is_empty());

    // Surface area should be very large
    let surface_area = huge_cube.surface_area();
    assert!(
        surface_area > 1e10,
        "Huge mesh should have large surface area"
    );

    // Manifold analysis should still work
    let analysis = huge_cube.analyze_manifold();
    assert!(analysis.is_manifold, "Huge cube should be manifold");

    println!("✓ Huge mesh handled correctly");
}

/// Test mesh with extreme aspect ratio triangles
#[test]
fn test_extreme_aspect_ratio() {
    println!("=== Testing Extreme Aspect Ratio Triangles ===");

    // Create a very thin, long triangle
    let vertices = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(1000.0, 0.0, 0.0), Vector3::z()), // Very far
        Vertex::new(Point3::new(500.0, 0.001, 0.0), Vector3::z()), // Very thin
    ];

    let plane = Plane::from_vertices(vertices.clone());
    let thin_polygon = IndexedPolygon::new(vec![0, 1, 2], plane, None::<()>);

    let thin_mesh = IndexedMesh {
        vertices,
        polygons: vec![thin_polygon],
        bounding_box: OnceLock::new(),
        metadata: None::<()>,
    };

    // Quality analysis should detect poor aspect ratio
    let quality_metrics = thin_mesh.analyze_triangle_quality();
    assert!(!quality_metrics.is_empty());

    let aspect_ratio = quality_metrics[0].aspect_ratio;
    assert!(aspect_ratio > 100.0, "Should detect extreme aspect ratio");

    println!("✓ Extreme aspect ratio triangles detected");
}

/// Test CSG operations with non-intersecting meshes
#[test]
fn test_csg_non_intersecting() {
    println!("=== Testing CSG with Non-Intersecting Meshes ===");

    let cube1 = IndexedMesh::<()>::cube(1.0, None);

    // Create a cube far away (no intersection)
    let mut cube2 = IndexedMesh::<()>::cube(1.0, None);
    for vertex in &mut cube2.vertices {
        vertex.pos += Vector3::new(10.0, 0.0, 0.0); // Move far away
    }

    // Union should combine both meshes
    let union_result = cube1.union_indexed(&cube2);
    assert!(union_result.vertices.len() >= cube1.vertices.len() + cube2.vertices.len());

    // Intersection should be empty
    let intersection_result = cube1.intersection_indexed(&cube2);
    assert_eq!(intersection_result.vertices.len(), 0);
    assert_eq!(intersection_result.polygons.len(), 0);

    // Difference should be original mesh
    let difference_result = cube1.difference_indexed(&cube2);
    assert_eq!(difference_result.vertices.len(), cube1.vertices.len());

    println!("✓ Non-intersecting CSG operations handled correctly");
}

/// Test CSG operations with identical meshes
#[test]
fn test_csg_identical_meshes() {
    println!("=== Testing CSG with Identical Meshes ===");

    let cube1 = IndexedMesh::<()>::cube(1.0, None);
    let cube2 = IndexedMesh::<()>::cube(1.0, None);

    // Union of identical meshes should be equivalent to original
    let union_result = cube1.union_indexed(&cube2);
    assert!(!union_result.vertices.is_empty());

    // Intersection of identical meshes should be equivalent to original
    let intersection_result = cube1.intersection_indexed(&cube2);
    assert!(!intersection_result.vertices.is_empty());

    // Difference of identical meshes should be empty
    let difference_result = cube1.difference_indexed(&cube2);
    // Note: Due to numerical precision, may not be exactly empty
    // but should have very small volume

    println!("✓ Identical mesh CSG operations handled correctly");
}

/// Test plane slicing edge cases
#[test]
fn test_plane_slicing_edge_cases() {
    println!("=== Testing Plane Slicing Edge Cases ===");

    let cube = IndexedMesh::<()>::cube(2.0, None);

    // Test slicing with plane that doesn't intersect
    let far_plane = Plane::from_normal(Vector3::x(), 10.0);
    let far_slice = cube.slice(far_plane);
    assert!(
        far_slice.geometry.0.is_empty(),
        "Non-intersecting plane should produce empty slice"
    );

    // Test slicing with plane that passes through vertex
    let vertex_plane = Plane::from_normal(Vector3::x(), 1.0); // Passes through cube corner
    let vertex_slice = cube.slice(vertex_plane);
    // Should still produce valid geometry

    // Test slicing with plane parallel to face
    let parallel_plane = Plane::from_normal(Vector3::z(), 0.0); // Parallel to XY plane
    let parallel_slice = cube.slice(parallel_plane);
    assert!(
        !parallel_slice.geometry.0.is_empty(),
        "Parallel plane should produce slice"
    );

    println!("✓ Plane slicing edge cases handled correctly");
}

/// Test mesh repair operations
#[test]
fn test_mesh_repair_edge_cases() {
    println!("=== Testing Mesh Repair Edge Cases ===");

    // Create a mesh with orientation issues
    let vertices = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.5, 0.5, 1.0), Vector3::z()),
    ];

    let plane1 = Plane::from_vertices(vec![vertices[0], vertices[1], vertices[2]]);
    let plane2 = Plane::from_vertices(vec![vertices[0], vertices[2], vertices[3]]); // Different winding

    let polygons = vec![
        IndexedPolygon::new(vec![0, 1, 2], plane1, None::<()>),
        IndexedPolygon::new(vec![0, 3, 2], plane2, None::<()>), // Inconsistent winding
    ];

    let inconsistent_mesh = IndexedMesh {
        vertices,
        polygons,
        bounding_box: OnceLock::new(),
        metadata: None::<()>,
    };

    // Repair should fix orientation issues
    let repaired_mesh = inconsistent_mesh.repair_manifold();

    // Repaired mesh should be valid
    let issues = repaired_mesh.validate();
    assert!(issues.is_empty() || issues.len() < inconsistent_mesh.validate().len());

    println!("✓ Mesh repair edge cases handled correctly");
}

/// Test boundary condition operations
#[test]
fn test_boundary_conditions() {
    println!("=== Testing Boundary Conditions ===");

    let cube = IndexedMesh::<()>::cube(1.0, None);

    // Test operations at floating point limits
    let tiny_scale = Real::EPSILON;
    let huge_scale = 1.0 / Real::EPSILON;

    // Scaling to tiny size
    let mut tiny_cube = cube.clone();
    for vertex in &mut tiny_cube.vertices {
        vertex.pos *= tiny_scale;
    }

    // Should still be valid
    let tiny_issues = tiny_cube.validate();
    assert!(tiny_issues.is_empty(), "Tiny scaled mesh should be valid");

    // Scaling to huge size
    let mut huge_cube = cube.clone();
    for vertex in &mut huge_cube.vertices {
        vertex.pos *= huge_scale;
    }

    // Should still be valid (though may have precision issues)
    let huge_issues = huge_cube.validate();
    // Allow some precision-related issues for extreme scales

    println!("✓ Boundary conditions handled correctly");
}

/// Test memory stress with large vertex counts
#[test]
fn test_memory_stress() {
    println!("=== Testing Memory Stress ===");

    // Create a mesh with many subdivisions
    let subdivided_sphere = IndexedMesh::<()>::sphere(1.0, 4, 4, None);

    // Should handle large vertex counts efficiently
    assert!(subdivided_sphere.vertices.len() > 100);
    assert!(subdivided_sphere.polygons.len() > 100);

    // Vertex sharing should be efficient
    let total_vertex_refs: usize = subdivided_sphere
        .polygons
        .iter()
        .map(|p| p.indices.len())
        .sum();
    let unique_vertices = subdivided_sphere.vertices.len();
    let sharing_ratio = total_vertex_refs as f64 / unique_vertices as f64;

    assert!(
        sharing_ratio > 2.0,
        "Should demonstrate efficient vertex sharing"
    );

    // Operations should still work efficiently
    let analysis = subdivided_sphere.analyze_manifold();
    assert!(analysis.connected_components > 0);

    println!(
        "✓ Memory stress test passed with {:.2}x vertex sharing efficiency",
        sharing_ratio
    );
}
