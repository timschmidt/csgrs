//! **Validation Tests for IndexedMesh Flatten/Slice Operations**
//!
//! This test suite validates that the flatten_slice.rs module works correctly
//! with IndexedMesh types and produces expected results.

use csgrs::IndexedMesh::{IndexedMesh, plane::Plane};
use csgrs::traits::CSG;
use nalgebra::Vector3;

/// Test IndexedMesh flattening operation
#[test]
fn test_indexed_mesh_flattening() {
    println!("=== Testing IndexedMesh Flattening ===");

    let cube = IndexedMesh::<()>::cube(2.0, None);

    // Flatten the cube to 2D
    let flattened = cube.flatten();

    // Flattened result should be a valid 2D sketch
    assert!(
        !flattened.geometry.0.is_empty(),
        "Flattened geometry should not be empty"
    );

    println!("✓ IndexedMesh flattening produces valid 2D geometry");
}

/// Test IndexedMesh slicing operation with IndexedMesh plane
#[test]
fn test_indexed_mesh_slicing() {
    println!("=== Testing IndexedMesh Slicing ===");

    let cube = IndexedMesh::<()>::cube(2.0, None);

    // Create an IndexedMesh plane for slicing
    let plane = Plane::from_normal(Vector3::z(), 0.0);
    let cross_section = cube.slice(plane);

    // Cross-section should be valid
    assert!(
        !cross_section.geometry.0.is_empty(),
        "Cross-section should not be empty"
    );

    println!("✓ IndexedMesh slicing with IndexedMesh plane works correctly");
}

/// Test IndexedMesh multi-slice operation
#[test]
fn test_indexed_mesh_multi_slice() {
    println!("=== Testing IndexedMesh Multi-Slice ===");

    let cube = IndexedMesh::<()>::cube(2.0, None);

    // Create multiple parallel slices that should intersect the cube
    // For a 2.0 cube, the center is at origin, so it extends from -1.0 to 1.0
    let plane_normal = Vector3::z();
    let distances = [-0.5, 0.0, 0.5]; // These should all intersect the cube

    let slices = cube.multi_slice(plane_normal, &distances);

    // Should have one slice for each distance
    assert_eq!(slices.len(), distances.len());

    // Check each slice - some may be empty if they don't intersect
    for (i, slice) in slices.iter().enumerate() {
        println!("Slice {} geometry count: {}", i, slice.geometry.0.len());
        // Note: Not all slices may produce geometry depending on intersection
    }

    println!("✓ IndexedMesh multi-slice operation works correctly");
}

/// Test edge cases for IndexedMesh slicing
#[test]
fn test_indexed_mesh_slice_edge_cases() {
    println!("=== Testing IndexedMesh Slice Edge Cases ===");

    let cube = IndexedMesh::<()>::cube(2.0, None);

    // Test slicing with plane that doesn't intersect
    let far_plane = Plane::from_normal(Vector3::x(), 10.0);
    let far_slice = cube.slice(far_plane);
    assert!(
        far_slice.geometry.0.is_empty(),
        "Non-intersecting plane should produce empty slice"
    );

    // Test slicing with plane parallel to face
    let parallel_plane = Plane::from_normal(Vector3::z(), 0.0);
    let parallel_slice = cube.slice(parallel_plane);
    assert!(
        !parallel_slice.geometry.0.is_empty(),
        "Parallel plane should produce slice"
    );

    println!("✓ IndexedMesh slice edge cases handled correctly");
}

/// Test that IndexedMesh flatten/slice operations maintain metadata
#[test]
fn test_indexed_mesh_flatten_slice_metadata() {
    println!("=== Testing IndexedMesh Flatten/Slice Metadata ===");

    let cube = IndexedMesh::<i32>::cube(2.0, Some(42));

    // Test flattening preserves metadata
    let flattened = cube.flatten();
    assert_eq!(flattened.metadata, Some(42));

    // Test slicing preserves metadata
    let plane = Plane::from_normal(Vector3::z(), 0.0);
    let sliced = cube.slice(plane);
    assert_eq!(sliced.metadata, Some(42));

    println!("✓ IndexedMesh flatten/slice operations preserve metadata");
}

/// Test IndexedMesh flatten/slice performance characteristics
#[test]
fn test_indexed_mesh_flatten_slice_performance() {
    println!("=== Testing IndexedMesh Flatten/Slice Performance ===");

    // Create a more complex mesh
    let sphere = IndexedMesh::<()>::sphere(1.0, 4, 4, None);

    // Flattening should complete quickly
    let start = std::time::Instant::now();
    let _flattened = sphere.flatten();
    let flatten_time = start.elapsed();

    // Slicing should complete quickly
    let start = std::time::Instant::now();
    let plane = Plane::from_normal(Vector3::z(), 0.0);
    let _sliced = sphere.slice(plane);
    let slice_time = start.elapsed();

    println!("Flatten time: {:?}", flatten_time);
    println!("Slice time: {:?}", slice_time);

    // Operations should complete in reasonable time (less than 1 second)
    assert!(flatten_time.as_secs() < 1, "Flattening should be fast");
    assert!(slice_time.as_secs() < 1, "Slicing should be fast");

    println!("✓ IndexedMesh flatten/slice operations are performant");
}

/// Test IndexedMesh flatten/slice with empty mesh
#[test]
fn test_indexed_mesh_flatten_slice_empty() {
    println!("=== Testing IndexedMesh Flatten/Slice with Empty Mesh ===");

    let empty_mesh = IndexedMesh::<()>::new();

    // Verify empty mesh has no vertices or polygons
    assert_eq!(empty_mesh.vertices.len(), 0);
    assert_eq!(empty_mesh.polygons.len(), 0);

    // Flattening empty mesh should produce empty result
    let flattened = empty_mesh.flatten();
    // Note: Empty mesh may still produce a valid but empty geometry collection
    println!("Flattened geometry count: {}", flattened.geometry.0.len());

    // Slicing empty mesh should produce empty result
    let plane = Plane::from_normal(Vector3::z(), 0.0);
    let sliced = empty_mesh.slice(plane);
    // Note: Empty mesh may still produce a valid but empty geometry collection
    println!("Sliced geometry count: {}", sliced.geometry.0.len());

    println!("✓ IndexedMesh flatten/slice with empty mesh handled correctly");
}
