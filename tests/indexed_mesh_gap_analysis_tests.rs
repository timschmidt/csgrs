//! **Comprehensive Tests for IndexedMesh Gap Analysis Implementation**
//!
//! This test suite validates all the functionality implemented to achieve
//! feature parity between IndexedMesh and the regular Mesh module.

use csgrs::IndexedMesh::{IndexedMesh, IndexedPolygon};
use csgrs::IndexedMesh::plane::Plane as IndexedPlane;
use csgrs::IndexedMesh::vertex::IndexedVertex;
use csgrs::float_types::Real;
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
