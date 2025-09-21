use csgrs::IndexedMesh::IndexedMesh;
use csgrs::traits::CSG;
use nalgebra::{Point3, Vector3};
use std::collections::HashMap;

#[test]
fn test_overlapping_meshes_shared_faces() {
    println!("=== Overlapping Meshes with Shared Faces Test ===");
    
    // Create two cubes that share a face
    let cube1 = IndexedMesh::cube(2.0, None);
    
    // Create second cube offset to share a face
    let mut cube2 = IndexedMesh::cube(2.0, None);
    // Translate cube2 so it shares the right face of cube1
    for vertex in &mut cube2.vertices {
        vertex.pos.x += 2.0;
    }
    
    println!("Input meshes:");
    println!("  Cube1: {} vertices, {} polygons", cube1.vertices.len(), cube1.polygons.len());
    println!("  Cube2: {} vertices, {} polygons", cube2.vertices.len(), cube2.polygons.len());
    
    // Test all CSG operations
    test_csg_operation_edge_case("Union", &cube1, &cube2, |a, b| a.union(b));
    test_csg_operation_edge_case("Difference", &cube1, &cube2, |a, b| a.difference(b));
    test_csg_operation_edge_case("Intersection", &cube1, &cube2, |a, b| a.intersection(b));
    
    println!("=== Overlapping Meshes with Shared Faces Test Complete ===");
}

#[test]
fn test_touching_non_intersecting_boundaries() {
    println!("=== Touching Non-Intersecting Boundaries Test ===");
    
    // Create two cubes that touch at a single point
    let cube1 = IndexedMesh::cube(1.0, None);
    
    let mut cube2 = IndexedMesh::cube(1.0, None);
    // Translate cube2 so it touches cube1 at a corner
    for vertex in &mut cube2.vertices {
        vertex.pos.x += 1.0;
        vertex.pos.y += 1.0;
        vertex.pos.z += 1.0;
    }
    
    println!("Input meshes:");
    println!("  Cube1: {} vertices, {} polygons", cube1.vertices.len(), cube1.polygons.len());
    println!("  Cube2: {} vertices, {} polygons", cube2.vertices.len(), cube2.polygons.len());
    
    // Test all CSG operations
    test_csg_operation_edge_case("Union", &cube1, &cube2, |a, b| a.union(b));
    test_csg_operation_edge_case("Difference", &cube1, &cube2, |a, b| a.difference(b));
    test_csg_operation_edge_case("Intersection", &cube1, &cube2, |a, b| a.intersection(b));
    
    println!("=== Touching Non-Intersecting Boundaries Test Complete ===");
}

#[test]
fn test_complex_multi_face_intersections() {
    println!("=== Complex Multi-Face Intersections Test ===");
    
    // Create two cubes with complex intersection
    let cube1 = IndexedMesh::cube(3.0, None);
    
    let mut cube2 = IndexedMesh::cube(2.0, None);
    // Rotate and translate cube2 to create complex intersection
    for vertex in &mut cube2.vertices {
        // Simple rotation around Z axis (45 degrees)
        let x = vertex.pos.x;
        let y = vertex.pos.y;
        let cos45 = 0.707106781;
        let sin45 = 0.707106781;
        vertex.pos.x = x * cos45 - y * sin45;
        vertex.pos.y = x * sin45 + y * cos45;
        
        // Translate to center intersection
        vertex.pos.x += 0.5;
        vertex.pos.y += 0.5;
        vertex.pos.z += 0.5;
    }
    
    println!("Input meshes:");
    println!("  Cube1: {} vertices, {} polygons", cube1.vertices.len(), cube1.polygons.len());
    println!("  Cube2: {} vertices, {} polygons", cube2.vertices.len(), cube2.polygons.len());
    
    // Test all CSG operations
    test_csg_operation_edge_case("Union", &cube1, &cube2, |a, b| a.union(b));
    test_csg_operation_edge_case("Difference", &cube1, &cube2, |a, b| a.difference(b));
    test_csg_operation_edge_case("Intersection", &cube1, &cube2, |a, b| a.intersection(b));
    
    println!("=== Complex Multi-Face Intersections Test Complete ===");
}

#[test]
fn test_degenerate_zero_volume_intersections() {
    println!("=== Degenerate Zero-Volume Intersections Test ===");
    
    // Create two cubes that intersect only along an edge
    let cube1 = IndexedMesh::cube(2.0, None);
    
    let mut cube2 = IndexedMesh::cube(2.0, None);
    // Translate cube2 so it only touches along an edge
    for vertex in &mut cube2.vertices {
        vertex.pos.x += 2.0; // Touch along the right edge
    }
    
    println!("Input meshes:");
    println!("  Cube1: {} vertices, {} polygons", cube1.vertices.len(), cube1.polygons.len());
    println!("  Cube2: {} vertices, {} polygons", cube2.vertices.len(), cube2.polygons.len());
    
    // Test all CSG operations
    test_csg_operation_edge_case("Union", &cube1, &cube2, |a, b| a.union(b));
    test_csg_operation_edge_case("Difference", &cube1, &cube2, |a, b| a.difference(b));
    test_csg_operation_edge_case("Intersection", &cube1, &cube2, |a, b| a.intersection(b));
    
    // Test intersection along a face (zero volume)
    let mut cube3 = IndexedMesh::cube(2.0, None);
    // Make cube3 very thin to create near-zero volume intersection
    for vertex in &mut cube3.vertices {
        if vertex.pos.x > 0.0 {
            vertex.pos.x = 0.01; // Very thin slice
        }
    }
    
    println!("\n--- Zero-Volume Face Intersection ---");
    println!("Cube1 vs Thin Cube:");
    test_csg_operation_edge_case("Union", &cube1, &cube3, |a, b| a.union(b));
    test_csg_operation_edge_case("Intersection", &cube1, &cube3, |a, b| a.intersection(b));
    
    println!("=== Degenerate Zero-Volume Intersections Test Complete ===");
}

#[test]
fn test_identical_meshes_csg() {
    println!("=== Identical Meshes CSG Test ===");
    
    // Test CSG operations on identical meshes
    let cube1 = IndexedMesh::cube(2.0, None);
    let cube2 = IndexedMesh::cube(2.0, None);
    
    println!("Input meshes:");
    println!("  Cube1: {} vertices, {} polygons", cube1.vertices.len(), cube1.polygons.len());
    println!("  Cube2: {} vertices, {} polygons", cube2.vertices.len(), cube2.polygons.len());
    
    // Test all CSG operations on identical meshes
    test_csg_operation_edge_case("Union", &cube1, &cube2, |a, b| a.union(b));
    test_csg_operation_edge_case("Difference", &cube1, &cube2, |a, b| a.difference(b));
    test_csg_operation_edge_case("Intersection", &cube1, &cube2, |a, b| a.intersection(b));
    
    println!("=== Identical Meshes CSG Test Complete ===");
}

#[test]
fn test_nested_meshes_csg() {
    println!("=== Nested Meshes CSG Test ===");
    
    // Create nested cubes (one inside the other)
    let outer_cube = IndexedMesh::cube(4.0, None);
    let inner_cube = IndexedMesh::cube(2.0, None);
    
    println!("Input meshes:");
    println!("  Outer cube: {} vertices, {} polygons", outer_cube.vertices.len(), outer_cube.polygons.len());
    println!("  Inner cube: {} vertices, {} polygons", inner_cube.vertices.len(), inner_cube.polygons.len());
    
    // Test all CSG operations
    test_csg_operation_edge_case("Union", &outer_cube, &inner_cube, |a, b| a.union(b));
    test_csg_operation_edge_case("Difference", &outer_cube, &inner_cube, |a, b| a.difference(b));
    test_csg_operation_edge_case("Intersection", &outer_cube, &inner_cube, |a, b| a.intersection(b));
    
    println!("=== Nested Meshes CSG Test Complete ===");
}

fn test_csg_operation_edge_case<F>(
    operation_name: &str,
    mesh1: &IndexedMesh<f64>,
    mesh2: &IndexedMesh<f64>,
    operation: F,
) where
    F: Fn(&IndexedMesh<f64>, &IndexedMesh<f64>) -> IndexedMesh<f64>,
{
    println!("\n--- {} Operation ---", operation_name);
    
    let result = operation(mesh1, mesh2);
    
    println!("Result: {} vertices, {} polygons", result.vertices.len(), result.polygons.len());
    
    // Analyze result quality
    let edge_analysis = analyze_edge_connectivity(&result);
    let topology_analysis = analyze_topology_quality(&result);
    
    println!("Edge analysis:");
    println!("  Manifold edges: {}", edge_analysis.manifold_edges);
    println!("  Boundary edges: {}", edge_analysis.boundary_edges);
    println!("  Non-manifold edges: {}", edge_analysis.non_manifold_edges);
    
    println!("Topology quality:");
    println!("  Is manifold: {}", topology_analysis.is_manifold);
    println!("  Is closed: {}", topology_analysis.is_closed);
    println!("  Volume: {:.6}", topology_analysis.volume);
    println!("  Surface area: {:.6}", topology_analysis.surface_area);
    
    // Check for geometric validity
    let geometric_analysis = analyze_geometric_validity(&result);
    
    println!("Geometric validity:");
    println!("  Valid polygons: {}/{}", geometric_analysis.valid_polygons, result.polygons.len());
    println!("  Degenerate polygons: {}", geometric_analysis.degenerate_polygons);
    println!("  Self-intersecting polygons: {}", geometric_analysis.self_intersecting_polygons);
    
    // Overall assessment
    let is_perfect = edge_analysis.boundary_edges == 0 && 
                    edge_analysis.non_manifold_edges == 0 &&
                    geometric_analysis.degenerate_polygons == 0 &&
                    geometric_analysis.self_intersecting_polygons == 0;
    
    if is_perfect {
        println!("✅ Perfect result - manifold topology with valid geometry");
    } else {
        println!("❌ Issues detected in {} result", operation_name);
    }
}

#[derive(Debug)]
struct EdgeConnectivity {
    manifold_edges: usize,
    boundary_edges: usize,
    non_manifold_edges: usize,
}

fn analyze_edge_connectivity(mesh: &IndexedMesh<f64>) -> EdgeConnectivity {
    let mut edge_count: HashMap<(usize, usize), usize> = HashMap::new();
    
    for polygon in &mesh.polygons {
        let indices = &polygon.indices;
        for i in 0..indices.len() {
            let v1 = indices[i];
            let v2 = indices[(i + 1) % indices.len()];
            let edge = if v1 < v2 { (v1, v2) } else { (v2, v1) };
            *edge_count.entry(edge).or_insert(0) += 1;
        }
    }
    
    let mut manifold_edges = 0;
    let mut boundary_edges = 0;
    let mut non_manifold_edges = 0;
    
    for count in edge_count.values() {
        match *count {
            1 => boundary_edges += 1,
            2 => manifold_edges += 1,
            n if n > 2 => non_manifold_edges += 1,
            _ => {}
        }
    }
    
    EdgeConnectivity {
        manifold_edges,
        boundary_edges,
        non_manifold_edges,
    }
}

#[derive(Debug)]
struct TopologyQuality {
    is_manifold: bool,
    is_closed: bool,
    volume: f64,
    surface_area: f64,
}

fn analyze_topology_quality(mesh: &IndexedMesh<f64>) -> TopologyQuality {
    let edge_analysis = analyze_edge_connectivity(mesh);
    
    let is_manifold = edge_analysis.non_manifold_edges == 0;
    let is_closed = edge_analysis.boundary_edges == 0;
    
    // Calculate volume using divergence theorem (simplified)
    let volume = calculate_mesh_volume(mesh);
    let surface_area = calculate_surface_area(mesh);
    
    TopologyQuality {
        is_manifold,
        is_closed,
        volume,
        surface_area,
    }
}

fn calculate_mesh_volume(mesh: &IndexedMesh<f64>) -> f64 {
    let mut volume = 0.0;
    
    for polygon in &mesh.polygons {
        if polygon.indices.len() >= 3 {
            // Use first vertex as origin for triangulation
            let v0 = mesh.vertices[polygon.indices[0]].pos;
            
            for i in 1..polygon.indices.len() - 1 {
                let v1 = mesh.vertices[polygon.indices[i]].pos;
                let v2 = mesh.vertices[polygon.indices[i + 1]].pos;
                
                // Calculate tetrahedron volume
                let tetrahedron_volume = v0.coords.dot(&v1.coords.cross(&v2.coords)) / 6.0;
                volume += tetrahedron_volume;
            }
        }
    }
    
    volume.abs()
}

fn calculate_surface_area(mesh: &IndexedMesh<f64>) -> f64 {
    let mut area = 0.0;
    
    for polygon in &mesh.polygons {
        if polygon.indices.len() >= 3 {
            // Triangulate polygon and sum triangle areas
            let v0 = mesh.vertices[polygon.indices[0]].pos;
            
            for i in 1..polygon.indices.len() - 1 {
                let v1 = mesh.vertices[polygon.indices[i]].pos;
                let v2 = mesh.vertices[polygon.indices[i + 1]].pos;
                
                let edge1 = v1 - v0;
                let edge2 = v2 - v0;
                let triangle_area = edge1.cross(&edge2).norm() / 2.0;
                area += triangle_area;
            }
        }
    }
    
    area
}

#[derive(Debug)]
struct GeometricValidity {
    valid_polygons: usize,
    degenerate_polygons: usize,
    self_intersecting_polygons: usize,
}

fn analyze_geometric_validity(mesh: &IndexedMesh<f64>) -> GeometricValidity {
    let mut valid_polygons = 0;
    let mut degenerate_polygons = 0;
    let mut self_intersecting_polygons = 0;
    
    for polygon in &mesh.polygons {
        if is_polygon_degenerate(mesh, polygon) {
            degenerate_polygons += 1;
        } else if is_polygon_self_intersecting(mesh, polygon) {
            self_intersecting_polygons += 1;
        } else {
            valid_polygons += 1;
        }
    }
    
    GeometricValidity {
        valid_polygons,
        degenerate_polygons,
        self_intersecting_polygons,
    }
}

fn is_polygon_degenerate(mesh: &IndexedMesh<f64>, polygon: &csgrs::IndexedMesh::IndexedPolygon<f64>) -> bool {
    if polygon.indices.len() < 3 {
        return true;
    }
    
    // Check for duplicate vertices
    for i in 0..polygon.indices.len() {
        for j in (i + 1)..polygon.indices.len() {
            let v1 = mesh.vertices[polygon.indices[i]].pos;
            let v2 = mesh.vertices[polygon.indices[j]].pos;
            if (v1 - v2).norm() < 1e-9 {
                return true;
            }
        }
    }
    
    // Check for collinear vertices (simplified)
    if polygon.indices.len() == 3 {
        let v0 = mesh.vertices[polygon.indices[0]].pos;
        let v1 = mesh.vertices[polygon.indices[1]].pos;
        let v2 = mesh.vertices[polygon.indices[2]].pos;
        
        let edge1 = v1 - v0;
        let edge2 = v2 - v0;
        let cross_product = edge1.cross(&edge2);
        
        return cross_product.norm() < 1e-9;
    }
    
    false
}

fn is_polygon_self_intersecting(mesh: &IndexedMesh<f64>, polygon: &csgrs::IndexedMesh::IndexedPolygon<f64>) -> bool {
    // Simplified self-intersection check
    // In a full implementation, this would check for edge-edge intersections
    if polygon.indices.len() < 4 {
        return false; // Triangles cannot self-intersect
    }
    
    // For now, assume no self-intersections (would need complex geometry algorithms)
    false
}
