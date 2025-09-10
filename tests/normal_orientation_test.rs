use csgrs::IndexedMesh::IndexedMesh;
use nalgebra::{Vector3, Point3};

#[test]
fn test_normal_orientation_after_csg() {
    // Create basic shapes
    let cube = IndexedMesh::<String>::cube(2.0, Some("cube".to_string()));
    let sphere = IndexedMesh::<String>::sphere(1.2, 16, 12, Some("sphere".to_string()));

    // Verify original shapes have outward normals
    assert!(has_outward_normals(&cube), "Original cube should have outward normals");
    assert!(has_outward_normals(&sphere), "Original sphere should have outward normals");

    // Test all CSG operations
    let union_result = cube.union_indexed(&sphere);
    let difference_result = cube.difference_indexed(&sphere);
    let intersection_result = cube.intersection_indexed(&sphere);
    let xor_result = cube.xor_indexed(&sphere);

    // All CSG results should have outward normals
    assert!(has_outward_normals(&union_result), "Union result should have outward normals");
    assert!(has_outward_normals(&difference_result), "Difference result should have outward normals");
    assert!(has_outward_normals(&intersection_result), "Intersection result should have outward normals");
    assert!(has_outward_normals(&xor_result), "XOR result should have outward normals");

    println!("✅ All CSG operations produce meshes with correct outward-facing normals");
}

#[test]
fn test_ensure_consistent_winding_logic() {
    // Create a simple cube
    let mut cube = IndexedMesh::<String>::cube(2.0, Some("cube".to_string()));
    
    // Verify it starts with correct normals
    assert!(has_outward_normals(&cube), "Cube should start with outward normals");
    
    // Manually call ensure_consistent_winding (this should not change anything)
    cube.ensure_consistent_winding();
    
    // Should still have outward normals
    assert!(has_outward_normals(&cube), "Cube should still have outward normals after ensure_consistent_winding");
    
    println!("✅ ensure_consistent_winding preserves correct normal orientation");
}

#[test]
fn test_normal_orientation_complex_operations() {
    // Test more complex nested operations
    let cube = IndexedMesh::<String>::cube(2.0, Some("cube".to_string()));
    let sphere = IndexedMesh::<String>::sphere(1.2, 16, 12, Some("sphere".to_string()));
    let cylinder = IndexedMesh::<String>::cylinder(0.8, 3.0, 12, Some("cylinder".to_string()));

    // Complex operation: (cube ∪ sphere) - cylinder
    let union_result = cube.union_indexed(&sphere);
    let complex_result = union_result.difference_indexed(&cylinder);
    
    assert!(has_outward_normals(&complex_result), "Complex CSG result should have outward normals");
    
    println!("✅ Complex CSG operations maintain correct normal orientation");
}

/// Helper function to check if a mesh has predominantly outward-facing normals
fn has_outward_normals<S: Clone + Send + Sync + std::fmt::Debug>(mesh: &IndexedMesh<S>) -> bool {
    if mesh.polygons.is_empty() {
        return true; // Empty mesh is trivially correct
    }
    
    // Compute mesh centroid
    let centroid = compute_mesh_centroid(mesh);
    
    let mut outward_count = 0;
    let mut inward_count = 0;
    
    // Test each polygon's normal orientation
    for polygon in &mesh.polygons {
        // Compute polygon center
        let polygon_center = compute_polygon_center(polygon, &mesh.vertices);
        
        // Vector from polygon center to mesh centroid
        let to_centroid = centroid - polygon_center;
        
        // Dot product with polygon normal
        let normal = polygon.plane.normal();
        let dot_product = normal.dot(&to_centroid);
        
        if dot_product > 0.01 {
            inward_count += 1;
        } else if dot_product < -0.01 {
            outward_count += 1;
        }
    }
    
    // Consider normals "outward" if at least 80% are outward-pointing
    // This allows for some tolerance in complex geometries
    let total = outward_count + inward_count;
    if total == 0 {
        return true; // All coplanar, assume correct
    }
    
    let outward_ratio = outward_count as f64 / total as f64;
    outward_ratio >= 0.8
}

fn compute_mesh_centroid<S: Clone + Send + Sync + std::fmt::Debug>(mesh: &IndexedMesh<S>) -> Point3<f64> {
    if mesh.vertices.is_empty() {
        return Point3::origin();
    }
    
    let sum: Vector3<f64> = mesh.vertices.iter()
        .map(|v| v.pos.coords)
        .sum();
    Point3::from(sum / mesh.vertices.len() as f64)
}

fn compute_polygon_center<S: Clone + Send + Sync + std::fmt::Debug>(
    polygon: &csgrs::IndexedMesh::IndexedPolygon<S>, 
    vertices: &[csgrs::IndexedMesh::vertex::IndexedVertex]
) -> Point3<f64> {
    if polygon.indices.is_empty() {
        return Point3::origin();
    }
    
    let sum: Vector3<f64> = polygon.indices.iter()
        .filter_map(|&idx| {
            if idx < vertices.len() {
                Some(vertices[idx].pos.coords)
            } else {
                None
            }
        })
        .sum();
    
    Point3::from(sum / polygon.indices.len() as f64)
}
