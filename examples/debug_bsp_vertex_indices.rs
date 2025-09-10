use csgrs::IndexedMesh::IndexedMesh;
use csgrs::mesh::Mesh;
use csgrs::traits::CSG;

fn main() {
    println!("=== BSP Vertex Index Management Debug ===");
    
    // Create simple test shapes
    let cube = IndexedMesh::<()>::cube(2.0, None);
    let sphere = IndexedMesh::<()>::sphere(1.0, 16, 16, None);
    
    println!("Cube: {} vertices, {} polygons", cube.vertices.len(), cube.polygons.len());
    println!("Sphere: {} vertices, {} polygons", sphere.vertices.len(), sphere.polygons.len());

    // Debug vertex indices in polygons (first few only)
    println!("\nCube polygon indices (first 3):");
    for (i, poly) in cube.polygons.iter().take(3).enumerate() {
        println!("  Polygon {}: indices {:?}", i, poly.indices);
        // Verify indices are valid
        for &idx in &poly.indices {
            if idx >= cube.vertices.len() {
                println!("    ERROR: Invalid index {} >= {}", idx, cube.vertices.len());
            }
        }
    }

    println!("\nSphere polygon indices (first 3):");
    for (i, poly) in sphere.polygons.iter().take(3).enumerate() {
        println!("  Polygon {}: indices {:?}", i, poly.indices);
        // Verify indices are valid
        for &idx in &poly.indices {
            if idx >= sphere.vertices.len() {
                println!("    ERROR: Invalid index {} >= {}", idx, sphere.vertices.len());
            }
        }
    }
    
    // Test BSP tree creation
    println!("\n=== Testing BSP Tree Creation ===");
    
    // Create combined vertex array like intersection does
    let mut result_vertices = cube.vertices.clone();
    let original_vertex_count = result_vertices.len();

    // Create BSP tree for cube
    let mut a = csgrs::IndexedMesh::bsp::IndexedNode::from_polygons(&cube.polygons, &mut result_vertices);
    println!("After BSP tree A creation: {} vertices", result_vertices.len());

    // Create separate vertex array for sphere (this is the problem!)
    let mut b_vertices = sphere.vertices.clone();
    let mut b = csgrs::IndexedMesh::bsp::IndexedNode::from_polygons(&sphere.polygons, &mut b_vertices);
    println!("BSP tree B uses separate vertex array: {} vertices", b_vertices.len());
    
    // This is where the problem occurs - a and b use different vertex arrays
    println!("\nPROBLEM: BSP tree A uses result_vertices, BSP tree B uses b_vertices");
    println!("When we do BSP operations between A and B, indices don't match!");
    
    // Test the correct approach - combine vertex arrays first
    println!("\n=== Testing Correct Approach ===");

    let mut combined_vertices = cube.vertices.clone();
    let offset = combined_vertices.len();
    combined_vertices.extend(sphere.vertices.iter().cloned());

    // Remap sphere polygon indices
    let mut sphere_remapped = sphere.polygons.clone();
    for poly in &mut sphere_remapped {
        for idx in &mut poly.indices {
            *idx += offset;
        }
    }

    println!("Combined vertex array: {} vertices", combined_vertices.len());
    println!("Sphere indices remapped by offset {}", offset);

    // Verify remapped indices (first few only)
    println!("\nSphere remapped polygon indices (first 3):");
    for (i, poly) in sphere_remapped.iter().take(3).enumerate() {
        println!("  Polygon {}: indices {:?}", i, poly.indices);
        // Verify indices are valid
        for &idx in &poly.indices {
            if idx >= combined_vertices.len() {
                println!("    ERROR: Invalid index {} >= {}", idx, combined_vertices.len());
            }
        }
    }

    // Now create BSP trees using the same vertex array
    let mut combined_vertices_a = combined_vertices.clone();
    let mut combined_vertices_b = combined_vertices.clone();

    let a_correct = csgrs::IndexedMesh::bsp::IndexedNode::from_polygons(&cube.polygons, &mut combined_vertices_a);
    let b_correct = csgrs::IndexedMesh::bsp::IndexedNode::from_polygons(&sphere_remapped, &mut combined_vertices_b);

    println!("BSP tree A (correct): {} vertices", combined_vertices_a.len());
    println!("BSP tree B (correct): {} vertices", combined_vertices_b.len());
    
    // Compare with regular Mesh intersection
    println!("\n=== Comparing with Regular Mesh ===");
    let mesh_cube = cube.to_mesh();
    let mesh_sphere = sphere.to_mesh();
    let mesh_intersection = mesh_cube.intersection(&mesh_sphere);
    println!("Regular Mesh intersection: {} polygons", mesh_intersection.polygons.len());

    // Test IndexedMesh intersection
    let indexed_intersection = cube.intersection_indexed(&sphere);
    println!("IndexedMesh intersection: {} polygons", indexed_intersection.polygons.len());
    
    if mesh_intersection.polygons.len() != indexed_intersection.polygons.len() {
        println!("MISMATCH: Different polygon counts between Mesh and IndexedMesh!");
    } else {
        println!("SUCCESS: Same polygon count");
    }
}
