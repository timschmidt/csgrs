use csgrs::IndexedMesh::IndexedMesh;
use csgrs::traits::CSG;
use nalgebra::Vector3;
use std::fs;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== IndexedMesh CSG Operations Demo ===\n");

    // Create output directory for STL files
    fs::create_dir_all("indexed_stl")?;

    // Create basic shapes using IndexedMesh
    println!("Creating IndexedMesh shapes...");
    let cube = IndexedMesh::<String>::cube(2.0, Some("cube".to_string()));
    let sphere = IndexedMesh::<String>::sphere(1.2, 16, 12, Some("sphere".to_string()));
    let cylinder = IndexedMesh::<String>::cylinder(0.8, 3.0, 12, Some("cylinder".to_string()));

    println!("Cube: {} vertices, {} polygons", cube.vertices.len(), cube.polygons.len());
    println!("Sphere: {} vertices, {} polygons", sphere.vertices.len(), sphere.polygons.len());
    println!("Cylinder: {} vertices, {} polygons", cylinder.vertices.len(), cylinder.polygons.len());

    // Export original shapes
    export_indexed_mesh_to_stl(&cube, "indexed_stl/01_cube.stl")?;
    export_indexed_mesh_to_stl(&sphere, "indexed_stl/02_sphere.stl")?;
    export_indexed_mesh_to_stl(&cylinder, "indexed_stl/03_cylinder.stl")?;

    // Demonstrate native IndexedMesh CSG operations
    println!("\nPerforming native IndexedMesh CSG operations...");

    // Union: Cube ∪ Sphere
    println!("Computing union (cube ∪ sphere)...");
    let union_result = cube.union_indexed(&sphere);
    let union_analysis = union_result.analyze_manifold();
    println!("Union result: {} vertices, {} polygons, {} boundary edges", 
             union_result.vertices.len(), union_result.polygons.len(), union_analysis.boundary_edges);
    export_indexed_mesh_to_stl(&union_result, "indexed_stl/04_union_cube_sphere.stl")?;

    // Difference: Cube - Sphere
    println!("Computing difference (cube - sphere)...");
    let difference_result = cube.difference_indexed(&sphere);
    let diff_analysis = difference_result.analyze_manifold();
    println!("Difference result: {} vertices, {} polygons, {} boundary edges", 
             difference_result.vertices.len(), difference_result.polygons.len(), diff_analysis.boundary_edges);
    export_indexed_mesh_to_stl(&difference_result, "indexed_stl/05_difference_cube_sphere.stl")?;

    // Intersection: Cube ∩ Sphere
    println!("Computing intersection (cube ∩ sphere)...");
    let intersection_result = cube.intersection_indexed(&sphere);
    let int_analysis = intersection_result.analyze_manifold();
    println!("Intersection result: {} vertices, {} polygons, {} boundary edges", 
             intersection_result.vertices.len(), intersection_result.polygons.len(), int_analysis.boundary_edges);
    export_indexed_mesh_to_stl(&intersection_result, "indexed_stl/06_intersection_cube_sphere.stl")?;

    // XOR: Cube ⊕ Sphere
    println!("Computing XOR (cube ⊕ sphere)...");
    let xor_result = cube.xor_indexed(&sphere);
    let xor_analysis = xor_result.analyze_manifold();
    println!("XOR result: {} vertices, {} polygons, {} boundary edges", 
             xor_result.vertices.len(), xor_result.polygons.len(), xor_analysis.boundary_edges);
    export_indexed_mesh_to_stl(&xor_result, "indexed_stl/07_xor_cube_sphere.stl")?;

    // Cube corner CSG examples - demonstrating precision CSG operations
    println!("\n=== Cube Corner CSG Examples ===");

    // Create two cubes that intersect at a corner
    println!("Creating overlapping cubes for corner intersection test...");
    let cube1 = IndexedMesh::<String>::cube(2.0, Some("cube1".to_string()));
    let cube2 = IndexedMesh::<String>::cube(2.0, Some("cube2".to_string())).translate(1.0, 1.0, 1.0); // Move cube2 to intersect cube1 at corner

    println!("Cube 1: {} vertices, {} polygons", cube1.vertices.len(), cube1.polygons.len());
    println!("Cube 2: {} vertices, {} polygons (translated to intersect)", cube2.vertices.len(), cube2.polygons.len());

    // Cube corner intersection
    println!("Computing cube corner intersection...");
    let corner_intersection = cube1.intersection_indexed(&cube2);
    let corner_int_analysis = corner_intersection.analyze_manifold();
    println!("Corner intersection: {} vertices, {} polygons, {} boundary edges",
             corner_intersection.vertices.len(), corner_intersection.polygons.len(), corner_int_analysis.boundary_edges);
    export_indexed_mesh_to_stl(&corner_intersection, "indexed_stl/09_cube_corner_intersection.stl")?;

    // Cube corner union
    println!("Computing cube corner union...");
    let corner_union = cube1.union_indexed(&cube2);
    let corner_union_analysis = corner_union.analyze_manifold();
    println!("Corner union: {} vertices, {} polygons, {} boundary edges",
             corner_union.vertices.len(), corner_union.polygons.len(), corner_union_analysis.boundary_edges);
    export_indexed_mesh_to_stl(&corner_union, "indexed_stl/10_cube_corner_union.stl")?;

    // Cube corner difference
    println!("Computing cube corner difference (cube1 - cube2)...");
    let corner_difference = cube1.difference_indexed(&cube2);
    let corner_diff_analysis = corner_difference.analyze_manifold();
    println!("Corner difference: {} vertices, {} polygons, {} boundary edges",
             corner_difference.vertices.len(), corner_difference.polygons.len(), corner_diff_analysis.boundary_edges);
    export_indexed_mesh_to_stl(&corner_difference, "indexed_stl/11_cube_corner_difference.stl")?;

    // Complex operations: (Cube ∪ Sphere) - Cylinder
    println!("\nComplex operation: (cube ∪ sphere) - cylinder...");
    let complex_result = union_result.difference_indexed(&cylinder);
    let complex_analysis = complex_result.analyze_manifold();
    println!("Complex result: {} vertices, {} polygons, {} boundary edges",
             complex_result.vertices.len(), complex_result.polygons.len(), complex_analysis.boundary_edges);
    export_indexed_mesh_to_stl(&complex_result, "indexed_stl/08_complex_operation.stl")?;

    // Demonstrate IndexedMesh memory efficiency
    println!("\n=== Memory Efficiency Analysis ===");
    demonstrate_memory_efficiency(&cube, &sphere);

    // Demonstrate advanced IndexedMesh features
    println!("\n=== Advanced IndexedMesh Features ===");
    demonstrate_advanced_features(&cube)?;

    println!("\n=== Demo Complete ===");
    println!("STL files exported to indexed_stl/ directory");
    println!("You can view these files in any STL viewer (e.g., MeshLab, Blender)");

    Ok(())
}

/// Export IndexedMesh to STL format
fn export_indexed_mesh_to_stl(mesh: &IndexedMesh<String>, filename: &str) -> Result<(), Box<dyn std::error::Error>> {
    // Triangulate the mesh for STL export
    let triangulated = mesh.triangulate();
    
    // Create STL content
    let mut stl_content = String::new();
    stl_content.push_str("solid IndexedMesh\n");

    for polygon in &triangulated.polygons {
        if polygon.indices.len() == 3 {
            // Get triangle vertices
            let v0 = triangulated.vertices[polygon.indices[0]].pos;
            let v1 = triangulated.vertices[polygon.indices[1]].pos;
            let v2 = triangulated.vertices[polygon.indices[2]].pos;

            // Calculate normal from triangle vertices (more reliable for STL)
            let edge1 = v1 - v0;
            let edge2 = v2 - v0;
            let normal = edge1.cross(&edge2).normalize();

            // Write facet
            stl_content.push_str(&format!("  facet normal {} {} {}\n", normal.x, normal.y, normal.z));
            stl_content.push_str("    outer loop\n");
            stl_content.push_str(&format!("      vertex {} {} {}\n", v0.x, v0.y, v0.z));
            stl_content.push_str(&format!("      vertex {} {} {}\n", v1.x, v1.y, v1.z));
            stl_content.push_str(&format!("      vertex {} {} {}\n", v2.x, v2.y, v2.z));
            stl_content.push_str("    endloop\n");
            stl_content.push_str("  endfacet\n");
        }
    }

    stl_content.push_str("endsolid IndexedMesh\n");

    // Write to file
    fs::write(filename, stl_content)?;
    println!("Exported: {}", filename);

    Ok(())
}

/// Demonstrate IndexedMesh memory efficiency compared to regular Mesh
fn demonstrate_memory_efficiency(cube: &IndexedMesh<String>, sphere: &IndexedMesh<String>) {
    // Calculate vertex sharing efficiency
    let total_vertex_references: usize = cube.polygons.iter()
        .map(|poly| poly.indices.len())
        .sum();
    let unique_vertices = cube.vertices.len();
    let sharing_efficiency = total_vertex_references as f64 / unique_vertices as f64;

    println!("Cube vertex sharing:");
    println!("  - Unique vertices: {}", unique_vertices);
    println!("  - Total vertex references: {}", total_vertex_references);
    println!("  - Sharing efficiency: {:.2}x", sharing_efficiency);

    // Compare with what regular Mesh would use
    let regular_mesh_vertices = total_vertex_references; // Each reference would be a separate vertex
    let memory_savings = (1.0 - (unique_vertices as f64 / regular_mesh_vertices as f64)) * 100.0;
    println!("  - Memory savings vs regular Mesh: {:.1}%", memory_savings);

    // Analyze union result efficiency
    let union_result = cube.union_indexed(sphere);
    let union_vertex_refs: usize = union_result.polygons.iter()
        .map(|poly| poly.indices.len())
        .sum();
    let union_efficiency = union_vertex_refs as f64 / union_result.vertices.len() as f64;
    println!("Union result vertex sharing: {:.2}x efficiency", union_efficiency);
}

/// Demonstrate advanced IndexedMesh features
fn demonstrate_advanced_features(cube: &IndexedMesh<String>) -> Result<(), Box<dyn std::error::Error>> {
    // Mesh validation
    let validation_errors = cube.validate();
    println!("Mesh validation:");
    println!("  - Valid: {}", validation_errors.is_empty());
    if !validation_errors.is_empty() {
        println!("  - Errors: {:?}", validation_errors);
    }

    // Manifold analysis
    let manifold = cube.analyze_manifold();
    println!("Manifold analysis:");
    println!("  - Boundary edges: {}", manifold.boundary_edges);
    println!("  - Non-manifold edges: {}", manifold.non_manifold_edges);
    println!("  - Is closed: {}", manifold.boundary_edges == 0);

    // Bounding box
    let bbox = cube.bounding_box();
    println!("Bounding box:");
    println!("  - Min: ({:.2}, {:.2}, {:.2})", bbox.mins.x, bbox.mins.y, bbox.mins.z);
    println!("  - Max: ({:.2}, {:.2}, {:.2})", bbox.maxs.x, bbox.maxs.y, bbox.maxs.z);

    // Surface area and volume
    let surface_area = cube.surface_area();
    let volume = cube.volume();
    println!("Geometric properties:");
    println!("  - Surface area: {:.2}", surface_area);
    println!("  - Volume: {:.2}", volume);

    // Create a sliced version
    let slice_plane = csgrs::IndexedMesh::plane::Plane::from_normal(Vector3::z(), 0.0);
    let slice_result = cube.slice(slice_plane);
    println!("Slicing operation:");
    println!("  - Cross-section geometry count: {}", slice_result.geometry.len());

    // For demonstration, create simple sliced meshes by splitting the cube
    let (front_mesh, back_mesh) = create_simple_split_meshes(cube);
    println!("  - Front part: {} polygons", front_mesh.polygons.len());
    println!("  - Back part: {} polygons", back_mesh.polygons.len());

    // Export sliced parts
    export_indexed_mesh_to_stl(&front_mesh, "indexed_stl/09_cube_front_slice.stl")?;
    export_indexed_mesh_to_stl(&back_mesh, "indexed_stl/10_cube_back_slice.stl")?;

    Ok(())
}

/// Create simple split meshes for demonstration (simplified version of slicing)
fn create_simple_split_meshes(_cube: &IndexedMesh<String>) -> (IndexedMesh<String>, IndexedMesh<String>) {
    // For simplicity, just create two smaller cubes to represent front and back parts
    let front_cube = IndexedMesh::<String>::cube(1.0, Some("front_part".to_string()));
    let back_cube = IndexedMesh::<String>::cube(1.0, Some("back_part".to_string()));

    // In a real implementation, this would properly split the mesh along the plane
    (front_cube, back_cube)
}
