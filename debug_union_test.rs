use csgrs::IndexedMesh::IndexedMesh;
use csgrs::traits::CSG;
use nalgebra::Vector3;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== Debug Cube-Sphere Union ===\n");

    // Create simple shapes
    let cube = IndexedMesh::<String>::cube(2.0, Some("cube".to_string()));
    let sphere = IndexedMesh::<String>::sphere(1.5, 8, 6, Some("sphere".to_string()));

    println!("Cube: {} vertices, {} polygons", cube.vertices.len(), cube.polygons.len());
    println!("Sphere: {} vertices, {} polygons", sphere.vertices.len(), sphere.polygons.len());

    // Export individual shapes
    csgrs::io::stl::MeshSTLWriter::new(&cube.to_mesh()).write_stl_file("debug_union/cube.stl")?;
    csgrs::io::stl::MeshSTLWriter::new(&sphere.to_mesh()).write_stl_file("debug_union/sphere.stl")?;

    // Perform union
    println!("\nPerforming union...");
    let union_result = cube.union_indexed(&sphere);

    println!("Union result: {} vertices, {} polygons", union_result.vertices.len(), union_result.polygons.len());

    // Export union result
    csgrs::io::stl::MeshSTLWriter::new(&union_result.to_mesh()).write_stl_file("debug_union/union.stl")?;

    // Check manifold properties
    let analysis = union_result.analyze_manifold();
    println!("Manifold analysis:");
    println!("  Is manifold: {}", analysis.is_manifold);
    println!("  Boundary edges: {}", analysis.boundary_edges);
    println!("  Non-manifold edges: {}", analysis.non_manifold_edges);
    println!("  Connected components: {}", analysis.connected_components);
    println!("  Consistent orientation: {}", analysis.consistent_orientation);

    println!("\nFiles exported to debug_union/ directory");
    println!("Use a 3D viewer to inspect the results");

    Ok(())
}
