//! **IndexedMesh Operations Example**
//!
//! Demonstrates the advanced vertex and polygon operations specifically
//! designed for IndexedMesh's indexed connectivity model.

use csgrs::IndexedMesh::{
    IndexedMesh,
    vertex::{IndexedVertex, IndexedVertexClustering, IndexedVertexOperations},
};
use csgrs::mesh::Mesh;

fn main() {
    println!("=== IndexedMesh Advanced Operations Demo ===\n");

    // Create a simple cube mesh and convert to IndexedMesh
    let cube_mesh = Mesh::<String>::cube(2.0, Some("cube".to_string()));
    let indexed_cube = IndexedMesh::from_polygons(&cube_mesh.polygons, cube_mesh.metadata);

    println!("Original cube:");
    println!("  Vertices: {}", indexed_cube.vertices.len());
    println!("  Polygons: {}", indexed_cube.polygons.len());

    // Demonstrate vertex operations
    demonstrate_vertex_operations(&indexed_cube);

    // Demonstrate polygon operations
    demonstrate_polygon_operations(&indexed_cube);

    // Demonstrate clustering operations
    demonstrate_clustering_operations(&indexed_cube);

    println!("\n=== Demo Complete ===");
}

fn demonstrate_vertex_operations(mesh: &IndexedMesh<String>) {
    println!("\n--- Vertex Operations ---");

    // Test vertex interpolation
    if mesh.vertices.len() >= 2 {
        let v1 = IndexedVertex::from(mesh.vertices[0]);
        let v2 = IndexedVertex::from(mesh.vertices[1]);

        let interpolated = v1.interpolate(&v2, 0.5);
        println!("Interpolated vertex between v0 and v1:");
        println!("  Position: {:?}", interpolated.pos);
        println!("  Normal: {:?}", interpolated.normal);

        let slerp_interpolated = v1.slerp_interpolate(&v2, 0.5);
        println!("SLERP interpolated vertex:");
        println!("  Position: {:?}", slerp_interpolated.pos);
        println!("  Normal: {:?}", slerp_interpolated.normal);
    }

    // Test weighted average
    let vertex_weights = vec![(0, 0.3), (1, 0.4), (2, 0.3)];
    if let Some(avg_vertex) =
        IndexedVertexOperations::weighted_average_by_indices(mesh, &vertex_weights)
    {
        println!("Weighted average vertex:");
        println!("  Position: {:?}", avg_vertex.pos);
        println!("  Normal: {:?}", avg_vertex.normal);
    }

    // Test barycentric interpolation
    if mesh.vertices.len() >= 3 {
        if let Some(barycentric_vertex) =
            IndexedVertexOperations::barycentric_interpolate_by_indices(
                mesh, 0, 1, 2, 0.33, 0.33, 0.34,
            )
        {
            println!("Barycentric interpolated vertex:");
            println!("  Position: {:?}", barycentric_vertex.pos);
            println!("  Normal: {:?}", barycentric_vertex.normal);
        }
    }

    // Test connectivity analysis
    for i in 0..3.min(mesh.vertices.len()) {
        let (valence, regularity) = IndexedVertexOperations::analyze_connectivity(mesh, i);
        println!(
            "Vertex {} connectivity: valence={}, regularity={:.3}",
            i, valence, regularity
        );
    }

    // Test curvature estimation
    for i in 0..3.min(mesh.vertices.len()) {
        let curvature = IndexedVertexOperations::estimate_mean_curvature(mesh, i);
        println!("Vertex {} mean curvature: {:.6}", i, curvature);
    }
}

fn demonstrate_polygon_operations(mesh: &IndexedMesh<String>) {
    println!("\n--- Polygon Operations ---");

    if let Some(polygon) = mesh.polygons.first() {
        println!("First polygon has {} vertices", polygon.indices.len());

        // Test triangulation using existing method
        let triangles = polygon.triangulate(&mesh.vertices);
        println!(
            "First polygon triangulated into {} triangles",
            triangles.len()
        );
        for (i, triangle) in triangles.iter().take(3).enumerate() {
            println!("  Triangle {}: indices {:?}", i, triangle);
        }

        // Test edge iteration using existing method
        println!("First polygon edges:");
        for (i, (start, end)) in polygon.edges().take(5).enumerate() {
            println!("  Edge {}: {} -> {}", i, start, end);
        }

        // Test bounding box
        let bbox = polygon.bounding_box(&mesh.vertices);
        println!(
            "First polygon bounding box: min={:?}, max={:?}",
            bbox.mins, bbox.maxs
        );

        // Test normal calculation
        let normal = polygon.calculate_new_normal(&mesh.vertices);
        println!("First polygon normal: {:?}", normal);
    }
}

fn demonstrate_clustering_operations(mesh: &IndexedMesh<String>) {
    println!("\n--- Clustering Operations ---");

    // Test k-means clustering
    let k = 3;
    let assignments = IndexedVertexClustering::k_means_clustering(mesh, k, 10, 1.0, 0.5);

    if !assignments.is_empty() {
        println!("K-means clustering (k={}):", k);
        let mut cluster_counts = vec![0; k];
        for &assignment in &assignments {
            if assignment < k {
                cluster_counts[assignment] += 1;
            }
        }
        for (i, count) in cluster_counts.iter().enumerate() {
            println!("  Cluster {}: {} vertices", i, count);
        }
    }

    // Test hierarchical clustering
    let clusters = IndexedVertexClustering::hierarchical_clustering(mesh, 1.0);
    println!("Hierarchical clustering (threshold=1.0):");
    println!("  Number of clusters: {}", clusters.len());
    for (i, cluster) in clusters.iter().take(5).enumerate() {
        println!("  Cluster {}: {} vertices", i, cluster.len());
    }

    // Test vertex cluster creation
    if mesh.vertices.len() >= 4 {
        let indices = vec![0, 1, 2, 3];
        if let Some(cluster) =
            csgrs::IndexedMesh::vertex::IndexedVertexCluster::from_indices(mesh, indices)
        {
            println!("Created vertex cluster:");
            println!("  Vertices: {}", cluster.vertex_indices.len());
            println!("  Centroid: {:?}", cluster.centroid);
            println!("  Normal: {:?}", cluster.normal);
            println!("  Radius: {:.6}", cluster.radius);

            let (compactness, normal_consistency, density) = cluster.quality_metrics(mesh);
            println!("  Quality metrics:");
            println!("    Compactness: {:.6}", compactness);
            println!("    Normal consistency: {:.6}", normal_consistency);
            println!("    Density: {:.6}", density);
        }
    }
}
