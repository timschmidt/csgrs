//! Example demonstrating IndexedMesh connectivity analysis functionality.
//!
//! This example shows how to:
//! 1. Create an IndexedMesh from basic shapes
//! 2. Build connectivity analysis using build_connectivity
//! 3. Analyze vertex connectivity and mesh properties

use csgrs::IndexedMesh::{IndexedMesh, connectivity::VertexIndexMap};
use csgrs::mesh::plane::Plane;
use csgrs::mesh::vertex::Vertex;
use csgrs::traits::CSG;
use hashbrown::HashMap;
use nalgebra::{Point3, Vector3};
use std::fs;

fn main() {
    println!("IndexedMesh Connectivity Analysis Example");
    println!("==========================================");

    // Create a simple cube mesh as an example
    let cube = create_simple_cube();
    println!(
        "Created IndexedMesh with {} vertices and {} polygons",
        cube.vertices.len(),
        cube.polygons.len()
    );

    // Build connectivity analysis
    println!("\nBuilding connectivity analysis...");
    let (vertex_map, adjacency_map) = cube.build_connectivity();

    println!("Connectivity analysis complete:");
    println!("- Vertex map size: {}", vertex_map.position_to_index.len());
    println!("- Adjacency map size: {}", adjacency_map.len());

    // Analyze connectivity properties
    analyze_connectivity(&cube, &adjacency_map);

    // Analyze open edges specifically
    analyze_open_edges(&cube);

    // Demonstrate vertex analysis
    demonstrate_vertex_analysis(&cube, &adjacency_map, &vertex_map);

    // Compare normal handling between IndexedMesh and regular Mesh
    compare_mesh_vs_indexed_mesh_normals();

    // Demonstrate triangle subdivision
    demonstrate_subdivision();

    // Demonstrate CSG: subtract a cylinder from a cube
    demonstrate_csg_cube_minus_cylinder();

    // Demonstrate IndexedMesh connectivity issues
    demonstrate_indexed_mesh_connectivity_issues();

    // Export to STL
    println!("\nExporting to STL...");
    export_to_stl(&cube);
}

fn create_simple_cube() -> IndexedMesh<()> {
    // Define cube vertices with correct normals based on their face orientations
    let vertices = vec![
        Vertex::new(
            Point3::new(0.0, 0.0, 0.0),
            Vector3::new(-1.0, -1.0, -1.0).normalize(),
        ), // 0: bottom-front-left (corner)
        Vertex::new(
            Point3::new(1.0, 0.0, 0.0),
            Vector3::new(1.0, -1.0, -1.0).normalize(),
        ), // 1: bottom-front-right
        Vertex::new(
            Point3::new(1.0, 1.0, 0.0),
            Vector3::new(1.0, 1.0, -1.0).normalize(),
        ), // 2: bottom-back-right
        Vertex::new(
            Point3::new(0.0, 1.0, 0.0),
            Vector3::new(-1.0, 1.0, -1.0).normalize(),
        ), // 3: bottom-back-left
        Vertex::new(
            Point3::new(0.0, 0.0, 1.0),
            Vector3::new(-1.0, -1.0, 1.0).normalize(),
        ), // 4: top-front-left
        Vertex::new(
            Point3::new(1.0, 0.0, 1.0),
            Vector3::new(1.0, -1.0, 1.0).normalize(),
        ), // 5: top-front-right
        Vertex::new(
            Point3::new(1.0, 1.0, 1.0),
            Vector3::new(1.0, 1.0, 1.0).normalize(),
        ), // 6: top-back-right
        Vertex::new(
            Point3::new(0.0, 1.0, 1.0),
            Vector3::new(-1.0, 1.0, 1.0).normalize(),
        ), // 7: top-back-left
    ];

    // Define cube faces as indexed polygons (6 faces, each with 4 vertices)
    // Vertices are ordered counter-clockwise when viewed from outside the cube
    let polygons = vec![
        // Bottom face (z=0) - normal (0,0,-1) - viewed from below: counter-clockwise
        csgrs::IndexedMesh::IndexedPolygon::new(
            vec![0, 3, 2, 1],
            csgrs::IndexedMesh::plane::Plane::from_indexed_vertices(vec![
                vertices[0].clone(),
                vertices[3].clone(),
                vertices[2].clone(),
            ]),
            None,
        ),
        // Top face (z=1) - normal (0,0,1) - viewed from above: counter-clockwise
        csgrs::IndexedMesh::IndexedPolygon::new(
            vec![4, 5, 6, 7],
            csgrs::IndexedMesh::plane::Plane::from_indexed_vertices(vec![
                vertices[4].clone(),
                vertices[5].clone(),
                vertices[6].clone(),
            ]),
            None,
        ),
        // Front face (y=0) - normal (0,-1,0) - viewed from front: counter-clockwise
        csgrs::IndexedMesh::IndexedPolygon::new(
            vec![0, 1, 5, 4],
            csgrs::IndexedMesh::plane::Plane::from_indexed_vertices(vec![
                vertices[0].clone(),
                vertices[1].clone(),
                vertices[5].clone(),
            ]),
            None,
        ),
        // Back face (y=1) - normal (0,1,0) - viewed from back: counter-clockwise
        csgrs::IndexedMesh::IndexedPolygon::new(
            vec![3, 7, 6, 2],
            csgrs::IndexedMesh::plane::Plane::from_indexed_vertices(vec![
                vertices[3].clone(),
                vertices[7].clone(),
                vertices[6].clone(),
            ]),
            None,
        ),
        // Left face (x=0) - normal (-1,0,0) - viewed from left: counter-clockwise
        csgrs::IndexedMesh::IndexedPolygon::new(
            vec![0, 4, 7, 3],
            csgrs::IndexedMesh::plane::Plane::from_indexed_vertices(vec![
                vertices[0].clone(),
                vertices[4].clone(),
                vertices[7].clone(),
            ]),
            None,
        ),
        // Right face (x=1) - normal (1,0,0) - viewed from right: counter-clockwise
        csgrs::IndexedMesh::IndexedPolygon::new(
            vec![1, 2, 6, 5],
            csgrs::IndexedMesh::plane::Plane::from_indexed_vertices(vec![
                vertices[1].clone(),
                vertices[2].clone(),
                vertices[6].clone(),
            ]),
            None,
        ),
    ];

    let cube = IndexedMesh {
        vertices.iter().map(|v| csgrs::IndexedMesh::vertex::IndexedVertex::from(v.clone())).collect(),
        polygons,
        bounding_box: std::sync::OnceLock::new(),
        metadata: None,
    };

    // Ensure vertex normals match face normals for correct rendering
    let mut normalized_cube = cube.clone();
    normalized_cube.renormalize();

    normalized_cube
}

fn analyze_connectivity<S: Clone + Send + Sync + std::fmt::Debug>(
    mesh: &IndexedMesh<S>,
    adjacency_map: &HashMap<usize, Vec<usize>>,
) {
    println!("\nConnectivity Analysis:");
    println!("======================");

    // Count vertices by their valence (number of neighbors)
    let mut valence_counts = std::collections::HashMap::new();
    for neighbors in adjacency_map.values() {
        let valence = neighbors.len();
        *valence_counts.entry(valence).or_insert(0) += 1;
    }

    println!("Vertex valence distribution:");
    for valence in 0..=6 {
        if let Some(count) = valence_counts.get(&valence) {
            println!("  Valence {}: {} vertices", valence, count);
        }
    }

    // Check if mesh is manifold (each edge appears exactly twice)
    let mut edge_counts = std::collections::HashMap::new();
    for polygon in &mesh.polygons {
        for &(i, j) in &polygon.edges().collect::<Vec<_>>() {
            let edge = if i < j { (i, j) } else { (j, i) };
            *edge_counts.entry(edge).or_insert(0) += 1;
        }
    }

    let manifold_edges = edge_counts.values().filter(|&&count| count == 2).count();
    let boundary_edges = edge_counts.values().filter(|&&count| count == 1).count();
    let non_manifold_edges = edge_counts.values().filter(|&&count| count > 2).count();

    println!("\nMesh topology:");
    println!("  Total edges: {}", edge_counts.len());
    println!("  Manifold edges: {}", manifold_edges);
    println!("  Boundary edges: {}", boundary_edges);
    println!("  Non-manifold edges: {}", non_manifold_edges);

    if non_manifold_edges == 0 && boundary_edges == 0 {
        println!("  → Mesh is a closed manifold");
    } else if non_manifold_edges == 0 {
        println!("  → Mesh is manifold with boundary");
    } else {
        println!("  → Mesh has non-manifold edges");
    }
}

fn analyze_open_edges<S: Clone + Send + Sync + std::fmt::Debug>(mesh: &IndexedMesh<S>) {
    println!("\nOpen Edges Analysis:");
    println!("===================");

    // Build edge-to-face mapping
    let mut edge_to_faces = std::collections::HashMap::new();
    for (face_idx, polygon) in mesh.polygons.iter().enumerate() {
        for &(i, j) in &polygon.edges().collect::<Vec<_>>() {
            let edge = if i < j { (i, j) } else { (j, i) };
            edge_to_faces
                .entry(edge)
                .or_insert_with(Vec::new)
                .push(face_idx);
        }
    }

    // Find open edges (boundary edges that appear in only one face)
    let mut open_edges = Vec::new();
    let mut edge_face_counts = std::collections::HashMap::new();

    for (edge, faces) in &edge_to_faces {
        edge_face_counts.insert(*edge, faces.len());
        if faces.len() == 1 {
            open_edges.push(*edge);
        }
    }

    println!("Total edges: {}", edge_to_faces.len());
    println!("Open edges: {}", open_edges.len());
    println!("Closed edges: {}", edge_to_faces.len() - open_edges.len());

    if open_edges.is_empty() {
        println!("✓ Mesh has no open edges (closed manifold)");
    } else {
        println!("⚠ Mesh has {} open edges:", open_edges.len());

        // Group open edges by their connected components (boundary loops)
        let mut visited = std::collections::HashSet::new();
        let mut boundary_loops = Vec::new();

        for &start_edge in &open_edges {
            if visited.contains(&start_edge) {
                continue;
            }

            // Try to find a boundary loop starting from this edge
            let mut loop_edges = Vec::new();
            let mut current_edge = start_edge;
            let _found_loop = false;

            // Follow the boundary by finding adjacent open edges
            for _ in 0..open_edges.len() {
                // Prevent infinite loops
                if visited.contains(&current_edge) {
                    break;
                }
                visited.insert(current_edge);
                loop_edges.push(current_edge);

                // Find the next edge that shares a vertex with current_edge
                let mut next_edge = None;
                for &candidate_edge in &open_edges {
                    if visited.contains(&candidate_edge) {
                        continue;
                    }

                    // Check if candidate_edge shares a vertex with current_edge
                    let shares_vertex = candidate_edge.0 == current_edge.0
                        || candidate_edge.0 == current_edge.1
                        || candidate_edge.1 == current_edge.0
                        || candidate_edge.1 == current_edge.1;

                    if shares_vertex {
                        next_edge = Some(candidate_edge);
                        break;
                    }
                }

                if let Some(next) = next_edge {
                    current_edge = next;
                } else {
                    // No more connected edges found
                    break;
                }
            }

            if loop_edges.len() > 1 {
                boundary_loops.push(loop_edges);
            } else if loop_edges.len() == 1 {
                // Single edge (could be part of a larger boundary)
                boundary_loops.push(loop_edges);
            }
        }

        println!("\nBoundary Analysis:");
        println!("Found {} boundary loops/components", boundary_loops.len());

        for (loop_idx, loop_edges) in boundary_loops.iter().enumerate() {
            println!("\nBoundary Loop {}: {} edges", loop_idx + 1, loop_edges.len());

            // Show vertices in this boundary loop
            let mut all_vertices = std::collections::HashSet::new();
            for &(v1, v2) in loop_edges {
                all_vertices.insert(v1);
                all_vertices.insert(v2);
            }

            let mut vertex_list: Vec<_> = all_vertices.into_iter().collect();
            vertex_list.sort();

            print!("  Vertices: ");
            for (i, &vertex) in vertex_list.iter().enumerate() {
                if i > 0 {
                    print!(", ");
                }
                print!("{}", vertex);
            }
            println!();

            // Show edge details
            println!("  Edges:");
            for (i, &(v1, v2)) in loop_edges.iter().enumerate() {
                let faces = edge_to_faces.get(&(v1, v2)).unwrap();
                println!(
                    "    Edge {}: vertices ({}, {}) in face {}",
                    i + 1,
                    v1,
                    v2,
                    faces[0]
                );
            }
        }

        // Analyze boundary vertices
        let mut boundary_vertices = std::collections::HashSet::new();
        for &(v1, v2) in &open_edges {
            boundary_vertices.insert(v1);
            boundary_vertices.insert(v2);
        }

        println!("\nBoundary Vertices:");
        println!("Total boundary vertices: {}", boundary_vertices.len());
        println!("Total vertices in mesh: {}", mesh.vertices.len());
        println!(
            "Ratio: {:.1}%",
            (boundary_vertices.len() as f64 / mesh.vertices.len() as f64) * 100.0
        );

        // Check for isolated boundary edges
        let mut isolated_edges = Vec::new();
        for &edge in &open_edges {
            let mut connected_count = 0;
            for &other_edge in &open_edges {
                if edge != other_edge {
                    let shares_vertex = other_edge.0 == edge.0
                        || other_edge.0 == edge.1
                        || other_edge.1 == edge.0
                        || other_edge.1 == edge.1;
                    if shares_vertex {
                        connected_count += 1;
                    }
                }
            }
            if connected_count == 0 {
                isolated_edges.push(edge);
            }
        }

        if !isolated_edges.is_empty() {
            println!("\n⚠ Isolated boundary edges (not connected to other boundaries):");
            for (v1, v2) in isolated_edges {
                let faces = edge_to_faces.get(&(v1, v2)).unwrap();
                println!("  Edge ({}, {}) in face {}", v1, v2, faces[0]);
            }
        }
    }

    println!("\nOpen Edges Analysis Summary:");
    println!("- Open edges represent mesh boundaries or holes");
    println!("- Each open edge belongs to exactly one face");
    println!("- Boundary loops show connected sequences of open edges");
    println!("- Isolated edges may indicate mesh defects or separate components");
}

fn demonstrate_vertex_analysis<S: Clone + Send + Sync + std::fmt::Debug>(
    _mesh: &IndexedMesh<S>,
    adjacency_map: &HashMap<usize, Vec<usize>>,
    _vertex_map: &VertexIndexMap,
) {
    println!("\nVertex Analysis Examples:");
    println!("========================");

    // Analyze a few specific vertices
    let vertices_to_analyze = [0, 1, 4]; // corner, edge, and face vertices

    for &vertex_idx in &vertices_to_analyze {
        if let Some(neighbors) = adjacency_map.get(&vertex_idx) {
            let valence = neighbors.len();

            // Calculate vertex type based on valence
            let vertex_type = match valence {
                3 => "Corner/Boundary vertex",
                4 => "Edge vertex",
                5 => "Interior vertex (near boundary)",
                6 => "Interior vertex",
                _ => "Irregular vertex",
            };

            println!(
                "Vertex {}: {} neighbors - {}",
                vertex_idx, valence, vertex_type
            );

            // Show neighbor connections
            print!("  Connected to vertices: ");
            for (i, &neighbor) in neighbors.iter().enumerate() {
                if i > 0 {
                    print!(", ");
                }
                print!("{}", neighbor);
            }
            println!();
        }
    }

    println!("\nConnectivity analysis demonstrates:");
    println!("- Efficient vertex adjacency tracking");
    println!("- Mesh topology validation");
    println!("- Vertex type classification");
    println!("- Support for manifold detection");
    println!("- STL export capability");
}

fn compare_mesh_vs_indexed_mesh_normals() {
    println!("\nNormal Comparison: IndexedMesh vs Regular Mesh");
    println!("==============================================");

    let indexed_cube = create_simple_cube();
    let regular_mesh = indexed_cube.to_mesh();

    println!("IndexedMesh vertices with their normals:");
    for (i, vertex) in indexed_cube.vertices.iter().enumerate() {
        println!(
            "  Vertex {}: pos={:?}, normal={:?}",
            i, vertex.pos, vertex.normal
        );
    }

    println!("\nRegular Mesh triangles with their normals (after triangulation):");
    let triangulated_mesh = regular_mesh.triangulate();
    for (i, triangle) in triangulated_mesh.polygons.iter().enumerate() {
        println!("  Triangle {}: normal={:?}", i, triangle.vertices[0].normal);
        for (j, vertex) in triangle.vertices.iter().enumerate() {
            println!("    Vertex {}: pos={:?}", j, vertex.pos);
        }
    }

    println!("\nKey Differences:");
    println!("- IndexedMesh: Each vertex has a single normal (averaged from adjacent faces)");
    println!("- Regular Mesh: Each triangle vertex gets the face normal from triangulation");
    println!("- STL Export: Uses triangulated normals (face normals) for rendering");
}

fn demonstrate_subdivision() {
    println!("\nTriangle Subdivision Demonstration");
    println!("===================================");

    let indexed_cube = create_simple_cube();

    println!("Original cube mesh:");
    println!("  - {} vertices", indexed_cube.vertices.len());
    println!("  - {} polygons", indexed_cube.polygons.len());

    // Triangulate first
    let triangulated = indexed_cube.triangulate();
    println!("\nAfter triangulation:");
    println!("  - {} vertices", triangulated.vertices.len());
    println!("  - {} triangles", triangulated.polygons.len());

    // Apply one level of subdivision
    let subdivided = triangulated.subdivide_triangles(1.try_into().expect("not zero"));
    println!("\nAfter 1 level of subdivision:");
    println!("  - {} vertices", subdivided.vertices.len());
    println!("  - {} triangles", subdivided.polygons.len());

    // Apply two levels of subdivision
    let subdivided2 = triangulated.subdivide_triangles(2.try_into().expect("not zero"));
    println!("\nAfter 2 levels of subdivision:");
    println!("  - {} vertices", subdivided2.vertices.len());
    println!("  - {} triangles", subdivided2.polygons.len());

    println!("\nSubdivision Results:");
    println!("- Each triangle splits into 4 smaller triangles");
    println!("- Level 1: 12 triangles → 48 triangles (+36 new triangles)");
    println!("- Level 2: 48 triangles → 192 triangles (+144 new triangles)");
    println!("- Edge midpoints are shared between adjacent triangles");
    println!("- Vertex normals are interpolated at midpoints");
}

fn demonstrate_csg_cube_minus_cylinder() {
    println!("\nCSG Cube Minus Cylinder Demonstration");
    println!("=====================================");

    // Create a cube with side length 2.0
    let cube = csgrs::mesh::Mesh::<()>::cube(2.0, None);
    println!("Created cube: {} polygons", cube.polygons.len());

    // Create a cylinder that's longer than the cube
    // Radius 0.3, height 3.0 (cube is only 2.0 tall)
    let cylinder = csgrs::mesh::Mesh::<()>::cylinder(0.3, 3.0, 16, None);
    println!("Created cylinder: {} polygons", cylinder.polygons.len());

    // Position the cylinder in the center of the cube
    // Cube goes from (0,0,0) to (2,2,2), so center is at (1,1,1)
    // Cylinder is created along Z-axis from (0,0,0) to (0,0,3), so we need to:
    // 1. Translate it to center horizontally (x=1, y=1)
    // 2. Translate it down so it extends below the cube (z=-0.5 to start at z=-0.5)
    let positioned_cylinder = cylinder.translate(1.0, 1.0, -0.5);

    println!("Positioned cylinder at center of cube");

    // Perform the CSG difference operation: cube - cylinder
    let result = cube.difference(&positioned_cylinder);
    println!("After CSG difference: {} polygons", result.polygons.len());

    // Convert to IndexedMesh for analysis
    let indexed_result =
        csgrs::IndexedMesh::IndexedMesh::from_polygons(&result.polygons, result.metadata);
    println!(
        "Converted to IndexedMesh: {} vertices, {} polygons",
        indexed_result.vertices.len(),
        indexed_result.polygons.len()
    );

    // Analyze the result
    let (vertex_map, adjacency_map) = indexed_result.build_connectivity();
    println!(
        "Result connectivity: {} vertices, {} adjacency entries",
        vertex_map.position_to_index.len(),
        adjacency_map.len()
    );

    // Analyze open edges in the CSG result
    analyze_open_edges(&indexed_result);

    println!("\nCSG Operation Summary:");
    println!("- Original cube: 6 faces (12 triangles after triangulation)");
    println!("- Cylinder: 3 faces (bottom, top, sides) with 16 segments");
    println!("- Result: Cube with cylindrical hole through center");
    println!("- Hole extends beyond cube boundaries (cylinder height 3.0 > cube height 2.0)");

    // Export the CSG result to STL
    println!("\nExporting CSG result to STL...");
    export_csg_result(&result);
}

fn export_to_stl<S: Clone + Send + Sync + std::fmt::Debug>(indexed_mesh: &IndexedMesh<S>) {
    // Convert IndexedMesh to Mesh for STL export
    let mesh = indexed_mesh.to_mesh();
    if let Err(e) = fs::create_dir_all("stl") {
        println!("Warning: Could not create stl directory: {}", e);
        return;
    }

    // Export as binary STL
    match mesh.to_stl_binary("IndexedMesh_Cube") {
        Ok(stl_data) => match fs::write("stl/indexed_mesh_cube.stl", stl_data) {
            Ok(_) => println!("✓ Successfully exported binary STL: stl/indexed_mesh_cube.stl"),
            Err(e) => println!("✗ Failed to write binary STL file: {}", e),
        },
        Err(e) => println!("✗ Failed to generate binary STL: {}", e),
    }

    // Export as ASCII STL
    let stl_ascii = mesh.to_stl_ascii("IndexedMesh_Cube");
    match fs::write("stl/indexed_mesh_cube_ascii.stl", stl_ascii) {
        Ok(_) => {
            println!("✓ Successfully exported ASCII STL: stl/indexed_mesh_cube_ascii.stl")
        },
        Err(e) => println!("✗ Failed to write ASCII STL file: {}", e),
    }

    println!("  Mesh statistics:");
    println!("  - {} vertices", mesh.vertices().len());
    println!("  - {} polygons", mesh.polygons.len());
    println!(
        "  - {} triangles (after triangulation)",
        mesh.triangulate().polygons.len()
    );
}

fn export_csg_result<S: Clone + Send + Sync + std::fmt::Debug>(mesh: &csgrs::mesh::Mesh<S>) {
    // Export as binary STL
    match mesh.to_stl_binary("CSG_Cube_Minus_Cylinder") {
        Ok(stl_data) => match fs::write("stl/csg_cube_minus_cylinder.stl", stl_data) {
            Ok(_) => println!(
                "✓ Successfully exported CSG binary STL: stl/csg_cube_minus_cylinder.stl"
            ),
            Err(e) => println!("✗ Failed to write CSG binary STL file: {}", e),
        },
        Err(e) => println!("✗ Failed to generate CSG binary STL: {}", e),
    }

    // Export as ASCII STL
    let stl_ascii = mesh.to_stl_ascii("CSG_Cube_Minus_Cylinder");
    match fs::write("stl/csg_cube_minus_cylinder_ascii.stl", stl_ascii) {
        Ok(_) => println!(
            "✓ Successfully exported CSG ASCII STL: stl/csg_cube_minus_cylinder_ascii.stl"
        ),
        Err(e) => println!("✗ Failed to write CSG ASCII STL file: {}", e),
    }

    println!("  CSG Mesh statistics:");
    println!("  - {} vertices", mesh.vertices().len());
    println!("  - {} polygons", mesh.polygons.len());
    println!(
        "  - {} triangles (after triangulation)",
        mesh.triangulate().polygons.len()
    );
}

fn demonstrate_indexed_mesh_connectivity_issues() {
    println!("\nIndexedMesh Connectivity Issues Demonstration");
    println!("=============================================");

    // Create a simple cube as IndexedMesh
    let original_cube = create_simple_cube();
    println!("Original IndexedMesh cube:");
    println!("  - {} vertices", original_cube.vertices.len());
    println!("  - {} polygons", original_cube.polygons.len());

    // Analyze connectivity of original
    let (orig_vertex_map, orig_adjacency) = original_cube.build_connectivity();
    println!(
        "  - Connectivity: {} vertices, {} adjacency entries",
        orig_vertex_map.position_to_index.len(),
        orig_adjacency.len()
    );

    // Convert to regular Mesh and back to IndexedMesh (simulating CSG round-trip)
    let regular_mesh = original_cube.to_mesh();
    let reconstructed_cube = csgrs::IndexedMesh::IndexedMesh::from_polygons(
        &regular_mesh.polygons,
        regular_mesh.metadata,
    );

    println!("\nAfter Mesh ↔ IndexedMesh round-trip:");
    println!("  - {} vertices", reconstructed_cube.vertices.len());
    println!("  - {} polygons", reconstructed_cube.polygons.len());

    // Analyze connectivity of reconstructed
    let (recon_vertex_map, recon_adjacency) = reconstructed_cube.build_connectivity();
    println!(
        "  - Connectivity: {} vertices, {} adjacency entries",
        recon_vertex_map.position_to_index.len(),
        recon_adjacency.len()
    );

    // Check for issues
    let mut issues_found = Vec::new();

    // Check vertex count difference
    if original_cube.vertices.len() != reconstructed_cube.vertices.len() {
        issues_found.push(format!(
            "Vertex count changed: {} → {}",
            original_cube.vertices.len(),
            reconstructed_cube.vertices.len()
        ));
    }

    // Check for duplicate vertices that should have been merged
    let mut vertex_positions = std::collections::HashMap::new();
    for (i, vertex) in reconstructed_cube.vertices.iter().enumerate() {
        let key = (
            vertex.pos.x.to_bits(),
            vertex.pos.y.to_bits(),
            vertex.pos.z.to_bits(),
        );
        if let Some(&existing_idx) = vertex_positions.get(&key) {
            issues_found.push(format!(
                "Duplicate vertices at same position: indices {}, {}",
                existing_idx, i
            ));
        } else {
            vertex_positions.insert(key, i);
        }
    }

    // Check adjacency consistency
    for (vertex_idx, neighbors) in &orig_adjacency {
        if let Some(recon_neighbors) = recon_adjacency.get(vertex_idx) {
            if neighbors.len() != recon_neighbors.len() {
                issues_found.push(format!(
                    "Vertex {} adjacency changed: {} → {} neighbors",
                    vertex_idx,
                    neighbors.len(),
                    recon_neighbors.len()
                ));
            }
        } else {
            issues_found.push(format!("Vertex {} lost adjacency information", vertex_idx));
        }
    }

    if issues_found.is_empty() {
        println!("✓ No connectivity issues detected in round-trip conversion");
    } else {
        println!("⚠ Connectivity issues found:");
        for issue in issues_found {
            println!("  - {}", issue);
        }
    }

    // Demonstrate the issue with CSG operations
    println!("\nCSG Operation Connectivity Issues:");
    println!("===================================");

    let cube_mesh = csgrs::mesh::Mesh::<()>::cube(2.0, None);
    let cylinder_mesh = csgrs::mesh::Mesh::<()>::cylinder(0.3, 3.0, 16, None);
    let positioned_cylinder = cylinder_mesh.translate(1.0, 1.0, -0.5);
    let csg_result_mesh = cube_mesh.difference(&positioned_cylinder);

    // Convert CSG result to IndexedMesh
    let csg_indexed = csgrs::IndexedMesh::IndexedMesh::from_polygons(
        &csg_result_mesh.polygons,
        csg_result_mesh.metadata,
    );

    println!("CSG result as IndexedMesh:");
    println!("  - {} vertices", csg_indexed.vertices.len());
    println!("  - {} polygons", csg_indexed.polygons.len());

    // Analyze connectivity
    let (_csg_vertex_map, csg_adjacency) = csg_indexed.build_connectivity();

    // Check for isolated vertices (common issue after CSG)
    let isolated_count = csg_adjacency
        .values()
        .filter(|neighbors| neighbors.is_empty())
        .count();
    if isolated_count > 0 {
        println!(
            "⚠ Found {} isolated vertices (vertices with no adjacent faces)",
            isolated_count
        );
        println!(
            "  This is a common issue after CSG operations due to improper vertex welding"
        );
    }

    // Check for non-manifold edges
    let mut edge_count = std::collections::HashMap::new();
    for poly in &csg_indexed.polygons {
        for i in 0..poly.indices.len() {
            let a = poly.indices[i];
            let b = poly.indices[(i + 1) % poly.indices.len()];
            let edge = if a < b { (a, b) } else { (b, a) };
            *edge_count.entry(edge).or_insert(0) += 1;
        }
    }

    let non_manifold_count = edge_count.values().filter(|&&count| count > 2).count();
    if non_manifold_count > 0 {
        println!(
            "⚠ Found {} non-manifold edges (edges shared by more than 2 faces)",
            non_manifold_count
        );
        println!("  This indicates mesh topology issues after CSG operations");
    }

    // Summary of IndexedMesh connectivity issues
    println!("\nIndexedMesh Connectivity Issues Summary:");
    println!("=========================================");
    println!(
        "1. **CSG Round-trip Problem**: Converting Mesh ↔ IndexedMesh loses connectivity"
    );
    println!(
        "2. **Vertex Deduplication**: Bit-perfect comparison misses near-coincident vertices"
    );
    println!("3. **Adjacency Loss**: Edge connectivity information is not preserved");
    println!(
        "4. **Isolated Vertices**: CSG operations often create vertices with no adjacent faces"
    );
    println!("5. **Non-manifold Edges**: Boolean operations can create invalid mesh topology");
    println!("6. **Open Edges**: CSG naturally creates boundaries that need proper handling");

    println!("\n**Root Cause**: IndexedMesh CSG operations convert to regular Mesh,");
    println!("perform operations, then convert back using `from_polygons()` which doesn't");
    println!("robustly handle vertex welding or preserve connectivity information.");

    println!("\n**Impact**: Mesh analysis tools work correctly, but the underlying");
    println!("connectivity structure is compromised, leading to:");
    println!("- Inefficient storage (duplicate vertices)");
    println!("- Broken adjacency relationships");
    println!("- Invalid mesh topology for downstream processing");
    println!("- Poor performance in mesh operations");
}
