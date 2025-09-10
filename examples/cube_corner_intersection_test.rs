use csgrs::IndexedMesh::IndexedMesh;
use csgrs::traits::CSG;
use std::fs;

fn main() {
    println!("=== Cube Corner Intersection Debug Test ===\n");

    // Create two cubes that intersect at a corner
    // Cube 1: positioned at origin, size 2x2x2
    let cube_corner1 = IndexedMesh::<()>::cube(2.0, None);
    println!("Cube 1: {} vertices, {} polygons", cube_corner1.vertices.len(), cube_corner1.polygons.len());

    // Cube 2: positioned to intersect cube1's corner at (1,1,1)
    // We'll translate it so its corner is at cube1's corner
    let cube_corner2 = IndexedMesh::<()>::cube(2.0, None).translate(1.0, 1.0, 1.0);
    println!("Cube 2: {} vertices, {} polygons (translated to intersect)", cube_corner2.vertices.len(), cube_corner2.polygons.len());

    // Debug: Check cube2 vertices after translation
    println!("Cube 2 vertices after translation:");
    for (i, vertex) in cube_corner2.vertices.iter().enumerate() {
        println!("  Vertex {}: {:?}", i, vertex.pos);
    }

    // Check if vertices are shared/intersecting
    println!("\n=== Checking for shared vertices ===");
    let mut shared_vertices = 0;
    for v1 in &cube_corner1.vertices {
        for v2 in &cube_corner2.vertices {
            let distance = (v1.pos - v2.pos).magnitude();
            if distance < 1e-6 {
                shared_vertices += 1;
                println!("Shared vertex at: {:?}", v1.pos);
                break;
            }
        }
    }
    println!("Found {} shared vertices", shared_vertices);

    // Choose which operation to test
    let test_intersection = false; // Set to true to test intersection instead

    if test_intersection {
        // Perform intersection
        println!("\n=== Performing intersection ===");
        let intersection_result = cube_corner1.intersection_indexed(&cube_corner2);
        println!("Intersection result: {} vertices, {} polygons", intersection_result.vertices.len(), intersection_result.polygons.len());

        // Export intersection result
        let intersection_mesh = intersection_result.to_mesh();
        let obj_data = intersection_mesh.to_obj("cube_corner_intersection");
        fs::write("cube_corner_intersection.obj", obj_data).unwrap();
        let stl_data = intersection_mesh.to_stl_ascii("cube_corner_intersection");
        fs::write("cube_corner_intersection.stl", stl_data).unwrap();
        println!("Exported intersection to cube_corner_intersection.obj and .stl");
    } else {
        // Compare with regular Mesh difference
        println!("\n=== Comparing with regular Mesh difference ===");
        let mesh_cube1 = cube_corner1.to_mesh();
        let mesh_cube2 = cube_corner2.to_mesh();
        let mesh_difference = mesh_cube1.difference(&mesh_cube2);
        println!("Regular Mesh difference: {} polygons", mesh_difference.polygons.len());
        let mesh_obj = mesh_difference.to_obj("mesh_difference");
        fs::write("mesh_difference.obj", mesh_obj).unwrap();

        // Perform IndexedMesh difference (cube1 - cube2)
        println!("\n=== Performing IndexedMesh difference (cube1 - cube2) ===");
        let difference_result = cube_corner1.difference_indexed(&cube_corner2);
        println!("IndexedMesh difference: {} vertices, {} polygons", difference_result.vertices.len(), difference_result.polygons.len());

        // Export difference result
        let difference_mesh = difference_result.to_mesh();
        let obj_data = difference_mesh.to_obj("cube_corner_difference");
        fs::write("cube_corner_difference.obj", obj_data).unwrap();
        let stl_data = difference_mesh.to_stl_ascii("cube_corner_difference");
        fs::write("cube_corner_difference.stl", stl_data).unwrap();
        println!("Exported difference to cube_corner_difference.obj and .stl");
    }

    // Export individual cubes for comparison
    let cube1_mesh = cube_corner1.to_mesh();
    let cube2_mesh = cube_corner2.to_mesh();
    let cube1_obj = cube1_mesh.to_obj("cube1");
    let cube2_obj = cube2_mesh.to_obj("cube2");
    fs::write("cube1.obj", cube1_obj).unwrap();
    fs::write("cube2.obj", cube2_obj).unwrap();
    println!("Exported individual cubes for comparison");

    // Also export the individual cubes for comparison
    let cube1_mesh = cube_corner1.to_mesh();
    let cube2_mesh = cube_corner2.to_mesh();
    let cube1_obj = cube1_mesh.to_obj("cube1");
    let cube2_obj = cube2_mesh.to_obj("cube2");
    fs::write("cube1.obj", cube1_obj).unwrap();
    fs::write("cube2.obj", cube2_obj).unwrap();
    println!("Exported individual cubes for comparison");

    println!("\n=== Test Complete ===");
}
