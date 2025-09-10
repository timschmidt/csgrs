use csgrs::IndexedMesh::IndexedMesh;
use csgrs::traits::CSG;
use std::fs;

fn main() {
    println!("=== Cube Corner Difference Test ===\n");

    // Create two cubes that intersect at a corner
    // Cube 1: positioned at origin, size 2x2x2
    let cube_corner1 = IndexedMesh::<()>::cube(2.0, None);
    println!("Cube 1: {} vertices, {} polygons", cube_corner1.vertices.len(), cube_corner1.polygons.len());

    // Cube 2: positioned to intersect cube1's corner at (1,1,1)
    // We'll translate it so its corner is at cube1's corner
    let cube_corner2 = IndexedMesh::<()>::cube(2.0, None).translate(1.0, 1.0, 1.0);
    println!("Cube 2: {} vertices, {} polygons (translated to intersect)", cube_corner2.vertices.len(), cube_corner2.polygons.len());

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

    // Compare with regular Mesh difference
    println!("\n=== Comparing with regular Mesh difference ===");
    let mesh_cube1 = cube_corner1.to_mesh();
    let mesh_cube2 = cube_corner2.to_mesh();
    let mesh_difference = mesh_cube1.difference(&mesh_cube2);
    println!("Regular Mesh difference: {} polygons", mesh_difference.polygons.len());
    let mesh_obj = mesh_difference.to_obj("mesh_difference");
    fs::write("mesh_difference.obj", mesh_obj).unwrap();

    // Debug: Check regular mesh polygon planes
    println!("\n=== Regular Mesh polygon planes ===");
    for (i, polygon) in mesh_difference.polygons.iter().enumerate() {
        println!("Polygon {}: normal={:?}, w={}", i, polygon.plane.normal(), polygon.plane.offset());
    }

    // Perform IndexedMesh difference (cube1 - cube2)
    println!("\n=== Performing IndexedMesh difference (cube1 - cube2) ===");
    let difference_result = cube_corner1.difference_indexed(&cube_corner2);
    println!("IndexedMesh difference: {} vertices, {} polygons", difference_result.vertices.len(), difference_result.polygons.len());

    // Debug: Check polygon planes to understand the geometry
    println!("\n=== Checking polygon planes ===");
    for (i, polygon) in difference_result.polygons.iter().enumerate() {
        let plane = &polygon.plane;
        println!("Polygon {}: normal={:?}, w={}", i, plane.normal, plane.w);
    }

    // Export difference result
    let difference_mesh = difference_result.to_mesh();
    let obj_data = difference_mesh.to_obj("cube_corner_difference");
    fs::write("cube_corner_difference.obj", obj_data).unwrap();
    let stl_data = difference_mesh.to_stl_ascii("cube_corner_difference");
    fs::write("cube_corner_difference.stl", stl_data).unwrap();
    println!("Exported difference to cube_corner_difference.obj and .stl");

    // Export individual cubes for comparison
    let cube1_mesh = cube_corner1.to_mesh();
    let cube2_mesh = cube_corner2.to_mesh();
    let cube1_obj = cube1_mesh.to_obj("cube1");
    let cube2_obj = cube2_mesh.to_obj("cube2");
    fs::write("cube1.obj", cube1_obj).unwrap();
    fs::write("cube2.obj", cube2_obj).unwrap();
    println!("Exported individual cubes for comparison");

    println!("\n=== Difference Test Complete ===");
}
