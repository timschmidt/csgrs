use csgrs::IndexedMesh::IndexedMesh;
use csgrs::IndexedMesh::plane::Plane;
use csgrs::mesh::Mesh;
use csgrs::float_types::Real;
use nalgebra::{Point3, Vector3};

fn main() {
    println!("=== Splitting Logic Comparison Test ===\n");

    // Create a simple test case - a square that gets split by a diagonal plane
    let vertices = vec![
        Point3::new(0.0, 0.0, 0.0),  // bottom-left
        Point3::new(2.0, 0.0, 0.0),  // bottom-right
        Point3::new(2.0, 2.0, 0.0),  // top-right
        Point3::new(0.0, 2.0, 0.0),  // top-left
    ];

    // Create IndexedMesh version
    let indexed_vertices: Vec<_> = vertices.iter().enumerate().map(|(i, &pos)| {
        csgrs::IndexedMesh::vertex::IndexedVertex::new(pos, Vector3::z())
    }).collect();

    let indexed_square = csgrs::IndexedMesh::IndexedPolygon::new(
        vec![0, 1, 2, 3],
        Plane::from_points(vertices[0], vertices[1], vertices[2]),
        None::<()>
    );

    let mut indexed_mesh = IndexedMesh {
        vertices: indexed_vertices,
        polygons: vec![indexed_square],
        bounding_box: std::sync::OnceLock::new(),
        metadata: None,
    };

    // Create regular Mesh version
    let regular_vertices: Vec<_> = vertices.iter().map(|&pos| {
        csgrs::mesh::vertex::Vertex::new(pos, Vector3::z())
    }).collect();

    let regular_square = csgrs::mesh::polygon::Polygon::new(regular_vertices, None::<()>);

    let regular_mesh = Mesh {
        polygons: vec![regular_square],
        bounding_box: std::sync::OnceLock::new(),
        metadata: None,
    };

    // Create a diagonal plane that should split the square
    let diagonal_plane_point = Point3::new(1.0, 1.0, 0.0);
    let diagonal_plane_normal = Vector3::new(1.0, -1.0, 0.0).normalize();

    // IndexedMesh plane
    let indexed_plane = Plane::from_normal(diagonal_plane_normal, diagonal_plane_normal.dot(&diagonal_plane_point.coords));

    // Regular Mesh plane
    let regular_plane = csgrs::mesh::plane::Plane::from_normal(diagonal_plane_normal, diagonal_plane_normal.dot(&diagonal_plane_point.coords));

    println!("Test plane: normal={:?}", diagonal_plane_normal);
    println!("IndexedMesh plane: normal={:?}, w={}", indexed_plane.normal, indexed_plane.w);
    println!("Regular Mesh plane: normal={:?}, offset={}", regular_plane.normal(), regular_plane.offset());

    // Test vertex classifications
    println!("\n=== Vertex Classifications ===");
    for (i, &pos) in vertices.iter().enumerate() {
        let indexed_classification = indexed_plane.orient_point(&pos);
        let regular_classification = regular_plane.orient_point(&pos);

        let indexed_desc = match indexed_classification {
            0 => "COPLANAR",
            1 => "FRONT",
            2 => "BACK",
            _ => "UNKNOWN",
        };

        let regular_desc = match regular_classification {
            0 => "COPLANAR",
            1 => "FRONT",
            2 => "BACK",
            _ => "UNKNOWN",
        };

        println!("Vertex {}: IndexedMesh={}, Regular Mesh={}", i, indexed_desc, regular_desc);
    }

    // Test polygon classifications
    println!("\n=== Polygon Classifications ===");
    let indexed_poly_classification = indexed_plane.classify_polygon(&indexed_mesh.polygons[0], &indexed_mesh.vertices);
    let regular_poly_classification = regular_plane.classify_polygon(&regular_mesh.polygons[0]);

    let indexed_poly_desc = match indexed_poly_classification {
        0 => "COPLANAR",
        1 => "FRONT",
        2 => "BACK",
        3 => "SPANNING",
        _ => "UNKNOWN",
    };

    let regular_poly_desc = match regular_poly_classification {
        0 => "COPLANAR",
        1 => "FRONT",
        2 => "BACK",
        3 => "SPANNING",
        _ => "UNKNOWN",
    };

    println!("Polygon: IndexedMesh={}, Regular Mesh={}", indexed_poly_desc, regular_poly_desc);

    // Test splitting
    if indexed_poly_classification == 3 || regular_poly_classification == 3 {
        println!("\n=== Splitting Test ===");

        let mut temp_vertices = indexed_mesh.vertices.clone();
        let (cf, cb, f, b) = indexed_plane.split_indexed_polygon(&indexed_mesh.polygons[0], &mut temp_vertices);

        println!("IndexedMesh splitting results:");
        println!("  Coplanar front: {}", cf.len());
        println!("  Coplanar back: {}", cb.len());
        println!("  Front polygons: {}", f.len());
        println!("  Back polygons: {}", b.len());
        println!("  Total vertices added: {}", temp_vertices.len() - indexed_mesh.vertices.len());

        let (rcf, rcb, rf, rb) = regular_plane.split_polygon(&regular_mesh.polygons[0]);

        println!("Regular Mesh splitting results:");
        println!("  Coplanar front: {}", rcf.len());
        println!("  Coplanar back: {}", rcb.len());
        println!("  Front polygons: {}", rf.len());
        println!("  Back polygons: {}", rb.len());
    }
}
