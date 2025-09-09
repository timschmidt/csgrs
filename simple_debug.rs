use csgrs::IndexedMesh::IndexedMesh;
use csgrs::IndexedMesh::plane::Plane;
use csgrs::float_types::Real;
use nalgebra::{Point3, Vector3};

fn main() {
    println!("=== Simple Debug Test ===\n");

    // Create a simple cube
    let cube = IndexedMesh::<()>::cube(2.0, None);
    println!("Cube has {} polygons", cube.polygons.len());

    // Create a plane that should clip one corner of the cube
    // This plane represents part of a sphere surface
    let plane_point = Point3::new(1.5, 1.5, 1.5);
    let plane_normal = Vector3::new(0.577, 0.577, 0.577).normalize(); // Normalized vector
    let plane = Plane::from_normal(plane_normal, plane_normal.dot(&plane_point.coords));

    println!("Test plane: normal={:?}, w={}", plane.normal, plane.w);

    // Test which cube polygons are clipped by this plane
    let mut front_count = 0;
    let mut back_count = 0;
    let mut coplanar_count = 0;

    for (i, polygon) in cube.polygons.iter().enumerate() {
        let classification = plane.classify_polygon(polygon);
        match classification {
            1 => { // FRONT
                front_count += 1;
                println!("Polygon {}: FRONT", i);
            },
            2 => { // BACK
                back_count += 1;
                println!("Polygon {}: BACK", i);
            },
            0 => { // COPLANAR
                coplanar_count += 1;
                println!("Polygon {}: COPLANAR", i);
            },
            3 => { // SPANNING
                println!("Polygon {}: SPANNING", i);
                let (coplanar_front, coplanar_back, front_parts, back_parts) =
                    plane.split_indexed_polygon(polygon, &mut vec![]);
                println!("  Split into: {} front, {} back, {} coplanar_front, {} coplanar_back",
                    front_parts.len(), back_parts.len(), coplanar_front.len(), coplanar_back.len());
            },
            _ => println!("Polygon {}: UNKNOWN ({})", i, classification),
        }
    }

    println!("\nSummary:");
    println!("  Front polygons: {}", front_count);
    println!("  Back polygons: {}", back_count);
    println!("  Coplanar polygons: {}", coplanar_count);
}
