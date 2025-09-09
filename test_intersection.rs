use csgrs::IndexedMesh::IndexedMesh;
use csgrs::traits::CSG;
use nalgebra::Point3;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Create shapes
    let cube = IndexedMesh::<()>::cube(2.0, None);
    let sphere = IndexedMesh::<()>::sphere(1.5, 8, 6, None);

    // Check bounding boxes
    let cube_bb = cube.bounding_box();
    let sphere_bb = sphere.bounding_box();

    println!("Cube bounding box: min={:?}, max={:?}", cube_bb.mins, cube_bb.maxs);
    println!("Sphere bounding box: min={:?}, max={:?}", sphere_bb.mins, sphere_bb.maxs);

    // Check if they intersect
    let intersects = cube_bb.intersects(&sphere_bb);
    println!("Bounding boxes intersect: {}", intersects);

    // Check specific cube corner
    let cube_corner = Point3::new(1.0, 1.0, 1.0); // Corner of 2x2x2 cube
    let sphere_center = Point3::new(0.0, 0.0, 0.0);
    let distance_to_corner = (cube_corner - sphere_center).norm();
    println!("Distance from sphere center to cube corner: {}", distance_to_corner);
    println!("Sphere radius: 1.5");
    println!("Corner is inside sphere: {}", distance_to_corner < 1.5);

    Ok(())
}
