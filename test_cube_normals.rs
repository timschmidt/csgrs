use csgrs::IndexedMesh::IndexedMesh;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Create a simple cube
    let cube = IndexedMesh::<()>::cube(2.0, None);

    println!("Cube planes:");
    for (i, polygon) in cube.polygons.iter().enumerate() {
        let normal = polygon.plane.normal();
        println!("Polygon {}: normal = ({}, {}, {})", i, normal.x, normal.y, normal.z);

        // Check if normal points outward
        let centroid = polygon.indices.iter()
            .map(|&idx| cube.vertices[idx].pos)
            .sum::<nalgebra::Point3<f32>>() / polygon.indices.len() as f32;

        let to_center = -centroid.coords; // From centroid to origin
        let dot_product = normal.dot(&to_center);

        println!("  Centroid: ({}, {}, {})", centroid.x, centroid.y, centroid.z);
        println!("  To center: ({}, {}, {})", to_center.x, to_center.y, to_center.z);
        println!("  Dot product: {}", dot_product);
        println!("  Points outward: {}", dot_product > 0.0);
        println!();
    }

    Ok(())
}
