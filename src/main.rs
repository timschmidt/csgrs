// main.rs
//
// Minimal example that uses the csg library in lib.rs (same directory).

use std::fs::File;
use std::io::Write;

fn main() {
    // 1) Create some simple CSG shapes
    let cube = csgrs::CSG::cube(None);       // basic 2x2x2 cube (centered on 0)
    let sphere = csgrs::CSG::sphere(Some((&[0.0, 0.0, 0.0], 1.3, 16, 8)));

    // 2) Perform a constructive operation: let's do a difference
    let csg_result = cube.subtract(&sphere);

    // 3) Export the result to STL text
    let stl_data = csg_result.to_stl("cube_minus_sphere");

    // 4) Write the STL to disk
    let filename = "output.stl";
    let mut file = File::create(filename).expect("Failed to create file");
    file.write_all(stl_data.as_bytes()).expect("Failed to write STL");

    println!("Wrote STL data to {}", filename);
}

