//! This example demos creating airfoils with naca2412 and naca0015 profiles

use csgrs::CSG;
use std::{fs, path::Path};

const PATH: &str = "stl/airfoil";

fn main() {
    // Ensure the folder exists
    let _ = fs::create_dir_all(PATH);

    // 2-D profile for NACA 2412, 1 m chord, 100 pts / surface
    let naca2412 = CSG::airfoil("2412", 1.0, 100, None);
    write_example(&naca2412, "naca2412");

    // quick solid wing rib 5 mm thick
    let rib = naca2412.extrude(0.005);
    write_example(&rib, "naca2412_extruded");

    // symmetric foil for a centerboard
    let naca0015 = CSG::airfoil("0015", 0.3, 80, None)
        .extrude_vector(nalgebra::Vector3::new(0.0, 0.0, 1.2));
    write_example(&naca0015, "naca0015");
}

fn write_example(shape: &CSG, name: &str) {
    let _ = fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        shape.to_stl_binary(name).unwrap(),
    );
}
