//! This example shows distributing a supershape along a grid

use csgrs::csg::CSG;
use std::{fs, path::Path};

const PATH: &str = "stl/distribute_grid";

fn main() {
    // Ensure the folder exists
    let _ = fs::create_dir_all(PATH);

    // Create a supershape
    let sshape = CSG::supershape(1.0, 1.0, 6.0, 1.0, 1.0, 1.0, 128, None);

    // Make a 4x4 grid of the supershape
    let grid_of_ss = sshape.distribute_grid(4, 4, 3.0, 3.0);
    write_example(&grid_of_ss, "grid_of_ss");
}

fn write_example(shape: &CSG, name: &str) {
    let _ = fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        shape.to_stl_binary(name).unwrap(),
    );
}
