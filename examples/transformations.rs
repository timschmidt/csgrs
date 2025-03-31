//! This example shows transformations: Translate, Rotate, Scale, Mirror

use csgrs::csg::CSG;
use std::{fs, path::Path};
use nalgebra::Vector3;
use csgrs::plane::Plane;

const PATH: &str = "stl/transformations";

fn main() {
    // Ensure the /stls folder exists
    let _ = fs::create_dir_all(PATH);

    let cube = CSG::cube(2.0, 2.0, 2.0, None);
    write_example(&cube, "cube");

    // 2) Transformations: Translate, Rotate, Scale, Mirror
    let moved_cube = cube
        .translate(1.0, 0.0, 0.0)
        .rotate(0.0, 45.0, 0.0)
        .scale(1.0, 0.5, 2.0);
    write_example(&moved_cube, "cube_transformed");

    let plane_x = Plane { normal: Vector3::x(), intercept: 0.0 };
    let mirrored_cube = cube.mirror(plane_x);
    write_example(&mirrored_cube, "cube_mirrored_x");
}

fn write_example(shape: &CSG, name: &str) {
    let _ = fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        shape.to_stl_binary(name).unwrap()
    );
}
