//! This example shows signed distance field (sdf) usage

use csgrs::CSG;
use nalgebra::Point3;
use std::{fs, path::Path};

const PATH: &str = "stl/sdf";

fn main() {
    // Ensure the folder exists
    let _ = fs::create_dir_all(PATH);

    #[cfg(not(feature = "sdf"))]
    compile_error!("This example requires the sdf feature, try adding '--features sdf'");

    // Example SDF for a sphere of radius 2.5 centered at (0,0,0)
    let my_sdf = |p: &Point3<f64>| p.coords.norm() - 2.5;

    let resolution = (60, 60, 60);
    let min_pt = Point3::new(-2.7, -2.7, -2.7);
    let max_pt = Point3::new(2.7, 2.7, 2.7);
    let iso_value = 0.0; // Typically zero for SDF-based surfaces

    let csg_shape = CSG::sdf(my_sdf, resolution, min_pt, max_pt, iso_value, None);

    // Now `csg_shape` is your polygon mesh as a CSG you can union, subtract, or export:
    write_example(&csg_shape, "sdf_sphere");
}

fn write_example(shape: &CSG, name: &str) {
    let _ = fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        shape.to_stl_binary(name).unwrap(),
    );
}
