//! This example shows creating a `CSG` from a list of metaballs

use csgrs::{CSG, metaballs::MetaBall};
use std::{fs, path::Path};
use nalgebra::Point3;

const PATH: &str = "stl/metaballs";

fn main() {
    #[cfg(not(feature = "metaballs"))]
    compile_error!("This example requires the metaballs feature, try adding '--features metaballs'");

    // Ensure the folder exists
    let _ = fs::create_dir_all(PATH);

    // Suppose we want two overlapping metaballs
    let balls = &[
        MetaBall::new(Point3::origin(), 8.0),
        MetaBall::new(Point3::new(14.0, 0.0, 0.0), 8.0),
    ];

    let resolution = (65, 65, 65);
    let iso_value = 1.0;
    let padding = 1.5;

    let metaball_csg = CSG::metaballs(
        balls,
        resolution,
        iso_value,
        padding,
        None,
    );

    // For instance, save to STL
    // let stl_data = metaball_csg.to_stl_binary("my_metaballs").unwrap();
    // std::fs::write("stl/metaballs.stl", stl_data)
    //     .expect("Failed to write metaballs.stl");
    write_example(&metaball_csg, "two_overlapping_metaballs");

    // Now suppose we want 4 far metaballs
    let balls = &[
        MetaBall::new(Point3::origin(), 8.0),
        MetaBall::new(Point3::new(20.0, 0.0, 0.0), 8.0),
        MetaBall::new(Point3::new(0.0, 20.0, 0.0), 8.0),
        MetaBall::new(Point3::new(-1.0, -1.0, 20.0), 8.0),
    ];

    let resolution = (72, 72, 72);
    let iso_value = 1.0;
    let padding = 1.9;

    let metaball_csg = CSG::metaballs(
        balls,
        resolution,
        iso_value,
        padding,
        None,
    );

    write_example(&metaball_csg, "four_metaballs");
}

fn write_example(shape: &CSG, name: &str) {
    let _ = fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        shape.to_stl_binary(name).unwrap()
    );
}
