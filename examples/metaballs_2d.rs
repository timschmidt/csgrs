//! This example shows creating a `CSG` from a list of 2d metaballs

use csgrs::CSG;
use std::{fs, path::Path};
use nalgebra::Point2;

const PATH: &str = "stl/metaballs_2d";

fn main() {
    #[cfg(not(feature = "metaballs"))]
    compile_error!("This example requires the metaballs feature, try adding '--features metaballs'");

    // Ensure the folder exists
    let _ = fs::create_dir_all(PATH);

    // Create a 2D "metaball" shape from 3 circles
    let balls_2d = vec![
        (Point2::new(0.0, 0.0), 1.0),
        (Point2::new(1.5, 0.0), 1.0),
        (Point2::new(0.75, 1.0), 0.5),
    ];
    let mb2d = CSG::metaballs2d(&balls_2d, (100, 100), 1.0, 0.25, None);

    write_example(&mb2d, "three_overlapping_2d_metaballs");
}

fn write_example(shape: &CSG, name: &str) {
    let _ = fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        shape.to_stl_binary(name).unwrap()
    );
}
