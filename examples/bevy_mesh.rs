//! This example demos converting a `CSG` to a bevy mesh

use csgrs::CSG;
use std::{fs, path::Path};

const PATH: &str = "stl/bevymesh";

fn main() {
    #[cfg(not(feature = "bevymesh"))]
    compile_error!("This example requires the `bevymesh` feature to be enabled");

    #[cfg(feature = "bevymesh")]
    println!("{:#?}", bezier_3d.to_bevy_mesh());
}

fn write_example(shape: &CSG, name: &str) {
    let _ = fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        shape.to_stl_binary(name).unwrap(),
    );
}
