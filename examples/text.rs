//! This example shows text usage

use csgrs::prelude::*;
use std::{fs, path::Path};

const PATH: &str = "stl/text";

fn main() {
    // Ensure the folder exists
    let _ = fs::create_dir_all(PATH);

    // 13) Text example (2D). Provide a valid TTF font data below:
    // (Replace "asar.ttf" with a real full .ttf file in your project.)
    let font_data = include_bytes!("../asar.ttf");
    #[cfg(feature = "truetype-text")]
    let text_csg = CSG::text("HELLO", font_data, 15.0, None);
    #[cfg(feature = "truetype-text")]
    write_example(&text_csg, "text_hello_2d");

    // Optionally extrude the text:
    #[cfg(feature = "truetype-text")]
    let text_extruded = text_csg.extrude(2.0);
    #[cfg(feature = "truetype-text")]
    write_example(&text_extruded, "text_hello_extruded");
}

fn write_example(shape: &CSG, name: &str) {
    let _ = fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        shape.to_stl_binary(name).unwrap()
    );
}
