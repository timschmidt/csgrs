//! Basic 2D sketches, including offset variants, exported as flat STL files.

use csgrs::sketch::Sketch;
use std::{fs, path::Path};

const PATH: &str = "stl/examples/basic2d_shapes";

fn main() {
    fs::create_dir_all(PATH).unwrap();

    let square = Sketch::<()>::square(2.0, None);
    let circle = Sketch::<()>::circle(1.0, 64, None);
    let ring = Sketch::<()>::ring(2.0, 0.25, 64, None);
    let keyhole = Sketch::<()>::keyhole(1.0, 0.4, 1.5, 32, None);

    write_sketch(&square, "square");
    write_sketch(&circle, "circle");
    write_sketch(&ring, "ring");
    write_sketch(&keyhole, "keyhole");

    #[cfg(feature = "offset")]
    {
        write_sketch(&square.offset(0.2), "square_offset_out");
        write_sketch(&circle.offset(-0.15), "circle_offset_in");
    }
}

fn write_sketch(sketch: &Sketch<()>, name: &str) {
    fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        sketch.to_stl_binary(name).unwrap(),
    )
    .unwrap();
}
