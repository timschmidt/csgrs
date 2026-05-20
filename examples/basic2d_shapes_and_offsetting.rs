//! Basic 2D sketches, including offset variants, exported as flat STL files.

use csgrs::sketch::Profile;
use std::{fs, path::Path};

const PATH: &str = "stl/examples/basic2d_shapes";

fn main() {
    fs::create_dir_all(PATH).unwrap();

    let square = Profile::<()>::square(2.0, ());
    let circle = Profile::<()>::circle(1.0, 64, ());
    let ring = Profile::<()>::ring(2.0, 0.25, 64, ());
    let keyhole = Profile::<()>::keyhole(1.0, 0.4, 1.5, 32, ());

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

fn write_sketch(sketch: &Profile<()>, name: &str) {
    fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        sketch.to_stl_binary(name).unwrap(),
    )
    .unwrap();
}
