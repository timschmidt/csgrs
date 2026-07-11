//! Basic 2D sketches, including offset variants, exported as flat STL files.

use csgrs::profile::Profile;
use hyperlattice::Real;
use std::{fs, path::Path};

const PATH: &str = "stl/examples/basic2d_shapes";

fn main() {
    fs::create_dir_all(PATH).unwrap();

    let square = Profile::<()>::square(r(2.0), ());
    let circle = Profile::<()>::circle(r(1.0), 64, ());
    let ring = Profile::<()>::ring(r(2.0), r(0.25), 64, ());
    let keyhole = Profile::<()>::keyhole(r(1.0), r(0.4), r(1.5), 32, ());

    write_sketch(&square, "square");
    write_sketch(&circle, "circle");
    write_sketch(&ring, "ring");
    write_sketch(&keyhole, "keyhole");

    #[cfg(feature = "offset")]
    {
        write_sketch(&square.offset(r(0.2)), "square_offset_out");
        write_sketch(&circle.offset(r(-0.15)), "circle_offset_in");
    }
}

fn r(value: f64) -> Real {
    Real::try_from(value).expect("example values must be finite")
}

fn write_sketch(sketch: &Profile<()>, name: &str) {
    fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        sketch.to_stl_binary(name).unwrap(),
    )
    .unwrap();
}
