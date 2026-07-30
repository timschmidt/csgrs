//! Basic 2D curves, including offset variants, exported as flat STL files.

use csgrs::{GeometryContext, curve, io::stl::to_stl_binary};
use hypercurve::CurveRegion2;
use hyperlattice::Real;
use std::{fs, path::Path};

const PATH: &str = "stl/examples/basic2d_shapes";

fn main() {
    fs::create_dir_all(PATH).unwrap();

    let square = curve::square(r(2.0));
    let circle = curve::circle(r(1.0), 64);
    let ring = curve::ring(r(2.0), r(0.25), 64, &GeometryContext::APPROXIMATE_512)
        .expect("ring Boolean")
        .into_value();
    let keyhole =
        curve::keyhole(r(1.0), r(0.4), r(1.5), 32, &GeometryContext::APPROXIMATE_512)
            .expect("keyhole Boolean")
            .into_value();

    write_curve(&square, "square");
    write_curve(&circle, "circle");
    write_curve(&ring, "ring");
    write_curve(&keyhole, "keyhole");

    #[cfg(feature = "offset")]
    {
        write_curve(
            &curve::offset(&square, r(0.2), &hypercurve::CurvePolicy::STRICT)
                .expect("square offset")
                .into_value(),
            "square_offset_out",
        );
        write_curve(
            &curve::offset(&circle, r(-0.15), &hypercurve::CurvePolicy::STRICT)
                .expect("circle offset")
                .into_value(),
            "circle_offset_in",
        );
    }
}

fn r(value: f64) -> Real {
    Real::try_from(value).expect("example values must be finite")
}

fn write_curve(region: &CurveRegion2, name: &str) {
    fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        to_stl_binary(
            &curve::try_triangulate(region, &csgrs::GeometryContext::STRICT)
                .unwrap()
                .into_value(),
            name,
        )
        .unwrap(),
    )
    .unwrap();
}
