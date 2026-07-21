//! Per-constructor profile benchmarks for isolating exact-geometry costs.

mod support;

use std::hint::black_box;

use csgrs::{Real, csg::CSG, sketch::Profile};
use hypercurve::Point2;
use support::{Config, Measurement, print_header};

fn run_profile(
    config: &Config,
    case: &str,
    iterations: usize,
    mut build: impl FnMut() -> Profile,
) {
    config.run("profile_primitives", "constructor", case, iterations, || {
        let profile = black_box(build());
        let contours = profile.material_contour_count() + profile.hole_contour_count();
        let wires = profile.wires().len() + profile.curve_paths().len();
        Measurement::new(
            1,
            (contours + wires) as u64,
            ((contours as u64) << 32) ^ wires as u64,
        )
    });
}

fn main() {
    print_header();
    let config = Config::from_env();
    let polygon = [
        [Real::from(0), Real::from(0)],
        [Real::from(4), Real::from(0)],
        [Real::from(2), Real::from(3)],
    ];
    let polygon_points = [
        Point2::new(Real::from(0), Real::from(0)),
        Point2::new(Real::from(4), Real::from(0)),
        Point2::new(Real::from(2), Real::from(3)),
    ];
    let bezier_control = [
        [Real::from(0), Real::from(0)],
        [Real::from(1), Real::from(2)],
        [Real::from(2), Real::from(2)],
        [Real::from(3), Real::from(0)],
    ];

    run_profile(&config, "rectangle", 64, || {
        Profile::rectangle(Real::from(12), Real::from(8))
    });
    run_profile(&config, "square", 64, || Profile::square(Real::from(8)));
    run_profile(&config, "circle", 16, || Profile::circle(Real::from(4), 24));
    run_profile(&config, "right_triangle", 64, || {
        Profile::right_triangle(Real::from(6), Real::from(4))
    });
    run_profile(&config, "polygon", 64, || Profile::polygon(&polygon));
    run_profile(&config, "polygon_points", 64, || {
        Profile::polygon_points(&polygon_points)
    });
    run_profile(&config, "ellipse", 16, || {
        Profile::ellipse(Real::from(8), Real::from(4), 24)
    });
    run_profile(&config, "regular_ngon", 16, || {
        Profile::regular_ngon(7, Real::from(4))
    });
    run_profile(&config, "arrow", 64, || {
        Profile::arrow(Real::from(6), Real::from(2), Real::from(3), Real::from(4))
    });
    run_profile(&config, "trapezoid", 64, || {
        Profile::trapezoid(Real::from(4), Real::from(8), Real::from(4), Real::from(2))
    });
    run_profile(&config, "rounded_rectangle", 8, || {
        Profile::rounded_rectangle(Real::from(12_u8), Real::from(8_u8), Real::from(2_u8), 8)
    });
    run_profile(&config, "star", 16, || {
        Profile::star(12, Real::from(8_u8), Real::from(4_u8))
    });
    run_profile(&config, "teardrop", 8, || {
        Profile::teardrop(Real::from(6), Real::from(10), 24)
    });
    run_profile(&config, "egg", 8, || {
        Profile::egg(Real::from(6), Real::from(10), 24)
    });
    run_profile(&config, "squircle", 4, || {
        Profile::squircle(Real::from(8), Real::from(6), 24)
    });
    run_profile(&config, "keyhole", 4, || {
        Profile::keyhole(Real::from(4), Real::from(2), Real::from(6), 24)
    });
    run_profile(&config, "reuleaux", 2, || {
        Profile::reuleaux(3, Real::from(6), 24)
    });
    run_profile(&config, "ring", 4, || {
        Profile::ring(Real::from(6), Real::from(2), 24)
    });
    run_profile(&config, "pie_slice", 8, || {
        Profile::pie_slice(Real::from(4), Real::from(10), Real::from(100), 12)
    });
    run_profile(&config, "supershape", 4, || {
        Profile::supershape(
            Real::from(1),
            Real::from(1),
            Real::from(5),
            Real::from(2),
            Real::from(2),
            Real::from(2),
            32,
        )
    });
    run_profile(&config, "circle_with_keyway", 2, || {
        Profile::circle_with_keyway(Real::from(6), 24, Real::from(2), Real::from(2))
    });
    run_profile(&config, "circle_with_flat", 2, || {
        Profile::circle_with_flat(Real::from(6), 24, Real::from(2))
    });
    run_profile(&config, "circle_with_two_flats", 2, || {
        Profile::circle_with_two_flats(Real::from(6), 24, Real::from(2))
    });
    run_profile(&config, "bezier", 8, || Profile::bezier(&bezier_control, 16));
    run_profile(&config, "bspline", 8, || {
        Profile::bspline(&bezier_control, 3, 8)
    });
    run_profile(&config, "heart", 4, || {
        Profile::heart(Real::from(8), Real::from(8), 32)
    });
    run_profile(&config, "crescent", 2, || {
        Profile::crescent(Real::from(6), Real::from(4), Real::from(3), 24)
    });
    run_profile(&config, "involute_gear", 1, || {
        Profile::involute_gear(
            Real::from(2_u8),
            20,
            Real::from(20_u8),
            Real::zero(),
            Real::zero(),
            4,
        )
    });
    run_profile(&config, "cycloidal_gear", 1, || {
        Profile::cycloidal_gear(Real::from(2), 12, Real::from(1), Real::zero(), 4)
    });
    run_profile(&config, "involute_rack", 2, || {
        Profile::involute_rack(Real::from(2), 4, Real::from(20), Real::zero(), Real::zero())
    });
    run_profile(&config, "cycloidal_rack", 2, || {
        Profile::cycloidal_rack(Real::from(2), 4, Real::zero(), 8)
    });
    run_profile(&config, "airfoil_naca4", 4, || {
        Profile::airfoil_naca4(
            Real::from(2_u8),
            Real::from(4_u8),
            Real::from(12_u8),
            Real::from(20_u8),
            80,
        )
    });
    run_profile(&config, "hilbert_curve", 8, || {
        Profile::square(Real::from(8)).hilbert_curve(3, Real::from(1))
    });

    let cubic_wire = Profile::bezier(&bezier_control, 16);
    config.run(
        "profile_curves",
        "finite_projection",
        "cubic_bezier",
        8,
        || {
            let polylines = black_box(cubic_wire.wire_polylines());
            let points = polylines.iter().map(Vec::len).sum::<usize>();
            Measurement::new(1, points as u64, points as u64)
        },
    );
    let closed_control = [
        [Real::from(0), Real::from(0)],
        [Real::from(4), Real::from(0)],
        [Real::from(4), Real::from(4)],
        [Real::from(0), Real::from(0)],
    ];
    let curved_region = Profile::bezier(&closed_control, 16);
    config.run("profile_curves", "extrusion", "cubic_region", 4, || {
        let mesh = black_box(curved_region.extrude(Real::from(2), ()));
        Measurement::new(
            1,
            mesh.triangles().len() as u64,
            mesh.triangles().len() as u64,
        )
    });
    let disjoint =
        Profile::square(Real::from(2)).translate(Real::from(10), Real::zero(), Real::zero());
    config.run(
        "profile_curves",
        "boolean",
        "curved_line_arc_union",
        4,
        || {
            let result = black_box(curved_region.try_union(&disjoint).unwrap());
            Measurement::new(
                1,
                result.material_contour_count() as u64,
                result.as_curve_region().len() as u64,
            )
        },
    );
}
