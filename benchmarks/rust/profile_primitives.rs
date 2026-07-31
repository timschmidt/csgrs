//! Per-constructor profile benchmarks for isolating exact-geometry costs.

#[path = "../support/harness.rs"]
mod support;

use std::hint::black_box;

use csgrs::{GeometryContext, Real, curve, curve::CurveRegionExt};
use hypercurve::{CurvePath2, CurvePolicy, CurveRegion2, FiniteProjectionOptions, Point2};
use hyperlattice::Matrix4;
use support::{Config, Measurement, print_header};

fn run_profile(
    config: &Config,
    case: &str,
    iterations: usize,
    mut build: impl FnMut() -> CurveRegion2,
) {
    config.run("profile_primitives", "constructor", case, iterations, || {
        let profile = black_box(build());
        let contours = profile.len();
        let wires = 0;
        Measurement::new(
            1,
            (contours + wires) as u64,
            ((contours as u64) << 32) ^ wires as u64,
        )
    });
}

fn run_path(
    config: &Config,
    case: &str,
    iterations: usize,
    mut build: impl FnMut() -> CurvePath2,
) {
    config.run("profile_primitives", "constructor", case, iterations, || {
        let path = black_box(build());
        Measurement::new(1, path.curves().len() as u64, path.curves().len() as u64)
    });
}

fn compound_region(
    result: Result<csgrs::GeometryOutcome<CurveRegion2>, csgrs::errors::CurveBooleanError>,
) -> CurveRegion2 {
    result.expect("compound curve Boolean").into_value()
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
        curve::rectangle(Real::from(12), Real::from(8))
    });
    run_profile(&config, "square", 64, || curve::square(Real::from(8)));
    run_profile(&config, "circle", 16, || curve::circle(Real::from(4), 24));
    run_profile(&config, "right_triangle", 64, || {
        curve::right_triangle(Real::from(6), Real::from(4))
    });
    run_profile(&config, "polygon", 64, || curve::polygon(&polygon));
    run_profile(&config, "polygon_points", 64, || {
        curve::polygon_points(&polygon_points)
    });
    run_profile(&config, "ellipse", 16, || {
        curve::ellipse(Real::from(8), Real::from(4), 24)
    });
    run_profile(&config, "regular_ngon", 16, || {
        curve::regular_ngon(7, Real::from(4))
    });
    run_profile(&config, "arrow", 64, || {
        curve::arrow(Real::from(6), Real::from(2), Real::from(3), Real::from(4))
    });
    run_profile(&config, "trapezoid", 64, || {
        curve::trapezoid(Real::from(4), Real::from(8), Real::from(4), Real::from(2))
    });
    run_profile(&config, "rounded_rectangle", 8, || {
        curve::rounded_rectangle(Real::from(12_u8), Real::from(8_u8), Real::from(2_u8), 8)
    });
    run_profile(&config, "star", 16, || {
        curve::star(12, Real::from(8_u8), Real::from(4_u8))
    });
    run_profile(&config, "teardrop", 8, || {
        curve::teardrop(Real::from(6), Real::from(10), 24)
    });
    run_profile(&config, "egg", 8, || {
        curve::egg(Real::from(6), Real::from(10), 24)
    });
    run_profile(&config, "squircle", 4, || {
        curve::squircle(Real::from(8), Real::from(6), 24)
    });
    run_profile(&config, "keyhole", 4, || {
        compound_region(curve::keyhole(
            Real::from(4),
            Real::from(2),
            Real::from(6),
            24,
            &GeometryContext::APPROXIMATE_512,
        ))
    });
    run_profile(&config, "reuleaux", 2, || {
        compound_region(curve::reuleaux(
            3,
            Real::from(6),
            24,
            &GeometryContext::APPROXIMATE_512,
        ))
    });
    run_profile(&config, "ring", 4, || {
        compound_region(curve::ring(
            Real::from(6),
            Real::from(2),
            24,
            &GeometryContext::APPROXIMATE_512,
        ))
    });
    run_profile(&config, "pie_slice", 8, || {
        curve::pie_slice(Real::from(4), Real::from(10), Real::from(100), 12)
    });
    run_profile(&config, "supershape", 4, || {
        curve::supershape(
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
        compound_region(curve::circle_with_keyway(
            Real::from(6),
            24,
            Real::from(2),
            Real::from(2),
            &GeometryContext::APPROXIMATE_512,
        ))
    });
    run_profile(&config, "circle_with_flat", 2, || {
        compound_region(curve::circle_with_flat(
            Real::from(6),
            24,
            Real::from(2),
            &GeometryContext::APPROXIMATE_512,
        ))
    });
    run_profile(&config, "circle_with_two_flats", 2, || {
        compound_region(curve::circle_with_two_flats(
            Real::from(6),
            24,
            Real::from(2),
            &GeometryContext::APPROXIMATE_512,
        ))
    });
    run_path(&config, "bezier", 8, || {
        curve::bezier_path(&bezier_control, 16).expect("open Bezier path")
    });
    run_path(&config, "bspline", 8, || {
        curve::bspline_path(&bezier_control, 3, 8).expect("open B-spline path")
    });
    run_profile(&config, "heart", 4, || {
        curve::heart(Real::from(8), Real::from(8), 32)
    });
    run_profile(&config, "crescent", 2, || {
        compound_region(curve::crescent(
            Real::from(6),
            Real::from(4),
            Real::from(3),
            24,
            &GeometryContext::APPROXIMATE_512,
        ))
    });
    run_profile(&config, "involute_gear", 1, || {
        curve::involute_gear(
            Real::from(2_u8),
            20,
            Real::from(20_u8),
            Real::zero(),
            Real::zero(),
            4,
        )
    });
    run_profile(&config, "cycloidal_gear", 1, || {
        curve::cycloidal_gear(Real::from(2), 12, Real::from(2), Real::zero(), 4)
    });
    run_profile(&config, "involute_rack", 2, || {
        curve::involute_rack(Real::from(2), 4, Real::from(20), Real::zero(), Real::zero())
    });
    run_profile(&config, "cycloidal_rack", 2, || {
        curve::cycloidal_rack(Real::from(2), 4, Real::zero(), 8)
    });
    run_profile(&config, "airfoil_naca4", 4, || {
        curve::airfoil_naca4(
            Real::from(2_u8),
            Real::from(4_u8),
            Real::from(12_u8),
            Real::from(20_u8),
            80,
        )
    });
    config.run(
        "profile_primitives",
        "constructor",
        "hilbert_curve",
        8,
        || {
            let strings =
                curve::hilbert_strings(&curve::square(Real::from(8)), 3, Real::from(1));
            Measurement::new(1, strings.len() as u64, strings.len() as u64)
        },
    );

    let cubic_wire = curve::bezier_path(&bezier_control, 16).expect("open cubic Bezier path");
    let projection = FiniteProjectionOptions::try_new(1.0e-3)
        .expect("positive finite projection tolerance");
    config.run(
        "profile_curves",
        "finite_projection",
        "cubic_bezier",
        8,
        || {
            let polyline = black_box(&cubic_wire)
                .project_to_finite_polyline(&projection)
                .expect("finite Bezier projection");
            let points = polyline.points().len();
            Measurement::new(1, points as u64, points as u64)
        },
    );
    let closed_control = [
        [Real::from(0), Real::from(0)],
        [Real::from(4), Real::from(0)],
        [Real::from(4), Real::from(4)],
        [Real::from(0), Real::from(0)],
    ];
    let curved_region = curve::bezier_region(&closed_control, 16);
    config.run("profile_curves", "extrusion", "cubic_region", 4, || {
        let mesh = curve::try_extrude(
            black_box(&curved_region),
            Real::from(2),
            &csgrs::GeometryContext::STRICT,
        )
        .expect("extrusion")
        .into_value();
        Measurement::new(1, mesh.triangles.len() as u64, mesh.triangles.len() as u64)
    });

    let disjoint = curve::transformed(
        &curve::square(Real::from(2)),
        &Matrix4::affine_translation([Real::from(10), Real::zero(), Real::zero()]),
    );
    config.run(
        "profile_curves",
        "boolean",
        "curved_line_arc_union",
        4,
        || {
            let result = black_box(
                curved_region
                    .try_union(&disjoint, &CurvePolicy::STRICT)
                    .unwrap()
                    .into_value(),
            );
            Measurement::new(1, result.len() as u64, result.len() as u64)
        },
    );
}
