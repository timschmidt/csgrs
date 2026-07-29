//! Fuzz target for curve shape constructors.

#![no_main]

mod support;

use csgrs::curve;
use hypercurve::{CurvePolicy, CurveRegion2, FiniteProjectionOptions, Point2};
use hyperlattice::Real;
use libfuzzer_sys::fuzz_target;

fn real(value: f64) -> Real {
    Real::try_from(value).expect("fuzz decoder clamps to finite values")
}

fn tolerance() -> Real {
    real(1.0e-9)
}

fn at_least_tolerance(value: Real) -> Real {
    let tolerance = tolerance();
    value.max(&tolerance).clone()
}

fn decode_real(bytes: &[u8], idx: &mut usize) -> Real {
    let mut raw = [0u8; 8];
    for slot in &mut raw {
        *slot = bytes[*idx % bytes.len()];
        *idx += 1;
    }
    let value = i64::from_le_bytes(raw) as f64 / 1.0e12;
    real(value.clamp(-1.0e3, 1.0e3))
}

fn assert_curve_finite(region: &CurveRegion2) {
    let profiles = curve::finite_profiles(region);
    for ring in profiles.iter().flat_map(|profile| {
        std::iter::once(profile.material().points())
            .chain(profile.holes().iter().map(|hole| hole.points()))
    }) {
        for point in ring {
            assert!(point[0].is_finite());
            assert!(point[1].is_finite());
        }
    }
}

fuzz_target!(|bytes: &[u8]| {
    if bytes.len() < 4 {
        return;
    }

    let mut idx = 0usize;
    let tag = bytes[idx] % 31;
    idx += 1;
    let a = decode_real(bytes, &mut idx);
    let b = decode_real(bytes, &mut idx);
    let c = decode_real(bytes, &mut idx);
    let segments = (bytes[idx % bytes.len()] as usize % 32) + 1;
    idx += 1;
    let teeth = (bytes[idx % bytes.len()] as usize % 24) + 1;
    let reuleaux_sides = ((bytes[(idx + 1) % bytes.len()] as usize % 7) * 2) + 3;
    let reuleaux_segments = if bytes[idx % bytes.len()] & 1 == 0 {
        reuleaux_sides * 4
    } else {
        segments.max(6)
    };
    let positive_a = at_least_tolerance(a.abs());
    let positive_b = at_least_tolerance(b.abs());
    let exponent_a = Real::from((bytes[idx % bytes.len()] % 9) as i8 - 4);
    let exponent_b = Real::from((bytes[(idx + 1) % bytes.len()] % 9) as i8 - 4);
    let curve_control = [
        [Real::zero(), Real::zero()],
        [a.clone(), b.clone()],
        [c.clone(), a.clone()],
        [Real::one(), Real::zero()],
    ];
    let native_points = [
        Point2::new(Real::zero(), Real::zero()),
        Point2::new(a.clone(), b.clone()),
        Point2::new(c.clone(), positive_a.clone()),
    ];

    let region = match tag {
        0 => curve::rectangle(a, b),
        1 => curve::square(a),
        2 => curve::circle(a, segments),
        3 => curve::right_triangle(a, b),
        4 => curve::ellipse(a, b, segments),
        5 => curve::regular_ngon(segments, a),
        6 => curve::arrow(a, b, c, positive_b),
        7 => curve::trapezoid(a, b, c, real(0.25)),
        8 => curve::star(segments, a, b),
        9 => curve::rounded_rectangle(a, b, c, segments),
        10 => curve::squircle(a, b, segments),
        11 => curve::keyhole(a, b, c, segments),
        12 => curve::reuleaux(reuleaux_sides, a, reuleaux_segments),
        13 => curve::ring(a, b, segments),
        14 => curve::pie_slice(a, b, c, segments),
        15 => curve::heart(a, b, segments),
        16 => curve::crescent(positive_a, positive_b, c, segments),
        17 => curve::airfoil_naca4(a, b, real(12.0), positive_a, segments.max(2)),
        18 => {
            curve::involute_gear(a, teeth, b.clone(), c, real(0.01) * b, segments.clamp(2, 6))
        },
        19 => curve::cycloidal_gear(a, teeth, positive_b, b, segments.clamp(2, 6)),
        20 => curve::involute_rack(a, teeth, b, c.clone(), real(0.01) * c),
        21 => curve::cycloidal_rack(a, teeth, c, segments.clamp(4, 8)),
        22 => {
            if bytes[idx % bytes.len()] & 1 == 0 {
                curve::teardrop(a, b, segments)
            } else {
                curve::teardrop(positive_a.clone(), positive_a + positive_b, segments.max(2))
            }
        },
        23 => {
            if bytes[idx % bytes.len()] & 1 == 0 {
                curve::egg(a, b, segments)
            } else {
                curve::egg(positive_a, positive_b, segments.max(3))
            }
        },
        24 => curve::circle_with_keyway(a, segments, b, c),
        25 => curve::circle_with_flat(a, segments, b),
        26 => curve::circle_with_two_flats(a, segments, b),
        27 => curve::bezier_region(&curve_control, segments),
        28 => {
            if let Some(path) = curve::bspline_path(
                &curve_control,
                bytes[idx % bytes.len()] as usize % 5,
                segments,
            ) {
                let options =
                    FiniteProjectionOptions::try_new(1.0e-3).expect("positive tolerance");
                if let Ok(polyline) = path.project_to_finite_polyline(&options) {
                    for point in polyline.points() {
                        assert!(point[0].is_finite());
                        assert!(point[1].is_finite());
                    }
                }
            }
            return;
        },
        29 => curve::supershape(
            positive_a,
            positive_b,
            c,
            real(2.0),
            exponent_a,
            exponent_b,
            segments,
        ),
        _ => curve::polygon_points(&native_points),
    };

    let _policy = CurvePolicy::certified();
    assert_curve_finite(&region);
    let flat = curve::triangulate(&region);
    support::validate_triangle_mesh(&flat, false);
    if !region.is_empty() {
        let solid = curve::extrude(&region, Real::one());
        support::validate_triangle_mesh(&solid, true);
    }
});
