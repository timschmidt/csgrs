use csgrs::curve::*;
use csgrs::{GeometryCertainty, GeometryContext};
use hypercurve::{Axis2, CurvePoint2};
use hyperlattice::Point2;
use hyperreal::{Rational, Real};
use std::cmp::Ordering;

fn main() {
    let teeth = 4;
    let rack = cycloidal_rack(Real::one(), teeth, Real::zero(), 8);
    let paths = rack.boundary_paths().unwrap();
    assert_eq!(paths.len(), 1);
    let curves = paths[0].curves();
    let pitch = CurvePoint2::from(Point2::new(Real::zero(), Real::zero()));
    let mut pitch_vertices = 0;
    let mut crossings = 0;
    for curve in curves {
        let start = curve.start().compare_coordinate(&pitch, Axis2::Y).unwrap();
        let end = curve.end().compare_coordinate(&pitch, Axis2::Y).unwrap();
        pitch_vertices += usize::from(start == Ordering::Equal);
        crossings += usize::from((start == Ordering::Less) != (end == Ordering::Less));
    }
    println!(
        "rack curves={}, pitch_vertices={pitch_vertices}, pitch_crossings={crossings}",
        curves.len()
    );
    assert_eq!(crossings, teeth * 2);
    let epsilon = Real::from(Rational::fraction(1, 1_000_000_000_000).unwrap());
    for tooth in 0..teeth {
        for side in [-1, 1] {
            let x = Real::pi()
                * (Real::from(tooth as u64) + (Real::from(side) / Real::from(4)).unwrap());
            for (y, expected) in [
                (Real::zero(), None),
                (-&epsilon, Some(true)),
                (epsilon.clone(), Some(false)),
            ] {
                let answer =
                    contains_xy_with_context(&rack, x.clone(), y, &GeometryContext::STRICT)
                        .unwrap();
                assert_eq!(answer.certainty, GeometryCertainty::Certified);
                assert_eq!(answer.value, expected);
            }
        }
    }
    println!("all eight exact pitch contacts and +/-1e-12 inside/outside witnesses certified");
    for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
        let crescent =
            crescent(Real::from(2), Real::one(), Real::one(), 16, &context).unwrap();
        println!("crescent certainty={:?}", crescent.certainty);
        assert_eq!(crescent.certainty, GeometryCertainty::Certified);
        assert!(!crescent.value.is_empty());
        let profiles = try_finite_profiles(&crescent.value, &GeometryContext::STRICT).unwrap();
        assert_eq!(profiles.certainty, GeometryCertainty::Certified);
        assert!(!profiles.value.is_empty());
        // The 16-gon centered at zero with circumradius 2 contains (-1,0).
        // Its subtracted unit 16-gon centered at (1,0) contains (1,0).
        for (x, expected) in [(-1, true), (1, false), (3, false)] {
            let location = contains_xy_with_context(
                &crescent.value,
                Real::from(x),
                Real::zero(),
                &GeometryContext::STRICT,
            )
            .unwrap();
            assert_eq!(location.certainty, GeometryCertainty::Certified);
            assert_eq!(location.value, Some(expected));
        }
    }
    println!("crescent membership and projection certified under both construction policies");
}
