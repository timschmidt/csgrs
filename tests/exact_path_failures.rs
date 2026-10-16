#![cfg(feature = "curve")]

use csgrs::{
    GeometryCertainty, GeometryContext, Real, curve,
    errors::{ValidationError, ValidationErrorKind},
};
use hypercurve::{BooleanOp, CurveError, CurveFamily2, CurvePath2, ExactCurveError};
use std::error::Error;

fn r(value: i32) -> Real {
    Real::from(value)
}

fn q(numerator: i32, denominator: i32) -> Real {
    (r(numerator) / r(denominator)).unwrap()
}

fn cubic_controls() -> [[Real; 2]; 4] {
    [[r(0), r(0)], [r(3), r(0)], [r(0), r(3)], [r(0), r(0)]]
}

fn invalid<T: std::fmt::Debug>(result: Result<T, ValidationError>) -> ValidationError {
    let error = result.expect_err("invalid construction must not publish geometry");
    assert_eq!(error.kind(), ValidationErrorKind::InvalidInput);
    error
}

#[test]
fn malformed_controls_and_display_counts_are_typed_failures() {
    let closed = cubic_controls();
    let open = [[r(0), r(0)], [r(1), r(0)], [r(1), r(1)], [r(0), r(1)]];
    for (controls, display) in [(&[][..], 16), (&closed[..1], 16), (&closed[..], 0)] {
        invalid(curve::bezier_path(controls, display));
        invalid(curve::bezier_region(controls, display));
        for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
            invalid(curve::bezier_path_with_context(controls, display, &context));
            invalid(curve::bezier_region_with_context(controls, display, &context));
        }
    }
    invalid(curve::bezier_region(&open, 16));
    for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
        invalid(curve::bezier_region_with_context(&open, 16, &context));
        for (controls, degree, display) in [
            (&[][..], 2, 8),
            (&open[..], usize::MAX, 8),
            (&open[..], 4, 8),
            (&open[..], 2, 0),
        ] {
            invalid(curve::bspline_path(controls, degree, display));
            invalid(curve::bspline_path_with_context(
                controls, degree, display, &context,
            ));
        }
    }
}

#[test]
fn native_line_and_spline_errors_retain_their_causes() {
    let repeated = [[r(0), r(0)], [r(0), r(0)]];
    let controls = cubic_controls();
    for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
        let line = invalid(curve::bezier_path_with_context(&repeated, 1, &context));
        assert!(matches!(
            line,
            ValidationError::Curve(CurveError::ZeroLengthLine)
        ));
        assert!(line.source().unwrap().downcast_ref::<CurveError>().is_some());
        let spline = invalid(curve::bspline_path_with_context(&controls, 0, 8, &context));
        assert!(matches!(spline, ValidationError::ExactCurve(_)));
        assert!(
            spline
                .source()
                .unwrap()
                .downcast_ref::<ExactCurveError>()
                .is_some()
        );
    }
    assert!(matches!(
        invalid(curve::bezier_path(&repeated, 1)),
        ValidationError::Curve(CurveError::ZeroLengthLine)
    ));
    assert!(matches!(
        invalid(curve::bspline_path(&controls, 0, 8)),
        ValidationError::ExactCurve(_)
    ));
}

#[test]
fn unresolved_closure_is_undecided_across_policy_reuse() {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    let zero = &sine * &sine + &cosine * &cosine - Real::one();
    let controls = [[r(0), r(0)], [r(3), r(0)], [r(0), r(3)], [zero, r(0)]];
    for context in [
        GeometryContext::STRICT,
        GeometryContext::APPROXIMATE_512,
        GeometryContext::STRICT,
    ] {
        let error = curve::bezier_region_with_context(&controls, 16, &context)
            .expect_err("unresolved closure must not become empty geometry");
        assert_eq!(error.kind(), ValidationErrorKind::Undecided);
        assert!(matches!(error, ValidationError::ExactCurve(_)));
        assert!(
            error
                .source()
                .unwrap()
                .downcast_ref::<ExactCurveError>()
                .is_some()
        );
    }
    assert_eq!(
        curve::bezier_region(&controls, 16).unwrap_err().kind(),
        ValidationErrorKind::Undecided
    );
}

fn assert_midpoint(path: &CurvePath2, x: &Real, y: &Real) {
    assert_eq!(path.curves().len(), 1);
    let point = path.curves()[0].point_at(&q(1, 2).into()).unwrap();
    let coordinates = point
        .coordinates()
        .expect("rational evaluation stays represented");
    assert_eq!(coordinates.x(), x);
    assert_eq!(coordinates.y(), y);
}

#[test]
fn exact_control_nets_survive_display_counts_policies_and_reversal() {
    let controls = [
        [r(0), r(0)],
        [r(1), r(2)],
        [r(2), r(1)],
        [r(3), r(0)],
        [r(4), r(2)],
    ];
    // Independently evaluated Bernstein polynomials at t=1/2. A clamped
    // single-span polynomial B-spline has these same polynomial values.
    let expected = [
        (q(1, 2), r(1), CurveFamily2::Line),
        (r(1), q(5, 4), CurveFamily2::QuadraticBezier),
        (q(3, 2), q(9, 8), CurveFamily2::CubicBezier),
        (r(2), r(1), CurveFamily2::RationalBezier),
    ];
    for (count, (x, y, family)) in (2..=5).zip(expected) {
        for display in [1, 16, usize::MAX] {
            let control = &controls[..count];
            assert_midpoint(&curve::bezier_path(control, display).unwrap(), &x, &y);
            assert_midpoint(
                &curve::bspline_path(control, count - 1, display).unwrap(),
                &x,
                &y,
            );
            for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
                let bezier =
                    curve::bezier_path_with_context(control, display, &context).unwrap();
                let spline =
                    curve::bspline_path_with_context(control, count - 1, display, &context)
                        .unwrap();
                assert_eq!(bezier.value.curves()[0].family(), family);
                assert_eq!(
                    spline.value.curves()[0].family(),
                    CurveFamily2::PolynomialBSpline
                );
                for outcome in [bezier, spline] {
                    assert_eq!(outcome.certainty, GeometryCertainty::Certified);
                    assert_midpoint(&outcome.value, &x, &y);
                    let reversed = outcome.value.reversed().unwrap();
                    assert_midpoint(&reversed, &x, &y);
                    assert!(reversed.start().coincides_with(&outcome.value.end()).unwrap());
                    assert!(reversed.end().coincides_with(&outcome.value.start()).unwrap());
                }
            }
        }
    }
}

#[test]
fn closed_cubic_preserves_exact_area_membership_and_composition() {
    for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
        let outcome =
            curve::bezier_region_with_context(&cubic_controls(), 16, &context).unwrap();
        assert_eq!(outcome.certainty, GeometryCertainty::Certified);
        // x=9t(1-t)^2, y=9t^2(1-t); 1/2 integral(x y'-y x') = 27/20.
        let region = outcome.value;
        assert_eq!(region.signed_area().unwrap(), Some(q(27, 20)));
        for (x, y, expected) in [
            (q(1, 2), q(1, 2), Some(true)),
            (q(9, 8), q(9, 8), None),
            (q(3, 2), q(3, 2), Some(false)),
        ] {
            let location = curve::contains_xy_with_context(&region, x, y, &context).unwrap();
            assert_eq!(location.certainty, GeometryCertainty::Certified);
            assert_eq!(location.value, expected);
        }
        let restored = region
            .boolean_region(&curve::empty(), BooleanOp::Union)
            .unwrap();
        assert_eq!(restored.signed_area().unwrap(), Some(q(27, 20)));
        let scaled = curve::scaled(&restored, r(2), r(3)).unwrap();
        assert_eq!(scaled.signed_area().unwrap(), Some(q(81, 10)));
        assert_eq!(
            curve::contains_xy(&scaled, r(1), q(3, 2)).unwrap(),
            Some(true)
        );
        assert_eq!(
            curve::contains_xy(&scaled, r(3), q(9, 2)).unwrap(),
            Some(false)
        );
    }
}

#[test]
fn certified_retraced_boundary_is_empty_and_remains_composable() {
    let controls = [[r(0), r(0)], [r(1), r(0)], [r(0), r(0)]];
    assert!(curve::bezier_region(&controls, 16).unwrap().is_empty());
    for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
        let empty = curve::bezier_region_with_context(&controls, 16, &context).unwrap();
        assert_eq!(empty.certainty, GeometryCertainty::Certified);
        assert!(empty.value.is_empty());
        assert!(curve::scaled(&empty.value, r(2), r(3)).unwrap().is_empty());
        let square = curve::square(r(2)).unwrap();
        let restored = empty.value.boolean_region(&square, BooleanOp::Union).unwrap();
        assert_eq!(restored.signed_area().unwrap(), Some(r(4)));
        assert_eq!(curve::contains_xy(&restored, r(1), r(1)).unwrap(), Some(true));
        assert_eq!(
            curve::contains_xy(&restored, r(3), r(1)).unwrap(),
            Some(false)
        );
    }
}
