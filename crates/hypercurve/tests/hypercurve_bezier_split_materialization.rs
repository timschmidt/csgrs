use hypercurve::{
    BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, BezierSplitFragment2, BezierSubcurve2, Classification, CubicBezier2,
    CurvePolicy, Point2, QuadraticBezier2, RationalQuadraticBezier2, Real, UncertaintyReason,
};
use proptest::prelude::*;

fn r(value: i32) -> Real {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(r(x), r(y))
}

fn policy() -> CurvePolicy {
    CurvePolicy::certified()
}

fn exact(value: Real) -> BezierParameter2 {
    match BezierParameter2::exact(value, &policy()).unwrap() {
        Classification::Decided(parameter) => parameter,
        Classification::Uncertain(reason) => {
            panic!("exact parameter unexpectedly uncertain: {reason:?}")
        }
    }
}

fn algebraic_midpoint_interval(start: Real, end: Real) -> BezierParameter2 {
    let polynomial =
        match BezierParameterPolynomial::try_new_power_basis(vec![r(-1), r(2)], &policy()).unwrap()
        {
            Classification::Decided(polynomial) => polynomial,
            Classification::Uncertain(reason) => {
                panic!("polynomial unexpectedly uncertain: {reason:?}")
            }
        };
    let interval = match BezierParameterInterval::try_new(start, end, &policy()).unwrap() {
        Classification::Decided(interval) => interval,
        Classification::Uncertain(reason) => panic!("interval unexpectedly uncertain: {reason:?}"),
    };
    match BezierAlgebraicParameter2::try_isolate(polynomial, interval, &policy()).unwrap() {
        Classification::Decided(parameter) => BezierParameter2::algebraic(parameter),
        Classification::Uncertain(reason) => {
            panic!("algebraic parameter unexpectedly uncertain: {reason:?}")
        }
    }
}

#[test]
fn exact_quadratic_split_materializes_native_subcurves() {
    let curve = QuadraticBezier2::new(p(0, 0), p(2, 4), p(4, 0));
    let materialization = match curve
        .split_at_parameters(&[exact(q(1, 2))], &policy())
        .unwrap()
    {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("split unexpectedly uncertain: {reason:?}"),
    };

    assert!(materialization.is_fully_materialized());
    assert_eq!(materialization.fragments().len(), 2);
    let BezierSplitFragment2::Materialized {
        curve: BezierSubcurve2::Quadratic(left),
        ..
    } = &materialization.fragments()[0]
    else {
        panic!("first fragment should be a quadratic");
    };
    let BezierSplitFragment2::Materialized {
        curve: BezierSubcurve2::Quadratic(right),
        ..
    } = &materialization.fragments()[1]
    else {
        panic!("second fragment should be a quadratic");
    };

    let midpoint = curve.point_at(q(1, 2));
    assert_eq!(left.end(), &midpoint);
    assert_eq!(right.start(), &midpoint);
    assert_eq!(left.start(), curve.start());
    assert_eq!(right.end(), curve.end());
}

#[test]
fn exact_cubic_subcurve_matches_original_endpoints_at_range_bounds() {
    let curve = CubicBezier2::new(p(0, 0), p(2, 6), p(6, -2), p(8, 0));
    let subcurve = curve
        .subcurve_between_exact(&q(1, 4), &q(3, 4), &policy())
        .unwrap();

    assert_eq!(subcurve.start(), &curve.point_at(q(1, 4)));
    assert_eq!(subcurve.end(), &curve.point_at(q(3, 4)));
}

#[test]
fn exact_rational_quadratic_split_preserves_conic_endpoint_evaluation() {
    let curve =
        RationalQuadraticBezier2::try_unit_end_weights(p(1, 0), p(1, 1), p(0, 1), q(1, 2)).unwrap();
    let subcurve = curve
        .subcurve_between_exact(&r(0), &q(1, 2), &policy())
        .unwrap();
    let expected_midpoint = match curve.point_at(q(1, 2), &policy()) {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => {
            panic!("conic midpoint unexpectedly uncertain: {reason:?}")
        }
    };

    assert_eq!(subcurve.start(), curve.start());
    assert_eq!(subcurve.end(), &expected_midpoint);
}

#[test]
fn algebraic_boundary_is_carried_without_approximate_materialization() {
    let curve = QuadraticBezier2::new(p(0, 0), p(2, 4), p(4, 0));
    let materialization = match curve
        .split_at_parameters(
            &[
                exact(q(1, 4)),
                algebraic_midpoint_interval(q(2, 5), q(3, 5)),
                exact(q(3, 4)),
            ],
            &policy(),
        )
        .unwrap()
    {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("split unexpectedly uncertain: {reason:?}"),
    };

    assert!(materialization.has_unresolved_fragments());
    assert_eq!(materialization.fragments().len(), 4);
    assert!(matches!(
        materialization.fragments()[0],
        BezierSplitFragment2::Materialized { .. }
    ));
    assert!(matches!(
        materialization.fragments()[1],
        BezierSplitFragment2::Unresolved { .. }
    ));
    assert!(matches!(
        materialization.fragments()[2],
        BezierSplitFragment2::Unresolved { .. }
    ));
}

#[test]
fn broad_algebraic_interval_refuses_to_order_against_endpoints() {
    let curve = QuadraticBezier2::new(p(0, 0), p(2, 4), p(4, 0));
    let split = curve
        .split_at_parameters(&[algebraic_midpoint_interval(r(0), r(1))], &policy())
        .unwrap();

    assert_eq!(
        split,
        Classification::Uncertain(UncertaintyReason::Ordering)
    );
}

proptest! {
    #[test]
    fn exact_quadratic_split_endpoints_match_original(
        start_n in 0_i32..=15,
        width_n in 1_i32..=16,
    ) {
        let end_n = (start_n + width_n).min(16);
        prop_assume!(start_n < end_n);
        let start = q(start_n, 16);
        let end = q(end_n, 16);
        let curve = QuadraticBezier2::new(p(-3, 1), p(5, 9), p(11, -7));
        let subcurve = curve
            .subcurve_between_exact(&start, &end, &policy())
            .map_err(|error| TestCaseError::fail(format!("split failed: {error:?}")))?;

        prop_assert_eq!(subcurve.start(), &curve.point_at(start));
        prop_assert_eq!(subcurve.end(), &curve.point_at(end));
    }
}
