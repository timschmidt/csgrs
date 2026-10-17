mod support;

use hypercurve::{
    BezierParameterPolynomial, CurvePath2, CurveRegion2, ExactCurveError, FillRule, LineSeg2,
    OffsetCap, OffsetCornerStyle2, Point2, PredicatePolicy, Real, RegionPointLocation,
    UncertaintyReason, WorkBudget, WorkUsage, evaluate_under, evaluate_with_budget,
};

fn equal(actual: &Real, expected: &Real) {
    assert_eq!(
        hyperlimit::compare_reals(actual, expected, PredicatePolicy::STRICT).value(),
        Some(std::cmp::Ordering::Equal)
    );
}

fn path(end: [i32; 2]) -> CurvePath2 {
    CurvePath2::try_new(vec![
        LineSeg2::try_new(
            Point2::from_values(0, 0),
            Point2::from_values(end[0], end[1]),
        )
        .unwrap()
        .into(),
    ])
    .unwrap()
}

fn stroke(path: &CurvePath2, cap: OffsetCap) -> CurveRegion2 {
    CurveRegion2::stroke_path(path, Real::one(), &OffsetCornerStyle2::Round, cap).unwrap()
}

#[test]
fn diagonal_capsules_preserve_analytic_area_bounds_and_reusable_boundaries() {
    for end in [[1, 1], [-1, 1], [1, -1], [-1, -1], [3, 4], [0, -2]] {
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let region = evaluate_under(policy, || stroke(&path(end), OffsetCap::Round))
                .certified()
                .unwrap();
            let length = Real::from(end[0] * end[0] + end[1] * end[1])
                .sqrt()
                .unwrap();
            let area = Real::from(2) * length + Real::pi();
            equal(&region.filled_area().unwrap().unwrap(), &area);
            let bounds = region.bounds().unwrap().unwrap();
            for (actual, expected) in [
                (bounds.min_x(), end[0].min(0) - 1),
                (bounds.min_y(), end[1].min(0) - 1),
                (bounds.max_x(), end[0].max(0) + 1),
                (bounds.max_y(), end[1].max(0) + 1),
            ] {
                equal(actual, &Real::from(expected));
            }
            let rebuilt = CurveRegion2::try_from_boundary_paths(
                &region.boundary_paths().unwrap(),
                FillRule::NonZero,
            )
            .unwrap();
            equal(&rebuilt.filled_area().unwrap().unwrap(), &area);
            let origin = Point2::from_values(0, 0).into();
            assert_eq!(
                rebuilt.classify_point(&origin).unwrap(),
                RegionPointLocation::Inside
            );
        }
    }
}

#[test]
fn butt_diagonal_area_keeps_the_exact_surd() {
    for end in [[1, 1], [-1, 1], [1, -1], [-1, -1]] {
        let region = stroke(&path(end), OffsetCap::Butt);
        equal(
            &region.filled_area().unwrap().unwrap(),
            &(Real::from(2) * Real::from(2).sqrt().unwrap()),
        );
        assert_eq!(
            region
                .classify_point(&Point2::from_values(0, 0).into())
                .unwrap(),
            RegionPointLocation::Boundary
        );
    }
}

#[test]
fn unresolved_single_line_closure_keeps_the_strict_blocker() {
    let line = LineSeg2::try_new(
        Point2::from_values(0, 0),
        Point2::new(support::terminally_unresolved_zero(), Real::zero()),
    )
    .unwrap();
    let path = CurvePath2::try_new(vec![line.into()]).unwrap();
    for cap in [OffsetCap::Butt, OffsetCap::Round] {
        let result = CurveRegion2::stroke_path(&path, Real::one(), &OffsetCornerStyle2::Round, cap);
        assert!(matches!(result, Err(ExactCurveError::Blocked(blocker))
            if blocker.reason() == UncertaintyReason::RealSign));
    }
}

#[test]
fn many_capsules_preserve_tangent_contacts_and_nested_union_area() {
    let mut paths = Vec::new();
    for index in 0..32 {
        let source = CurvePath2::try_new(vec![
            LineSeg2::try_new(
                Point2::from_values(6 * index, 0),
                Point2::from_values(6 * index + 4, 0),
            )
            .unwrap()
            .into(),
        ])
        .unwrap();
        paths.extend(stroke(&source, OffsetCap::Round).boundary_paths().unwrap());
    }
    let tangent = CurveRegion2::try_from_boundary_paths(&paths, FillRule::NonZero).unwrap();
    equal(
        &tangent.filled_area().unwrap().unwrap(),
        &(Real::from(32) * (Real::from(8) + Real::pi())),
    );
    for index in 0..31 {
        assert_eq!(
            tangent
                .classify_point(&Point2::from_values(6 * index + 5, 0).into())
                .unwrap(),
            RegionPointLocation::Boundary
        );
    }
    let enclosing = CurveRegion2::stroke_path(
        &path([190, 0]),
        Real::from(2),
        &OffsetCornerStyle2::Round,
        OffsetCap::Round,
    )
    .unwrap();
    paths.extend(enclosing.boundary_paths().unwrap());
    let nested = CurveRegion2::try_from_boundary_paths(&paths, FillRule::NonZero).unwrap();
    equal(
        &nested.filled_area().unwrap().unwrap(),
        &(Real::from(760) + Real::from(4) * Real::pi()),
    );
}

#[test]
fn capsule_aggregation_keeps_the_exact_path_without_finite_enclosures() {
    let center: Real = "1e400".parse().unwrap();
    let mut paths = Vec::new();
    for y in [0, 10] {
        let source = CurvePath2::try_new(vec![
            LineSeg2::try_new(
                Point2::new(center.clone(), Real::from(y)),
                Point2::new(&center + Real::from(4), Real::from(y)),
            )
            .unwrap()
            .into(),
        ])
        .unwrap();
        paths.extend(stroke(&source, OffsetCap::Round).boundary_paths().unwrap());
    }
    let combined = CurveRegion2::try_from_boundary_paths(&paths, FillRule::NonZero).unwrap();
    equal(
        &combined.filled_area().unwrap().unwrap(),
        &(Real::from(16) + Real::from(2) * Real::pi()),
    );
}

#[test]
fn independent_diagonal_capsules_preserve_exact_total_area() {
    let mut boundaries = Vec::new();
    let mut expected = Real::zero();
    for index in 0..40 {
        let [dx, dy] = [[1, 2], [2, 3], [3, 5]][index % 3];
        let x = Real::from((10 * index) as u32);
        let start = Point2::new(&x + Real::pi(), Real::from(17));
        let end = Point2::new(start.x() + Real::from(dx), start.y() + Real::from(dy));
        let path =
            CurvePath2::try_new(vec![LineSeg2::try_new(start, end).unwrap().into()]).unwrap();
        boundaries.extend(stroke(&path, OffsetCap::Round).boundary_paths().unwrap());
        expected += Real::from(2) * Real::from(dx * dx + dy * dy).sqrt().unwrap() + Real::pi();
    }
    let combined = CurveRegion2::try_from_boundary_paths(&boundaries, FillRule::NonZero).unwrap();
    let area = combined.filled_area().unwrap().unwrap();
    assert_eq!(
        hyperlimit::compare_reals(&area, &Real::zero(), PredicatePolicy::STRICT).value(),
        Some(std::cmp::Ordering::Greater)
    );
    equal(&area, &expected);
    let actual = area.certified_rational_interval(-128).unwrap();
    let expected = expected.certified_rational_interval(-128).unwrap();
    let tolerance = "1e-30".parse::<hypercurve::Real>().unwrap();
    let tolerance = tolerance.exact_rational_ref().unwrap();
    assert!(&actual[1] - &actual[0] < *tolerance);
    assert!(&expected[1] - &expected[0] < *tolerance);
    assert!(actual[0] <= expected[1] && expected[0] <= actual[1]);
}

#[test]
fn native_stroke_construction_retains_zero_budget_direct_certificates() {
    for cap in [OffsetCap::Butt, OffsetCap::Round] {
        let result = evaluate_with_budget(PredicatePolicy::STRICT, WorkBudget::ZERO, || {
            stroke(&path([3, 4]), cap)
        });
        assert_eq!(result.usage, WorkUsage::default());
        let region = result.result.unwrap().certified().unwrap();
        let area = if cap == OffsetCap::Round {
            Real::from(10) + Real::pi()
        } else {
            Real::from(10)
        };
        equal(&region.filled_area().unwrap().unwrap(), &area);
    }
}

#[test]
fn native_stroke_and_warm_measurements_cannot_publish_an_exhausted_budget_result() {
    let path = path([3, 4]);
    let region = stroke(&path, OffsetCap::Round);
    region.filled_area().unwrap();
    region.bounds().unwrap();
    let denied = evaluate_with_budget(PredicatePolicy::STRICT, WorkBudget::ZERO, || {
        let polynomial = BezierParameterPolynomial::try_new_power_basis(
            [-36, 1368, -11034, 31728, -38280, 16620]
                .into_iter()
                .map(Real::from)
                .collect(),
        )
        .unwrap();
        assert!(polynomial.isolate_unit_interval_roots().is_err());
        let construction = CurveRegion2::stroke_path(
            &path,
            Real::one(),
            &OffsetCornerStyle2::Round,
            OffsetCap::Round,
        )
        .unwrap_err();
        assert!(matches!(construction, ExactCurveError::Blocked(blocker)
            if blocker.reason() == UncertaintyReason::BudgetExhausted));
        // Native budgets guard the outer result. Already certified warm
        // measurements need no new work, but cannot escape that failed result.
        (region.filled_area(), region.bounds())
    });
    assert!(denied.result.is_err());
    equal(
        &region.filled_area().unwrap().unwrap(),
        &(Real::from(10) + Real::pi()),
    );
}
