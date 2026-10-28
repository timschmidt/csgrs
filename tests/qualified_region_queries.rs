#![cfg(feature = "curve")]

use hypercurve::{
    Curve2, CurveCertainty, CurvePath2, CurveRegion2, FillRule, Point2, PredicatePolicy,
    QuadraticBezier2, Real,
};

fn point(x: i32, y: i32) -> Point2 {
    Point2::new(x.into(), y.into())
}

fn positive_height_qualified_empty_region() -> (Real, CurveRegion2) {
    let height = (Real::e() + Real::one()).sin()
        - Real::e().sin() * Real::one().cos()
        - Real::e().cos() * Real::one().sin()
        + Real::from(2).powi_i64(-1024).unwrap();
    let path = CurvePath2::try_new(vec![
        QuadraticBezier2::new(
            point(0, 0),
            Point2::new(Real::one(), height.clone()),
            point(2, 0),
        )
        .into(),
        Curve2::try_line(point(2, 0).into(), point(0, 0).into()).unwrap(),
    ])
    .unwrap();
    let constructed = hypercurve::evaluate_under(PredicatePolicy::APPROXIMATE_512, || {
        CurveRegion2::try_from_boundary_paths_with_loop_semantics(
            &[path],
            &[hypercurve::CurveRegionLoopRole::Material],
            &[FillRule::EvenOdd],
        )
        .unwrap()
    });
    assert_eq!(
        constructed.certainty(),
        CurveCertainty::Approximate512Consumed
    );
    let region = constructed.into_unverified();
    assert!(region.is_empty());
    // The source cap has positive area 2*height/3. Prove the independent
    // sign after construction so this check does not warm its decision path.
    assert_eq!(
        height.certified_sign_until(-2048).sign(),
        Some(hypercurve::RealSign::Positive)
    );
    (height, region)
}

fn exact_square_region() -> CurveRegion2 {
    let points = [point(4, 4), point(6, 4), point(6, 6), point(4, 6)];
    let boundary = CurvePath2::try_new(
        (0..4)
            .map(|i| {
                Curve2::try_line(points[i].clone().into(), points[(i + 1) % 4].clone().into())
                    .unwrap()
            })
            .collect(),
    )
    .unwrap();
    CurveRegion2::try_from_boundary_paths(&[boundary], FillRule::NonZero).unwrap()
}

#[test]
fn csgrs_empty_membership_preserves_inherited_topology_receipt() {
    let (height, region) = positive_height_qualified_empty_region();
    let y = (height / Real::from(4)).unwrap();
    for context in [
        csgrs::GeometryContext::STRICT,
        csgrs::GeometryContext::APPROXIMATE_512,
        csgrs::GeometryContext::STRICT,
    ] {
        let result =
            csgrs::curve::contains_xy_with_context(&region, Real::one(), y.clone(), &context);
        if context == csgrs::GeometryContext::STRICT {
            let error = result.expect_err("the original cap contains this point; qualified emptiness cannot certify Outside");
            assert_eq!(error.kind(), csgrs::errors::ValidationErrorKind::Undecided);
            assert!(
                matches!(error, csgrs::errors::ValidationError::ExactCurve(hypercurve::ExactCurveError::Blocked(reason)) if reason.reason()==hypercurve::UncertaintyReason::Predicate)
            );
        } else {
            let outcome = result.unwrap();
            assert_eq!(outcome.value, Some(false));
            assert_eq!(
                outcome.certainty,
                csgrs::GeometryCertainty::Approximate512Consumed
            );
        }
    }
    let error = csgrs::curve::contains_xy(&region, Real::one(), y).unwrap_err();
    assert_eq!(error.kind(), csgrs::errors::ValidationErrorKind::Undecided);
}

#[test]
fn csgrs_empty_bounds_preserve_inherited_topology_receipt() {
    let (_, region) = positive_height_qualified_empty_region();
    for context in [
        csgrs::GeometryContext::STRICT,
        csgrs::GeometryContext::APPROXIMATE_512,
        csgrs::GeometryContext::STRICT,
    ] {
        let result = csgrs::curve::try_bounding_box_with_context(&region, &context);
        if context == csgrs::GeometryContext::STRICT {
            assert_eq!(
                result.unwrap_err().kind(),
                csgrs::errors::ValidationErrorKind::Undecided
            );
        } else {
            let outcome = result.unwrap();
            assert!(outcome.value.is_none());
            assert_eq!(
                outcome.certainty,
                csgrs::GeometryCertainty::Approximate512Consumed
            );
        }
    }
}

#[test]
fn csgrs_region_query_controls_preserve_exact_and_inherited_certainty() {
    let square = exact_square_region();
    for region in [CurveRegion2::empty(), square.clone()] {
        let expected = Some(!region.is_empty());
        for context in [
            csgrs::GeometryContext::STRICT,
            csgrs::GeometryContext::APPROXIMATE_512,
            csgrs::GeometryContext::STRICT,
        ] {
            let query =
                csgrs::curve::contains_xy_with_context(&region, 5.into(), 5.into(), &context)
                    .unwrap();
            assert_eq!(query.value, expected);
            assert_eq!(query.certainty, csgrs::GeometryCertainty::Certified);
            assert_eq!(
                csgrs::curve::contains_xy(&region, 5.into(), 5.into()).unwrap(),
                expected
            );
            let bounds =
                csgrs::curve::try_bounding_box_with_context(&region, &context).unwrap();
            assert_eq!(bounds.value.is_none(), region.is_empty());
            assert_eq!(bounds.certainty, csgrs::GeometryCertainty::Certified);
        }
    }
    let (_, collapsed) = positive_height_qualified_empty_region();
    let constructed = hypercurve::evaluate_under(PredicatePolicy::APPROXIMATE_512, || {
        collapsed
            .boolean_region(&square, hypercurve::BooleanOp::Union)
            .unwrap()
    });
    assert_eq!(
        constructed.certainty(),
        CurveCertainty::Approximate512Consumed
    );
    let region = constructed.into_unverified();
    for context in [
        csgrs::GeometryContext::STRICT,
        csgrs::GeometryContext::APPROXIMATE_512,
        csgrs::GeometryContext::STRICT,
    ] {
        let query =
            csgrs::curve::contains_xy_with_context(&region, 5.into(), 5.into(), &context);
        let bounds = csgrs::curve::try_bounding_box_with_context(&region, &context);
        if context == csgrs::GeometryContext::STRICT {
            assert_eq!(
                query.unwrap_err().kind(),
                csgrs::errors::ValidationErrorKind::Undecided
            );
            assert_eq!(
                bounds.unwrap_err().kind(),
                csgrs::errors::ValidationErrorKind::Undecided
            );
        } else {
            let query = query.unwrap();
            assert_eq!(query.value, Some(true));
            assert_eq!(
                query.certainty,
                csgrs::GeometryCertainty::Approximate512Consumed
            );
            let bounds = bounds.unwrap();
            assert_eq!(
                bounds.certainty,
                csgrs::GeometryCertainty::Approximate512Consumed
            );
            let bounds = bounds.value.unwrap();
            assert_eq!(
                [bounds.mins.x, bounds.mins.y, bounds.maxs.x, bounds.maxs.y],
                [Real::from(4), Real::from(4), Real::from(6), Real::from(6)]
            );
        }
        assert_eq!(
            csgrs::curve::contains_xy(&square, 5.into(), 5.into()).unwrap(),
            Some(true)
        );
    }
}
