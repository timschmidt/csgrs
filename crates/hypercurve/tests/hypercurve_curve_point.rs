use std::cmp::Ordering;

use hypercurve::{
    Axis2, BezierAlgebraicParameter2, BezierParameterInterval, BezierParameterPolynomial,
    Classification, CurveCertainty, CurveContext, CurveOutcome, CurvePoint2, Point2,
    RationalBezier2, Real,
};

mod support;

fn decided<T: std::fmt::Debug>(value: Classification<T>) -> T {
    match value {
        Classification::Decided(value) => value,
        other => panic!("expected a certified decision: {other:?}"),
    }
}

fn certified<T: std::fmt::Debug>(outcome: CurveOutcome<Classification<T>>) -> T {
    assert_eq!(outcome.certainty, CurveCertainty::Certified);
    decided(outcome.value)
}

fn selected_point(reversed: bool) -> CurvePoint2 {
    let policy = CurveContext::STRICT;
    // alpha^5 + alpha = 1 has one root in (0, 1). The second chart
    // independently selects u = 1 - alpha from (1 - u)^5 - u = 0.
    let coefficients: &[i32] = if reversed {
        &[1, -6, 10, -10, 5, -1]
    } else {
        &[-1, 1, 0, 0, 0, 1]
    };
    let polynomial = decided(
        BezierParameterPolynomial::try_new_power_basis(
            coefficients.iter().copied().map(Real::from).collect(),
            &policy,
        )
        .unwrap(),
    );
    let interval =
        decided(BezierParameterInterval::try_new(Real::zero(), Real::one(), &policy).unwrap());
    let parameter =
        decided(BezierAlgebraicParameter2::try_isolate(polynomial, interval, &policy).unwrap());
    let mut controls = vec![
        Point2::new(Real::zero(), Real::pi()),
        Point2::new(Real::one(), Real::pi()),
    ];
    if reversed {
        controls.reverse();
    }
    let curve = RationalBezier2::try_new(controls, vec![Real::one(); 2]).unwrap();
    CurvePoint2::from(
        curve
            .point_at_algebraic_parameter(&parameter, &policy)
            .unwrap(),
    )
}

#[test]
fn independent_selected_charts_share_the_general_point_queries() {
    let first = selected_point(false);
    let second = selected_point(true);
    assert!(first.coordinates().is_none());
    assert!(second.coordinates().is_none());
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        assert!(certified(first.coincides_with(&second, &policy)));
        assert_eq!(
            certified(
                first
                    .compare_coordinate(&second, Axis2::X, &policy)
                    .unwrap()
            ),
            Ordering::Equal,
        );
        let at_height = CurvePoint2::from(Point2::new(Real::zero(), Real::pi()));
        assert_eq!(
            certified(
                first
                    .compare_coordinate(&at_height, Axis2::Y, &policy)
                    .unwrap()
            ),
            Ordering::Equal,
        );
        assert_eq!(
            certified(
                first
                    .compare_coordinate(&at_height, Axis2::X, &policy)
                    .unwrap()
            ),
            Ordering::Greater,
        );
        let bounds = certified(first.bounds(&policy));
        assert_eq!(bounds.min().y(), &Real::pi());
        assert_eq!(bounds.max().y(), &Real::pi());
    }
    // Queries keep the selected parameter representation available for reuse.
    assert!(first.coordinates().is_none());
    assert!(certified(
        first.coincides_with(&first.clone(), &CurveContext::STRICT)
    ));
}

#[test]
fn coordinate_view_accepts_arbitrary_exact_reals_and_preserves_certainty() {
    let coordinates = Point2::new(Real::pi(), Real::e());
    let point = CurvePoint2::from(coordinates.clone());
    assert_eq!(point.coordinates(), Some(&coordinates));
    let bounds = certified(point.bounds(&CurveContext::APPROXIMATE_512));
    assert_eq!(bounds.min(), &coordinates);
    assert_eq!(bounds.max(), &coordinates);
    assert!(certified(
        point.coincides_with(&point.clone(), &CurveContext::APPROXIMATE_512)
    ));

    let origin = CurvePoint2::from(Point2::new(Real::zero(), Real::zero()));
    let unresolved = CurvePoint2::from(Point2::new(
        support::terminally_unresolved_zero(),
        Real::zero(),
    ));
    let strict = origin.coincides_with(&unresolved, &CurveContext::STRICT);
    assert_eq!(strict.certainty, CurveCertainty::Certified);
    assert!(matches!(strict.value, Classification::Uncertain(_)));
    let approximate = origin.coincides_with(&unresolved, &CurveContext::APPROXIMATE_512);
    assert_eq!(
        approximate.certainty,
        CurveCertainty::Approximate512Consumed
    );
    assert_eq!(approximate.value, Classification::Decided(true));
    assert!(matches!(
        origin
            .coincides_with(&unresolved, &CurveContext::STRICT)
            .value,
        Classification::Uncertain(_),
    ));
}
