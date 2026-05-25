use hypercurve::{
    BezierAlgebraicParameter2, BezierAlgebraicSameTangentOrderStatus,
    BezierAlgebraicTangentOrderStatus, BezierAlgebraicTangentVector2,
    BezierAlgebraicTangentVectorReport, BezierAlgebraicTangentVectorStatus,
    BezierEndpointTangentImage2, BezierParameterInterval, BezierParameterPolynomial,
    BezierTangentTurnOrdering2, Classification, CurvePolicy, Point2, QuadraticBezier2, Real,
    compare_algebraic_same_tangent_second_order, compare_algebraic_same_tangent_third_order,
    compare_algebraic_tangent_turn_from_base,
};

fn r(value: i32) -> Real {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn p(x: Real, y: Real) -> Point2 {
    Point2::new(x, y)
}

fn pi(x: i32, y: i32) -> Point2 {
    p(r(x), r(y))
}

fn policy() -> CurvePolicy {
    CurvePolicy::certified()
}

fn decided<T>(classification: Classification<T>) -> T {
    match classification {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("unexpected uncertainty: {reason:?}"),
    }
}

fn polynomial(coefficients: Vec<Real>) -> BezierParameterPolynomial {
    decided(BezierParameterPolynomial::try_new_power_basis(coefficients, &policy()).unwrap())
}

fn interval(start: Real, end: Real) -> BezierParameterInterval {
    decided(BezierParameterInterval::try_new(start, end, &policy()).unwrap())
}

fn sqrt_half_parameter() -> BezierAlgebraicParameter2 {
    decided(
        BezierAlgebraicParameter2::try_isolate(
            polynomial(vec![r(-1), r(0), r(2)]),
            interval(q(1, 2), r(1)),
            &policy(),
        )
        .unwrap(),
    )
}

fn tangent_vector(curve: &QuadraticBezier2) -> BezierAlgebraicTangentVector2 {
    let parameter = sqrt_half_parameter();
    let tangent = curve
        .tangent_at_algebraic_parameter(&parameter, &policy())
        .unwrap();
    let BezierAlgebraicTangentVectorReport { status, vector, .. } =
        BezierAlgebraicTangentVector2::from_endpoint_image(
            &BezierEndpointTangentImage2::Polynomial(tangent),
        );
    assert_eq!(status, BezierAlgebraicTangentVectorStatus::Extracted);
    vector.unwrap()
}

fn horizontal() -> QuadraticBezier2 {
    QuadraticBezier2::new(p(r(0), r(0)), p(q(1, 2), r(0)), p(r(1), r(0)))
}

fn upward() -> QuadraticBezier2 {
    QuadraticBezier2::new(p(r(0), r(0)), p(r(0), q(1, 2)), p(r(0), r(1)))
}

fn downward() -> QuadraticBezier2 {
    QuadraticBezier2::new(p(r(0), r(0)), p(r(0), q(-1, 2)), p(r(0), r(-1)))
}

#[test]
fn algebraic_tangent_order_separates_opposite_half_turns() {
    let base = tangent_vector(&horizontal());
    let first = tangent_vector(&upward());
    let second = tangent_vector(&downward());

    let report = decided(compare_algebraic_tangent_turn_from_base(
        &base,
        &first,
        &second,
        &policy(),
    ));

    assert_eq!(report.status, BezierAlgebraicTangentOrderStatus::Ordered);
    assert_eq!(
        report.ordering,
        Some(BezierTangentTurnOrdering2::FirstBeforeSecond)
    );
    assert!(report.base_first_cross.unwrap().sign.unwrap().is_gt());
    assert!(report.base_second_cross.unwrap().sign.unwrap().is_lt());
}

#[test]
fn algebraic_tangent_order_uses_represented_cross_product_for_same_half() {
    let base = tangent_vector(&horizontal());
    let first = tangent_vector(&QuadraticBezier2::new(pi(0, 0), pi(0, 0), p(q(1, 2), r(1))));
    let second = tangent_vector(&QuadraticBezier2::new(
        pi(0, 0),
        p(q(1, 2), r(0)),
        p(r(1), q(1, 2)),
    ));

    let report = decided(compare_algebraic_tangent_turn_from_base(
        &base,
        &first,
        &second,
        &policy(),
    ));

    assert_eq!(report.status, BezierAlgebraicTangentOrderStatus::Ordered);
    assert_eq!(
        report.ordering,
        Some(BezierTangentTurnOrdering2::SecondBeforeFirst)
    );
    let cross = report.first_second_cross.unwrap();
    assert!(cross.scalar.unwrap().is_valid());
    assert!(cross.sign.unwrap().is_lt());
}

#[test]
fn algebraic_tangent_order_reports_same_direction_without_guessing() {
    let base = tangent_vector(&horizontal());
    let first = tangent_vector(&upward());
    let second = tangent_vector(&upward());

    let report = decided(compare_algebraic_tangent_turn_from_base(
        &base,
        &first,
        &second,
        &policy(),
    ));

    assert_eq!(
        report.status,
        BezierAlgebraicTangentOrderStatus::SameDirection
    );
    assert!(report.ordering.is_none());
}

#[test]
fn algebraic_tangent_order_rejects_zero_tangent() {
    let base = tangent_vector(&horizontal());
    let zero = tangent_vector(&QuadraticBezier2::new(pi(0, 0), pi(0, 0), pi(0, 0)));
    let second = tangent_vector(&upward());

    let report = decided(compare_algebraic_tangent_turn_from_base(
        &base,
        &zero,
        &second,
        &policy(),
    ));

    assert_eq!(
        report.status,
        BezierAlgebraicTangentOrderStatus::ZeroTangent
    );
    assert!(report.ordering.is_none());
}

#[test]
fn algebraic_same_tangent_order_uses_second_derivative_side_witness() {
    let tangent = tangent_vector(&horizontal());
    let upward_second = BezierAlgebraicTangentVector2::new(
        tangent.dx().clone(),
        tangent_vector(&upward()).dy().clone(),
    );
    let downward_second = BezierAlgebraicTangentVector2::new(
        tangent.dx().clone(),
        tangent_vector(&downward()).dy().clone(),
    );

    let report = decided(compare_algebraic_same_tangent_second_order(
        &tangent,
        &upward_second,
        &tangent,
        &downward_second,
        &policy(),
    ));

    assert_eq!(
        report.status,
        BezierAlgebraicSameTangentOrderStatus::Ordered
    );
    assert_eq!(
        report.ordering,
        Some(BezierTangentTurnOrdering2::FirstBeforeSecond)
    );
    assert!(report.first_curvature_cross.unwrap().sign.unwrap().is_gt());
    assert!(report.second_curvature_cross.unwrap().sign.unwrap().is_lt());
}

#[test]
fn algebraic_same_tangent_order_rejects_equal_second_order_evidence() {
    let tangent = tangent_vector(&horizontal());
    let upward_second = BezierAlgebraicTangentVector2::new(
        tangent.dx().clone(),
        tangent_vector(&upward()).dy().clone(),
    );

    let report = decided(compare_algebraic_same_tangent_second_order(
        &tangent,
        &upward_second,
        &tangent,
        &upward_second,
        &policy(),
    ));

    assert_eq!(
        report.status,
        BezierAlgebraicSameTangentOrderStatus::SameDirection
    );
    assert!(report.ordering.is_none());
}

#[test]
fn algebraic_same_tangent_order_uses_third_derivative_after_zero_curvature() {
    let tangent = tangent_vector(&horizontal());
    let zero_second = BezierAlgebraicTangentVector2::new(
        tangent_vector(&upward()).dx().clone(),
        tangent_vector(&upward()).dx().clone(),
    );
    let upward_third = BezierAlgebraicTangentVector2::new(
        tangent_vector(&upward()).dx().clone(),
        tangent_vector(&upward()).dy().clone(),
    );
    let downward_third = BezierAlgebraicTangentVector2::new(
        tangent_vector(&downward()).dx().clone(),
        tangent_vector(&downward()).dy().clone(),
    );

    let second_report = decided(compare_algebraic_same_tangent_second_order(
        &tangent,
        &zero_second,
        &tangent,
        &zero_second,
        &policy(),
    ));
    assert_eq!(
        second_report.status,
        BezierAlgebraicSameTangentOrderStatus::SameDirection
    );

    let third_report = decided(compare_algebraic_same_tangent_third_order(
        &tangent,
        &upward_third,
        &tangent,
        &downward_third,
        &policy(),
    ));

    assert_eq!(
        third_report.status,
        BezierAlgebraicSameTangentOrderStatus::Ordered
    );
    assert_eq!(
        third_report.ordering,
        Some(BezierTangentTurnOrdering2::FirstBeforeSecond)
    );
    assert!(
        third_report
            .first_curvature_cross
            .unwrap()
            .sign
            .unwrap()
            .is_gt()
    );
    assert!(
        third_report
            .second_curvature_cross
            .unwrap()
            .sign
            .unwrap()
            .is_lt()
    );
}
