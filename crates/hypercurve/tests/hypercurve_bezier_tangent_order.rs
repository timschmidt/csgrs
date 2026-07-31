#![cfg(feature = "predicates")]

use hypercurve::{
    BezierAlgebraicEndpointImage2, BezierAlgebraicParameter2,
    BezierAlgebraicSameTangentOrderStatus, BezierAlgebraicTangentOrderStatus,
    BezierAlgebraicTangentVector2, BezierAlgebraicTangentVectorEvidence,
    BezierAlgebraicTangentVectorStatus, BezierEndpointTangentImage2, BezierParameterInterval,
    BezierParameterPolynomial, BezierTangentTurnOrdering2, Classification, CurveContext, Point2,
    QuadraticBezier2, RationalQuadraticBezier2, Real, compare_algebraic_same_tangent_second_order,
    compare_algebraic_same_tangent_third_order, compare_algebraic_tangent_turn_from_base,
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

fn policy() -> CurveContext {
    CurveContext::STRICT
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
            interval(q(7, 10), q(18, 25)),
            &policy(),
        )
        .unwrap(),
    )
}

fn sqrt_three_quarters_parameter() -> BezierAlgebraicParameter2 {
    decided(
        BezierAlgebraicParameter2::try_isolate(
            polynomial(vec![r(-3), r(0), r(4)]),
            interval(q(17, 20), q(7, 8)),
            &policy(),
        )
        .unwrap(),
    )
}

fn algebraic_midpoint_parameter() -> BezierAlgebraicParameter2 {
    decided(
        BezierAlgebraicParameter2::try_isolate(
            polynomial(vec![r(-1), r(2)]),
            interval(q(2, 5), q(3, 5)),
            &policy(),
        )
        .unwrap(),
    )
}

fn tangent_vector(curve: &QuadraticBezier2) -> BezierAlgebraicTangentVector2 {
    tangent_vector_at(curve, &sqrt_half_parameter())
}

fn tangent_vector_at(
    curve: &QuadraticBezier2,
    parameter: &BezierAlgebraicParameter2,
) -> BezierAlgebraicTangentVector2 {
    let tangent = curve
        .tangent_at_algebraic_parameter(parameter, &policy())
        .unwrap();
    let BezierAlgebraicTangentVectorEvidence { status, vector, .. } =
        BezierAlgebraicTangentVector2::from_endpoint_image(
            &BezierEndpointTangentImage2::Polynomial(tangent),
        );
    assert_eq!(status, BezierAlgebraicTangentVectorStatus::Extracted);
    vector.unwrap()
}

fn rising() -> QuadraticBezier2 {
    QuadraticBezier2::new(pi(0, 0), p(q(1, 2), r(0)), p(r(1), q(1, 2)))
}

fn rational_endpoint_vectors(
    curve: &RationalQuadraticBezier2,
) -> (BezierAlgebraicTangentVector2, BezierAlgebraicTangentVector2) {
    let image = BezierAlgebraicEndpointImage2::rational_quadratic(
        curve,
        &algebraic_midpoint_parameter(),
        &policy(),
    )
    .unwrap();
    let tangent = BezierAlgebraicTangentVector2::from_endpoint_image(image.tangent());
    assert_eq!(
        tangent.status,
        BezierAlgebraicTangentVectorStatus::Extracted
    );
    let second_derivative = BezierAlgebraicTangentVector2::from_endpoint_image(
        image
            .second_derivative()
            .expect("rational conic endpoint should retain second derivative evidence"),
    );
    assert_eq!(
        second_derivative.status,
        BezierAlgebraicTangentVectorStatus::Extracted
    );
    (tangent.vector.unwrap(), second_derivative.vector.unwrap())
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

fn rational_horizontal_midpoint_inflection(curvature: i32) -> RationalQuadraticBezier2 {
    RationalQuadraticBezier2::try_new(
        Point2::new(r(-1), r(curvature)),
        Point2::new(r(0), r(-curvature)),
        Point2::new(r(1), r(curvature)),
        r(1),
        r(1),
        r(1),
    )
    .unwrap()
}

#[test]
fn algebraic_tangent_order_separates_opposite_half_turns() {
    let base = tangent_vector(&horizontal());
    let first = tangent_vector(&upward());
    let second = tangent_vector(&downward());

    let evidence = decided(compare_algebraic_tangent_turn_from_base(
        &base,
        &first,
        &second,
        &policy(),
    ));

    assert_eq!(evidence.status, BezierAlgebraicTangentOrderStatus::Ordered);
    assert_eq!(
        evidence.ordering,
        Some(BezierTangentTurnOrdering2::FirstBeforeSecond)
    );
    assert!(evidence.base_first_cross.unwrap().sign.unwrap().is_gt());
    assert!(evidence.base_second_cross.unwrap().sign.unwrap().is_lt());
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

    let evidence = decided(compare_algebraic_tangent_turn_from_base(
        &base,
        &first,
        &second,
        &policy(),
    ));

    assert_eq!(evidence.status, BezierAlgebraicTangentOrderStatus::Ordered);
    assert_eq!(
        evidence.ordering,
        Some(BezierTangentTurnOrdering2::SecondBeforeFirst)
    );
    let cross = evidence.first_second_cross.unwrap();
    assert!(cross.scalar.unwrap().is_valid());
    assert!(cross.sign.unwrap().is_lt());
}

#[test]
fn algebraic_tangent_order_handles_distinct_generators_with_disjoint_enclosures() {
    let base = tangent_vector(&horizontal());
    let curve = rising();
    let first_source = tangent_vector_at(&curve, &sqrt_half_parameter());
    let second_source = tangent_vector_at(&curve, &sqrt_three_quarters_parameter());
    let first =
        BezierAlgebraicTangentVector2::new(first_source.dy().clone(), first_source.dx().clone());
    let second =
        BezierAlgebraicTangentVector2::new(second_source.dx().clone(), second_source.dy().clone());

    let evidence = decided(compare_algebraic_tangent_turn_from_base(
        &base,
        &first,
        &second,
        &policy(),
    ));

    assert_eq!(evidence.status, BezierAlgebraicTangentOrderStatus::Ordered);
    assert_eq!(
        evidence.ordering,
        Some(BezierTangentTurnOrdering2::SecondBeforeFirst)
    );
    let cross = evidence.first_second_cross.unwrap();
    assert!(cross.sign.unwrap().is_lt());
}

#[test]
fn algebraic_tangent_order_evidence_same_direction_without_guessing() {
    let base = tangent_vector(&horizontal());
    let first = tangent_vector(&upward());
    let second = tangent_vector(&upward());

    let evidence = decided(compare_algebraic_tangent_turn_from_base(
        &base,
        &first,
        &second,
        &policy(),
    ));

    assert_eq!(
        evidence.status,
        BezierAlgebraicTangentOrderStatus::SameDirection
    );
    assert!(evidence.ordering.is_none());
}

#[test]
fn algebraic_tangent_order_rejects_zero_tangent() {
    let base = tangent_vector(&horizontal());
    let zero = tangent_vector(&QuadraticBezier2::new(pi(0, 0), pi(0, 0), pi(0, 0)));
    let second = tangent_vector(&upward());

    let evidence = decided(compare_algebraic_tangent_turn_from_base(
        &base,
        &zero,
        &second,
        &policy(),
    ));

    assert_eq!(
        evidence.status,
        BezierAlgebraicTangentOrderStatus::ZeroTangent
    );
    assert!(evidence.ordering.is_none());
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

    let evidence = decided(compare_algebraic_same_tangent_second_order(
        &tangent,
        &upward_second,
        &tangent,
        &downward_second,
        &policy(),
    ));

    assert_eq!(
        evidence.status,
        BezierAlgebraicSameTangentOrderStatus::Ordered
    );
    assert_eq!(
        evidence.ordering,
        Some(BezierTangentTurnOrdering2::FirstBeforeSecond)
    );
    assert!(
        evidence
            .first_curvature_cross
            .unwrap()
            .sign
            .unwrap()
            .is_gt()
    );
    assert!(
        evidence
            .second_curvature_cross
            .unwrap()
            .sign
            .unwrap()
            .is_lt()
    );
}

#[test]
fn rational_algebraic_same_tangent_order_uses_second_derivative_side_witness() {
    let upward = rational_horizontal_midpoint_inflection(1);
    let downward = rational_horizontal_midpoint_inflection(-1);
    let (upward_tangent, upward_second) = rational_endpoint_vectors(&upward);
    let (downward_tangent, downward_second) = rational_endpoint_vectors(&downward);

    let evidence = decided(compare_algebraic_same_tangent_second_order(
        &upward_tangent,
        &upward_second,
        &downward_tangent,
        &downward_second,
        &policy(),
    ));

    assert_eq!(
        evidence.status,
        BezierAlgebraicSameTangentOrderStatus::Ordered
    );
    assert_eq!(
        evidence.ordering,
        Some(BezierTangentTurnOrdering2::FirstBeforeSecond)
    );
    assert!(
        evidence
            .first_curvature_cross
            .unwrap()
            .sign
            .unwrap()
            .is_gt()
    );
    assert!(
        evidence
            .second_curvature_cross
            .unwrap()
            .sign
            .unwrap()
            .is_lt()
    );
}

#[test]
fn algebraic_same_tangent_order_rejects_equal_second_order_evidence() {
    let tangent = tangent_vector(&horizontal());
    let upward_second = BezierAlgebraicTangentVector2::new(
        tangent.dx().clone(),
        tangent_vector(&upward()).dy().clone(),
    );

    let evidence = decided(compare_algebraic_same_tangent_second_order(
        &tangent,
        &upward_second,
        &tangent,
        &upward_second,
        &policy(),
    ));

    assert_eq!(
        evidence.status,
        BezierAlgebraicSameTangentOrderStatus::SameDirection
    );
    assert!(evidence.ordering.is_none());
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

    let second_evidence = decided(compare_algebraic_same_tangent_second_order(
        &tangent,
        &zero_second,
        &tangent,
        &zero_second,
        &policy(),
    ));
    assert_eq!(
        second_evidence.status,
        BezierAlgebraicSameTangentOrderStatus::SameDirection
    );

    let third_evidence = decided(compare_algebraic_same_tangent_third_order(
        &tangent,
        &upward_third,
        &tangent,
        &downward_third,
        &policy(),
    ));

    assert_eq!(
        third_evidence.status,
        BezierAlgebraicSameTangentOrderStatus::Ordered
    );
    assert_eq!(
        third_evidence.ordering,
        Some(BezierTangentTurnOrdering2::FirstBeforeSecond)
    );
    assert!(
        third_evidence
            .first_curvature_cross
            .unwrap()
            .sign
            .unwrap()
            .is_gt()
    );
    assert!(
        third_evidence
            .second_curvature_cross
            .unwrap()
            .sign
            .unwrap()
            .is_lt()
    );
}
