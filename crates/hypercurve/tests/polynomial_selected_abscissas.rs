//! Polynomial source charts must consume retained rational coordinates.

use hypercurve::{Curve2, CurvePoint2, Point2, PolynomialCurveSet2, QuadraticBezier2, Real};
use hypersolve::bivariate_topology::{
    BivariateCurveDomain, BivariateTopologyConfig, decompose_bivariate_curve,
};
use hypersolve::{AlgebraicFiberRootIsolationConfig, BivariatePolynomial};
use std::cmp::Ordering;

fn polynomial(rows: &[&[i32]]) -> BivariatePolynomial {
    BivariatePolynomial::new(
        rows.iter()
            .map(|row| row.iter().copied().map(Real::from).collect())
            .collect(),
    )
}

fn curves(equation: BivariatePolynomial) -> PolynomialCurveSet2 {
    let topology = decompose_bivariate_curve(
        &equation,
        &BivariateCurveDomain {
            first: [(-1).into(), 1.into()],
            second: [(-2).into(), 2.into()],
        },
        BivariateTopologyConfig::default(),
    )
    .unwrap();
    PolynomialCurveSet2::from_topology(topology, AlgebraicFiberRootIsolationConfig::default())
        .unwrap()
}

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn on(point: &CurvePoint2, equation: BivariatePolynomial) {
    assert_eq!(
        point
            .rational_form(equation, polynomial(&[&[1]]))
            .unwrap()
            .compare_to_real(&Real::zero())
            .unwrap(),
        Ordering::Equal
    );
}

fn quintic() -> BivariatePolynomial {
    polynomial(&[&[5, 5, 0, 0, 0, 1], &[-1]])
}

#[test]
fn polynomial_branch_consumes_retained_rational_abscissa() {
    // C(t)=(t^3,t^2); at y=1/2 the positive t and x=t^3 remain correlated.
    let cubic = Curve2::from(hypercurve::CubicBezier2::new(
        Point2::from_values(0, 0),
        Point2::from_values(0, 0),
        Point2::new(0.into(), q(1, 3)),
        Point2::from_values(1, 1),
    ));
    let line = Curve2::from(
        hypercurve::LineSeg2::try_new(
            Point2::new(0.into(), q(1, 2)),
            Point2::new(1.into(), q(1, 2)),
        )
        .unwrap(),
    );
    let contact = cubic.intersect_curve(&line).unwrap();
    assert_eq!(contact.contacts().len(), 1);
    let point = contact.contacts()[0].point().clone();
    assert!(point.coordinates().is_none());
    on(&point, polynomial(&[&[0, 0, 0, 1], &[], &[-1]]));
    let x = point
        .affine_form(1.into(), 0.into(), 0.into())
        .parameter()
        .unwrap();
    drop(contact);
    drop(cubic);
    drop(line);
    let set = curves(polynomial(&[&[0, 0, 0, 1], &[], &[-1]]));
    assert_eq!(set.curves().len(), 2);
    let branch = &set.curves()[1];
    assert_eq!(
        branch
            .parameter_domain()
            .start()
            .compare(&Real::zero().into())
            .unwrap(),
        Ordering::Equal
    );
    let replay = branch.point_at(&x).unwrap();
    assert!(replay.coincides_with(&point).unwrap());
    on(&replay, polynomial(&[&[0, 0, 0, 1], &[], &[-1]]));
    assert_eq!(
        replay
            .affine_form(0.into(), 1.into(), 0.into())
            .compare_to_real(&q(1, 2))
            .unwrap(),
        Ordering::Equal
    );
}

#[test]
fn polynomial_branch_consumes_fifth_degree_rational_abscissa() {
    // C(t)=(t^2,t^5), so at y=1/2 the source x solves 4x^5=1.
    let controls = (0..=5)
        .map(|i| {
            Point2::new(
                q(i * (i - 1), 20),
                if i == 5 { Real::one() } else { Real::zero() },
            )
        })
        .collect();
    let source =
        Curve2::from(hypercurve::RationalBezier2::try_new(controls, vec![Real::one(); 6]).unwrap());
    let line = Curve2::from(
        hypercurve::LineSeg2::try_new(
            Point2::new(0.into(), q(1, 2)),
            Point2::new(1.into(), q(1, 2)),
        )
        .unwrap(),
    );
    let contact = source.intersect_curve(&line).unwrap();
    assert_eq!(contact.contacts().len(), 1);
    let point = contact.contacts()[0].point().clone();
    assert!(point.coordinates().is_none());
    let equation = polynomial(&[&[0, 0, 1], &[], &[], &[], &[], &[-1]]);
    on(&point, equation.clone());
    let x = point
        .affine_form(1.into(), 0.into(), 0.into())
        .parameter()
        .unwrap();
    drop(contact);
    drop(source);
    drop(line);
    let set = curves(equation.clone());
    assert_eq!(set.curves().len(), 2);
    let replay = set.curves()[1].point_at(&x).unwrap();
    assert!(replay.coincides_with(&point).unwrap());
    on(&replay, equation);
    on(&replay, polynomial(&[&[-1], &[], &[], &[], &[], &[4]]));
    assert_eq!(
        replay
            .affine_form(0.into(), 1.into(), 0.into())
            .compare_to_real(&q(1, 2))
            .unwrap(),
        Ordering::Equal
    );
}

#[test]
fn polynomial_branch_consumes_generic_quintic_abscissa_without_projection() {
    let polynomial = hypercurve::BezierParameterPolynomial::try_new_power_basis(
        [-5, 5, 1, 0, 0, 1].into_iter().map(Real::from).collect(),
    )
    .unwrap();
    let roots = polynomial.isolate_unit_interval_roots().unwrap();
    assert_eq!(roots.len(), 1);
    let parameter = roots[0].clone().into();
    let source = Curve2::from(QuadraticBezier2::new(
        Point2::from_values(0, 0),
        Point2::new(0.into(), q(-1, 2)),
        Point2::from_values(1, -1),
    ));
    let point = source.point_at(&parameter).unwrap();
    assert!(point.coordinates().is_none());
    on(&point, quintic());
    on(&point, polynomial_from_parabola());
    let x = point
        .affine_form(1.into(), 0.into(), 0.into())
        .parameter()
        .unwrap();
    assert!(x.as_bezier_parameter().is_none());
    drop(source);
    drop(roots);
    let set = curves(quintic());
    let replay = set.curves()[0].point_at(&x).unwrap();
    assert!(replay.coincides_with(&point).unwrap());
    on(&replay, quintic());
    on(&replay, polynomial_from_parabola());
}

fn polynomial_from_parabola() -> BivariatePolynomial {
    polynomial(&[&[0, 0, 1], &[-1]])
}

fn generic_parameter() -> hypercurve::BezierParameter2 {
    let polynomial = hypercurve::BezierParameterPolynomial::try_new_power_basis(
        [-5, 5, 1, 0, 0, 1].into_iter().map(Real::from).collect(),
    )
    .unwrap();
    let roots = polynomial.isolate_unit_interval_roots().unwrap();
    assert_eq!(roots.len(), 1);
    roots[0].clone()
}

fn generic_point() -> CurvePoint2 {
    Curve2::from(QuadraticBezier2::new(
        Point2::from_values(0, 0),
        Point2::new(0.into(), q(-1, 2)),
        Point2::from_values(1, -1),
    ))
    .point_at(&generic_parameter().into())
    .unwrap()
}

#[test]
fn selected_abscissa_reuses_its_output_coordinate() {
    let point = generic_point();
    let set = curves(quintic());
    let mut x = point
        .affine_form(1.into(), 0.into(), 0.into())
        .parameter()
        .unwrap();
    for _ in 0..4 {
        let result = set.curves()[0].point_at(&x).unwrap();
        assert!(result.coincides_with(&point).unwrap());
        on(&result, quintic());
        x = result
            .affine_form(1.into(), 0.into(), 0.into())
            .parameter()
            .unwrap();
        // Rebuild the definition; this must not depend on the result's live cache.
        let replay = result.definition().unwrap().into_point().unwrap();
        assert!(replay.coincides_with(&point).unwrap());
        let replay_x = replay
            .affine_form(1.into(), 0.into(), 0.into())
            .parameter()
            .unwrap();
        assert!(
            set.curves()[0]
                .point_at(&replay_x)
                .unwrap()
                .coincides_with(&point)
                .unwrap()
        );
    }
}

#[test]
fn selected_abscissa_preserves_rational_denominators() {
    let circle = polynomial(&[&[-1, 0, 1], &[], &[1]]);
    let set = curves(circle.clone());
    assert_eq!(set.curves().len(), 2);
    for sign in [-1, 1] {
        // The same lower circular point with either sign of its shared weight.
        let point = hypercurve::CurvePointDefinition2::RationalExpression {
            parameter: generic_parameter(),
            x_numerator: vec![Real::from(sign), Real::zero(), Real::from(-sign)],
            y_numerator: vec![Real::zero(), Real::from(-2 * sign)],
            denominator: vec![Real::from(sign), Real::zero(), Real::from(sign)],
        }
        .into_point()
        .unwrap();
        on(&point, circle.clone());
        let x = point
            .affine_form(1.into(), 0.into(), 0.into())
            .parameter()
            .unwrap();
        assert!(x.as_bezier_parameter().is_none());
        let result = set.curves()[0].point_at(&x).unwrap();
        assert!(result.coincides_with(&point).unwrap());
        on(&result, circle.clone());
        assert_eq!(
            result
                .affine_form(0.into(), 1.into(), 0.into())
                .compare_to_real(&Real::zero())
                .unwrap(),
            Ordering::Less
        );
        let replay = result.definition().unwrap().into_point().unwrap();
        on(&replay, circle.clone());
        assert!(replay.coincides_with(&point).unwrap());
    }
    // Original denominator guards must reject an excluded selected parameter,
    // even though cancelling identical numerator/denominator gives finite values.
    let vanishing: Vec<_> = [-5, 5, 1, 0, 0, 1].into_iter().map(Real::from).collect();
    assert!(matches!(
        hypercurve::CurvePointDefinition2::RationalExpression {
            parameter: generic_parameter(),
            x_numerator: vanishing.clone(),
            y_numerator: vanishing.clone(),
            denominator: vanishing,
        }
        .into_point(),
        Err(hypercurve::ExactCurveError::Invalid { .. })
    ));
}

#[test]
fn selected_abscissa_keeps_the_ordered_branch() {
    let point = generic_point();
    let x = point
        .affine_form(1.into(), 0.into(), 0.into())
        .parameter()
        .unwrap();
    let set = curves(polynomial_from_parabola());
    assert_eq!(set.curves().len(), 2);
    let negative = set.curves()[0].point_at(&x).unwrap();
    let positive = set.curves()[1].point_at(&x).unwrap();
    assert!(negative.coincides_with(&point).unwrap());
    assert!(!positive.coincides_with(&negative).unwrap());
    for (result, sign) in [(&negative, Ordering::Less), (&positive, Ordering::Greater)] {
        on(result, polynomial_from_parabola());
        assert_eq!(
            result
                .affine_form(0.into(), 1.into(), 0.into())
                .compare_to_real(&Real::zero())
                .unwrap(),
            sign
        );
    }
    assert_eq!(
        positive
            .affine_form(0.into(), 1.into(), 0.into())
            .compare(&point.affine_form(0.into(), (-1).into(), 0.into()))
            .unwrap(),
        Ordering::Equal
    );
}

#[test]
fn selected_abscissa_keeps_restrictions_and_transforms() {
    let point = generic_point();
    let x = point
        .affine_form(1.into(), 0.into(), 0.into())
        .parameter()
        .unwrap();
    let set = curves(quintic());
    let branch = &set.curves()[0];
    let restricted = branch
        .restrict_source(
            hypercurve::CurveParameterRange2::try_new(q(1, 4).into(), q(3, 4).into()).unwrap(),
        )
        .unwrap();
    assert!(
        restricted
            .point_at(&x)
            .unwrap()
            .coincides_with(&point)
            .unwrap()
    );
    let reversed = restricted.reversed().unwrap();
    assert!(
        reversed
            .point_at(&x)
            .unwrap()
            .coincides_with(&point)
            .unwrap()
    );
    let transform = hypercurve::Similarity2::try_from_real_affine(
        0.into(),
        (-1).into(),
        1.into(),
        0.into(),
        3.into(),
        4.into(),
    )
    .unwrap();
    let transformed = reversed.transform_similarity(&transform).unwrap();
    let result = transformed.point_at(&x).unwrap();
    // F( y-4, 3-x ) = 0, independently expanded.
    on(
        &result,
        polynomial(&[&[267, -1], &[-410], &[270], &[-90], &[15], &[-1]]),
    );
    let expected = hypercurve::CurvePointDefinition2::Similarity {
        source: point,
        transform,
    }
    .into_point()
    .unwrap();
    assert!(result.coincides_with(&expected).unwrap());
    assert!(matches!(
        restricted.point_at(&q(7, 8).into()),
        Err(hypercurve::ExactCurveError::Invalid { .. })
    ));
    let outside = generic_point()
        .affine_form(1.into(), 0.into(), 2.into())
        .parameter()
        .unwrap();
    assert!(matches!(
        branch.point_at(&outside),
        Err(hypercurve::ExactCurveError::Invalid { .. })
    ));
}

#[test]
fn selected_abscissa_retries_after_exhaustion() {
    let point = generic_point();
    let x = point
        .affine_form(1.into(), 0.into(), 0.into())
        .parameter()
        .unwrap();
    let set = curves(quintic());
    let branch = &set.curves()[0];
    let denied = hypersolve::work_budget::with_budget(
        hypersolve::work_budget::WorkBudget::ZERO,
        || {
            assert!(
                matches!(branch.point_at(&x), Err(hypercurve::ExactCurveError::Blocked(blocker)) if blocker.reason() == hypercurve::UncertaintyReason::BudgetExhausted)
            );
        },
    );
    let reached = denied.result.unwrap_err();
    assert_eq!((reached.limit, reached.used), (0, 0));
    assert!(reached.requested > 0);
    let retry = hypersolve::work_budget::with_budget(
        hypersolve::work_budget::WorkBudget {
            refinements: 100_000,
            promotions: 100_000,
        },
        || branch.point_at(&x),
    );
    let result = retry.result.unwrap().unwrap();
    assert!(result.coincides_with(&point).unwrap());
    for policy in [
        hypercurve::PredicatePolicy::STRICT,
        hypercurve::PredicatePolicy::APPROXIMATE_512,
        hypercurve::PredicatePolicy::STRICT,
    ] {
        let outcome = hypercurve::evaluate_under(policy, || {
            let result = branch.point_at(&x).unwrap();
            on(&result, quintic());
            assert!(result.coincides_with(&point).unwrap());
        });
        assert_eq!(outcome.certainty(), hypercurve::CurveCertainty::Certified);
        outcome.certified().unwrap();
    }
}

#[test]
fn selected_abscissa_consumes_zero_distance_source_definition() {
    let source = QuadraticBezier2::new(
        Point2::from_values(0, 0),
        Point2::new(0.into(), q(-1, 2)),
        Point2::from_values(1, -1),
    );
    let point = hypercurve::CurvePointDefinition2::Parallel {
        source: source.parallel_left(Real::zero()).unwrap(),
        parameter: generic_parameter().into(),
    }
    .into_point()
    .unwrap();
    assert!(matches!(
        point.definition().unwrap(),
        hypercurve::CurvePointDefinition2::Parallel { .. }
    ));
    let x = point
        .affine_form(1.into(), 0.into(), 0.into())
        .parameter()
        .unwrap();
    assert!(x.as_bezier_parameter().is_none());
    let set = curves(quintic());
    let result = set.curves()[0].point_at(&x).unwrap();
    on(&result, quintic());
    on(&result, polynomial_from_parabola());
    assert!(result.coincides_with(&point).unwrap());
}

#[test]
fn selected_abscissa_distinguishes_changed_sources_and_nearby_values() {
    let source = Curve2::from(QuadraticBezier2::new(
        Point2::from_values(0, 0),
        Point2::new(0.into(), q(-1, 2)),
        Point2::from_values(1, -1),
    ));
    let set = curves(polynomial_from_parabola());
    let branch = &set.curves()[0];
    let mut results = Vec::new();
    for constant in [-5, -4] {
        let defining = hypercurve::BezierParameterPolynomial::try_new_power_basis(
            [constant, 5, 1, 0, 0, 1]
                .into_iter()
                .map(Real::from)
                .collect(),
        )
        .unwrap();
        let roots = defining.isolate_unit_interval_roots().unwrap();
        assert_eq!(roots.len(), 1);
        let original = source.point_at(&roots[0].clone().into()).unwrap();
        let x = original
            .affine_form(1.into(), 0.into(), 0.into())
            .parameter()
            .unwrap();
        assert!(x.as_bezier_parameter().is_none());
        let point = branch.point_at(&x).unwrap();
        assert!(point.coincides_with(&original).unwrap());
        on(&point, polynomial_from_parabola());
        results.push(point);
    }
    assert!(!results[0].coincides_with(&results[1]).unwrap());
    let tiny = q(1, 1 << 20);
    let tiny = &tiny * &tiny;
    let tiny = &tiny * &tiny;
    let shifted_x = generic_point()
        .affine_form(1.into(), 0.into(), tiny)
        .parameter()
        .unwrap();
    let shifted = branch.point_at(&shifted_x).unwrap();
    on(&shifted, polynomial_from_parabola());
    assert!(!results[0].coincides_with(&shifted).unwrap());
    assert_eq!(
        shifted
            .affine_form(1.into(), 0.into(), 0.into())
            .compare(&results[0].affine_form(1.into(), 0.into(), 0.into()))
            .unwrap(),
        Ordering::Greater
    );
}
