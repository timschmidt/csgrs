//! Selected polynomial points retain incidence in their original source chart.

use hypercurve::{
    Curve2, CurveParameter2, CurvePoint2, CurvePointLocations2, Point2, PolynomialCurveSet2,
    QuadraticBezier2, Real,
};
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

fn located(curve: &Curve2, point: &CurvePoint2, expected: &CurveParameter2) -> CurveParameter2 {
    let CurvePointLocations2::Locations(locations) = curve.point_locations(point).unwrap() else {
        panic!("an injective graph has a single preimage");
    };
    assert_eq!(locations.len(), 1);
    assert_eq!(locations[0].span_index(), 0);
    let parameter = locations[0].parameter().unwrap();
    assert_eq!(parameter.compare(expected).unwrap(), Ordering::Equal);
    assert!(
        curve
            .point_at(&parameter)
            .unwrap()
            .coincides_with(point)
            .unwrap()
    );
    parameter
}

fn absent(curve: &Curve2, point: &CurvePoint2) {
    let CurvePointLocations2::Locations(locations) = curve.point_locations(point).unwrap() else {
        panic!("an off-curve point has no preimage");
    };
    assert!(locations.is_empty());
}

#[test]
fn polynomial_locations_consume_selected_fiber_output() {
    let set = curves(polynomial(&[&[5, 5, 0, 0, 0, 1], &[-1]]));
    assert_eq!(set.curves().len(), 1);
    let curve = &set.curves()[0];
    let x = CurveParameter2::from(Real::zero());
    let point = curve.point_at(&x).unwrap();
    assert!(point.coordinates().is_none());
    located(curve, &point, &x);
    located(&curve.reversed().unwrap(), &point, &x);
    let replay = point.definition().unwrap().into_point().unwrap();
    located(curve, &replay, &x);
}

#[test]
fn polynomial_locations_check_native_points_and_closed_trims() {
    let set = curves(polynomial(&[&[0, 1], &[], &[-1]]));
    assert_eq!(set.curves().len(), 1);
    let curve = &set.curves()[0];
    located(
        curve,
        &Point2::new(q(1, 2), q(1, 4)).into(),
        &q(1, 2).into(),
    );
    absent(curve, &Point2::from_values(0, 1).into());
    absent(curve, &Point2::from_values(2, 4).into());
    let trim = curve
        .restrict_source(
            hypercurve::CurveParameterRange2::try_new(q(-1, 2).into(), q(1, 2).into()).unwrap(),
        )
        .unwrap();
    for x in [q(-1, 2), q(1, 2)] {
        located(&trim, &Point2::new(x.clone(), &x * &x).into(), &x.into());
    }
    absent(&trim, &Point2::from_values(1, 1).into());
}

#[test]
fn polynomial_locations_consume_generated_rational_contact() {
    // C(t)=(t^2,-t), with t^5+t^2+5t-5=0 selecting the only contact.
    let set = curves(polynomial(&[&[5, 5, 0, 0, 0, 1], &[-1]]));
    let curve = &set.curves()[0];
    let rational = Curve2::from(QuadraticBezier2::new(
        Point2::from_values(0, 0),
        Point2::new(0.into(), q(-1, 2)),
        Point2::from_values(1, -1),
    ));
    let result = curve.intersect_curve(&rational).unwrap();
    assert_eq!(result.contacts().len(), 1);
    let contact = &result.contacts()[0];
    let x = contact.first().parameter().unwrap();
    let point = rational
        .point_at(&contact.second().parameter().unwrap())
        .unwrap();
    assert!(point.coordinates().is_none());
    located(curve, &point, &x);
    let replay = point.definition().unwrap().into_point().unwrap();
    located(curve, &replay, &x);
}

fn generic_point() -> CurvePoint2 {
    let roots = hypercurve::BezierParameterPolynomial::try_new_power_basis(
        [-5, 5, 1, 0, 0, 1].into_iter().map(Real::from).collect(),
    )
    .unwrap()
    .isolate_unit_interval_roots()
    .unwrap();
    assert_eq!(roots.len(), 1);
    Curve2::from(QuadraticBezier2::new(
        Point2::from_values(0, 0),
        Point2::new(0.into(), q(-1, 2)),
        Point2::from_values(1, -1),
    ))
    .point_at(&roots[0].clone().into())
    .unwrap()
}

#[test]
fn polynomial_locations_distinguish_ordered_branches_and_nearby_points() {
    let set = curves(polynomial(&[&[0, 0, 1], &[-1]]));
    assert_eq!(set.curves().len(), 2);
    let x = CurveParameter2::from(q(1, 2));
    for index in 0..2 {
        let point = set.curves()[index].point_at(&x).unwrap();
        located(&set.curves()[index], &point, &x);
        absent(&set.curves()[1 - index], &point);
        located(
            &set.curves()[index],
            &Point2::from_values(0, 0).into(),
            &Real::zero().into(),
        );
    }
    let parabola = curves(polynomial(&[&[0, 1], &[], &[-1]]));
    let small = q(1, 1 << 20);
    let small = &small * &small;
    let small = &small * &small;
    for y in [small.clone(), -small] {
        absent(&parabola.curves()[0], &Point2::new(0.into(), y).into());
    }
}

#[test]
fn polynomial_locations_keep_selected_boundaries_for_later_operations() {
    let point = generic_point();
    let x = point
        .affine_form(1.into(), 0.into(), 0.into())
        .parameter()
        .unwrap();
    assert!(x.as_bezier_parameter().is_none());
    let set = curves(polynomial(&[&[5, 5, 0, 0, 0, 1], &[-1]]));
    let curve = &set.curves()[0];
    let trim = curve
        .restrict_source(
            hypercurve::CurveParameterRange2::try_new(x.clone(), q(3, 4).into()).unwrap(),
        )
        .unwrap();
    for piece in [trim.clone(), trim.reversed().unwrap()] {
        assert_eq!(located(&piece, &point, &x), x);
        absent(&piece, &curve.point_at(&q(1, 4).into()).unwrap());
        let cut = Curve2::from(
            hypercurve::LineSeg2::try_new(
                Point2::new(q(2, 3), (-2).into()),
                Point2::new(q(2, 3), 2.into()),
            )
            .unwrap(),
        );
        let topology = piece.intersection_topology(&cut).unwrap();
        assert_eq!(topology.first().len(), 2);
        let contact = piece.intersect_curve(&cut).unwrap();
        assert_eq!(contact.contacts().len(), 1);
        for part in topology.first() {
            located(part, contact.contacts()[0].point(), &q(2, 3).into());
        }
    }
}

fn on(point: &CurvePoint2, equation: BivariatePolynomial) {
    assert_eq!(
        point
            .rational_form(equation, polynomial(&[&[1]]))
            .unwrap()
            .compare_to_real(&Real::zero())
            .unwrap(),
        Ordering::Equal,
    );
}

#[test]
fn polynomial_locations_transport_rational_and_algebraic_similarities() {
    let point = generic_point();
    let x = point
        .affine_form(1.into(), 0.into(), 0.into())
        .parameter()
        .unwrap();
    let set = curves(polynomial(&[&[5, 5, 0, 0, 0, 1], &[-1]]));
    let scale = Real::from(2).sqrt().unwrap();
    let cases = [
        (
            hypercurve::Similarity2::try_from_real_affine(
                0.into(),
                (-1).into(),
                1.into(),
                0.into(),
                3.into(),
                4.into(),
            )
            .unwrap(),
            polynomial(&[&[267, -1], &[-410], &[270], &[-90], &[15], &[-1]]),
        ),
        (
            hypercurve::Similarity2::try_from_real_affine(
                (-1).into(),
                0.into(),
                0.into(),
                1.into(),
                0.into(),
                0.into(),
            )
            .unwrap(),
            polynomial(&[&[5, 5, 0, 0, 0, 1], &[1]]),
        ),
        (
            hypercurve::Similarity2::try_from_real_affine(
                scale.clone(),
                0.into(),
                0.into(),
                scale.clone(),
                0.into(),
                0.into(),
            )
            .unwrap(),
            BivariatePolynomial::new(vec![
                vec![
                    Real::from(20) * scale,
                    20.into(),
                    0.into(),
                    0.into(),
                    0.into(),
                    1.into(),
                ],
                vec![(-4).into()],
            ]),
        ),
    ];
    for (transform, equation) in cases {
        let curve = set.curves()[0]
            .reversed()
            .unwrap()
            .transform_similarity(&transform)
            .unwrap();
        let query = hypercurve::CurvePointDefinition2::Similarity {
            source: point.clone(),
            transform,
        }
        .into_point()
        .unwrap();
        on(&query, equation);
        located(&curve, &query, &x);
        located(
            &curve,
            &query.definition().unwrap().into_point().unwrap(),
            &x,
        );
        absent(&curve, &point);
    }
}

#[test]
fn polynomial_locations_preserve_singular_endpoint_incidence() {
    for equation in [
        polynomial(&[&[0, 0, 1], &[], &[], &[-1]]),
        polynomial(&[&[0, 0, 1], &[], &[-1], &[-1]]),
    ] {
        let set = curves(equation.clone());
        assert!(set.curves().len() >= 2);
        let origin = CurvePoint2::from(Point2::from_values(0, 0));
        on(&origin, equation);
        for curve in set.curves() {
            located(curve, &origin, &Real::zero().into());
            located(&curve.reversed().unwrap(), &origin, &Real::zero().into());
        }
    }
}

#[test]
fn polynomial_locations_retry_after_counted_exhaustion() {
    let point = generic_point();
    let x = point
        .affine_form(1.into(), 0.into(), 0.into())
        .parameter()
        .unwrap();
    let set = curves(polynomial(&[&[5, 5, 0, 0, 0, 1], &[-1]]));
    let branch = &set.curves()[0];
    let denied = hypersolve::work_budget::with_budget(
        hypersolve::work_budget::WorkBudget::ZERO,
        || {
            assert!(
                matches!(branch.point_locations(&point), Err(hypercurve::ExactCurveError::Blocked(blocker)) if blocker.reason() == hypercurve::UncertaintyReason::BudgetExhausted)
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
        || located(branch, &point, &x),
    );
    assert_eq!(retry.result.unwrap().compare(&x).unwrap(), Ordering::Equal);
    for policy in [
        hypercurve::PredicatePolicy::STRICT,
        hypercurve::PredicatePolicy::APPROXIMATE_512,
        hypercurve::PredicatePolicy::STRICT,
    ] {
        let outcome = hypercurve::evaluate_under(policy, || located(branch, &point, &x));
        assert_eq!(outcome.certainty(), hypercurve::CurveCertainty::Certified);
        outcome.certified().unwrap();
    }
}
