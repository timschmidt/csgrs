//! A certified construction must keep its exact source usable across policies.

use hypercurve::{
    Curve2, CurveCertainty, CurveParameter2, CurvePoint2, LineSeg2, Point2, PolynomialCurveSet2,
    PredicatePolicy, Real, Similarity2, evaluate_under,
};
use hypersolve::BivariatePolynomial;
use hypersolve::bivariate_topology::{BivariateCurveDomain, decompose_bivariate_curve};
use std::cmp::Ordering;

fn branch() -> Curve2 {
    // The unique real root alpha of y^5+5y+5 lies in (-1,0):
    // the derivative is positive, p(-1)=-1 and p(0)=5.
    let equation = BivariatePolynomial::new(vec![vec![
        5.into(),
        5.into(),
        0.into(),
        0.into(),
        0.into(),
        1.into(),
    ]]);
    let topology = decompose_bivariate_curve(
        &equation,
        &BivariateCurveDomain {
            first: [(-1).into(), 1.into()],
            second: [(-2).into(), 2.into()],
        },
        Default::default(),
    )
    .unwrap();
    let set = PolynomialCurveSet2::from_topology(topology, Default::default()).unwrap();
    assert_eq!(set.curves().len(), 1);
    set.curves()[0].clone()
}

fn turn() -> Similarity2 {
    Similarity2::try_from_real_affine(
        0.into(),
        (-1).into(),
        1.into(),
        0.into(),
        3.into(),
        4.into(),
    )
    .unwrap()
}

fn check_point(point: &CurvePoint2, expected: &CurvePoint2, source_x: i32) {
    // T(x,y)=(3-y,4+x), so the world ordinate is known exactly and
    // the abscissa remains selected in (3,4).
    assert_eq!(
        point
            .affine_form(0.into(), 1.into(), (-(4 + source_x)).into())
            .compare_to_real(&Real::zero())
            .unwrap(),
        Ordering::Equal
    );
    for (bound, expected_order) in [(3, Ordering::Greater), (4, Ordering::Less)] {
        assert_eq!(
            point
                .affine_form(1.into(), 0.into(), (-bound).into())
                .compare_to_real(&Real::zero())
                .unwrap(),
            expected_order
        );
    }
    assert!(point.coincides_with(expected).unwrap());
    let replay = point.definition().unwrap().into_point().unwrap();
    assert!(replay.coincides_with(expected).unwrap());
}

#[test]
fn certified_selected_point_created_under_approximate_policy_replays_under_strict() {
    let curve = branch().transform_similarity(&turn()).unwrap();
    let parameter = CurveParameter2::from(Real::zero());
    let expected = curve.point_at(&parameter).unwrap();
    check_point(&expected, &expected, 0);
    let outcome = evaluate_under(PredicatePolicy::APPROXIMATE_512, || {
        curve.point_at(&parameter).unwrap()
    });
    assert_eq!(outcome.certainty(), CurveCertainty::Certified);
    let point = outcome.certified().unwrap();
    check_point(&point, &expected, 0);
    let chord = Curve2::try_line(Point2::from_values(0, 4).into(), point).unwrap();
    let contact = curve.intersect_curve(&chord).unwrap();
    assert!(contact.is_complete());
    assert_eq!(contact.contacts().len(), 1);
    assert!(
        contact.contacts()[0]
            .point()
            .coincides_with(&expected)
            .unwrap()
    );
}

#[test]
fn certified_selected_similarity_created_under_approximate_policy_keeps_strict_endpoints() {
    let source = branch();
    let expected = source.transform_similarity(&turn()).unwrap();
    let outcome = evaluate_under(PredicatePolicy::APPROXIMATE_512, || {
        source.transform_similarity(&turn()).unwrap()
    });
    assert_eq!(outcome.certainty(), CurveCertainty::Certified);
    let image = outcome.certified().unwrap();
    for (point, reference, parameter) in [
        (image.start(), expected.start(), -1),
        (image.end(), expected.end(), 1),
    ] {
        check_point(&point, &reference, parameter);
    }
    let cut = Curve2::from(
        LineSeg2::try_new(Point2::from_values(0, 4), Point2::from_values(5, 4)).unwrap(),
    );
    let topology = image.intersection_topology(&cut).unwrap();
    assert!(topology.result().is_complete());
    assert_eq!(topology.result().contacts().len(), 1);
    assert_eq!(topology.first().len(), 2);
    let contact = topology.result().contacts()[0].point();
    check_point(
        contact,
        &expected.point_at(&Real::zero().into()).unwrap(),
        0,
    );
}
