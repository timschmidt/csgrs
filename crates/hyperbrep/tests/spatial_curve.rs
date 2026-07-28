use hyperbrep::{
    BrepCurve3, BrepCurveErrorKind3, BrepCurveFamily3, BrepCurveGeometry3, BrepCurveOperation3,
    BrepLineSegment3, BrepNurbsCurve3, BrepRationalBezier3,
};
use hyperlimit::Point3;
use hyperreal::Real;

fn r(value: i32) -> Real {
    Real::from(value)
}

fn q(numerator: i32, denominator: i32) -> Real {
    (r(numerator) / r(denominator)).unwrap()
}

fn p(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}

#[test]
fn top_level_spatial_line_evaluates_exactly() {
    let curve = BrepCurve3::new(BrepCurveGeometry3::Line(Box::new(BrepLineSegment3::new(
        p(0, 0, 0),
        p(2, 4, 6),
    ))));

    assert_eq!(curve.family(), BrepCurveFamily3::Line);
    assert_eq!(curve.point_at(&q(1, 2)).unwrap(), p(1, 2, 3));
    assert_eq!(curve.parameter_domain().start(), &r(0));
    assert_eq!(curve.parameter_domain().end(), &r(1));
}

#[test]
fn spatial_rational_bezier_clones_evaluate_identically() {
    let curve = BrepRationalBezier3::try_new(
        vec![p(0, 0, 0), p(1, 2, 0), p(2, 0, 2)],
        vec![r(1), r(1), r(1)],
    )
    .unwrap();
    let clone = curve.clone();

    assert_eq!(
        curve.point_at(&q(1, 2)).unwrap(),
        Point3::new(r(1), r(1), q(1, 2))
    );
    assert_eq!(
        clone.point_at(&q(1, 2)).unwrap(),
        Point3::new(r(1), r(1), q(1, 2))
    );
    assert_eq!(clone.point_at(&r(0)).unwrap(), p(0, 0, 0));
    assert_eq!(clone.point_at(&r(1)).unwrap(), p(2, 0, 2));
}

#[test]
fn spatial_nurbs_matches_its_clamped_rational_bezier() {
    let controls = vec![p(0, 0, 0), p(1, 2, 0), p(2, 0, 2)];
    let weights = vec![r(1), r(2), r(1)];
    let bezier = BrepRationalBezier3::try_new(controls.clone(), weights.clone()).unwrap();
    let nurbs = BrepNurbsCurve3::try_new(
        2,
        controls,
        weights,
        vec![r(0), r(0), r(0), r(1), r(1), r(1)],
    )
    .unwrap();
    let clone = nurbs.clone();

    assert_eq!(nurbs.parameter_domain().start(), &r(0));
    assert_eq!(nurbs.parameter_domain().end(), &r(1));
    for parameter in [r(0), q(1, 3), q(1, 2), r(1)] {
        assert_eq!(
            nurbs.point_at(&parameter).unwrap(),
            bezier.point_at(&parameter).unwrap()
        );
    }
    assert_eq!(
        clone.point_at(&q(1, 2)).unwrap(),
        bezier.point_at(&q(1, 2)).unwrap()
    );
}

#[test]
fn spatial_nurbs_evaluates_each_nonuniform_active_span() {
    let nurbs = BrepNurbsCurve3::try_new(
        1,
        vec![p(0, 0, 0), p(2, 4, 6), p(6, 8, 10)],
        vec![r(1), r(1), r(1)],
        vec![r(0), r(0), r(2), r(5), r(5)],
    )
    .unwrap();

    assert_eq!(nurbs.parameter_domain().start(), &r(0));
    assert_eq!(nurbs.parameter_domain().end(), &r(5));
    assert_eq!(nurbs.point_at(&r(1)).unwrap(), p(1, 2, 3));
    assert_eq!(nurbs.point_at(&q(7, 2)).unwrap(), p(4, 6, 8));
    assert_eq!(nurbs.point_at(&r(5)).unwrap(), p(6, 8, 10));
}

#[test]
fn spatial_nurbs_validates_degree_knots_and_active_domain() {
    let controls = vec![p(0, 0, 0), p(1, 1, 1), p(2, 0, 2)];
    let weights = vec![r(1), r(1), r(1)];

    let degree =
        BrepNurbsCurve3::try_new(3, controls.clone(), weights.clone(), vec![r(0); 7]).unwrap_err();
    assert_eq!(degree.kind(), &BrepCurveErrorKind3::InvalidDegree);

    let count =
        BrepNurbsCurve3::try_new(2, controls.clone(), weights.clone(), vec![r(0); 5]).unwrap_err();
    assert_eq!(count.kind(), &BrepCurveErrorKind3::InvalidKnotCount);

    let order = BrepNurbsCurve3::try_new(
        2,
        controls.clone(),
        weights.clone(),
        vec![r(0), r(0), r(1), r(0), r(1), r(1)],
    )
    .unwrap_err();
    assert_eq!(order.kind(), &BrepCurveErrorKind3::InvalidKnotOrder);

    let domain = BrepNurbsCurve3::try_new(2, controls, weights, vec![r(0); 6]).unwrap_err();
    assert_eq!(domain.kind(), &BrepCurveErrorKind3::InvalidParameterDomain);
}

#[test]
fn top_level_spatial_errors_retain_operation_and_family() {
    let singular =
        BrepRationalBezier3::try_new(vec![p(0, 0, 0), p(1, 1, 1)], vec![r(0), r(0)]).unwrap();
    let curve = BrepCurve3::new(BrepCurveGeometry3::RationalBezier(singular));
    let error = curve.point_at(&q(1, 2)).unwrap_err();
    assert_eq!(error.operation(), BrepCurveOperation3::Evaluation);
    assert_eq!(error.family(), BrepCurveFamily3::RationalBezier);
    assert_eq!(error.kind(), &BrepCurveErrorKind3::ProjectiveDivision);

    let outside = BrepCurve3::from(BrepLineSegment3::new(p(0, 0, 0), p(1, 0, 0)))
        .point_at(&r(2))
        .unwrap_err();
    assert_eq!(outside.kind(), &BrepCurveErrorKind3::ParameterOutsideDomain);
}
