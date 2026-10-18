use hypercurve::{
    BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, BezierParameterRange2, CubicBezier2, Curve2, CurveCertainty,
    CurvePoint2, LineSeg2, Point2, PredicatePolicy, QuadraticBezier2, Real, Similarity2,
};
use std::cmp::Ordering;

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn selected() -> BezierParameter2 {
    BezierParameter2::Algebraic(
        BezierAlgebraicParameter2::try_isolate(
            BezierParameterPolynomial::try_new_power_basis(vec![
                (-1).into(),
                0.into(),
                0.into(),
                2.into(),
            ])
            .unwrap(),
            BezierParameterInterval::try_new(q(3, 4), q(7, 8)).unwrap(),
        )
        .unwrap(),
    )
}

fn polynomial() -> Curve2 {
    CubicBezier2::new(
        Point2::from_values(0, 0),
        Point2::new(q(1, 3), 0.into()),
        Point2::new(q(2, 3), 0.into()),
        Point2::from_values(1, 1),
    )
    .into()
}

fn point() -> CurvePoint2 {
    let parameter = selected();
    assert!(parameter.scalar().is_none());
    let point = polynomial().point_at(&parameter.into()).unwrap();
    assert!(point.coordinates().is_none());
    point
}

pub fn authored_and_constant_forms_preserve_exact_values() {
    let point = CurvePoint2::from(Point2::from_values(2, 3));
    let form = point.affine_form(4.into(), (-2).into(), 5.into());
    assert_eq!(form.compare_to_real(&7.into()).unwrap(), Ordering::Equal);
    assert_eq!(form.bounds().unwrap(), (7.into(), 7.into()));
    assert_eq!(form.source(), &point);
    assert_eq!(
        form.coefficients(),
        &[Real::from(4), Real::from(-2), Real::from(5)]
    );
    let constant = self::point().affine_form(0.into(), 0.into(), Real::pi());
    assert_eq!(
        constant.compare_to_real(&Real::pi()).unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        constant
            .compare(&point.affine_form(0.into(), 0.into(), Real::pi()))
            .unwrap(),
        Ordering::Equal
    );
}

pub fn selected_forms_preserve_root_correlation_and_cancellation() {
    // P(t)=(t,t^3), with 2t^3=1 and 3/4<t<7/8.
    let p = point();
    let form = p.affine_form(0.into(), 2.into(), (-1).into());
    assert_eq!(
        form.compare_to_real(&Real::zero()).unwrap(),
        Ordering::Equal
    );
    let first = p.affine_form(3.into(), 2.into(), (-1).into());
    let second = p.affine_form(3.into(), 0.into(), 0.into());
    assert_eq!(first.compare(&second).unwrap(), Ordering::Equal);
    let tiny: Real = "1e-1000".parse().unwrap();
    assert_eq!(
        p.affine_form(3.into(), 2.into(), Real::from(-1) + tiny)
            .compare(&second)
            .unwrap(),
        Ordering::Greater
    );
    let independent = point();
    assert_eq!(
        first
            .compare(&independent.affine_form(3.into(), 2.into(), (-1).into()))
            .unwrap(),
        Ordering::Equal
    );
    let transformed = polynomial()
        .transform_similarity(
            &Similarity2::try_from_real_affine(
                3.into(),
                4.into(),
                (-4).into(),
                3.into(),
                5.into(),
                7.into(),
            )
            .unwrap(),
        )
        .unwrap()
        .point_at(&selected().into())
        .unwrap();
    // Inverse x=(3*x'-4*y'+13)/25, retaining the same root relation.
    assert_eq!(
        transformed
            .affine_form(q(3, 25), q(-4, 25), q(13, 25))
            .compare(&p.affine_form(1.into(), 0.into(), 0.into()))
            .unwrap(),
        Ordering::Equal
    );
}

pub fn selected_form_bounds_and_policy_reuse_keep_exact_meaning() {
    let p = point();
    let form = p.affine_form((-2).into(), 3.into(), 4.into());
    let (lower, upper) = form.bounds().unwrap();
    assert_ne!(form.compare_to_real(&lower).unwrap(), Ordering::Less);
    assert_ne!(form.compare_to_real(&upper).unwrap(), Ordering::Greater);
    assert_eq!(form.compare_to_real(&q(15, 4)).unwrap(), Ordering::Greater);
    assert_eq!(form.compare_to_real(&4.into()).unwrap(), Ordering::Less);
    for policy in [
        PredicatePolicy::STRICT,
        PredicatePolicy::APPROXIMATE_512,
        PredicatePolicy::STRICT,
    ] {
        let result = hypercurve::evaluate_under(policy, || {
            p.affine_form(0.into(), 2.into(), (-1).into())
                .compare_to_real(&Real::zero())
        });
        assert_eq!(result.certainty(), CurveCertainty::Certified);
        assert_eq!(result.into_unverified().unwrap(), Ordering::Equal);
    }
}

fn parallel(distance: i32) -> Curve2 {
    let parallel = QuadraticBezier2::new(
        Point2::from_values(0, 0),
        Point2::from_values(2, 2),
        Point2::from_values(4, 0),
    )
    .parallel_left(distance.into())
    .unwrap();
    Curve2::try_analytic_parallel(
        parallel,
        BezierParameterRange2::try_new(
            BezierParameter2::Exact(0.into()),
            BezierParameter2::Exact(1.into()),
        )
        .unwrap(),
    )
    .unwrap()
}

pub fn analytic_form_incidence_preserves_the_positive_normal_sheet() {
    let curve = parallel(1);
    let line = Curve2::from(
        LineSeg2::try_new(Point2::from_values(0, 3), Point2::from_values(4, -1)).unwrap(),
    );
    let contact = curve.intersect_curve(&line).unwrap();
    assert!(contact.is_complete());
    assert_eq!(contact.contacts().len(), 1);
    let parameter = contact.contacts()[0].first().parameter().unwrap();
    assert!(parameter.scalar().is_none());
    let point = curve.point_at(&parameter).unwrap();
    assert!(point.coordinates().is_none());
    let form = point.affine_form(1.into(), 1.into(), (-3).into());
    assert_eq!(
        form.compare_to_real(&Real::zero()).unwrap(),
        Ordering::Equal
    );
    // Left/right sums differ by 4t/sqrt(1+(1-2t)^2), positive for 0<t<1.
    let other_sheet = parallel(-1).point_at(&parameter).unwrap();
    assert_eq!(
        other_sheet
            .affine_form(1.into(), 1.into(), (-3).into())
            .compare_to_real(&Real::zero())
            .unwrap(),
        Ordering::Less
    );
    let tiny: Real = "1e-12".parse().unwrap();
    assert_eq!(form.compare_to_real(&tiny).unwrap(), Ordering::Less);
    assert_eq!(form.compare_to_real(&(-tiny)).unwrap(), Ordering::Greater);
}

pub fn symbolic_forms_keep_correlated_coefficients() {
    let point = point();
    let pi = Real::pi();
    let form = point.affine_form(0.into(), pi.clone(), -&pi * q(1, 2));
    assert_eq!(
        form.compare_to_real(&Real::zero()).unwrap(),
        Ordering::Equal
    );
    let surd = Real::from(2).sqrt().unwrap();
    let first = point.affine_form(surd.clone(), pi.clone(), -&pi * q(1, 2));
    let second = point.affine_form(surd, 0.into(), 0.into());
    assert_eq!(first.compare(&second).unwrap(), Ordering::Equal);
    let (lo, hi) = first.bounds().unwrap();
    assert_ne!(first.compare_to_real(&lo).unwrap(), Ordering::Less);
    assert_ne!(first.compare_to_real(&hi).unwrap(), Ordering::Greater);
}

pub fn parallel_injectivity_requires_the_consumed_range() {
    let source = parallel(1);
    assert!(source.has_certified_injective_image().unwrap());
    let (first, second) = source.split_at(selected().into()).unwrap();
    for curve in [first, second, source.reversed().unwrap()] {
        assert!(curve.analytic_parallel().is_some());
        assert!(curve.has_certified_injective_image().unwrap());
    }
    // Nodal cubic: monotone x on [0,1], but P(-1)=P(1)=(0,0).
    let support = CubicBezier2::new(
        Point2::from_values(-1, 0),
        Point2::new((-1).into(), q(-1, 3)),
        Point2::new(q(-2, 3), q(-2, 3)),
        Point2::from_values(0, 0),
    )
    .parallel_left(Real::zero())
    .unwrap();
    let exterior = Curve2::try_analytic_parallel(
        support.clone(),
        BezierParameterRange2::try_new(
            BezierParameter2::Exact((-2).into()),
            BezierParameter2::Exact(2.into()),
        )
        .unwrap(),
    )
    .unwrap();
    let unit = Curve2::try_analytic_parallel(
        support,
        BezierParameterRange2::try_new(
            BezierParameter2::Exact(0.into()),
            BezierParameter2::Exact(1.into()),
        )
        .unwrap(),
    )
    .unwrap();
    assert!(
        exterior
            .point_at(&Real::from(-1).into())
            .unwrap()
            .coincides_with(&exterior.point_at(&Real::one().into()).unwrap())
            .unwrap()
    );
    for policy in [
        PredicatePolicy::STRICT,
        PredicatePolicy::APPROXIMATE_512,
        PredicatePolicy::STRICT,
    ] {
        let result = hypercurve::evaluate_under(policy, || {
            assert!(unit.has_certified_injective_image().unwrap());
            assert!(!exterior.has_certified_injective_image().unwrap());
        });
        assert_eq!(result.certainty(), CurveCertainty::Certified);
    }
}
