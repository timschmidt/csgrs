use hypercurve::*;
use hypersolve::{BivariatePolynomial, IsolatedRootInterval, work_budget};
use std::cmp::Ordering;

fn root(coefficients: Vec<Real>, lower: i32, upper: i32) -> BezierParameter2 {
    BezierParameter2::Algebraic(
        BezierAlgebraicParameter2::try_isolate(
            BezierParameterPolynomial::try_new_power_basis(coefficients).unwrap(),
            BezierParameterInterval::try_new(lower.into(), upper.into()).unwrap(),
        )
        .unwrap(),
    )
}
pub fn source() -> CurvePoint2 {
    CurvePointDefinition2::PolynomialFiber {
        equation: BivariatePolynomial::new(vec![
            vec![0.into(), 0.into(), 0.into(), 1.into()],
            vec![(-1).into()],
        ]),
        parameter: root(vec![(-2).into(), 0.into(), 1.into()], 1, 2).into(),
        root: IsolatedRootInterval {
            lower: 1.into(),
            upper: (Real::from(3) / Real::from(2)).unwrap(),
            exact_root: None,
            distinct_root_count: 1,
        },
    }
    .into_point()
    .unwrap()
}

pub fn mixed_affine_parameters_replay_and_compose() {
    let point = source();
    // From beta^3=alpha and alpha^2=2: z=alpha+beta satisfies
    // z^3+6z=3alpha(z^2+1), hence z^6-6z^4-18=0, z in (2,3).
    let expected: CurveParameter2 = root(
        vec![
            (-18).into(),
            0.into(),
            0.into(),
            0.into(),
            (-6).into(),
            0.into(),
            1.into(),
        ],
        2,
        3,
    )
    .into();
    let value = point
        .affine_form(1.into(), 1.into(), 0.into())
        .parameter()
        .unwrap();
    assert!(value.scalar().is_none());
    assert!(value.as_bezier_parameter().is_none());
    let form = value
        .affine_form_source()
        .expect("retain the source and recipe");
    assert_eq!(form.source(), &point);
    assert_eq!(value.compare(&expected).unwrap(), Ordering::Equal);
    assert_eq!(
        form.parameter().unwrap().compare(&value).unwrap(),
        Ordering::Equal
    );
    let mapped = value.affine_image(&(-2).into(), &7.into()).unwrap();
    let inverse = mapped
        .affine_image(
            &(Real::from(-1) / Real::from(2)).unwrap(),
            &(Real::from(7) / Real::from(2)).unwrap(),
        )
        .unwrap();
    assert_eq!(inverse.compare(&expected).unwrap(), Ordering::Equal);
    assert_eq!(inverse.affine_form_source().unwrap().source(), &point);
    let tiny = (Real::one() / Real::from(1_000_000_000_000_i64)).unwrap();
    assert_eq!(
        value
            .affine_image(&Real::one(), &tiny)
            .unwrap()
            .compare(&value)
            .unwrap(),
        Ordering::Greater
    );
    let image = value
        .affine_point(2.into(), 1.into(), (-3).into(), 4.into())
        .unwrap();
    let recover = image
        .affine_form(
            (Real::one() / Real::from(2)).unwrap(),
            0.into(),
            (Real::from(-1) / Real::from(2)).unwrap(),
        )
        .parameter()
        .unwrap();
    assert_eq!(recover.compare(&value).unwrap(), Ordering::Equal);
    image.bounds().unwrap();
    // Zero and pure-coordinate forms retain their existing exact meanings.
    assert_eq!(
        point
            .affine_form(0.into(), 0.into(), 7.into())
            .parameter()
            .unwrap()
            .scalar(),
        Some(&Real::from(7))
    );
    assert!(
        point
            .affine_form(0.into(), 1.into(), 0.into())
            .parameter()
            .unwrap()
            .affine_form_source()
            .is_some()
    );
}

pub fn conjugate_selection_similarity_and_policy_recovery() {
    let original = source();
    let transform = Similarity2::try_from_real_affine(
        3.into(),
        (-4).into(),
        4.into(),
        3.into(),
        2.into(),
        (-1).into(),
    )
    .unwrap();
    let line = Curve2::try_line(original.clone(), Point2::from_values(2, 2).into()).unwrap();
    let image = line.transform_similarity(&transform).unwrap();
    for (a, b, c) in [(3, -4, 2), (4, 3, -1), (-3, 4, -2)] {
        let direct = original
            .affine_form(a.into(), b.into(), c.into())
            .parameter()
            .unwrap();
        let canonical = direct.affine_form_source().unwrap();
        assert_eq!(canonical.source(), &original);
        assert_eq!(
            direct.compare(&canonical.parameter().unwrap()).unwrap(),
            Ordering::Equal
        );
    }
    let transformed = image
        .start()
        .affine_form(1.into(), 0.into(), 0.into())
        .parameter()
        .unwrap();
    let expected = original
        .affine_form(3.into(), (-4).into(), 2.into())
        .parameter()
        .unwrap();
    assert_eq!(transformed.compare(&expected).unwrap(), Ordering::Equal);
    assert_eq!(
        transformed.affine_form_source().unwrap().source(),
        &original
    );
    let restored = image.start().definition().unwrap().into_point().unwrap();
    assert_eq!(
        restored
            .affine_form(1.into(), 0.into(), 0.into())
            .parameter()
            .unwrap()
            .compare(&expected)
            .unwrap(),
        Ordering::Equal
    );
    for policy in [
        PredicatePolicy::STRICT,
        PredicatePolicy::APPROXIMATE_512,
        PredicatePolicy::STRICT,
    ] {
        let result = evaluate_under(policy, || {
            original
                .affine_form(1.into(), 1.into(), 0.into())
                .parameter()
                .unwrap()
        });
        assert_eq!(result.certainty(), CurveCertainty::Certified);
        assert!(result.certified().unwrap().affine_form_source().is_some());
    }
    let denied = work_budget::with_budget(
        work_budget::WorkBudget {
            refinements: 4096,
            promotions: 0,
        },
        || {
            let error = original
                .affine_form(1.into(), 1.into(), 0.into())
                .parameter()
                .unwrap_err();
            assert!(
                matches!(error, ExactCurveError::Blocked(blocker) if blocker.reason() == UncertaintyReason::BudgetExhausted)
            );
        },
    );
    assert!(denied.result.is_err());
    original
        .affine_form(1.into(), 1.into(), 0.into())
        .parameter()
        .unwrap();
}
