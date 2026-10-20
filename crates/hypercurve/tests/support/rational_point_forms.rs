use hypercurve::*;
use hypersolve::{BivariatePolynomial, IsolatedRootInterval};
use std::cmp::Ordering;

fn q(a: i32, b: i32) -> Real {
    (Real::from(a) / Real::from(b)).unwrap()
}

fn polynomial(rows: &[&[i32]]) -> BivariatePolynomial {
    BivariatePolynomial::new(
        rows.iter()
            .map(|row| row.iter().copied().map(Real::from).collect())
            .collect(),
    )
}

pub fn joint_uv() -> CurvePoint2 {
    // u = sqrt(2)/4, v = 2^(1/6)/4. Both share 16v³ = u.
    CurvePointDefinition2::PolynomialFiber {
        equation: polynomial(&[&[0, 0, 0, 16], &[-1]]),
        parameter: BezierParameter2::Algebraic(
            BezierAlgebraicParameter2::try_isolate(
                BezierParameterPolynomial::try_new_power_basis(vec![q(-1, 8), 0.into(), 1.into()])
                    .unwrap(),
                BezierParameterInterval::try_new(q(1, 3), q(1, 2)).unwrap(),
            )
            .unwrap(),
        ),
        root: IsolatedRootInterval {
            lower: q(1, 4),
            upper: q(3, 8),
            exact_root: None,
            distinct_root_count: 1,
        },
    }
    .into_point()
    .unwrap()
}

pub fn joint_identities_and_original_poles() {
    let point = joint_uv();
    assert!(point.coordinates().is_none());
    let form = point
        .rational_form(
            polynomial(&[&[0, 0, 0, 16], &[1]]),
            polynomial(&[&[], &[2]]),
        )
        .unwrap();
    assert_eq!(form.source(), &point);
    assert_eq!(form.compare_to_real(&1.into()).unwrap(), Ordering::Equal);
    let negative = point
        .rational_form(
            polynomial(&[&[0, 0, 0, 16], &[1]]),
            polynomial(&[&[], &[-2]]),
        )
        .unwrap();
    assert_eq!(
        negative.compare_to_real(&(-1).into()).unwrap(),
        Ordering::Equal
    );
    assert_eq!(negative.compare(&form).unwrap(), Ordering::Less);
    let square = point
        .rational_form(polynomial(&[&[], &[], &[8]]), polynomial(&[&[1]]))
        .unwrap();
    assert_eq!(square.compare(&form).unwrap(), Ordering::Equal);
    let pole = polynomial(&[&[-1], &[], &[8]]);
    assert!(
        matches!(
            point.rational_form(pole.clone(), pole),
            Err(ExactCurveError::Invalid {
                cause: CurveError::ZeroRationalPointDenominator,
                ..
            })
        ),
        "algebraic 0/0 cannot cancel to one"
    );
    assert!(matches!(
        point.rational_form(polynomial(&[&[]]), polynomial(&[&[0, 0, 0, 16], &[-1]])),
        Err(ExactCurveError::Invalid {
            cause: CurveError::ZeroRationalPointDenominator,
            ..
        })
    ));
    let tiny = (Real::one() / Real::from(1_000_000_000_000_i64)).unwrap();
    let shifted = point
        .rational_form(
            BivariatePolynomial::new(vec![vec![tiny], vec![], vec![8.into()]]),
            polynomial(&[&[1]]),
        )
        .unwrap();
    assert_eq!(shifted.compare(&form).unwrap(), Ordering::Greater);
    let (lo, hi) = form.bounds().unwrap();
    assert_ne!(
        CurveParameter2::from(lo)
            .compare(&Real::one().into())
            .unwrap(),
        Ordering::Greater
    );
    assert_ne!(
        CurveParameter2::from(hi)
            .compare(&Real::one().into())
            .unwrap(),
        Ordering::Less
    );
}

pub fn rational_queries_preserve_similarity_and_cold_sources() {
    let point = joint_uv();
    let original = point
        .rational_form(polynomial(&[&[0, 0, 25], &[], &[25]]), polynomial(&[&[1]]))
        .unwrap();
    let transform = Similarity2::try_from_real_affine(
        3.into(),
        (-4).into(),
        4.into(),
        3.into(),
        2.into(),
        (-1).into(),
    )
    .unwrap();
    let line = Curve2::try_line(point.clone(), Point2::from_values(1, 1).into()).unwrap();
    let mapped = line.transform_similarity(&transform).unwrap();
    // (x-2)^2 + (y+1)^2 = 25(u²+v²).
    let query = |point: &CurvePoint2| {
        point
            .rational_form(polynomial(&[&[5, 2, 1], &[-4], &[1]]), polynomial(&[&[1]]))
            .unwrap()
    };
    let mapped_form = query(&mapped.start());
    assert_eq!(mapped_form.compare(&original).unwrap(), Ordering::Equal);
    let restored = mapped.start().definition().unwrap().into_point().unwrap();
    assert_eq!(
        query(&restored).compare(&original).unwrap(),
        Ordering::Equal
    );
    let (lower, upper) = mapped_form.bounds().unwrap();
    assert_ne!(mapped_form.compare_to_real(&lower).unwrap(), Ordering::Less);
    assert_ne!(
        mapped_form.compare_to_real(&upper).unwrap(),
        Ordering::Greater
    );
    let other = joint_uv();
    let independent = other
        .rational_form(polynomial(&[&[0, 0, 25], &[], &[25]]), polynomial(&[&[1]]))
        .unwrap();
    assert_eq!(independent.compare(&original).unwrap(), Ordering::Equal);
}

pub fn rational_curve_source_and_authored_values() {
    let selected = BezierAlgebraicParameter2::try_isolate(
        BezierParameterPolynomial::try_new_power_basis(vec![
            q(-1, 2),
            0.into(),
            0.into(),
            1.into(),
        ])
        .unwrap(),
        BezierParameterInterval::try_new(q(3, 4), q(7, 8)).unwrap(),
    )
    .unwrap();
    let curve = Curve2::from(
        RationalBezier2::try_new(
            vec![
                Point2::from_values(0, 0),
                Point2::new(q(1, 2), 0.into()),
                Point2::from_values(1, 1),
            ],
            vec![1.into(); 3],
        )
        .unwrap(),
    );
    let point = curve
        .point_at(&BezierParameter2::Algebraic(selected).into())
        .unwrap();
    assert!(point.coordinates().is_none());
    let residual = point
        .rational_form(
            polynomial(&[&[0, 1], &[], &[-1]]),
            polynomial(&[&[1], &[1]]),
        )
        .unwrap();
    assert_eq!(
        residual.compare_to_real(&0.into()).unwrap(),
        Ordering::Equal
    );
    let constant = point
        .rational_form(polynomial(&[&[], &[], &[], &[2]]), polynomial(&[&[1]]))
        .unwrap();
    assert_eq!(
        constant.compare_to_real(&1.into()).unwrap(),
        Ordering::Equal
    );
    let authored: CurvePoint2 = Point2::from_values(2, 3).into();
    let identity = authored
        .rational_form(polynomial(&[&[0], &[3]]), polynomial(&[&[0, 2]]))
        .unwrap();
    assert_eq!(identity.compare(&constant).unwrap(), Ordering::Equal);
    assert!(matches!(
        point.rational_form(polynomial(&[&[1]]), polynomial(&[&[-1], &[], &[], &[2]])),
        Err(ExactCurveError::Invalid {
            cause: CurveError::ZeroRationalPointDenominator,
            ..
        })
    ));
    residual.bounds().unwrap();
}

pub fn scalar_chart_images_keep_their_joint_source() {
    let point = joint_uv();
    let beta = point
        .affine_form(0.into(), 1.into(), 0.into())
        .parameter()
        .unwrap();
    let chart = beta
        .affine_point(0.into(), q(1, 2), 1.into(), 0.into())
        .unwrap();
    let original = point
        .rational_form(polynomial(&[&[0, 0, 0, 16]]), polynomial(&[&[1]]))
        .unwrap();
    let image = chart
        .rational_form(polynomial(&[&[0, 0, 0, 16]]), polynomial(&[&[1]]))
        .unwrap();
    assert_eq!(image.compare(&original).unwrap(), Ordering::Equal);
    assert_eq!(image.compare_to_real(&q(1, 3)).unwrap(), Ordering::Greater);
    image.bounds().unwrap();
    let restored = chart.definition().unwrap().into_point().unwrap();
    assert_eq!(
        restored
            .rational_form(polynomial(&[&[0, 0, 0, 16]]), polynomial(&[&[1]]))
            .unwrap()
            .compare(&original)
            .unwrap(),
        Ordering::Equal
    );
}

pub fn rational_queries_recover_after_budget_refusal() {
    let point = joint_uv();
    let query = || {
        point
            .rational_form(polynomial(&[&[0, 1], &[1]]), polynomial(&[&[1]]))
            .unwrap()
    };
    for policy in [
        PredicatePolicy::STRICT,
        PredicatePolicy::APPROXIMATE_512,
        PredicatePolicy::STRICT,
    ] {
        let outcome = evaluate_under(policy, || {
            let form = query();
            assert_eq!(form.compare_to_real(&q(1, 2)).unwrap(), Ordering::Greater);
            form.bounds().unwrap();
        });
        assert_eq!(outcome.certainty(), CurveCertainty::Certified);
    }
    let form = query();
    let denied = hypersolve::work_budget::with_budget(
        hypersolve::work_budget::WorkBudget {
            refinements: 4096,
            promotions: 0,
        },
        || {
            let error = form.bounds().unwrap_err();
            assert!(
                matches!(error,ExactCurveError::Blocked(blocker) if blocker.reason()==UncertaintyReason::BudgetExhausted)
            );
        },
    );
    assert!(denied.result.is_err());
    form.bounds().unwrap();
    assert_eq!(form.compare_to_real(&q(1, 2)).unwrap(), Ordering::Greater);
}

pub fn independent_support_fields_keep_rational_identities() {
    let root = |degree: usize| -> CurveParameter2 {
        let mut coefficients = vec![Real::zero(); degree + 1];
        coefficients[0] = (-1).into();
        coefficients[degree] = 2.into();
        BezierParameter2::Algebraic(
            BezierAlgebraicParameter2::try_isolate(
                BezierParameterPolynomial::try_new_power_basis(coefficients).unwrap(),
                BezierParameterInterval::try_new(q(3, 4), q(7, 8)).unwrap(),
            )
            .unwrap(),
        )
        .into()
    };
    let (x, y) = (root(3), root(5));
    let point = CurvePointDefinition2::SupportingLineIntersection {
        first: [
            x.affine_point(1.into(), 0.into(), 0.into(), 0.into())
                .unwrap(),
            x.affine_point(1.into(), 0.into(), 0.into(), 2.into())
                .unwrap(),
        ],
        second: [
            y.affine_point(0.into(), 0.into(), 1.into(), 0.into())
                .unwrap(),
            y.affine_point(0.into(), 2.into(), 1.into(), 0.into())
                .unwrap(),
        ],
    }
    .into_point()
    .unwrap();
    assert!(matches!(
        point.definition().unwrap(),
        CurvePointDefinition2::SupportingLineIntersection { .. }
    ));
    for point in [
        point.clone(),
        point.definition().unwrap().into_point().unwrap(),
    ] {
        // x^3 = y^5 = 1/2; a shared composite field must preserve both equations.
        let numerator = polynomial(&[&[0, 0, 0, 0, 0, 2], &[], &[], &[2]]);
        let denominator = polynomial(&[&[1]]);
        let form = point.rational_form(numerator, denominator).unwrap();
        assert_eq!(form.compare_to_real(&2.into()).unwrap(), Ordering::Equal);
        form.bounds().unwrap();
        assert!(matches!(
            point.rational_form(polynomial(&[&[1]]), polynomial(&[&[-1], &[], &[], &[2]])),
            Err(ExactCurveError::Invalid {
                cause: CurveError::ZeroRationalPointDenominator,
                ..
            })
        ));
    }
}
