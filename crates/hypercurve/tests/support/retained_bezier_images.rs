use hypercurve::{
    BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, Curve2, CurveCertainty, CurveGeometry2, CurveParameterRange2,
    ExactCurveError, Point2, PredicatePolicy, QuadraticBezier2, RationalBezier2, Real,
    UncertaintyReason,
};

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn root(coefficients: &[i32], start: Real, end: Real) -> BezierAlgebraicParameter2 {
    BezierAlgebraicParameter2::try_isolate(
        BezierParameterPolynomial::try_new_power_basis(
            coefficients.iter().map(|&v| Real::from(v)).collect(),
        )
        .unwrap(),
        BezierParameterInterval::try_new(start, end).unwrap(),
    )
    .unwrap()
}

fn equal(a: &Real, b: &Real) {
    assert_eq!(
        hyperlimit::compare_reals(a, b, PredicatePolicy::STRICT).value(),
        Some(std::cmp::Ordering::Equal),
    );
}

fn scalar(root: &BezierAlgebraicParameter2) -> Option<Real> {
    BezierParameter2::Algebraic(root.clone()).scalar().cloned()
}

pub fn selected_quadratic_image_keeps_the_complete_curve_and_original_roots() {
    let roots = [
        root(&[1, -8, 8], 0.into(), q(1, 2)),
        root(&[1, -8, 8], q(1, 2), 1.into()),
    ];
    assert!(roots.iter().all(|r| scalar(r).is_none()));
    let intervals = roots.each_ref().map(|r| r.interval().clone());
    let source = Curve2::from(QuadraticBezier2::new(
        Point2::from_values(-2, 4),
        Point2::from_values(0, -4),
        Point2::from_values(2, 4),
    ));
    let trim = source
        .restrict_source(
            CurveParameterRange2::try_new(
                BezierParameter2::Algebraic(roots[0].clone()).into(),
                BezierParameter2::Algebraic(roots[1].clone()).into(),
            )
            .unwrap(),
        )
        .unwrap();
    let before = roots.each_ref().map(scalar);
    let radical = Real::from(2).sqrt().unwrap();
    for reversed in [false, true] {
        let curve = if reversed {
            trim.reversed().unwrap()
        } else {
            trim.clone()
        };
        for policy in [
            PredicatePolicy::STRICT,
            PredicatePolicy::APPROXIMATE_512,
            PredicatePolicy::STRICT,
        ] {
            let outcome = hypercurve::evaluate_under(policy, || curve.native_image());
            assert_eq!(outcome.certainty(), CurveCertainty::Certified);
            let image = outcome.into_unverified().unwrap();
            assert!(std::ptr::eq(image.source(), &curve));
            let CurveGeometry2::RationalBezier(native) = image.geometry() else {
                panic!("a source restriction retains its rational evaluator");
            };
            assert_eq!(native.degree(), 2);
            for weight in native.weights() {
                equal(weight, &Real::one());
            }
            for (actual, expected) in native.affine_control_points().unwrap().iter().zip([
                Point2::new(if reversed { radical.clone() } else { -&radical }, 2.into()),
                Point2::from_values(0, -2),
                Point2::new(if reversed { -&radical } else { radical.clone() }, 2.into()),
            ]) {
                equal(actual.x(), expected.x());
                equal(actual.y(), expected.y());
            }
            let native = Curve2::new(image.geometry().clone());
            for i in 0..=8 {
                let u = q(i, 8);
                let x = &radical * (Real::from(2) * &u - Real::one());
                let x = if reversed { -x } else { x };
                let point = native.point_at(&u.into()).unwrap();
                let point = point.coordinates().unwrap();
                equal(point.x(), &x);
                equal(point.y(), &(&x * &x));
            }
            assert!(image.source().parameter_domain().start().scalar().is_none());
            assert!(image.source().parameter_domain().end().scalar().is_none());
        }
    }
    assert_eq!(roots.each_ref().map(scalar), before);
    assert_eq!(roots.each_ref().map(|r| r.interval().clone()), intervals);
}

pub fn nonmonotone_native_image_preserves_its_entire_parameterized_polynomial() {
    let source = RationalBezier2::try_new(
        [0, 1, 0, 0, 1]
            .into_iter()
            .map(|x| Point2::from_values(x, 0))
            .collect(),
        vec![Real::one(); 5],
    )
    .unwrap();
    let curve = Curve2::from(source);
    let trim = curve
        .restrict_source(CurveParameterRange2::try_new(q(1, 4).into(), q(1, 2).into()).unwrap())
        .unwrap();
    assert!(matches!(
        trim.native_segment(),
        Err(ExactCurveError::Blocked(_))
    ));
    let image = trim.native_image().unwrap();
    let CurveGeometry2::RationalBezier(native) = image.geometry() else {
        panic!("the reversing quartic must not become an endpoint line");
    };
    assert_eq!(native.degree(), 4);
    let native = Curve2::new(image.into_geometry());
    for i in 0..=12 {
        let u = q(i, 12);
        let t = q(1, 4) + q(1, 4) * &u;
        let complement = Real::one() - &t;
        let expected =
            Real::from(4) * &t * &complement * &complement * complement + &t * &t * &t * &t;
        let actual = native.point_at(&u.into()).unwrap();
        equal(actual.coordinates().unwrap().x(), &expected);
        equal(actual.coordinates().unwrap().y(), &Real::zero());
    }
}

pub fn unavailable_higher_degree_views_do_not_replace_selected_evidence() {
    let selected = root(&[-1, 0, 0, 2], q(1, 2), 1.into());
    assert!(scalar(&selected).is_none());
    let curve = Curve2::from(QuadraticBezier2::new(
        Point2::from_values(0, 0),
        Point2::from_values(1, 0),
        Point2::from_values(2, 2),
    ));
    let trim = curve
        .restrict_source(
            CurveParameterRange2::try_new(
                BezierParameter2::Algebraic(selected.clone()).into(),
                Real::one().into(),
            )
            .unwrap(),
        )
        .unwrap();
    let before = scalar(&selected);
    assert!(
        matches!(trim.native_image(), Err(ExactCurveError::Blocked(blocker))
        if blocker.reason() == UncertaintyReason::Unsupported)
    );
    assert_eq!(scalar(&selected), before);
    assert!(trim.source_restriction().is_some());
}

pub fn selected_weighted_curve_images_preserve_exact_quotients() {
    let roots = [
        root(&[1, -8, 8], 0.into(), q(1, 2)),
        root(&[1, -8, 8], q(1, 2), 1.into()),
    ];
    let curve = Curve2::from(
        RationalBezier2::try_new(
            [(0, 0), (1, 2), (3, 1), (4, 0)]
                .into_iter()
                .map(|(x, y)| Point2::from_values(x, y))
                .collect(),
            [1, 2, 3, 1].into_iter().map(Real::from).collect(),
        )
        .unwrap(),
    );
    let trim = curve
        .restrict_source(
            CurveParameterRange2::try_new(
                BezierParameter2::Algebraic(roots[0].clone()).into(),
                BezierParameter2::Algebraic(roots[1].clone()).into(),
            )
            .unwrap(),
        )
        .unwrap();
    let radical = Real::from(2).sqrt().unwrap();
    for reversed in [false, true] {
        let source = if reversed {
            trim.reversed().unwrap()
        } else {
            trim.clone()
        };
        let image = source.native_image().unwrap();
        let CurveGeometry2::RationalBezier(native) = image.geometry() else {
            panic!("the complete weighted cubic needs its rational representation");
        };
        assert_eq!(native.degree(), 3);
        let image = Curve2::new(image.into_geometry());
        for i in 0..=6 {
            let u = q(i, 6);
            let t = q(1, 2) + &radical * q(1, 4) * (Real::from(2) * &u - Real::one());
            let t = if reversed { Real::one() - t } else { t };
            let t2 = &t * &t;
            let t3 = &t2 * &t;
            // Independently expanded homogeneous Bernstein numerators.
            let w = Real::one() + Real::from(3) * &t - Real::from(3) * &t3;
            let x =
                ((Real::from(6) * &t + Real::from(15) * &t2 - Real::from(17) * &t3) / &w).unwrap();
            let y =
                ((Real::from(12) * &t - Real::from(15) * &t2 + Real::from(3) * &t3) / w).unwrap();
            let actual = image.point_at(&u.into()).unwrap();
            equal(actual.coordinates().unwrap().x(), &x);
            equal(actual.coordinates().unwrap().y(), &y);
        }
    }
}
