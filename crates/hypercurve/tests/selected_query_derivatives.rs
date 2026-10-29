use hypercurve::*;
use hypersolve::{BivariatePolynomial, IsolatedRootInterval};
use std::cmp::Ordering;

fn polynomial(rows: &[&[i32]]) -> BivariatePolynomial {
    BivariatePolynomial::new(
        rows.iter()
            .map(|row| row.iter().copied().map(Real::from).collect())
            .collect(),
    )
}
fn cube_point(n: i32) -> CurvePoint2 {
    CurvePointDefinition2::PolynomialFiber {
        equation: polynomial(&[&[0, 0, 0, 1], &[-1]]),
        parameter: BezierParameter2::Exact(Real::from(n)).into(),
        root: IsolatedRootInterval {
            lower: 1.into(),
            upper: 2.into(),
            exact_root: None,
            distinct_root_count: 1,
        },
    }
    .into_point()
    .unwrap()
}
fn y(point: &CurvePoint2) -> CurvePointRationalForm2 {
    point
        .rational_form(polynomial(&[&[0, 1]]), polynomial(&[&[1]]))
        .unwrap()
}
fn constant(point: &CurvePoint2, n: i32) -> CurvePointRationalForm2 {
    point
        .rational_form(polynomial(&[&[n]]), polynomial(&[&[1]]))
        .unwrap()
}
fn q(a: i32, b: i32) -> Real {
    (Real::from(a) / Real::from(b)).unwrap()
}
fn joint() -> CurvePoint2 {
    y(&cube_point(2))
        .add(&y(&cube_point(3)))
        .unwrap()
        .source()
        .clone()
}
fn assert_relation(point: &CurvePoint2, relation: BivariatePolynomial) {
    assert_eq!(
        point
            .rational_form(relation, polynomial(&[&[1]]))
            .unwrap()
            .compare_to_real(&Real::zero())
            .unwrap(),
        Ordering::Equal
    );
}

fn selected_query_derivative_source() -> (Curve2, CurveParameter2) {
    let point = joint();
    let parameter = point
        .affine_form(q(1, 4), q(1, 4), Real::zero())
        .parameter()
        .unwrap();
    let curve: Curve2 = RationalBezier2::try_new(
        vec![
            Point2::new(0.into(), 0.into()),
            Point2::new(q(1, 2), 0.into()),
            Point2::new(1.into(), 1.into()),
        ],
        vec![Real::one(); 3],
    )
    .unwrap()
    .into();
    (curve, parameter)
}
fn verify_selected_query_parabola_derivatives(curve: &Curve2, parameter: &CurveParameter2) {
    let jet = curve
        .derivatives_at(parameter, 3)
        .expect("generated selected query parameter must feed rational derivatives");
    assert_eq!(jet.len(), 3);
    let first = jet[0].coordinate_image().unwrap();
    assert_eq!(
        first
            .affine_form(Real::one(), Real::zero(), Real::zero())
            .compare_to_real(&Real::one())
            .unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        first
            .affine_form(Real::zero(), q(1, 2), Real::zero())
            .parameter()
            .unwrap()
            .compare(parameter)
            .unwrap(),
        Ordering::Equal
    );
    assert!(
        jet[1]
            .coordinate_image()
            .unwrap()
            .coincides_with(&Point2::new(0.into(), 2.into()).into())
            .unwrap()
    );
    assert!(
        jet[2]
            .coordinate_image()
            .unwrap()
            .coincides_with(&Point2::new(0.into(), 0.into()).into())
            .unwrap()
    );
    let replay = first.definition().unwrap().into_point().unwrap();
    assert!(replay.coincides_with(&first).unwrap());
}
#[test]
fn selected_query_derivative_native_control() {
    let (curve, parameter) = selected_query_derivative_source();
    assert_relation(
        &curve.point_at(&parameter).unwrap(),
        polynomial(&[&[0, -1], &[], &[1]]),
    );
    verify_selected_query_parabola_derivatives(&curve, &CurveParameter2::from(q(2, 3)));
    let outside = parameter.affine_image(&4.into(), &Real::zero()).unwrap();
    assert!(matches!(
        curve.derivative_at(&outside),
        Err(ExactCurveError::Invalid { .. })
    ));
}
#[test]
fn selected_query_derivative_native_parabola() {
    let (curve, parameter) = selected_query_derivative_source();
    verify_selected_query_parabola_derivatives(&curve, &parameter);
}
#[test]
fn selected_query_derivative_retained_parabola() {
    let (curve, parameter) = selected_query_derivative_source();
    let curve = curve
        .restrict_source(
            CurveParameterRange2::try_new(CurveParameter2::from(Real::zero()), parameter.clone())
                .unwrap(),
        )
        .unwrap();
    verify_selected_query_parabola_derivatives(&curve, &parameter);
}
#[test]
fn selected_query_derivative_rational_quotient() {
    let point = joint();
    let parameter = point
        .affine_form(q(1, 4), q(1, 4), Real::zero())
        .parameter()
        .unwrap();
    let curve: Curve2 = RationalBezier2::try_new(
        vec![
            Point2::new(0.into(), 0.into()),
            Point2::new(q(1, 2), 0.into()),
            Point2::new(q(1, 2), q(1, 2)),
        ],
        vec![1.into(), 1.into(), 2.into()],
    )
    .unwrap()
    .into();
    // C(t)=(t,t^2)/(1+t^2): independent quotient-rule expectations.
    let t = point
        .rational_form(polynomial(&[&[0, 1], &[1]]), polynomial(&[&[4]]))
        .unwrap();
    let t2 = t.multiply(&t).unwrap();
    let one = constant(&point, 1);
    let denominator = one.add(&t2).unwrap();
    let denominator2 = denominator.multiply(&denominator).unwrap();
    let dx = one.subtract(&t2).unwrap().divide(&denominator2).unwrap();
    let dy = t.scaled(&2.into()).unwrap().divide(&denominator2).unwrap();
    let derivative = curve
        .derivative_at(&parameter)
        .expect("rational quotient at generated exact parameter")
        .coordinate_image()
        .unwrap();
    for (axis, expected) in [
        (polynomial(&[&[0], &[1]]), dx),
        (polynomial(&[&[0, 1]]), dy),
    ] {
        assert_eq!(
            derivative
                .rational_form(axis, polynomial(&[&[1]]))
                .unwrap()
                .compare(&expected)
                .unwrap(),
            Ordering::Equal
        );
    }
}

fn assert_vector(
    vector: &CurveVector2,
    dx: &CurvePointRationalForm2,
    dy: &CurvePointRationalForm2,
) {
    let image = vector.coordinate_image().unwrap();
    for (axis, expected) in [
        (polynomial(&[&[0], &[1]]), dx),
        (polynomial(&[&[0, 1]]), dy),
    ] {
        assert_eq!(
            image
                .rational_form(axis, polynomial(&[&[1]]))
                .unwrap()
                .compare(expected)
                .unwrap(),
            Ordering::Equal
        );
    }
}
fn power(value: &CurvePointRationalForm2, exponent: usize) -> CurvePointRationalForm2 {
    (0..exponent).fold(constant(value.source(), 1), |result, _| {
        result.multiply(value).unwrap()
    })
}
fn choose(n: usize, k: usize) -> i32 {
    if n < k {
        return 0;
    }
    (1..=k).fold(1, |result, i| result * (n + 1 - i) / i) as i32
}
#[test]
fn selected_query_high_polynomial_jets_differentiate_original_source() {
    let point = cube_point(2);
    let cubic = point
        .rational_form(polynomial(&[&[0, 1]]), polynomial(&[&[2]]))
        .unwrap();
    let joint = joint()
        .rational_form(polynomial(&[&[0, 1], &[1]]), polynomial(&[&[4]]))
        .unwrap();
    // C(t)=(t^3,t^7). In the first field t^3=1/4, but its derivative is not zero.
    let source: Curve2 = RationalBezier2::try_new(
        (0..=7)
            .map(|i| Point2::new(q(choose(i, 3), choose(7, 3)), Real::from(choose(i, 7))))
            .collect(),
        vec![Real::one(); 8],
    )
    .unwrap()
    .into();
    for t in [cubic, joint] {
        let parameter = t.parameter().unwrap();
        let derivatives = source.derivatives_at(&parameter, 9).unwrap();
        assert_eq!(derivatives.len(), 9);
        for (index, derivative) in derivatives.iter().enumerate() {
            let order = index + 1;
            let expected = |degree: usize| {
                if order > degree {
                    return constant(t.source(), 0);
                }
                let factor = (0..order).fold(1, |factor, i| factor * (degree - i) as i32);
                power(&t, degree - order)
                    .scaled(&Real::from(factor))
                    .unwrap()
            };
            assert_vector(derivative, &expected(3), &expected(7));
        }
    }
}
#[test]
fn selected_query_rational_jets_match_closed_form() {
    let point = joint();
    let t = point
        .rational_form(polynomial(&[&[0, 1], &[1]]), polynomial(&[&[4]]))
        .unwrap();
    // C(t)=(t,2t)/(1+t), with k-th derivative (-1)^(k+1)*k!/(1+t)^(k+1).
    let source: Curve2 = RationalBezier2::try_new(
        vec![Point2::from_values(0, 0), Point2::new(q(1, 2), Real::one())],
        vec![1.into(), 2.into()],
    )
    .unwrap()
    .into();
    let denominator = constant(&point, 1).add(&t).unwrap();
    let mut factorial = 1;
    for (i, derivative) in source
        .derivatives_at(&t.parameter().unwrap(), 6)
        .unwrap()
        .iter()
        .enumerate()
    {
        let k = i + 1;
        factorial *= k as i32;
        let dx = constant(&point, if k % 2 == 0 { -factorial } else { factorial })
            .divide(&power(&denominator, k + 1))
            .unwrap();
        assert_vector(derivative, &dx, &dx.scaled(&2.into()).unwrap());
    }
}
#[test]
fn selected_query_spline_jets_keep_source_scale_and_reversal() {
    let point = joint();
    let t = point
        .rational_form(polynomial(&[&[0, 1], &[1]]), polynomial(&[&[4]]))
        .unwrap();
    let parameter = t
        .parameter()
        .unwrap()
        .affine_image(&3.into(), &2.into())
        .unwrap();
    let spline: Curve2 = NurbsCurve2::try_new(
        2,
        vec![
            Point2::from_values(0, 0),
            Point2::new(q(1, 2), Real::zero()),
            Point2::from_values(1, 1),
        ],
        vec![Real::one(); 3],
        vec![2.into(), 2.into(), 2.into(), 5.into(), 5.into(), 5.into()],
    )
    .unwrap()
    .into();
    let restricted = spline
        .restrict_source(
            CurveParameterRange2::try_new(CurveParameter2::from(Real::from(2)), parameter.clone())
                .unwrap(),
        )
        .unwrap();
    let reversed = restricted.reversed().unwrap();
    for curve in [&spline, &restricted, &reversed] {
        let jet = curve.derivatives_at(&parameter, 3).unwrap();
        assert_vector(
            &jet[0],
            &constant(&point, 1).scaled(&q(1, 3)).unwrap(),
            &t.scaled(&q(2, 3)).unwrap(),
        );
        assert_vector(
            &jet[1],
            &constant(&point, 0),
            &constant(&point, 1).scaled(&q(2, 9)).unwrap(),
        );
        assert_vector(&jet[2], &constant(&point, 0), &constant(&point, 0));
    }
}
#[test]
fn selected_query_equal_knot_uses_requested_side() {
    let point = joint();
    let knot = constant(&point, 1).parameter().unwrap();
    let spline: Curve2 = NurbsCurve2::try_new(
        1,
        vec![
            Point2::from_values(0, 0),
            Point2::from_values(1, 0),
            Point2::from_values(1, 2),
        ],
        vec![Real::one(); 3],
        vec![0.into(), 0.into(), 1.into(), 2.into(), 2.into()],
    )
    .unwrap()
    .into();
    for (side, expected) in [
        (CurveParameterSide2::Left, Point2::from_values(1, 0)),
        (CurveParameterSide2::Right, Point2::from_values(0, 2)),
    ] {
        let jet = spline.derivatives_at_side(&knot, 2, side).unwrap();
        assert!(
            jet[0]
                .coordinate_image()
                .unwrap()
                .coincides_with(&expected.into())
                .unwrap()
        );
        assert!(
            jet[1]
                .coordinate_image()
                .unwrap()
                .coincides_with(&Point2::from_values(0, 0).into())
                .unwrap()
        );
    }
    assert!(
        matches!(spline.derivative_at(&knot), Err(ExactCurveError::Blocked(ref blocker)) if blocker.reason() == UncertaintyReason::Boundary)
    );
}
#[test]
fn selected_query_jets_validate_domains_and_retry_budgets() {
    use hypersolve::work_budget::{WorkBudget, with_budget};
    let (curve, parameter) = selected_query_derivative_source();
    assert!(curve.derivatives_at(&parameter, 0).unwrap().is_empty());
    let outside = parameter.affine_image(&4.into(), &Real::zero()).unwrap();
    for order in [0, 3] {
        assert!(matches!(
            curve.derivatives_at(&outside, order),
            Err(ExactCurveError::Invalid { .. })
        ));
    }
    let reused = with_budget(
        WorkBudget {
            refinements: 4096,
            promotions: 0,
        },
        || curve.derivatives_at(&parameter, 3),
    );
    assert_eq!(reused.usage.promotions, 0);
    assert_eq!(reused.result.unwrap().unwrap().len(), 3);
    let refused = with_budget(
        WorkBudget {
            refinements: 0,
            promotions: 4096,
        },
        || {
            let result = curve.derivatives_at(&parameter, 3);
            assert!(
                matches!(result, Err(ExactCurveError::Blocked(ref blocker)) if blocker.reason() == UncertaintyReason::BudgetExhausted)
            );
        },
    );
    let exhausted = refused.result.unwrap_err();
    assert_eq!(
        exhausted.kind,
        hypersolve::work_budget::WorkKind::RootRefinement
    );
    assert_eq!(exhausted.limit, 0);
    let point = joint();
    let half = constant(&point, 1)
        .scaled(&q(1, 2))
        .unwrap()
        .parameter()
        .unwrap();
    let pole: Curve2 = RationalBezier2::try_new(
        vec![Point2::from_values(0, 0), Point2::from_values(1, 1)],
        vec![1.into(), (-1).into()],
    )
    .unwrap()
    .into();
    for order in [0, 3] {
        assert!(matches!(
            pole.derivatives_at(&half, order),
            Err(ExactCurveError::Invalid {
                cause: CurveError::InvalidCurveParameter,
                ..
            })
        ));
    }
    verify_selected_query_parabola_derivatives(&curve, &parameter);
    for policy in [
        PredicatePolicy::STRICT,
        PredicatePolicy::APPROXIMATE_512,
        PredicatePolicy::STRICT,
    ] {
        let result = evaluate_under(policy, || curve.derivatives_at(&parameter, 3));
        assert_eq!(result.certainty(), CurveCertainty::Certified);
        assert_eq!(result.into_unverified().unwrap().len(), 3);
    }
    assert!(
        matches!(curve.derivatives_at(&parameter,usize::MAX),Err(ExactCurveError::Blocked(ref blocker)) if blocker.reason()==UncertaintyReason::Unsupported)
    );
}
#[test]
fn selected_query_derivative_replay_budget_does_not_poison_retry() {
    use hypersolve::work_budget::{WorkBudget, WorkKind, with_budget};
    let (curve, parameter) = selected_query_derivative_source();
    let form = parameter.rational_form_source().unwrap();
    let definition = form.source().definition().unwrap();
    let attempt = || {
        let point = definition.clone().into_point()?;
        let parameter = point
            .rational_form(form.numerator().clone(), form.denominator().clone())?
            .parameter()?;
        curve.derivatives_at(&parameter, 3)
    };
    for _ in 0..2 {
        let refused = with_budget(
            WorkBudget {
                refinements: 4096,
                promotions: 0,
            },
            || {
                let result = attempt();
                assert!(
                    matches!(result, Err(ExactCurveError::Blocked(ref blocker)) if blocker.reason() == UncertaintyReason::BudgetExhausted)
                );
            },
        );
        let exhausted = refused.result.unwrap_err();
        assert_eq!(exhausted.kind, WorkKind::AlgebraicPromotion);
        assert_eq!(exhausted.limit, 0);
        let complete = with_budget(
            WorkBudget {
                refinements: 4096,
                promotions: 1024,
            },
            attempt,
        );
        let jet = complete.result.unwrap().unwrap();
        assert_eq!(jet.len(), 3);
        let first = jet[0].coordinate_image().unwrap();
        assert_eq!(
            first
                .affine_form(Real::zero(), q(1, 2), Real::zero())
                .parameter()
                .unwrap()
                .compare(&parameter)
                .unwrap(),
            Ordering::Equal
        );
        assert!(
            jet[1]
                .coordinate_image()
                .unwrap()
                .coincides_with(&Point2::from_values(0, 2).into())
                .unwrap()
        );
        assert!(
            jet[2]
                .coordinate_image()
                .unwrap()
                .coincides_with(&Point2::from_values(0, 0).into())
                .unwrap()
        );
    }
}
#[cfg(feature = "svg")]
#[test]
fn selected_query_derivatives_replay_cold_and_feed_another_operation() {
    const INPUT: &str = "HYPERCURVE_SELECTED_QUERY_DERIVATIVE_REPLAY";
    fn verify(text: &str) {
        let replay = import_svg_document(text).unwrap();
        let curve = &replay.paths()[0].curves()[0];
        let parameter = curve.parameter_domain().end().clone();
        verify_selected_query_parabola_derivatives(curve, &parameter);
        let vector = replay.paths()[1].curves()[0].start();
        let actual = curve
            .derivative_at(&parameter)
            .unwrap()
            .coordinate_image()
            .unwrap();
        assert!(actual.coincides_with(&vector).unwrap());
        let next_parameter = vector
            .affine_form(Real::zero(), q(1, 2), Real::zero())
            .parameter()
            .unwrap();
        assert_eq!(next_parameter.compare(&parameter).unwrap(), Ordering::Equal);
        let endpoint = curve.point_at(&next_parameter).unwrap();
        assert!(endpoint.coincides_with(&curve.end()).unwrap());
        verify_selected_query_parabola_derivatives(curve, &next_parameter);
        let text2 = export_svg_document(&replay).unwrap();
        let again = import_svg_document(&text2).unwrap();
        verify_selected_query_parabola_derivatives(&again.paths()[0].curves()[0], &next_parameter);
    }
    if let Some(path) = std::env::var_os(INPUT) {
        verify(&std::fs::read_to_string(path).unwrap());
        return;
    }
    let (curve, parameter) = selected_query_derivative_source();
    let vector = curve
        .derivative_at(&parameter)
        .unwrap()
        .coordinate_image()
        .unwrap();
    let end = CurvePointDefinition2::Similarity {
        source: vector.clone(),
        transform: Similarity2::try_from_real_affine(
            1.into(),
            0.into(),
            0.into(),
            1.into(),
            1.into(),
            0.into(),
        )
        .unwrap(),
    }
    .into_point()
    .unwrap();
    let line = Curve2::try_line(vector, end).unwrap();
    let restricted = curve
        .restrict_source(
            CurveParameterRange2::try_new(CurveParameter2::from(Real::zero()), parameter).unwrap(),
        )
        .unwrap();
    let geometry = SvgGeometry2::new(
        CurveRegion2::empty(),
        vec![],
        vec![
            CurvePath2::try_new(vec![restricted]).unwrap(),
            CurvePath2::try_new(vec![line]).unwrap(),
        ],
    );
    let text = export_svg_document(&geometry).unwrap();
    verify(&text);
    let path = std::env::temp_dir().join(format!(
        "query-derivative-{}-{}.svg",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    ));
    std::fs::write(&path, text).unwrap();
    let child = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "selected_query_derivatives_replay_cold_and_feed_another_operation",
            "--nocapture",
        ])
        .env(INPUT, &path)
        .output()
        .unwrap();
    std::fs::remove_file(path).unwrap();
    assert!(
        child.status.success(),
        "{}{}",
        String::from_utf8_lossy(&child.stdout),
        String::from_utf8_lossy(&child.stderr)
    );
}

#[test]
fn selected_query_derivatives_keep_similarity_and_retained_chart() {
    let (source, parameter) = selected_query_derivative_source();
    let t = parameter.rational_form_source().unwrap();
    let point = t.source();
    let transform = Similarity2::try_from_real_affine(
        0.into(),
        (-2).into(),
        2.into(),
        0.into(),
        7.into(),
        (-5).into(),
    )
    .unwrap();
    let retained = source
        .restrict_source(
            CurveParameterRange2::try_new(CurveParameter2::from(Real::zero()), parameter.clone())
                .unwrap(),
        )
        .unwrap()
        .reversed()
        .unwrap();
    for original in [source, retained] {
        let curve = original.transform_similarity(&transform).unwrap();
        let jet = curve.derivatives_at(&parameter, 3).unwrap();
        assert_vector(
            &jet[0],
            &t.scaled(&(-4).into()).unwrap(),
            &constant(point, 2),
        );
        assert_vector(&jet[1], &constant(point, -4), &constant(point, 0));
        assert_vector(&jet[2], &constant(point, 0), &constant(point, 0));
    }
}
