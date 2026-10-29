use hypercurve::*;
use hypersolve::bivariate_topology::{
    BivariateCurveDomain, BivariateTopologyConfig, decompose_bivariate_curve,
};
use hypersolve::{AlgebraicFiberRootIsolationConfig, BivariatePolynomial, IsolatedRootInterval};
use std::cmp::Ordering;

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}
fn p(rows: &[&[i32]]) -> BivariatePolynomial {
    BivariatePolynomial::new(
        rows.iter()
            .map(|row| row.iter().copied().map(Real::from).collect())
            .collect(),
    )
}
fn x() -> BivariatePolynomial {
    p(&[&[], &[1]])
}
fn y() -> BivariatePolynomial {
    p(&[&[0, 1]])
}
fn form(point: &CurvePoint2, numerator: BivariatePolynomial) -> CurvePointRationalForm2 {
    point.rational_form(numerator, p(&[&[1]])).unwrap()
}
fn zero(point: &CurvePoint2, equation: BivariatePolynomial) {
    assert_eq!(
        form(point, equation)
            .compare_to_real(&Real::zero())
            .unwrap(),
        Ordering::Equal
    );
}
fn source() -> CurvePoint2 {
    let parameter = BezierParameter2::Algebraic(
        BezierAlgebraicParameter2::try_isolate(
            BezierParameterPolynomial::try_new_power_basis(vec![q(-1, 8), 0.into(), 1.into()])
                .unwrap(),
            BezierParameterInterval::try_new(q(1, 3), q(1, 2)).unwrap(),
        )
        .unwrap(),
    );
    CurvePointDefinition2::PolynomialFiber {
        equation: p(&[&[0, 0, 0, 16], &[-1]]),
        parameter: parameter.into(),
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
fn curves(equation: BivariatePolynomial) -> PolynomialCurveSet2 {
    curves_with_config(equation, AlgebraicFiberRootIsolationConfig::default())
}
fn curves_with_config(
    equation: BivariatePolynomial,
    config: AlgebraicFiberRootIsolationConfig,
) -> PolynomialCurveSet2 {
    let topology = decompose_bivariate_curve(
        &equation,
        &BivariateCurveDomain {
            first: [(-1).into(), 1.into()],
            second: [(-2).into(), 2.into()],
        },
        BivariateTopologyConfig::default(),
    )
    .unwrap();
    PolynomialCurveSet2::from_topology(topology, config).unwrap()
}

#[test]
fn selected_fiber_abscissa_has_independent_exact_relations() {
    let source = source();
    zero(&source, p(&[&[-1], &[], &[8]]));
    zero(&source, p(&[&[0, 0, 0, 16], &[-1]]));
    let value = form(&source, y());
    assert_eq!(value.compare_to_real(&q(1, 4)).unwrap(), Ordering::Greater);
    assert_eq!(value.compare_to_real(&q(3, 8)).unwrap(), Ordering::Less);
    let parameter = value.parameter().unwrap();
    assert!(parameter.as_bezier_parameter().is_none());
    assert!(parameter.rational_form_source().is_some());
}

#[test]
fn polynomial_branch_consumes_a_nested_selected_fiber_query() {
    let source = source();
    let value = form(&source, y());
    let parameter = value.parameter().unwrap();
    let set = curves(p(&[&[0, 0, 1], &[-1]]));
    assert_eq!(set.curves().len(), 2);
    let branch = &set.curves()[1];
    let point = branch
        .point_at(&parameter)
        .expect("the positive square-root branch must consume the selected cubic fiber query");
    zero(&point, p(&[&[0, 0, 1], &[-1]]));
    zero(&point, p(&[&[-1], &[], &[], &[], &[], &[], &[2048]]));
    assert_eq!(form(&point, x()).compare(&value).unwrap(), Ordering::Equal);
    assert_eq!(
        form(&point, y()).compare_to_real(&Real::zero()).unwrap(),
        Ordering::Greater
    );
    let CurvePointLocations2::Locations(locations) = branch.point_locations(&point).unwrap() else {
        panic!("isolated graph preimage")
    };
    assert_eq!(locations.len(), 1);
    assert_eq!(
        locations[0]
            .parameter()
            .unwrap()
            .compare(&parameter)
            .unwrap(),
        Ordering::Equal
    );
    let replay = point.definition().unwrap().into_point().unwrap();
    assert!(replay.coincides_with(&point).unwrap());
    let next = curves(p(&[&[0, 1], &[], &[-1]]));
    let ordinate = form(&replay, y()).parameter().unwrap();
    let next_point = next.curves()[0].point_at(&ordinate).unwrap();
    assert_eq!(
        form(&next_point, y()).compare(&value).unwrap(),
        Ordering::Equal
    );
    zero(&next_point, p(&[&[0, 1], &[], &[-1]]));
}

#[test]
fn selected_query_fiber_respects_the_configured_subdivision_limit() {
    let parameter = form(&source(), y()).parameter().unwrap();
    let equation = p(&[&[0, 0, 1], &[-1]]);
    let limited = curves_with_config(
        equation.clone(),
        AlgebraicFiberRootIsolationConfig {
            max_subdivision_depth: 0,
            refinement_steps: 8,
        },
    );
    let error = limited.curves()[1]
        .point_at(&parameter)
        .expect_err("a zero subdivision allowance must not publish a completed fiber");
    assert!(
        matches!(error, ExactCurveError::Blocked(blocker) if blocker.reason() == UncertaintyReason::BudgetExhausted)
    );
    let retry = curves(equation.clone());
    zero(&retry.curves()[1].point_at(&parameter).unwrap(), equation);
}

#[test]
fn selected_query_fibers_keep_opposite_roots_repeated_equations_and_exact_events() {
    let parameter = form(&source(), y()).parameter().unwrap();
    let repeated = p(&[&[0, 0, 0, 0, 1], &[0, 0, -2], &[1]]);
    let make = |equation, lower, upper, exact| {
        CurvePointDefinition2::PolynomialFiber {
            equation,
            parameter: parameter.clone(),
            root: IsolatedRootInterval {
                lower,
                upper,
                exact_root: exact,
                distinct_root_count: 99,
            },
        }
        .into_point()
    };
    let positive = make(repeated.clone(), q(1, 2), q(3, 4), None).unwrap();
    let negative = make(repeated.clone(), q(-3, 4), q(-1, 2), None).unwrap();
    for point in [&positive, &negative] {
        zero(point, repeated.clone());
        zero(point, p(&[&[0, 0, 1], &[-1]]));
    }
    assert!(!positive.coincides_with(&negative).unwrap());
    assert_eq!(
        form(&positive, y())
            .add(&form(&negative, y()))
            .unwrap()
            .compare_to_real(&Real::zero())
            .unwrap(),
        Ordering::Equal
    );
    let exact = make(p(&[&[-1, 2]]), q(1, 2), q(1, 2), Some(q(1, 2))).unwrap();
    assert_eq!(
        form(&exact, y()).compare_to_real(&q(1, 2)).unwrap(),
        Ordering::Equal
    );
    for result in [
        make(repeated, (-1).into(), 1.into(), None),
        make(p(&[&[1, 0, 1]]), 0.into(), 1.into(), None),
        make(p(&[&[0]]), 0.into(), 0.into(), Some(0.into())),
        make(p(&[&[-1, 2]]), 0.into(), 0.into(), Some(0.into())),
        make(p(&[&[-1, 2]]), 0.into(), 1.into(), Some(q(1, 2))),
        make(p(&[&[-1, 2]]), 1.into(), 0.into(), None),
    ] {
        assert!(matches!(result, Err(ExactCurveError::Invalid { .. })));
    }
}

#[test]
fn selected_query_fibers_admit_degree_seventeen_and_feed_rational_geometry() {
    let parameter = form(&source(), y()).parameter().unwrap();
    let mut row = vec![Real::zero(); 18];
    row[17] = Real::one();
    let equation = BivariatePolynomial::new(vec![row, vec![Real::from(-1)]]);
    let point = CurvePointDefinition2::PolynomialFiber {
        equation: equation.clone(),
        parameter,
        root: IsolatedRootInterval {
            lower: 0.into(),
            upper: 1.into(),
            exact_root: None,
            distinct_root_count: 1,
        },
    }
    .into_point()
    .unwrap();
    zero(&point, equation);
    let root = form(&point, y()).parameter().unwrap();
    let next = root
        .affine_point(2.into(), 1.into(), 3.into(), (-1).into())
        .unwrap();
    // X=1+2r, Y=3r-1, so 3X-2Y=5 independently of r's degree.
    zero(&next, p(&[&[-5, -2], &[3]]));
    assert_eq!(
        next.affine_form(q(1, 2), 0.into(), q(-1, 2))
            .parameter()
            .unwrap()
            .compare(&root)
            .unwrap(),
        Ordering::Equal
    );
    assert!(
        point
            .definition()
            .unwrap()
            .into_point()
            .unwrap()
            .coincides_with(&point)
            .unwrap()
    );
}

#[test]
fn selected_query_fibers_preserve_trim_reversal_transform_and_certainty() {
    let source = source();
    // A quotient consumes both coordinates of the retained cubic fiber.
    let query = source
        .rational_form(p(&[&[0, 1], &[1]]), p(&[&[2], &[1]]))
        .unwrap();
    let parameter = query.parameter().unwrap();
    let equation = p(&[&[0, 0, 1], &[-1]]);
    let set = curves(equation.clone());
    let branch = &set.curves()[1];
    for policy in [
        PredicatePolicy::STRICT,
        PredicatePolicy::APPROXIMATE_512,
        PredicatePolicy::STRICT,
    ] {
        let outcome = evaluate_under(policy, || {
            let point = branch.point_at(&parameter).unwrap();
            zero(&point, equation.clone());
            assert_eq!(form(&point, x()).compare(&query).unwrap(), Ordering::Equal);
            let restricted = branch
                .restrict_source(
                    CurveParameterRange2::try_new(parameter.clone(), q(3, 4).into()).unwrap(),
                )
                .unwrap();
            assert!(restricted.start().coincides_with(&point).unwrap());
            assert!(
                restricted
                    .reversed()
                    .unwrap()
                    .end()
                    .coincides_with(&point)
                    .unwrap()
            );
            let negative = set.curves()[0].point_at(&parameter).unwrap();
            let CurvePointLocations2::Locations(locations) =
                branch.point_locations(&negative).unwrap()
            else {
                panic!("finite preimages")
            };
            assert!(locations.is_empty());
            let transform = Similarity2::try_from_real_affine(
                0.into(),
                (-2).into(),
                2.into(),
                0.into(),
                1.into(),
                3.into(),
            )
            .unwrap();
            let transformed = CurvePointDefinition2::Similarity {
                source: point.clone(),
                transform,
            }
            .into_point()
            .unwrap();
            assert_eq!(
                transformed
                    .affine_form(0.into(), q(1, 2), q(-3, 2))
                    .parameter()
                    .unwrap()
                    .compare(&parameter)
                    .unwrap(),
                Ordering::Equal
            );
            let gap = (Real::one() / Real::from(1_000_000_000_000_u64)).unwrap();
            let nearby_parameter = parameter.affine_image(&Real::one(), &gap).unwrap();
            let nearby = branch.point_at(&nearby_parameter).unwrap();
            assert!(!nearby.coincides_with(&point).unwrap());
            assert_eq!(
                form(&nearby, x())
                    .subtract(&form(&point, x()))
                    .unwrap()
                    .compare_to_real(&gap)
                    .unwrap(),
                Ordering::Equal
            );
            zero(&nearby, equation.clone());
            let moved = point
                .rational_image(&RationalMap2::new([p(&[&[1], &[1]]), y()], p(&[&[1]])))
                .unwrap();
            assert!(!moved.coincides_with(&point).unwrap());
            assert_eq!(
                form(&moved, p(&[&[-1], &[1]])).compare(&query).unwrap(),
                Ordering::Equal
            );
        });
        assert_eq!(outcome.certainty(), CurveCertainty::Certified);
    }
}

#[test]
fn selected_query_fibers_share_work_limits_and_retry_after_exhaustion() {
    use hypersolve::work_budget::{WorkBudget, with_budget};
    let parameter = form(&source(), y()).parameter().unwrap();
    let equation = p(&[&[0, 0, 1], &[-1]]);
    let set = curves(equation.clone());
    let limited = with_budget(WorkBudget::ZERO, || {
        let result = set.curves()[1].point_at(&parameter);
        assert!(
            matches!(result, Err(ExactCurveError::Blocked(blocker)) if blocker.reason() == UncertaintyReason::BudgetExhausted)
        );
    });
    assert!(limited.result.is_err());
    let retry = set.curves()[1].point_at(&parameter).unwrap();
    zero(&retry, equation);
}

#[test]
fn selected_query_fibers_consume_mixed_authored_and_generated_sources() {
    let generated = form(&source(), y());
    let authored: CurvePoint2 = Point2::new(q(1, 3).sqrt().unwrap(), Real::zero()).into();
    let term = form(&authored, x()).scaled(&q(1, 4)).unwrap();
    let mixed = generated.add(&term).unwrap();
    assert!(mixed.operands().is_some());
    let parameter = mixed.parameter().unwrap();
    let set = curves(p(&[&[0, 0, 1], &[-1]]));
    let point = set.curves()[1].point_at(&parameter).unwrap();
    zero(&point, p(&[&[0, 0, 1], &[-1]]));
    assert_eq!(form(&point, x()).compare(&mixed).unwrap(), Ordering::Equal);
    let remainder = form(&point, x()).subtract(&generated).unwrap();
    assert_eq!(
        remainder
            .multiply(&remainder)
            .unwrap()
            .compare_to_real(&q(1, 48))
            .unwrap(),
        Ordering::Equal
    );
    assert!(
        point
            .definition()
            .unwrap()
            .into_point()
            .unwrap()
            .coincides_with(&point)
            .unwrap()
    );
}

#[cfg(feature = "svg")]
#[test]
fn selected_query_fibers_replay_in_a_fresh_process_and_remain_consumable() {
    const INPUT: &str = "HYPERCURVE_SELECTED_QUERY_FIBER_REPLAY";
    fn verify(text: &str) {
        let geometry = import_svg_document(text).unwrap();
        let line = &geometry.paths()[0].curves()[0];
        let point = line.start();
        zero(&point, p(&[&[0, 0, 1], &[-1]]));
        zero(&point, p(&[&[-1], &[], &[], &[], &[], &[], &[2048]]));
        assert_eq!(
            form(&point, y()).compare_to_real(&Real::zero()).unwrap(),
            Ordering::Greater
        );
        let next = curves(p(&[&[0, 1], &[], &[-1]]));
        let root = form(&point, y()).parameter().unwrap();
        let result = next.curves()[0].point_at(&root).unwrap();
        assert_eq!(
            form(&result, y()).compare(&form(&point, x())).unwrap(),
            Ordering::Equal
        );
        assert_eq!(export_svg_document(&geometry).unwrap(), text);
    }
    if let Some(path) = std::env::var_os(INPUT) {
        verify(&std::fs::read_to_string(path).unwrap());
        return;
    }
    let parameter = form(&source(), y()).parameter().unwrap();
    let set = curves(p(&[&[0, 0, 1], &[-1]]));
    let point = set.curves()[1].point_at(&parameter).unwrap();
    let end = point
        .rational_image(&RationalMap2::new([p(&[&[1], &[1]]), y()], p(&[&[1]])))
        .unwrap();
    let line = Curve2::try_line(point, end).unwrap();
    let geometry = SvgGeometry2::new(
        CurveRegion2::empty(),
        vec![],
        vec![CurvePath2::try_new(vec![line]).unwrap()],
    );
    let text = export_svg_document(&geometry).unwrap();
    verify(&text);
    let limited = SvgOptions {
        max_point_definition_depth: 0,
        ..SvgOptions::default()
    };
    assert!(matches!(
        import_svg_document_with_options(&text, limited),
        Err(SvgError::SizeOverflow { .. })
    ));
    assert!(matches!(
        export_svg_document_with_options(&geometry, limited),
        Err(SvgError::SizeOverflow { .. })
    ));
    let path =
        std::env::temp_dir().join(format!("hypercurve-query-fiber-{}.svg", std::process::id()));
    std::fs::write(&path, &text).unwrap();
    let child = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "selected_query_fibers_replay_in_a_fresh_process_and_remain_consumable",
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
fn invalid_query_fiber_preserves_construction_operation() {
    let parameter = form(&source(), y()).parameter().unwrap();
    let error = CurvePointDefinition2::PolynomialFiber {
        equation: p(&[&[1, 0, 1]]),
        parameter,
        root: IsolatedRootInterval {
            lower: 0.into(),
            upper: 1.into(),
            exact_root: None,
            distinct_root_count: 1,
        },
    }
    .into_point()
    .unwrap_err();
    assert!(matches!(error, ExactCurveError::Invalid { .. }));
    assert_eq!(error.operation(), CurveOperation2::Construction);
}

fn integration_dependent_fiber() -> CurvePoint2 {
    let parameter = form(&source(), y()).parameter().unwrap();
    curves(p(&[&[0, 0, 1], &[-1]])).curves()[1]
        .point_at(&parameter)
        .unwrap()
}

#[test]
fn integration_dependent_fiber_source_control() {
    let point = integration_dependent_fiber();
    zero(&point, p(&[&[0, 0, 1], &[-1]]));
    zero(&point, p(&[&[-1], &[], &[], &[], &[], &[], &[2048]]));
    assert_eq!(
        form(&point, y()).compare_to_real(&Real::zero()).unwrap(),
        Ordering::Greater,
    );
}

#[test]
fn integration_dependent_fiber_feeds_derivatives_and_another_polynomial() {
    let point = integration_dependent_fiber();
    let parameter = form(&point, y()).parameter().unwrap();
    let rational: Curve2 = RationalBezier2::try_new(
        vec![
            Point2::from_values(0, 0),
            Point2::new(q(1, 2), 0.into()),
            Point2::from_values(1, 1),
        ],
        vec![Real::one(); 3],
    )
    .unwrap()
    .into();
    // C(t)=(t,t^2), so the independent derivative is (1,2t).
    let jet = rational
        .derivatives_at(&parameter, 3)
        .expect("a generated dependent fiber must feed the next derivative operation");
    assert_eq!(jet.len(), 3);
    let first = jet[0].coordinate_image().unwrap();
    assert_eq!(
        form(&first, x()).compare_to_real(&Real::one()).unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        form(&first, y())
            .scaled(&q(1, 2))
            .unwrap()
            .compare(&form(&point, y()))
            .unwrap(),
        Ordering::Equal,
    );
    for (vector, expected) in [
        (&jet[1], Point2::from_values(0, 2)),
        (&jet[2], Point2::from_values(0, 0)),
    ] {
        assert!(
            vector
                .coordinate_image()
                .unwrap()
                .coincides_with(&expected.into())
                .unwrap()
        );
    }
    let replay = first.definition().unwrap().into_point().unwrap();
    let next_parameter = form(&replay, y())
        .scaled(&q(1, 2))
        .unwrap()
        .parameter()
        .unwrap();
    let next = curves(p(&[&[0, 1], &[], &[-1]])).curves()[0]
        .point_at(&next_parameter)
        .unwrap();
    // The next graph squares the derivative's retained t and recovers v.
    assert_eq!(
        form(&next, y()).compare(&form(&point, x())).unwrap(),
        Ordering::Equal
    );
    zero(&next, p(&[&[0, 1], &[], &[-1]]));
}

#[test]
fn integration_dependent_fiber_quotient_replay_retains_the_original_pole() {
    let point = integration_dependent_fiber();
    let first = point.rational_form(p(&[&[1]]), p(&[&[1, 1]])).unwrap();
    let second = point.rational_form(p(&[&[1]]), p(&[&[-1, 0, 1]])).unwrap();
    let quotient = first.divide(&second).unwrap();
    assert_eq!(
        quotient.compare(&form(&point, p(&[&[-1, 1]]))).unwrap(),
        Ordering::Equal
    );
    let recipe = quotient
        .parameter()
        .unwrap()
        .rational_form_source()
        .unwrap();
    // The reduced value y-1 is finite at y=1, but the original divisor is
    // undefined there. Rebinding the exact recipe must retain that exclusion.
    let pole = curves(p(&[&[0, 0, 1], &[-1]])).curves()[1]
        .point_at(&Real::one().into())
        .unwrap();
    zero(&pole, p(&[&[-1, 0, 1]]));
    let rebound = CurvePointDefinition2::RationalQueryPair {
        sources: [pole, Point2::from_values(0, 0).into()],
        numerators: [recipe.numerator().clone(), p(&[&[0]])],
        denominators: [recipe.denominator().clone(), p(&[&[1]])],
    }
    .into_point();
    assert!(
        matches!(
            rebound,
            Err(ExactCurveError::Invalid {
                cause: CurveError::ZeroRationalPointDenominator,
                ..
            })
        ),
        "the original divisor pole must survive generated-fiber composition: {rebound:?}",
    );
}

#[cfg(feature = "svg")]
#[test]
fn integration_dependent_fiber_derivative_replays_cold_and_feeds_geometry() {
    const INPUT: &str = "HYPERCURVE_INTEGRATED_FIBER_DERIVATIVE_REPLAY";
    fn rational() -> Curve2 {
        RationalBezier2::try_new(
            vec![
                Point2::from_values(0, 0),
                Point2::new(q(1, 2), 0.into()),
                Point2::from_values(1, 1),
            ],
            vec![Real::one(); 3],
        )
        .unwrap()
        .into()
    }
    fn verify(text: &str) {
        let geometry = import_svg_document(text).unwrap();
        let first = geometry.paths()[0].curves()[0].start();
        assert_eq!(
            form(&first, x()).compare_to_real(&Real::one()).unwrap(),
            Ordering::Equal
        );
        assert_eq!(
            form(&first, y()).compare_to_real(&Real::zero()).unwrap(),
            Ordering::Greater
        );
        // 2048 v^6=1, t^2=v and dy=2t imply dy^12=2.
        let mut coefficients = vec![Real::zero(); 13];
        coefficients[0] = (-2).into();
        coefficients[12] = 1.into();
        zero(&first, BivariatePolynomial::new(vec![coefficients]));
        let parameter = form(&first, y())
            .scaled(&q(1, 2))
            .unwrap()
            .parameter()
            .unwrap();
        let next = curves(p(&[&[0, 1], &[], &[-1]])).curves()[0]
            .point_at(&parameter)
            .unwrap();
        zero(&next, p(&[&[0, 1], &[], &[-1]]));
        zero(&next, p(&[&[-1, 0, 0, 0, 0, 0, 2048]]));
        let derivative = rational()
            .derivative_at(&parameter)
            .unwrap()
            .coordinate_image()
            .unwrap();
        assert!(derivative.coincides_with(&first).unwrap());
        assert_eq!(export_svg_document(&geometry).unwrap(), text);
    }
    if let Some(path) = std::env::var_os(INPUT) {
        verify(&std::fs::read_to_string(path).unwrap());
        return;
    }
    let point = integration_dependent_fiber();
    let parameter = form(&point, y()).parameter().unwrap();
    let first = rational()
        .derivative_at(&parameter)
        .unwrap()
        .coordinate_image()
        .unwrap();
    let end = first
        .rational_image(&RationalMap2::new([p(&[&[1], &[1]]), y()], p(&[&[1]])))
        .unwrap();
    let curve = Curve2::try_line(first, end).unwrap();
    let geometry = SvgGeometry2::new(
        CurveRegion2::empty(),
        vec![],
        vec![CurvePath2::try_new(vec![curve]).unwrap()],
    );
    let text = export_svg_document(&geometry).unwrap();
    verify(&text);
    let path = std::env::temp_dir().join(format!(
        "hypercurve-integrated-fiber-derivative-{}.svg",
        std::process::id()
    ));
    std::fs::write(&path, &text).unwrap();
    let child = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "integration_dependent_fiber_derivative_replays_cold_and_feeds_geometry",
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
