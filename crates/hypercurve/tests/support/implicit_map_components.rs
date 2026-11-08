//! Selected graph guard domains and consuming operations.
use super::*;

fn constant_map_with_guard(guard: BivariatePolynomial) -> RationalMap2 {
    use hypersolve::bivariate_arithmetic::bivariate_scale;
    RationalMap2::new(
        [
            bivariate_scale(guard.clone(), &2.into()),
            bivariate_scale(guard.clone(), &3.into()),
        ],
        guard,
    )
}
fn sqrt_branch_with_components(extra: bool) -> Curve2 {
    let equation = p(vec![vec![0, 0, 1], vec![-1]]);
    let equation = if extra {
        hypersolve::bivariate_arithmetic::bivariate_multiply(&equation, &p(vec![vec![-2, 1]]))
    } else {
        equation
    };
    let candidates = branches(
        equation,
        [Real::zero(), Real::one()],
        [Real::zero(), 3.into()],
    );
    let point: CurvePoint2 = Point2::new(q(1, 4), q(1, 2)).into();
    candidates
        .into_iter()
        .find(|curve| {
            curve
                .point_at(&q(1, 4).into())
                .unwrap()
                .coincides_with(&point)
                .unwrap()
        })
        .unwrap()
}
#[test]
fn implicit_constant_images_keep_canceled_poles_and_selected_source_points() {
    use support::IntoCertified;
    for policy in [
        hypercurve::PredicatePolicy::STRICT,
        hypercurve::PredicatePolicy::APPROXIMATE_512,
    ] {
        let source = sqrt_branch_with_components(false);
        let query = selected_constant_query();
        assert!(query.coordinates().is_none());
        let map = constant_map_with_guard(p(vec![vec![-1], vec![2]]));
        let result = support::under(&policy, || {
            source.rational_image_point_locations(&map, &query)
        })
        .unwrap()
        .into_certified();
        let expected = [
            (Real::zero().into(), q(1, 2).into(), [true, false]),
            (q(1, 2).into(), Real::one().into(), [false, true]),
        ];
        assert_components(result.clone(), &expected);
        let CurvePointLocations2::Components(parts) = &result else {
            unreachable!()
        };
        for (parameter, expected) in [
            (Real::zero(), true),
            (q(1, 4), true),
            (q(1, 2), false),
            (q(3, 4), true),
            (Real::one(), true),
        ] {
            assert_eq!(
                parts
                    .iter()
                    .any(|c| c.contains_parameter(&parameter.clone().into()).unwrap()),
                expected
            );
        }
        let retained = source
            .point_at_component(&parts[0], &q(1, 4).into())
            .unwrap();
        assert!(retained.coordinates().is_none());
        assert!(
            retained
                .coincides_with(&Point2::new(q(1, 4), q(1, 2)).into())
                .unwrap()
        );
        assert!(
            source
                .point_at_component(&parts[0], &q(1, 2).into())
                .is_err()
        );
        assert_components(
            source.intersect_point_locations(&result, &result).unwrap(),
            &expected,
        );
        assert_components(
            source
                .reversed()
                .unwrap()
                .rational_image_point_locations(&map, &query)
                .unwrap(),
            &expected,
        );
        assert_components(
            source
                .subcurve(q(1, 4).into(), q(3, 4).into())
                .unwrap()
                .rational_image_point_locations(&map, &query)
                .unwrap(),
            &[
                (q(1, 4).into(), q(1, 2).into(), [true, false]),
                (q(1, 2).into(), q(3, 4).into(), [false, true]),
            ],
        );
        let absent: CurvePoint2 = Point2::from_values(2, 4).into();
        assert!(
            matches!(source.rational_image_point_locations(&map,&absent).unwrap(),CurvePointLocations2::Locations(points) if points.is_empty())
        );
    }
}
#[test]
fn implicit_guard_census_handles_ordinates_algebraic_roots_and_endpoints() {
    let source = sqrt_branch_with_components(false);
    let query = selected_constant_query();
    let mixed: CurveParameter2 = hypercurve::BezierParameterPolynomial::try_new_power_basis(
        [1, -3, 1].map(Real::from).to_vec(),
    )
    .unwrap()
    .isolate_unit_interval_roots()
    .unwrap()[0]
        .clone()
        .into();
    for (guard, root) in [
        (p(vec![vec![-1, 2]]), q(1, 4).into()),
        (p(vec![vec![-1, 1], vec![1]]), mixed),
    ] {
        let map = constant_map_with_guard(guard);
        assert_components(
            source.rational_image_point_locations(&map, &query).unwrap(),
            &[
                (Real::zero().into(), root.clone(), [true, false]),
                (root, Real::one().into(), [false, true]),
            ],
        );
    }
    let map = constant_map_with_guard(p(vec![vec![0, 1], vec![-1]]));
    assert_components(
        source.rational_image_point_locations(&map, &query).unwrap(),
        &[(Real::zero().into(), Real::one().into(), [false, false])],
    );
    let repeated = constant_map_with_guard(p(vec![vec![1], vec![-4], vec![4]]));
    assert_components(
        source
            .rational_image_point_locations(&repeated, &query)
            .unwrap(),
        &[
            (Real::zero().into(), q(1, 2).into(), [true, false]),
            (q(1, 2).into(), Real::one().into(), [false, true]),
        ],
    );
}
#[test]
fn implicit_guard_shared_factors_distinguish_selected_and_unselected_components() {
    let source = sqrt_branch_with_components(true);
    let query = selected_constant_query();
    let unselected = p(vec![vec![-2, 1]]);
    let map = constant_map_with_guard(unselected.clone());
    assert!(matches!(
        source.rational_image_point_locations(&map, &query).unwrap(),
        CurvePointLocations2::EntireCurve
    ));
    let mixed = hypersolve::bivariate_arithmetic::bivariate_multiply(
        &unselected,
        &p(vec![vec![-1], vec![2]]),
    );
    assert_components(
        source
            .rational_image_point_locations(&constant_map_with_guard(mixed), &query)
            .unwrap(),
        &[
            (Real::zero().into(), q(1, 2).into(), [true, false]),
            (q(1, 2).into(), Real::one().into(), [false, true]),
        ],
    );
    let selected = p(vec![vec![0, 0, 1], vec![-1]]);
    for guard in [
        selected.clone(),
        hypersolve::bivariate_arithmetic::bivariate_multiply(&selected, &unselected),
    ] {
        assert!(
            matches!(source.rational_image_point_locations(&constant_map_with_guard(guard),&query).unwrap(),CurvePointLocations2::Locations(points) if points.is_empty())
        );
    }
    assert!(matches!(
        source.rational_image_point_locations(&constant_map_with_guard(p(vec![])), &query),
        Err(hypercurve::ExactCurveError::Invalid { .. })
    ));
}

#[test]
fn implicit_constant_guard_census_admits_degree_nine_and_recovers_from_budgets() {
    let mut equation = vec![vec![Real::zero(); 10], vec![Real::from(-1)]];
    equation[0][9] = Real::one();
    let topology = decompose_bivariate_curve(
        &BivariatePolynomial::new(equation),
        &BivariateCurveDomain {
            first: [Real::zero(), Real::one()],
            second: [Real::zero(), Real::one()],
        },
        BivariateTopologyConfig {
            max_axis_degree: 10,
            max_projection_degree: 512,
            ..Default::default()
        },
    )
    .unwrap();
    let source = PolynomialCurveSet2::from_topology(topology, Default::default())
        .unwrap()
        .curves()[0]
        .clone();
    let query = selected_constant_query();
    let map = constant_map_with_guard(p(vec![vec![-1, 2]]));
    let expected = [
        (Real::zero().into(), q(1, 512).into(), [true, false]),
        (q(1, 512).into(), Real::one().into(), [false, true]),
    ];
    for policy in [
        hypercurve::PredicatePolicy::STRICT,
        hypercurve::PredicatePolicy::APPROXIMATE_512,
        hypercurve::PredicatePolicy::STRICT,
    ] {
        let refused =
            hypercurve::evaluate_with_budget(policy, hypercurve::WorkBudget::ZERO, || {
                source.rational_image_point_locations(&map, &query)
            });
        let exhausted = refused.result.unwrap_err();
        assert_eq!(exhausted.limit, 0);
        assert!(exhausted.requested > 0);
        let retry = hypercurve::evaluate_with_budget(
            policy,
            hypercurve::WorkBudget {
                promotions: 100_000,
                refinements: 1_000_000,
            },
            || source.rational_image_point_locations(&map, &query),
        );
        let outcome = retry.result.unwrap();
        assert_eq!(outcome.certainty(), hypercurve::CurveCertainty::Certified);
        assert_components(outcome.certified().unwrap().unwrap(), &expected);
    }
}

#[test]
fn implicit_constant_images_keep_independent_irrational_values() {
    use support::IntoCertified;
    // The source's ordinate and the unrelated line query independently select
    // alpha^5+alpha=1. Neither selected point needs a stored Real coordinate.
    let source = branches(
        p(vec![vec![-1, 1, 0, 0, 0, 1]]),
        [Real::zero(), Real::one()],
        [Real::zero(), Real::one()],
    )
    .remove(0);
    let independent = |x: Real, sign: i32| {
        Curve2::from(
            LineSeg2::try_new(
                Point2::new(x.clone(), Real::zero()),
                Point2::new(x, sign.into()),
            )
            .unwrap(),
        )
        .point_at(&alpha())
        .unwrap()
    };
    let query = independent(2.into(), 1);
    let absent = independent(2.into(), -1);
    assert!(query.coordinates().is_none());
    assert!(absent.coordinates().is_none());
    let map = RationalMap2::new(
        [p(vec![vec![-2], vec![4]]), p(vec![vec![0, -1], vec![0, 2]])],
        p(vec![vec![-1], vec![2]]),
    );
    for policy in [
        hypercurve::PredicatePolicy::STRICT,
        hypercurve::PredicatePolicy::APPROXIMATE_512,
    ] {
        let locus = support::under(&policy, || -> hypercurve::ExactCurveResult<_> {
            let locus = source.rational_image_point_locations(&map, &query)?;
            let CurvePointLocations2::Components(parts) = &locus else { panic!("two punctured intervals") };
            assert_eq!(parts.len(), 2);
            let value = source.point_at_component(&parts[0], &q(1, 4).into())?;
            assert!(value.coordinates().is_none());
            assert!(value.coincides_with(&independent(q(1, 4), 1))?);
            assert!(value.rational_image(&map)?.coincides_with(&query)?);
            let CurvePointLocations2::Locations(visits) = source.point_locations(&value)? else { panic!("one source visit") };
            assert_eq!(visits.len(), 1);
            assert_eq!(visits[0].parameter()?.compare(&q(1, 4).into()).unwrap(), Ordering::Equal);
            assert!(matches!(source.rational_image_point_locations(&map, &absent)?, CurvePointLocations2::Locations(points) if points.is_empty()));
            Ok(locus)
        }).unwrap().into_certified();
        assert_components(
            locus,
            &[
                (Real::zero().into(), q(1, 2).into(), [true, false]),
                (q(1, 2).into(), Real::one().into(), [false, true]),
            ],
        );
    }
}

#[cfg(feature = "svg")]
#[test]
fn implicit_punctured_loci_replay_cold_and_consume_retained_map_domains() {
    const INPUT: &str = "HYPERCURVE_IMPLICIT_PUNCTURED_LOCUS_REPLAY";
    fn verify(text: &str) {
        let geometry = hypercurve::import_svg_document(text).unwrap();
        let source = &geometry.paths()[0].curves()[0];
        let query = geometry.paths()[1].curves()[0].start();
        let retained = geometry.paths()[2].curves()[0].start();
        assert!(source.polynomial_branch().is_some());
        let map = constant_map_with_guard(p(vec![vec![-1, -2], vec![2]]));
        let result = source.rational_image_point_locations(&map, &query).unwrap();
        assert_components(
            result.clone(),
            &[
                (q(1, 4).into(), q(1, 2).into(), [true, false]),
                (q(1, 2).into(), q(3, 4).into(), [false, true]),
            ],
        );
        let CurvePointLocations2::Components(parts) = &result else {
            unreachable!()
        };
        let value = source
            .point_at_component(&parts[0], &q(3, 8).into())
            .unwrap();
        assert!(value.coincides_with(&retained).unwrap());
        let located = source.point_locations(&value).unwrap();
        let CurvePointLocations2::Locations(points) =
            source.intersect_point_locations(&result, &located).unwrap()
        else {
            panic!("one retained visit")
        };
        assert_eq!(points.len(), 1);
        assert_eq!(
            points[0]
                .parameter()
                .unwrap()
                .compare(&q(3, 8).into())
                .unwrap(),
            Ordering::Equal
        );
        assert_eq!(hypercurve::export_svg_document(&geometry).unwrap(), text);
    }
    if let Some(path) = std::env::var_os(INPUT) {
        verify(&std::fs::read_to_string(path).unwrap());
        return;
    }
    // Identity with original nonzero guard 1+x, then a shear. The query guard
    // 2(X-Y)-1 must be pulled back to original x, retaining the earlier guard.
    let identity = RationalMap2::new(
        [
            p(vec![vec![], vec![1], vec![1]]),
            p(vec![vec![0, 1], vec![0, 1]]),
        ],
        p(vec![vec![1], vec![1]]),
    );
    let shear = RationalMap2::new(
        [p(vec![vec![0, 1], vec![1]]), p(vec![vec![0, 1]])],
        p(vec![vec![1]]),
    );
    let source = sqrt_branch_with_components(false)
        .rational_image(&identity)
        .unwrap()
        .rational_image(&shear)
        .unwrap()
        .subcurve(q(1, 4).into(), q(3, 4).into())
        .unwrap()
        .reversed()
        .unwrap();
    let query = selected_constant_query();
    let map = constant_map_with_guard(p(vec![vec![-1, -2], vec![2]]));
    let CurvePointLocations2::Components(parts) =
        source.rational_image_point_locations(&map, &query).unwrap()
    else {
        panic!("two intervals")
    };
    let point = source
        .point_at_component(&parts[0], &q(3, 8).into())
        .unwrap();
    let line = |start: CurvePoint2| {
        let end = start
            .rational_image(&RationalMap2::new(
                [p(vec![vec![1], vec![1]]), p(vec![vec![0, 1]])],
                p(vec![vec![1]]),
            ))
            .unwrap();
        hypercurve::CurvePath2::try_new(vec![Curve2::try_line(start, end).unwrap()]).unwrap()
    };
    let geometry = hypercurve::SvgGeometry2::new(
        hypercurve::CurveRegion2::empty(),
        vec![],
        vec![
            hypercurve::CurvePath2::try_new(vec![source]).unwrap(),
            line(query),
            line(point),
        ],
    );
    let text = hypercurve::export_svg_document(&geometry).unwrap();
    verify(&text);
    let path = std::env::temp_dir().join(format!(
        "hypercurve-implicit-punctures-{}.svg",
        std::process::id()
    ));
    std::fs::write(&path, &text).unwrap();
    let result = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "implicit_map_components::implicit_punctured_loci_replay_cold_and_consume_retained_map_domains",
            "--nocapture",
        ])
        .env(INPUT, &path)
        .output()
        .unwrap();
    std::fs::remove_file(path).unwrap();
    assert!(
        result.status.success(),
        "{}\n{}",
        String::from_utf8_lossy(&result.stdout),
        String::from_utf8_lossy(&result.stderr)
    );
}
