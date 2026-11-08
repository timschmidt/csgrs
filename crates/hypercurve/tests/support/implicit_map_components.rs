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
fn implicit_shared_sections_preserve_original_five_case_census() {
    use support::IntoCertified;
    let alpha: CurveParameter2 = hypercurve::BezierParameterPolynomial::try_new_power_basis(
        [1, -3, 1].map(Real::from).to_vec(),
    )
    .unwrap()
    .isolate_unit_interval_roots()
    .unwrap()[0]
        .clone()
        .into();
    for policy in [
        hypercurve::PredicatePolicy::STRICT,
        hypercurve::PredicatePolicy::APPROXIMATE_512,
    ] {
        support::under(&policy, || -> hypercurve::ExactCurveResult<()> {
            #[cfg(feature = "dispatch-trace")]
            let _recording = hyperreal::dispatch_trace::recording_scope();
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::reset();
            let source = branches(
                p(vec![vec![0, 0, 1], vec![-1]]),
                [0.into(), 1.into()],
                [0.into(), 1.into()],
            )
            .remove(0);
            let shared = sqrt_branch_with_components(true);
            for (curve, d, expected) in [
                (&source, p(vec![vec![-1], vec![2]]), vec![q(1, 2).into()]),
                (&source, p(vec![vec![-1, 2]]), vec![q(1, 4).into()]),
                (
                    &source,
                    p(vec![vec![0, 1], vec![-1]]),
                    vec![Real::zero().into(), Real::one().into()],
                ),
                (&source, p(vec![vec![-1, 1], vec![1]]), vec![alpha.clone()]),
                (&shared, p(vec![vec![-2, 1]]), vec![]),
            ] {
                let squared = hypersolve::bivariate_arithmetic::bivariate_multiply(&d, &d);
                let map = RationalMap2::new([d, squared], p(vec![vec![1]]));
                let actual = locations(curve, &map, &Point2::from_values(0, 0).into());
                assert_eq!(actual.len(), expected.len());
                for (actual, expected) in actual.iter().zip(&expected) {
                    assert_eq!(actual.compare(expected).unwrap(), Ordering::Equal);
                }
            }
            #[cfg(feature = "dispatch-trace")]
            assert_eq!(
                hyperreal::dispatch_trace::take()
                    .iter()
                    .filter(|entry| {
                        entry.layer == "hypercurve"
                            && entry.operation == "polynomial-section"
                            && entry.path == "reuse-zero-projection"
                    })
                    .map(|entry| entry.count)
                    .sum::<u64>(),
                1,
                "the shared-component retry must reuse its checked ordinary projection"
            );
            Ok(())
        })
        .unwrap()
        .into_certified();
    }
}

#[test]
fn implicit_unselected_common_support_retains_isolated_contacts_and_original_guards() {
    use hypersolve::bivariate_arithmetic::bivariate_multiply;
    use support::IntoCertified;
    for policy in [
        hypercurve::PredicatePolicy::STRICT,
        hypercurve::PredicatePolicy::APPROXIMATE_512,
    ] {
        support::under(&policy, || -> hypercurve::ExactCurveResult<()> {
            let source = sqrt_branch_with_components(true);
            let query = Point2::from_values(0, 0).into();
            let common = p(vec![vec![-2, 1]]);
            for (section, expected) in [
                (p(vec![vec![-1], vec![2]]), vec![q(1, 2).into()]),
                (
                    p(vec![vec![0, 1], vec![-1]]),
                    vec![Real::zero().into(), Real::one().into()],
                ),
                // A double zero is still one location; it is not an overlap.
                (p(vec![vec![1], vec![-4], vec![4]]), vec![q(1, 2).into()]),
            ] {
                let d = bivariate_multiply(&common, &section);
                let square = bivariate_multiply(&d, &d);
                let map = RationalMap2::new([d.clone(), square.clone()], p(vec![vec![1]]));
                for curve in [source.clone(), source.reversed()?] {
                    let actual = locations(&curve, &map, &query);
                    assert_eq!(actual.len(), expected.len());
                    for (actual, expected) in actual.iter().zip(&expected) {
                        assert_eq!(actual.compare(expected).unwrap(), Ordering::Equal);
                    }
                }
                // A zero of the original denominator remains excluded even
                // when both image numerators vanish at the same parameter.
                let guarded = RationalMap2::new([d, square], section);
                assert!(locations(&source, &guarded, &query).is_empty());
            }
            Ok(())
        })
        .unwrap()
        .into_certified();
    }
}

#[test]
fn implicit_shared_sections_accept_independent_algebraic_query_fields_and_retained_maps() {
    use support::IntoCertified;
    // a=1/sqrt(2) is selected independently of the source decomposition.
    let alpha: CurveParameter2 = hypercurve::BezierParameterPolynomial::try_new_power_basis(
        [-1, 0, 2].map(Real::from).to_vec(),
    )
    .unwrap()
    .isolate_unit_interval_roots()
    .unwrap()[0]
        .clone()
        .into();
    let query = monomials(2, 1, 2).point_at(&alpha).unwrap();
    assert!(query.coordinates().is_none());
    for policy in [
        hypercurve::PredicatePolicy::STRICT,
        hypercurve::PredicatePolicy::APPROXIMATE_512,
    ] {
        support::under(&policy, || -> hypercurve::ExactCurveResult<()> {
            // F=(y²-x)(2y²-1), with an unselected y=a component.
            let f = hypersolve::bivariate_arithmetic::bivariate_multiply(
                &p(vec![vec![0, 0, 1], vec![-1]]),
                &p(vec![vec![-1, 0, 2]]),
            );
            let source = branches(f, [0.into(), 1.into()], [0.into(), 1.into()])
                .into_iter()
                .find(|curve| {
                    curve.point_at(&q(1, 4).into()).is_ok_and(|p| {
                        p.coincides_with(&Point2::new(q(1, 4), q(1, 2)).into())
                            .unwrap()
                    })
                })
                .unwrap();
            let map = RationalMap2::new(
                [p(vec![vec![0, 1]]), p(vec![vec![0, 0, 1]])],
                p(vec![vec![1]]),
            );
            let actual = locations(&source, &map, &query);
            assert_eq!(actual.len(), 1);
            assert_eq!(actual[0].compare(&q(1, 2).into()).unwrap(), Ordering::Equal);
            let image = source.rational_image(&map)?;
            let CurvePointLocations2::Locations(visits) = image.point_locations(&query)? else {
                panic!("one selected endpoint contact")
            };
            assert_eq!(visits.len(), 1);
            let parameter = visits[0].parameter()?;
            assert_eq!(parameter.compare(&q(1, 2).into()).unwrap(), Ordering::Equal);
            assert!(image.point_at(&parameter)?.coincides_with(&query)?);
            let trim = source
                .subcurve(Real::zero().into(), q(1, 4).into())?
                .reversed()?;
            assert!(locations(&trim, &map, &query).is_empty());
            Ok(())
        })
        .unwrap()
        .into_certified();
    }
}

#[test]
fn implicit_shared_sections_keep_independent_quintic_endpoint_incidence() {
    use support::IntoCertified;
    // g(a)=0 is irreducible: its reciprocal is Eisenstein at 2.
    let coefficients = [-1, 2, 2, 2, 2, 2];
    let alpha: CurveParameter2 = hypercurve::BezierParameterPolynomial::try_new_power_basis(
        coefficients.map(Real::from).to_vec(),
    )
    .unwrap()
    .isolate_unit_interval_roots()
    .unwrap()[0]
        .clone()
        .into();
    let query = monomials(2, 1, 2).point_at(&alpha).unwrap();
    assert!(query.coordinates().is_none());
    // Writing g(a)=E(a²)+a*O(a²), E²-t*O² gives this independent
    // polynomial for t=a². It is strictly decreasing on [0,1].
    let expected: CurveParameter2 = hypercurve::BezierParameterPolynomial::try_new_power_basis(
        [1, -8, -8, -4, -4, -4].map(Real::from).to_vec(),
    )
    .unwrap()
    .isolate_unit_interval_roots()
    .unwrap()[0]
        .clone()
        .into();
    for policy in [
        hypercurve::PredicatePolicy::STRICT,
        hypercurve::PredicatePolicy::APPROXIMATE_512,
    ] {
        support::under(&policy, || -> hypercurve::ExactCurveResult<()> {
            let f = hypersolve::bivariate_arithmetic::bivariate_multiply(
                &p(vec![vec![0, 0, 1], vec![-1]]),
                &p(vec![coefficients.to_vec()]),
            );
            let source = branches(f, [0.into(), 1.into()], [0.into(), 1.into()])
                .into_iter()
                .find(|curve| {
                    curve.point_at(&q(1, 100).into()).is_ok_and(|p| {
                        p.coincides_with(&Point2::new(q(1, 100), q(1, 10)).into())
                            .unwrap()
                    })
                })
                .unwrap();
            let map = RationalMap2::new(
                [p(vec![vec![0, 1]]), p(vec![vec![0, 0, 1]])],
                p(vec![vec![1]]),
            );
            let actual = locations(&source, &map, &query);
            assert_eq!(actual.len(), 1);
            assert_eq!(actual[0].compare(&expected).unwrap(), Ordering::Equal);
            let point = source.point_at(&actual[0])?;
            assert!(point.coordinates().is_none());
            assert!(point.rational_image(&map)?.coincides_with(&query)?);
            let CurvePointLocations2::Locations(visits) =
                source.rational_image(&map)?.point_locations(&query)?
            else {
                panic!("one selected quintic endpoint")
            };
            assert_eq!(visits.len(), 1);
            assert_eq!(
                visits[0].parameter()?.compare(&expected).unwrap(),
                Ordering::Equal
            );
            Ok(())
        })
        .unwrap()
        .into_certified();
    }
}

#[test]
fn implicit_shared_section_budgets_cancellation_and_retry_keep_empty_success_distinct() {
    let source = sqrt_branch_with_components(true);
    let d = p(vec![vec![-2, 1]]);
    let map = RationalMap2::new(
        [
            d.clone(),
            hypersolve::bivariate_arithmetic::bivariate_multiply(&d, &d),
        ],
        p(vec![vec![1]]),
    );
    let query = Point2::from_values(0, 0).into();
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
        let token = hypercurve::CancellationToken::new();
        token.cancel();
        let cancelled = hypercurve::evaluate_with_cancellable_budget(
            policy,
            hypercurve::WorkBudget::UNLIMITED,
            &token,
            || source.rational_image_point_locations(&map, &query),
        );
        assert_eq!(
            cancelled.result.unwrap_err().kind,
            hypercurve::WorkKind::Cancellation
        );
        let retry = hypercurve::evaluate_with_budget(
            policy,
            hypercurve::WorkBudget {
                promotions: 100_000,
                refinements: 1_000_000,
            },
            || source.rational_image_point_locations(&map, &query),
        );
        let outcome = retry.result.unwrap();
        assert!(
            matches!(outcome.certified().unwrap().unwrap(), CurvePointLocations2::Locations(points) if points.is_empty())
        );
        assert!(matches!(
            source.rational_image_point_locations(
                &RationalMap2::new([d.clone(), d.clone()], p(vec![])),
                &query
            ),
            Err(hypercurve::ExactCurveError::Invalid { .. })
        ));
    }
}

#[test]
fn implicit_selected_continuous_sections_require_a_correspondence() {
    // The new finite-candidate projection must never turn a selected common
    // support into an empty finite section. General continuous sections remain
    // an explicit implementation gap until a domain correspondence is carried.
    let f = hypersolve::bivariate_arithmetic::bivariate_multiply(
        &p(vec![vec![0, 0, 1], vec![-1]]),
        &p(vec![vec![-2, 1]]),
    );
    let source = branches(f, [0.into(), 1.into()], [0.into(), 3.into()])
        .into_iter()
        .find(|curve| {
            curve.point_at(&q(1, 4).into()).is_ok_and(|p| {
                p.coincides_with(&Point2::new(q(1, 4), 2.into()).into())
                    .unwrap()
            })
        })
        .unwrap();
    let d = p(vec![vec![-2, 1]]);
    let map = RationalMap2::new(
        [
            d.clone(),
            hypersolve::bivariate_arithmetic::bivariate_multiply(&d, &d),
        ],
        p(vec![vec![1]]),
    );
    for policy in [
        hypercurve::PredicatePolicy::STRICT,
        hypercurve::PredicatePolicy::APPROXIMATE_512,
    ] {
        let result = support::under(&policy, || {
            source.rational_image_point_locations(&map, &Point2::from_values(0, 0).into())
        });
        assert!(
            matches!(result, Err(hypercurve::ExactCurveError::Blocked(blocker)) if blocker.reason() == hypercurve::UncertaintyReason::Unsupported)
        );
    }
}

#[cfg(feature = "svg")]
#[test]
fn implicit_shared_sections_replay_cold_after_affine_mapping_trimming_and_reversal() {
    const INPUT: &str = "HYPERCURVE_IMPLICIT_SHARED_SECTION_REPLAY";
    fn map() -> RationalMap2 {
        // The retained shear is (X,Y)=(x+y,y). Thus the original
        // (y-2)(2x-1) pulls forward to (Y-2)(2X-2Y-1).
        let d = hypersolve::bivariate_arithmetic::bivariate_multiply(
            &p(vec![vec![-2, 1]]),
            &p(vec![vec![-1, -2], vec![2]]),
        );
        let square = hypersolve::bivariate_arithmetic::bivariate_multiply(&d, &d);
        RationalMap2::new([d, square], p(vec![vec![1]]))
    }
    fn verify(text: &str) {
        use support::IntoCertified;
        let geometry = hypercurve::import_svg_document(text).unwrap();
        let source = &geometry.paths()[0].curves()[0];
        let retained = geometry.paths()[1].curves()[0].start();
        for policy in [
            hypercurve::PredicatePolicy::STRICT,
            hypercurve::PredicatePolicy::APPROXIMATE_512,
        ] {
            support::under(&policy, || -> hypercurve::ExactCurveResult<()> {
                let actual = locations(source, &map(), &Point2::from_values(0, 0).into());
                assert_eq!(actual.len(), 1);
                assert_eq!(actual[0].compare(&q(1, 2).into()).unwrap(), Ordering::Equal);
                let point = source.point_at(&actual[0])?;
                assert!(point.coincides_with(&retained)?);
                // Independent defining relations certify the retained point.
                let one = p(vec![vec![1]]);
                assert_eq!(
                    point
                        .rational_form(p(vec![vec![0, -1], vec![1]]), one.clone())?
                        .compare_to_real(&q(1, 2))?,
                    Ordering::Equal
                );
                assert_eq!(
                    point
                        .rational_form(p(vec![vec![0, 0, 1]]), one)?
                        .compare_to_real(&q(1, 2))?,
                    Ordering::Equal
                );
                let CurvePointLocations2::Locations(visits) = source.point_locations(&retained)?
                else {
                    panic!("one retained source visit")
                };
                assert_eq!(visits.len(), 1);
                assert_eq!(
                    visits[0].parameter()?.compare(&actual[0]).unwrap(),
                    Ordering::Equal
                );
                Ok(())
            })
            .unwrap()
            .into_certified();
        }
        assert_eq!(hypercurve::export_svg_document(&geometry).unwrap(), text);
    }
    if let Some(path) = std::env::var_os(INPUT) {
        verify(&std::fs::read_to_string(path).unwrap());
        return;
    }
    let shear = RationalMap2::new(
        [p(vec![vec![0, 1], vec![1]]), p(vec![vec![0, 1]])],
        p(vec![vec![1]]),
    );
    let source = sqrt_branch_with_components(true)
        .rational_image(&shear)
        .unwrap()
        .subcurve(q(1, 4).into(), q(3, 4).into())
        .unwrap()
        .reversed()
        .unwrap();
    let parameter = locations(&source, &map(), &Point2::from_values(0, 0).into()).remove(0);
    let point = source.point_at(&parameter).unwrap();
    let end = point
        .rational_image(&RationalMap2::new(
            [p(vec![vec![1], vec![1]]), p(vec![vec![0, 1]])],
            p(vec![vec![1]]),
        ))
        .unwrap();
    let geometry = hypercurve::SvgGeometry2::new(
        hypercurve::CurveRegion2::empty(),
        vec![],
        vec![
            hypercurve::CurvePath2::try_new(vec![source]).unwrap(),
            hypercurve::CurvePath2::try_new(vec![Curve2::try_line(point, end).unwrap()]).unwrap(),
        ],
    );
    let text = hypercurve::export_svg_document(&geometry).unwrap();
    verify(&text);
    let path = std::env::temp_dir().join(format!(
        "hypercurve-implicit-shared-section-{}.svg",
        std::process::id()
    ));
    std::fs::write(&path, text).unwrap();
    let child = std::process::Command::new(std::env::current_exe().unwrap()).args([
        "--exact", "implicit_map_components::implicit_shared_sections_replay_cold_after_affine_mapping_trimming_and_reversal", "--nocapture",
    ]).env(INPUT, &path).output().unwrap();
    std::fs::remove_file(path).unwrap();
    assert!(
        child.status.success(),
        "{}\n{}",
        String::from_utf8_lossy(&child.stdout),
        String::from_utf8_lossy(&child.stderr)
    );
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
    // Keep the original reducible degree-five selector and its cubic real
    // root. The second selector has an Eisenstein-at-two reciprocal, so its
    // selected ordinate has field degree five. Source and line-query roots
    // are selected independently without stored Real coordinates.
    for coefficients in [[-1, 1, 0, 0, 0, 1], [-1, 2, 2, 2, 2, 2]] {
        let source = branches(
            p(vec![coefficients.to_vec()]),
            [Real::zero(), Real::one()],
            [Real::zero(), Real::one()],
        )
        .remove(0);
        let parameter: CurveParameter2 =
            hypercurve::BezierParameterPolynomial::try_new_power_basis(
                coefficients.map(Real::from).to_vec(),
            )
            .unwrap()
            .isolate_unit_interval_roots()
            .unwrap()[0]
                .clone()
                .into();
        let independent = |x: Real, sign: i32| {
            Curve2::from(
                LineSeg2::try_new(
                    Point2::new(x.clone(), Real::zero()),
                    Point2::new(x, sign.into()),
                )
                .unwrap(),
            )
            .point_at(&parameter)
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
