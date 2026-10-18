use crate::geometry::{
    Coord, LineString, MultiPolygon, Polygon, polygons_to_profile, rect_polygon,
};
use crate::{PREDICATE_POLICY, PcbRegion, Scalar};

fn unavailable_area() -> Polygon<f64> {
    use hypercurve::{
        Curve2, CurvePath2, CurveRegion2, CurveRegionLoopRole, FillRule, LineSeg2, Point2,
        RationalBezier2,
    };
    let curve = RationalBezier2::try_new(
        [(0, 0), (1, 1), (2, 2), (3, 2), (4, 1), (5, 0)]
            .into_iter()
            .map(|(x, y)| Point2::from_values(x, y))
            .collect(),
        [1, 2, 3, 4, 5, 7].into_iter().map(Scalar::from).collect(),
    )
    .unwrap();
    let path = CurvePath2::try_new(vec![
        Curve2::from(curve),
        Curve2::from(
            LineSeg2::try_new(Point2::from_values(5, 0), Point2::from_values(0, 0)).unwrap(),
        ),
    ])
    .unwrap();
    let native = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[path],
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
    )
    .unwrap();
    assert_eq!(native.filled_area().unwrap(), None);
    Polygon::from_exact_region(
        native,
        LineString(vec![
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 5.0, y: 0.0 },
            Coord {
                x: 325.0 / 113.0,
                y: 175.0 / 113.0,
            },
            Coord { x: 0.0, y: 0.0 },
        ]),
        vec![],
    )
}
fn unavailable_profile() -> PcbRegion {
    polygons_to_profile(vec![unavailable_area()], None)
}
fn uncertainty(findings: &[crate::Violation]) {
    assert!(
        findings.iter().any(|f| f.check == "geometry-uncertainty"),
        "{findings:?}"
    );
}
fn deny_refinement() {
    let polynomial = hypercurve::BezierParameterPolynomial::try_new_power_basis(
        [-36, 1368, -11034, 31728, -38280, 16620]
            .into_iter()
            .map(Scalar::from)
            .collect(),
    )
    .unwrap();
    assert!(polynomial.isolate_unit_interval_roots().is_err());
}
#[test]
fn tiny_feature_cannot_skip_unavailable_area() {
    uncertainty(&crate::checks::tiny_layer_feature_readiness(
        "F.Cu",
        &unavailable_profile(),
        &Scalar::one(),
    ));
}
#[test]
fn layer_sanity_cannot_describe_unavailable_area_as_empty() {
    let findings = crate::checks::layer_sanity("F.Cu", &unavailable_profile(), None);
    uncertainty(&findings);
    assert!(!findings.iter().any(|f| {
        f.message
            .as_deref()
            .is_some_and(|m| m.contains("parsed to empty"))
    }));
}
#[test]
fn copper_balance_cannot_drop_a_layer_with_unavailable_area() {
    let findings = crate::checks::copper_balance(
        &[
            ("F.Cu".into(), unavailable_profile()),
            (
                "B.Cu".into(),
                polygons_to_profile(vec![rect_polygon([0.0, 0.0], [1.0, 1.0], 0.0)], None),
            ),
        ],
        &Scalar::from(2),
        &Scalar::zero(),
    );
    uncertainty(&findings);
    assert_eq!(findings[0].layers, ["F.Cu", "B.Cu"]);
}
#[test]
fn duplicate_layer_check_cannot_drop_unavailable_area() {
    let profile = unavailable_profile();
    let findings = crate::checks::duplicate_layer_geometry_readiness(
        &[("F.Cu".into(), profile.clone()), ("B.Cu".into(), profile)],
        &Scalar::zero(),
    );
    uncertainty(&findings);
    assert_eq!(findings[0].layers, ["B.Cu", "F.Cu"]);
}
#[test]
fn paste_ratio_cannot_skip_a_required_copper_area() {
    uncertainty(&crate::checks::paste_aperture_ratio(
        "F.Paste",
        &crate::geometry::empty_profile(None),
        "F.Cu",
        &unavailable_profile(),
        &Scalar::one(),
        &Scalar::from(2),
        &Scalar::zero(),
    ));
}
#[test]
fn report_filter_cannot_erase_an_unavailable_area() {
    let shapes = crate::geometry::multipolygon_to_shapes_scalar(
        &MultiPolygon(vec![unavailable_area()]),
        &Scalar::zero(),
    );
    assert!(
        shapes.is_err(),
        "unavailable area must be a typed failure, not empty success"
    );
}
#[test]
fn report_filter_cannot_treat_an_unknown_threshold_as_below_gate() {
    let sine = Scalar::e().sin();
    let cosine = Scalar::e().cos();
    let unknown_zero =
        crate::test_support::terminal_zero(&sine * &sine + &cosine * &cosine - Scalar::one());
    let shapes = crate::geometry::multipolygon_to_shapes_scalar(
        &MultiPolygon(vec![rect_polygon([0.0, 0.0], [1.0, 1.0], 0.0)]),
        &(Scalar::one() + unknown_zero),
    );
    assert!(
        shapes.is_err(),
        "unknown comparison must be a typed failure, not empty success"
    );
}
#[test]
fn exhausted_empty_area_collection_cannot_publish_zero() {
    let result =
        hypercurve::evaluate_with_budget(PREDICATE_POLICY, hypercurve::WorkBudget::ZERO, || {
            deny_refinement();
            assert!(crate::geometry::multipolygon_area_scalar(&MultiPolygon(vec![])).is_err());
        });
    assert!(result.result.is_err());
}
#[test]
fn copper_balance_uses_filled_sets_not_authored_duplicate_counts() {
    let polygon = rect_polygon([0.0, 0.0], [1.0, 1.0], 0.0);
    let layers = [
        (
            "F.Cu".into(),
            polygons_to_profile(vec![polygon.clone(), polygon.clone()], None),
        ),
        ("B.Cu".into(), polygons_to_profile(vec![polygon], None)),
    ];
    let findings = crate::checks::copper_balance(&layers, &Scalar::one(), &Scalar::zero());
    assert!(findings.is_empty(), "{findings:?}");
}
#[test]
fn retained_polygon_failure_already_blocks_measurement() {
    let polygon =
        rect_polygon([0.0, 0.0], [1.0, 1.0], 0.0).with_exact_construction_error("retained failure");
    assert!(crate::geometry::polygon_area_scalar(&polygon).is_err());
}

#[test]
fn duplicate_paste_profiles_cannot_double_the_coverage_ratio() {
    let pad = rect_polygon([0.0, 0.0], [1.0, 1.0], 0.0);
    let paste = polygons_to_profile(vec![pad.clone(), pad.clone()], None);
    let copper = polygons_to_profile(vec![pad], None);
    let findings = crate::checks::paste_aperture_ratio(
        "F.Paste",
        &paste,
        "F.Cu",
        &copper,
        &crate::scalar::scalar("0.9"),
        &crate::scalar::scalar("1.1"),
        &Scalar::zero(),
    );
    assert!(findings.is_empty(), "{findings:?}");
}

#[test]
fn duplicate_mask_profiles_cannot_double_the_opening_ratio() {
    let pad = rect_polygon([0.0, 0.0], [1.0, 1.0], 0.0);
    let mask = polygons_to_profile(vec![pad.clone(), pad.clone()], None);
    let copper = polygons_to_profile(vec![pad], None);
    let findings = crate::checks::solder_mask_opening_ratio_readiness(
        "F.Cu",
        &copper,
        "F.Mask",
        &mask,
        &crate::scalar::scalar("0.9"),
        &crate::scalar::scalar("1.1"),
        &Scalar::zero(),
    );
    assert!(findings.is_empty(), "{findings:?}");
}

#[test]
fn duplicate_apertures_cannot_claim_to_be_a_windowpane() {
    let pad = rect_polygon([0.0, 0.0], [2.0, 2.0], 0.0);
    let paste = polygons_to_profile(vec![pad.clone(), pad.clone()], None);
    let copper = polygons_to_profile(vec![pad], None);
    let findings = crate::checks::thermal_pad_paste_windowpane_readiness(
        "F.Paste",
        &paste,
        "F.Cu",
        &copper,
        &Scalar::one(),
        &crate::scalar::scalar("0.8"),
        &Scalar::zero(),
    );
    assert_eq!(findings.len(), 1, "{findings:?}");
    assert_eq!(findings[0].check, "thermal-pad-paste-windowpane-readiness");
}

#[test]
fn duplicate_paste_cannot_create_a_tombstone_imbalance() {
    let left = rect_polygon([0.0, 0.0], [1.0, 1.0], 0.0);
    let right = rect_polygon([1.25, 0.0], [1.0, 1.0], 0.0);
    let paste = polygons_to_profile(vec![left.clone(), left.clone(), right.clone()], None);
    let copper = polygons_to_profile(vec![left, right], None);
    let findings = crate::checks::tombstone_paste_imbalance_readiness(
        "F.Paste",
        &paste,
        "F.Cu",
        &copper,
        &Scalar::from(2),
        &crate::scalar::scalar("0.1"),
        &Scalar::zero(),
    );
    assert!(findings.is_empty(), "{findings:?}");
}

#[test]
fn report_area_gate_uses_native_circle_independently_of_sampling() {
    for samples in [17, 48, 128] {
        let polygon = crate::geometry::circle_polygon([0.0, 0.0], 1.0, samples);
        let shapes = crate::geometry::multipolygon_to_shapes_scalar(
            &MultiPolygon(vec![polygon]),
            &crate::scalar::scalar("3.13"),
        )
        .unwrap();
        assert_eq!(shapes.len(), 1);
        assert_eq!(shapes[0].exterior.len(), samples + 1);
    }
}

#[test]
fn area_failures_become_blocking_readiness_receipts() {
    let mut findings = Vec::new();
    let coverage = crate::ReadinessRunner::new([crate::Check::LayerSanity])
        .run(&mut findings, |_, sink| {
            sink.extend(crate::checks::tiny_layer_feature_readiness(
                "F.Cu",
                &unavailable_profile(),
                &Scalar::one(),
            ));
            Ok::<_, std::convert::Infallible>(crate::CheckRunDisposition::Executed)
        })
        .unwrap();
    assert_eq!(
        coverage.checks[0].status,
        crate::CheckExecutionStatus::Uncertain
    );
    assert_eq!(coverage.blocking_count(), 1);
    uncertainty(&findings);
}

#[test]
fn required_aperture_measurements_do_not_skip_unavailable_integrals() {
    let region = unavailable_profile();
    let checks = [
        crate::checks::minimum_paste_aperture(
            "F.Paste",
            &region,
            &Scalar::from(10),
            &Scalar::zero(),
        ),
        crate::checks::minimum_mask_opening("F.Mask", &region, &Scalar::from(10), &Scalar::zero()),
        crate::checks::stencil_area_ratio_readiness(
            "F.Paste",
            &region,
            &Scalar::one(),
            &Scalar::one(),
            &Scalar::zero(),
        ),
        crate::checks::paste_aperture_aspect_ratio_readiness(
            "F.Paste",
            &region,
            &Scalar::one(),
            &Scalar::zero(),
        ),
    ];
    for findings in checks {
        uncertainty(&findings);
    }
}

#[test]
fn unknown_area_limits_stay_uncertain_inside_preview() {
    let sine = Scalar::e().sin();
    let cosine = Scalar::e().cos();
    let zero =
        crate::test_support::terminal_zero(&sine * &sine + &cosine * &cosine - Scalar::one());
    let region = polygons_to_profile(vec![rect_polygon([0.0, 0.0], [1.0, 1.0], 0.0)], None);
    let query = || {
        uncertainty(&crate::checks::layer_sanity(
            "F.Cu",
            &region,
            Some(&(Scalar::one() + &zero)),
        ));
        uncertainty(&crate::checks::tiny_layer_feature_readiness(
            "F.Cu",
            &region,
            &(Scalar::one() + &zero),
        ));
        uncertainty(&crate::checks::copper_balance(
            &[
                ("F.Cu".into(), region.clone()),
                ("B.Cu".into(), region.clone()),
            ],
            &(Scalar::one() + &zero),
            &Scalar::zero(),
        ));
    };
    query();
    hypercurve::CurvePreviewOptions::try_approximate_512(1.0, 1.0)
        .unwrap()
        .evaluate(query);
}

#[test]
fn report_filter_cannot_publish_partial_shapes_or_nonfinite_output() {
    let valid = rect_polygon([0.0, 0.0], [1.0, 1.0], 0.0);
    assert!(
        crate::geometry::multipolygon_to_shapes_scalar(
            &MultiPolygon(vec![valid, unavailable_area()]),
            &Scalar::zero()
        )
        .is_err()
    );
    let huge = rect_polygon([0.0, 0.0], [1.0e200, 1.0e200], 0.0);
    assert!(huge.try_area().is_ok());
    let error =
        crate::geometry::multipolygon_to_shapes_scalar(&MultiPolygon(vec![huge]), &Scalar::zero())
            .unwrap_err();
    assert_eq!(error.operation, "report-area-filter");
    assert!(error.detail.contains("not finite"));
}

#[test]
fn area_cache_and_report_queries_reject_exhaustion_then_retry() {
    for warm in [false, true] {
        let polygon = rect_polygon([0.0, 0.0], [1.0, 1.0], 0.0);
        let view = MultiPolygon(vec![polygon.clone()]);
        if warm {
            polygon.try_area().unwrap();
            crate::geometry::multipolygon_to_shapes_scalar(&view, &Scalar::zero()).unwrap();
        }
        let result = hypercurve::evaluate_with_budget(
            PREDICATE_POLICY,
            hypercurve::WorkBudget::ZERO,
            || {
                deny_refinement();
                assert!(polygon.try_area().is_err());
                assert!(crate::geometry::multipolygon_area_scalar(&view).is_err());
                assert!(
                    crate::geometry::multipolygon_to_shapes_scalar(&view, &Scalar::zero()).is_err()
                );
                assert!(
                    crate::geometry::multipolygon_to_shapes_scalar(
                        &MultiPolygon(vec![]),
                        &Scalar::zero()
                    )
                    .is_err()
                );
                assert!(crate::checks::combine_regions(vec![]).is_err());
            },
        );
        assert!(result.result.is_err());
        assert_eq!(polygon.try_area().unwrap(), Scalar::one());
        assert_eq!(
            crate::geometry::multipolygon_to_shapes_scalar(&view, &Scalar::zero())
                .unwrap()
                .len(),
            1
        );
    }
}

#[test]
fn area_collection_regularizes_overlap_and_preserves_holes() {
    let exterior = rect_polygon([0.0, 0.0], [4.0, 4.0], 0.0);
    let hole = rect_polygon([0.0, 0.0], [2.0, 2.0], 0.0);
    let annulus = Polygon::new(exterior.exterior().clone(), vec![hole.exterior().clone()]);
    let island = rect_polygon([0.0, 0.0], [1.0, 1.0], 0.0);
    let view = MultiPolygon(vec![annulus.clone(), annulus, island]);
    assert_eq!(
        crate::geometry::multipolygon_area_scalar(&view).unwrap(),
        Scalar::from(13)
    );
    assert_eq!(
        crate::geometry::multipolygon_area_scalar(&MultiPolygon(vec![])).unwrap(),
        Scalar::zero()
    );
    let failed = crate::geometry::empty_profile(None)
        .with_exact_construction_error("coverage source failure");
    assert!(crate::checks::combine_regions(vec![failed]).is_err());
}

#[test]
fn concurrent_preview_and_denied_area_queries_remain_independent() {
    let polygon = crate::geometry::circle_polygon([0.0, 0.0], 1.0, 17);
    let start = std::sync::Barrier::new(4);
    std::thread::scope(|scope| {
        for worker in 0..4 {
            let (polygon, start) = (&polygon, &start);
            scope.spawn(move || {
                start.wait();
                for _ in 0..8 {
                    if worker % 2 == 0 {
                        let denied = hypercurve::evaluate_with_budget(
                            PREDICATE_POLICY,
                            hypercurve::WorkBudget::ZERO,
                            || {
                                deny_refinement();
                                assert!(polygon.try_area().is_err());
                                assert!(
                                    crate::geometry::multipolygon_to_shapes_scalar(
                                        &MultiPolygon(vec![polygon.clone()]),
                                        &Scalar::zero()
                                    )
                                    .is_err()
                                );
                            },
                        );
                        assert!(denied.result.is_err());
                    } else {
                        hypercurve::CurvePreviewOptions::try_approximate_512(1.0, 1.0)
                            .unwrap()
                            .evaluate(|| {
                                assert_eq!(
                                    crate::scalar::try_compare(
                                        &polygon.try_area().unwrap(),
                                        &Scalar::pi()
                                    )
                                    .unwrap(),
                                    std::cmp::Ordering::Equal
                                );
                                assert_eq!(
                                    crate::geometry::multipolygon_to_shapes_scalar(
                                        &MultiPolygon(vec![polygon.clone()]),
                                        &Scalar::zero()
                                    )
                                    .unwrap()
                                    .len(),
                                    1
                                );
                            });
                    }
                }
            });
        }
    });
}

#[test]
fn one_aperture_split_by_a_copper_hole_remains_one_physical_opening() {
    let outer = rect_polygon([0.0, 0.0], [4.0, 4.0], 0.0);
    let hole = rect_polygon([0.0, 0.0], [2.0, 2.0], 0.0);
    let copper = polygons_to_profile(
        vec![Polygon::new(
            outer.exterior().clone(),
            vec![hole.exterior().clone()],
        )],
        None,
    );
    let paste = polygons_to_profile(vec![rect_polygon([0.0, 0.0], [4.0, 1.0], 0.0)], None);
    let findings = crate::checks::thermal_pad_paste_windowpane_readiness(
        "F.Paste",
        &paste,
        "F.Cu",
        &copper,
        &Scalar::one(),
        &crate::scalar::scalar("0.1"),
        &Scalar::zero(),
    );
    assert_eq!(findings.len(), 1, "{findings:?}");
    assert_eq!(findings[0].check, "thermal-pad-paste-windowpane-readiness");
}
