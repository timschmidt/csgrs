use crate::geometry::{Coord, LineString, Polygon, polygons_to_profile, rect_polygon};
use crate::{PREDICATE_POLICY, PcbRegion, PcbRegionExt, Scalar};

fn square() -> PcbRegion {
    polygons_to_profile(vec![rect_polygon([0.0, 0.0], [2.0, 1.0], 0.0)], None)
}
fn failed_empty() -> PcbRegion {
    crate::geometry::empty_profile(None)
        .with_exact_construction_error("retained empty placeholder failure")
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
fn quadratic() -> Polygon<f64> {
    use hypercurve::{
        Curve2, CurvePath2, CurveRegion2, CurveRegionLoopRole, FillRule, LineSeg2, Point2,
        QuadraticBezier2,
    };
    let path = CurvePath2::try_new(vec![
        Curve2::from(QuadraticBezier2::new(
            Point2::from_values(0, 0),
            Point2::from_values(1, 2),
            Point2::from_values(2, 0),
        )),
        Curve2::from(
            LineSeg2::try_new(Point2::from_values(2, 0), Point2::from_values(0, 0)).unwrap(),
        ),
    ])
    .unwrap();
    let native = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[path],
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
    )
    .unwrap();
    Polygon::from_exact_region(
        native,
        LineString(vec![
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 1.0, y: 1.0 },
            Coord { x: 2.0, y: 0.0 },
            Coord { x: 0.0, y: 0.0 },
        ]),
        vec![],
    )
}

#[test]
fn failed_native_area_cannot_publish_a_placeholder_value() {
    assert!(failed_empty().try_area().is_err());
}
#[test]
fn failed_native_bounds_cannot_publish_placeholder_bounds() {
    assert!(failed_empty().try_bounding_box().is_err());
}
#[test]
fn failed_region_cannot_claim_to_be_empty() {
    assert!(failed_empty().try_is_empty().is_err());
}
#[test]
fn failed_polygon_cannot_expose_unchecked_native_geometry() {
    let polygon = rect_polygon([0.0, 0.0], [1.0, 1.0], 0.0)
        .with_exact_construction_error("retained polygon failure");
    assert!(polygon.try_exact_region().is_err());
}
#[test]
fn single_quadratic_profile_keeps_its_native_geometry() {
    let polygon = quadratic();
    let native = polygon.shared_exact_region();
    let profile = polygons_to_profile(vec![polygon], None);
    assert!(std::sync::Arc::ptr_eq(&profile.region, &native));
    assert!(profile.try_to_multipolygon().is_ok());
    assert_eq!(
        profile.try_area().unwrap(),
        (Scalar::from(4) / Scalar::from(3)).unwrap()
    );
}
#[test]
fn mixed_native_profile_aggregation_retains_supported_curve_families() {
    let profile = polygons_to_profile(
        vec![quadratic(), rect_polygon([5.0, 0.0], [1.0, 1.0], 0.0)],
        None,
    );
    assert!(profile.try_to_multipolygon().is_ok());
    assert_eq!(
        profile.try_area().unwrap(),
        (Scalar::from(7) / Scalar::from(3)).unwrap()
    );
}
#[test]
fn exhausted_profile_construction_cannot_escape_as_success() {
    let polygon = rect_polygon([0.0, 0.0], [1.0, 1.0], 0.0);
    let mut constructed = None;
    let denied =
        hypercurve::evaluate_with_budget(PREDICATE_POLICY, hypercurve::WorkBudget::ZERO, || {
            deny_refinement();
            constructed = Some(polygons_to_profile(vec![polygon.clone()], None));
        });
    assert!(denied.result.is_err());
    assert!(constructed.unwrap().try_to_multipolygon().is_err());
    assert!(
        polygons_to_profile(vec![polygon], None)
            .try_to_multipolygon()
            .is_ok()
    );
}
#[test]
fn source_failure_is_not_an_ordinary_net_length_measurement() {
    use crate::constraint_policy::NetClassConfig;
    use crate::kicad::{BoardModel, CopperFeature, CopperKind};
    let board = BoardModel {
        source: "length source".into(),
        copper: vec![CopperFeature {
            layer: "F.Cu".into(),
            net: Some("SIGNAL".into()),
            kind: CopperKind::Segment,
            region: square().with_exact_construction_error("retained length failure"),
            location: [Scalar::zero(), Scalar::zero()],
        }],
        drills: vec![],
        board_outline: None,
        panel_features: None,
    };
    let class = NetClassConfig {
        name: "length class".into(),
        nets: vec!["SIGNAL".into()],
        max_length: Some(Scalar::one()),
        ..NetClassConfig::default()
    };
    let findings = crate::checks::net_constraint_readiness(&[class], None, &[board], &[]);
    assert!(
        findings
            .iter()
            .any(|finding| finding.check == "geometry-uncertainty"),
        "{findings:?}"
    );
}

#[test]
fn native_handoff_preserves_geometry_and_rejects_failed_sources() {
    let polygon = quadratic();
    let expected = polygon.try_exact_region().unwrap().clone();
    let region = polygons_to_profile(vec![polygon], None);
    assert_eq!(region.try_into_native().unwrap(), expected);
    let failed = PcbRegion::new(
        hypercurve::CurveRegion2::empty(),
        Some(crate::LayerMetadata {
            name: "F.Cu".into(),
        }),
    )
    .with_exact_construction_error("handoff source sentinel");
    for error in [
        failed.try_area().unwrap_err(),
        failed.try_is_empty().unwrap_err(),
        failed.try_into_native().unwrap_err(),
    ] {
        assert_eq!(error.source.as_deref(), Some("F.Cu"));
        assert!(error.detail.contains("handoff source sentinel"));
    }
}

#[test]
fn native_replacement_clears_old_facts_without_changing_clones_or_metadata() {
    let mut region = polygons_to_profile(
        vec![rect_polygon([0.0, 0.0], [2.0, 1.0], 0.0)],
        Some(crate::LayerMetadata {
            name: "F.Cu".into(),
        }),
    );
    let old_native = region.region.clone();
    let old_view = region.try_to_multipolygon().unwrap();
    region.exact_component_regions().unwrap();
    let unchanged = region.clone();
    region.had_non_finite_input = true;
    region = region.with_exact_construction_error("discarded input failure");
    let failed_clone = region.clone();
    region.replace_native(
        crate::geometry::circle_region(&[Scalar::zero(), Scalar::zero()], &Scalar::one()).unwrap(),
    );
    assert_eq!(region.metadata().as_ref().unwrap().name, "F.Cu");
    assert!(!region.had_non_finite_input());
    assert!(region.exact_construction_error().is_none());
    assert!(region.exact_bounds().is_none());
    assert!(region.finite_projection.get().is_none());
    assert!(region.exact_component_regions.get().is_none());
    assert_eq!(
        crate::scalar::compare(&region.try_area().unwrap(), &Scalar::pi()),
        Some(std::cmp::Ordering::Equal)
    );
    assert!(std::sync::Arc::ptr_eq(&unchanged.region, &old_native));
    assert_eq!(unchanged.try_to_multipolygon().unwrap(), old_view);
    assert_eq!(unchanged.try_area().unwrap(), Scalar::from(2));
    assert!(failed_clone.try_area().is_err());
}

#[test]
fn genuine_empty_queries_and_handoff_need_no_refinement() {
    let result =
        hypercurve::evaluate_with_budget(PREDICATE_POLICY, hypercurve::WorkBudget::ZERO, || {
            let empty = polygons_to_profile(vec![], None);
            assert!(empty.try_is_empty().unwrap());
            assert_eq!(empty.try_area().unwrap(), Scalar::zero());
            assert!(empty.try_into_native().unwrap().is_empty());
        });
    assert!(result.result.is_ok());
    assert_eq!(result.usage, hypercurve::WorkUsage::default());
}

#[test]
fn cold_and_warm_handoffs_reject_exhaustion_and_allow_retry() {
    for warm in [false, true] {
        let polygon = quadratic();
        let region = polygons_to_profile(vec![polygon.clone()], None);
        if warm {
            region.try_area().unwrap();
            region.try_is_empty().unwrap();
            region.try_to_multipolygon().unwrap();
            region.clone().try_into_native().unwrap();
        }
        let result = hypercurve::evaluate_with_budget(
            PREDICATE_POLICY,
            hypercurve::WorkBudget::ZERO,
            || {
                deny_refinement();
                assert!(
                    region
                        .try_area()
                        .unwrap_err()
                        .detail
                        .contains("budget exhausted")
                );
                assert!(region.try_is_empty().is_err());
                assert!(region.clone().try_into_native().is_err());
                assert!(polygon.try_exact_region().is_err());
                assert!(polygons_to_profile(vec![], None).try_is_empty().is_err());
            },
        );
        assert!(result.result.is_err());
        assert_eq!(
            region.try_area().unwrap(),
            (Scalar::from(4) / Scalar::from(3)).unwrap()
        );
        assert!(!region.try_is_empty().unwrap());
        assert!(polygon.try_exact_region().is_ok());
        assert!(region.try_into_native().is_ok());
    }
}

#[test]
fn mixed_aggregation_regularizes_overlap_duplicates_and_containment() {
    let cap = quadratic();
    let overlap = rect_polygon([0.5, 0.5], [1.0, 1.0], 0.0);
    let region = polygons_to_profile(vec![cap.clone(), overlap, cap.clone()], None);
    assert_eq!(
        region.try_area().unwrap(),
        (Scalar::from(5) / Scalar::from(3)).unwrap()
    );
    assert_eq!(
        region.try_to_multipolygon().unwrap().0.len(),
        3,
        "authored report decomposition remains available"
    );
    let contained = polygons_to_profile(vec![cap, rect_polygon([1.0, 0.5], [2.0, 1.0], 0.0)], None);
    assert_eq!(contained.try_area().unwrap(), Scalar::from(2));
}

#[test]
fn aggregation_is_independent_of_report_coordinates() {
    let cap = quadratic();
    let alternate = Polygon::from_shared_exact_region(
        cap.shared_exact_region(),
        rect_polygon([100.0, 100.0], [1.0, 1.0], 0.0)
            .exterior()
            .clone(),
        vec![],
    );
    for polygon in [cap, alternate] {
        let region = polygons_to_profile(
            vec![polygon, rect_polygon([5.0, 0.0], [1.0, 1.0], 0.0)],
            None,
        );
        assert_eq!(
            region.try_area().unwrap(),
            (Scalar::from(7) / Scalar::from(3)).unwrap()
        );
        assert_eq!(
            region
                .classify_xy(crate::scalar::scalar("0.5"), crate::scalar::scalar("0.7"))
                .unwrap(),
            hypercurve::RegionPointLocation::Inside
        );
        assert_eq!(
            region.classify_xy(100.into(), 100.into()).unwrap(),
            hypercurve::RegionPointLocation::Outside
        );
    }
}

#[test]
fn aggregation_does_not_skip_invalid_inputs_on_any_family_path() {
    let failed = rect_polygon([9.0, 0.0], [1.0, 1.0], 0.0)
        .with_exact_construction_error("bad aggregate member");
    for polygons in [
        vec![failed.clone()],
        vec![quadratic(), failed.clone()],
        vec![failed, quadratic()],
    ] {
        let region = polygons_to_profile(polygons, None);
        assert!(
            region
                .try_is_empty()
                .unwrap_err()
                .detail
                .contains("bad aggregate member")
        );
        assert!(region.try_to_multipolygon().is_err());
        assert!(region.try_into_native().is_err());
    }
}

#[test]
fn preview_and_exhausted_threads_cannot_change_handoff_or_aggregation_policy() {
    let polygon = quadratic();
    let region = polygons_to_profile(vec![polygon.clone()], None);
    let start = std::sync::Barrier::new(4);
    std::thread::scope(|scope| {
        for worker in 0..4 {
            let (polygon, region, start) = (&polygon, &region, &start);
            scope.spawn(move || {
                start.wait();
                for _ in 0..8 {
                    if worker % 2 == 0 {
                        let denied = hypercurve::evaluate_with_budget(
                            PREDICATE_POLICY,
                            hypercurve::WorkBudget::ZERO,
                            || {
                                deny_refinement();
                                assert!(region.try_area().is_err());
                                assert!(region.try_is_empty().is_err());
                                assert!(region.clone().try_into_native().is_err());
                                assert!(polygon.try_exact_region().is_err());
                                assert!(
                                    polygons_to_profile(vec![polygon.clone()], None)
                                        .try_area()
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
                                    region.try_area().unwrap(),
                                    (Scalar::from(4) / Scalar::from(3)).unwrap()
                                );
                                assert!(!region.try_is_empty().unwrap());
                                assert_eq!(
                                    region.clone().try_into_native().unwrap(),
                                    *polygon.try_exact_region().unwrap()
                                );
                                assert_eq!(
                                    polygons_to_profile(vec![polygon.clone()], None)
                                        .try_area()
                                        .unwrap(),
                                    region.try_area().unwrap()
                                );
                            });
                    }
                }
            });
        }
    });
}

#[test]
fn failed_contributions_do_not_publish_partial_net_or_pair_lengths() {
    use crate::constraint_policy::{DifferentialRole, NetClassConfig};
    use crate::kicad::{BoardModel, CopperFeature, CopperKind};
    let feature = |net: &str, region| CopperFeature {
        layer: "F.Cu".into(),
        net: Some(net.into()),
        kind: CopperKind::Segment,
        region,
        location: [Scalar::zero(), Scalar::zero()],
    };
    let board = BoardModel {
        source: "partial lengths".into(),
        copper: vec![
            feature("P", square()),
            feature("P", failed_empty()),
            feature("N", square()),
            feature("N", square()),
        ],
        drills: vec![],
        board_outline: None,
        panel_features: None,
    };
    let classes = [
        ("P", DifferentialRole::Positive),
        ("N", DifferentialRole::Negative),
    ]
    .map(|(net, role)| NetClassConfig {
        name: net.into(),
        nets: vec![net.into()],
        differential_pair: Some("PAIR".into()),
        differential_role: Some(role),
        max_length: (net == "P").then(Scalar::one),
        max_pair_skew: Some(Scalar::one()),
        ..NetClassConfig::default()
    });
    let findings = crate::checks::net_constraint_readiness(&classes, None, &[board], &[]);
    assert_eq!(findings.len(), 1, "{findings:?}");
    assert_eq!(findings[0].check, "geometry-uncertainty");
}
