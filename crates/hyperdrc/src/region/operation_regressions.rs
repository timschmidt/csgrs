use crate::geometry::{circle_polygon, polygons_to_profile, rect_polygon};
use crate::{PREDICATE_POLICY, PcbRegion, Scalar};

fn square() -> PcbRegion {
    polygons_to_profile(vec![rect_polygon([2.0, 3.0], [1.0, 1.0], 0.0)], None)
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
macro_rules! denied_boolean {
    ($name:ident, $method:ident) => {
        #[test]
        fn $name() {
            let left = square();
            let right = square();
            let result = hypercurve::evaluate_with_budget(
                PREDICATE_POLICY,
                hypercurve::WorkBudget::ZERO,
                || {
                    deny_refinement();
                    assert!(left.$method(&right).is_err());
                },
            );
            assert!(result.result.is_err());
            assert!(left.$method(&right).is_ok());
        }
    };
}
denied_boolean!(denied_union_cannot_publish_geometry, try_union);
denied_boolean!(
    denied_intersection_cannot_publish_geometry,
    try_intersection
);
denied_boolean!(denied_difference_cannot_publish_geometry, try_difference);
denied_boolean!(denied_xor_cannot_publish_geometry, try_xor);

#[test]
fn denied_offset_cannot_publish_geometry() {
    let region = square();
    let result =
        hypercurve::evaluate_with_budget(PREDICATE_POLICY, hypercurve::WorkBudget::ZERO, || {
            deny_refinement();
            assert!(region.offset(Scalar::zero()).is_err());
        });
    assert!(result.result.is_err());
    assert!(region.offset(Scalar::zero()).is_ok());
}
#[test]
fn failed_source_cannot_classify_point() {
    let failed = square().with_exact_construction_error("retained membership source failure");
    assert!(
        failed
            .classify_xy(Scalar::from(2), Scalar::from(3))
            .is_err()
    );
}
#[test]
fn unresolved_circle_construction_does_not_unwind() {
    let sine = Scalar::e().sin();
    let cosine = Scalar::e().cos();
    let radius = &sine * &sine + &cosine * &cosine - Scalar::one();
    let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        crate::geometry::circle_region(&[Scalar::zero(), Scalar::zero()], &(radius))
    }));
    assert!(
        outcome
            .expect("unresolved circle construction must not unwind")
            .is_err()
    );
}
#[test]
fn collapsed_offset_cannot_retain_inferred_nonempty_bounds() {
    let region = polygons_to_profile(vec![circle_polygon([2.0, 3.0], 1.0, 17)], None);
    let collapsed = region.offset(Scalar::from(-2)).unwrap();
    assert!(collapsed.is_empty());
    assert!(collapsed.try_native_bounds().unwrap().is_none());
}
#[test]
fn required_width_cannot_publish_a_placeholder_measurement() {
    use crate::kicad::{BoardModel, CopperFeature, CopperKind};
    let mut board = BoardModel {
        source: "failed-width-fixture".into(),
        copper: vec![],
        drills: vec![],
        board_outline: None,
        panel_features: None,
    };
    board.copper.push(CopperFeature {
        layer: "F.Cu".into(),
        net: Some("VCC".into()),
        kind: CopperKind::Segment,
        region: square().with_exact_construction_error("retained width source failure"),
        location: [2.into(), 3.into()],
    });
    let findings = crate::checks::copper_width_readiness(&board, &[], &Scalar::from(2));
    assert!(
        findings
            .iter()
            .any(|finding| finding.check == "geometry-uncertainty"),
        "{findings:?}"
    );
}

#[test]
fn classification_distinguishes_boundary_empty_failure_and_denial() {
    use hypercurve::RegionPointLocation::{Boundary, Inside, Outside};
    let region = square();
    assert_eq!(region.classify_xy(2.into(), 3.into()).unwrap(), Inside);
    assert_eq!(region.classify_xy(0.into(), 0.into()).unwrap(), Outside);
    assert_eq!(
        region
            .classify_xy(crate::scalar::scalar("2.5"), 3.into())
            .unwrap(),
        Boundary
    );
    let empty = crate::geometry::empty_profile(None);
    assert_eq!(empty.classify_xy(2.into(), 3.into()).unwrap(), Outside);
    assert!(empty.try_bounding_box().unwrap().is_none());
    assert!(region.try_bounding_box().unwrap().is_some());
    let denied =
        hypercurve::evaluate_with_budget(PREDICATE_POLICY, hypercurve::WorkBudget::ZERO, || {
            deny_refinement();
            assert!(region.classify_xy(2.into(), 3.into()).is_err());
            assert!(region.try_bounding_box().is_err());
            assert!(empty.classify_xy(2.into(), 3.into()).is_err());
            assert!(empty.try_bounding_box().is_err());
        });
    assert!(denied.result.is_err());
    let failed = region.with_exact_construction_error("retained source error");
    assert!(failed.try_bounding_box().is_err());
    assert!(failed.offset(Scalar::zero()).is_err());
}

#[test]
fn guarded_booleans_preserve_sets_metadata_and_source_failures() {
    let left = polygons_to_profile(
        vec![rect_polygon([0.0, 0.0], [2.0, 2.0], 0.0)],
        Some(crate::LayerMetadata {
            name: "left source".into(),
        }),
    );
    let right = polygons_to_profile(
        vec![rect_polygon([1.0, 0.0], [2.0, 2.0], 0.0)],
        Some(crate::LayerMetadata {
            name: "right source".into(),
        }),
    );
    for (result, area) in [
        (left.try_union(&right), 6),
        (left.try_intersection(&right), 2),
        (left.try_difference(&right), 2),
        (left.try_xor(&right), 4),
    ] {
        let result = result.unwrap();
        assert_eq!(result.filled_area().unwrap().unwrap(), Scalar::from(area));
        assert_eq!(result.metadata().as_ref().unwrap().name, "left source");
        assert!(result.try_bounding_box().unwrap().is_some());
    }
    let failed = right.with_exact_construction_error("retained right failure");
    for result in [
        left.try_union(&failed),
        left.try_intersection(&failed),
        left.try_difference(&failed),
        left.try_xor(&failed),
        failed.try_union(&left),
    ] {
        let error = result.unwrap_err();
        assert_eq!(error.source.as_deref(), Some("right source"));
        assert!(error.detail.contains("retained right failure"));
    }
}

#[test]
fn eroded_component_cannot_leave_stale_width_bounds() {
    let region = polygons_to_profile(
        vec![
            circle_polygon([0.0, 0.0], 2.0, 17),
            circle_polygon([100.0, 0.0], 0.1, 17),
        ],
        None,
    );
    let eroded = region.offset(crate::scalar::scalar("-0.2")).unwrap();
    let bounds = eroded.try_native_bounds().unwrap().unwrap();
    assert_eq!(
        bounds,
        [
            crate::scalar::scalar("-1.8"),
            crate::scalar::scalar("-1.8"),
            crate::scalar::scalar("1.8"),
            crate::scalar::scalar("1.8")
        ]
    );
}

#[test]
fn enclosing_preview_cannot_resolve_unknown_classification_or_offset() {
    let region = square();
    let sine = Scalar::e().sin();
    let cosine = Scalar::e().cos();
    let zero = &sine * &sine + &cosine * &cosine - Scalar::one();
    let check = || {
        assert!(
            region
                .classify_xy(crate::scalar::scalar("2.5") + &zero, 3.into())
                .is_err()
        );
        assert!(region.offset(zero.clone()).is_err());
    };
    check();
    hypercurve::CurvePreviewOptions::try_approximate_512(1.0, 1.0)
        .unwrap()
        .evaluate(check);
}

#[test]
fn invalid_drill_radius_reaches_the_requested_check_without_panicking() {
    let drill = crate::kicad::DrillFeature {
        location: [Scalar::zero(), Scalar::zero()],
        diameter: Scalar::from(-1),
        plated: false,
        net: None,
    };
    let failure = crate::checks::drill::drills_to_region(
        &[drill],
        "invalid drill source",
        "native-drill-regression",
    )
    .unwrap_err();
    assert_eq!(failure.check, "geometry-uncertainty");
    assert_eq!(failure.layers, ["invalid drill source"]);
    assert!(
        failure
            .message
            .as_ref()
            .unwrap()
            .contains("native-drill-regression")
    );
    assert!(
        failure
            .message
            .as_ref()
            .unwrap()
            .contains("negative circle radius")
    );
}

#[test]
fn failed_envelope_measurements_reach_readiness_and_pad_selection() {
    use crate::kicad::{BoardModel, CopperFeature, CopperKind};
    let board = BoardModel {
        source: "failed-measurements".into(),
        copper: vec![CopperFeature {
            layer: "F.Cu".into(),
            net: Some("VCC".into()),
            kind: CopperKind::Pad,
            region: square().with_exact_construction_error("retained envelope failure"),
            location: [2.into(), 3.into()],
        }],
        drills: vec![],
        board_outline: None,
        panel_features: None,
    };
    let mut findings = Vec::new();
    let coverage = crate::ReadinessRunner::new([crate::Check::CopperWidthReadiness])
        .run(&mut findings, |_, findings| {
            findings.extend(crate::checks::copper_width_readiness(
                &board,
                &[],
                &Scalar::from(2),
            ));
            Ok::<_, ()>(crate::CheckRunDisposition::Executed)
        })
        .unwrap();
    assert_eq!(
        coverage.checks[0].status,
        crate::CheckExecutionStatus::Uncertain
    );
    assert!(!coverage.is_complete());
    for findings in [
        crate::checks::component_spacing_readiness(&board, &[], &Scalar::one(), &Scalar::one()),
        crate::checks::thermal_pad_via_readiness(&board, &[], &Scalar::one()),
    ] {
        assert!(
            findings
                .iter()
                .any(|finding| finding.check == "geometry-uncertainty"),
            "{findings:?}"
        );
    }
}

#[test]
fn concurrent_preview_and_denied_operations_keep_scope_local() {
    let region = square();
    assert!(region.try_bounding_box().unwrap().is_some());
    assert!(region.try_union(&region).is_ok());
    std::thread::scope(|scope| {
        for denied in [false, true, false, true] {
            let region = &region;
            scope.spawn(move || {
                for _ in 0..16 {
                    if denied {
                        let result = hypercurve::evaluate_with_budget(
                            PREDICATE_POLICY,
                            hypercurve::WorkBudget::ZERO,
                            || {
                                deny_refinement();
                                assert!(region.try_union(region).is_err());
                                assert!(region.try_difference(region).is_err());
                                assert!(region.try_intersection(region).is_err());
                                assert!(region.try_xor(region).is_err());
                                assert!(region.offset(Scalar::zero()).is_err());
                                assert!(region.classify_xy(2.into(), 3.into()).is_err());
                                assert!(region.try_bounding_box().is_err());
                            },
                        );
                        assert!(result.result.is_err());
                    } else {
                        hypercurve::CurvePreviewOptions::try_approximate_512(1.0, 1.0)
                            .unwrap()
                            .evaluate(|| {
                                assert_eq!(
                                    region
                                        .try_union(region)
                                        .unwrap()
                                        .filled_area()
                                        .unwrap()
                                        .unwrap(),
                                    Scalar::one()
                                );
                                assert!(region.try_difference(region).unwrap().is_empty());
                                assert!(region.try_intersection(region).is_ok());
                                assert!(region.try_xor(region).unwrap().is_empty());
                                assert!(region.offset(Scalar::zero()).is_ok());
                                assert_eq!(
                                    region.classify_xy(2.into(), 3.into()).unwrap(),
                                    hypercurve::RegionPointLocation::Inside
                                );
                                assert!(region.try_bounding_box().unwrap().is_some());
                            });
                    }
                }
            });
        }
    });
}
