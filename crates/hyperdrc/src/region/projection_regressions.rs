use crate::geometry::{polygons_to_profile, rect_polygon};
use crate::scalar::scalar;
use crate::*;

fn square() -> PcbRegion {
    polygons_to_profile(vec![rect_polygon([0.5, 0.5], [1.0, 1.0], 0.0)], None)
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
fn failed_source_construction_cannot_publish_components() {
    let failed = square().with_exact_construction_error("retained source fixture");
    assert!(failed.exact_component_regions().is_err());
}

#[test]
fn denied_component_extraction_is_retryable() {
    let region = square();
    let mut during = None;
    let denied =
        hypercurve::evaluate_with_budget(PREDICATE_POLICY, hypercurve::WorkBudget::ZERO, || {
            deny_refinement();
            during = Some(region.exact_component_regions().map(<[_]>::len).ok());
        });
    assert!(denied.result.is_err());
    assert_eq!(during, Some(None));
    assert_eq!(
        region.exact_component_regions().map(<[_]>::len).ok(),
        Some(1)
    );
}

#[test]
fn warm_component_cache_cannot_bypass_an_exhausted_scope() {
    let region = square();
    assert_eq!(
        region.exact_component_regions().map(<[_]>::len).ok(),
        Some(1)
    );
    let mut during = None;
    let denied =
        hypercurve::evaluate_with_budget(PREDICATE_POLICY, hypercurve::WorkBudget::ZERO, || {
            deny_refinement();
            during = Some(region.exact_component_regions().map(<[_]>::len).ok());
        });
    assert!(denied.result.is_err());
    assert_eq!(during, Some(None));
}

#[test]
fn native_projection_retains_certified_component_identity() {
    let region = PcbRegion::new(
        geometry::circle_region(&[Scalar::zero(), Scalar::zero()], &Scalar::one()).unwrap(),
        None,
    );
    let projected = region.try_to_multipolygon().unwrap();
    assert_eq!(projected.0.len(), 1);
    assert_eq!(
        crate::scalar::compare(projected.0[0].exact_area().unwrap(), &Scalar::pi()),
        Some(std::cmp::Ordering::Equal)
    );
    assert!(std::sync::Arc::ptr_eq(
        &projected.0[0].shared_exact_region(),
        &region.exact_component_regions().unwrap()[0]
    ));
}

#[test]
fn ordinary_finite_projection_failure_does_not_unwind() {
    let region = PcbRegion::new(
        geometry::circle_region(&[scalar("1e400"), Scalar::zero()], &Scalar::one()).unwrap(),
        None,
    );
    let attempted = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        region.try_to_multipolygon()
    }));
    let error = attempted
        .expect("ordinary projection failure must not unwind")
        .unwrap_err();
    assert_eq!(error.operation, "report-projection");
    assert!(error.detail.contains("not finite"));
}

fn native_circle(radius: Scalar) -> PcbRegion {
    PcbRegion::new(
        geometry::circle_region(&[Scalar::zero(), Scalar::zero()], &radius).unwrap(),
        Some(LayerMetadata {
            name: "native circles".into(),
        }),
    )
}

#[test]
fn authored_report_evidence_keeps_duplicate_profiles_and_sample_counts() {
    let polygon = geometry::circle_polygon([0.0, 0.0], 1.0, 17);
    let vertex_count = polygon.exterior().0.len();
    let region = polygons_to_profile(vec![polygon.clone(), polygon], None);
    assert_eq!(region.exact_component_regions().unwrap().len(), 1);
    let report = region.try_to_multipolygon().unwrap();
    assert_eq!(report.0.len(), 2);
    assert!(
        report
            .0
            .iter()
            .all(|polygon| polygon.exterior().0.len() == vertex_count)
    );
    let failed = region.with_exact_construction_error("source failed after report cache warmed");
    let error = failed.try_to_multipolygon().unwrap_err();
    assert!(error.detail.contains("source failed"));
}

#[test]
fn report_cache_denial_is_retryable_for_authored_cold_and_warm_generated_views() {
    for (region, warm) in [
        (square(), false),
        (native_circle(Scalar::one()), false),
        (native_circle(Scalar::one()), true),
    ] {
        if warm {
            assert_eq!(region.try_to_multipolygon().unwrap().0.len(), 1);
        }
        let was_cached = region.finite_projection.get().is_some();
        let denied = hypercurve::evaluate_with_budget(
            PREDICATE_POLICY,
            hypercurve::WorkBudget::ZERO,
            || {
                deny_refinement();
                let error = region.try_to_multipolygon().unwrap_err();
                assert!(error.detail.contains("budget exhausted"));
                assert_eq!(region.finite_projection.get().is_some(), was_cached);
            },
        );
        assert!(denied.result.is_err());
        assert_eq!(region.try_to_multipolygon().unwrap().0.len(), 1);
    }
}

#[test]
fn projection_failure_is_not_cached_and_mutation_invalidates_components_and_views() {
    let mut region = native_circle(Scalar::one());
    let original = region.try_to_multipolygon().unwrap();
    let clone = region.clone();
    region.replace_native(
        geometry::circle_region(&[Scalar::zero(), Scalar::zero()], &Scalar::from(2)).unwrap(),
    );
    assert!(region.finite_projection.get().is_none());
    assert!(region.exact_component_regions.get().is_none());
    let changed = region.try_to_multipolygon().unwrap();
    assert_eq!(
        crate::scalar::compare(
            changed.0[0].exact_area().unwrap(),
            &(Scalar::from(4) * Scalar::pi())
        ),
        Some(std::cmp::Ordering::Equal)
    );
    assert!(std::sync::Arc::ptr_eq(
        &original.0[0].shared_exact_region(),
        &clone.try_to_multipolygon().unwrap().0[0].shared_exact_region()
    ));
    region.replace_native(
        geometry::circle_region(&[scalar("1e400"), Scalar::zero()], &Scalar::one()).unwrap(),
    );
    assert!(region.try_to_multipolygon().is_err());
    assert!(region.finite_projection.get().is_none());
    region.replace_native(hypercurve::CurveRegion2::empty());
    assert!(region.try_to_multipolygon().unwrap().0.is_empty());
}

#[test]
fn generated_report_retains_hole_and_nested_material_identity() {
    let ring = native_circle(Scalar::from(3))
        .try_difference(&native_circle(Scalar::from(2)))
        .unwrap();
    let region = ring.try_union(&native_circle(Scalar::one())).unwrap();
    let components = region.exact_component_regions().unwrap();
    assert_eq!(components.len(), 2);
    let view = region.try_to_multipolygon().unwrap();
    assert_eq!(
        view.0
            .iter()
            .map(|polygon| polygon.interiors().len())
            .sum::<usize>(),
        1
    );
    for (polygon, component) in view.0.iter().zip(components) {
        assert!(std::sync::Arc::ptr_eq(
            &polygon.shared_exact_region(),
            component
        ));
    }
    assert_eq!(
        crate::scalar::compare(
            &geometry::multipolygon_area_scalar(&view).unwrap(),
            &(Scalar::from(6) * Scalar::pi())
        ),
        Some(std::cmp::Ordering::Equal)
    );
}

#[test]
fn generated_quadratic_report_retains_native_curve_authority() {
    use hypercurve::{
        Curve2, CurvePath2, CurveRegionLoopRole, FillRule, LineSeg2, Point2, QuadraticBezier2,
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
    let native = hypercurve::CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[path],
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
    )
    .unwrap();
    let region = PcbRegion::new(native, None);
    let components = region.exact_component_regions().unwrap();
    let view = region.try_to_multipolygon().unwrap();
    assert_eq!(view.0.len(), 1);
    assert!(std::sync::Arc::ptr_eq(
        &view.0[0].shared_exact_region(),
        &components[0]
    ));
    assert!(
        view.0[0]
            .try_exact_region()
            .unwrap()
            .native_contours_fast_path()
            .unwrap()
            .is_none()
    );
}

#[test]
fn concurrent_preview_and_denied_queries_keep_independent_scope_receipts() {
    let region = native_circle(Scalar::one());
    std::thread::scope(|scope| {
        for denied in [true, false, true, false] {
            let region = &region;
            scope.spawn(move || {
                if denied {
                    let result = hypercurve::evaluate_with_budget(
                        PREDICATE_POLICY,
                        hypercurve::WorkBudget::ZERO,
                        || {
                            deny_refinement();
                            assert!(region.try_to_multipolygon().is_err());
                            assert!(region.exact_component_regions().is_err());
                        },
                    );
                    assert!(result.result.is_err());
                } else {
                    hypercurve::CurvePreviewOptions::try_approximate_512(1.0, 1.0)
                        .unwrap()
                        .evaluate(|| {
                            let view = region.try_to_multipolygon().unwrap();
                            assert_eq!(
                                crate::scalar::compare(
                                    view.0[0].exact_area().unwrap(),
                                    &Scalar::pi()
                                ),
                                Some(std::cmp::Ordering::Equal)
                            );
                            assert!(std::sync::Arc::ptr_eq(
                                &view.0[0].shared_exact_region(),
                                &region.exact_component_regions().unwrap()[0]
                            ));
                        });
                }
            });
        }
    });
}

#[test]
fn projection_uncertainty_reaches_readiness_with_requested_check_and_source() {
    let region = PcbRegion::new(
        geometry::circle_region(&[scalar("1e400"), Scalar::zero()], &Scalar::one()).unwrap(),
        Some(LayerMetadata {
            name: "large-coordinate source".into(),
        }),
    );
    let mut findings = Vec::new();
    let coverage = ReadinessRunner::new([Check::LayerSanity])
        .run(&mut findings, |_, findings| {
            findings.extend(checks::layer_sanity("reviewed layer", &region, None));
            Ok::<_, ()>(CheckRunDisposition::Executed)
        })
        .unwrap();
    assert_eq!(findings.len(), 1);
    assert_eq!(findings[0].check, "geometry-uncertainty");
    assert_eq!(findings[0].layers, ["reviewed layer"]);
    let message = findings[0].message.as_deref().unwrap();
    assert!(message.contains("layer-sanity") && message.contains("large-coordinate source"));
    assert_eq!(coverage.checks[0].status, CheckExecutionStatus::Uncertain);
    assert!(!coverage.is_complete());
}
