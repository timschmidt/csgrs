use crate::geometry::{
    Coord, LineString, Polygon, polygon_to_profile, polygons_to_profile, rect_polygon,
};
use crate::{PcbRegion, Scalar};

fn unknown_zero() -> Scalar {
    let sine = Scalar::e().sin();
    let cosine = Scalar::e().cos();
    &sine * &sine + &cosine * &cosine - Scalar::one()
}

fn native_polygon(points: Vec<hypercurve::Point2>, report: Vec<[f64; 2]>) -> Polygon<f64> {
    use hypercurve::{Contour2, CurveRegion2, LineSeg2, Segment2};
    let segments = points
        .iter()
        .cloned()
        .zip(points.iter().cloned().cycle().skip(1))
        .map(|(start, end)| Segment2::Line(LineSeg2::try_new(start, end).unwrap()))
        .collect();
    let contour = Contour2::try_new(segments).unwrap();
    let region = CurveRegion2::try_from_native_contours(vec![contour], vec![]).unwrap();
    Polygon::from_exact_region(
        region,
        LineString(report.into_iter().map(|[x, y]| Coord { x, y }).collect()),
        vec![],
    )
}

pub(crate) fn uncertain_bounds_polygon() -> Polygon<f64> {
    use hypercurve::Point2;
    let polygon = native_polygon(
        vec![
            Point2::from_values(0, 0),
            Point2::from_values(1, 0),
            Point2::new(unknown_zero(), 1.into()),
        ],
        vec![[0.0, 0.0], [1.0, 0.0], [0.0, 1.0], [0.0, 0.0]],
    );
    let native = polygon.try_exact_region().unwrap();
    assert!(native.bounds().is_err(), "fixture must lack a STRICT bound");
    let area = polygon.try_area().unwrap();
    assert_eq!(
        crate::scalar::compare(&area, &Scalar::zero()),
        Some(std::cmp::Ordering::Greater)
    );
    polygon
}

fn uncertain_profile() -> PcbRegion {
    polygon_to_profile(uncertain_bounds_polygon(), None)
}
fn uncertainty(findings: &[crate::Violation]) {
    assert!(
        findings.iter().any(|f| f.check == "geometry-uncertainty"),
        "{findings:?}"
    );
}

#[test]
fn minimum_paste_aperture_cannot_skip_unknown_bounds() {
    uncertainty(&crate::checks::minimum_paste_aperture(
        "F.Paste",
        &uncertain_profile(),
        &2.into(),
        &Scalar::zero(),
    ));
}
#[test]
fn minimum_mask_opening_cannot_skip_unknown_bounds() {
    uncertainty(&crate::checks::minimum_mask_opening(
        "F.Mask",
        &uncertain_profile(),
        &2.into(),
        &Scalar::zero(),
    ));
}
#[test]
fn stencil_area_ratio_cannot_skip_unknown_bounds() {
    uncertainty(&crate::checks::stencil_area_ratio_readiness(
        "F.Paste",
        &uncertain_profile(),
        &Scalar::one(),
        &Scalar::one(),
        &Scalar::zero(),
    ));
}
#[test]
fn aperture_aspect_ratio_cannot_skip_unknown_bounds() {
    uncertainty(&crate::checks::paste_aperture_aspect_ratio_readiness(
        "F.Paste",
        &uncertain_profile(),
        &Scalar::one(),
        &Scalar::zero(),
    ));
}
#[test]
fn text_height_cannot_skip_unknown_bounds() {
    uncertainty(&crate::checks::silkscreen_text_height_readiness(
        "F.Silk",
        &uncertain_profile(),
        &2.into(),
        &Scalar::zero(),
    ));
}
#[test]
fn skinny_feature_cannot_skip_unknown_bounds() {
    uncertainty(&crate::checks::skinny_layer_feature_readiness(
        "F.Cu",
        &uncertain_profile(),
        &2.into(),
        &Scalar::zero(),
    ));
}
#[test]
fn duplicate_layers_cannot_skip_unknown_bounds() {
    let profile = uncertain_profile();
    uncertainty(&crate::checks::duplicate_layer_geometry_readiness(
        &[("F.Cu".into(), profile.clone()), ("B.Cu".into(), profile)],
        &Scalar::zero(),
    ));
}
#[test]
fn density_cannot_drop_a_layer_with_unknown_bounds() {
    uncertainty(&crate::checks::local_copper_density_readiness(
        &[
            ("F.Cu".into(), uncertain_profile()),
            (
                "B.Cu".into(),
                polygon_to_profile(rect_polygon([0.0, 0.0], [1.0, 1.0], 0.0), None),
            ),
        ],
        &Scalar::one(),
        &2.into(),
        &Scalar::zero(),
    ));
}
#[test]
fn tombstone_cannot_drop_a_pad_with_unknown_center() {
    let copper = polygons_to_profile(
        vec![
            uncertain_bounds_polygon(),
            rect_polygon([2.0, 0.5], [1.0, 0.5], 0.0),
        ],
        None,
    );
    assert!(
        copper.try_is_empty().is_ok(),
        "fixture construction must succeed"
    );
    uncertainty(&crate::checks::tombstone_paste_imbalance_readiness(
        "F.Paste",
        &crate::geometry::empty_profile(None),
        "F.Cu",
        &copper,
        &3.into(),
        &Scalar::one(),
        &Scalar::zero(),
    ));
}
#[test]
fn required_bounds_helper_cannot_confuse_failure_with_empty() {
    let bounds = crate::geometry::polygon_bounds_scalar(&uncertain_bounds_polygon());
    assert!(bounds.is_err(), "required bounds must retain a typed error");
}
#[test]
fn pcb_bounds_already_retain_the_unknown_order() {
    assert!(uncertain_profile().try_native_bounds().is_err());
}
#[test]
fn skinny_feature_cannot_choose_a_dimension_on_unknown_order() {
    use hypercurve::Point2;
    let height = Scalar::one() + unknown_zero();
    let polygon = native_polygon(
        vec![
            Point2::from_values(0, 0),
            Point2::from_values(1, 0),
            Point2::new(1.into(), height.clone()),
            Point2::new(0.into(), height),
        ],
        vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0], [0.0, 0.0]],
    );
    assert!(polygon.try_exact_region().unwrap().bounds().is_ok());
    uncertainty(&crate::checks::skinny_layer_feature_readiness(
        "F.Cu",
        &polygon_to_profile(polygon, None),
        &2.into(),
        &Scalar::zero(),
    ));
}
#[test]
fn text_height_location_cannot_overflow_a_finite_native_center() {
    let polygon = rect_polygon([1.3e308, 0.0], [4e307, 1e-310], 0.0);
    let profile = polygon_to_profile(polygon, None);
    let findings = crate::checks::silkscreen_text_height_readiness(
        "F.Silk",
        &profile,
        &"1e308".parse().unwrap(),
        &Scalar::zero(),
    );
    assert_eq!(findings.len(), 1, "{findings:?}");
    assert_eq!(findings[0].check, "silkscreen-text-height-readiness");
    assert!(
        findings[0]
            .locations
            .iter()
            .flatten()
            .all(|value| value.is_finite()),
        "{:?}",
        findings[0].locations
    );
}

#[test]
fn duplicate_island_detection_cannot_depend_on_circle_report_samples() {
    let region = polygons_to_profile(
        vec![
            crate::geometry::circle_polygon([0.0, 0.0], 1.0, 17),
            crate::geometry::circle_polygon([0.0, 0.0], 1.0, 128),
        ],
        None,
    );
    let findings =
        crate::checks::duplicate_layer_island_readiness("F.Cu", &region, &Scalar::zero());
    assert_eq!(findings.len(), 1, "{findings:?}");
    assert_eq!(findings[0].check, "layer-sanity");
    assert!(
        findings[0]
            .message
            .as_deref()
            .unwrap()
            .contains("duplicate polygon")
    );
}

#[test]
fn proven_tiny_area_does_not_require_an_optional_native_marker() {
    let findings =
        crate::checks::tiny_layer_feature_readiness("F.Cu", &uncertain_profile(), &Scalar::one());
    assert_eq!(findings.len(), 1, "{findings:?}");
    assert_eq!(findings[0].check, "layer-sanity");
    assert!(
        findings[0]
            .message
            .as_deref()
            .unwrap()
            .contains("reportable feature area")
    );
    assert!(
        findings[0]
            .locations
            .iter()
            .flatten()
            .all(|value| value.is_finite())
    );
}

#[test]
fn proven_duplicate_islands_do_not_require_an_optional_native_marker() {
    let polygon = uncertain_bounds_polygon();
    let region = polygons_to_profile(vec![polygon.clone(), polygon], None);
    let findings =
        crate::checks::duplicate_layer_island_readiness("F.Cu", &region, &Scalar::zero());
    assert_eq!(findings.len(), 1, "{findings:?}");
    assert_eq!(findings[0].check, "layer-sanity");
    assert!(
        findings[0]
            .message
            .as_deref()
            .unwrap()
            .contains("duplicate polygon")
    );
    assert!(
        findings[0]
            .locations
            .iter()
            .flatten()
            .all(|value| value.is_finite())
    );
}

#[test]
fn empty_and_warm_bounds_retain_budget_failure_and_retry() {
    for empty in [false, true] {
        for warm in [false, true] {
            let polygon = if empty {
                Polygon::from_exact_region(
                    hypercurve::CurveRegion2::empty(),
                    LineString(vec![]),
                    vec![],
                )
            } else {
                rect_polygon([0.0, 0.0], [2.0, 2.0], 0.0)
            };
            if warm {
                assert_eq!(polygon.try_bounds().unwrap().is_none(), empty);
            }
            let result = hypercurve::evaluate_with_budget(
                crate::PREDICATE_POLICY,
                hypercurve::WorkBudget::ZERO,
                || {
                    let polynomial = hypercurve::BezierParameterPolynomial::try_new_power_basis(
                        [-36, 1368, -11034, 31728, -38280, 16620]
                            .into_iter()
                            .map(Scalar::from)
                            .collect(),
                    )
                    .unwrap();
                    assert!(polynomial.isolate_unit_interval_roots().is_err());
                    assert!(polygon.try_bounds().is_err());
                    assert!(polygon.conservative_bounds().is_none());
                },
            );
            assert!(result.result.is_err());
            assert_eq!(polygon.try_bounds().unwrap().is_none(), empty);
        }
    }
}

#[test]
fn enclosing_preview_cannot_install_approximate_polygon_bounds() {
    let polygon = uncertain_bounds_polygon();
    hypercurve::CurvePreviewOptions::try_approximate_512(1.0, 1.0)
        .unwrap()
        .evaluate(|| {
            assert!(polygon.try_bounds().is_err());
            assert!(polygon.conservative_bounds().is_none());
        });
    assert!(polygon.try_bounds().is_err());
}

#[test]
fn required_bounds_failure_becomes_a_blocking_readiness_receipt() {
    let mut findings = Vec::new();
    let coverage = crate::ReadinessRunner::new([crate::Check::LayerSanity])
        .run(&mut findings, |_, sink| {
            sink.extend(crate::checks::skinny_layer_feature_readiness(
                "F.Cu",
                &uncertain_profile(),
                &2.into(),
                &Scalar::zero(),
            ));
            Ok::<_, std::convert::Infallible>(crate::CheckRunDisposition::Executed)
        })
        .unwrap();
    assert_eq!(
        coverage.checks[0].status,
        crate::CheckExecutionStatus::Uncertain
    );
    assert_eq!(coverage.blocking_count(), 1);
    assert_eq!(findings[0].layers, ["F.Cu"]);
    uncertainty(&findings);
}
