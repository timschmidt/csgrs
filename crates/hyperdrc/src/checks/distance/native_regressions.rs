use super::native::{
    PreparedRegionBoundary, point_region_boundary_within, region_boundaries_within,
    region_boundary_distance,
};
use crate::geometry::polygons_to_profile;
use crate::geometry::{Coord, LineString, MultiPolygon};
use crate::geometry::{Polygon, circle_polygon, rect_polygon};
use crate::scalar::scalar;
use crate::{PREDICATE_POLICY, Scalar};
use crate::{PcbRegion, PcbRegionExt};

#[test]
fn native_circle_distance_is_independent_of_report_resolution() {
    let expected = scalar("4.015625").sqrt().unwrap() - Scalar::from(2);
    for samples in [16, 17, 48, 128] {
        let left = MultiPolygon(vec![circle_polygon([0.0, 0.0], 1.0, samples)]);
        let right = MultiPolygon(vec![circle_polygon([2.0, 0.125], 1.0, samples)]);
        let actual = region_boundary_distance(
            &polygons_to_profile(left.0, None),
            &polygons_to_profile(right.0, None),
        )
        .unwrap()
        .unwrap();
        assert_eq!(
            crate::scalar::compare(&actual, &expected),
            Some(std::cmp::Ordering::Equal),
            "native circle gap with {samples} report samples"
        );
    }
}

#[test]
fn point_clearance_uses_the_authored_circle() {
    let polygons = MultiPolygon(vec![circle_polygon([0.0, 0.0], 1.0, 48)]);
    let point = [scalar("0.999"), scalar("0.03")];
    let threshold = scalar("0.00075");
    let norm_squared = &point[0] * &point[0] + &point[1] * &point[1];
    let inner_radius = Scalar::one() - &threshold;
    assert!(norm_squared < Scalar::one());
    assert!(norm_squared > &inner_radius * &inner_radius);
    assert!(
        point_region_boundary_within(&point, &polygons_to_profile(polygons.0, None), &threshold)
            .unwrap()
    );
}

#[test]
fn overlapping_profiles_do_not_publish_internal_seams_as_boundaries() {
    let polygons = MultiPolygon(vec![
        rect_polygon([1.0, 1.0], [2.0, 2.0], 0.0),
        rect_polygon([2.0, 1.0], [2.0, 2.0], 0.0),
    ]);
    // The union is [0,3] x [0,2]. Its closest boundary is one unit away,
    // although the second authored rectangle has an edge through this point.
    assert!(
        !point_region_boundary_within(
            &[Scalar::one(), Scalar::one()],
            &polygons_to_profile(polygons.0, None),
            &scalar("0.1")
        )
        .unwrap()
    );
}

#[test]
fn native_hole_boundary_retains_its_radius() {
    let origin = [Scalar::zero(), Scalar::zero()];
    let outer = crate::geometry::circle_region(&origin, &Scalar::from(3)).unwrap();
    let inner = crate::geometry::circle_region(&origin, &Scalar::one()).unwrap();
    let native_outer = outer.native_contours_fast_path().unwrap().unwrap();
    let native_inner = inner.native_contours_fast_path().unwrap().unwrap();
    let region = hypercurve::CurveRegion2::try_from_native_contours(
        native_outer.material_contours().to_vec(),
        native_inner.material_contours().to_vec(),
    )
    .unwrap();
    let polygons = PcbRegion::new(region, None).try_to_multipolygon().unwrap();
    assert!(
        !point_region_boundary_within(
            &origin,
            &polygons_to_profile(polygons.0, None),
            &scalar("0.999999")
        )
        .unwrap()
    );
}

fn unsupported_native_region() -> PcbRegion {
    use hypercurve::{
        Curve2, CurvePath2, CurveRegionLoopRole, FillRule, LineSeg2, RationalBezier2,
    };
    let curve = RationalBezier2::try_new(
        [(0, 0), (1, 1), (2, 2), (3, 2), (4, 1), (5, 0)]
            .into_iter()
            .map(|(x, y)| hypercurve::Point2::from_values(x, y))
            .collect(),
        [1, 2, 3, 4, 5, 7].into_iter().map(Scalar::from).collect(),
    )
    .unwrap();
    let path = CurvePath2::try_new(vec![
        Curve2::from(curve),
        Curve2::from(
            LineSeg2::try_new(
                hypercurve::Point2::from_values(5, 0),
                hypercurve::Point2::from_values(0, 0),
            )
            .unwrap(),
        ),
    ])
    .unwrap();
    let region = hypercurve::CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[path],
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
    )
    .unwrap();
    let polygon = Polygon::from_exact_region(
        region,
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
    );
    PcbRegion::new_shared(polygon.shared_exact_region(), None)
}

#[test]
fn unsupported_native_boundary_cannot_borrow_report_segments() {
    let query = MultiPolygon(vec![rect_polygon([8.0, 0.0], [1.0, 1.0], 0.0)]);
    assert!(
        region_boundary_distance(
            &unsupported_native_region(),
            &polygons_to_profile(query.0, None)
        )
        .is_err()
    );
}

#[test]
fn exhausted_prepared_metric_does_not_publish_a_cached_distance() {
    let left = MultiPolygon(vec![rect_polygon([0.0, 0.0], [1.0, 1.0], 0.0)]);
    let right = MultiPolygon(vec![rect_polygon([2.0, 0.0], [1.0, 1.0], 0.0)]);
    let left = PreparedRegionBoundary::new(&polygons_to_profile(left.0, None)).unwrap();
    let right = PreparedRegionBoundary::new(&polygons_to_profile(right.0, None)).unwrap();
    assert_eq!(left.distance_to(&right).unwrap(), Some(Scalar::one()));
    let mut observation = None;
    let denied =
        hypercurve::evaluate_with_budget(PREDICATE_POLICY, hypercurve::WorkBudget::ZERO, || {
            let polynomial = hypercurve::BezierParameterPolynomial::try_new_power_basis(
                [-36, 1368, -11034, 31728, -38280, 16620]
                    .into_iter()
                    .map(Scalar::from)
                    .collect(),
            )
            .unwrap();
            assert!(polynomial.isolate_unit_interval_roots().is_err());
            observation = Some(left.distance_to(&right));
            assert!(left.within(&right, &Scalar::one()).is_err());
            assert!(
                left.point_within(&[Scalar::zero(), Scalar::zero()], &Scalar::one())
                    .is_err()
            );
            assert!(super::compare_metrics(&Scalar::one(), &Scalar::one()).is_err());
        });
    assert!(denied.result.is_err());
    assert!(
        observation
            .unwrap()
            .unwrap_err()
            .detail
            .contains("budget exhausted")
    );
    assert_eq!(left.distance_to(&right).unwrap(), Some(Scalar::one()));
}

fn square(x: f64, y: f64, size: f64) -> Polygon<f64> {
    Polygon::new(
        LineString(vec![
            Coord { x, y },
            Coord { x: x + size, y },
            Coord {
                x: x + size,
                y: y + size,
            },
            Coord { x, y: y + size },
            Coord { x, y },
        ]),
        Vec::new(),
    )
}

#[test]
fn native_boundary_distance_retains_exact_line_coordinates() {
    let left = MultiPolygon(vec![square(0.0, 0.0, 1.0)]);
    let right = MultiPolygon(vec![square(3.0, 0.0, 1.0)]);

    assert_eq!(
        region_boundary_distance(
            &polygons_to_profile(left.0, None),
            &polygons_to_profile(right.0, None)
        )
        .unwrap(),
        Some(crate::scalar::scalar("2"))
    );
}

#[test]
fn prepared_boundary_reuses_large_exact_segment_field() {
    let field = MultiPolygon(
        (0..2_000)
            .map(|index| square(f64::from(index) * 4.0, 0.0, 1.0))
            .collect(),
    );
    let preparation_started = std::time::Instant::now();
    let prepared_field = PreparedRegionBoundary::new(&polygons_to_profile(field.0, None))
        .expect("finite field has exact boundaries");
    let preparation_elapsed = preparation_started.elapsed();
    let started = std::time::Instant::now();
    let mut rows = Vec::new();

    for index in 0..64 {
        let probe = MultiPolygon(vec![square(f64::from(index) * 4.0, 2.0, 1.0)]);
        let prepared_probe = PreparedRegionBoundary::new(&polygons_to_profile(probe.0, None))
            .expect("finite probe has exact boundaries");
        let distance = prepared_field
            .distance_to(&prepared_probe)
            .unwrap()
            .unwrap();
        assert_eq!(distance, crate::scalar::scalar("1"));
        rows.push(distance.to_string());
    }

    let query_elapsed = started.elapsed();
    use sha2::Digest;
    let serialized = serde_json::to_vec(&rows).unwrap();
    println!(
        "native_boundary_field squares=2000 probes={} prepare_us={} query_us={} bytes={} sha256={:x}",
        rows.len(),
        preparation_elapsed.as_micros(),
        query_elapsed.as_micros(),
        serialized.len(),
        sha2::Sha256::digest(&serialized)
    );
    assert!(
        query_elapsed.as_secs_f64() < 2.0,
        "prepared exact boundary field should be indexed and reused"
    );
}

#[test]
fn native_thresholds_preserve_empty_contact_and_nested_boundary_semantics() {
    let empty = crate::geometry::empty_profile(None);
    let left = polygons_to_profile(vec![square(0.0, 0.0, 1.0)], None);
    let contact = polygons_to_profile(vec![square(1.0, 0.0, 1.0)], None);
    let outer = polygons_to_profile(vec![square(-1.0, -1.0, 3.0)], None);
    assert_eq!(region_boundary_distance(&left, &empty).unwrap(), None);
    assert!(!region_boundaries_within(&left, &empty, &Scalar::from(100)).unwrap());
    assert!(region_boundaries_within(&left, &contact, &Scalar::zero()).unwrap());
    assert!(!region_boundaries_within(&left, &contact, &(-Scalar::one())).unwrap());
    assert!(region_boundaries_within(&left, &outer, &Scalar::one()).unwrap());
    assert!(!region_boundaries_within(&left, &outer, &scalar("0.999999999999")).unwrap());
}

#[test]
fn invalid_construction_is_distinct_from_empty_geometry() {
    let invalid = polygons_to_profile(
        vec![square(0.0, 0.0, 1.0).with_exact_construction_error("retained source failure")],
        None,
    );
    assert!(
        PreparedRegionBoundary::new(&invalid)
            .err()
            .unwrap()
            .detail
            .contains("retained source failure")
    );
}

#[test]
fn native_metrics_suspend_enclosing_display_preview() {
    let circle = PcbRegion::new(
        crate::geometry::circle_region(&[Scalar::zero(), Scalar::zero()], &Scalar::one()).unwrap(),
        None,
    );
    let gap = scalar("0.000000000001");
    let point = [Scalar::zero(), Scalar::one() + &gap];
    for preview in [
        hypercurve::CurvePreviewOptions::try_strict(0.1, 0.1).unwrap(),
        hypercurve::CurvePreviewOptions::try_approximate_512(0.1, 0.1).unwrap(),
    ] {
        preview.evaluate(|| {
            assert!(!point_region_boundary_within(&point, &circle, &Scalar::zero()).unwrap());
            assert!(point_region_boundary_within(&point, &circle, &gap).unwrap());
            assert!(
                !point_region_boundary_within(&point, &circle, &crate::scalar::half(&gap)).unwrap()
            );
        });
    }
}

#[test]
fn missing_finite_enclosures_keep_native_candidates() {
    let huge = scalar("1e400");
    let circle = |x: Scalar| {
        PcbRegion::new(
            crate::geometry::circle_region(&[x, Scalar::zero()], &Scalar::one()).unwrap(),
            None,
        )
    };
    let left = circle(huge.clone());
    let right = circle(&huge + Scalar::from(3));
    assert!(super::finite_enclosure(&huge).is_none());
    assert_eq!(
        region_boundary_distance(&left, &right).unwrap(),
        Some(Scalar::one())
    );
    assert!(region_boundaries_within(&left, &right, &Scalar::one()).unwrap());
    assert!(!region_boundaries_within(&left, &right, &scalar("0.999999")).unwrap());
    assert!(
        point_region_boundary_within(
            &[huge + Scalar::from(2), Scalar::zero()],
            &left,
            &Scalar::one()
        )
        .unwrap()
    );
}

#[test]
fn unsupported_native_metric_reaches_readiness_receipt() {
    use crate::kicad::{BoardModel, CopperFeature, CopperKind};
    use crate::readiness::{CheckExecutionStatus, CheckRunDisposition, ReadinessRunner};
    let board = BoardModel {
        source: "unsupported native cap".to_owned(),
        copper: vec![CopperFeature {
            layer: "F.Cu".to_owned(),
            net: Some("CARD_EDGE".to_owned()),
            kind: CopperKind::Pad,
            region: unsupported_native_region(),
            location: [Scalar::zero(), Scalar::zero()],
        }],
        drills: Vec::new(),
        board_outline: Some(polygons_to_profile(vec![square(-10.0, -10.0, 20.0)], None)),
        panel_features: None,
    };
    let mut findings = Vec::new();
    let coverage = ReadinessRunner::new([crate::Check::GoldFingerEdgeReadiness])
        .run(&mut findings, |_, findings| {
            findings.extend(crate::checks::gold_finger_edge_readiness(
                &board,
                &[],
                &Scalar::one(),
            ));
            Ok::<_, ()>(CheckRunDisposition::Executed)
        })
        .unwrap();
    assert_eq!(findings.len(), 1);
    assert_eq!(findings[0].check, "geometry-uncertainty");
    assert_eq!(findings[0].layers, ["F.Cu"]);
    assert!(
        findings[0]
            .message
            .as_deref()
            .unwrap()
            .contains("gold-finger-edge-readiness")
    );
    assert_eq!(coverage.checks[0].status, CheckExecutionStatus::Uncertain);
    assert!(!coverage.is_complete());
    let serialized = serde_json::to_value(&coverage).unwrap();
    assert_eq!(serialized["checks"][0]["status"], "uncertain");
}
