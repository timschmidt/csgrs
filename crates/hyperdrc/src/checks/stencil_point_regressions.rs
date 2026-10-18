use super::*;
use crate::geometry::{circle_polygon, rect_polygon};

fn unknown_zero() -> Scalar {
    let sine = Scalar::e().sin();
    let cosine = Scalar::e().cos();
    let zero =
        crate::test_support::terminal_zero(&sine * &sine + &cosine * &cosine - Scalar::one());
    assert!(crate::scalar::compare(&zero, &Scalar::zero()).is_none());
    zero
}

fn pads() -> (PcbRegion, PcbRegion) {
    let left = rect_polygon([0.0, 0.0], [0.5, 0.5], 0.0);
    let right = rect_polygon([2.0, 0.0], [0.5, 0.5], 0.0);
    (
        polygons_to_profile(vec![left.clone()], None),
        polygons_to_profile(vec![left, right], None),
    )
}

fn tombstone(gap: &Scalar) -> Vec<Violation> {
    let (paste, copper) = pads();
    tombstone_paste_imbalance_readiness(
        "F.Paste",
        &paste,
        "F.Cu",
        &copper,
        gap,
        &crate::scalar::scalar("0.25"),
        &Scalar::zero(),
    )
}

fn uncertainty(findings: &[Violation]) {
    assert_eq!(findings.len(), 1, "{findings:?}");
    assert_eq!(findings[0].check, "geometry-uncertainty", "{findings:?}");
}

fn board(location: [Scalar; 2], drills: Vec<DrillFeature>) -> BoardModel {
    BoardModel {
        source: "native-stencil-regression".into(),
        copper: vec![CopperFeature {
            layer: "F.Cu".into(),
            net: Some("GND".into()),
            kind: CopperKind::Via,
            region: polygon_to_profile(circle_polygon([0.0, 0.0], 0.2, 17), None),
            location,
        }],
        drills,
        board_outline: None,
        panel_features: None,
    }
}

fn drill(location: [Scalar; 2], diameter: Scalar) -> DrillFeature {
    DrillFeature {
        location,
        diameter,
        plated: true,
        net: Some("GND".into()),
    }
}

fn exposure(board: &BoardModel) -> Vec<Violation> {
    let paste = polygon_to_profile(rect_polygon([0.0, 0.0], [1.0, 1.0], 0.0), None);
    paste_via_exposure_readiness("F.Paste", &paste, board, &[], &Scalar::zero())
}

#[test]
fn unresolved_tombstone_gap_cannot_become_a_definite_pair() {
    uncertainty(&tombstone(&(Scalar::from(2) + unknown_zero())));
}

#[test]
fn unresolved_tombstone_gap_remains_uncertain_inside_preview() {
    let gap = Scalar::from(2) + unknown_zero();
    hypercurve::CurvePreviewOptions::try_approximate_512(1.0, 1.0)
        .unwrap()
        .evaluate(|| uncertainty(&tombstone(&gap)));
}

#[test]
fn unrepresentable_tombstone_radius_keeps_qualifying_pairs() {
    let findings = tombstone(&"1e10000".parse().unwrap());
    assert_eq!(findings.len(), 1, "{findings:?}");
    assert_eq!(findings[0].check, "tombstone-paste-imbalance-readiness");
}

#[test]
fn negative_tombstone_radius_has_no_pairs() {
    assert!(tombstone(&Scalar::from(-1)).is_empty());
}

#[test]
fn exact_pair_boundary_is_included() {
    let findings = tombstone(&2.into());
    assert_eq!(findings.len(), 1, "{findings:?}");
    assert_eq!(findings[0].check, "tombstone-paste-imbalance-readiness");
    assert!(tombstone(&crate::scalar::scalar("1.999999999999999999999999")).is_empty());
}

#[test]
fn unresolved_drill_match_cannot_choose_copper_fallback() {
    let source = board(
        [Scalar::one() + unknown_zero(), Scalar::zero()],
        vec![drill([Scalar::zero(), Scalar::zero()], Scalar::one())],
    );
    uncertainty(&exposure(&source));
}

#[test]
fn unresolved_matching_radius_cannot_choose_an_arm() {
    let source = board(
        [Scalar::zero(), Scalar::zero()],
        vec![drill(
            [Scalar::zero(), Scalar::zero()],
            crate::scalar::scalar("0.05") + unknown_zero(),
        )],
    );
    let matched = matching_plated_drill(&source, &source.copper[0]);
    assert!(
        matched.is_err(),
        "unknown matching radius must remain a typed error"
    );
}

#[test]
fn unresolved_first_drill_cannot_be_skipped_for_a_later_match() {
    let source = board(
        [Scalar::zero(), Scalar::zero()],
        vec![
            drill(
                [Scalar::one() + unknown_zero(), Scalar::zero()],
                Scalar::one(),
            ),
            drill(
                [Scalar::zero(), Scalar::zero()],
                crate::scalar::scalar("0.1"),
            ),
        ],
    );
    uncertainty(&exposure(&source));
}

#[test]
fn report_anchor_projection_failure_cannot_panic() {
    let source = board(["1e10000".parse().unwrap(), Scalar::zero()], vec![]);
    // Public BoardModel permits anchors independent of a feature's geometry.
    // This report projection must fail explicitly even with valid local copper.
    uncertainty(&exposure(&source));
}

#[test]
fn irrelevant_drills_do_not_require_numeric_certificates() {
    let diameter = crate::scalar::scalar("0.05") + unknown_zero();
    let mut unplated = drill([Scalar::zero(), Scalar::zero()], diameter.clone());
    unplated.plated = false;
    let mut other_net = drill([Scalar::zero(), Scalar::zero()], diameter);
    other_net.net = Some("OTHER".into());
    let source = board([Scalar::zero(), Scalar::zero()], vec![unplated, other_net]);
    let findings = exposure(&source);
    assert_eq!(findings.len(), 1, "{findings:?}");
    assert_eq!(findings[0].check, "paste-via-exposure-readiness");
}

#[test]
fn a_known_first_drill_does_not_require_later_match_certificates() {
    let source = board(
        [Scalar::zero(), Scalar::zero()],
        vec![
            drill(
                [Scalar::zero(), Scalar::zero()],
                crate::scalar::scalar("0.1"),
            ),
            drill(
                [Scalar::one() + unknown_zero(), Scalar::zero()],
                Scalar::one(),
            ),
        ],
    );
    let findings = exposure(&source);
    assert_eq!(findings.len(), 1, "{findings:?}");
    assert_eq!(findings[0].check, "paste-via-exposure-readiness");
}

#[test]
fn native_point_candidates_cover_exact_contacts_across_representations() {
    let huge: Scalar = "1e10000".parse().unwrap();
    let root = Scalar::from(2).sqrt().unwrap();
    let tiny: Scalar = "1e-1000".parse().unwrap();
    let zero = Scalar::zero();
    let points = [
        [zero.clone(), zero.clone()],
        [Scalar::one(), zero.clone()],
        [
            crate::scalar::scalar("0.999999999999999999999999"),
            zero.clone(),
        ],
        [
            crate::scalar::scalar("1.000000000000000000000001"),
            zero.clone(),
        ],
        [(-1).into(), zero.clone()],
        [root.clone(), root],
        [tiny.clone(), zero.clone()],
        [huge.clone(), zero.clone()],
        [&huge + Scalar::one(), zero.clone()],
        [-&huge, zero.clone()],
    ];
    for radius in [zero, tiny, Scalar::one(), Scalar::from(2), huge] {
        let index = NativePointSpatialIndex::new(points.iter(), &radius);
        for left in 0..points.len() {
            let candidates = index.later_candidates_near(left);
            assert!(candidates.windows(2).all(|pair| pair[0] < pair[1]));
            for right in left + 1..points.len() {
                if super::super::measurements::points_within_radius(
                    &points[left],
                    &points[right],
                    &radius,
                )
                .unwrap()
                {
                    assert!(
                        candidates.contains(&right),
                        "pair {left}/{right} was discarded"
                    );
                }
            }
        }
    }
}

#[test]
fn native_point_candidates_keep_unknown_threshold_pairs() {
    let points = [[Scalar::zero(), Scalar::zero()], [2.into(), Scalar::zero()]];
    let radius = Scalar::from(2) + unknown_zero();
    let index = NativePointSpatialIndex::new(points.iter(), &radius);
    assert_eq!(index.later_candidates_near(0), vec![1]);
    assert!(
        super::super::measurements::points_within_radius(&points[0], &points[1], &radius).is_err()
    );
}

#[test]
fn squared_native_distance_handles_huge_and_radical_exact_contacts() {
    let origin = [Scalar::zero(), Scalar::zero()];
    let point = [Scalar::from(3), Scalar::from(4)];
    assert!(super::super::measurements::points_within_radius(&origin, &point, &5.into()).unwrap());
    assert!(
        !super::super::measurements::points_within_radius(&origin, &point, &(-5).into()).unwrap()
    );
    let root = Scalar::from(2).sqrt().unwrap();
    assert!(
        super::super::measurements::points_within_radius(&origin, &[root.clone(), root], &2.into())
            .unwrap()
    );
    let huge: Scalar = "1e10000".parse().unwrap();
    let left = [huge.clone(), Scalar::zero()];
    let right = [&huge + Scalar::one(), Scalar::zero()];
    assert!(
        super::super::measurements::points_within_radius(&left, &right, &Scalar::one()).unwrap()
    );
}

#[test]
fn denied_native_point_queries_retain_candidates_and_fail_then_retry() {
    let points = [[Scalar::zero(), Scalar::zero()], [2.into(), Scalar::zero()]];
    let radius = Scalar::from(2);
    let warm = NativePointSpatialIndex::new(points.iter(), &radius);
    assert!(
        super::super::measurements::points_within_radius(&points[0], &points[1], &radius).unwrap()
    );
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
            let cold = NativePointSpatialIndex::new(points.iter(), &radius);
            assert_eq!(warm.later_candidates_near(0), vec![1]);
            assert_eq!(cold.later_candidates_near(0), vec![1]);
            assert!(
                super::super::measurements::points_within_radius(&points[0], &points[1], &radius)
                    .is_err()
            );
            let source = board([Scalar::zero(), Scalar::zero()], vec![]);
            assert!(matching_plated_drill(&source, &source.copper[0]).is_err());
        },
    );
    assert!(result.result.is_err());
    assert!(
        super::super::measurements::points_within_radius(&points[0], &points[1], &radius).unwrap()
    );
}

#[test]
fn unresolved_stencil_geometry_becomes_a_blocking_receipt() {
    let gap = Scalar::from(2) + unknown_zero();
    let mut findings = Vec::new();
    let coverage = crate::ReadinessRunner::new([crate::Check::TombstonePasteImbalanceReadiness])
        .run(&mut findings, |_, sink| {
            sink.extend(tombstone(&gap));
            Ok::<_, std::convert::Infallible>(crate::CheckRunDisposition::Executed)
        })
        .unwrap();
    assert_eq!(
        coverage.checks[0].status,
        crate::CheckExecutionStatus::Uncertain
    );
    assert_eq!(coverage.blocking_count(), 1);
    assert_eq!(findings[0].layers, ["F.Paste", "F.Cu"]);
}
