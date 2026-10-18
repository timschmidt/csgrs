use super::*;
use crate::geometry::{polygons_to_profile, rect_polygon};
use crate::kicad::{BoardModel, CopperFeature, CopperKind};
use crate::scalar::scalar;

#[derive(Clone, Copy, Debug)]
enum Rule {
    Fiducial,
    Escape,
    ViaSpacing,
    MaskBridge,
}
const RULES: [Rule; 4] = [
    Rule::Fiducial,
    Rule::Escape,
    Rule::ViaSpacing,
    Rule::MaskBridge,
];

fn rectangle(x: f64, width: f64) -> crate::PcbRegion {
    polygons_to_profile(vec![rect_polygon([x, 0.0], [width, width], 0.0)], None)
}
fn feature(kind: CopperKind, x: Scalar, shape_x: f64, width: f64) -> CopperFeature {
    CopperFeature {
        layer: "F.Cu".into(),
        net: Some("PAD".into()),
        kind,
        location: [x, Scalar::zero()],
        region: rectangle(shape_x, width),
    }
}
fn fiducial(x: Scalar) -> CopperFeature {
    let mut feature = feature(CopperKind::Pad, x, 0.0, 0.8);
    feature.net = None;
    feature
}
impl Rule {
    fn board(self, center: Scalar) -> BoardModel {
        let mut copper = (0..16)
            .map(|index| {
                feature(
                    CopperKind::Pad,
                    &center + Scalar::from(index) - scalar("7.5"),
                    f64::from(index) * 2.0,
                    0.2,
                )
            })
            .collect::<Vec<_>>();
        match self {
            Self::Fiducial => {
                copper.push(fiducial(&center - scalar("0.5")));
                copper.push(fiducial(&center + scalar("0.5")));
            }
            Self::Escape | Self::ViaSpacing => {
                copper.push(feature(CopperKind::Via, center + scalar("0.5"), 100.0, 0.2))
            }
            Self::MaskBridge => {}
        }
        BoardModel {
            source: "native dense cluster".into(),
            copper,
            drills: vec![],
            board_outline: None,
            panel_features: None,
        }
    }
    fn bad_board(self, center: Scalar) -> BoardModel {
        let mut board = self.board(center);
        match self {
            Self::Fiducial | Self::Escape => board.copper.truncate(16),
            Self::ViaSpacing => board.copper[16].region = board.copper[0].region.clone(),
            Self::MaskBridge => board.copper[1].region = board.copper[0].region.clone(),
        }
        board
    }
    fn run(self, board: &BoardModel, pitch: &Scalar, radius: &Scalar) -> Vec<Violation> {
        match self {
            Self::Fiducial => local_fiducial_readiness(board, &[], pitch, radius),
            Self::Escape => dense_pad_escape_readiness(board, &[], pitch, radius),
            Self::ViaSpacing => dense_pad_via_spacing_readiness(
                board,
                &[],
                pitch,
                radius,
                &scalar("0.1"),
                &Scalar::zero(),
            ),
            Self::MaskBridge => dense_pad_mask_bridge_readiness(board, &[], pitch, &scalar("0.1")),
        }
    }
    fn check(self) -> &'static str {
        match self {
            Self::Fiducial => "local-fiducial-readiness",
            Self::Escape => "dense-pad-escape-readiness",
            Self::ViaSpacing => "dense-pad-via-spacing-readiness",
            Self::MaskBridge => "dense-pad-mask-bridge-readiness",
        }
    }
}
fn unknown_zero() -> Scalar {
    let sine = Scalar::e().sin();
    let cosine = Scalar::e().cos();
    crate::test_support::terminal_zero(&sine * &sine + &cosine * &cosine - Scalar::one())
}
fn uncertain(findings: &[Violation]) {
    assert_eq!(findings.len(), 1, "{findings:?}");
    assert_eq!(findings[0].check, "geometry-uncertainty", "{findings:?}");
}
macro_rules! cluster {
    ($name:ident, $rule:ident) => {
        mod $name {
            use super::*;
            #[test]
            fn huge_centers_need_no_projection_for_a_passing_check() {
                let board = Rule::$rule.board(scalar("1e400"));
                assert!(
                    Rule::$rule
                        .run(&board, &Scalar::one(), &Scalar::one())
                        .is_empty()
                );
            }
            #[test]
            fn unknown_pitch_cannot_become_a_sparse_cluster() {
                let board = Rule::$rule.bad_board(Scalar::zero());
                uncertain(&Rule::$rule.run(
                    &board,
                    &(Scalar::one() + unknown_zero()),
                    &Scalar::one(),
                ));
            }
            #[test]
            fn required_large_report_marker_is_typed() {
                let board = Rule::$rule.bad_board(scalar("1e400"));
                uncertain(&Rule::$rule.run(&board, &Scalar::one(), &Scalar::one()));
            }
        }
    };
}
cluster!(fiducials, Fiducial);
cluster!(escape, Escape);
cluster!(via_spacing, ViaSpacing);
cluster!(mask_bridge, MaskBridge);

#[test]
fn finite_passes_and_findings_remain_distinct() {
    for rule in RULES {
        assert!(
            rule.run(&rule.board(Scalar::zero()), &Scalar::one(), &Scalar::one())
                .is_empty(),
            "{rule:?}"
        );
        let findings = rule.run(
            &rule.bad_board(Scalar::zero()),
            &Scalar::one(),
            &Scalar::one(),
        );
        assert_eq!(findings.len(), 1, "{rule:?}: {findings:?}");
        assert_eq!(findings[0].check, rule.check());
    }
}
#[test]
fn unknown_fiducial_radius_cannot_become_an_exact_count_of_one() {
    let mut board = Rule::Fiducial.board(Scalar::zero());
    board.copper[17].location[0] = Scalar::one();
    uncertain(&Rule::Fiducial.run(&board, &Scalar::one(), &(Scalar::one() + unknown_zero())));
}
#[test]
fn unknown_escape_radius_cannot_become_absence() {
    let mut board = Rule::Escape.board(Scalar::zero());
    board.copper[16].location[0] = Scalar::one();
    uncertain(&Rule::Escape.run(&board, &Scalar::one(), &(Scalar::one() + unknown_zero())));
}
#[test]
fn unknown_via_radius_cannot_suppress_close_copper() {
    let mut board = Rule::ViaSpacing.bad_board(Scalar::zero());
    board.copper[16].location[0] = Scalar::one();
    uncertain(&Rule::ViaSpacing.run(&board, &Scalar::one(), &(Scalar::one() + unknown_zero())));
}

fn native_rectangle(width: Scalar, height: Scalar) -> crate::PcbRegion {
    native_rectangle_at(Scalar::zero(), width, height)
}

fn native_rectangle_at(x: Scalar, width: Scalar, height: Scalar) -> crate::PcbRegion {
    use hypercurve::{Contour2, CurveRegion2, LineSeg2, Point2, Segment2};
    let points = [
        Point2::new(x.clone(), Scalar::zero()),
        Point2::new(&x + &width, Scalar::zero()),
        Point2::new(&x + width, height.clone()),
        Point2::new(x, height),
    ];
    let edges = points
        .iter()
        .cloned()
        .zip(points.iter().cloned().cycle().skip(1))
        .map(|(a, b)| Segment2::Line(LineSeg2::try_new(a, b).unwrap()))
        .collect();
    crate::PcbRegion::new(
        CurveRegion2::try_from_native_contours(vec![Contour2::try_new(edges).unwrap()], vec![])
            .unwrap(),
        None,
    )
}
#[test]
fn fiducial_minimum_dimension_is_native_below_a_float_ulp() {
    let mut board = Rule::Fiducial.board(Scalar::zero());
    for feature in &mut board.copper[16..] {
        feature.region = native_rectangle(scalar("0.5") - scalar("1e-50"), scalar("0.5"));
    }
    let findings = Rule::Fiducial.run(&board, &Scalar::one(), &Scalar::one());
    assert_eq!(findings.len(), 1);
    assert_eq!(findings[0].check, Rule::Fiducial.check());
}
#[test]
fn unresolved_fiducial_dimension_is_not_selected_or_dropped() {
    let mut board = Rule::Fiducial.board(Scalar::zero());
    board.copper[16].region = native_rectangle(scalar("0.5") + unknown_zero(), scalar("0.5"));
    uncertain(&Rule::Fiducial.run(&board, &Scalar::one(), &Scalar::one()));
}
#[test]
fn failed_fiducial_source_cannot_become_an_ordinary_pad() {
    let mut board = Rule::Fiducial.board(Scalar::zero());
    board.copper[16].region = board.copper[16]
        .region
        .clone()
        .with_exact_construction_error("retained fiducial source failure");
    uncertain(&Rule::Fiducial.run(&board, &Scalar::one(), &Scalar::one()));
}
#[test]
fn unknown_native_fiducial_bounds_are_required_evidence() {
    let mut board = Rule::Fiducial.board(Scalar::zero());
    board.copper[16].region = polygons_to_profile(
        vec![crate::bounds_regressions::uncertain_bounds_polygon()],
        None,
    );
    uncertain(&Rule::Fiducial.run(&board, &Scalar::one(), &Scalar::one()));
}
#[test]
fn huge_search_radius_is_supported() {
    for rule in RULES {
        assert!(
            rule.run(
                &rule.board(Scalar::zero()),
                &Scalar::one(),
                &scalar("1e400")
            )
            .is_empty(),
            "{rule:?}"
        );
    }
}

#[test]
fn two_certified_fiducials_win_after_an_unknown_match() {
    let mut board = Rule::Fiducial.board(Scalar::zero());
    board.copper[16].location[0] = Scalar::one() + unknown_zero();
    board.copper.push(fiducial(scalar("-0.5")));
    assert!(
        Rule::Fiducial
            .run(&board, &Scalar::one(), &Scalar::one())
            .is_empty()
    );
}
#[test]
fn certified_escape_via_wins_after_an_unknown_match() {
    let mut board = Rule::Escape.board(Scalar::zero());
    board.copper[16].location[0] = Scalar::one() + unknown_zero();
    board
        .copper
        .push(feature(CopperKind::Via, scalar("0.5"), 100.0, 0.2));
    assert!(
        Rule::Escape
            .run(&board, &Scalar::one(), &Scalar::one())
            .is_empty()
    );
}
#[test]
fn other_layer_fiducial_does_not_require_a_matching_certificate() {
    let mut board = Rule::Fiducial.board(Scalar::zero());
    board.copper[16].location[0] = Scalar::one() + unknown_zero();
    board.copper[16].layer = "B.Cu".into();
    let findings = Rule::Fiducial.run(&board, &Scalar::one(), &Scalar::one());
    assert_eq!(findings.len(), 1);
    assert_eq!(findings[0].check, Rule::Fiducial.check());
    assert!(
        findings[0]
            .message
            .as_deref()
            .unwrap()
            .contains("only 1 likely")
    );
}
#[test]
fn fiducial_eligibility_needs_no_width_height_order() {
    let mut board = Rule::Fiducial.board(Scalar::zero());
    board.copper[16].region = native_rectangle(scalar("0.8") + unknown_zero(), scalar("0.8"));
    assert!(
        Rule::Fiducial
            .run(&board, &Scalar::one(), &Scalar::one())
            .is_empty()
    );
}
#[test]
fn certified_fiducial_exclusion_wins_after_unknown_dimension() {
    let mut board = Rule::Fiducial.board(Scalar::zero());
    board.copper[16].region = native_rectangle(scalar("0.5") + unknown_zero(), Scalar::from(10));
    let findings = Rule::Fiducial.run(&board, &Scalar::one(), &Scalar::one());
    assert_eq!(findings.len(), 1);
    assert_eq!(findings[0].check, Rule::Fiducial.check());
}
#[test]
fn unused_pad_shapes_do_not_override_center_matches() {
    for rule in [Rule::Fiducial, Rule::Escape] {
        let mut board = rule.board(Scalar::zero());
        for pad in &mut board.copper[..16] {
            pad.region = pad
                .region
                .clone()
                .with_exact_construction_error("unused pad shape");
        }
        assert!(rule.run(&board, &Scalar::one(), &Scalar::one()).is_empty());
    }
}
#[test]
fn a_dense_pitch_witness_survives_unknown_minimum_order() {
    for rule in RULES {
        let mut board = rule.bad_board(Scalar::zero());
        board.copper[0].location[0] += unknown_zero();
        let findings = rule.run(&board, &Scalar::from(2), &Scalar::one());
        assert_eq!(findings.len(), 1, "{rule:?}: {findings:?}");
        assert_eq!(findings[0].check, rule.check());
        assert!(
            findings[0]
                .message
                .as_deref()
                .unwrap()
                .contains("certified pitch")
        );
    }
}
#[test]
fn required_shape_failures_survive_candidate_scheduling() {
    for rule in [Rule::ViaSpacing, Rule::MaskBridge] {
        let mut board = rule.board(Scalar::zero());
        board.copper[0].region = board.copper[0]
            .region
            .clone()
            .with_exact_construction_error("required spacing shape");
        uncertain(&rule.run(&board, &Scalar::one(), &Scalar::one()));
    }
}
#[test]
fn certified_bad_pairs_survive_unavailable_optional_minima() {
    for rule in [Rule::ViaSpacing, Rule::MaskBridge] {
        let mut board = rule.bad_board(Scalar::zero());
        board.copper[2].region = board.copper[1].region.clone();
        if matches!(rule, Rule::ViaSpacing) {
            board.copper[1].region = board.copper[16].region.clone();
        }
        board.copper[0].region = board.copper[0]
            .region
            .clone()
            .with_exact_construction_error("optional minimum candidate");
        let findings = rule.run(&board, &Scalar::one(), &Scalar::one());
        assert_eq!(findings.len(), 1, "{rule:?}: {findings:?}");
        assert_eq!(findings[0].check, rule.check());
        assert!(
            findings[0]
                .message
                .as_deref()
                .unwrap()
                .contains("certified pad")
        );
    }
}
#[test]
fn empty_shapes_are_not_failed_measurements() {
    for rule in [Rule::ViaSpacing, Rule::MaskBridge] {
        let mut board = rule.board(Scalar::zero());
        board.copper[0].region = crate::geometry::empty_profile(None);
        assert!(rule.run(&board, &Scalar::one(), &Scalar::one()).is_empty());
    }
}
#[test]
fn negative_search_radius_cannot_become_a_positive_match() {
    for rule in [Rule::Fiducial, Rule::Escape, Rule::ViaSpacing] {
        let board = rule.board(Scalar::zero());
        let findings = rule.run(&board, &Scalar::one(), &Scalar::from(-1));
        if matches!(rule, Rule::ViaSpacing) {
            assert!(findings.is_empty());
        } else {
            assert_eq!(findings.len(), 1);
            assert_eq!(findings[0].check, rule.check());
        }
    }
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
fn exhausted_warm_checks_retain_failure_and_retry() {
    let boards = RULES.map(|rule| rule.board(Scalar::zero()));
    for (rule, board) in RULES.into_iter().zip(&boards) {
        assert!(rule.run(board, &Scalar::one(), &Scalar::one()).is_empty());
    }
    let denied = hypercurve::evaluate_with_budget(
        crate::PREDICATE_POLICY,
        hypercurve::WorkBudget::ZERO,
        || {
            deny_refinement();
            for (rule, board) in RULES.into_iter().zip(&boards) {
                uncertain(&rule.run(board, &Scalar::one(), &Scalar::one()));
            }
        },
    );
    assert!(denied.result.is_err());
    for (rule, board) in RULES.into_iter().zip(&boards) {
        assert!(rule.run(board, &Scalar::one(), &Scalar::one()).is_empty());
    }
}
#[test]
fn exhausted_empty_paths_retain_failure() {
    let mut board = Rule::Escape.board(Scalar::zero());
    board.copper.clear();
    let denied = hypercurve::evaluate_with_budget(
        crate::PREDICATE_POLICY,
        hypercurve::WorkBudget::ZERO,
        || {
            deny_refinement();
            for rule in RULES {
                uncertain(&rule.run(&board, &Scalar::zero(), &Scalar::zero()));
            }
        },
    );
    assert!(denied.result.is_err());
}
#[test]
fn enclosing_preview_cannot_decide_unknown_pitch() {
    for rule in RULES {
        let board = rule.bad_board(Scalar::zero());
        let pitch = Scalar::one() + unknown_zero();
        let expected = serde_json::to_vec(&rule.run(&board, &pitch, &Scalar::one())).unwrap();
        let preview =
            hypercurve::evaluate_under(hyperlimit::PredicatePolicy::APPROXIMATE_512, || {
                let findings = rule.run(&board, &pitch, &Scalar::one());
                uncertain(&findings);
                serde_json::to_vec(&findings).unwrap()
            })
            .certified()
            .unwrap();
        assert_eq!(expected, preview, "{rule:?}");
    }
}
#[test]
fn cluster_uncertainty_blocks_readiness_receipts() {
    use crate::Check;
    let checks = [
        Check::LocalFiducialReadiness,
        Check::DensePadEscapeReadiness,
        Check::DensePadViaSpacingReadiness,
        Check::DensePadMaskBridgeReadiness,
    ];
    for (rule, check) in RULES.into_iter().zip(checks) {
        let board = rule.bad_board(Scalar::zero());
        let mut findings = Vec::new();
        let coverage = crate::ReadinessRunner::new([check])
            .run(&mut findings, |_, sink| {
                sink.extend(rule.run(&board, &(Scalar::one() + unknown_zero()), &Scalar::one()));
                Ok::<_, std::convert::Infallible>(crate::CheckRunDisposition::Executed)
            })
            .unwrap();
        assert_eq!(
            coverage.checks[0].status,
            crate::CheckExecutionStatus::Uncertain
        );
        assert_eq!(coverage.blocking_count(), 1);
        assert_eq!(findings[0].layers, [board.source]);
    }
}
#[test]
fn native_shape_index_retains_unavailable_enclosures_and_original_indices() {
    use super::spatial::NativeRegionSpatialIndex;
    let regions = [
        rectangle(0.0, 1.0),
        crate::geometry::empty_profile(None)
            .with_exact_construction_error("unavailable stored enclosure"),
        rectangle(100.0, 1.0),
    ];
    let index = NativeRegionSpatialIndex::new(&regions, &Scalar::zero());
    assert_eq!(
        index.candidates_near(&rectangle(0.5, 1.0), &Scalar::zero()),
        [0, 1]
    );
    assert_eq!(
        index.candidates_near(&regions[1], &Scalar::zero()),
        [0, 1, 2]
    );
    assert_eq!(
        index.candidates_near(&regions[0], &scalar("1e400")),
        [0, 1, 2]
    );
    assert_eq!(index.later_candidates_near(1, &Scalar::zero()), [2]);
}
#[test]
fn native_shape_index_keeps_exact_closed_contacts_across_scales() {
    use super::spatial::NativeRegionSpatialIndex;
    let regions = [
        rectangle(0.0, 1.0),
        rectangle(1.0, 1.0),
        rectangle(1.0_f64.next_up(), 1.0),
        rectangle(1.0_f64.next_down(), 1.0),
        native_rectangle_at(scalar("1e20"), Scalar::one(), Scalar::one()),
        native_rectangle_at(scalar("1e20") + Scalar::one(), Scalar::one(), Scalar::one()),
    ];
    let index = NativeRegionSpatialIndex::new(&regions, &Scalar::zero());
    for radius in [Scalar::zero(), scalar("0.125"), scalar("1e30")] {
        for (left, region) in regions.iter().enumerate() {
            let bounds = region.try_native_bounds().unwrap().unwrap();
            let candidates = index.candidates_near(region, &radius);
            assert!(candidates.windows(2).all(|pair| pair[0] < pair[1]));
            for (right, other) in regions.iter().enumerate() {
                let other_bounds = other.try_native_bounds().unwrap().unwrap();
                let overlaps = (0..2).all(|axis| {
                    crate::scalar::try_compare(&(&bounds[axis + 2] + &radius), &other_bounds[axis])
                        .unwrap()
                        .is_ge()
                        && crate::scalar::try_compare(
                            &(&other_bounds[axis + 2] + &radius),
                            &bounds[axis],
                        )
                        .unwrap()
                        .is_ge()
                });
                if overlaps {
                    assert!(candidates.contains(&right), "left={left}, right={right}");
                }
            }
        }
    }
}
