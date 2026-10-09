use super::*;

#[test]
fn thermal_certified_zone_wins_after_unknown_alternative() {
    let mut board = thermal_board(0.into(), Scalar::one() + unknown_zero());
    board
        .copper
        .push(feature(CopperKind::Zone, "VCC", "F.Cu", scalar("0.5")));
    assert!(thermal_copper_area_readiness(&board, &[], &Scalar::one()).is_empty());
}
#[test]
fn thermal_metadata_filters_precede_required_matching() {
    let mut board = thermal_board(0.into(), unknown_zero());
    board.copper[1].layer = "B.Cu".into();
    board
        .copper
        .push(feature(CopperKind::Zone, "GND", "F.Cu", unknown_zero()));
    ordinary(
        &thermal_copper_area_readiness(&board, &[], &unknown_zero()),
        1,
        "thermal-copper-area-readiness",
    );
    assert!(
        thermal_copper_area_readiness(&board, &["Other.Cu".into()], &unknown_zero()).is_empty()
    );
}
#[test]
fn authored_center_proxies_do_not_require_unused_shapes() {
    let mut board = thermal_board(0.into(), scalar("0.5"));
    for feature in &mut board.copper {
        feature.region = feature
            .region
            .clone()
            .with_exact_construction_error("unused shape");
    }
    assert!(thermal_copper_area_readiness(&board, &[], &Scalar::one()).is_empty());
    assert!(access(&board, &[point(0.into())], &Scalar::one(), &Scalar::one()).is_empty());
    assert!(testpoint_coverage_readiness(&board, &[point(0.into())], &[]).is_empty());
}
#[test]
fn thermal_negative_radius_cannot_be_squared_into_a_match() {
    ordinary(
        &thermal_copper_area_readiness(&thermal_board(0.into(), 0.into()), &[], &Scalar::from(-1)),
        1,
        "thermal-copper-area-readiness",
    );
}
#[test]
fn access_expected_side_proof_wins_after_unknown_other_side() {
    let board = board(vec![
        feature(
            CopperKind::Pad,
            "VCC",
            "B.Cu",
            scalar("1.5") + unknown_zero(),
        ),
        feature(CopperKind::Via, "VCC", "F.Cu", scalar("0.5")),
    ]);
    assert!(access(&board, &[point(0.into())], &Scalar::one(), &Scalar::one()).is_empty());
}
#[test]
fn access_absent_opposite_side_does_not_need_expected_match() {
    let board = board(vec![
        feature(
            CopperKind::Pad,
            "VCC",
            "F.Cu",
            scalar("1.5") + unknown_zero(),
        ),
        feature(CopperKind::Via, "VCC", "B.Cu", Scalar::from(4)),
    ]);
    assert!(access(&board, &[point(0.into())], &Scalar::one(), &Scalar::one()).is_empty());
}
#[test]
fn access_both_sides_skip_parity_geometry() {
    let mut point = point(scalar("1e400"));
    point.access_side = Some(Ipc356AccessSide::Both);
    point.feature_type = Some(Ipc356FeatureType::ThroughHole);
    let board = board(vec![feature(
        CopperKind::Pad,
        "VCC",
        "B.Cu",
        unknown_zero(),
    )]);
    assert!(access(&board, &[point], &Scalar::one(), &unknown_zero()).is_empty());
}
#[test]
fn access_spacing_floor_needs_no_threshold_order() {
    let board = board(vec![feature(
        CopperKind::Pad,
        "VCC",
        "B.Cu",
        scalar("0.75"),
    )]);
    ordinary(
        &access(
            &board,
            &[point(0.into())],
            &Scalar::one(),
            &(scalar("0.25") + unknown_zero()),
        ),
        1,
        "testpoint-accessibility-readiness",
    );
}
#[test]
fn access_spacing_is_native_below_one_float_ulp() {
    let a = point(Scalar::zero());
    let b = point(Scalar::from(2) - scalar("1e-50"));
    ordinary(
        &access(
            &board(vec![]),
            &[a.clone(), b],
            &Scalar::one(),
            &Scalar::one(),
        ),
        1,
        "testpoint-accessibility-readiness",
    );
    let b = point(Scalar::from(2) + scalar("1e-50"));
    assert!(access(&board(vec![]), &[a, b], &Scalar::one(), &Scalar::one()).is_empty());
}
#[test]
fn access_huge_spacing_threshold_is_native() {
    ordinary(
        &access(
            &board(vec![]),
            &[point(0.into()), point(3.into())],
            &Scalar::one(),
            &scalar("1e400"),
        ),
        1,
        "testpoint-accessibility-readiness",
    );
}
#[test]
fn access_nonpositive_sum_of_radii_and_spacing_cannot_match() {
    assert!(
        access(
            &board(vec![]),
            &[point(0.into()), point(0.into())],
            &Scalar::one(),
            &Scalar::from(-1)
        )
        .is_empty()
    );
}
#[test]
fn access_missing_diameter_preserves_spacing_source_indices() {
    let mut missing = point(100.into());
    missing.diameter = None;
    let mut left = point(0.into());
    left.net = "LEFT".into();
    let mut right = point(1.into());
    right.net = "RIGHT".into();
    let findings = access(
        &board(vec![]),
        &[missing, left, right],
        &Scalar::one(),
        &Scalar::one(),
    );
    ordinary(&findings, 2, "testpoint-accessibility-readiness");
    assert_eq!(findings[1].layers, ["net:LEFT", "net:RIGHT"]);
    assert_eq!(findings[1].locations, [[0.0, 0.0], [1.0, 0.0]]);
}
#[test]
fn access_outline_source_failure_remains_required() {
    let mut board = board(vec![]);
    board.board_outline = Some(rectangle(0.0, 4.0).with_exact_construction_error("failed outline"));
    uncertain(&access(
        &board,
        &[point(0.into())],
        &Scalar::one(),
        &Scalar::one(),
    ));
}
#[test]
fn access_outline_closed_clearance_is_native() {
    let mut board = board(vec![]);
    board.board_outline = Some(rectangle(0.0, 4.0));
    ordinary(
        &testpoint_accessibility_readiness(
            &board,
            &[point(0.into())],
            &Scalar::one(),
            &Scalar::one(),
            &scalar("1.5"),
        ),
        1,
        "testpoint-accessibility-readiness",
    );
    assert!(
        testpoint_accessibility_readiness(
            &board,
            &[point(0.into())],
            &Scalar::one(),
            &Scalar::one(),
            &(scalar("1.5") - scalar("1e-50"))
        )
        .is_empty()
    );
    uncertain(&testpoint_accessibility_readiness(
        &board,
        &[point(0.into())],
        &Scalar::one(),
        &Scalar::one(),
        &(scalar("1.5") + unknown_zero()),
    ));
}
#[test]
fn copper_same_net_metadata_does_not_require_unused_geometry() {
    let mut feature = feature(CopperKind::Pad, "vcc", "F.Cu", unknown_zero());
    feature.region = feature
        .region
        .with_exact_construction_error("same net unused");
    let mut point = point(unknown_zero());
    point.diameter = Some(Scalar::from(-1));
    assert!(
        clearance(
            &board(vec![feature]),
            point,
            &unknown_zero(),
            &unknown_zero()
        )
        .is_empty()
    );
}
#[test]
fn copper_empty_shapes_are_certified_absence() {
    let mut feature = feature(CopperKind::Pad, "OTHER", "F.Cu", unknown_zero());
    feature.region = polygons_to_profile(Vec::new(), None);
    let mut point = point(unknown_zero());
    point.diameter = Some(unknown_zero());
    assert!(
        clearance(
            &board(vec![feature]),
            point,
            &unknown_zero(),
            &unknown_zero()
        )
        .is_empty()
    );
}
#[test]
fn copper_shape_failure_survives_unavailable_enclosure() {
    let mut feature = feature(CopperKind::Pad, "OTHER", "F.Cu", Scalar::from(100));
    feature.region = feature
        .region
        .with_exact_construction_error("required copper shape");
    uncertain(&clearance(
        &board(vec![feature]),
        point(0.into()),
        &Scalar::one(),
        &Scalar::one(),
    ));
}
#[test]
fn copper_minimum_radius_can_prove_a_hit_without_maximum_order() {
    let mut feature = feature(CopperKind::Pad, "OTHER", "F.Cu", Scalar::one());
    feature.region = rectangle(1.0, 1.0);
    let mut point = point(0.into());
    point.diameter = Some(Scalar::one() + unknown_zero());
    ordinary(
        &clearance(
            &board(vec![feature]),
            point,
            &Scalar::one(),
            &Scalar::zero(),
        ),
        1,
        "testpoint-copper-clearance-readiness",
    );
}
#[test]
fn copper_shape_candidate_does_not_depend_on_authored_center() {
    let mut feature = feature(CopperKind::Pad, "OTHER", "F.Cu", Scalar::from(1000));
    feature.region = rectangle(0.0, 1.0);
    let findings = clearance(
        &board(vec![feature]),
        point(0.into()),
        &Scalar::one(),
        &Scalar::zero(),
    );
    ordinary(&findings, 1, "testpoint-copper-clearance-readiness");
    assert_eq!(findings[0].locations, [[0.0, 0.0], [1000.0, 0.0]]);
}
#[test]
fn circle_index_retains_missing_unknown_and_huge_entries() {
    use crate::checks::spatial::NativeCircleSpatialIndex;
    let centers = [
        point(0.into()),
        point(0.into()),
        point(100.into()),
        point(1.into()),
        point(scalar("1e400")),
    ];
    let sizes = [
        None,
        Some(Scalar::one()),
        Some(unknown_zero()),
        Some(Scalar::one()),
        Some(Scalar::one()),
    ];
    let index = NativeCircleSpatialIndex::new(
        centers
            .iter()
            .zip(&sizes)
            .map(|(p, d)| (&p.location, d.as_ref())),
        &Scalar::zero(),
    );
    let candidates = index.later_candidates_within_spacing(1, &Scalar::zero());
    assert_eq!(candidates, [2, 3, 4]);
    assert_eq!(
        index.later_candidates_within_spacing(0, &scalar("1e400")),
        [1, 2, 3, 4]
    );
}

#[derive(Clone, Copy)]
enum Rule {
    Thermal,
    Coverage,
    Access,
    Copper,
}
const RULES: [Rule; 4] = [Rule::Thermal, Rule::Coverage, Rule::Access, Rule::Copper];
impl Rule {
    fn run(self, board: &BoardModel, points: &[Ipc356Point]) -> Vec<Violation> {
        match self {
            Self::Thermal => thermal_copper_area_readiness(board, &[], &Scalar::one()),
            Self::Coverage => testpoint_coverage_readiness(board, points, &[]),
            Self::Access => access(board, points, &Scalar::one(), &Scalar::one()),
            Self::Copper => testpoint_copper_clearance_readiness(
                board,
                points,
                &[],
                &Scalar::one(),
                &Scalar::one(),
                &Scalar::zero(),
            ),
        }
    }
    fn failed(self) -> (BoardModel, Vec<Ipc356Point>) {
        let huge = scalar("1e400");
        match self {
            Self::Thermal | Self::Coverage => (
                board(vec![feature(CopperKind::Pad, "VCC", "F.Cu", huge)]),
                Vec::new(),
            ),
            Self::Access => {
                let mut point = point(huge);
                point.soldermask = Some(Ipc356Soldermask::Covered);
                (board(vec![]), vec![point])
            }
            Self::Copper => (
                board(vec![feature(CopperKind::Pad, "OTHER", "F.Cu", huge)]),
                vec![point(0.into())],
            ),
        }
    }
    fn check(self) -> crate::Check {
        match self {
            Self::Thermal => crate::Check::ThermalCopperAreaReadiness,
            Self::Coverage => crate::Check::TestpointCoverageReadiness,
            Self::Access => crate::Check::TestpointAccessibilityReadiness,
            Self::Copper => crate::Check::TestpointCopperClearanceReadiness,
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
fn warm_checks_retain_exhaustion_and_retry() {
    let board = thermal_board(0.into(), scalar("0.5"));
    let points = [point(0.into())];
    for rule in RULES {
        assert!(rule.run(&board, &points).is_empty());
    }
    let denied = hypercurve::evaluate_with_budget(
        crate::PREDICATE_POLICY,
        hypercurve::WorkBudget::ZERO,
        || {
            deny_refinement();
            for rule in RULES {
                uncertain(&rule.run(&board, &points));
            }
        },
    );
    assert!(denied.result.is_err());
    for rule in RULES {
        assert!(rule.run(&board, &points).is_empty());
    }
}
#[test]
fn empty_checks_retain_exhaustion() {
    let denied = hypercurve::evaluate_with_budget(
        crate::PREDICATE_POLICY,
        hypercurve::WorkBudget::ZERO,
        || {
            deny_refinement();
            for rule in RULES {
                uncertain(&rule.run(&board(vec![]), &[]));
            }
        },
    );
    assert!(denied.result.is_err());
}
#[test]
fn preview_cannot_decide_unknown_access_or_thermal_predicates() {
    let unknown = Scalar::one() + unknown_zero();
    let evaluate = || {
        let thermal =
            thermal_copper_area_readiness(&thermal_board(0.into(), 1.into()), &[], &unknown);
        let access = access(&board(vec![]), &[point(0.into())], &unknown, &Scalar::one());
        uncertain(&thermal);
        uncertain(&access);
        serde_json::to_vec(&(thermal, access)).unwrap()
    };
    let expected = evaluate();
    let preview =
        hypercurve::evaluate_under(hyperlimit::PredicatePolicy::APPROXIMATE_512, evaluate)
            .certified()
            .unwrap();
    assert_eq!(expected, preview);
}
#[test]
fn fixture_uncertainty_blocks_readiness_receipts() {
    for rule in RULES {
        let (board, points) = rule.failed();
        let mut findings = Vec::new();
        let coverage = crate::ReadinessRunner::new([rule.check()])
            .run(&mut findings, |_, sink| {
                sink.extend(rule.run(&board, &points));
                Ok::<_, std::convert::Infallible>(crate::CheckRunDisposition::Executed)
            })
            .unwrap();
        assert_eq!(
            coverage.checks[0].status,
            crate::CheckExecutionStatus::Uncertain
        );
        assert_eq!(coverage.blocking_count(), 1);
        uncertain(&findings);
        assert_eq!(findings[0].layers, [board.source]);
    }
}
