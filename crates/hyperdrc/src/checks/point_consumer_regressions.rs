use super::*;
use crate::geometry::{polygons_to_profile, rect_polygon};
use crate::kicad::{BoardModel, CopperFeature, CopperKind, DrillFeature};
use crate::scalar::scalar;

#[derive(Clone, Copy, Debug)]
enum Rule {
    Edge,
    Chassis,
    Return,
    Power,
    PairProximity,
    PairReturn,
    Mouse,
}

const RULES: [Rule; 7] = [
    Rule::Edge,
    Rule::Chassis,
    Rule::Return,
    Rule::Power,
    Rule::PairProximity,
    Rule::PairReturn,
    Rule::Mouse,
];

fn copper(net: &str, x: Scalar) -> CopperFeature {
    CopperFeature {
        layer: "F.Cu".into(),
        net: Some(net.into()),
        kind: CopperKind::Via,
        // Public anchors are independent of the carrier. Keep the edge-band
        // fixture finite while exercising native anchors at their own boundary.
        region: polygons_to_profile(vec![rect_polygon([0.95, 0.0], [0.1, 0.1], 0.0)], None),
        location: [x, Scalar::zero()],
    }
}

fn drill(x: Scalar) -> DrillFeature {
    DrillFeature {
        location: [x, Scalar::zero()],
        diameter: scalar("0.4"),
        plated: false,
        net: None,
    }
}

impl Rule {
    fn board(self, center: Scalar, neighbor: Scalar) -> BoardModel {
        let drills = if matches!(self, Self::Mouse) {
            vec![drill(center.clone()), drill(neighbor.clone())]
        } else {
            vec![]
        };
        let copper = match self {
            Self::Chassis => vec![copper("SHIELD", center), copper("GND", neighbor)],
            Self::Power => vec![copper("VCC", center), copper("VCC", neighbor)],
            Self::PairProximity => vec![copper("USB_DP", center), copper("USB_DM", neighbor)],
            Self::PairReturn => vec![
                copper("USB_DP", center.clone()),
                copper("USB_DM", center),
                copper("GND", neighbor),
            ],
            Self::Mouse => vec![],
            _ => vec![
                copper("USB_DP", center.clone()),
                copper("GND", neighbor.clone()),
            ],
        };
        BoardModel {
            source: "native point consumer fixture".into(),
            copper,
            drills,
            board_outline: Some(polygons_to_profile(
                vec![rect_polygon([0.0, 0.0], [2.0, 2.0], 0.0)],
                None,
            )),
            panel_features: None,
        }
    }

    fn run(self, board: &BoardModel, radius: &Scalar) -> Vec<Violation> {
        self.run_selected(board, radius, &[])
    }

    fn run_selected(
        self,
        board: &BoardModel,
        radius: &Scalar,
        layers: &[String],
    ) -> Vec<Violation> {
        match self {
            Self::Edge => {
                edge_stitching_readiness(board, layers, &scalar("0.2"), radius, &Scalar::zero())
            }
            Self::Chassis => chassis_stitching_readiness(board, layers, radius),
            Self::Return => return_path_readiness(board, radius, layers),
            Self::Power => power_via_array_readiness(board, layers, radius),
            Self::PairProximity => differential_pair_via_proximity_readiness(board, layers, radius),
            Self::PairReturn => differential_pair_via_return_readiness(board, layers, radius),
            Self::Mouse => mouse_bite_readiness(
                board,
                &[],
                &scalar("0.2"),
                &scalar("0.5"),
                &Scalar::zero(),
                radius,
            ),
        }
    }
}

fn unknown_zero() -> Scalar {
    let sine = Scalar::e().sin();
    let cosine = Scalar::e().cos();
    &sine * &sine + &cosine * &cosine - Scalar::one()
}

fn uncertain(findings: &[Violation]) {
    assert_eq!(findings.len(), 1, "{findings:?}");
    assert_eq!(findings[0].check, "geometry-uncertainty", "{findings:?}");
}

macro_rules! consumer {
    ($name:ident, $rule:ident) => {
        mod $name {
            use super::*;
            #[test]
            fn huge_native_centers_do_not_require_report_values() {
                let x = scalar("1e400");
                let board = Rule::$rule.board(x.clone(), x + Scalar::one());
                assert!(Rule::$rule.run(&board, &2.into()).is_empty());
            }
            #[test]
            fn huge_native_radius_preserves_neighbors() {
                let board = Rule::$rule.board(0.into(), 1.into());
                assert!(Rule::$rule.run(&board, &scalar("1e400")).is_empty());
            }
            #[test]
            fn unresolved_radius_is_not_certified_absence() {
                let board = Rule::$rule.board(0.into(), 1.into());
                uncertain(&Rule::$rule.run(&board, &(Scalar::one() + unknown_zero())));
            }
        }
    };
}

consumer!(edge, Edge);
consumer!(chassis, Chassis);
consumer!(return_path, Return);
consumer!(power, Power);
consumer!(pair_proximity, PairProximity);
consumer!(pair_return, PairReturn);
consumer!(mouse, Mouse);

#[test]
fn chassis_native_separation_survives_equal_float_projections() {
    let x = scalar("9007199254740992");
    let board = Rule::Chassis.board(x.clone(), x + Scalar::one());
    let findings = Rule::Chassis.run(&board, &scalar("0.5"));
    assert_eq!(findings.len(), 1);
    assert_eq!(findings[0].check, "chassis-stitching-readiness");
}

#[test]
fn edge_native_separation_survives_equal_float_projections() {
    let x = scalar("9007199254740992");
    let board = Rule::Edge.board(x.clone(), x + Scalar::one());
    let findings = Rule::Edge.run(&board, &scalar("0.5"));
    assert_eq!(findings.len(), 1);
    assert_eq!(findings[0].check, "edge-stitching-readiness");
}

#[test]
fn chassis_negative_radius_cannot_become_positive_by_squaring() {
    let board = Rule::Chassis.board(0.into(), 1.into());
    assert_eq!(Rule::Chassis.run(&board, &(-2).into()).len(), 1);
}

#[test]
fn edge_negative_radius_cannot_become_positive_by_squaring() {
    let board = Rule::Edge.board(0.into(), 1.into());
    assert_eq!(Rule::Edge.run(&board, &(-2).into()).len(), 1);
}

#[test]
fn chassis_negative_radius_rejects_coincident_centers() {
    let board = Rule::Chassis.board(0.into(), 0.into());
    assert_eq!(Rule::Chassis.run(&board, &(-2).into()).len(), 1);
}

#[test]
fn edge_negative_radius_rejects_coincident_centers() {
    let board = Rule::Edge.board(0.into(), 0.into());
    assert_eq!(Rule::Edge.run(&board, &(-2).into()).len(), 1);
}

#[test]
fn mouse_unresolved_diameter_cannot_disappear_from_candidates() {
    let mut board = Rule::Mouse.board(0.into(), 1.into());
    board.drills[0].diameter = scalar("0.5") + unknown_zero();
    uncertain(&Rule::Mouse.run(&board, &2.into()));
}

#[test]
fn mouse_irrelevant_nearest_order_does_not_block_valid_pitch() {
    let mut board = Rule::Mouse.board(0.into(), 1.into());
    board.drills.push(drill(Scalar::one() + unknown_zero()));
    assert!(Rule::Mouse.run(&board, &2.into()).is_empty());
}

#[test]
fn finite_closed_radius_and_strict_outside_are_preserved() {
    for rule in RULES {
        let board = rule.board(0.into(), 1.into());
        assert!(rule.run(&board, &Scalar::one()).is_empty(), "{rule:?}");
        let outside = rule.run(&board, &scalar("0.9999999999999999"));
        assert!(!outside.is_empty(), "{rule:?}");
        assert!(
            outside
                .iter()
                .all(|finding| finding.check != "geometry-uncertainty"),
            "{rule:?}: {outside:?}"
        );
    }
}

#[test]
fn certified_ground_match_wins_after_an_unresolved_candidate() {
    for rule in [Rule::Edge, Rule::Chassis, Rule::Return, Rule::PairReturn] {
        let mut board = rule.board(0.into(), Scalar::one() + unknown_zero());
        board.copper.push(copper("GND", scalar("0.5")));
        assert!(rule.run(&board, &Scalar::one()).is_empty(), "{rule:?}");
    }
}

#[test]
fn required_unprojectable_finding_markers_fail_typed_for_every_consumer() {
    for rule in RULES {
        let x = scalar("1e400");
        let board = rule.board(x.clone(), x + Scalar::from(10));
        uncertain(&rule.run(&board, &Scalar::one()));
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
fn exhausted_checks_reject_warm_results_and_retry_after_scope() {
    let boards = RULES.map(|rule| rule.board(0.into(), 1.into()));
    for (rule, board) in RULES.into_iter().zip(&boards) {
        assert!(rule.run(board, &2.into()).is_empty());
    }
    let denied = hypercurve::evaluate_with_budget(
        crate::PREDICATE_POLICY,
        hypercurve::WorkBudget::ZERO,
        || {
            deny_refinement();
            for (rule, board) in RULES.into_iter().zip(&boards) {
                uncertain(&rule.run(board, &2.into()));
            }
        },
    );
    assert!(denied.result.is_err());
    for (rule, board) in RULES.into_iter().zip(&boards) {
        assert!(rule.run(board, &2.into()).is_empty());
    }
}

#[test]
fn exhausted_empty_fast_paths_retain_failure() {
    let mut board = Rule::Mouse.board(0.into(), 1.into());
    board.drills.clear();
    board.board_outline = None;
    let denied = hypercurve::evaluate_with_budget(
        crate::PREDICATE_POLICY,
        hypercurve::WorkBudget::ZERO,
        || {
            deny_refinement();
            for rule in RULES {
                uncertain(&rule.run(&board, &Scalar::one()));
            }
        },
    );
    assert!(denied.result.is_err());
}

#[test]
fn enclosing_preview_cannot_decide_unresolved_native_radius() {
    for rule in RULES {
        let board = rule.board(0.into(), 1.into());
        let radius = Scalar::one() + unknown_zero();
        let direct = serde_json::to_vec(&rule.run(&board, &radius)).unwrap();
        let preview =
            hypercurve::evaluate_under(hyperlimit::PredicatePolicy::APPROXIMATE_512, || {
                let findings = rule.run(&board, &radius);
                uncertain(&findings);
                serde_json::to_vec(&findings).unwrap()
            })
            .certified()
            .unwrap();
        assert_eq!(direct, preview, "{rule:?}");
    }
}

#[test]
fn every_consumer_retains_a_blocking_uncertainty_receipt() {
    use crate::Check;
    let checks = [
        Check::EdgeStitchingReadiness,
        Check::ChassisStitchingReadiness,
        Check::ReturnPathReadiness,
        Check::PowerViaArrayReadiness,
        Check::DifferentialPairViaProximityReadiness,
        Check::DifferentialPairViaReturnReadiness,
        Check::MouseBiteReadiness,
    ];
    for (rule, check) in RULES.into_iter().zip(checks) {
        let board = rule.board(0.into(), 1.into());
        let mut findings = Vec::new();
        let coverage = crate::ReadinessRunner::new([check])
            .run(&mut findings, |_, sink| {
                sink.extend(rule.run(&board, &(Scalar::one() + unknown_zero())));
                Ok::<_, std::convert::Infallible>(crate::CheckRunDisposition::Executed)
            })
            .unwrap();
        assert_eq!(
            coverage.checks[0].status,
            crate::CheckExecutionStatus::Uncertain,
            "{rule:?}"
        );
        assert_eq!(coverage.blocking_count(), 1);
        assert_eq!(findings[0].layers, [board.source]);
    }
}

#[test]
fn selected_out_centers_do_not_require_projection_or_matching() {
    for rule in RULES
        .into_iter()
        .filter(|rule| !matches!(rule, Rule::Mouse))
    {
        let board = rule.board(scalar("1e400"), scalar("-1e400"));
        assert!(
            rule.run_selected(&board, &(Scalar::one() + unknown_zero()), &["B.Cu".into()])
                .is_empty(),
            "{rule:?}"
        );
    }
}

#[test]
fn mouse_plated_drill_does_not_require_eligibility_facts() {
    let mut board = Rule::Mouse.board(scalar("1e400"), scalar("-1e400"));
    for drill in &mut board.drills {
        drill.plated = true;
        drill.diameter = scalar("0.5") + unknown_zero();
    }
    assert!(Rule::Mouse.run(&board, &scalar("1e400")).is_empty());
}

fn mouse_with_pitch(board: &BoardModel, minimum: &str, maximum: &str) -> Vec<Violation> {
    mouse_bite_readiness(
        board,
        &[],
        &scalar("0.2"),
        &scalar("0.5"),
        &scalar(minimum),
        &scalar(maximum),
    )
}

#[test]
fn mouse_certified_bad_pair_wins_over_an_unresolved_candidate() {
    let zero = unknown_zero();
    let mut board = Rule::Mouse.board(0.into(), scalar("0.4") + &zero);
    board
        .drills
        .extend([drill(scalar("0.1")), drill(scalar("0.5") + zero)]);
    let findings = mouse_with_pitch(&board, "0.4", "2");
    assert_eq!(findings.len(), 4, "{findings:?}");
    assert!(
        findings
            .iter()
            .all(|finding| finding.check == "mouse-bite-readiness")
    );
}

#[test]
fn mouse_valid_neighbor_cannot_erase_unresolved_minimum_pitch() {
    let mut board = Rule::Mouse.board(0.into(), scalar("0.4") + unknown_zero());
    board.drills.push(drill(1.into()));
    uncertain(&mouse_with_pitch(&board, "0.4", "2"));
}

#[test]
fn mouse_unresolved_maximum_membership_can_be_irrelevant() {
    let mut board = Rule::Mouse.board(0.into(), Scalar::one() + unknown_zero());
    board.drills.push(drill(scalar("0.5")));
    assert!(mouse_with_pitch(&board, "0.4", "1").is_empty());
}

#[test]
fn mouse_unresolved_bad_neighbor_order_retains_a_certified_finding() {
    let mut board = Rule::Mouse.board(0.into(), 1.into());
    board.drills.push(drill(Scalar::one() + unknown_zero()));
    let findings = mouse_with_pitch(&board, "1.5", "2");
    assert_eq!(findings.len(), 3, "{findings:?}");
    assert!(
        findings
            .iter()
            .all(|finding| finding.check == "mouse-bite-readiness")
    );
}

#[test]
fn power_coincident_layer_copies_are_not_distinct_neighbors() {
    let mut board = Rule::Power.board(0.into(), 0.into());
    board.copper[1].layer = "B.Cu".into();
    let findings = Rule::Power.run(&board, &Scalar::one());
    assert_eq!(findings.len(), 1);
    assert_eq!(findings[0].layers, ["B.Cu", "F.Cu"]);
    assert_eq!(findings[0].locations, [[0.0, 0.0]; 2]);
}

#[test]
fn power_unresolved_distinctness_cannot_become_isolation() {
    let board = Rule::Power.board(0.into(), unknown_zero());
    uncertain(&Rule::Power.run(&board, &Scalar::one()));
}

#[test]
fn power_certified_distinct_neighbor_wins_after_unresolved_distinctness() {
    let mut board = Rule::Power.board(0.into(), unknown_zero());
    board.copper.push(copper("VCC", scalar("0.5")));
    assert!(Rule::Power.run(&board, &Scalar::one()).is_empty());
}

#[test]
fn differential_certified_alternatives_resolve_both_sides() {
    let mut board = Rule::PairProximity.board(0.into(), Scalar::one() + unknown_zero());
    board.copper.extend([
        copper("USB_DP", scalar("0.5")),
        copper("USB_DM", scalar("0.5")),
    ]);
    assert!(Rule::PairProximity.run(&board, &Scalar::one()).is_empty());
}

#[test]
fn mouse_finite_nearest_bad_marker_is_preserved() {
    let mut board = Rule::Mouse.board(0.into(), scalar("0.3"));
    board.drills.push(drill(scalar("0.1")));
    let findings = mouse_with_pitch(&board, "0.4", "1.2");
    assert_eq!(findings.len(), 3);
    assert_eq!(findings[0].locations, [[0.0, 0.0], [0.1, 0.0]]);
    assert_eq!(findings[1].locations, [[0.3, 0.0], [0.1, 0.0]]);
    assert_eq!(findings[2].locations, [[0.1, 0.0], [0.0, 0.0]]);
}
