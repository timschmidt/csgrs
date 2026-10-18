use super::*;
use crate::geometry::{polygons_to_profile, rect_polygon};
use crate::kicad::{BoardModel, CopperFeature, CopperKind, DrillFeature};
use crate::scalar::scalar;

#[derive(Clone, Copy, Debug)]
enum Rule {
    Connector,
    Decoupling,
    RfFence,
    Plating,
    MountGround,
    MountPlating,
}

const RULES: [Rule; 6] = [
    Rule::Connector,
    Rule::Decoupling,
    Rule::RfFence,
    Rule::Plating,
    Rule::MountGround,
    Rule::MountPlating,
];

fn copper(net: &str, x: Scalar, kind: CopperKind) -> CopperFeature {
    CopperFeature {
        layer: "F.Cu".into(),
        net: Some(net.into()),
        kind,
        location: [x, Scalar::zero()],
        // Public authored centers are independent of the shape. Only the
        // connector edge-selection rule needs this finite edge-touching carrier.
        region: polygons_to_profile(vec![rect_polygon([0.95, 0.0], [0.1, 0.1], 0.0)], None),
    }
}

impl Rule {
    fn board(self, center: Scalar, neighbor: Scalar) -> BoardModel {
        let mut features = vec![self.neighbor(neighbor)];
        let mut drills = Vec::new();
        match self {
            Self::Connector => features.insert(0, copper("USB_DP", center, CopperKind::Pad)),
            Self::Decoupling => features.insert(0, copper("VCC", center, CopperKind::Pad)),
            Self::RfFence => features.insert(0, copper("RF", center, CopperKind::Segment)),
            _ => drills.push(DrillFeature {
                location: [center, Scalar::zero()],
                diameter: 2.into(),
                net: Some("SIG".into()),
                plated: !matches!(self, Self::MountGround),
            }),
        }
        BoardModel {
            source: "authored center fixture".into(),
            copper: features,
            drills,
            board_outline: Some(polygons_to_profile(
                vec![rect_polygon([0.0, 0.0], [2.0, 2.0], 0.0)],
                None,
            )),
            panel_features: None,
        }
    }

    fn neighbor(self, x: Scalar) -> CopperFeature {
        copper(
            if matches!(self, Self::Plating) {
                "SIG"
            } else {
                "GND"
            },
            x,
            if matches!(self, Self::RfFence) {
                CopperKind::Via
            } else {
                CopperKind::Pad
            },
        )
    }

    fn boundary(self) -> Scalar {
        if matches!(self, Self::MountGround | Self::MountPlating) {
            2.into()
        } else {
            1.into()
        }
    }

    fn check(self) -> &'static str {
        match self {
            Self::Connector => "connector-return-path-readiness",
            Self::Decoupling => "decoupling-proximity-readiness",
            Self::RfFence => "rf-via-fence-readiness",
            Self::Plating => "plating-intent",
            Self::MountGround => "mounting-hole-grounding-readiness",
            Self::MountPlating => "mounting-hole-plating-intent-readiness",
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
            Self::Connector => {
                connector_return_path_readiness(board, layers, &scalar("0.1"), radius)
            }
            Self::Decoupling => decoupling_proximity_readiness(board, layers, radius),
            Self::RfFence => rf_via_fence_readiness(board, layers, radius),
            Self::Plating => plating_intent(board, layers, radius),
            Self::MountGround => mounting_hole_grounding_readiness(board, layers, radius),
            Self::MountPlating => mounting_hole_plating_intent_readiness(board, layers, radius),
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

macro_rules! consumer {
    ($name:ident, $rule:ident) => {
        mod $name {
            use super::*;
            #[test]
            fn huge_native_centers_need_no_report_projection_for_a_match() {
                let x = scalar("1e400");
                let board = Rule::$rule.board(x.clone(), x + Scalar::one());
                assert!(Rule::$rule.run(&board, &Scalar::one()).is_empty());
            }
            #[test]
            fn huge_native_radius_preserves_matches() {
                let mut board = Rule::$rule.board(0.into(), 1.into());
                if matches!(Rule::$rule, Rule::MountGround | Rule::MountPlating) {
                    board.drills[0].diameter = scalar("1e400");
                }
                assert!(Rule::$rule.run(&board, &scalar("1e400")).is_empty());
            }
            #[test]
            fn unresolved_radius_is_not_certified_absence() {
                let board = Rule::$rule.board(0.into(), Rule::$rule.boundary());
                uncertain(&Rule::$rule.run(&board, &(Scalar::one() + unknown_zero())));
            }
            #[test]
            fn required_nonfinite_marker_fails_typed() {
                let x = scalar("1e400");
                let board = Rule::$rule.board(x.clone(), x + Scalar::from(10));
                uncertain(&Rule::$rule.run(&board, &Scalar::one()));
            }
        }
    };
}

consumer!(connector, Connector);
consumer!(decoupling, Decoupling);
consumer!(rf_fence, RfFence);
consumer!(plating, Plating);
consumer!(mount_ground, MountGround);
consumer!(mount_plating, MountPlating);

#[test]
fn closed_boundary_and_strict_outside_keep_finite_findings() {
    for rule in RULES {
        let board = rule.board(0.into(), rule.boundary());
        assert!(rule.run(&board, &Scalar::one()).is_empty(), "{rule:?}");
        let outside = rule.board(0.into(), rule.boundary() + scalar("0.000001"));
        let findings = rule.run(&outside, &Scalar::one());
        assert_eq!(findings.len(), 1, "{rule:?}: {findings:?}");
        assert_eq!(findings[0].check, rule.check());
    }
}

#[test]
fn certified_alternative_wins_after_unknown_candidate() {
    for rule in RULES {
        let mut board = rule.board(0.into(), rule.boundary() + unknown_zero());
        board.copper.push(rule.neighbor(scalar("0.5")));
        assert!(rule.run(&board, &Scalar::one()).is_empty(), "{rule:?}");
    }
}

#[test]
fn layer_specific_matching_does_not_borrow_another_layer() {
    for rule in [Rule::Connector, Rule::Decoupling, Rule::RfFence] {
        let mut board = rule.board(0.into(), scalar("0.5"));
        board.copper[1].layer = "B.Cu".into();
        let findings = rule.run(&board, &Scalar::one());
        assert_eq!(findings.len(), 1, "{rule:?}: {findings:?}");
        assert_eq!(findings[0].check, rule.check());
    }
}

#[test]
fn plated_mount_ground_net_is_sufficient_without_a_report_center() {
    let mut board = Rule::MountPlating.board(scalar("1e400"), 0.into());
    board.drills[0].net = Some("CHASSIS".into());
    assert!(Rule::MountPlating.run(&board, &Scalar::one()).is_empty());
}

#[test]
fn unknown_npth_diameter_does_not_erase_plating_evidence() {
    let mut board = Rule::Plating.board(0.into(), 2.into());
    board.drills[0].plated = false;
    board.drills[0].diameter = Scalar::from(2) + unknown_zero();
    uncertain(&Rule::Plating.run(&board, &Scalar::one()));
}

#[test]
fn negative_npth_diameter_is_a_typed_domain_failure() {
    let mut board = Rule::Plating.board(0.into(), 10.into());
    board.drills[0].plated = false;
    board.drills[0].diameter = (-1).into();
    uncertain(&Rule::Plating.run(&board, &Scalar::one()));
}

#[test]
fn unresolved_mounting_diameter_is_not_an_ineligible_hole() {
    for rule in [Rule::MountGround, Rule::MountPlating] {
        let mut board = rule.board(0.into(), 10.into());
        board.drills[0].diameter = Scalar::one() + unknown_zero();
        uncertain(&rule.run(&board, &Scalar::one()));
    }
}

#[test]
fn shared_mounting_selector_keeps_uncertainty_in_copper_keepout() {
    let mut board = Rule::MountGround.board(0.into(), 10.into());
    board.drills[0].diameter = Scalar::one() + unknown_zero();
    uncertain(&mounting_hole_copper_keepout_readiness(
        &board,
        &[],
        &Scalar::one(),
        &Scalar::zero(),
    ));
}

#[test]
fn shared_mounting_selector_keeps_uncertainty_in_edge_clearance() {
    let mut board = Rule::MountGround.board(0.into(), 10.into());
    board.drills[0].diameter = Scalar::one() + unknown_zero();
    uncertain(&mounting_hole_edge_clearance_readiness(
        &board,
        &Scalar::one(),
        &Scalar::zero(),
    ));
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
        assert!(rule.run(board, &Scalar::one()).is_empty());
    }
    let denied = hypercurve::evaluate_with_budget(
        crate::PREDICATE_POLICY,
        hypercurve::WorkBudget::ZERO,
        || {
            deny_refinement();
            for (rule, board) in RULES.into_iter().zip(&boards) {
                uncertain(&rule.run(board, &Scalar::one()));
            }
        },
    );
    assert!(denied.result.is_err());
    for (rule, board) in RULES.into_iter().zip(&boards) {
        assert!(rule.run(board, &Scalar::one()).is_empty());
    }
}

#[test]
fn exhausted_empty_fast_paths_retain_failure() {
    let mut board = Rule::Connector.board(0.into(), 1.into());
    board.copper.clear();
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
        let board = rule.board(0.into(), rule.boundary());
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
        Check::ConnectorReturnPathReadiness,
        Check::DecouplingProximityReadiness,
        Check::RfViaFenceReadiness,
        Check::PlatingIntent,
        Check::MountingHoleGroundingReadiness,
        Check::MountingHolePlatingIntentReadiness,
    ];
    for (rule, check) in RULES.into_iter().zip(checks) {
        let board = rule.board(0.into(), rule.boundary());
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
fn unrelated_layer_centers_need_no_matching_certificate() {
    for rule in [Rule::Connector, Rule::Decoupling, Rule::RfFence] {
        let mut board = rule.board(0.into(), Scalar::one() + unknown_zero());
        board.copper[1].layer = "B.Cu".into();
        let findings = rule.run(&board, &Scalar::one());
        assert_eq!(findings.len(), 1);
        assert_eq!(findings[0].check, rule.check());
        assert!(
            rule.run_selected(&board, &Scalar::one(), &["In1.Cu".into()])
                .is_empty()
        );
    }
}

#[test]
fn plated_matching_filters_net_and_kind_before_coordinates() {
    for (kind, net) in [(CopperKind::Segment, "SIG"), (CopperKind::Pad, "OTHER")] {
        let mut board = Rule::Plating.board(0.into(), Scalar::one() + unknown_zero());
        board.copper[0].kind = kind;
        board.copper[0].net = Some(net.into());
        let findings = Rule::Plating.run(&board, &Scalar::one());
        assert_eq!(findings.len(), 1);
        assert_eq!(findings[0].check, "plating-intent");
    }
    let mut board = Rule::Plating.board(0.into(), 0.into());
    board.drills[0].net = None;
    board.copper[0].net = Some("OTHER".into());
    board.drills[0].diameter = (-1).into(); // Plated matching does not use diameter.
    assert!(Rule::Plating.run(&board, &Scalar::one()).is_empty());
}

#[test]
fn npth_matching_uses_expanded_radius_for_all_copper_kinds() {
    let mut board = Rule::Plating.board(0.into(), 2.into());
    board.drills[0].plated = false;
    board.copper[0].kind = CopperKind::Segment;
    board.copper[0].net = Some("OTHER".into());
    let findings = Rule::Plating.run(&board, &Scalar::one());
    assert_eq!(findings.len(), 1);
    assert_eq!(findings[0].check, "plating-intent");
    board.copper[0].location[0] += scalar("0.000001");
    assert!(Rule::Plating.run(&board, &Scalar::one()).is_empty());
    board.copper[0].location[0] = 2.into();
    assert!(
        Rule::Plating
            .run_selected(&board, &Scalar::one(), &["B.Cu".into()])
            .is_empty()
    );
}

#[test]
fn grounded_plated_metadata_does_not_need_eligibility_or_radius() {
    let mut board = Rule::MountPlating.board(scalar("1e400"), scalar("-1e400"));
    board.drills[0].net = Some("GND".into());
    board.drills[0].diameter = Scalar::one() + unknown_zero();
    assert!(Rule::MountPlating.run(&board, &unknown_zero()).is_empty());
}

#[test]
fn mounting_eligibility_needs_no_order_between_thresholds() {
    for rule in [Rule::MountGround, Rule::MountPlating] {
        let board = rule.board(0.into(), scalar("0.5"));
        assert!(
            rule.run(&board, &(Scalar::one() + unknown_zero()))
                .is_empty()
        );
    }
}

#[test]
fn certified_mounting_ineligibility_wins_after_unknown_bound() {
    for rule in [Rule::MountGround, Rule::MountPlating] {
        let mut board = rule.board(scalar("1e400"), scalar("-1e400"));
        board.drills[0].diameter = Scalar::one() + unknown_zero();
        assert!(rule.run(&board, &Scalar::from(2)).is_empty());
    }
}

#[test]
fn negative_radius_does_not_become_a_positive_squared_match() {
    for rule in RULES {
        let board = rule.board(0.into(), 0.into());
        let findings = rule.run(&board, &Scalar::from(-3));
        assert_eq!(findings.len(), 1, "{rule:?}: {findings:?}");
        assert_eq!(findings[0].check, rule.check());
    }
}

#[test]
fn sub_ulp_center_separations_remain_outside() {
    for rule in RULES {
        let center = scalar("1e20");
        let board = rule.board(center.clone(), center + rule.boundary() + scalar("0.001"));
        let findings = rule.run(&board, &Scalar::one());
        assert_eq!(findings.len(), 1, "{rule:?}: {findings:?}");
        assert_eq!(findings[0].check, rule.check());
    }
}

#[test]
fn connector_retains_required_edge_uncertainty_and_source_failure() {
    let mut board = Rule::Connector.board(0.into(), 0.into());
    board.board_outline = Some(board.copper[0].region.clone());
    uncertain(&connector_return_path_readiness(
        &board,
        &[],
        &unknown_zero(),
        &Scalar::one(),
    ));
    board.copper[0].region = board.copper[0]
        .region
        .clone()
        .with_exact_construction_error("required connector source failure");
    uncertain(&Rule::Connector.run(&board, &Scalar::one()));
    board.copper[0].region = crate::geometry::empty_profile(None);
    assert!(Rule::Connector.run(&board, &Scalar::one()).is_empty());
}

#[test]
fn unused_copper_shape_failure_does_not_override_center_evidence() {
    for rule in RULES {
        let mut board = rule.board(0.into(), scalar("0.5"));
        for (index, feature) in board.copper.iter_mut().enumerate() {
            if matches!(rule, Rule::Connector) && index == 0 {
                continue;
            }
            feature.region = feature
                .region
                .clone()
                .with_exact_construction_error("unused copper carrier");
        }
        assert!(rule.run(&board, &Scalar::one()).is_empty(), "{rule:?}");
    }
}

#[test]
fn variable_point_query_keeps_original_indices_and_unbounded_fallbacks() {
    let points = [
        [Scalar::from(2), Scalar::zero()],
        [scalar("1e400"), Scalar::zero()],
        [Scalar::zero(), Scalar::zero()],
        [Scalar::from(100), Scalar::zero()],
    ];
    let index = super::spatial::NativePointSpatialIndex::new(&points, &Scalar::one());
    let candidates = index.candidates_near_with_radius(&points[2], &Scalar::from(2));
    assert_eq!(candidates, [0, 1, 2]);
    assert_eq!(
        index.candidates_near_with_radius(&points[2], &scalar("1e400")),
        [0, 1, 2, 3]
    );
    assert_eq!(
        index.candidates_near_with_radius(&points[1], &Scalar::one()),
        [0, 1, 2, 3]
    );
    let near_boundary =
        index.candidates_near_with_radius(&points[2], &(Scalar::from(2) + unknown_zero()));
    assert!(near_boundary.contains(&0));
    assert!(near_boundary.windows(2).all(|pair| pair[0] < pair[1]));
}

#[test]
fn unresolved_zero_diagnostic_uses_absolute_precision() {
    let mut board = Rule::Decoupling.board(0.into(), 10.into());
    board.copper.pop();
    let findings = Rule::Decoupling.run(&board, &unknown_zero());
    assert_eq!(findings.len(), 1);
    assert_eq!(findings[0].check, Rule::Decoupling.check());
    assert!(
        findings[0]
            .message
            .as_deref()
            .unwrap()
            .contains("within 0.000000")
    );
}
