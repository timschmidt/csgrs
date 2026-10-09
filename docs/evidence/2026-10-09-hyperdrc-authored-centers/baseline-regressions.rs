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
