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
        match self {
            Self::Edge => {
                edge_stitching_readiness(board, &[], &scalar("0.2"), radius, &Scalar::zero())
            }
            Self::Chassis => chassis_stitching_readiness(board, &[], radius),
            Self::Return => return_path_readiness(board, radius, &[]),
            Self::Power => power_via_array_readiness(board, &[], radius),
            Self::PairProximity => differential_pair_via_proximity_readiness(board, &[], radius),
            Self::PairReturn => differential_pair_via_return_readiness(board, &[], radius),
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
