use super::*;
use crate::geometry::{circle_polygon, rect_polygon};

fn unknown_zero() -> Scalar {
    let sine = Scalar::e().sin();
    let cosine = Scalar::e().cos();
    let zero = &sine * &sine + &cosine * &cosine - Scalar::one();
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
        "F.Paste", &paste, "F.Cu", &copper, gap,
        &crate::scalar::scalar("0.25"), &Scalar::zero(),
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
            layer: "F.Cu".into(), net: Some("GND".into()), kind: CopperKind::Via,
            region: polygon_to_profile(circle_polygon([0.0, 0.0], 0.2, 17), None),
            location,
        }],
        drills, board_outline: None, panel_features: None,
    }
}

fn drill(location: [Scalar; 2], diameter: Scalar) -> DrillFeature {
    DrillFeature { location, diameter, plated: true, net: Some("GND".into()) }
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
        .unwrap().evaluate(|| uncertainty(&tombstone(&gap)));
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
    let source = board([Scalar::one() + unknown_zero(), Scalar::zero()],
        vec![drill([Scalar::zero(), Scalar::zero()], Scalar::one())]);
    uncertainty(&exposure(&source));
}

#[test]
fn unresolved_matching_radius_cannot_choose_an_arm() {
    let source = board([Scalar::zero(), Scalar::zero()],
        vec![drill([Scalar::zero(), Scalar::zero()], crate::scalar::scalar("0.05") + unknown_zero())]);
    let matched = matching_plated_drill(&source, &source.copper[0]);
    assert!(format!("{matched:?}").starts_with("Err("), "unknown matching radius must remain a typed error");
}

#[test]
fn unresolved_first_drill_cannot_be_skipped_for_a_later_match() {
    let source = board([Scalar::zero(), Scalar::zero()], vec![
        drill([Scalar::one() + unknown_zero(), Scalar::zero()], Scalar::one()),
        drill([Scalar::zero(), Scalar::zero()], crate::scalar::scalar("0.1")),
    ]);
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
    let source = board([Scalar::zero(), Scalar::zero()], vec![
        drill([Scalar::zero(), Scalar::zero()], crate::scalar::scalar("0.1")),
        drill([Scalar::one() + unknown_zero(), Scalar::zero()], Scalar::one()),
    ]);
    let findings = exposure(&source);
    assert_eq!(findings.len(), 1, "{findings:?}");
    assert_eq!(findings[0].check, "paste-via-exposure-readiness");
}
