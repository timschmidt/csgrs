use super::drill::{drill_spacing, drill_table_consistency};
use super::mechanical::{castellation_pitch_readiness, mounting_hole_spacing_readiness};
use crate::geometry::{polygons_to_profile, rect_polygon};
use crate::ipc356::Ipc356Point;
use crate::kicad::{BoardModel, DrillFeature};
use crate::scalar::scalar;
use crate::{Scalar, Violation};

fn drill(x: Scalar, y: Scalar, diameter: Scalar) -> DrillFeature {
    DrillFeature {
        location: [x, y],
        diameter,
        plated: true,
        net: None,
    }
}
fn point(x: Scalar, diameter: Option<Scalar>) -> Ipc356Point {
    Ipc356Point {
        net: "test".into(),
        reference: None,
        pin: None,
        location: [x, Scalar::zero()],
        diameter,
        access_side: None,
        feature_type: None,
        soldermask: None,
    }
}
fn board(drills: Vec<DrillFeature>) -> BoardModel {
    BoardModel {
        source: "native drill fixture".into(),
        copper: vec![],
        drills,
        board_outline: Some(polygons_to_profile(
            vec![rect_polygon([10.0, 10.0], [20.0, 20.0], 0.0)],
            None,
        )),
        panel_features: None,
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
fn pair() -> Vec<DrillFeature> {
    vec![
        drill(0.into(), 0.into(), 1.into()),
        drill(2.into(), 0.into(), 1.into()),
    ]
}

#[test]
fn unprojectable_ipc_record_cannot_shift_later_record_identity() {
    let extra = [drill(0.into(), 0.into(), 1.into())];
    let points = [
        point(scalar("1e400"), Some(3.into())),
        point(0.into(), Some(2.into())),
    ];
    let findings = drill_table_consistency(&[], &extra, &points, &scalar("0.1"));
    assert_eq!(findings.len(), 1);
    assert_eq!(findings[0].check, "drill-table-consistency");
    assert!(findings[0].message.as_ref().unwrap().contains("2.000000"));
}

#[test]
fn unprojectable_drill_center_becomes_uncertainty_in_required_report() {
    let drills = [
        drill(scalar("1e400"), 0.into(), 1.into()),
        drill(scalar("1e400"), 0.into(), 1.into()),
    ];
    uncertain(&drill_spacing(&drills, &[], &scalar("0.1")));
}

#[test]
fn huge_native_diameter_cannot_disappear_from_spacing_candidates() {
    let drills = [
        drill(0.into(), 0.into(), scalar("1e400")),
        drill(10.into(), 0.into(), 1.into()),
    ];
    let findings = drill_spacing(&drills, &[], &scalar("0.1"));
    assert_eq!(findings.len(), 1);
    assert_eq!(findings[0].check, "drill-spacing");
}

#[test]
fn huge_native_spacing_does_not_require_finite_projection() {
    let findings = drill_spacing(&pair(), &[], &scalar("1e400"));
    assert_eq!(findings.len(), 1);
    assert_eq!(findings[0].check, "drill-spacing");
}

#[test]
fn unresolved_spacing_cannot_be_a_definite_violation() {
    uncertain(&drill_spacing(
        &pair(),
        &[],
        &(Scalar::one() + unknown_zero()),
    ));
}

#[test]
fn unresolved_drill_match_cannot_be_silently_skipped() {
    let left = [drill(0.into(), 0.into(), 1.into())];
    let right = [drill(Scalar::one() + unknown_zero(), 0.into(), 3.into())];
    uncertain(&drill_table_consistency(&left, &right, &[], &Scalar::one()));
}

#[test]
fn unresolved_diameter_conflict_cannot_be_silently_skipped() {
    let left = [drill(0.into(), 0.into(), 1.into())];
    let right = [drill(0.into(), 0.into(), Scalar::from(2) + unknown_zero())];
    uncertain(&drill_table_consistency(&left, &right, &[], &Scalar::one()));
}

#[test]
fn unresolved_hardware_spacing_remains_uncertain() {
    let board = board(vec![
        drill(0.into(), 0.into(), 3.into()),
        drill(4.into(), 0.into(), 3.into()),
    ]);
    uncertain(&mounting_hole_spacing_readiness(
        &board,
        &(Scalar::one() + unknown_zero()),
    ));
}

#[test]
fn unresolved_castellation_spacing_remains_uncertain() {
    let board = board(vec![
        drill(0.into(), 0.into(), 3.into()),
        drill(0.into(), 4.into(), 3.into()),
    ]);
    uncertain(&castellation_pitch_readiness(
        &board,
        &(Scalar::one() + unknown_zero()),
    ));
}

#[test]
fn unresolved_castellation_membership_remains_uncertain() {
    let board = board(vec![drill(
        scalar("1.5") + unknown_zero(),
        5.into(),
        1.into(),
    )]);
    uncertain(&castellation_pitch_readiness(&board, &Scalar::one()));
}

#[test]
fn exact_spacing_boundary_and_nearby_distinct_threshold_are_preserved() {
    assert!(drill_spacing(&pair(), &[], &Scalar::one()).is_empty());
    assert_eq!(
        drill_spacing(&pair(), &[], &scalar("1.0000000000000001")).len(),
        1
    );
}

#[test]
fn ipc_records_without_diameter_do_not_require_match_certificates() {
    let points = [point(Scalar::one() + unknown_zero(), None)];
    let extra = [drill(0.into(), 0.into(), 1.into())];
    assert!(drill_table_consistency(&[], &extra, &points, &Scalar::one()).is_empty());
}
