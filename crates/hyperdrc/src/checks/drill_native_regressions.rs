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
    let isolated = drill_table_consistency(&[], &extra, &points[1..], &scalar("0.1"));
    assert_eq!(
        serde_json::to_value(findings).unwrap(),
        serde_json::to_value(isolated).unwrap()
    );
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

#[test]
fn board_ipc_check_preserves_unknown_matches_and_diameter_orders() {
    let board = board(vec![drill(0.into(), 0.into(), 1.into())]);
    for points in [
        vec![point(Scalar::one() + unknown_zero(), Some(3.into()))],
        vec![point(0.into(), Some(Scalar::from(2) + unknown_zero()))],
    ] {
        uncertain(&super::board::ipc356_drill_diameter(
            &board,
            &points,
            &Scalar::one(),
        ));
    }
}

#[test]
fn board_ipc_check_preserves_unprojectable_matches_until_reporting() {
    let board = board(vec![drill(scalar("1e400"), 0.into(), 1.into())]);
    let points = [point(scalar("1e400"), Some(2.into()))];
    uncertain(&super::board::ipc356_drill_diameter(
        &board,
        &points,
        &scalar("0.1"),
    ));
}

#[test]
fn native_drill_candidates_retain_every_certified_spacing_violation() {
    let root = Scalar::from(2).sqrt().unwrap();
    let drills = vec![
        drill(0.into(), 0.into(), 0.into()),
        drill(scalar("1e-400"), 0.into(), scalar("1e-400")),
        drill(scalar("0.5"), 0.into(), 1.into()),
        drill(root.clone(), root.clone(), root),
        drill(
            scalar("1.0000000000000000001"),
            scalar("0.1"),
            scalar("0.2"),
        ),
        drill(scalar("1e400"), 0.into(), 1.into()),
        drill(scalar("1e400") + Scalar::one(), 0.into(), 1.into()),
        drill(10.into(), 0.into(), scalar("1e400")),
    ];
    for spacing in [
        scalar("-1"),
        Scalar::zero(),
        scalar("0.1"),
        Scalar::one(),
        scalar("1e400"),
    ] {
        let index = super::spatial::DrillSpatialIndex::new(&drills, &spacing);
        for (i, left) in drills.iter().enumerate() {
            let candidates = index.later_candidates_within_spacing(i, &spacing);
            assert!(candidates.windows(2).all(|pair| pair[0] < pair[1]));
            assert!(candidates.iter().all(|&candidate| candidate > i));
            for (j, right) in drills.iter().enumerate().skip(i + 1) {
                let gap = super::measurements::drill_edge_gap(left, right).unwrap();
                if crate::scalar::try_compare(&gap, &spacing).is_ok_and(|order| order.is_lt()) {
                    assert!(candidates.contains(&j), "lost pair {i}, {j}");
                }
            }
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
fn cold_and_warm_drill_indices_retain_candidates_under_budget_denial() {
    let drills = pair();
    let spacing = Scalar::one();
    let warm = super::spatial::DrillSpatialIndex::new(&drills, &spacing);
    let result = hypercurve::evaluate_with_budget(
        crate::PREDICATE_POLICY,
        hypercurve::WorkBudget::ZERO,
        || {
            deny_refinement();
            let cold = super::spatial::DrillSpatialIndex::new(&drills, &spacing);
            for index in [&warm, &cold] {
                assert_eq!(index.later_candidates_within_spacing(0, &spacing), vec![1]);
            }
            assert!(super::measurements::drill_edge_gap(&drills[0], &drills[1]).is_err());
            uncertain(&drill_spacing(&drills, &[], &spacing));
            uncertain(&drill_table_consistency(&drills, &drills, &[], &spacing));
        },
    );
    assert!(result.result.is_err());
    assert!(drill_spacing(&drills, &[], &spacing).is_empty());
    assert!(drill_table_consistency(&drills, &drills, &[], &spacing).is_empty());
}

#[test]
fn castellation_selection_uses_native_outline_independent_of_report_samples() {
    let mut board = board(vec![
        drill(9.into(), 0.into(), 1.into()),
        drill(9.into(), 1.into(), 1.into()),
    ]);
    let mut previous = None;
    for samples in [17, 128] {
        board.board_outline = Some(polygons_to_profile(
            vec![crate::geometry::circle_polygon([0.0, 0.0], 10.0, samples)],
            None,
        ));
        let findings = castellation_pitch_readiness(&board, &scalar("0.6"));
        assert_eq!(findings.len(), 1);
        assert_eq!(findings[0].check, "castellation-pitch-readiness");
        let current = serde_json::to_value(findings).unwrap();
        if let Some(previous) = previous {
            assert_eq!(previous, current);
        }
        previous = Some(current);
    }
}

#[test]
fn negative_spacing_preserves_signed_overlap_semantics() {
    let drills = [
        drill(0.into(), 0.into(), 2.into()),
        drill(scalar("0.5"), 0.into(), 2.into()),
    ];
    assert_eq!(drill_spacing(&drills, &[], &scalar("-1")).len(), 1);
    assert!(drill_spacing(&drills, &[], &scalar("-1.5")).is_empty());
}

#[test]
fn invalid_drill_radius_fails_when_required_for_spacing() {
    let drills = [
        drill(0.into(), 0.into(), (-1).into()),
        drill(0.into(), 0.into(), 1.into()),
    ];
    uncertain(&drill_spacing(&drills, &[], &Scalar::one()));
}

#[test]
fn hardware_selection_retains_unknown_diameter_in_spacing_and_distribution() {
    let board = board(vec![drill(
        0.into(),
        0.into(),
        Scalar::one() + unknown_zero(),
    )]);
    uncertain(&mounting_hole_spacing_readiness(&board, &scalar("0.5")));
    uncertain(&super::mechanical::mounting_hole_distribution_readiness(
        &board,
        &Scalar::from(4),
    ));
}

#[test]
fn unresolved_drill_spacing_becomes_a_blocking_readiness_receipt() {
    let mut findings = Vec::new();
    let coverage = crate::ReadinessRunner::new([crate::Check::DrillSpacing])
        .run(&mut findings, |_, sink| {
            sink.extend(drill_spacing(
                &pair(),
                &[],
                &(Scalar::one() + unknown_zero()),
            ));
            Ok::<_, std::convert::Infallible>(crate::CheckRunDisposition::Executed)
        })
        .unwrap();
    assert_eq!(
        coverage.checks[0].status,
        crate::CheckExecutionStatus::Uncertain
    );
    assert_eq!(coverage.blocking_count(), 1);
    assert_eq!(findings[0].layers, ["drills"]);
}
