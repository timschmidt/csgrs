use super::board::{apply_ipc356_nets, ipc356_coverage};
use crate::ipc356::Ipc356Point;
use crate::kicad::{BoardModel, CopperFeature, CopperKind, DrillFeature};
use crate::scalar::scalar;
use crate::{PcbRegion, Scalar, Violation};

fn copper(x: Scalar) -> CopperFeature {
    let location = [x, Scalar::zero()];
    CopperFeature {
        layer: "F.Cu".into(),
        net: None,
        kind: CopperKind::Pad,
        region: PcbRegion::new(
            crate::geometry::circle_region(&location, &Scalar::one()).unwrap(),
            None,
        ),
        location,
    }
}

fn drill(x: Scalar, diameter: Scalar) -> DrillFeature {
    DrillFeature {
        location: [x, Scalar::zero()],
        diameter,
        plated: true,
        net: None,
    }
}

fn board(copper: Vec<CopperFeature>, drills: Vec<DrillFeature>) -> BoardModel {
    BoardModel {
        source: "native IPC fixture".into(),
        copper,
        drills,
        board_outline: None,
        panel_features: None,
    }
}

fn point(x: Scalar, net: &str, diameter: Option<Scalar>) -> Ipc356Point {
    Ipc356Point {
        net: net.into(),
        reference: Some("TP1".into()),
        pin: None,
        location: [x, Scalar::zero()],
        diameter,
        access_side: None,
        feature_type: None,
        soldermask: None,
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

fn metadata(board: &BoardModel) -> (Vec<Option<String>>, Vec<(Option<String>, Scalar)>) {
    (
        board
            .copper
            .iter()
            .map(|feature| feature.net.clone())
            .collect(),
        board
            .drills
            .iter()
            .map(|drill| (drill.net.clone(), drill.diameter.clone()))
            .collect(),
    )
}

#[test]
fn annotation_matches_unprojectable_copper_centers() {
    let mut board = board(vec![copper(scalar("1e400"))], vec![]);
    let points = [point(scalar("1e400"), "HUGE", None)];
    let _ = apply_ipc356_nets(&mut board, &points, &scalar("0.1"));
    assert_eq!(board.copper[0].net.as_deref(), Some("HUGE"));
}

#[test]
fn annotation_matches_unprojectable_drill_centers() {
    let mut board = board(vec![], vec![drill(scalar("1e400"), Scalar::zero())]);
    let points = [point(scalar("1e400"), "HUGE", Some(scalar("0.4")))];
    let _ = apply_ipc356_nets(&mut board, &points, &scalar("0.1"));
    assert_eq!(board.drills[0].net.as_deref(), Some("HUGE"));
    assert_eq!(board.drills[0].diameter, scalar("0.4"));
}

#[test]
fn annotation_accepts_native_tolerance_without_finite_projection() {
    let mut board = board(vec![copper(0.into())], vec![]);
    let _ = apply_ipc356_nets(
        &mut board,
        &[point(1.into(), "WIDE", None)],
        &scalar("1e400"),
    );
    assert_eq!(board.copper[0].net.as_deref(), Some("WIDE"));
}

#[test]
fn unresolved_later_match_cannot_publish_earlier_annotations() {
    let mut board = board(
        vec![copper(0.into()), copper(10.into())],
        vec![drill(0.into(), Scalar::zero())],
    );
    let original = metadata(&board);
    let points = [
        point(0.into(), "FIRST", Some(scalar("0.4"))),
        point(Scalar::from(11) + unknown_zero(), "UNKNOWN", None),
    ];
    let _ = apply_ipc356_nets(&mut board, &points, &Scalar::one());
    assert_eq!(metadata(&board), original);
}

#[test]
fn unresolved_missing_diameter_cannot_publish_net_annotation() {
    let mut board = board(
        vec![copper(0.into())],
        vec![drill(0.into(), unknown_zero())],
    );
    let original = metadata(&board);
    let _ = apply_ipc356_nets(
        &mut board,
        &[point(0.into(), "NET", Some(scalar("0.4")))],
        &Scalar::one(),
    );
    assert_eq!(metadata(&board), original);
}

#[test]
fn invalid_sidecar_diameter_cannot_enter_board_metadata() {
    let mut board = board(
        vec![copper(0.into())],
        vec![drill(0.into(), Scalar::zero())],
    );
    let original = metadata(&board);
    let _ = apply_ipc356_nets(
        &mut board,
        &[point(0.into(), "NET", Some((-1).into()))],
        &Scalar::one(),
    );
    assert_eq!(metadata(&board), original);
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
fn exhausted_annotation_leaves_metadata_unchanged() {
    let mut board = board(
        vec![copper(0.into())],
        vec![drill(0.into(), Scalar::zero())],
    );
    let original = metadata(&board);
    let result = hypercurve::evaluate_with_budget(
        crate::PREDICATE_POLICY,
        hypercurve::WorkBudget::ZERO,
        || {
            deny_refinement();
            let _ = apply_ipc356_nets(
                &mut board,
                &[point(0.into(), "NET", Some(scalar("0.4")))],
                &Scalar::one(),
            );
            assert_eq!(metadata(&board), original);
        },
    );
    assert!(result.result.is_err());
}

#[test]
fn coverage_unresolved_match_is_not_a_definite_missing_feature() {
    let board = board(vec![copper(0.into())], vec![]);
    uncertain(&ipc356_coverage(
        &board,
        &[point(Scalar::one() + unknown_zero(), "NET", None)],
        &Scalar::one(),
    ));
}

#[test]
fn coverage_unprojectable_missing_point_remains_a_required_failure() {
    let board = board(vec![], vec![]);
    uncertain(&ipc356_coverage(
        &board,
        &[point(scalar("1e400"), "NET", None)],
        &Scalar::one(),
    ));
}

#[test]
fn coverage_matches_unprojectable_native_centers() {
    let board = board(vec![copper(scalar("1e400"))], vec![]);
    assert!(
        ipc356_coverage(
            &board,
            &[point(scalar("1e400"), "NET", None)],
            &Scalar::one()
        )
        .is_empty()
    );
}

#[test]
fn coverage_accepts_native_tolerance_without_finite_projection() {
    let board = board(vec![copper(0.into())], vec![]);
    assert!(ipc356_coverage(&board, &[point(1.into(), "NET", None)], &scalar("1e400")).is_empty());
}

#[test]
fn coverage_rejects_exhausted_work_before_an_empty_fast_path() {
    let board = board(vec![], vec![]);
    let result = hypercurve::evaluate_with_budget(
        crate::PREDICATE_POLICY,
        hypercurve::WorkBudget::ZERO,
        || {
            deny_refinement();
            uncertain(&ipc356_coverage(&board, &[], &Scalar::one()));
        },
    );
    assert!(result.result.is_err());
}

#[test]
fn exact_matching_boundary_and_nearby_distinct_point_are_preserved() {
    let board = board(vec![copper(0.into())], vec![]);
    assert!(ipc356_coverage(&board, &[point(1.into(), "AT", None)], &Scalar::one()).is_empty());
    let findings = ipc356_coverage(
        &board,
        &[point(scalar("1.0000000000000001"), "BEYOND", None)],
        &Scalar::one(),
    );
    assert_eq!(findings.len(), 1);
    assert_eq!(findings[0].check, "ipc356-coverage");
}

#[test]
fn annotation_preserves_existing_values_and_first_matching_record() {
    let mut board = board(
        vec![copper(0.into()), copper(0.into())],
        vec![
            drill(0.into(), Scalar::zero()),
            drill(0.into(), scalar("0.3")),
        ],
    );
    board.copper[0].net = Some("AUTHORED".into());
    board.drills[1].net = Some("AUTHORED".into());
    let points = [
        point(0.into(), "FIRST", Some(scalar("0.4"))),
        point(0.into(), "LATER", Some(scalar("0.5"))),
    ];
    let _ = apply_ipc356_nets(&mut board, &points, &Scalar::one());
    assert_eq!(
        metadata(&board),
        (
            vec![Some("AUTHORED".into()), Some("FIRST".into())],
            vec![
                (Some("FIRST".into()), scalar("0.4")),
                (Some("AUTHORED".into()), scalar("0.3"))
            ]
        )
    );
}
