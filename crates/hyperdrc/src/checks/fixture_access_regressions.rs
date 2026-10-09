use super::*;
use crate::geometry::{polygons_to_profile, rect_polygon};
use crate::ipc356::{Ipc356AccessSide, Ipc356FeatureType, Ipc356Point, Ipc356Soldermask};
use crate::kicad::{BoardModel, CopperFeature, CopperKind};
use crate::scalar::scalar;

fn rectangle(x: f64, width: f64) -> crate::PcbRegion {
    polygons_to_profile(vec![rect_polygon([x, 0.0], [width, width], 0.0)], None)
}
fn feature(kind: CopperKind, net: &str, layer: &str, x: Scalar) -> CopperFeature {
    CopperFeature {
        kind,
        net: Some(net.into()),
        layer: layer.into(),
        location: [x, Scalar::zero()],
        region: rectangle(0.0, 0.5),
    }
}
fn board(copper: Vec<CopperFeature>) -> BoardModel {
    BoardModel {
        source: "native fixture access".into(),
        copper,
        drills: vec![],
        board_outline: None,
        panel_features: None,
    }
}
fn point(x: Scalar) -> Ipc356Point {
    Ipc356Point {
        net: "VCC".into(),
        reference: None,
        pin: None,
        location: [x, Scalar::zero()],
        diameter: Some(Scalar::one()),
        access_side: Some(Ipc356AccessSide::Top),
        feature_type: Some(Ipc356FeatureType::Smd),
        soldermask: Some(Ipc356Soldermask::Open),
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
    assert!(
        findings[0]
            .locations
            .iter()
            .flatten()
            .all(|x| x.is_finite())
    );
}
fn ordinary(findings: &[Violation], count: usize, check: &str) {
    assert_eq!(findings.len(), count, "{findings:?}");
    assert!(findings.iter().all(|v| v.check == check));
    assert!(
        findings
            .iter()
            .flat_map(|v| &v.locations)
            .flatten()
            .all(|x| x.is_finite())
    );
}
fn thermal_board(center: Scalar, zone: Scalar) -> BoardModel {
    board(vec![
        feature(CopperKind::Pad, "VCC", "F.Cu", center),
        feature(CopperKind::Zone, "VCC", "F.Cu", zone),
    ])
}
fn access(
    board: &BoardModel,
    points: &[Ipc356Point],
    diameter: &Scalar,
    spacing: &Scalar,
) -> Vec<Violation> {
    testpoint_accessibility_readiness(board, points, diameter, spacing, &Scalar::one())
}
fn clearance(
    board: &BoardModel,
    point: Ipc356Point,
    diameter: &Scalar,
    spacing: &Scalar,
) -> Vec<Violation> {
    testpoint_copper_clearance_readiness(board, &[point], &[], diameter, spacing, &Scalar::zero())
}

#[test]
fn thermal_huge_centers_need_no_projection_for_a_match() {
    let huge = scalar("1e400");
    assert!(
        thermal_copper_area_readiness(
            &thermal_board(huge.clone(), huge + Scalar::one()),
            &[],
            &Scalar::one()
        )
        .is_empty()
    );
}
#[test]
fn thermal_huge_radius_is_native() {
    assert!(
        thermal_copper_area_readiness(&thermal_board(0.into(), 1.into()), &[], &scalar("1e400"))
            .is_empty()
    );
}
#[test]
fn thermal_unknown_radius_cannot_become_absence() {
    uncertain(&thermal_copper_area_readiness(
        &thermal_board(0.into(), 1.into()),
        &[],
        &(Scalar::one() + unknown_zero()),
    ));
}
#[test]
fn thermal_required_marker_failure_is_typed() {
    let board = board(vec![feature(
        CopperKind::Pad,
        "VCC",
        "F.Cu",
        scalar("1e400"),
    )]);
    uncertain(&thermal_copper_area_readiness(&board, &[], &Scalar::one()));
}
#[test]
fn coverage_does_not_project_covered_nets() {
    let board = board(vec![feature(
        CopperKind::Pad,
        "VCC",
        "F.Cu",
        scalar("1e400"),
    )]);
    assert!(testpoint_coverage_readiness(&board, &[point(0.into())], &[]).is_empty());
}
#[test]
fn coverage_required_marker_failure_is_typed() {
    let board = board(vec![feature(
        CopperKind::Pad,
        "VCC",
        "F.Cu",
        scalar("1e400"),
    )]);
    uncertain(&testpoint_coverage_readiness(&board, &[], &[]));
}
#[test]
fn coverage_projects_only_the_three_reported_markers() {
    let mut features = (0..3)
        .map(|x| feature(CopperKind::Pad, "VCC", "F.Cu", Scalar::from(x)))
        .collect::<Vec<_>>();
    features.push(feature(CopperKind::Pad, "VCC", "F.Cu", scalar("1e400")));
    let findings = testpoint_coverage_readiness(&board(features), &[], &[]);
    ordinary(&findings, 1, "testpoint-coverage-readiness");
    assert_eq!(findings[0].locations, [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]]);
}
#[test]
fn access_huge_centers_can_prove_expected_side() {
    let huge = scalar("1e400");
    let board = board(vec![feature(CopperKind::Pad, "VCC", "F.Cu", huge.clone())]);
    assert!(access(&board, &[point(huge)], &Scalar::one(), &Scalar::one()).is_empty());
}
#[test]
fn access_metadata_marker_cannot_contain_nan() {
    let mut point = point(scalar("1e400"));
    point.soldermask = Some(Ipc356Soldermask::Covered);
    uncertain(&access(
        &board(vec![]),
        &[point],
        &Scalar::one(),
        &Scalar::one(),
    ));
}
#[test]
fn access_unknown_minimum_diameter_cannot_pass() {
    uncertain(&access(
        &board(vec![]),
        &[point(0.into())],
        &(Scalar::one() + unknown_zero()),
        &Scalar::one(),
    ));
}
#[test]
fn access_unknown_side_match_cannot_disappear() {
    let board = board(vec![feature(CopperKind::Pad, "VCC", "B.Cu", scalar("1.5"))]);
    uncertain(&access(
        &board,
        &[point(0.into())],
        &Scalar::one(),
        &(Scalar::one() + unknown_zero()),
    ));
}
#[test]
fn access_unknown_expected_side_cannot_prove_a_mismatch() {
    let board = board(vec![
        feature(
            CopperKind::Pad,
            "VCC",
            "F.Cu",
            scalar("1.5") + unknown_zero(),
        ),
        feature(CopperKind::Pad, "VCC", "B.Cu", Scalar::zero()),
    ]);
    uncertain(&access(
        &board,
        &[point(0.into())],
        &Scalar::one(),
        &Scalar::one(),
    ));
}
#[test]
fn access_unknown_spacing_cannot_become_a_finding() {
    uncertain(&access(
        &board(vec![]),
        &[point(0.into()), point(2.into())],
        &Scalar::one(),
        &(Scalar::one() + unknown_zero()),
    ));
}
#[test]
fn access_unknown_point_distance_cannot_become_a_finding() {
    uncertain(&access(
        &board(vec![]),
        &[point(0.into()), point(Scalar::from(2) + unknown_zero())],
        &Scalar::one(),
        &Scalar::one(),
    ));
}
#[test]
fn access_huge_diameters_are_not_dropped_from_spacing() {
    let mut points = [point(0.into()), point(0.into())];
    for point in &mut points {
        point.diameter = Some(scalar("1e400"));
    }
    ordinary(
        &access(&board(vec![]), &points, &Scalar::one(), &Scalar::one()),
        1,
        "testpoint-accessibility-readiness",
    );
}
#[test]
fn access_spacing_markers_cannot_contain_nan() {
    let huge = scalar("1e400");
    uncertain(&access(
        &board(vec![]),
        &[point(huge.clone()), point(huge + Scalar::one())],
        &Scalar::one(),
        &Scalar::one(),
    ));
}
#[test]
fn access_negative_probe_diameter_is_typed() {
    let mut point = point(0.into());
    point.diameter = Some(Scalar::from(-1));
    uncertain(&access(
        &board(vec![]),
        &[point],
        &Scalar::zero(),
        &Scalar::one(),
    ));
}
#[test]
fn copper_unused_authored_marker_does_not_control_candidates() {
    let mut feature = feature(CopperKind::Pad, "OTHER", "F.Cu", scalar("1e400"));
    feature.region = rectangle(100.0, 1.0);
    assert!(
        clearance(
            &board(vec![feature]),
            point(0.into()),
            &Scalar::one(),
            &Scalar::one()
        )
        .is_empty()
    );
}
#[test]
fn copper_required_marker_failure_is_typed() {
    let feature = feature(CopperKind::Pad, "OTHER", "F.Cu", scalar("1e400"));
    uncertain(&clearance(
        &board(vec![feature]),
        point(0.into()),
        &Scalar::one(),
        &Scalar::one(),
    ));
}
#[test]
fn copper_huge_keepout_radius_is_native() {
    let mut feature = feature(CopperKind::Pad, "OTHER", "F.Cu", 2.into());
    feature.region = rectangle(2.0, 1.0);
    ordinary(
        &clearance(
            &board(vec![feature]),
            point(0.into()),
            &Scalar::one(),
            &scalar("1e400"),
        ),
        1,
        "testpoint-copper-clearance-readiness",
    );
}
#[test]
fn copper_unknown_keepout_cannot_also_emit_a_hit() {
    let mut feature = feature(CopperKind::Pad, "OTHER", "F.Cu", 2.into());
    feature.region = rectangle(2.0, 1.0);
    uncertain(&clearance(
        &board(vec![feature]),
        point(0.into()),
        &Scalar::one(),
        &(Scalar::one() + unknown_zero()),
    ));
}
#[test]
fn copper_negative_probe_diameter_is_typed() {
    let feature = feature(CopperKind::Pad, "OTHER", "F.Cu", 0.into());
    let mut point = point(0.into());
    point.diameter = Some(Scalar::from(-1));
    uncertain(&clearance(
        &board(vec![feature]),
        point,
        &Scalar::one(),
        &Scalar::one(),
    ));
}
#[test]
fn copper_predicate_does_not_need_a_diameter_maximum_order() {
    let mut feature = feature(CopperKind::Pad, "OTHER", "F.Cu", 2.into());
    feature.region = rectangle(2.0, 1.0);
    let mut point = point(0.into());
    point.diameter = Some(Scalar::one() + unknown_zero());
    assert!(
        clearance(
            &board(vec![feature]),
            point,
            &Scalar::one(),
            &scalar("0.75")
        )
        .is_empty()
    );
}
#[test]
fn finite_fixture_checks_distinguish_findings_from_passes() {
    ordinary(
        &thermal_copper_area_readiness(&thermal_board(0.into(), 3.into()), &[], &Scalar::one()),
        1,
        "thermal-copper-area-readiness",
    );
    assert!(
        access(
            &board(vec![]),
            &[point(0.into()), point(3.into())],
            &Scalar::one(),
            &Scalar::one()
        )
        .is_empty()
    );
    ordinary(
        &access(
            &board(vec![]),
            &[point(0.into()), point(1.into())],
            &Scalar::one(),
            &Scalar::one(),
        ),
        1,
        "testpoint-accessibility-readiness",
    );
}

mod certification;
