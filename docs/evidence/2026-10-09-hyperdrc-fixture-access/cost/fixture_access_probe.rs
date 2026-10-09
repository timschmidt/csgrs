use super::*;
use hyperdrc::geometry::{polygons_to_profile, rect_polygon};
use hyperdrc::ipc356::{Ipc356AccessSide, Ipc356FeatureType, Ipc356Point, Ipc356Soldermask};
use hyperdrc::kicad::{BoardModel, CopperFeature, CopperKind};
use hyperdrc::checks::*;
use hyperdrc::Violation;
fn scalar(text: &str) -> Scalar { text.parse().unwrap() }

fn rectangle(x: f64, width: f64) -> hyperdrc::PcbRegion {
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
fn measure(label: &str, iterations: usize, count: usize, check: &str, query: impl FnMut() -> Vec<Violation>) {
    measure_value(label, iterations, query, |findings| findings_witness(findings, count, check));
}

pub(super) fn run() {
    let one = Scalar::one();
    let zero = Scalar::zero();
    let huge = scalar("1e400");
    let b = thermal_board(huge.clone(), &huge + &one);
    measure("fixture_thermal_huge_centers", 100, 0, "unused", || thermal_copper_area_readiness(&b, &[], &one));
    let b = thermal_board(0.into(), 1.into());
    measure("fixture_thermal_huge_radius", 100, 0, "unused", || thermal_copper_area_readiness(&b, &[], &huge));
    let threshold = &one + unknown_zero();
    measure("fixture_thermal_unknown_radius", 25, 1, "geometry-uncertainty", || thermal_copper_area_readiness(&b, &[], &threshold));
    let mut b = thermal_board(0.into(), &one + unknown_zero());
    b.copper.push(feature(CopperKind::Zone, "VCC", "F.Cu", scalar("0.5")));
    measure("fixture_thermal_alternative_after_unknown", 25, 0, "unused", || thermal_copper_area_readiness(&b, &[], &one));
    let mut b = thermal_board(0.into(), 1.into());
    b.copper.extend((0..2_000).map(|i| feature(CopperKind::Zone, "VCC", "F.Cu", Scalar::from(100 + i * 5))));
    assert_eq!(b.copper.len(), 2_002);
    measure("fixture_thermal_sparse_2002", 10, 0, "unused", || thermal_copper_area_readiness(&b, &[], &one));
    let b = thermal_board(0.into(), 0.into());
    let negative = Scalar::from(-1);
    measure("fixture_thermal_negative_radius", 100, 1, "thermal-copper-area-readiness", || thermal_copper_area_readiness(&b, &[], &negative));

    let b = board(vec![feature(CopperKind::Pad, "VCC", "F.Cu", huge.clone())]);
    let points = [point(0.into())];
    measure("fixture_coverage_huge_covered", 100, 0, "unused", || testpoint_coverage_readiness(&b, &points, &[]));
    measure("fixture_coverage_required_marker", 100, 1, "geometry-uncertainty", || testpoint_coverage_readiness(&b, &[], &[]));
    let mut b = board((0..3).map(|i| feature(CopperKind::Pad, "VCC", "F.Cu", Scalar::from(i))).collect());
    b.copper.push(feature(CopperKind::Pad, "VCC", "F.Cu", huge.clone()));
    measure("fixture_coverage_only_three_markers", 100, 1, "testpoint-coverage-readiness", || testpoint_coverage_readiness(&b, &[], &[]));
    let b = board((0..2_000).map(|i| feature(CopperKind::Pad, "VCC", "F.Cu", Scalar::from(i * 5))).collect());
    let points = [point(0.into()), point(10000.into())];
    assert_eq!(b.copper.len() + points.len(), 2_002);
    measure("fixture_coverage_sparse_2002", 10, 0, "unused", || testpoint_coverage_readiness(&b, &points, &[]));

    let b = board(vec![feature(CopperKind::Pad, "VCC", "F.Cu", huge.clone())]);
    let points = [point(huge.clone())];
    measure("fixture_access_huge_centers", 100, 0, "unused", || access(&b, &points, &one, &one));
    let b = board(vec![]); let points = [point(0.into())];
    measure("fixture_access_unknown_minimum", 25, 1, "geometry-uncertainty", || access(&b, &points, &threshold, &one));
    let b = board(vec![feature(CopperKind::Pad, "VCC", "B.Cu", scalar("1.5"))]);
    measure("fixture_access_unknown_side", 25, 1, "geometry-uncertainty", || access(&b, &points, &one, &threshold));
    let b = board(vec![]); let points = [point(0.into()), point(2.into())];
    measure("fixture_access_unknown_spacing", 25, 1, "geometry-uncertainty", || access(&b, &points, &one, &threshold));
    let mut points = [point(0.into()), point(0.into())];
    for p in &mut points { p.diameter = Some(huge.clone()); }
    measure("fixture_access_huge_diameters", 50, 1, "testpoint-accessibility-readiness", || access(&b, &points, &one, &one));
    let b = board(vec![feature(CopperKind::Pad, "VCC", "B.Cu", scalar("0.75"))]);
    let points = [point(0.into())]; let spacing = scalar("0.25") + unknown_zero();
    measure("fixture_access_spacing_floor", 25, 1, "testpoint-accessibility-readiness", || access(&b, &points, &one, &spacing));
    let b = board(vec![]);
    let points = [point(0.into()), point(Scalar::from(2) - scalar("1e-50"))];
    measure("fixture_access_sub_ulp_spacing", 50, 1, "testpoint-accessibility-readiness", || access(&b, &points, &one, &one));
    let mut points = (0..2_000).map(|i| point(Scalar::from(i * 5))).collect::<Vec<_>>();
    points.extend([point((-10).into()), point(scalar("-8.5"))]);
    assert_eq!(points.len(), 2_002);
    measure("fixture_access_spacing_sparse_2002", 5, 1, "testpoint-accessibility-readiness", || access(&b, &points, &one, &one));
    let mut b = board((0..2_000).map(|i| feature(CopperKind::Pad, "VCC", "F.Cu", Scalar::from(100 + i * 5))).collect());
    b.copper.push(feature(CopperKind::Pad, "VCC", "B.Cu", zero.clone()));
    let points = [point(0.into())];
    assert_eq!(b.copper.len() + points.len(), 2_002);
    measure("fixture_access_parity_sparse_2002", 10, 1, "testpoint-accessibility-readiness", || access(&b, &points, &one, &one));

    let mut f = feature(CopperKind::Pad, "OTHER", "F.Cu", huge.clone());
    f.region = rectangle(100.0, 1.0); let b = board(vec![f]);
    let points = [point(0.into())];
    let query = |b: &BoardModel, points: &[Ipc356Point], min: &Scalar, gap: &Scalar| testpoint_copper_clearance_readiness(b, points, &[], min, gap, &zero);
    measure("fixture_copper_unused_huge_marker", 100, 0, "unused", || query(&b, &points, &one, &one));
    let mut f = feature(CopperKind::Pad, "OTHER", "F.Cu", Scalar::from(2)); f.region = rectangle(2.0, 1.0); let b = board(vec![f]);
    measure("fixture_copper_huge_radius", 50, 1, "testpoint-copper-clearance-readiness", || query(&b, &points, &one, &huge));
    measure("fixture_copper_unknown_clearance", 25, 1, "geometry-uncertainty", || query(&b, &points, &one, &threshold));
    let mut p = point(0.into()); p.diameter = Some(&one + unknown_zero()); let points = [p]; let gap = scalar("0.75");
    measure("fixture_copper_maximum_order_unneeded", 25, 0, "unused", || query(&b, &points, &one, &gap));
    let mut f = feature(CopperKind::Pad, "OTHER", "F.Cu", Scalar::one()); f.region = rectangle(1.0, 1.0); let b = board(vec![f]);
    measure("fixture_copper_minimum_radius_witness", 25, 1, "testpoint-copper-clearance-readiness", || query(&b, &points, &one, &zero));
    let mut f = feature(CopperKind::Pad, "OTHER", "F.Cu", zero.clone()); f.region = polygons_to_profile(Vec::new(), None); let b = board(vec![f]);
    let uncertain = unknown_zero();
    measure("fixture_copper_empty_unknown", 25, 0, "unused", || query(&b, &points, &uncertain, &uncertain));
    let failed = rectangle(1e20, 16_384.0); assert!(failed.try_bounding_box().is_err());
    let mut f = feature(CopperKind::Pad, "OTHER", "F.Cu", zero.clone()); f.region = failed; let b = board(vec![f]); let points = [point(0.into())];
    measure("fixture_copper_failed_shape", 100, 1, "geometry-uncertainty", || query(&b, &points, &one, &one));
    let mut b = board((0..2_000).map(|i| {
        let x = 100 + i * 5; let mut f = feature(CopperKind::Pad, "OTHER", "F.Cu", Scalar::from(x)); f.region = rectangle(f64::from(x), 1.0); f
    }).collect());
    b.copper.push(feature(CopperKind::Pad, "OTHER", "F.Cu", zero.clone()));
    assert_eq!(b.copper.len() + points.len(), 2_002);
    measure("fixture_copper_sparse_2002", 5, 1, "testpoint-copper-clearance-readiness", || query(&b, &points, &one, &one));
}
