use crate::{PcbRegion, PcbRegionExt, Scalar, PREDICATE_POLICY};
use crate::geometry::{Polygon, LineString, Coord, polygons_to_profile, rect_polygon};

fn square() -> PcbRegion {
    polygons_to_profile(vec![rect_polygon([0.0, 0.0], [2.0, 1.0], 0.0)], None)
}
fn failed_empty() -> PcbRegion {
    crate::geometry::empty_profile(None).with_exact_construction_error("retained empty placeholder failure")
}
fn deny_refinement() {
    let polynomial = hypercurve::BezierParameterPolynomial::try_new_power_basis(
        [-36, 1368, -11034, 31728, -38280, 16620].into_iter().map(Scalar::from).collect(),
    ).unwrap();
    assert!(polynomial.isolate_unit_interval_roots().is_err());
}
fn quadratic() -> Polygon<f64> {
    use hypercurve::{Curve2, CurvePath2, CurveRegion2, CurveRegionLoopRole, FillRule, LineSeg2, Point2, QuadraticBezier2};
    let path = CurvePath2::try_new(vec![
        Curve2::from(QuadraticBezier2::new(Point2::from_values(0, 0), Point2::from_values(1, 2), Point2::from_values(2, 0))),
        Curve2::from(LineSeg2::try_new(Point2::from_values(2, 0), Point2::from_values(0, 0)).unwrap()),
    ]).unwrap();
    let native = CurveRegion2::try_from_boundary_paths_with_loop_semantics(&[path], &[CurveRegionLoopRole::Material], &[FillRule::NonZero]).unwrap();
    Polygon::from_exact_region(native, LineString(vec![Coord{x:0.0,y:0.0},Coord{x:1.0,y:1.0},Coord{x:2.0,y:0.0},Coord{x:0.0,y:0.0}]), vec![])
}

#[test]
fn failed_native_area_cannot_publish_a_placeholder_value() {
    assert!(failed_empty().filled_area().is_err());
}
#[test]
fn failed_native_bounds_cannot_publish_placeholder_bounds() {
    assert!(failed_empty().bounds().is_err());
}
#[test]
fn failed_region_cannot_claim_to_be_empty() {
    assert!(!failed_empty().is_empty(), "construction failure is not a certified empty set");
}
#[test]
fn failed_polygon_cannot_expose_unchecked_native_geometry() {
    let polygon = rect_polygon([0.0, 0.0], [1.0, 1.0], 0.0).with_exact_construction_error("retained polygon failure");
    assert!(polygon.exact_region().filled_area().is_err());
}
#[test]
fn single_quadratic_profile_keeps_its_native_geometry() {
    let profile = polygons_to_profile(vec![quadratic()], None);
    assert!(profile.try_to_multipolygon().is_ok());
    assert_eq!(profile.filled_area().unwrap().unwrap(), (Scalar::from(4) / Scalar::from(3)).unwrap());
}
#[test]
fn mixed_native_profile_aggregation_retains_supported_curve_families() {
    let profile = polygons_to_profile(vec![quadratic(), rect_polygon([5.0,0.0], [1.0,1.0], 0.0)], None);
    assert!(profile.try_to_multipolygon().is_ok());
    assert_eq!(profile.filled_area().unwrap().unwrap(), (Scalar::from(7) / Scalar::from(3)).unwrap());
}
#[test]
fn exhausted_profile_construction_cannot_escape_as_success() {
    let polygon = rect_polygon([0.0,0.0], [1.0,1.0], 0.0);
    let mut constructed = None;
    let denied = hypercurve::evaluate_with_budget(PREDICATE_POLICY, hypercurve::WorkBudget::ZERO, || {
        deny_refinement();
        constructed = Some(polygons_to_profile(vec![polygon.clone()], None));
    });
    assert!(denied.result.is_err());
    assert!(constructed.unwrap().try_to_multipolygon().is_err());
    assert!(polygons_to_profile(vec![polygon], None).try_to_multipolygon().is_ok());
}
#[test]
fn source_failure_is_not_an_ordinary_net_length_measurement() {
    use crate::kicad::{BoardModel, CopperFeature, CopperKind};
    use crate::constraint_policy::NetClassConfig;
    let board = BoardModel { source: "length source".into(), copper: vec![CopperFeature {
        layer: "F.Cu".into(), net: Some("SIGNAL".into()), kind: CopperKind::Segment,
        region: square().with_exact_construction_error("retained length failure"), location: [Scalar::zero(), Scalar::zero()],
    }], drills: vec![], board_outline: None, panel_features: None };
    let class = NetClassConfig { name: "length class".into(), nets: vec!["SIGNAL".into()], max_length: Some(Scalar::one()), ..NetClassConfig::default() };
    let findings = crate::checks::net_constraint_readiness(&[class], None, &[board], &[]);
    assert!(findings.iter().any(|finding| finding.check == "geometry-uncertainty"), "{findings:?}");
}
