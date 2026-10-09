use crate::geometry::{Coord, LineString, MultiPolygon, Polygon, polygons_to_profile, rect_polygon};
use crate::{PcbRegion, Scalar, PREDICATE_POLICY};

fn unavailable_area() -> Polygon<f64> {
    use hypercurve::{Curve2, CurvePath2, CurveRegion2, CurveRegionLoopRole, FillRule, LineSeg2, Point2, RationalBezier2};
    let curve = RationalBezier2::try_new(
        [(0, 0), (1, 1), (2, 2), (3, 2), (4, 1), (5, 0)].into_iter().map(|(x, y)| Point2::from_values(x, y)).collect(),
        [1, 2, 3, 4, 5, 7].into_iter().map(Scalar::from).collect(),
    ).unwrap();
    let path = CurvePath2::try_new(vec![Curve2::from(curve), Curve2::from(LineSeg2::try_new(Point2::from_values(5, 0), Point2::from_values(0, 0)).unwrap())]).unwrap();
    let native = CurveRegion2::try_from_boundary_paths_with_loop_semantics(&[path], &[CurveRegionLoopRole::Material], &[FillRule::NonZero]).unwrap();
    assert_eq!(native.filled_area().unwrap(), None);
    Polygon::from_exact_region(native, LineString(vec![Coord{x:0.0,y:0.0},Coord{x:5.0,y:0.0},Coord{x:325.0/113.0,y:175.0/113.0},Coord{x:0.0,y:0.0}]), vec![])
}
fn unavailable_profile() -> PcbRegion { polygons_to_profile(vec![unavailable_area()], None) }
fn uncertainty(findings: &[crate::Violation]) {
    assert!(findings.iter().any(|f| f.check == "geometry-uncertainty"), "{findings:?}");
}
fn deny_refinement() {
    let polynomial = hypercurve::BezierParameterPolynomial::try_new_power_basis([-36,1368,-11034,31728,-38280,16620].into_iter().map(Scalar::from).collect()).unwrap();
    assert!(polynomial.isolate_unit_interval_roots().is_err());
}
#[test]
fn tiny_feature_cannot_skip_unavailable_area() {
    uncertainty(&crate::checks::tiny_layer_feature_readiness("F.Cu", &unavailable_profile(), &Scalar::one()));
}
#[test]
fn layer_sanity_cannot_describe_unavailable_area_as_empty() {
    let findings = crate::checks::layer_sanity("F.Cu", &unavailable_profile(), None);
    uncertainty(&findings);
    assert!(!findings.iter().any(|f| f.message.as_deref().is_some_and(|m| m.contains("parsed to empty"))));
}
#[test]
fn copper_balance_cannot_drop_a_layer_with_unavailable_area() {
    uncertainty(&crate::checks::copper_balance(&[("F.Cu".into(),unavailable_profile()),("B.Cu".into(),polygons_to_profile(vec![rect_polygon([0.0,0.0],[1.0,1.0],0.0)],None))], &Scalar::from(2), &Scalar::zero()));
}
#[test]
fn duplicate_layer_check_cannot_drop_unavailable_area() {
    let profile = unavailable_profile();
    uncertainty(&crate::checks::duplicate_layer_geometry_readiness(&[("F.Cu".into(),profile.clone()),("B.Cu".into(),profile)], &Scalar::zero()));
}
#[test]
fn paste_ratio_cannot_skip_a_required_copper_area() {
    uncertainty(&crate::checks::paste_aperture_ratio("F.Paste", &crate::geometry::empty_profile(None), "F.Cu", &unavailable_profile(), &Scalar::one(), &Scalar::from(2), &Scalar::zero()));
}
#[test]
fn report_filter_cannot_erase_an_unavailable_area() {
    let shapes = crate::geometry::multipolygon_to_shapes_scalar(&MultiPolygon(vec![unavailable_area()]), &Scalar::zero());
    assert!(!shapes.is_empty(), "unavailable area must be a typed failure, not empty success");
}
#[test]
fn report_filter_cannot_treat_an_unknown_threshold_as_below_gate() {
    let sine = Scalar::e().sin(); let cosine = Scalar::e().cos();
    let unknown_zero = &sine * &sine + &cosine * &cosine - Scalar::one();
    let shapes = crate::geometry::multipolygon_to_shapes_scalar(&MultiPolygon(vec![rect_polygon([0.0,0.0],[1.0,1.0],0.0)]), &(Scalar::one()+unknown_zero));
    assert!(!shapes.is_empty(), "unknown comparison must be a typed failure, not empty success");
}
#[test]
fn exhausted_empty_area_collection_cannot_publish_zero() {
    let result = hypercurve::evaluate_with_budget(PREDICATE_POLICY, hypercurve::WorkBudget::ZERO, || {
        deny_refinement();
        assert!(crate::geometry::multipolygon_area_scalar(&MultiPolygon(vec![])).is_none());
    });
    assert!(result.result.is_err());
}
#[test]
fn copper_balance_uses_filled_sets_not_authored_duplicate_counts() {
    let polygon=rect_polygon([0.0,0.0],[1.0,1.0],0.0);
    let layers = [("F.Cu".into(),polygons_to_profile(vec![polygon.clone(),polygon.clone()],None)),("B.Cu".into(),polygons_to_profile(vec![polygon],None))];
    let findings=crate::checks::copper_balance(&layers,&Scalar::one(),&Scalar::zero());
    assert!(findings.is_empty(), "{findings:?}");
}
#[test]
fn retained_polygon_failure_already_blocks_measurement() {
    let polygon=rect_polygon([0.0,0.0],[1.0,1.0],0.0).with_exact_construction_error("retained failure");
    assert!(crate::geometry::polygon_area_scalar(&polygon).is_none());
}
