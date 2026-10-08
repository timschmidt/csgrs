use crate::{PcbRegion, Scalar, PREDICATE_POLICY};
use crate::geometry::{circle_polygon, polygons_to_profile, rect_polygon};

fn square() -> PcbRegion {
    polygons_to_profile(vec![rect_polygon([2.0, 3.0], [1.0, 1.0], 0.0)], None)
}
fn deny_refinement() {
    let polynomial = hypercurve::BezierParameterPolynomial::try_new_power_basis(
        [-36, 1368, -11034, 31728, -38280, 16620].into_iter().map(Scalar::from).collect(),
    ).unwrap();
    assert!(polynomial.isolate_unit_interval_roots().is_err());
}
macro_rules! denied_boolean {
    ($name:ident, $method:ident) => {
        #[test]
        fn $name() {
            let left = square();
            let right = square();
            let result = hypercurve::evaluate_with_budget(PREDICATE_POLICY, hypercurve::WorkBudget::ZERO, || {
                deny_refinement();
                assert!(left.$method(&right).is_err());
            });
            assert!(result.result.is_err());
            assert!(left.$method(&right).is_ok());
        }
    };
}
denied_boolean!(denied_union_cannot_publish_geometry, try_union);
denied_boolean!(denied_intersection_cannot_publish_geometry, try_intersection);
denied_boolean!(denied_difference_cannot_publish_geometry, try_difference);
denied_boolean!(denied_xor_cannot_publish_geometry, try_xor);

#[test]
fn denied_offset_cannot_publish_geometry() {
    let region = square();
    let result = hypercurve::evaluate_with_budget(PREDICATE_POLICY, hypercurve::WorkBudget::ZERO, || {
        deny_refinement();
        assert!(region.offset(Scalar::zero()).is_err());
    });
    assert!(result.result.is_err());
    assert!(region.offset(Scalar::zero()).is_ok());
}
#[test]
fn failed_source_cannot_classify_point() {
    let failed = square().with_exact_construction_error("retained membership source failure");
    assert_eq!(failed.contains_xy(Scalar::from(2), Scalar::from(3)), None);
}
#[test]
fn unresolved_circle_construction_does_not_unwind() {
    let sine = Scalar::e().sin();
    let cosine = Scalar::e().cos();
    let radius = &sine * &sine + &cosine * &cosine - Scalar::one();
    let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        crate::translated_circle(radius, 64, Scalar::zero(), Scalar::zero())
    }));
    assert!(outcome.is_ok(), "unresolved circle construction must not unwind");
}
#[test]
fn collapsed_offset_cannot_retain_inferred_nonempty_bounds() {
    let region = polygons_to_profile(vec![circle_polygon([2.0, 3.0], 1.0, 17)], None);
    let collapsed = region.offset(Scalar::from(-2)).unwrap();
    assert!(collapsed.is_empty());
    assert!(collapsed.try_native_bounds().unwrap().is_none());
}
#[test]
fn required_width_cannot_publish_a_placeholder_measurement() {
    use crate::kicad::{BoardModel, CopperFeature, CopperKind};
    let mut board = BoardModel { source: "failed-width-fixture".into(), copper: vec![], drills: vec![], board_outline: None, panel_features: None };
    board.copper.push(CopperFeature {
        layer: "F.Cu".into(), net: Some("VCC".into()), kind: CopperKind::Segment,
        region: square().with_exact_construction_error("retained width source failure"),
        location: [2.into(), 3.into()],
    });
    let findings = crate::checks::copper_width_readiness(&board, &[], &Scalar::from(2));
    assert!(findings.iter().any(|finding| finding.check == "geometry-uncertainty"), "{findings:?}");
}
