use crate::*;
use crate::geometry::{polygons_to_profile, rect_polygon};
use crate::scalar::scalar;

fn square() -> PcbRegion {
    polygons_to_profile(vec![rect_polygon([0.5, 0.5], [1.0, 1.0], 0.0)], None)
}

fn deny_refinement() {
    let polynomial = hypercurve::BezierParameterPolynomial::try_new_power_basis(
        [-36, 1368, -11034, 31728, -38280, 16620].into_iter().map(Scalar::from).collect(),
    ).unwrap();
    assert!(polynomial.isolate_unit_interval_roots().is_err());
}

#[test]
fn failed_source_construction_cannot_publish_components() {
    let failed = square().with_exact_construction_error("retained source fixture");
    assert!(failed.exact_component_regions().is_none());
}

#[test]
fn denied_component_extraction_is_retryable() {
    let region = square();
    let mut during = None;
    let denied = hypercurve::evaluate_with_budget(PREDICATE_POLICY, hypercurve::WorkBudget::ZERO, || {
        deny_refinement();
        during = Some(region.exact_component_regions().map(<[_]>::len));
    });
    assert!(denied.result.is_err());
    assert_eq!(during, Some(None));
    assert_eq!(region.exact_component_regions().map(<[_]>::len), Some(1));
}

#[test]
fn warm_component_cache_cannot_bypass_an_exhausted_scope() {
    let region = square();
    assert_eq!(region.exact_component_regions().map(<[_]>::len), Some(1));
    let mut during = None;
    let denied = hypercurve::evaluate_with_budget(PREDICATE_POLICY, hypercurve::WorkBudget::ZERO, || {
        deny_refinement();
        during = Some(region.exact_component_regions().map(<[_]>::len));
    });
    assert!(denied.result.is_err());
    assert_eq!(during, Some(None));
}

#[test]
fn missing_component_identity_cannot_promote_report_chords() {
    let native = geometry::circle_region(&[Scalar::zero(), Scalar::zero()], &Scalar::one()).unwrap();
    let projected = exact_backed_finite_polygons(&native, None);
    assert_eq!(projected.0.len(), 1);
    assert_eq!(crate::scalar::compare(projected.0[0].exact_area().unwrap(), &Scalar::pi()), Some(std::cmp::Ordering::Equal));
}

#[test]
fn ordinary_finite_projection_failure_does_not_unwind() {
    let region = PcbRegion::new(geometry::circle_region(&[scalar("1e400"), Scalar::zero()], &Scalar::one()).unwrap(), None);
    let attempted = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| region.to_multipolygon()));
    assert!(attempted.is_ok(), "unrepresentable report coordinates require a typed failure");
}
