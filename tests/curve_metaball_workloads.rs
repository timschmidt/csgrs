#![cfg(all(feature = "curve", feature = "metaballs"))]
use csgrs::{GeometryCertainty, GeometryContext, Real, curve};
use hypercurve::{CurveRegionLoopRole, Point2};

fn validate(balls: &[(Point2, Real)], resolution: usize, padding: Real) {
    let result = curve::metaballs_with_context(
        balls,
        (resolution, resolution),
        1.into(),
        padding,
        &GeometryContext::STRICT,
    )
    .unwrap();
    assert_eq!(result.certainty, GeometryCertainty::Certified);
    assert!(!result.value.is_empty());
    assert_eq!(
        result.value.loop_roles().unwrap(),
        vec![CurveRegionLoopRole::Material]
    );
    let area = result.value.filled_area().unwrap().unwrap();
    assert_eq!(
        hyperlimit::classify_real_sign(&area, hyperlimit::PredicatePolicy::STRICT).value(),
        Some(hyperlimit::Sign::Positive)
    );
    for (center, _) in balls {
        assert_eq!(
            curve::contains_xy(&result.value, center.x().clone(), center.y().clone()),
            Some(true)
        );
    }
}

#[test]
fn readme_planar_metaball_completes_at_original_resolution_twenty_four() {
    let balls = [(Point2::new(Real::zero(), Real::zero()), Real::one())];
    validate(&balls, 24, Real::zero());
}

#[test]
fn benchmark_planar_metaballs_complete_at_original_resolution_forty_eight() {
    let balls = [
        (Point2::new(Real::zero(), Real::zero()), Real::from(2)),
        (Point2::new(Real::from(2), Real::zero()), Real::from(2)),
    ];
    validate(&balls, 48, Real::one());
}
