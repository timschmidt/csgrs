#![cfg(all(feature = "curve", feature = "metaballs"))]
//! Public planar metaball failures and independently derived sampled geometry.
use csgrs::{
    Real, curve,
    errors::{ValidationError, ValidationErrorKind},
};
use hypercurve::{CurveRegion2, Point2};

trait FailureKind {
    fn failure_kind(&self) -> Option<ValidationErrorKind>;
}
impl<T> FailureKind for Result<T, ValidationError> {
    fn failure_kind(&self) -> Option<ValidationErrorKind> {
        self.as_ref().err().map(ValidationError::kind)
    }
}
trait IntoRegion {
    fn into_region(self) -> CurveRegion2;
}
impl IntoRegion for Result<CurveRegion2, ValidationError> {
    fn into_region(self) -> CurveRegion2 {
        self.expect("valid sampled field")
    }
}
fn point(x: i32, y: i32) -> Point2 {
    Point2::new(x.into(), y.into())
}
fn one_ball(radius: Real) -> [(Point2, Real); 1] {
    [(point(0, 0), radius)]
}
fn unknown_zero() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    &sine * &sine + &cosine * &cosine - Real::one()
}

#[test]
fn zero_radius_is_invalid_instead_of_empty() {
    assert_eq!(
        curve::metaballs(&one_ball(0.into()), (3, 3), 1.into(), 0.into()).failure_kind(),
        Some(ValidationErrorKind::InvalidInput)
    );
}
#[test]
fn unresolved_radius_preserves_uncertainty() {
    assert_eq!(
        curve::metaballs(&one_ball(unknown_zero()), (3, 3), 1.into(), 0.into()).failure_kind(),
        Some(ValidationErrorKind::Undecided)
    );
}
#[test]
fn negative_padding_is_invalid() {
    assert_eq!(
        curve::metaballs(&one_ball(1.into()), (3, 3), 1.into(), (-1).into()).failure_kind(),
        Some(ValidationErrorKind::InvalidInput)
    );
}
#[test]
fn empty_input_cannot_hide_invalid_iso() {
    assert_eq!(
        curve::metaballs(&[], (3, 3), 0.into(), 0.into()).failure_kind(),
        Some(ValidationErrorKind::InvalidInput)
    );
}
#[test]
fn empty_input_cannot_hide_invalid_padding() {
    assert_eq!(
        curve::metaballs(&[], (3, 3), 1.into(), (-1).into()).failure_kind(),
        Some(ValidationErrorKind::InvalidInput)
    );
}
#[test]
fn too_small_grid_is_not_empty_geometry() {
    assert_eq!(
        curve::metaballs(&one_ball(1.into()), (1, 3), 1.into(), 0.into()).failure_kind(),
        Some(ValidationErrorKind::InvalidInput)
    );
}
#[test]
fn grid_overflow_is_resource_exhaustion() {
    assert_eq!(
        curve::metaballs(&one_ball(1.into()), (usize::MAX, 2), 1.into(), 0.into())
            .failure_kind(),
        Some(ValidationErrorKind::ResourceExhausted)
    );
}
#[test]
fn normalized_unit_source_has_exact_diamond_area() {
    // F=(iso-I)/(iso+I), a=101/100: each axis crossing is
    // (a^2+1)/(2a)=20201/20200. The four crossings bound a diamond of area 2b^2.
    let region =
        curve::metaballs(&one_ball(1.into()), (3, 3), 1.into(), 0.into()).into_region();
    let b = (Real::from(20201) / Real::from(20200)).unwrap();
    assert_eq!(region.filled_area().unwrap(), Some(Real::from(2) * &b * &b));
    assert_eq!(
        curve::contains_xy(&region, 0.into(), 0.into()).unwrap(),
        Some(true)
    );
    assert_eq!(
        curve::contains_xy(&region, 2.into(), 0.into()).unwrap(),
        Some(false)
    );
}
#[test]
fn a_ring_of_sources_preserves_its_hole() {
    let balls = [point(-2, 0), point(2, 0), point(0, -2), point(0, 2)]
        .map(|center| (center, Real::one()));
    // Center I=1 is outside iso=11/10; the four bridges at (+/-1,+/-1)
    // have I=6/5 and are inside. Keep every source and the declared 25^2 grid.
    let region = curve::metaballs(
        &balls,
        (25, 25),
        (Real::from(11) / Real::from(10)).unwrap(),
        0.into(),
    )
    .into_region();
    assert!(!region.is_empty());
    assert_eq!(
        curve::contains_xy(&region, 0.into(), 0.into()).unwrap(),
        Some(false)
    );
    for (x, y) in [(2, 0), (-2, 0), (0, 2), (0, -2), (1, 1)] {
        assert_eq!(
            curve::contains_xy(&region, x.into(), y.into()).unwrap(),
            Some(true)
        );
    }
}

#[test]
fn failed_construction_does_not_contaminate_later_policies_or_composition() {
    use csgrs::{GeometryCertainty, GeometryContext};
    use hypercurve::BooleanOp;
    use std::error::Error;
    for context in [
        &GeometryContext::STRICT,
        &GeometryContext::APPROXIMATE_512,
        &GeometryContext::STRICT,
    ] {
        let error = curve::metaballs_with_context(
            &one_ball(unknown_zero()),
            (3, 3),
            1.into(),
            0.into(),
            context,
        )
        .unwrap_err();
        assert_eq!(error.kind(), ValidationErrorKind::Undecided);
        assert!(matches!(
            error
                .source()
                .unwrap()
                .downcast_ref::<hypersdf::SdfEvaluationError>(),
            Some(hypersdf::SdfEvaluationError::Undecided { .. })
        ));
        let result = curve::metaballs_with_context(
            &one_ball(1.into()),
            (3, 3),
            1.into(),
            0.into(),
            context,
        )
        .unwrap();
        assert_eq!(result.certainty, GeometryCertainty::Certified);
        let empty = curve::metaballs(&[], (3, 3), 1.into(), 0.into()).unwrap();
        let accumulated = empty.boolean_region(&result.value, BooleanOp::Union).unwrap();
        assert_eq!(
            accumulated.filled_area().unwrap(),
            result.value.filled_area().unwrap()
        );
        let removed = accumulated
            .boolean_region(&result.value, BooleanOp::Difference)
            .unwrap();
        assert!(removed.is_empty());
        assert!(
            !removed
                .boolean_region(&result.value, BooleanOp::Union)
                .unwrap()
                .is_empty()
        );
        let mesh =
            curve::try_extrude(&result.value, 2.into(), &GeometryContext::STRICT).unwrap();
        assert_eq!(mesh.certainty, GeometryCertainty::Certified);
        assert!(
            mesh.value
                .is_closed_manifold_geometry(&GeometryContext::STRICT.mesh_context())
                .unwrap()
                .value
        );
        let six_volume = mesh.value.triangles.iter().fold(Real::zero(), |sum, t| {
            let [a, b, c] = t.indices().map(|i| mesh.value.positions[i].to_vector());
            sum + a.dot(&b.cross(&c))
        });
        assert_eq!(
            six_volume,
            Real::from(12) * result.value.filled_area().unwrap().unwrap()
        );
    }
}

#[test]
fn the_original_twelve_by_twelve_closed_fixture_remains_supported() {
    let region = curve::metaballs(&one_ball(1.into()), (12, 12), 1.into(), 0.into()).unwrap();
    assert!(!region.is_empty());
    assert_eq!(
        curve::contains_xy(&region, 0.into(), 0.into()).unwrap(),
        Some(true)
    );
}
