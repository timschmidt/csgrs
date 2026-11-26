#![cfg(all(feature = "curve", feature = "metaballs"))]
//! Public planar metaball failures and independently derived sampled geometry.
//! Exact expectations concern normalized-field marching-squares chords, not the continuous level set.
#[path = "support/uncertainty.rs"]
/// Shared proved-zero control and independently unresolved identity for radius validation.
mod uncertainty;

use csgrs::{
    Real, curve,
    errors::{ValidationError, ValidationErrorKind},
};
use hypercurve::{CurveRegion2, Point2};

/// Extracts a modeling failure category without treating successful geometry as an error.
trait FailureKind {
    /// Returns the existing error category, or `None` on success.
    fn failure_kind(&self) -> Option<ValidationErrorKind>;
}
impl<T> FailureKind for Result<T, ValidationError> {
    /// Borrows the result and maps only its error through the public category classifier.
    fn failure_kind(&self) -> Option<ValidationErrorKind> {
        self.as_ref().err().map(ValidationError::kind)
    }
}
/// Success adapter used by the fixed sampled-region witnesses.
trait IntoRegion {
    /// Obtains the completed region or panics on a construction failure.
    fn into_region(self) -> CurveRegion2;
}
impl IntoRegion for Result<CurveRegion2, ValidationError> {
    /// Requires successful sampling without changing or further validating its region.
    fn into_region(self) -> CurveRegion2 {
        self.expect("valid sampled field")
    }
}
/// Constructs an exact integer planar point.
fn point(x: i32, y: i32) -> Point2 {
    Point2::new(x.into(), y.into())
}
/// Supplies one origin-centered source with an intentionally unvalidated radius.
fn one_ball(radius: Real) -> [(Point2, Real); 1] {
    [(point(0, 0), radius)]
}
/// Checks the Pythagorean control and returns the shared strictly unresolved sine-addition zero.
fn unknown_zero() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    uncertainty::terminal_zero(&sine * &sine + &cosine * &cosine - Real::one())
}

#[test]
/// Requires zero radius to be invalid rather than successful empty geometry.
fn zero_radius_is_invalid_instead_of_empty() {
    assert_eq!(
        curve::metaballs(&one_ball(0.into()), (3, 3), 1, 0).failure_kind(),
        Some(ValidationErrorKind::InvalidInput)
    );
}
#[test]
/// Requires an unresolved exact-zero radius to retain the undecided category.
fn unresolved_radius_preserves_uncertainty() {
    assert_eq!(
        curve::metaballs(&one_ball(unknown_zero()), (3, 3), 1, 0).failure_kind(),
        Some(ValidationErrorKind::Undecided)
    );
}
#[test]
/// Requires negative padding on a nonempty source list to remain invalid.
fn negative_padding_is_invalid() {
    assert_eq!(
        curve::metaballs(&one_ball(1.into()), (3, 3), 1, -1).failure_kind(),
        Some(ValidationErrorKind::InvalidInput)
    );
}
#[test]
/// Requires positive-isovalue validation even when no sources could generate material.
fn empty_input_cannot_hide_invalid_iso() {
    assert_eq!(
        curve::metaballs(&[], (3, 3), 0, 0).failure_kind(),
        Some(ValidationErrorKind::InvalidInput)
    );
}
#[test]
/// Requires padding validation before successful empty-source handling.
fn empty_input_cannot_hide_invalid_padding() {
    assert_eq!(
        curve::metaballs(&[], (3, 3), 1, -1).failure_kind(),
        Some(ValidationErrorKind::InvalidInput)
    );
}
#[test]
/// Requires a one-sample X dimension to be invalid instead of clamped or treated as empty.
fn too_small_grid_is_not_empty_geometry() {
    assert_eq!(
        curve::metaballs(&one_ball(1.into()), (1, 3), 1, 0).failure_kind(),
        Some(ValidationErrorKind::InvalidInput)
    );
}
#[test]
/// Checks an unaddressable grid maps to resource exhaustion; this does not exercise allocator exhaustion.
fn grid_overflow_is_resource_exhaustion() {
    assert_eq!(
        curve::metaballs(&one_ball(1.into()), (usize::MAX, 2), 1, 0).failure_kind(),
        Some(ValidationErrorKind::ResourceExhausted)
    );
}
#[test]
/// Checks the 3-by-3 normalized-field diamond's independent exact area and two membership probes.
/// The crossing coordinate `20201/20200` comes from linear sample interpolation, not an analytic radius.
fn normalized_unit_source_has_exact_diamond_area() {
    // F=(iso-I)/(iso+I), a=101/100: each axis crossing is
    // (a^2+1)/(2a)=20201/20200. The four crossings bound a diamond of area 2b^2.
    let region = curve::metaballs(&one_ball(1.into()), (3, 3), 1, 0).into_region();
    let b = (Real::from(20201) / Real::from(20200)).unwrap();
    assert_eq!(region.filled_area().unwrap(), Some(Real::from(2) * &b * &b));
    assert_eq!(curve::contains_xy(&region, 0, 0).unwrap(), Some(true));
    assert_eq!(curve::contains_xy(&region, 2, 0).unwrap(), Some(false));
}
#[test]
/// Preserves four ring sources and the 25-by-25 grid, checking an exterior center and five interior probes.
/// These selected memberships do not independently count boundary loops or prove full hole topology.
fn a_ring_of_sources_preserves_its_hole() {
    let balls = [point(-2, 0), point(2, 0), point(0, -2), point(0, 2)]
        .map(|center| (center, Real::one()));
    // Center I=1 is outside iso=11/10; the four bridges at (+/-1,+/-1)
    // have I=6/5 and are inside. Keep every source and the declared 25^2 grid.
    let region = curve::metaballs(
        &balls,
        (25, 25),
        (Real::from(11) / Real::from(10)).unwrap(),
        0,
    )
    .into_region();
    assert!(!region.is_empty());
    assert_eq!(curve::contains_xy(&region, 0, 0).unwrap(), Some(false));
    for (x, y) in [(2, 0), (-2, 0), (0, 2), (0, -2), (1, 1)] {
        assert_eq!(curve::contains_xy(&region, x, y).unwrap(), Some(true));
    }
}

#[test]
/// Keeps strict field uncertainty typed under three meshing-policy passes, then samples a valid source.
/// Checks certified local work, empty/Boolean continuation and height-two extrusion edge/volume invariants.
/// Recovery changes the radius rather than increasing the failing attempt's budget.
fn failed_construction_does_not_contaminate_later_policies_or_composition() {
    use csgrs::{GeometryCertainty, GeometryContext};
    use hypercurve::BooleanOp;
    use std::error::Error;
    for context in [
        GeometryContext::STRICT,
        GeometryContext::APPROXIMATE_512,
        GeometryContext::STRICT,
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
        let empty = curve::metaballs(&[], (3, 3), 1, 0).unwrap();
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
            curve::try_extrude(&result.value, 2.into(), GeometryContext::STRICT).unwrap();
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
/// Retains the 12-by-12 unit-source regression and requires nonempty material containing its center.
fn the_original_twelve_by_twelve_closed_fixture_remains_supported() {
    let region = curve::metaballs(&one_ball(1.into()), (12, 12), 1, 0).unwrap();
    assert!(!region.is_empty());
    assert_eq!(curve::contains_xy(&region, 0, 0).unwrap(), Some(true));
}
