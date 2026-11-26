//! Planar modeling operations preserve matrix meaning and typed failure.
#![cfg(feature = "curve")]
#[path = "support/uncertainty.rs"]
/// Shared proved-zero control and unresolved scalar witness for matrix/query admission.
mod uncertainty;

use csgrs::{GeometryCertainty, GeometryContext, Real, curve, errors::ValidationError};
use hypercurve::{BooleanOp, CurveRegion2, ExactCurveError};
use hyperlattice::Matrix4;
use std::error::Error;

/// Normalizes legacy values and current fallible APIs without hiding a successful empty/None result.
/// This compatibility witness ensures changed signatures cannot retroactively reinterpret baseline success as error.
trait Attempt<T> {
    /// Preserves successful values and converts only an existing error to the modeling vocabulary.
    fn attempt(self) -> Result<T, ValidationError>;
}
impl Attempt<CurveRegion2> for CurveRegion2 {
    /// Wraps a region as success even when it is empty.
    fn attempt(self) -> Result<CurveRegion2, ValidationError> {
        Ok(self)
    }
}
impl Attempt<Option<bool>> for Option<bool> {
    /// Wraps a classification as success, including `None` for a boundary result.
    fn attempt(self) -> Result<Option<bool>, ValidationError> {
        Ok(self)
    }
}
impl<T, E: Into<ValidationError>> Attempt<T> for Result<T, E> {
    /// Keeps the success value and converts the existing error without inspecting its category.
    fn attempt(self) -> Result<T, ValidationError> {
        self.map_err(Into::into)
    }
}
/// Checks the retained Pythagorean control and returns the shared strictly unresolved exact zero.
fn zero_unknown_to_bounded_predicates() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    uncertainty::terminal_zero(&sine * &sine + &cosine * &cosine - Real::one())
}
/// Exact one-half for hole placement and nonvertex membership queries.
fn half() -> Real {
    (Real::one() / Real::from(2)).unwrap()
}

#[test]
/// Requires singular X scale to remain invalid before both empty and nonempty region handling.
fn primary_scale_rejects_singular_input_including_empty() {
    for region in [curve::empty(), curve::square(1).unwrap()] {
        let error = curve::scaled(&region, 0, 1)
            .attempt()
            .expect_err("singular scale became successful geometry");
        assert_eq!(error.kind(), csgrs::errors::ValidationErrorKind::InvalidInput);
    }
}

#[test]
/// Requires errors for a singular planar matrix on empty and square inputs, without asserting the error category.
fn primary_transform_rejects_singular_input_including_empty() {
    let matrix = Matrix4::affine_nonuniform_scale([0.into(), 1.into(), 1.into()]);
    for region in [curve::empty(), curve::square(1).unwrap()] {
        assert!(curve::transformed(&region, &matrix).attempt().is_err());
    }
}

#[test]
/// Requires undecided scale/determinant admission instead of successful collapsed geometry.
fn unknown_scale_does_not_become_empty_geometry() {
    let error = curve::scaled(
        &curve::square(1).unwrap(),
        zero_unknown_to_bounded_predicates(),
        1,
    )
    .attempt()
    .expect_err("unknown determinant became successful geometry");
    assert_eq!(error.kind(), csgrs::errors::ValidationErrorKind::Undecided);
}

#[test]
/// Requires an unresolved face-coordinate query to return undecided, not a successful boundary `None`.
fn primary_query_distinguishes_uncertainty_from_boundary() {
    let square = curve::square(1).unwrap();
    let error = curve::contains_xy(&square, zero_unknown_to_bounded_predicates(), half())
        .attempt()
        .expect_err("unknown point location became a boundary result");
    assert_eq!(error.kind(), csgrs::errors::ValidationErrorKind::Undecided);
}

#[test]
/// Checks contextual query uncertainty retains the exact-curve variant and a downcastable native source.
fn contextual_query_retains_the_native_blocker() {
    let error = curve::contains_xy_with_context(
        &curve::square(1).unwrap(),
        zero_unknown_to_bounded_predicates(),
        half(),
        GeometryContext::STRICT,
    )
    .unwrap_err();
    assert_eq!(error.kind(), csgrs::errors::ValidationErrorKind::Undecided);
    assert!(matches!(error, ValidationError::ExactCurve(_)));
    assert!(
        error
            .source()
            .unwrap()
            .downcast_ref::<ExactCurveError>()
            .is_some()
    );
}

#[test]
/// Normalizes three signed homogeneous gauges of the same invertible XY map despite a singular 4D matrix.
/// Checks exact area six, bounds, one hole-free linear profile and all four transformed corner values.
fn supported_homogeneous_scale_is_divided_out() {
    let square = curve::square(1).unwrap();
    // On z=0: (x,y) -> (2*x+1, -3*y+2). The third column is irrelevant.
    // The 4D matrix need not be invertible for the planar map to be invertible.
    let values = [4, 0, 17, 2, 0, -6, 19, 4, 0, 0, 0, 0, 0, 0, 23, 2];
    for gauge in [1, -1, 3] {
        let matrix = Matrix4::from_row_major(values.map(|v| Real::from(v * gauge)));
        let region = curve::transformed(&square, &matrix).attempt().unwrap();
        assert_eq!(region.filled_area().unwrap(), Some(6.into()));
        let bounds = curve::bounding_box(&region).unwrap().unwrap();
        assert_eq!(bounds.mins.x, Real::one());
        assert_eq!(bounds.maxs.x, Real::from(3));
        assert_eq!(bounds.mins.y, Real::from(-1));
        assert_eq!(bounds.maxs.y, Real::from(2));
        let profiles = region.boundary_profiles().unwrap();
        assert_eq!(profiles.len(), 1);
        let (material, holes) = profiles[0].linear_contours().unwrap().unwrap();
        assert!(holes.is_empty());
        let vertices = material
            .segments()
            .iter()
            .map(|edge| [edge.start().x().clone(), edge.start().y().clone()])
            .collect::<Vec<_>>();
        assert_eq!(vertices.len(), 4);
        for [x, y] in [[1, -1], [3, -1], [3, 2], [1, 2]] {
            assert!(vertices.contains(&[Real::from(x), Real::from(y)]));
        }
    }
}

#[test]
/// Requires an X-dependent homogeneous weight to remain unsupported even for an empty region.
fn perspective_is_not_silently_discarded() {
    let matrix = Matrix4::from_row_major(
        [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1].map(Real::from),
    );
    for region in [curve::empty(), curve::square(1).unwrap()] {
        let error = curve::transformed(&region, &matrix)
            .attempt()
            .expect_err("perspective became an affine identity");
        assert_eq!(error.kind(), csgrs::errors::ValidationErrorKind::Unsupported);
    }
}

#[test]
/// Requires nonzero Z contributions from X, Y or translation to remain unsupported before empty handling.
fn out_of_plane_geometry_is_not_silently_projected() {
    for (row, column) in [(2, 0), (2, 1), (2, 3)] {
        let mut matrix = Matrix4::identity();
        matrix.0[row][column] = 1.into();
        for region in [curve::empty(), curve::square(1).unwrap()] {
            let error = curve::transformed(&region, &matrix)
                .attempt()
                .expect_err("out-of-plane image became an XY projection");
            assert_eq!(error.kind(), csgrs::errors::ValidationErrorKind::Unsupported);
        }
    }
}

#[test]
/// Distinguishes a nonzero coordinate image at zero homogeneous weight as unbounded for both inputs.
fn zero_homogeneous_weight_is_not_finite_geometry() {
    let mut matrix = Matrix4::identity();
    matrix.0[3][3] = 0.into();
    for region in [curve::empty(), curve::square(1).unwrap()] {
        let error = curve::transformed(&region, &matrix)
            .attempt()
            .expect_err("zero homogeneous weight became finite geometry");
        assert_eq!(error.kind(), csgrs::errors::ValidationErrorKind::Unbounded);
    }
}

#[test]
/// Requires the all-zero homogeneous image to be invalid, rather than a point at infinity or empty success.
fn a_zero_homogeneous_coordinate_image_is_invalid_instead_of_unbounded() {
    let matrix = Matrix4::from_row_major(std::array::from_fn(|_| Real::zero()));
    for region in [curve::empty(), curve::square(1).unwrap()] {
        let error = curve::transformed(&region, &matrix).unwrap_err();
        assert_eq!(error.kind(), csgrs::errors::ValidationErrorKind::InvalidInput);
    }
}

#[test]
/// Uses an unresolved exact-zero Z translation to exercise strict/approximate/strict plane admission.
/// Strict calls stay undecided; approximate output retains its local qualification and exact area one.
fn matrix_admission_uses_the_selected_policy_without_certainty_upgrade() {
    let square = curve::square(1).unwrap();
    let mut matrix = Matrix4::identity();
    matrix.0[2][3] = zero_unknown_to_bounded_predicates();
    for context in [
        GeometryContext::STRICT,
        GeometryContext::APPROXIMATE_512,
        GeometryContext::STRICT,
    ] {
        let result = curve::transformed_with_context(&square, &matrix, context);
        if context == GeometryContext::STRICT {
            let error = result
                .attempt()
                .expect_err("undecided plane constraint was ignored");
            assert_eq!(error.kind(), csgrs::errors::ValidationErrorKind::Undecided);
        } else {
            let result = result.unwrap();
            assert_eq!(result.certainty, GeometryCertainty::Approximate512Consumed);
            assert_eq!(result.value.filled_area().unwrap(), Some(1.into()));
        }
    }
}

#[test]
/// Checks definite square interior/boundary/exterior classifications and canonical-empty outside membership.
fn boundary_inside_outside_and_empty_queries_remain_definite() {
    let square = curve::square(1).unwrap();
    for (x, y, expected) in [
        (half(), half(), Some(true)),
        (0.into(), half(), None),
        ((-1).into(), half(), Some(false)),
    ] {
        assert_eq!(curve::contains_xy(&square, x, y).attempt().unwrap(), expected);
    }
    assert_eq!(
        curve::contains_xy(&curve::empty(), 0, 0).attempt().unwrap(),
        Some(false)
    );
}

#[test]
/// Translates, reflects/scales, rotates and identity-transforms empty geometry before union and subtraction.
/// Requires emptiness to remain consumable, restored area four and a subsequent empty difference.
fn canonical_empty_transforms_feed_subsequent_composition() {
    let empty = curve::translated(&curve::empty(), 2, 3).attempt().unwrap();
    let empty = curve::scaled(&empty, -2, 3).attempt().unwrap();
    let empty = curve::rotated(&empty, 90).attempt().unwrap();
    let empty = curve::transformed(&empty, &Matrix4::identity())
        .attempt()
        .unwrap();
    assert!(empty.is_empty());
    let square = curve::square(2).unwrap();
    let restored = empty.boolean_region(&square, BooleanOp::Union).unwrap();
    assert_eq!(restored.filled_area().unwrap(), Some(4.into()));
    assert!(
        restored
            .boolean_region(&square, BooleanOp::Difference)
            .unwrap()
            .is_empty()
    );
}

#[test]
/// Reflects/scales, translates and rotates a square with a square hole, checking area eighteen at each stage.
/// Height-two extrusion must be locally certified, have paired geometric edges and signed six-volume 216.
fn strict_transforms_keep_holes_area_and_extrusion_volume() {
    let square = curve::square(2).unwrap();
    let hole = curve::translated(&curve::square(1).unwrap(), half(), half())
        .attempt()
        .unwrap();
    let cut = square.boolean_region(&hole, BooleanOp::Difference).unwrap();
    let scaled = curve::scaled(&cut, -2, 3).attempt().unwrap();
    let moved = curve::translated(&scaled, 5, -4).attempt().unwrap();
    let rotated = curve::rotated(&moved, 90).attempt().unwrap();
    for region in [scaled, moved, rotated] {
        assert_eq!(region.filled_area().unwrap(), Some(18.into()));
        let mesh = curve::try_extrude(&region, 2.into(), GeometryContext::STRICT).unwrap();
        assert_eq!(mesh.certainty, GeometryCertainty::Certified);
        assert!(
            mesh.value
                .is_closed_manifold_geometry(&GeometryContext::STRICT.mesh_context())
                .unwrap()
                .value
        );
        let six_volume = mesh
            .value
            .triangles
            .iter()
            .fold(Real::zero(), |sum, triangle| {
                let [a, b, c] = triangle
                    .indices()
                    .map(|i| mesh.value.positions[i].to_vector());
                sum + a.dot(&b.cross(&c))
            });
        assert_eq!(six_volume, Real::from(216));
    }
}

#[test]
/// Requires ordinary/contextual singular scaling through the raw adapter to retain exact-curve modeling sources.
fn scalar_adapter_preserves_native_transform_errors() {
    use csgrs::adapter::{AdapterError, RawCurveRegion};
    let square = RawCurveRegion::square(1.into()).unwrap();
    for result in [
        square.scale(0.into(), 1.into(), 1.into()),
        square
            .scale_with_context(0.into(), 1.into(), 1.into(), GeometryContext::STRICT)
            .map(|outcome| outcome.value),
    ] {
        let error = result.unwrap_err();
        assert!(matches!(
            error,
            AdapterError::Construction(ValidationError::ExactCurve(_))
        ));
        assert!(
            error
                .source()
                .unwrap()
                .downcast_ref::<ValidationError>()
                .is_some()
        );
    }
}

#[test]
/// Requires NaN Z translation and infinite Z scale to fail even though planar source coordinates have zero Z.
/// Checks error presence only, not a specific scalar category.
fn scalar_adapter_does_not_ignore_nonfinite_third_components() {
    let square = csgrs::adapter::CurveRegionF64::square(1.0).unwrap();
    assert!(square.translate(0.0, 0.0, f64::NAN).is_err());
    assert!(square.scale(1.0, 1.0, f64::INFINITY).is_err());
}

#[test]
/// Requires Z translation and an X quarter-turn to retain unsupported-transform errors through both adapter paths.
fn scalar_adapter_rejects_out_of_plane_translation_and_rotation() {
    use csgrs::adapter::{AdapterError, RawCurveRegion};
    let square = RawCurveRegion::square(1.into()).unwrap();
    for result in [
        square.translate(0.into(), 0.into(), 1.into()),
        square.rotate(90.into(), 0.into(), 0.into()),
        square
            .translate_with_context(0.into(), 0.into(), 1.into(), GeometryContext::STRICT)
            .map(|outcome| outcome.value),
        square
            .rotate_with_context(90.into(), 0.into(), 0.into(), GeometryContext::STRICT)
            .map(|outcome| outcome.value),
    ] {
        let error = result.expect_err("out-of-plane argument was ignored");
        let AdapterError::Construction(error) = error else {
            panic!("adapter discarded the native diagnostic");
        };
        assert_eq!(error.kind(), csgrs::errors::ValidationErrorKind::Unsupported);
    }
}

#[test]
/// Checks ordinary/contextual X half-turns reflect the square's Y bounds while remaining in the XY plane.
fn scalar_adapter_keeps_plane_preserving_half_turns() {
    let square = csgrs::adapter::CurveRegionF64::square(1.0).unwrap();
    // X half-turn maps (x,y,0) to (x,-y,0), not the identity.
    for rotated in [
        square.rotate(180.0, 0.0, 0.0).unwrap(),
        square
            .rotate_with_context(180.0, 0.0, 0.0, GeometryContext::STRICT)
            .unwrap()
            .value,
    ] {
        let bounds = rotated.bounding_box().unwrap().unwrap();
        assert_eq!(bounds.mins, [0.0, -1.0, 0.0]);
        assert_eq!(bounds.maxs, [1.0, 0.0, 0.0]);
    }
}

#[test]
/// Warms identity admission, adds exact rational `10^-12` Z translation and requires unsupported errors under both policies.
/// Restoring zero must recover area one; this changes the input rather than retrying with a larger budget.
fn changed_matrix_constraints_are_rechecked_and_recovery_succeeds() {
    let square = curve::square(1).unwrap();
    let mut matrix = Matrix4::identity();
    curve::transformed(&square, &matrix).unwrap();
    // A nonzero exact offset remains nonzero under either policy.
    matrix.0[2][3] = (Real::one() / Real::from(1_000_000_000_000_u64)).unwrap();
    for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
        assert_eq!(
            curve::transformed_with_context(&square, &matrix, context)
                .unwrap_err()
                .kind(),
            csgrs::errors::ValidationErrorKind::Unsupported
        );
    }
    matrix.0[2][3] = 0.into();
    assert_eq!(
        curve::transformed(&square, &matrix)
            .unwrap()
            .filled_area()
            .unwrap(),
        Some(1.into())
    );
}

#[test]
/// Compares affine-transform-after-cut with cut-after-transform for an exact square/hole fixture.
/// Requires determinant-scaled area eighteen on both paths and empty symmetric difference; the native Boolean
/// equivalence check is separate from the independently derived area expectation.
fn affine_transform_commutes_with_a_regularized_cut() {
    let square = curve::square(2).unwrap();
    let hole = curve::translated(&curve::square(1).unwrap(), half(), half()).unwrap();
    let matrix = Matrix4::from_row_major(
        [2, 1, 0, 3, 0, -3, 0, -2, 0, 0, 1, 0, 0, 0, 0, 1].map(Real::from),
    );
    let transformed_cut = curve::transformed(
        &square.boolean_region(&hole, BooleanOp::Difference).unwrap(),
        &matrix,
    )
    .unwrap();
    let cut_transformed = curve::transformed(&square, &matrix)
        .unwrap()
        .boolean_region(
            &curve::transformed(&hole, &matrix).unwrap(),
            BooleanOp::Difference,
        )
        .unwrap();
    // The source area is 3, and |det([[2,1],[0,-3]])| = 6.
    assert_eq!(transformed_cut.filled_area().unwrap(), Some(18.into()));
    assert_eq!(cut_transformed.filled_area().unwrap(), Some(18.into()));
    assert!(
        transformed_cut
            .boolean_region(&cut_transformed, BooleanOp::Xor)
            .unwrap()
            .is_empty()
    );
}
