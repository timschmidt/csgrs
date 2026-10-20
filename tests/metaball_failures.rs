#![cfg(feature = "metaballs")]
//! Invalid metaballs and failed samples must not publish successful geometry.
#[path = "support/uncertainty.rs"]
mod uncertainty;

use csgrs::{
    Real, TriangleMesh,
    errors::{ValidationError, ValidationErrorKind},
    solid::{self, MetaBall},
};
use hyperlattice::Point3;

trait FailureKind {
    fn failure_kind(&self) -> Option<ValidationErrorKind>;
}
impl<T> FailureKind for Result<T, ValidationError> {
    fn failure_kind(&self) -> Option<ValidationErrorKind> {
        self.as_ref().err().map(ValidationError::kind)
    }
}
trait InfluenceFailure {
    fn rejects_singularity(&self) -> bool;
}
impl InfluenceFailure for Result<Real, hypersdf::SdfEvaluationError> {
    fn rejects_singularity(&self) -> bool {
        matches!(
            self,
            Err(hypersdf::SdfEvaluationError::Scalar {
                source: hyperreal::Problem::DivideByZero,
                ..
            })
        )
    }
}
trait IntoMesh {
    fn into_mesh(self) -> Result<TriangleMesh, ValidationError>;
}
impl IntoMesh for Result<TriangleMesh, ValidationError> {
    fn into_mesh(self) -> Result<TriangleMesh, ValidationError> {
        self
    }
}
fn ball(radius: Real) -> MetaBall {
    MetaBall::new(Point3::origin(), radius)
}
fn unknown_zero() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    uncertainty::terminal_zero(&sine * &sine + &cosine * &cosine - Real::one())
}

#[test]
fn zero_radius_is_invalid_input_instead_of_empty_geometry() {
    assert_eq!(
        solid::metaballs(&[ball(0.into())], (3, 3, 3), 1.into(), 0.into()).failure_kind(),
        Some(ValidationErrorKind::InvalidInput)
    );
}
#[test]
fn negative_padding_is_invalid_input_instead_of_empty_geometry() {
    assert_eq!(
        solid::metaballs_with_diagnostics(&[ball(1.into())], (3, 3, 3), 1.into(), (-1).into())
            .failure_kind(),
        Some(ValidationErrorKind::InvalidInput)
    );
}
#[test]
fn unresolved_radius_is_undecided_instead_of_invalid_or_empty() {
    assert_eq!(
        solid::metaballs(&[ball(unknown_zero())], (3, 3, 3), 1.into(), 0.into())
            .failure_kind(),
        Some(ValidationErrorKind::Undecided)
    );
}
#[test]
fn empty_input_does_not_hide_invalid_iso_value() {
    assert_eq!(
        solid::metaballs(&[], (3, 3, 3), 0.into(), 0.into()).failure_kind(),
        Some(ValidationErrorKind::InvalidInput)
    );
}
#[test]
fn empty_input_does_not_hide_invalid_padding() {
    assert_eq!(
        solid::metaballs(&[], (3, 3, 3), 1.into(), (-1).into()).failure_kind(),
        Some(ValidationErrorKind::InvalidInput)
    );
}
#[test]
fn resolution_is_validated_instead_of_silently_clamped() {
    assert_eq!(
        solid::metaballs(&[ball(1.into())], (1, 3, 3), 1.into(), 0.into()).failure_kind(),
        Some(ValidationErrorKind::InvalidInput)
    );
}
#[test]
fn grid_overflow_is_resource_exhaustion() {
    assert_eq!(
        solid::metaballs_with_diagnostics(
            &[ball(1.into())],
            (usize::MAX, 2, 2),
            1.into(),
            0.into()
        )
        .failure_kind(),
        Some(ValidationErrorKind::ResourceExhausted)
    );
}
#[test]
fn inverse_square_influence_does_not_discard_distant_contributions() {
    let value = ball(1.into())
        .influence(&Point3::new(100.into(), 0.into(), 0.into()))
        .expect("nonzero distance");
    assert_eq!(value, (Real::one() / Real::from(10000)).unwrap());
}
#[test]
fn singular_inverse_square_influence_is_not_a_finite_sentinel() {
    assert!(
        ball(1.into())
            .influence(&Point3::origin())
            .rejects_singularity()
    );
}
#[test]
fn normalized_level_field_has_independently_known_exact_sampled_vertices() {
    // Declared migration: sample F=(iso-I)/(iso+I), continuously extended to -1 at poles.
    // For one unit ball, core half-extent a=101/100 and resolution 3,
    // an axis edge crosses at (a²+1)/(2a); averaging its three crossings
    // gives each coordinate magnitude 20201/60600. No f32 construction is involved.
    let mesh = solid::metaballs(&[ball(1.into())], (3, 3, 3), 1.into(), 0.into())
        .into_mesh()
        .unwrap();
    let coordinate = (Real::from(20201) / Real::from(60600)).unwrap();
    assert!(!mesh.triangles.is_empty());
    assert!(mesh.positions.iter().all(|p| {
        [&p.x, &p.y, &p.z]
            .into_iter()
            .all(|v| v == &coordinate || v == &(-&coordinate))
    }));
}

#[test]
fn sampled_body_has_exact_volume_topology_and_downstream_composition() {
    use csgrs::{GeometryCertainty, GeometryContext, solid::SolidExt};
    let balls = [ball(1.into())];
    let result = solid::metaballs_with_diagnostics_and_context(
        &balls,
        (3, 3, 3),
        1.into(),
        0.into(),
        &GeometryContext::APPROXIMATE_512,
    )
    .unwrap();
    assert_eq!(result.certainty, GeometryCertainty::Certified);
    let (mesh, diagnostics) = result.value;
    assert!(
        mesh.is_closed_manifold_geometry(&GeometryContext::STRICT.mesh_context())
            .unwrap()
            .value
    );
    let coordinate = (Real::from(20201) / Real::from(60600)).unwrap();
    let width = Real::from(2) * &coordinate;
    let six_volume = mesh.triangles.iter().fold(Real::zero(), |sum, triangle| {
        let [a, b, c] = triangle.indices().map(|i| mesh.positions[i].to_vector());
        sum + a.dot(&b.cross(&c))
    });
    assert_eq!(six_volume, Real::from(6) * &width * &width * &width);
    assert_eq!(diagnostics.resolution, (5, 5, 5));
    assert_eq!(diagnostics.sample_count, 125);
    assert_eq!(diagnostics.negative_sample_count, 1);
    assert_eq!(diagnostics.zero_sample_count, 0);
    assert_eq!(diagnostics.positive_sample_count, 124);
    assert_eq!(diagnostics.min_finite_value, Some(-Real::one()));
    assert!(solid::metaballs(&[ball(0.into())], (3, 3, 3), 1.into(), 0.into()).is_err());
    let empty = solid::metaballs(&[], (3, 3, 3), 1.into(), 0.into()).unwrap();
    assert!(empty.positions.is_empty() && empty.triangles.is_empty());
    let restored = empty.try_union(&mesh).unwrap();
    let removed = restored
        .try_difference(&mesh)
        .unwrap()
        .try_union(&empty)
        .unwrap();
    assert!(removed.positions.is_empty() && removed.triangles.is_empty());
}

#[test]
fn strict_field_errors_survive_approximate_meshing_and_recovery() {
    use csgrs::{GeometryCertainty, GeometryContext};
    use std::error::Error;
    for context in [
        &GeometryContext::STRICT,
        &GeometryContext::APPROXIMATE_512,
        &GeometryContext::STRICT,
    ] {
        let error = solid::metaballs_with_context(
            &[ball(unknown_zero())],
            (3, 3, 3),
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
    }
    let balls = [ball(1.into()), ball(1.into())];
    let result = solid::metaballs_with_context(
        &balls,
        (3, 3, 3),
        2.into(),
        0.into(),
        &GeometryContext::STRICT,
    )
    .unwrap();
    assert_eq!(result.certainty, GeometryCertainty::Certified);
    let expected = (Real::from(20201) / Real::from(60600)).unwrap();
    assert!(result.value.positions.iter().all(|p| {
        [&p.x, &p.y, &p.z]
            .into_iter()
            .all(|v| v == &expected || v == &(-&expected))
    }));
    assert!(!result.value.triangles.is_empty());
}
