#![cfg(feature = "metaballs")]
//! Invalid metaballs and failed samples must not publish successful geometry.
//! Independent coordinate/volume expectations describe normalized-field Surface Nets output.
#[path = "support/uncertainty.rs"]
/// Shared proved-zero control and independently unresolved radius witness.
mod uncertainty;

use csgrs::{
    Real, TriangleMesh,
    errors::{ValidationError, ValidationErrorKind},
    solid::{self, MetaBall},
};
use hyperlattice::Point3;

/// Observes a modeling result's error category without changing successful values.
trait FailureKind {
    /// Returns the category of an existing failure or `None` for success.
    fn failure_kind(&self) -> Option<ValidationErrorKind>;
}
impl<T> FailureKind for Result<T, ValidationError> {
    /// Borrows and classifies only the error side of the result.
    fn failure_kind(&self) -> Option<ValidationErrorKind> {
        self.as_ref().err().map(ValidationError::kind)
    }
}
/// Recognizes the specific singular inverse-square influence failure used by this fixture.
trait InfluenceFailure {
    /// Reports whether the result is a scalar divide-by-zero error, not merely any failure.
    fn rejects_singularity(&self) -> bool;
}
impl InfluenceFailure for Result<Real, hypersdf::SdfEvaluationError> {
    /// Matches the typed nested scalar cause without substituting a finite influence value.
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
/// Result pass-through retained by the sampled-mesh acceptance witness.
trait IntoMesh {
    /// Preserves the complete mesh result, including any typed failure.
    fn into_mesh(self) -> Result<TriangleMesh, ValidationError>;
}
impl IntoMesh for Result<TriangleMesh, ValidationError> {
    /// Returns the result unchanged; callers explicitly decide whether to unwrap it.
    fn into_mesh(self) -> Result<TriangleMesh, ValidationError> {
        self
    }
}
/// Creates an origin-centered source with the supplied radius for later field validation.
fn ball(radius: Real) -> MetaBall {
    MetaBall::new(Point3::origin(), radius)
}
/// Checks the retained Pythagorean control and obtains the shared strictly unresolved zero.
fn unknown_zero() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    uncertainty::terminal_zero(&sine * &sine + &cosine * &cosine - Real::one())
}

#[test]
/// Requires a zero source radius to remain invalid instead of producing empty geometry.
fn zero_radius_is_invalid_input_instead_of_empty_geometry() {
    assert_eq!(
        solid::metaballs(&[ball(0.into())], (3, 3, 3), 1, 0).failure_kind(),
        Some(ValidationErrorKind::InvalidInput)
    );
}
#[test]
/// Requires negative padding to fail through the contextual API even when diagnostics are requested.
fn negative_padding_is_invalid_input_instead_of_empty_geometry() {
    assert_eq!(
        solid::metaballs_with_context(
            &[ball(1.into())],
            (3, 3, 3),
            1.into(),
            (-1).into(),
            csgrs::GeometryContext::STRICT,
            csgrs::solid::SampleOptions::with_diagnostics()
        )
        .failure_kind(),
        Some(ValidationErrorKind::InvalidInput)
    );
}
#[test]
/// Distinguishes an undecided radius sign from a proved invalid radius or successful empty output.
fn unresolved_radius_is_undecided_instead_of_invalid_or_empty() {
    assert_eq!(
        solid::metaballs(&[ball(unknown_zero())], (3, 3, 3), 1, 0).failure_kind(),
        Some(ValidationErrorKind::Undecided)
    );
}
#[test]
/// Requires isovalue validation before an empty source list can return successfully.
fn empty_input_does_not_hide_invalid_iso_value() {
    assert_eq!(
        solid::metaballs(&[], (3, 3, 3), 0, 0).failure_kind(),
        Some(ValidationErrorKind::InvalidInput)
    );
}
#[test]
/// Requires nonnegative-padding validation even for an empty source list.
fn empty_input_does_not_hide_invalid_padding() {
    assert_eq!(
        solid::metaballs(&[], (3, 3, 3), 1, -1).failure_kind(),
        Some(ValidationErrorKind::InvalidInput)
    );
}
#[test]
/// Requires a one-sample grid axis to fail instead of silently changing the requested resolution.
fn resolution_is_validated_instead_of_silently_clamped() {
    assert_eq!(
        solid::metaballs(&[ball(1.into())], (1, 3, 3), 1, 0).failure_kind(),
        Some(ValidationErrorKind::InvalidInput)
    );
}
#[test]
/// Classifies an unaddressable grid as resource exhausted; no allocator or total-work limit is exercised.
fn grid_overflow_is_resource_exhaustion() {
    assert_eq!(
        solid::metaballs_with_context(
            &[ball(1.into())],
            (usize::MAX, 2, 2),
            1.into(),
            0.into(),
            csgrs::GeometryContext::STRICT,
            csgrs::solid::SampleOptions::with_diagnostics(),
        )
        .failure_kind(),
        Some(ValidationErrorKind::ResourceExhausted)
    );
}
#[test]
/// Checks the exact `1/10000` influence of a unit source at distance one hundred, without a cutoff.
fn inverse_square_influence_does_not_discard_distant_contributions() {
    let value = ball(1.into())
        .influence(&Point3::new(100.into(), 0.into(), 0.into()))
        .expect("nonzero distance");
    assert_eq!(value, (Real::one() / Real::from(10000)).unwrap());
}
#[test]
/// Requires raw inverse-square influence at the source center to retain divide-by-zero failure.
/// The separately normalized sampling field has its own continuous extension at that point.
fn singular_inverse_square_influence_is_not_a_finite_sentinel() {
    assert!(
        ball(1.into())
            .influence(&Point3::origin())
            .rejects_singularity()
    );
}
#[test]
/// Checks nonempty 3-cubed extraction has only the independently derived coordinate magnitudes `20201/60600`.
/// This checks sampled vertex coordinates, not equality to a continuous spherical level set.
fn normalized_level_field_has_independently_known_exact_sampled_vertices() {
    // Declared migration: sample F=(iso-I)/(iso+I), continuously extended to -1 at poles.
    // For one unit ball, core half-extent a=101/100 and resolution 3,
    // an axis edge crosses at (a²+1)/(2a); averaging its three crossings
    // gives each coordinate magnitude 20201/60600. No f32 construction is involved.
    let mesh = solid::metaballs(&[ball(1.into())], (3, 3, 3), 1, 0)
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
/// Checks certified local extraction, paired geometric edges, exact signed volume and bordered-grid diagnostics.
/// A 3-cubed core becomes 5-cubed sampling; subsequent invalid/empty construction and Boolean removal
/// exercise continued use of successful sampled and canonical-empty outputs.
fn sampled_body_has_exact_volume_topology_and_downstream_composition() {
    use csgrs::{GeometryCertainty, GeometryContext, solid::SolidExt};
    let balls = [ball(1.into())];
    let result = solid::metaballs_with_context(
        &balls,
        (3, 3, 3),
        1.into(),
        0.into(),
        GeometryContext::APPROXIMATE_512,
        csgrs::solid::SampleOptions::with_diagnostics(),
    )
    .unwrap();
    assert_eq!(result.certainty, GeometryCertainty::Certified);
    let solid::SampledMesh {
        mesh,
        diagnostics: Some(diagnostics),
    } = result.value
    else {
        panic!("requested diagnostics must be returned");
    };
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
    assert!(solid::metaballs(&[ball(0.into())], (3, 3, 3), 1, 0).is_err());
    let empty = solid::metaballs(&[], (3, 3, 3), 1, 0).unwrap();
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
/// Requires native field uncertainty under strict/approximate/strict meshing policies, then recovers with valid inputs.
/// Two coincident unit sources at isovalue two must yield the same independent coordinate magnitudes;
/// this changes the field rather than retrying the unresolved radius with a larger budget.
fn strict_field_errors_survive_approximate_meshing_and_recovery() {
    use csgrs::{GeometryCertainty, GeometryContext};
    use std::error::Error;
    for context in [
        GeometryContext::STRICT,
        GeometryContext::APPROXIMATE_512,
        GeometryContext::STRICT,
    ] {
        let error = solid::metaballs_with_context(
            &[ball(unknown_zero())],
            (3, 3, 3),
            1.into(),
            0.into(),
            context,
            csgrs::solid::SampleOptions::default(),
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
        GeometryContext::STRICT,
        csgrs::solid::SampleOptions::default(),
    )
    .unwrap();
    assert_eq!(result.certainty, GeometryCertainty::Certified);
    let expected = (Real::from(20201) / Real::from(60600)).unwrap();
    assert!(result.value.mesh.positions.iter().all(|p| {
        [&p.x, &p.y, &p.z]
            .into_iter()
            .all(|v| v == &expected || v == &(-&expected))
    }));
    assert!(!result.value.mesh.triangles.is_empty());
}
