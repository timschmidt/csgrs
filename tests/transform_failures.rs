//! Transform failure is not a successfully constructed (possibly empty) model.
#[path = "support/uncertainty.rs"]
mod uncertainty;

use csgrs::{
    Real, TriangleMesh,
    adapter::{AdapterError, RawTriangleMesh},
    errors::ValidationError,
    solid::{self, SolidExt},
};
use hyperlattice::{Matrix4, Point3};
use hypermesh::{HypermeshError, MeshContext, Plane, Triangle};
use std::error::Error;

fn malformed() -> TriangleMesh {
    TriangleMesh::new(vec![Point3::origin()], vec![Triangle::new(0, 1, 0)])
}

fn reciprocal_x() -> Matrix4 {
    // (x, y, z) -> (1/x, y/x, z/x). Invertible, but undefined on x = 0.
    Matrix4::from_row_major([0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0].map(Real::from))
}

#[test]
fn singular_transform_is_a_failure_even_for_empty_input() {
    let singular = Matrix4::affine_nonuniform_scale([Real::zero(), Real::one(), Real::one()]);
    for mesh in [solid::empty(), solid::cube(Real::one()).unwrap()] {
        assert!(solid::transform(&mesh, &singular).is_err());
        assert!(mesh.transformed(&singular).is_err());
        assert!(solid::scale(&mesh, Real::zero(), Real::one(), Real::one()).is_err());
    }
}

#[test]
fn invalid_plane_does_not_become_empty_geometry() {
    let invalid =
        Plane::from_coefficients(Real::zero(), Real::zero(), Real::zero(), Real::one());
    assert!(solid::mirror(&solid::cube(Real::one()).unwrap(), &invalid).is_err());
}

#[test]
fn native_transforms_reject_invalid_indices() {
    let mesh = malformed();
    let context = MeshContext::new(hyperlimit::PredicatePolicy::STRICT);
    assert!(mesh.try_transformed(&context, &Matrix4::identity()).is_err());
    assert!(
        mesh.try_rotated_xyz_degrees(&context, Real::zero(), Real::zero(), Real::zero())
            .is_err()
    );
}

#[test]
fn modeling_transforms_reject_invalid_indices() {
    let mesh = malformed();
    assert!(
        mesh.translated(Real::one(), Real::zero(), Real::zero())
            .is_err()
    );
    assert!(solid::rotate(&mesh, Real::zero(), Real::zero(), Real::zero()).is_err());
}

#[test]
fn projective_triangle_cannot_cross_infinity_between_finite_vertices() {
    let mesh = TriangleMesh::new(
        vec![
            Point3::new((-1).into(), 0.into(), 0.into()),
            Point3::new(1.into(), 0.into(), 0.into()),
            Point3::new(1.into(), 1.into(), 0.into()),
        ],
        vec![Triangle::new(0, 1, 2)],
    );
    let context = MeshContext::new(hyperlimit::PredicatePolicy::STRICT);
    // Its edge contains x = 0, although every supplied vertex has nonzero x.
    assert_eq!(
        mesh.try_transformed(&context, &reciprocal_x()).unwrap_err(),
        HypermeshError::UnboundedProjectiveTriangle { triangle_index: 0 }
    );
    let error = solid::transform(&mesh, &reciprocal_x()).unwrap_err();
    assert_eq!(
        error,
        ValidationError::Mesh(HypermeshError::UnboundedProjectiveTriangle {
            triangle_index: 0
        })
    );
    assert_eq!(error.kind(), csgrs::errors::ValidationErrorKind::Unbounded);
    assert!(
        error
            .source()
            .unwrap()
            .downcast_ref::<HypermeshError>()
            .is_some()
    );
    assert!(matches!(
        RawTriangleMesh::from_native(mesh).transform(&reciprocal_x()),
        Err(AdapterError::Construction(ValidationError::Mesh(
            HypermeshError::UnboundedProjectiveTriangle { triangle_index: 0 }
        )))
    ));
}

#[test]
fn separate_components_may_have_opposite_projective_scale_signs() {
    let positions = [
        (1, 0, 0),
        (2, 0, 0),
        (1, 1, 0),
        (-1, 0, 0),
        (-2, 0, 0),
        (-1, 1, 0),
    ]
    .map(|(x, y, z)| Point3::new(x.into(), y.into(), z.into()))
    .to_vec();
    let mesh = TriangleMesh::new(
        positions,
        vec![Triangle::new(0, 1, 2), Triangle::new(3, 4, 5)],
    );
    let half = (Real::one() / Real::from(2)).unwrap();
    let expected = [
        Point3::new(1.into(), 0.into(), 0.into()),
        Point3::new(half.clone(), 0.into(), 0.into()),
        Point3::new(1.into(), 1.into(), 0.into()),
        Point3::new((-1).into(), 0.into(), 0.into()),
        Point3::new(-half, 0.into(), 0.into()),
        Point3::new((-1).into(), (-1).into(), 0.into()),
    ];
    for context in [
        csgrs::GeometryContext::STRICT,
        csgrs::GeometryContext::APPROXIMATE_512,
    ] {
        let result =
            solid::try_transform_with_context(&mesh, &reciprocal_x(), &context).unwrap();
        assert_eq!(result.certainty, csgrs::GeometryCertainty::Certified);
        assert_eq!(result.value.positions.as_ref(), &expected);
        // The determinant is -1; reverse winding to preserve the represented orientation.
        assert_eq!(
            result.value.triangles.as_ref(),
            &[Triangle::new(2, 1, 0), Triangle::new(5, 4, 3)]
        );
    }
}

#[test]
fn a_vertex_at_infinity_is_a_typed_unbounded_result() {
    let point = TriangleMesh::new(vec![Point3::origin()], Vec::new());
    let error = solid::transform(&point, &reciprocal_x()).unwrap_err();
    assert_eq!(error, ValidationError::Mesh(HypermeshError::PointAtInfinity));
    assert_eq!(error.kind(), csgrs::errors::ValidationErrorKind::Unbounded);
}

#[test]
fn unknown_transform_is_not_invalid_input_or_empty_success() {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    let zero = uncertainty::terminal_zero(&sine * &sine + &cosine * &cosine - Real::one());
    let cube = RawTriangleMesh::cube(Real::one()).unwrap();
    assert!(matches!(
        cube.scale(zero.clone(), Real::one(), Real::one()),
        Err(AdapterError::Construction(
            ValidationError::PredicateUndecided { .. }
        ))
    ));
    assert!(matches!(
        cube.scale_with_context(
            zero.clone(),
            Real::one(),
            Real::one(),
            &csgrs::GeometryContext::APPROXIMATE_512
        ),
        Err(AdapterError::Construction(ValidationError::InvalidArguments))
    ));
    assert!(matches!(
        cube.scale(zero, Real::one(), Real::one()),
        Err(AdapterError::Construction(
            ValidationError::PredicateUndecided { .. }
        ))
    ));
}

#[test]
fn exact_transforms_preserve_topology_volume_and_subsequent_composition() {
    let cube = solid::cube(Real::one()).unwrap();
    let scaled = solid::scale(&cube, (-2).into(), 3.into(), 4.into()).unwrap();
    let shifted = scaled.translated(5.into(), 6.into(), 7.into()).unwrap();
    let rotated = solid::rotate(&shifted, 0.into(), 0.into(), 90.into()).unwrap();
    let reflected = solid::mirror(&rotated, &Plane::axis_aligned(0, 0.into())).unwrap();
    for mesh in [scaled, shifted, rotated, reflected] {
        // Independent signed tetrahedral volume: |det(scale)| * unit cube = 24.
        let six_volume = mesh.triangles.iter().fold(Real::zero(), |volume, t| {
            let [a, b, c] = t.indices().map(|i| mesh.positions[i].to_vector());
            volume + a.dot(&b.cross(&c))
        });
        assert_eq!(six_volume, Real::from(144));
        assert!(
            mesh.is_closed_manifold_geometry(&csgrs::GeometryContext::STRICT.mesh_context())
                .unwrap()
                .value
        );
        let restored = solid::empty().try_union(&mesh).unwrap();
        let removed = restored.try_difference(&mesh).unwrap();
        assert!(removed.positions.is_empty() && removed.triangles.is_empty());
    }
}

#[test]
fn canonical_empty_transforms_compose_and_mutated_indices_still_fail() {
    let empty = solid::empty()
        .translated(1.into(), 2.into(), 3.into())
        .unwrap();
    let empty = solid::scale(&empty, (-1).into(), 2.into(), 3.into()).unwrap();
    let empty = solid::rotate(&empty, 0.into(), 0.into(), 90.into()).unwrap();
    let empty = solid::mirror(&empty, &Plane::axis_aligned(0, 0.into())).unwrap();
    assert!(empty.positions.is_empty() && empty.triangles.is_empty());
    let cube = solid::cube(Real::one()).unwrap();
    let mut restored = empty.try_union(&cube).unwrap();
    restored.translated(1.into(), 0.into(), 0.into()).unwrap();
    std::sync::Arc::make_mut(&mut restored.triangles)[0].v0 = restored.positions.len();
    assert!(matches!(
        restored.translated(1.into(), 0.into(), 0.into()),
        Err(ValidationError::Mesh(
            HypermeshError::VertexIndexOutOfBounds { .. }
        ))
    ));
}

#[test]
fn adapter_preserves_the_native_validation_payload() {
    let cube = RawTriangleMesh::cube(Real::one()).unwrap();
    let error = cube
        .scale(Real::zero(), Real::one(), Real::one())
        .unwrap_err();
    assert!(matches!(
        error,
        AdapterError::Construction(ValidationError::InvalidArguments)
    ));
    assert!(
        error
            .source()
            .unwrap()
            .downcast_ref::<ValidationError>()
            .is_some()
    );
}
