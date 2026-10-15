//! Bounds preserve absence and uncertainty through the modeling boundary.
#[cfg(feature = "sdf")]
use csgrs::errors::ValidationError;
use csgrs::{
    Real, TriangleMesh,
    adapter::RawTriangleMesh,
    errors::ValidationErrorKind,
    solid::{self, SolidExt},
};
use hyperlattice::Point3;
use hypermesh::Triangle;
use std::error::Error;

fn uncertain_points() -> TriangleMesh {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    let zero = &sine * &sine + &cosine * &cosine - Real::one();
    TriangleMesh::new(
        vec![
            Point3::new(sine.clone(), 0.into(), sine.clone()),
            Point3::new(&sine + &zero, 1.into(), sine + zero),
        ],
        Vec::new(),
    )
}

#[test]
fn empty_bounds_are_absent_rather_than_a_point_at_the_origin() {
    let mesh = solid::empty();
    assert!(solid::try_bounding_box(&mesh).unwrap().is_none());
    assert!(solid::bounding_box(&mesh).unwrap().is_none());
    let point = TriangleMesh::new(vec![Point3::origin()], Vec::new());
    let bounds = SolidExt::exact_bounds(&point).unwrap().unwrap();
    assert_eq!(bounds.mins, Point3::origin());
    assert_eq!(bounds.maxs, Point3::origin());
    let placed = solid::float(&solid::center(&point).unwrap()).unwrap();
    assert_eq!(placed.positions.as_ref(), &[Point3::origin()]);
}

#[test]
fn undecided_bounds_do_not_become_an_origin_box_or_empty_model() {
    let mesh = uncertain_points();
    assert!(solid::bounding_box(&mesh).is_err());
    assert!(solid::center(&mesh).is_err());
    assert!(solid::float(&mesh).is_err());
}

#[test]
fn bounds_reject_malformed_indices() {
    let mesh = TriangleMesh::new(vec![Point3::origin()], vec![Triangle::new(0, 1, 0)]);
    assert!(
        mesh.exact_bounds(&csgrs::GeometryContext::STRICT.mesh_context())
            .is_err()
    );
    let error = solid::try_bounding_box(&mesh).unwrap_err();
    assert_eq!(error.kind(), ValidationErrorKind::InvalidInput);
    assert!(matches!(
        error
            .source()
            .unwrap()
            .downcast_ref::<hypermesh::HypermeshError>(),
        Some(hypermesh::HypermeshError::VertexIndexOutOfBounds { .. })
    ));
    assert!(solid::center(&mesh).is_err());
    assert!(solid::float(&mesh).is_err());
}

#[test]
fn placement_preserves_exact_box_geometry_and_empty_composition() {
    let source = solid::cuboid(2.into(), 4.into(), 6.into())
        .unwrap()
        .translated(7.into(), (-3).into(), 11.into())
        .unwrap();
    let centered = solid::center(&source).unwrap();
    let floated = solid::float(&centered).unwrap();
    for (mesh, minimum, maximum) in [
        (&centered, [-1, -2, -3], [1, 2, 3]),
        (&floated, [-1, -2, 0], [1, 2, 6]),
    ] {
        let bounds = solid::bounding_box(mesh).unwrap().unwrap();
        assert_eq!(
            bounds.mins,
            Point3::new(minimum[0].into(), minimum[1].into(), minimum[2].into())
        );
        assert_eq!(
            bounds.maxs,
            Point3::new(maximum[0].into(), maximum[1].into(), maximum[2].into())
        );
        let six_volume = mesh.triangles.iter().fold(Real::zero(), |sum, t| {
            let [a, b, c] = t.indices().map(|i| mesh.positions[i].to_vector());
            sum + a.dot(&b.cross(&c))
        });
        assert_eq!(six_volume, Real::from(288)); // 6 * 2 * 4 * 6.
        assert!(
            mesh.is_closed_manifold_geometry(&csgrs::GeometryContext::STRICT.mesh_context())
                .unwrap()
                .value
        );
        let empty = solid::float(&solid::center(&solid::empty()).unwrap()).unwrap();
        assert!(empty.positions.is_empty() && empty.triangles.is_empty());
        let restored = empty.try_union(mesh).unwrap();
        let removed = restored
            .try_difference(mesh)
            .unwrap()
            .try_difference(mesh)
            .unwrap();
        assert!(removed.positions.is_empty() && removed.triangles.is_empty());
    }
}

#[test]
fn cached_position_bounds_do_not_validate_mutated_indices() {
    let mut mesh = solid::cube(Real::one()).unwrap();
    solid::try_bounding_box(&mesh).unwrap();
    std::sync::Arc::make_mut(&mut mesh.triangles)[0].v0 = mesh.positions.len();
    assert!(solid::try_bounding_box(&mesh).is_err());
}

#[test]
fn undecided_bounds_retain_their_category_through_the_adapter() {
    let mesh = RawTriangleMesh::from_native(uncertain_points());
    let error = mesh.bounding_box().unwrap_err();
    let csgrs::adapter::AdapterError::Construction(error) = error else {
        panic!("native failure was flattened")
    };
    assert_eq!(error.kind(), ValidationErrorKind::Undecided);
}

#[cfg(feature = "curve")]
#[test]
fn empty_curve_bounds_are_absent() {
    let curve = hypercurve::CurveRegion2::empty();
    assert!(csgrs::curve::try_bounding_box(&curve).unwrap().is_none());
    assert!(csgrs::curve::bounding_box(&curve).unwrap().is_none());
    let paths = csgrs::curve::hilbert_strings_with_context(
        &curve,
        2,
        Real::zero(),
        &csgrs::GeometryContext::STRICT,
    )
    .unwrap();
    assert!(paths.value.is_empty());
    assert_eq!(paths.certainty, csgrs::GeometryCertainty::Certified);
    assert!(
        csgrs::curve::hilbert_strings_with_context(
            &curve,
            0,
            Real::zero(),
            &csgrs::GeometryContext::STRICT
        )
        .is_err()
    );
    let rectangle = csgrs::curve::rectangle(2.into(), 3.into());
    let bounds = csgrs::curve::bounding_box(&rectangle).unwrap().unwrap();
    assert_eq!(bounds.mins, Point3::origin());
    assert_eq!(bounds.maxs, Point3::new(2.into(), 3.into(), 0.into()));
}

#[cfg(feature = "sdf")]
#[test]
fn tpms_consumers_preserve_bounds_and_capacity_failures() {
    assert_eq!(
        solid::gyroid(&uncertain_points(), 4, 1.into(), 0.into())
            .unwrap_err()
            .kind(),
        ValidationErrorKind::Undecided
    );
    let mesh = solid::cube(Real::one()).unwrap();
    assert_eq!(
        solid::gyroid(&mesh, usize::MAX, 1.into(), 0.into()).unwrap_err(),
        ValidationError::CapacityOverflow
    );
}

#[cfg(feature = "sdf")]
#[test]
fn all_tpms_families_compose_on_empty_and_zero_thickness() {
    let empty = solid::empty();
    let cube = solid::cube(Real::one()).unwrap();
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    let unknown_zero = &sine * &sine + &cosine * &cosine - Real::one();
    for sheet in [solid::gyroid, solid::schwarz_p, solid::schwarz_d] {
        let mesh = sheet(&empty, 4, 1.into(), 0.into()).unwrap();
        assert!(mesh.positions.is_empty() && mesh.triangles.is_empty());
        let removed = mesh.try_union(&cube).unwrap().try_difference(&cube).unwrap();
        assert!(removed.positions.is_empty() && removed.triangles.is_empty());
        assert_eq!(
            sheet(&empty, 4, unknown_zero.clone(), 0.into())
                .unwrap_err()
                .kind(),
            ValidationErrorKind::Undecided
        );
        assert_eq!(
            sheet(&empty, 4, 0.into(), 0.into()).unwrap_err().kind(),
            ValidationErrorKind::InvalidInput
        );
    }
    for thick in [
        solid::gyroid_solid,
        solid::schwarz_p_solid,
        solid::schwarz_d_solid,
    ] {
        for bounds in [&empty, &cube] {
            let mesh = thick(bounds, 4, 1.into(), 0.into(), 0.into()).unwrap();
            assert!(mesh.positions.is_empty() && mesh.triangles.is_empty());
            assert_eq!(
                thick(bounds, 4, 1.into(), 0.into(), unknown_zero.clone())
                    .unwrap_err()
                    .kind(),
                ValidationErrorKind::Undecided
            );
            assert_eq!(
                thick(bounds, 4, 1.into(), 0.into(), (-1).into())
                    .unwrap_err()
                    .kind(),
                ValidationErrorKind::InvalidInput
            );
        }
        let mesh = thick(&empty, 4, 1.into(), 0.into(), 1.into()).unwrap();
        assert!(mesh.positions.is_empty() && mesh.triangles.is_empty());
    }
}

#[cfg(feature = "sdf")]
#[test]
fn tpms_invalid_parameters_are_not_successful_empty_models() {
    for mesh in [solid::empty(), solid::cube(Real::one()).unwrap()] {
        assert!(solid::gyroid(&mesh, 4, 0.into(), 0.into()).is_err());
        assert!(solid::schwarz_p_solid(&mesh, 4, 1.into(), 0.into(), (-1).into()).is_err());
        assert!(solid::schwarz_d(&mesh, 1, 1.into(), 0.into()).is_err());
    }
}

#[cfg(feature = "sdf")]
#[test]
fn shared_sampled_field_path_retains_native_failure_and_recovers() {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    let zero = &sine * &sine + &cosine * &cosine - Real::one();
    let context = csgrs::GeometryContext::STRICT;
    let error = solid::sdf_with_context(
        |_| zero.clone(),
        (3, 3, 3),
        Point3::origin(),
        Point3::new(1.into(), 1.into(), 1.into()),
        Real::zero(),
        &context,
    )
    .unwrap_err();
    assert_eq!(error.kind(), ValidationErrorKind::Undecided);
    assert!(matches!(
        error,
        ValidationError::SurfaceNets(hypermesh::SurfaceNetsError::UnknownSampleSign { .. })
    ));
    assert!(
        error
            .source()
            .unwrap()
            .downcast_ref::<hypermesh::SurfaceNetsError>()
            .is_some()
    );
    let complete = solid::sdf_with_context(
        |_| Real::one(),
        (3, 3, 3),
        Point3::origin(),
        Point3::new(1.into(), 1.into(), 1.into()),
        Real::zero(),
        &context,
    )
    .unwrap();
    assert_eq!(complete.certainty, csgrs::GeometryCertainty::Certified);
    assert!(complete.value.positions.is_empty() && complete.value.triangles.is_empty());
}
