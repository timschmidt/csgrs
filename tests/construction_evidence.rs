//! Construction qualifications are independent of geometry equality and local proofs.
#[path = "support/uncertainty.rs"]
mod uncertainty;

use csgrs::{
    ConstructionEvidence as Evidence, GeometryCertainty, GeometryContext, Real, TriangleMesh,
    solid,
};
use hyperlattice::{Matrix4, Point3, Vector3};
use hypermesh::{BooleanOp, BooleanProgram, MeshCertainty};
use std::{num::NonZeroU32, sync::Arc};

fn unresolved_zero() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    uncertainty::terminal_zero(&sine * &sine + &cosine * &cosine - Real::one())
}

fn approximate_mesh() -> TriangleMesh {
    let cube = solid::cube(Real::one()).unwrap();
    let mut points = cube.positions.to_vec();
    points.push(Point3::new(unresolved_zero(), 0.into(), 0.into()));
    let outcome =
        hypermesh::convex_hull(&GeometryContext::APPROXIMATE_512.mesh_context(), &points)
            .unwrap();
    assert_eq!(outcome.certainty, MeshCertainty::Approximate512Consumed);
    outcome.into_value()
}

fn assert_approximate(mesh: &TriangleMesh) {
    assert_eq!(
        mesh.construction_evidence().predicate_certainty(),
        Some(MeshCertainty::Approximate512Consumed)
    );
}

#[test]
fn local_validation_and_parent_composition_preserve_approximation() {
    let mesh = approximate_mesh();
    let context = GeometryContext::STRICT.mesh_context();
    assert_eq!(
        mesh.validate_closed_pwn(&context).unwrap().certainty,
        MeshCertainty::Certified
    );
    assert_approximate(&mesh);
    let cube = solid::cube(1.into()).unwrap();
    let evidence = mesh.construction_evidence();
    assert_eq!(core::mem::size_of::<Evidence>(), 1);
    assert_eq!(
        evidence.with_predicate_certainty(MeshCertainty::Certified),
        evidence
    );
    for derived in [
        mesh.clone(),
        mesh.reversed_winding(),
        mesh.subdivide_triangles(NonZeroU32::new(1).unwrap()).unwrap(),
        mesh.laplacian_smooth(&Real::from(0), 1).unwrap(),
        mesh.taubin_smooth(&Real::from(0), &Real::from(0), 1).unwrap(),
        mesh.convex_hull(&context).unwrap().value,
        mesh.try_transformed(
            &context,
            &Matrix4::affine_translation([2.into(), 3.into(), 4.into()]),
        )
        .unwrap()
        .value,
        TriangleMesh::try_concatenate(&[cube.clone(), mesh.clone()]).unwrap(),
        solid::minkowski_hull(&mesh, &cube).unwrap(),
        solid::minkowski_sum(&mesh, &solid::empty()).unwrap(),
        csgrs::adapter::TriangleMeshF64::from_native(mesh.clone())
            .inverse()
            .into_native(),
    ] {
        assert_approximate(&derived);
    }
    let projected = mesh.materialize_finite().unwrap();
    assert_approximate(&projected);
    assert!(projected.construction_evidence().coordinates_projected());
    assert!(!mesh.construction_evidence().coordinates_projected());
}

#[test]
fn mutation_and_borrowed_views_expose_missing_history_without_erasing_known_facts() {
    let mesh = approximate_mesh();
    let mut changed = mesh.clone();
    // Even an equal replacement lacks the retained construction identity.
    Arc::make_mut(&mut changed.positions)[0] = mesh.positions[0].clone();
    let evidence = changed.construction_evidence();
    assert!(evidence.source_history_missing() && evidence.buffers_changed());
    assert!(evidence.approximate_512_consumed());
    assert_eq!(evidence.predicate_certainty(), None);
    changed
        .validate_closed_pwn(&GeometryContext::STRICT.mesh_context())
        .unwrap();
    assert_eq!(changed.construction_evidence(), evidence);
    assert_approximate(&mesh);
    assert_eq!(
        mesh.as_ref().construction_evidence(),
        mesh.construction_evidence()
    );
    let mut view = mesh.as_ref();
    view.triangles = &[];
    assert!(view.construction_evidence().source_history_missing());
    assert!(view.construction_evidence().approximate_512_consumed());
    let mut changed_indices = mesh.clone();
    Arc::make_mut(&mut changed_indices.triangles).swap(0, 1);
    assert!(changed_indices.construction_evidence().buffers_changed());
    let raw = TriangleMesh::new(mesh.positions.to_vec(), mesh.triangles.to_vec());
    assert_eq!(raw.construction_evidence(), Evidence::UNTRACKED);
    assert!(
        raw.convex_hull(&GeometryContext::STRICT.mesh_context())
            .unwrap()
            .value
            .construction_evidence()
            .source_history_missing()
    );
    // Public new-claim construction is explicit; equality still compares geometry.
    assert_eq!(raw, mesh);
    assert_ne!(raw.construction_evidence(), mesh.construction_evidence());
}

#[test]
fn boolean_batch_and_empty_bridges_keep_both_parents() {
    let mesh = approximate_mesh();
    let cube = solid::cube(1.into()).unwrap();
    let batch = hypermesh::boolean(
        &GeometryContext::STRICT.mesh_context(),
        &[mesh.as_ref(), cube.as_ref()],
        BooleanProgram::Operation(BooleanOp::Union),
    )
    .unwrap();
    assert_eq!(batch.certainty, MeshCertainty::Certified);
    assert_eq!(
        batch.value.construction_evidence(),
        mesh.construction_evidence()
    );
    assert_approximate(&batch.value.clone().into_triangle_meshes().unwrap()[0]);
    let mut changed = batch.value.clone();
    Arc::make_mut(&mut changed.results)[0].triangles.swap(0, 1);
    let changed_evidence = changed.construction_evidence();
    assert!(
        changed_evidence.source_history_missing()
            && changed_evidence.approximate_512_consumed()
    );
    assert_eq!(
        changed.into_triangle_meshes().unwrap()[0].construction_evidence(),
        changed_evidence
    );
    let mut changed = batch.value;
    Arc::make_mut(&mut changed.vertices)[0] = mesh.positions[0].clone();
    assert!(changed.construction_evidence().buffers_changed());
    let empty = solid::boolean_with_context(
        &mesh,
        &mesh,
        BooleanOp::Difference,
        &GeometryContext::STRICT,
    )
    .unwrap();
    assert!(empty.value.positions.is_empty() && empty.value.triangles.is_empty());
    assert_eq!(empty.certainty, GeometryCertainty::Certified);
    assert_approximate(&empty.value);
    for (left, right) in [
        (&empty.value, &cube),
        (&cube, &empty.value),
        (&empty.value, &solid::empty()),
    ] {
        for operation in [
            BooleanOp::Union,
            BooleanOp::Intersection,
            BooleanOp::Difference,
        ] {
            let result =
                solid::boolean_with_context(left, right, operation, &GeometryContext::STRICT)
                    .unwrap();
            assert_eq!(result.certainty, GeometryCertainty::Certified);
            assert_approximate(&result.value);
        }
    }
}

#[test]
fn modeling_cache_keys_include_construction_qualifications() {
    use solid::cache::{CachePolicy, with_policy};
    let cube = solid::cube(1.into()).unwrap();
    let qualified = cube
        .clone()
        .with_inherited_construction(approximate_mesh().construction_evidence());
    assert!(Arc::ptr_eq(&cube.positions, &qualified.positions));
    assert!(Arc::ptr_eq(&cube.triangles, &qualified.triangles));
    for policy in [
        CachePolicy::DEFAULT,
        CachePolicy::DISABLED,
        CachePolicy {
            distributions: 1,
            merges: 1,
            ..CachePolicy::DEFAULT
        },
    ] {
        with_policy(policy, || {
            for context in [
                GeometryContext::STRICT,
                GeometryContext::APPROXIMATE_512,
                GeometryContext::STRICT,
            ] {
                for source in [&cube, &qualified, &cube, &qualified] {
                    for count in [0, 1, 2] {
                        let result = solid::distribute_linear_with_context(
                            source,
                            count,
                            Vector3::x(),
                            3.into(),
                            &context,
                        )
                        .unwrap();
                        assert_eq!(
                            result.value.construction_evidence(),
                            source.construction_evidence()
                        );
                        assert_eq!(result.certainty, GeometryCertainty::Certified);
                    }
                    let merged = solid::merge(&[source.clone(), solid::empty()]).unwrap();
                    assert_eq!(merged.construction_evidence(), source.construction_evidence());
                }
            }
        });
    }
}

#[test]
fn concurrent_local_proofs_do_not_upgrade_construction() {
    let mesh = approximate_mesh();
    std::thread::scope(|scope| {
        for _ in 0..4 {
            let mesh = mesh.clone();
            scope.spawn(move || {
                for _ in 0..8 {
                    let local = mesh
                        .validate_closed_pwn(&GeometryContext::STRICT.mesh_context())
                        .unwrap();
                    assert_eq!(local.certainty, MeshCertainty::Certified);
                    assert_approximate(&mesh);
                }
            });
        }
    });
    assert_approximate(&mesh);
}

#[cfg(feature = "curve")]
#[test]
fn bare_curve_history_is_untracked_and_sampling_is_separate() {
    let square = csgrs::curve::square(1.into()).unwrap();
    let mesh = csgrs::curve::try_extrude(&square, 1.into(), &GeometryContext::STRICT).unwrap();
    assert_eq!(mesh.certainty, GeometryCertainty::Certified);
    assert!(mesh.value.construction_evidence().source_history_missing());
    let sphere = solid::sphere(1.into(), 8, 4).unwrap();
    let evidence = sphere.construction_evidence();
    assert!(evidence.source_sampled());
    assert_eq!(evidence.predicate_certainty(), Some(MeshCertainty::Certified));
}

#[cfg(feature = "stl-io")]
#[path = "support/construction_evidence_pipeline.rs"]
mod pipeline;
