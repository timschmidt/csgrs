use csgrs::adapter::{AdapterError, F32, F64, I128, RawReal, ScalarAdapter, ScalarMesh};
#[path = "support/uncertainty.rs"]
mod uncertainty;

use csgrs::errors::{ValidationError, ValidationErrorKind};
use csgrs::{GeometryContext, Real, TriangleMesh, solid};
use hyperlattice::{Matrix4, Point3};
use hypermesh::{BooleanMeshResult, GpuMeshError, Triangle};
use hyperreal::Problem;
use std::error::Error;

fn unresolved_zero() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    uncertainty::terminal_zero(&sine * &sine + &cosine * &cosine - Real::one())
}

fn triangle(height: Real) -> TriangleMesh {
    TriangleMesh::new(
        vec![
            Point3::origin(),
            Point3::new(1.into(), 0.into(), 0.into()),
            Point3::new(0.into(), height, 0.into()),
        ],
        vec![Triangle::new(0, 1, 2)],
    )
}

fn assert_validation(error: AdapterError, native: ValidationError, kind: ValidationErrorKind) {
    assert_eq!(native.kind(), kind);
    assert_eq!(error.to_string(), native.to_string());
    assert_eq!(error, AdapterError::Construction(native.clone()));
    assert_eq!(
        error.source().unwrap().downcast_ref::<ValidationError>(),
        Some(&native)
    );
}

fn polyhedron_failure<A: ScalarAdapter>() {
    let native_points =
        [[0, 0, 0], [3, 3, 0], [0, 2, 0], [2, 0, 0]].map(|row| row.map(Real::from));
    let points = native_points
        .clone()
        .map(|row| row.map(|value| A::from_real(&value).unwrap()));
    let faces: &[&[usize]] = &[&[0, 1, 2, 3]];
    assert_validation(
        ScalarMesh::<A>::polyhedron(&points, faces).err().unwrap(),
        solid::polyhedron(&native_points, faces).unwrap_err(),
        ValidationErrorKind::InvalidInput,
    );
    for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
        assert_validation(
            ScalarMesh::<A>::polyhedron_with_context(&points, faces, &context)
                .err()
                .unwrap(),
            solid::polyhedron_with_context(&native_points, faces, &context).unwrap_err(),
            ValidationErrorKind::InvalidInput,
        );
    }
}

#[test]
fn polyhedron_failures_keep_native_causes_and_policy() {
    polyhedron_failure::<RawReal>();
    polyhedron_failure::<F32>();
    polyhedron_failure::<F64>();
    polyhedron_failure::<I128>();
    let points = [
        [0, 0, 0].map(Real::from),
        [1, 0, 0].map(Real::from),
        [2.into(), unresolved_zero(), 0.into()],
    ];
    let faces: &[&[usize]] = &[&[0, 1, 2]];
    assert_validation(
        ScalarMesh::<RawReal>::polyhedron(&points, faces)
            .err()
            .unwrap(),
        solid::polyhedron(&points, faces).unwrap_err(),
        ValidationErrorKind::Undecided,
    );
    for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
        let native = solid::polyhedron_with_context(&points, faces, &context).unwrap_err();
        let kind = native.kind();
        assert_validation(
            ScalarMesh::<RawReal>::polyhedron_with_context(&points, faces, &context)
                .err()
                .unwrap(),
            native,
            kind,
        );
    }
}

fn point_failures<A: ScalarAdapter>() {
    let point = [1, 2, 3].map(|v| A::from_real(&v.into()).unwrap());
    for (w, expected) in [
        (Real::zero(), Problem::DivideByZero),
        (unresolved_zero(), Problem::UnknownZero),
    ] {
        let matrix = Matrix4::new([
            [1.into(), 0.into(), 0.into(), 0.into()],
            [0.into(), 1.into(), 0.into(), 0.into()],
            [0.into(), 0.into(), 1.into(), 0.into()],
            [0.into(), 0.into(), 0.into(), w],
        ]);
        assert_eq!(
            matrix.transform_point3(&Point3::new(1.into(), 2.into(), 3.into())),
            Err(expected)
        );
        let error =
            ScalarMesh::<A>::transform_adapter_points(std::slice::from_ref(&point), &matrix)
                .unwrap_err();
        assert_eq!(error, AdapterError::Hyperreal(expected));
        assert_eq!(
            error.source().unwrap().downcast_ref::<Problem>(),
            Some(&expected)
        );
    }
    let matrix = Matrix4::new([
        [1.into(), 0.into(), 0.into(), 4.into()],
        [0.into(), 1.into(), 0.into(), 5.into()],
        [0.into(), 0.into(), 1.into(), 6.into()],
        [0.into(), 0.into(), 0.into(), 1.into()],
    ]);
    assert_eq!(
        ScalarMesh::<A>::transform_adapter_points(&[point], &matrix).unwrap(),
        vec![[5, 7, 9].map(|v| A::from_real(&v.into()).unwrap())]
    );
}

#[test]
fn point_transforms_keep_scalar_domain_and_undecided_causes() {
    point_failures::<RawReal>();
    point_failures::<F32>();
    point_failures::<F64>();
    point_failures::<I128>();
}

fn graphics_failures<A: ScalarAdapter>() {
    for (mesh, kind) in [
        (
            TriangleMesh::new(vec![], vec![Triangle::new(0, 1, 2)]),
            ValidationErrorKind::InvalidInput,
        ),
        (triangle(Real::zero()), ValidationErrorKind::InvalidInput),
        (triangle(unresolved_zero()), ValidationErrorKind::Undecided),
    ] {
        let native = mesh.to_exact_gpu_mesh_buffers().unwrap_err();
        let adapter = ScalarMesh::<A>::from_native(mesh);
        for _ in 0..2 {
            let error = adapter.graphics_mesh().err().unwrap();
            assert_eq!(error.to_string(), native.to_string());
            let cause = error.source().expect("graphics source survives adapter");
            let validation = cause.downcast_ref::<ValidationError>().unwrap();
            assert_eq!(validation.kind(), kind);
            assert_eq!(
                cause.source().unwrap().downcast_ref::<GpuMeshError>(),
                Some(&native)
            );
        }
    }
    let empty = ScalarMesh::<A>::from_native(TriangleMesh::new(vec![], vec![]));
    for _ in 0..2 {
        let graphics = empty.graphics_mesh().unwrap();
        assert!(graphics.vertices.is_empty() && graphics.indices.is_empty());
    }
    let good = ScalarMesh::<A>::from_native(triangle(2.into()));
    let expected = [0, 0, 1].map(|v| A::from_real(&v.into()).unwrap());
    for _ in 0..2 {
        let graphics = good.graphics_mesh().unwrap();
        assert_eq!(&*graphics.indices, [0, 1, 2]);
        for (_, normal) in graphics.vertices.iter() {
            assert_eq!(normal, &expected);
        }
    }
}

#[test]
fn graphics_failures_keep_native_categories_in_every_scalar_family() {
    graphics_failures::<RawReal>();
    graphics_failures::<F32>();
    graphics_failures::<F64>();
    graphics_failures::<I128>();
}

#[test]
fn native_export_distinguishes_zero_from_unproved_zero_in_both_carriers() {
    let degenerate = triangle(Real::zero());
    let undecided = triangle(unresolved_zero());
    let zero = degenerate.to_exact_gpu_mesh_buffers().unwrap_err();
    let unknown = undecided.to_exact_gpu_mesh_buffers().unwrap_err();
    assert_ne!(zero, unknown, "lack of a proof is not degeneracy");
    let result = BooleanMeshResult {
        triangles: vec![[0, 1, 2]],
        sources: vec![],
        exterior_inside: false,
    };
    assert_eq!(
        result
            .to_exact_gpu_mesh_buffers(&degenerate.positions)
            .unwrap_err(),
        zero
    );
    assert_eq!(
        result
            .to_exact_gpu_mesh_buffers(&undecided.positions)
            .unwrap_err(),
        unknown
    );
}

#[test]
fn exact_normal_values_and_integer_projection_contract_survive() {
    let mesh = TriangleMesh::new(
        vec![
            Point3::origin(),
            Point3::new(1.into(), 0.into(), 0.into()),
            Point3::new(0.into(), 3.into(), 4.into()),
        ],
        vec![Triangle::new(0, 1, 2)],
    );
    let graphics = ScalarMesh::<RawReal>::from_native(mesh.clone())
        .graphics_mesh()
        .unwrap();
    let expected = [
        Real::zero(),
        (Real::from(-4) / Real::from(5)).unwrap(),
        (Real::from(3) / Real::from(5)).unwrap(),
    ];
    assert_eq!(&*graphics.indices, [0, 1, 2]);
    for (index, (position, normal)) in graphics.vertices.iter().enumerate() {
        let point = &mesh.positions[index];
        assert_eq!(position, &[point.x.clone(), point.y.clone(), point.z.clone()]);
        assert_eq!(normal, &expected);
    }
    assert_eq!(
        ScalarMesh::<I128>::from_native(mesh)
            .graphics_mesh()
            .unwrap_err(),
        AdapterError::Hyperreal(Problem::NotAnInteger)
    );
}

#[test]
#[cfg(feature = "attributed")]
fn stronger_proof_recovers_attributed_exports_and_shared_metadata_cache() {
    use csgrs::AttributedMesh;
    use hyperreal::RealSign;
    let pi = Real::pi();
    let lower = pi.certified_rational_interval(-1024).unwrap()[0].clone();
    let width = &pi - Real::from(lower);
    let mesh = triangle(width.clone());
    let attributed = AttributedMesh::from_uniform(mesh.clone(), "source");
    let mapped = attributed.clone().map_metadata(|_| "mapped");
    // A prior strict proof may already permit cold success. A cold failure may
    // not survive a stronger proof that makes the same native mesh consumable.
    let _ = attributed.exact_gpu_mesh_buffers();
    assert_eq!(
        width.certified_sign_until(-1100).sign(),
        Some(RealSign::Positive)
    );
    let expected = mesh.to_exact_gpu_mesh_buffers().unwrap();
    assert_eq!(attributed.exact_gpu_mesh_buffers().unwrap(), &expected);
    assert_eq!(mapped.exact_gpu_mesh_buffers().unwrap(), &expected);
    assert!(std::ptr::eq(
        attributed.exact_gpu_mesh_buffers().unwrap(),
        mapped.exact_gpu_mesh_buffers().unwrap()
    ));
    assert_eq!(attributed.geometry(), &mesh);
}

#[test]
fn graphics_cache_keeps_source_and_returned_buffer_mutations_isolated() {
    use std::sync::Arc;
    let mut mesh = triangle(2.into());
    let original = ScalarMesh::<RawReal>::from_native(mesh.clone());
    let expected = original.graphics_mesh().unwrap();
    let mut returned = expected.clone();
    Arc::make_mut(&mut returned.vertices)[0].1 = [9, 9, 9].map(Real::from);
    assert_eq!(original.graphics_mesh().unwrap(), expected);

    Arc::make_mut(&mut mesh.positions)[2] = Point3::new(0.into(), 3.into(), 4.into());
    let changed = ScalarMesh::<RawReal>::from_native(mesh.clone())
        .graphics_mesh()
        .unwrap();
    assert_eq!(changed.vertices[2].0, [0, 3, 4].map(Real::from));
    assert_eq!(
        changed.vertices[0].1,
        [
            Real::zero(),
            (Real::from(-4) / Real::from(5)).unwrap(),
            (Real::from(3) / Real::from(5)).unwrap()
        ]
    );
    Arc::make_mut(&mut mesh.triangles)[0] = Triangle::new(0, 1, 7);
    assert!(matches!(
        ScalarMesh::<RawReal>::from_native(mesh).graphics_mesh(),
        Err(AdapterError::Construction(ValidationError::Gpu(
            GpuMeshError::SourceTriangleIndexOutOfBounds { index: 7, .. }
        )))
    ));
    // The scalar adapter cache retains at most eight distinct source identities.
    for height in 10..19 {
        ScalarMesh::<RawReal>::from_native(triangle(height.into()))
            .graphics_mesh()
            .unwrap();
    }
    assert_eq!(original.graphics_mesh().unwrap(), expected);
}

#[test]
#[cfg(feature = "attributed")]
fn attributed_cache_shares_concurrent_success_without_stale_geometry() {
    use csgrs::AttributedMesh;
    use std::sync::{Arc, Barrier};
    let mut geometry = triangle(2.into());
    let sidecar = AttributedMesh::from_uniform(geometry.clone(), ());
    let barrier = Barrier::new(4);
    let addresses = std::thread::scope(|scope| {
        (0..4)
            .map(|_| {
                scope.spawn(|| {
                    barrier.wait();
                    let rows = sidecar.exact_gpu_mesh_buffers().unwrap();
                    assert_eq!(rows.vertices[2].0, [0, 2, 0].map(Real::from));
                    assert_eq!(rows.vertices[0].1, [0, 0, 1].map(Real::from));
                    std::ptr::from_ref(rows) as usize
                })
            })
            .collect::<Vec<_>>()
            .into_iter()
            .map(|handle| handle.join().unwrap())
            .collect::<Vec<_>>()
    });
    assert!(addresses.iter().all(|address| address == &addresses[0]));
    Arc::make_mut(&mut geometry.positions)[2].y = 7.into();
    let changed = AttributedMesh::from_uniform(geometry, ());
    assert_eq!(
        changed.exact_gpu_mesh_buffers().unwrap().vertices[2].0,
        [0, 7, 0].map(Real::from)
    );
    assert_eq!(
        sidecar.exact_gpu_mesh_buffers().unwrap().vertices[2].0,
        [0, 2, 0].map(Real::from)
    );
}

#[test]
#[cfg(feature = "attributed")]
fn authored_normal_rows_remain_explicit_boundary_attributes() {
    use csgrs::AttributedMesh;
    use hyperlattice::Vector3;
    let geometry = triangle(Real::zero());
    let normal = Vector3::from_xyz(2.into(), 3.into(), 4.into());
    let sidecar = AttributedMesh::with_authored_normals(
        geometry.clone(),
        vec![()],
        Some(vec![normal; 3]),
    )
    .unwrap();
    for _ in 0..2 {
        let rows = sidecar.exact_gpu_mesh_buffers().unwrap();
        for (_, normal) in &rows.vertices {
            assert_eq!(normal, &[2, 3, 4].map(Real::from));
        }
    }
    assert_eq!(sidecar.geometry(), &geometry);
    assert!(matches!(
        sidecar.geometry().to_exact_gpu_mesh_buffers(),
        Err(GpuMeshError::TriangleNormalUnavailable { .. })
    ));
}
