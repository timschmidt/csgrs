#![no_main]

use hypermesh::exact::{ExactMesh, ValidationPolicy};
use hypervoxel::{
    HypermeshTriangleSolidAdapterBlocker, PreparedExactTriangleSolidMesh,
    adapt_hypermesh_exact_solid,
};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: (u8, bool, bool)| {
    let (scale_raw, exact_source, closed_policy) = data;
    let scale = i64::from((scale_raw % 5) + 1);
    let idx = [0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3];
    let mesh = if exact_source {
        ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 0, 0, //
                scale, 0, 0, //
                0, scale, 0, //
                0, 0, scale,
            ],
            &idx,
            if closed_policy {
                ValidationPolicy::CLOSED
            } else {
                ValidationPolicy::ALLOW_BOUNDARY
            },
        )
    } else {
        ExactMesh::from_f64_triangles_with_policy(
            &[
                0.0,
                0.0,
                0.0,
                scale as f64,
                0.0,
                0.0,
                0.0,
                scale as f64,
                0.0,
                0.0,
                0.0,
                scale as f64,
            ],
            &idx,
            if closed_policy {
                ValidationPolicy::CLOSED
            } else {
                ValidationPolicy::ALLOW_BOUNDARY
            },
        )
    }
    .unwrap();

    let adapter = adapt_hypermesh_exact_solid(&mesh, None, None).unwrap();
    if exact_source && closed_policy {
        assert!(adapter.report.exact_triangle_solid_ready);
        let prepared = PreparedExactTriangleSolidMesh::prepare(adapter.solid.unwrap()).unwrap();
        assert!(prepared.report().exact_prepared_solid_ready);
    } else {
        assert!(!adapter.report.exact_triangle_solid_ready);
        assert!(adapter.solid.is_none());
        if !exact_source && closed_policy {
            assert!(
                adapter
                    .report
                    .blockers
                    .contains(&HypermeshTriangleSolidAdapterBlocker::SourceNotExact)
            );
        }
        if !closed_policy {
            assert!(
                adapter
                    .report
                    .blockers
                    .contains(&HypermeshTriangleSolidAdapterBlocker::SolidHandoffNotReady)
            );
        }
    }
});
