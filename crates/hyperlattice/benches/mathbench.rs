use std::{cell::Cell, collections::BTreeMap, fs, hint::black_box, path::Path};

use criterion::{BenchmarkGroup, BenchmarkId, Criterion};
use realistic_blas::Rational;
use realistic_blas::{
    ApproxBackend, Backend, Complex, Matrix3, Matrix4, RealisticBackend, Scalar, Vector3, Vector4,
};

type RealisticScalar = Scalar<RealisticBackend>;

include!("mathbench/backends.rs");
include!("mathbench/fixtures.rs");
include!("mathbench/comparisons.rs");
include!("mathbench/scalar_ops.rs");
include!("mathbench/complex_ops.rs");
include!("mathbench/vector_ops.rs");
include!("mathbench/matrix_ops.rs");
include!("mathbench/borrowed_ops.rs");
include!("mathbench/precision.rs");
include!("mathbench/report.rs");

fn main() {
    if std::env::args().any(|arg| arg == "--update-benchmarks-md") {
        update_benchmarks_doc();
        return;
    }

    let mut criterion = Criterion::default().configure_from_args();
    bench_vectors(&mut criterion);
    bench_matrix3(&mut criterion);
    bench_matrix4(&mut criterion);
    bench_scalar_trig(&mut criterion);
    bench_scalar_operations(&mut criterion);
    bench_complex_operations(&mut criterion);
    bench_vector_operations(&mut criterion);
    bench_matrix_operations(&mut criterion);
    bench_borrowed_operations(&mut criterion);
    bench_precisions(&mut criterion);
    criterion.final_summary();
    update_benchmarks_doc();
}
