use std::{cell::Cell, collections::BTreeMap, env, fs, hint::black_box, path::Path};

use criterion::{BatchSize, BenchmarkGroup, BenchmarkId, Criterion};
use realistic_blas::Rational;
use realistic_blas::{
    ApproxBackend, Backend, Complex, HyperrealBackend, Matrix3, Matrix4, Scalar, Vector3, Vector4,
};

type HyperrealScalar = Scalar<HyperrealBackend>;

include!("mathbench/backends.rs");
include!("mathbench/fixtures.rs");
include!("mathbench/comparisons.rs");
include!("mathbench/scalar_ops.rs");
include!("mathbench/complex_ops.rs");
include!("mathbench/vector_ops.rs");
include!("mathbench/matrix_ops.rs");
include!("mathbench/borrowed_ops.rs");
include!("mathbench/precision.rs");
include!("mathbench/dispatch_trace.rs");
include!("mathbench/report.rs");

fn initialize_symbolica() {
    if let Some(key) = load_symbolica_license_key() {
        let _ = symbolica::LicenseManager::set_license_key(&key);
    } else if !symbolica::LicenseManager::is_licensed() {
        // Example fallback for users who want the bench to request a hobbyist key:
        // let _ = symbolica::LicenseManager::request_hobbyist_license("YOUR_NAME", "YOUR_EMAIL");
    }

    // In restricted mode, Symbolica builds Rayon’s global pool the first time
    // its workspace is touched. Do that here on the benchmark thread before
    // Criterion or another dependency initializes Rayon first.
    let _ = symbolica::state::Workspace::get_local();
}

fn load_symbolica_license_key() -> Option<String> {
    env::var("SYMBOLICA_LICENSE_KEY")
        .ok()
        .map(|key| key.trim().to_owned())
        .filter(|key| !key.is_empty())
        .or_else(|| {
            fs::read_to_string(".symbolica-license")
                .ok()
                .map(|key| key.trim().to_owned())
                .filter(|key| !key.is_empty())
        })
}

fn main() {
    let args: Vec<String> = env::args().collect();
    // Dispatch trace can be targeted with this comma-separated pattern list so
    // we can measure narrow regression surfaces while avoiding a full trace
    // run during normal iteration.
    // Note: comma-delimited fragments are matched with substring checks by
    // design to keep filter overhead low and stable in benchmark-driver code.
    let trace_filter = args
        .iter()
        .enumerate()
        .find_map(|(index, arg)| {
            arg.strip_prefix("--trace-dispatch-filter=")
                .map(std::string::ToString::to_string)
                .or_else(|| {
                    (arg == "--trace-dispatch-filter" && index + 1 < args.len())
                        .then(|| args[index + 1].clone())
                })
        });

    if args.iter().any(|arg| arg == "--update-benchmarks-md") {
        update_benchmarks_doc();
        return;
    }

    initialize_symbolica();

    let trace_only = args
        .iter()
        .any(|arg| arg == "--write-dispatch-trace-md" || arg == "--dispatch-trace-only");
    if trace_only {
        begin_dispatch_trace_run(trace_filter.as_deref());
    }

    let mut criterion = if trace_only {
        Criterion::default().with_filter("$^")
    } else {
        Criterion::default().configure_from_args()
    };
    bench_vectors(&mut criterion);
    bench_matrix3(&mut criterion);
    bench_matrix4(&mut criterion);
    bench_scalar_trig(&mut criterion);
    bench_scalar_operations(&mut criterion);
    bench_complex_operations(&mut criterion);
    bench_vector_operations(&mut criterion);
    bench_matrix_operations(&mut criterion);
    bench_targeted_matrix_forms(&mut criterion);
    bench_borrowed_operations(&mut criterion);
    bench_precisions(&mut criterion);
    if trace_only {
        write_dispatch_trace_report();
    } else {
        criterion.final_summary();
        update_benchmarks_doc();
    }
}
