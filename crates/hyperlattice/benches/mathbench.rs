//! Criterion comparisons and optional untimed dispatch diagnostics for Hyperlattice.
//!
//! Textual includes share this crate's imports and private fixture/engine/report
//! helpers. The driver registers sixteen benchmark families spanning scalar,
//! complex, vector, matrix, borrowed-operator and representation workloads. Their
//! inputs and completion criteria belong to the individual rows; registration is
//! not proof of equivalent numerical contracts between comparison engines.
//!
//! Normal mode lets Criterion parse argv, initializes Symbolica, runs the family
//! registration functions, summarizes Criterion and attempts two report updates.
//! The exact --update-benchmarks-md flag takes precedence and attempts stored-data
//! updates without constructing Criterion or initializing Symbolica. Each writer
//! retains its own skip gate; the flag does not bypass those gates.
//!
//! Either --write-dispatch-trace-md or --dispatch-trace-only selects diagnostic
//! mode: Criterion's filter is set to the nonmatching pattern $^, while registration
//! functions and their fixture setup still run. Trace hooks execute selected
//! closures outside Criterion's measured loops. This mode still initializes
//! Symbolica and does not bound setup, allocation or scalar work. With tracing
//! disabled at compile time the hooks and trace writer are no-ops, but setup still
//! occurs. A trace filter controls hook names, not which families are initialized.
//!
//! Report writers can overwrite working-directory files or the crate's generated
//! benchmark section. HYPERLATTICE_SKIP_BENCHMARK_REPORTS suppresses these Markdown
//! updates; it does not suppress workload setup or Criterion's own output files.
//! No policy receipt or numerical correctness check is added by this driver.

use std::{cell::Cell, collections::BTreeMap, env, fs, hint::black_box, path::Path};

use criterion::{BatchSize, BenchmarkGroup, Criterion};
use hyperlattice::Rational;
use hyperlattice::{Complex, Matrix3, Matrix4, Real, SignedAxis4, Vector3, Vector4};
use symbolica::domains::float::RealLike;

/// Shared manifest/stored-result report writer used after the local mathbench writer.
///
/// Its gate requires an exact --bench argument and honors skip/test/list/help flags;
/// it targets the crate's benchmarks.md and may include stored rows from other runs.
#[path = "support/benchmark_report.rs"]
#[allow(dead_code)]
mod benchmark_report;

include!("mathbench/engines.rs");
include!("mathbench/fixtures.rs");
include!("mathbench/comparisons.rs");
include!("mathbench/scalar_ops.rs");
include!("mathbench/complex_ops.rs");
include!("mathbench/vector_ops.rs");
include!("mathbench/matrix_ops.rs");
include!("mathbench/borrowed_ops.rs");
include!("mathbench/representation_ops.rs");
include!("mathbench/dispatch_trace.rs");
include!("mathbench/report.rs");

/// Attempt optional Symbolica licensing, then touch its workspace before workloads.
///
/// A nonempty loaded key is passed to set_license_key; its result is ignored. If
/// no key is found, is_licensed is queried, but the unlicensed branch performs no
/// license request (the hobbyist example is a comment only). There is no retry or
/// typed initialization outcome. Workspace initialization and dependency failures
/// retain Symbolica's own behavior.
///
/// Main constructs Criterion first and selects the in-process Plotters backend;
/// this ordering is intended to avoid Criterion's plotting subprocesses interfering
/// with restricted Symbolica's process/worker setup. It is not a general guarantee
/// about other callers' thread pools or child processes.
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

/// Load a trimmed, nonempty optional key from the environment, then a local file.
///
/// Prefers Unicode SYMBOLICA_LICENSE_KEY; missing, non-Unicode or whitespace-only
/// values fall through to .symbolica-license in the process working directory.
/// File read/UTF-8 errors and empty trimmed contents become None. This helper does
/// not validate a key, request a license, print its contents or report read errors.
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

/// Select stored-report, diagnostic or Criterion mode, then invoke families in order.
///
/// The first trace-filter occurrence wins, accepting --trace-dispatch-filter=value
/// or a separate following token. A following flag can itself be consumed as that
/// token; a final bare flag yields no value. Enabled trace support splits the value
/// on commas, trims it and uses case-sensitive substring matches. The filter alone
/// does not enable tracing and is not stripped before normal Criterion parsing.
///
/// Stored-report mode precedes tracing and all engine setup. Diagnostic mode skips
/// Criterion's argv parsing, installs $^ to exclude measured rows, begins trace
/// collection and eventually attempts only the trace report. Other flags do not
/// automatically short-circuit that mode. Normal mode calls final_summary followed
/// by local and shared Markdown writers; all sixteen family functions run in their
/// listed order after Symbolica initialization, subject to earlier errors/exits.
/// This entry point neither catches workload panics nor validates numeric outputs.
fn main() {
    let args: Vec<String> = env::args().collect();
    // Dispatch trace can be targeted with this comma-separated pattern list so
    // we can measure narrow regression surfaces while avoiding a full trace
    // run during normal iteration.
    // Note: comma-delimited fragments are matched with substring checks by
    // design to keep filter overhead low and stable in benchmark-driver code.
    let trace_filter = args.iter().enumerate().find_map(|(index, arg)| {
        arg.strip_prefix("--trace-dispatch-filter=")
            .map(std::string::ToString::to_string)
            .or_else(|| {
                (arg == "--trace-dispatch-filter" && index + 1 < args.len())
                    .then(|| args[index + 1].clone())
            })
    });

    if args.iter().any(|arg| arg == "--update-benchmarks-md") {
        update_benchmarks_doc();
        update_complete_benchmark_report();
        return;
    }

    let trace_only = args
        .iter()
        .any(|arg| arg == "--write-dispatch-trace-md" || arg == "--dispatch-trace-only");
    if trace_only {
        begin_dispatch_trace_run(trace_filter.as_deref());
    }

    // Restricted Symbolica aborts as a second instance when any child process
    // is spawned while it holds its instance port. Criterion probes for
    // gnuplot when it is built and spawns it for every plot, so build it
    // before initializing Symbolica and plot in process.
    let criterion = Criterion::default().plotting_backend(criterion::PlottingBackend::Plotters);
    let mut criterion = if trace_only {
        criterion.with_filter("$^")
    } else {
        criterion.configure_from_args()
    };
    initialize_symbolica();
    bench_vectors(&mut criterion);
    bench_matrix3(&mut criterion);
    bench_matrix4(&mut criterion);
    bench_scalar_trig(&mut criterion);
    bench_scalar_operations(&mut criterion);
    bench_scalar_sqrt_cases(&mut criterion);
    bench_scalar_hyperbolic_cases(&mut criterion);
    bench_scalar_hyperbolic_f64_cases(&mut criterion);
    bench_large_integer_exp(&mut criterion);
    bench_complex_operations(&mut criterion);
    bench_complex_mul_cold(&mut criterion);
    bench_vector_operations(&mut criterion);
    bench_matrix_operations(&mut criterion);
    bench_targeted_matrix_forms(&mut criterion);
    bench_borrowed_operations(&mut criterion);
    bench_real_representations(&mut criterion);
    if trace_only {
        write_dispatch_trace_report();
    } else {
        criterion.final_summary();
        update_benchmarks_doc();
        update_complete_benchmark_report();
    }
}

/// Apply the shared writer's argv/environment gate and report its I/O outcome.
///
/// Success logs destination and stored-data counts, not newly measured coverage.
/// Errors log to stderr without failing the process. The shared gate can suppress
/// this call even when --update-benchmarks-md explicitly selected stored-report mode.
fn update_complete_benchmark_report() {
    if benchmark_report::reports_disabled() {
        return;
    }
    match benchmark_report::write_benchmarks_md() {
        Ok(summary) => eprintln!(
            "updated {} from {} Criterion rows, {} comparisons, and {} benchmark suites",
            summary.path.display(),
            summary.rows,
            summary.comparisons,
            summary.suites,
        ),
        Err(error) => eprintln!("failed to update complete benchmark report: {error}"),
    }
}
