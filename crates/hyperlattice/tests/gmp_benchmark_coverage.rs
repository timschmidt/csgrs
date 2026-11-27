//! Source-text inventory checks for the Numerica-labeled and GMP/MPFR benchmarks.
//!
//! The Numerica label names the SymbolicaFloat adapter. These checks compare
//! naming patterns in selected source files and require report-generator labels;
//! they neither compile nor execute the adapters or regenerate a report. Passing
//! does not establish equal inputs, signatures, arithmetic, cancellation behavior,
//! output materialization, or completed workloads across engines.

use std::{collections::BTreeSet, fs, path::Path};

/// Six benchmark source files scanned relative to this crate's benches/mathbench directory.
///
/// This explicit list excludes representation_ops and other benchmark binaries.
/// Adapter declaration order and report labels have separate checks below.
const BENCHMARK_MODULES: &[&str] = &[
    "borrowed_ops.rs",
    "comparisons.rs",
    "complex_ops.rs",
    "matrix_ops.rs",
    "scalar_ops.rs",
    "vector_ops.rs",
];

/// Collect sorted, distinct ASCII identifier-character runs following raw prefix matches.
///
/// Scans nonoverlapping substring matches anywhere, including comments, strings,
/// calls and declarations. A run can be empty or start with a digit. A match is
/// omitted if its remaining suffix has no terminating non-identifier character
/// before EOF. Does not parse Rust or count declarations or benchmark registrations.
fn function_suffixes(source: &str, prefix: &str) -> BTreeSet<String> {
    source
        .match_indices(prefix)
        .filter_map(|(start, _)| {
            let suffix = &source[start + prefix.len()..];
            let end = suffix
                .find(|character: char| !character.is_ascii_alphanumeric() && character != '_')?;
            Some(suffix[..end].to_owned())
        })
        .collect()
}

/// Extract a flattened, source-ordered sequence of names from literal public-function text.
///
/// On each line, takes the text after the first `pub fn ` and before the first
/// opening parenthesis or angle bracket, trimming outer whitespace. Lines without
/// both delimiters are skipped; duplicates and matches in comments/strings remain.
/// Does not associate names with types, inspect signatures, or recognize alternate
/// visibility and modifier spellings such as `pub(crate) fn` or `pub async fn`.
fn public_function_sequence(source: &str) -> Vec<String> {
    source
        .lines()
        .filter_map(|line| {
            let declaration = line.split_once("pub fn ")?.1;
            let end = declaration.find(['(', '<'])?;
            Some(declaration[..end].trim().to_owned())
        })
        .collect()
}

/// Require matching registrar suffix sets and implementation-label prefix counts in six files.
///
/// Reads sources under the compile-time crate root; unreadable files panic. The
/// suffix sets discard multiplicity, while label counts are raw substring counts
/// starting at a double quote, without requiring a closing quote. Neither check
/// pairs individual rows or proves workload equivalence; comments can affect both.
#[test]
fn every_numerica_benchmark_family_has_gmp_parity() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("benches/mathbench");
    for file in BENCHMARK_MODULES {
        let source =
            fs::read_to_string(root.join(file)).expect("benchmark module must be readable");
        assert_eq!(
            function_suffixes(&source, "bench_numerica_"),
            function_suffixes(&source, "bench_gmp_"),
            "{file} has mismatched Numerica and GMP benchmark functions"
        );
        assert_eq!(
            source.matches("\"numerica128").count(),
            source.matches("\"gmp_mpfr128").count(),
            "{file} has mismatched literal Numerica and GMP benchmark rows"
        );
    }
}

/// Require the two adapter source slices to have identical public-function name sequences.
///
/// Uses the first literal module markers in the engine source, assuming Numerica,
/// GMP, then Symbolica order. Read failures, missing markers or invalid slice order
/// panic. Names are flattened across all types: matching sequences do not certify
/// equal owners, signatures, implementations or supported mathematical domains.
#[test]
fn gmp_engine_exposes_the_complete_numerica_adapter_surface() {
    let source = fs::read_to_string(
        Path::new(env!("CARGO_MANIFEST_DIR")).join("benches/mathbench/engines.rs"),
    )
    .expect("engine adapters must be readable");
    let numerica_start = source
        .find("mod numerica_engine {")
        .expect("Numerica engine module must exist");
    let gmp_start = source
        .find("mod gmp_engine {")
        .expect("GMP engine module must exist");
    let symbolica_start = source
        .find("mod symbolica_engine {")
        .expect("Symbolica engine module must exist");

    assert_eq!(
        public_function_sequence(&source[numerica_start..gmp_start]),
        public_function_sequence(&source[gmp_start..symbolica_start]),
        "GMP adapter must expose every operation available through the Numerica adapter"
    );
}

/// Require three GMP-related label substrings in the curated report generator's source.
///
/// Reads report.rs, not generated Markdown; unreadable source panics. The raw
/// substring checks accept occurrences anywhere, including comments. They do not
/// exercise rendering, check the ratio formula, or establish stored-data coverage.
#[test]
fn generated_report_includes_gmp_column_and_ratio() {
    let report = fs::read_to_string(
        Path::new(env!("CARGO_MANIFEST_DIR")).join("benches/mathbench/report.rs"),
    )
    .expect("report generator must be readable");
    assert!(report.contains("GMP/MPFR 128"));
    assert!(report.contains("Exact dyadic / GMP"));
    assert!(report.contains("\"gmp_mpfr128\""));
}
