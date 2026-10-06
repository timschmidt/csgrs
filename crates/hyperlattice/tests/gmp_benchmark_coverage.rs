//! Guards parity between the existing 128-bit Numerica benchmark surface and
//! the 128-bit GMP/MPFR comparison surface.

use std::{collections::BTreeSet, fs, path::Path};

const BENCHMARK_MODULES: &[&str] = &[
    "borrowed_ops.rs",
    "comparisons.rs",
    "complex_ops.rs",
    "matrix_ops.rs",
    "scalar_ops.rs",
    "vector_ops.rs",
];

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
