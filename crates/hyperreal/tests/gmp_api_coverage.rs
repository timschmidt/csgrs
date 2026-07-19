//! Guards the GMP/MPFR benchmark's public numeric API classification.
//!
//! A public function name must either occur in `benches/gmp_api.rs` or be
//! explicitly classified below as a Hyperreal representation/certification API
//! without a like-for-like GMP/MPFR operation. The audit is intentionally
//! name-based: overloads share one numeric operation and one classification.

use std::{collections::BTreeSet, fs, path::Path};

const NO_GMP_ANALOG: &[&str] = &[
    // Cancellation and formatting-framework hooks.
    "abort",
    "approx_signal",
    "approximate",
    "decimal",
    // Rational storage and display-policy introspection.
    "one_ref",
    "prefer_fraction",
    "storage_identity",
    // Hidden magnitude probes exist only for internal algorithm crossover
    // benchmarks; they are not part of the rational numeric API.
    "gcd_magnitudes",
    "gcd_magnitudes_half_gcd_candidate",
    "gcd_magnitudes_lehmer_baseline",
    "multiply_magnitudes_toom4_candidate",
    "multiply_magnitudes_toom6_candidate",
    "multiply_magnitudes_toom8_candidate",
    "multiply_magnitudes_ntt_candidate",
    "multiply_magnitudes_selected",
    "div_rem_magnitudes_backend_batch",
    "div_rem_magnitudes_barrett_batch_candidate",
    "div_rem_magnitudes_barrett_candidate",
    // Structural facts, retained exact values, and exact-set schedules.
    "best_sign",
    "definitely_not_equal",
    "definitely_one",
    "definitely_zero",
    "detailed_facts",
    "exact_rational",
    "exact_rational_complex_product_known_exact",
    "exact_rational_complex_quotient_known_exact",
    "exact_rational_matrix3_inverse_known_exact",
    "exact_rational_normalize_known_exact",
    "exact_rational_ref",
    "exact_rational_reuse_evidence",
    "exact_rational_signed_product_sum",
    "exact_rational_signed_product_sum2_known_exact",
    "exact_rational_signed_product_sum_known_exact",
    "exact_rational_signed_product_sum_known_shared_denominator",
    "exact_set_facts",
    "from_reals",
    "has_dyadic_schedule",
    "has_integer_grid_schedule",
    "has_shared_denominator_schedule",
    "has_signed_unit_schedule",
    "is_exact_dyadic_rational",
    "is_nonempty_exact_rational",
    "is_rational",
    "shared_denominator_kind",
    "sign_pattern",
    "structural_facts",
    "zero_one_or_minus_one",
    "zero_or_one",
    // Certified domain/evidence queries. MPFR reports a value or NaN but does
    // not expose Hyperreal's proof state, precision schedule, or refinement API.
    "acosh_domain",
    "asin_acos_domain",
    "atanh_domain",
    "certified_cmp_until",
    "certified_dyadic_interval",
    "certified_eq_until",
    "certified_sign_until",
    "domain_facts",
    "log_domain",
    "reciprocal_domain",
    "refine_sign_until",
    "sqrt_domain",
    // Certified predicates and prepared exact/filtered geometry queries.
    "certified_affine_det2_sign",
    "certified_affine_det3_sign",
    "certified_incircle2d_sign",
    "certified_insphere3d_sign",
    "certified_linear_form3_sign",
    "prepare_affine_det2_exact_word_filter",
    "prepare_affine_det2_filter",
    "prepare_affine_det3_exact_word_filter",
    "prepare_affine_det3_filter",
    "prepare_incircle2d_filter",
    "prepare_insphere3d_filter",
    "prepare_linear_form3_filter",
    "prepare_rational_affine_point3_query",
    "prepare_rational_line2_filter",
    "prepare_rational_linear_form4_filter",
    "prepare_rational_linear_form4_query",
    "sign_prepared",
    "sign_rational",
];

fn collect_rust_files(directory: &Path, files: &mut Vec<std::path::PathBuf>) {
    for entry in fs::read_dir(directory).expect("source directory must be readable") {
        let path = entry.expect("source entry must be readable").path();
        if path.is_dir() {
            collect_rust_files(&path, files);
        } else if path.extension().is_some_and(|extension| extension == "rs") {
            files.push(path);
        }
    }
}

fn public_function_names(source: &str) -> impl Iterator<Item = &str> {
    source.lines().filter_map(|line| {
        let declaration = line
            .split_once("pub const fn ")
            .map(|(_, declaration)| declaration)
            .or_else(|| {
                line.split_once("pub fn ")
                    .map(|(_, declaration)| declaration)
            })?;
        declaration
            .split(['(', '<'])
            .next()
            .map(str::trim)
            .filter(|name| !name.is_empty())
    })
}

fn contains_word(source: &str, word: &str) -> bool {
    source.match_indices(word).any(|(start, _)| {
        let before = source[..start].chars().next_back();
        let after = source[start + word.len()..].chars().next();
        !before.is_some_and(|character| character.is_ascii_alphanumeric() || character == '_')
            && !after.is_some_and(|character| character.is_ascii_alphanumeric() || character == '_')
    })
}

#[test]
fn every_public_numeric_api_is_benchmarked_or_explicitly_classified() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let benchmark = fs::read_to_string(root.join("benches/gmp_api.rs"))
        .expect("GMP benchmark source must exist");
    let excluded: BTreeSet<_> = NO_GMP_ANALOG.iter().copied().collect();

    let mut source_files = Vec::new();
    for directory in ["src/rational", "src/real", "src/computable"] {
        collect_rust_files(&root.join(directory), &mut source_files);
    }

    let mut public = BTreeSet::new();
    for path in source_files {
        let source = fs::read_to_string(path).expect("Rust source must be readable");
        public.extend(public_function_names(&source).map(str::to_owned));
    }

    let unclassified: Vec<_> = public
        .iter()
        .filter(|name| !excluded.contains(name.as_str()) && !contains_word(&benchmark, name))
        .cloned()
        .collect();
    assert!(
        unclassified.is_empty(),
        "public numeric APIs missing a GMP benchmark or explicit no-analog classification: {unclassified:?}"
    );

    let stale_exclusions: Vec<_> = excluded
        .iter()
        .filter(|name| !public.contains(**name))
        .copied()
        .collect();
    assert!(
        stale_exclusions.is_empty(),
        "stale GMP no-analog classifications: {stale_exclusions:?}"
    );
}
