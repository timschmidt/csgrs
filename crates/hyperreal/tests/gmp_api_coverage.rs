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
    "affine_plane3_coefficients_known_dyadic",
    "best_sign",
    "checked_exact_integer_cross_difference_quotient",
    "checked_exact_integer_scaled_difference",
    "checked_exact_integer_quotient",
    "clear_common_denominator_slice",
    "dyadic_difference_numerator_magnitude",
    "definitely_not_equal",
    "definitely_one",
    "definitely_zero",
    "detailed_facts",
    "exact_rational",
    "exact_rational_normal_form",
    "exact_rational_affine_det2_word_sign",
    "exact_rational_affine_det3_word_sign",
    "certified_rational_linear_form4_sign",
    "exact_rational_complex_product_known_exact",
    "exact_rational_complex_quotient_known_exact",
    "exact_rational_det3_word_sign",
    "exact_rational_matrix3_inverse_known_exact",
    "exact_rational_matrix3_inverse_known_dyadic",
    "exact_rational_matrix4_inverse_known_exact",
    "exact_rational_matrix4_inverse_known_dyadic",
    "exact_rational_normalize_known_exact",
    "exact_rational_ref",
    "exact_rational_reuse_evidence",
    "exact_rational_sparse_homogeneous_plane_intersection3",
    "exact_rational_signed_product_sum",
    "exact_rational_signed_product_sum2_known_exact",
    "exact_rational_signed_product_sum_known_exact",
    "exact_rational_signed_product_sum_known_dyadic",
    "exact_rational_signed_product_sum_known_shared_denominator",
    "exact_set_facts",
    "from_f64",
    "from_affine_point3",
    "from_rationals",
    "from_reals",
    "has_dyadic_schedule",
    "has_integer_grid_schedule",
    "has_shared_denominator_schedule",
    "has_signed_unit_schedule",
    "is_exact_dyadic_rational",
    "is_nonempty_exact_rational",
    "is_rational",
    "numerator_magnitude_gcd",
    "primitive_bigint_ratio",
    "primitive_integer_ratio",
    "shared_denominator_kind",
    "sign_pattern",
    "signed_product_sum2_ordering_slice",
    "signed_product_sum_known_dyadic",
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
    "inverse_ref_assuming_nonzero",
    "log_domain",
    "reciprocal_domain",
    "refine_sign_until",
    "sqrt_domain",
    "try_atan2",
    "try_atan2_until",
    "try_compare_to_until",
    // Certified predicates and retained exact/filtered geometry queries.
    "certified_affine_det2_sign",
    "certified_affine_det2_sign_exact_dyadic_f64",
    "certified_affine_det3_sign",
    "certified_enclosure",
    "certified_incircle2_sign",
    "certified_insphere3_sign",
    "certified_linear_form3_sign",
    "certified_rational_line2_sign",
    "compare_first_parameter",
    "compare_first_parameter_normalized",
    "compare_first_parameter_to_compact",
    "compare_second_parameter",
    "compare_second_parameter_normalized",
    "compare_second_parameter_to_compact",
    "exact_rational_dominant_affine_cross_axis",
    "exact_rational_line_intersection2_point_known_dyadic",
    "exact_rational_line_intersection2_point_known_dyadic_wide",
    "first_signs",
    "from_certified_enclosures",
    "materialize_first_parameter",
    "materialize",
    "materialize_second_parameter",
    "normalized_coefficients",
    "intersection_point",
    "intersection_point_f64",
    "from_point3",
    "second_signs",
    "sign_reals",
    "signs_exact_dyadic_f64",
    "sign_point3",
    "sign_point3_pair",
    "sign_rationals",
    "retained_intersection_point_f64",
    "wide_intersection_point",
    "wide_intersection_point_f64",
    "wide_retained_intersection_point_f64",
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
