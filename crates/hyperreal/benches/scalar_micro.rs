//! Measure fixed scalar construction, query, aggregate and backend workloads.
//!
//! The eleven groups exercise exact rationals, symbolic Real constructions, explicit
//! lossy binary64 exports, warmed Computable approximations and selected certified
//! cotangent intervals. These are different output contracts: constructing an exact
//! expression does not demand its numeric evaluation. Binary64 fixtures enter with
//! their exact binary values, not their displayed decimal fractions.
//!
//! Ordinary iteration includes in-loop clones, construction and output destruction.
//! Batched setup and returned-output destruction are outside the measured routine;
//! consumed-input cleanup can remain inside it. Cloning prepared values can retain
//! proofs and caches, and even fresh wrappers do not reset global state. Cold/retained
//! and algorithm labels describe the intended fixture, not a dispatch-count test.
//!
//! Most results are only black-boxed. Backend comparison/equality assertions run in
//! setup, including large inputs even when a Criterion row filter excludes them.
//! Some candidate rows reuse differently seeded baseline rows, so their names do
//! not guarantee identical operands. Precision, fixed iteration sizes and limb labels
//! are not total-work or memory limits. Unwraps/assertions panic on unmet assumptions.
//!
//! The static catalogue omits some registered rows and contains unchecked path
//! claims. Its writer runs before cache rows and again in the final report callback,
//! subject to the shared write guards. Criterion generates benches and main, but
//! these generated functions have no attached Rustdoc; this overview does not close
//! those item-level coverage gaps.

use criterion::{BatchSize, Criterion, black_box, criterion_group, criterion_main};
use hyperreal::{Computable, Rational, Real};
use num::Integer as _;
use num::bigint::{BigInt, BigUint};

#[path = "support/bench_docs.rs"]
/// Render the authored scalar benchmark catalogue using the shared report/write guards.
mod bench_docs;
#[path = "support/benchmark_report.rs"]
/// Aggregate Criterion estimates through the shared report finalizer and its write guards.
mod benchmark_report;

use bench_docs::{BenchDoc, BenchGroupDoc};

/// Authored descriptions used to render eleven groups, not to register or validate them.
///
/// The catalogue omits the ten detailed_facts and ten to_f64_lossy structural rows,
/// and sqrt_scaled_exp_squarefree. Cache wording mentions cold requests although
/// all six raw-cache rows prewarm at -256. Algorithm and same-operand claims are
/// not checked by this metadata; see the dispatch group's actual comparison scope.
const SCALAR_MICRO_GROUPS: &[BenchGroupDoc] = &[
    BenchGroupDoc {
        name: "construction_speed",
        description: "Cost of constructing common exact scalar identities and small integers.",
        benches: &[
            BenchDoc {
                name: "rational_one",
                description: "Constructs `Rational::one()`.",
            },
            BenchDoc {
                name: "rational_new_one",
                description: "Constructs one through `Rational::new(1)`.",
            },
            BenchDoc {
                name: "rational_from_u8_four",
                description: "Constructs positive four through unsigned primitive conversion.",
            },
            BenchDoc {
                name: "rational_from_i8_minus_four",
                description: "Constructs negative four through signed primitive conversion.",
            },
            BenchDoc {
                name: "computable_one",
                description: "Constructs `Computable::one()`.",
            },
            BenchDoc {
                name: "real_new_rational_one",
                description: "Constructs one through `Real::new(Rational::one())`.",
            },
            BenchDoc {
                name: "real_one",
                description: "Constructs one through `Real::one()`.",
            },
            BenchDoc {
                name: "real_from_i32_one",
                description: "Constructs one through integer conversion.",
            },
            BenchDoc {
                name: "real_from_u8_four",
                description: "Constructs positive four as an exact `Real` from `u8`.",
            },
            BenchDoc {
                name: "real_from_i8_minus_four",
                description: "Constructs negative four as an exact `Real` from `i8`.",
            },
        ],
    },
    BenchGroupDoc {
        name: "raw_cache_hit_cost",
        description: "Cost of cold and cached `Computable::approx` calls for simple values.",
        benches: &[
            BenchDoc {
                name: "zero",
                description: "Cached approximation request for exact zero.",
            },
            BenchDoc {
                name: "one",
                description: "Cached approximation request for exact one.",
            },
            BenchDoc {
                name: "two",
                description: "Cached approximation request for exact two.",
            },
            BenchDoc {
                name: "e",
                description: "Cached approximation request for Euler's constant.",
            },
            BenchDoc {
                name: "pi",
                description: "Cached approximation request for pi.",
            },
            BenchDoc {
                name: "tau",
                description: "Cached approximation request for two pi.",
            },
        ],
    },
    BenchGroupDoc {
        name: "structural_query_speed",
        description: "Speed of public structural queries across exact, transcendental, and composite `Real` values.",
        benches: &[
            BenchDoc {
                name: "zero_zero_status",
                description: "Checks zero/nonzero facts for exact zero.",
            },
            BenchDoc {
                name: "zero_sign_query",
                description: "Reads sign facts for exact zero.",
            },
            BenchDoc {
                name: "zero_msd_query",
                description: "Reads magnitude facts for exact zero.",
            },
            BenchDoc {
                name: "zero_structural_facts",
                description: "Computes full structural facts for exact zero.",
            },
            BenchDoc {
                name: "one_zero_status",
                description: "Checks zero/nonzero facts for exact one.",
            },
            BenchDoc {
                name: "one_sign_query",
                description: "Reads sign facts for exact one.",
            },
            BenchDoc {
                name: "one_msd_query",
                description: "Reads magnitude facts for exact one.",
            },
            BenchDoc {
                name: "one_structural_facts",
                description: "Computes full structural facts for exact one.",
            },
            BenchDoc {
                name: "negative_zero_status",
                description: "Checks zero/nonzero facts for an exact negative integer.",
            },
            BenchDoc {
                name: "negative_sign_query",
                description: "Reads sign facts for an exact negative integer.",
            },
            BenchDoc {
                name: "negative_msd_query",
                description: "Reads magnitude facts for an exact negative integer.",
            },
            BenchDoc {
                name: "negative_structural_facts",
                description: "Computes full structural facts for an exact negative integer.",
            },
            BenchDoc {
                name: "tiny_exact_zero_status",
                description: "Checks zero/nonzero facts for a tiny exact rational.",
            },
            BenchDoc {
                name: "tiny_exact_sign_query",
                description: "Reads sign facts for a tiny exact rational.",
            },
            BenchDoc {
                name: "tiny_exact_msd_query",
                description: "Reads magnitude facts for a tiny exact rational.",
            },
            BenchDoc {
                name: "tiny_exact_structural_facts",
                description: "Computes full structural facts for a tiny exact rational.",
            },
            BenchDoc {
                name: "pi_zero_status",
                description: "Checks zero/nonzero facts for pi.",
            },
            BenchDoc {
                name: "pi_sign_query",
                description: "Reads sign facts for pi.",
            },
            BenchDoc {
                name: "pi_msd_query",
                description: "Reads magnitude facts for pi.",
            },
            BenchDoc {
                name: "pi_structural_facts",
                description: "Computes full structural facts for pi.",
            },
            BenchDoc {
                name: "e_zero_status",
                description: "Checks zero/nonzero facts for e.",
            },
            BenchDoc {
                name: "e_sign_query",
                description: "Reads sign facts for e.",
            },
            BenchDoc {
                name: "e_msd_query",
                description: "Reads magnitude facts for e.",
            },
            BenchDoc {
                name: "e_structural_facts",
                description: "Computes full structural facts for e.",
            },
            BenchDoc {
                name: "tau_zero_status",
                description: "Checks zero/nonzero facts for tau.",
            },
            BenchDoc {
                name: "tau_sign_query",
                description: "Reads sign facts for tau.",
            },
            BenchDoc {
                name: "tau_msd_query",
                description: "Reads magnitude facts for tau.",
            },
            BenchDoc {
                name: "tau_structural_facts",
                description: "Computes full structural facts for tau.",
            },
            BenchDoc {
                name: "sqrt_two_zero_status",
                description: "Checks zero/nonzero facts for sqrt(2).",
            },
            BenchDoc {
                name: "sqrt_two_sign_query",
                description: "Reads sign facts for sqrt(2).",
            },
            BenchDoc {
                name: "sqrt_two_msd_query",
                description: "Reads magnitude facts for sqrt(2).",
            },
            BenchDoc {
                name: "sqrt_two_structural_facts",
                description: "Computes full structural facts for sqrt(2).",
            },
            BenchDoc {
                name: "pi_minus_three_zero_status",
                description: "Checks zero/nonzero facts for pi - 3.",
            },
            BenchDoc {
                name: "pi_minus_three_sign_query",
                description: "Reads sign facts for pi - 3.",
            },
            BenchDoc {
                name: "pi_minus_three_msd_query",
                description: "Reads magnitude facts for pi - 3.",
            },
            BenchDoc {
                name: "pi_minus_three_structural_facts",
                description: "Computes full structural facts for pi - 3.",
            },
            BenchDoc {
                name: "dense_expr_zero_status",
                description: "Checks zero/nonzero facts for a dense composite expression.",
            },
            BenchDoc {
                name: "dense_expr_sign_query",
                description: "Reads sign facts for a dense composite expression.",
            },
            BenchDoc {
                name: "dense_expr_msd_query",
                description: "Reads magnitude facts for a dense composite expression.",
            },
            BenchDoc {
                name: "dense_expr_structural_facts",
                description: "Computes full structural facts for a dense composite expression.",
            },
            BenchDoc {
                name: "structural_negation_match",
                description: "Certifies opposite rational scales over one shared exact symbolic basis.",
            },
            BenchDoc {
                name: "structural_negation_miss",
                description: "Rejects an unrelated exact symbolic basis without constructing a negation.",
            },
        ],
    },
    BenchGroupDoc {
        name: "pure_scalar_algorithm_speed",
        description: "Core scalar algorithms that do not require high-precision transcendental approximation.",
        benches: &[
            BenchDoc {
                name: "rational_add",
                description: "Adds two nontrivial rational values.",
            },
            BenchDoc {
                name: "rational_sub",
                description: "Subtracts two nontrivial rational values.",
            },
            BenchDoc {
                name: "rational_add_wide_dyadic_cold",
                description: "Adds fresh integer and wide-dyadic operands without retained work.",
            },
            BenchDoc {
                name: "rational_sub_wide_dyadic_cold",
                description: "Subtracts fresh integer and wide-dyadic operands without retained work.",
            },
            BenchDoc {
                name: "rational_add_shared_cold",
                description: "Adds fresh operands whose storage is cloned but whose arithmetic pair is not yet observed.",
            },
            BenchDoc {
                name: "rational_sub_shared_cold",
                description: "Subtracts fresh operands whose storage is cloned but whose arithmetic pair is not yet observed.",
            },
            BenchDoc {
                name: "rational_scaled_difference_composed_cold",
                description: "Computes a fresh wide-integer scaled difference through multiply then subtract.",
            },
            BenchDoc {
                name: "rational_scaled_difference_fused_cold",
                description: "Computes the same fresh wide-integer scaled difference with the fused integer kernel.",
            },
            BenchDoc {
                name: "rational_cross_difference_unit_divisor_composed_cold",
                description: "Computes a fresh wide-integer cross difference and divides it by negative one through general operations.",
            },
            BenchDoc {
                name: "rational_cross_difference_unit_divisor_fused_cold",
                description: "Computes the same cross difference through the checked fused unit-divisor path.",
            },
            BenchDoc {
                name: "rational_mul",
                description: "Multiplies two nontrivial rational values.",
            },
            BenchDoc {
                name: "rational_mul_retained_general",
                description: "Reuses one retained exact product for an immutable rational operand pair.",
            },
            BenchDoc {
                name: "rational_mul_wide_dyadic_cold",
                description: "Multiplies fresh wide-denominator dyadics whose numerators fit `u128`.",
            },
            BenchDoc {
                name: "rational_mul_dyadic_general_cross_cancel",
                description: "Multiplies a wide dyadic rational by a general rational with a power-of-two numerator.",
            },
            BenchDoc {
                name: "rational_div",
                description: "Divides two nontrivial rational values.",
            },
            BenchDoc {
                name: "rational_inverse_owned_cold",
                description: "Inverts a fresh uniquely owned nontrivial rational.",
            },
            BenchDoc {
                name: "rational_inverse_retained",
                description: "Reuses the retained reciprocal of a shared nontrivial rational.",
            },
            BenchDoc {
                name: "rational_neg_owned_cold",
                description: "Negates a fresh uniquely owned nontrivial rational in place.",
            },
            BenchDoc {
                name: "rational_neg_retained",
                description: "Reuses the retained opposite sign of a shared nontrivial rational.",
            },
            BenchDoc {
                name: "real_exact_powi_i64_owned_cold",
                description: "Raises a fresh uniquely owned exact rational Real to the fifth power.",
            },
            BenchDoc {
                name: "real_exact_powi_i64_retained",
                description: "Reuses the bounded exact product chain for a shared fifth power.",
            },
            BenchDoc {
                name: "real_exact_add",
                description: "Adds exact rational-backed `Real` values.",
            },
            BenchDoc {
                name: "real_exact_average_pair",
                description: "Averages exact rational-backed `Real` values through the fused pair kernel.",
            },
            BenchDoc {
                name: "real_exact_average_pair_expanded",
                description: "Averages exact rational-backed `Real` values through separate add and divide operations.",
            },
            BenchDoc {
                name: "real_exact_sub",
                description: "Subtracts exact rational-backed `Real` values.",
            },
            BenchDoc {
                name: "real_exact_mul",
                description: "Multiplies exact rational-backed `Real` values.",
            },
            BenchDoc {
                name: "real_exact_mul_retained",
                description: "Reuses the retained exact product beneath rational-backed `Real` values.",
            },
            BenchDoc {
                name: "real_exact_div",
                description: "Divides exact rational-backed `Real` values.",
            },
            BenchDoc {
                name: "real_exact_sqrt_owned_cold",
                description: "Reduces a fresh uniquely owned exact square-root expression.",
            },
            BenchDoc {
                name: "real_exact_sqrt_reduce",
                description: "Reuses the retained reduction of an exact square-root expression.",
            },
            BenchDoc {
                name: "real_exact_dyadic_sqrt_reduce",
                description: "Reuses the square-root reduction of a large exact dyadic rational.",
            },
            BenchDoc {
                name: "real_exact_general_sqrt_reduce",
                description: "Reuses the square-root reduction of a non-dyadic rational sum of squares.",
            },
            BenchDoc {
                name: "real_exact_dyadic_radical_scale",
                description: "Scales an exact reciprocal radical by one exact binary64-derived dyadic coordinate.",
            },
            BenchDoc {
                name: "real_exact_ln_reduce",
                description: "Reduces an exact logarithm of a power of two.",
            },
            BenchDoc {
                name: "real_pow_small_integer_exponent",
                description: "Dispatches `Real::pow` with an exact small-integer exponent.",
            },
        ],
    },
    BenchGroupDoc {
        name: "rational_algorithm_dispatch_speed",
        description: "Cold backend algorithm families and retained rational fact dispatch selected from GMP-style operand shapes.",
        benches: &[
            BenchDoc {
                name: "dyadic_fact_cold",
                description: "Classifies a fresh non-dyadic denominator and retains the result.",
            },
            BenchDoc {
                name: "dyadic_fact_retained",
                description: "Reads an already-retained non-dyadic denominator classification.",
            },
            BenchDoc {
                name: "compare_leading_significand_retained_1024_bits",
                description: "Compares retained wide rational magnitudes through the certified leading-significand interval.",
            },
            BenchDoc {
                name: "compare_dyadic_shifted_retained_1024_bits",
                description: "Compares retained wide dyadics with equal scaled width and a five-bit denominator-shift difference.",
            },
            BenchDoc {
                name: "equality_leading_significand_retained_1024_bits",
                description: "Compares unequal retained wide fractions with separated leading significands.",
            },
            BenchDoc {
                name: "equality_dyadic_shifted_retained_1024_bits",
                description: "Rejects unequal retained wide dyadics without cross products.",
            },
            BenchDoc {
                name: "equality_shared_identity_retained",
                description: "Compares two references to one retained rational allocation.",
            },
            BenchDoc {
                name: "equality_same_denominator_retained",
                description: "Rejects unequal small numerators with a common denominator.",
            },
            BenchDoc {
                name: "equality_word_retained",
                description: "Rejects unequal small fractions with different non-dyadic denominators.",
            },
            BenchDoc {
                name: "equality_equal_distinct_retained",
                description: "Recognizes equal small fractions held in distinct allocations.",
            },
            BenchDoc {
                name: "equality_different_signs_retained",
                description: "Rejects wide fractions with opposite signs before magnitude comparison.",
            },
            BenchDoc {
                name: "equality_close_retained_1024_bits",
                description: "Rejects near-equal wide fractions through the exact cross-product fallback.",
            },
            BenchDoc {
                name: "mul_backend_basecase_cold",
                description: "Multiplies fresh balanced 16-limb integers through the backend basecase kernel.",
            },
            BenchDoc {
                name: "mul_backend_half_karatsuba_cold",
                description: "Multiplies fresh unbalanced 33-by-66-limb integers through half-Karatsuba.",
            },
            BenchDoc {
                name: "mul_backend_karatsuba_cold",
                description: "Multiplies fresh balanced 40-limb integers through Karatsuba.",
            },
            BenchDoc {
                name: "mul_backend_toom3_cold",
                description: "Multiplies fresh balanced 257-limb integers through Toom-3.",
            },
            BenchDoc {
                name: "mul_toom4_candidate_4096_bits",
                description: "Runs Hyperreal's seven-product Rust-native Toom-4 candidate on balanced 4,096-bit operands.",
            },
            BenchDoc {
                name: "mul_backend_reference_4096_bits",
                description: "Runs the native backend product on the same 4,096-bit operands.",
            },
            BenchDoc {
                name: "mul_toom4_candidate_16384_bits",
                description: "Runs Hyperreal's Rust-native Toom-4 candidate on balanced 16,384-bit operands.",
            },
            BenchDoc {
                name: "mul_backend_reference_16384_bits",
                description: "Runs the native backend product on the same 16,384-bit operands.",
            },
            BenchDoc {
                name: "mul_toom4_candidate_65536_bits",
                description: "Runs Hyperreal's Rust-native Toom-4 candidate on balanced 65,536-bit operands.",
            },
            BenchDoc {
                name: "mul_backend_reference_65536_bits",
                description: "Runs the native backend product on the same 65,536-bit operands.",
            },
            BenchDoc {
                name: "mul_toom4_candidate_262144_bits",
                description: "Runs Hyperreal's Rust-native Toom-4 candidate on balanced 262,144-bit operands.",
            },
            BenchDoc {
                name: "mul_backend_reference_262144_bits",
                description: "Runs the native backend product on the same 262,144-bit operands.",
            },
            BenchDoc {
                name: "mul_toom4_candidate_524288_bits",
                description: "Runs Hyperreal's Rust-native Toom-4 candidate on balanced 524,288-bit operands.",
            },
            BenchDoc {
                name: "mul_backend_reference_524288_bits",
                description: "Runs the native backend product on the same 524,288-bit operands.",
            },
            BenchDoc {
                name: "mul_toom4_candidate_1048576_bits",
                description: "Runs Hyperreal's Rust-native Toom-4 candidate on balanced 1,048,576-bit operands.",
            },
            BenchDoc {
                name: "mul_backend_reference_1048576_bits",
                description: "Runs the native backend product on the same 1,048,576-bit operands.",
            },
            BenchDoc {
                name: "mul_toom4_candidate_2097152_bits",
                description: "Runs Hyperreal's Rust-native Toom-4 candidate on balanced 2,097,152-bit operands.",
            },
            BenchDoc {
                name: "mul_backend_reference_2097152_bits",
                description: "Runs the native backend product on the same 2,097,152-bit operands.",
            },
            BenchDoc {
                name: "mul_selected_1048576_bits",
                description: "Runs the retained production Toom-8 selector above its balanced crossover.",
            },
            BenchDoc {
                name: "mul_selected_2097152_bits",
                description: "Runs the retained production Toom-8 selector on balanced 2,097,152-bit operands.",
            },
            BenchDoc {
                name: "mul_toom6_candidate_1048576_bits",
                description: "Runs Hyperreal's eleven-product Rust-native Toom-6 candidate above its crossover.",
            },
            BenchDoc {
                name: "mul_toom6_candidate_131072_bits",
                description: "Runs Hyperreal's Rust-native Toom-6 candidate on balanced 131,072-bit operands.",
            },
            BenchDoc {
                name: "mul_backend_reference_131072_bits",
                description: "Runs the retained native backend selector on the same 131,072-bit operands.",
            },
            BenchDoc {
                name: "mul_toom6_candidate_262144_bits",
                description: "Runs Hyperreal's Rust-native Toom-6 candidate on balanced 262,144-bit operands.",
            },
            BenchDoc {
                name: "mul_toom6_candidate_524288_bits",
                description: "Runs Hyperreal's Rust-native Toom-6 candidate on balanced 524,288-bit operands.",
            },
            BenchDoc {
                name: "mul_selected_524288_bits",
                description: "Runs the retained production Toom-8 selector above its balanced crossover.",
            },
            BenchDoc {
                name: "mul_toom6_candidate_2097152_bits",
                description: "Runs Hyperreal's Rust-native Toom-6 candidate on balanced 2,097,152-bit operands.",
            },
            BenchDoc {
                name: "mul_selected_toom4_unbalanced_1258291_by_1048576",
                description: "Runs retained Toom-4 on a 6:5 operand pair outside Toom-6's balance band.",
            },
            BenchDoc {
                name: "mul_backend_unbalanced_1258291_by_1048576",
                description: "Runs the native backend on the same 6:5 operand pair.",
            },
            BenchDoc {
                name: "mul_toom8_candidate_262144_bits",
                description: "Runs Hyperreal's fifteen-product Rust-native Toom-8 candidate on balanced 262,144-bit operands.",
            },
            BenchDoc {
                name: "mul_selected_262144_bits",
                description: "Runs the retained production Toom-8 selector at its balanced crossover.",
            },
            BenchDoc {
                name: "mul_toom8_candidate_65536_bits",
                description: "Runs Hyperreal's Rust-native Toom-8 candidate on balanced 65,536-bit operands.",
            },
            BenchDoc {
                name: "mul_toom8_candidate_131072_bits",
                description: "Runs Hyperreal's Rust-native Toom-8 candidate on balanced 131,072-bit operands.",
            },
            BenchDoc {
                name: "mul_toom8_candidate_524288_bits",
                description: "Runs Hyperreal's Rust-native Toom-8 candidate at the Toom-6 crossover.",
            },
            BenchDoc {
                name: "mul_toom8_candidate_1048576_bits",
                description: "Runs Hyperreal's Rust-native Toom-8 candidate on balanced 1,048,576-bit operands.",
            },
            BenchDoc {
                name: "mul_toom8_candidate_2097152_bits",
                description: "Runs Hyperreal's Rust-native Toom-8 candidate on balanced 2,097,152-bit operands.",
            },
            BenchDoc {
                name: "mul_toom8_candidate_4194304_bits",
                description: "Runs Hyperreal's Rust-native Toom-8 candidate on balanced 4,194,304-bit operands.",
            },
            BenchDoc {
                name: "mul_selected_4194304_bits",
                description: "Runs the retained production Toom-8 selector on the same 4,194,304-bit operands.",
            },
            BenchDoc {
                name: "mul_selected_toom6_unbalanced_599186_by_524288",
                description: "Runs retained Toom-6 on an 8:7 operand pair outside Toom-8's balance band.",
            },
            BenchDoc {
                name: "mul_backend_unbalanced_599186_by_524288",
                description: "Runs the native backend on the same 8:7 operand pair.",
            },
            BenchDoc {
                name: "mul_ntt_candidate_262144_bits",
                description: "Runs Hyperreal's exact two-prime Rust-native NTT/CRT candidate on balanced 262,144-bit operands.",
            },
            BenchDoc {
                name: "mul_ntt_candidate_1048576_bits",
                description: "Runs the Rust-native NTT/CRT candidate on balanced 1,048,576-bit operands.",
            },
            BenchDoc {
                name: "mul_ntt_candidate_4194304_bits",
                description: "Runs the Rust-native NTT/CRT candidate on balanced 4,194,304-bit operands.",
            },
            BenchDoc {
                name: "reduce_backend_single_limb_cold",
                description: "Reduces a fresh wide fraction by a single-limb exact divisor.",
            },
            BenchDoc {
                name: "reduce_backend_knuth_cold",
                description: "Reduces a fresh wide fraction through normalized Knuth basecase division.",
            },
            BenchDoc {
                name: "reduce_backend_large_knuth_cold",
                description: "Reduces a fresh 129-limb numerator by a 65-limb exact divisor through normalized Knuth division.",
            },
            BenchDoc {
                name: "reduce_fixed_512_coprime_cold",
                description: "Reduces fresh balanced 512-bit operands through the fixed-limb rational-operation GCD.",
            },
            BenchDoc {
                name: "exact_remainder_large_knuth",
                description: "Computes a wide rational fractional remainder through the traced normalized Knuth backend.",
            },
            BenchDoc {
                name: "division_trivial_small_quotient",
                description: "Exercises the backend's zero-quotient magnitude division exit on wide operands.",
            },
            BenchDoc {
                name: "gcd_selected_128_bits",
                description: "Runs selected magnitude GCD on an ascending balanced two-limb pair.",
            },
            BenchDoc {
                name: "gcd_euclidean_128_bits",
                description: "Runs the full-width Euclidean baseline on the same 128-bit pair.",
            },
            BenchDoc {
                name: "gcd_selected_192_bits",
                description: "Runs selected magnitude GCD at the retained three-limb Lehmer crossover.",
            },
            BenchDoc {
                name: "gcd_euclidean_192_bits",
                description: "Runs the full-width Euclidean baseline on the same 192-bit pair.",
            },
            BenchDoc {
                name: "gcd_selected_512_bits",
                description: "Runs selected magnitude GCD above the Lehmer crossover.",
            },
            BenchDoc {
                name: "gcd_euclidean_512_bits",
                description: "Runs the full-width Euclidean baseline on the same 512-bit pair.",
            },
            BenchDoc {
                name: "gcd_selected_1024_bits",
                description: "Runs selected magnitude GCD above the Lehmer crossover.",
            },
            BenchDoc {
                name: "gcd_euclidean_1024_bits",
                description: "Runs the full-width Euclidean baseline on the same 1,024-bit pair.",
            },
            BenchDoc {
                name: "gcd_selected_4096_bits",
                description: "Runs selected magnitude GCD well above the Lehmer crossover.",
            },
            BenchDoc {
                name: "gcd_euclidean_4096_bits",
                description: "Runs the full-width Euclidean baseline on the same 4,096-bit pair.",
            },
            BenchDoc {
                name: "gcd_selected_unbalanced_to_lehmer_192_bits",
                description: "Runs selected magnitude GCD on an initially unbalanced pair whose first remainder is balanced at 192 bits.",
            },
            BenchDoc {
                name: "gcd_euclidean_unbalanced_to_lehmer_192_bits",
                description: "Runs the full-width Euclidean baseline on the same initially unbalanced pair.",
            },
            BenchDoc {
                name: "gcd_selected_unbalanced_to_lehmer_256_bits",
                description: "Runs selected magnitude GCD on an initially unbalanced pair whose first remainder is balanced at 256 bits.",
            },
            BenchDoc {
                name: "gcd_euclidean_unbalanced_to_lehmer_256_bits",
                description: "Runs the full-width Euclidean baseline on the same initially unbalanced pair.",
            },
            BenchDoc {
                name: "gcd_selected_unbalanced_to_lehmer_512_bits",
                description: "Runs selected magnitude GCD on an initially unbalanced pair whose first remainder is balanced at 512 bits.",
            },
            BenchDoc {
                name: "gcd_euclidean_unbalanced_to_lehmer_512_bits",
                description: "Runs the full-width Euclidean baseline on the same initially unbalanced pair.",
            },
            BenchDoc {
                name: "gcd_selected_unbalanced_to_lehmer_1024_bits",
                description: "Runs selected magnitude GCD on an initially unbalanced pair whose first remainder is balanced at 1,024 bits.",
            },
            BenchDoc {
                name: "gcd_euclidean_unbalanced_to_lehmer_1024_bits",
                description: "Runs the full-width Euclidean baseline on the same initially unbalanced pair.",
            },
            BenchDoc {
                name: "gcd_selected_unbalanced_to_lehmer_4096_bits",
                description: "Runs selected magnitude GCD on an initially unbalanced pair whose first remainder is balanced at 4,096 bits.",
            },
            BenchDoc {
                name: "gcd_euclidean_unbalanced_to_lehmer_4096_bits",
                description: "Runs the full-width Euclidean baseline on the same initially unbalanced pair.",
            },
            BenchDoc {
                name: "half_gcd_candidate_8192_bits",
                description: "Runs the recursive half-GCD candidate below its provisional crossover.",
            },
            BenchDoc {
                name: "half_gcd_lehmer_8192_bits",
                description: "Runs the quadratic Lehmer baseline on the same 8,192-bit pair.",
            },
            BenchDoc {
                name: "half_gcd_candidate_16384_bits",
                description: "Runs the recursive half-GCD candidate at its provisional crossover.",
            },
            BenchDoc {
                name: "half_gcd_lehmer_16384_bits",
                description: "Runs the quadratic Lehmer baseline on the same 16,384-bit pair.",
            },
            BenchDoc {
                name: "half_gcd_candidate_65536_bits",
                description: "Runs the recursive half-GCD candidate well above its provisional crossover.",
            },
            BenchDoc {
                name: "half_gcd_lehmer_65536_bits",
                description: "Runs the quadratic Lehmer baseline on the same 65,536-bit pair.",
            },
            BenchDoc {
                name: "half_gcd_candidate_262144_bits",
                description: "Runs recursive half-GCD with selected higher-Toom matrix products at 262,144 bits.",
            },
            BenchDoc {
                name: "half_gcd_lehmer_262144_bits",
                description: "Runs the Lehmer baseline on the same 262,144-bit pair.",
            },
            BenchDoc {
                name: "half_gcd_candidate_1048576_bits",
                description: "Runs recursive half-GCD with selected higher-Toom matrix products at 1,048,576 bits.",
            },
            BenchDoc {
                name: "half_gcd_lehmer_1048576_bits",
                description: "Runs the Lehmer baseline on the same 1,048,576-bit pair.",
            },
            BenchDoc {
                name: "barrett_one_shot_8192_by_1024",
                description: "Prepares a Rust-native Barrett reciprocal and divides one 8,192-bit value by a 1,024-bit divisor.",
            },
            BenchDoc {
                name: "backend_one_shot_8192_by_1024",
                description: "Runs the native backend div-rem baseline for the same one-shot operands.",
            },
            BenchDoc {
                name: "barrett_batch16_8192_by_1024",
                description: "Amortizes one Rust-native Barrett reciprocal over sixteen 8,192-bit dividends.",
            },
            BenchDoc {
                name: "backend_batch16_8192_by_1024",
                description: "Runs sixteen native backend div-rem operations on the same values.",
            },
            BenchDoc {
                name: "barrett_batch16_65536_by_4096",
                description: "Amortizes one Rust-native Barrett reciprocal over sixteen 65,536-bit dividends.",
            },
            BenchDoc {
                name: "backend_batch16_65536_by_4096",
                description: "Runs sixteen native backend div-rem operations on the same large values.",
            },
            BenchDoc {
                name: "perfect_power_factor_reject",
                description: "Rejects 12 after small-factor multiplicities collapse to gcd one.",
            },
            BenchDoc {
                name: "perfect_power_general_seventh",
                description: "Discovers an exact rational seventh power whose base primes exceed the trial table.",
            },
            BenchDoc {
                name: "perfect_power_fixed_seventh",
                description: "Checks the same value when the seventh-root degree is already known.",
            },
            BenchDoc {
                name: "perfect_power_unfactored_reject",
                description: "Rejects mismatched seventh- and fifth-power rational components beyond the trial table.",
            },
            BenchDoc {
                name: "radix_format_small_integer",
                description: "Formats a 16-limb integer using repeated single-limb radix division.",
            },
            BenchDoc {
                name: "radix_format_large_integer",
                description: "Formats a 32-limb integer using divide-and-conquer radix conversion.",
            },
            BenchDoc {
                name: "radix_parse_short_decimal",
                description: "Parses a short exact decimal through the checked word-sized path.",
            },
            BenchDoc {
                name: "radix_parse_short_scientific",
                description: "Parses a representative file-I/O scientific literal through the checked word-sized path.",
            },
            BenchDoc {
                name: "radix_parse_wide_scientific",
                description: "Parses a 5,120-digit significand with a negative decimal exponent exactly.",
            },
            BenchDoc {
                name: "radix_parse_wide_scientific_expanded",
                description: "Parses the same exact wide value after expanding its decimal point as a baseline.",
            },
            BenchDoc {
                name: "radix_parse_large_integer",
                description: "Parses a large below-threshold decimal fixture through chunked multiply-add conversion.",
            },
            BenchDoc {
                name: "radix_parse_divide_conquer_10240_digits",
                description: "Parses 10,240 digits through the divide-and-conquer product tree.",
            },
            BenchDoc {
                name: "radix_parse_backend_chunked_10240_digits",
                description: "Parses the same 10,240 digits with the backend chunked multiply-add baseline.",
            },
            BenchDoc {
                name: "radix_parse_divide_conquer_20480_digits",
                description: "Parses 20,480 digits through the divide-and-conquer product tree.",
            },
            BenchDoc {
                name: "radix_parse_backend_chunked_20480_digits",
                description: "Parses the same 20,480 digits with the backend chunked multiply-add baseline.",
            },
            BenchDoc {
                name: "radix_format_fraction_decimal",
                description: "Formats a rational decimal through exact repeated digit division.",
            },
        ],
    },
    BenchGroupDoc {
        name: "borrowed_op_overhead",
        description: "Borrowed versus owned operation overhead for rational and real operands.",
        benches: &[
            BenchDoc {
                name: "rational_clone_pair",
                description: "Clones two rational values.",
            },
            BenchDoc {
                name: "rational_add_refs",
                description: "Adds rational references.",
            },
            BenchDoc {
                name: "rational_add_owned",
                description: "Adds owned rational values.",
            },
            BenchDoc {
                name: "real_clone_pair",
                description: "Clones two scaled transcendental `Real` values.",
            },
            BenchDoc {
                name: "real_unscaled_add_refs",
                description: "Adds borrowed unscaled transcendental `Real` values.",
            },
            BenchDoc {
                name: "real_unscaled_add_owned",
                description: "Adds owned unscaled transcendental `Real` values.",
            },
            BenchDoc {
                name: "real_add_refs",
                description: "Adds borrowed scaled transcendental `Real` values.",
            },
            BenchDoc {
                name: "real_add_owned",
                description: "Adds owned scaled transcendental `Real` values.",
            },
            BenchDoc {
                name: "real_dot2_refs_dense_symbolic",
                description: "Computes a borrowed two-lane symbolic dot product with no rational shortcut terms.",
            },
            BenchDoc {
                name: "real_active_dot2_refs_dense_symbolic",
                description: "Computes a borrowed two-lane symbolic dot product after the caller has already classified every lane active.",
            },
            BenchDoc {
                name: "real_dot2_refs_mixed_structural",
                description: "Computes a borrowed two-lane symbolic dot product with an exact zero lane and a rational scale lane.",
            },
            BenchDoc {
                name: "real_dot3_refs_dense_symbolic",
                description: "Computes a borrowed three-lane symbolic dot product with no rational shortcut terms.",
            },
            BenchDoc {
                name: "real_active_dot3_refs_dense_symbolic",
                description: "Computes a borrowed three-lane symbolic dot product after the caller has already classified every lane active.",
            },
            BenchDoc {
                name: "real_dot3_refs_mixed_structural",
                description: "Computes a borrowed three-lane symbolic dot product with exact zero and rational scale terms.",
            },
            BenchDoc {
                name: "real_dot4_refs_dense_symbolic",
                description: "Computes a borrowed four-lane symbolic dot product with no rational shortcut terms.",
            },
            BenchDoc {
                name: "real_active_dot4_refs_dense_symbolic",
                description: "Computes a borrowed four-lane symbolic dot product after the caller has already classified every lane active.",
            },
            BenchDoc {
                name: "real_dot4_refs_mixed_structural",
                description: "Computes a borrowed four-lane symbolic dot product with exact zero and rational scale terms.",
            },
        ],
    },
    BenchGroupDoc {
        name: "dense_algebra",
        description: "Small dense algebra kernels that stress repeated exact and symbolic operations.",
        benches: &[
            BenchDoc {
                name: "rational_dot_64",
                description: "Computes a 64-element rational dot product.",
            },
            BenchDoc {
                name: "rational_matmul_8",
                description: "Computes an 8x8 rational matrix multiply.",
            },
            BenchDoc {
                name: "real_dot_36",
                description: "Computes a 36-element dot product over symbolic `Real` values.",
            },
            BenchDoc {
                name: "real_matmul_6",
                description: "Computes a 6x6 matrix multiply over symbolic `Real` values.",
            },
            BenchDoc {
                name: "real_sum_refs_64_symbolic",
                description: "Constructs an arbitrary-length sum of 64 borrowed symbolic square roots.",
            },
            BenchDoc {
                name: "real_sum_refs_64_symbolic_to_f64",
                description: "Constructs and approximates the same arbitrary-length symbolic sum.",
            },
            BenchDoc {
                name: "real_sum_refs_1024_symbolic_sequential",
                description: "Constructs a 1,024-term symbolic sum with the former sequential fold.",
            },
            BenchDoc {
                name: "real_sum_refs_1024_symbolic",
                description: "Constructs the same 1,024-term symbolic sum through the balanced public aggregate.",
            },
            BenchDoc {
                name: "real_sum_refs_1024_symbolic_to_f64_sequential",
                description: "Sequentially constructs and approximates a 1,024-term symbolic sum with cold child caches.",
            },
            BenchDoc {
                name: "real_sum_refs_1024_symbolic_to_f64",
                description: "Constructs and approximates the same cold 1,024-term symbolic sum through the balanced public aggregate.",
            },
            BenchDoc {
                name: "real_sum_refs_1024_rational_sequential",
                description: "Constructs a 1,024-term exact rational sum with the former sequential fold.",
            },
            BenchDoc {
                name: "real_sum_refs_1024_rational",
                description: "Constructs the same exact rational sum through the homogeneous-prefix public aggregate.",
            },
            BenchDoc {
                name: "real_sum_owned_1024_symbolic_former_clone_path",
                description: "Consumes a 1,024-term symbolic sum while reproducing the former extra per-value clone.",
            },
            BenchDoc {
                name: "real_sum_owned_1024_symbolic",
                description: "Consumes the same symbolic sum by moving owned values into the balanced reducer.",
            },
        ],
    },
    BenchGroupDoc {
        name: "exact_transcendental_special_forms",
        description: "Construction-time shortcuts for exact rational multiples of pi and inverse compositions.",
        benches: &[
            BenchDoc {
                name: "sin_pi_7",
                description: "Builds the exact special form for sin(pi/7).",
            },
            BenchDoc {
                name: "cos_pi_7",
                description: "Builds the exact special form for cos(pi/7).",
            },
            BenchDoc {
                name: "tan_pi_7",
                description: "Builds the exact special form for tan(pi/7).",
            },
            BenchDoc {
                name: "asin_sin_6pi_7",
                description: "Recognizes the principal branch of asin(sin(6pi/7)).",
            },
            BenchDoc {
                name: "acos_cos_9pi_7",
                description: "Recognizes the principal branch of acos(cos(9pi/7)).",
            },
            BenchDoc {
                name: "atan_tan_6pi_7",
                description: "Recognizes the principal branch of atan(tan(6pi/7)).",
            },
            BenchDoc {
                name: "asinh_large",
                description: "Builds a large inverse hyperbolic sine without exact intermediate Reals.",
            },
            BenchDoc {
                name: "atanh_sqrt_half",
                description: "Builds atanh(sqrt(2)/2) after exact structural domain checks.",
            },
            BenchDoc {
                name: "atanh_sqrt_two_error",
                description: "Rejects atanh(sqrt(2)) through exact structural domain checks.",
            },
            BenchDoc {
                name: "sinh_ln_two",
                description: "Folds sinh(ln(2)) to the exact rational 3/4 via the integer-log-collapse shortcut.",
            },
            BenchDoc {
                name: "cosh_ln_two",
                description: "Folds cosh(ln(2)) to the exact rational 5/4 via the integer-log-collapse shortcut.",
            },
            BenchDoc {
                name: "tanh_ln_two",
                description: "Folds tanh(ln(2)) to the exact rational 3/5 via the integer-log-collapse shortcut.",
            },
            BenchDoc {
                name: "sinh_rational_one",
                description: "Builds sinh(1) through the generic (exp(x) - exp(-x))/2 identity path.",
            },
            BenchDoc {
                name: "cosh_rational_one",
                description: "Builds cosh(1) through the generic (exp(x) + exp(-x))/2 identity path.",
            },
            BenchDoc {
                name: "tanh_rational_one",
                description: "Builds tanh(1) through the generic (exp(x) - exp(-x))/(exp(x) + exp(-x)) identity path.",
            },
            BenchDoc {
                name: "atan2_origin",
                description: "Hits the origin (0, 0) short-circuit returning exact zero.",
            },
            BenchDoc {
                name: "atan2_axis_positive_y",
                description: "Hits the positive-y axis short-circuit returning exact pi/2.",
            },
            BenchDoc {
                name: "atan2_axis_negative_x",
                description: "Hits the negative-x axis short-circuit returning exact pi.",
            },
            BenchDoc {
                name: "atan2_quadrant_one_unit_diagonal",
                description: "Quadrant I unit diagonal reduces to atan(1) = pi/4 exact special form.",
            },
            BenchDoc {
                name: "atan2_quadrant_two_pi_correction",
                description: "Quadrant II (1, -2) exercises atan(small ratio) + pi correction.",
            },
            BenchDoc {
                name: "atan2_quadrant_three_negative_pi",
                description: "Quadrant III (-1, -2) exercises atan(small ratio) - pi correction.",
            },
            BenchDoc {
                name: "log2_power_of_two",
                description: "Folds log2(1024) to the exact rational 10 via the integer-log-detection shortcut.",
            },
            BenchDoc {
                name: "log2_rational_three",
                description: "Builds log2(3) as a lightweight Log2 symbolic certificate.",
            },
            BenchDoc {
                name: "log2_ln_quotient_fold",
                description: "Folds ln(5) / ln(2) into a Log2 certificate via the divide-recognize shortcut.",
            },
        ],
    },
    BenchGroupDoc {
        name: "real_cotangent",
        description: "Public cotangent construction and cold refinement against the two compositional alternatives.",
        benches: &[
            BenchDoc {
                name: "atan_two_direct_construct",
                description: "Folds cot(atan(2)) through the public exact inverse-trig rewrite.",
            },
            BenchDoc {
                name: "atan_two_quotient_construct",
                description: "Builds cot(atan(2)) as the control expression cos(x)/sin(x).",
            },
            BenchDoc {
                name: "atan_two_inverse_tan_construct",
                description: "Builds cot(atan(2)) as the incomplete control expression 1/tan(x).",
            },
            BenchDoc {
                name: "tiny_direct_cold_p256",
                description: "Constructs and certifies public cot(2^-10) at 256 fractional bits.",
            },
            BenchDoc {
                name: "tiny_quotient_cold_p256",
                description: "Constructs and certifies cos(2^-10)/sin(2^-10) at 256 fractional bits.",
            },
            BenchDoc {
                name: "tiny_inverse_tan_cold_p256",
                description: "Constructs and certifies 1/tan(2^-10) at 256 fractional bits.",
            },
        ],
    },
    BenchGroupDoc {
        name: "symbolic_reductions",
        description: "Existing symbolic constant algebra cases considered for additional reductions.",
        benches: &[
            BenchDoc {
                name: "sqrt_pi_square",
                description: "Reduces sqrt(pi^2).",
            },
            BenchDoc {
                name: "sqrt_pi_e_square",
                description: "Reduces sqrt((pi * e)^2).",
            },
            BenchDoc {
                name: "ln_scaled_e",
                description: "Reduces ln(2 * e).",
            },
            BenchDoc {
                name: "sub_pi_three",
                description: "Builds the certified pi - 3 constant-offset form.",
            },
            BenchDoc {
                name: "pi_minus_three_facts",
                description: "Reads structural facts for the cached pi - 3 offset form.",
            },
            BenchDoc {
                name: "div_exp_exp",
                description: "Reduces e^3 / e.",
            },
            BenchDoc {
                name: "div_pi_square_e",
                description: "Reduces pi^2 / e.",
            },
            BenchDoc {
                name: "div_const_products",
                description: "Reduces (pi^3 * e^5) / (pi * e^2).",
            },
            BenchDoc {
                name: "inverse_pi",
                description: "Builds the reciprocal of pi.",
            },
            BenchDoc {
                name: "div_one_pi",
                description: "Reduces 1 / pi.",
            },
            BenchDoc {
                name: "div_rational_exp",
                description: "Reduces 2 / e.",
            },
            BenchDoc {
                name: "div_e_pi",
                description: "Reduces e / pi.",
            },
            BenchDoc {
                name: "mul_pi_inverse_pi",
                description: "Multiplies pi by its reciprocal.",
            },
            BenchDoc {
                name: "mul_pi_e_sqrt_two",
                description: "Builds the factored pi * e * sqrt(2) form.",
            },
            BenchDoc {
                name: "mul_const_product_sqrt_sqrt",
                description: "Cancels sqrt(2) from (pi * e * sqrt(2)) * sqrt(2).",
            },
            BenchDoc {
                name: "div_const_product_sqrt_e",
                description: "Reduces (pi * e * sqrt(2)) / e.",
            },
            BenchDoc {
                name: "inverse_const_product_sqrt",
                description: "Builds a rationalized reciprocal of pi * e * sqrt(2).",
            },
            BenchDoc {
                name: "inverse_sqrt_two",
                description: "Builds the rationalized reciprocal of unit-scaled sqrt(2).",
            },
            BenchDoc {
                name: "div_sqrt_two_sqrt_three",
                description: "Rationalizes a quotient of two unit-scaled square roots.",
            },
        ],
    },
    BenchGroupDoc {
        name: "exact_product_sums",
        description: "Fixed product-sum reducers used by determinant and cofactor kernels.",
        benches: &[
            BenchDoc {
                name: "signed_product_sum_lcm_6x2",
                description: "Computes an exact rational six-term signed product sum with mixed denominators.",
            },
            BenchDoc {
                name: "signed_product_sum_common_scale_6x2",
                description: "Computes an exact rational six-term signed product sum through the carried common-scale reducer.",
            },
            BenchDoc {
                name: "signed_product_sum_sparse_single_6x2",
                description: "Computes a sparse exact rational six-term signed product sum with one active product.",
            },
            BenchDoc {
                name: "real_signed_product_sum_rational_det3",
                description: "Computes a 3x3 determinant-shaped signed product sum through the public `Real` builder.",
            },
            BenchDoc {
                name: "real_signed_product_sum_mixed_symbolic_det3",
                description: "Computes the same determinant-shaped builder with symbolic factors and rational scales.",
            },
            BenchDoc {
                name: "exact_rational_sparse_homogeneous_plane_intersection3",
                description: "Computes a canonical exact three-plane cofactor tuple from one sparse dyadic row.",
            },
        ],
    },
];

/// Construct a normalized exact signed word ratio; a zero denominator panics on unwrap.
fn rational(n: i64, d: u64) -> Rational {
    Rational::fraction(n, d).unwrap()
}

/// Wrap an exact signed word ratio as Real, panicking for a zero denominator.
fn real(n: i64, d: u64) -> Real {
    Real::new(rational(n, d))
}

/// Return the shared computable representation of two pi without requesting approximation.
fn tau_computable() -> Computable {
    Computable::tau()
}

/// Request and discard the scaled-integer approximation at the supplied binary precision.
///
/// This allows subsequent requests to reuse the value's refinement state. It neither
/// checks the approximation nor resets other caches, and supplies no work limit.
fn warm_cache(value: &Computable, precision: i32) {
    black_box(value.approx(precision));
}

/// Register ten ordinary-iteration constructors for exact one and signed small integers.
///
/// The Rational, Computable and Real rows include construction and output destruction;
/// no numeric approximation or equality assertion is requested. Constant storage may
/// be shared, so these are not measurements of uniformly fresh heap allocations.
fn bench_construction_speed(c: &mut Criterion) {
    let mut group = c.benchmark_group("construction_speed");

    group.bench_function("rational_one", |b| b.iter(|| black_box(Rational::one())));
    group.bench_function("rational_new_one", |b| {
        b.iter(|| black_box(Rational::new(black_box(1))))
    });
    group.bench_function("rational_from_u8_four", |b| {
        b.iter(|| black_box(Rational::from(black_box(4_u8))))
    });
    group.bench_function("rational_from_i8_minus_four", |b| {
        b.iter(|| black_box(Rational::from(black_box(-4_i8))))
    });
    group.bench_function("computable_one", |b| {
        b.iter(|| black_box(Computable::one()))
    });
    group.bench_function("real_new_rational_one", |b| {
        b.iter(|| black_box(Real::new(Rational::one())))
    });
    group.bench_function("real_one", |b| b.iter(|| black_box(Real::one())));
    group.bench_function("real_from_i32_one", |b| {
        b.iter(|| black_box(Real::from(black_box(1_i32))))
    });
    group.bench_function("real_from_u8_four", |b| {
        b.iter(|| black_box(Real::from(black_box(4_u8))))
    });
    group.bench_function("real_from_i8_minus_four", |b| {
        b.iter(|| black_box(Real::from(black_box(-4_i8))))
    });

    group.finish();
}

/// Build ten labeled Real fixtures spanning rational, named, radical and composite forms.
///
/// They are 0, 1, -7, 2^-160, pi, e, tau, sqrt(2), pi-3 and
/// (7*pi/8 + sqrt(2))*3/5. Positive roots and nonzero denominators justify unwraps.
/// The vector gives deterministic order and labels, not independent expected facts;
/// constructing it does not promise cold child or shared-constant caches.
fn structural_values() -> Vec<(&'static str, Real)> {
    let tiny = Real::new(
        Rational::from_bigint_fraction(BigInt::from(1), BigUint::from(1_u8) << 160).unwrap(),
    );
    let tau = Real::tau();
    let pi_minus_three = Real::pi() - Real::new(Rational::new(3));
    let sqrt_two = Real::new(Rational::new(2)).sqrt().unwrap();
    let dense_expr = ((Real::pi() * real(7, 8)) + sqrt_two.clone()) * real(3, 5);

    vec![
        ("zero", Real::zero()),
        ("one", Real::one()),
        ("negative", Real::new(Rational::new(-7))),
        ("tiny_exact", tiny),
        ("pi", Real::pi()),
        ("e", Real::e()),
        ("tau", tau),
        ("sqrt_two", sqrt_two),
        ("pi_minus_three", pi_minus_three),
        ("dense_expr", dense_expr),
    ]
}

/// Left-fold borrowed products from zero over the shorter of two rational slices.
///
/// Zip silently ignores an unmatched tail; either empty slice yields zero. Factors
/// are black-boxed, and each product/addition materializes exact rational arithmetic.
/// This control has no length validation or resource bound.
fn rational_dot(left: &[Rational], right: &[Rational]) -> Rational {
    left.iter()
        .zip(right)
        .fold(Rational::zero(), |acc, (left, right)| {
            acc + black_box(left) * black_box(right)
        })
}

/// Left-fold borrowed Real products from zero over the shorter input slice.
///
/// Unmatched trailing entries are ignored and an empty side gives zero. Exact
/// expression construction does not force numeric evaluation; factor black-boxing
/// prevents the benchmark from assuming known inputs. No length check is made.
fn real_dot(left: &[Real], right: &[Real]) -> Real {
    left.iter()
        .zip(right)
        .fold(Real::zero(), |acc, (left, right)| {
            acc + (black_box(left) * black_box(right))
        })
}

/// Allocate and fill a row-major 8-by-8 exact rational matrix product.
///
/// Each output uses eight multiply/add steps from zero. Both slices must have at
/// least 64 entries or indexing panics; extra entries are ignored. Output allocation
/// and arithmetic belong to this helper; it is a fixed-size benchmark control, not
/// a shape-checked general matrix API.
fn rational_matmul_8(left: &[Rational], right: &[Rational]) -> Vec<Rational> {
    let n = 8;
    let mut out = vec![Rational::zero(); n * n];
    for row in 0..n {
        for col in 0..n {
            let mut sum = Rational::zero();
            for k in 0..n {
                sum = sum + black_box(&left[row * n + k]) * black_box(&right[k * n + col]);
            }
            out[row * n + col] = sum;
        }
    }
    out
}

/// Allocate a row-major 6-by-6 Real product with six terms per output entry.
///
/// Slices shorter than 36 entries panic and trailing entries are ignored. Products
/// and sums preserve exact expressions without forcing approximation. The result
/// includes its vector allocation; no caller-provided output buffer is reused.
fn real_matmul_6(left: &[Real], right: &[Real]) -> Vec<Real> {
    let n = 6;
    let mut out = vec![Real::zero(); n * n];
    for row in 0..n {
        for col in 0..n {
            let mut sum = Real::zero();
            for k in 0..n {
                sum += black_box(&left[row * n + k]) * black_box(&right[k * n + col]);
            }
            out[row * n + col] = sum;
        }
    }
    out
}

/// Write catalogue metadata, then measure six prewarmed approximation requests.
///
/// Zero, one, two, e, pi and tau are requested at -256 before their timed -128
/// requests. Both precisions are absolute binary scales for integer approximations.
/// Despite the catalogue's cold/cached wording, no cold approximation row is
/// registered here; the results are black-boxed without accuracy assertions.
fn bench_raw_cache_hit_cost(c: &mut Criterion) {
    bench_docs::write_benchmark_docs(
        "scalar_micro",
        "Microbenchmarks for scalar operations, structural queries, cache hits, and dense exact arithmetic.",
        SCALAR_MICRO_GROUPS,
    );

    let mut group = c.benchmark_group("raw_cache_hit_cost");
    let precision = -128;
    let cached_precision = -256;
    let values = [
        ("zero", Computable::rational(Rational::zero())),
        ("one", Computable::one()),
        ("two", Computable::rational(Rational::new(2))),
        ("e", Computable::e()),
        ("pi", Computable::pi()),
        ("tau", tau_computable()),
    ];

    for (name, value) in values {
        warm_cache(&value, cached_precision);
        group.bench_function(name, |b| {
            b.iter(|| black_box(value.approx(black_box(precision))))
        });
    }

    group.finish();
}

/// Register six queries per structural fixture plus two structural-negation probes.
///
/// Each fixture first requests structural_facts outside timing. Timed rows query
/// zero status, sign, magnitude, full/detailed facts and lossy binary64 conversion
/// in that order; later rows may inherit more cache evidence. The final comparisons
/// use sqrt(2) with its negative and with pi. No query result or floating-point
/// accuracy is asserted; an Unknown fact must not be interpreted as a false claim.
fn bench_structural_query_speed(c: &mut Criterion) {
    let mut group = c.benchmark_group("structural_query_speed");

    for (name, value) in structural_values() {
        black_box(value.structural_facts());

        group.bench_function(format!("{name}_zero_status"), |b| {
            b.iter(|| black_box(black_box(&value).zero_status()))
        });
        group.bench_function(format!("{name}_sign_query"), |b| {
            b.iter(|| black_box(black_box(&value).structural_facts().sign))
        });
        group.bench_function(format!("{name}_msd_query"), |b| {
            b.iter(|| black_box(black_box(&value).structural_facts().magnitude))
        });
        group.bench_function(format!("{name}_structural_facts"), |b| {
            b.iter(|| black_box(black_box(&value).structural_facts()))
        });
        group.bench_function(format!("{name}_detailed_facts"), |b| {
            b.iter(|| black_box(black_box(&value).detailed_facts()))
        });
        group.bench_function(format!("{name}_to_f64_lossy"), |b| {
            b.iter(|| black_box(black_box(&value).to_f64_lossy()))
        });
    }

    let sqrt_two = Real::from(2).sqrt().expect("positive exact square root");
    let negative_sqrt_two = -sqrt_two.clone();
    let pi = Real::pi();
    group.bench_function("structural_negation_match", |b| {
        b.iter(|| {
            black_box(black_box(&sqrt_two).is_structural_negation_of(black_box(&negative_sqrt_two)))
        })
    });
    group.bench_function("structural_negation_miss", |b| {
        b.iter(|| black_box(black_box(&sqrt_two).is_structural_negation_of(black_box(&pi))))
    });

    group.finish();
}

/// Compare exact arithmetic/construction on prepared and batched fresh scalar fixtures.
///
/// The main rational pair is 123456789/987654321 and its reciprocal. Explicit
/// precomputation warms selected product, inverse, negation, fifth-power and sqrt(90)
/// paths. Fresh setup variants rebuild operands outside timing, including shared
/// clones kept alive during addition/subtraction. No dispatch counts are asserted.
///
/// Binary64-derived dyadics include 1e-9, 1e-12 and three norm coordinates; they
/// are imported exactly as binary numbers. Other rows form a rational sum of squares,
/// scale a reciprocal norm, construct ln(1024), and construct (7/5)^17. Reusing norm
/// inputs may warm reductions over iterations; cloning them does not clear caches.
///
/// Fused/composed integer controls use A=2^191+17 and B=2^159+29 for A-3B and
/// (7A-4B)/(-1). Checked fused outputs are merely black-boxed, so their success and
/// equality with composed arithmetic are not asserted. Other unwraps require success
/// but do not test the value. Batched setup/output-drop and ordinary timing follow
/// the module contract; no general numeric evaluation is requested.
fn bench_pure_scalar_algorithm_speed(c: &mut Criterion) {
    let mut group = c.benchmark_group("pure_scalar_algorithm_speed");
    let lhs = rational(123_456_789, 987_654_321);
    let rhs = rational(987_654_321, 123_456_789);
    let exact_real_lhs = Real::new(lhs.clone());
    let exact_real_rhs = Real::new(rhs.clone());
    let retained_lhs = Rational::new(1_000_000_000);
    let retained_rhs = Rational::try_from(1.0e-9_f64).expect("finite f64 imports exactly");
    let _ = black_box(&retained_lhs * &retained_rhs);
    let retained_inverse_input = rational(123_456_789, 987_654_321);
    let _retained_inverse = retained_inverse_input.clone().inverse().unwrap();
    let retained_negation_input = rational(123_456_789, 987_654_321);
    let _retained_negation = -&retained_negation_input;
    let retained_powi_input = Real::new(rational(123_456_789, 987_654_321));
    let _cold_powi = retained_powi_input.clone().powi_i64(5).unwrap();
    let _retained_powi = retained_powi_input.clone().powi_i64(5).unwrap();
    let retained_real_lhs = Real::new(Rational::new(1_000_000_000));
    let retained_real_rhs =
        Real::new(Rational::try_from(1.0e-9_f64).expect("finite f64 imports exactly"));
    let _ = black_box(&retained_real_lhs * &retained_real_rhs);
    let sqrt_input = Real::new(Rational::new(90));
    let _cold_sqrt = sqrt_input.clone().sqrt().unwrap();
    let _retained_sqrt = sqrt_input.clone().sqrt().unwrap();
    let dyadic_components = [
        Rational::try_from(1.234_567_890_123_45_f64).expect("finite f64 imports exactly"),
        Rational::try_from(-2.345_678_901_234_56_f64).expect("finite f64 imports exactly"),
        Rational::try_from(3.456_789_012_345_67_f64).expect("finite f64 imports exactly"),
    ];
    let dyadic_sqrt_input = Real::new(
        &(&dyadic_components[0] * &dyadic_components[0])
            + &(&dyadic_components[1] * &dyadic_components[1])
            + &(&dyadic_components[2] * &dyadic_components[2]),
    );
    let dyadic_general_lhs = dyadic_components[0].clone();
    let dyadic_general_rhs = Rational::from_bigint_fraction(
        BigInt::from(1_u8) << 160_usize,
        (BigUint::from(1_u8) << 127_usize) + BigUint::from(123_u8),
    )
    .expect("the synthetic reciprocal scale has a nonzero denominator");
    let dyadic_radical_scale = dyadic_sqrt_input
        .clone()
        .sqrt()
        .expect("a sum of exact squares is nonnegative")
        .inverse()
        .expect("the benchmark norm is nonzero");
    let dyadic_component_real = Real::new(dyadic_components[0].clone());
    let norm_components = [
        rational(123_456_789_012_345, 100_000_000_000_000),
        rational(-234_567_890_123_456, 100_000_000_000_000),
        rational(345_678_901_234_567, 100_000_000_000_000),
    ];
    let general_sqrt_input = Real::new(
        &(&norm_components[0] * &norm_components[0])
            + &(&norm_components[1] * &norm_components[1])
            + &(&norm_components[2] * &norm_components[2]),
    );
    let ln_input = Real::new(Rational::new(1024));
    let pow_base = Real::new(rational(7, 5));
    let pow_exponent = Real::new(Rational::new(17));

    group.bench_function("rational_add", |b| {
        b.iter(|| black_box(black_box(&lhs) + black_box(&rhs)))
    });
    group.bench_function("rational_sub", |b| {
        b.iter(|| black_box(black_box(&lhs) - black_box(&rhs)))
    });
    group.bench_function("rational_add_wide_dyadic_cold", |b| {
        b.iter_batched(
            || {
                (
                    Rational::new(1_000_000_000),
                    Rational::try_from(1.0e-9_f64).expect("finite f64 imports exactly"),
                )
            },
            |(left, right)| black_box(black_box(&left) + black_box(&right)),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("rational_sub_wide_dyadic_cold", |b| {
        b.iter_batched(
            || {
                (
                    Rational::new(1_000_000_000),
                    Rational::try_from(1.0e-9_f64).expect("finite f64 imports exactly"),
                )
            },
            |(left, right)| black_box(black_box(&left) - black_box(&right)),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("rational_add_shared_cold", |b| {
        b.iter_batched(
            || {
                let left = Rational::new(1_000_000_000);
                let right = Rational::try_from(1.0e-9_f64).expect("finite f64 imports exactly");
                let shared_left = left.clone();
                let shared_right = right.clone();
                (left, right, shared_left, shared_right)
            },
            |(left, right, shared_left, shared_right)| {
                let result = black_box(&left) + black_box(&right);
                black_box((&shared_left, &shared_right));
                black_box(result)
            },
            BatchSize::SmallInput,
        )
    });
    group.bench_function("rational_sub_shared_cold", |b| {
        b.iter_batched(
            || {
                let left = Rational::new(1_000_000_000);
                let right = Rational::try_from(1.0e-9_f64).expect("finite f64 imports exactly");
                let shared_left = left.clone();
                let shared_right = right.clone();
                (left, right, shared_left, shared_right)
            },
            |(left, right, shared_left, shared_right)| {
                let result = black_box(&left) - black_box(&right);
                black_box((&shared_left, &shared_right));
                black_box(result)
            },
            BatchSize::SmallInput,
        )
    });
    group.bench_function("rational_scaled_difference_composed_cold", |b| {
        b.iter_batched(
            || {
                (
                    Rational::from_bigint(BigInt::from(
                        (BigUint::from(1_u8) << 191_usize) + BigUint::from(17_u8),
                    )),
                    Rational::from_bigint(BigInt::from(
                        (BigUint::from(1_u8) << 159_usize) + BigUint::from(29_u8),
                    )),
                )
            },
            |(left, subtractand)| {
                black_box(black_box(&left) - &(black_box(&subtractand) * Rational::new(3)))
            },
            BatchSize::SmallInput,
        )
    });
    group.bench_function("rational_scaled_difference_fused_cold", |b| {
        b.iter_batched(
            || {
                (
                    Rational::from_bigint(BigInt::from(
                        (BigUint::from(1_u8) << 191_usize) + BigUint::from(17_u8),
                    )),
                    Rational::from_bigint(BigInt::from(
                        (BigUint::from(1_u8) << 159_usize) + BigUint::from(29_u8),
                    )),
                )
            },
            |(left, subtractand)| {
                black_box(
                    black_box(&left)
                        .checked_exact_integer_scaled_difference(black_box(&subtractand), 3),
                )
            },
            BatchSize::SmallInput,
        )
    });
    group.bench_function(
        "rational_cross_difference_unit_divisor_composed_cold",
        |b| {
            b.iter_batched(
                || {
                    (
                        Rational::from_bigint(BigInt::from(
                            (BigUint::from(1_u8) << 191_usize) + BigUint::from(17_u8),
                        )),
                        Rational::new(7),
                        Rational::from_bigint(BigInt::from(
                            (BigUint::from(1_u8) << 159_usize) + BigUint::from(29_u8),
                        )),
                        Rational::new(4),
                    )
                },
                |(left, right, subtract_left, subtract_right)| {
                    black_box(
                        &(black_box(&left) * black_box(&right)
                            - &(black_box(&subtract_left) * black_box(&subtract_right)))
                            / Rational::new(-1),
                    )
                },
                BatchSize::SmallInput,
            )
        },
    );
    group.bench_function("rational_cross_difference_unit_divisor_fused_cold", |b| {
        b.iter_batched(
            || {
                (
                    Rational::from_bigint(BigInt::from(
                        (BigUint::from(1_u8) << 191_usize) + BigUint::from(17_u8),
                    )),
                    Rational::new(7),
                    Rational::from_bigint(BigInt::from(
                        (BigUint::from(1_u8) << 159_usize) + BigUint::from(29_u8),
                    )),
                    Rational::new(4),
                )
            },
            |(left, right, subtract_left, subtract_right)| {
                black_box(left.checked_exact_integer_cross_difference_quotient(
                    black_box(&right),
                    black_box(&subtract_left),
                    black_box(&subtract_right),
                    &Rational::new(-1),
                ))
            },
            BatchSize::SmallInput,
        )
    });
    group.bench_function("rational_mul", |b| {
        b.iter(|| black_box(black_box(&lhs) * black_box(&rhs)))
    });
    group.bench_function("rational_mul_retained_general", |b| {
        b.iter(|| black_box(black_box(&retained_lhs) * black_box(&retained_rhs)))
    });
    group.bench_function("rational_mul_wide_dyadic_cold", |b| {
        b.iter_batched(
            || {
                let value = Rational::try_from(1.0e-12_f64).expect("finite f64 imports exactly");
                let negative = -value.clone();
                (value, negative)
            },
            |(left, right)| black_box(black_box(&left) * black_box(&right)),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("rational_mul_dyadic_general_cross_cancel", |b| {
        b.iter(|| black_box(black_box(&dyadic_general_lhs) * black_box(&dyadic_general_rhs)))
    });
    group.bench_function("rational_div", |b| {
        b.iter(|| black_box(black_box(&lhs) / black_box(&rhs)))
    });
    group.bench_function("rational_inverse_owned_cold", |b| {
        b.iter_batched(
            || rational(123_456_789, 987_654_321),
            |value| black_box(value.inverse().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("rational_inverse_retained", |b| {
        b.iter(|| black_box(black_box(retained_inverse_input.clone()).inverse().unwrap()))
    });
    group.bench_function("rational_neg_owned_cold", |b| {
        b.iter_batched(
            || rational(123_456_789, 987_654_321),
            |value| black_box(-value),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("rational_neg_retained", |b| {
        b.iter(|| black_box(-black_box(&retained_negation_input)))
    });
    group.bench_function("real_exact_powi_i64_owned_cold", |b| {
        b.iter_batched(
            || Real::new(rational(123_456_789, 987_654_321)),
            |value| black_box(value.powi_i64(5).unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("real_exact_powi_i64_retained", |b| {
        b.iter(|| black_box(black_box(retained_powi_input.clone()).powi_i64(5).unwrap()))
    });
    group.bench_function("real_exact_add", |b| {
        b.iter(|| black_box(black_box(&exact_real_lhs) + black_box(&exact_real_rhs)))
    });
    group.bench_function("real_exact_average_pair", |b| {
        b.iter(|| {
            black_box(Real::average_pair(
                black_box(&exact_real_lhs),
                black_box(&exact_real_rhs),
            ))
        })
    });
    group.bench_function("real_exact_average_pair_expanded", |b| {
        b.iter(|| {
            black_box(
                ((black_box(&exact_real_lhs) + black_box(&exact_real_rhs)) / Real::from(2_u8))
                    .unwrap(),
            )
        })
    });
    group.bench_function("real_exact_sub", |b| {
        b.iter(|| black_box(black_box(&exact_real_lhs) - black_box(&exact_real_rhs)))
    });
    group.bench_function("real_exact_mul", |b| {
        b.iter(|| black_box(black_box(&exact_real_lhs) * black_box(&exact_real_rhs)))
    });
    group.bench_function("real_exact_mul_retained", |b| {
        b.iter(|| black_box(black_box(&retained_real_lhs) * black_box(&retained_real_rhs)))
    });
    group.bench_function("real_exact_div", |b| {
        b.iter(|| black_box((black_box(&exact_real_lhs) / black_box(&exact_real_rhs)).unwrap()))
    });
    group.bench_function("real_exact_sqrt_owned_cold", |b| {
        b.iter_batched(
            || Real::new(Rational::new(90)),
            |value| black_box(value.sqrt().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("real_exact_sqrt_reduce", |b| {
        b.iter_batched(
            || sqrt_input.clone(),
            |value| black_box(value.sqrt().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("real_exact_dyadic_sqrt_reduce", |b| {
        b.iter_batched(
            || dyadic_sqrt_input.clone(),
            |value| black_box(value.sqrt().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("real_exact_general_sqrt_reduce", |b| {
        b.iter_batched(
            || general_sqrt_input.clone(),
            |value| black_box(value.sqrt().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("real_exact_dyadic_radical_scale", |b| {
        b.iter(|| black_box(black_box(&dyadic_component_real) * black_box(&dyadic_radical_scale)))
    });
    group.bench_function("real_exact_ln_reduce", |b| {
        b.iter_batched(
            || ln_input.clone(),
            |value| black_box(value.ln().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("real_pow_small_integer_exponent", |b| {
        b.iter_batched(
            || (pow_base.clone(), pow_exponent.clone()),
            |(base, exponent)| black_box(base.pow(exponent).unwrap()),
            BatchSize::SmallInput,
        )
    });

    group.finish();
}

/// Build the integer 2^((limbs-1)*usize::BITS) + low as an exact Rational.
///
/// Positive limbs and a nonoverflowing shift calculation are required. Zero limbs
/// underflow, huge inputs can overflow or exhaust allocation, and no limit is checked.
/// The native-word count is a fixture convention, not a portable proof of a selected
/// backend algorithm. At one limb the result is 1+low, including even values.
fn backend_limb_rational(limbs: usize, low: u8) -> Rational {
    let magnitude =
        (BigUint::from(1_u8) << ((limbs - 1) * usize::BITS as usize)) + BigUint::from(low);
    Rational::from_bigint(BigInt::from(magnitude))
}

/// Generate a deterministic positive integer of exactly bits bits from a u64 seed.
///
/// Append ceil(bits/64) wrapping LCG words using multiplier 6364136223846793005
/// and increment 1442695040888963407, keep the low bits bits, then force the top bit.
/// For partial words this retains the low bits of the concatenation. bits must be
/// positive: zero underflows in bits-1. Repeated growing shifts/copies and allocations
/// have no explicit work or memory cap; this is neither a random nor cryptographic
/// source, and seed equality/bit width alone do not establish benchmark comparability.
fn benchmark_magnitude(bits: usize, mut state: u64) -> BigUint {
    let mut value = BigUint::ZERO;
    for _ in 0..bits.div_ceil(64) {
        state = state
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        value = (value << 64_usize) + state;
    }
    let mask = (BigUint::from(1_u8) << bits) - 1_u8;
    (value & mask) | (BigUint::from(1_u8) << (bits - 1))
}

/// Compute a nonnegative integer GCD by descending full-width Euclidean remainders.
///
/// Clone/sort the operands, then repeatedly replace (a,b) with (b,a mod b) until b
/// is zero. gcd(0,0)=0 and gcd(a,0)=a. This benchmark baseline includes input cloning
/// and intermediate allocation; it has no iteration/resource limit or cancellation.
fn euclidean_magnitude_gcd(left: &BigUint, right: &BigUint) -> BigUint {
    let (mut larger, mut smaller) = if left >= right {
        (left.clone(), right.clone())
    } else {
        (right.clone(), left.clone())
    };
    while smaller != BigUint::ZERO {
        let remainder = &larger % &smaller;
        larger = smaller;
        smaller = remainder;
    }
    larger
}

/// Exercise rational facts, wide integer algorithms, perfect powers and exact radix I/O.
///
/// Setup asserts non-dyadic 7/15, two rational orderings, eight equality cases,
/// candidate products against BigUint, selected unbalanced products, selected GCDs
/// against Euclid, half-GCD against Lehmer, and Barrett div/rem against backend
/// results. Assertions and generated inputs execute even for filtered-out rows.
/// Timing results themselves are generally unchecked, and no dispatch counts are
/// asserted. Limb-shaped fixtures depend on usize width; labels are not path proofs.
///
/// Balanced products range from 4096 to 4194304 bits. Toom-4, Toom-6, Toom-8 and
/// NTT use distinct seed pairs. Several later candidates reuse earlier named
/// backend/selected baselines at the same width but with different operands; NTT
/// has no same-seed timed baseline. The 131072-bit backend_reference row actually
/// calls the production selector. Unbalanced selected/backend pairs do share inputs.
///
/// GCD workloads use 128..4096-bit balanced values, constructed unbalanced dividends
/// whose first remainder is smaller than the divisor, and 8192..1048576-bit
/// half-GCD/Lehmer pairs. Barrett one-shot includes preparation; each batch call
/// prepares once for sixteen dividends and returns the vector of quotient/remainder
/// pairs, including preparation and materialization in timing.
///
/// Other rows construct fresh reduced fractions, take a retained fractional part,
/// compute a zero quotient, query perfect powers of 12 or 101/103 and a mismatched
/// seventh/fifth power, and parse/format exact decimals. Scientific and expanded
/// 5120-digit fixtures denote the same value; 10240/20480-digit parse comparisons
/// return Rational versus BigUint, with distinct representation costs. Formatting
/// 1/7 to 32 decimal places is approximate display, not exact replay. Predicate and
/// formatting results are not generally checked. Large fixed workloads have no
/// internal time/memory cap; Criterion filters do not contain their setup cost.
fn bench_rational_algorithm_dispatch_speed(c: &mut Criterion) {
    let mut group = c.benchmark_group("rational_algorithm_dispatch_speed");

    group.bench_function("dyadic_fact_cold", |b| {
        b.iter_batched(
            || Rational::fraction(7, 15).unwrap(),
            |value| black_box(value.is_dyadic()),
            BatchSize::SmallInput,
        )
    });
    let retained_non_dyadic = Rational::fraction(7, 15).unwrap();
    assert!(!retained_non_dyadic.is_dyadic());
    group.bench_function("dyadic_fact_retained", |b| {
        b.iter(|| black_box(black_box(&retained_non_dyadic).is_dyadic()))
    });

    let scale = BigUint::from(1_u8) << 1024_usize;
    let compare_left = Rational::from_bigint_fraction(
        BigInt::from(&scale + (&scale >> 2_usize) + 3_u8),
        &scale + 65_537_u32,
    )
    .unwrap();
    let compare_right = Rational::from_bigint_fraction(
        BigInt::from(&scale + (&scale >> 3_usize) + 5_u8),
        &scale + 131_071_u32,
    )
    .unwrap();
    assert_eq!(
        compare_left.partial_cmp(&compare_right),
        Some(std::cmp::Ordering::Greater)
    );
    group.bench_function("compare_leading_significand_retained_1024_bits", |b| {
        b.iter(|| black_box(black_box(&compare_left).partial_cmp(black_box(&compare_right))))
    });

    let dyadic_left_scale = BigUint::from(1_u8) << 1023_usize;
    let dyadic_right_scale = BigUint::from(1_u8) << 1018_usize;
    let dyadic_left = Rational::from_bigint_fraction(
        BigInt::from(&dyadic_left_scale + (&dyadic_left_scale >> 2_usize) + 3_u8),
        BigUint::from(1_u8) << 40_usize,
    )
    .unwrap();
    let dyadic_right = Rational::from_bigint_fraction(
        BigInt::from(&dyadic_right_scale + (&dyadic_right_scale >> 3_usize) + 5_u8),
        BigUint::from(1_u8) << 35_usize,
    )
    .unwrap();
    assert_eq!(
        dyadic_left.partial_cmp(&dyadic_right),
        Some(std::cmp::Ordering::Greater)
    );
    group.bench_function("compare_dyadic_shifted_retained_1024_bits", |b| {
        b.iter(|| black_box(black_box(&dyadic_left).partial_cmp(black_box(&dyadic_right))))
    });

    for (name, left, right, expected) in [
        (
            "equality_leading_significand_retained_1024_bits",
            compare_left.clone(),
            compare_right,
            false,
        ),
        (
            "equality_dyadic_shifted_retained_1024_bits",
            dyadic_left,
            dyadic_right,
            false,
        ),
        (
            "equality_shared_identity_retained",
            compare_left.clone(),
            compare_left.clone(),
            true,
        ),
        (
            "equality_same_denominator_retained",
            Rational::fraction(7, 13).unwrap(),
            Rational::fraction(8, 13).unwrap(),
            false,
        ),
        (
            "equality_word_retained",
            Rational::fraction(7, 15).unwrap(),
            Rational::fraction(8, 17).unwrap(),
            false,
        ),
        (
            "equality_equal_distinct_retained",
            Rational::fraction(7, 13).unwrap(),
            Rational::fraction(7, 13).unwrap(),
            true,
        ),
        (
            "equality_different_signs_retained",
            compare_left.clone(),
            -compare_left,
            false,
        ),
        (
            "equality_close_retained_1024_bits",
            Rational::from_bigint_fraction(BigInt::from(&scale + 1_u8), &scale + 3_u8).unwrap(),
            Rational::from_bigint_fraction(BigInt::from(&scale + 3_u8), &scale + 5_u8).unwrap(),
            false,
        ),
    ] {
        assert_eq!(left == right, expected);
        group.bench_function(name, |b| {
            b.iter(|| black_box(black_box(&left) == black_box(&right)))
        });
    }

    for (name, left_limbs, right_limbs) in [
        ("mul_backend_basecase_cold", 16, 16),
        ("mul_backend_half_karatsuba_cold", 33, 66),
        ("mul_backend_karatsuba_cold", 40, 40),
        ("mul_backend_toom3_cold", 257, 257),
    ] {
        group.bench_function(name, |b| {
            b.iter_batched(
                || {
                    (
                        backend_limb_rational(left_limbs, 3),
                        backend_limb_rational(right_limbs, 5),
                    )
                },
                |(left, right)| black_box(black_box(&left) * black_box(&right)),
                BatchSize::SmallInput,
            )
        });
    }

    for (bits, candidate_name, backend_name) in [
        (
            4096,
            "mul_toom4_candidate_4096_bits",
            "mul_backend_reference_4096_bits",
        ),
        (
            16_384,
            "mul_toom4_candidate_16384_bits",
            "mul_backend_reference_16384_bits",
        ),
        (
            65_536,
            "mul_toom4_candidate_65536_bits",
            "mul_backend_reference_65536_bits",
        ),
        (
            262_144,
            "mul_toom4_candidate_262144_bits",
            "mul_backend_reference_262144_bits",
        ),
        (
            524_288,
            "mul_toom4_candidate_524288_bits",
            "mul_backend_reference_524288_bits",
        ),
        (
            1_048_576,
            "mul_toom4_candidate_1048576_bits",
            "mul_backend_reference_1048576_bits",
        ),
        (
            2_097_152,
            "mul_toom4_candidate_2097152_bits",
            "mul_backend_reference_2097152_bits",
        ),
    ] {
        let left = benchmark_magnitude(bits, 0x6a09_e667_f3bc_c909);
        let right = benchmark_magnitude(bits, 0xbb67_ae85_84ca_a73b);
        assert_eq!(
            Rational::multiply_magnitudes_toom4_candidate(&left, &right),
            &left * &right
        );
        group.bench_function(candidate_name, |b| {
            b.iter(|| {
                black_box(Rational::multiply_magnitudes_toom4_candidate(
                    black_box(&left),
                    black_box(&right),
                ))
            })
        });
        group.bench_function(backend_name, |b| {
            b.iter(|| black_box(black_box(&left) * black_box(&right)))
        });
        if bits >= 1_048_576 {
            let selected_name = if bits == 1_048_576 {
                "mul_selected_1048576_bits"
            } else {
                "mul_selected_2097152_bits"
            };
            group.bench_function(selected_name, |b| {
                b.iter(|| {
                    black_box(Rational::multiply_magnitudes_selected(
                        black_box(&left),
                        black_box(&right),
                    ))
                })
            });
        }
    }

    for (bits, candidate_name, selected_name) in [
        (
            131_072,
            "mul_toom6_candidate_131072_bits",
            "mul_backend_reference_131072_bits",
        ),
        (
            262_144,
            "mul_toom6_candidate_262144_bits",
            "mul_backend_reference_262144_bits",
        ),
        (
            524_288,
            "mul_toom6_candidate_524288_bits",
            "mul_selected_524288_bits",
        ),
        (
            1_048_576,
            "mul_toom6_candidate_1048576_bits",
            "mul_selected_1048576_bits",
        ),
        (
            2_097_152,
            "mul_toom6_candidate_2097152_bits",
            "mul_selected_2097152_bits",
        ),
    ] {
        // The first two selected baselines are already registered above.
        let left = benchmark_magnitude(bits, 0x5be0_cd19_137e_2179);
        let right = benchmark_magnitude(bits, 0x1f83_d9ab_fb41_bd6b);
        assert_eq!(
            Rational::multiply_magnitudes_toom6_candidate(&left, &right),
            &left * &right
        );
        group.bench_function(candidate_name, |b| {
            b.iter(|| {
                black_box(Rational::multiply_magnitudes_toom6_candidate(
                    black_box(&left),
                    black_box(&right),
                ))
            })
        });
        if bits == 131_072 || bits == 524_288 {
            group.bench_function(selected_name, |b| {
                b.iter(|| {
                    black_box(Rational::multiply_magnitudes_selected(
                        black_box(&left),
                        black_box(&right),
                    ))
                })
            });
        }
    }

    let toom4_shorter_bits = 1_048_576;
    let toom4_longer_bits = toom4_shorter_bits + toom4_shorter_bits / 5;
    let toom4_unbalanced_left = benchmark_magnitude(toom4_longer_bits, 0xa54f_f53a_5f1d_36f1);
    let toom4_unbalanced_right = benchmark_magnitude(toom4_shorter_bits, 0x3c6e_f372_fe94_f82b);
    assert!(
        Rational::multiply_magnitudes_selected(&toom4_unbalanced_left, &toom4_unbalanced_right)
            == &toom4_unbalanced_left * &toom4_unbalanced_right
    );
    group.bench_function("mul_selected_toom4_unbalanced_1258291_by_1048576", |b| {
        b.iter(|| {
            black_box(Rational::multiply_magnitudes_selected(
                black_box(&toom4_unbalanced_left),
                black_box(&toom4_unbalanced_right),
            ))
        })
    });
    group.bench_function("mul_backend_unbalanced_1258291_by_1048576", |b| {
        b.iter(|| black_box(black_box(&toom4_unbalanced_left) * black_box(&toom4_unbalanced_right)))
    });

    for (bits, candidate_name, selected_name) in [
        (
            65_536,
            "mul_toom8_candidate_65536_bits",
            "mul_backend_reference_65536_bits",
        ),
        (
            131_072,
            "mul_toom8_candidate_131072_bits",
            "mul_backend_reference_131072_bits",
        ),
        (
            262_144,
            "mul_toom8_candidate_262144_bits",
            "mul_selected_262144_bits",
        ),
        (
            524_288,
            "mul_toom8_candidate_524288_bits",
            "mul_selected_524288_bits",
        ),
        (
            1_048_576,
            "mul_toom8_candidate_1048576_bits",
            "mul_selected_1048576_bits",
        ),
        (
            2_097_152,
            "mul_toom8_candidate_2097152_bits",
            "mul_selected_2097152_bits",
        ),
        (
            4_194_304,
            "mul_toom8_candidate_4194304_bits",
            "mul_selected_4194304_bits",
        ),
    ] {
        let left = benchmark_magnitude(bits, 0xcbbb_9d5d_c105_9ed8);
        let right = benchmark_magnitude(bits, 0x629a_292a_367c_d507);
        assert_eq!(
            Rational::multiply_magnitudes_toom8_candidate(&left, &right),
            &left * &right
        );
        group.bench_function(candidate_name, |b| {
            b.iter(|| {
                black_box(Rational::multiply_magnitudes_toom8_candidate(
                    black_box(&left),
                    black_box(&right),
                ))
            })
        });
        if bits == 262_144 || bits == 4_194_304 {
            group.bench_function(selected_name, |b| {
                b.iter(|| {
                    black_box(Rational::multiply_magnitudes_selected(
                        black_box(&left),
                        black_box(&right),
                    ))
                })
            });
        }
    }

    let toom6_shorter_bits = 524_288;
    let toom6_longer_bits = toom6_shorter_bits + toom6_shorter_bits / 7;
    let toom6_unbalanced_left = benchmark_magnitude(toom6_longer_bits, 0x6c44_198c_4a47_5817);
    let toom6_unbalanced_right = benchmark_magnitude(toom6_shorter_bits, 0x7f4a_7c15_f39c_c060);
    assert!(
        Rational::multiply_magnitudes_selected(&toom6_unbalanced_left, &toom6_unbalanced_right)
            == &toom6_unbalanced_left * &toom6_unbalanced_right
    );
    group.bench_function("mul_selected_toom6_unbalanced_599186_by_524288", |b| {
        b.iter(|| {
            black_box(Rational::multiply_magnitudes_selected(
                black_box(&toom6_unbalanced_left),
                black_box(&toom6_unbalanced_right),
            ))
        })
    });
    group.bench_function("mul_backend_unbalanced_599186_by_524288", |b| {
        b.iter(|| black_box(black_box(&toom6_unbalanced_left) * black_box(&toom6_unbalanced_right)))
    });

    for (bits, candidate_name) in [
        (262_144, "mul_ntt_candidate_262144_bits"),
        (1_048_576, "mul_ntt_candidate_1048576_bits"),
        (4_194_304, "mul_ntt_candidate_4194304_bits"),
    ] {
        let left = benchmark_magnitude(bits, 0x9159_015a_3070_dd17);
        let right = benchmark_magnitude(bits, 0x152f_ecd8_f70e_5939);
        assert_eq!(
            Rational::multiply_magnitudes_ntt_candidate(&left, &right),
            &left * &right
        );
        group.bench_function(candidate_name, |b| {
            b.iter(|| {
                black_box(Rational::multiply_magnitudes_ntt_candidate(
                    black_box(&left),
                    black_box(&right),
                ))
            })
        });
    }

    let single_limb_magnitude = (BigUint::from(1_u8) << (9 * usize::BITS as usize)) * 3_u8;
    group.bench_function("reduce_backend_single_limb_cold", |b| {
        b.iter_batched(
            || (single_limb_magnitude.clone(), BigUint::from(3_u8)),
            |(numerator, denominator)| {
                black_box(
                    Rational::from_bigint_fraction(BigInt::from(numerator), denominator).unwrap(),
                )
            },
            BatchSize::SmallInput,
        )
    });

    let knuth_common = (BigUint::from(1_u8) << (9 * usize::BITS as usize)) + 1_u8;
    group.bench_function("reduce_backend_knuth_cold", |b| {
        b.iter_batched(
            || (&knuth_common * 3_u8, &knuth_common * 5_u8),
            |(numerator, denominator)| {
                black_box(
                    Rational::from_bigint_fraction(BigInt::from(numerator), denominator).unwrap(),
                )
            },
            BatchSize::SmallInput,
        )
    });

    let large_knuth_common = (BigUint::from(1_u8) << (64 * usize::BITS as usize)) + 1_u8;
    let large_knuth_factor = (BigUint::from(1_u8) << (64 * usize::BITS as usize)) + 2_u8;
    group.bench_function("reduce_backend_large_knuth_cold", |b| {
        b.iter_batched(
            || {
                (
                    &large_knuth_common * &large_knuth_factor,
                    &large_knuth_common * 3_u8,
                )
            },
            |(numerator, denominator)| {
                black_box(
                    Rational::from_bigint_fraction(BigInt::from(numerator), denominator).unwrap(),
                )
            },
            BatchSize::SmallInput,
        )
    });

    let fixed_512_left = benchmark_magnitude(512, 0x243f_6a88_85a3_08d3);
    let fixed_512_right = benchmark_magnitude(512, 0xa409_3822_299f_31d0);
    group.bench_function("reduce_fixed_512_coprime_cold", |b| {
        b.iter_batched(
            || (fixed_512_left.clone(), fixed_512_right.clone()),
            |(numerator, denominator)| {
                black_box(
                    Rational::from_bigint_fraction(BigInt::from(numerator), denominator).unwrap(),
                )
            },
            BatchSize::SmallInput,
        )
    });

    let remainder_denominator = (BigUint::from(1_u8) << (64 * usize::BITS as usize)) + 3_u8;
    let remainder_numerator = (&remainder_denominator << (64 * usize::BITS as usize)) + 17_u8;
    let remainder_rational =
        Rational::from_bigint_fraction(BigInt::from(remainder_numerator), remainder_denominator)
            .unwrap();
    group.bench_function("exact_remainder_large_knuth", |b| {
        b.iter(|| black_box(black_box(&remainder_rational).fract()))
    });

    let small_quotient_divisor = benchmark_magnitude(8192, 0x510e_527f_ade6_82d1);
    let small_quotient_dividend = &small_quotient_divisor - 1_u8;
    group.bench_function("division_trivial_small_quotient", |b| {
        b.iter(|| {
            black_box(black_box(&small_quotient_dividend) / black_box(&small_quotient_divisor))
        })
    });

    for (bits, selected_name, euclidean_name) in [
        (128, "gcd_selected_128_bits", "gcd_euclidean_128_bits"),
        (192, "gcd_selected_192_bits", "gcd_euclidean_192_bits"),
        (512, "gcd_selected_512_bits", "gcd_euclidean_512_bits"),
        (1024, "gcd_selected_1024_bits", "gcd_euclidean_1024_bits"),
        (4096, "gcd_selected_4096_bits", "gcd_euclidean_4096_bits"),
    ] {
        let left = benchmark_magnitude(bits, 0x243f_6a88_85a3_08d3);
        let right = benchmark_magnitude(bits, 0xa409_3822_299f_31d0);
        let (left, right) = if bits == 128 && left > right {
            (right, left)
        } else {
            (left, right)
        };
        assert_eq!(
            Rational::gcd_magnitudes(&left, &right),
            euclidean_magnitude_gcd(&left, &right)
        );
        group.bench_function(selected_name, |b| {
            b.iter(|| {
                black_box(Rational::gcd_magnitudes(
                    black_box(&left),
                    black_box(&right),
                ))
            })
        });
        group.bench_function(euclidean_name, |b| {
            b.iter(|| black_box(euclidean_magnitude_gcd(black_box(&left), black_box(&right))))
        });
    }

    for (bits, selected_name, euclidean_name) in [
        (
            192,
            "gcd_selected_unbalanced_to_lehmer_192_bits",
            "gcd_euclidean_unbalanced_to_lehmer_192_bits",
        ),
        (
            256,
            "gcd_selected_unbalanced_to_lehmer_256_bits",
            "gcd_euclidean_unbalanced_to_lehmer_256_bits",
        ),
        (
            512,
            "gcd_selected_unbalanced_to_lehmer_512_bits",
            "gcd_euclidean_unbalanced_to_lehmer_512_bits",
        ),
        (
            1024,
            "gcd_selected_unbalanced_to_lehmer_1024_bits",
            "gcd_euclidean_unbalanced_to_lehmer_1024_bits",
        ),
        (
            4096,
            "gcd_selected_unbalanced_to_lehmer_4096_bits",
            "gcd_euclidean_unbalanced_to_lehmer_4096_bits",
        ),
    ] {
        let divisor = benchmark_magnitude(bits, 0x3c6e_f372_fe94_f82b);
        let remainder = benchmark_magnitude(bits - 1, 0xbb67_ae85_84ca_a73b);
        let dividend = &divisor * (BigUint::from(1_u8) << bits) + &remainder;
        assert!(remainder < divisor);
        assert_eq!(
            Rational::gcd_magnitudes(&dividend, &divisor),
            euclidean_magnitude_gcd(&dividend, &divisor)
        );
        group.bench_function(selected_name, |b| {
            b.iter(|| {
                black_box(Rational::gcd_magnitudes(
                    black_box(&dividend),
                    black_box(&divisor),
                ))
            })
        });
        group.bench_function(euclidean_name, |b| {
            b.iter(|| {
                black_box(euclidean_magnitude_gcd(
                    black_box(&dividend),
                    black_box(&divisor),
                ))
            })
        });
    }

    for (bits, selected_name, lehmer_name) in [
        (
            8192,
            "half_gcd_candidate_8192_bits",
            "half_gcd_lehmer_8192_bits",
        ),
        (
            16_384,
            "half_gcd_candidate_16384_bits",
            "half_gcd_lehmer_16384_bits",
        ),
        (
            65_536,
            "half_gcd_candidate_65536_bits",
            "half_gcd_lehmer_65536_bits",
        ),
        (
            262_144,
            "half_gcd_candidate_262144_bits",
            "half_gcd_lehmer_262144_bits",
        ),
        (
            1_048_576,
            "half_gcd_candidate_1048576_bits",
            "half_gcd_lehmer_1048576_bits",
        ),
    ] {
        let left = benchmark_magnitude(bits, 0x1319_8a2e_0370_7344);
        let right = benchmark_magnitude(bits, 0x082e_fa98_ec4e_6c89);
        assert_eq!(
            Rational::gcd_magnitudes_half_gcd_candidate(&left, &right),
            Rational::gcd_magnitudes_lehmer_baseline(&left, &right)
        );
        group.bench_function(selected_name, |b| {
            b.iter(|| {
                black_box(Rational::gcd_magnitudes_half_gcd_candidate(
                    black_box(&left),
                    black_box(&right),
                ))
            })
        });
        group.bench_function(lehmer_name, |b| {
            b.iter(|| {
                black_box(Rational::gcd_magnitudes_lehmer_baseline(
                    black_box(&left),
                    black_box(&right),
                ))
            })
        });
    }

    let barrett_divisor_1024 = benchmark_magnitude(1024, 0x3bd3_9e10_cb0e_f593);
    let barrett_dividend_8192 = benchmark_magnitude(8192, 0xc0ac_29b7_c97c_50dd);
    assert_eq!(
        Rational::div_rem_magnitudes_barrett_candidate(
            &barrett_dividend_8192,
            &barrett_divisor_1024,
        ),
        barrett_dividend_8192.div_rem(&barrett_divisor_1024)
    );
    group.bench_function("barrett_one_shot_8192_by_1024", |b| {
        b.iter(|| {
            black_box(Rational::div_rem_magnitudes_barrett_candidate(
                black_box(&barrett_dividend_8192),
                black_box(&barrett_divisor_1024),
            ))
        })
    });
    group.bench_function("backend_one_shot_8192_by_1024", |b| {
        b.iter(|| {
            black_box(black_box(&barrett_dividend_8192).div_rem(black_box(&barrett_divisor_1024)))
        })
    });

    for (divisor_bits, dividend_bits, barrett_name, backend_name) in [
        (
            1024,
            8192,
            "barrett_batch16_8192_by_1024",
            "backend_batch16_8192_by_1024",
        ),
        (
            4096,
            65_536,
            "barrett_batch16_65536_by_4096",
            "backend_batch16_65536_by_4096",
        ),
    ] {
        let divisor = benchmark_magnitude(divisor_bits, 0x9b05_688c_2b3e_6c1f);
        let dividends: Vec<_> = (0_u64..16)
            .map(|index| {
                benchmark_magnitude(
                    dividend_bits,
                    0x1f83_d9ab_fb41_bd6b ^ index.wrapping_mul(0x9e37_79b9),
                )
            })
            .collect();
        assert_eq!(
            Rational::div_rem_magnitudes_barrett_batch_candidate(&dividends, &divisor),
            Rational::div_rem_magnitudes_backend_batch(&dividends, &divisor)
        );
        group.bench_function(barrett_name, |b| {
            b.iter(|| {
                black_box(Rational::div_rem_magnitudes_barrett_batch_candidate(
                    black_box(&dividends),
                    black_box(&divisor),
                ))
            })
        });
        group.bench_function(backend_name, |b| {
            b.iter(|| {
                black_box(Rational::div_rem_magnitudes_backend_batch(
                    black_box(&dividends),
                    black_box(&divisor),
                ))
            })
        });
    }

    let perfect_power_reject = Rational::new(12);
    let perfect_power_seventh = Rational::fraction(101_i64.pow(7), 103_u64.pow(7)).unwrap();
    let perfect_power_mismatch = Rational::fraction(101_i64.pow(7), 103_u64.pow(5)).unwrap();
    group.bench_function("perfect_power_factor_reject", |b| {
        b.iter(|| black_box(black_box(&perfect_power_reject).is_perfect_power()))
    });
    group.bench_function("perfect_power_general_seventh", |b| {
        b.iter(|| black_box(black_box(&perfect_power_seventh).is_perfect_power()))
    });
    group.bench_function("perfect_power_fixed_seventh", |b| {
        b.iter(|| black_box(black_box(&perfect_power_seventh).perfect_nth_root(7)))
    });
    group.bench_function("perfect_power_unfactored_reject", |b| {
        b.iter(|| black_box(black_box(&perfect_power_mismatch).is_perfect_power()))
    });

    let small_radix = backend_limb_rational(16, 3);
    let large_radix = backend_limb_rational(32, 17);
    let large_radix_decimal = large_radix.to_string();
    let short_radix_decimal = "-12345.678901";
    let short_radix_scientific = "-7.78437e-005";
    let wide_scientific_digits = "1234567890".repeat(512);
    let wide_radix_scientific = format!("{wide_scientific_digits}e-4096");
    let wide_radix_scientific_expanded = format!(
        "{}.{}",
        &wide_scientific_digits[..1024],
        &wide_scientific_digits[1024..]
    );
    let divide_conquer_decimal_10240 = "1234567890".repeat(1024);
    let divide_conquer_decimal_20480 = "1234567890".repeat(2048);
    let decimal_fraction = Rational::fraction(1, 7).unwrap();
    group.bench_function("radix_format_small_integer", |b| {
        b.iter(|| black_box(format!("{}", black_box(&small_radix))))
    });
    group.bench_function("radix_format_large_integer", |b| {
        b.iter(|| black_box(format!("{}", black_box(&large_radix))))
    });
    group.bench_function("radix_parse_short_decimal", |b| {
        b.iter(|| black_box(black_box(short_radix_decimal).parse::<Rational>().unwrap()))
    });
    group.bench_function("radix_parse_short_scientific", |b| {
        b.iter(|| {
            black_box(
                black_box(short_radix_scientific)
                    .parse::<Rational>()
                    .unwrap(),
            )
        })
    });
    group.bench_function("radix_parse_wide_scientific", |b| {
        b.iter(|| {
            black_box(
                black_box(&wide_radix_scientific)
                    .parse::<Rational>()
                    .unwrap(),
            )
        })
    });
    group.bench_function("radix_parse_wide_scientific_expanded", |b| {
        b.iter(|| {
            black_box(
                black_box(&wide_radix_scientific_expanded)
                    .parse::<Rational>()
                    .unwrap(),
            )
        })
    });
    group.bench_function("radix_parse_large_integer", |b| {
        b.iter(|| black_box(black_box(&large_radix_decimal).parse::<Rational>().unwrap()))
    });
    for (digits, divide_conquer_name, backend_name) in [
        (
            &divide_conquer_decimal_10240,
            "radix_parse_divide_conquer_10240_digits",
            "radix_parse_backend_chunked_10240_digits",
        ),
        (
            &divide_conquer_decimal_20480,
            "radix_parse_divide_conquer_20480_digits",
            "radix_parse_backend_chunked_20480_digits",
        ),
    ] {
        group.bench_function(divide_conquer_name, |b| {
            b.iter(|| black_box(black_box(digits).parse::<Rational>().unwrap()))
        });
        group.bench_function(backend_name, |b| {
            b.iter(|| black_box(BigUint::parse_bytes(black_box(digits.as_bytes()), 10).unwrap()))
        });
    }
    group.bench_function("radix_format_fraction_decimal", |b| {
        b.iter(|| black_box(format!("{:#.32}", black_box(&decimal_fraction))))
    });

    group.finish();
}

/// Compare cloned/owned and borrowed addition, then symbolic dot products of two to four lanes.
///
/// Rationals use a reciprocal pair; Real additions use pi/e and scales 7/8, 5/6.
/// Dense dot factors combine pi, e, sqrt(2) and their products, while mixed arrays
/// include exact zero/one/integer lanes. Active variants receive the dense inputs
/// without a separate timed activity-classification pass. All outputs are constructed
/// and black-boxed without numeric or equality checks. Owned addition excludes setup
/// clones and returned-output drops; ordinary borrowed/clone rows include their drops.
/// Prepared operands and clone-shared evidence persist across iterations.
fn bench_borrowed_op_overhead(c: &mut Criterion) {
    let mut group = c.benchmark_group("borrowed_op_overhead");
    let rational_lhs = rational(123_456_789, 987_654_321);
    let rational_rhs = rational(987_654_321, 123_456_789);
    let real_unscaled_lhs = Real::pi();
    let real_unscaled_rhs = Real::e();
    let real_lhs = Real::pi() * real(7, 8);
    let real_rhs = Real::e() * real(5, 6);
    let sqrt_two = Real::from(2_i32).sqrt().unwrap();
    let dense_dot_left = [
        Real::pi() * real(7, 8),
        Real::e() * real(5, 6),
        sqrt_two.clone() * real(11, 13),
        (Real::pi() * Real::e()) * real(3, 5),
    ];
    let dense_dot_right = [
        Real::e() * real(2, 7),
        sqrt_two.clone() * real(17, 19),
        Real::pi() * real(23, 29),
        (Real::pi() * sqrt_two.clone()) * real(31, 37),
    ];
    let mixed_dot_left = [
        Real::one(),
        Real::zero(),
        Real::from(2_i32),
        Real::pi() * real(5, 7),
    ];
    let mixed_dot_right = [Real::pi(), Real::e(), Real::e() * real(3, 5), Real::zero()];

    group.bench_function("rational_clone_pair", |b| {
        b.iter(|| {
            black_box((
                black_box(&rational_lhs).clone(),
                black_box(&rational_rhs).clone(),
            ))
        })
    });
    group.bench_function("rational_add_refs", |b| {
        b.iter(|| black_box(black_box(&rational_lhs) + black_box(&rational_rhs)))
    });
    group.bench_function("rational_add_owned", |b| {
        b.iter_batched(
            || (rational_lhs.clone(), rational_rhs.clone()),
            |(left, right)| black_box(left + right),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("real_clone_pair", |b| {
        b.iter(|| black_box((black_box(&real_lhs).clone(), black_box(&real_rhs).clone())))
    });
    group.bench_function("real_unscaled_add_refs", |b| {
        b.iter(|| black_box(black_box(&real_unscaled_lhs) + black_box(&real_unscaled_rhs)))
    });
    group.bench_function("real_unscaled_add_owned", |b| {
        b.iter_batched(
            || (real_unscaled_lhs.clone(), real_unscaled_rhs.clone()),
            |(left, right)| black_box(left + right),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("real_add_refs", |b| {
        b.iter(|| black_box(black_box(&real_lhs) + black_box(&real_rhs)))
    });
    group.bench_function("real_add_owned", |b| {
        b.iter_batched(
            || (real_lhs.clone(), real_rhs.clone()),
            |(left, right)| black_box(left + right),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("real_dot2_refs_dense_symbolic", |b| {
        b.iter(|| {
            black_box(Real::dot2_refs(
                [black_box(&dense_dot_left[0]), black_box(&dense_dot_left[1])],
                [
                    black_box(&dense_dot_right[0]),
                    black_box(&dense_dot_right[1]),
                ],
            ))
        })
    });
    group.bench_function("real_active_dot2_refs_dense_symbolic", |b| {
        b.iter(|| {
            black_box(Real::active_dot2_refs(
                [black_box(&dense_dot_left[0]), black_box(&dense_dot_left[1])],
                [
                    black_box(&dense_dot_right[0]),
                    black_box(&dense_dot_right[1]),
                ],
            ))
        })
    });
    group.bench_function("real_dot2_refs_mixed_structural", |b| {
        b.iter(|| {
            black_box(Real::dot2_refs(
                [black_box(&mixed_dot_left[0]), black_box(&mixed_dot_left[1])],
                [
                    black_box(&mixed_dot_right[0]),
                    black_box(&mixed_dot_right[1]),
                ],
            ))
        })
    });
    group.bench_function("real_dot3_refs_dense_symbolic", |b| {
        b.iter(|| {
            black_box(Real::dot3_refs(
                [
                    black_box(&dense_dot_left[0]),
                    black_box(&dense_dot_left[1]),
                    black_box(&dense_dot_left[2]),
                ],
                [
                    black_box(&dense_dot_right[0]),
                    black_box(&dense_dot_right[1]),
                    black_box(&dense_dot_right[2]),
                ],
            ))
        })
    });
    group.bench_function("real_active_dot3_refs_dense_symbolic", |b| {
        b.iter(|| {
            black_box(Real::active_dot3_refs(
                [
                    black_box(&dense_dot_left[0]),
                    black_box(&dense_dot_left[1]),
                    black_box(&dense_dot_left[2]),
                ],
                [
                    black_box(&dense_dot_right[0]),
                    black_box(&dense_dot_right[1]),
                    black_box(&dense_dot_right[2]),
                ],
            ))
        })
    });
    group.bench_function("real_dot3_refs_mixed_structural", |b| {
        b.iter(|| {
            black_box(Real::dot3_refs(
                [
                    black_box(&mixed_dot_left[0]),
                    black_box(&mixed_dot_left[1]),
                    black_box(&mixed_dot_left[2]),
                ],
                [
                    black_box(&mixed_dot_right[0]),
                    black_box(&mixed_dot_right[1]),
                    black_box(&mixed_dot_right[2]),
                ],
            ))
        })
    });
    group.bench_function("real_dot4_refs_dense_symbolic", |b| {
        b.iter(|| {
            black_box(Real::dot4_refs(
                [
                    black_box(&dense_dot_left[0]),
                    black_box(&dense_dot_left[1]),
                    black_box(&dense_dot_left[2]),
                    black_box(&dense_dot_left[3]),
                ],
                [
                    black_box(&dense_dot_right[0]),
                    black_box(&dense_dot_right[1]),
                    black_box(&dense_dot_right[2]),
                    black_box(&dense_dot_right[3]),
                ],
            ))
        })
    });
    group.bench_function("real_active_dot4_refs_dense_symbolic", |b| {
        b.iter(|| {
            black_box(Real::active_dot4_refs(
                [
                    black_box(&dense_dot_left[0]),
                    black_box(&dense_dot_left[1]),
                    black_box(&dense_dot_left[2]),
                    black_box(&dense_dot_left[3]),
                ],
                [
                    black_box(&dense_dot_right[0]),
                    black_box(&dense_dot_right[1]),
                    black_box(&dense_dot_right[2]),
                    black_box(&dense_dot_right[3]),
                ],
            ))
        })
    });
    group.bench_function("real_dot4_refs_mixed_structural", |b| {
        b.iter(|| {
            black_box(Real::dot4_refs(
                [
                    black_box(&mixed_dot_left[0]),
                    black_box(&mixed_dot_left[1]),
                    black_box(&mixed_dot_left[2]),
                    black_box(&mixed_dot_left[3]),
                ],
                [
                    black_box(&mixed_dot_right[0]),
                    black_box(&mixed_dot_right[1]),
                    black_box(&mixed_dot_right[2]),
                    black_box(&mixed_dot_right[3]),
                ],
            ))
        })
    });

    group.finish();
}

/// Register fixed rational/symbolic dots, matrix products and 64/1024-term aggregate controls.
///
/// Rational vectors have 64 terms n/(n mod 11+2), n=1..64, and n/(n mod 13+2),
/// n=65..128. The 36-term Real vectors scale pi and e by n/(n mod 7+2) and
/// n/(n mod 5+2). Helpers also treat these arrays as row-major matrices.
///
/// Symbolic sums use sqrt(n) for n=2..65 or 2..1025, including perfect squares.
/// The rational sum alternates signs with numerator index mod 97+1 and denominator
/// 1009+2*index over index=0..1023. Sequential folds and public aggregates construct
/// results; rows ending to_f64 additionally request lossy approximation. Those names
/// do not imply certified floating-point accuracy, and no equivalence is asserted.
///
/// Large approximation rows rebuild children in batched setup, while other rows reuse
/// prepared children. Fresh roots are not a global cache reset. Owned controls clone
/// the input vector outside timing; the former path adds one clone/drop per element
/// inside the routine. Follow the module's distinct ordinary/batched drop contracts.
fn bench_dense_algebra(c: &mut Criterion) {
    let mut group = c.benchmark_group("dense_algebra");
    let rational_left: Vec<_> = (1..=64).map(|n| rational(n, (n as u64 % 11) + 2)).collect();
    let rational_right: Vec<_> = (65..=128)
        .map(|n| rational(n, (n as u64 % 13) + 2))
        .collect();
    let real_left: Vec<_> = (1..=36)
        .map(|n| Real::pi() * real(n, (n as u64 % 7) + 2))
        .collect();
    let real_right: Vec<_> = (37..=72)
        .map(|n| Real::e() * real(n, (n as u64 % 5) + 2))
        .collect();
    let symbolic_sum: Vec<_> = (2..=65)
        .map(|n| Real::from(n).sqrt().expect("positive radicand"))
        .collect();
    let large_symbolic_sum: Vec<_> = (2..=1_025)
        .map(|n| Real::from(n).sqrt().expect("positive radicand"))
        .collect();
    let large_rational_sum: Vec<_> = (0_u64..1_024)
        .map(|index| {
            let magnitude = i64::try_from(index % 97 + 1).expect("small numerator fits i64");
            let numerator = if index % 2 == 0 {
                magnitude
            } else {
                -magnitude
            };
            real(numerator, 1_009 + 2 * index)
        })
        .collect();
    let sequential_sum_refs =
        |values: &[Real]| values.iter().fold(Real::zero(), |sum, value| sum + value);

    group.bench_function("rational_dot_64", |b| {
        b.iter(|| {
            black_box(rational_dot(
                black_box(&rational_left),
                black_box(&rational_right),
            ))
        })
    });
    group.bench_function("rational_matmul_8", |b| {
        b.iter(|| {
            black_box(rational_matmul_8(
                black_box(&rational_left),
                black_box(&rational_right),
            ))
        })
    });
    group.bench_function("real_dot_36", |b| {
        b.iter(|| black_box(real_dot(black_box(&real_left), black_box(&real_right))))
    });
    group.bench_function("real_matmul_6", |b| {
        b.iter(|| black_box(real_matmul_6(black_box(&real_left), black_box(&real_right))))
    });
    group.bench_function("real_sum_refs_64_symbolic", |b| {
        b.iter(|| black_box(Real::sum_refs(black_box(symbolic_sum.iter()))))
    });
    group.bench_function("real_sum_refs_64_symbolic_to_f64", |b| {
        b.iter(|| {
            let sum = Real::sum_refs(black_box(symbolic_sum.iter()));
            black_box(sum.to_f64_lossy())
        })
    });
    group.bench_function("real_sum_refs_1024_symbolic_sequential", |b| {
        b.iter(|| black_box(sequential_sum_refs(black_box(&large_symbolic_sum))))
    });
    group.bench_function("real_sum_refs_1024_symbolic", |b| {
        b.iter(|| black_box(Real::sum_refs(black_box(large_symbolic_sum.iter()))))
    });
    group.bench_function("real_sum_refs_1024_symbolic_to_f64_sequential", |b| {
        b.iter_batched(
            || {
                (2..=1_025)
                    .map(|n| Real::from(n).sqrt().expect("positive radicand"))
                    .collect::<Vec<_>>()
            },
            |values| {
                let sum = sequential_sum_refs(black_box(&values));
                black_box(sum.to_f64_lossy())
            },
            BatchSize::SmallInput,
        )
    });
    group.bench_function("real_sum_refs_1024_symbolic_to_f64", |b| {
        b.iter_batched(
            || {
                (2..=1_025)
                    .map(|n| Real::from(n).sqrt().expect("positive radicand"))
                    .collect::<Vec<_>>()
            },
            |values| {
                let sum = Real::sum_refs(black_box(values.iter()));
                black_box(sum.to_f64_lossy())
            },
            BatchSize::SmallInput,
        )
    });
    group.bench_function("real_sum_refs_1024_rational_sequential", |b| {
        b.iter(|| black_box(sequential_sum_refs(black_box(&large_rational_sum))))
    });
    group.bench_function("real_sum_refs_1024_rational", |b| {
        b.iter(|| black_box(Real::sum_refs(black_box(large_rational_sum.iter()))))
    });
    group.bench_function("real_sum_owned_1024_symbolic_former_clone_path", |b| {
        b.iter_batched(
            || large_symbolic_sum.clone(),
            |values| black_box(sum_owned_with_former_per_value_clone(black_box(values))),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("real_sum_owned_1024_symbolic", |b| {
        b.iter_batched(
            || large_symbolic_sum.clone(),
            |values| black_box(Real::sum_owned(black_box(values))),
            BatchSize::SmallInput,
        )
    });

    group.finish();
}

/// Consume a vector through the public sum reducer while cloning each yielded element.
///
/// The deliberately redundant map reproduces a former per-value clone/drop path for
/// comparison with direct moves. It retains shared inner state, returns zero for an
/// empty vector, and constructs the sum without demanding numeric approximation.
/// The narrow Clippy allowance keeps this historical control expression intact.
#[allow(clippy::map_clone)]
fn sum_owned_with_former_per_value_clone(values: Vec<Real>) -> Real {
    Real::sum_owned(values.into_iter().map(|value| value.clone()))
}

/// Measure construction of rational-angle, inverse, hyperbolic, atan2 and log2 expressions.
///
/// Fixtures include pi/7; principal inverse compositions at 6*pi/7 and 9*pi/7;
/// asinh(1000000); atanh(sqrt(2)/2) and invalid atanh(sqrt(2)); hyperbolics of ln(2)
/// and one; six atan2 axis/quadrant cases; log2(1024), log2(3), and ln(5)/ln(2).
/// The intended principal angles are pi/7, 5*pi/7 and -pi/7 respectively. Hyperbolics
/// of ln(2) mathematically equal 3/4, 5/4 and 3/5.
///
/// Batched clones exclude source preparation/output destruction. Success unwraps
/// and one unwrap_err check only outcome polarity, not result values, error variant,
/// representation or dispatch. atan2 results are only black-boxed, including the
/// library's defined origin behavior. No numerical approximation is requested.
fn bench_exact_transcendental_special_forms(c: &mut Criterion) {
    let mut group = c.benchmark_group("exact_transcendental_special_forms");
    let pi_over_7 = Real::pi() * real(1, 7);
    let six_pi_over_7 = Real::pi() * real(6, 7);
    let nine_pi_over_7 = Real::pi() * real(9, 7);
    let asinh_large = Real::new(Rational::new(1_000_000));
    let atanh_sqrt_half = Real::new(Rational::new(2)).sqrt().unwrap() * real(1, 2);
    let atanh_sqrt_two_error = Real::new(Rational::new(2)).sqrt().unwrap();

    group.bench_function("sin_pi_7", |b| {
        b.iter_batched(
            || pi_over_7.clone(),
            |value| black_box(value.sin()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("cos_pi_7", |b| {
        b.iter_batched(
            || pi_over_7.clone(),
            |value| black_box(value.cos()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("tan_pi_7", |b| {
        b.iter_batched(
            || pi_over_7.clone(),
            |value| black_box(value.tan().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("asin_sin_6pi_7", |b| {
        b.iter_batched(
            || six_pi_over_7.clone(),
            |value| black_box(value.sin().asin().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("acos_cos_9pi_7", |b| {
        b.iter_batched(
            || nine_pi_over_7.clone(),
            |value| black_box(value.cos().acos().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("atan_tan_6pi_7", |b| {
        b.iter_batched(
            || six_pi_over_7.clone(),
            |value| black_box(value.tan().unwrap().atan().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("asinh_large", |b| {
        b.iter_batched(
            || asinh_large.clone(),
            |value| black_box(value.asinh().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("atanh_sqrt_half", |b| {
        b.iter_batched(
            || atanh_sqrt_half.clone(),
            |value| black_box(value.atanh().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("atanh_sqrt_two_error", |b| {
        b.iter_batched(
            || atanh_sqrt_two_error.clone(),
            |value| black_box(value.atanh().unwrap_err()),
            BatchSize::SmallInput,
        )
    });

    let ln_two = Real::new(Rational::new(2)).ln().unwrap();
    let rational_one = Real::one();

    group.bench_function("sinh_ln_two", |b| {
        b.iter_batched(
            || ln_two.clone(),
            |value| black_box(value.sinh().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("cosh_ln_two", |b| {
        b.iter_batched(
            || ln_two.clone(),
            |value| black_box(value.cosh().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("tanh_ln_two", |b| {
        b.iter_batched(
            || ln_two.clone(),
            |value| black_box(value.tanh().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("sinh_rational_one", |b| {
        b.iter_batched(
            || rational_one.clone(),
            |value| black_box(value.sinh().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("cosh_rational_one", |b| {
        b.iter_batched(
            || rational_one.clone(),
            |value| black_box(value.cosh().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("tanh_rational_one", |b| {
        b.iter_batched(
            || rational_one.clone(),
            |value| black_box(value.tanh().unwrap()),
            BatchSize::SmallInput,
        )
    });

    let zero = Real::zero();
    let positive_one = Real::one();
    let negative_one = -Real::one();
    let negative_two = Real::from(-2_i32);

    group.bench_function("atan2_origin", |b| {
        b.iter_batched(
            || (zero.clone(), zero.clone()),
            |(y, x)| black_box(y.atan2(x)),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("atan2_axis_positive_y", |b| {
        b.iter_batched(
            || (positive_one.clone(), zero.clone()),
            |(y, x)| black_box(y.atan2(x)),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("atan2_axis_negative_x", |b| {
        b.iter_batched(
            || (zero.clone(), negative_one.clone()),
            |(y, x)| black_box(y.atan2(x)),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("atan2_quadrant_one_unit_diagonal", |b| {
        b.iter_batched(
            || (positive_one.clone(), positive_one.clone()),
            |(y, x)| black_box(y.atan2(x)),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("atan2_quadrant_two_pi_correction", |b| {
        b.iter_batched(
            || (positive_one.clone(), negative_two.clone()),
            |(y, x)| black_box(y.atan2(x)),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("atan2_quadrant_three_negative_pi", |b| {
        b.iter_batched(
            || (negative_one.clone(), negative_two.clone()),
            |(y, x)| black_box(y.atan2(x)),
            BatchSize::SmallInput,
        )
    });

    let log2_power = Real::new(Rational::new(1024));
    let log2_three = Real::new(Rational::new(3));
    let ln_five = Real::new(Rational::new(5)).ln().unwrap();
    let ln_two_for_quotient = Real::new(Rational::new(2)).ln().unwrap();

    group.bench_function("log2_power_of_two", |b| {
        b.iter_batched(
            || log2_power.clone(),
            |value| black_box(value.log2().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("log2_rational_three", |b| {
        b.iter_batched(
            || log2_three.clone(),
            |value| black_box(value.log2().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("log2_ln_quotient_fold", |b| {
        b.iter_batched(
            || (ln_five.clone(), ln_two_for_quotient.clone()),
            |(num, den)| black_box((num / den).unwrap()),
            BatchSize::SmallInput,
        )
    });

    group.finish();
}

/// Construct cos(value)/sin(value), cloning the input for the cosine branch.
///
/// The benchmark requires a nonzero sine and successful division; failure panics.
/// This composition is defined at odd half-pi angles where cotangent is zero, unlike
/// the reciprocal-tangent control. Construction does not request numeric output.
fn cot_via_quotient(value: Real) -> Real {
    (value.clone().cos() / value.sin()).expect("benchmark cotangent input has nonzero sine")
}

/// Construct the reciprocal of tan(value), panicking if either operation fails.
///
/// This control requires finite defined tangent and a nonzero tangent. It excludes
/// odd half-pi angles although cotangent itself is zero there, as well as cotangent
/// poles. It constructs the result without requesting a numerical enclosure.
fn cot_via_inverse_tangent(value: Real) -> Real {
    value
        .tan()
        .expect("benchmark cotangent input is not a tangent pole")
        .inverse()
        .expect("benchmark tangent is nonzero")
}

/// Compare public cotangent, cosine/sine and reciprocal-tangent constructions.
///
/// Three rows use atan(2), whose mathematical cotangent is 1/2, without forcing
/// approximation. Three use exact 1/1024 and request certified rational intervals
/// at absolute precision -256. Setup clones are outside timing; construction and
/// interval generation are inside. Cold labels do not reset shared caches. Unwraps
/// require success, but neither interval width nor equality across paths is checked.
/// The reciprocal-tangent helper has a narrower domain than public cotangent.
fn bench_real_cotangent(c: &mut Criterion) {
    let mut group = c.benchmark_group("real_cotangent");
    let atan_two = Real::from(2_i32).atan().unwrap();

    group.bench_function("atan_two_direct_construct", |b| {
        b.iter_batched(
            || atan_two.clone(),
            |value| black_box(value.cot().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("atan_two_quotient_construct", |b| {
        b.iter_batched(
            || atan_two.clone(),
            |value| black_box(cot_via_quotient(value)),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("atan_two_inverse_tan_construct", |b| {
        b.iter_batched(
            || atan_two.clone(),
            |value| black_box(cot_via_inverse_tangent(value)),
            BatchSize::SmallInput,
        )
    });

    let tiny = Real::new(Rational::fraction(1, 1_024).unwrap());
    group.bench_function("tiny_direct_cold_p256", |b| {
        b.iter_batched(
            || tiny.clone(),
            |value| {
                black_box(
                    value
                        .cot()
                        .unwrap()
                        .certified_rational_interval(-256)
                        .unwrap(),
                )
            },
            BatchSize::SmallInput,
        )
    });
    group.bench_function("tiny_quotient_cold_p256", |b| {
        b.iter_batched(
            || tiny.clone(),
            |value| {
                black_box(
                    cot_via_quotient(value)
                        .certified_rational_interval(-256)
                        .unwrap(),
                )
            },
            BatchSize::SmallInput,
        )
    });
    group.bench_function("tiny_inverse_tan_cold_p256", |b| {
        b.iter_batched(
            || tiny.clone(),
            |value| {
                black_box(
                    cot_via_inverse_tangent(value)
                        .certified_rational_interval(-256)
                        .unwrap(),
                )
            },
            BatchSize::SmallInput,
        )
    });

    group.finish();
}

/// Construct twenty fixed root/log/product/quotient reductions of pi, e and radicals.
///
/// Prepared sources include pi squared, (pi*e) squared, 18*exp(2), 2e, pi-3,
/// pi^3*exp(5), pi*exp(2), sqrt(2) and sqrt(3). The extra squarefree exponential
/// root row is absent from the static catalogue. Other expressions exercise reciprocal
/// rationalization, cancellation and fact retrieval; their names are not assertions
/// of canonical output or dispatch. Most setup clones and returned-output drops are
/// outside timing, while the facts row uses ordinary iteration. No numeric evaluation
/// or independent expected value is demanded, and retained inputs may carry caches.
fn bench_symbolic_reductions(c: &mut Criterion) {
    let mut group = c.benchmark_group("symbolic_reductions");
    let pi = Real::pi();
    let e = Real::e();
    let pi_square = &pi * &pi;
    let pi_e = &pi * &e;
    let pi_e_square = &pi_e * &pi_e;
    let scaled_e = Real::new(Rational::new(2)) * e.clone();
    let scaled_exp_squarefree =
        Real::new(Rational::new(18)) * Real::new(Rational::new(2)).exp().unwrap();
    let pi_minus_three = Real::pi() - Real::new(Rational::new(3));
    let sqrt_two = Real::new(Rational::new(2)).sqrt().unwrap();
    let sqrt_three = Real::new(Rational::new(3)).sqrt().unwrap();
    let pi_e_sqrt_two = &pi_e * &sqrt_two;
    let e_three = Real::new(Rational::new(3)).exp().unwrap();
    let pi_square_over_e_left = pi_square.clone();
    let pi_cube_e_five =
        (&(&pi_square * &pi) * &Real::new(Rational::new(5)).exp().unwrap()).clone();
    let pi_e_two = &pi * &Real::new(Rational::new(2)).exp().unwrap();
    let one = Real::new(Rational::one());
    let two = Real::new(Rational::new(2));
    let inverse_pi = pi.clone().inverse().unwrap();

    group.bench_function("sqrt_pi_square", |b| {
        b.iter_batched(
            || pi_square.clone(),
            |value| black_box(value.sqrt().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("sqrt_pi_e_square", |b| {
        b.iter_batched(
            || pi_e_square.clone(),
            |value| black_box(value.sqrt().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("sqrt_scaled_exp_squarefree", |b| {
        b.iter_batched(
            || scaled_exp_squarefree.clone(),
            |value| black_box(value.sqrt().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("ln_scaled_e", |b| {
        b.iter_batched(
            || scaled_e.clone(),
            |value| black_box(value.ln().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("sub_pi_three", |b| {
        b.iter_batched(
            || (Real::pi(), Real::new(Rational::new(3))),
            |(left, right)| black_box(left - right),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("pi_minus_three_facts", |b| {
        b.iter(|| black_box(black_box(&pi_minus_three).structural_facts()))
    });
    group.bench_function("div_exp_exp", |b| {
        b.iter_batched(
            || (e_three.clone(), e.clone()),
            |(left, right)| black_box((left / right).unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("div_pi_square_e", |b| {
        b.iter_batched(
            || (pi_square_over_e_left.clone(), e.clone()),
            |(left, right)| black_box((left / right).unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("div_const_products", |b| {
        b.iter_batched(
            || (pi_cube_e_five.clone(), pi_e_two.clone()),
            |(left, right)| black_box((left / right).unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("inverse_pi", |b| {
        b.iter_batched(
            || pi.clone(),
            |value| black_box(value.inverse().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("div_one_pi", |b| {
        b.iter_batched(
            || (one.clone(), pi.clone()),
            |(left, right)| black_box((left / right).unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("div_rational_exp", |b| {
        b.iter_batched(
            || (two.clone(), e.clone()),
            |(left, right)| black_box((left / right).unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("div_e_pi", |b| {
        b.iter_batched(
            || (e.clone(), pi.clone()),
            |(left, right)| black_box((left / right).unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("mul_pi_inverse_pi", |b| {
        b.iter_batched(
            || (pi.clone(), inverse_pi.clone()),
            |(left, right)| black_box(left * right),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("mul_pi_e_sqrt_two", |b| {
        b.iter_batched(
            || (pi_e.clone(), sqrt_two.clone()),
            |(left, right)| black_box(left * right),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("mul_const_product_sqrt_sqrt", |b| {
        b.iter_batched(
            || (pi_e_sqrt_two.clone(), sqrt_two.clone()),
            |(left, right)| black_box(left * right),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("div_const_product_sqrt_e", |b| {
        b.iter_batched(
            || (pi_e_sqrt_two.clone(), e.clone()),
            |(left, right)| black_box((left / right).unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("inverse_const_product_sqrt", |b| {
        b.iter_batched(
            || pi_e_sqrt_two.clone(),
            |value| black_box(value.inverse().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("inverse_sqrt_two", |b| {
        b.iter_batched(
            || sqrt_two.clone(),
            |value| black_box(value.inverse().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("div_sqrt_two_sqrt_three", |b| {
        b.iter_batched(
            || (sqrt_two.clone(), sqrt_three.clone()),
            |(left, right)| black_box((left / right).unwrap()),
            BatchSize::SmallInput,
        )
    });

    group.finish();
}

/// Construct six exact determinant/cofactor workloads from prepared rational or symbolic data.
///
/// Three six-term two-factor sums use signs +,-,+,+,-,+ with mixed denominators,
/// a common denominator 15, or only the third term active. The first and sparse
/// specialized calls return Options that are black-boxed without requiring Some;
/// the known-shared-denominator call returns Real. The common-scale fixture sums
/// to 392/225, and the sparse fixture is (37/41)*(43/47).
///
/// Two general signed-product sums expand 3-by-3 determinants, one rational and
/// one with pi/e/tau/sqrt(2) and exact zero entries. A sparse three-plane fixture
/// has third row (0,0,-41/8,0); its specialized cofactor Option is unwrapped, but
/// neither the tuple nor its plane incidences is checked. These are exact expression
/// outputs, not a general geometry validation. Ordinary timing includes construction
/// and output destruction, reuses prepared factors, and makes no numeric query.
fn bench_exact_product_sums(c: &mut Criterion) {
    let mut group = c.benchmark_group("exact_product_sums");
    let zero = Real::zero();
    let terms = [
        [
            Real::new(Rational::fraction(7, 11).unwrap()),
            Real::new(Rational::fraction(13, 17).unwrap()),
        ],
        [
            Real::new(Rational::fraction(19, 23).unwrap()),
            Real::new(Rational::fraction(29, 31).unwrap()),
        ],
        [
            Real::new(Rational::fraction(37, 41).unwrap()),
            Real::new(Rational::fraction(43, 47).unwrap()),
        ],
        [
            Real::new(Rational::fraction(53, 59).unwrap()),
            Real::new(Rational::fraction(61, 67).unwrap()),
        ],
        [
            Real::new(Rational::fraction(71, 73).unwrap()),
            Real::new(Rational::fraction(79, 83).unwrap()),
        ],
        [
            Real::new(Rational::fraction(89, 97).unwrap()),
            Real::new(Rational::fraction(101, 103).unwrap()),
        ],
    ];

    group.bench_function("signed_product_sum_lcm_6x2", |b| {
        b.iter(|| {
            black_box(Real::exact_rational_signed_product_sum(
                [true, false, true, true, false, true],
                [
                    [&terms[0][0], &terms[0][1]],
                    [&terms[1][0], &terms[1][1]],
                    [&terms[2][0], &terms[2][1]],
                    [&terms[3][0], &terms[3][1]],
                    [&terms[4][0], &terms[4][1]],
                    [&terms[5][0], &terms[5][1]],
                ],
            ))
        })
    });

    let common_scale_terms = [
        [real(7, 15), real(13, 15)],
        [real(8, 15), real(-2, 15)],
        [real(11, 15), real(14, 15)],
        [real(-4, 15), real(7, 15)],
        [real(2, 15), real(-8, 15)],
        [real(13, 15), real(11, 15)],
    ];
    group.bench_function("signed_product_sum_common_scale_6x2", |b| {
        b.iter(|| {
            black_box(
                Real::exact_rational_signed_product_sum_known_shared_denominator(
                    [true, false, true, true, false, true],
                    [
                        [&common_scale_terms[0][0], &common_scale_terms[0][1]],
                        [&common_scale_terms[1][0], &common_scale_terms[1][1]],
                        [&common_scale_terms[2][0], &common_scale_terms[2][1]],
                        [&common_scale_terms[3][0], &common_scale_terms[3][1]],
                        [&common_scale_terms[4][0], &common_scale_terms[4][1]],
                        [&common_scale_terms[5][0], &common_scale_terms[5][1]],
                    ],
                ),
            )
        })
    });

    group.bench_function("signed_product_sum_sparse_single_6x2", |b| {
        b.iter(|| {
            black_box(Real::exact_rational_signed_product_sum(
                [true, false, true, true, false, true],
                [
                    [&zero, &terms[0][1]],
                    [&terms[1][0], &zero],
                    [&terms[2][0], &terms[2][1]],
                    [&zero, &terms[3][1]],
                    [&terms[4][0], &zero],
                    [&zero, &terms[5][1]],
                ],
            ))
        })
    });

    let det3 = [
        [real(3, 7), real(5, 11), real(13, 17)],
        [real(19, 23), real(29, 31), real(37, 41)],
        [real(43, 47), real(53, 59), real(61, 67)],
    ];
    group.bench_function("real_signed_product_sum_rational_det3", |b| {
        b.iter(|| {
            black_box(Real::signed_product_sum(
                [true, false, false, true, true, false],
                [
                    [&det3[0][0], &det3[1][1], &det3[2][2]],
                    [&det3[0][0], &det3[1][2], &det3[2][1]],
                    [&det3[0][1], &det3[1][0], &det3[2][2]],
                    [&det3[0][1], &det3[1][2], &det3[2][0]],
                    [&det3[0][2], &det3[1][0], &det3[2][1]],
                    [&det3[0][2], &det3[1][1], &det3[2][0]],
                ],
            ))
        })
    });

    let symbolic = [
        [Real::pi(), Real::e(), Real::tau()],
        [
            Real::pi() * real(2, 5),
            Real::e() * real(3, 7),
            Real::tau() * real(5, 11),
        ],
        [
            Real::from(2_i32).sqrt().unwrap(),
            Real::from(3_i32),
            Real::zero(),
        ],
    ];
    group.bench_function("real_signed_product_sum_mixed_symbolic_det3", |b| {
        b.iter(|| {
            black_box(Real::signed_product_sum(
                [true, false, false, true, true, false],
                [
                    [&symbolic[0][0], &symbolic[1][1], &symbolic[2][2]],
                    [&symbolic[0][0], &symbolic[1][2], &symbolic[2][1]],
                    [&symbolic[0][1], &symbolic[1][0], &symbolic[2][2]],
                    [&symbolic[0][1], &symbolic[1][2], &symbolic[2][0]],
                    [&symbolic[0][2], &symbolic[1][0], &symbolic[2][1]],
                    [&symbolic[0][2], &symbolic[1][1], &symbolic[2][0]],
                ],
            ))
        })
    });

    let sparse_planes = [
        [real(3, 8), real(-5, 16), real(7, 32), real(11, 64)],
        [real(-13, 16), real(17, 32), real(19, 64), real(-23, 128)],
        [Real::zero(), Real::zero(), real(-41, 8), Real::zero()],
    ];
    let sparse_plane_refs = sparse_planes.each_ref().map(|row| row.each_ref());
    group.bench_function(
        "exact_rational_sparse_homogeneous_plane_intersection3",
        |b| {
            b.iter(|| {
                black_box(
                    Real::exact_rational_sparse_homogeneous_plane_intersection3(sparse_plane_refs)
                        .unwrap(),
                )
            })
        },
    );

    group.finish();
}

criterion_group!(
    benches,
    bench_construction_speed,
    bench_raw_cache_hit_cost,
    bench_structural_query_speed,
    bench_pure_scalar_algorithm_speed,
    bench_rational_algorithm_dispatch_speed,
    bench_borrowed_op_overhead,
    bench_dense_algebra,
    bench_exact_transcendental_special_forms,
    bench_real_cotangent,
    bench_symbolic_reductions,
    bench_exact_product_sums,
    finish_benchmark_report
);
criterion_main!(benches);

/// Rewrite the scalar catalogue, then run the shared Criterion report finalizer.
///
/// The catalogue is also written before raw-cache measurements. Both helpers apply
/// their own write/skip/error handling; this callback performs no numerical check
/// or benchmark-completion validation and does not propagate a typed write result.
fn finish_benchmark_report(c: &mut Criterion) {
    bench_docs::write_benchmark_docs(
        "scalar_micro",
        "Microbenchmarks for scalar operations, structural queries, cache hits, and dense exact arithmetic.",
        SCALAR_MICRO_GROUPS,
    );
    benchmark_report::finish_benchmark_report(c);
}
