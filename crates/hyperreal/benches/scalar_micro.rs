use criterion::{BatchSize, Criterion, black_box, criterion_group, criterion_main};
use hyperreal::{Computable, Rational, Real};
use num::Integer as _;
use num::bigint::{BigInt, BigUint};

#[path = "support/bench_docs.rs"]
mod bench_docs;

use bench_docs::{BenchDoc, BenchGroupDoc};

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
                name: "exact_remainder_large_knuth",
                description: "Computes a wide rational fractional remainder through the traced normalized Knuth backend.",
            },
            BenchDoc {
                name: "division_trivial_small_quotient",
                description: "Exercises the backend's zero-quotient magnitude division exit on wide operands.",
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
        ],
    },
];

fn rational(n: i64, d: u64) -> Rational {
    Rational::fraction(n, d).unwrap()
}

fn real(n: i64, d: u64) -> Real {
    Real::new(rational(n, d))
}

fn tau_computable() -> Computable {
    Computable::tau()
}

fn warm_cache(value: &Computable, precision: i32) {
    black_box(value.approx(precision));
}

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

fn rational_dot(left: &[Rational], right: &[Rational]) -> Rational {
    left.iter()
        .zip(right)
        .fold(Rational::zero(), |acc, (left, right)| {
            acc + black_box(left) * black_box(right)
        })
}

fn real_dot(left: &[Real], right: &[Real]) -> Real {
    left.iter()
        .zip(right)
        .fold(Real::zero(), |acc, (left, right)| {
            acc + (black_box(left) * black_box(right))
        })
}

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

    group.finish();
}

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

fn backend_limb_rational(limbs: usize, low: u8) -> Rational {
    let magnitude =
        (BigUint::from(1_u8) << ((limbs - 1) * usize::BITS as usize)) + BigUint::from(low);
    Rational::from_bigint(BigInt::from(magnitude))
}

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
        (192, "gcd_selected_192_bits", "gcd_euclidean_192_bits"),
        (512, "gcd_selected_512_bits", "gcd_euclidean_512_bits"),
        (1024, "gcd_selected_1024_bits", "gcd_euclidean_1024_bits"),
        (4096, "gcd_selected_4096_bits", "gcd_euclidean_4096_bits"),
    ] {
        let left = benchmark_magnitude(bits, 0x243f_6a88_85a3_08d3);
        let right = benchmark_magnitude(bits, 0xa409_3822_299f_31d0);
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

    group.finish();
}

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
    bench_symbolic_reductions,
    bench_exact_product_sums
);
criterion_main!(benches);
