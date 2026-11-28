//! Microbenchmarks for native approximation, retained facts and exact scalar recipes.
//!
//! Five groups register 134 cases. Rational inputs retain exact values; the trig
//! binary64 fixture retains its exact binary value rather than the decimal ideal.
//! Depth parameters count construction iterations, not retained graph nodes:
//! native simplification can collapse identities and compact scaled chains.
//!
//! Batched setup excludes input cloning/construction from timing, except for the
//! single scaled-square-root case that constructs inside the measured closure.
//! Returned batched outputs are dropped after timing; ordinary `iter` rows include
//! output destruction. Cached rows explicitly warm their values, but "cold" labels
//! do not reset shared source or process-wide constant caches. Constructors can
//! perform their own evaluation even when no explicit approximation is requested.
//!
//! Three pre-registration assertions check coarse exp/expm1 integers and the
//! very negative exponential case. Other outcomes are black-boxed, including
//! interval options and sign/comparison results; names do not assert certification,
//! implementation thresholds or dispatch paths. Precision and loop depth are not
//! total-work or memory bounds, and the runner adds no cancellation protocol.
//!
//! The static catalogue has eleven stale sign-query names. Writers use their own
//! skip guards; benchmarks and preparatory assertions still run in smoke mode.
//! Criterion generates `benches` for callbacks and `main` for the harness; those
//! generated items have no attached Rustdoc here.
use criterion::{BatchSize, Criterion, black_box, criterion_group, criterion_main};
use hyperreal::{Computable, Rational, Real};
use num::bigint::{BigInt, BigUint};
use std::ops::Neg;

/// Publish the static catalogue and available Criterion estimates through the shared writer.
#[path = "support/bench_docs.rs"]
mod bench_docs;
/// Aggregate completed Criterion output using the shared report callback and write guards.
#[path = "support/benchmark_report.rs"]
mod benchmark_report;

use bench_docs::{BenchDoc, BenchGroupDoc};

/// Describe five groups and 134 intended workloads for the generated catalogue.
///
/// Eleven sign-row names differ from registration names, so absent estimates may
/// reflect a name mismatch rather than missing execution. Descriptions of cold
/// caches, node counts, certification, algorithm limits and selected paths express
/// intent, not assertions. In particular, `compare_absolute` uses an absolute
/// tolerance to order signed values; it does not compare absolute magnitudes.
const NUMERICAL_MICRO_GROUPS: &[BenchGroupDoc] = &[
    BenchGroupDoc {
        name: "computable_cache",
        description: "Cold versus cached approximation of basic `Computable` expressions.",
        benches: &[
            BenchDoc {
                name: "ratio_approx_cold_p128",
                description: "Approximates a rational value at p=-128 from a fresh clone.",
            },
            BenchDoc {
                name: "ratio_approx_cached_p128",
                description: "Repeats an already cached rational approximation at p=-128.",
            },
            BenchDoc {
                name: "pi_approx_cold_p128",
                description: "Approximates pi at p=-128 from a fresh clone.",
            },
            BenchDoc {
                name: "pi_approx_cached_p128",
                description: "Repeats an already cached pi approximation at p=-128.",
            },
            BenchDoc {
                name: "pi_plus_tiny_cold_p128",
                description: "Approximates pi plus a tiny exact rational perturbation.",
            },
            BenchDoc {
                name: "pi_minus_tiny_cold_p128",
                description: "Approximates pi minus a tiny exact rational perturbation.",
            },
        ],
    },
    BenchGroupDoc {
        name: "computable_bounds",
        description: "Structural sign and bound discovery for deep or perturbed computable trees.",
        benches: &[
            BenchDoc {
                name: "deep_scaled_product_sign",
                description: "Finds the sign of a deep scaled product.",
            },
            BenchDoc {
                name: "scaled_square_sign",
                description: "Finds the sign of repeated squaring with exact scale factors.",
            },
            BenchDoc {
                name: "sqrt_scaled_square_sign",
                description: "Finds the sign after taking a square root of a scaled square.",
            },
            BenchDoc {
                name: "deep_structural_bound_sign",
                description: "Finds sign through repeated multiply/inverse/negate structural transformations.",
            },
            BenchDoc {
                name: "deep_structural_bound_sign_cached",
                description: "Reads the cached sign of the deep structural-bound chain.",
            },
            BenchDoc {
                name: "deep_structural_bound_facts_cached",
                description: "Reads cached structural facts for the deep structural-bound chain.",
            },
            BenchDoc {
                name: "perturbed_scaled_product_sign",
                description: "Finds sign for a deeply scaled value with a tiny perturbation.",
            },
            BenchDoc {
                name: "perturbed_scaled_product_sign_until",
                description: "Refines sign for the perturbed scaled product only to p=-128.",
            },
            BenchDoc {
                name: "pi_minus_tiny_sign",
                description: "Finds sign for pi minus a tiny exact rational.",
            },
            BenchDoc {
                name: "pi_minus_tiny_sign_cached",
                description: "Reads cached sign for pi minus a tiny exact rational.",
            },
            BenchDoc {
                name: "exp_unknown_sign_arg_sign",
                description: "Finds sign for exp(1 - pi), where exp can prove positivity structurally.",
            },
            BenchDoc {
                name: "exp_unknown_sign_arg_sign_cached",
                description: "Reads cached sign for exp(1 - pi).",
            },
            BenchDoc {
                name: "mixed_pi_e_sign_until_p0_cold",
                description: "Certifies a fresh mixed pi/e expression with the bounded binary64 interval filter.",
            },
            BenchDoc {
                name: "near_pi_sign_until_p64_cold",
                description: "Certifies a fresh close pi/rational difference at its permitted p=-64 floor.",
            },
            BenchDoc {
                name: "near_pi_sign_until_p0_inconclusive_cold",
                description: "Preserves an inconclusive p=0 result for the same close pi/rational difference.",
            },
            BenchDoc {
                name: "unsupported_sin_difference_sign_until_p0_cold",
                description: "Measures bounded-filter rejection and arbitrary-precision fallback for an unsupported sine expression.",
            },
            BenchDoc {
                name: "unsupported_sin_difference_sign_until_p64_cold",
                description: "Measures algebraic-certificate rejection after ordinary refinement reaches p=-64.",
            },
        ],
    },
    BenchGroupDoc {
        name: "computable_algebraic_roots",
        description: "Explicit bounded-degree roots and exact algebraic-zero certification.",
        benches: &[
            BenchDoc {
                name: "root5_construct",
                description: "Constructs the non-perfect fifth root of 17.",
            },
            BenchDoc {
                name: "root5_interval_p128_cold",
                description: "Computes a p=-128 certified interval for a freshly constructed fifth root.",
            },
            BenchDoc {
                name: "root5_interval_p2048_cold",
                description: "Computes a p=-2048 certified interval for a freshly constructed fifth root.",
            },
            BenchDoc {
                name: "root9_interval_p128_cold",
                description: "Computes a p=-128 certified interval at the explicit-root degree cap.",
            },
            BenchDoc {
                name: "root9_interval_p2048_cold",
                description: "Computes a p=-2048 certified interval at the explicit-root degree cap.",
            },
            BenchDoc {
                name: "root10_interval_p128_fallback_cold",
                description: "Exercises the retained exp/ln fallback just above the explicit-root degree cap.",
            },
            BenchDoc {
                name: "eighth_root_near_dyadic_sign_p64_cold",
                description: "Certifies an ordinary nonzero eighth-root difference before algebraic zero metadata is needed.",
            },
            BenchDoc {
                name: "ramanujan_one_zero_sign_p2048_cold",
                description: "Certifies the first archived Ramanujan radical identity as exactly zero.",
            },
            BenchDoc {
                name: "ramanujan_two_zero_sign_p2048_cold",
                description: "Certifies the nested archived Ramanujan radical identity as exactly zero.",
            },
            BenchDoc {
                name: "many_digits_c10_zero_sign_p2048_cold",
                description: "Certifies the archived mixed cube/fifth-root C10 identity as exactly zero.",
            },
        ],
    },
    BenchGroupDoc {
        name: "computable_compare",
        description: "Ordering and absolute-comparison shortcuts.",
        benches: &[
            BenchDoc {
                name: "compare_to_opposite_sign",
                description: "Compares values with known opposite signs.",
            },
            BenchDoc {
                name: "compare_to_exact_msd_gap",
                description: "Compares values with a large exact magnitude gap.",
            },
            BenchDoc {
                name: "compare_to_clone_shared_composite",
                description: "Compares two handles that share one composite expression node.",
            },
            BenchDoc {
                name: "compare_absolute_exact_rational",
                description: "Compares exact rationals using an absolute error tolerance.",
            },
            BenchDoc {
                name: "compare_absolute_exact_rational_same_numerator",
                description: "Compares exact rationals with matching numerator magnitudes.",
            },
            BenchDoc {
                name: "compare_absolute_mixed_exact_leaf_kinds",
                description: "Compares opposite-sign exact values stored as `One` and `Ratio` leaves.",
            },
            BenchDoc {
                name: "compare_absolute_dominant_add",
                description: "Compares a dominant term against the same term plus a tiny addend.",
            },
            BenchDoc {
                name: "compare_absolute_exact_msd_gap",
                description: "Compares absolute values with a large exact magnitude gap.",
            },
        ],
    },
    BenchGroupDoc {
        name: "computable_transcendentals",
        description: "Low-level approximation kernels and deep expression-tree stress cases.",
        benches: &[
            BenchDoc {
                name: "e_constant_cold_p128",
                description: "Approximates the shared e constant from a fresh clone.",
            },
            BenchDoc {
                name: "e_constant_cached_p128",
                description: "Repeats a cached approximation of e.",
            },
            BenchDoc {
                name: "exp_cold_p128",
                description: "Approximates exp(7/5) from a fresh clone.",
            },
            BenchDoc {
                name: "exp_cached_p128",
                description: "Repeats a cached exp(7/5) approximation.",
            },
            BenchDoc {
                name: "exp_deferred_constructor",
                description: "Constructs exp(7/2) without requesting an approximation.",
            },
            BenchDoc {
                name: "exp_coarse_cold_p1",
                description: "Approximates exp(3/2) at coarse precision, requiring certified range reduction.",
            },
            BenchDoc {
                name: "exp_deferred_negative_cold_p128",
                description: "Approximates exp(-10000), exercising deferred binary-scaling fallback.",
            },
            BenchDoc {
                name: "exp_large_cold_p128",
                description: "Approximates exp(128), exercising the bounded exact-integer power path.",
            },
            BenchDoc {
                name: "expm1_tiny_cold_p128",
                description: "Approximates expm1(1/1000000) directly without subtractive cancellation.",
            },
            BenchDoc {
                name: "expm1_coarse_cold_p1",
                description: "Approximates expm1(3/2) at coarse precision after checking the argument range.",
            },
            BenchDoc {
                name: "expm1_coarse_negative_cold_p1",
                description: "Approximates expm1(-32) at coarse precision, where zero is a valid result.",
            },
            BenchDoc {
                name: "exp_negative_integer_cold_p128",
                description: "Approximates exp(-32), retaining signed ln(2) range reduction.",
            },
            BenchDoc {
                name: "exp_integer_limit_cold_p128",
                description: "Approximates exp(256), guarding the binary e-power limit.",
            },
            BenchDoc {
                name: "exp_integer_above_limit_cold_p128",
                description: "Approximates exp(257), retaining the ln(2) range-reduction fallback.",
            },
            BenchDoc {
                name: "exp_half_cold_p128",
                description: "Approximates exp(1/2).",
            },
            BenchDoc {
                name: "exp_near_limit_cold_p128",
                description: "Approximates exp near a prescaling threshold.",
            },
            BenchDoc {
                name: "exp_near_limit_cached_p128",
                description: "Repeats a cached near-threshold exp approximation.",
            },
            BenchDoc {
                name: "exp_zero_cold_p128",
                description: "Approximates exp(0).",
            },
            BenchDoc {
                name: "ln_cold_p128",
                description: "Approximates ln(11/7).",
            },
            BenchDoc {
                name: "ln_cached_p128",
                description: "Repeats a cached ln(11/7) approximation.",
            },
            BenchDoc {
                name: "ln_smooth_rational_cold_p128",
                description: "Approximates ln(45/14), which can decompose into shared prime-log constants.",
            },
            BenchDoc {
                name: "ln_nonsmooth_rational_cold_p128",
                description: "Approximates ln(11/13), guarding the generic exact-rational log fallback.",
            },
            BenchDoc {
                name: "ln_large_cold_p128",
                description: "Approximates ln(1024), exercising large-input reduction.",
            },
            BenchDoc {
                name: "ln_large_cached_p128",
                description: "Repeats a cached ln(1024) approximation.",
            },
            BenchDoc {
                name: "ln_tiny_cold_p128",
                description: "Approximates ln(2^-1024), exercising tiny-input reduction.",
            },
            BenchDoc {
                name: "ln_near_limit_cold_p128",
                description: "Approximates ln near the prescaled-ln limit.",
            },
            BenchDoc {
                name: "ln_near_limit_cached_p128",
                description: "Repeats a cached near-limit ln approximation.",
            },
            BenchDoc {
                name: "ln_one_cold_p128",
                description: "Approximates ln(1).",
            },
            BenchDoc {
                name: "sqrt_cold_p128",
                description: "Approximates sqrt(2).",
            },
            BenchDoc {
                name: "sqrt_squarefree_scaled_cold_p128",
                description: "Approximates sqrt(12), which can reduce to 2*sqrt(3).",
            },
            BenchDoc {
                name: "sqrt_cached_p128",
                description: "Repeats a cached sqrt(2) approximation.",
            },
            BenchDoc {
                name: "sqrt_single_scaled_square_cold_p128",
                description: "Builds and approximates sqrt((7*pi/8)^2).",
            },
            BenchDoc {
                name: "sin_cold_p96",
                description: "Approximates sin(7/5).",
            },
            BenchDoc {
                name: "sin_cached_p96",
                description: "Repeats a cached sin(7/5) approximation.",
            },
            BenchDoc {
                name: "cos_cold_p96",
                description: "Approximates cos(7/5).",
            },
            BenchDoc {
                name: "sin_f64_cold_p96",
                description: "Approximates sin of the exact binary64-derived dyadic for 1.23456789.",
            },
            BenchDoc {
                name: "cos_f64_cold_p96",
                description: "Approximates cos of the exact binary64-derived dyadic for 1.23456789.",
            },
            BenchDoc {
                name: "sin_1e6_cold_p96",
                description: "Approximates sin(1000000).",
            },
            BenchDoc {
                name: "cos_1e6_cold_p96",
                description: "Approximates cos(1000000).",
            },
            BenchDoc {
                name: "sin_1e30_cold_p96",
                description: "Approximates sin(10^30).",
            },
            BenchDoc {
                name: "cos_1e30_cold_p96",
                description: "Approximates cos(10^30).",
            },
            BenchDoc {
                name: "cos_cached_p96",
                description: "Repeats a cached cos(7/5) approximation.",
            },
            BenchDoc {
                name: "tan_cold_p96",
                description: "Approximates tan(7/5).",
            },
            BenchDoc {
                name: "tan_cached_p96",
                description: "Repeats a cached tan(7/5) approximation.",
            },
            BenchDoc {
                name: "sin_zero_cold_p96",
                description: "Approximates sin(0).",
            },
            BenchDoc {
                name: "cos_zero_cold_p96",
                description: "Approximates cos(0).",
            },
            BenchDoc {
                name: "tan_zero_cold_p96",
                description: "Approximates tan(0).",
            },
            BenchDoc {
                name: "tan_near_half_pi_cold_p96",
                description: "Approximates tangent near pi/2.",
            },
            BenchDoc {
                name: "tan_near_half_pi_cached_p96",
                description: "Repeats cached tangent near pi/2.",
            },
            BenchDoc {
                name: "sin_huge_cold_p96",
                description: "Approximates sine of a huge pi multiple plus offset.",
            },
            BenchDoc {
                name: "cos_huge_cold_p96",
                description: "Approximates cosine of a huge pi multiple plus offset.",
            },
            BenchDoc {
                name: "tan_huge_cold_p96",
                description: "Approximates tangent of a huge pi multiple plus offset.",
            },
            BenchDoc {
                name: "asin_cold_p96",
                description: "Approximates a computable asin expression.",
            },
            BenchDoc {
                name: "asin_cached_p96",
                description: "Repeats a cached computable asin approximation.",
            },
            BenchDoc {
                name: "acos_cold_p96",
                description: "Approximates a computable acos expression.",
            },
            BenchDoc {
                name: "acos_cached_p96",
                description: "Repeats a cached computable acos approximation.",
            },
            BenchDoc {
                name: "asin_tiny_cold_p96",
                description: "Approximates asin(1e-12), exercising the tiny-input series.",
            },
            BenchDoc {
                name: "acos_tiny_cold_p96",
                description: "Approximates acos(1e-12), exercising the tiny-input complement.",
            },
            BenchDoc {
                name: "asin_near_one_cold_p96",
                description: "Approximates asin(0.999999), exercising the endpoint complement.",
            },
            BenchDoc {
                name: "acos_near_one_cold_p96",
                description: "Approximates acos(0.999999), exercising the endpoint transform.",
            },
            BenchDoc {
                name: "atan_cold_p96",
                description: "Approximates atan(7/10).",
            },
            BenchDoc {
                name: "atan_cached_p96",
                description: "Repeats a cached atan(7/10) approximation.",
            },
            BenchDoc {
                name: "atan_large_cold_p96",
                description: "Approximates atan(8), exercising argument reduction.",
            },
            BenchDoc {
                name: "asin_zero_cold_p96",
                description: "Approximates asin(0) expression.",
            },
            BenchDoc {
                name: "atan_zero_cold_p96",
                description: "Approximates atan(0).",
            },
            BenchDoc {
                name: "asinh_cold_p128",
                description: "Approximates a computable asinh expression.",
            },
            BenchDoc {
                name: "asinh_three_quarters_cold_p128",
                description: "Approximates asinh(3/4) across the series/ln1p crossover.",
            },
            BenchDoc {
                name: "asinh_cached_p128",
                description: "Repeats a cached computable asinh approximation.",
            },
            BenchDoc {
                name: "acosh_cold_p128",
                description: "Approximates a computable acosh expression.",
            },
            BenchDoc {
                name: "acosh_cached_p128",
                description: "Repeats a cached computable acosh approximation.",
            },
            BenchDoc {
                name: "atanh_cold_p128",
                description: "Approximates a computable atanh expression.",
            },
            BenchDoc {
                name: "atanh_cached_p128",
                description: "Repeats a cached computable atanh approximation.",
            },
            BenchDoc {
                name: "atanh_tiny_cold_p128",
                description: "Approximates atanh(1e-12), exercising the tiny-input series.",
            },
            BenchDoc {
                name: "atanh_near_one_cold_p128",
                description: "Approximates atanh(0.999999), exercising the endpoint log transform.",
            },
            BenchDoc {
                name: "asinh_zero_cold_p128",
                description: "Approximates asinh(0) expression.",
            },
            BenchDoc {
                name: "atanh_zero_cold_p128",
                description: "Approximates atanh(0) expression.",
            },
            BenchDoc {
                name: "deep_add_chain_cold_p128",
                description: "Approximates a 5000-node addition chain.",
            },
            BenchDoc {
                name: "deep_multiply_chain_cold_p128",
                description: "Approximates a 5000-node multiply-by-one chain.",
            },
            BenchDoc {
                name: "deep_multiply_identity_chain_cold_p128",
                description: "Approximates a deep identity multiplication chain around pi.",
            },
            BenchDoc {
                name: "deep_scaled_product_chain_cold_p128",
                description: "Approximates a deep product of exact scale factors.",
            },
            BenchDoc {
                name: "perturbed_scaled_product_chain_cold_p128",
                description: "Approximates a deep scaled product with a tiny perturbation.",
            },
            BenchDoc {
                name: "scaled_square_chain_cold_p128",
                description: "Approximates repeated squaring of a scaled irrational.",
            },
            BenchDoc {
                name: "asymmetric_product_bad_order_cold_p128",
                description: "Approximates an asymmetric product order stress case.",
            },
            BenchDoc {
                name: "sqrt_scaled_square_chain_cold_p128",
                description: "Approximates sqrt of a scaled-square chain.",
            },
            BenchDoc {
                name: "warmed_zero_product_cold_p128",
                description: "Approximates a product involving a warmed zero sum.",
            },
            BenchDoc {
                name: "inverse_scaled_product_chain_cold_p128",
                description: "Approximates the inverse of a deep scaled product.",
            },
            BenchDoc {
                name: "deep_inverse_pair_chain_cold_p128",
                description: "Approximates a chain of inverse(inverse(x)) pairs.",
            },
            BenchDoc {
                name: "deep_negated_square_chain_cold_p128",
                description: "Approximates repeated negate-square-sqrt transformations.",
            },
            BenchDoc {
                name: "deep_negative_one_product_chain_cold_p128",
                description: "Approximates repeated multiplication by -1.",
            },
            BenchDoc {
                name: "deep_half_product_chain_cold_p128",
                description: "Approximates repeated multiplication by 1/2.",
            },
            BenchDoc {
                name: "deep_half_square_chain_cold_p128",
                description: "Approximates repeated squaring after scaling by 1/2.",
            },
            BenchDoc {
                name: "deep_sqrt_square_chain_cold_p128",
                description: "Approximates repeated sqrt-square simplification.",
            },
            BenchDoc {
                name: "inverse_half_product_chain_cold_p128",
                description: "Approximates the inverse of a deep half-product chain.",
            },
        ],
    },
];

/// Build exact `depth + 1` by repeatedly adding one to an initial one.
///
/// Depth counts construction iterations; native exact simplification can prevent
/// a deep retained tree. No approximation or graph-size assertion is made here.
fn deep_add_chain(depth: usize) -> Computable {
    let mut value = Computable::one();
    for _ in 0..depth {
        value = value.add(Computable::one());
    }
    value
}

/// Repeatedly multiply exact one by one, retaining the mathematical value one.
///
/// The loop stresses identity construction; it does not ensure a node per step
/// or perform any approximation.
fn deep_multiply_chain(depth: usize) -> Computable {
    let mut value = Computable::one();
    for _ in 0..depth {
        value = value.multiply(Computable::one());
    }
    value
}

/// Repeatedly multiply pi by one, preserving pi through native identity simplification.
///
/// Depth is a construction count, not a promise of retained evaluator depth.
fn deep_multiply_identity_chain(depth: usize) -> Computable {
    let mut value = Computable::pi();
    for _ in 0..depth {
        value = value.multiply(Computable::one());
    }
    value
}

/// Build exact `pi * (7/8)^depth` through repeated rational scaling.
///
/// Native normalization can compact the graph; coefficient growth and construction
/// work have no local bound beyond the supplied iteration count.
fn deep_scaled_product_chain(depth: usize) -> Computable {
    let scale = Computable::rational(Rational::fraction(7, 8).unwrap());
    let mut value = Computable::pi();
    for _ in 0..depth {
        value = value.multiply(scale.clone());
    }
    value
}

/// Build exact `(7*pi/8)^(2^depth)` by squaring the initially scaled pi repeatedly.
///
/// Even depth zero includes the initial scaling. Exponent/value complexity can
/// grow rapidly; depth does not bound coefficient size, memory or evaluation work.
fn scaled_square_chain(depth: usize) -> Computable {
    let scale = Computable::rational(Rational::fraction(7, 8).unwrap());
    let mut value = Computable::pi().multiply(scale);
    for _ in 0..depth {
        value = value.square();
    }
    value
}

/// Warm pi at precision -16, then iterate `x -> 8/(7*x)` via multiply/inverse/negate.
///
/// The value is pi at even depth and `8/(7*pi)` at odd depth. The initial
/// approximation is unconditional and discarded without an accuracy assertion.
/// Each nonzero iterate admits the inverse; graph simplification and cache reuse
/// can change the retained shape, and no total-resource contract is added.
fn deep_structural_bound_chain(depth: usize) -> Computable {
    let scale = Computable::rational(Rational::fraction(-7, 8).unwrap());
    let mut value = Computable::pi();
    value.approx(-16);
    for _ in 0..depth {
        value = value.multiply(scale.clone()).inverse().negate();
    }
    value
}

/// Warm pi at -16 and repeatedly apply inverse twice, retaining pi.
///
/// The warmup occurs even at depth zero. Its result is discarded; inverse-pair
/// normalization can collapse nodes, so loop count is not graph depth.
fn deep_inverse_pair_chain(depth: usize) -> Computable {
    let mut value = Computable::pi();
    value.approx(-16);
    for _ in 0..depth {
        value = value.inverse().inverse();
    }
    value
}

/// Repeat negate, square and principal square root starting from positive pi.
///
/// Each iteration mathematically returns pi. This exercises native normalization
/// rather than guaranteeing a deep graph; the helper requests no approximation.
fn deep_negated_square_chain(depth: usize) -> Computable {
    let mut value = Computable::pi();
    for _ in 0..depth {
        value = value.negate().square().sqrt();
    }
    value
}

/// Build `(-1)^depth*pi` by successive multiplication by exact minus one.
///
/// The sign depends on parity; native simplification can compact the expression.
fn deep_negative_one_product_chain(depth: usize) -> Computable {
    let minus_one = Computable::rational(Rational::one().neg());
    let mut value = Computable::pi();
    for _ in 0..depth {
        value = value.multiply(minus_one.clone());
    }
    value
}

/// Build exact `pi*2^-depth` by repeated multiplication by one-half.
///
/// Construction may compact dyadic scaling; no explicit approximation is requested.
fn deep_half_product_chain(depth: usize) -> Computable {
    let half = Computable::rational(Rational::fraction(1, 2).unwrap());
    let mut value = Computable::pi();
    for _ in 0..depth {
        value = value.multiply(half.clone());
    }
    value
}

/// Build exact `(pi/2)^(2^depth)` by repeated squaring after one initial half-scale.
///
/// Depth zero returns pi/2. Exponential value/representation growth is not limited
/// by a separate work or allocation budget.
fn deep_half_square_chain(depth: usize) -> Computable {
    let half = Computable::rational(Rational::fraction(1, 2).unwrap());
    let mut value = Computable::pi().multiply(half);
    for _ in 0..depth {
        value = value.square();
    }
    value
}

/// Repeat principal square root followed by square starting from exact two.
///
/// Every iterate mathematically equals two; native normalization can collapse it.
/// The helper requests no approximation and does not assert a retained shape.
fn deep_sqrt_square_chain(depth: usize) -> Computable {
    let mut value = Computable::rational(Rational::new(2));
    for _ in 0..depth {
        value = value.sqrt().square();
    }
    value
}

/// Invert the nonzero half-product chain to represent exact `2^depth/pi`.
///
/// Construction and evaluation inherit the native chain/inverse resource limits.
fn inverse_half_product_chain(depth: usize) -> Computable {
    deep_half_product_chain(depth).inverse()
}

/// Warm `pi+1/1024` at -16, then construct `(pi+1/1024)*(7/8)^depth`.
///
/// The initial approximation is unconditional, including at depth zero, and not
/// validated. Later clones share retained work; native simplification may compact
/// the construction without changing its exact intended value.
fn perturbed_scaled_product_chain(depth: usize) -> Computable {
    let scale = Computable::rational(Rational::fraction(7, 8).unwrap());
    let epsilon = Computable::rational(Rational::fraction(1, 1024).unwrap());
    let mut value = Computable::pi().add(epsilon);
    value.approx(-16);
    for _ in 0..depth {
        value = value.multiply(scale.clone());
    }
    value
}

/// Construct exact `pi^2 - (29/21)*e` for the precision-zero sign workload.
///
/// This helper does not establish which filter will decide the resulting sign.
fn mixed_pi_e_filter_expression() -> Computable {
    Computable::pi()
        .square()
        .add(Computable::e().multiply(Computable::rational(-Rational::fraction(29, 21).unwrap())))
}

/// Construct exact `pi - 103993/33102` for close sign-query workloads.
///
/// Neither a particular decision path nor inconclusiveness at precision zero is
/// asserted; shared constant caches can carry earlier work.
fn near_pi_filter_expression() -> Computable {
    Computable::pi().add(Computable::rational(
        -Rational::fraction(103_993, 33_102).unwrap(),
    ))
}

/// Construct `sin(1/3) - 1095149352672539061/3347087722986034159` exactly.
///
/// This cancellation fixture is queried at floors zero and -64. "Unsupported" is
/// a historical workload name, not an assertion about current decision support.
fn unsupported_sin_difference_expression() -> Computable {
    let one_third = Rational::fraction(1, 3).unwrap();
    let near_sine =
        Rational::fraction(1_095_149_352_672_539_061_i64, 3_347_087_722_986_034_159_u64).unwrap();
    Computable::rational(one_third)
        .sin()
        .add(Computable::rational(-near_sine))
}

/// Construct a selected real degree-th root of an exact word-sized rational.
///
/// Zero denominators and `Real::root_n` errors panic through unwrap. Branch/domain
/// selection follows that API, with no added work budget or floating conversion.
fn algebraic_root(numerator: i64, denominator: u64, degree: u32) -> Real {
    Real::new(Rational::fraction(numerator, denominator).unwrap())
        .root_n(degree)
        .unwrap()
}

/// Build `3*sqrt(cbrt(5)-cbrt(4)) - (cbrt(2)+cbrt(20)-cbrt(25))`.
///
/// This first Ramanujan zero-identity fixture retains real roots for the -2048
/// sign workload. Root errors panic; the benchmark does not assert the sign result.
fn ramanujan_one_identity() -> Real {
    Real::from(3_i32)
        * (algebraic_root(5, 1, 3) - algebraic_root(4, 1, 3))
            .sqrt()
            .unwrap()
        - (algebraic_root(2, 1, 3) + algebraic_root(20, 1, 3) - algebraic_root(25, 1, 3))
}

/// Build `cbrt(cbrt(2)-1) - (cbrt(1/9)-cbrt(2/9)+cbrt(4/9))`.
///
/// This nested Ramanujan zero-identity fixture uses real cube roots. Root errors
/// panic; the later sign-query result is black-boxed without a zero assertion.
fn ramanujan_two_identity() -> Real {
    (algebraic_root(2, 1, 3) - Real::one()).root_n(3).unwrap()
        - (algebraic_root(1, 9, 3) - algebraic_root(2, 9, 3) + algebraic_root(4, 9, 3))
}

/// Build the Many Digits C10 cancellation fixture from real cube and fifth roots.
///
/// For `r = 2^(1/5)`, the exact expression is
/// `cbrt(7+r-5*r^3) + r^2 - r - 1`. Root errors panic; the benchmark does not
/// assert that the later query proves this zero identity.
fn many_digits_c10_identity() -> Real {
    let fifth_root_two = algebraic_root(2, 1, 5);
    (Real::from(7_i32) + fifth_root_two.clone() - Real::from(5_i32) * algebraic_root(8, 1, 5))
        .root_n(3)
        .unwrap()
        + algebraic_root(4, 1, 5)
        - fifth_root_two
        - Real::one()
}

/// Build exact `2^(1/8) - 1199025932245/2^40` for a close sign query.
///
/// The dyadic is subtracted without rounding the selected root; the later -64
/// query is measured without asserting a particular sign or certificate.
fn eighth_root_near_dyadic() -> Real {
    algebraic_root(2, 1, 8)
        - Real::new(Rational::fraction(1_199_025_932_245_i64, 1_u64 << 40).unwrap())
}

/// Invert the nonzero scaled product to represent exact `(8/7)^depth/pi`.
///
/// The native inverse and source chain retain their resource limitations.
fn inverse_scaled_product_chain(depth: usize) -> Computable {
    deep_scaled_product_chain(depth).inverse()
}

/// Forward a caller-admitted `[-1,1]` input to native principal arcsine.
///
/// This benchmark adapter adds no validation, approximation or typed error result;
/// construction inherits the native method's reduction and resource behavior.
fn computable_asin(value: Computable) -> Computable {
    value.asin()
}

/// Forward a caller-admitted `[-1,1]` input to native principal arccosine.
///
/// This adapter adds no domain check or typed error and makes no explicit
/// approximation request; native construction and resource behavior still apply.
fn computable_acos(value: Computable) -> Computable {
    value.acos()
}

/// Forward a defined finite real input to native inverse hyperbolic sine.
///
/// The adapter adds no explicit approximation, but native construction may evaluate
/// the source to choose a reduction. It adds no error or total-resource contract.
fn computable_asinh(value: Computable) -> Computable {
    value.asinh()
}

/// Forward a caller-admitted real input at least one to native principal acosh.
///
/// There is no adapter validation or typed failure. Native construction may evaluate
/// the source; its signal and resource behavior is inherited.
fn computable_acosh(value: Computable) -> Computable {
    value.acosh()
}

/// Forward a caller-admitted input in `(-1,1)` to native real atanh.
///
/// The adapter adds no validation, typed failure or explicit approximation request;
/// native reduction and resource limitations remain applicable.
fn computable_atanh(value: Computable) -> Computable {
    value.atanh()
}

/// Build exact `pi^2*(7/8)^50*2^200`, placing the small factor before the large one.
///
/// The `2^200*pi` factor is warmed at -16 before multiplication. The historical
/// "bad order" label does not assert a slower path or a particular current
/// scheduler; source construction and this warmup precede the benchmark timing.
fn asymmetric_product_bad_order() -> Computable {
    let small = deep_scaled_product_chain(50);
    let large_scale = Computable::rational(Rational::from_bigint(BigInt::from(1_u8) << 200));
    let large = Computable::pi().multiply(large_scale);
    large.approx(-16);
    small.multiply(large)
}

/// Construct exact `pi-pi`, request precision -128, and retain the resulting source.
///
/// The approximation is discarded without asserting zero. The later benchmark
/// multiplies this warmed source by pi; known cancellation may already simplify it.
fn warmed_zero_sum() -> Computable {
    let zero = Computable::pi().add(Computable::pi().negate());
    zero.approx(-128);
    zero
}

/// Publish metadata and register six retained-source approximation workloads at -128.
///
/// Inputs are exact `355/113`, pi and `pi +/- 2^-200`. "Cold" rows clone the same
/// prepared source in untimed batched setup; caches are not reset. Cached ratio/pi
/// rows explicitly approximate once before ordinary `iter` timing. Batched integer
/// drops occur afterward, while ordinary rows include them. Results are unchecked;
/// the report writer's skip guard does not skip the benchmark callback.
fn bench_computable_cache(c: &mut Criterion) {
    bench_docs::write_benchmark_docs(
        "numerical_micro",
        "Low-level `Computable` microbenchmarks for approximation kernels, caches, structural facts, comparisons, and deep evaluator trees.",
        NUMERICAL_MICRO_GROUPS,
    );

    let mut group = c.benchmark_group("computable_cache");
    let ratio = Computable::rational(Rational::fraction(355, 113).unwrap());
    let pi = Computable::pi();
    let pi_plus_tiny = Computable::pi().add(Computable::rational(
        Rational::from_bigint_fraction(BigInt::from(1), BigUint::from(1_u8) << 200).unwrap(),
    ));
    let pi_minus_tiny = Computable::pi().add(Computable::rational(
        Rational::from_bigint_fraction(BigInt::from(-1), BigUint::from(1_u8) << 200).unwrap(),
    ));

    group.bench_function("ratio_approx_cold_p128", |b| {
        b.iter_batched(
            || ratio.clone(),
            |value| black_box(value.approx(-128)),
            BatchSize::SmallInput,
        )
    });
    ratio.approx(-128);
    group.bench_function("ratio_approx_cached_p128", |b| {
        b.iter(|| black_box(ratio.approx(-128)))
    });

    group.bench_function("pi_approx_cold_p128", |b| {
        b.iter_batched(
            || pi.clone(),
            |value| black_box(value.approx(-128)),
            BatchSize::SmallInput,
        )
    });
    pi.approx(-128);
    group.bench_function("pi_approx_cached_p128", |b| {
        b.iter(|| black_box(pi.approx(-128)))
    });

    group.bench_function("pi_plus_tiny_cold_p128", |b| {
        b.iter_batched(
            || pi_plus_tiny.clone(),
            |value| black_box(value.approx(-128)),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("pi_minus_tiny_cold_p128", |b| {
        b.iter_batched(
            || pi_minus_tiny.clone(),
            |value| black_box(value.approx(-128)),
            BatchSize::SmallInput,
        )
    });

    group.finish();
}

/// Register seventeen bounded sign/structural-fact workloads with distinct setup histories.
///
/// Depth-200 scaled, structural and perturbed chains, a six-square value, its root,
/// `pi-2^-200` and `exp(1-pi)` use retained sources and mostly -2000 sign floors;
/// the perturbed source also uses -128. Cached sign/fact rows explicitly query
/// before timing. The last five rows rebuild mixed pi/e, close pi/rational or
/// sine-difference fixtures in untimed setup and query at zero or -64.
///
/// Batched setup may itself evaluate; shared constants can remain warm. Neither
/// sign outcomes, uncertainty, fact contents nor selected dispatch are asserted.
/// Eleven catalogue names are stale relative to the registered sign-query names.
fn bench_computable_bounds(c: &mut Criterion) {
    let mut group = c.benchmark_group("computable_bounds");

    let deep_scaled_product = deep_scaled_product_chain(200);
    group.bench_function("deep_scaled_product_sign_until_p2000", |b| {
        b.iter_batched(
            || deep_scaled_product.clone(),
            |value| black_box(value.sign_until(-2000)),
            BatchSize::SmallInput,
        )
    });

    let scaled_square = scaled_square_chain(6);
    group.bench_function("scaled_square_sign_until_p2000", |b| {
        b.iter_batched(
            || scaled_square.clone(),
            |value| black_box(value.sign_until(-2000)),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("sqrt_scaled_square_sign_until_p2000", |b| {
        b.iter_batched(
            || scaled_square.clone().sqrt(),
            |value| black_box(value.sign_until(-2000)),
            BatchSize::SmallInput,
        )
    });

    let structural_bound = deep_structural_bound_chain(200);
    group.bench_function("deep_structural_bound_sign_until_p2000", |b| {
        b.iter_batched(
            || structural_bound.clone(),
            |value| black_box(value.sign_until(-2000)),
            BatchSize::SmallInput,
        )
    });
    let structural_bound_cached = deep_structural_bound_chain(200);
    structural_bound_cached.sign_until(-2000);
    group.bench_function("deep_structural_bound_sign_until_cached", |b| {
        b.iter(|| black_box(structural_bound_cached.sign_until(-2000)))
    });
    group.bench_function("deep_structural_bound_facts_cached", |b| {
        b.iter(|| black_box(structural_bound_cached.structural_facts()))
    });

    let perturbed = perturbed_scaled_product_chain(200);
    group.bench_function("perturbed_scaled_product_sign_until_p2000", |b| {
        b.iter_batched(
            || perturbed.clone(),
            |value| black_box(value.sign_until(-2000)),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("perturbed_scaled_product_sign_until_p128", |b| {
        b.iter_batched(
            || perturbed.clone(),
            |value| black_box(value.sign_until(-128)),
            BatchSize::SmallInput,
        )
    });

    let pi_minus_tiny = Computable::pi().add(Computable::rational(
        Rational::from_bigint_fraction(BigInt::from(-1), BigUint::from(1_u8) << 200).unwrap(),
    ));
    group.bench_function("pi_minus_tiny_sign_until_p2000", |b| {
        b.iter_batched(
            || pi_minus_tiny.clone(),
            |value| black_box(value.sign_until(-2000)),
            BatchSize::SmallInput,
        )
    });
    let pi_minus_tiny_cached = Computable::pi().add(Computable::rational(
        Rational::from_bigint_fraction(BigInt::from(-1), BigUint::from(1_u8) << 200).unwrap(),
    ));
    pi_minus_tiny_cached.sign_until(-2000);
    group.bench_function("pi_minus_tiny_sign_until_cached", |b| {
        b.iter(|| black_box(pi_minus_tiny_cached.sign_until(-2000)))
    });

    let exp_unknown_sign_arg = Computable::one().add(Computable::pi().negate()).exp();
    group.bench_function("exp_unknown_sign_arg_sign_until_p2000", |b| {
        b.iter_batched(
            || exp_unknown_sign_arg.clone(),
            |value| black_box(value.sign_until(-2000)),
            BatchSize::SmallInput,
        )
    });
    let exp_unknown_sign_arg_cached = Computable::one().add(Computable::pi().negate()).exp();
    exp_unknown_sign_arg_cached.sign_until(-2000);
    group.bench_function("exp_unknown_sign_arg_sign_until_cached", |b| {
        b.iter(|| black_box(exp_unknown_sign_arg_cached.sign_until(-2000)))
    });

    group.bench_function("mixed_pi_e_sign_until_p0_cold", |b| {
        b.iter_batched(
            mixed_pi_e_filter_expression,
            |value| black_box(value.sign_until(0)),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("near_pi_sign_until_p64_cold", |b| {
        b.iter_batched(
            near_pi_filter_expression,
            |value| black_box(value.sign_until(-64)),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("near_pi_sign_until_p0_inconclusive_cold", |b| {
        b.iter_batched(
            near_pi_filter_expression,
            |value| black_box(value.sign_until(0)),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("unsupported_sin_difference_sign_until_p0_cold", |b| {
        b.iter_batched(
            unsupported_sin_difference_expression,
            |value| black_box(value.sign_until(0)),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("unsupported_sin_difference_sign_until_p64_cold", |b| {
        b.iter_batched(
            unsupported_sin_difference_expression,
            |value| black_box(value.sign_until(-64)),
            BatchSize::SmallInput,
        )
    });

    group.finish();
}

/// Register ten real-root construction, interval and sign-query workloads.
///
/// The construction row times and drops `17^(1/5)`. Batched interval rows construct
/// fifth/ninth roots of 17 outside timing, then request -128 or -2048 intervals;
/// the tenth root requests -128. Other rows prepare the eighth-root/dyadic
/// difference or three nested identities outside timing and measure sign queries.
///
/// Interval options and sign results are black-boxed without requiring successful
/// certification. Historical degree-cap/fallback labels do not assert the current
/// representation or dispatch. Construction can establish facts before the timer;
/// shared constants and native resource limitations remain in scope.
fn bench_computable_algebraic_roots(c: &mut Criterion) {
    let mut group = c.benchmark_group("computable_algebraic_roots");

    group.bench_function("root5_construct", |b| {
        b.iter(|| black_box(algebraic_root(17, 1, 5)))
    });
    group.bench_function("root5_interval_p128_cold", |b| {
        b.iter_batched(
            || algebraic_root(17, 1, 5),
            |value| black_box(value.certified_rational_interval(-128)),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("root5_interval_p2048_cold", |b| {
        b.iter_batched(
            || algebraic_root(17, 1, 5),
            |value| black_box(value.certified_rational_interval(-2_048)),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("root9_interval_p128_cold", |b| {
        b.iter_batched(
            || algebraic_root(17, 1, 9),
            |value| black_box(value.certified_rational_interval(-128)),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("root9_interval_p2048_cold", |b| {
        b.iter_batched(
            || algebraic_root(17, 1, 9),
            |value| black_box(value.certified_rational_interval(-2_048)),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("root10_interval_p128_fallback_cold", |b| {
        b.iter_batched(
            || algebraic_root(17, 1, 10),
            |value| black_box(value.certified_rational_interval(-128)),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("eighth_root_near_dyadic_sign_p64_cold", |b| {
        b.iter_batched(
            eighth_root_near_dyadic,
            |value| black_box(value.refine_sign_until(-64)),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("ramanujan_one_zero_sign_p2048_cold", |b| {
        b.iter_batched(
            ramanujan_one_identity,
            |value| black_box(value.refine_sign_until(-2_048)),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("ramanujan_two_zero_sign_p2048_cold", |b| {
        b.iter_batched(
            ramanujan_two_identity,
            |value| black_box(value.refine_sign_until(-2_048)),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("many_digits_c10_zero_sign_p2048_cold", |b| {
        b.iter_batched(
            many_digits_c10_identity,
            |value| black_box(value.refine_sign_until(-2_048)),
            BatchSize::SmallInput,
        )
    });

    group.finish();
}

/// Register eight signed-order queries on retained exact/native expressions.
///
/// The first three use optional comparison for opposite signs, a `2^200` scale
/// gap and a shared composite node. Others use absolute tolerance -40, except
/// `pi+2^-200` versus pi at -128. "Absolute" describes the tolerance, not operand
/// magnitudes, and an `Equal` fallback need not certify exact equality.
///
/// Operands are prepared once; the scale-gap source is explicitly warmed at -16.
/// One row constructs the one leaf inside timing. Outcomes are black-boxed without
/// assertions, so names do not certify a shortcut, sign or equality result.
fn bench_computable_compare(c: &mut Criterion) {
    let mut group = c.benchmark_group("computable_compare");
    let minus_pi = Computable::pi().negate();
    let pi = Computable::pi();
    group.bench_function("compare_to_opposite_sign", |b| {
        b.iter(|| black_box(minus_pi.try_compare_to(&pi)))
    });

    let base = Computable::pi();
    base.approx(-16);
    let huge = base
        .clone()
        .multiply(Computable::rational(Rational::from_bigint(
            BigInt::from(1_u8) << 200,
        )));
    group.bench_function("compare_to_exact_msd_gap", |b| {
        b.iter(|| black_box(huge.try_compare_to(&base)))
    });

    let composite = Computable::pi()
        .multiply(Computable::e())
        .add(Computable::rational(Rational::fraction(7, 13).unwrap()));
    let shared_composite = composite.clone();
    group.bench_function("compare_to_clone_shared_composite", |b| {
        b.iter(|| black_box(composite.try_compare_to(&shared_composite)))
    });

    let left = Computable::rational(Rational::fraction(-7, 8).unwrap());
    let right = Computable::rational(Rational::fraction(9, 10).unwrap());
    group.bench_function("compare_absolute_exact_rational", |b| {
        b.iter(|| black_box(left.compare_absolute(&right, -40)))
    });
    let same_num: BigInt = BigInt::from(1_u8) << 130;
    let same_num_left = Computable::rational(
        Rational::from_bigint_fraction(same_num.clone(), BigUint::from(8_u8)).unwrap(),
    );
    let same_num_right = Computable::rational(
        Rational::from_bigint_fraction(-same_num, BigUint::from(10_u8)).unwrap(),
    );
    group.bench_function("compare_absolute_exact_rational_same_numerator", |b| {
        b.iter(|| black_box(same_num_left.compare_absolute(&same_num_right, -40)))
    });
    let negative_large =
        Computable::rational(Rational::fraction(-100, 3).expect("three is nonzero"));
    group.bench_function("compare_absolute_mixed_exact_leaf_kinds", |b| {
        b.iter(|| black_box(Computable::one().compare_absolute(&negative_large, -40)))
    });

    let big = Computable::pi();
    let tiny = Computable::rational(
        Rational::from_bigint_fraction(BigInt::from(1), BigUint::from(1_u8) << 200).unwrap(),
    );
    let sum = big.clone().add(tiny);
    group.bench_function("compare_absolute_dominant_add", |b| {
        b.iter(|| black_box(sum.compare_absolute(&big, -128)))
    });

    group.bench_function("compare_absolute_exact_msd_gap", |b| {
        b.iter(|| black_box(huge.compare_absolute(&base, -40)))
    });

    group.finish();
}

/// Register 93 approximation/construction rows and check three preliminary integer results.
///
/// Most kernels use precision -128; trig/inverse-trig rows use -96, and coarse
/// exp/expm1 rows use positive precision 1. Preparation asserts the scaled
/// exp(3/2)/2 approximation is 2 or 3, expm1(3/2)/2 is 1 or 2, and
/// exp(-10000) at -128 yields integer zero. These finite checks do not validate
/// other outcomes or enforce any particular internal path.
///
/// Most batched rows construct the operation in untimed setup and time only
/// approximation; prepared-chain rows clone retained objects. Cached rows warm
/// explicitly and use ordinary `iter`. The deferred-exp constructor row measures
/// cloning/construction/destruction, while the single scaled-square-root row
/// constructs and approximates inside its measured closure. Batched returned
/// integer drops are deferred. Rebuilding does not clear shared constant caches.
///
/// The two exp-half/near-limit rows both use exact one-half; ln near-limit uses
/// 47/32. Near-pole tangent uses `pi/2-1/64`, and huge trig uses `2^512*pi+7/5`.
/// Inverse trig uses 7/10, zero and exact endpoint neighbors; asinh/atanh use
/// one-half and acosh uses two, with additional tiny/endpoint cases. Identity
/// chains use 5000 iterations, products/inverse pairs 200, squarings six and
/// root-square normalizations forty. Those counts do not establish graph depth.
/// Callback-level construction, warmups and assertions still run with row filters;
/// there is no independent oracle for the remaining black-boxed results.
fn bench_computable_transcendentals(c: &mut Criterion) {
    let mut group = c.benchmark_group("computable_transcendentals");
    let p = -128;
    let trig_p = -96;

    group.bench_function("e_constant_cold_p128", |b| {
        b.iter_batched(
            || Computable::rational(Rational::one()).exp(),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });
    let e_cached = Computable::rational(Rational::one()).exp();
    e_cached.approx(p);
    group.bench_function("e_constant_cached_p128", |b| {
        b.iter(|| black_box(e_cached.approx(p)))
    });

    let exp_input = Computable::rational(Rational::fraction(7, 5).unwrap());
    group.bench_function("exp_cold_p128", |b| {
        b.iter_batched(
            || exp_input.clone().exp(),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });
    let exp_cached = exp_input.clone().exp();
    exp_cached.approx(p);
    group.bench_function("exp_cached_p128", |b| {
        b.iter(|| black_box(exp_cached.approx(p)))
    });

    let deferred_input = Computable::rational(Rational::fraction(7, 2).unwrap());
    group.bench_function("exp_deferred_constructor", |b| {
        b.iter(|| black_box(deferred_input.clone().exp()))
    });
    let coarse_input = Computable::rational(Rational::fraction(3, 2).unwrap());
    // e^(3/2)/2 lies strictly between 2 and 3. Zero is not a valid coarse
    // approximation, even though it is valid for the small Taylor domain.
    let coarse_result = coarse_input.clone().exp().approx(1);
    assert!(coarse_result == BigInt::from(2) || coarse_result == BigInt::from(3));
    group.bench_function("exp_coarse_cold_p1", |b| {
        b.iter_batched(
            || coarse_input.clone().exp(),
            |value| black_box(value.approx(1)),
            BatchSize::SmallInput,
        )
    });
    let deferred_negative = Computable::rational(Rational::new(-10_000));
    assert_eq!(deferred_negative.clone().exp().approx(p), BigInt::from(0));
    group.bench_function("exp_deferred_negative_cold_p128", |b| {
        b.iter_batched(
            || deferred_negative.clone().exp(),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });

    let exp_large_input = Computable::rational(Rational::new(128));
    group.bench_function("exp_large_cold_p128", |b| {
        b.iter_batched(
            || exp_large_input.clone().exp(),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });
    let exp_negative_integer_input = Computable::rational(Rational::new(-32));
    group.bench_function("exp_negative_integer_cold_p128", |b| {
        b.iter_batched(
            || exp_negative_integer_input.clone().exp(),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });
    let exp_integer_limit_input = Computable::rational(Rational::new(256));
    group.bench_function("exp_integer_limit_cold_p128", |b| {
        b.iter_batched(
            || exp_integer_limit_input.clone().exp(),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });
    let exp_integer_above_limit_input = Computable::rational(Rational::new(257));
    group.bench_function("exp_integer_above_limit_cold_p128", |b| {
        b.iter_batched(
            || exp_integer_above_limit_input.clone().exp(),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });

    let expm1_tiny = Computable::rational(Rational::fraction(1, 1_000_000).unwrap());
    group.bench_function("expm1_tiny_cold_p128", |b| {
        b.iter_batched(
            || expm1_tiny.clone().expm1(),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });
    let expm1_coarse = Computable::rational(Rational::fraction(3, 2).unwrap());
    let coarse_result = expm1_coarse.clone().expm1().approx(1);
    assert!(coarse_result == BigInt::from(1) || coarse_result == BigInt::from(2));
    group.bench_function("expm1_coarse_cold_p1", |b| {
        b.iter_batched(
            || expm1_coarse.clone().expm1(),
            |value| black_box(value.approx(1)),
            BatchSize::SmallInput,
        )
    });
    let expm1_negative = Computable::rational(Rational::new(-32));
    group.bench_function("expm1_coarse_negative_cold_p1", |b| {
        b.iter_batched(
            || expm1_negative.clone().expm1(),
            |value| black_box(value.approx(1)),
            BatchSize::SmallInput,
        )
    });

    let exp_near_limit_input = Computable::rational(Rational::fraction(1, 2).unwrap());
    group.bench_function("exp_half_cold_p128", |b| {
        b.iter_batched(
            || exp_near_limit_input.clone().exp(),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("exp_near_limit_cold_p128", |b| {
        b.iter_batched(
            || exp_near_limit_input.clone().exp(),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });
    let exp_near_limit_cached = exp_near_limit_input.clone().exp();
    exp_near_limit_cached.approx(p);
    group.bench_function("exp_near_limit_cached_p128", |b| {
        b.iter(|| black_box(exp_near_limit_cached.approx(p)))
    });

    let exp_zero_input = Computable::rational(Rational::zero());
    group.bench_function("exp_zero_cold_p128", |b| {
        b.iter_batched(
            || exp_zero_input.clone().exp(),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });

    let ln_input = Computable::rational(Rational::fraction(11, 7).unwrap());
    group.bench_function("ln_cold_p128", |b| {
        b.iter_batched(
            || ln_input.clone().ln(),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });
    let ln_cached = ln_input.clone().ln();
    ln_cached.approx(p);
    group.bench_function("ln_cached_p128", |b| {
        b.iter(|| black_box(ln_cached.approx(p)))
    });

    let ln_smooth_input = Computable::rational(Rational::fraction(45, 14).unwrap());
    group.bench_function("ln_smooth_rational_cold_p128", |b| {
        b.iter_batched(
            || ln_smooth_input.clone().ln(),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });

    let ln_nonsmooth_input = Computable::rational(Rational::fraction(11, 13).unwrap());
    group.bench_function("ln_nonsmooth_rational_cold_p128", |b| {
        b.iter_batched(
            || ln_nonsmooth_input.clone().ln(),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });

    let ln_large_input = Computable::rational(Rational::new(1024));
    group.bench_function("ln_large_cold_p128", |b| {
        b.iter_batched(
            || ln_large_input.clone().ln(),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });
    let ln_large_cached = ln_large_input.clone().ln();
    ln_large_cached.approx(p);
    group.bench_function("ln_large_cached_p128", |b| {
        b.iter(|| black_box(ln_large_cached.approx(p)))
    });

    let ln_tiny_input = Computable::rational(
        Rational::from_bigint_fraction(BigInt::from(1), BigUint::from(1_u8) << 1024).unwrap(),
    );
    group.bench_function("ln_tiny_cold_p128", |b| {
        b.iter_batched(
            || ln_tiny_input.clone().ln(),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });

    let ln_near_limit_input = Computable::rational(Rational::fraction(47, 32).unwrap());
    group.bench_function("ln_near_limit_cold_p128", |b| {
        b.iter_batched(
            || ln_near_limit_input.clone().ln(),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });
    let ln_near_limit_cached = ln_near_limit_input.clone().ln();
    ln_near_limit_cached.approx(p);
    group.bench_function("ln_near_limit_cached_p128", |b| {
        b.iter(|| black_box(ln_near_limit_cached.approx(p)))
    });

    let ln_one_input = Computable::rational(Rational::one());
    group.bench_function("ln_one_cold_p128", |b| {
        b.iter_batched(
            || ln_one_input.clone().ln(),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });

    let sqrt_input = Computable::rational(Rational::new(2));
    group.bench_function("sqrt_cold_p128", |b| {
        b.iter_batched(
            || sqrt_input.clone().sqrt(),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });
    let sqrt_squarefree_scaled_input = Computable::rational(Rational::new(12));
    group.bench_function("sqrt_squarefree_scaled_cold_p128", |b| {
        b.iter_batched(
            || sqrt_squarefree_scaled_input.clone().sqrt(),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });
    let sqrt_cached = sqrt_input.clone().sqrt();
    sqrt_cached.approx(p);
    group.bench_function("sqrt_cached_p128", |b| {
        b.iter(|| black_box(sqrt_cached.approx(p)))
    });

    group.bench_function("sqrt_single_scaled_square_cold_p128", |b| {
        b.iter_batched(
            || (),
            |_| {
                let value = Computable::pi()
                    .multiply(Computable::rational(Rational::fraction(7, 8).unwrap()))
                    .square()
                    .sqrt();
                black_box(value.approx(p))
            },
            BatchSize::SmallInput,
        )
    });

    let trig_input = Computable::rational(Rational::fraction(7, 5).unwrap());
    group.bench_function("sin_cold_p96", |b| {
        b.iter_batched(
            || trig_input.clone().sin(),
            |value| black_box(value.approx(trig_p)),
            BatchSize::SmallInput,
        )
    });
    let sin_cached = trig_input.clone().sin();
    sin_cached.approx(trig_p);
    group.bench_function("sin_cached_p96", |b| {
        b.iter(|| black_box(sin_cached.approx(trig_p)))
    });

    group.bench_function("cos_cold_p96", |b| {
        b.iter_batched(
            || trig_input.clone().cos(),
            |value| black_box(value.approx(trig_p)),
            BatchSize::SmallInput,
        )
    });
    let f64_trig_input = Computable::rational(Rational::try_from(1.23456789_f64).unwrap());
    group.bench_function("sin_f64_cold_p96", |b| {
        b.iter_batched(
            || f64_trig_input.clone().sin(),
            |value| black_box(value.approx(trig_p)),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("cos_f64_cold_p96", |b| {
        b.iter_batched(
            || f64_trig_input.clone().cos(),
            |value| black_box(value.approx(trig_p)),
            BatchSize::SmallInput,
        )
    });
    let million_trig_input = Computable::rational(Rational::new(1_000_000));
    group.bench_function("sin_1e6_cold_p96", |b| {
        b.iter_batched(
            || million_trig_input.clone().sin(),
            |value| black_box(value.approx(trig_p)),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("cos_1e6_cold_p96", |b| {
        b.iter_batched(
            || million_trig_input.clone().cos(),
            |value| black_box(value.approx(trig_p)),
            BatchSize::SmallInput,
        )
    });
    let e30_trig_input = Computable::rational(Rational::from_bigint(BigInt::from(10_u8).pow(30)));
    group.bench_function("sin_1e30_cold_p96", |b| {
        b.iter_batched(
            || e30_trig_input.clone().sin(),
            |value| black_box(value.approx(trig_p)),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("cos_1e30_cold_p96", |b| {
        b.iter_batched(
            || e30_trig_input.clone().cos(),
            |value| black_box(value.approx(trig_p)),
            BatchSize::SmallInput,
        )
    });
    let cos_cached = trig_input.clone().cos();
    cos_cached.approx(trig_p);
    group.bench_function("cos_cached_p96", |b| {
        b.iter(|| black_box(cos_cached.approx(trig_p)))
    });

    group.bench_function("tan_cold_p96", |b| {
        b.iter_batched(
            || trig_input.clone().tan(),
            |value| black_box(value.approx(trig_p)),
            BatchSize::SmallInput,
        )
    });
    let tan_cached = trig_input.clone().tan();
    tan_cached.approx(trig_p);
    group.bench_function("tan_cached_p96", |b| {
        b.iter(|| black_box(tan_cached.approx(trig_p)))
    });

    let zero_trig_input = Computable::rational(Rational::zero());
    group.bench_function("sin_zero_cold_p96", |b| {
        b.iter_batched(
            || zero_trig_input.clone().sin(),
            |value| black_box(value.approx(trig_p)),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("cos_zero_cold_p96", |b| {
        b.iter_batched(
            || zero_trig_input.clone().cos(),
            |value| black_box(value.approx(trig_p)),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("tan_zero_cold_p96", |b| {
        b.iter_batched(
            || zero_trig_input.clone().tan(),
            |value| black_box(value.approx(trig_p)),
            BatchSize::SmallInput,
        )
    });

    let tan_near_half_pi_input = Computable::pi()
        .multiply(Computable::rational(Rational::fraction(1, 2).unwrap()))
        .add(Computable::rational(Rational::fraction(1, 64).unwrap()).negate());
    group.bench_function("tan_near_half_pi_cold_p96", |b| {
        b.iter_batched(
            || tan_near_half_pi_input.clone().tan(),
            |value: Computable| black_box(value.approx(trig_p)),
            BatchSize::SmallInput,
        )
    });
    let tan_near_half_pi_cached = tan_near_half_pi_input.clone().tan();
    tan_near_half_pi_cached.approx(trig_p);
    group.bench_function("tan_near_half_pi_cached_p96", |b| {
        b.iter(|| black_box(tan_near_half_pi_cached.approx(trig_p)))
    });

    let huge_multiple = BigInt::from(1_u8) << 512;
    let huge_trig_input = Computable::pi()
        .multiply(Computable::rational(Rational::from_bigint(huge_multiple)))
        .add(Computable::rational(Rational::fraction(7, 5).unwrap()));
    group.bench_function("sin_huge_cold_p96", |b| {
        b.iter_batched(
            || huge_trig_input.clone().sin(),
            |value| black_box(value.approx(trig_p)),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("cos_huge_cold_p96", |b| {
        b.iter_batched(
            || huge_trig_input.clone().cos(),
            |value| black_box(value.approx(trig_p)),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("tan_huge_cold_p96", |b| {
        b.iter_batched(
            || huge_trig_input.clone().tan(),
            |value| black_box(value.approx(trig_p)),
            BatchSize::SmallInput,
        )
    });

    let inverse_trig_input = Computable::rational(Rational::fraction(7, 10).unwrap());
    group.bench_function("asin_cold_p96", |b| {
        b.iter_batched(
            || computable_asin(inverse_trig_input.clone()),
            |value| black_box(value.approx(trig_p)),
            BatchSize::SmallInput,
        )
    });
    let asin_cached = computable_asin(inverse_trig_input.clone());
    asin_cached.approx(trig_p);
    group.bench_function("asin_cached_p96", |b| {
        b.iter(|| black_box(asin_cached.approx(trig_p)))
    });

    group.bench_function("acos_cold_p96", |b| {
        b.iter_batched(
            || computable_acos(inverse_trig_input.clone()),
            |value| black_box(value.approx(trig_p)),
            BatchSize::SmallInput,
        )
    });
    let acos_cached = computable_acos(inverse_trig_input.clone());
    acos_cached.approx(trig_p);
    group.bench_function("acos_cached_p96", |b| {
        b.iter(|| black_box(acos_cached.approx(trig_p)))
    });

    let tiny_inverse_trig_input =
        Computable::rational(Rational::fraction(1, 1_000_000_000_000).unwrap());
    group.bench_function("asin_tiny_cold_p96", |b| {
        b.iter_batched(
            || tiny_inverse_trig_input.clone().asin(),
            |value| black_box(value.approx(trig_p)),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("acos_tiny_cold_p96", |b| {
        b.iter_batched(
            || tiny_inverse_trig_input.clone().acos(),
            |value| black_box(value.approx(trig_p)),
            BatchSize::SmallInput,
        )
    });

    let near_one_inverse_trig_input =
        Computable::rational(Rational::fraction(999_999, 1_000_000).unwrap());
    group.bench_function("asin_near_one_cold_p96", |b| {
        b.iter_batched(
            || near_one_inverse_trig_input.clone().asin(),
            |value| black_box(value.approx(trig_p)),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("acos_near_one_cold_p96", |b| {
        b.iter_batched(
            || near_one_inverse_trig_input.clone().acos(),
            |value| black_box(value.approx(trig_p)),
            BatchSize::SmallInput,
        )
    });

    group.bench_function("atan_cold_p96", |b| {
        b.iter_batched(
            || inverse_trig_input.clone().atan(),
            |value| black_box(value.approx(trig_p)),
            BatchSize::SmallInput,
        )
    });
    let atan_cached = inverse_trig_input.clone().atan();
    atan_cached.approx(trig_p);
    group.bench_function("atan_cached_p96", |b| {
        b.iter(|| black_box(atan_cached.approx(trig_p)))
    });

    let atan_large_input = Computable::rational(Rational::new(8));
    group.bench_function("atan_large_cold_p96", |b| {
        b.iter_batched(
            || atan_large_input.clone().atan(),
            |value| black_box(value.approx(trig_p)),
            BatchSize::SmallInput,
        )
    });

    let zero_inverse_trig_input = Computable::rational(Rational::zero());
    group.bench_function("asin_zero_cold_p96", |b| {
        b.iter_batched(
            || computable_asin(zero_inverse_trig_input.clone()),
            |value| black_box(value.approx(trig_p)),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("atan_zero_cold_p96", |b| {
        b.iter_batched(
            || zero_inverse_trig_input.clone().atan(),
            |value| black_box(value.approx(trig_p)),
            BatchSize::SmallInput,
        )
    });

    let hyperbolic_input = Computable::rational(Rational::fraction(1, 2).unwrap());
    group.bench_function("asinh_cold_p128", |b| {
        b.iter_batched(
            || computable_asinh(hyperbolic_input.clone()),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });
    let asinh_three_quarters_input = Computable::rational(Rational::fraction(3, 4).unwrap());
    group.bench_function("asinh_three_quarters_cold_p128", |b| {
        b.iter_batched(
            || computable_asinh(asinh_three_quarters_input.clone()),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });
    let asinh_cached = computable_asinh(hyperbolic_input.clone());
    asinh_cached.approx(p);
    group.bench_function("asinh_cached_p128", |b| {
        b.iter(|| black_box(asinh_cached.approx(p)))
    });

    let acosh_input = Computable::rational(Rational::new(2));
    group.bench_function("acosh_cold_p128", |b| {
        b.iter_batched(
            || computable_acosh(acosh_input.clone()),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });
    let acosh_cached = computable_acosh(acosh_input.clone());
    acosh_cached.approx(p);
    group.bench_function("acosh_cached_p128", |b| {
        b.iter(|| black_box(acosh_cached.approx(p)))
    });

    group.bench_function("atanh_cold_p128", |b| {
        b.iter_batched(
            || computable_atanh(hyperbolic_input.clone()),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });
    let atanh_cached = computable_atanh(hyperbolic_input.clone());
    atanh_cached.approx(p);
    group.bench_function("atanh_cached_p128", |b| {
        b.iter(|| black_box(atanh_cached.approx(p)))
    });

    group.bench_function("atanh_tiny_cold_p128", |b| {
        b.iter_batched(
            || tiny_inverse_trig_input.clone().atanh(),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("atanh_near_one_cold_p128", |b| {
        b.iter_batched(
            || near_one_inverse_trig_input.clone().atanh(),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });

    let zero_hyperbolic_input = Computable::rational(Rational::zero());
    group.bench_function("asinh_zero_cold_p128", |b| {
        b.iter_batched(
            || computable_asinh(zero_hyperbolic_input.clone()),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("atanh_zero_cold_p128", |b| {
        b.iter_batched(
            || computable_atanh(zero_hyperbolic_input.clone()),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });

    let deep_add = deep_add_chain(5000);
    group.bench_function("deep_add_chain_cold_p128", |b| {
        b.iter_batched(
            || deep_add.clone(),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });
    let deep_multiply = deep_multiply_chain(5000);
    group.bench_function("deep_multiply_chain_cold_p128", |b| {
        b.iter_batched(
            || deep_multiply.clone(),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });
    let deep_multiply_identity = deep_multiply_identity_chain(5000);
    group.bench_function("deep_multiply_identity_chain_cold_p128", |b| {
        b.iter_batched(
            || deep_multiply_identity.clone(),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });

    let deep_scaled_product = deep_scaled_product_chain(200);
    group.bench_function("deep_scaled_product_chain_cold_p128", |b| {
        b.iter_batched(
            || deep_scaled_product.clone(),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });

    let perturbed_scaled_product = perturbed_scaled_product_chain(200);
    group.bench_function("perturbed_scaled_product_chain_cold_p128", |b| {
        b.iter_batched(
            || perturbed_scaled_product.clone(),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });

    let scaled_square = scaled_square_chain(6);
    group.bench_function("scaled_square_chain_cold_p128", |b| {
        b.iter_batched(
            || scaled_square.clone(),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });
    let asymmetric_bad = asymmetric_product_bad_order();
    group.bench_function("asymmetric_product_bad_order_cold_p128", |b| {
        b.iter_batched(
            || asymmetric_bad.clone(),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("sqrt_scaled_square_chain_cold_p128", |b| {
        b.iter_batched(
            || scaled_square.clone().sqrt(),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });

    let zero_sum = warmed_zero_sum();
    let zero_product = zero_sum.clone().multiply(Computable::pi());
    group.bench_function("warmed_zero_product_cold_p128", |b| {
        b.iter_batched(
            || zero_product.clone(),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });

    let inverse_scaled_product = inverse_scaled_product_chain(200);
    group.bench_function("inverse_scaled_product_chain_cold_p128", |b| {
        b.iter_batched(
            || inverse_scaled_product.clone(),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });

    let inverse_pairs = deep_inverse_pair_chain(200);
    group.bench_function("deep_inverse_pair_chain_cold_p128", |b| {
        b.iter_batched(
            || inverse_pairs.clone(),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });

    let negated_squares = deep_negated_square_chain(40);
    group.bench_function("deep_negated_square_chain_cold_p128", |b| {
        b.iter_batched(
            || negated_squares.clone(),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });

    let negative_one_products = deep_negative_one_product_chain(200);
    group.bench_function("deep_negative_one_product_chain_cold_p128", |b| {
        b.iter_batched(
            || negative_one_products.clone(),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });

    let half_products = deep_half_product_chain(200);
    group.bench_function("deep_half_product_chain_cold_p128", |b| {
        b.iter_batched(
            || half_products.clone(),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });

    let half_squares = deep_half_square_chain(6);
    group.bench_function("deep_half_square_chain_cold_p128", |b| {
        b.iter_batched(
            || half_squares.clone(),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });

    let sqrt_squares = deep_sqrt_square_chain(40);
    group.bench_function("deep_sqrt_square_chain_cold_p128", |b| {
        b.iter_batched(
            || sqrt_squares.clone(),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });

    let inverse_half_products = inverse_half_product_chain(200);
    group.bench_function("inverse_half_product_chain_cold_p128", |b| {
        b.iter_batched(
            || inverse_half_products.clone(),
            |value| black_box(value.approx(p)),
            BatchSize::SmallInput,
        )
    });

    group.finish();
}

criterion_group!(
    benches,
    bench_computable_cache,
    bench_computable_bounds,
    bench_computable_algebraic_roots,
    bench_computable_compare,
    bench_computable_transcendentals,
    finish_benchmark_report
);
criterion_main!(benches);

/// Refresh the catalogue, then delegate final Criterion reporting to the shared writer.
///
/// Each writer keeps its own skip and I/O-error handling. This callback supplies
/// no numerical validation, count reconciliation or successful-write guarantee.
fn finish_benchmark_report(c: &mut Criterion) {
    bench_docs::write_benchmark_docs(
        "numerical_micro",
        "Low-level `Computable` microbenchmarks for approximation kernels, caches, structural facts, comparisons, and deep evaluator trees.",
        NUMERICAL_MICRO_GROUPS,
    );
    benchmark_report::finish_benchmark_report(c);
}
