use criterion::{BatchSize, Criterion, black_box, criterion_group, criterion_main};
use hyperreal::{Computable, Rational};
use num::Signed;
use num::bigint::{BigInt, BigUint};
use std::ops::Neg;

#[path = "support/bench_docs.rs"]
mod bench_docs;

use bench_docs::{BenchDoc, BenchGroupDoc};

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
                name: "compare_absolute_exact_rational",
                description: "Compares absolute values of exact rationals.",
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
                name: "legacy_exp_one_p128",
                description: "Runs the legacy direct exp series for input 1 at p=-128.",
            },
            BenchDoc {
                name: "e_constant_cold_p128",
                description: "Approximates the shared e constant from a fresh clone.",
            },
            BenchDoc {
                name: "e_constant_cached_p128",
                description: "Repeats a cached approximation of e.",
            },
            BenchDoc {
                name: "legacy_exp_half_p128",
                description: "Runs the legacy direct exp series for input 1/2 at p=-128.",
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
                name: "exp_large_cold_p128",
                description: "Approximates exp(128), exercising large-argument reduction.",
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
                description: "Approximates sin(1.23456789 imported exactly from f64).",
            },
            BenchDoc {
                name: "cos_f64_cold_p96",
                description: "Approximates cos(1.23456789 imported exactly from f64).",
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

fn deep_add_chain(depth: usize) -> Computable {
    let mut value = Computable::one();
    for _ in 0..depth {
        value = value.add(Computable::one());
    }
    value
}

fn deep_multiply_chain(depth: usize) -> Computable {
    let mut value = Computable::one();
    for _ in 0..depth {
        value = value.multiply(Computable::one());
    }
    value
}

fn deep_multiply_identity_chain(depth: usize) -> Computable {
    let mut value = Computable::pi();
    for _ in 0..depth {
        value = value.multiply(Computable::one());
    }
    value
}

fn deep_scaled_product_chain(depth: usize) -> Computable {
    let scale = Computable::rational(Rational::fraction(7, 8).unwrap());
    let mut value = Computable::pi();
    for _ in 0..depth {
        value = value.multiply(scale.clone());
    }
    value
}

fn scaled_square_chain(depth: usize) -> Computable {
    let scale = Computable::rational(Rational::fraction(7, 8).unwrap());
    let mut value = Computable::pi().multiply(scale);
    for _ in 0..depth {
        value = value.square();
    }
    value
}

fn deep_structural_bound_chain(depth: usize) -> Computable {
    let scale = Computable::rational(Rational::fraction(-7, 8).unwrap());
    let mut value = Computable::pi();
    value.approx(-16);
    for _ in 0..depth {
        value = value.multiply(scale.clone()).inverse().negate();
    }
    value
}

fn deep_inverse_pair_chain(depth: usize) -> Computable {
    let mut value = Computable::pi();
    value.approx(-16);
    for _ in 0..depth {
        value = value.inverse().inverse();
    }
    value
}

fn deep_negated_square_chain(depth: usize) -> Computable {
    let mut value = Computable::pi();
    for _ in 0..depth {
        value = value.negate().square().sqrt();
    }
    value
}

fn deep_negative_one_product_chain(depth: usize) -> Computable {
    let minus_one = Computable::rational(Rational::one().neg());
    let mut value = Computable::pi();
    for _ in 0..depth {
        value = value.multiply(minus_one.clone());
    }
    value
}

fn deep_half_product_chain(depth: usize) -> Computable {
    let half = Computable::rational(Rational::fraction(1, 2).unwrap());
    let mut value = Computable::pi();
    for _ in 0..depth {
        value = value.multiply(half.clone());
    }
    value
}

fn deep_half_square_chain(depth: usize) -> Computable {
    let half = Computable::rational(Rational::fraction(1, 2).unwrap());
    let mut value = Computable::pi().multiply(half);
    for _ in 0..depth {
        value = value.square();
    }
    value
}

fn deep_sqrt_square_chain(depth: usize) -> Computable {
    let mut value = Computable::rational(Rational::new(2));
    for _ in 0..depth {
        value = value.sqrt().square();
    }
    value
}

fn inverse_half_product_chain(depth: usize) -> Computable {
    deep_half_product_chain(depth).inverse()
}

fn bench_scale(n: BigInt, p: i32) -> BigInt {
    if p >= 0 {
        n << p
    } else {
        let shifted = n >> (-p - 1);
        (shifted + BigInt::from(1_u8)) >> 1
    }
}

fn bound_log2(n: i32) -> i32 {
    let abs_n = n.abs();
    let ans = ((abs_n + 1) as f64).ln() / 2.0_f64.ln();
    ans.ceil() as i32
}

fn legacy_exp_rational(input: &Rational, p: i32) -> BigInt {
    if p >= 1 {
        return BigInt::from(0_u8);
    }

    let iterations_needed = -p / 2 + 2;
    let calc_precision = p - bound_log2(2 * iterations_needed) - 4;
    let op_prec = p - 3;
    let op_appr = input.shifted_big_integer(-op_prec);
    let scaled_1 = BigInt::from(1_u8) << -calc_precision;
    let max_trunc_error = BigInt::from(1_u8) << (p - 4 - calc_precision);
    let mut current_term = scaled_1.clone();
    let mut sum = scaled_1;
    let mut n: i32 = 0;

    while current_term.abs() > max_trunc_error {
        n += 1;
        current_term = bench_scale(current_term * &op_appr, op_prec) / n;
        sum += &current_term;
    }

    bench_scale(sum, calc_precision - p)
}

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

fn inverse_scaled_product_chain(depth: usize) -> Computable {
    deep_scaled_product_chain(depth).inverse()
}

fn computable_asin(value: Computable) -> Computable {
    value.asin()
}

fn computable_acos(value: Computable) -> Computable {
    value.acos()
}

fn computable_asinh(value: Computable) -> Computable {
    value.asinh()
}

fn computable_acosh(value: Computable) -> Computable {
    value.acosh()
}

fn computable_atanh(value: Computable) -> Computable {
    value.atanh()
}

fn asymmetric_product_bad_order() -> Computable {
    let small = deep_scaled_product_chain(50);
    let large_scale = Computable::rational(Rational::from_bigint(BigInt::from(1_u8) << 200));
    let large = Computable::pi().multiply(large_scale);
    large.approx(-16);
    small.multiply(large)
}

fn warmed_zero_sum() -> Computable {
    let zero = Computable::pi().add(Computable::pi().negate());
    zero.approx(-128);
    zero
}

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

fn bench_computable_bounds(c: &mut Criterion) {
    let mut group = c.benchmark_group("computable_bounds");

    let deep_scaled_product = deep_scaled_product_chain(200);
    group.bench_function("deep_scaled_product_sign", |b| {
        b.iter_batched(
            || deep_scaled_product.clone(),
            |value| black_box(value.sign()),
            BatchSize::SmallInput,
        )
    });

    let scaled_square = scaled_square_chain(6);
    group.bench_function("scaled_square_sign", |b| {
        b.iter_batched(
            || scaled_square.clone(),
            |value| black_box(value.sign()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("sqrt_scaled_square_sign", |b| {
        b.iter_batched(
            || scaled_square.clone().sqrt(),
            |value| black_box(value.sign()),
            BatchSize::SmallInput,
        )
    });

    let structural_bound = deep_structural_bound_chain(200);
    group.bench_function("deep_structural_bound_sign", |b| {
        b.iter_batched(
            || structural_bound.clone(),
            |value| black_box(value.sign()),
            BatchSize::SmallInput,
        )
    });
    let structural_bound_cached = deep_structural_bound_chain(200);
    structural_bound_cached.sign();
    group.bench_function("deep_structural_bound_sign_cached", |b| {
        b.iter(|| black_box(structural_bound_cached.sign()))
    });
    group.bench_function("deep_structural_bound_facts_cached", |b| {
        b.iter(|| black_box(structural_bound_cached.structural_facts()))
    });

    let perturbed = perturbed_scaled_product_chain(200);
    group.bench_function("perturbed_scaled_product_sign", |b| {
        b.iter_batched(
            || perturbed.clone(),
            |value| black_box(value.sign()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("perturbed_scaled_product_sign_until", |b| {
        b.iter_batched(
            || perturbed.clone(),
            |value| black_box(value.sign_until(-128)),
            BatchSize::SmallInput,
        )
    });

    let pi_minus_tiny = Computable::pi().add(Computable::rational(
        Rational::from_bigint_fraction(BigInt::from(-1), BigUint::from(1_u8) << 200).unwrap(),
    ));
    group.bench_function("pi_minus_tiny_sign", |b| {
        b.iter_batched(
            || pi_minus_tiny.clone(),
            |value| black_box(value.sign()),
            BatchSize::SmallInput,
        )
    });
    let pi_minus_tiny_cached = Computable::pi().add(Computable::rational(
        Rational::from_bigint_fraction(BigInt::from(-1), BigUint::from(1_u8) << 200).unwrap(),
    ));
    pi_minus_tiny_cached.sign();
    group.bench_function("pi_minus_tiny_sign_cached", |b| {
        b.iter(|| black_box(pi_minus_tiny_cached.sign()))
    });

    group.finish();
}

fn bench_computable_compare(c: &mut Criterion) {
    let mut group = c.benchmark_group("computable_compare");
    let minus_pi = Computable::pi().negate();
    let pi = Computable::pi();
    group.bench_function("compare_to_opposite_sign", |b| {
        b.iter(|| black_box(minus_pi.compare_to(&pi)))
    });

    let base = Computable::pi();
    base.approx(-16);
    let huge = base
        .clone()
        .multiply(Computable::rational(Rational::from_bigint(
            BigInt::from(1_u8) << 200,
        )));
    group.bench_function("compare_to_exact_msd_gap", |b| {
        b.iter(|| black_box(huge.compare_to(&base)))
    });

    let left = Computable::rational(Rational::fraction(-7, 8).unwrap());
    let right = Computable::rational(Rational::fraction(9, 10).unwrap());
    group.bench_function("compare_absolute_exact_rational", |b| {
        b.iter(|| black_box(left.compare_absolute(&right, -40)))
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

fn bench_computable_transcendentals(c: &mut Criterion) {
    let mut group = c.benchmark_group("computable_transcendentals");
    let p = -128;
    let trig_p = -96;

    let e_input = Rational::one();
    group.bench_function("legacy_exp_one_p128", |b| {
        b.iter(|| black_box(legacy_exp_rational(&e_input, p)))
    });
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

    let exp_half_input = Rational::fraction(1, 2).unwrap();
    group.bench_function("legacy_exp_half_p128", |b| {
        b.iter(|| black_box(legacy_exp_rational(&exp_half_input, p)))
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

    let exp_large_input = Computable::rational(Rational::new(128));
    group.bench_function("exp_large_cold_p128", |b| {
        b.iter_batched(
            || exp_large_input.clone().exp(),
            |value| black_box(value.approx(p)),
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
    bench_computable_compare,
    bench_computable_transcendentals
);
criterion_main!(benches);
