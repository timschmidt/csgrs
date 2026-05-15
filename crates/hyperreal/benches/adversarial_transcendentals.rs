use criterion::{BatchSize, Criterion, black_box, criterion_group, criterion_main};
use hyperreal::{Computable, Problem, Rational, Real};
use num::bigint::{BigInt, BigUint};
use std::ops::Neg;
use std::time::Duration;

#[path = "support/bench_docs.rs"]
mod bench_docs;

use bench_docs::{BenchDoc, BenchGroupDoc};

const ADVERSARIAL_TRANSCENDENTAL_GROUPS: &[BenchGroupDoc] = &[
    BenchGroupDoc {
        name: "trig_adversarial_approx",
        description: "Cold approximation of sine, cosine, and tangent at exact, tiny, huge, and near-singular arguments.",
        benches: &[
            BenchDoc {
                name: "sin_tiny_rational_p96",
                description: "Approximates sin(1e-12), stressing direct tiny-argument setup.",
            },
            BenchDoc {
                name: "cos_tiny_rational_p96",
                description: "Approximates cos(1e-12), stressing direct tiny-argument setup.",
            },
            BenchDoc {
                name: "tan_tiny_rational_p96",
                description: "Approximates tan(1e-12), stressing direct tiny-argument setup.",
            },
            BenchDoc {
                name: "sin_medium_rational_p96",
                description: "Approximates sin(7/5), a moderate non-pi rational.",
            },
            BenchDoc {
                name: "cos_medium_rational_p96",
                description: "Approximates cos(7/5), a moderate non-pi rational.",
            },
            BenchDoc {
                name: "tan_medium_rational_p96",
                description: "Approximates tan(7/5), a moderate non-pi rational.",
            },
            BenchDoc {
                name: "sin_f64_exact_p96",
                description: "Approximates sin(1.23456789 imported as an exact dyadic rational).",
            },
            BenchDoc {
                name: "cos_f64_exact_p96",
                description: "Approximates cos(1.23456789 imported as an exact dyadic rational).",
            },
            BenchDoc {
                name: "sin_1e6_p96",
                description: "Approximates sin(1000000), stressing integer argument reduction.",
            },
            BenchDoc {
                name: "cos_1e6_p96",
                description: "Approximates cos(1000000), stressing integer argument reduction.",
            },
            BenchDoc {
                name: "tan_1e6_p96",
                description: "Approximates tan(1000000), stressing integer argument reduction.",
            },
            BenchDoc {
                name: "sin_1e30_p96",
                description: "Approximates sin(10^30), stressing very large integer reduction.",
            },
            BenchDoc {
                name: "cos_1e30_p96",
                description: "Approximates cos(10^30), stressing very large integer reduction.",
            },
            BenchDoc {
                name: "tan_1e30_p96",
                description: "Approximates tan(10^30), stressing very large integer reduction.",
            },
            BenchDoc {
                name: "sin_huge_pi_plus_offset_p96",
                description: "Approximates sin(2^512*pi + 7/5), stressing exact pi-multiple cancellation.",
            },
            BenchDoc {
                name: "cos_huge_pi_plus_offset_p96",
                description: "Approximates cos(2^512*pi + 7/5), stressing exact pi-multiple cancellation.",
            },
            BenchDoc {
                name: "tan_huge_pi_plus_offset_p96",
                description: "Approximates tan(2^512*pi + 7/5), stressing exact pi-multiple cancellation.",
            },
            BenchDoc {
                name: "tan_near_half_pi_p96",
                description: "Approximates tan(pi/2 - 2^-40), stressing the cotangent complement path.",
            },
        ],
    },
    BenchGroupDoc {
        name: "inverse_trig_adversarial_approx",
        description: "Cold approximation of asin, acos, and atan near exact values, zero, endpoints, and large atan inputs.",
        benches: &[
            BenchDoc {
                name: "asin_zero_p96",
                description: "Approximates asin(0), which should collapse before the generic inverse-trig path.",
            },
            BenchDoc {
                name: "acos_zero_p96",
                description: "Approximates acos(0), which should reduce to pi/2.",
            },
            BenchDoc {
                name: "atan_zero_p96",
                description: "Approximates atan(0), which should collapse to zero.",
            },
            BenchDoc {
                name: "asin_tiny_positive_p96",
                description: "Approximates asin(1e-12), stressing the tiny odd series.",
            },
            BenchDoc {
                name: "acos_tiny_positive_p96",
                description: "Approximates acos(1e-12), stressing pi/2 minus the tiny asin path.",
            },
            BenchDoc {
                name: "atan_tiny_positive_p96",
                description: "Approximates atan(1e-12), stressing direct tiny atan setup.",
            },
            BenchDoc {
                name: "asin_mid_positive_p96",
                description: "Approximates asin(7/10), a generic in-domain value.",
            },
            BenchDoc {
                name: "acos_mid_positive_p96",
                description: "Approximates acos(7/10), a generic in-domain value.",
            },
            BenchDoc {
                name: "atan_mid_positive_p96",
                description: "Approximates atan(7/10), a generic in-domain value.",
            },
            BenchDoc {
                name: "asin_near_one_p96",
                description: "Approximates asin(0.999999), stressing endpoint transforms.",
            },
            BenchDoc {
                name: "acos_near_one_p96",
                description: "Approximates acos(0.999999), stressing endpoint transforms.",
            },
            BenchDoc {
                name: "asin_near_minus_one_p96",
                description: "Approximates asin(-0.999999), stressing odd symmetry near the endpoint.",
            },
            BenchDoc {
                name: "acos_near_minus_one_p96",
                description: "Approximates acos(-0.999999), stressing negative endpoint transforms.",
            },
            BenchDoc {
                name: "atan_large_p96",
                description: "Approximates atan(8), stressing reciprocal reduction.",
            },
            BenchDoc {
                name: "atan_huge_p96",
                description: "Approximates atan(10^30), stressing very large reciprocal reduction.",
            },
        ],
    },
    BenchGroupDoc {
        name: "trig_fuzz_adversarial_approx",
        description: "Deterministic broad sweeps of sine, cosine, and tangent over tiny, ordinary, huge, pi-offset, and near-pole exact inputs.",
        benches: &[
            BenchDoc {
                name: "sin_sweep_768_p96",
                description: "Approximates sin over 768 deterministic exact inputs spanning tiny, ordinary, huge, dyadic, rational, and pi-offset cases.",
            },
            BenchDoc {
                name: "cos_sweep_768_p96",
                description: "Approximates cos over the same 768-input deterministic fuzz sweep.",
            },
            BenchDoc {
                name: "tan_sweep_768_p96",
                description: "Approximates tan over the same deterministic sweep, including near-half-pi stress cases.",
            },
            BenchDoc {
                name: "sin_promoted_slow_candidates_p96",
                description: "Approximates sin over promoted slow candidates found by prior sweep-style runs.",
            },
            BenchDoc {
                name: "cos_promoted_slow_candidates_p96",
                description: "Approximates cos over promoted slow candidates found by prior sweep-style runs.",
            },
            BenchDoc {
                name: "tan_promoted_slow_candidates_p96",
                description: "Approximates tan over promoted near-pole and large-reduction slow candidates.",
            },
        ],
    },
    BenchGroupDoc {
        name: "inverse_hyperbolic_adversarial_approx",
        description: "Cold approximation of inverse hyperbolic functions at tiny, moderate, large, and endpoint-adjacent arguments.",
        benches: &[
            BenchDoc {
                name: "asinh_tiny_positive_p128",
                description: "Approximates asinh(1e-12), stressing cancellation avoidance near zero.",
            },
            BenchDoc {
                name: "asinh_mid_positive_p128",
                description: "Approximates asinh(1/2), a moderate positive value.",
            },
            BenchDoc {
                name: "asinh_large_positive_p128",
                description: "Approximates asinh(10^6), stressing large-input logarithmic behavior.",
            },
            BenchDoc {
                name: "asinh_large_negative_p128",
                description: "Approximates asinh(-10^6), stressing odd symmetry for large inputs.",
            },
            BenchDoc {
                name: "acosh_one_plus_tiny_p128",
                description: "Approximates acosh(1 + 1e-12), stressing the near-one endpoint.",
            },
            BenchDoc {
                name: "acosh_sqrt_two_p128",
                description: "Approximates acosh(sqrt(2)), a symbolic square-root input.",
            },
            BenchDoc {
                name: "acosh_two_p128",
                description: "Approximates acosh(2), a moderate exact rational.",
            },
            BenchDoc {
                name: "acosh_large_positive_p128",
                description: "Approximates acosh(10^6), stressing large-input logarithmic behavior.",
            },
            BenchDoc {
                name: "atanh_tiny_positive_p128",
                description: "Approximates atanh(1e-12), stressing the tiny odd series.",
            },
            BenchDoc {
                name: "atanh_mid_positive_p128",
                description: "Approximates atanh(1/2), a moderate exact rational.",
            },
            BenchDoc {
                name: "atanh_near_one_p128",
                description: "Approximates atanh(0.999999), stressing endpoint logarithmic behavior.",
            },
            BenchDoc {
                name: "atanh_near_minus_one_p128",
                description: "Approximates atanh(-0.999999), stressing odd symmetry near the endpoint.",
            },
        ],
    },
    BenchGroupDoc {
        name: "real_shortcut_adversarial",
        description: "Public `Real` construction shortcuts and domain checks for the same transcendental families.",
        benches: &[
            BenchDoc {
                name: "sin_exact_pi_over_six",
                description: "Constructs sin(pi/6), which should return the exact rational 1/2.",
            },
            BenchDoc {
                name: "cos_exact_pi_over_three",
                description: "Constructs cos(pi/3), which should return the exact rational 1/2.",
            },
            BenchDoc {
                name: "tan_exact_pi_over_four",
                description: "Constructs tan(pi/4), which should return the exact rational 1.",
            },
            BenchDoc {
                name: "asin_exact_half",
                description: "Constructs asin(1/2), which should return pi/6.",
            },
            BenchDoc {
                name: "acos_exact_half",
                description: "Constructs acos(1/2), which should return pi/3.",
            },
            BenchDoc {
                name: "atan_exact_one",
                description: "Constructs atan(1), which should return pi/4.",
            },
            BenchDoc {
                name: "asin_domain_error",
                description: "Rejects asin(1 + 1e-12).",
            },
            BenchDoc {
                name: "acos_domain_error",
                description: "Rejects acos(1 + 1e-12).",
            },
            BenchDoc {
                name: "atanh_endpoint_infinity",
                description: "Rejects atanh(1) as an infinite endpoint.",
            },
            BenchDoc {
                name: "atanh_domain_error",
                description: "Rejects atanh(1 + 1e-12).",
            },
            BenchDoc {
                name: "acosh_domain_error",
                description: "Rejects acosh(1 - 1e-12).",
            },
        ],
    },
];

fn rational(n: i64, d: u64) -> Rational {
    Rational::fraction(n, d).unwrap()
}

fn rational_big(n: BigInt, d: BigUint) -> Rational {
    Rational::from_bigint_fraction(n, d).unwrap()
}

fn tiny() -> Rational {
    rational(1, 1_000_000_000_000)
}

fn near_one() -> Rational {
    rational(999_999, 1_000_000)
}

fn one_plus_tiny() -> Rational {
    Rational::one() + tiny()
}

fn one_minus_tiny() -> Rational {
    Rational::one() - tiny()
}

fn computable(r: Rational) -> Computable {
    Computable::rational(r)
}

fn real(r: Rational) -> Real {
    Real::new(r)
}

fn pi_fraction(n: i64, d: u64) -> Real {
    real(rational(n, d)) * Real::pi()
}

fn huge_pi_plus_offset() -> Computable {
    Computable::pi()
        .multiply(computable(Rational::from_bigint(BigInt::from(1_u8) << 512)))
        .add(computable(rational(7, 5)))
}

fn near_half_pi() -> Computable {
    let offset = rational_big(BigInt::from(1_u8), BigUint::from(1_u8) << 40);
    Computable::pi()
        .multiply(computable(rational(1, 2)))
        .add(computable(offset).negate())
}

fn scaled_rational(seed: u64, exponent: i32) -> Rational {
    let numerator_seed = i64::try_from((seed % 997) + 1).expect("small numerator fits i64");
    let denominator_seed = (seed % 251) + 1;
    let sign = if seed & 1 == 0 { 1 } else { -1 };
    if exponent >= 0 {
        let numerator = BigInt::from(sign * numerator_seed) << exponent;
        rational_big(numerator, BigUint::from(denominator_seed))
    } else {
        let shift = usize::try_from(-exponent).expect("negative exponent magnitude fits usize");
        let denominator = BigUint::from(denominator_seed) << shift;
        rational_big(BigInt::from(sign * numerator_seed), denominator)
    }
}

fn trig_sweep_inputs() -> Vec<Computable> {
    let mut inputs = Vec::with_capacity(768);
    let exponents = [
        -32, -24, -16, -12, -8, -4, -2, -1, 0, 1, 2, 3, 4, 8, 12, 16, 24, 32,
    ];
    let mut state = 0x9e37_79b9_7f4a_7c15_u64;
    while inputs.len() < 520 {
        state = state
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        let exponent = exponents[(state as usize) % exponents.len()];
        inputs.push(computable(scaled_rational(state.rotate_left(17), exponent)));
    }

    let pi = Computable::pi();
    for exponent in [-32, -24, -16, -8, -4, 0, 4, 8, 16, 24, 32] {
        for numerator in [-7, -3, -1, 1, 3, 7] {
            inputs.push(computable(scaled_rational(numerator as u64, exponent)));
        }
    }
    for shift in [8_usize, 16, 24, 32] {
        let offset = computable(rational_big(
            BigInt::from(1_u8),
            BigUint::from(1_u8) << shift,
        ));
        inputs.push(
            pi.clone()
                .multiply(computable(rational(1, 2)))
                .add(offset.clone()),
        );
        inputs.push(
            pi.clone()
                .multiply(computable(rational(1, 2)))
                .add(offset.clone().negate()),
        );
        let huge_pi = pi.clone().multiply(computable(Rational::from_bigint(
            BigInt::from(1_u8) << shift,
        )));
        inputs.push(huge_pi.clone().add(offset.clone()));
        inputs.push(huge_pi.add(offset.negate()));
    }
    while inputs.len() < 768 {
        state = state
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        let exponent = exponents[(state.rotate_right(11) as usize) % exponents.len()];
        inputs.push(computable(scaled_rational(
            state ^ 0xa5a5_a5a5_a5a5_a5a5,
            exponent,
        )));
    }
    inputs
}

fn promoted_trig_slow_candidates() -> Vec<Computable> {
    vec![
        computable(Rational::from_bigint(BigInt::from(10_u8).pow(30))),
        computable(Rational::from_bigint(BigInt::from(10_u8).pow(80))),
        computable(scaled_rational(47, 512)),
        computable(scaled_rational(191, -512)),
        huge_pi_plus_offset(),
        near_half_pi(),
        Computable::pi()
            .multiply(computable(Rational::from_bigint(BigInt::from(1_u8) << 768)))
            .add(computable(rational(1, 1_000_003))),
        Computable::pi()
            .multiply(computable(rational(1, 2)))
            .add(computable(rational_big(
                BigInt::from(-1_i8),
                BigUint::from(1_u8) << 96,
            ))),
    ]
}

fn bench_approx<F>(
    group: &mut criterion::BenchmarkGroup<'_, criterion::measurement::WallTime>,
    name: &'static str,
    input: Computable,
    precision: i32,
    op: F,
) where
    F: Fn(Computable) -> Computable + Copy,
{
    group.bench_function(name, |b| {
        b.iter_batched(
            || input.clone(),
            |value| black_box(op(value).approx(precision)),
            BatchSize::SmallInput,
        )
    });
}

fn bench_approx_sweep<F>(
    group: &mut criterion::BenchmarkGroup<'_, criterion::measurement::WallTime>,
    name: &'static str,
    inputs: &[Computable],
    precision: i32,
    op: F,
) where
    F: Fn(Computable) -> Computable + Copy,
{
    group.bench_function(name, |b| {
        b.iter(|| {
            for input in inputs {
                black_box(op(input.clone()).approx(precision));
            }
        })
    });
}

fn bench_real<F>(
    group: &mut criterion::BenchmarkGroup<'_, criterion::measurement::WallTime>,
    name: &'static str,
    input: Real,
    op: F,
) where
    F: Fn(Real) -> Real + Copy,
{
    group.bench_function(name, |b| {
        b.iter_batched(
            || input.clone(),
            |value| black_box(op(value)),
            BatchSize::SmallInput,
        )
    });
}

fn bench_real_result<F>(
    group: &mut criterion::BenchmarkGroup<'_, criterion::measurement::WallTime>,
    name: &'static str,
    input: Real,
    op: F,
) where
    F: Fn(Real) -> Result<Real, Problem> + Copy,
{
    group.bench_function(name, |b| {
        b.iter_batched(
            || input.clone(),
            |value| black_box(op(value)),
            BatchSize::SmallInput,
        )
    });
}

fn configure_group(group: &mut criterion::BenchmarkGroup<'_, criterion::measurement::WallTime>) {
    group.sample_size(10);
    group.warm_up_time(Duration::from_millis(100));
    group.measurement_time(Duration::from_millis(400));
}

fn bench_trig_adversarial(c: &mut Criterion) {
    bench_docs::write_benchmark_docs(
        "adversarial_transcendentals",
        "Adversarial transcendental benchmarks for `hyperreal` trig, inverse trig, and inverse hyperbolic construction and approximation paths.",
        ADVERSARIAL_TRANSCENDENTAL_GROUPS,
    );

    let mut group = c.benchmark_group("trig_adversarial_approx");
    configure_group(&mut group);
    let p = -96;
    let tiny_input = computable(tiny());
    let medium_input = computable(rational(7, 5));
    let f64_input = computable(Rational::try_from(1.23456789_f64).unwrap());
    let million_input = computable(Rational::new(1_000_000));
    let e30_input = computable(Rational::from_bigint(BigInt::from(10_u8).pow(30)));
    let huge_pi_input = huge_pi_plus_offset();
    let near_pole_input = near_half_pi();

    bench_approx(
        &mut group,
        "sin_tiny_rational_p96",
        tiny_input.clone(),
        p,
        Computable::sin,
    );
    bench_approx(
        &mut group,
        "cos_tiny_rational_p96",
        tiny_input.clone(),
        p,
        Computable::cos,
    );
    bench_approx(
        &mut group,
        "tan_tiny_rational_p96",
        tiny_input,
        p,
        Computable::tan,
    );
    bench_approx(
        &mut group,
        "sin_medium_rational_p96",
        medium_input.clone(),
        p,
        Computable::sin,
    );
    bench_approx(
        &mut group,
        "cos_medium_rational_p96",
        medium_input.clone(),
        p,
        Computable::cos,
    );
    bench_approx(
        &mut group,
        "tan_medium_rational_p96",
        medium_input,
        p,
        Computable::tan,
    );
    bench_approx(
        &mut group,
        "sin_f64_exact_p96",
        f64_input.clone(),
        p,
        Computable::sin,
    );
    bench_approx(
        &mut group,
        "cos_f64_exact_p96",
        f64_input,
        p,
        Computable::cos,
    );
    bench_approx(
        &mut group,
        "sin_1e6_p96",
        million_input.clone(),
        p,
        Computable::sin,
    );
    bench_approx(
        &mut group,
        "cos_1e6_p96",
        million_input.clone(),
        p,
        Computable::cos,
    );
    bench_approx(&mut group, "tan_1e6_p96", million_input, p, Computable::tan);
    bench_approx(
        &mut group,
        "sin_1e30_p96",
        e30_input.clone(),
        p,
        Computable::sin,
    );
    bench_approx(
        &mut group,
        "cos_1e30_p96",
        e30_input.clone(),
        p,
        Computable::cos,
    );
    bench_approx(&mut group, "tan_1e30_p96", e30_input, p, Computable::tan);
    bench_approx(
        &mut group,
        "sin_huge_pi_plus_offset_p96",
        huge_pi_input.clone(),
        p,
        Computable::sin,
    );
    bench_approx(
        &mut group,
        "cos_huge_pi_plus_offset_p96",
        huge_pi_input.clone(),
        p,
        Computable::cos,
    );
    bench_approx(
        &mut group,
        "tan_huge_pi_plus_offset_p96",
        huge_pi_input,
        p,
        Computable::tan,
    );
    bench_approx(
        &mut group,
        "tan_near_half_pi_p96",
        near_pole_input,
        p,
        Computable::tan,
    );
    group.finish();
}

fn bench_inverse_trig_adversarial(c: &mut Criterion) {
    let mut group = c.benchmark_group("inverse_trig_adversarial_approx");
    configure_group(&mut group);
    let p = -96;
    let zero = computable(Rational::zero());
    let tiny_input = computable(tiny());
    let mid_input = computable(rational(7, 10));
    let near_one_input = computable(near_one());
    let near_minus_one_input = computable(near_one().neg());
    let large_input = computable(Rational::new(8));
    let huge_input = computable(Rational::from_bigint(BigInt::from(10_u8).pow(30)));

    bench_approx(
        &mut group,
        "asin_zero_p96",
        zero.clone(),
        p,
        Computable::asin,
    );
    bench_approx(
        &mut group,
        "acos_zero_p96",
        zero.clone(),
        p,
        Computable::acos,
    );
    bench_approx(&mut group, "atan_zero_p96", zero, p, Computable::atan);
    bench_approx(
        &mut group,
        "asin_tiny_positive_p96",
        tiny_input.clone(),
        p,
        Computable::asin,
    );
    bench_approx(
        &mut group,
        "acos_tiny_positive_p96",
        tiny_input.clone(),
        p,
        Computable::acos,
    );
    bench_approx(
        &mut group,
        "atan_tiny_positive_p96",
        tiny_input,
        p,
        Computable::atan,
    );
    bench_approx(
        &mut group,
        "asin_mid_positive_p96",
        mid_input.clone(),
        p,
        Computable::asin,
    );
    bench_approx(
        &mut group,
        "acos_mid_positive_p96",
        mid_input.clone(),
        p,
        Computable::acos,
    );
    bench_approx(
        &mut group,
        "atan_mid_positive_p96",
        mid_input,
        p,
        Computable::atan,
    );
    bench_approx(
        &mut group,
        "asin_near_one_p96",
        near_one_input.clone(),
        p,
        Computable::asin,
    );
    bench_approx(
        &mut group,
        "acos_near_one_p96",
        near_one_input,
        p,
        Computable::acos,
    );
    bench_approx(
        &mut group,
        "asin_near_minus_one_p96",
        near_minus_one_input.clone(),
        p,
        Computable::asin,
    );
    bench_approx(
        &mut group,
        "acos_near_minus_one_p96",
        near_minus_one_input,
        p,
        Computable::acos,
    );
    bench_approx(
        &mut group,
        "atan_large_p96",
        large_input,
        p,
        Computable::atan,
    );
    bench_approx(&mut group, "atan_huge_p96", huge_input, p, Computable::atan);
    group.finish();
}

fn bench_trig_fuzz_adversarial(c: &mut Criterion) {
    let mut group = c.benchmark_group("trig_fuzz_adversarial_approx");
    configure_group(&mut group);
    let p = -96;
    let sweep = trig_sweep_inputs();
    let promoted = promoted_trig_slow_candidates();

    bench_approx_sweep(&mut group, "sin_sweep_768_p96", &sweep, p, Computable::sin);
    bench_approx_sweep(&mut group, "cos_sweep_768_p96", &sweep, p, Computable::cos);
    bench_approx_sweep(&mut group, "tan_sweep_768_p96", &sweep, p, Computable::tan);
    bench_approx_sweep(
        &mut group,
        "sin_promoted_slow_candidates_p96",
        &promoted,
        p,
        Computable::sin,
    );
    bench_approx_sweep(
        &mut group,
        "cos_promoted_slow_candidates_p96",
        &promoted,
        p,
        Computable::cos,
    );
    bench_approx_sweep(
        &mut group,
        "tan_promoted_slow_candidates_p96",
        &promoted,
        p,
        Computable::tan,
    );
    group.finish();
}

fn bench_inverse_hyperbolic_adversarial(c: &mut Criterion) {
    let mut group = c.benchmark_group("inverse_hyperbolic_adversarial_approx");
    configure_group(&mut group);
    let p = -128;
    let tiny_input = computable(tiny());
    let mid_input = computable(rational(1, 2));
    let large_input = computable(Rational::new(1_000_000));
    let large_negative_input = computable(Rational::new(-1_000_000));
    let one_plus_tiny_input = computable(one_plus_tiny());
    let sqrt_two_input = computable(Rational::new(2)).sqrt();
    let two_input = computable(Rational::new(2));
    let near_one_input = computable(near_one());
    let near_minus_one_input = computable(near_one().neg());

    bench_approx(
        &mut group,
        "asinh_tiny_positive_p128",
        tiny_input.clone(),
        p,
        Computable::asinh,
    );
    bench_approx(
        &mut group,
        "asinh_mid_positive_p128",
        mid_input.clone(),
        p,
        Computable::asinh,
    );
    bench_approx(
        &mut group,
        "asinh_large_positive_p128",
        large_input.clone(),
        p,
        Computable::asinh,
    );
    bench_approx(
        &mut group,
        "asinh_large_negative_p128",
        large_negative_input,
        p,
        Computable::asinh,
    );
    bench_approx(
        &mut group,
        "acosh_one_plus_tiny_p128",
        one_plus_tiny_input,
        p,
        Computable::acosh,
    );
    bench_approx(
        &mut group,
        "acosh_sqrt_two_p128",
        sqrt_two_input,
        p,
        Computable::acosh,
    );
    bench_approx(
        &mut group,
        "acosh_two_p128",
        two_input.clone(),
        p,
        Computable::acosh,
    );
    bench_approx(
        &mut group,
        "acosh_large_positive_p128",
        large_input,
        p,
        Computable::acosh,
    );
    bench_approx(
        &mut group,
        "atanh_tiny_positive_p128",
        tiny_input,
        p,
        Computable::atanh,
    );
    bench_approx(
        &mut group,
        "atanh_mid_positive_p128",
        mid_input,
        p,
        Computable::atanh,
    );
    bench_approx(
        &mut group,
        "atanh_near_one_p128",
        near_one_input,
        p,
        Computable::atanh,
    );
    bench_approx(
        &mut group,
        "atanh_near_minus_one_p128",
        near_minus_one_input,
        p,
        Computable::atanh,
    );
    group.finish();
}

fn bench_real_shortcut_adversarial(c: &mut Criterion) {
    let mut group = c.benchmark_group("real_shortcut_adversarial");
    configure_group(&mut group);
    let half = real(rational(1, 2));
    let one_plus_tiny = real(one_plus_tiny());
    let one_minus_tiny = real(one_minus_tiny());

    bench_real(
        &mut group,
        "sin_exact_pi_over_six",
        pi_fraction(1, 6),
        Real::sin,
    );
    bench_real(
        &mut group,
        "cos_exact_pi_over_three",
        pi_fraction(1, 3),
        Real::cos,
    );
    bench_real_result(
        &mut group,
        "tan_exact_pi_over_four",
        pi_fraction(1, 4),
        Real::tan,
    );
    bench_real_result(&mut group, "asin_exact_half", half.clone(), Real::asin);
    bench_real_result(&mut group, "acos_exact_half", half.clone(), Real::acos);
    bench_real_result(
        &mut group,
        "atan_exact_one",
        real(Rational::one()),
        Real::atan,
    );
    bench_real_result(
        &mut group,
        "asin_domain_error",
        one_plus_tiny.clone(),
        Real::asin,
    );
    bench_real_result(
        &mut group,
        "acos_domain_error",
        one_plus_tiny.clone(),
        Real::acos,
    );
    group.bench_function("atanh_endpoint_infinity", |b| {
        b.iter_batched(
            || real(Rational::one()),
            |value| black_box(value.atanh().unwrap_err()),
            BatchSize::SmallInput,
        )
    });
    bench_real_result(&mut group, "atanh_domain_error", one_plus_tiny, Real::atanh);
    bench_real_result(
        &mut group,
        "acosh_domain_error",
        one_minus_tiny,
        Real::acosh,
    );
    group.finish();
}

criterion_group!(
    benches,
    bench_trig_adversarial,
    bench_inverse_trig_adversarial,
    bench_trig_fuzz_adversarial,
    bench_inverse_hyperbolic_adversarial,
    bench_real_shortcut_adversarial
);
criterion_main!(benches);
