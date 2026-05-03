use criterion::{BatchSize, Criterion, black_box, criterion_group, criterion_main};
use num::bigint::{BigInt, BigUint};
use hyperreal::{Computable, Rational};
use std::ops::Neg;

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

    let perturbed = perturbed_scaled_product_chain(200);
    group.bench_function("perturbed_scaled_product_sign", |b| {
        b.iter_batched(
            || perturbed.clone(),
            |value| black_box(value.sign()),
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
    let huge =
        base.clone()
            .multiply(Computable::rational(Rational::from_bigint(BigInt::from(1_u8) << 200)));
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
