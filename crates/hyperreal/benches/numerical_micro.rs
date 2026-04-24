use criterion::{BatchSize, Criterion, black_box, criterion_group, criterion_main};
use num::bigint::{BigInt, BigUint, Sign};
use realistic::{Computable, Rational};

fn half_pi() -> Computable {
    Computable::pi().multiply(Computable::rational(Rational::fraction(1, 2).unwrap()))
}

fn legacy_pi_multiple(value: &Computable) -> BigInt {
    let mut rough_appr = value.approx(-1);
    let mut multiple = &rough_appr / BigInt::from(6);

    loop {
        let adjustment = Computable::pi()
            .multiply(Computable::rational(Rational::from_bigint(multiple.clone())).negate());
        let sum = value.clone().add(adjustment);
        rough_appr = sum.approx(-1);
        multiple += &rough_appr / BigInt::from(6);

        if rough_appr.magnitude() < BigInt::from(6).magnitude() {
            return multiple;
        }
    }
}

fn legacy_half_pi_multiple(value: &Computable) -> BigInt {
    let mut rough_appr = value.approx(-1);
    let mut multiple = &rough_appr / BigInt::from(3);

    loop {
        let adjustment = half_pi()
            .multiply(Computable::rational(Rational::from_bigint(multiple.clone())).negate());
        let sum = value.clone().add(adjustment);
        rough_appr = sum.approx(-1);

        if rough_appr.magnitude() < BigInt::from(2).magnitude() {
            return multiple;
        }

        let mut delta = &rough_appr / BigInt::from(3);
        if delta == BigInt::from(0) {
            delta = match rough_appr.sign() {
                Sign::Minus => BigInt::from(-1),
                _ => BigInt::from(1),
            };
        }
        multiple += delta;
    }
}

fn legacy_huge_sin(value: Computable) -> Computable {
    let multiplier = legacy_half_pi_multiple(&value);
    let adjustment = half_pi()
        .multiply(Computable::rational(Rational::from_bigint(multiplier.clone())).negate());
    let reduced = value.add(adjustment);
    let four = BigInt::from(4);
    let quadrant = ((&multiplier % &four) + &four) % &four;

    if quadrant == BigInt::from(0) {
        reduced.sin()
    } else if quadrant == BigInt::from(1) {
        reduced.cos()
    } else if quadrant == BigInt::from(2) {
        reduced.sin().negate()
    } else {
        reduced.cos().negate()
    }
}

fn legacy_huge_cos(value: Computable) -> Computable {
    let multiplier = legacy_pi_multiple(&value);
    let adjustment =
        Computable::pi().multiply(Computable::rational(Rational::from_bigint(multiplier.clone())).negate());
    if (&multiplier & BigInt::from(1)) == BigInt::from(1) {
        value.add(adjustment).cos().negate()
    } else {
        value.add(adjustment).cos()
    }
}

fn legacy_huge_tan(value: Computable) -> Computable {
    let multiplier = legacy_pi_multiple(&value);
    let adjustment =
        Computable::pi().multiply(Computable::rational(Rational::from_bigint(multiplier)).negate());
    value.add(adjustment).tan()
}

fn legacy_huge_exp(value: Computable) -> Computable {
    let rough_appr = value.approx(-4);
    if rough_appr.sign() == Sign::Minus {
        return legacy_huge_exp(value.negate()).inverse();
    }
    if rough_appr > BigInt::from(8) {
        let half = value.multiply(Computable::rational(Rational::fraction(1, 2).unwrap()));
        return legacy_huge_exp(half).square();
    }
    value.exp()
}

fn bench_computable_cache(c: &mut Criterion) {
    let mut group = c.benchmark_group("computable_cache");
    let ratio = Computable::rational(Rational::fraction(355, 113).unwrap());
    let pi = Computable::pi();

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
    group.bench_function("exp_large_legacy_p128", |b| {
        b.iter_batched(
            || exp_large_input.clone(),
            |value| black_box(legacy_huge_exp(value).approx(p)),
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
    group.bench_function("sin_huge_legacy_p96", |b| {
        b.iter_batched(
            || huge_trig_input.clone(),
            |value| black_box(legacy_huge_sin(value).approx(trig_p)),
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
    group.bench_function("cos_huge_legacy_p96", |b| {
        b.iter_batched(
            || huge_trig_input.clone(),
            |value| black_box(legacy_huge_cos(value).approx(trig_p)),
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
    group.bench_function("tan_huge_legacy_p96", |b| {
        b.iter_batched(
            || huge_trig_input.clone(),
            |value| black_box(legacy_huge_tan(value).approx(trig_p)),
            BatchSize::SmallInput,
        )
    });

    group.finish();
}

criterion_group!(
    benches,
    bench_computable_cache,
    bench_computable_transcendentals
);
criterion_main!(benches);
