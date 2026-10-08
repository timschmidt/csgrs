use hyperreal::{Rational, Real};
use num::{BigInt, BigUint};
use std::hint::black_box;
use std::time::Instant;

fn original(coefficients: &[Real], argument: &Real) -> Real {
    let Some((last, rest)) = coefficients.split_last() else {
        return Real::zero();
    };
    let x = argument.exact_rational_ref().unwrap();
    if x.is_zero() {
        return coefficients[0].clone();
    }
    if x.is_one() {
        return Real::sum_refs(coefficients);
    }
    assert!(
        coefficients
            .iter()
            .all(|c| c.exact_rational_ref().is_some())
    );
    let mut value = last.exact_rational_ref().unwrap().clone();
    for coefficient in rest.iter().rev() {
        value = &value * x + coefficient.exact_rational_ref().unwrap();
    }
    Real::new(value)
}

fn measure(operation: impl Fn() -> Real) -> f64 {
    let start = Instant::now();
    black_box(operation());
    let first = start.elapsed().as_secs_f64();
    let repeats = ((0.02 / first.max(1e-9)).ceil() as usize).clamp(1, 20_000);
    let mut samples = Vec::new();
    for _ in 0..5 {
        let start = Instant::now();
        for _ in 0..repeats {
            black_box(operation());
        }
        samples.push(start.elapsed().as_secs_f64() / repeats as f64);
    }
    samples.sort_by(f64::total_cmp);
    samples[2] * 1e6
}

fn main() {
    let chosen = std::env::args().nth(1).unwrap();
    let mut cases = Vec::new();
    for degree in [1_usize, 2, 4, 8, 32, 64] {
        let coefficients = (0..=degree)
            .map(|i| Real::from((i as i32 * 17 % 13) - 6))
            .collect::<Vec<_>>();
        for (label, x) in [
            ("dyadic", Rational::fraction(3, 8).unwrap()),
            ("odd", Rational::fraction(-3, 7).unwrap()),
        ] {
            cases.push((
                format!("small-{degree}-{label}"),
                coefficients.clone(),
                Real::new(x),
            ));
        }
    }
    for degree in [2_usize, 8, 32, 64] {
        for bits in [128_usize, 512] {
            let coefficients = (0..=degree)
                .map(|i| {
                    let magnitude =
                        (BigInt::from(i + 1) << (i % 3 * 130)) + BigInt::from(i * 7 + 3);
                    Real::new(
                        Rational::from_bigint_fraction(
                            if i % 2 == 0 { magnitude } else { -magnitude },
                            BigUint::from([1_u32, 7, 11, 17][i % 4]),
                        )
                        .unwrap(),
                    )
                })
                .collect::<Vec<_>>();
            for (label, denominator) in [
                ("dyadic", BigUint::from(1_u8) << bits),
                ("odd", (BigUint::from(1_u8) << bits) + BigUint::from(15_u8)),
            ] {
                let x = Rational::from_bigint_fraction(
                    (BigInt::from(1_u8) << bits) - BigInt::from(3_u8),
                    denominator,
                )
                .unwrap();
                cases.push((
                    format!("wide-{degree}-{bits}-{label}"),
                    coefficients.clone(),
                    Real::new(x),
                ));
            }
        }
    }
    for bits in [128_usize, 512] {
        let p = (BigInt::from(1_u8) << bits) - BigInt::from(3_u8);
        let q = (BigUint::from(1_u8) << bits) + BigUint::from(15_u8);
        let x = Real::new(Rational::from_bigint_fraction(p.clone(), q.clone()).unwrap());
        let mut coefficients = vec![Real::new(Rational::fraction(7, 11).unwrap())];
        for _ in 0..32 {
            coefficients.push(Real::new(Rational::from_bigint(-&p)));
            coefficients.push(Real::new(Rational::from_bigint(BigInt::from(q.clone()))));
        }
        cases.push((format!("cancelled-64-{bits}"), coefficients, x));
    }
    if chosen == "list" {
        for (name, _, _) in cases {
            println!("{name}");
        }
        return;
    }
    let (name, coefficients, x) = cases.into_iter().find(|case| case.0 == chosen).unwrap();
    let expected = original(&coefficients, &x);
    let actual = Real::eval_poly(&coefficients, &x);
    assert!(actual == expected);
    // Alternate the order across workload names; each result is a median of
    // five repeated samples, with construction outside the measured region.
    let (before, after) = if name.bytes().map(u64::from).sum::<u64>() % 2 == 0 {
        (
            measure(|| original(black_box(&coefficients), black_box(&x))),
            measure(|| Real::eval_poly(black_box(&coefficients), black_box(&x))),
        )
    } else {
        let after = measure(|| Real::eval_poly(black_box(&coefficients), black_box(&x)));
        (
            measure(|| original(black_box(&coefficients), black_box(&x))),
            after,
        )
    };
    println!(
        "{{\"name\":\"{name}\",\"reference_us\":{before},\"candidate_us\":{after},\"speedup\":{}}}",
        before / after
    );
}
