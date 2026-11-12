#![cfg(feature = "rational")]
use hyperinterval::{IntervalError, RationalInterval as I};
use hyperreal::{Problem, Rational as R};

#[cfg(not(target_env = "msvc"))]
#[path = "support/rational_oracle.rs"]
mod rational_oracle;

fn interval(a: i64, b: i64, d: u64) -> I {
    I::new(R::fraction(a, d).unwrap(), R::fraction(b, d).unwrap()).unwrap()
}

#[test]
fn signed_powers_preserve_domain_parity_and_empty() {
    for exponent in [i64::MIN, -7, -2, -1, 0, 1, 2, 7, i64::MAX] {
        assert!(I::empty().checked_pow(exponent).unwrap().is_empty());
        assert_eq!(
            I::point(R::one()).checked_pow(exponent).unwrap(),
            I::point(R::one())
        );
        let minus_one = I::point((-1).into());
        assert_eq!(
            minus_one.checked_pow(exponent).unwrap(),
            I::point(if exponent % 2 == 0 {
                1.into()
            } else {
                (-1).into()
            })
        );
        for x in [
            interval(-2, 3, 1),
            interval(-2, 0, 1),
            interval(0, 3, 1),
            interval(0, 0, 1),
        ] {
            if exponent < 0 {
                assert_eq!(x.checked_pow(exponent), Err(IntervalError::ContainsZero));
            }
            if exponent == 0 {
                assert_eq!(x.checked_pow(exponent).unwrap(), I::point(R::one()));
            }
        }
    }
    assert_eq!(
        interval(-2, 3, 1).checked_pow(4).unwrap(),
        interval(0, 81, 1)
    );
    assert_eq!(
        interval(-2, 3, 1).checked_pow(3).unwrap(),
        interval(-8, 27, 1)
    );
    assert_eq!(
        interval(-3, -2, 1).checked_pow(-2).unwrap(),
        I::new(R::fraction(1, 9).unwrap(), R::fraction(1, 4).unwrap()).unwrap()
    );
    assert_eq!(
        interval(-3, -2, 1).checked_pow(-3).unwrap(),
        I::new(R::fraction(-1, 8).unwrap(), R::fraction(-1, 27).unwrap()).unwrap()
    );
    assert_eq!(
        interval(-1, 1, 1).checked_pow(i64::MAX).unwrap(),
        interval(-1, 1, 1)
    );
    assert_eq!(
        interval(-1, 1, 1).checked_pow(i64::MAX - 1).unwrap(),
        interval(0, 1, 1)
    );
    assert_eq!(
        interval(0, 0, 1).checked_pow(i64::MAX).unwrap(),
        interval(0, 0, 1)
    );
}

#[test]
fn scalar_exhaustion_keeps_its_typed_cause_and_retry() {
    use core::error::Error;
    let x = I::point(2.into());
    let error = x.checked_pow(21_846).unwrap_err();
    assert_eq!(error, IntervalError::Scalar(Problem::Exhausted));
    assert_eq!(
        error.source().unwrap().downcast_ref::<Problem>(),
        Some(&Problem::Exhausted)
    );
    assert!(error.to_string().contains("Exhausted"));
    assert!(IntervalError::ContainsZero.source().is_none());
    for exponent in [i64::MIN, i64::MAX, -21_846] {
        assert_eq!(
            x.checked_pow(exponent),
            Err(IntervalError::Scalar(Problem::Exhausted))
        );
    }
    assert_eq!(x.checked_pow(3).unwrap(), I::point(8.into()));
    assert_eq!(x, I::point(2.into()));
    // A first endpoint succeeds but the second reaches the kernel's guard;
    // no partial interval is published and later operations remain usable.
    let mixed = interval(1, 2, 1);
    assert_eq!(
        mixed.checked_pow(i64::MAX),
        Err(IntervalError::Scalar(Problem::Exhausted))
    );
    assert_eq!(mixed.checked_pow(3).unwrap(), interval(1, 8, 1));
}

#[test]
fn even_zero_crossing_powers_do_not_materialize_unused_endpoints() {
    for x in [
        I::new(R::fraction(-1, 3).unwrap(), R::one()).unwrap(),
        I::new((-1).into(), R::fraction(1, 3).unwrap()).unwrap(),
    ] {
        assert_eq!(x.checked_pow(i64::MAX - 1).unwrap(), interval(0, 1, 1));
    }
    for x in [
        I::new(R::fraction(-1, 3).unwrap(), 2.into()).unwrap(),
        I::new((-2).into(), R::fraction(1, 3).unwrap()).unwrap(),
    ] {
        assert_eq!(
            x.checked_pow(i64::MAX - 1),
            Err(IntervalError::Scalar(Problem::Exhausted))
        );
    }
}

#[cfg(not(target_env = "msvc"))]
#[test]
fn signed_images_match_independent_gmp_powers() {
    use rational_oracle::mpq;
    use rug::{Rational as G, ops::Pow};
    for denominator in [1, 2, 3, 7] {
        for a in -6..=6 {
            for b in a..=6 {
                let x = interval(a, b, denominator);
                for exponent in -9i32..=9 {
                    if exponent < 0 && a <= 0 && b >= 0 {
                        assert_eq!(
                            x.checked_pow(i64::from(exponent)),
                            Err(IntervalError::ContainsZero)
                        );
                        continue;
                    }
                    // Exact grid members include every possible critical point
                    // (zero), as well as the endpoints. Monotonicity on each
                    // side proves these are also the continuous interval extrema.
                    let values: Vec<_> = (a..=b)
                        .map(|n| G::from((n, denominator)).pow(exponent))
                        .collect();
                    let image = x.checked_pow(i64::from(exponent)).unwrap();
                    let (lo, hi) = image.bounds().unwrap();
                    assert_eq!(mpq(lo), *values.iter().min().unwrap());
                    assert_eq!(mpq(hi), *values.iter().max().unwrap());
                    assert_eq!(image.abs().square(), image.square());
                    if !image.signs().contains_zero() {
                        let inverse = image.checked_pow(-1).unwrap();
                        assert!(image.mul(&inverse).contains_point(&R::one()));
                    }
                }
            }
        }
    }
    // Mixed denominators and coefficients beyond primitive width.
    let inputs = [
        "-123456789012345678901234567890123456789/17",
        "-9/11",
        "-1/1000000000000000000000000000000000000000",
        "0",
        "1/1000000000000000000000000000000000000000",
        "4/7",
        "987654321098765432109876543210987654321/19",
    ];
    for i in 0..inputs.len() {
        for j in i..inputs.len() {
            let x = I::new(inputs[i].parse().unwrap(), inputs[j].parse().unwrap()).unwrap();
            let a: G = inputs[i].parse().unwrap();
            let b: G = inputs[j].parse().unwrap();
            for exponent in [-17, -4, -1, 0, 1, 2, 5, 18] {
                if exponent < 0 && a <= 0 && b >= 0 {
                    continue;
                }
                let mut values = vec![a.clone().pow(exponent), b.clone().pow(exponent)];
                if exponent > 0 && a <= 0 && b >= 0 {
                    values.push(G::from(0));
                }
                let image = x.checked_pow(i64::from(exponent)).unwrap();
                assert_eq!(mpq(image.lower().unwrap()), *values.iter().min().unwrap());
                assert_eq!(mpq(image.upper().unwrap()), *values.iter().max().unwrap());
            }
        }
    }
}

#[cfg(not(target_env = "msvc"))]
#[test]
fn native_growth_boundary_is_checked_against_completed_exact_output() {
    use rational_oracle::mpq;
    use rug::{Integer, Rational as G};
    // For exact 2, the native bound is (2 numerator bits + 1 denominator bit)
    // times exponent <= 65536. The successful neighbor must really complete.
    let exact = Integer::from(1) << 21_845u32;
    let positive = I::point(2.into()).checked_pow(21_845).unwrap();
    let negative = I::point(2.into()).checked_pow(-21_845).unwrap();
    assert_eq!(mpq(positive.lower().unwrap()), G::from(exact.clone()));
    assert_eq!(mpq(negative.lower().unwrap()), G::from((1, exact)));
    assert_eq!(positive.mul(&negative), I::point(R::one()));
    assert!(positive.is_singleton());
    assert!(negative.is_singleton());
    // Modest exponents use the inherited amplification contract, not a newly
    // invented universal 65536-bit cap. A large caller-owned input stays valid.
    let source: R = (Integer::from(1) << 400u32).to_string().parse().unwrap();
    let image = I::point(source).checked_pow(255).unwrap();
    assert_eq!(
        mpq(image.lower().unwrap()),
        G::from(Integer::from(1) << 102_000u32)
    );
}

#[cfg(feature = "serde")]
#[test]
fn generated_powers_replay_and_feed_further_exact_operations() {
    for x in [I::empty(), interval(-5, -3, 4), interval(3, 5, 4)] {
        let generated = x.checked_pow(3).unwrap().checked_pow(-2).unwrap();
        let bytes = serde_json::to_vec(&generated).unwrap();
        let restored: I = serde_json::from_slice(&bytes).unwrap();
        assert_eq!(restored, generated);
        assert_eq!(serde_json::to_vec(&restored).unwrap(), bytes);
        assert_eq!(restored.checked_pow(-3), generated.checked_pow(-3));
        assert_eq!(restored.square(), generated.square());
        if !x.is_empty() {
            assert_eq!(
                generated,
                I::new(
                    R::fraction(4096, 15625).unwrap(),
                    R::fraction(4096, 729).unwrap()
                )
                .unwrap()
            );
        }
    }
}

#[cfg(feature = "dyadic")]
#[test]
fn powered_rationals_convert_outward_and_continue_on_dyadic_grids() {
    use hyperinterval::DyadicBudget;
    let budget = || DyadicBudget::new(4096, 1_000_000_000);
    let source = interval(3, 5, 4)
        .checked_pow(3)
        .unwrap()
        .checked_pow(-2)
        .unwrap();
    let mut previous = None;
    for precision in [-8, -32, -128] {
        let dyadic = source.to_dyadic(precision, &mut budget()).unwrap();
        let exact_bounds = dyadic.to_rational(&mut budget()).unwrap();
        assert!(source.subset_of(&exact_bounds));
        if let Some(previous) = previous {
            assert!(exact_bounds.subset_of(&previous));
        }
        assert!(
            source.square().subset_of(
                &dyadic
                    .checked_pow(2, &mut budget())
                    .unwrap()
                    .to_rational(&mut budget())
                    .unwrap()
            )
        );
        previous = Some(exact_bounds);
    }
    assert_eq!(
        source.to_dyadic(-32, &mut DyadicBudget::new(4096, 0)),
        Err(IntervalError::WorkExhausted)
    );
    assert!(source.to_dyadic(-32, &mut budget()).is_ok());
}
