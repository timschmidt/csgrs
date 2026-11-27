// Non-interned source values make lifetime assertions independent of the small-integer pool.
use super::*;
use std::sync::{Arc, Barrier};

fn exact_reference(value: &Rational) -> num::BigRational {
    num::BigRational::new(
        BigInt::from_biguint(value.sign(), value.numerator().clone()),
        BigInt::from(value.denominator().clone()),
    )
}

#[test]
fn initial_cache_kinds_preserve_two_linear_pairs_and_both_unary_results() {
    for first in 0..4 {
        let left = Rational::new(5_000_000_000_i64);
        let first_right = Rational::fraction(11, 1_000_000_000).unwrap();
        let second_right = Rational::fraction(13, 1_000_000_000).unwrap();
        match first {
            0 => {
                let _ = &left + &first_right;
                let _ = &left + &first_right;
            }
            1 => {
                let _ = left.clone().inverse().unwrap();
            }
            2 => {
                let _ = -&left;
            }
            3 => {
                let _ = left.clone().extract_square_reduced_retained();
                let _ = left.clone().extract_square_reduced_retained();
            }
            _ => unreachable!(),
        }
        let inverse = left.clone().inverse().unwrap();
        let negative = -&left;
        let _ = &left + &first_right;
        let a = &left + &first_right;
        let _ = &left + &second_right;
        let b = &left + &second_right;
        let _ = left.clone().extract_square_reduced_retained();
        let square = left.clone().extract_square_reduced_retained();
        assert_eq!(
            a,
            Rational::fraction(5_000_000_000_000_000_011_i64, 1_000_000_000).unwrap()
        );
        assert_eq!(
            b,
            Rational::fraction(5_000_000_000_000_000_013_i64, 1_000_000_000).unwrap()
        );
        assert_eq!(inverse, Rational::fraction(1_i64, 5_000_000_000).unwrap());
        assert_eq!(negative, Rational::new(-5_000_000_000_i64));
        assert_eq!(square, (Rational::new(50_000), Rational::new(2)));
        assert!(Arc::ptr_eq(&a.0, &(&left + &first_right).0));
        assert!(Arc::ptr_eq(&b.0, &(&left + &second_right).0));
        assert!(Arc::ptr_eq(&inverse.0, &left.clone().inverse().unwrap().0));
        assert!(Arc::ptr_eq(&negative.0, &(-&left).0));
        let owner = Arc::downgrade(&left.0);
        drop(left);
        assert!(owner.upgrade().is_none());
        assert_eq!(&inverse * &negative, Rational::new(-1));
        assert_eq!(&b - &a, Rational::fraction(1, 500_000_000).unwrap());
    }
}

#[test]
fn concurrent_late_unary_publication_preserves_values_and_source_lifetime() {
    let left = Rational::new(5_000_000_000_i64);
    let one = Rational::fraction(11, 1_000_000_000).unwrap();
    let two = Rational::fraction(13, 1_000_000_000).unwrap();
    let _ = left.clone().extract_square_reduced_retained();
    let _ = left.clone().extract_square_reduced_retained();
    let _ = &left + &one;
    let _ = &left + &two;
    let barrier = Arc::new(Barrier::new(5));
    let owner = Arc::downgrade(&left.0);
    let threads = (0..4)
        .map(|worker| {
            let value = left.clone();
            let barrier = barrier.clone();
            std::thread::spawn(move || {
                barrier.wait();
                for _ in 0..32 {
                    let (inverse, negative) = if worker % 2 == 0 {
                        (value.clone().inverse().unwrap(), -&value)
                    } else {
                        let negative = -&value;
                        (value.clone().inverse().unwrap(), negative)
                    };
                    assert_eq!(inverse, Rational::fraction(1_i64, 5_000_000_000).unwrap());
                    assert_eq!(negative, Rational::new(-5_000_000_000_i64));
                    assert_eq!(&inverse * &negative, Rational::new(-1));
                }
            })
        })
        .collect::<Vec<_>>();
    drop(left);
    barrier.wait();
    for worker in threads {
        worker.join().unwrap();
    }
    assert!(owner.upgrade().is_none());
}

#[test]
fn wide_values_and_replaced_operands_keep_independent_exact_answers() {
    let numerator = (BigUint::one() << 513_usize) + 33_u8;
    let denominator = (BigUint::one() << 257_usize) + 73_u8;
    let value = Rational::from_parts_raw(Plus, numerator.clone(), denominator.clone());
    let reference = num::BigRational::new(BigInt::from(numerator), BigInt::from(denominator));
    let owner = Arc::downgrade(&value.0);
    let mut retained = Vec::new();
    for index in 1..=24_i64 {
        let other = Rational::fraction(index, 97).unwrap();
        let other_reference = num::BigRational::new(BigInt::from(index), BigInt::from(97));
        let weak_other = Arc::downgrade(&other.0);
        for _ in 0..3 {
            assert_eq!(
                exact_reference(&(&value + &other)),
                &reference + &other_reference
            );
            assert_eq!(
                exact_reference(&(&value - &other)),
                &reference - &other_reference
            );
            assert_eq!(
                exact_reference(&(&value * &other)),
                &reference * &other_reference
            );
            assert_eq!(exact_reference(&(-&value)), -&reference);
            assert_eq!(
                exact_reference(&value.clone().inverse().unwrap()),
                reference.recip()
            );
        }
        retained.push((&value + &other, &reference + other_reference));
        drop(other);
        assert!(weak_other.upgrade().is_none());
    }
    drop(value);
    assert!(owner.upgrade().is_none());
    for (actual, expected) in retained {
        assert_eq!(exact_reference(&actual), expected);
    }
}

/// `numerator · 2^80 / denominator`, wider than a machine word.
fn wide(numerator: i64, denominator: u64) -> Rational {
    &Rational::fraction(numerator, denominator).unwrap()
        * &Rational::from_bigint(BigInt::from(1_u8) << 80_u32)
}

#[test]
fn interned_constants_do_not_pin_results_computed_with_them() {
    // Word-sized arithmetic results come from the interned pools, which live
    // for the whole process (simulation states start out this small).
    let pooled = |make: &dyn Fn() -> Rational| {
        let (a, b) = (make(), make());
        assert!(Arc::ptr_eq(&a.0, &b.0), "{a:?} is not interned");
        a
    };
    let third = Rational::fraction(1, 3).unwrap();
    let constants = [
        pooled(&|| &Rational::fraction(3, 2).unwrap() * &Rational::new(2)),
        pooled(&|| &Rational::fraction(-3, 2).unwrap() * &Rational::new(2)),
        pooled(&|| &Rational::fraction(1, 128).unwrap() * &Rational::new(2)),
        pooled(&|| &Rational::fraction(-5, 21).unwrap() * &(&third * &Rational::new(9))),
        Rational::one(),
        Rational::minus_one(),
    ];
    for constant in &constants {
        let pooled = Arc::downgrade(&constant.0);
        // Wider than a word, so the interned owner may not keep the results.
        let value = wide(5_000_000_011, 1_000_000_007);
        // Fill the value's own write-once slots, so later results can only be
        // retained on the interned partner.
        for partner in 0..3_i64 {
            let partner = wide(17 + partner, 1_000_000_009);
            let _ = &value * &partner;
            let _ = &value + &partner;
            let _ = &value - &partner;
        }
        let product = &value * constant;
        let sum = &value + constant;
        let difference = constant - &value;
        let products = Arc::downgrade(&product.0);
        let sums = Arc::downgrade(&sum.0);
        let differences = Arc::downgrade(&difference.0);
        assert_eq!(
            exact_reference(&product),
            exact_reference(&value) * exact_reference(constant)
        );
        drop((product, sum, difference, value));
        assert!(
            products.upgrade().is_none(),
            "{constant:?} pinned a product"
        );
        assert!(sums.upgrade().is_none(), "{constant:?} pinned a sum");
        assert!(
            differences.upgrade().is_none(),
            "{constant:?} pinned a difference"
        );
        assert!(pooled.upgrade().is_some());
    }
}

#[test]
fn small_results_kept_by_interned_constants_start_no_chain() {
    let pooled = &Rational::fraction(3, 2).unwrap() * &Rational::new(2);
    let again = &Rational::fraction(3, 2).unwrap() * &Rational::new(2);
    assert!(Arc::ptr_eq(&pooled.0, &again.0), "3 is interned");
    // A word-sized result that the interned value may keep, and so lives as
    // long as the pool.
    let small = Rational::fraction(1_000_000_007, 1_000_000_009).unwrap();
    for partner in 0..3_i64 {
        let partner = Rational::fraction(1_000_000_011 + partner, 1_000_000_013).unwrap();
        let _ = &small * &partner;
        let _ = &small + &partner;
    }
    let kept = &small * &pooled;
    // Results computed from the kept value must not stay alive through it.
    let value = wide(5_000_000_011, 1_000_000_007);
    for partner in 0..3_i64 {
        let partner = wide(17 + partner, 1_000_000_009);
        let _ = &value * &partner;
        let _ = &value + &partner;
        let _ = &value - &partner;
    }
    let product = &value * &kept;
    let sum = &value + &kept;
    let products = Arc::downgrade(&product.0);
    let sums = Arc::downgrade(&sum.0);
    drop((product, sum, value, small, kept));
    assert!(
        products.upgrade().is_none(),
        "a kept small result pinned a product"
    );
    assert!(sums.upgrade().is_none(), "a kept small result pinned a sum");
}
