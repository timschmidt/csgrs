//! Independent directed-quotient expectations and resource contract witnesses.
use super::*;

fn assert_directed_bounds(n: &BigInt, d: &BigInt, shift: i64) {
    let mut budget = DyadicBudget::new(4096, 1_000_000_000);
    let (lower, upper) = scaled_quotient(n, d, shift, &mut budget).unwrap();
    let (mut a, mut b) = if shift >= 0 {
        (n << shift as usize, d.clone())
    } else {
        (n.clone(), d << (-shift) as usize)
    };
    if b < BigInt::zero() {
        a = -a;
        b = -b;
    }
    // Multiplication inequalities uniquely characterize floor and ceiling,
    // independently of the quotient/remainder algorithm being exercised.
    assert!(&lower * &b <= a && a < (&lower + 1u32) * &b);
    assert!((&upper - 1u32) * &b < a && a <= &upper * &b);
    assert!(&upper - &lower <= BigInt::one());
}

#[test]
fn directed_quotients_match_small_integer_extrema_and_large_limb_inequalities() {
    for n in -33i128..=33 {
        for d in -17i128..=17 {
            if d == 0 {
                continue;
            }
            for shift in [-12i64, -3, 0, 1, 12] {
                let (a, b) = if shift >= 0 {
                    (n << shift, d)
                } else {
                    (n, d << (-shift))
                };
                let (q, r) = (a / b, a % b);
                let floor = q - i128::from(r != 0 && (a < 0) != (b < 0));
                let ceil = q + i128::from(r != 0 && (a < 0) == (b < 0));
                let mut budget = DyadicBudget::new(4096, 1_000_000_000);
                assert_eq!(
                    scaled_quotient(&n.into(), &d.into(), shift, &mut budget).unwrap(),
                    (floor.into(), ceil.into())
                );
                assert_directed_bounds(&n.into(), &d.into(), shift);
            }
        }
    }
    for bits in [1usize, 31, 32, 63, 64, 65, 127, 128, 129, 256, 1024] {
        let denominator: BigInt = (BigInt::one() << bits) + 3;
        let numerators: [BigInt; 4] = [
            &denominator - 1,
            &denominator + 1,
            &denominator * 17,
            &denominator * 17 + 1,
        ];
        for numerator in numerators {
            for n in [numerator.clone(), -&numerator] {
                for d in [denominator.clone(), -&denominator] {
                    for shift in [-65, -1, 0, 1, 65] {
                        assert_directed_bounds(&n, &d, shift);
                    }
                }
            }
        }
    }
}

#[test]
fn quotient_admission_preserves_failure_precedence_and_exact_work_boundary() {
    let n = BigInt::from(7);
    let d = BigInt::from(3);
    // Scaling reserves max(3+4,2)+1 = 8 bits, with four quadratic stages.
    let mut size = DyadicBudget::new(7, 256);
    assert_eq!(
        scaled_quotient(&n, &d, 4, &mut size),
        Err(IntervalError::SizeExhausted)
    );
    assert_eq!(size.remaining_work(), 256);
    let mut work = DyadicBudget::new(8, 255);
    assert_eq!(
        scaled_quotient(&n, &d, 4, &mut work),
        Err(IntervalError::WorkExhausted)
    );
    assert_eq!(work.remaining_work(), 255);
    let mut exact = DyadicBudget::new(8, 256);
    assert_eq!(
        scaled_quotient(&n, &d, 4, &mut exact).unwrap(),
        (37.into(), 38.into())
    );
    assert_eq!(exact.remaining_work(), 0);
    for shift in [i64::MIN, i64::from(i32::MIN), i64::from(i32::MAX), i64::MAX] {
        let mut fresh = DyadicBudget::new(64, 100_000);
        assert_eq!(
            scaled_quotient(&n, &d, shift, &mut fresh),
            Err(IntervalError::SizeExhausted)
        );
        assert_eq!(fresh.remaining_work(), 100_000);
        assert_eq!(
            scaled_quotient(&BigInt::zero(), &d, shift, &mut fresh),
            Err(IntervalError::SizeExhausted)
        );
        assert_eq!(fresh.remaining_work(), 100_000);
        assert_eq!(
            scaled_quotient(&n, &BigInt::zero(), shift, &mut fresh),
            Err(IntervalError::ContainsZero)
        );
        assert_eq!(fresh.remaining_work(), 100_000);
    }
    let mut none = DyadicBudget::new(0, 0);
    assert_eq!(
        scaled_quotient(&n, &BigInt::zero(), 0, &mut none),
        Err(IntervalError::ContainsZero)
    );
    let mut overflow = DyadicBudget::new(u64::MAX, u64::MAX);
    assert_eq!(
        scaled_quotient(&n, &d, i64::MIN, &mut overflow),
        Err(IntervalError::WorkExhausted)
    );
    assert_eq!(overflow.remaining_work(), u64::MAX);
    let mut retry = DyadicBudget::new(8, 256);
    assert_eq!(
        scaled_quotient(&n, &d, 4, &mut retry).unwrap(),
        (37.into(), 38.into())
    );
    assert_eq!(retry.remaining_work(), 0);
}

#[cfg(all(feature = "rational", feature = "dispatch-trace"))]
#[test]
fn directed_quotient_never_enters_real_dispatch() {
    use hyperreal::dispatch_trace;
    for n in [-7, 0, 8] {
        for d in [-3, 0, 2] {
            dispatch_trace::reset();
            let _ = dispatch_trace::with_recording(|| {
                scaled_quotient(
                    &BigInt::from(n),
                    &BigInt::from(d),
                    -9,
                    &mut DyadicBudget::new(4096, 1_000_000),
                )
            });
            assert!(dispatch_trace::take_trace().dispatch.is_empty());
        }
    }
}
