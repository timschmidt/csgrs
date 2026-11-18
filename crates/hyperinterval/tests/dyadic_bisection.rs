#![cfg(feature = "dyadic")]
use hyperinterval::{DyadicBudget, DyadicInterval, IntervalError};
use num_bigint::BigInt;

fn d(a: i128, b: i128, e: i32) -> DyadicInterval {
    DyadicInterval::new(a.into(), b.into(), e).unwrap()
}

#[test]
fn both_children_match_integer_midpoint_and_preserve_the_source() {
    let mut budget = DyadicBudget::new(1024, 1_000_000_000);
    for (a, b) in (-8..=8)
        .flat_map(|a| (a + 1..=9).map(move |b| (a, b)))
        .chain([(i128::from(i64::MIN), i128::from(i64::MAX))])
    {
        for e in [i32::MIN, -44, 0, i32::MAX] {
            let source = d(a, b, e);
            let result = source.bisect(&mut budget);
            if e == i32::MIN && (a + b) % 2 != 0 {
                assert_eq!(result, Err(IntervalError::Overflow));
                continue;
            }
            let (lo, mid, hi, scale) = if e == i32::MIN {
                (a, (a + b) / 2, b, e)
            } else {
                (2 * a, a + b, 2 * b, e - 1)
            };
            let [left, right] = result.unwrap().unwrap();
            assert_eq!(left, d(lo, mid, scale));
            assert_eq!(right, d(mid, hi, scale));
            assert_eq!(left.hull(&right, &mut budget).unwrap(), source);
            assert_eq!(
                left.intersection(&right, &mut budget).unwrap(),
                d(mid, mid, scale)
            );
        }
    }
}

#[test]
fn exhausted_splits_can_retry_and_feed_a_second_split() {
    let source = d(-3, 4, -20);
    for (bits, work, error) in [
        (3, 100, IntervalError::SizeExhausted),
        (4, 15, IntervalError::WorkExhausted),
    ] {
        assert_eq!(
            source.bisect(&mut DyadicBudget::new(bits, work)),
            Err(error)
        );
    }
    let mut exact = DyadicBudget::new(4, 16);
    let [left, right] = source.bisect(&mut exact).unwrap().unwrap();
    assert_eq!(exact.remaining_work(), 0);
    assert_eq!(right, d(1, 8, -21));
    assert_eq!(
        left.bisect(&mut DyadicBudget::new(4, 16)).unwrap(),
        Some([d(-12, -5, -22), d(-5, 2, -22)])
    );
    for unsplittable in [
        DyadicInterval::empty(),
        d(0, 0, i32::MIN),
        d(-3, -3, i32::MAX),
    ] {
        assert_eq!(unsplittable.bisect(&mut DyadicBudget::new(0, 0)), Ok(None));
    }
}

#[test]
fn split_retains_wide_mantissas() {
    let n = BigInt::from(1u8) << 8191usize;
    let source = DyadicInterval::new(-&n, &n + 1u8, -8192).unwrap();
    let expected = [
        DyadicInterval::new(-(&n << 1usize), 1.into(), -8193).unwrap(),
        DyadicInterval::new(1.into(), (&n + 1u8) << 1usize, -8193).unwrap(),
    ];
    assert_eq!(
        source
            .bisect(&mut DyadicBudget::new(8193, 4 * 8193))
            .unwrap(),
        Some(expected)
    );
}

#[cfg(all(feature = "rational", feature = "dispatch-trace"))]
#[test]
fn scalar_subdivision_does_not_promote() {
    use hyperreal::dispatch_trace;
    dispatch_trace::reset();
    let source = d(-3, 4, -20);
    let result = dispatch_trace::with_recording(|| source.bisect(&mut DyadicBudget::new(4, 16)));
    assert_eq!(result.unwrap(), Some([d(-6, 1, -21), d(1, 8, -21)]));
    assert!(dispatch_trace::take_trace().dispatch.is_empty());
}
