#![cfg(feature = "dyadic")]
use hyperinterval::{DyadicBudget, DyadicInterval};

fn d(a: i64, b: i64, e: i32) -> DyadicInterval {
    DyadicInterval::new(a.into(), b.into(), e).unwrap()
}
fn budget() -> DyadicBudget {
    DyadicBudget::new(4096, 1_000_000_000)
}

#[test]
fn upper_exponent_product_is_representable() {
    assert_eq!(
        d(1, 1, i32::MAX).mul(&d(1, 1, 1), &mut budget()).unwrap(),
        d(2, 2, i32::MAX)
    );
}

#[test]
fn lower_exponent_product_hull_removes_common_factor() {
    assert_eq!(
        d(-1, 2, -1_073_741_824)
            .mul(&d(-1, 2, -1_073_741_825), &mut budget())
            .unwrap(),
        d(-1, 2, i32::MIN)
    );
}

#[test]
fn boundary_hulls_match_independent_integer_corner_oracle() {
    use hyperinterval::IntervalError;
    for (e, f) in [
        (i32::MAX - 1, 1),
        (i32::MAX - 1, 2),
        (i32::MAX - 1, 4),
        (-1_073_741_824, -1_073_741_824),
        (-1_073_741_824, -1_073_741_825),
        (-1_073_741_824, -1_073_741_826),
    ] {
        for a in -4i64..=4 {
            for b in a..=4 {
                for c in -4i64..=4 {
                    for h in c..=4 {
                        let corners = [a * c, a * h, b * c, b * h];
                        let lo = *corners.iter().min().unwrap();
                        let hi = *corners.iter().max().unwrap();
                        let exponent = i64::from(e) + i64::from(f);
                        let expected = if exponent < i64::from(i32::MIN) {
                            let divisor = 1i64 << (i64::from(i32::MIN) - exponent);
                            if lo % divisor != 0 || hi % divisor != 0 {
                                Err(IntervalError::Overflow)
                            } else {
                                Ok(d(lo / divisor, hi / divisor, i32::MIN))
                            }
                        } else if exponent > i64::from(i32::MAX) {
                            let factor = 1i64 << (exponent - i64::from(i32::MAX));
                            Ok(d(lo * factor, hi * factor, i32::MAX))
                        } else {
                            Ok(d(lo, hi, exponent as i32))
                        };
                        let left = d(a, b, e);
                        let right = d(c, h, f);
                        assert_eq!(left.mul(&right, &mut budget()), expected);
                        assert_eq!(right.mul(&left, &mut budget()), expected);
                    }
                }
            }
        }
    }
}

#[test]
fn ordinary_products_preserve_exact_bounds_and_accounting() {
    for a in -4i64..=4 {
        for b in a..=4 {
            for c in -4i64..=4 {
                for h in c..=4 {
                    for (e, f) in [(-17, 11), (0, 0), (31, -47)] {
                        let left = d(a, b, e);
                        let right = d(c, h, f);
                        let mut work = budget();
                        let product = left.mul(&right, &mut work).unwrap();
                        let corners = [a * c, a * h, b * c, b * h];
                        assert_eq!(
                            product,
                            d(
                                *corners.iter().min().unwrap(),
                                *corners.iter().max().unwrap(),
                                e + f
                            )
                        );
                        let bits = |x: &DyadicInterval| {
                            let (lo, hi, _) = x.bounds().unwrap();
                            lo.bits().max(hi.bits())
                        };
                        let n = bits(&left) + bits(&right);
                        let expected_work = if (a == 0 && b == 0) || (c == 0 && h == 0) {
                            0
                        } else {
                            4 * n * n
                        };
                        assert_eq!(1_000_000_000 - work.remaining_work(), expected_work);
                    }
                }
            }
        }
    }
}

#[test]
fn normalization_limits_are_charged_before_growth_and_retry_is_independent() {
    use hyperinterval::IntervalError::{Overflow, SizeExhausted, WorkExhausted};
    let high = d(1, 1, i32::MAX);
    let factor = d(1, 1, 3);
    // Products reserve 2 bits and 16 units, then normalization needs 4 bits/8 units.
    let mut size = DyadicBudget::new(3, 100);
    assert_eq!(high.mul(&factor, &mut size), Err(SizeExhausted));
    assert_eq!(size.remaining_work(), 84);
    let mut work = DyadicBudget::new(4, 23);
    assert_eq!(high.mul(&factor, &mut work), Err(WorkExhausted));
    assert_eq!(work.remaining_work(), 7);
    let mut exact = DyadicBudget::new(4, 24);
    assert_eq!(high.mul(&factor, &mut exact).unwrap(), d(8, 8, i32::MAX));
    assert_eq!(exact.remaining_work(), 0);
    // Extreme positive growth is rejected before allocating billions of bits.
    assert_eq!(high.mul(&high, &mut budget()), Err(SizeExhausted));
    let low = d(-1, 2, -1_073_741_824);
    let lower = d(-1, 2, -1_073_741_825);
    let mut short = DyadicBudget::new(4, 69);
    assert_eq!(low.mul(&lower, &mut short), Err(WorkExhausted));
    assert_eq!(short.remaining_work(), 5);
    let mut enough = DyadicBudget::new(4, 70);
    assert_eq!(low.mul(&lower, &mut enough).unwrap(), d(-1, 2, i32::MIN));
    assert_eq!(enough.remaining_work(), 0);
    assert_eq!(
        d(1, 1, i32::MIN).mul(&d(1, 1, -1), &mut budget()),
        Err(Overflow)
    );
    // Product admission takes precedence on inputs formerly rejected by exponent sum.
    assert_eq!(
        high.mul(&factor, &mut DyadicBudget::new(1, 100)),
        Err(SizeExhausted)
    );
    assert_eq!(
        high.mul(&factor, &mut DyadicBudget::new(4, 0)),
        Err(WorkExhausted)
    );
    for value in [high, low, lower] {
        let mut none = DyadicBudget::new(0, 0);
        let empty = DyadicInterval::empty();
        let zero = d(0, 0, 0);
        assert_eq!(value.mul(&empty, &mut none).unwrap(), empty);
        assert_eq!(empty.mul(&value, &mut none).unwrap(), empty);
        assert_eq!(value.mul(&zero, &mut none).unwrap(), zero);
        assert_eq!(zero.mul(&value, &mut none).unwrap(), zero);
        assert_eq!(none.remaining_work(), 0);
    }
}

#[test]
fn normalized_products_feed_exact_composition_and_replay() {
    for (left, right, expected) in [
        (d(1, 2, i32::MAX), d(1, 1, 1), d(2, 4, i32::MAX)),
        (
            d(-1, 2, -1_073_741_824),
            d(-1, 2, -1_073_741_825),
            d(-1, 2, i32::MIN),
        ),
    ] {
        let output = left.mul(&right, &mut budget()).unwrap();
        assert_eq!(output, expected);
        assert_eq!(
            output
                .neg(&mut budget())
                .unwrap()
                .neg(&mut budget())
                .unwrap(),
            expected
        );
        assert_eq!(output.mul(&d(1, 1, 0), &mut budget()).unwrap(), expected);
        assert_eq!(output.add(&d(0, 0, 0), &mut budget()).unwrap(), expected);
        assert_eq!(
            output.intersection(&expected, &mut budget()).unwrap(),
            expected
        );
        #[cfg(feature = "serde")]
        {
            let bytes = serde_json::to_string(&output).unwrap();
            let replay: DyadicInterval = serde_json::from_str(&bytes).unwrap();
            assert_eq!(serde_json::to_string(&replay).unwrap(), bytes);
            assert_eq!(replay.mul(&d(1, 1, 0), &mut budget()).unwrap(), expected);
        }
    }
}

#[cfg(all(feature = "rational", feature = "dispatch-trace"))]
#[test]
fn products_do_not_dispatch_through_exact_real() {
    use hyperreal::dispatch_trace;
    for (a, b) in [
        (d(-3, 7, -8), d(-5, 9, 2)),
        (d(1, 1, i32::MAX), d(1, 1, 1)),
        (d(-1, 2, -1_073_741_824), d(-1, 2, -1_073_741_825)),
    ] {
        dispatch_trace::reset();
        let result = dispatch_trace::with_recording(|| a.mul(&b, &mut budget()));
        assert!(result.is_ok());
        assert!(dispatch_trace::take_trace().dispatch.is_empty());
    }
}
