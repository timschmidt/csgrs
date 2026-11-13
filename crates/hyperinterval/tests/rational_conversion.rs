#![cfg(all(feature = "dyadic", feature = "rational"))]

use hyperinterval::{DyadicBudget, DyadicInterval, IntervalError, RationalInterval};
use hyperreal::Rational;
use num_bigint::{BigInt, BigUint};

fn budget() -> DyadicBudget {
    DyadicBudget::new(16_384, 1_000_000_000_000)
}

#[test]
fn rational_grid_bounds_match_independent_integer_extrema_and_compose() {
    for n in -20_i128..=20 {
        for d in 1_i128..=19 {
            let value = Rational::fraction(n as i64, d as u64).unwrap();
            for p in -8_i32..=8 {
                let (numerator, denominator) = if p < 0 {
                    (n << p.unsigned_abs(), d)
                } else {
                    (n, d << p)
                };
                let lo = numerator.div_euclid(denominator);
                let hi = lo + i128::from(numerator.rem_euclid(denominator) != 0);
                let expected = DyadicInterval::new(lo.into(), hi.into(), p).unwrap();
                let actual = DyadicInterval::from_rational(&value, p, &mut budget()).unwrap();
                assert_eq!(actual, expected, "{n}/{d} at {p}");
                assert_eq!(
                    RationalInterval::point(value.clone())
                        .to_dyadic(p, &mut budget())
                        .unwrap(),
                    actual
                );
                #[cfg(feature = "hyperreal")]
                assert_eq!(
                    DyadicInterval::from_real(
                        &hyperreal::Real::new(value.clone()),
                        p,
                        &mut budget()
                    )
                    .unwrap(),
                    actual
                );
                // Consume the new enclosure in native exact set/arithmetic paths.
                assert!(
                    actual
                        .subset_of(&actual.regrid(p + 1, &mut budget()).unwrap(), &mut budget())
                        .unwrap()
                );
                assert!(
                    actual
                        .sub(&actual, &mut budget())
                        .unwrap()
                        .contains_point(&0.into(), 0, &mut budget())
                        .unwrap()
                );
            }
        }
    }
}

#[test]
fn rational_conversion_limits_precede_copies_and_retry_is_independent() {
    for bits in [128_usize, 4096, 131072] {
        let value = Rational::from_bigint_fraction(
            BigInt::from((BigUint::from(1_u8) << bits) + BigUint::from(1_u8)),
            BigUint::from(3_u8),
        )
        .unwrap();
        let mut tiny = DyadicBudget::new(64, 1000);
        assert_eq!(
            DyadicInterval::from_rational(&value, 0, &mut tiny),
            Err(IntervalError::SizeExhausted)
        );
        assert_eq!(tiny.remaining_work(), 1000);
        let mut generous = DyadicBudget::new(200_000, 1_000_000_000_000);
        let result = DyadicInterval::from_rational(&value, 0, &mut generous).unwrap();
        assert!(
            result
                .to_rational(&mut generous)
                .unwrap()
                .contains_point(&value)
        );
    }
    for n in [-17, -1, 0, 1, 19] {
        for p in [-64, 0, 16] {
            let value = Rational::fraction(n, 7).unwrap();
            let mut measured = budget();
            let expected = DyadicInterval::from_rational(&value, p, &mut measured).unwrap();
            let work = budget().remaining_work() - measured.remaining_work();
            let mut exact = DyadicBudget::new(16_384, work);
            assert_eq!(
                DyadicInterval::from_rational(&value, p, &mut exact).unwrap(),
                expected
            );
            assert_eq!(exact.remaining_work(), 0);
            assert_eq!(
                DyadicInterval::from_rational(&value, p, &mut DyadicBudget::new(16_384, work - 1)),
                Err(IntervalError::WorkExhausted)
            );
            assert_eq!(
                DyadicInterval::from_rational(&value, p, &mut budget()).unwrap(),
                expected
            );
        }
    }
    for p in [i32::MIN, i32::MAX] {
        assert_eq!(
            DyadicInterval::from_rational(&Rational::one(), p, &mut budget()),
            Err(IntervalError::SizeExhausted)
        );
    }
}

#[test]
fn lazy_rational_conversion_keeps_stored_parts_and_cache_independent_cost() {
    let line = hyperreal::ExactDyadicLine2::from_f64([0.0, 0.0], [6.0, 6.0]).unwrap();
    let (_, point) = line.intersection_point_f64([0.0, 3.0], [6.0, 3.0]).unwrap();
    let mut unreduced = 0;
    for coordinate in &point {
        let value = coordinate.exact_rational_ref().unwrap();
        let reference = Rational::from(3);
        let parts = value.ratio_parts();
        let mut cold = budget();
        let actual = DyadicInterval::from_rational(value, -32, &mut cold).unwrap();
        assert!(
            actual
                .to_rational(&mut budget())
                .unwrap()
                .contains_point(&reference)
        );
        unreduced += usize::from(parts.1 != value.numerator() || parts.2 != value.denominator());
        let mut warm = budget();
        assert_eq!(
            DyadicInterval::from_rational(value, -32, &mut warm).unwrap(),
            actual
        );
        assert_eq!(cold.remaining_work(), warm.remaining_work());
        assert_eq!(parts, value.ratio_parts());
    }
    assert!(
        unreduced > 0,
        "fixture must exercise delayed canonicalization"
    );
}

#[cfg(all(feature = "hyperreal", feature = "dispatch-trace"))]
#[test]
fn rational_bridge_does_not_dispatch_refinement_or_canonicalization() {
    use hyperreal::{Real, dispatch_trace};
    let line = hyperreal::ExactDyadicLine2::from_f64([0.0, 0.0], [6.0, 6.0]).unwrap();
    let (_, point) = line.intersection_point_f64([0.0, 3.0], [6.0, 3.0]).unwrap();
    for value in &point {
        dispatch_trace::reset();
        let result =
            dispatch_trace::with_recording(|| DyadicInterval::from_real(value, -32, &mut budget()))
                .unwrap();
        assert!(dispatch_trace::take_trace().dispatch.is_empty());
        assert_eq!(result, DyadicInterval::point(3.into(), 0).unwrap());
    }
    let huge = Real::new(Rational::from_bigint(BigInt::from(1_u8) << 131072_usize));
    let mut limited = DyadicBudget::new(64, 1000);
    dispatch_trace::reset();
    let result =
        dispatch_trace::with_recording(|| DyadicInterval::from_real(&huge, 0, &mut limited));
    assert_eq!(result, Err(IntervalError::SizeExhausted));
    assert!(dispatch_trace::take_trace().dispatch.is_empty());
    // Only the existing grid admission has spent one unit.
    assert_eq!(limited.remaining_work(), 999);
}
