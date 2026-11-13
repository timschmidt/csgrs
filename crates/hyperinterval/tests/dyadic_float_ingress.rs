#![cfg(all(
    feature = "dyadic",
    any(feature = "float-proposal", feature = "float-enclosure")
))]
use hyperinterval::{DyadicBudget, DyadicInterval, IntervalError};

fn budget() -> DyadicBudget {
    DyadicBudget::new(4096, 1_000_000_000)
}

#[test]
fn finite_ieee_points_preserve_exact_values_and_compose() {
    // Independent binary64 values, including both sides of the normal boundary.
    for (bits, mantissa, exponent) in [
        (0, 0i64, 0),
        (1, 1, -1074),
        (2, 1, -1073),
        (0x000f_ffff_ffff_ffff, (1 << 52) - 1, -1074),
        (0x0010_0000_0000_0000, 1, -1022),
        (0x0010_0000_0000_0001, (1 << 52) + 1, -1074),
        (0x3ff0_0000_0000_0000, 1, 0),
        (0x3ff0_0000_0000_0001, (1 << 52) + 1, -52),
        (0x3fb9_9999_9999_999a, 3_602_879_701_896_397, -55),
        (0x7fef_ffff_ffff_ffff, (1 << 53) - 1, 971),
    ] {
        for negative in [false, true] {
            let bits = bits | if negative { 1 << 63 } else { 0 };
            let m = if negative { -mantissa } else { mantissa };
            let imported = DyadicInterval::from_f64(f64::from_bits(bits), &mut budget()).unwrap();
            assert_eq!(imported, DyadicInterval::point(m.into(), exponent).unwrap());
            assert!(!imported.is_empty());
            assert_eq!(
                imported.add(&imported, &mut budget()).unwrap(),
                DyadicInterval::point((2 * m).into(), exponent).unwrap()
            );
        }
    }
    let a = DyadicInterval::from_f64(0.1, &mut budget()).unwrap();
    let b = DyadicInterval::from_f64(0.2, &mut budget()).unwrap();
    let c = DyadicInterval::from_f64(0.3, &mut budget()).unwrap();
    assert_ne!(a.add(&b, &mut budget()).unwrap(), c);
    let tiny = DyadicInterval::from_f64(f64::from_bits(1), &mut budget()).unwrap();
    let zero = DyadicInterval::from_f64(-0.0, &mut budget()).unwrap();
    assert_eq!(
        zero.hull(&tiny, &mut budget()).unwrap(),
        DyadicInterval::new(0.into(), 1.into(), -1074).unwrap()
    );
}

#[test]
fn validation_precedes_allocation_and_reservations_are_retriable() {
    for bits in [
        0x7ff0_0000_0000_0000,
        0x7ff0_0000_0000_0001,
        0x7ff8_0000_0000_0000,
        0x7fff_ffff_ffff_ffff,
    ] {
        for sign in [0, 1 << 63] {
            let mut denied = DyadicBudget::new(0, 0);
            assert_eq!(
                DyadicInterval::from_f64(f64::from_bits(bits | sign), &mut denied),
                Err(IntervalError::NonFiniteEndpoint)
            );
            assert_eq!(denied.remaining_work(), 0);
        }
    }
    // Account for the decoded significand before canonical normalization.
    for (value, bits, work) in [
        (0.0, 0, 2),
        (f64::from_bits(1), 1, 2),
        (1.0, 53, 106),
        (f64::MAX, 53, 106),
    ] {
        let mut exact = DyadicBudget::new(bits, work);
        let expected = DyadicInterval::from_f64(value, &mut exact).unwrap();
        assert_eq!(exact.remaining_work(), 0);
        let mut short = DyadicBudget::new(bits, work - 1);
        assert_eq!(
            DyadicInterval::from_f64(value, &mut short),
            Err(IntervalError::WorkExhausted)
        );
        assert_eq!(short.remaining_work(), work - 1);
        if bits > 0 {
            let mut small = DyadicBudget::new(bits - 1, work);
            assert_eq!(
                DyadicInterval::from_f64(value, &mut small),
                Err(IntervalError::SizeExhausted)
            );
            assert_eq!(small.remaining_work(), work);
        }
        assert_eq!(
            DyadicInterval::from_f64(value, &mut budget()).unwrap(),
            expected
        );
    }
}

#[cfg(all(feature = "rational", feature = "dispatch-trace"))]
#[test]
fn native_ingress_never_dispatches_through_exact_real_arithmetic() {
    use hyperreal::dispatch_trace;
    for value in [0.0, -0.0, f64::from_bits(1), 0.1, -f64::MAX, f64::NAN] {
        dispatch_trace::reset();
        let imported =
            dispatch_trace::with_recording(|| DyadicInterval::from_f64(value, &mut budget()));
        assert_eq!(imported.is_ok(), value.is_finite());
        assert!(dispatch_trace::take_trace().dispatch.is_empty());
    }
}

#[cfg(feature = "rational")]
#[test]
fn seeded_ieee_patterns_match_independent_rational_ingress() {
    use hyperinterval::RationalInterval;
    use hyperreal::Rational;
    let mut seed = 0xabc7_29d4_51f0_03bdu64;
    for _ in 0..4096 {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        let value = f64::from_bits(seed);
        match Rational::try_from(value) {
            Ok(expected) => {
                let actual = DyadicInterval::from_f64(value, &mut budget()).unwrap();
                assert_eq!(
                    actual.to_rational(&mut budget()).unwrap(),
                    RationalInterval::point(expected)
                );
            }
            Err(_) => assert_eq!(
                DyadicInterval::from_f64(value, &mut budget()),
                Err(IntervalError::NonFiniteEndpoint)
            ),
        }
    }
}
