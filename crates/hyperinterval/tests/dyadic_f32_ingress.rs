#![cfg(all(
    feature = "dyadic",
    any(feature = "float-proposal", feature = "float-enclosure")
))]

use hyperinterval::{DyadicBudget, DyadicInterval, IntervalError};

#[test]
fn source_width_budget_errors_and_retry() {
    for (raw, mantissa, exponent, bits) in [
        (0, 0i32, 0, 0),
        (1, 1, -149, 1),
        (2, 1, -148, 2),
        (0x007f_ffff, 0x7f_ffff, -149, 23),
        (0x0080_0000, 1, -126, 24),
        (0x3d_cc_cc_cd, 13_421_773, -27, 24),
        (0x3f80_0000, 1, 0, 24),
        (0x7f7f_ffff, 0xff_ffff, 104, 24),
    ] {
        let work = 2 * bits.max(1u64);
        for sign in [0, 1 << 31] {
            let value = f32::from_bits(raw | sign);
            let mut short = DyadicBudget::new(bits, work - 1);
            assert_eq!(
                DyadicInterval::from_f32(value, &mut short),
                Err(IntervalError::WorkExhausted)
            );
            assert_eq!(short.remaining_work(), work - 1);
            if bits > 0 {
                let mut small = DyadicBudget::new(bits - 1, work);
                assert_eq!(
                    DyadicInterval::from_f32(value, &mut small),
                    Err(IntervalError::SizeExhausted)
                );
                assert_eq!(small.remaining_work(), work);
            }
            let signed = if sign == 0 { mantissa } else { -mantissa };
            let mut exact = DyadicBudget::new(bits, work);
            let point = DyadicInterval::from_f32(value, &mut exact).unwrap();
            assert_eq!(exact.remaining_work(), 0);
            assert_eq!(
                point,
                DyadicInterval::point(signed.into(), exponent).unwrap()
            );
            let mut sum_budget = DyadicBudget::new(26, 1024);
            assert_eq!(
                point.add(&point, &mut sum_budget).unwrap(),
                DyadicInterval::point(signed.into(), exponent + 1).unwrap()
            );
        }
    }
    for payload in [0, 1, 0x40_0000, 0x7f_ffff] {
        for sign in [0, 1 << 31] {
            let mut denied = DyadicBudget::new(0, 0);
            assert_eq!(
                DyadicInterval::from_f32(f32::from_bits(sign | 0x7f80_0000 | payload), &mut denied),
                Err(IntervalError::NonFiniteEndpoint)
            );
            assert_eq!(denied.remaining_work(), 0);
        }
    }
}

#[cfg(feature = "rational")]
#[test]
fn exact_rational_oracle_for_each_exponent_and_subnormal_leading_bit() {
    use hyperinterval::RationalInterval;
    use hyperreal::Rational;
    let check = |raw| {
        let value = f32::from_bits(raw);
        let actual = DyadicInterval::from_f32(value, &mut DyadicBudget::new(24, 48)).unwrap();
        let expected = RationalInterval::point(Rational::try_from(value).unwrap());
        // Exact rational egress reserves four quadratic stages, including the
        // denominator grid. Its cost is separate from the scalar import above.
        let mut egress = DyadicBudget::new(174, 4 * 174 * 174);
        assert_eq!(actual.to_rational(&mut egress).unwrap(), expected);
    };
    for sign in [0, 1 << 31] {
        for field in 0..255 {
            for fraction in [0, 1, 0x40_0000, 0x55_5555, 0x7f_ffff] {
                check(sign | (field << 23) | fraction);
            }
        }
        for bit in 0..23 {
            check(sign | (1 << bit));
        }
    }
}
