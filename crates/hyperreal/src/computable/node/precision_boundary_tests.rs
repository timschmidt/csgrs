#[cfg(test)]
mod precision_boundary_tests {
    use super::*;
    use num::Integer;

    #[test]
    fn integer_approximation_matches_exact_rounding() {
        for n in -257..=257 {
            let integer = BigInt::from(n);
            let value = Computable::integer(integer.clone());
            let raw = Approximation::Int(integer.clone());
            for p in -16_i32..=16 {
                let expected = if p <= 0 {
                    &integer * BigInt::from(2).pow(p.unsigned_abs())
                } else {
                    let divisor = BigInt::from(2).pow(p as u32);
                    let (quotient, remainder) = integer.div_mod_floor(&divisor);
                    if remainder * 2 >= divisor {
                        quotient + 1
                    } else {
                        quotient
                    }
                };
                assert_eq!(value.approx(p), expected, "integer {n}, precision {p}");
                assert_eq!(raw.approximate(&None, p), expected);
            }
        }
        for p in -16_i32..=16 {
            assert_eq!(
                Computable::one().approx(p),
                Computable::integer(1.into()).approx(p)
            );
            assert_eq!(
                Approximation::One.approximate(&None, p),
                Computable::one().approx(p)
            );
        }
    }

    #[test]
    fn zero_at_minimum_precision_and_extreme_right_shifts() {
        assert!(Computable::zero().approx(i32::MIN).is_zero());
        assert!(
            Approximation::Int(BigInt::zero())
                .approximate(&None, i32::MIN)
                .is_zero()
        );
        for n in [-65_537, -3, -2, -1, 0, 1, 2, 3, 65_537] {
            let integer = BigInt::from(n);
            assert_eq!(
                shift(integer.clone(), i32::MIN),
                if n < 0 { (-1).into() } else { 0.into() }
            );
            assert!(scale(integer.clone(), i32::MIN).is_zero());
            assert!(Computable::integer(integer).approx(i32::MAX).is_zero());
        }
    }
}
