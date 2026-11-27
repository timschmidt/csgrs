#[cfg(test)]
/// Exact integer rounding and symbolic offset preservation at precision/exponent boundaries.
mod precision_boundary_tests {
    use super::*;
    use num::Integer;

    #[test]
    /// Compare public and raw integer approximation against exact Euclidean-division rounding for 16,995 input/precision pairs.
    /// Halfway cases go toward positive infinity, including negative integers. Also require
    /// both One representations to agree with integer one over the same 33 precisions.
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
    /// Check zero at i32::MIN precision and nine signed small integers under extreme right shifts.
    /// Floor shift preserves a negative sign as -1, while nearest scaling and maximum
    /// positive approximation precision yield zero. No huge nonzero left-shift allocation is attempted.
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

    /// Interpret the restricted raw syntax as exact exponents (k,e) for pi^k * 2^e.
    /// Use i128 arithmetic to inspect fixture rewrites without evaluating huge integers
    /// or invoking production equality. Unsupported node tags panic as fixture errors;
    /// this recursive oracle is limited to constants pi/inverse-pi, offsets, squares and inverses.
    fn pi_monomial(value: &Computable) -> (i128, i128) {
        match &value.internal.approximation {
            Approximation::Constant(SharedConstant::Pi) => (1, 0),
            Approximation::Constant(SharedConstant::InvPi) => (-1, 0),
            Approximation::Offset(child, exponent) => {
                let (power, offset) = pi_monomial(child);
                (power, offset + i128::from(*exponent))
            }
            Approximation::Square(child) => {
                let (power, offset) = pi_monomial(child);
                (2 * power, 2 * offset)
            }
            Approximation::Inverse(child) => {
                let (power, offset) = pi_monomial(child);
                (-power, -offset)
            }
            _ => panic!("unexpected expression in the pi monomial oracle"),
        }
    }

    /// Twelve signed offsets spanning i32 endpoints and the half-range thresholds that squaring can cross.
    const OFFSET_BOUNDARIES: [i32; 12] = [
        i32::MIN,
        i32::MIN + 1,
        -1_073_741_825,
        -1_073_741_824,
        -8,
        -1,
        0,
        1,
        8,
        1_073_741_823,
        1_073_741_824,
        i32::MAX,
    ];

    /// Check a shifted pi, its square, its inverse and its twice-squared inverse against independent exponent pairs.
    /// Expected exponents use i128, preserving the meaning when no single i32 offset can store the result.
    fn assert_offset_rewrites(value: Computable, exponent: i32) {
        let exponent = i128::from(exponent);
        assert_eq!(pi_monomial(&value), (1, exponent));
        assert_eq!(pi_monomial(&value.clone().square()), (2, 2 * exponent));
        assert_eq!(pi_monomial(&value.clone().inverse()), (-1, -exponent));
        assert_eq!(
            pi_monomial(&value.square().square().inverse()),
            (-4, -4 * exponent)
        );
    }

    #[test]
    /// Apply all twelve boundary offsets through the native constructor and verify the full symbolic rewrite chain.
    fn offset_rewrites_preserve_values_at_exponent_limits() {
        for exponent in OFFSET_BOUNDARIES {
            assert_offset_rewrites(Computable::pi().shift_left(exponent), exponent);
        }
    }

    #[cfg(feature = "serde")]
    #[test]
    /// Deserialize raw pi-offset graphs at all twelve boundaries, then verify the same exponent-preserving rewrites.
    /// This checks semantic replay and later composition in this process, not byte-stable serialization or fresh-process state.
    fn deserialized_offset_rewrites_preserve_values_at_exponent_limits() {
        let pi = serde_json::to_value(Computable::pi()).unwrap();
        for exponent in OFFSET_BOUNDARIES {
            let value = serde_json::from_value(serde_json::json!({
                "internal": { "Offset": [pi, exponent] }
            }))
            .unwrap();
            assert_offset_rewrites(value, exponent);
        }
    }
}
