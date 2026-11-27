#[cfg(test)]
/// Large-quotient and unrepresentable-scale regressions for optional range reduction.
mod range_reduction_tests {
    use super::*;

    #[test]
    /// For three positive divisors and both signs of 2^1100, check quotient recovery from divisor*n+1/8.
    /// Require the chosen integer to differ from n by at most one in all six cases.
    /// The fixture does not assert a uniquely nearest or correctly rounded quotient,
    /// nor validate arbitrary divisor domains or interrupted evaluation.
    fn opaque_huge_arguments_have_a_certified_near_integer_quotient() {
        let huge = BigInt::one() << 1100;
        for divisor in [
            Computable::pi(),
            Computable::pi().shift_left(-1),
            Computable::ln2(),
        ] {
            for sign in [-1, 1] {
                let n: BigInt = &huge * sign;
                let input = divisor
                    .clone()
                    .multiply(Computable::rational(Rational::from_bigint(n.clone())))
                    .add(Computable::rational(Rational::fraction(1, 8).unwrap()));
                let actual = input.integer_ratio_nearest(divisor.clone());
                assert!(
                    (actual - n).abs() <= BigInt::one(),
                    "range reduction lost periods"
                );
            }
        }
    }

    #[test]
    /// Require both signs of a 2^1100 exponent to decline the optional reduced-exp representation.
    /// Public construction must retain a PrescaledExp node with no numeric cache.
    /// This checks lazy construction only; it does not approximate
    /// the enormous exponential or claim a bounded completed evaluation.
    fn optional_exp_reduction_declines_unrepresentable_binary_scales() {
        let huge = BigInt::one() << 1100;
        for sign in [-1, 1] {
            let exponent = Computable::rational(Rational::from_bigint(&huge * sign));
            assert!(exponent.reduced_exp().is_none());
            let lazy = exponent.exp();
            assert!(matches!(
                lazy.internal.approximation,
                Approximation::PrescaledExp(_)
            ));
            assert!(lazy.cached().is_none());
        }
    }
}
