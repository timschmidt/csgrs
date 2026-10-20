#[cfg(test)]
mod range_reduction_tests {
    use super::*;

    #[test]
    fn opaque_huge_arguments_have_a_certified_near_integer_quotient() {
        let huge = BigInt::one() << 1100;
        for divisor in [Computable::pi(), Computable::pi().shift_left(-1), Computable::ln2()] {
            for sign in [-1, 1] {
                let n: BigInt = &huge * sign;
                let input = divisor.clone().multiply(Computable::rational(Rational::from_bigint(n.clone())))
                    .add(Computable::rational(Rational::fraction(1, 8).unwrap()));
                let actual = input.integer_ratio_nearest(divisor.clone());
                assert!((actual - n).abs() <= BigInt::one(), "range reduction lost periods");
            }
        }
    }

    #[test]
    fn optional_exp_reduction_declines_unrepresentable_binary_scales() {
        let huge = BigInt::one() << 1100;
        for sign in [-1, 1] {
            let exponent = Computable::rational(Rational::from_bigint(&huge * sign));
            assert!(exponent.reduced_exp().is_none());
            let lazy = exponent.exp();
            assert!(matches!(lazy.internal.approximation, Approximation::PrescaledExp(_)));
            assert!(lazy.cached().is_none());
        }
    }
}
