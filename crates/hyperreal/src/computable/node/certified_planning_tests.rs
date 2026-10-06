#[cfg(test)]
mod certified_planning_tests {
    use super::*;
    use rug::{
        Float, Integer,
        float::{Constant, Round},
        ops::{AddAssignRound, DivAssignRound, MulAssignRound},
    };

    fn assert_enclosed(actual: BigInt, lower: Float, upper: Float, context: &str) {
        let answer = Float::with_val(
            256,
            Integer::from_str_radix(&actual.to_string(), 10).unwrap(),
        );
        let admitted_lower = Float::with_val_round(256, &lower - 1, Round::Down).0;
        let admitted_upper = Float::with_val_round(256, &upper + 1, Round::Up).0;
        assert!(
            admitted_lower <= answer && answer <= admitted_upper,
            "{context}: {actual} is outside one unit of [{lower}, {upper}]"
        );
    }

    fn pi_sum(terms: i64) -> Computable {
        let mut sum = Computable::rational(Rational::new(16));
        for _ in 0..terms {
            sum = sum.add(Computable::pi());
        }
        sum
    }

    #[test]
    fn chained_sum_product_uses_certified_precision() {
        for terms in [32, 128, 512, 1024] {
            let actual = pi_sum(terms).multiply(Computable::pi()).approx(0);
            let mut lower = Float::with_val_round(256, Constant::Pi, Round::Down).0;
            let mut upper = Float::with_val_round(256, Constant::Pi, Round::Up).0;
            let pi_lower = lower.clone();
            let pi_upper = upper.clone();
            lower.mul_assign_round(terms, Round::Down);
            upper.mul_assign_round(terms, Round::Up);
            lower.add_assign_round(16, Round::Down);
            upper.add_assign_round(16, Round::Up);
            lower.mul_assign_round(pi_lower, Round::Down);
            upper.mul_assign_round(pi_upper, Round::Up);
            assert_enclosed(actual, lower, upper, "(16 + n*pi)*pi");
        }
    }

    #[test]
    fn chained_sum_square_root_does_not_use_a_stale_zero_cutoff() {
        let actual = pi_sum(512).sqrt().approx(3);
        let mut lower = Float::with_val_round(256, Constant::Pi, Round::Down).0;
        let mut upper = Float::with_val_round(256, Constant::Pi, Round::Up).0;
        lower.mul_assign_round(512, Round::Down);
        upper.mul_assign_round(512, Round::Up);
        lower.add_assign_round(16, Round::Down);
        upper.add_assign_round(16, Round::Up);
        lower.sqrt_round(Round::Down);
        upper.sqrt_round(Round::Up);
        lower.div_assign_round(8, Round::Down);
        upper.div_assign_round(8, Round::Up);
        assert_enclosed(actual, lower, upper, "sqrt(16 + 512*pi)/8");
    }

    #[test]
    fn tolerance_comparison_does_not_hide_a_large_perturbation() {
        for terms in [0, 32, 512] {
            for negative in [false, true] {
                for negative_base in [false, true] {
                    let mut perturbation = pi_sum(terms);
                    if negative {
                        perturbation = perturbation.negate();
                    }
                    let mut base = Computable::pi().sqrt();
                    if negative_base {
                        base = base.negate();
                    }
                    let exact_order = if negative {
                        Ordering::Less
                    } else {
                        Ordering::Greater
                    };
                    // With no pi terms the gap is exactly 16 < 2^5. For
                    // the other cases, pi > 3 proves 16 + terms*pi > 2^5
                    // independently of any Hyperreal approximation.
                    let expected = if terms == 0 {
                        Ordering::Equal
                    } else {
                        assert!(16 + 3 * terms > 32);
                        exact_order
                    };
                    for value in [
                        base.clone().add(perturbation.clone()),
                        perturbation.clone().add(base.clone()),
                    ] {
                        assert_eq!(value.compare_absolute(&base, 5), expected);
                        assert_eq!(base.compare_absolute(&value, 5), expected.reverse());
                        assert_eq!(value.try_compare_to_until(&base, 5), Some(exact_order));
                        assert_eq!(base.try_compare_to_until(&value, 5), Some(exact_order.reverse()));
                    }
                }
            }
        }
    }

    #[test]
    fn near_cancelled_reciprocals_use_certified_precision() {
        for denominator in [256_u64, 1000, 65_536, 16_777_216] {
            for with_pi in [false, true] {
                let radicand = Rational::fraction((4 * denominator - 1) as i64, denominator).unwrap();
                let inverse = Computable::rational(Rational::new(2))
                    .add(Computable::rational(radicand).sqrt().negate())
                    .inverse();
                let value = if with_pi {
                    inverse.multiply(Computable::pi().inverse())
                } else {
                    inverse
                };
                let actual = value.approx(0);
                // Rationalize the oracle to avoid subtracting nearly equal
                // roots: 1/(2-sqrt(4-1/d)) = d*(2+sqrt(4-1/d)).
                let mut lower = Float::with_val(256, 4 * denominator - 1);
                let mut upper = lower.clone();
                lower.div_assign_round(denominator, Round::Down);
                upper.div_assign_round(denominator, Round::Up);
                lower.sqrt_round(Round::Down);
                upper.sqrt_round(Round::Up);
                lower.add_assign_round(2, Round::Down);
                upper.add_assign_round(2, Round::Up);
                lower.mul_assign_round(denominator, Round::Down);
                upper.mul_assign_round(denominator, Round::Up);
                if with_pi {
                    let pi_lower = Float::with_val_round(256, Constant::Pi, Round::Down).0;
                    let pi_upper = Float::with_val_round(256, Constant::Pi, Round::Up).0;
                    lower.div_assign_round(pi_upper, Round::Down);
                    upper.div_assign_round(pi_lower, Round::Up);
                }
                assert_enclosed(actual, lower, upper, "near-cancelled reciprocal");
            }
        }
    }
}
