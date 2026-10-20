use super::*;
use rug::{Float, Integer, Rational as RugRational, float::Round};

#[test]
fn negative_exp_tail_includes_precision_endpoints_without_integer_overflow() {
    let argument = Computable::rational("-1e320".parse().unwrap());
    for precision in [i32::MIN, i32::MIN + 1, -4096, -128, -1, 0, 1, i32::MAX] {
        assert_eq!(exp(&None, &argument, precision), BigInt::zero());
    }
}

#[test]
fn negative_exp_tail_threshold_and_adjacent_values_match_directed_mpfr() {
    crate::mpfr_test::cleanup_on_thread_exit();
    for precision in [4, 1, 0, -1, -4, -16, -64, -128, -512] {
        for offset in -17..=17 {
            // Probe both sides of the proof threshold, including the rough
            // sample's one-unit error. The oracle never uses our exp kernel.
            let numerator = i64::from(precision.min(0)) * 16 + offset;
            let exact = Rational::fraction(numerator, 16).unwrap();
            let argument = Computable::rational(exact);
            let oracle = RugRational::from((numerator, 16));
            let mut lower = Float::with_val(1024, &oracle);
            let mut upper = lower.clone();
            lower.exp_round(Round::Down);
            upper.exp_round(Round::Up);
            lower >>= precision;
            upper >>= precision;
            let actual =
                Integer::from_str_radix(&exp(&None, &argument, precision).to_string(), 10).unwrap();
            for bound in [lower, upper] {
                assert!(
                    Float::with_val(1024, &actual - bound).abs() < 1,
                    "exp({numerator}/16), precision={precision} exceeds one unit"
                );
            }
        }
    }
}
