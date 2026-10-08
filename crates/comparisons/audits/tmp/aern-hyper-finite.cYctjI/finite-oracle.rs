use hyperreal::Rational;
use num::{BigInt, BigUint, bigint::Sign};
use rug::{Integer, Rational as GmpRational, integer::Order};
fn input(case: &str) -> Rational {
    let max = || Rational::try_from(f64::MAX).unwrap();
    let dyadic = |n: BigUint, shift: usize| Rational::from_bigint_fraction(
        BigInt::from(n), BigUint::from(1_u8) << shift).unwrap();
    match case {
        "exact_word" => Rational::fraction(3, 4).unwrap(),
        "exact_wide" => Rational::try_from(2.0_f64.powi(700)).unwrap(),
        "exact_subnormal" => Rational::try_from(f64::from_bits(17)).unwrap(),
        "inexact_word" => Rational::fraction((1_i64 << 54) + 1, 8).unwrap(),
        "inexact_wide" => dyadic((BigUint::from(1_u8) << 200) + BigUint::from(1_u8), 180),
        "non_dyadic" => Rational::fraction(1, 3).unwrap(),
        "non_dyadic_wide" => Rational::from_bigint_fraction(
            BigInt::from((BigUint::from(1_u8) << 4096) + BigUint::from(1_u8)),
            (BigUint::from(1_u8) << 4096) + BigUint::from(3_u8)).unwrap(),
        "near_max_below" => max() - Rational::one(),
        "near_max_above" => max() + Rational::one(),
        "near_max_negative" => -(max() - Rational::one()),
        "near_max_deep" => max() - dyadic(BigUint::from(1_u8), 4096),
        "overflow" => dyadic(BigUint::from(1_u8) << 1024, 0),
        "unsupported_subnormal" => dyadic(BigUint::from(3_u8), 1076),
        "exact_max" => max(),
        _ => panic!("unknown case"),
    }
}

fn main() {
    for name in ["exact_word","exact_wide","exact_subnormal","inexact_word","inexact_wide",
        "non_dyadic","non_dyadic_wide","near_max_below","near_max_above","near_max_negative",
        "near_max_deep","overflow","unsupported_subnormal","exact_max"] {
        let value = input(name);
        let mut exact = GmpRational::from((
            Integer::from_digits(&value.numerator().to_u64_digits(), Order::Lsf),
            Integer::from_digits(&value.denominator().to_u64_digits(), Order::Lsf)));
        if value.sign() == Sign::Minus { exact = -exact; }
        let bounds = value.to_f64_enclosure();
        let valid = bounds.is_none_or(|[lo,hi]| lo.is_finite() && hi.is_finite()
            && GmpRational::from_f64(lo).is_some_and(|lo| lo <= exact)
            && GmpRational::from_f64(hi).is_some_and(|hi| hi >= exact));
        println!("{name}: bits={:?}; finite-exact-enclosure={valid}", bounds.map(|b| b.map(f64::to_bits)));
    }
}
