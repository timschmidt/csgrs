use hyperreal::Rational;
use num::{BigInt, BigUint};

fn main() {
    let largest = Rational::try_from(f64::MAX).unwrap();
    for (label, value) in [
        ("MAX exact", largest.clone()),
        ("MAX minus one", &largest - Rational::one()),
        ("MAX plus one", &largest + Rational::one()),
        ("negative MAX plus one", -(&largest - Rational::one())),
        ("negative MAX minus one", -(&largest + Rational::one())),
        ("three-quarter subnormal", Rational::from_bigint_fraction(
            BigInt::from(3), BigUint::from(1_u8) << 1076).unwrap()),
    ] {
        let result = value.to_f64_enclosure();
        let finite = result.is_none_or(|bounds| bounds.iter().all(|x| x.is_finite()));
        let encloses = result.is_none_or(|[lo, hi]| {
            Rational::try_from(lo).is_ok_and(|r| r <= value)
                && Rational::try_from(hi).is_ok_and(|r| r >= value)
        });
        println!("{label}: {result:?}; finite={finite}; exact-enclosure={encloses}");
    }
}
