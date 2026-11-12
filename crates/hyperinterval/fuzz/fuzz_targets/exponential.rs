#![no_main]
#[path = "../../../hyperreal/tests/support/mpfr.rs"]
mod mpfr_cache;
use hyperinterval::{DomainCoverage, DyadicBudget, DyadicInterval, RationalInterval};
use hyperreal::Rational;
use libfuzzer_sys::fuzz_target;
use rug::Rational as Q;
#[path = "../../tests/support/exp_oracle.rs"]
mod oracle;

fuzz_target!(|data: [i16; 4]| {
    let [a, b, c, p] = data;
    let (a, b) = (
        i64::from(a.rem_euclid(17) - 8),
        i64::from(b.rem_euclid(17) - 8),
    );
    let (a, b) = (a.min(b), a.max(b));
    let denominator = 1 + u64::from(c.unsigned_abs() % 64);
    let exponent = i32::from(c % 3);
    let precision = -i32::from(p.unsigned_abs() % 73);
    let budget = || DyadicBudget::new(4096, 100_000_000_000);
    let dyadic = DyadicInterval::new(a.into(), b.into(), exponent).unwrap();
    let rational = RationalInterval::new(
        Rational::fraction(a, denominator).unwrap(),
        Rational::fraction(b, denominator).unwrap(),
    )
    .unwrap();
    for (image, lo, hi) in [
        (
            dyadic.exp(precision, &mut budget()).unwrap(),
            oracle::q(&a, exponent),
            oracle::q(&b, exponent),
        ),
        (
            rational.exp(precision, &mut budget()).unwrap(),
            Q::from((a, denominator)),
            Q::from((b, denominator)),
        ),
    ] {
        assert_eq!(image.domain, DomainCoverage::Complete);
        oracle::verify(&image.range, &lo, &hi, precision);
        let restored: DyadicInterval =
            serde_json::from_slice(&serde_json::to_vec(&image.range).unwrap()).unwrap();
        assert_eq!(restored, image.range);
        let coarse = restored.regrid(precision + 4, &mut budget()).unwrap();
        assert!(restored.subset_of(&coarse, &mut budget()).unwrap());
    }
});
