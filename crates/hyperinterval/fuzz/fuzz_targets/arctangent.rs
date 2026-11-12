#![no_main]
use hyperinterval::{
    DomainCoverage, DyadicBudget, DyadicInterval, ExtendedInterval, RationalInterval,
};
use hyperreal::Rational;
use libfuzzer_sys::fuzz_target;
use rug::Rational as Q;
#[path = "../../tests/support/atan_oracle.rs"]
mod oracle;
fuzz_target!(|data: [i16; 4]| {
    let [a, b, c, p] = data;
    let a = i64::from(a.rem_euclid(33) - 16);
    let b = i64::from(b.rem_euclid(33) - 16);
    let (a, b) = (a.min(b), a.max(b));
    let exponent = i32::from(c % 16);
    let denominator = 1 + u64::from(c.unsigned_abs() % 128);
    let precision = 2 - i32::from(p.unsigned_abs() % 131);
    let budget = || DyadicBudget::new(4096, 100_000_000_000);
    let d = DyadicInterval::new(a.into(), b.into(), exponent).unwrap();
    let r = RationalInterval::new(
        Rational::fraction(a, denominator).unwrap(),
        Rational::fraction(b, denominator).unwrap(),
    )
    .unwrap();
    for (image, lo, hi) in [
        (
            d.atan(precision, &mut budget()).unwrap(),
            oracle::q(&a, exponent),
            oracle::q(&b, exponent),
        ),
        (
            r.atan(precision, &mut budget()).unwrap(),
            Q::from((a, denominator)),
            Q::from((b, denominator)),
        ),
    ] {
        assert_eq!(image.domain, DomainCoverage::Complete);
        oracle::verify(&image.range, &lo, &hi, precision);
        let bytes = serde_json::to_vec(&image.range).unwrap();
        let restored: DyadicInterval = serde_json::from_slice(&bytes).unwrap();
        assert_eq!(restored, image.range);
        assert!(
            !restored
                .atan(precision, &mut budget())
                .unwrap()
                .range
                .is_empty()
        );
        let neg = restored.neg(&mut budget()).unwrap();
        assert!(
            restored
                .subset_of(&restored.hull(&neg, &mut budget()).unwrap(), &mut budget())
                .unwrap()
        );
    }
    let whole = ExtendedInterval::<DyadicInterval>::entire()
        .atan(precision, &mut budget())
        .unwrap();
    let (_, upper) = oracle::half_pi();
    oracle::finite(&whole.range, -upper.clone(), upper, precision);
});
