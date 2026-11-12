#![no_main]
use hyperinterval::{
    DomainCoverage, DyadicBudget, DyadicInterval, ExtendedInterval, RationalInterval,
};
use hyperreal::Rational;
use libfuzzer_sys::fuzz_target;
use rug::Rational as Q;
#[path = "../../tests/support/trig_oracle.rs"]
mod oracle;
fuzz_target!(|data: [i16; 4]| {
    let [a, b, c, p] = data;
    let a = i64::from(a.rem_euclid(33) - 16);
    let b = i64::from(b.rem_euclid(33) - 16);
    let (a, b) = (a.min(b), a.max(b));
    let exponent = i32::from(c % 8);
    let denominator = 1 + u64::from(c.unsigned_abs() % 128);
    let precision = 2 - i32::from(p.unsigned_abs() % 67);
    let budget = || DyadicBudget::new(4096, 100_000_000_000);
    let d = DyadicInterval::new(a.into(), b.into(), exponent).unwrap();
    let r = RationalInterval::new(
        Rational::fraction(a, denominator).unwrap(),
        Rational::fraction(b, denominator).unwrap(),
    )
    .unwrap();
    for (image, lo, hi, cosine) in [
        (
            d.sin(precision, &mut budget()).unwrap(),
            oracle::q(&a, exponent),
            oracle::q(&b, exponent),
            false,
        ),
        (
            d.cos(precision, &mut budget()).unwrap(),
            oracle::q(&a, exponent),
            oracle::q(&b, exponent),
            true,
        ),
        (
            r.sin(precision, &mut budget()).unwrap(),
            Q::from((a, denominator)),
            Q::from((b, denominator)),
            false,
        ),
        (
            r.cos(precision, &mut budget()).unwrap(),
            Q::from((a, denominator)),
            Q::from((b, denominator)),
            true,
        ),
    ] {
        assert_eq!(image.domain, DomainCoverage::Complete);
        oracle::verify(&image.range, &lo, &hi, precision, cosine);
        let bytes = serde_json::to_vec(&image.range).unwrap();
        let restored: DyadicInterval = serde_json::from_slice(&bytes).unwrap();
        assert_eq!(restored, image.range);
        assert!(
            !restored
                .sin(precision, &mut budget())
                .unwrap()
                .range
                .is_empty()
        );
    }
    let entire = ExtendedInterval::<DyadicInterval>::entire()
        .cos(precision, &mut budget())
        .unwrap();
    assert_eq!(
        entire.range,
        DyadicInterval::new((-1).into(), 1.into(), precision.max(0)).unwrap()
    );
});
