#![no_main]
use hyperinterval::{
    DomainCoverage, DyadicBudget, DyadicInterval, IntervalError, RationalInterval,
};
use hyperreal::Rational;
use libfuzzer_sys::fuzz_target;
use rug::Rational as Q;
#[path = "../../tests/support/sqrt_oracle.rs"]
mod sqrt_oracle;
use sqrt_oracle::{q, verify};

fuzz_target!(|data: [i16; 5]| {
    let [a, b, e, p, denominator] = data;
    let (a, b) = (i64::from(a.min(b)), i64::from(a.max(b)));
    let exponent = i32::from(e % 1024);
    let precision = i32::from(p % 128);
    let denominator = u64::from(denominator.unsigned_abs()) + 1;
    let budget = || DyadicBudget::new(4096, 1_000_000_000);
    let dyadic = DyadicInterval::new(a.into(), b.into(), exponent).unwrap();
    let rational = RationalInterval::new(
        Rational::fraction(a, denominator).unwrap(),
        Rational::fraction(b, denominator).unwrap(),
    )
    .unwrap();
    for (result, lower, upper) in [
        (
            dyadic.sqrt(precision, &mut budget()),
            q(&a.max(0), exponent),
            q(&b, exponent),
        ),
        (
            rational.sqrt(precision, &mut budget()),
            Q::from((a.max(0), denominator)),
            Q::from((b, denominator)),
        ),
    ] {
        if b < 0 {
            assert_eq!(result, Err(IntervalError::EmptyDomain));
            continue;
        }
        let result = result.unwrap();
        assert_eq!(
            result.domain,
            if a < 0 {
                DomainCoverage::Partial
            } else {
                DomainCoverage::Complete
            }
        );
        verify(&result.range, lower, upper, precision);
        let restored: DyadicInterval =
            serde_json::from_slice(&serde_json::to_vec(&result.range).unwrap()).unwrap();
        assert_eq!(restored, result.range);
        let root = restored.sqrt(precision - 16, &mut budget()).unwrap().range;
        let squared = root.square(&mut budget()).unwrap();
        assert!(restored.subset_of(&squared, &mut budget()).unwrap());
    }
});
