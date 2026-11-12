#![no_main]
#[path = "../../../hyperreal/tests/support/mpfr.rs"]
mod mpfr_cache;
use hyperinterval::{
    DomainCoverage, DyadicBudget, DyadicInterval, ExtendedInterval, IntervalError, RationalInterval,
};
use hyperreal::Rational;
use libfuzzer_sys::fuzz_target;
use rug::Rational as Q;
#[path = "../../tests/support/log_oracle.rs"]
mod oracle;
fuzz_target!(|data: [i16; 4]| {
    let [a, b, c, p] = data;
    let (a, b) = (
        i64::from(a.rem_euclid(17) - 8),
        i64::from(b.rem_euclid(17) - 8),
    );
    let (a, b) = (a.min(b), a.max(b));
    let denominator = 1 + u64::from(c.unsigned_abs() % 64);
    let exponent = i32::from(c % 8);
    let precision = -i32::from(p.unsigned_abs() % 65);
    let budget = || DyadicBudget::new(4096, 100_000_000_000);
    let dyadic = DyadicInterval::new(a.into(), b.into(), exponent).unwrap();
    let rational = RationalInterval::new(
        Rational::fraction(a, denominator).unwrap(),
        Rational::fraction(b, denominator).unwrap(),
    )
    .unwrap();
    for (result, lo, hi) in [
        (
            dyadic.log(precision, &mut budget()),
            oracle::q(&a, exponent),
            oracle::q(&b, exponent),
        ),
        (
            rational.log(precision, &mut budget()),
            Q::from((a, denominator)),
            Q::from((b, denominator)),
        ),
    ] {
        if b <= 0 {
            assert_eq!(result, Err(IntervalError::EmptyDomain));
            continue;
        }
        let result = result.unwrap();
        assert_eq!(
            result.domain,
            if a <= 0 {
                DomainCoverage::Partial
            } else {
                DomainCoverage::Complete
            }
        );
        oracle::verify(&result.range, &lo, &hi, precision);
        let restored: ExtendedInterval<DyadicInterval> =
            serde_json::from_slice(&serde_json::to_vec(&result.range).unwrap()).unwrap();
        assert_eq!(restored, result.range);
        let negated = restored.neg(&mut budget()).unwrap();
        let hull = restored.hull(&negated, &mut budget()).unwrap();
        assert!(restored.subset_of(&hull, &mut budget()).unwrap());
        assert_eq!(
            restored.intersection(&hull, &mut budget()).unwrap(),
            restored
        );
        // Log outputs remain valid inputs to the next monotone function.
        let back = restored.exp(precision, &mut budget()).unwrap();
        assert!(!back.range.is_empty());
    }
    let power = DyadicInterval::point(1.into(), exponent)
        .unwrap()
        .log(precision, &mut budget())
        .unwrap();
    oracle::log_two_multiple(&power.range, i64::from(exponent), precision);
});
