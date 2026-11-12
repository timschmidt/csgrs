#![no_main]
use hyperinterval::{
    DomainCoverage, DyadicBudget, DyadicInterval, ExtendedInterval as E, IntervalSet2,
    RationalInterval,
};
use hyperreal::Rational;
use libfuzzer_sys::fuzz_target;
use rug::Rational as Q;
#[path = "../../tests/support/tangent_oracle.rs"]
mod oracle;
fuzz_target!(|data: [i16; 4]| {
    let [a, b, c, p] = data;
    let a = i64::from(a.rem_euclid(17) - 8);
    let b = i64::from(b.rem_euclid(17) - 8);
    let (a, b) = (a.min(b), a.max(b));
    let e = i32::from(c % 5);
    let den = 1 + u64::from(c.unsigned_abs() % 64);
    let p = 2 - i32::from(p.unsigned_abs() % 35);
    let budget = || DyadicBudget::new(8192, 1_000_000_000_000);
    let dyadic = DyadicInterval::new(a.into(), b.into(), e).unwrap();
    let rational = RationalInterval::new(
        Rational::fraction(a, den).unwrap(),
        Rational::fraction(b, den).unwrap(),
    )
    .unwrap();
    for (result, lo, hi, cot) in [
        (
            dyadic.tan(p, &mut budget()),
            oracle::q(&a, e),
            oracle::q(&b, e),
            false,
        ),
        (
            dyadic.cot(p, &mut budget()),
            oracle::q(&a, e),
            oracle::q(&b, e),
            true,
        ),
        (
            rational.tan(p, &mut budget()),
            Q::from((a, den)),
            Q::from((b, den)),
            false,
        ),
        (
            rational.cot(p, &mut budget()),
            Q::from((a, den)),
            Q::from((b, den)),
            true,
        ),
    ] {
        oracle::verify(
            result.clone(),
            &Some((oracle::End::Value(lo), oracle::End::Value(hi))),
            p,
            cot,
        );
        if let Ok(image) = result {
            let bytes = serde_json::to_vec(&image.range).unwrap();
            let restored: IntervalSet2<DyadicInterval> = serde_json::from_slice(&bytes).unwrap();
            assert_eq!(restored, image.range);
            for component in restored.components() {
                let atan = component.atan(-16, &mut budget()).unwrap();
                assert_eq!(atan.domain, DomainCoverage::Complete);
                assert!(!atan.range.is_empty());
            }
        }
    }
    let entire = E::<DyadicInterval>::entire().tan(p, &mut budget()).unwrap();
    assert_eq!(entire.range.components(), &[E::entire()]);
    assert_eq!(entire.domain, DomainCoverage::Partial);
});
