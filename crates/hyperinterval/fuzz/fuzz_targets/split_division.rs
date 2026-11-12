#![no_main]
use hyperinterval::{DyadicBudget, DyadicInterval, ExtendedInterval as E, IntervalSet2};
use libfuzzer_sys::fuzz_target;
#[path = "../../tests/support/division_oracle.rs"]
mod oracle;
fn source(a: i64, b: i64, e: i32, kind: u16) -> (E<DyadicInterval>, oracle::Source) {
    use oracle::End::{Minus as M, Plus as P, Value as V};
    let point = |n: i64| DyadicInterval::point(n.into(), e).unwrap();
    match kind % 5 {
        0 => (
            E::from_finite(DyadicInterval::new(a.into(), b.into(), e).unwrap()),
            Some((V(oracle::q(&a, e)), V(oracle::q(&b, e)))),
        ),
        1 => (E::above(point(a)).unwrap(), Some((V(oracle::q(&a, e)), P))),
        2 => (E::below(point(b)).unwrap(), Some((M, V(oracle::q(&b, e))))),
        3 => (E::entire(), Some((M, P))),
        _ => (E::empty(), None),
    }
}
fuzz_target!(|data: [i16; 8]| {
    let [a, b, c, d, e, f, p, k] = data;
    let [a, b, c, d] = [a, b, c, d].map(|n| i64::from(n.rem_euclid(17) - 8));
    let (a, qa) = source(a.min(b), a.max(b), i32::from(e % 8), k.unsigned_abs());
    let (b, qb) = source(c.min(d), c.max(d), i32::from(f % 8), k.unsigned_abs() / 5);
    let p = 4 - i32::from(p.unsigned_abs() % 69);
    let budget = || DyadicBudget::new(4096, 100_000_000_000);
    let result = a.div(&b, p, &mut budget());
    oracle::verify(result.clone(), &qa, &qb, p);
    if let Ok(image) = result {
        let bytes = serde_json::to_vec(&image.range).unwrap();
        let restored: IntervalSet2<DyadicInterval> = serde_json::from_slice(&bytes).unwrap();
        assert_eq!(restored, image.range);
        assert_eq!(
            restored
                .neg(&mut budget())
                .unwrap()
                .neg(&mut budget())
                .unwrap(),
            restored
        );
        assert!(restored.subset_of(&restored, &mut budget()).unwrap());
        let hull = IntervalSet2::from_interval(restored.hull(&mut budget()).unwrap());
        assert!(restored.subset_of(&hull, &mut budget()).unwrap());
    }
});
