#![no_main]
#[path = "../../../hyperreal/tests/support/mpfr.rs"]
mod mpfr_cache;
use hyperinterval::{DyadicBudget, DyadicInterval, ExtendedInterval as E, IntervalSet2};
use libfuzzer_sys::fuzz_target;
#[path = "../../tests/support/power_oracle.rs"]
mod oracle;
#[cfg(feature = "rational")]
#[path = "../../tests/support/rational_coordinate.rs"]
mod rational_coordinate;
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
fuzz_target!(|data: [i16; 6]| {
    let [a, b, e, n, q, p] = data;
    let (a, b) = (
        i64::from(a.rem_euclid(17) - 8),
        i64::from(b.rem_euclid(17) - 8),
    );
    let (source, exact) = source(a.min(b), a.max(b), i32::from(e % 5), e.unsigned_abs());
    let (n, q) = (i32::from(n % 6), i32::from(q % 7));
    let p = 4 - i32::from(p.unsigned_abs() % 37);
    let budget = || DyadicBudget::new(8192, 1_000_000_000_000);
    #[cfg(feature = "rational")]
    {
        let shift = i32::from(e % 5);
        let scale = 2_f64.powi(shift);
        let (lo, hi) = (a.min(b), a.max(b));
        let base = hyperinterval::RationalInterval::new(
            rational_coordinate::coordinate(6., lo as f64 * scale),
            rational_coordinate::coordinate(10., hi as f64 * scale),
        )
        .unwrap();
        let exact = Some((
            oracle::End::Value(oracle::q(&lo, shift)),
            oracle::End::Value(oracle::q(&hi, shift)),
        ));
        oracle::verify(
            base.pow_rational(&n.into(), &q.into(), p, &mut budget()),
            &exact,
            n,
            q,
            p,
        );
    }
    let result = source.pow_rational(&n.into(), &q.into(), p, &mut budget());
    oracle::verify(result.clone(), &exact, n, q, p);
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
        if let Some((oracle::End::Value(a), oracle::End::Value(b))) = exact
            && a == b
            && a != 0
            && q > 0
            && n != 0
        {
            // Compare an independent directed MPFR point on admitted raw branches.
            if a > 0 || q % 2 == 1 {
                oracle::mpfr_point(&restored, &a, n, q as u32);
            }
        }
    }
});
