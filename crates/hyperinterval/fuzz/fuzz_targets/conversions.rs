#![no_main]
use hyperinterval::{
    DyadicBudget, DyadicInterval, FixedEndpoint, FixedInterval, IntervalError, RationalInterval,
};
use hyperreal::Rational;
use libfuzzer_sys::fuzz_target;
use rug::Rational as Q;

#[path = "../../tests/support/grid_oracle.rs"]
mod grid;
#[path = "../../tests/support/rational_oracle.rs"]
mod rational_oracle;

const WORK: u64 = 1_000_000_000;

fn budget() -> DyadicBudget {
    DyadicBudget::new(4096, WORK)
}

fn exact_bounds(value: &DyadicInterval) -> Option<(Q, Q)> {
    value
        .bounds()
        .map(|(lo, hi, e)| (grid::q(lo, e), grid::q(hi, e)))
}

fn fixed_bounds<I: FixedEndpoint + Into<i64>, const F: u32>(a: I, b: I) {
    let (lo, hi) = (a.min(b), a.max(b));
    let source = FixedInterval::<I, F>::new(lo, hi).unwrap();
    let expected = (
        Q::from((lo.into(), 1_u64 << F)),
        Q::from((hi.into(), 1_u64 << F)),
    );
    assert_eq!(
        exact_bounds(&source.to_dyadic(&mut budget()).unwrap()).unwrap(),
        expected
    );
    let image = source.to_rational();
    let (lo, hi) = image.bounds().unwrap();
    assert_eq!(
        (rational_oracle::mpq(lo), rational_oracle::mpq(hi)),
        expected
    );
}

fuzz_target!(|data: [u64; 8]| {
    let [a, b, c, d, p, q, ieee, mode] = data;
    match mode % 3 {
        0 => fixed_bounds::<i32, 30>(a as i32, c as i32),
        1 => fixed_bounds::<i64, 32>(a as i64, c as i64),
        _ => fixed_bounds::<i64, 63>(a as i64, c as i64),
    }
    let a = Rational::fraction(a as i64, b.max(1)).unwrap();
    let b = Rational::fraction(c as i64, d.max(1)).unwrap();
    let (a, b) = if a <= b { (a, b) } else { (b, a) };
    let source = if mode & 1 != 0 {
        RationalInterval::empty()
    } else if mode & 2 != 0 {
        RationalInterval::point(a)
    } else {
        RationalInterval::new(a, b).unwrap()
    };
    let p = (p % 257) as i32 - 128;
    let q = (q % 257) as i32 - 128;
    let expected = source
        .bounds()
        .map(|(lo, hi)| grid::bounds(rational_oracle::mpq(lo), rational_oracle::mpq(hi), p));
    let mut measured = budget();
    let converted = source.to_dyadic(p, &mut measured).unwrap();
    assert_eq!(exact_bounds(&converted), expected);
    let work = WORK - measured.remaining_work();
    assert_eq!(
        source
            .to_dyadic(p, &mut DyadicBudget::new(4096, work))
            .unwrap(),
        converted
    );
    if work != 0 {
        assert_eq!(
            source.to_dyadic(p, &mut DyadicBudget::new(4096, work - 1)),
            Err(IntervalError::WorkExhausted)
        );
    }
    assert_eq!(source.to_dyadic(p, &mut budget()).unwrap(), converted);

    let encoded = serde_json::to_vec(&converted).unwrap();
    let restored: DyadicInterval = serde_json::from_slice(&encoded).unwrap();
    assert_eq!(restored, converted);
    let rational = restored.to_rational(&mut budget()).unwrap();
    assert!(source.subset_of(&rational));
    assert_eq!(
        rational
            .bounds()
            .map(|(lo, hi)| (rational_oracle::mpq(lo), rational_oracle::mpq(hi))),
        expected
    );
    let regridded = restored.regrid(q, &mut budget()).unwrap();
    assert_eq!(
        exact_bounds(&regridded),
        expected.clone().map(|(lo, hi)| grid::bounds(lo, hi, q))
    );
    assert_eq!(rational.to_dyadic(p, &mut budget()).unwrap(), restored);

    if let Some((lo, hi)) = source.bounds() {
        let admitted = DyadicInterval::from_grid_bounds(lo, hi, p, &mut budget());
        if expected == Some((rational_oracle::mpq(lo), rational_oracle::mpq(hi))) {
            assert_eq!(admitted.unwrap(), converted);
        } else {
            assert_eq!(admitted, Err(IntervalError::NonDyadicEndpoint));
        }
        let point = DyadicInterval::from_rational(lo, q, &mut budget()).unwrap();
        assert_eq!(
            exact_bounds(&point),
            Some(grid::bounds(
                rational_oracle::mpq(lo),
                rational_oracle::mpq(lo),
                q
            ))
        );
        if lo < hi {
            assert_eq!(
                DyadicInterval::from_grid_bounds(hi, lo, p, &mut DyadicBudget::new(0, 0)),
                Err(IntervalError::ReversedEndpoints)
            );
        }
    }
    for precision in [i32::MIN, i32::MAX] {
        let extreme = source.to_dyadic(precision, &mut budget());
        if source.is_empty() {
            assert_eq!(extreme.unwrap(), DyadicInterval::empty());
        } else {
            assert_eq!(extreme, Err(IntervalError::SizeExhausted));
        }
    }

    let value = f64::from_bits(ieee);
    if let Some(exact) = Q::from_f64(value) {
        let point = DyadicInterval::from_f64(value, &mut budget()).unwrap();
        assert_eq!(exact_bounds(&point), Some((exact.clone(), exact)));
        let exact = point.to_rational(&mut budget()).unwrap();
        assert_eq!(
            DyadicInterval::from_rational(
                exact.lower().unwrap(),
                point.bounds().unwrap().2,
                &mut budget()
            )
            .unwrap(),
            point
        );
    } else {
        let mut denied = DyadicBudget::new(0, 0);
        assert_eq!(
            DyadicInterval::from_f64(value, &mut denied),
            Err(IntervalError::NonFiniteEndpoint)
        );
        assert_eq!(denied.remaining_work(), 0);
    }
});
