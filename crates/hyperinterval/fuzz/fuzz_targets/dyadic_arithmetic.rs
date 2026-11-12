#![no_main]
use hyperinterval::{DyadicBudget, DyadicInterval, IntervalError};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: [i16; 7]| {
    let [a, b, c, d, e, f, p] = data;
    let (a, b, c, d) = (
        i128::from(a.min(b)),
        i128::from(a.max(b)),
        i128::from(c.min(d)),
        i128::from(c.max(d)),
    );
    let (e, f, p) = (i32::from(e % 9), i32::from(f % 9), i32::from(p % 9));
    let x = DyadicInterval::new(a.into(), b.into(), e).unwrap();
    let y = DyadicInterval::new(c.into(), d.into(), f).unwrap();
    let mut budget = DyadicBudget::new(256, 10_000_000);
    let products = [a * c, a * d, b * c, b * d];
    assert_eq!(
        x.mul(&y, &mut budget).unwrap(),
        DyadicInterval::new(
            (*products.iter().min().unwrap()).into(),
            (*products.iter().max().unwrap()).into(),
            e + f
        )
        .unwrap()
    );
    let grid = e.min(f);
    assert_eq!(
        x.add(&y, &mut budget).unwrap(),
        DyadicInterval::new(
            ((a << (e - grid)) + (c << (f - grid))).into(),
            ((b << (e - grid)) + (d << (f - grid))).into(),
            grid
        )
        .unwrap()
    );
    if c <= 0 && d >= 0 {
        assert_eq!(x.div(&y, p, &mut budget), Err(IntervalError::ContainsZero));
    } else {
        let shift = e - f - p;
        let mut lower = i128::MAX;
        let mut upper = i128::MIN;
        for (n, den) in [(a, c), (a, d), (b, c), (b, d)] {
            let n = n << shift.max(0);
            let den = den << (-shift).max(0);
            let quotient = n / den;
            let remainder = n % den;
            lower = lower.min(quotient - i128::from(remainder != 0 && (n < 0) != (den < 0)));
            upper = upper.max(quotient + i128::from(remainder != 0 && (n < 0) == (den < 0)));
        }
        assert_eq!(
            x.div(&y, p, &mut budget).unwrap(),
            DyadicInterval::new(lower.into(), upper.into(), p).unwrap()
        );
    }
});
