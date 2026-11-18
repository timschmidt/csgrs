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
    let point = |n: i128, exponent| DyadicInterval::point(n.into(), exponent).unwrap();
    assert_eq!(x.width(&mut budget).unwrap(), point(b - a, e));
    let midpoint = x.midpoint(&mut budget).unwrap();
    let radius = x.radius(&mut budget).unwrap();
    assert_eq!(midpoint, point(a + b, e - 1));
    assert_eq!(radius, point(b - a, e - 1));
    let halves = (a < b).then(|| {
        [
            DyadicInterval::new((2 * a).into(), (a + b).into(), e - 1).unwrap(),
            DyadicInterval::new((a + b).into(), (2 * b).into(), e - 1).unwrap(),
        ]
    });
    assert_eq!(x.bisect(&mut budget).unwrap(), halves);
    let lower = midpoint.sub(&radius, &mut budget).unwrap();
    let upper = midpoint.add(&radius, &mut budget).unwrap();
    assert_eq!(lower.hull(&upper, &mut budget).unwrap(), x);
    let low_square = if a <= 0 && b >= 0 {
        0
    } else {
        (a * a).min(b * b)
    };
    assert_eq!(
        x.square(&mut budget).unwrap(),
        DyadicInterval::new(low_square.into(), (a * a).max(b * b).into(), 2 * e).unwrap()
    );
    // i16 endpoint powers through seven fit i128, independent of BigInt.
    let power = p.unsigned_abs() % 8;
    let aa = a.pow(power);
    let bb = b.pow(power);
    let low_power = if power != 0 && power.is_multiple_of(2) && a <= 0 && b >= 0 {
        0
    } else {
        aa.min(bb)
    };
    assert_eq!(
        x.checked_pow(power, &mut budget).unwrap(),
        DyadicInterval::new(low_power.into(), aa.max(bb).into(), e * power as i32).unwrap()
    );
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
    let (aa, bb, cc, dd) = (
        a << (e - grid),
        b << (e - grid),
        c << (f - grid),
        d << (f - grid),
    );
    assert_eq!(
        x.min_image(&y, &mut budget).unwrap(),
        DyadicInterval::new(aa.min(cc).into(), bb.min(dd).into(), grid).unwrap()
    );
    assert_eq!(
        x.max_image(&y, &mut budget).unwrap(),
        DyadicInterval::new(aa.max(cc).into(), bb.max(dd).into(), grid).unwrap()
    );
    let min_abs = if a <= 0 && b >= 0 {
        0
    } else {
        a.abs().min(b.abs())
    };
    assert_eq!(
        x.abs(&mut budget).unwrap(),
        DyadicInterval::new(min_abs.into(), a.abs().max(b.abs()).into(), e).unwrap()
    );
    let point_on_grid = c << (f - grid);
    assert_eq!(
        x.contains_point(&c.into(), f, &mut budget).unwrap(),
        (a << (e - grid)) <= point_on_grid && point_on_grid <= (b << (e - grid))
    );
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
