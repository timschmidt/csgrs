#![no_main]

use hyperinterval::{FixedInterval, IntervalError};
use libfuzzer_sys::fuzz_target;

type I = FixedInterval<i32, 30>;
const SCALE: i128 = 1 << 30;

fn check(result: Result<I, IntervalError>, candidates: &[(i128, i128)]) {
    let mut outside = false;
    for &(mut numerator, mut denominator) in candidates {
        assert_ne!(denominator, 0);
        if denominator < 0 {
            numerator = -numerator;
            denominator = -denominator;
        }
        outside |= numerator < i128::from(i32::MIN) * denominator
            || numerator > i128::from(i32::MAX) * denominator;
        if let Ok(interval) = result {
            let (lower, upper) = interval.bounds().unwrap();
            assert!(i128::from(lower) * denominator <= numerator);
            assert!(numerator <= i128::from(upper) * denominator);
        }
    }
    if outside {
        assert_eq!(result, Err(IntervalError::Overflow));
    } else {
        assert!(result.is_ok());
    }
}

fuzz_target!(|values: [i32; 4]| {
    let [a, b, c, d] = values;
    let (a, b) = (a.min(b), a.max(b));
    let (c, d) = (c.min(d), c.max(d));
    let x = I::new(a, b).unwrap();
    let y = I::new(c, d).unwrap();
    let (a, b, c, d) = (i128::from(a), i128::from(b), i128::from(c), i128::from(d));
    assert_eq!(
        x.min_image(&y).bounds(),
        Some((a.min(c) as i32, b.min(d) as i32))
    );
    assert_eq!(
        x.max_image(&y).bounds(),
        Some((a.max(c) as i32, b.max(d) as i32))
    );
    check(x.abs(), &[(a.abs(), 1), (b.abs(), 1)]);
    if a <= 0
        && b >= 0
        && let Ok(image) = x.abs()
    {
        assert_eq!(image.lower(), Some(0));
    }
    check(
        x.checked_mul(&y),
        &[
            (a * c, SCALE),
            (a * d, SCALE),
            (b * c, SCALE),
            (b * d, SCALE),
        ],
    );
    check(
        x.checked_add(&y),
        &[(a + c, 1), (a + d, 1), (b + c, 1), (b + d, 1)],
    );
    check(
        x.checked_sub(&y),
        &[(a - c, 1), (a - d, 1), (b - c, 1), (b - d, 1)],
    );
    check(x.checked_neg(), &[(-b, 1), (-a, 1)]);
    if c <= 0 && 0 <= d {
        assert_eq!(x.checked_div(&y), Err(IntervalError::ContainsZero));
    } else {
        check(
            x.checked_div(&y),
            &[
                (a * SCALE, c),
                (a * SCALE, d),
                (b * SCALE, c),
                (b * SCALE, d),
            ],
        );
    }
    check(x.square(), &[(a * a, SCALE), (b * b, SCALE)]);
    if x.contains_point(0)
        && let Ok(square) = x.square()
    {
        assert_eq!(square.lower(), Some(0));
    }
});
