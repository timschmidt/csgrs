//! Compare the two mathematical extrema against arbitrary-precision GMP.
use hyperinterval::{IntegerRange, IntervalError};
use rug::Integer;

macro_rules! check {
    ($name:ident, $ty:ty, $narrow:ident) => {
        pub fn $name(a: $ty, b: $ty, factor: $ty, offset: $ty) {
            let source = IntegerRange::new(a.min(b), a.max(b)).unwrap();
            let image = |x| Integer::from(x) * Integer::from(factor) + Integer::from(offset);
            let x = image(a);
            let y = image(b);
            let (lo, hi) = if x <= y { (x, y) } else { (y, x) };
            let expected = match (lo.$narrow(), hi.$narrow()) {
                (Some(lo), Some(hi)) => Ok(Some((lo, hi))),
                _ => Err(IntervalError::Overflow),
            };
            assert_eq!(source.affine(factor, offset).map(|r| r.bounds()), expected);
        }
    };
}

check!(signed, i128, to_i128);
check!(unsigned, u128, to_u128);
