//! Tight lattice hull of a nonempty image with known, attainable rational extrema.
use hyperinterval::{FixedEndpoint, FixedInterval, IntervalError};

pub fn check<I: FixedEndpoint + Into<i128>, const FRAC: u32>(
    result: Result<FixedInterval<I, FRAC>, IntervalError>,
    points: &[(i128, i128)],
) {
    let limit = 1_i128 << (I::BITS - 1);
    let bounds = result.map(|image| {
        let (lo, hi) = image.bounds().unwrap();
        (lo.into(), hi.into())
    });
    let mut outside = false;
    let mut needs_lower = false;
    let mut needs_upper = false;
    for &(n, d) in points {
        assert_ne!(d, 0);
        let (n, d) = if d < 0 { (-n, -d) } else { (n, d) };
        // The stored endpoints fit in i64 and d is at most 2^63, so even
        // the adjacent-lattice cross-products fit in i128.
        outside |= n < -limit * d || n > (limit - 1) * d;
        if let Ok((lo, hi)) = bounds {
            assert!(lo * d <= n && n <= hi * d);
            needs_lower |= n < (lo + 1) * d;
            needs_upper |= n > (hi - 1) * d;
        }
    }
    if outside {
        assert_eq!(bounds, Err(IntervalError::Overflow));
    } else {
        bounds.unwrap();
        assert!(needs_lower && needs_upper, "an endpoint can move inward");
    }
}
