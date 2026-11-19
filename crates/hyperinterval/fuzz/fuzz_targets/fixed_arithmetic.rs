#![no_main]
use hyperinterval::{FixedInterval, IntervalError};
use libfuzzer_sys::{Corpus, fuzz_target};

#[path = "../../tests/support/fixed_hull_oracle.rs"]
mod oracle;
use oracle::check;

macro_rules! fixed_cases {
    ($name:ident, $endpoint:ty, $frac:literal) => {
        fn $name([a, b, c, d]: [$endpoint; 4]) {
            const SCALE: i128 = 1 << $frac;
            type I = FixedInterval<$endpoint, $frac>;
            let (a, b) = (a.min(b), a.max(b));
            let (c, d) = (c.min(d), c.max(d));
            let x = I::new(a, b).unwrap();
            let y = I::new(c, d).unwrap();
            assert_eq!(x.min_image(&y).bounds(), Some((a.min(c), b.min(d))));
            assert_eq!(x.max_image(&y).bounds(), Some((a.max(c), b.max(d))));

            let [a, b, c, d] = [a, b, c, d].map(i128::from);
            let nearest_zero = 0_i128.clamp(a, b);
            let corners = [(a, c), (a, d), (b, c), (b, d)];
            check(
                x.abs(),
                &[(a.abs(), 1), (b.abs(), 1), (nearest_zero.abs(), 1)],
            );
            check(x.checked_neg(), &[(-b, 1), (-a, 1)]);
            check(
                x.square(),
                &[
                    (a * a, SCALE),
                    (b * b, SCALE),
                    (nearest_zero * nearest_zero, SCALE),
                ],
            );
            check(x.checked_add(&y), &corners.map(|(p, q)| (p + q, 1)));
            check(x.checked_sub(&y), &corners.map(|(p, q)| (p - q, 1)));
            check(x.checked_mul(&y), &corners.map(|(p, q)| (p * q, SCALE)));
            if c <= 0 && d >= 0 {
                assert_eq!(x.checked_div(&y), Err(IntervalError::ContainsZero));
            } else {
                check(x.checked_div(&y), &corners.map(|(p, q)| (p * SCALE, q)));
            }
        }
    };
}

fixed_cases!(q2_30, i32, 30);
fixed_cases!(q31_32, i64, 32);

fuzz_target!(|bytes: &[u8]| -> Corpus {
    // Keep the original four little-endian i32 endpoints as a corpus prefix.
    let Some(words) = bytes.as_chunks::<4>().0.first_chunk::<4>() else {
        return Corpus::Reject;
    };
    q2_30(words.map(i32::from_le_bytes));
    if let Some(words) = bytes.as_chunks::<8>().0.first_chunk::<4>() {
        q31_32(words.map(i64::from_le_bytes));
    }
    Corpus::Keep
});
