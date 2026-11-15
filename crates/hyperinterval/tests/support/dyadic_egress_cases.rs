//! Exact bit expectations shared by native tests and the actual WASM consumer.
use hyperinterval::{DyadicBudget, DyadicInterval, F32Enclosure, F64Enclosure, IntervalError};
use num_bigint::BigInt;

pub fn verify() -> u32 {
    let mut count = 0;
    macro_rules! cases {
        ($carrier:ident,$fraction:literal,$bias:literal,$minimum:literal,$sign:expr) => {{
            let one = ($bias as u64) << $fraction;
            let max = ((2 * $bias as u64) << $fraction) | ((1u64 << $fraction) - 1);
            let min_normal = 1u64 << $fraction;
            let examples = [
                (BigInt::from(0), 0, 0, 0),
                (BigInt::from(1), 0, one, one),
                (BigInt::from(1), $minimum, 1, 1),
                (BigInt::from(1), $minimum - 1, 0, 1),
                (BigInt::from(3), $minimum - 1, 1, 2),
                (BigInt::from(1), i32::MIN, 0, 1),
                (
                    (BigInt::from(1) << ($fraction + 1)) - 1,
                    $minimum - 1,
                    min_normal - 1,
                    min_normal,
                ),
                (
                    (BigInt::from(1) << ($fraction + 1)) - 1,
                    $bias - $fraction,
                    max,
                    max,
                ),
                ((BigInt::from(1) << 4096) + 1, -4096, one, one + 1),
                ((BigInt::from(1) << 4096) - 1, -4096, one - 1, one),
                ((BigInt::from(1) << 4096) + 1, $minimum - 4096, 1, 2),
            ];
            for (mantissa, e, lo, hi) in examples {
                for negative in [false, true] {
                    let m = if negative {
                        -&mantissa
                    } else {
                        mantissa.clone()
                    };
                    let source = DyadicInterval::point(m, e).unwrap();
                    let result =
                        $carrier::from_dyadic(&source, &mut DyadicBudget::new(8192, 100_000))
                            .unwrap();
                    let (a, b) = result.bounds().unwrap();
                    let neg = |bits: u64| if bits == 0 { 0 } else { bits | $sign };
                    let want = if negative {
                        (neg(hi), neg(lo))
                    } else {
                        (lo, hi)
                    };
                    assert_eq!((a.to_bits() as u64, b.to_bits() as u64), want);
                    count += 1;
                }
            }
            for (mantissa, e) in [
                (BigInt::from(1), i32::MAX),
                (BigInt::from(1), $bias + 1),
                (
                    (BigInt::from(1) << ($fraction + 2)) - 1,
                    $bias - $fraction - 1,
                ),
            ] {
                for m in [mantissa.clone(), -mantissa] {
                    let source = DyadicInterval::point(m, e).unwrap();
                    assert_eq!(
                        $carrier::from_dyadic(&source, &mut DyadicBudget::new(8192, 100_000)),
                        Err(IntervalError::Overflow)
                    );
                    count += 1;
                }
            }
        }};
    }
    cases!(F32Enclosure, 23, 127, -149, 1u64 << 31);
    cases!(F64Enclosure, 52, 1023, -1074, 1u64 << 63);
    count
}
