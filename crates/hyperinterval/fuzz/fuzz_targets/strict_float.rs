#![no_main]
#[path = "../../../hyperreal/tests/support/mpfr.rs"]
mod mpfr_cache;
use hyperinterval::{F32Enclosure, F64Enclosure, IntervalError};
use libfuzzer_sys::fuzz_target;
use rug::Rational;

#[path = "../../tests/support/float_oracle.rs"]
mod oracle;
#[path = "../../tests/support/float_range_oracle.rs"]
mod ranges;

macro_rules! exercise {
    ($data:ident, $interval:ident, $float:ty, $bits:ty, $from:ident, $round:ident) => {{
        let a = <$float>::from_bits($data[0] as $bits);
        let b = <$float>::from_bits($data[1] as $bits);
        let c = <$float>::from_bits($data[2] as $bits);
        let x = $interval::point(a);
        let y = $interval::point(b);
        if !a.is_finite() {
            assert_eq!(x, Err(IntervalError::NonFiniteEndpoint));
        }
        if !b.is_finite() {
            assert_eq!(y, Err(IntervalError::NonFiniteEndpoint));
        }
        if let (Ok(x), Ok(y)) = (x, y) {
            let aa = Rational::$from(a).unwrap();
            let bb = Rational::$from(b).unwrap();
            let mut budget = hyperinterval::DyadicBudget::new(8192, 1_000_000_000);
            let dx = x.to_dyadic(&mut budget).unwrap();
            let dy = y.to_dyadic(&mut budget).unwrap();
            for (exact, expected) in [
                (
                    dx.add(&dy, &mut budget).unwrap(),
                    oracle::$round(&Rational::from(&aa + &bb)),
                ),
                (
                    dx.mul(&dy, &mut budget).unwrap(),
                    oracle::$round(&Rational::from(&aa * &bb)),
                ),
            ] {
                let image = $interval::from_dyadic(&exact, &mut budget);
                assert_eq!(
                    image.map(|value| {
                        let (lo, hi) = value.bounds().unwrap();
                        (lo.to_bits(), hi.to_bits())
                    }),
                    expected
                );
            }
            let affine = if let Some(cc) = Rational::$from(c) {
                oracle::$round(&(Rational::from(&aa * &bb) + cc))
            } else {
                Err(IntervalError::NonFiniteEndpoint)
            };
            for (got, expected) in [
                (Ok(x.abs()), oracle::$round(&aa.clone().abs())),
                (Ok(x.min_image(&y)), oracle::$round((&aa).min(&bb))),
                (Ok(x.max_image(&y)), oracle::$round((&aa).max(&bb))),
                (x.affine(b, c), affine),
                (x.add(&y), oracle::$round(&Rational::from(&aa + &bb))),
                (x.sub(&y), oracle::$round(&Rational::from(&aa - &bb))),
                (x.mul(&y), oracle::$round(&Rational::from(&aa * &bb))),
                (
                    x.div(&y),
                    if bb == 0 {
                        Err(IntervalError::ContainsZero)
                    } else {
                        oracle::$round(&Rational::from(&aa / &bb))
                    },
                ),
                (
                    x.hull(&y).midpoint_enclosure(),
                    oracle::$round(&((aa.clone() + &bb) / 2)),
                ),
                (
                    x.hull(&y).radius_enclosure(),
                    oracle::$round(&((aa.clone() - &bb).abs() / 2)),
                ),
            ] {
                let bits = got.map(|value| {
                    let (lo, hi) = value.bounds().unwrap();
                    (lo.to_bits(), hi.to_bits())
                });
                assert_eq!(bits, expected);
                if let Ok(value) = got {
                    let bytes = serde_json::to_vec(&value).unwrap();
                    assert_eq!(serde_json::from_slice::<$interval>(&bytes).unwrap(), value);
                    assert!(value.subset_of(&value.hull(&x)));
                }
            }
        }
        let bytes = serde_json::to_vec(&($data[0] as $bits, $data[1] as $bits)).unwrap();
        assert_eq!(
            serde_json::from_slice::<$interval>(&bytes).ok(),
            $interval::new(a, b).ok()
        );
    }};
}

fuzz_target!(|bytes: &[u8]| {
    if !F64Enclosure::backend_supported() {
        assert_eq!(
            F64Enclosure::point(0.0),
            Err(IntervalError::UnqualifiedEnvironment)
        );
        return;
    }
    // The original three little-endian words retain their point/affine semantics.
    // Zero-fill short inputs so mutations can reach the arithmetic from an empty corpus.
    let mut words = [[0; 8]; 4];
    let n = bytes.len().min(32);
    words.as_flattened_mut()[..n].copy_from_slice(&bytes[..n]);
    let data = words.map(u64::from_le_bytes);
    exercise!(data, F32Enclosure, f32, u32, from_f32, round_f32);
    exercise!(data, F64Enclosure, f64, u64, from_f64, round_f64);
    ranges::check_f32(data.map(|word| f32::from_bits(word as u32)));
    ranges::check_f64(data.map(f64::from_bits));
});
