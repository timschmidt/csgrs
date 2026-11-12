#![no_main]
#[path = "../../../hyperreal/tests/support/mpfr.rs"]
mod mpfr_cache;
use hyperinterval::{F32Enclosure, F64Enclosure, IntervalError};
use libfuzzer_sys::fuzz_target;
use rug::Rational;

#[path = "../../tests/support/float_oracle.rs"]
mod oracle;

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

fuzz_target!(|data: [u64; 3]| {
    if !F64Enclosure::backend_supported() {
        assert_eq!(
            F64Enclosure::point(0.0),
            Err(IntervalError::UnqualifiedEnvironment)
        );
        return;
    }
    exercise!(data, F32Enclosure, f32, u32, from_f32, round_f32);
    exercise!(data, F64Enclosure, f64, u64, from_f64, round_f64);
});
