#![no_main]
#[path = "../../../hyperreal/tests/support/mpfr.rs"]
mod mpfr_cache;
use hyperinterval::{DomainCoverage, F32Enclosure, F64Enclosure, FixedInterval, IntervalError};
use libfuzzer_sys::fuzz_target;
#[path = "../../tests/support/fixed_sqrt_oracle.rs"]
mod fixed_oracle;
#[path = "../../tests/support/primitive_sqrt_oracle.rs"]
mod oracle;

fuzz_target!(|data: [u64; 2]| {
    macro_rules! fixed {
        ($ty:ty, $($frac:literal),*) => {$({
            let a=data[0] as $ty;
            let b=data[1] as $ty;
            let (a,b)=(a.min(b),a.max(b));
            let result=FixedInterval::<$ty,$frac>::new(a,b).unwrap().sqrt()
                .map(|x| {let (a,b)=x.range.bounds().unwrap(); ((i64::from(a),i64::from(b)),x.domain)});
            fixed_oracle::fixed(i64::from(a),i64::from(b),$frac,i64::from(<$ty>::MAX),result);
        })*};
    }
    fixed!(i32, 0, 15, 30, 31);
    fixed!(i64, 0, 30, 62, 63);
    macro_rules! ieee {
        ($float:ty, $bits:ty, $interval:ident, $root:ident) => {{
            let a = <$float>::from_bits(data[0] as $bits);
            let b = <$float>::from_bits(data[1] as $bits);
            let (x, y) = ($interval::point(a), $interval::point(b));
            if !a.is_finite() {
                assert_eq!(x, Err(IntervalError::NonFiniteEndpoint));
            }
            if !b.is_finite() {
                assert_eq!(y, Err(IntervalError::NonFiniteEndpoint));
            }
            if let (Ok(x), Ok(y)) = (x, y) {
                let result = x.hull(&y).sqrt();
                if a.max(b) < 0.0 {
                    assert_eq!(result, Err(IntervalError::EmptyDomain));
                } else {
                    let result = result.unwrap();
                    let (lower, upper) = result.range.bounds().unwrap();
                    assert_eq!(
                        (lower.to_bits(), upper.to_bits()),
                        (
                            oracle::$root(a.min(b).max(0.0)).unwrap().0,
                            oracle::$root(a.max(b)).unwrap().1
                        )
                    );
                    assert_eq!(
                        result.domain,
                        if a.min(b) < 0.0 {
                            DomainCoverage::Partial
                        } else {
                            DomainCoverage::Complete
                        }
                    );
                    let restored: $interval =
                        serde_json::from_slice(&serde_json::to_vec(&result.range).unwrap())
                            .unwrap();
                    assert_eq!(restored, result.range);
                    // Finite squaring may legitimately overflow for a rounded
                    // upper root of MAX. A successful square contains the source.
                    if let Ok(square) = restored.square() {
                        assert!(
                            x.hull(&y)
                                .intersection(&$interval::new(0.0, <$float>::MAX).unwrap())
                                .subset_of(&square)
                        );
                    }
                }
            }
        }};
    }
    ieee!(f32, u32, F32Enclosure, sqrt_f32);
    ieee!(f64, u64, F64Enclosure, sqrt_f64);
});
