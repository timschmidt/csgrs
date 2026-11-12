#![no_main]
use hyperinterval::{F32ProposalInterval, F64ProposalInterval, IntervalError};
use libfuzzer_sys::fuzz_target;

macro_rules! exercise {
    ($data:ident, $interval:ident, $float:ty, $bits:ty) => {{
        let a = <$float>::from_bits($data[0] as $bits);
        let b = <$float>::from_bits($data[1] as $bits);
        let c = <$float>::from_bits($data[2] as $bits);
        let d = <$float>::from_bits($data[3] as $bits);
        let x = $interval::new(a, b);
        let y = $interval::new(c, d);
        if !a.is_finite() || !b.is_finite() {
            assert_eq!(x, Err(IntervalError::NonFiniteEndpoint));
        } else if a > b {
            assert_eq!(x, Err(IntervalError::ReversedEndpoints));
        }
        if let (Ok(x), Ok(y)) = (x, y) {
            for result in [x.add(&y), x.sub(&y), x.mul(&y), x.div(&y), x.affine(c, d)] {
                if let Ok(value) = result {
                    let (lo, hi) = value.estimates().unwrap();
                    assert!(lo.is_finite() && hi.is_finite() && lo <= hi);
                    let bytes = serde_json::to_vec(&value).unwrap();
                    assert_eq!(serde_json::from_slice::<$interval>(&bytes).unwrap(), value);
                } else {
                    assert!(matches!(
                        result,
                        Err(IntervalError::Overflow | IntervalError::ContainsZero)
                    ));
                }
            }
        }
        // Bit-pattern replay must enforce exactly the same ingress contract.
        let bytes = serde_json::to_vec(&($data[0] as $bits, $data[1] as $bits)).unwrap();
        assert_eq!(serde_json::from_slice::<$interval>(&bytes).ok(), x.ok());
    }};
}

fuzz_target!(|data: [u64; 4]| {
    exercise!(data, F32ProposalInterval, f32, u32);
    exercise!(data, F64ProposalInterval, f64, u64);
});
