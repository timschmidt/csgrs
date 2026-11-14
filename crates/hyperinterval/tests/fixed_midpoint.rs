#![cfg(feature = "fixed")]
use hyperinterval::{FixedInterval, MidpointRounding};

macro_rules! check {
    ($endpoint:ty, $scale:expr) => {{
        type Interval = FixedInterval<$endpoint, $scale>;
        let values = [
            <$endpoint>::MIN,
            <$endpoint>::MIN + 1,
            -3,
            -2,
            -1,
            0,
            1,
            2,
            3,
            <$endpoint>::MAX - 1,
            <$endpoint>::MAX,
        ];
        let modes = [
            MidpointRounding::Floor,
            MidpointRounding::Ceiling,
            MidpointRounding::NearestTiesEven,
        ];
        let empty = Interval::empty().unwrap();
        assert_eq!(empty.width_ulps(), None);
        for mode in modes {
            assert_eq!(empty.midpoint(mode), None);
        }
        for lower in values {
            for upper in values {
                if lower > upper {
                    continue;
                }
                let value = Interval::new(lower, upper).unwrap();
                let sum = i128::from(lower) + i128::from(upper);
                let floor = sum.div_euclid(2);
                let remainder = sum.rem_euclid(2);
                assert_eq!(
                    u128::from(value.width_ulps().unwrap()),
                    (i128::from(upper) - i128::from(lower)) as u128
                );
                for mode in modes {
                    let expected = match mode {
                        MidpointRounding::Floor => floor,
                        MidpointRounding::Ceiling => floor + remainder,
                        MidpointRounding::NearestTiesEven => {
                            floor
                                + if floor.rem_euclid(2) == 1 {
                                    remainder
                                } else {
                                    0
                                }
                        }
                    };
                    let midpoint = value.midpoint(mode).unwrap();
                    assert_eq!(i128::from(midpoint), expected);
                    assert!(value.contains_point(midpoint));
                    let generated = Interval::point(midpoint).unwrap();
                    assert_eq!(generated.midpoint(mode), Some(midpoint));
                    assert_eq!(generated.width_ulps(), Some(0));
                }
            }
        }
    }};
}
macro_rules! scales {
    ($endpoint:ty; $($scale:expr),*) => { $(check!($endpoint, $scale);)* };
}
#[test]
fn all_supported_scales_preserve_exact_midpoints_and_widths() {
    scales!(i32; 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31);
    scales!(i64; 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63);
}

#[cfg(all(feature = "hyperreal", feature = "dispatch-trace"))]
#[test]
fn fixed_midpoints_and_widths_have_no_scalar_dispatch() {
    use hyperreal::dispatch_trace;
    dispatch_trace::reset();
    let result = dispatch_trace::with_recording(|| {
        let value = FixedInterval::<i64, 32>::new(i64::MIN, i64::MAX).unwrap();
        (
            value.midpoint(MidpointRounding::NearestTiesEven),
            value.width_ulps(),
        )
    });
    assert_eq!(result, (Some(0), Some(u64::MAX)));
    let trace = dispatch_trace::take_trace();
    assert!(trace.dispatch.is_empty());
    assert_eq!(trace.rational, Default::default());
}
