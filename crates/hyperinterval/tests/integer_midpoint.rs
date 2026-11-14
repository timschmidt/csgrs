#![cfg(feature = "integer")]
use hyperinterval::{IntegerRange, MidpointRounding};

#[test]
fn every_integer_family_preserves_midpoint_at_extreme_endpoints() {
    macro_rules! check {
        ($endpoint:ty) => {{
            let values: [$endpoint; 9] = [
                <$endpoint>::MIN,
                <$endpoint>::MIN + 1,
                <$endpoint>::MIN / 2,
                0,
                1,
                2,
                <$endpoint>::MAX / 2,
                <$endpoint>::MAX - 1,
                <$endpoint>::MAX,
            ];
            let modes = [
                MidpointRounding::Floor,
                MidpointRounding::Ceiling,
                MidpointRounding::NearestTiesEven,
            ];
            for mode in modes {
                assert_eq!(IntegerRange::<$endpoint>::empty().midpoint(mode), None);
            }
            for lower in values {
                for upper in values {
                    if lower > upper {
                        continue;
                    }
                    let range = IntegerRange::new(lower, upper).unwrap();
                    // Divide before adding: this independent quotient/remainder
                    // expectation also fits at the full i128/u128 extrema.
                    let halves = lower.div_euclid(2) + upper.div_euclid(2);
                    let residual = lower.rem_euclid(2) + upper.rem_euclid(2);
                    let floor = halves + residual / 2;
                    for mode in modes {
                        let expected = match mode {
                            MidpointRounding::Floor => floor,
                            MidpointRounding::Ceiling => floor + residual % 2,
                            MidpointRounding::NearestTiesEven => {
                                floor
                                    + if floor.rem_euclid(2) == 1 {
                                        residual % 2
                                    } else {
                                        0
                                    }
                            }
                        };
                        assert_eq!(range.midpoint(mode), Some(expected));
                        assert!(range.contains_point(expected));
                        assert_eq!(IntegerRange::point(expected).midpoint(mode), Some(expected));
                    }
                }
            }
        }};
    }
    check!(i8);
    check!(i16);
    check!(i32);
    check!(i64);
    check!(i128);
    check!(isize);
    check!(u8);
    check!(u16);
    check!(u32);
    check!(u64);
    check!(u128);
    check!(usize);
}
