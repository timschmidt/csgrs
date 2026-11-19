#![cfg(feature = "fixed")]
use hyperinterval::{FixedInterval, IntervalError};

// Independent exact cross-product witness for containment and tightness. It
// performs no integer division and does not call the implementation's helper.
fn tight(bounds: (i128, i128), a: i128, b: i128, divisor: i64) {
    let (lower, upper, denominator) = if divisor < 0 {
        (-b, -a, -i128::from(divisor))
    } else {
        (a, b, i128::from(divisor))
    };
    let (lo, hi) = bounds;
    assert!(lo * denominator <= lower && (lo + 1) * denominator > lower);
    assert!(hi * denominator >= upper && (hi - 1) * denominator < upper);
}

macro_rules! qualify {
    ($name:ident, $endpoint:ty, $($frac:literal),+) => {
        #[test]
        fn $name() {
            $(
                check_scale::<$frac>();
            )+
            fn check_scale<const F: u32>() {
                for a in -32..=32 {
                    for b in a..=32 {
                        let x = FixedInterval::<$endpoint, F>::new(a, b).unwrap();
                        for divisor in -32..=32 {
                            if divisor == 0 {
                                assert_eq!(x.div_integer(0), Err(IntervalError::ContainsZero));
                            } else {
                                let (lo, hi) = x.div_integer(divisor).unwrap().bounds().unwrap();
                                tight((lo.into(), hi.into()), a.into(), b.into(), divisor);
                            }
                        }
                    }
                }
                let endpoints = [<$endpoint>::MIN, <$endpoint>::MIN + 1, -1, 0, 1, <$endpoint>::MAX - 1, <$endpoint>::MAX];
                let divisors = [i64::MIN, i64::MIN + 1, -(1_i64 << 32), -3, -2, -1, 1, 2, 3, 1_i64 << 32, i64::MAX - 1, i64::MAX];
                for (index, &a) in endpoints.iter().enumerate() {
                    for &b in &endpoints[index..] {
                        let x = FixedInterval::<$endpoint, F>::new(a, b).unwrap();
                        for divisor in divisors {
                            let result = x.div_integer(divisor);
                            if a == <$endpoint>::MIN && divisor == -1 {
                                assert_eq!(result, Err(IntervalError::Overflow));
                            } else {
                                let (lo, hi) = result.unwrap().bounds().unwrap();
                                tight((lo.into(), hi.into()), a.into(), b.into(), divisor);
                                // The exact result remains a valid subsequent operand.
                                assert_eq!(result.unwrap().div_integer(1).unwrap(), result.unwrap());
                            }
                        }
                    }
                }
                let empty = FixedInterval::<$endpoint, F>::empty().unwrap();
                assert_eq!(empty.div_integer(0), Err(IntervalError::ContainsZero));
                for divisor in divisors { assert_eq!(empty.div_integer(divisor).unwrap(), empty); }
            }
        }
    }
}

qualify!(i32_all_selected_scales, i32, 0, 1, 4, 30, 31);
qualify!(i64_all_selected_scales, i64, 0, 1, 4, 30, 32, 62, 63);

#[test]
fn point_denominator_equivalence_and_nonrepresentable_integer_divisors() {
    // Exact point lifting is admitted only when it fits. The new operation
    // agrees there; it also admits integers outside the point representation.
    macro_rules! case {
        ($ty:ty, $frac:literal) => {{
            type I = FixedInterval<$ty, $frac>;
            let points: [$ty; 7] = [
                <$ty>::MIN,
                <$ty>::MIN + 1,
                -1,
                0,
                1,
                <$ty>::MAX - 1,
                <$ty>::MAX,
            ];
            for (i, &a) in points.iter().enumerate() {
                for &b in &points[i..] {
                    let x = I::new(a, b).unwrap();
                    for divisor in [i64::MIN, -3, -2, -1, 0, 1, 2, 3, i64::MAX] {
                        if let Ok(point) = I::from_ratio(divisor, 1) {
                            assert_eq!(x.div_integer(divisor), x.checked_div(&point));
                        }
                    }
                }
            }
        }};
    }
    case!(i32, 0);
    case!(i32, 30);
    case!(i32, 31);
    case!(i64, 0);
    case!(i64, 32);
    case!(i64, 63);
    type I = FixedInterval<i64, 63>;
    assert_eq!(I::from_ratio(3, 1), Err(IntervalError::Overflow));
    let x = I::new(i64::MIN, i64::MAX).unwrap();
    assert_eq!(x.div_integer(i64::MIN).unwrap().bounds(), Some((-1, 1)));
    let zero = I::point(0).unwrap();
    assert_eq!(zero.div_integer(i64::MIN).unwrap(), zero);
}

#[cfg(feature = "serde")]
#[test]
fn replayed_output_feeds_widening_and_further_division() {
    type I = FixedInterval<i32, 31>;
    const KEY: &str = "HYPERINTERVAL_INTEGER_DIV_REPLAY";
    if let Ok(text) = std::env::var(KEY) {
        let replay: I = serde_json::from_str(&text).unwrap();
        assert_eq!(replay.bounds(), Some((-715_827_883, 715_827_883)));
        assert_eq!(
            replay
                .convert::<i64>()
                .unwrap()
                .div_integer(-2)
                .unwrap()
                .bounds(),
            Some((-357_913_942, 357_913_942))
        );
        assert_eq!(serde_json::to_string(&replay).unwrap(), text);
        return;
    }
    let source = I::new(i32::MIN, i32::MAX).unwrap();
    let output = source.div_integer(3).unwrap();
    assert_eq!(output.bounds(), Some((-715_827_883, 715_827_883)));
    let text = serde_json::to_string(&output).unwrap();
    let replay: I = serde_json::from_str(&text).unwrap();
    assert_eq!(output, replay);
    assert_eq!(
        replay
            .convert::<i64>()
            .unwrap()
            .div_integer(-2)
            .unwrap()
            .bounds(),
        Some((-357_913_942, 357_913_942))
    );
    assert_eq!(serde_json::to_string(&replay).unwrap(), text);
    let child = std::process::Command::new(replay_binary::path().unwrap())
        .args([
            "--exact",
            "replayed_output_feeds_widening_and_further_division",
            "--nocapture",
        ])
        .env(KEY, text)
        .output()
        .unwrap();
    assert!(
        child.status.success(),
        "{} {}",
        String::from_utf8_lossy(&child.stdout),
        String::from_utf8_lossy(&child.stderr)
    );
}

#[test]
fn seeded_full_width_divisors_satisfy_exact_tightness() {
    let mut state = 20_261_113_u64;
    let mut next = || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        state as i64
    };
    for _ in 0..20_000 {
        let a = next();
        let b = next();
        let divisor = next();
        let (a, b) = (a.min(b), a.max(b));
        let x = FixedInterval::<i64, 63>::new(a, b).unwrap();
        if divisor == 0 {
            assert_eq!(x.div_integer(divisor), Err(IntervalError::ContainsZero));
        } else {
            let (lo, hi) = x.div_integer(divisor).unwrap().bounds().unwrap();
            tight((lo.into(), hi.into()), a.into(), b.into(), divisor);
        }
    }
}

#[cfg(feature = "serde")]
#[path = "support/replay_binary.rs"]
mod replay_binary;
