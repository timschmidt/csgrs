#![cfg(feature = "fixed")]
use hyperinterval::FixedInterval;
#[path = "../../hyperlattice-interval/conformance/allocations.rs"]
mod allocations;

#[test]
fn integer_divisors_and_failures_allocate_nothing() {
    let (_, calls, bytes) = allocations::measure(|| {
        macro_rules! case {
            ($i:ty,$endpoint:ty) => {{
                let all = <$i>::new(<$endpoint>::MIN, <$endpoint>::MAX).unwrap();
                let empty = <$i>::empty().unwrap();
                for divisor in [
                    i64::MIN,
                    i64::MIN + 1,
                    -3,
                    -2,
                    -1,
                    0,
                    1,
                    2,
                    3,
                    i64::MAX - 1,
                    i64::MAX,
                ] {
                    for source in [all, empty] {
                        let _ = std::hint::black_box(
                            std::hint::black_box(source).div_integer(std::hint::black_box(divisor)),
                        );
                    }
                }
            }};
        }
        case!(FixedInterval<i32,31>,i32);
        case!(FixedInterval<i64,63>,i64);
    });
    assert_eq!((calls, bytes), (0, 0));
}
