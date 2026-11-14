#![cfg(any(feature = "integer", feature = "fixed"))]
use hyperinterval::{IntervalError, integer_quotient_bounds};

#[cfg(all(
    feature = "integer",
    feature = "fixed",
    feature = "hyperreal",
    feature = "dispatch-trace"
))]
#[test]
fn native_clock_arithmetic_has_no_real_dispatch_or_reductions() {
    use hyperinterval::{FixedInterval, IntegerRange, MidpointRounding};
    use hyperreal::dispatch_trace;
    dispatch_trace::reset();
    let (image, midpoint, radius, fixed, quotient) = dispatch_trace::with_recording(|| {
        let retained = IntegerRange::new(-7_i128, 9).unwrap();
        let sample = IntegerRange::new(-3_i128, 12).unwrap();
        let image = retained.intersection(&sample).scale(-2).unwrap();
        let midpoint = image.midpoint(MidpointRounding::Floor);
        let radius = image.radius();
        let fixed = FixedInterval::<i64, 32>::from_ratio(-7, 3).unwrap();
        (
            image,
            midpoint,
            radius,
            fixed,
            integer_quotient_bounds(-7, 3),
        )
    });
    assert_eq!(image.bounds(), Some((-18, 6)));
    assert_eq!(midpoint, Some(-6));
    assert_eq!(radius, Some(12));
    assert_eq!(fixed.bounds(), Some((-10_021_590_358, -10_021_590_357)));
    assert_eq!(quotient, Ok((-3, -2)));
    let trace = dispatch_trace::take_trace();
    assert!(trace.dispatch.is_empty());
    assert_eq!(trace.rational, Default::default());
}

#[test]
fn exhaustive_byte_ratios_have_tight_bounds() {
    for n in -128_i128..=127 {
        for d in -128_i128..=127 {
            if d == 0 {
                assert_eq!(
                    integer_quotient_bounds(n, d),
                    Err(IntervalError::ContainsZero)
                );
                continue;
            }
            let (lo, hi) = integer_quotient_bounds(n, d).unwrap();
            // Independent cross-multiplication, including the adjacent lattice
            // points, establishes both enclosure and tightness without division.
            let (n, d) = if d < 0 { (-n, -d) } else { (n, d) };
            assert!(lo * d <= n && n < (lo + 1) * d);
            assert!((hi - 1) * d < n && n <= hi * d);
            assert!(hi == lo || hi == lo + 1);
        }
    }
}

#[test]
fn signed_extremes_and_error_precedence() {
    for (n, d, expected) in [
        (i128::MIN, 1, (i128::MIN, i128::MIN)),
        (i128::MAX, 1, (i128::MAX, i128::MAX)),
        (i128::MIN, i128::MIN, (1, 1)),
        (i128::MAX, i128::MIN, (-1, 0)),
        (1, i128::MIN, (-1, 0)),
        (-1, i128::MIN, (0, 1)),
        (i128::MIN + 1, -1, (i128::MAX, i128::MAX)),
        (i128::MIN, i128::MAX, (-2, -1)),
        (i128::MAX, -2, (-(i128::MAX / 2) - 1, -(i128::MAX / 2))),
    ] {
        assert_eq!(integer_quotient_bounds(n, d), Ok(expected));
    }
    assert_eq!(
        integer_quotient_bounds(i128::MIN, -1),
        Err(IntervalError::Overflow)
    );
    for n in [i128::MIN, 0, i128::MAX] {
        assert_eq!(
            integer_quotient_bounds(n, 0),
            Err(IntervalError::ContainsZero)
        );
    }
}

#[cfg(not(target_env = "msvc"))]
#[test]
fn full_width_gmp_inequalities_and_adjacent_lattice_points() {
    use rug::{Integer, Rational};
    fn check(n: i128, d: i128) {
        if d == 0 || (n == i128::MIN && d == -1) {
            return;
        }
        let exact = Rational::from((n, d));
        let (lo, hi) = integer_quotient_bounds(n, d).unwrap();
        assert!(lo <= exact && exact <= hi);
        assert!(exact < Integer::from(lo) + 1);
        assert!(Integer::from(hi) - 1 < exact);
    }
    let extremes = [
        i128::MIN,
        i128::MIN + 1,
        -(1_i128 << 126) - 1,
        -(1_i128 << 64),
        -3,
        -2,
        -1,
        0,
        1,
        2,
        3,
        (1_i128 << 64) + 1,
        (1_i128 << 126) + 1,
        i128::MAX - 1,
        i128::MAX,
    ];
    for n in extremes {
        for d in extremes {
            check(n, d);
        }
    }
    let mut state = 0x9e37_79b9_7f4a_7c15_243f_6a88_85a3_08d3_u128;
    for _ in 0..4096 {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        let n = state as i128;
        state = state.wrapping_mul(0xda94_2042_e4dd_58b5_9e37_79b9_7f4a_7c15);
        check(n, state as i128);
    }
}
