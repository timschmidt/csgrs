use alumina_foc::{FocError, Q30, Q30Interval};
use hyperinterval::{FixedInterval, IntervalError, MidpointRounding};

type Shared = FixedInterval<i32, 30>;

fn compare(shared: Result<Shared, IntervalError>, legacy: Result<Q30Interval, FocError>) {
    match (shared, legacy) {
        (Ok(shared), Ok(legacy)) => assert_eq!(
            shared.bounds(),
            Some((legacy.lower().bits(), legacy.upper().bits()))
        ),
        (Err(IntervalError::Overflow), Err(FocError::Range))
        | (Err(IntervalError::ContainsZero), Err(FocError::DivideByZero))
        | (Err(IntervalError::ReversedEndpoints), Err(FocError::IntervalOrder)) => {}
        (shared, legacy) => panic!("differential mismatch: shared={shared:?}, legacy={legacy:?}"),
    }
}

fn exercise(a: i32, b: i32, c: i32, d: i32) {
    compare(
        Shared::new(a, b),
        Q30Interval::new(Q30::from_bits(a), Q30::from_bits(b)),
    );
    let (a, b) = (a.min(b), a.max(b));
    let (c, d) = (c.min(d), c.max(d));
    let x = Shared::new(a, b).unwrap();
    let y = Shared::new(c, d).unwrap();
    let old_x = Q30Interval::new(Q30::from_bits(a), Q30::from_bits(b)).unwrap();
    let old_y = Q30Interval::new(Q30::from_bits(c), Q30::from_bits(d)).unwrap();
    compare(x.checked_add(&y), old_x.checked_add(old_y));
    compare(x.checked_sub(&y), old_x.checked_sub(old_y));
    compare(x.checked_mul(&y), old_x.checked_mul(old_y));
    compare(x.checked_div(&y), old_x.checked_div(old_y));
    compare(x.checked_neg(), old_x.checked_neg());
    compare(x.square(), old_x.checked_square());
    assert_eq!(
        x.midpoint(MidpointRounding::Floor),
        Some(old_x.midpoint().bits())
    );
    assert_eq!(x.width_ulps(), Some(old_x.width_ulps()));
}

#[test]
fn q30_boundary_cartesian_matrix_matches() {
    let boundaries = [
        i32::MIN,
        i32::MIN + 1,
        -(1 << 30),
        -2,
        -1,
        0,
        1,
        2,
        1 << 29,
        1 << 30,
        i32::MAX - 1,
        i32::MAX,
    ];
    for a in boundaries {
        for b in boundaries {
            for c in boundaries {
                for d in boundaries {
                    exercise(a, b, c, d);
                }
            }
        }
    }
}

#[test]
fn q30_deterministic_random_matrix_matches() {
    // Reproducible full-width values; numerical expectations remain the old
    // kernel plus the independent exact tests in the owning crate.
    let mut state = 0x98bc_a56f_2026_1112_u64;
    let mut next = || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        state
    };
    for _ in 0..100_000 {
        exercise(next() as i32, next() as i32, next() as i32, next() as i32);
        let numerator = next() as i64;
        let denominator = next() as i64;
        compare(
            Shared::from_ratio(numerator, denominator),
            Q30Interval::from_ratio(numerator, denominator),
        );
    }
}

#[test]
fn q30_extreme_ratio_matrix_matches() {
    let values = [
        i64::MIN,
        i64::MIN + 1,
        -3,
        -1,
        0,
        1,
        3,
        i64::MAX - 1,
        i64::MAX,
    ];
    for numerator in values {
        for denominator in values {
            compare(
                Shared::from_ratio(numerator, denominator),
                Q30Interval::from_ratio(numerator, denominator),
            );
        }
    }
}
