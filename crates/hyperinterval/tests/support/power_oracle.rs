//! Independent GMP power/root/grid expectations over monotone sign pieces.
use hyperinterval::{DomainCoverage, DyadicInterval, IntervalError, IntervalSet2, RangeEnclosure};
use rug::{Rational, ops::Pow};
#[path = "set_oracle.rs"]
mod set_oracle;
pub use set_oracle::{End, Source, actual, q};
fn zero() -> End {
    End::Value(0.into())
}
fn finite(x: &Rational, n: i32, d: u32, precision: i32) -> (End, End) {
    let mut value = x.clone().abs().pow(n.unsigned_abs());
    if n < 0 {
        value.recip_mut();
    }
    let shift = -i64::from(precision) * i64::from(d);
    if shift >= 0 {
        value <<= shift as u32;
    } else {
        value >>= shift.unsigned_abs() as u32;
    }
    let integral = value.clone().floor().numer().clone();
    let root = integral.root(d);
    let exact = root.clone().pow(d) == value;
    let ceil = if exact {
        root.clone()
    } else {
        root.clone() + 1
    };
    let (a, b) = if x < &0 && n.unsigned_abs() % 2 == 1 {
        (-ceil, -root)
    } else {
        (root, ceil)
    };
    (End::Value(q(&a, precision)), End::Value(q(&b, precision)))
}
fn limit(x: &End, side: i32, n: i32, d: u32, precision: i32) -> (End, End) {
    let infinite = || {
        if side < 0 && n.unsigned_abs() % 2 == 1 {
            End::Minus
        } else {
            End::Plus
        }
    };
    match x {
        End::Value(x) if x != &0 => finite(x, n, d, precision),
        End::Value(_) if n > 0 => (zero(), zero()),
        End::Value(_) => (infinite(), infinite()),
        _ if n < 0 => (zero(), zero()),
        _ => (infinite(), infinite()),
    }
}
pub fn expected(
    source: &Source,
    n: i32,
    d: i32,
    precision: i32,
) -> Result<(Vec<(End, End)>, DomainCoverage), IntervalError> {
    if d == 0 {
        return Err(IntervalError::ContainsZero);
    }
    let Some((a, b)) = source else {
        return Ok((vec![], DomainCoverage::Complete));
    };
    if n == 0 {
        let (a, b) = set_oracle::bounds(1.into(), 1.into(), precision);
        return Ok((
            vec![(End::Value(a), End::Value(b))],
            DomainCoverage::Complete,
        ));
    }
    let exponent = Rational::from((n, d));
    let n = exponent.numer().to_i32().unwrap();
    let d = exponent.denom().to_u32().unwrap();
    let has_zero = a <= &zero() && b >= &zero();
    let partial = (a < &zero() && d % 2 == 0) || (has_zero && n < 0);
    let mut pieces = vec![];
    if a < &zero() && d % 2 == 1 {
        let x = limit(a, -1, n, d, precision);
        let y = limit(&b.clone().min(zero()), -1, n, d, precision);
        pieces.push((x.0.min(y.0), x.1.max(y.1)));
    }
    if b > &zero() {
        let x = limit(&a.clone().max(zero()), 1, n, d, precision);
        let y = limit(b, 1, n, d, precision);
        pieces.push((x.0.min(y.0), x.1.max(y.1)));
    }
    if has_zero && n > 0 {
        pieces.push((zero(), zero()));
    }
    if pieces.is_empty() {
        return Err(IntervalError::EmptyDomain);
    }
    pieces.sort();
    let mut merged: Vec<(End, End)> = vec![];
    for (a, b) in pieces {
        if let Some(last) = merged.last_mut()
            && last.1 >= a
        {
            last.1 = last.1.clone().max(b);
        } else {
            merged.push((a, b));
        }
    }
    Ok((
        merged,
        if partial {
            DomainCoverage::Partial
        } else {
            DomainCoverage::Complete
        },
    ))
}
pub fn verify(
    result: Result<RangeEnclosure<IntervalSet2<DyadicInterval>>, IntervalError>,
    source: &Source,
    n: i32,
    d: i32,
    precision: i32,
) {
    match expected(source, n, d, precision) {
        Err(e) => assert_eq!(result, Err(e)),
        Ok((parts, coverage)) => {
            let result = result.unwrap();
            assert_eq!(result.domain, coverage, "{source:?} ^ {n}/{d}");
            assert_eq!(
                actual(&result.range),
                parts,
                "{source:?} ^ {n}/{d} grid={precision}"
            );
        }
    }
}
// Additional directed MPFR oracle for non-grid point values; exact grid cases
// are independently resolved by the GMP integer relation above.
pub fn mpfr_point(result: &IntervalSet2<DyadicInterval>, source: &Rational, n: i32, d: u32) {
    use rug::{Float, float::Round};
    let mut lo = Float::with_val_round(4096, source.clone().abs(), Round::Down).0;
    let mut hi = Float::with_val_round(4096, source.clone().abs(), Round::Up).0;
    lo.root_round(d, Round::Down);
    hi.root_round(d, Round::Up);
    use rug::ops::PowAssignRound;
    lo.pow_assign_round(n.unsigned_abs(), Round::Down);
    hi.pow_assign_round(n.unsigned_abs(), Round::Up);
    if n < 0 {
        (lo, hi) = (hi, lo);
        lo.recip_round(Round::Down);
        hi.recip_round(Round::Up);
    }
    if source < &0 && n.unsigned_abs() % 2 == 1 {
        (lo, hi) = (-hi, -lo);
    }
    let parts = actual(result);
    assert_eq!(parts.len(), 1);
    let (End::Value(a), End::Value(b)) = &parts[0] else {
        panic!("finite point image")
    };
    let (lo, hi) = (lo.to_rational().unwrap(), hi.to_rational().unwrap());
    if a == b {
        // A rounded intermediate root can straddle an exact final power.
        // GMP separately proves the singleton relation; MPFR must contain it.
        assert!(&lo <= a && a <= &hi);
    } else {
        assert!(a <= &lo && b >= &hi);
    }
}
