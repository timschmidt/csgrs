//! MPFR monotone point bounds plus GMP branch indices and one-sided limits.
use hyperinterval::{DomainCoverage, DyadicInterval, IntervalError, IntervalSet2, RangeEnclosure};
use rug::{Float, Integer, Rational, float::Round};
#[path = "set_oracle.rs"]
mod set_oracle;
pub use set_oracle::{End, Source, actual, q};
#[path = "phase_oracle.rs"]
mod phase_oracle;
pub use phase_oracle::half_pi;
// Each branch is bounded by consecutive poles. This closed-form floor
// calculation is independent of production's extremum-residue scan.
fn branch(x: &Rational, cot: bool, pi: &(Rational, Rational)) -> Integer {
    let half_index = phase_oracle::index(x, pi);
    let shifted = if cot { half_index } else { half_index + 1 };
    (Rational::from(shifted) / 2i32).floor().numer().clone()
}
fn point(
    x: &Rational,
    cot: bool,
    precision: i32,
    pi: &(Rational, Rational),
) -> (Rational, Rational) {
    if x == &0 {
        assert!(!cot);
        return (0.into(), 0.into());
    }
    crate::mpfr_cache::cleanup_on_thread_exit();
    let (mut lo, _) = Float::with_val_round(4096, x, Round::Down);
    let (mut hi, _) = Float::with_val_round(4096, x, Round::Up);
    let expected = branch(x, cot, pi);
    assert_eq!(branch(&lo.to_rational().unwrap(), cot, pi), expected);
    assert_eq!(branch(&hi.to_rational().unwrap(), cot, pi), expected);
    if cot {
        (lo, hi) = (hi, lo);
        lo.cot_round(Round::Down);
        hi.cot_round(Round::Up);
    } else {
        lo.tan_round(Round::Down);
        hi.tan_round(Round::Up);
    }
    set_oracle::bounds(
        lo.to_rational().unwrap(),
        hi.to_rational().unwrap(),
        precision,
    )
}
pub fn expected(
    source: &Source,
    precision: i32,
    cot: bool,
) -> Result<(Vec<(End, End)>, DomainCoverage), IntervalError> {
    use End::{Minus as M, Plus as P, Value as V};
    let Some((a, b)) = source else {
        return Ok((vec![], DomainCoverage::Complete));
    };
    let (V(a), V(b)) = (a, b) else {
        return Ok((vec![(M, P)], DomainCoverage::Partial));
    };
    if cot && a == &0 && b == &0 {
        return Err(IntervalError::EmptyDomain);
    }
    let pi = half_pi();
    let endpoint_left = cot && a == &0;
    let endpoint_right = cot && b == &0;
    let count = branch(b, cot, &pi) - branch(a, cot, &pi) - i32::from(endpoint_right);
    if count >= 2 || (count >= 1 && (endpoint_left || endpoint_right)) {
        return Ok((vec![(M, P)], DomainCoverage::Partial));
    }
    let x = if endpoint_left {
        None
    } else {
        Some(point(a, cot, precision, &pi))
    };
    let y = if endpoint_right {
        None
    } else {
        Some(point(b, cot, precision, &pi))
    };
    let (lo, hi) = if cot { (y, x) } else { (x, y) };
    let mut parts = match (lo, hi) {
        (Some((lo, _)), Some((_, hi))) if count == 0 => {
            return Ok((vec![(V(lo), V(hi))], DomainCoverage::Complete));
        }
        (Some((lo, _)), Some((_, hi))) => vec![(M, V(hi)), (V(lo), P)],
        (Some((lo, _)), None) => vec![(V(lo), P)],
        (None, Some((_, hi))) => vec![(M, V(hi))],
        (None, None) => unreachable!(),
    };
    if parts.len() == 2 && parts[0].1 >= parts[1].0 {
        parts = vec![(M, P)];
    }
    Ok((parts, DomainCoverage::Partial))
}
pub fn verify(
    result: Result<RangeEnclosure<IntervalSet2<DyadicInterval>>, IntervalError>,
    source: &Source,
    precision: i32,
    cot: bool,
) {
    match expected(source, precision, cot) {
        Err(e) => assert_eq!(result, Err(e)),
        Ok((parts, domain)) => {
            let result = result.unwrap();
            assert_eq!(result.domain, domain, "{source:?},cot={cot}");
            assert_eq!(
                actual(&result.range),
                parts,
                "{source:?},p={precision},cot={cot}"
            );
        }
    }
}
