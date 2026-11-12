//! Independent four-corner/limit image of each denominator sign component.
use hyperinterval::{DomainCoverage, DyadicInterval, IntervalError, IntervalSet2, RangeEnclosure};
#[path = "set_oracle.rs"]
mod set_oracle;
pub use set_oracle::{End, Source, actual};
// Fuzz and cost consumers also construct exact scaled source values.
#[allow(unused_imports)]
pub use set_oracle::q;
fn zero() -> End {
    End::Value(0.into())
}
fn sign(x: &End) -> i32 {
    if x < &zero() {
        -1
    } else if x > &zero() {
        1
    } else {
        0
    }
}
fn corner(n: &End, d: &End, denominator_sign: i32) -> Option<End> {
    match (n, d) {
        (End::Value(n), End::Value(d)) if d != &0 => Some(End::Value(n.clone() / d)),
        (End::Value(n), End::Minus | End::Plus) => {
            let _ = n;
            Some(zero())
        }
        (End::Minus | End::Plus, End::Minus | End::Plus) => None,
        _ => {
            let s = sign(n) * denominator_sign;
            if s < 0 {
                Some(End::Minus)
            } else if s > 0 {
                Some(End::Plus)
            } else {
                None
            }
        }
    }
}
fn image(a: &(End, End), b: &(End, End), branch: i32, precision: i32) -> (End, End) {
    let mut values = Vec::new();
    for n in [&a.0, &a.1] {
        for d in [&b.0, &b.1] {
            if let Some(v) = corner(n, d, branch) {
                values.push(v);
            }
        }
    }
    if a.0 <= zero() && a.1 >= zero() {
        values.push(zero());
    }
    values.sort();
    let mut lo = values.first().unwrap().clone();
    let mut hi = values.last().unwrap().clone();
    if let End::Value(x) = lo {
        lo = End::Value(set_oracle::bounds(x.clone(), x, precision).0);
    }
    if let End::Value(x) = hi {
        hi = End::Value(set_oracle::bounds(x.clone(), x, precision).1);
    }
    (lo, hi)
}
pub fn expected(
    a: &Source,
    b: &Source,
    precision: i32,
) -> Result<(Vec<(End, End)>, DomainCoverage), IntervalError> {
    if matches!(b,Some((a,b)) if a==&zero()&&b==&zero()) {
        return Err(IntervalError::EmptyDomain);
    }
    let (Some(a), Some(b)) = (a, b) else {
        return Ok((vec![], DomainCoverage::Complete));
    };
    let mut images = Vec::new();
    if b.0 < zero() {
        images.push(image(
            a,
            &(b.0.clone(), b.1.clone().min(zero())),
            -1,
            precision,
        ));
    }
    if b.1 > zero() {
        images.push(image(
            a,
            &(b.0.clone().max(zero()), b.1.clone()),
            1,
            precision,
        ));
    }
    images.sort();
    if images.len() == 2 && images[0].1 >= images[1].0 {
        let hi = images[0].1.clone().max(images[1].1.clone());
        images[0].1 = hi;
        images.pop();
    }
    let coverage = if b.0 <= zero() && b.1 >= zero() {
        DomainCoverage::Partial
    } else {
        DomainCoverage::Complete
    };
    Ok((images, coverage))
}
pub fn verify(
    result: Result<RangeEnclosure<IntervalSet2<DyadicInterval>>, IntervalError>,
    a: &Source,
    b: &Source,
    precision: i32,
) {
    let expected = expected(a, b, precision);
    match expected {
        Err(e) => assert_eq!(result, Err(e)),
        Ok((parts, domain)) => {
            let result = result.unwrap();
            assert_eq!(result.domain, domain);
            assert_eq!(
                actual(&result.range),
                parts,
                "numerator={a:?}, denominator={b:?}, p={precision}"
            );
        }
    }
}
