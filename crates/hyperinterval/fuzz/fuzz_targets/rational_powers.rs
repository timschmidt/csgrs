#![no_main]
use hyperinterval::{IntervalError, RationalInterval as I};
use hyperreal::{Problem, Rational as R};
use libfuzzer_sys::fuzz_target;
use rug::{Rational as G, ops::Pow};

#[path = "../../tests/support/rational_oracle.rs"]
mod oracle;

fuzz_target!(|data: [i16; 5]| {
    let [a, b, c, d, p] = data;
    let ad = u32::from(c.unsigned_abs()) + 1;
    let bd = u32::from(d.unsigned_abs()) + 1;
    let aa = G::from((a, ad));
    let bb = G::from((b, bd));
    let (lo, hi) = if aa <= bb { (aa, bb) } else { (bb, aa) };
    let make = |n: i16, den: u32| R::fraction(i64::from(n), u64::from(den)).unwrap();
    let x = I::point(make(a, ad)).hull(&I::point(make(b, bd)));
    if p == i16::MIN && lo <= 0 && hi >= 0 {
        // Directed large-exponent bucket: select the mathematically necessary
        // maximum first. Only zero/unit outputs complete under the native guard;
        // the GMP oracle never attempts these enormous nontrivial powers.
        let high = lo.clone().abs().max(hi.clone().abs());
        let expected = if high == 0 {
            Ok(I::point(R::zero()))
        } else if high == 1 {
            Ok(I::new(R::zero(), R::one()).unwrap())
        } else {
            Err(IntervalError::Scalar(Problem::Exhausted))
        };
        assert_eq!(x.checked_pow(i64::MAX - 1), expected);
    }
    let exponent = i32::from(p % 33);
    if exponent < 0 && lo <= 0 && hi >= 0 {
        assert_eq!(
            x.checked_pow(i64::from(exponent)),
            Err(IntervalError::ContainsZero)
        );
        return;
    }
    // At most 32-fold coefficient growth: this campaign never asks GMP to
    // materialize unbounded random powers. The native guard has named fixtures.
    let mut values = vec![lo.clone().pow(exponent), hi.clone().pow(exponent)];
    if exponent > 0 && lo <= 0 && hi >= 0 {
        values.push(G::from(0));
    }
    let image = x.checked_pow(i64::from(exponent)).unwrap();
    assert_eq!(
        oracle::mpq(image.lower().unwrap()),
        *values.iter().min().unwrap()
    );
    assert_eq!(
        oracle::mpq(image.upper().unwrap()),
        *values.iter().max().unwrap()
    );
    let bytes = serde_json::to_vec(&image).unwrap();
    let replay: I = serde_json::from_slice(&bytes).unwrap();
    assert_eq!(replay, image);
    assert_eq!(replay.checked_pow(-1), image.reciprocal());
    assert_eq!(replay.abs().square(), image.square());
});
