//! The last exact stage of the strict sign cascade: one larger relation
//! attempt after the normal form and the radical towers decline.
//!
//! Rotations by rational turns beyond the cyclotomic order limit, such as
//! 1/10000 degree, leave zeros that only `sin^2 + cos^2 = 1` decides, at a
//! degree above the ordinary limit of 8.

use std::cmp::Ordering;

use hyperlimit::{
    Certainty, PredicateOutcome, PredicatePolicy, Sign, classify_real_sign, compare_reals,
};
use hyperreal::{Rational, Real};

fn tiny_turn() -> (Real, Real) {
    let angle = Real::new(Rational::fraction(1, 10_000).unwrap()).to_radians();
    (angle.clone().sin(), angle.cos())
}

/// `(sin^2 + cos^2)^5`, which is exactly one.
fn unit_circle_fifth_power() -> Real {
    let (sine, cosine) = tiny_turn();
    let circle = &sine * &sine + &cosine * &cosine;
    let squared = &circle * &circle;
    &squared * &squared * &circle
}

fn decided<T: std::fmt::Debug>(outcome: PredicateOutcome<T>) -> T {
    match outcome {
        PredicateOutcome::Decided {
            value, certainty, ..
        } => {
            assert_eq!(certainty, Certainty::Exact);
            value
        }
        PredicateOutcome::Unknown { .. } => panic!("strict predicate must decide"),
    }
}

#[test]
fn degree_ten_unit_circle_zeros_decide_under_strict() {
    let zero = unit_circle_fifth_power() - Real::one();
    // The ordinary normal form declines at its degree limit.
    assert_eq!(zero.exact_rational_normal_form(), None);
    assert_eq!(
        decided(classify_real_sign(&zero, PredicatePolicy::STRICT)),
        Sign::Zero
    );
    assert_eq!(
        decided(compare_reals(
            &unit_circle_fifth_power(),
            &Real::one(),
            PredicatePolicy::STRICT
        )),
        Ordering::Equal
    );
    // Nonzero offsets keep their signs, also far below refinement precision.
    for (numerator, sign) in [(-1, Sign::Negative), (1, Sign::Positive)] {
        let offset = Real::new(Rational::fraction(numerator, 1_u64 << 60).unwrap());
        let value = unit_circle_fifth_power() - Real::one() + offset;
        assert_eq!(
            decided(classify_real_sign(&value, PredicatePolicy::STRICT)),
            sign
        );
    }
}

#[test]
fn identities_outside_the_extended_attempt_stay_undecided() {
    // Angle addition is outside every exact stage; strict policy reports it
    // undecided instead of guessing, and the approximate terminal remains
    // the only route that answers.
    let zero = (Real::e() + Real::one()).sin()
        - Real::e().sin() * Real::one().cos()
        - Real::e().cos() * Real::one().sin();
    assert!(matches!(
        classify_real_sign(&zero, PredicatePolicy::STRICT),
        PredicateOutcome::Unknown { .. }
    ));
    assert!(matches!(
        classify_real_sign(&zero, PredicatePolicy::APPROXIMATE_512),
        PredicateOutcome::Decided {
            certainty: Certainty::Approximate,
            ..
        }
    ));
}
