//! Strict decisions for a degree-ten unit-circle identity and policy boundaries.
//!
//! The ordinary rational normal form declines the constructed zero; the strict
//! predicate must nevertheless prove it exactly. The assertions leave the
//! successful dispatch stage unconstrained. A separate angle-addition identity
//! exercises an unresolved strict result and an approximate-policy receipt.

use std::cmp::Ordering;

use hyperlimit::{
    Certainty, PredicateOutcome, PredicatePolicy, Sign, classify_real_sign, compare_reals,
};
use hyperreal::{Rational, Real};

/// Return sine and cosine of exactly 1/10000 degree, or `pi/1800000` radians.
fn tiny_turn() -> (Real, Real) {
    let angle = Real::new(Rational::fraction(1, 10_000).unwrap()).to_radians();
    (angle.clone().sin(), angle.cos())
}

/// Construct `(sin^2 + cos^2)^5`, exactly one, with formal degree ten in sine/cosine.
///
/// The shared squared subexpression retains the product construction under test.
fn unit_circle_fifth_power() -> Real {
    let (sine, cosine) = tiny_turn();
    let circle = &sine * &sine + &cosine * &cosine;
    let squared = &circle * &circle;
    &squared * &squared * &circle
}

/// Extract a decided value only after requiring Exact certainty; panic on uncertainty.
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

/// Check exact zero/equality after ordinary normal-form decline, and signs of offsets.
///
/// The offsets are exactly plus and minus `2^-60`. No assertion selects which
/// strict decision stage succeeds.
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
    // Nonzero offsets of plus and minus 2^-60 keep their exact signs.
    for (numerator, sign) in [(-1, Sign::Negative), (1, Sign::Positive)] {
        let offset = Real::new(Rational::fraction(numerator, 1_u64 << 60).unwrap());
        let value = unit_circle_fifth_power() - Real::one() + offset;
        assert_eq!(
            decided(classify_real_sign(&value, PredicatePolicy::STRICT)),
            sign
        );
    }
}

/// Require strict uncertainty for the angle-addition zero at `e + 1`.
///
/// Approximate policy must return a decided approximate receipt; this fixture
/// does not assert its value, uncertainty reason, or dispatch stage.
#[test]
fn identities_outside_the_extended_attempt_stay_undecided() {
    // This angle-addition fixture currently remains unresolved under STRICT.
    // The approximate-policy assertion below checks its receipt, not its value.
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
