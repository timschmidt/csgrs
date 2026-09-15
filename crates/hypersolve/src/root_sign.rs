//! Exact signs on a certified singleton through the Sturm-Tarski theorem.
//!
//! For endpoint-free (a,b), Var(SRemS(P,P'Q);a,b) is the sum of signs of Q
//! at P's distinct roots. A certified singleton therefore gives one sign.
//! See Li et al., https://doi.org/10.1007/s10817-017-9424-6, Theorem 1.

use std::cmp::Ordering;

use hyperlimit::{PredicatePolicy, compare_reals};
use hyperreal::Real;

use crate::integer_interpolation::primitive_integer_signed_remainder_sequence;
use crate::root_isolation::{IsolatedRootInterval, polynomial_div_rem};

/// Signs `predicate` at the unique selected root of `defining` in `interval`.
///
/// Both polynomials use ascending power coefficients. The caller owns the
/// singleton proof for `defining` on `interval`; no roots of the predicate
/// need to be isolated. `None` means this exact sign query did not decide.
/// Endpoint roots and exact point intervals decline so endpoint ownership
/// remains explicit; callers can evaluate an exact point directly.
/// Every coefficient decision here is STRICT, including the general-Real
/// fallback when the primitive integer chain does not apply.
pub fn sign_at_selected_root(
    defining: &[Real],
    predicate: &[Real],
    interval: &IsolatedRootInterval,
) -> Option<Ordering> {
    if defining.len() < 2
        || interval.distinct_root_count != 1
        || sign(&(&interval.upper - &interval.lower))? != Ordering::Greater
        || sign(&Real::eval_poly(defining, &interval.lower))? == Ordering::Equal
        || sign(&Real::eval_poly(defining, &interval.upper))? == Ordering::Equal
    {
        return None;
    }
    if predicate.is_empty() {
        return Some(Ordering::Equal);
    }
    let product_len = defining
        .len()
        .checked_add(predicate.len())?
        .checked_sub(2)?;
    let mut product = vec![Real::zero(); product_len];
    for (power, coefficient) in defining.iter().enumerate().skip(1) {
        let derivative = coefficient * Real::from(power as u128);
        for (other_power, other) in predicate.iter().enumerate() {
            product[power - 1 + other_power] += &derivative * other;
        }
    }
    let chain = primitive_integer_signed_remainder_sequence(defining, &product)
        .or_else(|| field_signed_sequence(defining.to_vec(), product))?;
    let lower = variations(&chain, &interval.lower)?;
    let upper = variations(&chain, &interval.upper)?;
    match (lower.checked_sub(upper), upper.checked_sub(lower)) {
        (Some(1), _) => Some(Ordering::Greater),
        (Some(0), Some(0)) => Some(Ordering::Equal),
        (_, Some(1)) => Some(Ordering::Less),
        _ => None,
    }
}

fn sign(value: &Real) -> Option<Ordering> {
    compare_reals(value, &Real::zero(), PredicatePolicy::STRICT).value()
}

fn field_signed_sequence(mut first: Vec<Real>, mut second: Vec<Real>) -> Option<Vec<Vec<Real>>> {
    let mut chain = vec![first.clone()];
    loop {
        while second
            .last()
            .is_some_and(|value| sign(value) == Some(Ordering::Equal))
        {
            second.pop();
        }
        if second.is_empty() {
            return Some(chain);
        }
        sign(second.last()?)?;
        let remainder = polynomial_div_rem(first, &second, PredicatePolicy::STRICT)?.1;
        chain.push(second.clone());
        first = second;
        second = remainder
            .into_iter()
            .map(|coefficient| -coefficient)
            .collect();
    }
}

fn variations(chain: &[Vec<Real>], point: &Real) -> Option<usize> {
    let mut previous = None;
    let mut count = 0;
    for polynomial in chain {
        let current = sign(&Real::eval_poly(polynomial, point))?;
        if current != Ordering::Equal {
            count += usize::from(previous.is_some_and(|previous| previous != current));
            previous = Some(current);
        }
    }
    Some(count)
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    proptest! {
        #[test]
        fn generated_signs_match_exact_evaluation_at_the_owned_root(
            selected in -3_i32..=3,
            other_roots in prop::collection::vec(-3_i32..=3, 0..6),
            predicate in prop::collection::vec(-5_i32..=5, 0..7),
            negate in any::<bool>(),
        ) {
            let mut defining = vec![Real::from(if negate { -1 } else { 1 })];
            for root in std::iter::once(selected).chain(other_roots) {
                let mut product = vec![Real::zero(); defining.len() + 1];
                for (power, coefficient) in defining.iter().enumerate() {
                    product[power] -= coefficient * Real::from(root);
                    product[power + 1] += coefficient;
                }
                defining = product;
            }
            let selected = Real::from(selected);
            let half = (Real::one() / Real::from(2)).unwrap();
            let interval = IsolatedRootInterval {
                lower: &selected - &half,
                upper: &selected + &half,
                exact_root: None,
                distinct_root_count: 1,
            };
            let predicate = predicate.into_iter().map(Real::from).collect::<Vec<_>>();
            prop_assert_eq!(
                sign_at_selected_root(&defining, &predicate, &interval),
                sign(&Real::eval_poly(&predicate, &selected)),
            );
        }
    }

    #[test]
    fn signs_select_the_owned_conjugate_and_allow_repeated_roots() {
        for defining in [
            vec![Real::from(-2), Real::zero(), Real::one()],
            vec![
                Real::from(4),
                Real::zero(),
                Real::from(-4),
                Real::zero(),
                Real::one(),
            ],
        ] {
            for (lower, upper, expected) in [(-2, -1, Ordering::Less), (1, 2, Ordering::Greater)] {
                let interval = IsolatedRootInterval {
                    lower: Real::from(lower),
                    upper: Real::from(upper),
                    exact_root: None,
                    distinct_root_count: 1,
                };
                assert_eq!(
                    sign_at_selected_root(&defining, &[Real::zero(), Real::one()], &interval),
                    Some(expected)
                );
                assert_eq!(
                    sign_at_selected_root(
                        &defining,
                        &[Real::from(-2), Real::zero(), Real::one()],
                        &interval
                    ),
                    Some(Ordering::Equal)
                );
                assert_eq!(
                    sign_at_selected_root(
                        &defining,
                        &[Real::from(-3), Real::zero(), Real::one()],
                        &interval
                    ),
                    Some(Ordering::Less)
                );
            }
        }
    }

    #[test]
    fn exact_real_coefficients_and_endpoint_ownership_remain_explicit() {
        let alpha = Real::from(2).sqrt().unwrap();
        let defining = [-alpha.clone(), Real::one()];
        let mut interval = IsolatedRootInterval {
            lower: Real::one(),
            upper: Real::from(2),
            exact_root: None,
            distinct_root_count: 1,
        };
        assert_eq!(
            sign_at_selected_root(&defining, &[Real::from(-1), Real::one()], &interval),
            Some(Ordering::Greater)
        );
        assert_eq!(
            sign_at_selected_root(&defining, &defining, &interval),
            Some(Ordering::Equal)
        );
        interval.upper = alpha;
        assert_eq!(
            sign_at_selected_root(&defining, &[Real::one()], &interval),
            None
        );
    }
}
