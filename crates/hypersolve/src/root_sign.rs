//! Exact signs on a certified singleton through the Sturm-Tarski theorem.
//!
//! For endpoint-free (a,b), Var(SRemS(P,P'Q);a,b) is the sum of signs of Q
//! at P's distinct roots. A certified singleton therefore gives one sign.
//! See Li et al., https://doi.org/10.1007/s10817-017-9424-6, Theorem 1.

use std::cmp::Ordering;

use hyperlimit::{PredicatePolicy, compare_reals};
use hyperreal::Real;

use crate::integer_interpolation::primitive_integer_signed_remainder_sequence;
use crate::ordered_field_roots::{
    OrderedFieldPolynomialContext, ordered_field_polynomial_sign_remainder, trim_polynomial,
};
use crate::root_isolation::{IsolatedRootInterval, polynomial_div_rem};

/// Signs a polynomial at a caller-certified singleton over an exact ordered field.
///
/// Coefficients are in ascending power order. `defining` must have exactly one
/// distinct root in the supplied interval; selection and coefficient-field
/// identity remain owned by the caller. Repeated roots are allowed. Endpoint
/// roots, point intervals and invalid interval metadata return `None`.
///
/// The division-free Sturm-Tarski chain uses only the existing field
/// arithmetic and exact signs. Positive pseudo-remainder scales preserve the
/// query's sign, including for nonmonic and negative leading coefficients.
/// Only two successive polynomials are retained; no root refinement, field
/// projection or independent root isolation is required. Unavailable field
/// decisions propagate through the context's error type.
pub fn ordered_field_sign_at_selected_root<C: Clone, F: OrderedFieldPolynomialContext<C>>(
    defining: &[C],
    predicate: &[C],
    interval: &IsolatedRootInterval,
    field: &mut F,
) -> Result<Option<Ordering>, F::Error> {
    if interval.distinct_root_count != 1
        || sign(&(&interval.upper - &interval.lower)) != Some(Ordering::Greater)
    {
        return Ok(None);
    }
    let mut first = defining.to_vec();
    trim_polynomial(&mut first, field)?;
    if first.len() < 2 {
        return Ok(None);
    }
    let lower_sign = field_polynomial_sign(&first, &interval.lower, field)?;
    let upper_sign = field_polynomial_sign(&first, &interval.upper, field)?;
    if lower_sign == Ordering::Equal || upper_sign == Ordering::Equal {
        return Ok(None);
    }
    let mut predicate = predicate.to_vec();
    trim_polynomial(&mut predicate, field)?;
    if predicate.is_empty() {
        return Ok(Some(Ordering::Equal));
    }
    if let [constant] = predicate.as_slice() {
        return field.sign(constant).map(Some);
    }
    let Some(product_len) = (first.len() - 1).checked_add(predicate.len() - 1) else {
        return Ok(None);
    };
    let mut second = vec![field.zero()?; product_len];
    for (power, coefficient) in first.iter().enumerate().skip(1) {
        let derivative = field.scale(coefficient, &Real::from(power as u128))?;
        for (other_power, other) in predicate.iter().enumerate() {
            let term = field.multiply(&derivative, other)?;
            let entry = &mut second[power - 1 + other_power];
            *entry = field.add(entry, &term)?;
        }
    }
    // The first polynomial's endpoint signs were already certified above.
    let mut previous = [lower_sign, upper_sign];
    let mut variations = [0_usize; 2];
    loop {
        trim_polynomial(&mut second, field)?;
        if second.is_empty() {
            return Ok(query_sign_from_variations(variations[0], variations[1]));
        }
        for (endpoint, point) in [&interval.lower, &interval.upper].into_iter().enumerate() {
            let current = field_polynomial_sign(&second, point, field)?;
            if current != Ordering::Equal {
                variations[endpoint] += usize::from(previous[endpoint] != current);
                previous[endpoint] = current;
            }
        }
        // A constant, including the retained zero coefficient, ends the chain.
        if second.len() == 1 {
            return Ok(query_sign_from_variations(variations[0], variations[1]));
        }
        let Some(remainder) = ordered_field_polynomial_sign_remainder(&first, &second, field)?
        else {
            return Ok(None);
        };
        first = second;
        second = remainder
            .iter()
            .map(|coefficient| field.scale(coefficient, &Real::from(-1_i8)))
            .collect::<Result<Vec<_>, _>>()?;
    }
}

fn field_polynomial_sign<C: Clone, F: OrderedFieldPolynomialContext<C>>(
    polynomial: &[C],
    point: &Real,
    field: &mut F,
) -> Result<Ordering, F::Error> {
    let Some((leading, remaining)) = polynomial.split_last() else {
        return Ok(Ordering::Equal);
    };
    let mut value = leading.clone();
    for coefficient in remaining.iter().rev() {
        value = field.scale(&value, point)?;
        value = field.add(&value, coefficient)?;
    }
    field.sign(&value)
}

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
    query_sign_from_variations(lower, upper)
}

fn query_sign_from_variations(lower: usize, upper: usize) -> Option<Ordering> {
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

    struct RealContext;

    impl OrderedFieldPolynomialContext<Real> for RealContext {
        type Error = ();

        fn zero(&mut self) -> Result<Real, Self::Error> {
            Ok(Real::zero())
        }

        fn add(&mut self, left: &Real, right: &Real) -> Result<Real, Self::Error> {
            Ok(left + right)
        }

        fn multiply(&mut self, left: &Real, right: &Real) -> Result<Real, Self::Error> {
            Ok(left * right)
        }

        fn scale(&mut self, value: &Real, scale: &Real) -> Result<Real, Self::Error> {
            Ok(value * scale)
        }

        fn sign(&mut self, value: &Real) -> Result<Ordering, Self::Error> {
            sign(value).ok_or(())
        }

        fn sign_if_separated(&mut self, value: &Real) -> Result<Option<Ordering>, Self::Error> {
            Ok(sign(value))
        }
    }

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
            prop_assert_eq!(
                ordered_field_sign_at_selected_root(
                    &defining, &predicate, &interval, &mut RealContext,
                ).unwrap(),
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
                for predicate in [
                    vec![Real::zero(), Real::one()],
                    vec![Real::from(-2), Real::zero(), Real::one()],
                    vec![Real::from(-3), Real::zero(), Real::one()],
                ] {
                    assert_eq!(
                        ordered_field_sign_at_selected_root(
                            &defining,
                            &predicate,
                            &interval,
                            &mut RealContext,
                        )
                        .unwrap(),
                        sign_at_selected_root(&defining, &predicate, &interval)
                    );
                }
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
        assert_eq!(
            ordered_field_sign_at_selected_root(&defining, &defining, &interval, &mut RealContext)
                .unwrap(),
            Some(Ordering::Equal)
        );
        interval.upper = alpha;
        assert_eq!(
            sign_at_selected_root(&defining, &[Real::one()], &interval),
            None
        );
        assert_eq!(
            ordered_field_sign_at_selected_root(
                &defining,
                &[Real::one()],
                &interval,
                &mut RealContext,
            )
            .unwrap(),
            None
        );
        interval.lower = interval.upper.clone();
        assert_eq!(
            ordered_field_sign_at_selected_root(&defining, &[], &interval, &mut RealContext,)
                .unwrap(),
            None
        );
    }

    #[test]
    fn ordered_field_query_preserves_proper_factors_and_tiny_signs() {
        let radical = Real::from(2_i8).sqrt().unwrap();
        let tiny = Real::from(2_i8).powi_i64(-600).unwrap();
        let interval = IsolatedRootInterval {
            lower: Real::one(),
            upper: (Real::from(3_i8) / Real::from(2_i8)).unwrap(),
            exact_root: None,
            distinct_root_count: 1,
        };
        for gauge in [Real::from(3_i8), Real::from(-3_i8)] {
            // (x^2-sqrt(2))*(x-2), selecting its positive fourth root of 2.
            let defining = [
                Real::from(2_i8) * &radical * &gauge,
                -&radical * &gauge,
                Real::from(-2_i8) * &gauge,
                gauge,
            ];
            for (shift, expected) in [
                (Real::zero(), Ordering::Equal),
                (tiny.clone(), Ordering::Greater),
                (-tiny.clone(), Ordering::Less),
            ] {
                let query = [-&radical + shift, Real::zero(), Real::one()];
                assert_eq!(
                    ordered_field_sign_at_selected_root(
                        &defining,
                        &query,
                        &interval,
                        &mut RealContext,
                    )
                    .unwrap(),
                    Some(expected)
                );
            }
        }
    }

    #[test]
    fn ordered_field_query_declines_unknown_degrees_and_invalid_selection() {
        let epsilon = Real::one() - Real::from(2_i8).powi_i64(-600).unwrap().cos();
        let interval = IsolatedRootInterval {
            lower: Real::zero(),
            upper: Real::one(),
            exact_root: None,
            distinct_root_count: 1,
        };
        let defining = [Real::from(-1_i8), Real::from(2_i8), epsilon.clone()];
        assert!(
            ordered_field_sign_at_selected_root(
                &defining,
                &[Real::one()],
                &interval,
                &mut RealContext,
            )
            .is_err()
        );
        assert!(
            ordered_field_sign_at_selected_root(
                &defining[..2],
                &[Real::one(), epsilon],
                &interval,
                &mut RealContext,
            )
            .is_err()
        );
        assert_eq!(
            ordered_field_sign_at_selected_root(
                &defining[..2],
                &[Real::one()],
                &IsolatedRootInterval {
                    distinct_root_count: 2,
                    ..interval
                },
                &mut RealContext,
            )
            .unwrap(),
            None
        );
    }
}
