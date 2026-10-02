//! Exact signs on a certified singleton from enclosures and Sturm-Tarski replay.
//!
//! For endpoint-free (a,b), Var(SRemS(P,P'Q);a,b) is the sum of signs of Q
//! at P's distinct roots. A certified singleton therefore gives one sign.
//! See Li et al., <https://doi.org/10.1007/s10817-017-9424-6>, Theorem 1.

use std::cmp::Ordering;

use hyperlimit::{PredicatePolicy, compare_reals};
use hyperreal::Real;

use crate::algebraic::AlgebraicRootRepresentation;
use crate::integer_interpolation::primitive_integer_signed_remainder_sequence;
use crate::ordered_field_roots::{
    OrderedFieldPolynomialContext, ordered_field_polynomial_sign_remainder, trim_polynomial,
};
use crate::root_isolation::{
    IsolatedRootInterval, polynomial_div_rem, polynomial_has_one_distinct_root_in_open_interval,
};
use crate::tensor_resultant::{DenseTensorPolynomial, compact_exact_coefficients};

/// Signs a tensor polynomial at a caller-certified tuple of exact roots.
///
/// Stored exact point witnesses are substituted without imposing a rational
/// payload requirement. If at most one selected root remains, its original
/// equation and isolator supply the Sturm-Tarski proof. A fully evaluated
/// tensor uses a STRICT scalar sign. More unresolved axes or unavailable exact
/// decisions return `None`; callers retain the original tuple for other proofs.
/// No defining equation, selected root, or interval is replaced in the caller.
pub fn sign_at_selected_tuple(
    polynomial: &DenseTensorPolynomial,
    sources: &[AlgebraicRootRepresentation],
) -> Option<Ordering> {
    if polynomial.dimensions().len() != sources.len() {
        return None;
    }
    let mut selected = None;
    for source in sources {
        if !source.is_valid() || source.interval.distinct_root_count != 1 {
            return None;
        }
        if source.exact_point_witness().is_none() && selected.replace(source).is_some() {
            return None;
        }
    }
    let mut polynomial = polynomial.clone();
    for (axis, source) in sources.iter().enumerate().rev() {
        if let Some(value) = source.exact_point_witness() {
            polynomial = polynomial.substitute_axis_value(axis, value)?;
        }
    }
    let (_, coefficients) = polynomial.compact_coefficients().into_parts();
    match selected {
        Some(source) => sign_at_selected_root(
            &compact_exact_coefficients(source.polynomial_coefficients.clone()),
            &coefficients,
            &source.interval,
        ),
        None => sign(&coefficients[0]),
    }
}

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
/// The field removes available common positive content before it compounds
/// through later remainders, preserving the caller's original root authority.
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
    field.normalize_positive_scale(&mut first);
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
    field.normalize_positive_scale(&mut predicate);
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
    // The chain's first division, `defining' * predicate mod defining`, has
    // a known elimination count; decline before forming its product.
    let mut budget = RemainderBudget::new(field.remainder_elimination_budget());
    if !budget.covers(product_len.saturating_sub(first.len() - 1)) {
        return Ok(None);
    }
    let mut second = vec![field.constant(&Real::zero())?; product_len];
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
        field.normalize_positive_scale(&mut second);
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
        if !budget.spend(&first, &second) {
            return Ok(None);
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

/// Decides whether `predicate` vanishes at the unique selected root of
/// `defining` in `interval`, over an exact ordered field.
///
/// The selected root vanishes `predicate` exactly when it is a root of
/// `gcd(defining, predicate)`. A Euclidean remainder sequence of the two
/// polynomials (positive pseudo-remainder scales; exact leading-coefficient
/// signs) finds that gcd without forming `defining' * predicate` or signing
/// any chain member at the interval endpoints. A constant gcd proves a
/// nonzero value. Because the gcd divides `defining`, a gcd sign change
/// across the interval proves zero at any degree; a linear gcd without one
/// proves a nonzero value. Other gcds, endpoint roots and invalid intervals
/// return `None`; callers keep the complete Sturm-Tarski sign query.
pub fn ordered_field_vanishes_at_selected_root<C: Clone, F: OrderedFieldPolynomialContext<C>>(
    defining: &[C],
    predicate: &[C],
    interval: &IsolatedRootInterval,
    field: &mut F,
) -> Result<Option<bool>, F::Error> {
    if interval.distinct_root_count != 1
        || sign(&(&interval.upper - &interval.lower)) != Some(Ordering::Greater)
    {
        return Ok(None);
    }
    let mut first = defining.to_vec();
    field.normalize_positive_scale(&mut first);
    trim_polynomial(&mut first, field)?;
    if first.len() < 2 {
        return Ok(None);
    }
    let mut second = predicate.to_vec();
    field.normalize_positive_scale(&mut second);
    trim_polynomial(&mut second, field)?;
    if second.len() == 1 && field.sign(&second[0])? == Ordering::Equal {
        second.clear();
    }
    if second.is_empty() {
        return Ok(Some(true));
    }
    let mut budget = RemainderBudget::new(field.remainder_elimination_budget());
    let gcd = loop {
        if second.len() == 1 {
            // A nonzero constant shares no root with the defining relation.
            return Ok(Some(false));
        }
        if !budget.spend(&first, &second) {
            return Ok(None);
        }
        let Some(mut remainder) = ordered_field_polynomial_sign_remainder(&first, &second, field)?
        else {
            return Ok(None);
        };
        field.normalize_positive_scale(&mut remainder);
        trim_polynomial(&mut remainder, field)?;
        if remainder.is_empty()
            || (remainder.len() == 1 && field.sign(&remainder[0])? == Ordering::Equal)
        {
            break second;
        }
        first = second;
        second = remainder;
    };
    // The gcd divides `defining`, whose only root in the interval is the
    // selected one. Any gcd root there is therefore that root, so opposite
    // nonzero endpoint signs prove vanishing at every gcd degree. A linear
    // gcd with equal endpoint signs has its only root outside.
    let lower = field_polynomial_sign(&gcd, &interval.lower, field)?;
    let upper = field_polynomial_sign(&gcd, &interval.upper, field)?;
    Ok(match (lower, upper) {
        (Ordering::Equal, _) | (_, Ordering::Equal) => None,
        (lower, upper) if lower != upper => Some(true),
        _ if gcd.len() == 2 => Some(false),
        _ => None,
    })
}

/// Leading-term eliminations left for one selected-root replay.
struct RemainderBudget(Option<usize>);

impl RemainderBudget {
    const fn new(limit: Option<usize>) -> Self {
        Self(limit)
    }

    /// Whether the remaining budget covers `eliminations` more, without
    /// charging them.
    fn covers(&self, eliminations: usize) -> bool {
        self.0.is_none_or(|remaining| eliminations <= remaining)
    }

    /// Charges the eliminations of `dividend mod divisor` for a trimmed
    /// nonconstant divisor; returns whether the budget still covers them.
    fn spend<C>(&mut self, dividend: &[C], divisor: &[C]) -> bool {
        let Some(remaining) = &mut self.0 else {
            return true;
        };
        let eliminations = dividend.len().saturating_sub(divisor.len() - 1);
        match remaining.checked_sub(eliminations) {
            Some(left) => {
                *remaining = left;
                true
            }
            None => false,
        }
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

/// Rational predicate coefficients above this size try the enclosure filter
/// before an exact primitive remainder chain.
const LARGE_RATIONAL_PREDICATE_COEFFICIENT_BITS: u64 = 1024;

/// Signs `predicate` at the unique selected root of `defining` in `interval`.
///
/// Both polynomials use ascending power coefficients. The caller owns the
/// singleton proof for `defining` on `interval`; no roots of the predicate
/// need to be isolated. `None` means this exact sign query did not decide.
/// Endpoint roots and exact point intervals decline so endpoint ownership
/// remains explicit; callers can evaluate an exact point directly.
/// A certified dyadic interval filter contracts an enclosure of the owned
/// root over general Real coefficients; rational inputs retain the primitive
/// integer chain. The filter's bounded work never restricts
/// exact replay; inseparable and repeated-root queries retain the chain.
/// Every coefficient decision here is STRICT, including the general-Real
/// fallback when the primitive integer chain does not apply.
/// If the supplied isolator is too narrow for an endpoint decision, a wider
/// dyadic bracket is used only after proving that it owns the same singleton.
pub fn sign_at_selected_root(
    defining: &[Real],
    predicate: &[Real],
    interval: &IsolatedRootInterval,
) -> Option<Ordering> {
    if defining.len() < 2
        || interval.distinct_root_count != 1
        || sign(&(&interval.upper - &interval.lower))? != Ordering::Greater
    {
        return None;
    }
    let endpoint_signs = [
        sign(&Real::eval_poly(defining, &interval.lower)),
        sign(&Real::eval_poly(defining, &interval.upper)),
    ];
    if endpoint_signs.contains(&Some(Ordering::Equal)) {
        return None;
    }
    // Unknown endpoint signs do not invalidate the retained singleton. Build
    // its query chain once and let a certified enclosing bracket replay it;
    // an early `?` here would make further root refinement lose decisions.
    let endpoints_decided = endpoint_signs.iter().all(Option::is_some);
    if predicate.is_empty() && endpoints_decided {
        return Some(Ordering::Equal);
    }
    // Primitive integer remainder chains are already cheap for rational
    // coefficients, especially exact-zero queries. Preserve that fast path;
    // a general Real field can avoid costly inverses through an enclosure.
    // Rational remainder chains slow down with coefficient size, because
    // each step extracts content with big-integer gcds. A long rational
    // predicate therefore tries the same certified enclosure filter first.
    let large_rational_predicate = || {
        predicate.iter().any(|coefficient| {
            coefficient.exact_rational_ref().is_some_and(|value| {
                value.numerator().bits() + value.denominator().bits()
                    > LARGE_RATIONAL_PREDICATE_COEFFICIENT_BITS
            })
        })
    };
    if endpoints_decided
        && (defining
            .iter()
            .chain(predicate)
            .any(|coefficient| coefficient.exact_rational_ref().is_none())
            || large_rational_predicate())
        && let Some(result) = sign_on_refined_singleton(defining, predicate, interval)
    {
        return Some(result);
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
    let selected_sign = if endpoints_decided {
        chain_sign(&chain, &interval.lower, &interval.upper)
    } else {
        None
    };
    selected_sign.or_else(|| sign_with_coarser_singleton(defining, &chain, interval))
}

/// Finest dyadic precision the refined-singleton filter may reach.
const REFINED_SINGLETON_MAX_PRECISION_BITS: u64 = 1 << 20;

/// Every box contains the caller's selected root. Interval Newton intersects
/// that box with m - P(m)/P'(box); the mean value theorem preserves ownership.
/// Outward dyadic rounding bounds denominator growth without changing P or
/// replacing the caller's root certificate. Failure only declines this filter.
fn sign_on_refined_singleton(
    defining: &[Real],
    predicate: &[Real],
    interval: &IsolatedRootInterval,
) -> Option<Ordering> {
    use hyperreal::Rational;

    fn enclose(values: &[Real], precision: i32) -> Option<Vec<[Rational; 2]>> {
        values
            .iter()
            .map(|value| value.certified_dyadic_interval(precision))
            .collect()
    }

    fn evaluate(polynomial: &[[Rational; 2]], point: &[Rational; 2]) -> [Rational; 2] {
        let mut value = [Rational::zero(), Rational::zero()];
        for [lower, upper] in polynomial.iter().rev() {
            let (lo, hi) = crate::interval::rational_interval_product(
                &value[0], &value[1], &point[0], &point[1],
            );
            value = [lo + lower, hi + upper];
        }
        value
    }

    fn separated([lower, upper]: &[Rational; 2]) -> Option<Ordering> {
        if lower.is_positive() {
            Some(Ordering::Greater)
        } else if upper.is_negative() {
            Some(Ordering::Less)
        } else if lower.is_zero() && upper.is_zero() {
            Some(Ordering::Equal)
        } else {
            None
        }
    }

    let mut retained: Option<[Rational; 2]> = None;
    // A bounded proof filter; inseparable values still use the exact chain.
    // Cancellation in a query scales with its coefficient size, so long
    // rational queries extend the doubling schedule in proportion.
    let coefficient_bits = predicate
        .iter()
        .filter_map(|coefficient| coefficient.exact_rational_ref())
        .map(|value| value.numerator().bits() + value.denominator().bits())
        .max()
        .unwrap_or(0);
    let finest = coefficient_bits
        .saturating_mul(4)
        .saturating_add(1024)
        .clamp(2048, REFINED_SINGLETON_MAX_PRECISION_BITS);
    let precisions = std::iter::successors(Some(64_u64), |bits| {
        (*bits < finest).then(|| (bits * 2).min(finest))
    });
    for precision in precisions {
        let precision = -i32::try_from(precision).ok()?;
        let mut bounds = [
            interval.lower.certified_dyadic_interval(precision)?[0].clone(),
            interval.upper.certified_dyadic_interval(precision)?[1].clone(),
        ];
        if let Some(previous) = retained.take() {
            if previous[0] > bounds[0] {
                bounds[0] = previous[0].clone();
            }
            if previous[1] < bounds[1] {
                bounds[1] = previous[1].clone();
            }
        }
        if bounds[0] > bounds[1] {
            return None;
        }
        let query = enclose(predicate, precision)?;
        if let Some(ordering) = separated(&evaluate(&query, &bounds)) {
            return Some(ordering);
        }
        let polynomial = enclose(defining, precision)?;
        let derivative = polynomial
            .iter()
            .enumerate()
            .skip(1)
            .map(|(power, [lo, hi])| {
                let power = Rational::new(i64::try_from(power).ok()?);
                Some([lo * &power, hi * &power])
            })
            .collect::<Option<Vec<_>>>()?;
        for _ in 0..2 {
            let slope = evaluate(&derivative, &bounds);
            if !slope[0].is_positive() && !slope[1].is_negative() {
                return None;
            }
            let midpoint = (&bounds[0] + &bounds[1]) * Rational::fraction(1, 2).ok()?;
            let residual = evaluate(&polynomial, &[midpoint.clone(), midpoint.clone()]);
            let (lo, hi) = crate::interval::rational_interval_product(
                &residual[0],
                &residual[1],
                &slope[1].clone().inverse().ok()?,
                &slope[0].clone().inverse().ok()?,
            );
            let lower = Real::new(&midpoint - hi).certified_dyadic_interval(precision)?[0].clone();
            let upper = Real::new(&midpoint - lo).certified_dyadic_interval(precision)?[1].clone();
            if lower > bounds[0] {
                bounds[0] = lower;
            }
            if upper < bounds[1] {
                bounds[1] = upper;
            }
            if bounds[0] > bounds[1] {
                return None;
            }
        }
        if let Some(ordering) = separated(&evaluate(&query, &bounds)) {
            return Some(ordering);
        }
        retained = Some(bounds);
    }
    None
}

fn chain_sign(chain: &[Vec<Real>], lower: &Real, upper: &Real) -> Option<Ordering> {
    query_sign_from_variations(variations(chain, lower)?, variations(chain, upper)?)
}

/// A tighter isolator can move a remainder-chain endpoint arbitrarily close
/// to one of its zeros. Reuse the completed chain on a certified enclosing
/// singleton instead of rebuilding it or refining the source still further.
fn sign_with_coarser_singleton(
    defining: &[Real],
    chain: &[Vec<Real>],
    interval: &IsolatedRootInterval,
) -> Option<Ordering> {
    let mut previous = (interval.lower.clone(), interval.upper.clone());
    for precision in [-256, -128, -64, -32, 0] {
        let lower = Real::new(interval.lower.certified_dyadic_interval(precision)?[0].clone());
        let upper = Real::new(interval.upper.certified_dyadic_interval(precision)?[1].clone());
        if previous == (lower.clone(), upper.clone()) {
            continue;
        }
        previous = (lower.clone(), upper.clone());
        if !matches!(
            sign(&Real::eval_poly(defining, &lower)),
            Some(Ordering::Less | Ordering::Greater)
        ) || !matches!(
            sign(&Real::eval_poly(defining, &upper)),
            Some(Ordering::Less | Ordering::Greater)
        ) {
            continue;
        }
        // Outward dyadic rounding encloses the original owned root. A fresh
        // exact count excludes every additional root, including endpoints;
        // otherwise a Tarski sum could be mistaken for that root's sign.
        if polynomial_has_one_distinct_root_in_open_interval(
            defining,
            &lower,
            &upper,
            PredicatePolicy::STRICT,
        ) != Some(true)
        {
            continue;
        }
        if let Some(sign) = chain_sign(chain, &lower, &upper) {
            return Some(sign);
        }
    }
    None
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
    first = compact_exact_coefficients(first);
    let mut chain = vec![first.clone()];
    loop {
        second = compact_exact_coefficients(second);
        while second
            .last()
            .is_some_and(|value| sign(value) == Some(Ordering::Equal))
        {
            second.pop();
        }
        if second.is_empty() {
            return Some(chain);
        }
        // The polynomial part of (P'Q)/P has no pole. Remove it before
        // recording the first numerator, after resolving a zero query without
        // imposing an unnecessary degree decision on the defining equation.
        if chain.len() == 1 && second.len() >= first.len() {
            second = polynomial_div_rem(second, &first, PredicatePolicy::STRICT)?.1;
            continue;
        }
        // Divide by the absolute leading coefficient: retain every sign
        // while removing field units before they compound in later remainders.
        let leading_sign = sign(second.last()?)?;
        second = crate::root_isolation::monic_normalize(second, PredicatePolicy::STRICT)?;
        if leading_sign == Ordering::Less {
            for coefficient in &mut second {
                *coefficient = -coefficient.clone();
            }
        }
        second = compact_exact_coefficients(second);
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

    #[test]
    fn certified_newton_signs_overcome_large_field_cancellation() {
        let inner = Real::from(5).sqrt().unwrap();
        let unit = &inner + (Real::from(50) - Real::from(20) * &inner).sqrt().unwrap();
        let wide = Real::from(2).powi_i64(256).unwrap();
        let tiny = Real::from(2).powi_i64(-600).unwrap();
        let outer = Real::from(2).sqrt().unwrap();
        assert!(outer.exact_rational_ref().is_none());
        for root_sign in [-1, 1] {
            let interval = if root_sign < 0 {
                IsolatedRootInterval {
                    lower: -&outer,
                    upper: -Real::one(),
                    exact_root: None,
                    distinct_root_count: 1,
                }
            } else {
                IsolatedRootInterval {
                    lower: Real::one(),
                    upper: outer.clone(),
                    exact_root: None,
                    distinct_root_count: 1,
                }
            };
            for orientation in [-1, 1] {
                // P = +/- unit * (x^3 -/+ 2) selects either real cube root.
                // Q = 2^256 P + 2^-600 x has the selected root's sign.
                // The nonrational endpoint is an enclosure, not a payload
                // reconstruction request; the original field and root stay owned.
                let defining = [-2 * root_sign, 0, 0, 1]
                    .map(|coefficient| Real::from(coefficient * orientation) * &unit);
                let mut predicate = defining.each_ref().map(|value| value * &wide);
                predicate[1] += &tiny;
                let expected = Some(root_sign.cmp(&0));
                assert_eq!(
                    sign_on_refined_singleton(&defining, &predicate, &interval),
                    expected
                );
                assert_eq!(
                    sign_at_selected_root(&defining, &predicate, &interval),
                    expected
                );
            }
        }
    }

    #[test]
    fn newton_uncertainty_retains_zero_and_repeated_root_proofs() {
        let interval = IsolatedRootInterval {
            lower: Real::one(),
            upper: Real::from(2),
            exact_root: None,
            distinct_root_count: 1,
        };
        let query = [-2, 0, 1].map(Real::from);
        for defining in [query.to_vec(), [4, 0, -4, 0, 1].map(Real::from).to_vec()] {
            assert_eq!(
                sign_on_refined_singleton(&defining, &query, &interval),
                None
            );
            assert_eq!(
                sign_at_selected_root(&defining, &query, &interval),
                Some(Ordering::Equal)
            );
        }
        let repeated = [4, 0, -4, 0, 1].map(Real::from);
        let positive = [-1, 1].map(Real::from);
        assert_eq!(
            sign_on_refined_singleton(&repeated, &positive, &interval),
            None
        );
        assert_eq!(
            sign_at_selected_root(&repeated, &positive, &interval),
            Some(Ordering::Greater)
        );
    }

    #[test]
    fn dyadic_filter_budget_does_not_limit_exact_query_precision() {
        let unit = Real::from(5).sqrt().unwrap();
        let defining = [-2, 0, 0, 1].map(|coefficient| Real::from(coefficient) * &unit);
        let interval = IsolatedRootInterval {
            lower: Real::one(),
            upper: Real::from(2),
            exact_root: None,
            distinct_root_count: 1,
        };
        for orientation in [-1, 1] {
            // A nonrational coefficient receives no size-based precision
            // extension, so the filter stops at its fixed budget.
            let predicate = [
                Real::zero(),
                Real::from(orientation) * Real::from(2).powi_i64(-4096).unwrap() * &unit,
            ];
            assert_eq!(
                sign_on_refined_singleton(&defining, &predicate, &interval),
                None
            );
            assert_eq!(
                sign_at_selected_root(&defining, &predicate, &interval),
                Some(orientation.cmp(&0))
            );
        }
    }

    #[test]
    fn long_rational_queries_extend_the_filter_precision() {
        let defining = [-2, 0, 0, 1].map(Real::from);
        let interval = IsolatedRootInterval {
            lower: Real::one(),
            upper: Real::from(2),
            exact_root: None,
            distinct_root_count: 1,
        };
        for orientation in [-1, 1] {
            let predicate = [
                Real::zero(),
                Real::from(orientation) * Real::from(2).powi_i64(-4096).unwrap(),
            ];
            assert_eq!(
                sign_on_refined_singleton(&defining, &predicate, &interval),
                Some(orientation.cmp(&0))
            );
        }
    }

    #[test]
    fn nonrational_query_quotients_preserve_tiny_signs_on_repeated_conjugates() {
        let inner = Real::from(5).sqrt().unwrap();
        let outer = (Real::from(50) - Real::from(20) * &inner).sqrt().unwrap();
        let unit = inner + outer;
        let wide = Real::from(2).powi_i64(256).unwrap();
        let tiny = Real::from(2).powi_i64(-256).unwrap();
        // (x^2 - 2)^2 owns one repeated root in each interval. A multiple
        // of this defining polynomial vanishes there, however large its
        // coefficients become compared with the retained linear query.
        let base = [4, 0, -4, 0, 1];
        for orientation in [-1_i32, 1] {
            let defining = base.map(|coefficient| Real::from(coefficient * orientation) * &unit);
            let quotient = [
                &wide * (&unit + Real::one()),
                -&wide * &unit,
                Real::zero(),
                &wide * &unit,
            ];
            let mut multiple = vec![Real::zero(); defining.len() + quotient.len() - 1];
            for (i, a) in defining.iter().enumerate() {
                for (j, b) in quotient.iter().enumerate() {
                    multiple[i + j] += a * b;
                }
            }
            for (lower, upper, root_sign) in [(-2, -1, -1_i32), (1, 2, 1)] {
                let interval = IsolatedRootInterval {
                    lower: Real::from(lower),
                    upper: Real::from(upper),
                    exact_root: None,
                    distinct_root_count: 1,
                };
                for query_sign in [-1_i32, 0, 1] {
                    let mut predicate = multiple.clone();
                    predicate[1] += Real::from(query_sign) * &tiny * &unit;
                    let expected = (query_sign * root_sign).cmp(&0);
                    assert_eq!(
                        sign_at_selected_root(&defining, &predicate, &interval),
                        Some(expected)
                    );
                }
            }
        }
    }

    #[test]
    fn zero_query_does_not_require_an_unused_leading_degree_decision() {
        let epsilon = Real::one() - Real::from(2).powi_i64(-600).unwrap().cos();
        // epsilon >= 0, so this polynomial is strictly increasing on [0,1]
        // with opposite endpoint signs, even if epsilon's scalar sign is not
        // available to the decision cascade. The zero query needs no degree.
        let defining = [Real::from(-1), Real::from(2), epsilon];
        let interval = IsolatedRootInterval {
            lower: Real::zero(),
            upper: Real::one(),
            exact_root: None,
            distinct_root_count: 1,
        };
        assert_eq!(
            sign_at_selected_root(&defining, &vec![Real::zero(); 8], &interval),
            Some(Ordering::Equal)
        );
    }

    fn selected_cubic_root() -> AlgebraicRootRepresentation {
        AlgebraicRootRepresentation {
            constraint_index: 23,
            symbol: crate::SymbolId(29),
            interval_index: 0,
            polynomial_coefficients: vec![
                Real::from(-1),
                Real::zero(),
                Real::zero(),
                Real::from(2),
            ],
            interval: IsolatedRootInterval {
                lower: (Real::from(3) / Real::from(4)).unwrap(),
                upper: Real::one(),
                exact_root: None,
                distinct_root_count: 1,
            },
            validation: crate::AlgebraicRootValidationReport {
                status: crate::AlgebraicRootValidationStatus::Valid,
                message: None,
            },
        }
    }

    #[test]
    fn tuple_signs_bind_arbitrary_exact_points_in_either_axis() {
        let source = selected_cubic_root();
        assert_eq!(
            crate::validate_algebraic_root_representation(&source, PredicatePolicy::STRICT).status,
            crate::AlgebraicRootValidationStatus::Valid,
        );
        for known_axis in 0..2 {
            for (scale, expected) in [(1, Ordering::Greater), (-1, Ordering::Less)] {
                let point = Real::from(scale) * Real::from(2).sqrt().unwrap();
                assert!(point.exact_rational_ref().is_none());
                let mut sources = vec![source.clone(); 2];
                sources[known_axis] = AlgebraicRootRepresentation::from_exact_value(&point);
                // Q(k,x)=(k^2-2)x^2+kx-1, x=cbrt(1/2).
                // x>3/4 and sqrt(2)>4/3 prove the positive case.
                let mut coefficients = vec![Real::zero(); 9];
                coefficients[0] = -Real::one();
                coefficients[4] = Real::one();
                coefficients[8] = Real::one();
                coefficients[if known_axis == 0 { 2 } else { 6 }] = Real::from(-2);
                let polynomial = DenseTensorPolynomial::try_new(vec![3, 3], coefficients).unwrap();
                assert_eq!(
                    sign_at_selected_tuple(&polynomial, &sources),
                    Some(expected)
                );
            }
        }
        let known = AlgebraicRootRepresentation::from_exact_value(&Real::from(2).sqrt().unwrap());
        let difference = DenseTensorPolynomial::try_new(
            vec![2, 2],
            vec![Real::zero(), -Real::one(), Real::one(), Real::zero()],
        )
        .unwrap();
        assert_eq!(
            sign_at_selected_tuple(&difference, &[known.clone(), known]),
            Some(Ordering::Equal),
        );
    }

    #[test]
    fn tuple_signs_decline_unresolved_axes_and_invalid_evidence() {
        let source = selected_cubic_root();
        let polynomial = DenseTensorPolynomial::try_new(vec![1, 1], vec![Real::one()]).unwrap();
        assert_eq!(sign_at_selected_tuple(&polynomial, &[]), None);
        assert_eq!(
            sign_at_selected_tuple(&polynomial, &[source.clone(), source.clone()]),
            None,
        );
        let mut point = AlgebraicRootRepresentation::from_exact_value(&Real::zero());
        point.interval.distinct_root_count = 2;
        assert_eq!(sign_at_selected_tuple(&polynomial, &[source, point]), None);
    }

    struct RealContext;

    impl OrderedFieldPolynomialContext<Real> for RealContext {
        type Error = ();

        fn constant(&mut self, value: &Real) -> Result<Real, Self::Error> {
            Ok(value.clone())
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

        fn normalize_positive_scale(&mut self, coefficients: &mut [Real]) {
            if let Some(normalized) =
                crate::integer_interpolation::primitive_integer_polynomial(coefficients)
            {
                coefficients.clone_from_slice(&normalized);
            }
        }

        fn sign(&mut self, value: &Real) -> Result<Ordering, Self::Error> {
            sign(value).ok_or(())
        }

        fn sign_if_separated(&mut self, value: &Real) -> Result<Option<Ordering>, Self::Error> {
            Ok(sign(value))
        }
    }

    fn integer_polynomial_from_roots(roots: &[i32]) -> Vec<Real> {
        let mut polynomial = vec![Real::one()];
        for root in roots {
            let mut next = vec![Real::zero(); polynomial.len() + 1];
            for (power, coefficient) in polynomial.iter().enumerate() {
                next[power + 1] = &next[power + 1] + coefficient;
                next[power] = &next[power] - coefficient * Real::from(*root);
            }
            polynomial = next;
        }
        polynomial
    }

    #[test]
    fn selected_root_vanishing_uses_the_common_divisor() {
        // (t - 1)(t - 3)(t^2 - 2), selected root 1 in (1/2, 5/4).
        let mut defining = integer_polynomial_from_roots(&[1, 3]);
        let quadratic = [Real::from(-2), Real::zero(), Real::one()];
        let mut product = vec![Real::zero(); defining.len() + 2];
        for (power, coefficient) in defining.iter().enumerate() {
            for (other_power, other) in quadratic.iter().enumerate() {
                product[power + other_power] = &product[power + other_power] + coefficient * other;
            }
        }
        defining = product;
        let interval = IsolatedRootInterval {
            lower: (Real::from(1) / Real::from(2)).unwrap(),
            upper: (Real::from(5) / Real::from(4)).unwrap(),
            exact_root: None,
            distinct_root_count: 1,
        };
        let query = |predicate: &[Real]| {
            ordered_field_vanishes_at_selected_root(
                &defining,
                predicate,
                &interval,
                &mut RealContext,
            )
            .unwrap()
        };
        // Shares the selected root.
        assert_eq!(query(&integer_polynomial_from_roots(&[1, -4])), Some(true));
        // Shares only an unselected root of the defining relation.
        assert_eq!(query(&integer_polynomial_from_roots(&[3, 5])), Some(false));
        // Coprime with the defining relation.
        assert_eq!(query(&integer_polynomial_from_roots(&[2, 7])), Some(false));
        // Shares only the quadratic factor, whose roots lie outside.
        assert_eq!(query(&quadratic), None);
        // Shares the selected root through a quadratic common divisor.
        assert_eq!(
            query(&integer_polynomial_from_roots(&[1, 3, 9])),
            Some(true)
        );
        // The zero polynomial vanishes everywhere.
        assert_eq!(query(&[]), Some(true));
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
            if let Some(filtered) = sign_on_refined_singleton(&defining, &predicate, &interval) {
                prop_assert_eq!(Some(filtered), sign(&Real::eval_poly(&predicate, &selected)));
            }
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
    fn ordered_field_replay_bounds_irrelevant_polynomial_scale() {
        #[derive(Default)]
        struct BoundedField {
            maximum_bits: u64,
        }
        impl BoundedField {
            fn retain(&mut self, value: Real) -> Result<Real, &'static str> {
                let rational = value.exact_rational_ref().ok_or("rational fixture")?;
                self.maximum_bits = self
                    .maximum_bits
                    .max(rational.numerator().bits())
                    .max(rational.denominator().bits());
                if self.maximum_bits > 256 {
                    return Err("coefficient budget exceeded");
                }
                Ok(value)
            }
        }
        impl OrderedFieldPolynomialContext<Real> for BoundedField {
            type Error = &'static str;

            fn constant(&mut self, value: &Real) -> Result<Real, Self::Error> {
                self.retain(value.clone())
            }
            fn add(&mut self, left: &Real, right: &Real) -> Result<Real, Self::Error> {
                self.retain(left + right)
            }
            fn multiply(&mut self, left: &Real, right: &Real) -> Result<Real, Self::Error> {
                self.retain(left * right)
            }
            fn scale(&mut self, value: &Real, scale: &Real) -> Result<Real, Self::Error> {
                self.retain(value * scale)
            }
            fn normalize_positive_scale(&mut self, coefficients: &mut [Real]) {
                RealContext.normalize_positive_scale(coefficients);
            }
            fn sign(&mut self, value: &Real) -> Result<Ordering, Self::Error> {
                sign(value).ok_or("exact rational sign")
            }
            fn sign_if_separated(&mut self, value: &Real) -> Result<Option<Ordering>, Self::Error> {
                self.sign(value).map(Some)
            }
        }

        // P=(t-1)(t^2+1)(t^2+2)(t^2+3) has exactly one real root, t=1.
        // Every defining gauge and every independently scaled predicate must
        // reuse that selection without carrying their content through the PRS.
        let defining = [-6, 6, -11, 11, -6, 6, -1, 1].map(Real::from);
        let predicate = [2, -3, 4, -2, 5, -6, 1].map(Real::from);
        let interval = IsolatedRootInterval {
            lower: Real::zero(),
            upper: Real::from(2),
            exact_root: None,
            distinct_root_count: 1,
        };
        let wide = Real::from(65536);
        for scale in [
            Real::one(),
            -Real::one(),
            wide.clone(),
            -&wide,
            (Real::one() / &wide).unwrap(),
            (-Real::one() / &wide).unwrap(),
        ] {
            let polynomial: Vec<_> = defining.iter().map(|value| value * &scale).collect();
            for (shift, query_scale) in [(0, 3), (0, -7), (-1, 5)] {
                let mut query = predicate.to_vec();
                query[0] += Real::from(shift);
                for coefficient in &mut query {
                    *coefficient *= Real::from(query_scale);
                }
                let expected = sign(&Real::eval_poly(&query, &Real::one()));
                let mut field = BoundedField::default();
                let actual =
                    ordered_field_sign_at_selected_root(&polynomial, &query, &interval, &mut field);
                assert_eq!(actual, Ok(expected), "peak bits: {}", field.maximum_bits);
            }
        }
    }

    #[test]
    fn narrow_nested_root_isolators_reuse_a_certified_coarser_bracket() {
        let alpha = Real::one() + Real::from(2).sqrt().unwrap();
        let root = alpha.clone().sqrt().unwrap();
        let bounds = root.certified_dyadic_interval(-4500).unwrap();
        let interval = IsolatedRootInterval {
            lower: Real::new(bounds[0].clone()),
            upper: Real::new(bounds[1].clone()),
            exact_root: None,
            distinct_root_count: 1,
        };
        // P=t(t^2-alpha). Its positive root r satisfies r^2=alpha.
        // A remainder in this query's chain is proportional to t-r, so
        // excessively fine endpoint evaluation loses its cheap sign proof.
        let defining = [Real::zero(), -alpha.clone(), Real::zero(), Real::one()];
        let predicate = [
            Real::from(2) * &alpha,
            &alpha * &root,
            &alpha - Real::from(3),
        ];
        // Q(r)=alpha(2 alpha-1)>0; reversing Q must reverse the exact result.
        for (scale, expected) in [(1, Ordering::Greater), (-1, Ordering::Less)] {
            let predicate = predicate.each_ref().map(|value| value * Real::from(scale));
            assert_eq!(
                sign_at_selected_root(&defining, &predicate, &interval),
                Some(expected),
            );
        }
    }

    #[test]
    fn narrow_isolators_replay_before_initial_endpoint_signs_are_available() {
        for (root, threshold) in [
            (Real::pi(), Real::from(3)),
            (
                (Real::one() / Real::from(3)).unwrap().exp().unwrap(),
                Real::one(),
            ),
        ] {
            for precision in [-600, -1200] {
                let [lower, upper] = root.certified_dyadic_interval(precision).unwrap();
                let interval = IsolatedRootInterval {
                    lower: Real::new(lower),
                    upper: Real::new(upper),
                    exact_root: None,
                    distinct_root_count: 1,
                };
                // Each linear equation owns this same exact root, even when
                // its initial endpoint differences cannot be signed. The
                // query is positive at that root under either defining gauge.
                for gauge in [Real::from(3), Real::from(-7)] {
                    let defining = [-&root * &gauge, gauge];
                    for (scale, expected) in [(1, Ordering::Greater), (-1, Ordering::Less)] {
                        let scale = Real::from(scale);
                        let query = [-&threshold * &scale, scale];
                        assert_eq!(
                            sign_at_selected_root(&defining, &query, &interval),
                            Some(expected),
                        );
                    }
                    assert_eq!(
                        sign_at_selected_root(&defining, &defining, &interval),
                        Some(Ordering::Equal),
                    );
                    assert_eq!(
                        sign_at_selected_root(&defining, &[], &interval),
                        Some(Ordering::Equal),
                    );
                }
            }
        }
    }

    #[test]
    fn coarser_brackets_must_not_admit_a_foreign_root() {
        let delta = Real::from(2).powi_i64(-300).unwrap();
        let epsilon = Real::from(2).powi_i64(-600).unwrap();
        let defining = [Real::zero(), -delta.clone(), Real::one()];
        // Q=t-delta/2 is negative at the selected root zero and positive at
        // the nearby root delta. Counting both would incorrectly yield zero.
        let product = [
            (&delta * &delta / Real::from(2)).unwrap(),
            -Real::from(2) * &delta,
            Real::from(2),
        ];
        let chain = primitive_integer_signed_remainder_sequence(&defining, &product).unwrap();
        let interval = IsolatedRootInterval {
            lower: -epsilon.clone(),
            upper: epsilon,
            exact_root: None,
            distinct_root_count: 1,
        };
        assert_eq!(
            chain_sign(&chain, &interval.lower, &interval.upper),
            Some(Ordering::Less),
        );
        let wide = Real::from(2).powi_i64(-256).unwrap();
        assert_eq!(chain_sign(&chain, &(-&wide), &wide), Some(Ordering::Equal));
        assert_eq!(
            sign_with_coarser_singleton(&defining, &chain, &interval),
            None
        );
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
